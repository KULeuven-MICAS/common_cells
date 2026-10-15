// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`include "common_cells/assertions.svh"

/// Behavioral model of `rr_arb_tree` for fast simulation. Bender compiles it instead of
/// `src/rr_arb_tree.sv` when the target `fast_sim` is set; it is the same module, with the same
/// parameters, ports and outputs on every cycle, but not the same structure.
///
/// `rr_arb_tree` builds its arbitration tree from continuous assignments: five per node (request,
/// select, index, data, grant), so a 64-input instance is several hundred tiny processes, and every
/// tree level is another combinational level a simulator steps through (and, multi-threaded,
/// synchronizes its threads at). Here:
/// * the requests and select bits of all nodes are computed in one pass from the leaves up;
/// * the output index and the data follow the select bits from the root down to one leaf, so the
///   data is muxed once instead of at every node and the grant goes to that leaf only;
/// * with `FairArb`, the next priority is searched for once per clock edge, in the clocked block,
///   from the granted input on and only on a grant, instead of two trailing-zero counters that
///   re-evaluate with every change of a request.
///
/// What differs from `rr_arb_tree`:
/// * Hierarchy: there are no `gen_levels`, `index_nodes`, `data_nodes`, `gnt_nodes`, `req_nodes`
///   and `i_lzc_*` to probe. `rr_q`, `req_d`, `lock_q` and `req_q` keep their names.
/// * Unknown values: an X in a request or in `rr_q` that reaches the arbitration decision makes
///   `idx_o`, `data_o` and the grants it could go to X (the tree turns it into the bitwise merge of
///   its branches, which is X wherever they differ), and an unknown update condition makes `rr_q`
///   X. An X request that does not reach the decision stays out of the outputs, as in the tree.
///   Two-state stimulus gives identical outputs; `test/rr_arb_tree_bench_tb.sv` checks that.
module rr_arb_tree #(
  /// Number of inputs to be arbitrated.
  parameter int unsigned NumIn      = 64,
  /// Data width of the payload in bits. Not needed if `DataType` is overwritten.
  parameter int unsigned DataWidth  = 32,
  /// Data type of the payload, can be overwritten with custom type. Only use of `DataWidth`.
  parameter type         DataType   = logic [DataWidth-1:0],
  /// The `ExtPrio` option allows to override the internal round robin counter via the
  /// `rr_i` signal. Set to 1'b1 to enable.
  parameter bit          ExtPrio    = 1'b0,
  /// If `AxiVldRdy` is set, the req/gnt signals are compliant with the AXI style vld/rdy
  /// handshake. Set to `1'b1` to treat req/gnt as vld/rdy.
  parameter bit          AxiVldRdy  = 1'b0,
  /// The `LockIn` option prevents the arbiter from changing the arbitration
  /// decision when the arbiter is disabled. Set to `1'b1` to enable.
  parameter bit          LockIn     = 1'b0,
  /// When set, ensures that throughput gets distributed evenly between all inputs.
  /// Set to `1'b0` to disable.
  parameter bit          FairArb    = 1'b1,
  /// Dependent parameter, do **not** overwrite.
  /// Width of the arbitration priority signal and the arbitrated index.
  parameter int unsigned IdxWidth   = (NumIn > 32'd1) ? unsigned'($clog2(NumIn)) : 32'd1,
  /// Dependent parameter, do **not** overwrite.
  /// Type for defining the arbitration priority and arbitrated index signal.
  parameter type         idx_t      = logic [IdxWidth-1:0]
) (
  /// Clock, positive edge triggered.
  input  logic                clk_i,
  /// Asynchronous reset, active low.
  input  logic                rst_ni,
  /// Clears the arbiter state. Only used if `ExtPrio` is `1'b0` or `LockIn` is `1'b1`.
  input  logic                flush_i,
  /// External round-robin priority. Only used if `ExtPrio` is `1'b1.`
  input  idx_t                rr_i,
  /// Input requests arbitration.
  input  logic    [NumIn-1:0] req_i,
  /// Input request is granted.
  output logic    [NumIn-1:0] gnt_o,
  /// Input data for arbitration.
  input  DataType [NumIn-1:0] data_i,
  /// Output request is valid.
  output logic                req_o,
  /// Output request is granted.
  input  logic                gnt_i,
  /// Output data.
  output DataType             data_o,
  /// Index from which input the data came from.
  output idx_t                idx_o
);

  // just pass through in this corner case
  if (NumIn == unsigned'(1)) begin : gen_pass_through
    assign req_o    = req_i[0];
    assign gnt_o[0] = gnt_i;
    assign data_o   = data_i[0];
    assign idx_o    = '0;
  // non-degenerate cases
  end else begin : gen_arbiter
    localparam int unsigned NumLevels = unsigned'($clog2(NumIn));
    localparam int unsigned NumNodes  = 2**NumLevels - 1;
    localparam int unsigned NumLeaves = 2**(NumLevels-1);  // nodes of the last tree level
    localparam int unsigned LeafBase  = NumLeaves - 1;     // index of the first of them

    idx_t                rr_q;
    logic [NumIn-1:0]    req_d;
    logic [NumNodes-1:0] req_nodes, sel_nodes;
    int unsigned         leaf;      // the last-level node the select bits lead to
    logic                leaf_sel;  // its select bit
    logic                leaf_pad;  // it is a padding node: no input, data zero
    logic                dec_x;     // a select bit on the path is unknown

    // Requests and select bits of every node, then the path from the root to one leaf. A select
    // bit is one bit of the index, the root's the most significant.
    always_comb begin : p_decision
      int unsigned node, l;
      logic        unk;
      idx_t        idx;
      // last level: up to two inputs per node
      for (int unsigned i = 0; i < NumLeaves; i++) begin
        if (i * 2 < NumIn - 1) begin
          req_nodes[LeafBase+i] = req_d[i*2] | req_d[i*2+1];
          sel_nodes[LeafBase+i] = ~req_d[i*2] | req_d[i*2+1] & rr_q[0];
        end else if (i * 2 == NumIn - 1) begin
          req_nodes[LeafBase+i] = req_d[i*2];
          sel_nodes[LeafBase+i] = 1'b0;
        end else begin
          req_nodes[LeafBase+i] = 1'b0;
          sel_nodes[LeafBase+i] = 1'b0;
        end
      end
      // the other levels, from the leaves up
      for (int level = int'(NumLevels) - 2; level >= 0; level--) begin
        for (int unsigned i = 0; i < 2**level; i++) begin
          int unsigned n0, n1;
          n0 = 2**level - 1 + i;
          n1 = 2**(level+1) - 1 + i * 2;
          req_nodes[n0] = req_nodes[n1] | req_nodes[n1+1];
          sel_nodes[n0] = ~req_nodes[n1] | req_nodes[n1+1] & rr_q[NumLevels-1-level];
        end
      end
      // the path down; an unknown select bit poisons the XOR of them
      node = 0;
      idx  = '0;
      unk  = 1'b0;
      for (int unsigned level = 0; level < NumLevels - 1; level++) begin
        unk ^= sel_nodes[node];
        if (sel_nodes[node]) idx |= idx_t'(1) << (NumLevels - 1 - level);
        node = 2 * node + 1 + int'(sel_nodes[node]);
      end
      leaf     = node;
      leaf_sel = sel_nodes[node];
      unk     ^= leaf_sel;
      dec_x    = $isunknown(unk);
      // the last level: a node with two inputs selects one, a node with one input has index bit 0
      // and a padding node has index bit 0 and data 0
      l        = node - LeafBase;
      leaf_pad = (l * 2 > NumIn - 1);
      if (l * 2 < NumIn - 1 && leaf_sel) idx |= idx_t'(1);
      req_o    = req_nodes[0];
      idx_o    = dec_x ? idx_t'('x) : idx;
    end

    assign data_o = dec_x ? DataType'('x) : leaf_pad ? DataType'('0) : data_i[idx_o];

    // The grant goes down the path only
    always_comb begin : p_grant
      int unsigned l;
      l     = leaf - LeafBase;
      gnt_o = '0;
      if (dec_x) begin
        gnt_o = {NumIn{gnt_i & 1'bx}} & (AxiVldRdy ? {NumIn{1'b1}} : req_d);
      end else if (l * 2 < NumIn - 1) begin
        gnt_o[l*2]   = gnt_i & (AxiVldRdy | req_d[l*2])   & ~leaf_sel;
        gnt_o[l*2+1] = gnt_i & (AxiVldRdy | req_d[l*2+1]) &  leaf_sel;
      end else if (l * 2 == NumIn - 1) begin
        gnt_o[l*2]   = gnt_i & (AxiVldRdy | req_d[l*2]);
      end
    end

    if (ExtPrio) begin : gen_ext_rr
      assign rr_q  = rr_i;
      assign req_d = req_i;
    end else begin : gen_int_rr
      // lock arbiter decision in case we got at least one req and no acknowledge
      if (LockIn) begin : gen_lock
        logic lock_d, lock_q;
        logic [NumIn-1:0] req_q;

        assign lock_d = req_o & ~gnt_i;
        assign req_d  = (lock_q) ? req_q : req_i;

        always_ff @(posedge clk_i or negedge rst_ni) begin : p_lock_regs
          if (!rst_ni) begin
            lock_q <= '0;
            req_q  <= '0;
          end else if (flush_i) begin
            lock_q <= '0;
            req_q  <= '0;
          end else begin
            lock_q <= lock_d;
            req_q  <= req_d;
          end
        end

        `ifndef COMMON_CELLS_ASSERTS_OFF
          `ASSERT(lock, req_o && (!gnt_i && !flush_i) |=> idx_o == $past(idx_o),
                  clk_i, !rst_ni || flush_i,
                  "Lock implies same arbiter decision in next cycle if output is not ready.")

          logic [NumIn-1:0] req_tmp;
          assign req_tmp = req_q & req_i;
          `ASSUME(lock_req, lock_d |=> req_tmp == req_q, clk_i, !rst_ni || flush_i,
                  "It is disallowed to deassert unserved request signals when LockIn is enabled.")
        `endif
      end else begin : gen_no_lock
        assign req_d = req_i;
      end

      // this holds the highest priority
      always_ff @(posedge clk_i or negedge rst_ni) begin : p_rr_regs
        if (!rst_ni) begin
          rr_q <= '0;
        end else if (flush_i) begin
          rr_q <= '0;
        end else if (gnt_i && req_o) begin
          if (FairArb) begin
            // the next requester above rr_q, else the first at or below it
            logic found;
            found = 1'b0;
            for (int unsigned i = unsigned'(rr_q) + 1; i < NumIn; i++) begin
              if (req_d[i]) begin
                rr_q  <= idx_t'(i);
                found = 1'b1;
                break;
              end
            end
            if (!found) begin
              for (int unsigned i = 0; i <= unsigned'(rr_q); i++) begin
                if (req_d[i]) begin
                  rr_q <= idx_t'(i);
                  break;
                end
              end
            end
          end else begin
            rr_q <= (rr_q == idx_t'(NumIn-1)) ? '0 : rr_q + 1'b1;
          end
        end else if ($isunknown(gnt_i && req_o)) begin
          rr_q <= idx_t'('x);
        end
      end
    end

    `ifndef COMMON_CELLS_ASSERTS_OFF
    `ASSERT_INIT(numin_0, NumIn, "Input must be at least one element wide.")
    `ASSERT_INIT(lockin_and_extprio, !(LockIn && ExtPrio),
                 "Cannot use LockIn feature together with external ExtPrio.")

    `ASSERT(hot_one, $onehot0(gnt_o), clk_i, !rst_ni || flush_i,
            "Grant signal must be hot1 or zero.")

    `ASSERT(gnt0, |gnt_o |-> gnt_i, clk_i, !rst_ni || flush_i, "Grant out implies grant in.")

    `ASSERT(gnt1, req_o |-> gnt_i |-> |gnt_o, clk_i, !rst_ni || flush_i,
            "Req out and grant in implies grant out.")

    `ASSERT(gnt_idx, req_o |->  gnt_i |-> gnt_o[idx_o], clk_i, !rst_ni || flush_i,
            "Idx_o / gnt_o do not match.")

    `ASSERT(req0, |req_i |-> req_o, clk_i, !rst_ni || flush_i, "Req in implies req out.")

    `ASSERT(req1, req_o |-> |req_i, clk_i, !rst_ni || flush_i, "Req out implies req in.")
    `endif
  end

endmodule : rr_arb_tree
