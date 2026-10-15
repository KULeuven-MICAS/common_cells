// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

/// One `rr_arb_tree` with a deterministic random driver and a running hash of its outputs.
///
/// The driver is a clocked xorshift generator, so a run does not depend on the simulator's
/// process scheduling and two runs of one design give one hash. Requests are held until granted
/// (the `LockIn` assumption) and, without `LockIn`, withdrawn at random. Of every 4096 cycles, 512
/// are sparse and 512 silent, so the arbiter also sees few requests and none.
module rr_arb_tree_bench_cell #(
  parameter int unsigned NumIn     = 8,
  parameter int unsigned DataWidth = 8,
  parameter bit          ExtPrio   = 1'b0,
  parameter bit          AxiVldRdy = 1'b0,
  parameter bit          LockIn    = 1'b0,
  parameter bit          FairArb   = 1'b1,
  /// New requests per idle input and cycle: 0 = 1/8, 1 = 1/4, 2 = 1/2, 3 = 3/4
  parameter int unsigned Density   = 2,
  parameter int unsigned Seed      = 1
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  output logic [63:0] hash_o,
  /// cycles with an unknown output
  output logic [31:0] xcycles_o,
  /// cycles with a grant that is not one-hot or not behind `gnt_i`
  output logic [31:0] errors_o
);

  localparam int unsigned IdxWidth = (NumIn > 32'd1) ? unsigned'($clog2(NumIn)) : 32'd1;
  localparam int unsigned OutWidth = NumIn + DataWidth + IdxWidth + 1;
  typedef logic [IdxWidth-1:0]  idx_t;
  typedef logic [DataWidth-1:0] data_t;

  function automatic logic [63:0] xorshift(input logic [63:0] x_i);
    logic [63:0] x;
    x = x_i;
    x = x ^ (x << 13);
    x = x ^ (x >> 7);
    x = x ^ (x << 17);
    return x;
  endfunction

  // the outputs folded to 64 bits
  function automatic logic [63:0] fold(input logic [OutWidth-1:0] v);
    logic [63:0] r;
    r = '0;
    for (int unsigned i = 0; i < OutWidth; i += 64) r = {r[62:0], r[63]} ^ 64'(v >> i);
    return r;
  endfunction

  logic [63:0]       rng_q;
  logic [11:0]       phase_q;
  logic [NumIn-1:0]  req_q;
  data_t [NumIn-1:0] data_q;
  logic              gnt_q, flush_q;
  idx_t              rr_q;

  logic [NumIn-1:0]  gnt_o;
  logic              req_o;
  data_t             data_o;
  idx_t              idx_o;

  // stimulus
  always_ff @(posedge clk_i or negedge rst_ni) begin : p_stimulus
    logic [63:0]  w [6];
    logic [127:0] a, b, c, fresh, keep;
    logic [NumIn-1:0] held;
    data_t        base;
    if (!rst_ni) begin
      rng_q   <= (64'(Seed) * 64'h9E3779B97F4A7C15) | 64'd1;
      phase_q <= '0;
      req_q   <= '0;
      data_q  <= '0;
      gnt_q   <= 1'b0;
      flush_q <= 1'b0;
      rr_q    <= '0;
    end else begin
      w[0] = xorshift(rng_q);
      for (int unsigned i = 1; i < 6; i++) w[i] = xorshift(w[i-1]);
      rng_q   <= w[5];
      phase_q <= phase_q + 1'b1;
      a = {w[1], w[0]};
      b = {w[3], w[2]};
      c = {w[5], w[4]};
      case (Density)
        0:       fresh = a & b & c;
        1:       fresh = a & b;
        2:       fresh = a;
        default: fresh = a | b;
      endcase
      if (phase_q[11:9] == 3'd6) fresh = a & b & c;
      if (phase_q[11:9] == 3'd7) fresh = '0;
      keep = b | c;
      held = req_q & ~gnt_o;
      if (!LockIn) held = held & keep[NumIn-1:0];
      req_q <= held | fresh[NumIn-1:0];
      base = data_t'(b);
      for (int unsigned k = 0; k < NumIn; k++) begin
        data_q[k] <= base ^ data_t'(64'(k) * 64'h9E3779B97F4A7C15);
      end
      gnt_q   <= c[0] | c[1];
      flush_q <= (c[26:16] == '0);
      rr_q    <= idx_t'(c[63:32] % NumIn);
    end
  end

  rr_arb_tree #(
    .NumIn     ( NumIn     ),
    .DataWidth ( DataWidth ),
    .ExtPrio   ( ExtPrio   ),
    .AxiVldRdy ( AxiVldRdy ),
    .LockIn    ( LockIn    ),
    .FairArb   ( FairArb   )
  ) i_dut (
    .clk_i,
    .rst_ni,
    .flush_i ( flush_q ),
    .rr_i    ( rr_q    ),
    .req_i   ( req_q   ),
    .gnt_o,
    .data_i  ( data_q  ),
    .req_o,
    .gnt_i   ( gnt_q   ),
    .data_o,
    .idx_o
  );

  // the outputs of every cycle
  always_ff @(posedge clk_i or negedge rst_ni) begin : p_hash
    logic [OutWidth-1:0] out;
    if (!rst_ni) begin
      hash_o    <= 64'hCBF29CE484222325;
      xcycles_o <= '0;
      errors_o  <= '0;
    end else begin
      out    = {gnt_o, data_o, idx_o, req_o};
      hash_o <= (hash_o ^ fold(out)) * 64'h100000001B3;
      if ($isunknown(out)) xcycles_o <= xcycles_o + 1;
      if (!$onehot0(gnt_o) || (|gnt_o && !gnt_q)) errors_o <= errors_o + 1;
    end
  end

endmodule


/// Testbench for the simulation speed and the equivalence of `rr_arb_tree` implementations: a set of
/// instances that stand for the arbiters of a cluster (the TCDM crossbar's 77-input bank arbiters,
/// the request muxes) driven for `NumCycles` cycles, then one line with the hash of all their
/// outputs. The structural `rr_arb_tree` and the `fast_sim` model give the same lines:
/// run it once per implementation (test/rr_arb_tree_bench.sh does) and compare the `CHK` lines.
module rr_arb_tree_bench_tb #(
  /// Cycles to simulate; overwritten by +cycles=N
  parameter int unsigned NumCycles = 20000,
  /// Cycles between two `CHK` lines
  parameter int unsigned ChkPeriod = 1000
);

  localparam time CyclTime = 10ns;

  // The instances: NumIn, DataWidth, ExtPrio, AxiVldRdy, LockIn, FairArb, Density
  //   0..15  bank arbiters of the TCDM crossbar (77 requesters, no payload)
  //  16..19  request muxes (4 ports, wide payload, vld/rdy, locked)
  //  20..21  odd sizes, unfair
  //  22..30  corners: power of two, 1, 2, 3, 16, an external priority, ...
  localparam int unsigned NumCells = 31;
  localparam int unsigned CfgNumIn     [NumCells] = '{
    77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
     4,  4,  4,  4,
     9, 100,
    33, 128,  1,  2,  3, 16, 17,  8, 64};
  localparam int unsigned CfgDataWidth [NumCells] = '{
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   128, 128, 128, 128,
    64, 32,
     8, 16,  8,  8,  8, 32, 45, 64, 12};
  localparam bit          CfgExtPrio   [NumCells] = '{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0,
    1, 0, 0, 0, 0, 0, 1, 0, 0};
  localparam bit          CfgAxiVldRdy [NumCells] = '{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1,
    0, 1,
    0, 0, 1, 0, 1, 1, 0, 0, 1};
  localparam bit          CfgLockIn    [NumCells] = '{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1,
    0, 1,
    0, 1, 0, 0, 1, 1, 0, 1, 0};
  localparam bit          CfgFairArb   [NumCells] = '{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1,
    0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 0};
  localparam int unsigned CfgDensity   [NumCells] = '{
    3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0,
    3, 2, 1, 0,
    2, 3,
    2, 1, 3, 2, 3, 0, 2, 3, 1};

  logic clk, rst_n;

  initial begin
    clk = 1'b0;
    forever #(CyclTime/2) clk = ~clk;
  end

  logic [63:0] hash    [NumCells];
  logic [31:0] xcycles [NumCells];
  logic [31:0] errors  [NumCells];

  for (genvar c = 0; c < NumCells; c++) begin : gen_cell
    rr_arb_tree_bench_cell #(
      .NumIn     ( CfgNumIn[c]     ),
      .DataWidth ( CfgDataWidth[c] ),
      .ExtPrio   ( CfgExtPrio[c]   ),
      .AxiVldRdy ( CfgAxiVldRdy[c] ),
      .LockIn    ( CfgLockIn[c]    ),
      .FairArb   ( CfgFairArb[c]   ),
      .Density   ( CfgDensity[c]   ),
      .Seed      ( c + 1           )
    ) i_cell (
      .clk_i     ( clk         ),
      .rst_ni    ( rst_n       ),
      .hash_o    ( hash[c]     ),
      .xcycles_o ( xcycles[c]  ),
      .errors_o  ( errors[c]   )
    );
  end

  function automatic logic [63:0] total_hash();
    logic [63:0] r;
    r = '0;
    for (int unsigned c = 0; c < NumCells; c++) r = {r[62:0], r[63]} ^ hash[c];
    return r;
  endfunction

  // Two requests of a DUT driven directly. An unknown request that does not reach the decision
  // must stay out of the outputs, one that does must not be resolved.
  logic [7:0] x_req;
  logic       x_req_o;
  logic [7:0] x_gnt_o, x_data_o;
  logic [2:0] x_idx_o;
  logic [7:0][7:0] x_data;
  rr_arb_tree #(
    .NumIn     ( 8 ),
    .DataWidth ( 8 )
  ) i_x_dut (
    .clk_i   ( clk      ),
    .rst_ni  ( rst_n    ),
    .flush_i ( 1'b0     ),
    .rr_i    ( '0       ),
    .req_i   ( x_req    ),
    .gnt_o   ( x_gnt_o  ),
    .data_i  ( x_data   ),
    .req_o   ( x_req_o  ),
    .gnt_i   ( 1'b1     ),
    .data_o  ( x_data_o ),
    .idx_o   ( x_idx_o  )
  );

  int unsigned x_errors;
  initial begin : proc_x_check
    x_errors = 0;
    for (int unsigned i = 0; i < 8; i++) x_data[i] = 8'(i + 1);
    x_req = '0;
    @(posedge rst_n);
    // request 0 wins with request 5 unknown: the decision is request 0's
    x_req = 8'b00x00001;
    #(CyclTime/4);
    if (x_req_o !== 1'b1 || x_idx_o !== 3'd0 || x_data_o !== 8'd1 || x_gnt_o !== 8'b00000001) begin
      x_errors++;
      $display("X CHECK 1 FAILED: req %b idx %b data %b gnt %b", x_req_o, x_idx_o, x_data_o, x_gnt_o);
    end
    // all requests unknown: the decision is unknown
    x_req = 8'bxxxxxxxx;
    #(CyclTime/4);
    if (!$isunknown(x_idx_o) || !$isunknown(x_data_o)) begin
      x_errors++;
      $display("X CHECK 2 FAILED: idx %b data %b", x_idx_o, x_data_o);
    end
    x_req = '0;
  end

  // run
  initial begin : proc_run
    automatic int unsigned cycles = NumCycles;
    automatic int unsigned bad_x  = 0;
    automatic int unsigned bad_e  = 0;
    void'($value$plusargs("cycles=%d", cycles));
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    for (int unsigned n = 1; n <= cycles; n++) begin
      @(posedge clk);
      if (n % ChkPeriod == 0) begin
        #(CyclTime/4);
        $display("CHK %0d %016h", n, total_hash());
      end
    end
    #(CyclTime/4);
    for (int unsigned c = 0; c < NumCells; c++) begin
      bad_x += xcycles[c];
      bad_e += errors[c];
    end
    $display("BENCH cells=%0d cycles=%0d hash=%016h unknown_cycles=%0d grant_errors=%0d x_check_errors=%0d",
             NumCells, cycles, total_hash(), bad_x, bad_e, x_errors);
    $finish;
  end

endmodule
