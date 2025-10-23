// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Yunhao Deng <yunhao.deng@kuleuven.be>

module sync #(
    parameter int unsigned STAGES = 2,
    parameter bit ResetValue = 1'b0
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic serial_i,
    output logic serial_o
);

  (* async *) wire [STAGES:0] d_chain;

  assign d_chain[0] = serial_i;

  for (genvar i = 0; i < STAGES; i++) begin : gen_sync_regs
    (* dont_touch = "true", ASYNC_REG = "true" *) logic d;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        d <= 1'b0;
      end else begin
        d <= d_chain[i];
      end
    end
    assign d_chain[i+1] = d;
  end

  assign serial_o = d_chain[STAGES];

endmodule
