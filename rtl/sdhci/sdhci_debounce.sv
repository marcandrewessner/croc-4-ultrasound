// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "common_cells/registers.svh"

module sdhci_debounce #(
  parameter int unsigned NumCycles = 256,
  localparam int unsigned CycleWidth = $clog2(NumCycles)
) (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic data_i,
  output logic stable_o,
  output logic data_o
);

  logic data_i_q;
  `FF(data_i_q, data_i, '0);

  logic [CycleWidth-1:0] counter_q;
  logic [CycleWidth-1:0] counter_d;
  `FF(counter_q, counter_d, '0);

  logic data_q, data_d;
  `FF(data_q, data_d, '0);

  logic stable;

  always_comb begin
    counter_d = counter_q + 1;
    data_d = data_q;
    if (data_i != data_i_q) begin
      counter_d = '0;
    end else if (counter_q == NumCycles) begin
      counter_d = counter_q;
      data_d = data_i_q;
    end
  end

  assign stable = (counter_q == NumCycles);

  // forward one cycle
  assign data_o = stable ? data_i_q : data_q;
  assign stable_o = stable;

endmodule
