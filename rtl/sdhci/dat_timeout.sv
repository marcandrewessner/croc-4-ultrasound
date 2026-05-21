// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "common_cells/registers.svh"

module dat_timeout #(
  parameter int unsigned ClockDiv = 1, // by how much to divide the clock to get the timeout clock
                              // make sure that the chosen divider allows
                              // enough time with the maximum timeout of 2**27 cycles:
                              // 500ms timeout is required for some newer SD cards,
                              // the spec specifies maximum timeout for erasing to be
                              // up to 3 minutes.
                              // Note: Linux also has larger timeouts that
                              // span to 60s, so err on the side of caution
  localparam int unsigned ClockDivWidth = $clog2(ClockDiv)
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic       running_i, // High as long as we are waiting. Pulling low resets the counter
  input  logic [3:0] timeout_bits_i,

  output logic       timeout_o
);
  // maximum timeout is 2**27
  logic [27:0] timeout_counter_q, timeout_counter_d;
  `FF (timeout_counter_q, timeout_counter_d, 0, clk_i, rst_ni);

  logic do_increment;

  generate
    if (ClockDiv > 1) begin
      logic [ClockDivWidth-1:0] clock_div_q, clock_div_d;
      `FF(clock_div_q, clock_div_d, '0, clk_i, rst_ni);
      
      always_comb begin
        clock_div_d = clock_div_q + 1;
        if (clock_div_q == ClockDiv - 1) begin
          clock_div_d = '0;
        end
      end

      assign do_increment = clock_div_q == '0;
    end else begin
      assign do_increment = 1'b1;
    end
  endgenerate
  
  always_comb begin
    timeout_counter_d = timeout_counter_q;
    // reset on low running_i
    if (!running_i) begin
      timeout_counter_d = '0;
    end else if (!timeout_o && do_increment) begin
      timeout_counter_d = timeout_counter_q + 1;
    end
  end
  assign timeout_o = (timeout_counter_q >= (1 << (timeout_bits_i + 13)));
endmodule
