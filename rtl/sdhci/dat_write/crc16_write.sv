// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Anton Buchner <abuchner@student.ethz.ch>

`include "common_cells/registers.svh"

module crc16_write (
  input logic clk_i,
  input logic sd_clk_en_i,
  input logic rst_ni,
  input logic clear_i,

  input logic shift_out_crc16_i,
  input logic dat_ser_i,

  output logic crc_ser_o
);
  logic [4:0] lower_5_d,  lower_5_q;
  logic [6:0] middle_7_d, middle_7_q;
  logic [3:0] upper_4_d,  upper_4_q;
  logic dat_i_xor_out;

  always_comb begin : crc16_comb
    lower_5_d   = lower_5_q;
    middle_7_d  = middle_7_q;
    upper_4_d   = upper_4_q;

    crc_ser_o     = upper_4_q[3];
    dat_i_xor_out = upper_4_q[3] ^ dat_ser_i;

    if (shift_out_crc16_i) begin : shift_out_result
       upper_4_d[3:1] =  upper_4_q[2:0];
       upper_4_d[0]   = middle_7_q[6];
      middle_7_d[6:1] = middle_7_q[5:0];
      middle_7_d[0]   =  lower_5_q[4];
       lower_5_d[4:1] =  lower_5_q[3:0];
       lower_5_d[0]   = 1'b0; //shift in zeros while shifting out crc, no reset needed.
    end else begin : calc_crc16
       upper_4_d[3:1] =  upper_4_q[2:0];
       upper_4_d[0]   = middle_7_q[6] ^ dat_i_xor_out;
      middle_7_d[6:1] = middle_7_q[5:0];
      middle_7_d[0]   =  lower_5_d[4] ^ dat_i_xor_out;
       lower_5_d[4:1] =  lower_5_q[3:0];
       lower_5_d[0]   = dat_i_xor_out;
    end
  end

  `FFLARNC ( upper_4_q,  upper_4_d, sd_clk_en_i, clear_i, 0, clk_i, rst_ni);
  `FFLARNC (middle_7_q, middle_7_d, sd_clk_en_i, clear_i, 0, clk_i, rst_ni);
  `FFLARNC ( lower_5_q,  lower_5_d, sd_clk_en_i, clear_i, 0, clk_i, rst_ni);

`ifdef VERILATOR
  logic [15:0] crc;
  assign crc = { upper_4_q, middle_7_q, lower_5_q };
`endif
endmodule
