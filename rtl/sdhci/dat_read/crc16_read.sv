// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Anton Buchner <abuchner@student.ethz.ch>

`include "common_cells/registers.svh"

module crc16_read (
  input   logic   clk_i,
  input   logic   sd_clk_en_i,
  input   logic   rst_ni,
  input   logic   clear_i,

  input   logic   shift_in_i,

  input   logic   dat_ser_i,
  output  logic   [15:0]  crc16_o
);
  logic [4:0] lower_5_d,  lower_5_q;
  logic [6:0] middle_7_d, middle_7_q;
  logic [3:0] upper_4_d,  upper_4_q;
  logic dat_i_xor_out;

  always_comb begin : crc_data_path
    crc16_o       = { upper_4_q, middle_7_q, lower_5_q };
    dat_i_xor_out = (dat_ser_i ^ upper_4_q[3]);

    if (shift_in_i) begin
       upper_4_d[3:1] =  upper_4_q[2:0];
       upper_4_d[0]   = middle_7_q[6] ^ dat_i_xor_out;
      middle_7_d[6:1] = middle_7_q[5:0];
      middle_7_d[0]   =  lower_5_q[4] ^ dat_i_xor_out;
       lower_5_d[4:1] =  lower_5_q[3:0];
       lower_5_d[0]   = dat_i_xor_out;
    end else begin
       upper_4_d = '0;
      middle_7_d = '0;
       lower_5_d = '0;
    end
  end

  `FFLARNC ( lower_5_q,  lower_5_d, sd_clk_en_i, clear_i, '0, clk_i, rst_ni);
  `FFLARNC (middle_7_q, middle_7_d, sd_clk_en_i, clear_i, '0, clk_i, rst_ni);
  `FFLARNC ( upper_4_q,  upper_4_d, sd_clk_en_i, clear_i, '0, clk_i, rst_ni);
endmodule
