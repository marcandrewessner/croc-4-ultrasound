// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Philippe Sauter <phsauter@iis.ee.ethz.ch>

module sdhci_top_synth #(
  parameter int unsigned AddrWidth = 32'd32,
  parameter type         reg_req_t   = struct packed { logic [31:0] addr; logic write; logic [31:0] wdata; logic [3:0] wstrb; logic valid; },
  parameter type         reg_rsp_t   = struct packed { logic [31:0] rdata; logic error; logic ready; }
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o,

  output logic       sd_clk_o,
  input  logic       sd_cd_ni,
  output logic       sd_cmd_en_o,
  output logic       sd_cmd_o,
  input  logic       sd_cmd_i,

  input  logic [3:0] sd_dat_i,
  output logic [3:0] sd_dat_o,
  output logic       sd_dat_en_o,

  output logic interrupt_o

);

sdhci_top #(
  .AddrWidth      ( 32'd32 ),
  .reg_req_t      ( reg_req_t ),
  .reg_rsp_t      ( reg_rsp_t ),
  .ClkPreDiv      ( 2 ),
  .TimeoutDivider ( 1 ), 
  .NumDebounceCycles ( 500_000 )
) i_top (
  .*
);
 

endmodule
