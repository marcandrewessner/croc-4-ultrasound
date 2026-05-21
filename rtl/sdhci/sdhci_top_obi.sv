// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "register_interface/typedef.svh"

module sdhci_top_obi #(
  parameter obi_pkg::obi_cfg_t ObiCfg            = obi_pkg::ObiDefaultConfig,
  parameter type               obi_req_t         = logic,
  parameter type               obi_rsp_t         = logic,
  parameter int unsigned       ClkPreDiv         = 2,
  parameter int unsigned       NumDebounceCycles = 500_000,
  parameter int unsigned       BufferNumWords    = 256,
  parameter bit                AllowNoncompliantBufferSizes = 1'b0,
  parameter int                TimeoutDivider    = 1
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  obi_req_t obi_req_i,
  output obi_rsp_t obi_rsp_o,

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
  `REG_BUS_TYPEDEF_ALL(
    reg,
    logic [ObiCfg.AddrWidth-1:0],
    logic [ObiCfg.DataWidth-1:0],
    logic [ObiCfg.AddrWidth/8-1:0]
  )
  reg_req_t reg_req;
  reg_rsp_t reg_rsp;

  sdhci_obi_to_reg #(
    .DATA_WIDTH (ObiCfg.DataWidth),
    .ID_WIDTH   (ObiCfg.IdWidth),

    .obi_req_t (obi_req_t),
    .obi_rsp_t (obi_rsp_t),
    .reg_req_t (reg_req_t),
    .reg_rsp_t (reg_rsp_t)
  ) i_obi_to_reg (
    .clk_i,
    .rst_ni,

    .obi_req_i (obi_req_i),
    .obi_rsp_o (obi_rsp_o),
    .reg_req_o (reg_req),
    .reg_rsp_i (reg_rsp)
  );

  sdhci_top #(
    .AddrWidth        (ObiCfg.AddrWidth),
    .reg_req_t        (reg_req_t),
    .reg_rsp_t        (reg_rsp_t),
    .ClkPreDiv        (ClkPreDiv),
    .NumDebounceCycles(NumDebounceCycles),
    .BufferNumWords   (BufferNumWords),
    .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes),
    .TimeoutDivider   (TimeoutDivider)
  ) i_sdhci_impl (
    .clk_i,
    .rst_ni,

    .reg_req_i(reg_req),
    .reg_rsp_o(reg_rsp),

    .sd_clk_o,
    .sd_cd_ni,

    .sd_cmd_en_o,
    .sd_cmd_o,
    .sd_cmd_i,

    .sd_dat_i,
    .sd_dat_o,
    .sd_dat_en_o,

    .interrupt_o
  );
endmodule
