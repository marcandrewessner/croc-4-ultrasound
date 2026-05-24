// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

module sdhci_fixture #(
    parameter time         ClkPeriod      = 50ns,
    parameter int unsigned RstCycles      = 1,
    parameter int unsigned TimeoutDivider = 1,
    parameter int unsigned BufferNumWords = 256,
    parameter bit          AllowNoncompliantBufferSizes = 1'b0
)();
  `include "obi/typedef.svh"

  logic clk, rst_n;

  localparam obi_pkg::obi_cfg_t sdhci_obi_cfg = obi_pkg::obi_default_cfg(32, 32, 1, '0);
  `OBI_TYPEDEF_DEFAULT_ALL(sdhci_obi, sdhci_obi_cfg);

  sdhci_obi_req_t obi_req;
  sdhci_obi_rsp_t obi_rsp;

  logic sdhc_dat_en, sdhc_cmd_en, sdhc_cmd, tb_cmd;
  logic [3:0] sdhc_dat, tb_dat;
  logic sd_clk, sd_cd;
  logic interrupt;

  sdhci_top_obi #(
      .ObiCfg           (sdhci_obi_cfg),
      .obi_req_t        (sdhci_obi_req_t),
      .obi_rsp_t        (sdhci_obi_rsp_t),
      .ClkPreDiv        (2),
      .NumDebounceCycles(2),
      .BufferNumWords   (BufferNumWords),
      .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes),
      .TimeoutDivider   (TimeoutDivider)
  ) i_sdhci_top (
      .clk_i  (clk),
      .rst_ni (rst_n),

      .obi_req_i  (obi_req),
      .obi_rsp_o  (obi_rsp),
      .sd_clk_o   (sd_clk),
      .sd_cd_ni   (sd_cd),

      .sd_cmd_i   (tb_cmd     ),
      .sd_cmd_o   (sdhc_cmd   ),
      .sd_cmd_en_o(sdhc_cmd_en),

      .sd_dat_i   (tb_dat     ),
      .sd_dat_o   (sdhc_dat   ),
      .sd_dat_en_o(sdhc_dat_en),

      .interrupt_o(interrupt)
  );

  sdhci_vip #(
    .obi_req_t(sdhci_obi_req_t),
    .obi_rsp_t(sdhci_obi_rsp_t)
  ) vip (
    .clk_o      (clk),
    .rst_no     (rst_n),

    .obi_req_o  (obi_req),
    .obi_rsp_i  (obi_rsp),
    .sd_clk_i   (sd_clk),
    .sd_cd_no   (sd_cd),

    .sd_cmd_o   (tb_cmd),
    .sd_cmd_i   (sdhc_cmd),
    .sd_cmd_en_i(sdhc_cmd_en),

    .sd_dat_o   (tb_dat),
    .sd_dat_i   (sdhc_dat),
    .sd_dat_en_i(sdhc_dat_en),

    .interrupt_i(interrupt)
  );

endmodule
