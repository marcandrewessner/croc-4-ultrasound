// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>
// - Micha Wehrli <miwehrli@student.ethz.ch>

module sdhci_vip #(
  parameter type obi_req_t = logic,
  parameter type obi_rsp_t = logic,

  parameter int unsigned RstCycles = 5,

  parameter time ClkPeriod = 20ns,
  parameter time TA = 5ns,
  parameter time TT = 15ns
)(
  output logic clk_o,
  output logic rst_no,

  output obi_req_t obi_req_o,
  input  obi_rsp_t obi_rsp_i,

  input  logic sd_clk_i,
  output logic sd_cd_no,

  output logic sd_cmd_o,
  input  logic sd_cmd_i,
  input  logic sd_cmd_en_i,

  output logic [3:0] sd_dat_o,
  input  logic [3:0] sd_dat_i,
  input  logic       sd_dat_en_i,

  input  logic interrupt_i
);
  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriod ),
    .RstClkCycles ( RstCycles )
  ) i_clk_rst_sys (
    .clk_o  (clk_o ),
    .rst_no (rst_no)
  );

  sdhci_obi_driver #(
    .obi_req_t(obi_req_t),
    .obi_rsp_t(obi_rsp_t),
    .TA(TA),
    .TT(TT)
  ) obi (
    .clk_i    (clk_o),
    .rst_ni   (rst_no),
    .obi_req_o(obi_req_o),
    .obi_rsp_i(obi_rsp_i)
  );

  sdhci_sd_driver #(
    .TA(TA),
    .TT(TT)
  ) sd (
    .sd_clk_i(sd_clk_i),
    .sd_cmd_o(sd_cmd_o),
    .sd_cmd_i(sd_cmd_i),
    .sd_cmd_en_i(sd_cmd_en_i),
    .sd_dat_o(sd_dat_o),
    .sd_dat_i(sd_dat_i),
    .sd_dat_en_i(sd_dat_en_i)
  );

  initial begin
    sd_cd_no  = 1'b1;
  end

  task automatic wait_for_reset();
    @(posedge rst_no);
    @(posedge clk_o);
  endtask

  task automatic wait_for_clk();
    @(posedge clk_o);
  endtask

  task automatic wait_for_sdclk();
    @(posedge sd_clk_i);
  endtask

  task automatic wait_for_interrupt();
    @(posedge clk_o);
    #(TT);
    while (interrupt_i != 1'b1) begin
      @(posedge clk_o);
      #(TT);
    end
  endtask

  task automatic test_delay();
    #(TT);
  endtask

  task automatic set_cd(logic card_absent);
    @(posedge clk_o);
    #(TA);
    sd_cd_no = card_absent;
  endtask

endmodule
