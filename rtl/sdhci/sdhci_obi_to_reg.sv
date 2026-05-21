// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// OBI to register-interface bridge for the SDHCI top. Unlike the generic
// adapter this only returns a response for requests accepted by the register
// side, so Buffer Data Port wait states do not create spurious rvalid pulses.
module sdhci_obi_to_reg #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ID_WIDTH   = 0,

  parameter type         obi_req_t = logic,
  parameter type         obi_rsp_t = logic,

  parameter type         reg_req_t = logic,
  parameter type         reg_rsp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  obi_req_t obi_req_i,
  output obi_rsp_t obi_rsp_o,

  output reg_req_t reg_req_o,
  input  reg_rsp_t reg_rsp_i
);
  logic accepted;
  logic rsp_valid_q;
  logic [ID_WIDTH-1:0] aid_q;
  logic [DATA_WIDTH-1:0] rdata_q;
  logic error_q;

  assign accepted = obi_req_i.req & reg_rsp_i.ready;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rsp_valid_q <= 1'b0;
      aid_q       <= '0;
      rdata_q     <= '0;
      error_q     <= 1'b0;
    end else begin
      rsp_valid_q <= accepted;
      if (accepted) begin
        aid_q   <= obi_req_i.a.aid;
        rdata_q <= reg_rsp_i.rdata;
        error_q <= reg_rsp_i.error;
      end
    end
  end

  assign reg_req_o.valid = obi_req_i.req;
  assign reg_req_o.addr  = obi_req_i.a.addr;
  assign reg_req_o.write = obi_req_i.a.we;
  assign reg_req_o.wdata = obi_req_i.a.wdata;
  assign reg_req_o.wstrb = obi_req_i.a.be;

  always_comb begin
    obi_rsp_o          = '0;
    obi_rsp_o.gnt      = accepted;
    obi_rsp_o.rvalid   = rsp_valid_q;
    obi_rsp_o.r.rdata  = rdata_q;
    obi_rsp_o.r.rid    = aid_q;
    obi_rsp_o.r.err    = error_q;
  end
endmodule
