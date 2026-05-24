// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

module tb_reg_reset #(
  parameter time ClkPeriod = 20ns
)();
  typedef struct packed {
    logic [31:0] addr;
    logic        write;
    logic [31:0] wdata;
    logic [3:0]  wstrb;
    logic        valid;
  } reg_req_t;

  typedef struct packed {
    logic [31:0] rdata;
    logic        error;
    logic        ready;
  } reg_rsp_t;

  localparam logic [31:0] BlockSizeCountOffset = 32'h004;
  localparam logic [31:0] ArgumentOffset       = 32'h008;
  localparam logic [31:0] TransferModeOffset   = 32'h00c;
  localparam logic [31:0] HostControlOffset    = 32'h028;
  localparam logic [31:0] ClockResetOffset     = 32'h02c;
  localparam logic [31:0] IntStatusEnOffset    = 32'h034;
  localparam logic [31:0] IntSignalEnOffset    = 32'h038;

  logic clk;
  logic rst_n;
  reg_req_t reg_req;
  reg_rsp_t reg_rsp;
  logic sd_clk;
  logic sd_cmd_en;
  logic sd_cmd;
  logic [3:0] sd_dat;
  logic sd_dat_en;
  logic interrupt;

  clk_rst_gen #(
    .ClkPeriod    (ClkPeriod),
    .RstClkCycles (2)
  ) i_clk_rst_sys (
    .clk_o  (clk),
    .rst_no (rst_n)
  );

  sdhci_top #(
    .AddrWidth (32),
    .reg_req_t (reg_req_t),
    .reg_rsp_t (reg_rsp_t)
  ) i_dut (
    .clk_i     (clk),
    .rst_ni    (rst_n),
    .reg_req_i (reg_req),
    .reg_rsp_o (reg_rsp),
    .sd_clk_o  (sd_clk),
    .sd_cd_ni  (1'b0),
    .sd_cmd_en_o(sd_cmd_en),
    .sd_cmd_o  (sd_cmd),
    .sd_cmd_i  (1'b1),
    .sd_dat_i  (4'hf),
    .sd_dat_o  (sd_dat),
    .sd_dat_en_o(sd_dat_en),
    .interrupt_o(interrupt)
  );

  task automatic reg_write(input logic [31:0] addr, input logic [3:0] be,
                           input logic [31:0] data);
    @(posedge clk);
    #1ns;
    reg_req.addr  = addr;
    reg_req.write = 1'b1;
    reg_req.wdata = data;
    reg_req.wstrb = be;
    reg_req.valid = 1'b1;
    #1ns;
    while (!reg_rsp.ready) begin
      @(posedge clk);
      #1ns;
    end
    @(posedge clk);
    #1ns;
    reg_req = '0;
  endtask

  task automatic reg_read(input logic [31:0] addr, input logic [3:0] be,
                          output logic [31:0] data);
    @(posedge clk);
    #1ns;
    reg_req.addr  = addr;
    reg_req.write = 1'b0;
    reg_req.wdata = '0;
    reg_req.wstrb = be;
    reg_req.valid = 1'b1;
    #1ns;
    while (!reg_rsp.ready) begin
      @(posedge clk);
      #1ns;
    end
    data = reg_rsp.rdata;
    @(posedge clk);
    #1ns;
    reg_req = '0;
  endtask

  task automatic expect_masked(input string name, input logic [31:0] actual,
                               input logic [31:0] expected, input logic [31:0] mask);
    if ((actual & mask) !== (expected & mask)) begin
      $fatal(1, "%s mismatch: got 0x%08x expected 0x%08x mask 0x%08x",
             name, actual, expected, mask);
    end
  endtask

  task automatic write_nonzero_register_state();
    reg_write(BlockSizeCountOffset, 4'b1111, 32'h0003_0200);
    reg_write(ArgumentOffset,       4'b1111, 32'h89ab_cdef);
    reg_write(TransferModeOffset,   4'b0001, 32'h0000_0032);
    reg_write(HostControlOffset,    4'b0001, 32'h0000_0002);
    reg_write(ClockResetOffset,     4'b0011, 32'h0000_0105);
    reg_write(ClockResetOffset,     4'b0100, 32'h000a_0000);
    reg_write(IntStatusEnOffset,    4'b1111, 32'h001f_00f3);
    reg_write(IntSignalEnOffset,    4'b1111, 32'h001f_00f3);
    repeat (2) @(posedge clk);
  endtask

  task automatic expect_nonzero_register_state(input string reset_context,
                                               input logic dat_counter_cleared = 1'b0);
    logic [31:0] data;
    reg_read(BlockSizeCountOffset, 4'b1111, data);
    expect_masked({reset_context, " block_size/count"}, data,
                  dat_counter_cleared ? 32'h0000_0200 : 32'h0003_0200, 32'hffff_0fff);
    reg_read(ArgumentOffset, 4'b1111, data);
    expect_masked({reset_context, " argument"}, data, 32'h89ab_cdef, 32'hffff_ffff);
    reg_read(TransferModeOffset, 4'b0001, data);
    expect_masked({reset_context, " transfer_mode"}, data, 32'h0000_0032, 32'h0000_003f);
    reg_read(HostControlOffset, 4'b0001, data);
    expect_masked({reset_context, " host_control"}, data, 32'h0000_0002, 32'h0000_0002);
    reg_read(ClockResetOffset, 4'b0111, data);
    expect_masked({reset_context, " clock/timeout"}, data, 32'h000a_0105, 32'h000f_ff05);
    reg_read(IntStatusEnOffset, 4'b1111, data);
    expect_masked({reset_context, " status_enable"}, data, 32'h001f_00f3, 32'hffff_ffff);
    reg_read(IntSignalEnOffset, 4'b1111, data);
    expect_masked({reset_context, " signal_enable"}, data, 32'h001f_00f3, 32'hffff_ffff);
  endtask

  task automatic expect_cleared_register_state();
    logic [31:0] data;
    reg_read(BlockSizeCountOffset, 4'b1111, data);
    expect_masked("all reset block_size/count", data, 32'h0, 32'hffff_0fff);
    reg_read(ArgumentOffset, 4'b1111, data);
    expect_masked("all reset argument", data, 32'h0, 32'hffff_ffff);
    reg_read(TransferModeOffset, 4'b0001, data);
    expect_masked("all reset transfer_mode", data, 32'h0, 32'h0000_003f);
    reg_read(HostControlOffset, 4'b0001, data);
    expect_masked("all reset host_control", data, 32'h0, 32'h0000_0002);
    reg_read(ClockResetOffset, 4'b0111, data);
    expect_masked("all reset clock/timeout", data, 32'h0, 32'h000f_ff05);
    reg_read(IntStatusEnOffset, 4'b1111, data);
    expect_masked("all reset status_enable", data, 32'h0, 32'hffff_ffff);
    reg_read(IntSignalEnOffset, 4'b1111, data);
    expect_masked("all reset signal_enable", data, 32'h0, 32'hffff_ffff);
  endtask

  task automatic software_reset(input logic [2:0] reset_bits);
    reg_write(ClockResetOffset, 4'b1000, {5'b0, reset_bits, 24'b0});
    repeat (6) @(posedge clk);
  endtask

  initial begin
    reg_req = '0;
    wait(rst_n);
    repeat (4) @(posedge clk);

    write_nonzero_register_state();
    software_reset(3'b010);
    expect_nonzero_register_state("cmd reset");

    software_reset(3'b100);
    expect_nonzero_register_state("dat reset", 1'b1);

    software_reset(3'b001);
    expect_cleared_register_state();

    repeat (10) @(posedge clk);
    $finish();
  end
endmodule
