// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module sdhci_obi_driver #(
  parameter type obi_req_t = logic,
  parameter type obi_rsp_t = logic,
  parameter time TA = 5ns,
  parameter time TT = 15ns
)(
  input  logic clk_i,
  input  logic rst_ni,

  output obi_req_t obi_req_o,
  input  obi_rsp_t obi_rsp_i
);

  initial begin
    obi_req_o = '0;
  end

  task automatic obi_write(
    logic [31:0] address,
    logic [3:0]  be,
    logic [31:0] data,
    logic finish_transaction = 1'b1
  );
    @(posedge clk_i);
    #(TA);
    obi_req_o.a.we    = 1'b1;
    obi_req_o.a.addr  = address;
    obi_req_o.a.be    = be;
    obi_req_o.a.wdata = data;
    obi_req_o.req = 1'b1;

    // wait for the interface to be ready
    #(TT - TA);
    while (obi_rsp_i.gnt == 1'b0) begin
      @(posedge clk_i)
      #(TT);
    end

    if (finish_transaction) begin
      @(posedge clk_i)
      #(TA);
      obi_req_o.req = 1'b0;
    end
  endtask

  task automatic obi_read(
    logic [31:0] address,
    logic [3:0]  be,
    output logic [31:0] data
  );
    @(posedge clk_i);
    #(TA);
    obi_req_o.a.we    = 1'b0;
    obi_req_o.a.addr  = address;
    obi_req_o.a.be    = be;
    obi_req_o.req = 1'b1;

    // wait for the interface to be ready
    #(TT - TA);
    while (obi_rsp_i.gnt == 1'b0) begin
      @(posedge clk_i)
      #(TT);
    end

    @(posedge clk_i)
    #(TA);
    obi_req_o.req = 1'b0;

    #(TT - TA);
    while (obi_rsp_i.rvalid == 1'b0) begin
      @(posedge clk_i)
      #(TT);
    end
    data = obi_rsp_i.r.rdata;
  endtask

  task automatic set_interrupt_status_enable(
    logic [15:0] normal_interrupt_status_enable = '0,
    logic [15:0] error_interrupt_status_enable  = '0,
    logic set_normal_ise = 1'b1,
    logic set_error_ise = 1'b1,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    if (set_normal_ise) begin
      be[1:0] = '1;
    end
    if (set_error_ise) begin
      be[3:2] = '1;
    end

    obi_write('h034, be, {error_interrupt_status_enable, normal_interrupt_status_enable}, finish_transaction);
  endtask

  task automatic set_interrupt_signal_enable(
    logic [15:0] normal_interrupt_signal_enable = '0,
    logic [15:0] error_interrupt_signal_enable  = '0,
    logic set_normal_ise = 1'b1,
    logic set_error_ise = 1'b1,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    if (set_normal_ise) begin
      be[1:0] = '1;
    end
    if (set_error_ise) begin
      be[3:2] = '1;
    end

    obi_write('h038, be, {error_interrupt_signal_enable, normal_interrupt_signal_enable}, finish_transaction);
  endtask

  task automatic set_frequency_select(
    logic [7:0] divider,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b0010;
    obi_write('h02C, be, {16'b0, divider, 8'b0}, finish_transaction);
  endtask

  task automatic set_clock_enable(
    logic enable,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b0001;
    obi_write('h02C, be, {16'b0, 5'b0, enable, 2'b0}, finish_transaction);
  endtask

  task automatic set_transfer_mode(
    logic is_multi_block,
    logic is_read,
    logic auto_cmd12_enable,
    logic block_count_enable,
    logic dma_enable,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b0001;
    obi_write('h00C, be, {24'b0, 2'b0, is_multi_block, is_read, 1'b0,
                          auto_cmd12_enable, block_count_enable, dma_enable}, finish_transaction);
  endtask

  task automatic set_host_control_1(
    /* logic       card_detect_signal_selection, */
    /* logic       card_detect_test_level, */
    /* logic       do_8_bit_transfer, */
    logic [1:0] dma_select,
    logic       high_speed_enable,
    logic       do_4_bit_transfer,
    /* logic       led_control, */
    logic finish_transaction
  );
    logic [3:0] be;
    be = 4'b0001;
    obi_write('h028, be, {24'b0, 3'b0, dma_select, high_speed_enable, do_4_bit_transfer, 1'b0}, finish_transaction);
  endtask

  task automatic set_block_size_count(
    logic [11:0] block_size,
    logic [15:0] block_count,
    logic set_size = 1'b1,
    logic set_count = 1'b1,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b0000;
    if (set_size == 1'b1) begin
      be[1:0] = 2'b11;
    end
    if (set_count == 1'b1) begin
      be[3:2] = 2'b11;
    end

    obi_write('h004, be, {block_count, 4'b0, block_size}, finish_transaction);
  endtask

  task automatic set_data_timeout(
    logic [3:0] exponent_minus_13,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b0100;

    obi_write('h02c, be, {24'b0, 4'b0, exponent_minus_13}, finish_transaction);
  endtask

  task automatic launch_command(
    logic [5:0] command_index,
    logic [1:0] command_type,
    logic data_present,
    logic index_check_enable,
    logic crc_check_enable,
    logic [1:0] response_type,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b1100;
    obi_write('h00C, be, {2'b0, command_index, command_type, data_present, index_check_enable,
                          crc_check_enable, 1'b0, response_type, 16'b0}, finish_transaction);
  endtask

  task automatic read_buffer_data(
    output logic [31:0] data
  );
    logic [3:0] be;
    be = 4'b1111;
    obi_read('h020, be, data);
  endtask

  task automatic write_buffer_data(
    logic [31:0] data,
    logic finish_transaction = 1'b1
  );
    logic [3:0] be;
    be = 4'b1111;
    obi_write('h020, be, data, finish_transaction);
  endtask

  task automatic get_interrupt_status(
    output logic [15:0] normal_interrupt_status,
    output logic [15:0] error_interrupt_status
  );
    logic [3:0] be;
    be = 4'b1111;
    obi_read('h030, be, {error_interrupt_status, normal_interrupt_status});
  endtask

  task automatic clear_interrupt_status(
    input logic [15:0] normal_interrupt_status,
    input logic [15:0] error_interrupt_status
  );
    logic [3:0] be;
    be = 4'b1111;
    obi_write('h030, be, {error_interrupt_status, normal_interrupt_status}, 1'b1);
  endtask

  task automatic get_acmd_error_status(
    output logic [7:0] error_status
  );
    logic [3:0] be;
    logic [31:0] response;
    be = 4'b0001;
    obi_read('h03C, be, response);
    error_status = response[7:0];
  endtask

  task automatic get_present_status_buffer_enable(
    output logic buffer_read_enable,
    output logic buffer_write_enable
  );
    logic [3:0] be;
    logic [31:0] response;
    be = 4'b0011;
    obi_read('h024, be, response);
    buffer_read_enable = response[11];
    buffer_write_enable = response[10];
  endtask
endmodule
