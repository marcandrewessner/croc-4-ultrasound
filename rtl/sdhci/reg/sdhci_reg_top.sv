// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module sdhci_reg_top #(
  parameter type reg_req_t = logic,
  parameter type reg_rsp_t = logic,
  parameter int AW = 8,
  parameter int unsigned BufferNumWords = 256,
  parameter bit AllowNoncompliantBufferSizes = 1'b0
) (
  input  logic clk_i,
  input  logic rst_ni,
  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o,

  output sdhci_reg_pkg::sdhci_reg2hw_t reg2hw,
  input  sdhci_reg_pkg::sdhci_hw2reg_t hw2reg,
  input  logic clear_i,
  input  logic buffer_data_port_read_ready_i,
  input  logic buffer_data_port_write_ready_i,

  input devmode_i
);

  import sdhci_reg_pkg::*;

  localparam int DW  = 32;
  localparam int DBW = DW / 8;

  localparam logic [DW-1:0] Capabilities = 32'h0100_32b2;
  localparam logic [15:0] BufferDataPortChunkBytes =
      AllowNoncompliantBufferSizes ? 16'(BufferNumWords * 4) : 16'd512;
  localparam logic [DW-1:0] VendorCapabilities = {
      AllowNoncompliantBufferSizes, 15'h0, BufferDataPortChunkBytes};

  sdhci_reg2hw_t reg2hw_q, reg2hw_d;
  sdhci_reg2hw_t reg2hw_o;

  logic [BlockAw-1:0] reg_addr;
  logic [DW-1:0]      reg_wdata;
  logic [DBW-1:0]     reg_be;
  logic               reg_we;
  logic               reg_re;
  logic               reg_ready;
  logic               accepted;
  logic               addrmiss;
  logic               wr_err;

  function automatic logic boundary_err(logic [DBW-1:0] be, logic [2:0] disallowed);
    boundary_err = |((be ^ (be >> 1)) & disallowed);
  endfunction

  function automatic logic w1c_next(
    logic q,
    logic d,
    logic de,
    logic write_clear,
    logic wd
  );
    w1c_next = de ? d : q;
    if (write_clear && wd) begin
      w1c_next = 1'b0;
    end
  endfunction

  function automatic logic w1s_next(
    logic q,
    logic d,
    logic de,
    logic write_set,
    logic wd
  );
    w1s_next = de ? d : q;
    if (write_set && wd) begin
      w1s_next = 1'b1;
    end
  endfunction

  assign reg_addr  = {reg_req_i.addr[BlockAw-1:2], 2'b00};
  assign reg_wdata = reg_req_i.wdata;
  assign reg_be    = reg_req_i.wstrb;
  assign reg_we    = reg_req_i.valid & reg_req_i.write;
  assign reg_re    = reg_req_i.valid & ~reg_req_i.write;

  always_comb begin
    addrmiss = 1'b0;

    unique case (reg_addr)
      SDHCI_SYSTEM_ADDRESS_OFFSET,
      SDHCI_BLOCK_SIZE_OFFSET,
      SDHCI_ARGUMENT_OFFSET,
      SDHCI_TRANSFER_MODE_OFFSET,
      SDHCI_RESPONSE0_OFFSET,
      SDHCI_RESPONSE1_OFFSET,
      SDHCI_RESPONSE2_OFFSET,
      SDHCI_RESPONSE3_OFFSET,
      SDHCI_BUFFER_DATA_PORT_OFFSET,
      SDHCI_PRESENT_STATE_OFFSET,
      SDHCI_HOST_CONTROL_OFFSET,
      SDHCI_CLOCK_CONTROL_OFFSET,
      SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET,
      SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE_OFFSET,
      SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE_OFFSET,
      SDHCI_AUTO_CMD12_ERROR_STATUS_OFFSET,
      SDHCI_CAPABILITIES_OFFSET,
      SDHCI_CAPABILITIES_RESERVED_OFFSET,
      SDHCI_MAXIMUM_CURRENT_CAPABILITIES_OFFSET,
      SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED_OFFSET,
      SDHCI_SLOT_INTERRUPT_STATUS_OFFSET: addrmiss = 1'b0;
      default: addrmiss = reg_req_i.valid;
    endcase
  end

  always_comb begin
    wr_err = 1'b0;

    if (reg_we) begin
      unique case (reg_addr)
        SDHCI_SYSTEM_ADDRESS_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_SYSTEM_ADDRESS]);
        SDHCI_BLOCK_SIZE_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_BLOCK_SIZE]) |
                   boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_BLOCK_COUNT]);
        SDHCI_ARGUMENT_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_ARGUMENT]);
        SDHCI_RESPONSE0_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_RESPONSE0]);
        SDHCI_RESPONSE1_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_RESPONSE1]);
        SDHCI_RESPONSE2_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_RESPONSE2]);
        SDHCI_RESPONSE3_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_RESPONSE3]);
        SDHCI_BUFFER_DATA_PORT_OFFSET:
          wr_err = boundary_err(reg_be, SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_BUFFER_DATA_PORT]);
        SDHCI_CAPABILITIES_RESERVED_OFFSET:
          wr_err = boundary_err(reg_be,
                                SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_CAPABILITIES_RESERVED]);
        SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED_OFFSET:
          wr_err = boundary_err(reg_be,
              SDHCI_DISALLOWED_BOUNDARY_CROSSINGS[SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED]);
        default: wr_err = 1'b0;
      endcase
    end
  end

  always_comb begin
    reg_ready = 1'b1;
    if (!wr_err && !(devmode_i && addrmiss) &&
        reg_addr == SDHCI_BUFFER_DATA_PORT_OFFSET && reg_req_i.valid) begin
      reg_ready = reg_we ? buffer_data_port_write_ready_i : buffer_data_port_read_ready_i;
    end
  end

  assign accepted = reg_req_i.valid & reg_ready & !((devmode_i & addrmiss) | wr_err);

  always_comb begin
    reg_rsp_o.ready = reg_ready;
    reg_rsp_o.error = (devmode_i & addrmiss) | wr_err;
    reg_rsp_o.rdata = '0;

    unique case (reg_addr)
      SDHCI_BLOCK_SIZE_OFFSET: begin
        reg_rsp_o.rdata[11:0]  = hw2reg.block_size.transfer_block_size.d;
        reg_rsp_o.rdata[14:12] = hw2reg.block_size.host_dma_buffer_boundary.d;
        reg_rsp_o.rdata[31:16] = hw2reg.block_count.d;
      end

      SDHCI_ARGUMENT_OFFSET: begin
        reg_rsp_o.rdata = reg2hw_q.argument.q;
      end

      SDHCI_TRANSFER_MODE_OFFSET: begin
        reg_rsp_o.rdata[1]     = hw2reg.transfer_mode.block_count_enable.d;
        reg_rsp_o.rdata[2]     = hw2reg.transfer_mode.auto_cmd12_enable.d;
        reg_rsp_o.rdata[4]     = hw2reg.transfer_mode.data_transfer_direction_select.d;
        reg_rsp_o.rdata[5]     = hw2reg.transfer_mode.multi_single_block_select.d;
        reg_rsp_o.rdata[17:16] = reg2hw_q.command.response_type_select.q;
        reg_rsp_o.rdata[19]    = reg2hw_q.command.command_crc_check_enable.q;
        reg_rsp_o.rdata[20]    = reg2hw_q.command.command_index_check_enable.q;
        reg_rsp_o.rdata[21]    = reg2hw_q.command.data_present_select.q;
        reg_rsp_o.rdata[23:22] = reg2hw_q.command.command_type.q;
        reg_rsp_o.rdata[29:24] = reg2hw_q.command.command_index.q;
      end

      SDHCI_RESPONSE0_OFFSET: reg_rsp_o.rdata = reg2hw_q.response0.q;
      SDHCI_RESPONSE1_OFFSET: reg_rsp_o.rdata = reg2hw_q.response1.q;
      SDHCI_RESPONSE2_OFFSET: reg_rsp_o.rdata = reg2hw_q.response2.q;
      SDHCI_RESPONSE3_OFFSET: reg_rsp_o.rdata = reg2hw_q.response3.q;
      SDHCI_BUFFER_DATA_PORT_OFFSET: reg_rsp_o.rdata = hw2reg.buffer_data_port.d;

      SDHCI_PRESENT_STATE_OFFSET: begin
        reg_rsp_o.rdata[0]     = reg2hw_q.present_state.command_inhibit_cmd.q;
        reg_rsp_o.rdata[1]     = reg2hw_q.present_state.command_inhibit_dat.q;
        reg_rsp_o.rdata[2]     = reg2hw_q.present_state.dat_line_active.q;
        reg_rsp_o.rdata[8]     = reg2hw_q.present_state.write_transfer_active.q;
        reg_rsp_o.rdata[9]     = reg2hw_q.present_state.read_transfer_active.q;
        reg_rsp_o.rdata[10]    = reg2hw_q.present_state.buffer_write_enable.q;
        reg_rsp_o.rdata[11]    = reg2hw_q.present_state.buffer_read_enable.q;
        reg_rsp_o.rdata[16]    = reg2hw_q.present_state.card_inserted.q;
        reg_rsp_o.rdata[17]    = reg2hw_q.present_state.card_state_stable.q;
        reg_rsp_o.rdata[18]    = reg2hw_q.present_state.card_detect_pin_level.q;
        reg_rsp_o.rdata[19]    = reg2hw_q.present_state.write_protect_switch_pin_level.q;
        reg_rsp_o.rdata[23:20] = reg2hw_q.present_state.dat_line_signal_level.q;
        reg_rsp_o.rdata[24]    = reg2hw_q.present_state.cmd_line_signal_level.q;
      end

      SDHCI_HOST_CONTROL_OFFSET: begin
        reg_rsp_o.rdata[1] = reg2hw_q.host_control.data_transfer_width.q;
      end

      SDHCI_CLOCK_CONTROL_OFFSET: begin
        reg_rsp_o.rdata[0]     = reg2hw_q.clock_control.internal_clock_enable.q;
        reg_rsp_o.rdata[1]     = reg2hw_q.clock_control.internal_clock_stable.q;
        reg_rsp_o.rdata[2]     = reg2hw_q.clock_control.sd_clock_enable.q;
        reg_rsp_o.rdata[15:8]  = reg2hw_q.clock_control.sdclk_frequency_select.q;
        reg_rsp_o.rdata[19:16] = reg2hw_q.timeout_control.data_timeout_counter_value.q;
        reg_rsp_o.rdata[24]    = reg2hw_q.software_reset.software_reset_for_all.q;
        reg_rsp_o.rdata[25]    = reg2hw_q.software_reset.software_reset_for_cmd_line.q;
        reg_rsp_o.rdata[26]    = reg2hw_q.software_reset.software_reset_for_dat_line.q;
      end

      SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET: begin
        reg_rsp_o.rdata[0]     = reg2hw_q.normal_interrupt_status.command_complete.q;
        reg_rsp_o.rdata[1]     = reg2hw_q.normal_interrupt_status.transfer_complete.q;
        reg_rsp_o.rdata[4]     = reg2hw_q.normal_interrupt_status.buffer_write_ready.q;
        reg_rsp_o.rdata[5]     = reg2hw_q.normal_interrupt_status.buffer_read_ready.q;
        reg_rsp_o.rdata[6]     = reg2hw_q.normal_interrupt_status.card_insertion.q;
        reg_rsp_o.rdata[7]     = reg2hw_q.normal_interrupt_status.card_removal.q;
        reg_rsp_o.rdata[15]    = reg2hw_q.normal_interrupt_status.error_interrupt.q;
        reg_rsp_o.rdata[16]    = reg2hw_q.error_interrupt_status.command_timeout_error.q;
        reg_rsp_o.rdata[17]    = reg2hw_q.error_interrupt_status.command_crc_error.q;
        reg_rsp_o.rdata[18]    = reg2hw_q.error_interrupt_status.command_end_bit_error.q;
        reg_rsp_o.rdata[19]    = reg2hw_q.error_interrupt_status.command_index_error.q;
        reg_rsp_o.rdata[20]    = reg2hw_q.error_interrupt_status.data_timeout_error.q;
        reg_rsp_o.rdata[21]    = reg2hw_q.error_interrupt_status.data_crc_error.q;
        reg_rsp_o.rdata[22]    = reg2hw_q.error_interrupt_status.data_end_bit_error.q;
        reg_rsp_o.rdata[24]    = reg2hw_q.error_interrupt_status.auto_cmd12_error.q;
      end

      SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE_OFFSET: begin
        reg_rsp_o.rdata[0]     = reg2hw_q.normal_interrupt_status_enable.command_complete_status_enable.q;
        reg_rsp_o.rdata[1]     = reg2hw_q.normal_interrupt_status_enable.transfer_complete_status_enable.q;
        reg_rsp_o.rdata[4]     = reg2hw_q.normal_interrupt_status_enable.buffer_write_ready_status_enable.q;
        reg_rsp_o.rdata[5]     = reg2hw_q.normal_interrupt_status_enable.buffer_read_ready_status_enable.q;
        reg_rsp_o.rdata[6]     = reg2hw_q.normal_interrupt_status_enable.card_insertion_status_enable.q;
        reg_rsp_o.rdata[7]     = reg2hw_q.normal_interrupt_status_enable.card_removal_status_enable.q;
        reg_rsp_o.rdata[8]     = reg2hw_q.normal_interrupt_status_enable.card_interrupt_status_enable.q;
        reg_rsp_o.rdata[15]    = reg2hw_q.normal_interrupt_status_enable.fixed_to_0.q;
        reg_rsp_o.rdata[16]    =
            reg2hw_q.error_interrupt_status_enable.command_timeout_error_status_enable.q;
        reg_rsp_o.rdata[17]    =
            reg2hw_q.error_interrupt_status_enable.command_crc_error_status_enable.q;
        reg_rsp_o.rdata[18]    =
            reg2hw_q.error_interrupt_status_enable.command_end_bit_error_status_enable.q;
        reg_rsp_o.rdata[19]    =
            reg2hw_q.error_interrupt_status_enable.command_index_error_status_enable.q;
        reg_rsp_o.rdata[20]    =
            reg2hw_q.error_interrupt_status_enable.data_timeout_error_status_enable.q;
        reg_rsp_o.rdata[21]    =
            reg2hw_q.error_interrupt_status_enable.data_crc_error_status_enable.q;
        reg_rsp_o.rdata[22]    =
            reg2hw_q.error_interrupt_status_enable.data_end_bit_error_status_enable.q;
        reg_rsp_o.rdata[24]    =
            reg2hw_q.error_interrupt_status_enable.auto_cmd12_error_status_enable.q;
      end

      SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE_OFFSET: begin
        reg_rsp_o.rdata[0]     = reg2hw_q.normal_interrupt_signal_enable.command_complete_signal_enable.q;
        reg_rsp_o.rdata[1]     = reg2hw_q.normal_interrupt_signal_enable.transfer_complete_signal_enable.q;
        reg_rsp_o.rdata[4]     = reg2hw_q.normal_interrupt_signal_enable.buffer_write_ready_signal_enable.q;
        reg_rsp_o.rdata[5]     = reg2hw_q.normal_interrupt_signal_enable.buffer_read_ready_signal_enable.q;
        reg_rsp_o.rdata[6]     = reg2hw_q.normal_interrupt_signal_enable.card_insertion_signal_enable.q;
        reg_rsp_o.rdata[7]     = reg2hw_q.normal_interrupt_signal_enable.card_removal_signal_enable.q;
        reg_rsp_o.rdata[8]     = reg2hw_q.normal_interrupt_signal_enable.card_interrupt_signal_enable.q;
        reg_rsp_o.rdata[16]    =
            reg2hw_q.error_interrupt_signal_enable.command_timeout_error_signal_enable.q;
        reg_rsp_o.rdata[17]    =
            reg2hw_q.error_interrupt_signal_enable.command_crc_error_signal_enable.q;
        reg_rsp_o.rdata[18]    =
            reg2hw_q.error_interrupt_signal_enable.command_end_bit_error_signal_enable.q;
        reg_rsp_o.rdata[19]    =
            reg2hw_q.error_interrupt_signal_enable.command_index_error_signal_enable.q;
        reg_rsp_o.rdata[20]    =
            reg2hw_q.error_interrupt_signal_enable.data_timeout_error_signal_enable.q;
        reg_rsp_o.rdata[21]    =
            reg2hw_q.error_interrupt_signal_enable.data_crc_error_signal_enable.q;
        reg_rsp_o.rdata[22]    =
            reg2hw_q.error_interrupt_signal_enable.data_end_bit_error_signal_enable.q;
        reg_rsp_o.rdata[24]    =
            reg2hw_q.error_interrupt_signal_enable.auto_cmd12_error_signal_enable.q;
      end

      SDHCI_AUTO_CMD12_ERROR_STATUS_OFFSET: begin
        reg_rsp_o.rdata[0] = reg2hw_q.auto_cmd12_error_status.auto_cmd12_not_executed.q;
        reg_rsp_o.rdata[1] = reg2hw_q.auto_cmd12_error_status.auto_cmd12_timeout_error.q;
        reg_rsp_o.rdata[2] = reg2hw_q.auto_cmd12_error_status.auto_cmd12_crc_error.q;
        reg_rsp_o.rdata[3] = reg2hw_q.auto_cmd12_error_status.auto_cmd12_end_bit_error.q;
        reg_rsp_o.rdata[4] = reg2hw_q.auto_cmd12_error_status.auto_cmd12_index_error.q;
        reg_rsp_o.rdata[7] =
            reg2hw_q.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.q;
      end

      SDHCI_CAPABILITIES_OFFSET: reg_rsp_o.rdata = Capabilities;
      SDHCI_CAPABILITIES_RESERVED_OFFSET: reg_rsp_o.rdata = VendorCapabilities;
      SDHCI_SLOT_INTERRUPT_STATUS_OFFSET: begin
        reg_rsp_o.rdata[7:0] = hw2reg.slot_interrupt_status.interrupt_signal_for_each_slot.d;
      end

      default: reg_rsp_o.rdata = '0;
    endcase
  end

  always_comb begin
    reg2hw_d = reg2hw_q;

    reg2hw_d.block_size.transfer_block_size.qe = 1'b0;
    reg2hw_d.block_size.host_dma_buffer_boundary.qe = 1'b0;
    reg2hw_d.block_count.qe = 1'b0;
    reg2hw_d.transfer_mode.block_count_enable.qe = 1'b0;
    reg2hw_d.transfer_mode.auto_cmd12_enable.qe = 1'b0;
    reg2hw_d.transfer_mode.data_transfer_direction_select.qe = 1'b0;
    reg2hw_d.transfer_mode.multi_single_block_select.qe = 1'b0;
    reg2hw_d.command.response_type_select.qe = 1'b0;
    reg2hw_d.command.command_crc_check_enable.qe = 1'b0;
    reg2hw_d.command.command_index_check_enable.qe = 1'b0;
    reg2hw_d.command.data_present_select.qe = 1'b0;
    reg2hw_d.command.command_type.qe = 1'b0;
    reg2hw_d.command.command_index.qe = 1'b0;
    reg2hw_d.buffer_data_port.qe = 1'b0;
    reg2hw_d.buffer_data_port.re = 1'b0;
    reg2hw_d.clock_control.internal_clock_enable.qe = 1'b0;
    reg2hw_d.clock_control.internal_clock_stable.qe = 1'b0;
    reg2hw_d.clock_control.sd_clock_enable.qe = 1'b0;
    reg2hw_d.clock_control.sdclk_frequency_select.qe = 1'b0;

    if (hw2reg.response0.de) begin
      reg2hw_d.response0.q = hw2reg.response0.d;
    end
    if (hw2reg.response1.de) begin
      reg2hw_d.response1.q = hw2reg.response1.d;
    end
    if (hw2reg.response2.de) begin
      reg2hw_d.response2.q = hw2reg.response2.d;
    end
    if (hw2reg.response3.de) begin
      reg2hw_d.response3.q = hw2reg.response3.d;
    end

    if (hw2reg.present_state.command_inhibit_cmd.de) begin
      reg2hw_d.present_state.command_inhibit_cmd.q = hw2reg.present_state.command_inhibit_cmd.d;
    end
    if (hw2reg.present_state.command_inhibit_dat.de) begin
      reg2hw_d.present_state.command_inhibit_dat.q = hw2reg.present_state.command_inhibit_dat.d;
    end
    if (hw2reg.present_state.dat_line_active.de) begin
      reg2hw_d.present_state.dat_line_active.q = hw2reg.present_state.dat_line_active.d;
    end
    if (hw2reg.present_state.write_transfer_active.de) begin
      reg2hw_d.present_state.write_transfer_active.q = hw2reg.present_state.write_transfer_active.d;
    end
    if (hw2reg.present_state.read_transfer_active.de) begin
      reg2hw_d.present_state.read_transfer_active.q = hw2reg.present_state.read_transfer_active.d;
    end
    if (hw2reg.present_state.buffer_write_enable.de) begin
      reg2hw_d.present_state.buffer_write_enable.q = hw2reg.present_state.buffer_write_enable.d;
    end
    if (hw2reg.present_state.buffer_read_enable.de) begin
      reg2hw_d.present_state.buffer_read_enable.q = hw2reg.present_state.buffer_read_enable.d;
    end
    if (hw2reg.present_state.card_inserted.de) begin
      reg2hw_d.present_state.card_inserted.q = hw2reg.present_state.card_inserted.d;
    end
    if (hw2reg.present_state.card_state_stable.de) begin
      reg2hw_d.present_state.card_state_stable.q = hw2reg.present_state.card_state_stable.d;
    end
    if (hw2reg.present_state.card_detect_pin_level.de) begin
      reg2hw_d.present_state.card_detect_pin_level.q = hw2reg.present_state.card_detect_pin_level.d;
    end
    if (hw2reg.present_state.write_protect_switch_pin_level.de) begin
      reg2hw_d.present_state.write_protect_switch_pin_level.q =
          hw2reg.present_state.write_protect_switch_pin_level.d;
    end
    if (hw2reg.present_state.dat_line_signal_level.de) begin
      reg2hw_d.present_state.dat_line_signal_level.q = hw2reg.present_state.dat_line_signal_level.d;
    end
    if (hw2reg.present_state.cmd_line_signal_level.de) begin
      reg2hw_d.present_state.cmd_line_signal_level.q = hw2reg.present_state.cmd_line_signal_level.d;
    end

    if (hw2reg.clock_control.internal_clock_stable.de) begin
      reg2hw_d.clock_control.internal_clock_stable.q = hw2reg.clock_control.internal_clock_stable.d;
    end

    reg2hw_d.software_reset.software_reset_for_all.q = w1s_next(
        reg2hw_q.software_reset.software_reset_for_all.q,
        hw2reg.software_reset.software_reset_for_all.d,
        hw2reg.software_reset.software_reset_for_all.de,
        accepted && reg_we && reg_addr == SDHCI_SOFTWARE_RESET_OFFSET && reg_be[3],
        reg_wdata[24]);
    reg2hw_d.software_reset.software_reset_for_cmd_line.q = w1s_next(
        reg2hw_q.software_reset.software_reset_for_cmd_line.q,
        hw2reg.software_reset.software_reset_for_cmd_line.d,
        hw2reg.software_reset.software_reset_for_cmd_line.de,
        accepted && reg_we && reg_addr == SDHCI_SOFTWARE_RESET_OFFSET && reg_be[3],
        reg_wdata[25]);
    reg2hw_d.software_reset.software_reset_for_dat_line.q = w1s_next(
        reg2hw_q.software_reset.software_reset_for_dat_line.q,
        hw2reg.software_reset.software_reset_for_dat_line.d,
        hw2reg.software_reset.software_reset_for_dat_line.de,
        accepted && reg_we && reg_addr == SDHCI_SOFTWARE_RESET_OFFSET && reg_be[3],
        reg_wdata[26]);

    reg2hw_d.normal_interrupt_status.command_complete.q = w1c_next(
        reg2hw_q.normal_interrupt_status.command_complete.q,
        hw2reg.normal_interrupt_status.command_complete.d,
        hw2reg.normal_interrupt_status.command_complete.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[0]);
    reg2hw_d.normal_interrupt_status.transfer_complete.q = w1c_next(
        reg2hw_q.normal_interrupt_status.transfer_complete.q,
        hw2reg.normal_interrupt_status.transfer_complete.d,
        hw2reg.normal_interrupt_status.transfer_complete.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[1]);
    reg2hw_d.normal_interrupt_status.buffer_write_ready.q = w1c_next(
        reg2hw_q.normal_interrupt_status.buffer_write_ready.q,
        hw2reg.normal_interrupt_status.buffer_write_ready.d,
        hw2reg.normal_interrupt_status.buffer_write_ready.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[4]);
    reg2hw_d.normal_interrupt_status.buffer_read_ready.q = w1c_next(
        reg2hw_q.normal_interrupt_status.buffer_read_ready.q,
        hw2reg.normal_interrupt_status.buffer_read_ready.d,
        hw2reg.normal_interrupt_status.buffer_read_ready.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[5]);
    reg2hw_d.normal_interrupt_status.card_insertion.q = w1c_next(
        reg2hw_q.normal_interrupt_status.card_insertion.q,
        hw2reg.normal_interrupt_status.card_insertion.d,
        hw2reg.normal_interrupt_status.card_insertion.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[6]);
    reg2hw_d.normal_interrupt_status.card_removal.q = w1c_next(
        reg2hw_q.normal_interrupt_status.card_removal.q,
        hw2reg.normal_interrupt_status.card_removal.d,
        hw2reg.normal_interrupt_status.card_removal.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[0],
        reg_wdata[7]);
    reg2hw_d.normal_interrupt_status.error_interrupt.q = w1c_next(
        reg2hw_q.normal_interrupt_status.error_interrupt.q,
        hw2reg.normal_interrupt_status.error_interrupt.d,
        hw2reg.normal_interrupt_status.error_interrupt.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[1],
        reg_wdata[15]);

    reg2hw_d.error_interrupt_status.command_timeout_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.command_timeout_error.q,
        hw2reg.error_interrupt_status.command_timeout_error.d,
        hw2reg.error_interrupt_status.command_timeout_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[16]);
    reg2hw_d.error_interrupt_status.command_crc_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.command_crc_error.q,
        hw2reg.error_interrupt_status.command_crc_error.d,
        hw2reg.error_interrupt_status.command_crc_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[17]);
    reg2hw_d.error_interrupt_status.command_end_bit_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.command_end_bit_error.q,
        hw2reg.error_interrupt_status.command_end_bit_error.d,
        hw2reg.error_interrupt_status.command_end_bit_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[18]);
    reg2hw_d.error_interrupt_status.command_index_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.command_index_error.q,
        hw2reg.error_interrupt_status.command_index_error.d,
        hw2reg.error_interrupt_status.command_index_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[19]);
    reg2hw_d.error_interrupt_status.data_timeout_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.data_timeout_error.q,
        hw2reg.error_interrupt_status.data_timeout_error.d,
        hw2reg.error_interrupt_status.data_timeout_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[20]);
    reg2hw_d.error_interrupt_status.data_crc_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.data_crc_error.q,
        hw2reg.error_interrupt_status.data_crc_error.d,
        hw2reg.error_interrupt_status.data_crc_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[21]);
    reg2hw_d.error_interrupt_status.data_end_bit_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.data_end_bit_error.q,
        hw2reg.error_interrupt_status.data_end_bit_error.d,
        hw2reg.error_interrupt_status.data_end_bit_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[2],
        reg_wdata[22]);
    reg2hw_d.error_interrupt_status.auto_cmd12_error.q = w1c_next(
        reg2hw_q.error_interrupt_status.auto_cmd12_error.q,
        hw2reg.error_interrupt_status.auto_cmd12_error.d,
        hw2reg.error_interrupt_status.auto_cmd12_error.de,
        accepted && reg_we && reg_addr == SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET && reg_be[3],
        reg_wdata[24]);

    if (hw2reg.auto_cmd12_error_status.auto_cmd12_not_executed.de) begin
      reg2hw_d.auto_cmd12_error_status.auto_cmd12_not_executed.q =
          hw2reg.auto_cmd12_error_status.auto_cmd12_not_executed.d;
    end
    if (hw2reg.auto_cmd12_error_status.auto_cmd12_timeout_error.de) begin
      reg2hw_d.auto_cmd12_error_status.auto_cmd12_timeout_error.q =
          hw2reg.auto_cmd12_error_status.auto_cmd12_timeout_error.d;
    end
    if (hw2reg.auto_cmd12_error_status.auto_cmd12_crc_error.de) begin
      reg2hw_d.auto_cmd12_error_status.auto_cmd12_crc_error.q =
          hw2reg.auto_cmd12_error_status.auto_cmd12_crc_error.d;
    end
    if (hw2reg.auto_cmd12_error_status.auto_cmd12_end_bit_error.de) begin
      reg2hw_d.auto_cmd12_error_status.auto_cmd12_end_bit_error.q =
          hw2reg.auto_cmd12_error_status.auto_cmd12_end_bit_error.d;
    end
    if (hw2reg.auto_cmd12_error_status.auto_cmd12_index_error.de) begin
      reg2hw_d.auto_cmd12_error_status.auto_cmd12_index_error.q =
          hw2reg.auto_cmd12_error_status.auto_cmd12_index_error.d;
    end
    if (hw2reg.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.de) begin
      reg2hw_d.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.q =
          hw2reg.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.d;
    end

    if (accepted && reg_we) begin
      unique case (reg_addr)
        SDHCI_BLOCK_SIZE_OFFSET: begin
          reg2hw_d.block_size.transfer_block_size.q = reg_wdata[11:0];
          reg2hw_d.block_size.transfer_block_size.qe = |(reg_be & 4'b0011);
          reg2hw_d.block_size.host_dma_buffer_boundary.q = reg_wdata[14:12];
          reg2hw_d.block_size.host_dma_buffer_boundary.qe = reg_be[1];
          reg2hw_d.block_count.q = reg_wdata[31:16];
          reg2hw_d.block_count.qe = |(reg_be & 4'b1100);
        end

        SDHCI_ARGUMENT_OFFSET: begin
          if (reg_be[0]) reg2hw_d.argument.q[7:0]   = reg_wdata[7:0];
          if (reg_be[1]) reg2hw_d.argument.q[15:8]  = reg_wdata[15:8];
          if (reg_be[2]) reg2hw_d.argument.q[23:16] = reg_wdata[23:16];
          if (reg_be[3]) reg2hw_d.argument.q[31:24] = reg_wdata[31:24];
        end

        SDHCI_TRANSFER_MODE_OFFSET: begin
          if (reg_be[0]) begin
            reg2hw_d.transfer_mode.block_count_enable.q = reg_wdata[1];
            reg2hw_d.transfer_mode.auto_cmd12_enable.q = reg_wdata[2];
            reg2hw_d.transfer_mode.data_transfer_direction_select.q = reg_wdata[4];
            reg2hw_d.transfer_mode.multi_single_block_select.q = reg_wdata[5];
            reg2hw_d.transfer_mode.block_count_enable.qe = 1'b1;
            reg2hw_d.transfer_mode.auto_cmd12_enable.qe = 1'b1;
            reg2hw_d.transfer_mode.data_transfer_direction_select.qe = 1'b1;
            reg2hw_d.transfer_mode.multi_single_block_select.qe = 1'b1;
          end
          if (reg_be[2]) begin
            reg2hw_d.command.response_type_select.q = reg_wdata[17:16];
            reg2hw_d.command.command_crc_check_enable.q = reg_wdata[19];
            reg2hw_d.command.command_index_check_enable.q = reg_wdata[20];
            reg2hw_d.command.data_present_select.q = reg_wdata[21];
            reg2hw_d.command.command_type.q = reg_wdata[23:22];
            reg2hw_d.command.response_type_select.qe = 1'b1;
            reg2hw_d.command.command_crc_check_enable.qe = 1'b1;
            reg2hw_d.command.command_index_check_enable.qe = 1'b1;
            reg2hw_d.command.data_present_select.qe = 1'b1;
            reg2hw_d.command.command_type.qe = 1'b1;
          end
          if (reg_be[3]) begin
            reg2hw_d.command.command_index.q = reg_wdata[29:24];
            reg2hw_d.command.command_index.qe = 1'b1;
          end
        end

        SDHCI_BUFFER_DATA_PORT_OFFSET: begin
          reg2hw_d.buffer_data_port.q = reg_wdata;
          reg2hw_d.buffer_data_port.qe = 1'b1;
        end

        SDHCI_HOST_CONTROL_OFFSET: begin
          if (reg_be[0]) begin
            reg2hw_d.host_control.data_transfer_width.q = reg_wdata[1];
          end
        end

        SDHCI_CLOCK_CONTROL_OFFSET: begin
          if (reg_be[0]) begin
            reg2hw_d.clock_control.internal_clock_enable.q = reg_wdata[0];
            reg2hw_d.clock_control.sd_clock_enable.q = reg_wdata[2];
            reg2hw_d.clock_control.internal_clock_enable.qe = 1'b1;
            reg2hw_d.clock_control.sd_clock_enable.qe = 1'b1;
          end
          if (reg_be[1]) begin
            reg2hw_d.clock_control.sdclk_frequency_select.q = reg_wdata[15:8];
            reg2hw_d.clock_control.sdclk_frequency_select.qe = 1'b1;
          end
          if (reg_be[2]) begin
            reg2hw_d.timeout_control.data_timeout_counter_value.q = reg_wdata[19:16];
          end
        end

        SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE_OFFSET: begin
          if (reg_be[0]) begin
            reg2hw_d.normal_interrupt_status_enable.command_complete_status_enable.q = reg_wdata[0];
            reg2hw_d.normal_interrupt_status_enable.transfer_complete_status_enable.q = reg_wdata[1];
            reg2hw_d.normal_interrupt_status_enable.buffer_write_ready_status_enable.q = reg_wdata[4];
            reg2hw_d.normal_interrupt_status_enable.buffer_read_ready_status_enable.q = reg_wdata[5];
            reg2hw_d.normal_interrupt_status_enable.card_insertion_status_enable.q = reg_wdata[6];
            reg2hw_d.normal_interrupt_status_enable.card_removal_status_enable.q = reg_wdata[7];
          end
          if (reg_be[1]) begin
            reg2hw_d.normal_interrupt_status_enable.card_interrupt_status_enable.q = reg_wdata[8];
          end
          if (reg_be[2]) begin
            reg2hw_d.error_interrupt_status_enable.command_timeout_error_status_enable.q =
                reg_wdata[16];
            reg2hw_d.error_interrupt_status_enable.command_crc_error_status_enable.q =
                reg_wdata[17];
            reg2hw_d.error_interrupt_status_enable.command_end_bit_error_status_enable.q =
                reg_wdata[18];
            reg2hw_d.error_interrupt_status_enable.command_index_error_status_enable.q =
                reg_wdata[19];
            reg2hw_d.error_interrupt_status_enable.data_timeout_error_status_enable.q =
                reg_wdata[20];
            reg2hw_d.error_interrupt_status_enable.data_crc_error_status_enable.q =
                reg_wdata[21];
            reg2hw_d.error_interrupt_status_enable.data_end_bit_error_status_enable.q =
                reg_wdata[22];
          end
          if (reg_be[3]) begin
            reg2hw_d.error_interrupt_status_enable.auto_cmd12_error_status_enable.q =
                reg_wdata[24];
          end
        end

        SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE_OFFSET: begin
          if (reg_be[0]) begin
            reg2hw_d.normal_interrupt_signal_enable.command_complete_signal_enable.q = reg_wdata[0];
            reg2hw_d.normal_interrupt_signal_enable.transfer_complete_signal_enable.q = reg_wdata[1];
            reg2hw_d.normal_interrupt_signal_enable.buffer_write_ready_signal_enable.q = reg_wdata[4];
            reg2hw_d.normal_interrupt_signal_enable.buffer_read_ready_signal_enable.q = reg_wdata[5];
            reg2hw_d.normal_interrupt_signal_enable.card_insertion_signal_enable.q = reg_wdata[6];
            reg2hw_d.normal_interrupt_signal_enable.card_removal_signal_enable.q = reg_wdata[7];
          end
          if (reg_be[1]) begin
            reg2hw_d.normal_interrupt_signal_enable.card_interrupt_signal_enable.q = reg_wdata[8];
          end
          if (reg_be[2]) begin
            reg2hw_d.error_interrupt_signal_enable.command_timeout_error_signal_enable.q =
                reg_wdata[16];
            reg2hw_d.error_interrupt_signal_enable.command_crc_error_signal_enable.q =
                reg_wdata[17];
            reg2hw_d.error_interrupt_signal_enable.command_end_bit_error_signal_enable.q =
                reg_wdata[18];
            reg2hw_d.error_interrupt_signal_enable.command_index_error_signal_enable.q =
                reg_wdata[19];
            reg2hw_d.error_interrupt_signal_enable.data_timeout_error_signal_enable.q =
                reg_wdata[20];
            reg2hw_d.error_interrupt_signal_enable.data_crc_error_signal_enable.q =
                reg_wdata[21];
            reg2hw_d.error_interrupt_signal_enable.data_end_bit_error_signal_enable.q =
                reg_wdata[22];
          end
          if (reg_be[3]) begin
            reg2hw_d.error_interrupt_signal_enable.auto_cmd12_error_signal_enable.q =
                reg_wdata[24];
          end
        end

        default: ;
      endcase
    end

    if (accepted && reg_re && reg_addr == SDHCI_BUFFER_DATA_PORT_OFFSET) begin
      reg2hw_d.buffer_data_port.re = 1'b1;
    end
  end

  always_comb begin
    reg2hw_o = reg2hw_q;

    reg2hw_o.block_size.transfer_block_size.q = reg2hw_d.block_size.transfer_block_size.q;
    reg2hw_o.block_size.transfer_block_size.qe = reg2hw_d.block_size.transfer_block_size.qe;
    reg2hw_o.block_size.host_dma_buffer_boundary.q = reg2hw_d.block_size.host_dma_buffer_boundary.q;
    reg2hw_o.block_size.host_dma_buffer_boundary.qe =
        reg2hw_d.block_size.host_dma_buffer_boundary.qe;
    reg2hw_o.block_count.q = reg2hw_d.block_count.q;
    reg2hw_o.block_count.qe = reg2hw_d.block_count.qe;

    reg2hw_o.transfer_mode.block_count_enable.q =
        reg2hw_d.transfer_mode.block_count_enable.q;
    reg2hw_o.transfer_mode.block_count_enable.qe =
        reg2hw_d.transfer_mode.block_count_enable.qe;
    reg2hw_o.transfer_mode.auto_cmd12_enable.q =
        reg2hw_d.transfer_mode.auto_cmd12_enable.q;
    reg2hw_o.transfer_mode.auto_cmd12_enable.qe =
        reg2hw_d.transfer_mode.auto_cmd12_enable.qe;
    reg2hw_o.transfer_mode.data_transfer_direction_select.q =
        reg2hw_d.transfer_mode.data_transfer_direction_select.q;
    reg2hw_o.transfer_mode.data_transfer_direction_select.qe =
        reg2hw_d.transfer_mode.data_transfer_direction_select.qe;
    reg2hw_o.transfer_mode.multi_single_block_select.q =
        reg2hw_d.transfer_mode.multi_single_block_select.q;
    reg2hw_o.transfer_mode.multi_single_block_select.qe =
        reg2hw_d.transfer_mode.multi_single_block_select.qe;

    reg2hw_o.buffer_data_port.q = reg2hw_d.buffer_data_port.q;
    reg2hw_o.buffer_data_port.qe = reg2hw_d.buffer_data_port.qe;
    reg2hw_o.buffer_data_port.re = reg2hw_d.buffer_data_port.re;
  end

  assign reg2hw = reg2hw_o;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      reg2hw_q <= '0;
      reg2hw_q.clock_control.internal_clock_stable.q <= 1'b1;
    end else if (clear_i) begin
      reg2hw_q <= '0;
      reg2hw_q.clock_control.internal_clock_stable.q <= 1'b1;
    end else begin
      reg2hw_q <= reg2hw_d;
    end
  end

endmodule

module sdhci_reg_top_intf #(
  parameter int AW = 8,
  parameter int unsigned BufferNumWords = 256,
  parameter bit AllowNoncompliantBufferSizes = 1'b0,
  localparam int DW = 32
) (
  input  logic clk_i,
  input  logic rst_ni,
  REG_BUS.in regbus_slave,

  output sdhci_reg_pkg::sdhci_reg2hw_t reg2hw,
  input  sdhci_reg_pkg::sdhci_hw2reg_t hw2reg,
  input  logic clear_i,
  input  logic buffer_data_port_read_ready_i,
  input  logic buffer_data_port_write_ready_i,

  input devmode_i
);

  localparam int unsigned STRB_WIDTH = DW / 8;

`include "register_interface/typedef.svh"
`include "register_interface/assign.svh"

  typedef logic [AW-1:0] addr_t;
  typedef logic [DW-1:0] data_t;
  typedef logic [STRB_WIDTH-1:0] strb_t;
  `REG_BUS_TYPEDEF_ALL(reg_bus, addr_t, data_t, strb_t)

  reg_bus_req_t s_reg_req;
  reg_bus_rsp_t s_reg_rsp;

  `REG_BUS_ASSIGN_TO_REQ(s_reg_req, regbus_slave)
  `REG_BUS_ASSIGN_FROM_RSP(regbus_slave, s_reg_rsp)

  sdhci_reg_top #(
    .reg_req_t(reg_bus_req_t),
    .reg_rsp_t(reg_bus_rsp_t),
    .AW(AW),
    .BufferNumWords(BufferNumWords),
    .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes)
  ) i_regs (
    .clk_i,
    .rst_ni,
    .reg_req_i(s_reg_req),
    .reg_rsp_o(s_reg_rsp),
    .reg2hw,
    .hw2reg,
    .clear_i,
    .buffer_data_port_read_ready_i,
    .buffer_data_port_write_ready_i,
    .devmode_i
  );

endmodule
