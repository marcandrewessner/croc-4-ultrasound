// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Anton Buchner <abuchner@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "common_cells/registers.svh"
`include "defines.svh"

// SDC note: sd_clk_o is a clk_i-registered clock-as-data pad output. The
// controller internals remain in the clk_i domain and use sd_clk_en_p/n strobes
// derived from the same divider. Do not propagate CTS through sd_clk_o; constrain
// SD CMD/DAT pad delays against clk_i, and document the board/protocol SDCLK
// period as ClkPreDiv times the SDHCI frequency-select divider. Hardware
// enforces a minimum effective divide by 2 for sdclk_frequency_select=0.
module sdhci_top #(
  parameter int unsigned AddrWidth = 32'd32,
  parameter type               reg_req_t   = logic,
  parameter type               reg_rsp_t   = logic,

  // Software handles SDHCI clock division. ClkPreDiv is a hidden integration
  // predivider folded into the same physical divider as sdclk_frequency_select;
  // advertise clk_i/ClkPreDiv as base_clock_frequency_for_sd_clock.
  parameter int unsigned       ClkPreDiv   = 2,

  parameter int unsigned TimeoutDivider = 1, // by how much to divide clk_i to get the timeout count frequency,
                                    // see dat_timeout for details

  // Warranty-void integration escape hatch: default mode is SDHCI-compliant
  // and requires enough FIFO space for a full 512-byte block. Setting
  // AllowNoncompliantBufferSizes permits smaller FIFOs and reports the usable
  // Buffer Data Port chunk size in the vendor extension at offset 0x44.
  parameter int unsigned BufferNumWords = 256,
  parameter bit          AllowNoncompliantBufferSizes = 1'b0,

  // clock runs at 100MHz, so 1ms is 100_000 cycles
  parameter int unsigned       NumDebounceCycles = 1_000_000 // 10ms
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
  logic sd_clear, sd_clear_cmd, sd_clear_dat;
  sdhci_reg_pkg::sdhci_reg2hw_t reg2hw, reg2hw_orig;
  sdhci_reg_pkg::sdhci_hw2reg_t hw2reg;
  logic buffer_data_port_read_ready;
  logic buffer_data_port_write_ready;

  // Soft Reset Logic /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  logic software_reset_all_q, software_reset_all_d, software_reset_cmd_q, software_reset_cmd_d, software_reset_dat_q, software_reset_dat_d;

  assign software_reset_all_d = reg2hw.software_reset.software_reset_for_all.q;
  `FF(software_reset_all_q, software_reset_all_d, '0, clk_i, rst_ni);

  assign software_reset_cmd_d = reg2hw.software_reset.software_reset_for_cmd_line.q;  // command circuit soft reset
  `FF(software_reset_cmd_q, software_reset_cmd_d, '0, clk_i, rst_ni);

  assign software_reset_dat_d = reg2hw.software_reset.software_reset_for_dat_line.q;  // dat circuit soft reset
  `FF(software_reset_dat_q, software_reset_dat_d, '0, clk_i, rst_ni);

  assign sd_clear     = software_reset_all_q;
  assign sd_clear_cmd = sd_clear || software_reset_cmd_q;
  assign sd_clear_dat = sd_clear || software_reset_dat_q;

  assign hw2reg.software_reset.software_reset_for_all.d = 1'b0;
  assign hw2reg.software_reset.software_reset_for_dat_line.d = 1'b0;
  assign hw2reg.software_reset.software_reset_for_cmd_line.d = 1'b0;

  assign hw2reg.software_reset.software_reset_for_all.de = software_reset_all_q;
  assign hw2reg.software_reset.software_reset_for_dat_line.de = software_reset_dat_q;
  assign hw2reg.software_reset.software_reset_for_cmd_line.de = software_reset_cmd_q;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  sdhci_reg_top #(
    .AW        (AddrWidth),
    .reg_req_t (reg_req_t),
    .reg_rsp_t (reg_rsp_t),
    .BufferNumWords(BufferNumWords),
    .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes)
  ) i_regs (
    .clk_i,
    .rst_ni,
    .reg_req_i,
    .reg_rsp_o,
    .reg2hw    (reg2hw_orig),
    .hw2reg,
    .clear_i   (sd_clear),
    // Routed unconditionally, not gated on AllowNoncompliantBufferSizes.
    // These drive reg_ready (sdhci_reg_top.sv), which the OBI bridge turns
    // into the bus grant (sdhci_obi_to_reg.sv: gnt = req & reg_rsp.ready), so
    // a BUFFER_DATA_PORT access that the DAT buffer cannot take right now
    // stalls the requester instead of completing. Previously these were tied
    // to 1'b1 for compliant buffer sizes, on the assumption that a host only
    // ever writes one block after BUFFER_WRITE_READY -- true for software,
    // but not for the ADC copy engine, which streams a whole multi-block
    // CMD25 session (adc_acquisition_sdcard_controller.sv) and does fill the
    // buffer. With the ready tied high, dat_buffer.sv still gated its
    // internal reg_push on the same signal, so those words were silently
    // dropped rather than deferred.
    //
    // Note this is deliberately not done by setting AllowNoncompliantBufferSizes,
    // which reaches the same signal but also disables the buffer-size
    // assertions and flips buffer_write_enable_o from has_block_space to
    // !reg_full, weakening BUFFER_WRITE_READY from "room for a whole block"
    // to "room for one word".
    .buffer_data_port_read_ready_i  (buffer_data_port_read_ready),
    .buffer_data_port_write_ready_i (buffer_data_port_write_ready),
    .devmode_i (1'b1)
  );

  logic  sd_cmd_dat_busy;

  `writable_reg_t([15:0]) block_count_hw;


  sdhci_reg_logic i_sdhci_reg_logic (
    .clk_i,
    .rst_ni,
    .clear_i     (sd_clear),
    .clear_cmd_i (sd_clear_cmd),
    .clear_dat_i (sd_clear_dat),

    .reg2hw_i          (reg2hw_orig),
    .hw2reg_i          (hw2reg),
    .reg2hw_modified_o (reg2hw),

    .sd_cmd_dat_busy_i (sd_cmd_dat_busy),

    .error_interrupt_o  (hw2reg.normal_interrupt_status.error_interrupt),
    .auto_cmd12_error_o (hw2reg.error_interrupt_status.auto_cmd12_error),

    .buffer_read_ready_o  (hw2reg.normal_interrupt_status.buffer_read_ready),
    .buffer_write_ready_o (hw2reg.normal_interrupt_status.buffer_write_ready),

    .dat_line_active_o     (hw2reg.present_state.dat_line_active),
    .command_inhibit_dat_o (hw2reg.present_state.command_inhibit_dat),

    .transfer_complete_o (hw2reg.normal_interrupt_status.transfer_complete),
    .command_complete_o  (hw2reg.normal_interrupt_status.command_complete),

    .card_removal_o    (hw2reg.normal_interrupt_status.card_removal),
    .card_insertion_o  (hw2reg.normal_interrupt_status.card_insertion),


    .block_count_o       (hw2reg.block_count),
    .block_count_hw_i    (block_count_hw),
    .block_size_reg_o    (hw2reg.block_size),
    .transfer_mode_reg_o (hw2reg.transfer_mode),

    .interrupt_signal_for_each_slot_o (hw2reg.slot_interrupt_status.interrupt_signal_for_each_slot.d),
    .interrupt_o
  );

  logic pause_sd_clk, sd_clk_en_p, sd_clk_en_n, div_1;
  sd_clk_generator #(
    .ClkPreDiv (ClkPreDiv)
  ) i_sd_clk_generator (
    .clk_i,
    .rst_ni,
    .clear_i (sd_clear),
    .reg2hw_i (reg2hw),

    .pause_sd_clk_i  (pause_sd_clk),
    .sd_clk_o        (sd_clk_o),
    .clk_en_p_o      (sd_clk_en_p),
    .clk_en_n_o      (sd_clk_en_n),
    .div_1_o         (div_1),
    .sd_clk_stable_o (hw2reg.clock_control.internal_clock_stable)
  );

  logic sd_card_detected;
  assign sd_card_detected = ~sd_cd_ni;
  logic sd_card_detected_stable;
  logic sd_card_detected_debounced;

  sdhci_debounce #(
    .NumCycles(NumDebounceCycles)
  ) i_debouncer (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .data_i   (sd_card_detected),
    .stable_o (sd_card_detected_stable),
    .data_o   (sd_card_detected_debounced)
  );

  assign hw2reg.present_state.dat_line_signal_level = '{ de: '1, d: sd_dat_i };
  assign hw2reg.present_state.cmd_line_signal_level = '{ de: '1, d: sd_cmd_i };

  assign hw2reg.present_state.write_protect_switch_pin_level = '{ de: '1, d: '1 };
  assign hw2reg.present_state.card_inserted                  = '{ de: '1, d: sd_card_detected_debounced };
  assign hw2reg.present_state.card_state_stable              = '{ de: '1, d: sd_card_detected_stable };
  assign hw2reg.present_state.card_detect_pin_level          = '{ de: '1, d: sd_card_detected };


  logic sd_cmd_done, sd_rsp_done, request_cmd12;

  logic cmd_started, cmd_needs_busy, cmd_data_present, cmd_transfer_direction;

  autocmd_wrap  i_autocmd_wrap (
    .clk_i           (clk_i),
    .rst_ni,
    .clear_i         (sd_clear_cmd),
    .clk_en_p_i      (sd_clk_en_p),
    .clk_en_n_i      (sd_clk_en_n),
    .div_1_i         (div_1),
    .sd_bus_cmd_i    (sd_cmd_i),
    .sd_bus_cmd_o    (sd_cmd_o),
    .sd_bus_cmd_en_o (sd_cmd_en_o),
    .reg2hw          (reg2hw),

    .request_cmd12_i (request_cmd12),

    .sd_cmd_done_o     (sd_cmd_done),
    .sd_rsp_done_o     (sd_rsp_done),

    .cmd_started_o            (cmd_started),
    .cmd_needs_busy_o         (cmd_needs_busy),
    .cmd_data_present_o       (cmd_data_present),
    .cmd_transfer_direction_o (cmd_transfer_direction),

    .response0_d_o  (hw2reg.response0.d),
    .response1_d_o  (hw2reg.response1.d),
    .response2_d_o  (hw2reg.response2.d),
    .response3_d_o  (hw2reg.response3.d),
    .response0_de_o (hw2reg.response0.de),
    .response1_de_o (hw2reg.response1.de),
    .response2_de_o (hw2reg.response2.de),
    .response3_de_o (hw2reg.response3.de),
    .command_inhibit_cmd_o    (hw2reg.present_state.command_inhibit_cmd),
    .command_end_bit_error_o  (hw2reg.error_interrupt_status.command_end_bit_error),
    .command_crc_error_o      (hw2reg.error_interrupt_status.command_crc_error),
    .command_index_error_o    (hw2reg.error_interrupt_status.command_index_error),
    .command_timeout_error_o  (hw2reg.error_interrupt_status.command_timeout_error),
    .auto_cmd12_errors_o      (hw2reg.auto_cmd12_error_status)
  );


  dat_wrap #(
    .TimeoutDivider (TimeoutDivider),
    .BufferNumWords (BufferNumWords),
    .AllowNoncompliantBufferSizes (AllowNoncompliantBufferSizes)
  ) i_dat_wrap (
    .clk_i,
    .sd_clk_en_p_i  (sd_clk_en_p),
    .sd_clk_en_n_i  (sd_clk_en_n),
    .div_1_i        (div_1),
    .rst_ni,
    .clear_i     (sd_clear_dat),

    .dat_i    (sd_dat_i),
    .dat_en_o (sd_dat_en_o),
    .dat_o    (sd_dat_o),

    .cmd_started_i            (cmd_started),
    .cmd_needs_busy_i         (cmd_needs_busy),
    .cmd_data_present_i       (cmd_data_present),
    .cmd_transfer_direction_i (cmd_transfer_direction),

    .sd_cmd_done_i   (sd_cmd_done),
    .sd_rsp_done_i   (sd_rsp_done),

    .sd_busy_o       (sd_cmd_dat_busy),
    .request_cmd12_o (request_cmd12),
    .pause_sd_clk_o  (pause_sd_clk),

    .reg2hw_i (reg2hw),

    .data_crc_error_o        (hw2reg.error_interrupt_status.data_crc_error),
    .data_end_bit_error_o    (hw2reg.error_interrupt_status.data_end_bit_error),
    .data_timeout_error_o    (hw2reg.error_interrupt_status.data_timeout_error),

    .buffer_data_port_d_o    (hw2reg.buffer_data_port.d),
    .buffer_read_enable_o    (hw2reg.present_state.buffer_read_enable),
    .buffer_write_enable_o   (hw2reg.present_state.buffer_write_enable),
    .buffer_data_port_read_ready_o  (buffer_data_port_read_ready),
    .buffer_data_port_write_ready_o (buffer_data_port_write_ready),

    .read_transfer_active_o  (hw2reg.present_state.read_transfer_active),
    .write_transfer_active_o (hw2reg.present_state.write_transfer_active),

    .block_count_o           (block_count_hw)
  );

endmodule
