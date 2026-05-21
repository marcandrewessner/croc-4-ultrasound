// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Anton Buchner <abuchner@student.ethz.ch>
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

import sdhci_reg_pkg::*;
`include "common_cells/registers.svh"
`include "defines.svh"

module autocmd_wrap (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,
  input  logic clk_en_p_i, // high before next sd_clk posedge
  input  logic clk_en_n_i, // high before next sd_clk negedge
  input  logic div_1_i,

  input  logic sd_bus_cmd_i,
  output logic sd_bus_cmd_o,
  output logic sd_bus_cmd_en_o,

  input  sdhci_reg2hw_t reg2hw,
  input  logic request_cmd12_i,

  output logic sd_cmd_done_o,
  output logic sd_rsp_done_o,

  output logic cmd_started_o,
  output logic cmd_needs_busy_o,
  output logic cmd_data_present_o,
  output logic cmd_transfer_direction_o,

  output logic [31:0] response0_d_o,
  output logic [31:0] response1_d_o,
  output logic [31:0] response2_d_o,
  output logic [31:0] response3_d_o,
  output logic response0_de_o,
  output logic response1_de_o,
  output logic response2_de_o,
  output logic response3_de_o,

  output `writable_reg_t() command_inhibit_cmd_o,
  output `writable_reg_t() command_end_bit_error_o,
  output `writable_reg_t() command_crc_error_o,
  output `writable_reg_t() command_index_error_o,
  output `writable_reg_t() command_timeout_error_o,

  output sdhci_reg_pkg::sdhci_hw2reg_auto_cmd12_error_status_reg_t auto_cmd12_errors_o
);
  ////////////////
  // Main Logic //
  ////////////////

  logic driver_cmd_queued_q, driver_cmd_queued_d;
  `FFARNC(driver_cmd_queued_q, driver_cmd_queued_d, clear_i, '0, clk_i, rst_ni);

  logic autocmd12_queued_q, autocmd12_queued_d;
  `FFARNC(autocmd12_queued_q, autocmd12_queued_d, clear_i, '0, clk_i, rst_ni);

  logic running_autocmd12_q, running_autocmd12_d;
  `FFARNC(running_autocmd12_q, running_autocmd12_d, clear_i, '0, clk_i, rst_ni);

  logic command_queued;
  assign command_queued = driver_cmd_queued_q || autocmd12_queued_q;

  always_comb begin
    cmd_data_present_o = reg2hw.command.data_present_select.q;

    if (autocmd12_queued_q) begin
      cmd_data_present_o = 1'b0;
    end
  end

  logic cmd_result_valid;
  logic command_ready;
  logic command_started;
  assign command_started = command_queued & command_ready;
  assign cmd_started_o   = command_started;

  logic end_bit_error;
  logic crc_error;
  logic index_error;
  logic timeout_error;

  logic cmd_errors_occured;
  assign cmd_errors_occured = end_bit_error || crc_error || index_error || timeout_error;

  logic active_transfer_direction_q, active_transfer_direction_d;
  `FFLARNC(active_transfer_direction_q, active_transfer_direction_d,
           command_started && !autocmd12_queued_q && cmd_data_present_o,
           clear_i, '0, clk_i, rst_ni);

  // Use running_autocmd12_q (not autocmd12_queued_q) so the mux output
  // stays stable throughout command execution, allowing cmd_logic to read
  // these directly without re-latching them.
  logic is_autocmd12;
  assign is_autocmd12 = autocmd12_queued_q | running_autocmd12_q;

  sdhci_pkg::cmd_t current_cmd;
  assign current_cmd = is_autocmd12 ? 6'd12 :
                       reg2hw.command.command_index.q;

  sdhci_pkg::cmd_arg_t current_arg;
  assign current_arg = is_autocmd12 ? '0 : reg2hw.argument.q;

  sdhci_pkg::response_type_e current_rsp_type;

  always_comb begin : rsp_type
    current_rsp_type = sdhci_pkg::response_type_e'(reg2hw.command.response_type_select.q);

    // according to electrical spec 7.8.4, CMD12 is R1 on reads and R1b on writes
    if (is_autocmd12) begin
      if (active_transfer_direction_q == 1'b0) begin
        // write -> R1b
        current_rsp_type = sdhci_pkg::RESPONSE_LENGTH_48_CHECK_BUSY;
      end else begin
        // read -> R1
        current_rsp_type = sdhci_pkg::RESPONSE_LENGTH_48;
      end
    end
  end

  assign cmd_needs_busy_o = current_rsp_type == sdhci_pkg::RESPONSE_LENGTH_48_CHECK_BUSY;
  assign cmd_transfer_direction_o = reg2hw.transfer_mode.data_transfer_direction_select.q;

  sdhci_pkg::cmd_t accepted_cmd_q, accepted_cmd_d;
  `FFLARNC(accepted_cmd_q, accepted_cmd_d, command_started, clear_i, '0, clk_i, rst_ni);

  sdhci_pkg::cmd_arg_t accepted_arg_q, accepted_arg_d;
  `FFLARNC(accepted_arg_q, accepted_arg_d, command_started, clear_i, '0, clk_i, rst_ni);

  sdhci_pkg::response_type_e accepted_rsp_type_q, accepted_rsp_type_d;
  `FFLARNC(accepted_rsp_type_q, accepted_rsp_type_d, command_started, clear_i, sdhci_pkg::NO_RESPONSE, clk_i, rst_ni);

  always_comb begin : request_commands
    driver_cmd_queued_d = driver_cmd_queued_q;
    autocmd12_queued_d = autocmd12_queued_q;
    auto_cmd12_errors_o.command_not_issued_by_auto_cmd12_error.de = 1'b0;
    auto_cmd12_errors_o.auto_cmd12_not_executed.de = 1'b0;
    running_autocmd12_d = running_autocmd12_q;
    active_transfer_direction_d = reg2hw.transfer_mode.data_transfer_direction_select.q;
    accepted_cmd_d = current_cmd;
    accepted_arg_d = current_arg;
    accepted_rsp_type_d = current_rsp_type;

    if (reg2hw.command.command_index.qe) begin
      driver_cmd_queued_d = 1'b1;
    end

    if (request_cmd12_i) begin
      autocmd12_queued_d = 1'b1;
    end

    if (command_started) begin
      // A command has just been submitted
      running_autocmd12_d = autocmd12_queued_q;
      if (autocmd12_queued_q) begin
        // autocmd12 has priority
        autocmd12_queued_d = 1'b0;
      end else begin
        driver_cmd_queued_d = 1'b0;
      end
    end

    if ((cmd_result_valid && cmd_errors_occured) || timeout_error) begin
      // This should never race with command submission,
      // as there is a period of time where the cmd line needs
      // to stay idle (per spec). Errors are reported during
      // that time
      driver_cmd_queued_d = 1'b0;
      autocmd12_queued_d  = 1'b0;

      if (running_autocmd12_q && driver_cmd_queued_q) begin
        // We aborted driver command
        auto_cmd12_errors_o.command_not_issued_by_auto_cmd12_error.de = 1'b1;
      end

      if (~running_autocmd12_q && autocmd12_queued_q) begin
        // We aborted autocmd12
        auto_cmd12_errors_o.auto_cmd12_not_executed.de = 1'b1;
      end
    end
  end

  ///////////////////
  // Normal Status //
  ///////////////////

  // inhibit status from cmd logic
  logic cmd_inhibit_logic;

  assign command_inhibit_cmd_o.de = '1;
  // autocmd12 execution should not inhibit the driver
  assign command_inhibit_cmd_o.d  = driver_cmd_queued_q | (cmd_inhibit_logic && ~running_autocmd12_q);

  logic [31:0] cmd_response0_d, cmd_response1_d, cmd_response2_d, cmd_response3_d;
  logic cmd_response0_de, cmd_response1_de, cmd_response2_de, cmd_response3_de;

  always_comb begin : rsp_assignment
    response0_d_o  = cmd_response0_d;
    response1_d_o  = cmd_response1_d;
    response2_d_o  = cmd_response2_d;
    response3_d_o  = cmd_response3_d;
    response0_de_o = cmd_response0_de && !running_autocmd12_q;
    response1_de_o = cmd_response1_de && !running_autocmd12_q;
    response2_de_o = cmd_response2_de && !running_autocmd12_q;
    response3_de_o = cmd_response3_de && !running_autocmd12_q;

    if (cmd_response3_de && accepted_rsp_type_q == sdhci_pkg::RESPONSE_LENGTH_136) begin
      response3_d_o = {reg2hw.response3.q[31:24], cmd_response3_d[23:0]};
    end

    if (running_autocmd12_q && cmd_response0_de) begin
      // Auto CMD12 response is stored in RESPONSE3.
      response3_d_o  = cmd_response0_d;
      response3_de_o = 1'b1;
    end
  end : rsp_assignment

  ////////////////////
  // Error Checking //
  ////////////////////

  logic check_end_bit_err, check_crc_err, check_index_err, check_timeout_error;

  assign check_end_bit_err   = reg2hw.error_interrupt_status_enable.command_end_bit_error_status_enable.q;
  assign check_crc_err       = reg2hw.error_interrupt_status_enable.command_crc_error_status_enable.q & reg2hw.command.command_crc_check_enable.q;
  assign check_index_err     = reg2hw.error_interrupt_status_enable.command_index_error_status_enable.q & reg2hw.command.command_index_check_enable.q;
  assign check_timeout_error = reg2hw.error_interrupt_status_enable.command_timeout_error_status_enable.q;

  // Only set error interrupt status, software should clear it
  assign command_index_error_o.d   = 1'b1;
  assign command_end_bit_error_o.d = 1'b1;
  assign command_crc_error_o.d     = 1'b1;
  assign command_timeout_error_o.d = 1'b1;

  assign auto_cmd12_errors_o.auto_cmd12_index_error.d   = 1'b1;
  assign auto_cmd12_errors_o.auto_cmd12_end_bit_error.d = 1'b1;
  assign auto_cmd12_errors_o.auto_cmd12_crc_error.d     = 1'b1;
  assign auto_cmd12_errors_o.auto_cmd12_timeout_error.d = 1'b1;
  assign auto_cmd12_errors_o.auto_cmd12_not_executed.d = 1'b1;
  assign auto_cmd12_errors_o.command_not_issued_by_auto_cmd12_error.d = 1'b1;

  // Timeout is not handshaked, so directly pass it through
  assign command_timeout_error_o.de                      = running_autocmd12_q ? 1'b0 : check_timeout_error & timeout_error;
  assign auto_cmd12_errors_o.auto_cmd12_timeout_error.de = running_autocmd12_q ? timeout_error : 1'b0;

  always_comb begin : cmd_seq_ctrl
    command_end_bit_error_o.de = 1'b0;
    command_crc_error_o.de     = 1'b0;
    command_index_error_o.de   = 1'b0;

    auto_cmd12_errors_o.auto_cmd12_index_error.de   = 1'b0;
    auto_cmd12_errors_o.auto_cmd12_end_bit_error.de = 1'b0;
    auto_cmd12_errors_o.auto_cmd12_crc_error.de     = 1'b0;

    if (cmd_result_valid) begin
      if (running_autocmd12_q) begin
        auto_cmd12_errors_o.auto_cmd12_end_bit_error.de = end_bit_error;
        auto_cmd12_errors_o.auto_cmd12_crc_error.de     = crc_error;
        auto_cmd12_errors_o.auto_cmd12_index_error.de   = index_error;
      end else begin
        command_end_bit_error_o.de = check_end_bit_err   & end_bit_error;
        command_crc_error_o.de     = check_crc_err       & crc_error;
        command_index_error_o.de   = check_index_err     & index_error;
      end
    end
  end

  ///////////////////////////
  // Module Instantiations //
  ///////////////////////////

  cmd_logic i_cmd_logic (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .clear_i           (clear_i),
    .clk_en_p_i        (clk_en_p_i),
    .clk_en_n_i        (clk_en_n_i),
    .div_1_i           (div_1_i),
    .sd_bus_cmd_i      (sd_bus_cmd_i),
    .sd_bus_cmd_o      (sd_bus_cmd_o),
    .sd_bus_cmd_en_o   (sd_bus_cmd_en_o),

    .cmd_done_o        (sd_cmd_done_o),
    .rsp_done_o        (sd_rsp_done_o),
    .cmd_inhibit_cmd_o (cmd_inhibit_logic),

    .cmd_i             (accepted_cmd_q),
    .cmd_arg_i         (accepted_arg_q),
    .response_type_i   (accepted_rsp_type_q),
    .cmd_valid_i       (command_queued),
    .cmd_ready_o       (command_ready),

    .cmd_result_valid_o(cmd_result_valid),
    .response0_d_o     (cmd_response0_d),
    .response1_d_o     (cmd_response1_d),
    .response2_d_o     (cmd_response2_d),
    .response3_d_o     (cmd_response3_d),
    .response0_de_o    (cmd_response0_de),
    .response1_de_o    (cmd_response1_de),
    .response2_de_o    (cmd_response2_de),
    .response3_de_o    (cmd_response3_de),
    .end_bit_error_o   (end_bit_error),
    .crc_error_o       (crc_error),
    .index_error_o     (index_error),

    .timeout_error_o   (timeout_error)
  );

endmodule
