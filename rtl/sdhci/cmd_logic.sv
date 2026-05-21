// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "common_cells/registers.svh"

module cmd_logic (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,
  input  logic clk_en_p_i,
  input  logic clk_en_n_i,
  input  logic div_1_i,

  input  logic sd_bus_cmd_i,
  output logic sd_bus_cmd_o,
  output logic sd_bus_cmd_en_o,

  output logic cmd_done_o,
  output logic rsp_done_o,
  output logic cmd_inhibit_cmd_o,

  input  sdhci_pkg::cmd_t cmd_i,
  input  sdhci_pkg::cmd_arg_t cmd_arg_i,
  input  sdhci_pkg::response_type_e response_type_i,
  input  logic cmd_valid_i,
  output logic cmd_ready_o,

  output logic cmd_result_valid_o,
  output logic [31:0] response0_d_o,
  output logic [31:0] response1_d_o,
  output logic [31:0] response2_d_o,
  output logic [31:0] response3_d_o,
  output logic        response0_de_o,
  output logic        response1_de_o,
  output logic        response2_de_o,
  output logic        response3_de_o,
  output logic index_error_o,
  output logic end_bit_error_o,
  output logic crc_error_o,

  output logic timeout_error_o
);

  typedef enum logic [2:0] {
    IDLE,
    START,
    SEND_CMD,
    WAIT_RSP,
    READ_RSP,
    BUS_COOLDOWN,
    RSP_TIMEOUT
  } cmd_fsm_t;

  cmd_fsm_t cmd_state_q, cmd_state_d;
  `FFARNC(cmd_state_q, cmd_state_d, clear_i, IDLE, clk_i, rst_ni);

  // TODO: this forces a sleep cycle in between transactions. This might not
  // be ideal -> reduce sleep cycles in BUS_COOLDOWN by one
  logic cmd_ready;
  assign cmd_ready = cmd_state_q == IDLE;

  logic start_cmd;
  assign start_cmd = cmd_ready && cmd_valid_i;

  // cmd_i, cmd_arg_i and response_type_i are the accepted command payload
  // latched by autocmd_wrap, so no local duplicate latches are needed here.

  // Electrical spec, 7.13.5; units are number of cycles
  localparam logic [6:0] N_CR_MIN = 2;  // minimum time between command (SDHC) and response (card)
  localparam logic [6:0] N_CR_MAX = 64; // maximum time between command and response
  localparam logic [6:0] N_ID     = 5;  // time between identification command and response
  localparam logic [6:0] N_RC     = 8;  // minimum time between response and next command
  localparam logic [6:0] N_CC     = 8;  // minimum time between consecutive commands
  // TODO: check counter to make sure there are no off-by-one errors
  
  logic tx_done;
  logic rsp_receiving;
  logic rsp_received;
  logic clear_cycle_counter;
  logic [6:0] cycles_waiting;

  assign clear_cycle_counter = (cmd_state_d == WAIT_RSP && cmd_state_q != WAIT_RSP) ||
                               (cmd_state_d == BUS_COOLDOWN && cmd_state_q != BUS_COOLDOWN);


  always_comb begin : cmd_fsm
    cmd_state_d = cmd_state_q;

    unique case (cmd_state_q)
      IDLE: begin
        if (start_cmd) begin
          cmd_state_d = START;
        end
      end
      START: begin
        // sync start to clock edge
        if (clk_en_p_i) begin
          cmd_state_d = SEND_CMD;
        end
      end
      SEND_CMD: begin
        if (tx_done) begin
          if (response_type_i == sdhci_pkg::NO_RESPONSE) begin
            cmd_state_d = BUS_COOLDOWN;
          end else begin
            cmd_state_d = WAIT_RSP;
          end
        end
      end
      WAIT_RSP: begin
        if (rsp_receiving) begin
          cmd_state_d = READ_RSP;
        end else begin
          // +2: +1 for being over, +1 as we get tx_done on the end bit cycle,
          // and thus are one cycle ahead
          if ((cmd_i == 'd01 || cmd_i == 'd02) && cycles_waiting == N_ID + 2) begin
            // The identification command (CMD2) has a shorter timeout period,
            // see 7.2.5
            cmd_state_d = RSP_TIMEOUT;
          end else if (cycles_waiting == N_CR_MAX + 2) begin
            cmd_state_d = RSP_TIMEOUT;
          end
        end
      end
      READ_RSP: begin
        if (rsp_received) begin
          cmd_state_d = BUS_COOLDOWN;
        end
      end
      BUS_COOLDOWN: begin
        if (response_type_i == sdhci_pkg::NO_RESPONSE
            && cycles_waiting == N_CC) begin
          cmd_state_d = IDLE;
        end else if (cycles_waiting == N_RC) begin
          cmd_state_d = IDLE;
        end
      end
      RSP_TIMEOUT: begin
        cmd_state_d = IDLE;
      end
    endcase
  end

  assign cmd_done_o  = cmd_state_d != SEND_CMD && cmd_state_q == SEND_CMD;
  assign rsp_done_o  = cmd_state_d == BUS_COOLDOWN && cmd_state_q != BUS_COOLDOWN &&
                       response_type_i != sdhci_pkg::NO_RESPONSE;

  assign cmd_ready_o = cmd_ready;

  assign timeout_error_o    = cmd_state_q == RSP_TIMEOUT;
  assign cmd_result_valid_o = rsp_received;

  logic crc_correct;
  assign crc_error_o = ~crc_correct;

  sdhci_pkg::cmd_t cmd_in_response;
  logic [5:0] response_index;
  assign cmd_in_response = sdhci_pkg::cmd_t'(response_index);
  // this line could optionally be masked by the valid line, leave it for now
  assign index_error_o = (cmd_in_response != cmd_i) & (response_type_i == sdhci_pkg::RESPONSE_LENGTH_48 |
                                                       response_type_i == sdhci_pkg::RESPONSE_LENGTH_48_CHECK_BUSY);

  always_comb begin : cmd_inhibit
    cmd_inhibit_cmd_o = 1'b1;
    unique case (cmd_state_q)
      IDLE: begin
        cmd_inhibit_cmd_o = 1'b0;
      end
      START: begin
        cmd_inhibit_cmd_o = 1'b1;
      end
      SEND_CMD: begin
        cmd_inhibit_cmd_o = 1'b1;
      end
      WAIT_RSP: begin
        cmd_inhibit_cmd_o = 1'b1;
      end
      READ_RSP: begin
        cmd_inhibit_cmd_o = 1'b1;
      end
      BUS_COOLDOWN: begin
        cmd_inhibit_cmd_o = 1'b0;
      end
      RSP_TIMEOUT: begin
        cmd_inhibit_cmd_o = 1'b1;
      end
    endcase
  end

  // TODO: this is clocked with clk_en_p_i
  cmd_write i_cmd_write (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .clear_i        (clear_i),

    .clk_en_p_i     (clk_en_p_i),
    .clk_en_n_i     (clk_en_n_i),
    .div_1_i        (div_1_i),

    .cmd_o          (sd_bus_cmd_o),
    .cmd_en_o       (sd_bus_cmd_en_o),
    .start_tx_i     (cmd_state_q == START),
    .cmd_argument_i (cmd_arg_i),
    .cmd_nr_i       (cmd_i),

    .tx_done_o      (tx_done)
  );

  rsp_read  i_rsp_read (
    .clk_i             (clk_i),
    .clk_en_i          (clk_en_p_i),
    .rst_ni            (rst_ni),
    .clear_i           (clear_i),
    .cmd_i             (sd_bus_cmd_i),
    .long_rsp_i        (response_type_i == sdhci_pkg::RESPONSE_LENGTH_136),
    .start_listening_i (cmd_state_q == WAIT_RSP && (cycles_waiting == N_CR_MIN - 1)),
    .timeout_i         (cmd_state_q == RSP_TIMEOUT),
    .receiving_o       (rsp_receiving),
    .rsp_valid_o       (rsp_received),
    .end_bit_err_o     (end_bit_error_o),
    .response0_d_o     (response0_d_o),
    .response1_d_o     (response1_d_o),
    .response2_d_o     (response2_d_o),
    .response3_d_o     (response3_d_o),
    .response0_de_o    (response0_de_o),
    .response1_de_o    (response1_de_o),
    .response2_de_o    (response2_de_o),
    .response3_de_o    (response3_de_o),
    .response_index_o  (response_index),
    .crc_corr_o        (crc_correct)
  );

  counter #(
    .WIDTH            (3'd7),
    .STICKY_OVERFLOW  (1'b0)
  ) i_counter (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .clear_i    (clear_i),
    .en_i       (clk_en_p_i),
    .load_i     (clear_cycle_counter),
    .down_i     (1'b0),
    .d_i        ({6'b0, div_1_i}),
    .q_o        (cycles_waiting),
    .overflow_o ()
  );

endmodule
