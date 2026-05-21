// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Anton Buchner <abuchner@student.ethz.ch>

// Handles SD CMD line transmission

`include "common_cells/registers.svh"

module cmd_write (
  input   logic         clk_i,
  input   logic         rst_ni,
  input   logic         clear_i,

  input   logic         clk_en_p_i,
  input   logic         clk_en_n_i,
  input   logic         div_1_i,        // is SD CLK frequency same as clk_i frequency?

  output  logic         cmd_o,          // to sd cmd line
  output  logic         cmd_en_o,       // for tri-state driver

  input   logic         start_tx_i,     // start transmission, only works when tx_done_o is high
  input   logic [31:0]  cmd_argument_i, // from cmd argument register
  input   logic [5:0]   cmd_nr_i,       // cmd index (eg. CMD12 == 6'b001100)

  output  logic         tx_done_o       // high when nothing is currently being transmitted (module is in READY state)
);
  /////////////////////////////
  // State Transisiton Logic //
  /////////////////////////////

  // for sequencing
  logic [5:0] bit_count;

  typedef enum logic [2:0] {
    READY,
    START_CNT,      // start sequencing counter
    SHIFT_REG_OUT,  // shift out cmd
    CRC7_OUT,       // shift out CRC7
    END_BIT_OUT     // shift out end bit
  } tx_state_e;
  tx_state_e tx_state_d, tx_state_q;

  always_comb begin : cmd_write_state_transition
    tx_state_d    = tx_state_q;

    unique case (tx_state_q)

      READY: begin
        if (start_tx_i) begin
          tx_state_d = START_CNT;
        end
      end

      START_CNT: begin
        tx_state_d = SHIFT_REG_OUT;
      end

      SHIFT_REG_OUT: begin
        if (bit_count == (1+1+6+32)-1) begin
          tx_state_d = CRC7_OUT;
        end
      end

      CRC7_OUT: begin
        if (bit_count == (1+1+6+32+7)-1) begin
          tx_state_d = END_BIT_OUT;
        end
      end

      END_BIT_OUT: begin
        tx_state_d = READY;
      end

      default: begin
        tx_state_d = READY;
      end
    endcase
  end : cmd_write_state_transition

  `FFLARNC(tx_state_q, tx_state_d, clk_en_p_i, clear_i, READY, clk_i, rst_ni);

  ///////////////
  // Data Path //
  ///////////////

  logic [39:0] cmd_bits_47_to_8; // cmd without crc7 and end bit
  assign cmd_bits_47_to_8 [39]    = 1'b0; // start bit
  assign cmd_bits_47_to_8 [38]    = 1'b1; // transmission bit
  assign cmd_bits_47_to_8 [37:32] = cmd_nr_i;
  assign cmd_bits_47_to_8 [31:0]  = cmd_argument_i;

  logic par_write_en, shift_en, crc7_shift_en, shift_reg_out, crc7_out, sd_cmd, sd_cmd_div1, sd_cmd_divn; // control signals
  logic tx_ongoing_d, tx_ongoing_q; // if transmission is ongoing


  always_comb begin : cmd_tx_datapath
    tx_ongoing_d  = tx_ongoing_q;

    par_write_en  = 1'b0;
    shift_en      = 1'b0;
    crc7_shift_en = 1'b0;
    sd_cmd        = 1'b1;
    cmd_en_o      = 1'b0;
    tx_done_o     = 1'b1;

    unique case (tx_state_q)
      READY:        tx_ongoing_d = 1'd0;

      START_CNT:    begin
        tx_ongoing_d = 1'b1;
        par_write_en = 1'b1; // write to shift reg
        tx_done_o    = 1'b0;
      end

      SHIFT_REG_OUT: begin
        shift_en  = 1'b1; // shift out shift register
        cmd_en_o  = 1'b1; // we write to bus
        sd_cmd    = shift_reg_out;
        tx_done_o = 1'b0;
      end

      CRC7_OUT:     begin
        crc7_shift_en = 1'b1; // shift out crc7
        cmd_en_o      = 1'b1; // we write to bus
        sd_cmd        = crc7_out;
        tx_done_o     = 1'b0;
      end

      END_BIT_OUT:  begin
        sd_cmd    = 1'b1; // end bit
        cmd_en_o  = 1'b1; // we write to bus
        tx_ongoing_d = 1'b0;
        tx_done_o     = 1'b0;
      end

      default: ;
    endcase
  end : cmd_tx_datapath

  `FFLARNC(tx_ongoing_q, tx_ongoing_d, clk_en_p_i, clear_i, '0, clk_i, rst_ni);


  // delay to negative edge of clk
  always_ff @( negedge clk_i or negedge rst_ni) begin
    if(!rst_ni) sd_cmd_div1 <= 1'b1;
    else if(clear_i) sd_cmd_div1 <= 1'b1;
    else sd_cmd_div1 <= sd_cmd;
  end

  // delay to negative edge of sd_clk
  `FFLARNC(sd_cmd_divn, sd_cmd, clk_en_n_i, clear_i, '1, clk_i, rst_ni);

  // if freq of sd_clk and clk is same, delay to negative edge of clk
  // otherwise delay to negative edge of sd_clk
  assign cmd_o = (div_1_i)  ? sd_cmd_div1 : sd_cmd_divn;

  ///////////////////////////
  // Module Instantiations //
  ///////////////////////////

  par_ser_shift_reg #(
    .NumBits    (40), // start bit + transmission bit + 6 cmd bits + 32 argument bits
    .ShiftInVal (0)
  ) i_cmd_shift_reg (
    .clk_i          (clk_i),
    .clk_en_i       (clk_en_p_i),
    .rst_ni         (rst_ni),
    .clear_i        (clear_i),
    .par_write_en_i (par_write_en),
    .shift_en_i     (shift_en),
    .dat_par_i      (cmd_bits_47_to_8),
    .dat_ser_o      (shift_reg_out)
  );

  crc7_write #(
  ) i_crc7_write (
    .clk_i            (clk_i),
    .clk_en_i         (clk_en_p_i),
    .rst_ni           (rst_ni),
    .clear_i          (clear_i),
    .shift_out_crc7_i (crc7_shift_en),
    .input_en_i       (shift_en), // only listen to input when shift reg outputs
    .dat_ser_i        (shift_reg_out),
    .crc_ser_o        (crc7_out)
  );

  counter #(
    .WIDTH            (6), // 6 bit counter (48 bits to transmit)
    .STICKY_OVERFLOW  (1'b0)  // overflow not needed
  ) i_tx_bits_counter (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .clear_i    (tx_done_o || clear_i), // clears to 0
    .en_i       (tx_ongoing_q && clk_en_p_i),
    .load_i     (1'b0), // always start at 0, no loading needed
    .down_i     (1'b0), // count up
    .d_i        (6'b0), // not needed
    .q_o        (bit_count),
    .overflow_o () // overflow not needed
  );
endmodule
