// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module sdhci_sd_driver #(
  parameter time TA = 5ns,
  parameter time TT = 15ns
)(
  input  logic sd_clk_i,

  output logic sd_cmd_o,
  input  logic sd_cmd_i,
  input  logic sd_cmd_en_i,

  output logic [3:0] sd_dat_o,
  input  logic [3:0] sd_dat_i,
  input  logic       sd_dat_en_i
);

  initial begin
    sd_cmd_o  = 1'b1;
    sd_dat_o  = '1;
  end

  task automatic apply_logic (
    input logic value,
    output logic to_write
  );
    @(posedge sd_clk_i);
    #(TA);
    to_write = value;
  endtask

  task automatic wait_for_cmd_released();
    @(posedge sd_clk_i);
    #(TT);
    while (sd_cmd_en_i != 1'b0) begin
      @(posedge sd_clk_i);
      #(TT);
    end
  endtask

  task automatic wait_for_cmd_held();
    @(posedge sd_clk_i);
    #(TT);
    while (sd_cmd_en_i != 1'b1) begin
      @(posedge sd_clk_i);
      #(TT);
    end
  endtask

  task automatic wait_for_dat_released();
    @(posedge sd_clk_i);
    #(TT);
    while (sd_dat_en_i != 1'b0) begin
      @(posedge sd_clk_i);
      #(TT);
    end
  endtask

  task automatic wait_for_dat_held();
    @(posedge sd_clk_i);
    #(TT);
    while (sd_dat_en_i != 1'b1) begin
      @(posedge sd_clk_i);
      #(TT);
    end
  endtask

  task automatic send_response_48 (
    input logic [5:0] index,
    input logic [6:0] crc,
    input logic [31:0] card_status = '0,
    input logic end_bit = 1'b1
  );
    // start bit
    apply_logic(1'b0, sd_cmd_o);

    // transmission bit
    apply_logic(1'b0, sd_cmd_o);

    for (int i = 0; i < 6; i += 1) begin
      apply_logic(index[5-i], sd_cmd_o);
    end

    for (int i = 0; i < 32; i += 1) begin
      apply_logic(card_status[31-i], sd_cmd_o);
    end

    for (int i = 0; i < 7; i += 1) begin
      apply_logic(crc[6-i], sd_cmd_o);
    end

    apply_logic(end_bit, sd_cmd_o);
  endtask

  task automatic claim_busy();
    apply_logic(1'b0, sd_dat_o[0]);
  endtask

  task automatic release_busy();
    apply_logic(1'b1, sd_dat_o[0]);
  endtask

  task automatic send_response_136(
    input logic [126:0] cid_status
  );
    // start bit
    apply_logic(1'b0, sd_cmd_o);
    // transmission bit
    apply_logic(1'b0, sd_cmd_o);

    // check bits
    repeat (6) apply_logic(1'b1, sd_cmd_o);

    // cid
    for (int i = 0; i < 127; i += 1) begin
      apply_logic(cid_status[126-i], sd_cmd_o);
    end

    // check bits
    apply_logic(1'b1, sd_cmd_o);
  endtask

  task automatic send_response_dat(
    input logic is_ok
  );
    // start bit
    apply_logic(1'b0, sd_dat_o[0]);
    // transmission ok: 010, transmission not ok: 101
    // keyword in spec: CRC status token
    apply_logic(~is_ok, sd_dat_o[0]);
    apply_logic( is_ok, sd_dat_o[0]);
    apply_logic(~is_ok, sd_dat_o[0]);
    // end bit
    apply_logic(1'b1, sd_dat_o[0]);
  endtask

  task automatic send_byte(
    logic [7:0] data,
    logic is_4_bit
  );
    if (is_4_bit) begin
      @(posedge sd_clk_i);
      #(TA);
      sd_dat_o = data[7:4];

      @(posedge sd_clk_i);
      #(TA);
      sd_dat_o = data[3:0];
    end else begin
      for (int i = 0; i < 8; ++i) begin
        @(posedge sd_clk_i);
        #(TA);
        sd_dat_o[0] = data[7-i];
      end
    end
  endtask

  task automatic send_data_block(
    logic [511:0][7:0] block,
    logic [9:0] block_size,
    logic is_4_bit
  );
    // start bit
    @(posedge sd_clk_i);
    #(TA);
    if (is_4_bit) begin
      sd_dat_o = '0;
    end else begin
      sd_dat_o[0] = '0;
    end

    for (int i = 0; i < block_size; i++) begin
      send_byte(.data(block[i]), .is_4_bit(is_4_bit));
    end

    if (is_4_bit) begin
      logic [511:0][7:0] reversed_block;
      logic [3:0][1023:0] dat_channels;
      logic [3:0][15:0]   dat_crc;
      reversed_block = reverse_bytes(block, block_size);
      dat_channels = split_data_into_dat_channels(.block(reversed_block), .block_size(block_size));
      for (int i = 0; i < 4; ++i) begin
        dat_crc[i] = calculate_crc16(.data(dat_channels[i]), .data_length(block_size * 2));
      end
      for (int i = 0; i < 16; ++i) begin
        @(posedge sd_clk_i);
        #(TA);
        for (int j = 0; j < 4; ++j) begin
          sd_dat_o[j] = dat_crc[j][15 - i];
        end
      end
    end else begin
      logic [511:0][7:0] reversed_block;
      logic [4095:0] flattened_block;
      logic [15:0]   dat0_crc;
      reversed_block = reverse_bytes(block, block_size);
      dat0_crc = calculate_crc16(.data(reversed_block), .data_length(block_size * 8));
      $display("%x", dat0_crc);
      for (int i = 0; i < 16; ++i) begin
        @(posedge sd_clk_i);
        #(TA);
        sd_dat_o[0] = dat0_crc[15 - i];
      end
    end

    @(posedge sd_clk_i);
    #(TA);
    // regardless of the width, all need idle at 1
    sd_dat_o = '1;
  endtask

  function automatic logic [511:0][7:0] reverse_bytes(
    logic [511:0][7:0] data,
    int data_length
  );
    logic [511:0][7:0] reversed;
    reversed = '0;

    for (int i = 0; i < data_length; ++i) begin
      reversed[data_length-1-i] = data[i];
    end

    return reversed;
  endfunction

  function automatic logic [3:0][1023:0] split_data_into_dat_channels(
    logic [511:0][7:0] block,
    logic [9:0] block_size
  );
    int current_block_index;
    int current_flattened_index;

    logic [3:0][1023:0] result;

    result = '0;
    current_flattened_index = block_size * 2 - 2;

    for (current_block_index = block_size-1; current_block_index >= 0; --current_block_index) begin
      for (int i = 0; i < 4; ++i) begin
        result[i][current_flattened_index+1] = block[current_block_index][i+4];
        result[i][current_flattened_index  ] = block[current_block_index][i];
      end
      current_flattened_index -= 2;
    end

    return result;
  endfunction

  function automatic logic [15:0] calculate_crc16(
    logic [4095:0] data,
    int data_length
  );
    int current_index;
    logic [4095+16:0] data_padded;
    logic [16:0] crc_polynomial;

    crc_polynomial = '0;
    crc_polynomial[16] = 1'b1;
    crc_polynomial[12] = 1'b1;
    crc_polynomial[ 5] = 1'b1;
    crc_polynomial[ 0] = 1'b1;

    data_padded = '0;
    // compiler does not allow us to assign a slice
    // with dynamic indexing, so doing it by hand
    for (int i = data_length-1; i >= 0; --i) begin
      data_padded[i+16] = data[i];
    end

    current_index = data_length-1;

    while (current_index >= 0 && |(data_padded[4095+16:16])) begin
      if (data_padded[current_index + 16] == 1'b1) begin
        for (int i = 0; i < 17; ++i) begin
          data_padded[current_index + 16 - i] ^= crc_polynomial[16-i];
        end
      end
      current_index--;
    end

    return data_padded[15:0];
  endfunction

  task automatic send_data_block_interruptible(
    logic [511:0][7:0] block,
    logic [9:0] block_size,
    logic is_4_bit,
    output logic was_interrupted
  );
    fork
      fork
        begin
          send_data_block(.block(block), .block_size(block_size), .is_4_bit(is_4_bit));
          was_interrupted = 1'b0;
        end
        begin
          wait_for_cmd_held();
          wait_for_cmd_released();
          // the second cycle after the cmd
          // should be the end bit
          was_interrupted = 1'b1;
        end
      join_any
      disable fork;
    join
    if (was_interrupted) begin
      @(posedge sd_clk_i);
      #(TA);
      sd_dat_o = '1;
    end
  endtask

endmodule
