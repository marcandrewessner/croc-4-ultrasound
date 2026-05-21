// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Anton Buchner <abuchner@student.ethz.ch>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "defines.svh"

module dat_buffer #(
  parameter int unsigned NumWords        = 256,
  parameter int unsigned MaxBlockBitSize = 10,
  parameter bit          AllowNoncompliantBufferSizes = 1'b0
) (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,

  input  logic read_operation_i,
  input  logic write_operation_i,

  input  logic        read_ready_i,
  output logic        read_valid_o,
  output logic [31:0] read_data_o,

  input  logic        write_valid_i,
  input  logic [31:0] write_data_i,
  output logic        write_ready_o,

  output logic        empty_o,
  output logic        buffer_data_port_read_ready_o,
  output logic        buffer_data_port_write_ready_o,

  input  logic [MaxBlockBitSize-1:0] block_size_i,
  input  logic [15:0]                block_count_i,
  input  logic                       multi_block_i,
  input  logic                       block_count_enable_i,
  input  logic                       buffer_data_port_re_i,
  input  logic                       buffer_data_port_qe_i,
  input  logic [31:0]                buffer_data_port_q_i,

  output logic [31:0]      buffer_data_port_d_o,
  output `writable_reg_t() buffer_read_enable_o,
  output `writable_reg_t() buffer_write_enable_o,

  output `writable_reg_t([15:0]) block_count_o
);
  localparam int NumBytes = NumWords * 4;

  `ASSERT_INIT(BufferSizeAtLeast32B, NumBytes >= 32, "data buffer must be at least 32 bytes")
  `ASSERT_INIT(BufferSizeStandardBlock,
      AllowNoncompliantBufferSizes || NumBytes >= 512,
      "set AllowNoncompliantBufferSizes to use a non-SDHCI-compliant buffer below 512 bytes")
  `ASSERT_INIT(BufferSizeAtMost1KiB, NumBytes <= 1024, "data buffer must be at most 1024 bytes")
  `ASSERT_INIT(BufferSizePowerOfTwo, (NumWords & (NumWords - 1)) == 0, "data buffer word count must be a power of two")

  logic [MaxBlockBitSize-1:0] effective_block_size;
  assign effective_block_size = (block_size_i == '0) ? MaxBlockBitSize'(1) : block_size_i;

  logic [MaxBlockBitSize-1:0] words_per_block;
  assign words_per_block = (effective_block_size + MaxBlockBitSize'(3)) >> 2;

  logic [MaxBlockBitSize-1:0] current_word_counter_q, current_word_counter_d;
  `FFARNC (current_word_counter_q, current_word_counter_d, clear_i, '0, clk_i, rst_ni);

  logic single_block_done_q, single_block_done_d;
  `FFARNC(single_block_done_q, single_block_done_d, clear_i, 1'b0, clk_i, rst_ni);

  logic reg_empty;
  assign empty_o = reg_empty;

  logic [cf_math_pkg::idx_width(NumWords + 1)-1:0] reg_length;
  logic [cf_math_pkg::idx_width(NumBytes + 1)-1:0] reg_remaining_bytes;
  assign reg_remaining_bytes = (cf_math_pkg::idx_width(NumBytes + 1))'(NumBytes - reg_length * 4);

  logic has_block, has_block_space;
  assign has_block       = reg_length * 4 >= effective_block_size;
  assign has_block_space = reg_remaining_bytes >= effective_block_size;

  logic accepts_data_port_chunk;
  assign accepts_data_port_chunk =
      (!multi_block_i && !single_block_done_q) ||
      (multi_block_i && (!block_count_enable_i || block_count_i != '0));

  logic enable_reg;
  assign enable_reg = read_operation_i || write_operation_i;

  logic reg_full, reg_push, reg_pop;
  logic [31:0] reg_push_data, reg_pop_data;

  always_comb begin
    reg_push      = '0;
    reg_push_data = 'X;
    reg_pop       = '0;

    buffer_read_enable_o  = '{ de: '1, d: '0 };
    buffer_write_enable_o = '{ de: '1, d: '0 };
    buffer_data_port_d_o  = '0;
    buffer_data_port_read_ready_o  = '0;
    buffer_data_port_write_ready_o = '0;

    block_count_o = '{ de: '0, d: 'X };

    write_ready_o = '0;
    read_valid_o  = '0;
    read_data_o   = 'X;

    if (read_operation_i) begin
      reg_push      = write_valid_i && !reg_full;
      reg_push_data = write_data_i;
      write_ready_o = !reg_full;

      buffer_data_port_read_ready_o = accepts_data_port_chunk && !reg_empty && !write_valid_i;
      buffer_read_enable_o.d = accepts_data_port_chunk &&
                               (AllowNoncompliantBufferSizes ? !reg_empty : has_block) && !write_valid_i;
      buffer_data_port_d_o   = reg_pop_data;
      reg_pop                = buffer_data_port_re_i && buffer_data_port_read_ready_o;
    end else if (write_operation_i) begin
      reg_pop      = read_ready_i && !reg_empty;
      read_data_o  = reg_pop_data;
      read_valid_o = !reg_empty;

      buffer_data_port_write_ready_o = accepts_data_port_chunk && !reg_full;
      buffer_write_enable_o.d = accepts_data_port_chunk &&
                                 (AllowNoncompliantBufferSizes ? !reg_full : has_block_space);
      reg_push_data           = buffer_data_port_q_i;
      reg_push                = buffer_data_port_qe_i && buffer_data_port_write_ready_o;
    end


    current_word_counter_d = current_word_counter_q;
    single_block_done_d = single_block_done_q;

    if (!read_operation_i && !write_operation_i) begin
      single_block_done_d = 1'b0;
    end

    if ((read_operation_i && reg_pop) || (write_operation_i && reg_push)) begin
      if (current_word_counter_q == words_per_block - 1) begin
        current_word_counter_d = '0;

        if (!multi_block_i) begin
          single_block_done_d = 1'b1;
        end

        if (multi_block_i && block_count_enable_i) begin
          // TODO this will have to be changed when adding support for suspend / resume
          block_count_o = '{ de: '1, d: block_count_i - 1 };
          if (block_count_i != 'b1) begin
            // To trigger an interrupt
            buffer_write_enable_o.d = '0;
            buffer_read_enable_o.d  = '0;
          end
        end
      end else begin
        current_word_counter_d = current_word_counter_q + 1;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (rst_ni && !clear_i && enable_reg) begin
      assert (block_size_i != '0)
        else $error("DAT block size must be non-zero during data transfers");
      assert (AllowNoncompliantBufferSizes || effective_block_size <= NumBytes)
        else $error("set AllowNoncompliantBufferSizes to use a DAT buffer smaller than the transfer block");
    end
  end
`endif


  sram_shift_reg #(
    .NumWords (NumWords)
  ) i_sram_shift_reg (
    .clk_i,
    .rst_ni,
    .clear_i,
  
    .en_i (enable_reg),

    .pop_front_i  (reg_pop),
    .front_data_o (reg_pop_data),
  
    .push_back_i  (reg_push),
    .back_data_i  (reg_push_data),
  
    .full_o   (reg_full),
    .empty_o  (reg_empty),
    .length_o (reg_length)
  );
endmodule
