// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`include "defines.svh"

module tb_dat_buffer_sizes #(
  parameter time ClkPeriod = 50ns
)();
  logic clk;
  logic rst_n;
  logic [2:0] done;

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriod ),
    .RstClkCycles ( 1 )
  ) i_clk_rst_sys (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );

  tb_dat_buffer_size_case #(
    .NumWords(8),
    .AllowNoncompliantBufferSizes(1'b1)
  ) i_32b_noncompliant (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .done_o (done[0])
  );

  tb_dat_buffer_size_case #(.NumWords(128)) i_512b (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .done_o (done[1])
  );

  tb_dat_buffer_size_case #(.NumWords(256)) i_1024b (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .done_o (done[2])
  );

  initial begin
    wait(&done);
    repeat (10) @(posedge clk);
    $finish();
  end
endmodule

module tb_dat_buffer_size_case #(
  parameter int unsigned NumWords = 8,
  parameter int unsigned BlockSize = 512,
  parameter bit          AllowNoncompliantBufferSizes = 1'b0
) (
  input  logic clk_i,
  input  logic rst_ni,
  output logic done_o
);
  localparam int unsigned BlockWords = BlockSize / 4;

  logic clear;
  logic read_operation;
  logic write_operation;
  logic read_ready;
  logic read_valid;
  logic [31:0] read_data;
  logic write_valid;
  logic [31:0] write_data;
  logic write_ready;
  logic empty;
  logic [31:0] buffer_data_port_d;
  logic buffer_data_port_read_ready;
  logic buffer_data_port_write_ready;
  logic buffer_read_enable;
  logic buffer_write_enable;
  logic [15:0] block_count;

  logic [9:0] block_size;
  logic [15:0] programmed_block_count;
  logic multi_block;
  logic block_count_enable;
  logic buffer_data_port_re;
  logic buffer_data_port_qe;
  logic [31:0] buffer_data_port_q;
  `writable_reg_t() buffer_read_enable_hw;
  `writable_reg_t() buffer_write_enable_hw;
  `writable_reg_t([15:0]) block_count_hw;

  assign buffer_read_enable = buffer_read_enable_hw.d;
  assign buffer_write_enable = buffer_write_enable_hw.d;
  assign block_count = block_count_hw.d;

  dat_buffer #(
    .NumWords        (NumWords),
    .MaxBlockBitSize (10),
    .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes)
  ) i_dat_buffer (
    .clk_i,
    .rst_ni,
    .clear_i           (clear),
    .read_operation_i  (read_operation),
    .write_operation_i (write_operation),
    .read_ready_i      (read_ready),
    .read_valid_o      (read_valid),
    .read_data_o       (read_data),
    .write_valid_i     (write_valid),
    .write_data_i      (write_data),
    .write_ready_o     (write_ready),
    .empty_o           (empty),
    .buffer_data_port_read_ready_o(buffer_data_port_read_ready),
    .buffer_data_port_write_ready_o(buffer_data_port_write_ready),
    .block_size_i      (block_size),
    .block_count_i     (programmed_block_count),
    .multi_block_i     (multi_block),
    .block_count_enable_i(block_count_enable),
    .buffer_data_port_re_i(buffer_data_port_re),
    .buffer_data_port_qe_i(buffer_data_port_qe),
    .buffer_data_port_q_i(buffer_data_port_q),
    .buffer_data_port_d_o(buffer_data_port_d),
    .buffer_read_enable_o(buffer_read_enable_hw),
    .buffer_write_enable_o(buffer_write_enable_hw),
    .block_count_o     (block_count_hw)
  );

  task automatic init_inputs();
    clear = 1'b0;
    read_operation = 1'b0;
    write_operation = 1'b0;
    read_ready = 1'b0;
    write_valid = 1'b0;
    write_data = '0;
    block_size = BlockSize;
    programmed_block_count = 16'd2;
    multi_block = 1'b1;
    block_count_enable = 1'b0;
    buffer_data_port_re = 1'b0;
    buffer_data_port_qe = 1'b0;
    buffer_data_port_q = '0;
  endtask

  task automatic tick();
    @(posedge clk_i);
    #1ns;
  endtask

  task automatic pulse_clear();
    clear = 1'b1;
    tick();
    clear = 1'b0;
    tick();
  endtask

  task automatic test_default_read_side();
    read_operation = 1'b1;
    tick();

    for (int unsigned i = 0; i < BlockWords - 1; i++) begin
      repeat (10) begin
        if (write_ready) begin
          break;
        end
        tick();
      end
      if (!write_ready) begin
        $fatal(1, "read-side buffer did not become writable for NumWords=%0d", NumWords);
      end
      write_data = 32'h1000_0000 + i;
      write_valid = 1'b1;
      tick();
    end

    write_valid = 1'b0;
    tick();
    if (buffer_read_enable) begin
      $fatal(1, "buffer_read_enable asserted before one chunk for NumWords=%0d", NumWords);
    end

    write_data = 32'h1000_0000 + BlockWords - 1;
    write_valid = 1'b1;
    tick();
    write_valid = 1'b0;
    tick();

    if (!buffer_read_enable) begin
      $fatal(1, "buffer_read_enable did not assert after one block for NumWords=%0d", NumWords);
    end

    for (int unsigned i = 0; i < BlockWords; i++) begin
      buffer_data_port_re = 1'b1;
      tick();
      buffer_data_port_re = 1'b0;
      write_valid = 1'b0;
      tick();
    end

    read_operation = 1'b0;
  endtask

  task automatic test_default_write_side();
    write_operation = 1'b1;
    tick();
    if (!buffer_write_enable) begin
      $fatal(1, "buffer_write_enable not asserted on empty buffer for NumWords=%0d", NumWords);
    end

    for (int unsigned i = 0; i < BlockWords; i++) begin
      buffer_data_port_q = 32'h2000_0000 + i;
      buffer_data_port_qe = 1'b1;
      tick();
      buffer_data_port_qe = 1'b0;
      tick();
    end

    if (BlockWords == NumWords && buffer_write_enable) begin
      $fatal(1, "buffer_write_enable remained asserted on full buffer for NumWords=%0d", NumWords);
    end

    for (int unsigned i = 0; i < BlockWords; i++) begin
      if (!read_valid) begin
        $fatal(1, "read_valid deasserted before buffered chunk drained for NumWords=%0d", NumWords);
      end
      read_ready = 1'b1;
      tick();
      read_ready = 1'b0;
      tick();
    end

    write_operation = 1'b0;
  endtask

  task automatic test_noncompliant_read_side();
    int unsigned produced;
    int unsigned consumed;
    int unsigned cycles;

    read_operation = 1'b1;
    produced = 0;
    consumed = 0;
    cycles = 0;
    tick();

    while (consumed < BlockWords) begin
      logic accepted_write;
      logic accepted_read;

      write_valid = 1'b0;
      buffer_data_port_re = 1'b0;

      if (produced < BlockWords && write_ready) begin
        write_data = 32'h3000_0000 + produced;
        write_valid = 1'b1;
      end else if (buffer_data_port_read_ready) begin
        buffer_data_port_re = 1'b1;
      end

      accepted_write = write_valid && write_ready;
      accepted_read = buffer_data_port_re && buffer_data_port_read_ready;
      tick();
      if (accepted_write) begin
        produced++;
      end
      if (accepted_read) begin
        consumed++;
      end

      cycles++;
      if (cycles > BlockWords * 16) begin
        $fatal(1, "noncompliant read-side buffer stalled for NumWords=%0d produced=%0d consumed=%0d",
               NumWords, produced, consumed);
      end
    end

    write_valid = 1'b0;
    buffer_data_port_re = 1'b0;
    read_operation = 1'b0;
  endtask

  task automatic test_noncompliant_write_stall_resume();
    write_operation = 1'b1;
    tick();

    for (int unsigned i = 0; i < NumWords; i++) begin
      if (!buffer_data_port_write_ready) begin
        $fatal(1, "noncompliant write-side buffer became full too early for NumWords=%0d", NumWords);
      end
      buffer_data_port_q = 32'h4000_0000 + i;
      buffer_data_port_qe = 1'b1;
      tick();
    end
    buffer_data_port_qe = 1'b0;
    tick();

    if (buffer_data_port_write_ready) begin
      $fatal(1, "noncompliant write-side Buffer Data Port did not stall when full for NumWords=%0d", NumWords);
    end

    buffer_data_port_q = 32'h4BAD_F00D;
    buffer_data_port_qe = 1'b1;
    tick();
    buffer_data_port_qe = 1'b0;

    read_ready = 1'b1;
    tick();
    read_ready = 1'b0;

    repeat (4) begin
      if (buffer_data_port_write_ready) begin
        break;
      end
      tick();
    end
    if (!buffer_data_port_write_ready) begin
      $fatal(1, "noncompliant write-side Buffer Data Port did not resume for NumWords=%0d", NumWords);
    end

    write_operation = 1'b0;
  endtask

  task automatic test_noncompliant_write_side();
    int unsigned produced;
    int unsigned consumed;
    int unsigned cycles;

    write_operation = 1'b1;
    produced = 0;
    consumed = 0;
    cycles = 0;
    tick();

    while (consumed < BlockWords) begin
      logic accepted_write;
      logic accepted_read;

      buffer_data_port_qe = 1'b0;
      read_ready = 1'b0;

      if (produced < BlockWords && buffer_data_port_write_ready) begin
        buffer_data_port_q = 32'h5000_0000 + produced;
        buffer_data_port_qe = 1'b1;
      end else if (read_valid) begin
        read_ready = 1'b1;
      end

      accepted_write = buffer_data_port_qe && buffer_data_port_write_ready;
      accepted_read = read_ready && read_valid;
      tick();
      if (accepted_write) begin
        produced++;
      end
      if (accepted_read) begin
        consumed++;
      end

      cycles++;
      if (cycles > BlockWords * 16) begin
        $fatal(1, "noncompliant write-side buffer stalled for NumWords=%0d produced=%0d consumed=%0d",
               NumWords, produced, consumed);
      end
    end

    buffer_data_port_qe = 1'b0;
    read_ready = 1'b0;
    write_operation = 1'b0;
  endtask

  initial begin
    done_o = 1'b0;
    init_inputs();
    wait(rst_ni);
    tick();

    if (AllowNoncompliantBufferSizes) begin
      pulse_clear();
      test_noncompliant_read_side();
      pulse_clear();
      test_noncompliant_write_stall_resume();
      pulse_clear();
      test_noncompliant_write_side();
    end else begin
      pulse_clear();
      test_default_read_side();
      pulse_clear();
      test_default_write_side();
    end

    done_o = 1'b1;
  end
endmodule
