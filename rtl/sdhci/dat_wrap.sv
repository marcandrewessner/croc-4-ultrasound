// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

`include "common_cells/registers.svh"
`include "defines.svh"

module dat_wrap #(
  parameter int MaxBlockBitSize = 10, // max_block_length = 512 in caps
  parameter int unsigned BufferNumWords = 256,
  parameter bit          AllowNoncompliantBufferSizes = 1'b0,
  parameter int unsigned TimeoutDivider = 1 // by how much to divide clk_i to get the timeout count frequency,
                                            // see dat_timeout for details
) (
  input  logic clk_i,
  input  logic sd_clk_en_p_i,
  input  logic sd_clk_en_n_i,
  input  logic div_1_i,
  input  logic rst_ni,
  input  logic clear_i,

  input  logic [3:0] dat_i,
  output logic       dat_en_o,
  output logic [3:0] dat_o,

  input  logic cmd_started_i,
  input  logic cmd_needs_busy_i,
  input  logic cmd_data_present_i,
  input  logic cmd_transfer_direction_i,

  input  logic sd_cmd_done_i,
  input  logic sd_rsp_done_i,

  output logic sd_busy_o,
  output logic request_cmd12_o,
  output logic pause_sd_clk_o,

  input  sdhci_reg_pkg::sdhci_reg2hw_t reg2hw_i,

  output `writable_reg_t()       data_crc_error_o,
  output `writable_reg_t()       data_end_bit_error_o,
  output `writable_reg_t()       data_timeout_error_o,

  output logic [31:0]            buffer_data_port_d_o,
  output `writable_reg_t()       buffer_read_enable_o,
  output `writable_reg_t()       buffer_write_enable_o,
  output logic                   buffer_data_port_read_ready_o,
  output logic                   buffer_data_port_write_ready_o,

  output `writable_reg_t()       read_transfer_active_o,
  output `writable_reg_t()       write_transfer_active_o,

  output `writable_reg_t([15:0]) block_count_o
);

  logic buffer_write_ready, buffer_write_valid, buffer_read_ready, buffer_read_valid, buffer_empty;
  logic [31:0] buffer_write_data, buffer_read_data;
  logic start_read, read_valid, read_done, read_crc_err, read_end_bit_err;
  logic write_done, write_crc_timeout;
  logic timeout_elapsed;

  logic block_count_limited;
  assign block_count_limited = !reg2hw_i.transfer_mode.multi_single_block_select.q ||
                               reg2hw_i.transfer_mode.block_count_enable.q;

  logic [15:0] transmitted_block_counter_q, transmitted_block_counter_d;
  `FFARNC (transmitted_block_counter_q, transmitted_block_counter_d, clear_i, '0, clk_i, rst_ni);

  typedef enum logic [1:0] {
    READY,
    BUSY,
    READ,
    WRITE
  } dat_state_e;

  typedef enum logic [2:0] {
    BUSY_WAIT_FOR_CMD,
    BUSY_WAIT_FOR_LOW,
    BUSY_WAIT_FOR_HIGH,
    BUSY_TIMEOUT,
    BUSY_DONE
  } busy_state_e;

  typedef enum logic [2:0] {
    WAIT_FOR_CMD,
    WAIT_FOR_READ_BUFFER,
    START_READING,
    READING,
    DONE_READING_BLOCK,
    READING_BUSY,
    TIMEOUT_READING,
    DONE_READING
  } read_state_e;

  typedef enum logic [2:0] {
    WAIT_FOR_RSP,
    WAIT_FOR_WRITE_BUFFER,
    START_WRITING,
    WRITING,
    DONE_WRITING_BLOCK,
    TIMEOUT_WRITING,
    DONE_WRITING
  } write_state_e;

  dat_state_e dat_state_q, dat_state_d;
  `FFARNC (dat_state_q, dat_state_d, clear_i, READY, clk_i, rst_ni);

  busy_state_e busy_state_q, busy_state_d;
  `FFARNC (busy_state_q, busy_state_d, clear_i, BUSY_WAIT_FOR_CMD, clk_i, rst_ni);

  read_state_e read_state_q, read_state_d;
  `FFARNC (read_state_q, read_state_d, clear_i, WAIT_FOR_CMD, clk_i, rst_ni);

  write_state_e write_state_q, write_state_d;
  `FFARNC (write_state_q, write_state_d, clear_i, WAIT_FOR_RSP, clk_i, rst_ni);

  always_comb begin : main_fsm
    dat_state_d = dat_state_q;

    unique case (dat_state_q)
      READY: begin
        if (cmd_started_i) begin
          if (cmd_data_present_i) begin
            if (cmd_transfer_direction_i) begin
              dat_state_d = READ;
            end else begin
              dat_state_d = WRITE;
            end
          end else if (cmd_needs_busy_i) begin
            dat_state_d = BUSY;
          end
        end
      end

      BUSY: begin
        if (busy_state_q == BUSY_DONE) begin
          dat_state_d = READY;
        end
      end

      READ: begin
        if (read_state_q == DONE_READING) begin
          dat_state_d = READY;
        end
      end

      WRITE: begin
        if (write_state_q == DONE_WRITING) begin
          dat_state_d = READY;
        end
      end

      default: begin
        dat_state_d = READY;
      end
    endcase
  end

  assign sd_busy_o = dat_state_q == BUSY;

  logic [1:0] busy_counter_q, busy_counter_d;
  `FFARNC(busy_counter_q, busy_counter_d, clear_i, 2'b0, clk_i, rst_ni);

  logic busy_saw_response_q, busy_saw_response_d;
  `FFARNC(busy_saw_response_q, busy_saw_response_d, clear_i, 1'b0, clk_i, rst_ni);

  always_comb begin : busy_fsm
    busy_state_d        = busy_state_q;
    busy_counter_d      = busy_counter_q;
    busy_saw_response_d = busy_saw_response_q;

    if (dat_state_q != BUSY) begin
      busy_state_d = BUSY_WAIT_FOR_CMD;
      busy_saw_response_d = 1'b0;
    end else begin
      if (sd_rsp_done_i) begin
        busy_saw_response_d = 1'b1;
      end

      unique case (busy_state_q)
        BUSY_WAIT_FOR_CMD: begin
          if (sd_cmd_done_i) begin
            busy_state_d   = BUSY_WAIT_FOR_LOW;
            busy_counter_d = 2'b0;
          end
        end
        BUSY_WAIT_FOR_LOW: begin
          busy_counter_d = busy_counter_q + sd_clk_en_p_i;
          if (dat_i[0] == 1'b0) begin
            busy_state_d = BUSY_WAIT_FOR_HIGH;
          end else if (busy_counter_q == 2'b11) begin
            // if the card wants to signal busy, it needs to do
            // so within 2 cycles of the command complete,
            // refer to 7.13.1 electrical. We allow for 4.
            busy_state_d = BUSY_DONE;
          end
        end
        BUSY_WAIT_FOR_HIGH: begin
          // according to SDHC simplified, present state register,
          // dat_line_active, busy is held until after the response
          // at least, independent of what the card is actually doing
          // (wording: "when busy after the the response is de-asserted")
          if (dat_i[0] == 1'b1 && busy_saw_response_q) begin
            busy_state_d = BUSY_DONE;
          end else if (timeout_elapsed) begin
            busy_state_d = BUSY_TIMEOUT;
          end
        end
        BUSY_TIMEOUT: begin
          busy_state_d = BUSY_DONE;
        end
        BUSY_DONE: begin
          busy_state_d = BUSY_DONE;
        end
        default: begin
          busy_state_d = BUSY_WAIT_FOR_CMD;
        end
      endcase
    end
  end

  always_comb begin : read_fsm
    read_state_d = read_state_q;

    if (dat_state_q != READ) begin
      read_state_d = WAIT_FOR_CMD;
    end else begin
      unique case (read_state_q)
        WAIT_FOR_CMD: begin
          if (sd_cmd_done_i) begin
            read_state_d = START_READING;
          end
        end
        WAIT_FOR_READ_BUFFER: begin
          if (buffer_write_ready) begin
            read_state_d = START_READING;
          end
        end
        START_READING: begin
          if (sd_clk_en_p_i) begin
            read_state_d = READING;
          end
        end
        READING: begin
          if (timeout_elapsed) begin
            read_state_d = TIMEOUT_READING;
            // Reset reader
          end else if (read_done) begin
            read_state_d = DONE_READING_BLOCK;
          end
        end
        DONE_READING_BLOCK: begin
          if (block_count_limited && transmitted_block_counter_q == 'b1) begin
            read_state_d = READING_BUSY;
          end else if (buffer_write_ready) begin
            read_state_d = START_READING;
          end else begin
            read_state_d = WAIT_FOR_READ_BUFFER;
          end
        end
        READING_BUSY: begin
          if (buffer_empty) begin
            read_state_d = DONE_READING;
          end
        end
        TIMEOUT_READING: begin
          read_state_d = DONE_READING;
        end
        DONE_READING: begin
          read_state_d = DONE_READING;
        end
        default: begin
          read_state_d = WAIT_FOR_CMD;
        end
      endcase
    end
  end

  always_comb begin : write_fsm
    write_state_d = write_state_q;

    if (dat_state_q != WRITE) begin
      write_state_d = WAIT_FOR_RSP;
    end else begin
      unique case (write_state_q)
        WAIT_FOR_RSP: begin
          if (sd_rsp_done_i) begin
            write_state_d = WAIT_FOR_WRITE_BUFFER;
          end
        end
        WAIT_FOR_WRITE_BUFFER: begin
          if (buffer_read_valid) begin
            write_state_d = START_WRITING;
          end
        end
        START_WRITING: begin
          if (sd_clk_en_p_i) begin
            write_state_d = WRITING;
          end
        end
        WRITING: begin
          if (write_done) begin
            write_state_d = DONE_WRITING_BLOCK;
          end
          if (timeout_elapsed || write_crc_timeout) begin
            write_state_d = TIMEOUT_WRITING;
          end
        end
        DONE_WRITING_BLOCK: begin
          if (block_count_limited && transmitted_block_counter_q == 'b1) begin
            write_state_d = DONE_WRITING;
          end else begin
            write_state_d = WAIT_FOR_WRITE_BUFFER;
          end
        end
        TIMEOUT_WRITING: begin
          write_state_d = DONE_WRITING;
        end
        DONE_WRITING: begin
          write_state_d = DONE_WRITING;
        end

        default: begin
          write_state_d = WAIT_FOR_RSP;
        end
      endcase
    end
  end

  logic [MaxBlockBitSize-1:0] block_size;
  assign block_size = MaxBlockBitSize'(reg2hw_i.block_size.transfer_block_size.q);

  logic [MaxBlockBitSize-1:0] effective_block_size;
  assign effective_block_size = (block_size == '0) ? MaxBlockBitSize'(1) : block_size;

  logic [MaxBlockBitSize-1:0] words_per_block;
  assign words_per_block = (effective_block_size + MaxBlockBitSize'(3)) >> 2;


  logic busy_waiting;
  logic read_waiting;
  logic write_waiting;
  logic run_timeout_clock;

  assign run_timeout_clock = busy_waiting | read_waiting | write_waiting;

  logic start_write, write_requests_next_word, write_crc_err, write_end_bit_err;
  logic [31:0] write_data, read_data;
  logic pause_sd_clk_read, pause_sd_clk_write;

  assign pause_sd_clk_o = pause_sd_clk_read || pause_sd_clk_write;

  logic [MaxBlockBitSize-1:0] write_word_counter_q, write_word_counter_d;
  `FFARNC(write_word_counter_q, write_word_counter_d, clear_i, '0, clk_i, rst_ni);

  always_comb begin
    write_word_counter_d = write_word_counter_q;
    if (dat_state_q != WRITE || write_state_q == WAIT_FOR_WRITE_BUFFER ||
        write_state_q == START_WRITING || write_state_q == DONE_WRITING_BLOCK) begin
      write_word_counter_d = '0;
    end else if (write_state_q == WRITING && buffer_read_ready &&
                 write_word_counter_q < words_per_block) begin
      write_word_counter_d = write_word_counter_q + 1'b1;
    end
  end

  always_comb begin : autocmd12
    request_cmd12_o   = '0;
    if ((dat_state_q == READ || dat_state_q == WRITE) &
        (dat_state_d == READY)) begin
      if (reg2hw_i.transfer_mode.auto_cmd12_enable.q) begin
        request_cmd12_o = '1;
      end
    end
  end

  assign read_transfer_active_o  = '{de: '1, d: dat_state_q == READ};
  assign write_transfer_active_o = '{de: '1, d: dat_state_q == WRITE};

  always_comb begin : error_reporting
    data_crc_error_o     = '{ de: '0, d: '1};
    data_end_bit_error_o = '{ de: '0, d: '1};
    data_timeout_error_o = '{ de: '0, d: '1};

    if (dat_state_q == BUSY) begin
      if (busy_state_q == BUSY_TIMEOUT) begin
        data_timeout_error_o.de = '1;
      end
    end

    if (dat_state_q == READ) begin
      if (read_state_q == READING) begin
        if (read_done) begin
          data_crc_error_o.de     = read_crc_err;
          data_end_bit_error_o.de = read_end_bit_err;
        end
      end
      if (read_state_q == TIMEOUT_READING) begin
        data_timeout_error_o.de = '1;
      end
    end

    if (dat_state_q == WRITE) begin
      if (write_state_q == WRITING) begin
        if (write_done) begin
          data_crc_error_o.de     = write_crc_err;
          data_end_bit_error_o.de = write_end_bit_err;
        end
      end
      if (write_state_q == TIMEOUT_WRITING) begin
        data_timeout_error_o.de = '1;
      end
    end
  end

  logic [15:0] new_block_count;
  always_comb begin
    if (reg2hw_i.transfer_mode.multi_single_block_select.q == 1'b0) begin
      new_block_count = 1'b1;
    end else if (!reg2hw_i.transfer_mode.block_count_enable.q) begin
      new_block_count = '0;
    end else begin
      new_block_count = reg2hw_i.block_count.q;
    end
  end

  always_comb begin : block_counter
    transmitted_block_counter_d = transmitted_block_counter_q;
    if (dat_state_q == READ) begin
      if (read_state_q == WAIT_FOR_CMD) begin
        transmitted_block_counter_d = new_block_count;
      end else if (block_count_limited && read_state_q == DONE_READING_BLOCK) begin
        transmitted_block_counter_d = transmitted_block_counter_q - 1;
      end
    end else if (dat_state_q == WRITE) begin
      if (write_state_q == WAIT_FOR_RSP) begin
        transmitted_block_counter_d = new_block_count;
      end else if (block_count_limited && write_state_q == DONE_WRITING_BLOCK) begin
        transmitted_block_counter_d = transmitted_block_counter_q - 1;
      end
    end
  end

  always_comb begin : busy_control
    busy_waiting = '0;
    if (dat_state_q == BUSY) begin
      unique case (busy_state_q)
        BUSY_WAIT_FOR_CMD: ;
        BUSY_WAIT_FOR_LOW: ;
        BUSY_WAIT_FOR_HIGH: begin
          busy_waiting = '1;
        end
        BUSY_TIMEOUT: ;
        BUSY_DONE: ;
        default: ;
      endcase
    end
  end

  always_comb begin : read_control
    pause_sd_clk_read = '0;
    start_read  = '0;

    buffer_write_valid = '0;
    buffer_write_data  = 'X;

    unique case (read_state_q)
      WAIT_FOR_CMD: ;
      WAIT_FOR_READ_BUFFER: begin
        pause_sd_clk_read = '1;
      end
      START_READING: begin
        start_read = '1;
      end
      READING: begin
        if (!buffer_write_ready) begin
          pause_sd_clk_read = '1;
        end else if (read_valid) begin
          buffer_write_valid = '1;
          buffer_write_data  = read_data;
        end
      end
      DONE_READING_BLOCK: ;
      READING_BUSY: ;
      TIMEOUT_READING: ;
      DONE_READING: ;
      default: ;
    endcase
  end

  always_comb begin : write_control
    start_write = '0;
    write_data  = 'X;
    buffer_read_ready  = '0;
    pause_sd_clk_write = '0;

    unique case (write_state_q)
      WAIT_FOR_RSP: ;
      WAIT_FOR_WRITE_BUFFER: ;
      START_WRITING: begin
        start_write = '1;
      end
      WRITING: begin
        if (!buffer_read_valid && write_word_counter_q < words_per_block) begin
          pause_sd_clk_write = '1;
        end else if (write_requests_next_word && write_word_counter_q < words_per_block) begin
          buffer_read_ready = '1;
        end

        write_data = buffer_read_data;
      end
      DONE_WRITING_BLOCK: ;
      TIMEOUT_WRITING: ;
      DONE_WRITING: ;
      default: ;
    endcase
  end

  dat_timeout #(
    .ClockDiv(TimeoutDivider)
  ) i_timeout (
    .clk_i,
    .rst_ni,

    .running_i      (run_timeout_clock),
    .timeout_bits_i (reg2hw_i.timeout_control.data_timeout_counter_value),

    .timeout_o      (timeout_elapsed)
  );


  dat_buffer #(
    .NumWords        (BufferNumWords),
    .MaxBlockBitSize (MaxBlockBitSize),
    .AllowNoncompliantBufferSizes (AllowNoncompliantBufferSizes)
  ) i_dat_buffer (
    .clk_i,
    .rst_ni,
    .clear_i,

    .read_operation_i  (reg2hw_i.present_state.read_transfer_active.q),
    .write_operation_i (reg2hw_i.present_state.write_transfer_active.q),

    .read_ready_i  (buffer_read_ready),
    .read_valid_o  (buffer_read_valid),
    .read_data_o   (buffer_read_data),

    .write_valid_i (buffer_write_valid),
    .write_data_i  (buffer_write_data),
    .write_ready_o (buffer_write_ready),

    .empty_o       (buffer_empty),
    .buffer_data_port_read_ready_o,
    .buffer_data_port_write_ready_o,

    .block_size_i             (block_size),
    .block_count_i            (reg2hw_i.block_count.q),
    .multi_block_i            (reg2hw_i.transfer_mode.multi_single_block_select.q),
    .block_count_enable_i     (reg2hw_i.transfer_mode.block_count_enable.q),
    .buffer_data_port_re_i    (reg2hw_i.buffer_data_port.re),
    .buffer_data_port_qe_i    (reg2hw_i.buffer_data_port.qe),
    .buffer_data_port_q_i     (reg2hw_i.buffer_data_port.q),
    .buffer_data_port_d_o,
    .buffer_read_enable_o,
    .buffer_write_enable_o,
    .block_count_o
  );

  dat_read #(
    .MaxBlockBitSize (MaxBlockBitSize)
  ) i_read (
    .clk_i,
    .sd_clk_en_i   (sd_clk_en_p_i),
    .rst_ni,
    .clear_i,
    .dat_i,

    .start_i          (start_read),
    .timeout_i        (timeout_elapsed),
    .block_size_i     (effective_block_size),
    .bus_width_is_4_i (reg2hw_i.host_control.data_transfer_width.q),

    .data_valid_o  (read_valid),
    .data_o        (read_data),

    .waiting_o     (read_waiting),
    .done_o        (read_done),
    .crc_err_o     (read_crc_err),
    .end_bit_err_o (read_end_bit_err)
  );

  dat_write #(
    .MaxBlockBitSize (MaxBlockBitSize)
  ) i_write (
    .clk_i,
    .sd_clk_en_p_i  (sd_clk_en_p_i),
    .sd_clk_en_n_i  (sd_clk_en_n_i),
    .div_1_i        (div_1_i),
    .rst_ni,
    .clear_i,
    .dat0_i         (dat_i[0]),
    .dat_o,
    .dat_en_o,

    .start_i          (start_write),
    .block_size_i     (effective_block_size),
    .bus_width_is_4_i (reg2hw_i.host_control.data_transfer_width.q),

    .data_i        (write_data),
    .next_word_o   (write_requests_next_word),

    .data_timeout_o(write_crc_timeout),
    .waiting_o     (write_waiting),
    .done_o        (write_done),
    .crc_err_o     (write_crc_err),
    .end_bit_err_o (write_end_bit_err)
  );

`ifndef SYNTHESIS
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (rst_ni && !clear_i && cmd_started_i && cmd_data_present_i) begin
      assert (block_size != '0)
        else $error("DAT block size must be non-zero for data commands");
    end
  end
`endif
endmodule
