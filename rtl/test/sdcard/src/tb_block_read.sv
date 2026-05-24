// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module tb_block_read #(
    parameter time         ClkPeriod     = 50ns,
    parameter int unsigned RstCycles     = 1,
    parameter int unsigned ClkEnPeriod   = 1,
    parameter int unsigned BlockSize     = 512,
    parameter int unsigned BlockCount    = 2,
    parameter logic        Do4Bit        = 1'b1,
    parameter int unsigned BufferNumWords = 256,
    parameter bit          AllowNoncompliantBufferSizes = 1'b0
)();
  localparam int unsigned ExpectedChunkBytes =
      AllowNoncompliantBufferSizes ? BufferNumWords * 4 : 512;
  localparam logic [31:0] ExpectedVendorCapabilities = {
      AllowNoncompliantBufferSizes, 15'h0, 16'(ExpectedChunkBytes)};

  sdhci_fixture #(
    .ClkPeriod(ClkPeriod),
    .RstCycles(RstCycles),
    .BufferNumWords(BufferNumWords),
    .AllowNoncompliantBufferSizes(AllowNoncompliantBufferSizes)
  ) fixture ();

  task automatic wfi(input int unsigned timeout_cycles, string error_context);
    fork
      begin
        fork
          begin
            fixture.vip.wait_for_interrupt();
          end
          begin
            repeat(timeout_cycles) fixture.vip.wait_for_sdclk();
            $fatal(1, "Interrupt timed out waiting for %s", error_context);
          end
        join_any
        disable fork;
      end
    join
  endtask

  task automatic check_irq(logic [15:0] expected_normal, logic [15:0] expected_error, string error_context);
    logic [15:0] error_interrupt_status;
    logic [15:0] error_interrupt_status_all;
    logic [15:0] normal_interrupt_status;
    logic [15:0] normal_interrupt_status_all;

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    normal_interrupt_status_all = normal_interrupt_status;
    error_interrupt_status_all  = error_interrupt_status;

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    while (normal_interrupt_status || error_interrupt_status) begin
      fixture.vip.obi.clear_interrupt_status(
        .normal_interrupt_status(normal_interrupt_status),
        .error_interrupt_status(error_interrupt_status)
      );

      normal_interrupt_status_all |= normal_interrupt_status;
      error_interrupt_status_all  |= error_interrupt_status;

      fixture.vip.obi.get_interrupt_status(
        .normal_interrupt_status(normal_interrupt_status),
        .error_interrupt_status(error_interrupt_status)
      );
    end

    if (error_interrupt_status_all != expected_error) begin
      $fatal(1, "Unexpected error interrupt status, got %x, expected %x (%s)", error_interrupt_status_all, expected_error, error_context);
    end

    if (normal_interrupt_status_all != expected_normal) begin
      $fatal(1, "Unexpected normal interrupt status, got %x, expected %x (%s)", normal_interrupt_status_all, expected_normal, error_context);
    end
  endtask

  task automatic wait_irq_bits(logic [15:0] required_normal, logic [15:0] allowed_normal,
                               logic [15:0] expected_error, int unsigned timeout_cycles,
                               string error_context);
    logic [15:0] error_interrupt_status;
    logic [15:0] normal_interrupt_status;
    logic [15:0] normal_interrupt_status_seen;

    normal_interrupt_status_seen = '0;
    while ((normal_interrupt_status_seen & required_normal) != required_normal) begin
      wfi(timeout_cycles, error_context);
      fixture.vip.obi.get_interrupt_status(
        .normal_interrupt_status(normal_interrupt_status),
        .error_interrupt_status(error_interrupt_status)
      );
      fixture.vip.obi.clear_interrupt_status(
        .normal_interrupt_status(normal_interrupt_status),
        .error_interrupt_status(error_interrupt_status)
      );

      if (error_interrupt_status != expected_error) begin
        $fatal(1, "Unexpected error interrupt status, got %x, expected %x (%s)",
               error_interrupt_status, expected_error, error_context);
      end
      if (normal_interrupt_status & ~allowed_normal) begin
        $fatal(1, "Unexpected normal interrupt status, got %x, allowed %x (%s)",
               normal_interrupt_status, allowed_normal, error_context);
      end

      normal_interrupt_status_seen |= normal_interrupt_status;
    end
  endtask

  task automatic read_noncompliant_block(output logic [31:0] read_data);
    int unsigned words_read;
    int unsigned words_left;
    int unsigned words_this_chunk;

    words_read = 0;
    while (words_read < BlockSize / 4) begin
      wait_irq_bits(
        .required_normal('h20), // noncompliant data ready
        .allowed_normal ('h20),
        .expected_error ('h0),
        .timeout_cycles (BlockSize * 8 + 500),
        .error_context  ("noncompliant cmd18 data ready")
      );

      words_left = (BlockSize / 4) - words_read;
      words_this_chunk = (words_left < BufferNumWords) ? words_left : BufferNumWords;
      repeat (words_this_chunk) begin
        fixture.vip.obi.read_buffer_data(.data(read_data));
      end
      words_read += words_this_chunk;
    end
  endtask

  initial begin : cmd_response
    fixture.vip.wait_for_reset();

    // cmd0 with data
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48('d18, 'h3A);

    // cmd12 with busy
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48('d12, 'h7A);
  end

  initial begin : dat_response
    logic was_interrupted;
    logic [511:0][7:0] block;
    block = {
      // 512 * 8 bits
      // -> 4096
      64 {
        // 64 bits
        {8'hde},
        {8'had},
        {8'hbe},
        {8'hef},
        {8'hca},
        {8'hfe},
        {8'hba},
        {8'hbe}
      }
  };

    was_interrupted = 1'b0;
    fixture.vip.wait_for_reset();

    // wait for the read command
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    while (was_interrupted == 1'b0) begin
      fixture.vip.wait_for_sdclk();
      fixture.vip.sd.send_data_block_interruptible(
        .block(block),
        .block_size(BlockSize),
        .is_4_bit(Do4Bit),
        .was_interrupted(was_interrupted)
      );
      repeat(100) fixture.vip.wait_for_sdclk();
    end
  end

  initial begin : obi_driver
    logic [31:0] read_data;
    logic [31:0] vendor_capabilities;
    logic buffer_read_enable, buffer_write_enable;

    fixture.vip.wait_for_reset();
    fixture.vip.obi.obi_read('h044, 4'b1111, vendor_capabilities);
    if (vendor_capabilities != ExpectedVendorCapabilities) begin
      $fatal(1, "Unexpected vendor capabilities: got 0x%08x expected 0x%08x",
             vendor_capabilities, ExpectedVendorCapabilities);
    end

    fixture.vip.obi.set_interrupt_status_enable(
      .normal_interrupt_status_enable('hFFFF),
      .error_interrupt_status_enable('hFFFF),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_interrupt_signal_enable(
      .normal_interrupt_signal_enable('hFFFF),
      .error_interrupt_signal_enable('hFFFF),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.set_host_control_1(
      .dma_select('0),
      .high_speed_enable(1'b1),
      .do_4_bit_transfer(Do4Bit),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.set_frequency_select(
      .divider(8'(ClkEnPeriod >> 1)),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_clock_enable(.enable(1'b1), .finish_transaction(1'b0));

    fixture.vip.obi.set_transfer_mode(
      .is_multi_block(1'b1),
      .is_read(1'b1),
      .auto_cmd12_enable(1'b0),
      .block_count_enable(1'b1),
      .dma_enable(1'b0),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.set_block_size_count(
      .block_size(BlockSize),
      .block_count(BlockCount),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.launch_command(
      .command_index(6'd18),
      .command_type (2'b00), // normal command
      .data_present (1'b1),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b10), // 48 bit no busy
      .finish_transaction(1'b1)
    );

    if (AllowNoncompliantBufferSizes) begin
      wait_irq_bits(
        .required_normal('h01), // cmd complete
        .allowed_normal ('h21), // cmd complete and noncompliant per-word data ready
        .expected_error ('h0),  // no error
        .timeout_cycles (BlockSize * 8 + 500),
        .error_context  ("noncompliant cmd18 complete")
      );

      repeat (BlockCount) begin
        read_noncompliant_block(read_data);
      end

      wait_irq_bits(
        .required_normal('h02), // transfer complete
        .allowed_normal ('h22), // transfer complete and noncompliant data ready
        .expected_error ('h0),  // no error
        .timeout_cycles (BlockSize * 8 + 500),
        .error_context  ("noncompliant cmd18 transfer complete")
      );
    end else begin
      wfi(200, "cmd18 complete");
      check_irq(
        .expected_normal('h01), // cmd complete
        .expected_error ('h0),  // no error
        .error_context("cmd18 complete")
      );

      wfi(BlockSize * 8 + 500, "first data present");
      check_irq(
        .expected_normal('h20), // data present
        .expected_error ('h0),  // no error
        .error_context("first data present")
      );

      repeat (BlockCount - 1) begin
        repeat (BlockSize / 4) begin
          fixture.vip.obi.read_buffer_data(.data(read_data));
        end
        fixture.vip.obi.get_present_status_buffer_enable(
          .buffer_read_enable(buffer_read_enable),
          .buffer_write_enable(buffer_write_enable)
        );
        if (!buffer_read_enable) begin
          wfi(BlockSize * 8 + 500, "data present during loop");
        end
        check_irq(
          .expected_normal('h20), // data present
          .expected_error ('h0),  // no error
          .error_context("data present during loop")
        );
      end
      repeat (BlockSize / 4) begin
        fixture.vip.obi.read_buffer_data(.data(read_data));
      end
      wfi(200, "cmd18 transfer complete");
      check_irq(
        .expected_normal('h02), // transfer complete
        .expected_error ('h0),  // no error
        .error_context("cmd18 transfer complete")
      );
    end

    fixture.vip.obi.get_present_status_buffer_enable(
      .buffer_read_enable(buffer_read_enable),
      .buffer_write_enable(buffer_write_enable)
    );

    if (buffer_read_enable) begin
      $fatal(1, "We should no longer have data!");
    end

    fixture.vip.obi.launch_command(
      .command_index(6'd12),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b11), // 48 bit with busy
      .finish_transaction(1'b1)
    );

    if (AllowNoncompliantBufferSizes) begin
      wait_irq_bits(
        .required_normal('h03), // cmd complete and transfer complete
        .allowed_normal ('h03),
        .expected_error ('h0),  // no error
        .timeout_cycles (200),
        .error_context  ("cmd12 complete and transfer complete")
      );
    end else begin
      wfi(200, "cmd12 complete and transfer complete");
      check_irq(
        .expected_normal('h03), // cmd complete
        .expected_error ('h0),  // no error
        .error_context("cmd12 complete and transfer complete")
      );
    end

    $display("All good");

    $finish();
  end

endmodule

module tb_noncompliant_block_read();
  tb_block_read #(
    .BlockCount(2),
    .BufferNumWords(8),
    .AllowNoncompliantBufferSizes(1'b1)
  ) i_noncompliant_block_read ();
endmodule
