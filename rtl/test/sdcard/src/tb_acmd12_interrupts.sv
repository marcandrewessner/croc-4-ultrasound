// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module tb_acmd12_interrupts #(
    parameter time         ClkPeriod     = 50ns,
    parameter int unsigned RstCycles     = 1
)();

  int ClkEnPeriod;

  sdhci_fixture #(
    .ClkPeriod(ClkPeriod),
    .RstCycles(RstCycles)
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

  initial begin : cmd_response
    fixture.vip.wait_for_reset();

    // cmd0 with data
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48(0, '0);

    // autocmd12 with busy
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48('d12, 'h7A);
  end

  initial begin : dat_response
    fixture.vip.wait_for_reset();

    // write
    fixture.vip.sd.wait_for_dat_held();
    fixture.vip.sd.wait_for_dat_released();

    // crc status after 2 idle cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_dat(.is_ok(1'b1));

    // claim busy for a while to assert interrupt timing
    fixture.vip.sd.claim_busy();
    repeat(50) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.release_busy();

    // autocmd12 with busy
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // claim busy for a while to assert interrupt timing
    fixture.vip.sd.claim_busy();
    repeat(50) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.release_busy();
  end

  initial begin : obi_driver
    fixture.vip.wait_for_reset();
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

    fixture.vip.obi.set_frequency_select(
      .divider(8'(ClkEnPeriod >> 1)),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_clock_enable(.enable(1'b1), .finish_transaction(1'b0));

    fixture.vip.obi.set_transfer_mode(
      .is_multi_block(1'b0),
      .is_read(1'b0),
      .auto_cmd12_enable(1'b1),
      .block_count_enable(1'b0),
      .dma_enable(1'b0),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.set_block_size_count(
      .block_size(12'd64),
      .block_count(16'd1),
      .finish_transaction(1'b0)
    );

    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b1),
      .index_check_enable(1'b0),
      .crc_check_enable(1'b0),
      .response_type(2'b10), // 48 bit no busy
      .finish_transaction(1'b1)
    );

    wfi(100, "buffer write ready");
    check_irq(
      .expected_normal('h10), // buffer write ready, command complete
      .expected_error ('h0),  // no error
      .error_context("buffer write ready, no error")
    );

    repeat (64 / 4 - 1) begin
      fixture.vip.obi.write_buffer_data(.data('hDEAD_BEEF), .finish_transaction(1'b0));
    end
    fixture.vip.obi.write_buffer_data(.data('hDEAD_BEEF), .finish_transaction(1'b1));

    wfi(100, "command complete");
    check_irq(
      .expected_normal('h01), // command complete
      .expected_error ('h0),  // no error
      .error_context("command complete")
    );

    fixture.vip.sd.wait_for_dat_held();
    fixture.vip.sd.wait_for_dat_released();

    // make sure that we do not receive an interrupt during busy being held
    repeat(50) fixture.vip.wait_for_sdclk();
    wfi(20, "transfer complete after write");
    check_irq(
      .expected_normal('h2), // transfer complete
      .expected_error ('h0), // no error
      .error_context  ("transfer complete after write")
    );

    // wait for autocmd12
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    // make sure that we do not receive an interrupt during busy being held
    repeat(48) fixture.vip.wait_for_sdclk();
    wfi(20, "transfer complete after autocmd");
    check_irq(
      .expected_normal('h2), // transfer complete
      .expected_error ('h0), // no error
      .error_context  ("transfer complete after autocmd")
    );

    $display("All good");

    $finish();
  end

endmodule
