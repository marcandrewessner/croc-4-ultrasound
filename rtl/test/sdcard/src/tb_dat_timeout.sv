// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module tb_dat_timeout #(
  parameter time         ClkPeriod = 50ns,
  parameter int unsigned RstCycles = 1,
  parameter int unsigned TimeoutDivider = 13
)();
  sdhci_fixture #(
    .ClkPeriod     (ClkPeriod),
    .RstCycles     (RstCycles),
    .TimeoutDivider(TimeoutDivider)
  ) fixture ();

  int ClkEnPeriod;
  logic [15:0] normal_interrupt_status;
  logic [15:0] error_interrupt_status;
  logic response_done;

  initial begin : configure_tb
    if (!$value$plusargs("ClkEnPeriod=%d", ClkEnPeriod)) begin
      ClkEnPeriod = 4;
    end
    $display("Testing timeouts with ClkEnPeriod=%d", ClkEnPeriod);
  end : configure_tb

  task wfi(input int unsigned timeout_cycles);
    fork
      begin
        fork
          begin
            fixture.vip.wait_for_interrupt();
          end
          begin
            repeat(timeout_cycles) fixture.vip.wait_for_sdclk();
            $fatal(1, "Interrupt timed out");
          end
        join_any
        disable fork;
      end
    join
  endtask

  initial begin
    fixture.vip.wait_for_reset();
    fixture.vip.obi.set_interrupt_status_enable(
      // enable command complete and transaction complete
      .normal_interrupt_status_enable('h0003),
      // enable data + command timeout error
      .error_interrupt_status_enable('h0011),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_interrupt_signal_enable(
      // enable command complete and transaction complete
      .normal_interrupt_signal_enable('h0003),
      // enable data + command timeout error
      .error_interrupt_signal_enable('h0011),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_frequency_select(
      .divider(ClkEnPeriod >> 1),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_data_timeout(.exponent_minus_13(4'b0), .finish_transaction(1'b0));
    fixture.vip.obi.set_clock_enable(.enable(1'b1), .finish_transaction(1'b0));
    // prepare command
    fixture.vip.obi.set_transfer_mode(
      .is_multi_block(1'b0),
      .is_read(1'b0),
      .auto_cmd12_enable(1'b0),
      .block_count_enable(1'b0),
      .dma_enable(1'b0),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b11), // 48bit busy
      .finish_transaction(1'b1)
    );

    wfi(200);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0000) begin
      $fatal(1, "Some error occured during the transaction");
    end

    if (normal_interrupt_status != 'h0001) begin
      $fatal(1, "Interrupt status wrong. Should be command complete ('h1), but got %x", normal_interrupt_status);
    end

    wfi(400);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0000) begin
      $fatal(1, "Some error occured during the transaction");
    end

    if (normal_interrupt_status != 'h0002) begin
      $fatal(1, "Interrupt status wrong. Should be transfer complete ('h2), but got %x", normal_interrupt_status);
    end

    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b11), // 48bit busy
      .finish_transaction(1'b1)
    );

    wfi(200);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0000) begin
      $fatal(1, "Some error occured during the transaction");
    end

    if (normal_interrupt_status != 'h0001) begin
      $fatal(1, "Interrupt status wrong. Should be command complete ('h1), but got %x", normal_interrupt_status);
    end

    // make sure that the timeout does not come earlier
    repeat((TimeoutDivider) * (2 << 12) - 1000 * ClkEnPeriod) fixture.vip.wait_for_clk();
    // if the timeout comes too early, we will have missed it by now
    wfi(2000 * ClkEnPeriod);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0010) begin
      $fatal(1, "Error interrupt status wrong. Should be data timeout ('h10), but got %x", error_interrupt_status);
    end

    if (normal_interrupt_status != 'h8000) begin
      $fatal(1, "Normal interrupt status wrong. Should be error interrupt ('h8000), but got %x", normal_interrupt_status);
    end

    wait (response_done);
    $display("All good");
    $finish();
  end

  initial begin
    response_done = 1'b0;
    fixture.vip.wait_for_reset();
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // the response must come within 64 cycles
    fixture.vip.wait_for_sdclk(); // cycle 1
    fixture.vip.sd.claim_busy();     // cycle 2
    repeat(61) fixture.vip.wait_for_sdclk(); // cycles 3-63
    fixture.vip.sd.send_response_48(
      .index(6'd0),
      .crc  (7'h0)
    );

    repeat(300) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.release_busy();

    response_done = 1'b1;
    fixture.vip.wait_for_clk();
    response_done = 1'b0;

    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // the response must come within 64 cycles
    fixture.vip.wait_for_sdclk(); // cycle 1
    fixture.vip.sd.claim_busy();     // cycle 2
    repeat(61) fixture.vip.wait_for_sdclk(); // cycles 3-63
    fixture.vip.sd.send_response_48(
      .index(6'd0),
      .crc  (7'h0)
    );

    // trigger the timeout
    repeat(TimeoutDivider * (2<<12) + 50) fixture.vip.wait_for_clk();
    response_done = 1'b1;
  end

endmodule
