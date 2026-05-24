// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

module tb_cmd_timeout #(
  parameter time         ClkPeriod = 50ns,
  parameter int unsigned RstCycles = 1,
  parameter int unsigned DelayCycles = 4
)();
  sdhci_fixture #(
    .ClkPeriod(ClkPeriod),
    .RstCycles(RstCycles)
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
    // wait for command complete
    fork
      begin
        fork
          begin
            fixture.vip.wait_for_interrupt();
          end
          begin
            repeat(timeout_cycles) fixture.vip.wait_for_sdclk();
            $fatal("Interrupt timed out");
          end
        join_any
        disable fork;
      end
    join
  endtask

  initial begin
    fixture.vip.wait_for_reset();
    fixture.vip.obi.set_interrupt_status_enable(
      // enable command complete
      .normal_interrupt_status_enable('h0001),
      // enable data + command timeout error
      .error_interrupt_status_enable('h0011),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_interrupt_signal_enable(
      // enable command complete
      .normal_interrupt_signal_enable('h0001),
      // enable data + command timeout error
      .error_interrupt_signal_enable('h0011),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_frequency_select(
      .divider(ClkEnPeriod >> 1),
      .finish_transaction(1'b0)
    );
    fixture.vip.obi.set_clock_enable(.enable(1'b1), .finish_transaction(1'b0));
    // prepare identification command
    fixture.vip.obi.set_transfer_mode(
      .is_multi_block(1'b0),
      .is_read(1'b0),
      .auto_cmd12_enable(1'b0),
      .block_count_enable(1'b0),
      .dma_enable(1'b0),
      .finish_transaction(1'b1)
    );

    repeat (DelayCycles) fixture.vip.wait_for_clk();

    fixture.vip.obi.launch_command(
      .command_index(6'd1),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b01), // 136bit
      .finish_transaction(1'b1)
    );

    wfi(400);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (|(error_interrupt_status)) begin
      $fatal("Command timed out");
    end

    fixture.vip.obi.launch_command(
      .command_index(6'd1),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b01), // 136bit
      .finish_transaction(1'b1)
    );

    wfi(400);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0001) begin
      $fatal("Command did not time out");
    end

    wait (response_done);

    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b10), // 48bit no busy
      .finish_transaction(1'b1)
    );

    wfi(400);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );
    
    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (|(error_interrupt_status)) begin
      $fatal("Command timed out");
    end

    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b10), // 48bit no busy
      .finish_transaction(1'b1)
    );

    wfi(400);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    fixture.vip.obi.clear_interrupt_status(
      .normal_interrupt_status(normal_interrupt_status),
      .error_interrupt_status(error_interrupt_status)
    );

    if (error_interrupt_status != 'h0001) begin
      $fatal("Command did not time out");
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
    // the response must come after 5 idle cycles on the bus
    repeat(5) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_136(127'h7F78_7068_6058_5048_4038_3028_2018_1008);

    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // the response must come after 5 idle cycles on the bus
    // so after 6 it must time out
    repeat(6) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_136(127'h7F78_7068_6058_5048_4038_3028_2018_1008);

    response_done = 1'b1;
    fixture.vip.wait_for_sdclk();
    response_done = 1'b0;

    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // the response must come within 64 cycles
    repeat(64) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48(
      .index(6'd0),
      .crc  (7'h0)
    );

    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // the response must come within 64 cycles
    // so after 65 it must time out
    repeat(65) fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48(
      .index(6'd0),
      .crc  (7'h0)
    );
    response_done = 1'b1;
  end

endmodule
