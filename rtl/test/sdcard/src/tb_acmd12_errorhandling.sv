// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>
// - Axel Vanoni <axvanoni@student.ethz.ch>

module tb_acmd12_errorhandling #(
    parameter time         ClkPeriod     = 50ns,
    parameter int unsigned RstCycles     = 1
)();

  int ClkEnPeriod;

  sdhci_fixture #(
    .ClkPeriod(ClkPeriod),
    .RstCycles(RstCycles)
  ) fixture ();

  // < 0 cmd12 gets delayed due to our command
  // = 0 both requests arrive at the same time, cmd12 goes first
  // > 0 our command gets delayed by cmd12
  int CyclesThatDriverCommandArrivesBeforeCMD12;
  int IsFirstResponseValid;

  logic AutoCMD12First;
  logic [15:0] normal_status;
  logic [15:0] error_status;
  logic [15:0] cmd12_error_status;
  logic buf_read_en;
  logic buf_write_en;
  logic response_done;

  initial begin : cmd_response
    response_done = 1'b0;
    fixture.vip.wait_for_reset();

    // response to the request that triggers autocmd12
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_48(0, '0);

    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();
    // bus is idle for 2 cycles
    fixture.vip.wait_for_sdclk();
    if (IsFirstResponseValid) begin
      // valid response, next command should run
      if (AutoCMD12First)
        fixture.vip.sd.send_response_48(12, 'h7A);
      else
        fixture.vip.sd.send_response_48(0, '0);
    end else begin
      // invalid response, next command should not run
      fixture.vip.sd.send_response_48('1, '1);
    end

    if (IsFirstResponseValid) begin
      fixture.vip.sd.wait_for_cmd_held();
      fixture.vip.sd.wait_for_cmd_released();
      // bus is idle for 2 cycles
      fixture.vip.wait_for_sdclk();

      // Valid response for the second command
      if (AutoCMD12First) begin
        fixture.vip.sd.send_response_48(0, '0);
      end else begin
        fixture.vip.sd.send_response_48(12, 'h7A);
      end
    end

    // give time for the status to propagate
    repeat (20) fixture.vip.wait_for_clk();
    response_done = 1'b1;
  end : cmd_response

  initial begin
    fixture.vip.wait_for_reset();
    fixture.vip.sd.wait_for_dat_held();
    fixture.vip.sd.wait_for_dat_released();
    fixture.vip.wait_for_sdclk();
    fixture.vip.sd.send_response_dat(.is_ok(1'b1));
  end

  initial begin
    $timeformat(-9, 0, "ns", 12);
    $dumpfile("tb_acmd12.vcd");
    $dumpvars(0);

    if (!$value$plusargs("CyclesThatDriverCommandArrivesBeforeCMD12=%d", CyclesThatDriverCommandArrivesBeforeCMD12)) begin
      // Keep the default regression point away from the timing boundary; directed
      // sweeps can still override this plusarg to exercise the collision window.
      CyclesThatDriverCommandArrivesBeforeCMD12 = 10;
    end
    if (!$value$plusargs("IsFirstResponseValid=%d", IsFirstResponseValid)) begin
      IsFirstResponseValid = 0;
    end
    if (!$value$plusargs("ClkEnPeriod=%d", ClkEnPeriod)) begin
      ClkEnPeriod = 4;
    end

    AutoCMD12First = CyclesThatDriverCommandArrivesBeforeCMD12 <= 0;

    $display("Testing auto cmd12 with CyclesThatDriverCommandArrivesBeforeCMD12=%d, IsFirstResponseValid=%d, ClkEnPeriod=%d", CyclesThatDriverCommandArrivesBeforeCMD12, IsFirstResponseValid, ClkEnPeriod);

    fixture.vip.wait_for_reset();

    fixture.vip.obi.set_interrupt_status_enable(
      .normal_interrupt_status_enable('hFFFF),
      .error_interrupt_status_enable('hFFFF),
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
      .block_count_enable(1'b1),
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

    fixture.vip.obi.get_present_status_buffer_enable(
      .buffer_read_enable(buf_read_en),
      .buffer_write_enable(buf_write_en)
    );

    while (buf_write_en != 1'b0) begin
      fixture.vip.obi.get_present_status_buffer_enable(
        .buffer_read_enable(buf_read_en),
        .buffer_write_enable(buf_write_en)
      );
    end

    repeat (64 / 4 - 1) begin
      fixture.vip.obi.write_buffer_data(.data('hDEAD_BEEF), .finish_transaction(1'b0));
    end
    fixture.vip.obi.write_buffer_data(.data('hDEAD_BEEF), .finish_transaction(1'b1));

    fixture.vip.obi.get_present_status_buffer_enable(
      .buffer_read_enable(buf_read_en),
      .buffer_write_enable(buf_write_en)
    );

    if (buf_write_en) begin
      // we submitted all the data for the block
      $fatal(1, "Buffer write enable should be off");
    end

    fixture.vip.sd.wait_for_dat_held();
    fixture.vip.sd.wait_for_dat_released();

    // These values are slightly random because of the offset between command start and sd clk period
    case (ClkEnPeriod)
      1: repeat(8 - CyclesThatDriverCommandArrivesBeforeCMD12) fixture.vip.wait_for_clk();
      2: repeat(17 - CyclesThatDriverCommandArrivesBeforeCMD12) fixture.vip.wait_for_clk();
      4: repeat(35 - CyclesThatDriverCommandArrivesBeforeCMD12) fixture.vip.wait_for_clk();
      default: $fatal(1, "ClkEnPeriod not supported");
    endcase

    fixture.vip.obi.launch_command(
      .command_index(6'd0),
      .command_type (2'b00), // normal command
      .data_present (1'b0),
      .index_check_enable(1'b1),
      .crc_check_enable(1'b1),
      .response_type(2'b10) // 48bit no busy
    );

    // wait for first response
    fixture.vip.sd.wait_for_cmd_held();
    fixture.vip.sd.wait_for_cmd_released();

    if (!IsFirstResponseValid) begin
      fork
        begin
          fork
            begin
              fixture.vip.sd.wait_for_cmd_held();
              $fatal("Second command should not have been sent");
            end
            begin
              repeat (80) fixture.vip.wait_for_sdclk();
            end
          join_any
          disable fork;
        end
      join
    end

    // wait for the response to be sent
    wait(response_done);

    fixture.vip.obi.get_interrupt_status(
      .normal_interrupt_status(normal_status),
      .error_interrupt_status(error_status)
    );

    fixture.vip.obi.get_acmd_error_status(cmd12_error_status);

    // leave some time so we get a cleaner waveform
    repeat(10) fixture.vip.wait_for_clk();

    if (IsFirstResponseValid) begin
      if (|(error_status)) begin
        $error("Got an error when we shouldnt have 0x%h", error_status);
        $fatal();
      end
      if (|(cmd12_error_status)) begin
        $error("Got a cmd12 error when we shouldnt have 0x%h", cmd12_error_status);
        $fatal();
      end
    end else begin
      if (AutoCMD12First) begin
        if (error_status != 'b1_0000_0000 /* acmd error */) begin
          $error("Got an error when we shouldnt have 0x%h", error_status);
          $fatal();
        end
        if (cmd12_error_status != /* command not issues by acmd12 error + crc error + index error */ 'b1001_0100) begin
          $error("Got a cmd12 error when we shouldnt have 0x%h", cmd12_error_status);
          $fatal();
        end
      end else begin
        if (error_status != /* acmd error + crc error + index error */ 'b1_0000_1010) begin
          $error("Got an error when we shouldnt have 0x%h", error_status);
          $fatal();
        end
        if (cmd12_error_status != /* command not executed */ 'b1) begin
          $error("Got a cmd12 error when we shouldnt have 0x%h", cmd12_error_status);
          $fatal();
        end
      end
    end

    $dumpflush;
    $finish();
  end

endmodule
