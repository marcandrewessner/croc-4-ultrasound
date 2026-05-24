// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>

module tb_dat #(
    parameter time         ClkPeriod     = 50ns,
    parameter int unsigned RstCycles     = 1
  )();
  localparam int unsigned TestClkPreDiv = 2;

  function automatic logic [7:0] encode_frequency_select(input int unsigned min_effective_div);
    int unsigned sdhci_div;
    int unsigned effective_div;
    sdhci_div = 1;
    effective_div = TestClkPreDiv;
    if (effective_div < 2) begin
      effective_div = 2;
    end
    while (effective_div < min_effective_div && sdhci_div < 256) begin
      sdhci_div <<= 1;
      effective_div = TestClkPreDiv * sdhci_div;
      if (effective_div[0]) begin
        effective_div++;
      end
    end
    return (sdhci_div == 1) ? 8'h00 : 8'(sdhci_div >> 1);
  endfunction

  logic clk;
  logic rst_n;

  logic fetch_en_i;
  logic status_o;

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriod ),
    .RstClkCycles ( RstCycles )
  ) i_clk_rst_sys (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );

  sdhci_reg_pkg::sdhci_reg2hw_t reg2hw_i;


  logic sd_clk, sd_clk_en_p, sd_clk_en_n, div_1;
  sd_clk_generator sd_clk_div (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .clear_i(1'b0),
    
    .reg2hw_i,

    .pause_sd_clk_i ('0),
    .sd_clk_o       (sd_clk),

    .clk_en_p_o (sd_clk_en_p),
    .clk_en_n_o (sd_clk_en_n),
    .div_1_o    (div_1),

    .sd_clk_stable_o ()
  );

  tri1 [3:0] dat;

  logic [31:0] data_i;
  logic start_write;
  logic next_word_o, done_write, dat_en_o;
  logic [3:0] dat_o;
  assign dat = dat_en_o ? dat_o : 'z;

  localparam int MaxBlockBitSize = 14;

  logic UseWideBus;
  logic [MaxBlockBitSize-1:0] BlockSize;

  dat_write #(
    .MaxBlockBitSize (MaxBlockBitSize)
  ) i_dat_write (
    .clk_i (clk),
    .sd_clk_en_p_i (sd_clk_en_p),
    .sd_clk_en_n_i (sd_clk_en_n),
    .div_1_i       (div_1),
    .rst_ni (rst_n),
    .clear_i(1'b0),

    .start_i (start_write),
    .block_size_i (MaxBlockBitSize'(BlockSize)),
    .bus_width_is_4_i (UseWideBus),

    .data_i,
    .next_word_o,

    .done_o        (done_write),
    .crc_err_o     (),
    .end_bit_err_o (),
    .data_timeout_o(),
    .waiting_o     (),

    .dat0_i (dat[0]),
    .dat_o,
    .dat_en_o
  );

  logic start_read, done_read, data_valid_o, done_o, crc_err_o, end_bit_err_o;
  logic [31:0] data_o;
  dat_read #(
    .MaxBlockBitSize (MaxBlockBitSize)
  ) i_dat_read (
    .clk_i (clk),
    .sd_clk_en_i (sd_clk_en_p),
    .rst_ni (rst_n),
    .clear_i(1'b0),
    .dat_i (dat),

    .start_i (start_read),
    .timeout_i (1'b0),
    .block_size_i (MaxBlockBitSize'(BlockSize)),
    .bus_width_is_4_i (UseWideBus),

    .data_valid_o,
    .data_o,

    .done_o (done_read),
    .crc_err_o,
    .end_bit_err_o,
    .waiting_o ()
  );

  /////////////////
  //  Testbench  //
  /////////////////

  always begin
    if (done_read && crc_err_o) begin
      $error("CRC Error");
      @(negedge clk)
      $fatal();
    end
    if (done_read && end_bit_err_o) begin
      $error("End Bit Error");
      @(negedge clk)
      $fatal();
    end
    @(negedge clk);
  end


  logic [31:0] entries [$];
  int ClkEnPeriod, remainingBlocks;
  int watchdog_cycles;
  logic [31:0] got, want;
  initial begin
    $timeformat(-9, 0, "ns", 12);
    if ($test$plusargs("vcd")) begin
      $dumpfile("tb_dat.vcd");
      $dumpvars(0);
    end

    if (!$value$plusargs("UseWideBus=%d", UseWideBus)) begin
      UseWideBus = 1;
    end
    if (!$value$plusargs("ClkEnPeriod=%d", ClkEnPeriod)) begin
      ClkEnPeriod = 3;
    end
    if (!$value$plusargs("BlockSize=%d", BlockSize)) begin
      BlockSize = 128;
    end

    reg2hw_i = '0;
    reg2hw_i.clock_control.sd_clock_enable.q = 1;
    reg2hw_i.clock_control.sdclk_frequency_select.q = encode_frequency_select(ClkEnPeriod);

    $display("Testing dat line with UseWideBus=%d, BlockSize=%d, ClkEnPeriod=%d", UseWideBus, BlockSize, ClkEnPeriod);

    #ClkPeriod;

    @(negedge sd_clk);

    start_write = '1;
    start_read = '1;

    data_i = $urandom();
    // $display("data_i = %h", data_i);
    entries.push_back(data_i);

    @(negedge sd_clk);

    start_write = '0;
    start_read = '0;

    remainingBlocks = BlockSize;
    watchdog_cycles = 0;
    while (!done_read)
    begin

      if (next_word_o)
      begin
        data_i = $urandom();
        // $display("data_i = %h", data_i);
        entries.push_back(data_i);
      end
      if (data_valid_o) begin
        // $display("data_o = %h", data_o);
        got = data_o;
        want = entries[0];
        unique case (remainingBlocks % 4)
          0: ;
          1: begin
            got[31:8] = '0;
            want[31:8] = '0;
          end
          2: begin
            got[31:15] = '0;
            want[31:15] = '0;
          end
          3: begin
            got[31:24] = '0;
            want[31:24] = '0;
          end
        endcase
        if (got != want) begin
          $display("Got %h, wanted %h", got, want);
          @(negedge sd_clk)
          @(negedge sd_clk)
          @(negedge sd_clk)
          $fatal();
        end

        void'(entries.pop_front());
        remainingBlocks -= 4;
      end

      @(negedge sd_clk);
      @(posedge clk);
      #1ns;
      watchdog_cycles++;
      if (watchdog_cycles > (BlockSize * (UseWideBus ? 16 : 64) + 1000)) begin
        $fatal(1, "DAT transfer timed out");
      end
    end

    repeat(50) @(posedge clk);

    if ($test$plusargs("vcd")) begin
      $dumpflush;
    end
    $finish();
  end

endmodule
