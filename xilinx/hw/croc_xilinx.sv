// Copyright 2024 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>
// Cyril Koenig <cykoenig@iis.ee.ethz.ch>
// Yann Picod <ypicod@ethz.ch>
// Paul Scheffler <paulsc@iis.ee.ethz.ch>
// Philippe Sauter <phsauter@iis.ee.ethz.ch>

`ifdef TARGET_GENESYS2
  `define USE_RESETN
  `define USE_JTAG_TRSTN
  `define USE_STATUS
  `define USE_SWITCHES
  `define USE_LEDS
  `define USE_FAN
  `define USE_VIO
`endif

`define ILA(__name, __signal)  \
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [$bits(__signal)-1:0] __name; \
  assign __name = __signal;

module croc_xilinx import croc_pkg::*; #(
  localparam int unsigned GpioCount = 4
) (
  input  logic  sys_clk_p,
  input  logic  sys_clk_n,

`ifdef USE_RESET
  input  logic  sys_reset,
`endif
`ifdef USE_RESETN
  input  logic  sys_resetn,
`endif

`ifdef USE_SWITCHES
  input  logic [GpioCount-1:0] gpio_i, // switch 0-3
`endif

`ifdef USE_LEDS
  output logic [GpioCount-1:0] gpio_o,
`endif

`ifdef USE_STATUS
  output logic   status_o,
`endif

  input  logic  jtag_tck_i,
  input  logic  jtag_tms_i,
  input  logic  jtag_tdi_i,
  output logic  jtag_tdo_o,
`ifdef USE_JTAG_TRSTN
  input  logic  jtag_trst_ni,
`endif
`ifdef USE_JTAG_VDDGND
  output logic  jtag_vdd_o,
  output logic  jtag_gnd_o,
`endif

`ifdef USE_FAN
  input  logic [2:0]  fan_sw, // switch 4-6
  output logic        fan_pwm,
`endif

  output logic  uart_tx_o,
  input  logic  uart_rx_i,

  // ADC (Pmod JA) -- only clk and the data MSB are wired so far
  input  logic  adc_clk_i,
  input  logic  adc_data_msb_i,

  // SD Card (Pmod JC, native 4-bit bus). cmd/dat are genuinely bidirectional
  // on the SD bus (host drives during commands/writes, card drives during
  // responses/reads), hence inout here with IOBUFs below.
  output logic       sd_clk_o,
  input  logic       sd_cd_ni,
  inout  wire        sd_cmd_io,
  inout  wire  [3:0] sd_dat_io
);

  ////////////////////////
  //  Clock Generation  //
  ////////////////////////

  wire sys_clk;
  wire soc_clk;

  IBUFDS #(
    .IBUF_LOW_PWR ("FALSE")
  ) i_bufds_sys_clk (
    .I  ( sys_clk_p ),
    .IB ( sys_clk_n ),
    .O  ( sys_clk   )
  );

  clkwiz i_clkwiz (
    .clk_in1  ( sys_clk ),
    .reset    ( '0 ),
    .locked   ( ),
    .clk_soc  ( soc_clk )
  );

  /////////////////////
  //  System Inputs  //
  /////////////////////

  // Select SoC reset
`ifdef USE_RESET
  logic sys_resetn;
  assign sys_resetn = ~sys_reset;
`elsif USE_RESETN
  logic sys_reset;
  assign sys_reset  = ~sys_resetn;
`endif

  // Tie off inputs of no switches
`ifndef USE_SWITCHES
  logic [GpioCount-2:0] gpio_i;
  assign test_mode_i = '0;
  assign gpio_i      = '0;
`endif

`ifndef USE_STATUS
  logic status_o;
`endif

`ifndef USE_LEDS
  logic [GpioCount-1:0] gpio_o;
`endif

  ////////////
  //  VIOs  //
  ////////////
  logic       vio_reset, vio_gpio;

`ifdef USE_VIO
  vio i_vio (
    .clk        ( soc_clk   ),
    .probe_out0 ( vio_reset ),
    .probe_out1 ( vio_gpio  )
  );
`else
  assign vio_reset = '0;
  assign vio_gpio  = '0;
`endif


  //////////////
  //  SOC IO  //
  //////////////

  logic  soc_rst_n;

  assign soc_rst = ~sys_resetn | vio_reset;

  logic [GpioCount-1:0] soc_gpio_i;
  logic [GpioCount-1:0] soc_gpio_o;
  logic [GpioCount-1:0] soc_gpio_out_en_o;

  for(genvar idx=0; idx<GpioCount; idx++) begin : gen_gpio
    assign gpio_o[idx] = soc_gpio_out_en_o[idx] ? soc_gpio_o[idx] : '0;

    if(idx == 0) begin : gen_gpio_0
      assign soc_gpio_i[idx] = ~soc_gpio_out_en_o[idx] ? vio_gpio | gpio_i[0] : '0;
    end else begin : gen_gpio_n
      assign soc_gpio_i[idx] = ~soc_gpio_out_en_o[idx] ? gpio_i[idx] : '0;
    end
  end


  ////////////
  //  ADC   //
  ////////////

  // Uncomment to feed a free-running 14-bit sawtooth counter (0, 1, 2, ...,
  // 16383, 0, 1, ...), incrementing every adc_clk_i edge, into the ADC data
  // bus instead of the real Pmod input -- a known, predictable ramp pattern
  // for exercising the acquisition/SD-card pipeline without a live ADC
  // signal. Comment out to go back to the real (MSB-only-wired) input.
  `define ADC_SAWTOOTH_SIM

  // Layout matches adc_acquisition_pkg::adc_input_signals_t (packed struct,
  // MSB-first): clk[16], rst_n[15], valid[14], data[13:0] (data[13] = MSB).
  // Only clk and the data MSB are physically wired (Pmod JA) so far. valid
  // is tied high (free-running: every adc_clk_i edge is treated as a valid
  // sample, no separate strobe pin needed); the remaining data bits are
  // still tied off until the rest of the ADC bus is wired.
  //
  // dont_touch: with data[12:0] constant, synthesis's optimizer (this repo
  // builds with Flow_PerfOptimized_high) can decide this "mostly constant"
  // path is equivalent to unrelated logic elsewhere (observed: merged into
  // an SDHCI shift register) and fold them together, leaving the port
  // without its own IBUF -- a placement error ("IO port ... does not have
  // an associated IO buffer"). dont_touch stops that merge.
  (* dont_touch = "yes" *) logic [16:0] adc_signals;
  assign adc_signals[16]   = adc_clk_i;
  assign adc_signals[15]   = 1'b1;           // rst_n, tie high until wired
  assign adc_signals[14]   = 1'b1;           // valid, free-running

`ifdef ADC_SAWTOOTH_SIM
  (* dont_touch = "yes" *) logic [13:0] adc_sawtooth_q = '0;
  always_ff @(posedge adc_clk_i) begin
    adc_sawtooth_q <= adc_sawtooth_q + 14'd1;
  end
  assign adc_signals[13:0] = adc_sawtooth_q;
`else
  assign adc_signals[13]   = adc_data_msb_i;
  assign adc_signals[12:0] = '0;             // data[12:0], unwired
`endif


  ///////////////
  //  SD Card  //
  ///////////////

  // Tri-state buffering: croc_soc splits the SD bus's bidirectional cmd/dat
  // lines into separate drive/enable/readback signals (one shared
  // sd_dat_en_o for all 4 DAT lines together, matching the SD protocol where
  // the host drives or releases them as one group, never individually).
  logic       sd_cmd_o, sd_cmd_i, sd_cmd_en_o;
  logic [3:0] sd_dat_o, sd_dat_i;
  logic       sd_dat_en_o;

  assign sd_cmd_io = sd_cmd_en_o ? sd_cmd_o : 1'bz;
  assign sd_cmd_i  = sd_cmd_io;

  assign sd_dat_io = sd_dat_en_o ? sd_dat_o : 4'bzzzz;
  assign sd_dat_i  = sd_dat_io;

  // Debug: CMD17 (read) never budges DAT off idle-high on real hardware even
  // though simulation and the card's own R1 status both look healthy -- see
  // if the pins themselves ever toggle, and whether the host is (still)
  // driving DAT/CMD when it shouldn't be.
  `ILA(dbg_sd_clk_o,     sd_clk_o)
  `ILA(dbg_sd_cmd_i,     sd_cmd_i)
  `ILA(dbg_sd_cmd_en_o,  sd_cmd_en_o)
  `ILA(dbg_sd_dat_i,     sd_dat_i)
  `ILA(dbg_sd_dat_en_o,  sd_dat_en_o)


  //////////////////
  //  Reset Sync  //
  //////////////////

  wire rst_n;

  rstgen i_rstgen (
    .clk_i        ( soc_clk     ),
    .rst_ni       ( ~soc_rst    ),
    .test_mode_i  ( '0          ),
    .rst_no       ( rst_n       ),
    .init_no      ( )
  );

  ////////////
  //  JTAG  //
  ////////////

`ifdef USE_JTAG_VDDGND
  assign jtag_vdd_o = 1'b1;
  assign jtag_gnd_o = 1'b0;
`endif
`ifndef USE_JTAG_TRSTN
  logic jtag_trst_ni;
  assign jtag_trst_ni = 1'b1;
`endif


  /////////////////////////
  // "RTC" Clock Divider //
  /////////////////////////

  logic rtc_clk_d, rtc_clk_q;
  logic [15:0] counter_d, counter_q;

  // Divide soc_clk (50 MHz) by 1525 => ~32.768kHz RTC Clock
  // TODO: does genesys 2 have a 32.768kHz reference clock?
  always_comb begin
    counter_d = counter_q + 1;
    rtc_clk_d = rtc_clk_q;

    if(counter_q == 1525) begin
      counter_d = '0;
      rtc_clk_d = ~rtc_clk_q;
    end
  end

  always_ff @(posedge soc_clk, negedge rst_n) begin
    if(~rst_n) begin
      counter_q <= '0;
      rtc_clk_q <= 0;
    end else begin
      counter_q <= counter_d;
      rtc_clk_q <= rtc_clk_d;
    end
  end

  /////////////////
  // Fan Control //
  /////////////////

`ifdef USE_FAN
  fan_ctrl i_fan_ctrl (
    .clk_i          ( soc_clk      ),
    .rst_ni         ( rst_n        ),
    .pwm_setting_i  ( {'0, fan_sw} ),
    .fan_pwm_o      ( fan_pwm      )
  );
`endif


  //////////////////
  // Cheshire SoC //
  //////////////////
  logic  soc_testmode_i;
  assign soc_testmode_i = '0;

  croc_soc #(
    .GpioCount( GpioCount )
  )
  i_croc_soc (
    .clk_i           ( soc_clk        ),
    .rst_ni          ( rst_n          ),
    .ref_clk_i       ( rtc_clk_q      ),
    .testmode_i      ( soc_testmode_i ),
    .status_o        ( status_o       ),

    .jtag_tck_i      ( jtag_tck_i   ),
    .jtag_tdi_i      ( jtag_tdi_i   ),
    .jtag_tdo_o      ( jtag_tdo_o   ),
    .jtag_tms_i      ( jtag_tms_i   ),
    .jtag_trst_ni    ( jtag_trst_ni ),

    .uart_rx_i       ( uart_rx_i ),
    .uart_tx_o       ( uart_tx_o ),

    .gpio_i          ( soc_gpio_i        ),
    .gpio_o          ( soc_gpio_o        ),
    .gpio_out_en_o   ( soc_gpio_out_en_o ),

    .adc_signals_i   ( adc_signals       ),

    .sd_clk_o        ( sd_clk_o     ),
    .sd_cd_ni        ( sd_cd_ni     ),
    .sd_cmd_en_o     ( sd_cmd_en_o  ),
    .sd_cmd_o        ( sd_cmd_o     ),
    .sd_cmd_i        ( sd_cmd_i     ),
    .sd_dat_i        ( sd_dat_i     ),
    .sd_dat_o        ( sd_dat_o     ),
    .sd_dat_en_o     ( sd_dat_en_o  )
  );

endmodule
