// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Anton Buchner <abuchner@student.ethz.ch>
// - Micha Wehrli <miwehrli@student.ethz.ch>

`include "common_cells/registers.svh"
`include "defines.svh"

module sd_clk_generator #(
  parameter int unsigned ClkPreDiv = 2
)(
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,

  input  sdhci_reg_pkg::sdhci_reg2hw_t reg2hw_i,

  input  logic pause_sd_clk_i,
  output logic sd_clk_o,

  output logic clk_en_p_o, // high for one clk_i cycle when sd_clk_o rises
  output logic clk_en_n_o, // high for one clk_i cycle when sd_clk_o falls

  output logic div_1_o,   // always low; hardware enforces a minimum divide-by-2 SD clock

  output `writable_reg_t() sd_clk_stable_o
);
  localparam int unsigned DivWidth = 16;
  localparam logic [DivWidth-1:0] MinDiv = {{(DivWidth-2){1'b0}}, 2'b10};

  if (ClkPreDiv == 0) begin : gen_invalid_prediv
    $error("ClkPreDiv must be at least 1");
  end

  function automatic logic freq_sel_valid(input logic [7:0] freq_sel);
    unique case (freq_sel)
      8'h00, 8'h01, 8'h02, 8'h04, 8'h08, 8'h10, 8'h20, 8'h40, 8'h80: begin
        freq_sel_valid = 1'b1;
      end
      default: begin
        freq_sel_valid = 1'b0;
      end
    endcase
  endfunction

  function automatic logic [8:0] decode_sdhci_div(input logic [7:0] freq_sel);
    unique case (freq_sel)
      8'h00: decode_sdhci_div = 9'd1;
      8'h01: decode_sdhci_div = 9'd2;
      8'h02: decode_sdhci_div = 9'd4;
      8'h04: decode_sdhci_div = 9'd8;
      8'h08: decode_sdhci_div = 9'd16;
      8'h10: decode_sdhci_div = 9'd32;
      8'h20: decode_sdhci_div = 9'd64;
      8'h40: decode_sdhci_div = 9'd128;
      8'h80: decode_sdhci_div = 9'd256;
      default: decode_sdhci_div = 9'd1;
    endcase
  endfunction

  logic [31:0] requested_div_full;
  assign requested_div_full = ClkPreDiv * decode_sdhci_div(
      reg2hw_i.clock_control.sdclk_frequency_select.q);

  logic [31:0] effective_div_full;
  // ClkPreDiv is arbitrary, so folding it into the SDHCI divider can produce
  // an odd effective divide. Round odd values up to keep sd_clk_o at 50% duty.
  assign effective_div_full = (requested_div_full < 32'd2) ? 32'd2 :
                              (requested_div_full[0] ? requested_div_full + 32'd1 :
                                                       requested_div_full);

  logic [DivWidth-1:0] requested_div;
  assign requested_div = effective_div_full[DivWidth-1:0];

  logic [DivWidth-1:0] div_d, div_q;
  always_comb begin
    div_d = div_q;
    if (!reg2hw_i.clock_control.sd_clock_enable.q &&
        freq_sel_valid(reg2hw_i.clock_control.sdclk_frequency_select.q) &&
        effective_div_full < (32'd1 << DivWidth)) begin
      div_d = requested_div;
    end
  end
  `FFARNC(div_q, div_d, clear_i, MinDiv, clk_i, rst_ni);

  logic [DivWidth-1:0] cycle_count;
  logic div_ready, div_loaded_q, div_loaded_d;
  always_comb begin
    div_loaded_d = div_loaded_q;
    if (div_d != div_q) begin
      div_loaded_d = 1'b0;
    end else if (div_ready) begin
      div_loaded_d = 1'b1;
    end
  end
  `FFARNC(div_loaded_q, div_loaded_d, clear_i, 1'b0, clk_i, rst_ni);

  logic sd_clk_running;
  assign sd_clk_running = reg2hw_i.clock_control.sd_clock_enable.q && div_loaded_q && !pause_sd_clk_i;

  clk_int_div #(
    .DIV_VALUE_WIDTH       (DivWidth),
    .DEFAULT_DIV_VALUE     (2),
    .ENABLE_CLOCK_IN_RESET (1'b0)
  ) i_clk_int_div (
    .clk_i,
    .rst_ni,
    .en_i          (sd_clk_running),
    .test_mode_en_i(1'b0),
    .div_i         (div_q),
    .div_valid_i   (!div_loaded_q),
    .div_ready_o   (div_ready),
    .clk_o         (),
    .cycl_count_o  (cycle_count)
  );

  logic sd_clk_data_d, sd_clk_data_q, sd_clk_data_prev_q;
  assign sd_clk_data_d = sd_clk_running ? (cycle_count < (div_q >> 1)) : 1'b1;
  `FFARNC(sd_clk_data_q, sd_clk_data_d, clear_i, 1'b1, clk_i, rst_ni);
  `FFARNC(sd_clk_data_prev_q, sd_clk_data_q, clear_i, 1'b1, clk_i, rst_ni);

  assign sd_clk_o    = sd_clk_data_q;
  assign clk_en_p_o  = sd_clk_running && !sd_clk_data_prev_q &&  sd_clk_data_q;
  assign clk_en_n_o  = sd_clk_running &&  sd_clk_data_prev_q && !sd_clk_data_q;
  assign div_1_o     = 1'b0;

  assign sd_clk_stable_o = '{ de: '1, d: reg2hw_i.clock_control.internal_clock_enable.q && div_loaded_q};

`ifndef SYNTHESIS
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (rst_ni && !clear_i && !reg2hw_i.clock_control.sd_clock_enable.q) begin
      assert (freq_sel_valid(reg2hw_i.clock_control.sdclk_frequency_select.q))
        else $error("Unsupported SDHCI SDCLK frequency select value %02h",
                    reg2hw_i.clock_control.sdclk_frequency_select.q);
      assert (effective_div_full >= 32'd2 && !effective_div_full[0])
        else $error("SD clock effective divider must be even and at least 2");
      assert (effective_div_full < (32'd1 << DivWidth))
        else $error("SD clock effective divider %0d exceeds divider width", requested_div_full);
    end
  end
`endif

endmodule
