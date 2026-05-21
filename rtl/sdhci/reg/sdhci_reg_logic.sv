// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>

`include "common_cells/registers.svh"
`include "defines.svh"

module sdhci_reg_logic (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,
  input  logic clear_cmd_i,
  input  logic clear_dat_i,

  input sdhci_reg_pkg::sdhci_reg2hw_t reg2hw_i,
  input sdhci_reg_pkg::sdhci_hw2reg_t hw2reg_i,

  output sdhci_reg_pkg::sdhci_reg2hw_t reg2hw_modified_o,

  input logic sd_cmd_dat_busy_i,

  output `writable_reg_t() error_interrupt_o,
  output `writable_reg_t() auto_cmd12_error_o,

  output `writable_reg_t() buffer_read_ready_o,
  output `writable_reg_t() buffer_write_ready_o,

  output `writable_reg_t() dat_line_active_o,
  output `writable_reg_t() command_inhibit_dat_o,

  output `writable_reg_t() transfer_complete_o,
  output `writable_reg_t() command_complete_o,

  output `writable_reg_t() card_removal_o,
  output `writable_reg_t() card_insertion_o,

  output logic [15:0]            block_count_o,
  input  `writable_reg_t([15:0]) block_count_hw_i,

  output sdhci_reg_pkg::sdhci_hw2reg_block_size_reg_t    block_size_reg_o,
  output sdhci_reg_pkg::sdhci_hw2reg_transfer_mode_reg_t transfer_mode_reg_o,

  output logic [7:0] interrupt_signal_for_each_slot_o,
  output logic interrupt_o
);
  logic [5:0] normal_interrupt_sources;
  logic [7:0] error_interrupt_sources;
  logic [7:0] visible_error_status;
  logic [5:0] auto_cmd12_error_events;

  assign normal_interrupt_sources = {
    reg2hw_i.normal_interrupt_status.card_removal.q &
        reg2hw_i.normal_interrupt_signal_enable.card_removal_signal_enable.q,
    reg2hw_i.normal_interrupt_status.card_insertion.q &
        reg2hw_i.normal_interrupt_signal_enable.card_insertion_signal_enable.q,
    reg2hw_i.normal_interrupt_status.buffer_read_ready.q &
        reg2hw_i.normal_interrupt_signal_enable.buffer_read_ready_signal_enable.q,
    reg2hw_i.normal_interrupt_status.buffer_write_ready.q &
        reg2hw_i.normal_interrupt_signal_enable.buffer_write_ready_signal_enable.q,
    reg2hw_i.normal_interrupt_status.transfer_complete.q &
        reg2hw_i.normal_interrupt_signal_enable.transfer_complete_signal_enable.q,
    reg2hw_i.normal_interrupt_status.command_complete.q &
        reg2hw_i.normal_interrupt_signal_enable.command_complete_signal_enable.q
  };

  assign error_interrupt_sources = {
    reg2hw_i.error_interrupt_status.auto_cmd12_error.q &
        reg2hw_i.error_interrupt_signal_enable.auto_cmd12_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.data_end_bit_error.q &
        reg2hw_i.error_interrupt_signal_enable.data_end_bit_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.data_crc_error.q &
        reg2hw_i.error_interrupt_signal_enable.data_crc_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.data_timeout_error.q &
        reg2hw_i.error_interrupt_signal_enable.data_timeout_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.command_index_error.q &
        reg2hw_i.error_interrupt_signal_enable.command_index_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.command_end_bit_error.q &
        reg2hw_i.error_interrupt_signal_enable.command_end_bit_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.command_crc_error.q &
        reg2hw_i.error_interrupt_signal_enable.command_crc_error_signal_enable.q,
    reg2hw_i.error_interrupt_status.command_timeout_error.q &
        reg2hw_i.error_interrupt_signal_enable.command_timeout_error_signal_enable.q
  };

  assign visible_error_status = {
    hw2reg_i.error_interrupt_status.auto_cmd12_error.de ?
        hw2reg_i.error_interrupt_status.auto_cmd12_error.d :
        reg2hw_i.error_interrupt_status.auto_cmd12_error.q,
    hw2reg_i.error_interrupt_status.data_end_bit_error.de ?
        hw2reg_i.error_interrupt_status.data_end_bit_error.d :
        reg2hw_i.error_interrupt_status.data_end_bit_error.q,
    hw2reg_i.error_interrupt_status.data_crc_error.de ?
        hw2reg_i.error_interrupt_status.data_crc_error.d :
        reg2hw_i.error_interrupt_status.data_crc_error.q,
    hw2reg_i.error_interrupt_status.data_timeout_error.de ?
        hw2reg_i.error_interrupt_status.data_timeout_error.d :
        reg2hw_i.error_interrupt_status.data_timeout_error.q,
    hw2reg_i.error_interrupt_status.command_index_error.de ?
        hw2reg_i.error_interrupt_status.command_index_error.d :
        reg2hw_i.error_interrupt_status.command_index_error.q,
    hw2reg_i.error_interrupt_status.command_end_bit_error.de ?
        hw2reg_i.error_interrupt_status.command_end_bit_error.d :
        reg2hw_i.error_interrupt_status.command_end_bit_error.q,
    hw2reg_i.error_interrupt_status.command_crc_error.de ?
        hw2reg_i.error_interrupt_status.command_crc_error.d :
        reg2hw_i.error_interrupt_status.command_crc_error.q,
    hw2reg_i.error_interrupt_status.command_timeout_error.de ?
        hw2reg_i.error_interrupt_status.command_timeout_error.d :
        reg2hw_i.error_interrupt_status.command_timeout_error.q
  };

  assign auto_cmd12_error_events = {
    hw2reg_i.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.de &
        ~reg2hw_i.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.q &
        hw2reg_i.auto_cmd12_error_status.command_not_issued_by_auto_cmd12_error.d,
    hw2reg_i.auto_cmd12_error_status.auto_cmd12_index_error.de &
        ~reg2hw_i.auto_cmd12_error_status.auto_cmd12_index_error.q &
        hw2reg_i.auto_cmd12_error_status.auto_cmd12_index_error.d,
    hw2reg_i.auto_cmd12_error_status.auto_cmd12_end_bit_error.de &
        ~reg2hw_i.auto_cmd12_error_status.auto_cmd12_end_bit_error.q &
        hw2reg_i.auto_cmd12_error_status.auto_cmd12_end_bit_error.d,
    hw2reg_i.auto_cmd12_error_status.auto_cmd12_crc_error.de &
        ~reg2hw_i.auto_cmd12_error_status.auto_cmd12_crc_error.q &
        hw2reg_i.auto_cmd12_error_status.auto_cmd12_crc_error.d,
    hw2reg_i.auto_cmd12_error_status.auto_cmd12_timeout_error.de &
        ~reg2hw_i.auto_cmd12_error_status.auto_cmd12_timeout_error.q &
        hw2reg_i.auto_cmd12_error_status.auto_cmd12_timeout_error.d,
    hw2reg_i.auto_cmd12_error_status.auto_cmd12_not_executed.de &
        ~reg2hw_i.auto_cmd12_error_status.auto_cmd12_not_executed.q &
        hw2reg_i.auto_cmd12_error_status.auto_cmd12_not_executed.d
  };
    
  assign interrupt_signal_for_each_slot_o[7:1] = '0;
  assign interrupt_signal_for_each_slot_o[0] =
      (|normal_interrupt_sources) | (|error_interrupt_sources);

  // Send interrupt if any interrupt status went from 0 to 1
  assign interrupt_o = interrupt_signal_for_each_slot_o[0];

  // Automatically write to Error Interrupt Status
  assign error_interrupt_o.d = !clear_i & (|visible_error_status);
  assign error_interrupt_o.de = '1;

  // Automatically write to AutoCMD12 Error Interrupt Status
  assign auto_cmd12_error_o.d = '1;
  assign auto_cmd12_error_o.de = !clear_i &
    reg2hw_i.error_interrupt_status_enable.auto_cmd12_error_status_enable.q &
    (|auto_cmd12_error_events);

  assign buffer_read_ready_o.d = '1;
  assign buffer_read_ready_o.de = !clear_dat_i &
    hw2reg_i.present_state.buffer_read_enable.de &
    ~reg2hw_i.present_state.buffer_read_enable.q &
    hw2reg_i.present_state.buffer_read_enable.d;

  assign buffer_write_ready_o.d = '1;
  assign buffer_write_ready_o.de = !clear_dat_i &
    hw2reg_i.present_state.buffer_write_enable.de &
    ~reg2hw_i.present_state.buffer_write_enable.q &
    hw2reg_i.present_state.buffer_write_enable.d;


  // technically, dat_line_active should be 0 once the last block of a read
  // transfer has been transferred into the buffer, at which point
  // read_transfer_active is still 1
  assign dat_line_active_o.de = '1;
  assign dat_line_active_o.d = !clear_dat_i & (sd_cmd_dat_busy_i |
    (hw2reg_i.present_state.write_transfer_active.de ?
        hw2reg_i.present_state.write_transfer_active.d :
        reg2hw_i.present_state.write_transfer_active.q) |
    (hw2reg_i.present_state.read_transfer_active.de ?
        hw2reg_i.present_state.read_transfer_active.d :
        reg2hw_i.present_state.read_transfer_active.q));

  // technically, command_inhibit_dat should be
  // dat_line_active | read_transfer_active, but as we or read_transfer_active
  // already into dat_line_active, this should be fine
  assign command_inhibit_dat_o.de = '1;
  assign command_inhibit_dat_o.d = !clear_dat_i &
    (hw2reg_i.present_state.dat_line_active.de ?
        hw2reg_i.present_state.dat_line_active.d :
        reg2hw_i.present_state.dat_line_active.q);

  // transfer complete fires on:
  // - read transfer active  1->0
  // - write transfer active 1->0
  // - dat line active       1->0
  // - command inhibit dat   1->0
  // command inhibit dat = (r/w tx | dat line),
  // write active implies dat line active,
  // so looking at command inhibit is enough
  assign transfer_complete_o.d = '1;
  assign transfer_complete_o.de = !clear_dat_i &
    hw2reg_i.present_state.command_inhibit_dat.de &
    reg2hw_i.present_state.command_inhibit_dat.q &
    ~hw2reg_i.present_state.command_inhibit_dat.d;

  assign command_complete_o.d = '1;
  assign command_complete_o.de = !clear_cmd_i &
    hw2reg_i.present_state.command_inhibit_cmd.de &
    reg2hw_i.present_state.command_inhibit_cmd.q &
    ~hw2reg_i.present_state.command_inhibit_cmd.d;


  assign card_insertion_o.d = '1;
  assign card_insertion_o.de = !clear_dat_i &
    hw2reg_i.present_state.card_inserted.de &
    ~reg2hw_i.present_state.card_inserted.q &
    hw2reg_i.present_state.card_inserted.d;

  assign card_removal_o.d = '1;
  assign card_removal_o.de = !clear_dat_i &
    hw2reg_i.present_state.card_inserted.de &
    reg2hw_i.present_state.card_inserted.q &
    ~hw2reg_i.present_state.card_inserted.d;
  
  // Writes to the transfer_mode register should be ignored when command_inhibit_cmd is active
  `FFLARNC (transfer_mode_reg_o.multi_single_block_select     .d, reg2hw_i.transfer_mode.multi_single_block_select     .q,
        !reg2hw_i.present_state.command_inhibit_cmd.q && reg2hw_i.transfer_mode.multi_single_block_select     .qe, clear_i, '0, clk_i, rst_ni)
  `FFLARNC (transfer_mode_reg_o.data_transfer_direction_select.d, reg2hw_i.transfer_mode.data_transfer_direction_select.q,
        !reg2hw_i.present_state.command_inhibit_cmd.q && reg2hw_i.transfer_mode.data_transfer_direction_select.qe, clear_i, '0, clk_i, rst_ni)
  `FFLARNC (transfer_mode_reg_o.auto_cmd12_enable             .d, reg2hw_i.transfer_mode.auto_cmd12_enable             .q,
        !reg2hw_i.present_state.command_inhibit_cmd.q && reg2hw_i.transfer_mode.auto_cmd12_enable             .qe, clear_i, '0, clk_i, rst_ni)
  `FFLARNC (transfer_mode_reg_o.block_count_enable            .d, reg2hw_i.transfer_mode.block_count_enable            .q,
        !reg2hw_i.present_state.command_inhibit_cmd.q && reg2hw_i.transfer_mode.block_count_enable            .qe, clear_i, '0, clk_i, rst_ni)
  // dma_enable is read-only zero in the register file (DMA not implemented), no shadow needed

  // Writes to the block_count and block_size register should be ignored when command_inhibit_dat is active
  logic [11:0] block_size;
  `FFLARNC (block_size, reg2hw_i.block_size.transfer_block_size.q,
        !reg2hw_i.present_state.command_inhibit_dat.q && reg2hw_i.block_size.transfer_block_size.qe, clear_i, '0, clk_i, rst_ni);

  assign block_size_reg_o.transfer_block_size.d = block_size;
  assign block_size_reg_o.host_dma_buffer_boundary.d = '0;

  logic [15:0] block_count_q, block_count_d;
  `FF (block_count_q, block_count_d, '0);
  always_comb begin
    block_count_d = block_count_q;   
    if (clear_dat_i) begin
      block_count_d = '0;
    end else if (!reg2hw_i.present_state.command_inhibit_dat.q && reg2hw_i.block_count.qe) begin
      block_count_d = reg2hw_i.block_count.q;
    end else if (block_count_hw_i.de) begin
      block_count_d = block_count_hw_i.d;   
    end
  end
  assign block_count_o = block_count_q;

  always_comb begin
    reg2hw_modified_o = reg2hw_i;   

    reg2hw_modified_o.transfer_mode.multi_single_block_select     .q = transfer_mode_reg_o.multi_single_block_select     .d;
    reg2hw_modified_o.transfer_mode.data_transfer_direction_select.q = transfer_mode_reg_o.data_transfer_direction_select.d;
    reg2hw_modified_o.transfer_mode.auto_cmd12_enable             .q = transfer_mode_reg_o.auto_cmd12_enable             .d;
    reg2hw_modified_o.transfer_mode.block_count_enable            .q = transfer_mode_reg_o.block_count_enable            .d;
    // dma_enable removed from struct (read-only zero in register file)

    reg2hw_modified_o.block_size.transfer_block_size.q = block_size_reg_o.transfer_block_size.d;

    reg2hw_modified_o.block_count.q = block_count_o;
  end
endmodule
