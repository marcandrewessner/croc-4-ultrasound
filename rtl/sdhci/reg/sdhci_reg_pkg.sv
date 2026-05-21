// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SDHCI register data structures.

package sdhci_reg_pkg;

  // Address widths within the block
  parameter int BlockAw = 8;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    struct packed {
      logic [11:0] q;
      logic        qe;
    } transfer_block_size;
    struct packed {
      logic [2:0]  q;
      logic        qe;
    } host_dma_buffer_boundary;
  } sdhci_reg2hw_block_size_reg_t;

  typedef struct packed {
    logic [15:0] q;
    logic        qe;
  } sdhci_reg2hw_block_count_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } sdhci_reg2hw_argument_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
      logic        qe;
    } block_count_enable;
    struct packed {
      logic        q;
      logic        qe;
    } auto_cmd12_enable;
    struct packed {
      logic        q;
      logic        qe;
    } data_transfer_direction_select;
    struct packed {
      logic        q;
      logic        qe;
    } multi_single_block_select;
  } sdhci_reg2hw_transfer_mode_reg_t;

  typedef struct packed {
    struct packed {
      logic [1:0]  q;
      logic        qe;
    } response_type_select;
    struct packed {
      logic        q;
      logic        qe;
    } command_crc_check_enable;
    struct packed {
      logic        q;
      logic        qe;
    } command_index_check_enable;
    struct packed {
      logic        q;
      logic        qe;
    } data_present_select;
    struct packed {
      logic [1:0]  q;
      logic        qe;
    } command_type;
    struct packed {
      logic [5:0]  q;
      logic        qe;
    } command_index;
  } sdhci_reg2hw_command_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } sdhci_reg2hw_response0_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } sdhci_reg2hw_response1_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } sdhci_reg2hw_response2_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } sdhci_reg2hw_response3_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
    logic        re;
  } sdhci_reg2hw_buffer_data_port_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_inhibit_cmd;
    struct packed {
      logic        q;
    } command_inhibit_dat;
    struct packed {
      logic        q;
    } dat_line_active;
    struct packed {
      logic        q;
    } write_transfer_active;
    struct packed {
      logic        q;
    } read_transfer_active;
    struct packed {
      logic        q;
    } buffer_write_enable;
    struct packed {
      logic        q;
    } buffer_read_enable;
    struct packed {
      logic        q;
    } card_inserted;
    struct packed {
      logic        q;
    } card_state_stable;
    struct packed {
      logic        q;
    } card_detect_pin_level;
    struct packed {
      logic        q;
    } write_protect_switch_pin_level;
    struct packed {
      logic [3:0]  q;
    } dat_line_signal_level;
    struct packed {
      logic        q;
    } cmd_line_signal_level;
  } sdhci_reg2hw_present_state_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } data_transfer_width;
  } sdhci_reg2hw_host_control_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
      logic        qe;
    } internal_clock_enable;
    struct packed {
      logic        q;
      logic        qe;
    } internal_clock_stable;
    struct packed {
      logic        q;
      logic        qe;
    } sd_clock_enable;
    struct packed {
      logic [7:0]  q;
      logic        qe;
    } sdclk_frequency_select;
  } sdhci_reg2hw_clock_control_reg_t;

  typedef struct packed {
    struct packed {
      logic [3:0]  q;
    } data_timeout_counter_value;
  } sdhci_reg2hw_timeout_control_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } software_reset_for_all;
    struct packed {
      logic        q;
    } software_reset_for_cmd_line;
    struct packed {
      logic        q;
    } software_reset_for_dat_line;
  } sdhci_reg2hw_software_reset_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_complete;
    struct packed {
      logic        q;
    } transfer_complete;
    struct packed {
      logic        q;
    } buffer_write_ready;
    struct packed {
      logic        q;
    } buffer_read_ready;
    struct packed {
      logic        q;
    } card_insertion;
    struct packed {
      logic        q;
    } card_removal;
    struct packed {
      logic        q;
    } error_interrupt;
  } sdhci_reg2hw_normal_interrupt_status_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_timeout_error;
    struct packed {
      logic        q;
    } command_crc_error;
    struct packed {
      logic        q;
    } command_end_bit_error;
    struct packed {
      logic        q;
    } command_index_error;
    struct packed {
      logic        q;
    } data_timeout_error;
    struct packed {
      logic        q;
    } data_crc_error;
    struct packed {
      logic        q;
    } data_end_bit_error;
    struct packed {
      logic        q;
    } auto_cmd12_error;
  } sdhci_reg2hw_error_interrupt_status_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_complete_status_enable;
    struct packed {
      logic        q;
    } transfer_complete_status_enable;
    struct packed {
      logic        q;
    } buffer_write_ready_status_enable;
    struct packed {
      logic        q;
    } buffer_read_ready_status_enable;
    struct packed {
      logic        q;
    } card_insertion_status_enable;
    struct packed {
      logic        q;
    } card_removal_status_enable;
    struct packed {
      logic        q;
    } card_interrupt_status_enable;
    struct packed {
      logic        q;
    } fixed_to_0;
  } sdhci_reg2hw_normal_interrupt_status_enable_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_timeout_error_status_enable;
    struct packed {
      logic        q;
    } command_crc_error_status_enable;
    struct packed {
      logic        q;
    } command_end_bit_error_status_enable;
    struct packed {
      logic        q;
    } command_index_error_status_enable;
    struct packed {
      logic        q;
    } data_timeout_error_status_enable;
    struct packed {
      logic        q;
    } data_crc_error_status_enable;
    struct packed {
      logic        q;
    } data_end_bit_error_status_enable;
    struct packed {
      logic        q;
    } auto_cmd12_error_status_enable;
  } sdhci_reg2hw_error_interrupt_status_enable_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_complete_signal_enable;
    struct packed {
      logic        q;
    } transfer_complete_signal_enable;
    struct packed {
      logic        q;
    } buffer_write_ready_signal_enable;
    struct packed {
      logic        q;
    } buffer_read_ready_signal_enable;
    struct packed {
      logic        q;
    } card_insertion_signal_enable;
    struct packed {
      logic        q;
    } card_removal_signal_enable;
    struct packed {
      logic        q;
    } card_interrupt_signal_enable;
  } sdhci_reg2hw_normal_interrupt_signal_enable_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } command_timeout_error_signal_enable;
    struct packed {
      logic        q;
    } command_crc_error_signal_enable;
    struct packed {
      logic        q;
    } command_end_bit_error_signal_enable;
    struct packed {
      logic        q;
    } command_index_error_signal_enable;
    struct packed {
      logic        q;
    } data_timeout_error_signal_enable;
    struct packed {
      logic        q;
    } data_crc_error_signal_enable;
    struct packed {
      logic        q;
    } data_end_bit_error_signal_enable;
    struct packed {
      logic        q;
    } auto_cmd12_error_signal_enable;
  } sdhci_reg2hw_error_interrupt_signal_enable_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } auto_cmd12_not_executed;
    struct packed {
      logic        q;
    } auto_cmd12_timeout_error;
    struct packed {
      logic        q;
    } auto_cmd12_crc_error;
    struct packed {
      logic        q;
    } auto_cmd12_end_bit_error;
    struct packed {
      logic        q;
    } auto_cmd12_index_error;
    struct packed {
      logic        q;
    } command_not_issued_by_auto_cmd12_error;
  } sdhci_reg2hw_auto_cmd12_error_status_reg_t;

  typedef struct packed {
    struct packed {
      logic [11:0] d;
    } transfer_block_size;
    struct packed {
      logic [2:0]  d;
    } host_dma_buffer_boundary;
  } sdhci_hw2reg_block_size_reg_t;

  typedef struct packed {
    logic [15:0] d;
  } sdhci_hw2reg_block_count_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
    } block_count_enable;
    struct packed {
      logic        d;
    } auto_cmd12_enable;
    struct packed {
      logic        d;
    } data_transfer_direction_select;
    struct packed {
      logic        d;
    } multi_single_block_select;
  } sdhci_hw2reg_transfer_mode_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } sdhci_hw2reg_response0_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } sdhci_hw2reg_response1_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } sdhci_hw2reg_response2_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } sdhci_hw2reg_response3_reg_t;

  typedef struct packed {
    logic [31:0] d;
  } sdhci_hw2reg_buffer_data_port_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } command_inhibit_cmd;
    struct packed {
      logic        d;
      logic        de;
    } command_inhibit_dat;
    struct packed {
      logic        d;
      logic        de;
    } dat_line_active;
    struct packed {
      logic        d;
      logic        de;
    } write_transfer_active;
    struct packed {
      logic        d;
      logic        de;
    } read_transfer_active;
    struct packed {
      logic        d;
      logic        de;
    } buffer_write_enable;
    struct packed {
      logic        d;
      logic        de;
    } buffer_read_enable;
    struct packed {
      logic        d;
      logic        de;
    } card_inserted;
    struct packed {
      logic        d;
      logic        de;
    } card_state_stable;
    struct packed {
      logic        d;
      logic        de;
    } card_detect_pin_level;
    struct packed {
      logic        d;
      logic        de;
    } write_protect_switch_pin_level;
    struct packed {
      logic [3:0]  d;
      logic        de;
    } dat_line_signal_level;
    struct packed {
      logic        d;
      logic        de;
    } cmd_line_signal_level;
  } sdhci_hw2reg_present_state_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } internal_clock_stable;
  } sdhci_hw2reg_clock_control_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } software_reset_for_all;
    struct packed {
      logic        d;
      logic        de;
    } software_reset_for_cmd_line;
    struct packed {
      logic        d;
      logic        de;
    } software_reset_for_dat_line;
  } sdhci_hw2reg_software_reset_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } command_complete;
    struct packed {
      logic        d;
      logic        de;
    } transfer_complete;
    struct packed {
      logic        d;
      logic        de;
    } buffer_write_ready;
    struct packed {
      logic        d;
      logic        de;
    } buffer_read_ready;
    struct packed {
      logic        d;
      logic        de;
    } card_insertion;
    struct packed {
      logic        d;
      logic        de;
    } card_removal;
    struct packed {
      logic        d;
      logic        de;
    } error_interrupt;
  } sdhci_hw2reg_normal_interrupt_status_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } command_timeout_error;
    struct packed {
      logic        d;
      logic        de;
    } command_crc_error;
    struct packed {
      logic        d;
      logic        de;
    } command_end_bit_error;
    struct packed {
      logic        d;
      logic        de;
    } command_index_error;
    struct packed {
      logic        d;
      logic        de;
    } data_timeout_error;
    struct packed {
      logic        d;
      logic        de;
    } data_crc_error;
    struct packed {
      logic        d;
      logic        de;
    } data_end_bit_error;
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_error;
  } sdhci_hw2reg_error_interrupt_status_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_not_executed;
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_timeout_error;
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_crc_error;
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_end_bit_error;
    struct packed {
      logic        d;
      logic        de;
    } auto_cmd12_index_error;
    struct packed {
      logic        d;
      logic        de;
    } command_not_issued_by_auto_cmd12_error;
  } sdhci_hw2reg_auto_cmd12_error_status_reg_t;

  typedef struct packed {
    struct packed {
      logic [7:0]  d;
    } interrupt_signal_for_each_slot;
  } sdhci_hw2reg_slot_interrupt_status_reg_t;

  // Register -> HW type
  typedef struct packed {
    sdhci_reg2hw_block_size_reg_t block_size; // [345:329]
    sdhci_reg2hw_block_count_reg_t block_count; // [328:312]
    sdhci_reg2hw_argument_reg_t argument; // [311:280]
    sdhci_reg2hw_transfer_mode_reg_t transfer_mode; // [279:272]
    sdhci_reg2hw_command_reg_t command; // [271:253]
    sdhci_reg2hw_response0_reg_t response0; // [252:221]
    sdhci_reg2hw_response1_reg_t response1; // [220:189]
    sdhci_reg2hw_response2_reg_t response2; // [188:157]
    sdhci_reg2hw_response3_reg_t response3; // [156:125]
    sdhci_reg2hw_buffer_data_port_reg_t buffer_data_port; // [124:91]
    sdhci_reg2hw_present_state_reg_t present_state; // [90:75]
    sdhci_reg2hw_host_control_reg_t host_control; // [74:74]
    sdhci_reg2hw_clock_control_reg_t clock_control; // [73:59]
    sdhci_reg2hw_timeout_control_reg_t timeout_control; // [58:55]
    sdhci_reg2hw_software_reset_reg_t software_reset; // [54:52]
    sdhci_reg2hw_normal_interrupt_status_reg_t normal_interrupt_status; // [51:45]
    sdhci_reg2hw_error_interrupt_status_reg_t error_interrupt_status; // [44:37]
    sdhci_reg2hw_normal_interrupt_status_enable_reg_t normal_interrupt_status_enable; // [36:29]
    sdhci_reg2hw_error_interrupt_status_enable_reg_t error_interrupt_status_enable; // [28:21]
    sdhci_reg2hw_normal_interrupt_signal_enable_reg_t normal_interrupt_signal_enable; // [20:14]
    sdhci_reg2hw_error_interrupt_signal_enable_reg_t error_interrupt_signal_enable; // [13:6]
    sdhci_reg2hw_auto_cmd12_error_status_reg_t auto_cmd12_error_status; // [5:0]
  } sdhci_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    sdhci_hw2reg_block_size_reg_t block_size; // [285:271]
    sdhci_hw2reg_block_count_reg_t block_count; // [270:255]
    sdhci_hw2reg_transfer_mode_reg_t transfer_mode; // [254:251]
    sdhci_hw2reg_response0_reg_t response0; // [250:218]
    sdhci_hw2reg_response1_reg_t response1; // [217:185]
    sdhci_hw2reg_response2_reg_t response2; // [184:152]
    sdhci_hw2reg_response3_reg_t response3; // [151:119]
    sdhci_hw2reg_buffer_data_port_reg_t buffer_data_port; // [118:87]
    sdhci_hw2reg_present_state_reg_t present_state; // [86:58]
    sdhci_hw2reg_clock_control_reg_t clock_control; // [57:56]
    sdhci_hw2reg_software_reset_reg_t software_reset; // [55:50]
    sdhci_hw2reg_normal_interrupt_status_reg_t normal_interrupt_status; // [49:36]
    sdhci_hw2reg_error_interrupt_status_reg_t error_interrupt_status; // [35:20]
    sdhci_hw2reg_auto_cmd12_error_status_reg_t auto_cmd12_error_status; // [19:8]
    sdhci_hw2reg_slot_interrupt_status_reg_t slot_interrupt_status; // [7:0]
  } sdhci_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] SDHCI_SYSTEM_ADDRESS_OFFSET = 8'h 0;
  parameter logic [BlockAw-1:0] SDHCI_BLOCK_SIZE_OFFSET = 8'h 4;
  parameter logic [BlockAw-1:0] SDHCI_BLOCK_COUNT_OFFSET = 8'h 4;
  parameter logic [BlockAw-1:0] SDHCI_ARGUMENT_OFFSET = 8'h 8;
  parameter logic [BlockAw-1:0] SDHCI_TRANSFER_MODE_OFFSET = 8'h c;
  parameter logic [BlockAw-1:0] SDHCI_COMMAND_OFFSET = 8'h c;
  parameter logic [BlockAw-1:0] SDHCI_RESPONSE0_OFFSET = 8'h 10;
  parameter logic [BlockAw-1:0] SDHCI_RESPONSE1_OFFSET = 8'h 14;
  parameter logic [BlockAw-1:0] SDHCI_RESPONSE2_OFFSET = 8'h 18;
  parameter logic [BlockAw-1:0] SDHCI_RESPONSE3_OFFSET = 8'h 1c;
  parameter logic [BlockAw-1:0] SDHCI_BUFFER_DATA_PORT_OFFSET = 8'h 20;
  parameter logic [BlockAw-1:0] SDHCI_PRESENT_STATE_OFFSET = 8'h 24;
  parameter logic [BlockAw-1:0] SDHCI_HOST_CONTROL_OFFSET = 8'h 28;
  parameter logic [BlockAw-1:0] SDHCI_POWER_CONTROL_OFFSET = 8'h 28;
  parameter logic [BlockAw-1:0] SDHCI_BLOCK_GAP_CONTROL_OFFSET = 8'h 28;
  parameter logic [BlockAw-1:0] SDHCI_WAKEUP_CONTROL_OFFSET = 8'h 28;
  parameter logic [BlockAw-1:0] SDHCI_CLOCK_CONTROL_OFFSET = 8'h 2c;
  parameter logic [BlockAw-1:0] SDHCI_TIMEOUT_CONTROL_OFFSET = 8'h 2c;
  parameter logic [BlockAw-1:0] SDHCI_SOFTWARE_RESET_OFFSET = 8'h 2c;
  parameter logic [BlockAw-1:0] SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET = 8'h 30;
  parameter logic [BlockAw-1:0] SDHCI_ERROR_INTERRUPT_STATUS_OFFSET = 8'h 30;
  parameter logic [BlockAw-1:0] SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE_OFFSET = 8'h 34;
  parameter logic [BlockAw-1:0] SDHCI_ERROR_INTERRUPT_STATUS_ENABLE_OFFSET = 8'h 34;
  parameter logic [BlockAw-1:0] SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE_OFFSET = 8'h 38;
  parameter logic [BlockAw-1:0] SDHCI_ERROR_INTERRUPT_SIGNAL_ENABLE_OFFSET = 8'h 38;
  parameter logic [BlockAw-1:0] SDHCI_AUTO_CMD12_ERROR_STATUS_OFFSET = 8'h 3c;
  parameter logic [BlockAw-1:0] SDHCI_CAPABILITIES_OFFSET = 8'h 40;
  parameter logic [BlockAw-1:0] SDHCI_CAPABILITIES_RESERVED_OFFSET = 8'h 44;
  parameter logic [BlockAw-1:0] SDHCI_MAXIMUM_CURRENT_CAPABILITIES_OFFSET = 8'h 48;
  parameter logic [BlockAw-1:0] SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED_OFFSET = 8'h 4c;
  parameter logic [BlockAw-1:0] SDHCI_SLOT_INTERRUPT_STATUS_OFFSET = 8'h fc;
  parameter logic [BlockAw-1:0] SDHCI_HOST_CONTROLLER_VERSION_OFFSET = 8'h fc;

  // Reset values for hwext registers and their fields
  parameter logic [15:0] SDHCI_BLOCK_SIZE_RESVAL = 16'h 0;
  parameter logic [0:0] SDHCI_BLOCK_SIZE_RSVD_15_RESVAL = 1'h 0;
  parameter logic [31:0] SDHCI_BLOCK_COUNT_RESVAL = 32'h 0;
  parameter logic [15:0] SDHCI_TRANSFER_MODE_RESVAL = 16'h 0;
  parameter logic [0:0] SDHCI_TRANSFER_MODE_DMA_ENABLE_RESVAL = 1'h 0;
  parameter logic [0:0] SDHCI_TRANSFER_MODE_RSVD_3_RESVAL = 1'h 0;
  parameter logic [1:0] SDHCI_TRANSFER_MODE_RSVD_6_RESVAL = 2'h 0;
  parameter logic [7:0] SDHCI_TRANSFER_MODE_RSVD_8_RESVAL = 8'h 0;
  parameter logic [31:0] SDHCI_BUFFER_DATA_PORT_RESVAL = 32'h 0;
  parameter logic [15:0] SDHCI_SLOT_INTERRUPT_STATUS_RESVAL = 16'h 0;
  parameter logic [7:0] SDHCI_SLOT_INTERRUPT_STATUS_INTERRUPT_SIGNAL_FOR_EACH_SLOT_RESVAL = 8'h 0;
  parameter logic [7:0] SDHCI_SLOT_INTERRUPT_STATUS_RSVD_8_RESVAL = 8'h 0;

  // Register index
  typedef enum int {
    SDHCI_SYSTEM_ADDRESS,
    SDHCI_BLOCK_SIZE,
    SDHCI_BLOCK_COUNT,
    SDHCI_ARGUMENT,
    SDHCI_TRANSFER_MODE,
    SDHCI_COMMAND,
    SDHCI_RESPONSE0,
    SDHCI_RESPONSE1,
    SDHCI_RESPONSE2,
    SDHCI_RESPONSE3,
    SDHCI_BUFFER_DATA_PORT,
    SDHCI_PRESENT_STATE,
    SDHCI_HOST_CONTROL,
    SDHCI_POWER_CONTROL,
    SDHCI_BLOCK_GAP_CONTROL,
    SDHCI_WAKEUP_CONTROL,
    SDHCI_CLOCK_CONTROL,
    SDHCI_TIMEOUT_CONTROL,
    SDHCI_SOFTWARE_RESET,
    SDHCI_NORMAL_INTERRUPT_STATUS,
    SDHCI_ERROR_INTERRUPT_STATUS,
    SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE,
    SDHCI_ERROR_INTERRUPT_STATUS_ENABLE,
    SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE,
    SDHCI_ERROR_INTERRUPT_SIGNAL_ENABLE,
    SDHCI_AUTO_CMD12_ERROR_STATUS,
    SDHCI_CAPABILITIES,
    SDHCI_CAPABILITIES_RESERVED,
    SDHCI_MAXIMUM_CURRENT_CAPABILITIES,
    SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED,
    SDHCI_SLOT_INTERRUPT_STATUS,
    SDHCI_HOST_CONTROLLER_VERSION
  } sdhci_id_e;

  // Register bytemaks used to see if a register is to be written to 
  parameter logic [3:0] SDHCI_BYTEMASK [32] = '{
    4'b 1111, // index[ 0] SDHCI_SYSTEM_ADDRESS
    4'b 0011, // index[ 1] SDHCI_BLOCK_SIZE
    4'b 1100, // index[ 2] SDHCI_BLOCK_COUNT
    4'b 1111, // index[ 3] SDHCI_ARGUMENT
    4'b 0011, // index[ 4] SDHCI_TRANSFER_MODE
    4'b 1100, // index[ 5] SDHCI_COMMAND
    4'b 1111, // index[ 6] SDHCI_RESPONSE0
    4'b 1111, // index[ 7] SDHCI_RESPONSE1
    4'b 1111, // index[ 8] SDHCI_RESPONSE2
    4'b 1111, // index[ 9] SDHCI_RESPONSE3
    4'b 1111, // index[10] SDHCI_BUFFER_DATA_PORT
    4'b 1111, // index[11] SDHCI_PRESENT_STATE
    4'b 0001, // index[12] SDHCI_HOST_CONTROL
    4'b 0010, // index[13] SDHCI_POWER_CONTROL
    4'b 0100, // index[14] SDHCI_BLOCK_GAP_CONTROL
    4'b 1000, // index[15] SDHCI_WAKEUP_CONTROL
    4'b 0011, // index[16] SDHCI_CLOCK_CONTROL
    4'b 0100, // index[17] SDHCI_TIMEOUT_CONTROL
    4'b 1000, // index[18] SDHCI_SOFTWARE_RESET
    4'b 0011, // index[19] SDHCI_NORMAL_INTERRUPT_STATUS
    4'b 1100, // index[20] SDHCI_ERROR_INTERRUPT_STATUS
    4'b 0011, // index[21] SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE
    4'b 1100, // index[22] SDHCI_ERROR_INTERRUPT_STATUS_ENABLE
    4'b 0011, // index[23] SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE
    4'b 1100, // index[24] SDHCI_ERROR_INTERRUPT_SIGNAL_ENABLE
    4'b 0011, // index[25] SDHCI_AUTO_CMD12_ERROR_STATUS
    4'b 1111, // index[26] SDHCI_CAPABILITIES
    4'b 1111, // index[27] SDHCI_CAPABILITIES_RESERVED
    4'b 1111, // index[28] SDHCI_MAXIMUM_CURRENT_CAPABILITIES
    4'b 1111, // index[29] SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED
    4'b 0011, // index[30] SDHCI_SLOT_INTERRUPT_STATUS
    4'b 1100  // index[31] SDHCI_HOST_CONTROLLER_VERSION
  };

  // Register boudary crossing infromation to make sure we don't write to half of a field
  parameter logic [2:0] SDHCI_DISALLOWED_BOUNDARY_CROSSINGS [32] = '{
    3'b 111, // index[ 0] SDHCI_SYSTEM_ADDRESS
    3'b 001, // index[ 1] SDHCI_BLOCK_SIZE
    3'b 100, // index[ 2] SDHCI_BLOCK_COUNT
    3'b 111, // index[ 3] SDHCI_ARGUMENT
    3'b 000, // index[ 4] SDHCI_TRANSFER_MODE
    3'b 000, // index[ 5] SDHCI_COMMAND
    3'b 111, // index[ 6] SDHCI_RESPONSE0
    3'b 111, // index[ 7] SDHCI_RESPONSE1
    3'b 111, // index[ 8] SDHCI_RESPONSE2
    3'b 111, // index[ 9] SDHCI_RESPONSE3
    3'b 111, // index[10] SDHCI_BUFFER_DATA_PORT
    3'b 000, // index[11] SDHCI_PRESENT_STATE
    3'b 000, // index[12] SDHCI_HOST_CONTROL
    3'b 000, // index[13] SDHCI_POWER_CONTROL
    3'b 000, // index[14] SDHCI_BLOCK_GAP_CONTROL
    3'b 000, // index[15] SDHCI_WAKEUP_CONTROL
    3'b 000, // index[16] SDHCI_CLOCK_CONTROL
    3'b 000, // index[17] SDHCI_TIMEOUT_CONTROL
    3'b 000, // index[18] SDHCI_SOFTWARE_RESET
    3'b 000, // index[19] SDHCI_NORMAL_INTERRUPT_STATUS
    3'b 000, // index[20] SDHCI_ERROR_INTERRUPT_STATUS
    3'b 000, // index[21] SDHCI_NORMAL_INTERRUPT_STATUS_ENABLE
    3'b 000, // index[22] SDHCI_ERROR_INTERRUPT_STATUS_ENABLE
    3'b 000, // index[23] SDHCI_NORMAL_INTERRUPT_SIGNAL_ENABLE
    3'b 000, // index[24] SDHCI_ERROR_INTERRUPT_SIGNAL_ENABLE
    3'b 000, // index[25] SDHCI_AUTO_CMD12_ERROR_STATUS
    3'b 000, // index[26] SDHCI_CAPABILITIES
    3'b 111, // index[27] SDHCI_CAPABILITIES_RESERVED
    3'b 000, // index[28] SDHCI_MAXIMUM_CURRENT_CAPABILITIES
    3'b 111, // index[29] SDHCI_MAXIMUM_CURRENT_CAPABILITIES_RESERVED
    3'b 000, // index[30] SDHCI_SLOT_INTERRUPT_STATUS
    3'b 000  // index[31] SDHCI_HOST_CONTROLLER_VERSION
  };

endpackage
