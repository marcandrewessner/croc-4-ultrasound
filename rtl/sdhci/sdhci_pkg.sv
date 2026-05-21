// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@student.ethz.ch>

package sdhci_pkg;
  typedef enum logic [1:0] {
    NO_RESPONSE                   = 2'b00,
    RESPONSE_LENGTH_136           = 2'b01,
    RESPONSE_LENGTH_48            = 2'b10,
    RESPONSE_LENGTH_48_CHECK_BUSY = 2'b11
  } response_type_e;

  typedef logic [5:0]  cmd_t;
  typedef logic [31:0] cmd_arg_t;
endpackage
