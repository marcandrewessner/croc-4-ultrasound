// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Micha Wehrli <miwehrli@student.ethz.ch>

`ifndef SDHCI_TOP_DEFINES_SVH_
`define SDHCI_TOP_DEFINES_SVH_

`define writable_reg_t(size) \
  struct packed {            \
    logic size d;            \
    logic de;                \
  }

`define ila(__name, __signal)  \
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [$bits(__signal)-1:0] __name; \
  assign __name = __signal;
`endif
