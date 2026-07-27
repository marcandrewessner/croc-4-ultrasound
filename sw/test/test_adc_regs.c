// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// Exercises the ADC_ACQ register file itself: bus addressability and
// per-field RW/RO/WO behaviour, independent of the acquisition datapath.
//
// Several fields are write-only from software's perspective (see
// adc_acquisition_reg_definition.rdl: hw=r, sw=w -- all of CNTRL, and the
// F0/F1 START/END_ADDR frame-config registers). The generated register
// file's readback mux (adc_acquisition_reg.sv, ~line 728) has no case for
// these addresses at all, so reading them back always yields 0 -- that is
// correct, expected behaviour, not a bug. A naive write-then-read-back test
// would misreport that as a failure; instead we verify their *effect*
// indirectly, e.g. RESET_WRITE_HEAD copying F0_START_ADDR into WRITE_HEAD
// (which *is* readable).

#include "uart.h"
#include "print.h"
#include "util.h"
#include "config.h"
#include "adc_acquisition.h"

int main(void) {
    uart_init();
    printf("test_adc_regs\n");

    // ---- reset defaults ------------------------------------------------
    CHECK_ASSERT(1, ADC_ACQ->CONF == ADC_ACQ_MODE_IDLE);
    CHECK_ASSERT(2, ADC_ACQ->STATUS == 0);
    CHECK_ASSERT(3, ADC_ACQ->WRITE_HEAD == 0);
    CHECK_ASSERT(4, ADC_ACQ->SDCARD_BLOCK_ADDR == 0);

    // ---- plain RW register: SDCARD_BLOCK_ADDR ---------------------------
    ADC_ACQ->SDCARD_BLOCK_ADDR = 0xDEADBEEFu;
    CHECK_ASSERT(5, ADC_ACQ->SDCARD_BLOCK_ADDR == 0xDEADBEEFu);
    ADC_ACQ->SDCARD_BLOCK_ADDR = 0;

    // ---- RW register with a restricted value range: CONF.MODE ----------
    // Use CONTINUOUS_ACQ_F0_F1: unlike the SINGLE_* modes it never
    // auto-reverts to IDLE by itself, so a readback right after the write
    // can't race the hardware. Frame span covers a whole bank each so no
    // frame can complete during the handful of cycles this test takes.
    ADC_ACQ->F0_START_ADDR = 0x10001000u;
    ADC_ACQ->F0_END_ADDR   = 0x100017FCu; // last word of bank0 (2 KiB)
    ADC_ACQ->F1_START_ADDR = 0x10001800u;
    ADC_ACQ->F1_END_ADDR   = 0x10001FFCu; // last word of bank1
    ADC_ACQ->CNTRL         = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF           = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1;
    CHECK_ASSERT(6, ADC_ACQ->CONF == ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1);
    ADC_ACQ->CONF           = ADC_ACQ_MODE_IDLE;
    CHECK_ASSERT(7, ADC_ACQ->CONF == ADC_ACQ_MODE_IDLE);

    // ---- write-only registers: verify effect, not readback --------------
    // F0_START_ADDR is sw=w. Prove the write actually landed by observing
    // it appear in WRITE_HEAD (sw=r) after a RESET_WRITE_HEAD pulse -- do
    // it twice with different values to rule out a stuck/latched write path.
    ADC_ACQ->F0_START_ADDR = 0x10001234u;
    ADC_ACQ->CNTRL          = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    CHECK_ASSERT(8, ADC_ACQ->WRITE_HEAD == 0x10001234u);

    ADC_ACQ->F0_START_ADDR = 0x10001008u;
    ADC_ACQ->CNTRL          = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    CHECK_ASSERT(9, ADC_ACQ->WRITE_HEAD == 0x10001008u);

    // ---- read-only registers ignore software writes ----------------------
    // STATUS fields are all sw=r (hw=rw or hw=r): a raw CPU write must be a
    // silent no-op, only CNTRL's clear-pulses / hardware may change them.
    ADC_ACQ->STATUS = 0xFFFFFFFFu;
    CHECK_ASSERT(10, ADC_ACQ->STATUS == 0);

    // WRITE_HEAD is sw=r (hw=rw): a raw CPU write must not move it.
    ADC_ACQ->WRITE_HEAD = 0xFFFFFFFFu;
    CHECK_ASSERT(11, ADC_ACQ->WRITE_HEAD == 0x10001008u);

    // ---- address-map isolation: writing one register must not alias -----
    // ---- into its neighbour (walking pattern across the reg map) --------
    ADC_ACQ->SDCARD_BLOCK_ADDR = 0xAAAAAAAAu;
    CHECK_ASSERT(12, ADC_ACQ->SDCARD_BLOCK_ADDR == 0xAAAAAAAAu);
    CHECK_ASSERT(13, ADC_ACQ->CONF == ADC_ACQ_MODE_IDLE); // untouched

    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE; // (still IDLE; exercises CONF's decode)
    CHECK_ASSERT(14, ADC_ACQ->SDCARD_BLOCK_ADDR == 0xAAAAAAAAu); // untouched

    ADC_ACQ->SDCARD_BLOCK_ADDR = 0x55555555u;
    CHECK_ASSERT(15, ADC_ACQ->SDCARD_BLOCK_ADDR == 0x55555555u);
    CHECK_ASSERT(16, ADC_ACQ->CONF == ADC_ACQ_MODE_IDLE);

    ADC_ACQ->SDCARD_BLOCK_ADDR = 0;
    printf("test_adc_regs OK\n");
    uart_write_flush();
    return 0;
}
