// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// End-to-end test for CONF.MODE = ACQ_SDCARD with SDCARD_FRAME_COUNT = 1
// (single-frame capture): exercises the full ADC -> pack -> CDC FIFO ->
// SRAM -> copy engine -> SDHCI -> card path, and the SDCARD_OVERFLOW
// safety path when the card isn't ready.
//
// Part A (happy path) opens a CMD25 write stream, lets hardware fill F0 and
// copy it to the SDHCI buffer, then reads the block back with CMD17 and
// diffs it against what's still sitting in SRAM -- this proves the copy
// engine moved the *right* words to the *right* place, not merely that
// *some* 128 words were transferred. It also independently checks the SRAM
// contents against the same self-referential ADC progression invariant
// used by the other ADC tests, so a failure here can be localised to
// "wrong data reached SRAM" vs. "right data reached SRAM but the copy
// engine mangled it".
//
// Part B (negative path) is the previously-unverified failure mode: start
// ACQ_SDCARD *without* ever opening a CMD25 stream, so
// BUFFER_WRITE_READY never asserts. The copy engine must detect this,
// raise SDCARD_OVERFLOW, and stop the ADC without touching
// SDCARD_BLOCK_ADDR or corrupting state -- none of which had ever run
// before this test.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "clint.h"
#include "config.h"
#include "adc_acquisition.h"
#include "sdhcreg.h"
#include "sdhci_helpers.h"

#define F0_BASE  0x10001000u
#define N_WORDS  128u                              // 512 B = 1 SD block
#define F0_END   (F0_BASE + (N_WORDS - 1u) * 4u)

static inline uint32_t lo14(uint32_t w) { return w & 0x3FFFu; }
static inline uint32_t hi14(uint32_t w) { return (w >> 16) & 0x3FFFu; }

static int check_progression(volatile uint32_t *words, uint32_t n) {
    uint32_t prev_hi = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t w = words[i];
        if (w & 0xC000C000u) return 0;
        uint32_t lo = lo14(w), hi = hi14(w);
        if (hi != ((lo + 1u) & 0x3FFFu)) return 0;
        if (i > 0 && lo != ((prev_hi + 1u) & 0x3FFFu)) return 0;
        prev_hi = hi;
    }
    return 1;
}

// Poll STATUS for the SDCARD_* / ADC_OVERFLOW terminal bits, bounded by a
// real wall-clock timeout (CLINT) since a stuck copy engine should time out
// cleanly here rather than hang the whole test binary. SDCARD_DONE now
// fires once the block is genuinely physically written (TRANSFER_COMPLETE),
// not just once its words reached the SDHCI buffer -- still well within
// this timeout on the sdModel.

static uint32_t wait_terminal_status(void) {
    uint64_t end = clint_get_mtime() + 20000; // ~600 ms
    uint32_t status = 0;
    while (!(status & (ADC_ACQ_STATUS_SDCARD_DONE |
                        ADC_ACQ_STATUS_SDCARD_OVERFLOW |
                        ADC_ACQ_STATUS_ADC_OVERFLOW))) {
        status = ADC_ACQ->STATUS;
        if (clint_get_mtime() >= end) break;
    }
    return status;
}

int main(void) {
    uart_init();
    printf("test_adc_sdcard\n");

    uint32_t rca = sdh_init();
    CHECK_ASSERT(1, rca != 0);

    // ---------------------------------------------------------------
    // Part A: CMD25 opened -> HW copy engine should complete the block.
    // ---------------------------------------------------------------
    CHECK_ASSERT(2, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);
    *reg16(SDH_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDH_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDH_BASE, SDHC_ARGUMENT)      = 0; // LBA 0
    *reg16(SDH_BASE, SDHC_TRANSFER_MODE) =
        SDHC_MULTI_BLOCK_MODE | SDHC_BLOCK_COUNT_ENABLE | SDHC_AUTO_CMD12_ENABLE;
    *reg16(SDH_BASE, SDHC_COMMAND) =
        (25 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_DATA_PRESENT_SELECT |
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48;
    CHECK_ASSERT(3, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE) == 0);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    CHECK_ASSERT(4, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_BUFFER_WRITE_READY) == 0);

    ADC_ACQ->SDCARD_BLOCK_ADDR  = 0;
    ADC_ACQ->SDCARD_FRAME_COUNT = 1;
    ADC_ACQ->F0_START_ADDR      = F0_BASE;
    ADC_ACQ->F0_END_ADDR        = F0_END;
    ADC_ACQ->CNTRL              = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF               = ADC_ACQ_MODE_ACQ_SDCARD;

    uint32_t status = wait_terminal_status();
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

    CHECK_ASSERT(5, status & ADC_ACQ_STATUS_SDCARD_DONE);
    CHECK_ASSERT(6, !(status & ADC_ACQ_STATUS_SDCARD_OVERFLOW));
    CHECK_ASSERT(7, !(status & ADC_ACQ_STATUS_ADC_OVERFLOW));
    CHECK_ASSERT(8, ADC_ACQ->SDCARD_BLOCK_ADDR == 1); // advanced by exactly 1 block

    // Close the CMD25 stream (AUTO_CMD12 may already have done this; an
    // explicit CMD12 is safe and leaves the card in a known TRAN state).
    sdh_cmd(12, 0, SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE |
                   SDHC_COMMAND_TYPE_ABORT | SDHC_RESP_LEN_48_CHK_BUSY);
    sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;

    // What actually reached SRAM before the copy engine touched it -- an
    // ADC/CDC/DMA bug and a copy-engine bug would show up as different
    // failing indices (this check vs. the CMD17 diff below).
    CHECK_ASSERT(9, check_progression((volatile uint32_t *)F0_BASE, N_WORDS));

    // Read block 0 back from the card (CMD17) and diff word-for-word
    // against SRAM (still intact -- N=1 never rewrites F0).
    CHECK_ASSERT(10, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);
    *reg16(SDH_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDH_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDH_BASE, SDHC_ARGUMENT)      = 0;
    *reg16(SDH_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
    *reg16(SDH_BASE, SDHC_COMMAND) =
        (17 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_DATA_PRESENT_SELECT |
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48;
    CHECK_ASSERT(11, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE) == 0);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    CHECK_ASSERT(12, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY) == 0);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;

    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < N_WORDS; i++) {
        uint32_t card_word = *reg32(SDH_BASE, SDHC_DATA);
        uint32_t sram_word = *reg32(F0_BASE, 4 * i);
        if (card_word != sram_word) mismatches++;
    }
    CHECK_ASSERT(13, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE) == 0);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    printf("mismatches=%x\n", mismatches);
    CHECK_ASSERT(14, mismatches == 0);
    printf("Part A OK\n");

    // ---------------------------------------------------------------
    // Part B: no CMD25 opened -> BUFFER_WRITE_READY never asserts.
    // Copy engine must raise SDCARD_OVERFLOW and stop the ADC cleanly.
    // ---------------------------------------------------------------
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS | (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);
    ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->SDCARD_FRAME_COUNT = 1;
    ADC_ACQ->CONF  = ADC_ACQ_MODE_ACQ_SDCARD;

    status = wait_terminal_status();
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

    CHECK_ASSERT(15, status & ADC_ACQ_STATUS_SDCARD_OVERFLOW);
    CHECK_ASSERT(16, !(status & ADC_ACQ_STATUS_SDCARD_DONE));
    // A failed copy must not advance the block pointer.
    CHECK_ASSERT(17, ADC_ACQ->SDCARD_BLOCK_ADDR == 1);

    // Overflow must be clearable, and clearing it must not resurrect DONE.
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS;
    CHECK_ASSERT(18, !(ADC_ACQ->STATUS & ADC_ACQ_STATUS_SDCARD_OVERFLOW));
    CHECK_ASSERT(19, !(ADC_ACQ->STATUS & ADC_ACQ_STATUS_SDCARD_DONE));
    printf("Part B OK\n");

    printf("test_adc_sdcard OK\n");
    uart_write_flush();
    return 0;
}
