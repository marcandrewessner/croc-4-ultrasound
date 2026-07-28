// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// End-to-end test for CONF.MODE = SDCARD_PULSE: the ADC fills F0, rolls on
// into F1, and only once *both* banks are full does the copy engine run,
// streaming all 4 KiB out as a single CMD25 session of 8 x 512 B blocks
// closed by AUTO_CMD12.
//
// What this covers that test_adc_sdcard.c (SDCARD_CONTINUOUS) does not:
//   - the both-banks job: one session spanning two Fx_START_ADDR bases,
//     with the copy engine switching its read base from F0 to F1 partway
//     through. Reading all 8 blocks back with CMD17 and diffing them
//     word-for-word against SRAM is what proves the switch happened at the
//     right word and in the right order -- a bank-switch off by even one
//     word would show as a 4 KiB-wide misalignment from block 4 onward,
//     and a switch that never happened would show F0's contents twice.
//   - both banks released together (F0_FULL and F1_FULL clear) by the one
//     job, and only when it physically committed.
//   - MODE auto-reverting to IDLE on SDCARD_DONE, which is what a
//     software re-arm relies on.
//   - SDCARD_FRAME_COUNT genuinely being ignored: it is deliberately set to
//     1 below, a value that in SDCARD_CONTINUOUS would stop the capture
//     after F0. If the pulse path were sharing the continuous mode's frame
//     budget, F1 would never fill and the session would never start.
//
// The capture also spans the F0 -> F1 boundary as one continuous ADC
// stream, so the self-referential sample-progression invariant is checked
// across all 1024 words rather than per bank: a lost or duplicated sample
// at the bank crossing shows up there and nowhere else.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "clint.h"
#include "config.h"
#include "adc_acquisition.h"
#include "sdhcreg.h"
#include "sdhci_helpers.h"

#define F0_BASE           ADC_ACQ_F0_BASE
#define F1_BASE           ADC_ACQ_F1_BASE
#define SD_BLOCK_BYTES    512u
#define N_WORDS           ADC_ACQ_BANK_WORDS                    // 512 words = 2 KiB per bank
#define FRAME_BYTES       (N_WORDS * 4u)
#define PULSE_WORDS       (2u * N_WORDS)                        // both banks
#define PULSE_BYTES       (2u * FRAME_BYTES)                    // 4 KiB
#define BLOCKS_PER_PULSE  (PULSE_BYTES / SD_BLOCK_BYTES)        // 8
#define WORDS_PER_BLOCK   (SD_BLOCK_BYTES / 4u)                 // 128
#define F0_END            ADC_ACQ_FRAME_END(F0_BASE, N_WORDS)
#define F1_END            ADC_ACQ_FRAME_END(F1_BASE, N_WORDS)

// The readback diff walks the pulse as one linear span, which is only
// legitimate because the two banks abut in the address map. That is a
// property of this SoC's memory map (adc_acquisition.h), not something the
// RTL requires -- the copy engine switches base explicitly -- so assert it
// here rather than let a future remap turn this test into a silent lie.
_Static_assert(F1_BASE == F0_BASE + FRAME_BYTES,
               "test assumes F1 abuts F0; readback diff walks them linearly");

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
    printf("test_adc_sdcard_pulse\n");

    uint32_t rca = sdh_init();
    CHECK_ASSERT(1, rca != 0);
    CHECK_ASSERT(2, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);

    // Byte addressing (BLOCK_UNITS=0) to match sdModel.v, which advertises
    // CCS=1 but indexes FLASHmem with a raw byte address -- see
    // sdh_card_is_block_addressed(). Hardcoded rather than negotiated
    // because the CMD17 readback below addresses blocks as b*512 too, which
    // makes this test sim-only by construction (same as test_adc_sdcard.c).
    ADC_ACQ->SDCARD_ADDR_MODE   = 0;
    ADC_ACQ->SDCARD_BLOCK_SIZE  = SD_BLOCK_BYTES;
    ADC_ACQ->SDCARD_BLOCK_COUNT = BLOCKS_PER_PULSE;  // both banks in one session
    ADC_ACQ->SDCARD_BLOCK_ADDR  = 0;
    ADC_ACQ->SDCARD_FRAME_COUNT = 1;                 // must be ignored -- see header
    ADC_ACQ->F0_START_ADDR      = F0_BASE;
    ADC_ACQ->F0_END_ADDR        = F0_END;
    ADC_ACQ->F1_START_ADDR      = F1_BASE;
    ADC_ACQ->F1_END_ADDR        = F1_END;
    ADC_ACQ->CNTRL              = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF               = ADC_ACQ_MODE_SDCARD_PULSE;

    uint32_t status = wait_terminal_status();

    CHECK_ASSERT(3, status & ADC_ACQ_STATUS_SDCARD_DONE);
    CHECK_ASSERT(4, !(status & ADC_ACQ_STATUS_SDCARD_OVERFLOW));
    CHECK_ASSERT(5, !(status & ADC_ACQ_STATUS_ADC_OVERFLOW));
    // The one session advanced the address by the whole pulse, once -- not
    // twice (which would mean two sessions ran) and not by one bank.
    CHECK_ASSERT(6, ADC_ACQ->SDCARD_BLOCK_ADDR == PULSE_BYTES);
    // One job released both banks.
    CHECK_ASSERT(7, !(status & ADC_ACQ_STATUS_F0_FULL));
    CHECK_ASSERT(8, !(status & ADC_ACQ_STATUS_F1_FULL));
    // Auto-revert to IDLE is what a software re-arm depends on.
    CHECK_ASSERT(9, (ADC_ACQ->CONF & ADC_ACQ_MODE_MASK) == ADC_ACQ_MODE_IDLE);

    // What actually reached SRAM before the copy engine touched it, checked
    // across the bank boundary as one stream.
    CHECK_ASSERT(10, check_progression((volatile uint32_t *)F0_BASE, PULSE_WORDS));

    // Read every block of the session back (CMD17 each) and diff word-for-word
    // against SRAM (still intact -- one pulse never rewrites a bank). Blocks
    // 0..3 came from F0 and blocks 4..7 from F1, so this is what checks both
    // the card's internal per-block advance within the session *and* the copy
    // engine's F0 -> F1 read-base switch.
    uint32_t mismatches = 0;
    for (uint32_t b = 0; b < BLOCKS_PER_PULSE; b++) {
        CHECK_ASSERT(11, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);
        *reg16(SDH_BASE, SDHC_BLOCK_SIZE)    = SD_BLOCK_BYTES;
        *reg16(SDH_BASE, SDHC_BLOCK_COUNT)   = 1;
        *reg32(SDH_BASE, SDHC_ARGUMENT)      = b * SD_BLOCK_BYTES; // byte addressing
        *reg16(SDH_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
        *reg16(SDH_BASE, SDHC_COMMAND) =
            (17 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_DATA_PRESENT_SELECT |
            SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48;
        CHECK_ASSERT(12, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
        CHECK_ASSERT(13, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;

        for (uint32_t i = 0; i < WORDS_PER_BLOCK; i++) {
            uint32_t card_word = *reg32(SDH_BASE, SDHC_DATA);
            uint32_t sram_word = *reg32(F0_BASE, 4 * (b * WORDS_PER_BLOCK + i));
            if (card_word != sram_word) mismatches++;
        }
        CHECK_ASSERT(14, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    }
    printf("mismatches=%x\n", mismatches);
    CHECK_ASSERT(15, mismatches == 0);

    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);

    printf("test_adc_sdcard_pulse OK\n");
    uart_write_flush();
    return 0;
}
