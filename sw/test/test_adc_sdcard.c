// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// End-to-end test for CONF.MODE = SDCARD_CONTINUOUS with SDCARD_FRAME_COUNT = 1
// (single-frame capture): exercises the full ADC -> pack -> CDC FIFO ->
// SRAM -> copy engine -> SDHCI -> card path, and the SDCARD_OVERFLOW
// safety path.
//
// A frame is one whole SRAM bank (2 KiB), which the copy engine streams out
// as a single CMD25 session of BLOCKS_PER_FRAME 512 B blocks, closed by
// AUTO_CMD12. Software opens nothing: the engine writes SDHCI's
// BLOCK_SIZE/BLOCK_COUNT/TRANSFER_MODE and issues CMD25 itself.
//
// Part A (happy path) lets hardware fill F0 and stream it to the card, then
// reads all BLOCKS_PER_FRAME blocks back with CMD17 and diffs them
// word-for-word against what is still sitting in SRAM -- this proves the
// copy engine moved the *right* words to the *right* place, not merely that
// *some* words were transferred, and that the card's internal per-block
// address advance within the session lined the blocks up contiguously. It
// also independently checks the SRAM contents against the same
// self-referential ADC progression invariant used by the other ADC tests,
// so a failure here can be localised to "wrong data reached SRAM" vs.
// "right data reached SRAM but the copy engine mangled it".
//
// Part B (negative path) exercises the copy engine's stall timeout: the
// session is configured with a BLOCK_COUNT smaller than the frame, so SDHCI
// stops accepting words partway through (accepts_data_port_chunk goes low
// once its block counter reaches 0) and withholds the OBI grant forever.
// The engine must time out into SDCARD_OVERFLOW rather than hang, and must
// not advance SDCARD_BLOCK_ADDR on a failed session.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "clint.h"
#include "config.h"
#include "adc_acquisition.h"
#include "sdhcreg.h"
#include "sdhci_helpers.h"

#define F0_BASE           0x10001000u
#define SD_BLOCK_BYTES    512u
#define N_WORDS           ADC_ACQ_BANK_WORDS                    // 512 words = 2 KiB
#define FRAME_BYTES       (N_WORDS * 4u)
#define BLOCKS_PER_FRAME  (FRAME_BYTES / SD_BLOCK_BYTES)        // 4
#define WORDS_PER_BLOCK   (SD_BLOCK_BYTES / 4u)                 // 128
#define F0_END            ADC_ACQ_FRAME_END(F0_BASE, N_WORDS)

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
    // Part A: happy path. Software opens nothing -- the copy engine issues
    // its own CMD25 for the whole frame and AUTO_CMD12 closes it.
    // ---------------------------------------------------------------
    CHECK_ASSERT(2, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);

    // Byte addressing (BLOCK_UNITS=0) to match sdModel.v, which advertises
    // CCS=1 but indexes FLASHmem with a raw byte address -- see
    // sw/sdcard_acquisition_Nx.c for the full explanation.
    ADC_ACQ->SDCARD_ADDR_MODE   = 0;
    ADC_ACQ->SDCARD_BLOCK_SIZE  = SD_BLOCK_BYTES;
    ADC_ACQ->SDCARD_BLOCK_COUNT = BLOCKS_PER_FRAME;
    ADC_ACQ->SDCARD_BLOCK_ADDR  = 0;
    ADC_ACQ->SDCARD_FRAME_COUNT = 1;
    ADC_ACQ->F0_START_ADDR      = F0_BASE;
    ADC_ACQ->F0_END_ADDR        = F0_END;
    ADC_ACQ->CNTRL              = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF               = ADC_ACQ_MODE_SDCARD_CONTINUOUS;

    uint32_t status = wait_terminal_status();
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

    CHECK_ASSERT(3, status & ADC_ACQ_STATUS_SDCARD_DONE);
    CHECK_ASSERT(4, !(status & ADC_ACQ_STATUS_SDCARD_OVERFLOW));
    CHECK_ASSERT(5, !(status & ADC_ACQ_STATUS_ADC_OVERFLOW));
    // One completed session advances by exactly one frame's worth.
    CHECK_ASSERT(6, ADC_ACQ->SDCARD_BLOCK_ADDR == FRAME_BYTES);

    // What actually reached SRAM before the copy engine touched it -- an
    // ADC/CDC/DMA bug and a copy-engine bug would show up as different
    // failing indices (this check vs. the CMD17 diff below).
    CHECK_ASSERT(7, check_progression((volatile uint32_t *)F0_BASE, N_WORDS));

    // Read every block of the session back (CMD17 each) and diff word-for-word
    // against SRAM (still intact -- one frame never rewrites F0). Reading all
    // BLOCKS_PER_FRAME of them is what checks that the card's internal
    // per-block advance within the CMD25 session placed them contiguously:
    // a single-block readback would pass even if blocks 1..3 had landed on
    // top of each other.
    uint32_t mismatches = 0;
    for (uint32_t b = 0; b < BLOCKS_PER_FRAME; b++) {
        CHECK_ASSERT(8, sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK) == 0);
        *reg16(SDH_BASE, SDHC_BLOCK_SIZE)    = SD_BLOCK_BYTES;
        *reg16(SDH_BASE, SDHC_BLOCK_COUNT)   = 1;
        *reg32(SDH_BASE, SDHC_ARGUMENT)      = b * SD_BLOCK_BYTES; // byte addressing
        *reg16(SDH_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
        *reg16(SDH_BASE, SDHC_COMMAND) =
            (17 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_DATA_PRESENT_SELECT |
            SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48;
        CHECK_ASSERT(9, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
        CHECK_ASSERT(10, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;

        for (uint32_t i = 0; i < WORDS_PER_BLOCK; i++) {
            uint32_t card_word = *reg32(SDH_BASE, SDHC_DATA);
            uint32_t sram_word = *reg32(F0_BASE, 4 * (b * WORDS_PER_BLOCK + i));
            if (card_word != sram_word) mismatches++;
        }
        CHECK_ASSERT(11, sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE) == 0);
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    }
    printf("mismatches=%x\n", mismatches);
    CHECK_ASSERT(12, mismatches == 0);
    printf("Part A OK\n");

    // ---------------------------------------------------------------
    // Part B: session geometry too small for the frame -> SDHCI stops
    // accepting words partway through and never grants again. The copy
    // engine's CE_COPY_WORD stall timeout must convert that into
    // SDCARD_OVERFLOW instead of hanging.
    // ---------------------------------------------------------------
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS | (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);
    ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->SDCARD_BLOCK_COUNT = 1;   // frame is BLOCKS_PER_FRAME blocks long
    ADC_ACQ->SDCARD_FRAME_COUNT = 1;

    uint32_t block_addr_before = ADC_ACQ->SDCARD_BLOCK_ADDR;
    ADC_ACQ->CONF  = ADC_ACQ_MODE_SDCARD_CONTINUOUS;

    status = wait_terminal_status();
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

    CHECK_ASSERT(13, status & ADC_ACQ_STATUS_SDCARD_OVERFLOW);
    CHECK_ASSERT(14, !(status & ADC_ACQ_STATUS_SDCARD_DONE));
    // A failed session must not advance the block pointer.
    CHECK_ASSERT(15, ADC_ACQ->SDCARD_BLOCK_ADDR == block_addr_before);

    // Overflow must be clearable, and clearing it must not resurrect DONE.
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS;
    CHECK_ASSERT(16, !(ADC_ACQ->STATUS & ADC_ACQ_STATUS_SDCARD_OVERFLOW));
    CHECK_ASSERT(17, !(ADC_ACQ->STATUS & ADC_ACQ_STATUS_SDCARD_DONE));
    printf("Part B OK\n");

    printf("test_adc_sdcard OK\n");
    uart_write_flush();
    return 0;
}
