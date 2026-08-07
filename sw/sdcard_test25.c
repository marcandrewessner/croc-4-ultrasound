// Copyright (c) 2024 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// Minimal, software-driven CMD25 (WRITE_MULTIPLE_BLOCK) + CMD18
// (READ_MULTIPLE_BLOCK) round trip over 2 blocks, with AUTO_CMD12. Same
// init sequence as sdcard_test.c (that file's CMD24/CMD17 single-block
// round trip is unchanged and still the reference for "does the link work
// at all"), diverging only at the data-transfer step -- this exists to
// isolate whether a CMD25 multi-block session specifically has trouble at
// the current SDCLK, independent of the hardware copy engine
// (rtl/adc_acquisition/adc_acquisition_sdcard_controller.sv) that
// sdcard_acquisition_pulse/_Nx rely on for their own CMD25 sessions -- see
// project_croc_sdcard_clock_limit memory for the SDCARD_OVERFLOW history
// that motivated this.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "clint.h"
#include "sdhcreg.h"

#define SDHCI_BASE    0x10010000
#define TIMEOUT       10000U    // ~300 ms at 32 kHz — enough for any single op

// Number of 512B blocks to write+verify in the CMD25/CMD18 session. Override
// on the command line like SIM_CARD, e.g. `make N_BLOCKS=8 sdcard_test25...`.
#ifndef N_BLOCKS
#define N_BLOCKS      8
#endif
#define WORDS_PER_BLK 128        // 512 B / 4
#define NWORDS        (N_BLOCKS * WORDS_PER_BLK)

// ---------------------------------------------------------------------------
// Polling helpers
// ---------------------------------------------------------------------------

static int spin_until_clear8(uint32_t base, int off, uint8_t mask) {
    uint64_t end = clint_get_mtime() + TIMEOUT;
    while (*reg8(base, off) & mask)
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

static int spin_until_set16(uint32_t base, int off, uint16_t mask) {
    uint64_t end = clint_get_mtime() + TIMEOUT;
    while (!(*reg16(base, off) & mask))
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

static int spin_until_clear32(uint32_t base, int off, uint32_t mask) {
    uint64_t end = clint_get_mtime() + TIMEOUT;
    while (*reg32(base, off) & mask)
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

// Print key registers: NINTR, EINTR, PRESENT_STATE (all %x, no %s)
static void print_regs(void) {
    uint16_t n  = *reg16(SDHCI_BASE, SDHC_NINTR_STATUS);
    uint16_t e  = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
    uint32_t ps = *reg32(SDHCI_BASE, SDHC_PRESENT_STATE);
    printf("NINTR=%x EINTR=%x PS=%x\n", (uint32_t)n, (uint32_t)e, ps);
}

// Numeric step code instead of a unique string per checkpoint -- same
// reasoning as sdcard_test.c: grep this file for the step number.
static int fail(int step, uint32_t detail) {
    printf("FAIL=%x:%x\n", step, detail);
    uart_write_flush();
    return step;
}

static void ok(int step, uint32_t detail) {
    printf("OK=%x:%x\n", step, detail);
}

// ---------------------------------------------------------------------------
// Send one command and wait for Command Complete.
// ---------------------------------------------------------------------------
static int sdhci_cmd(uint8_t idx, uint32_t arg, uint16_t flags) {
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD))
        return -1;

    *reg32(SDHCI_BASE, SDHC_ARGUMENT) = arg;
    *reg16(SDHCI_BASE, SDHC_COMMAND)  = ((uint16_t)idx << SDHC_COMMAND_INDEX_SHIFT) | flags;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return -2;

    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t err = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = err;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
        return (int)err | 0x10000;
    }

    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    return 0;
}

// Expected word value at flat index i (0..NWORDS-1): block 0 uses 0..127,
// block 1 uses 0x1000.. so a swapped/overlapped block boundary is obvious
// at a glance instead of just "some words wrong".
static uint32_t expect_word(int i) {
    int blk = i / WORDS_PER_BLK;
    int off = i % WORDS_PER_BLK;
    return (uint32_t)(blk * 0x1000 + off);
}

// AUTO_CMD12 never raises its own COMMAND_COMPLETE (autocmd_wrap.sv masks
// it), so there is a ~1-cycle window where PRESENT_STATE reads idle before
// AUTO_CMD12 has even started -- see
// adc_acquisition_sdcard_controller.sv's CE_WAIT_CARD_READY_RSP comment,
// which this mirrors: require two consecutive idle polls, not one.
static int wait_card_ready(void) {
    uint64_t end        = clint_get_mtime() + TIMEOUT;
    int      ready_once = 0;
    for (;;) {
        uint32_t ps = *reg32(SDHCI_BASE, SDHC_PRESENT_STATE);
        if ((ps & SDHC_CMD_INHIBIT_MASK) == 0 && (ps & SDHC_DAT0_LINE_LEVEL)) {
            if (ready_once) return 0;
            ready_once = 1;
        } else {
            ready_once = 0;
        }
        if (clint_get_mtime() >= end) return -1;
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    uart_init();

    // Sanity-check capabilities register
    uint32_t caps = *reg32(SDHCI_BASE, SDHC_CAPABILITIES);
    printf("CAPS=%x\n", caps);
    if (caps == 0 || caps == 0xffffffff) return fail(1, caps);

    // -----------------------------------------------------------------------
    // 1. Controller reset and clock setup
    // -----------------------------------------------------------------------
    *reg8(SDHCI_BASE, SDHC_SOFTWARE_RESET) = SDHC_RESET_ALL;
    if (spin_until_clear8(SDHCI_BASE, SDHC_SOFTWARE_RESET, SDHC_RESET_ALL))
        return fail(2, 0);

    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return fail(3, 0);

    // Identification-speed divider -- see sdcard_test.c for the full
    // ClkPreDiv/decode(freq_sel) derivation. freq_sel=128 -> divisor 512 ->
    // clk_soc/512, comfortably under the SD spec's 400kHz ceiling whatever
    // clk_soc currently is (check sw/config.h's TB_FREQUENCY, kept in sync).
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(128);
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return fail(3, 1);
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    *reg8(SDHCI_BASE, SDHC_POWER_CTL)   = (SDHC_VOLTAGE_3_3V << SDHC_VOLTAGE_SHIFT) | SDHC_BUS_POWER;
    *reg8(SDHCI_BASE, SDHC_TIMEOUT_CTL) = SDHC_TIMEOUT_MAX;

    // Enable status bits we will poll
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS_EN) =
        SDHC_COMMAND_COMPLETE   |
        SDHC_TRANSFER_COMPLETE  |
        SDHC_BUFFER_WRITE_READY |
        SDHC_BUFFER_READ_READY  |
        SDHC_ERROR_INTERRUPT;
    *reg16(SDHCI_BASE, SDHC_EINTR_STATUS_EN) = SDHC_EINTR_STATUS_MASK;
    ok(1, 0);

    // -----------------------------------------------------------------------
    // 2. CMD0 – GO_IDLE_STATE (no response)
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD))
        return fail(4, 0);
    *reg32(SDHCI_BASE, SDHC_ARGUMENT) = 0;
    *reg16(SDHCI_BASE, SDHC_COMMAND)  = (0 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_NO_RESPONSE;
    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return fail(5, 0);
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    ok(2, 0);

    // -----------------------------------------------------------------------
    // 3. CMD8 – SEND_IF_COND
    // -----------------------------------------------------------------------
    int r = sdhci_cmd(8, 0x000001AA,
                      SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(6, r);
    uint32_t r7 = *reg32(SDHCI_BASE, SDHC_RESPONSE);
    if ((r7 & 0xFF) != 0xAA) return fail(7, r7);
    ok(3, r7);

    // -----------------------------------------------------------------------
    // 4. CMD55 + ACMD41 loop — wait for OCR[31]=1
    // -----------------------------------------------------------------------
    uint32_t ocr = 0;
    for (int i = 0; i < 200; i++) {
        r = sdhci_cmd(55, 0,
                      SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
        if (r) return fail(8, r);
        r = sdhci_cmd(41, 0x40FF8000, SDHC_RESP_LEN_48);
        if (r) return fail(9, r);
        ocr = *reg32(SDHCI_BASE, SDHC_RESPONSE);
        if (ocr & (1u << 31)) break;
        clint_spin_ticks(5);
    }
    if (!(ocr & (1u << 31))) return fail(10, ocr);
    ok(4, ocr);

    // -----------------------------------------------------------------------
    // 5. CMD2 – ALL_SEND_CID
    // -----------------------------------------------------------------------
    r = sdhci_cmd(2, 0, SDHC_CRC_CHECK_ENABLE | SDHC_RESP_LEN_136);
    if (r) return fail(11, r);

    // -----------------------------------------------------------------------
    // 6. CMD3 – SEND_RELATIVE_ADDR
    // -----------------------------------------------------------------------
    r = sdhci_cmd(3, 0,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(12, r);
    uint32_t rca = (*reg32(SDHCI_BASE, SDHC_RESPONSE) >> 16) & 0xFFFF;
    printf("RCA=%x\n", rca);

    // -----------------------------------------------------------------------
    // 7. CMD7 – SELECT_CARD (R1b)
    // -----------------------------------------------------------------------
    r = sdhci_cmd(7, rca << 16,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48_CHK_BUSY);
    if (r) return fail(13, r);
    ok(5, rca);

    // -----------------------------------------------------------------------
    // 8. ACMD6 -- SET_BUS_WIDTH, sent at identification speed, before the
    //    clock switch below -- see sdcard_test.c for the full rationale.
    // -----------------------------------------------------------------------
    r = sdhci_cmd(55, rca << 16,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(24, r);
    r = sdhci_cmd(6, 2 /* bus width: 4-bit */,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(25, r);
    *reg8(SDHCI_BASE, SDHC_HOST_CTL) = SDHC_4BIT_MODE;
    ok(6, 0);

    // -----------------------------------------------------------------------
    // Card is configured -- switch from identification speed to full
    // data-transport speed. Same divider sdcard_test.c currently uses --
    // kept identical deliberately, since this program's whole point is
    // isolating CMD25 behavior at that same SDCLK, not testing a different
    // one.
    // -----------------------------------------------------------------------
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;                    // SD_CLOCK_EN=0
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(0); // 25MHz
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return fail(13, 1);
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    ok(7, 0);

    // -----------------------------------------------------------------------
    // 9. CMD16 – SET_BLOCKLEN = 512
    // -----------------------------------------------------------------------
    r = sdhci_cmd(16, 512,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(14, r);
    ok(8, 0);
    print_regs();

    // -----------------------------------------------------------------------
    // 10. Write N_BLOCKS blocks starting at block 0 (CMD25 –
    //     WRITE_MULTIPLE_BLOCK, MULTI_BLOCK | BLOCK_COUNT_ENABLE |
    //     AUTO_CMD12_ENABLE) -- mirrors adc_acquisition_sdcard_controller.sv's
    //     CE_SET_BLOCK_SIZE..CE_SUBMIT_CMD25 sequence.
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK))
        return fail(15, 0);

    *reg16(SDHCI_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDHCI_BASE, SDHC_BLOCK_COUNT)   = N_BLOCKS;
    *reg32(SDHCI_BASE, SDHC_ARGUMENT)      = 0;    // starting block 0
    *reg16(SDHCI_BASE, SDHC_TRANSFER_MODE) =
        SDHC_MULTI_BLOCK_MODE   |
        SDHC_BLOCK_COUNT_ENABLE |
        SDHC_AUTO_CMD12_ENABLE;                    // write direction: bit4=0
    *reg16(SDHCI_BASE, SDHC_COMMAND) =
        (25 << SDHC_COMMAND_INDEX_SHIFT) |
        SDHC_DATA_PRESENT_SELECT         |
        SDHC_CRC_CHECK_ENABLE            |
        SDHC_INDEX_CHECK_ENABLE          |
        SDHC_RESP_LEN_48;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return fail(16, 0);
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    ok(9, 0);
    printf("R1=%x\n", *reg32(SDHCI_BASE, SDHC_RESPONSE));

    // Wait for BUFFER_WRITE_READY once, then stream all NWORDS through --
    // not re-polled per block. SDHCI backpressures the writes transparently
    // block-to-block as long as BLOCK_COUNT_ENABLE is set (dat_wrap.sv).
    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_BUFFER_WRITE_READY)) {
        print_regs();
        return fail(17, 0);
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_WRITE_READY;
    ok(10, 0);

    for (int i = 0; i < NWORDS; i++)
        *reg32(SDHCI_BASE, SDHC_DATA) = expect_word(i);

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE)) {
        print_regs();
        return fail(18, 0);
    }
    print_regs();
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    ok(11, 0);

    // Two-consecutive-poll card-ready wait -- covers both the card's own
    // programming busy (DAT0 low) and AUTO_CMD12's completion, which raises
    // no COMMAND_COMPLETE of its own. See wait_card_ready()'s comment.
    if (wait_card_ready()) {
        print_regs();
        return fail(18, 1);
    }
    ok(11, 1);

    // -----------------------------------------------------------------------
    // 11. Read N_BLOCKS blocks back (CMD18 – READ_MULTIPLE_BLOCK) and verify
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK))
        return fail(19, 0);

    *reg16(SDHCI_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDHCI_BASE, SDHC_BLOCK_COUNT)   = N_BLOCKS;
    *reg32(SDHCI_BASE, SDHC_ARGUMENT)      = 0;
    *reg16(SDHCI_BASE, SDHC_TRANSFER_MODE) =
        SDHC_MULTI_BLOCK_MODE   |
        SDHC_BLOCK_COUNT_ENABLE |
        SDHC_AUTO_CMD12_ENABLE  |
        SDHC_READ_MODE;
    printf("BSZ=%x BCNT=%x TM=%x\n",
           (uint32_t)*reg16(SDHCI_BASE, SDHC_BLOCK_SIZE),
           (uint32_t)*reg16(SDHCI_BASE, SDHC_BLOCK_COUNT),
           (uint32_t)*reg16(SDHCI_BASE, SDHC_TRANSFER_MODE));
    *reg16(SDHCI_BASE, SDHC_COMMAND) =
        (18 << SDHC_COMMAND_INDEX_SHIFT) |
        SDHC_DATA_PRESENT_SELECT         |
        SDHC_CRC_CHECK_ENABLE            |
        SDHC_INDEX_CHECK_ENABLE          |
        SDHC_RESP_LEN_48;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return fail(20, 0);
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    ok(12, 0);
    printf("R1=%x\n", *reg32(SDHCI_BASE, SDHC_RESPONSE));
    print_regs();

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY)) {
        print_regs();
        return fail(21, 0);
    }
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;
    ok(13, 0);
    print_regs();

    // Read and verify all NWORDS -- not re-polled per block, same
    // reasoning as the write side.
    int errors = 0;
    int first_bad = -1;
    uint32_t first_bad_val = 0;
    uint32_t w0 = 0, w1 = 0, w2 = 0, w3 = 0;
    for (int i = 0; i < NWORDS; i++) {
        uint32_t val = *reg32(SDHCI_BASE, SDHC_DATA);
        if (i == 0) w0 = val;
        if (i == 1) w1 = val;
        if (i == WORDS_PER_BLK)     w2 = val; // first word of block 1
        if (i == WORDS_PER_BLK + 1) w3 = val;
        if (val != expect_word(i)) {
            errors++;
            if (first_bad < 0) { first_bad = i; first_bad_val = val; }
        }
    }

    printf("w0=%x w1=%x w2=%x w3=%x\n", w0, w1, w2, w3);
    printf("Errors=%x\n", errors);
    if (first_bad >= 0)
        printf("First bad word[%x]=%x\n", first_bad, first_bad_val);

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE)) {
        print_regs();
        return fail(22, 0);
    }
    print_regs();
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;

    if (wait_card_ready()) {
        print_regs();
        return fail(22, 1);
    }

    if (errors) return fail(23, (uint32_t)errors);

    printf("Success\n");
    uart_write_flush();
    return 0;
}
