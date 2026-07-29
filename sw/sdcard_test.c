// Copyright (c) 2024 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner

#include "uart.h"
#include "print.h"
#include "util.h"
#include "clint.h"
#include "sdhcreg.h"

#define SDHCI_BASE    0x10010000
#define TIMEOUT       10000U    // ~300 ms at 32 kHz — enough for any single op

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

static int spin_until_set32(uint32_t base, int off, uint32_t mask) {
    uint64_t end = clint_get_mtime() + TIMEOUT;
    while (!(*reg32(base, off) & mask))
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

// Diagnostic: like spin_until_set16, but samples NINTR/PRESENT_STATE
// periodically while waiting instead of returning one opaque pass/fail --
// lets us tell whether the card is still toggling DAT during a stall (just
// slow) or every sample is identical from t=0 (host or card frozen solid).
static int spin_diag(uint16_t mask) {
    uint64_t start = clint_get_mtime();
    uint64_t end   = start + TIMEOUT;
    uint32_t next  = 0;
    while (!(*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & mask)) {
        uint64_t now = clint_get_mtime();
        if (now >= end) return -1;
        if ((uint32_t)(now - start) >= next * (TIMEOUT / 8)) {
            printf("t=%x NINTR=%x PS=%x\n", (uint32_t)(now - start),
                   (uint32_t)*reg16(SDHCI_BASE, SDHC_NINTR_STATUS),
                   *reg32(SDHCI_BASE, SDHC_PRESENT_STATE));
            next++;
        }
    }
    return 0;
}

// Every checkpoint used to carry its own unique message string (~20-30 bytes
// of .rodata each, across ~40 call sites) -- that alone was enough to push
// .text+.rodata past the 4KB SRAM budget and into the stack's territory (see
// link.ld's SRAM/STACK split). A numeric step code plus one shared formatter
// costs a few bytes total; grep this file for the step number to find the
// checkpoint.
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

    // Program the identification-speed divider with SD_CLOCK_EN still 0 --
    // sd_clk_generator only latches a new divider while the SD clock is
    // disabled -- then wait for it to load before starting the clock.
    // sd_clk_generator's ClkPreDiv=2 folds into every divide, so the real
    // divisor is 2*decode(freq_sel), not freq_sel itself: freq_sel=128
    // (0x80) decodes to 256, so actual divisor = 512 -> 50MHz/512 =
    // ~97.7kHz, comfortably under the SD spec's 400kHz identification-speed
    // ceiling.
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
    // Card is selected -- switch from identification speed to full
    // data-transport speed. SD_CLOCK_EN must go back to 0 while the divider
    // is reprogrammed, same rule as the initial clock setup above.
    // -----------------------------------------------------------------------
    // The earlier near-instant Data Timeout Error on CMD17 at 6.25MHz turned
    // out to be the missing ACMD6 (card stuck in 1-bit mode), not a signal-
    // integrity/clock-speed problem -- see sdh_init()/this file's ACMD6 call
    // below. With that fixed, 25MHz (freq_sel=0, the SD spec's Default Speed
    // ceiling) was tried first, but that's a real signal-integrity limit on
    // this board: the very next command (CMD55 prefixing ACMD6) timed out at
    // the CMD-line level. Stepping down: freq_sel=1 decodes to 2 ->
    // divisor=ClkPreDiv(2)*2=4 -> 50MHz/4=12.5MHz.
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;                     // SD_CLOCK_EN=0
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(1); // 12.5MHz
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return fail(13, 1);
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    ok(6, 0);

    // -----------------------------------------------------------------------
    // 8. ACMD6 -- SET_BUS_WIDTH, so the *card* actually drives DAT1-3, then
    //    switch the host to match. Setting only the host's own HOST_CTL
    //    4BIT_MODE bit (all this used to do) never told the card anything --
    //    it stays in its post-reset default of 1-bit mode, so it only ever
    //    drives DAT0. That fits everything seen on real hardware: CMD24
    //    (write) "worked" because its only card-driven feedback is the DAT0
    //    CRC-status token either way, but CMD17 (read) needs the card to
    //    drive all 4 DAT lines -- which it never does while still in 1-bit
    //    mode -- so the host's 4-bit start-of-data detection never fires and
    //    DAT sits idle-high forever.
    // -----------------------------------------------------------------------
    r = sdhci_cmd(55, rca << 16,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(24, r);
    r = sdhci_cmd(6, 2 /* bus width: 4-bit */,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(25, r);
    *reg8(SDHCI_BASE, SDHC_HOST_CTL) = SDHC_4BIT_MODE;
    ok(7, 0);

    // -----------------------------------------------------------------------
    // 9. CMD16 – SET_BLOCKLEN = 512
    //    sdModel never initializes blockSize, Verilator sets it to 0.
    //    Without CMD16, the card sends/receives 0 data bytes per transfer.
    // -----------------------------------------------------------------------
    r = sdhci_cmd(16, 512,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return fail(14, r);
    ok(8, 0);
    print_regs();

    // -----------------------------------------------------------------------
    // 10. Write 512 bytes (counter 0..127) to block 0  (CMD24 – WRITE_BLOCK)
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK))
        return fail(15, 0);

    *reg16(SDHCI_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDHCI_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDHCI_BASE, SDHC_ARGUMENT)      = 0;    // block 0
    *reg16(SDHCI_BASE, SDHC_TRANSFER_MODE) = 0;    // write, single block
    *reg16(SDHCI_BASE, SDHC_COMMAND) =
        (24 << SDHC_COMMAND_INDEX_SHIFT) |
        SDHC_DATA_PRESENT_SELECT         |
        SDHC_CRC_CHECK_ENABLE            |
        SDHC_INDEX_CHECK_ENABLE          |
        SDHC_RESP_LEN_48;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return fail(16, 0);
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    ok(9, 0);
    printf("R1=%x\n", *reg32(SDHCI_BASE, SDHC_RESPONSE)); // CMD24's card status, see CMD17 comment below

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_BUFFER_WRITE_READY)) {
        print_regs();
        return fail(17, 0);
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_WRITE_READY;
    ok(10, 0);

    for (int i = 0; i < 128; i++)
        *reg32(SDHCI_BASE, SDHC_DATA) = (uint32_t)i;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE)) {
        print_regs();
        return fail(18, 0);
    }
    print_regs();
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    ok(11, 0);

    // TRANSFER_COMPLETE only means the host finished framing the block on
    // the bus -- the card still runs its own program-to-flash cycle
    // afterward and signals "busy" by holding DAT0 low throughout (SD spec
    // busy signalling). CMD_INHIBIT_DAT (checked below via CMD_INHIBIT_MASK)
    // is the *host's* bookkeeping bit and does not track this; issuing CMD17
    // while DAT0 is still low gets it dropped by a card still in the PRG
    // state. Poll the raw DAT0 line level directly instead.
    if (spin_until_set32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_DAT0_LINE_LEVEL)) {
        print_regs();
        return fail(18, 1);
    }

    // -----------------------------------------------------------------------
    // 11. Read block 0 back  (CMD17 – READ_SINGLE_BLOCK) and verify
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK))
        return fail(19, 0);

    *reg16(SDHCI_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDHCI_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDHCI_BASE, SDHC_ARGUMENT)      = 0;
    *reg16(SDHCI_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
    // Read back what actually landed -- transfer_mode's direction bit is
    // silently dropped by the RTL while command_inhibit_cmd is set (see
    // sdhci_reg_logic.sv), so confirm TM's bit4 (and BSZ/BCNT) really took
    // instead of assuming the write succeeded.
    printf("BSZ=%x BCNT=%x TM=%x\n",
           (uint32_t)*reg16(SDHCI_BASE, SDHC_BLOCK_SIZE),
           (uint32_t)*reg16(SDHCI_BASE, SDHC_BLOCK_COUNT),
           (uint32_t)*reg16(SDHCI_BASE, SDHC_TRANSFER_MODE));
    *reg16(SDHCI_BASE, SDHC_COMMAND) =
        (17 << SDHC_COMMAND_INDEX_SHIFT) |
        SDHC_DATA_PRESENT_SELECT         |
        SDHC_CRC_CHECK_ENABLE            |
        SDHC_INDEX_CHECK_ENABLE          |
        SDHC_RESP_LEN_48;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return fail(20, 0);
    // Check for errors at command phase
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    ok(12, 0);
    // R1 card status for CMD17 -- never actually inspected before: the SDHC
    // only reports whether a response FRAME arrived cleanly (CRC/index/
    // timeout), not what the card's own status bits inside it say. Bits
    // [12:9] are CURRENT_STATE (4=tran, 7=prg/busy); if the card still
    // thinks it's programming here, it will silently ignore the data phase
    // regardless of what DAT0's line level told the host beforehand.
    printf("R1=%x\n", *reg32(SDHCI_BASE, SDHC_RESPONSE));
    print_regs();

    if (spin_diag(SDHC_BUFFER_READ_READY)) {
        print_regs();
        return fail(21, 0);
    }
    // Check for CRC/end-bit errors before reading
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;
    ok(13, 0);
    print_regs();

    // Read and verify — also print first 4 words and any mismatches
    int errors = 0;
    int first_bad = -1;
    uint32_t first_bad_val = 0;
    uint32_t w0 = 0, w1 = 0, w2 = 0, w3 = 0;
    for (int i = 0; i < 128; i++) {
        uint32_t val = *reg32(SDHCI_BASE, SDHC_DATA);
        if (i == 0) w0 = val;
        if (i == 1) w1 = val;
        if (i == 2) w2 = val;
        if (i == 3) w3 = val;
        if (val != (uint32_t)i) {
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

    if (errors) return fail(23, (uint32_t)errors);

    printf("Success\n");
    uart_write_flush();
    return 0;
}
