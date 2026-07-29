// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// SDHCI bring-up helpers shared by SD-card test/example programs.
//
// Extracted from the known-working init sequence in sdcard_test.c so it
// doesn't get re-derived (and re-diverge) per program. Deliberately uses the
// same raw register-poke style as sdcard_test.c / sdcard_acquisition.c
// rather than the ported OpenBSD sdhc.c driver (lib/src/sdhc.c) -- that
// driver has never been exercised against this SDHCI RTL / sdModel and is
// far heavier than what a bring-up test needs.

#pragma once

#include <stdint.h>
#include "util.h"
#include "clint.h"
#include "print.h"
#include "sdhcreg.h"

#define SDH_BASE    0x10010000UL
#define SDH_TIMEOUT 10000U   // CLINT ticks (~300 ms at 32 kHz)

static inline int sdh_spin_until_clear8(int off, uint8_t mask) {
    uint64_t end = clint_get_mtime() + SDH_TIMEOUT;
    while (*reg8(SDH_BASE, off) & mask)
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

static inline int sdh_spin_until_set16(int off, uint16_t mask) {
    uint64_t end = clint_get_mtime() + SDH_TIMEOUT;
    while (!(*reg16(SDH_BASE, off) & mask))
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

static inline int sdh_spin_until_clear32(int off, uint32_t mask) {
    uint64_t end = clint_get_mtime() + SDH_TIMEOUT;
    while (*reg32(SDH_BASE, off) & mask)
        if (clint_get_mtime() >= end) return -1;
    return 0;
}

// Send one SD command and wait for CMD_COMPLETE.
// Returns 0 on success, negative on timeout, positive EINTR value on error.
static inline int sdh_cmd(uint8_t idx, uint32_t arg, uint16_t flags) {
    if (sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD))
        return -1;
    *reg32(SDH_BASE, SDHC_ARGUMENT) = arg;
    *reg16(SDH_BASE, SDHC_COMMAND)  =
        ((uint16_t)idx << SDHC_COMMAND_INDEX_SHIFT) | flags;
    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return -2;
    if (*reg16(SDH_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t err = *reg16(SDH_BASE, SDHC_EINTR_STATUS);
        *reg16(SDH_BASE, SDHC_EINTR_STATUS) = err;
        *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
        return (int)err | 0x10000;
    }
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    return 0;
}

// Full card bring-up: reset, identify, select, 4-bit bus, 512 B block length.
// Returns the 16-bit RCA on success, 0 on failure (reason printed).
static inline uint32_t sdh_init(void) {
    *reg8(SDH_BASE, SDHC_SOFTWARE_RESET) = SDHC_RESET_ALL;
    if (sdh_spin_until_clear8(SDHC_SOFTWARE_RESET, SDHC_RESET_ALL)) {
        printf("FAIL: sdh reset\n"); return 0;
    }

    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: sdh intclk\n"); return 0;
    }
    // Program the identification-speed divider with SD_CLOCK_EN still 0 --
    // sd_clk_generator only latches a new divider while the SD clock is
    // disabled -- then wait for it to load before starting the clock.
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(128);
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: sdh sdclk div\n"); return 0;
    }
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    *reg8(SDH_BASE, SDHC_POWER_CTL)   =
        (SDHC_VOLTAGE_3_3V << SDHC_VOLTAGE_SHIFT) | SDHC_BUS_POWER;
    *reg8(SDH_BASE, SDHC_TIMEOUT_CTL) = SDHC_TIMEOUT_MAX;
    *reg16(SDH_BASE, SDHC_NINTR_STATUS_EN) =
        SDHC_COMMAND_COMPLETE | SDHC_TRANSFER_COMPLETE |
        SDHC_BUFFER_WRITE_READY | SDHC_BUFFER_READ_READY | SDHC_ERROR_INTERRUPT;
    *reg16(SDH_BASE, SDHC_EINTR_STATUS_EN) = SDHC_EINTR_STATUS_MASK;

    // CMD0: GO_IDLE_STATE (no response)
    if (sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD)) {
        printf("FAIL: CMD0 inh\n"); return 0;
    }
    *reg32(SDH_BASE, SDHC_ARGUMENT) = 0;
    *reg16(SDH_BASE, SDHC_COMMAND)  = (0 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_NO_RESPONSE;
    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE)) {
        printf("FAIL: CMD0\n"); return 0;
    }
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;

    // CMD8: SEND_IF_COND
    int r = sdh_cmd(8, 0x000001AA,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD8=%x\n", r); return 0; }
    if ((*reg32(SDH_BASE, SDHC_RESPONSE) & 0xFF) != 0xAA) {
        printf("FAIL: CMD8 echo\n"); return 0;
    }

    // ACMD41 loop: wait for card to power-up (OCR bit 31)
    uint32_t ocr = 0;
    for (int i = 0; i < 200; i++) {
        r = sdh_cmd(55, 0,
            SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
        if (r) { printf("FAIL: CMD55=%x\n", r); return 0; }
        r = sdh_cmd(41, 0x40FF8000, SDHC_RESP_LEN_48);
        if (r) { printf("FAIL: ACMD41=%x\n", r); return 0; }
        ocr = *reg32(SDH_BASE, SDHC_RESPONSE);
        if (ocr & (1u << 31)) break;
        clint_spin_ticks(5);
    }
    if (!(ocr & (1u << 31))) { printf("FAIL: OCR=%x\n", ocr); return 0; }

    // CMD2: ALL_SEND_CID
    r = sdh_cmd(2, 0, SDHC_CRC_CHECK_ENABLE | SDHC_RESP_LEN_136);
    if (r) { printf("FAIL: CMD2=%x\n", r); return 0; }

    // CMD3: SEND_RELATIVE_ADDR
    r = sdh_cmd(3, 0,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD3=%x\n", r); return 0; }
    uint32_t rca = (*reg32(SDH_BASE, SDHC_RESPONSE) >> 16) & 0xFFFF;

    // CMD7: SELECT_CARD
    r = sdh_cmd(7, rca << 16,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48_CHK_BUSY);
    if (r) { printf("FAIL: CMD7=%x\n", r); return 0; }

    // Card is selected -- switch from identification speed to full
    // data-transport speed. SD_CLOCK_EN must go back to 0 while the divider
    // is reprogrammed, same rule as the initial clock setup above.
    // freq_sel=1 -> divisor=ClkPreDiv(2)*2=4 -> 50MHz/4=12.5MHz: the fastest
    // rate confirmed working end-to-end (incl. ACMD6, below) against real
    // Genesys2 hardware in sdcard_test.c. The controller's CAPABILITIES
    // register reports HIGH_SPEED_SUPP=0 (no 50MHz mode implemented) and
    // freq_sel=0 (25MHz, the Default Speed spec ceiling) failed with a
    // CMD-line timeout on this board's wiring, so 12.5MHz is the real
    // practical ceiling here, not just a conservative guess.
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;                     // SD_CLOCK_EN=0
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(1); // 12.5MHz
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: sdh sdclk speedup\n"); return 0;
    }
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;

    // ACMD6 -- SET_BUS_WIDTH, so the *card* actually drives DAT1-3, then
    // switch the host to match. Setting only the host's own HOST_CTL
    // 4BIT_MODE bit (all this used to do) never told the card anything -- it
    // stays in its post-reset default of 1-bit mode, so it only ever drives
    // DAT0. A CMD24 write can still look like it "succeeds" that way (its
    // only card-driven feedback is the DAT0 CRC-status token either way),
    // but any read needs the card to drive all 4 DAT lines, which it never
    // does while still in 1-bit mode.
    r = sdh_cmd(55, rca << 16,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD55(acmd6)=%x\n", r); return 0; }
    r = sdh_cmd(6, 2 /* bus width: 4-bit */,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: ACMD6=%x\n", r); return 0; }

    // Switch host controller to 4-bit bus mode
    *reg8(SDH_BASE, SDHC_HOST_CTL) = SDHC_4BIT_MODE;

    // CMD16: SET_BLOCKLEN = 512
    r = sdh_cmd(16, 512,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD16=%x\n", r); return 0; }

    return rca;
}
