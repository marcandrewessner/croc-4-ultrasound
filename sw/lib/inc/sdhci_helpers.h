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

// SIM_CARD: 1 = building against rtl/test/sdcard/model/sdModel.v, 0 = real
// SDHC/SDXC card. Build with -DSIM_CARD=0 for silicon (this is the sw/
// Makefile's default; pass SIM_CARD=1 when building for the Verilator sim
// flow instead). This is the single switch for every place the model and a
// real card genuinely need different behaviour.
//
// What it gates here is sdh_card_is_block_addressed()'s answer: a real card
// says whatever its own OCR CCS bit says (see sdh_ccs below), but
// sdModel.v is the exception and needs an override -- it advertises CCS=1
// in its OCR but its CMD25 handler indexes FLASHmem with the raw argument
// as a byte address, so it claims high capacity and then behaves like a
// standard-capacity card. See sdh_card_is_block_addressed() for the detail.
//
// (This header used to also gate a CMD6 SWITCH_FUNC High Speed negotiation
// here, mirroring how sdModel.v has no SWITCH_FUNC handler at all and would
// hang on it -- see sdh_switch_func() below. That negotiation isn't called
// from sdh_init() right now: this design's SD clock generator can't reach
// 50MHz against the current 50MHz system clock regardless -- see sdh_init()
// -- so it would buy nothing yet. Kept defined for when that's revisited.)
#ifndef SIM_CARD
#define SIM_CARD 0
#endif

// Card Capacity Status from the ACMD41 response, valid after sdh_init().
// See sdh_card_is_block_addressed(). Header-static like everything else here,
// which is fine for the single-translation-unit programs that use it -- a
// multi-TU user would get one copy per TU and must call both in the same one.
static uint32_t sdh_ccs;

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

// Numeric step code instead of a unique string per checkpoint: the same
// reasoning as sw/sdcard_test.c's ok()/fail() -- this header is included
// into programs that also have their own logic, and sw/link.ld only gives
// each program 3584 bytes of SRAM (see its SRAM/STACK split) to work with.
// Grep this file for the step number to find the checkpoint. Always
// returns 0, matching what every sdh_init() call site wants to return on
// failure anyway.
static inline uint32_t sdh_fail(int step, uint32_t detail) {
    printf("SDH FAIL=%x:%x\n", step, detail);
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

// CMD6 SWITCH_FUNC. mode 0 = check (query support, no state change), mode 1 =
// set. Group 1 (access mode) function 1 = High Speed; all other groups are
// left at 0xF ("no change"). The card answers with a 64-byte status block on
// DAT, which must be read even for a check, or the data lines stay busy into
// the next command.
//
// status_o receives the 64-byte response (caller-provided, 16 words).
// Returns 0 on success, negative on timeout/error.
static inline int sdh_switch_func(int mode, uint32_t *status_o) {
    const uint32_t arg = ((uint32_t)(mode & 1) << 31) | 0x00FFFFF1u;

    if (sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK))
        return -1;
    *reg16(SDH_BASE, SDHC_BLOCK_SIZE)    = 64;
    *reg16(SDH_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDH_BASE, SDHC_ARGUMENT)      = arg;
    *reg16(SDH_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
    *reg16(SDH_BASE, SDHC_COMMAND) =
        (6 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_DATA_PRESENT_SELECT |
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48;

    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return -2;
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;

    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY))
        return -3;
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;

    for (int i = 0; i < 16; i++)
        status_o[i] = *reg32(SDH_BASE, SDHC_DATA);

    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE))
        return -4;
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    return 0;
}

// What SDCARD_ADDR_MODE.BLOCK_UNITS should be set to for the card that
// sdh_init() just brought up: 1 = block addressing, 0 = byte addressing.
//
// Against a real card this is the card's own answer (OCR CCS), not a build
// switch -- an SDHC/SDXC card says 1, a standard-capacity card says 0, and
// getting it wrong puts every session at 512x the intended offset.
//
// The model is the exception and needs the override: sdModel.v advertises
// CCS=1 in its OCR but its CMD25 handler indexes FLASHmem with the raw
// argument as a byte address (`BlockAddr = inCmd[39:8]`, `BlockAddr += 512`,
// `if (BlockAddr % 512 != 0) **Block Misalign Error`). So it claims high
// capacity and behaves like a standard-capacity card.
static inline uint32_t sdh_card_is_block_addressed(void) {
#if SIM_CARD
    return 0u;
#else
    return sdh_ccs;
#endif
}

// Full card bring-up: reset, identify, select, 4-bit bus, 512 B block length.
// Returns the 16-bit RCA on success, 0 on failure (reason printed).
static inline uint32_t sdh_init(void) {
    *reg8(SDH_BASE, SDHC_SOFTWARE_RESET) = SDHC_RESET_ALL;
    if (sdh_spin_until_clear8(SDHC_SOFTWARE_RESET, SDHC_RESET_ALL))
        return sdh_fail(1, 0);

    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return sdh_fail(2, 0);
    // Program the identification-speed divider with SD_CLOCK_EN still 0 --
    // sd_clk_generator only latches a new divider while the SD clock is
    // disabled -- then wait for it to load before starting the clock.
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(128);
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return sdh_fail(3, 0);
    // Bus power before the clock, then let the card see clock for a while
    // before the first command. The SD spec requires at least 74 SD clocks
    // supplied to the card after power is stable and before CMD0, and allows
    // up to 1 ms for the supply ramp; the model needs neither, a real card
    // needs both. 64 CLINT ticks is ~2 ms at 32 kHz, which covers the ramp and
    // is ~390 identification-speed clocks. Cheap once, at init.
    *reg8(SDH_BASE, SDHC_POWER_CTL)   =
        (SDHC_VOLTAGE_3_3V << SDHC_VOLTAGE_SHIFT) | SDHC_BUS_POWER;
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    clint_spin_ticks(64);
    *reg8(SDH_BASE, SDHC_TIMEOUT_CTL) = SDHC_TIMEOUT_MAX;
    *reg16(SDH_BASE, SDHC_NINTR_STATUS_EN) =
        SDHC_COMMAND_COMPLETE | SDHC_TRANSFER_COMPLETE |
        SDHC_BUFFER_WRITE_READY | SDHC_BUFFER_READ_READY | SDHC_ERROR_INTERRUPT;
    *reg16(SDH_BASE, SDHC_EINTR_STATUS_EN) = SDHC_EINTR_STATUS_MASK;

    // CMD0: GO_IDLE_STATE (no response)
    if (sdh_spin_until_clear32(SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD))
        return sdh_fail(4, 0);
    *reg32(SDH_BASE, SDHC_ARGUMENT) = 0;
    *reg16(SDH_BASE, SDHC_COMMAND)  = (0 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_NO_RESPONSE;
    if (sdh_spin_until_set16(SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE))
        return sdh_fail(5, 0);
    *reg16(SDH_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;

    // CMD8: SEND_IF_COND
    int r = sdh_cmd(8, 0x000001AA,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return sdh_fail(6, r);
    if ((*reg32(SDH_BASE, SDHC_RESPONSE) & 0xFF) != 0xAA)
        return sdh_fail(7, 0);

    // ACMD41 loop: wait for the card to finish powering up (OCR bit 31).
    // Bounded by time, not by iteration count: the spec gives the card up to
    // 1 second to leave initialisation, and real cards routinely take 100 ms
    // or more. (A fixed 200 x ~156 us = ~31 ms budget is enough only for the
    // model, which is ready on the first poll.)
    uint32_t ocr = 0;
    uint64_t ocr_deadline = clint_get_mtime() + 32000U; // ~1 s at 32 kHz
    for (;;) {
        r = sdh_cmd(55, 0,
            SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
        if (r) return sdh_fail(8, r);
        r = sdh_cmd(41, 0x40FF8000, SDHC_RESP_LEN_48);
        if (r) return sdh_fail(9, r);
        ocr = *reg32(SDH_BASE, SDHC_RESPONSE);
        if (ocr & (1u << 31)) break;
        if (clint_get_mtime() >= ocr_deadline) break;
        clint_spin_ticks(5);
    }
    if (!(ocr & (1u << 31))) return sdh_fail(10, ocr);

    // OCR bit 30 (CCS) is what decides how the card wants to be addressed:
    // 1 = SDHC/SDXC, arguments are block numbers; 0 = standard capacity,
    // arguments are byte offsets. This is negotiated, not a build-time
    // property of the design, so record it for the caller to program into
    // SDCARD_ADDR_MODE -- see sdh_card_is_block_addressed().
    sdh_ccs = (ocr >> 30) & 1u;

    // CMD2: ALL_SEND_CID
    r = sdh_cmd(2, 0, SDHC_CRC_CHECK_ENABLE | SDHC_RESP_LEN_136);
    if (r) return sdh_fail(11, r);

    // CMD3: SEND_RELATIVE_ADDR
    r = sdh_cmd(3, 0,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return sdh_fail(12, r);
    uint32_t rca = (*reg32(SDH_BASE, SDHC_RESPONSE) >> 16) & 0xFFFF;

    // CMD7: SELECT_CARD
    r = sdh_cmd(7, rca << 16,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48_CHK_BUSY);
    if (r) return sdh_fail(13, r);

    // ACMD6: SET_BUS_WIDTH = 4 bits. This tells the *card* to listen on
    // DAT[3:0]; the host-side HOST_CTL write below only changes what the
    // controller drives. Both are required and the card must be told first --
    // otherwise it keeps listening on DAT0 alone while the host spreads each
    // byte across four lines, and every data transfer is silently corrupt.
    // (rtl/test/sdcard/model/sdModel.v hides this: its data path hardcodes
    // 4-bit reception via `bitBlockRec = blockSize * 2` regardless of its own
    // BusWidth register, so simulation passed without this command.)
    // Sent while still in 1-bit mode at identification speed, which is where
    // the card expects it.
    r = sdh_cmd(55, rca << 16,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return sdh_fail(14, r);
    r = sdh_cmd(6, 2,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return sdh_fail(15, r);

    // Switch host controller to 4-bit bus mode (card already switched above)
    *reg8(SDH_BASE, SDHC_HOST_CTL) = SDHC_4BIT_MODE;

    // CMD6 SWITCH_FUNC (High Speed, 50MHz) negotiation deliberately dropped
    // here: this design's SD clock generator has a fixed ClkPreDiv=2 against
    // a 50MHz system clock (see the divider comment below), so the true
    // 50MHz High Speed rate is unreachable regardless of what the card
    // negotiates -- 25MHz is the hard local ceiling either way, and that
    // already failed with a CMD-line timeout on this board (see below). So
    // High Speed would buy nothing here without also revisiting the system
    // clock, which is a bigger, separate decision. sdh_switch_func() (CMD6
    // check/set, above) is kept for that future work but unused for now.

    // Card is configured -- switch from identification speed to full
    // data-transport speed. SD_CLOCK_EN must go back to 0 while the divider
    // is reprogrammed, same rule as the initial clock setup above.
    // freq_sel=1 -> divisor=ClkPreDiv(2)*2=4 -> 50MHz/4=12.5MHz: the fastest
    // rate confirmed working end-to-end (incl. ACMD6, above) against real
    // Genesys2 hardware in sdcard_test.c. The controller's CAPABILITIES
    // register reports HIGH_SPEED_SUPP=0 (no 50MHz mode implemented) and
    // freq_sel=0 (25MHz, the Default Speed spec ceiling) failed with a
    // CMD-line timeout on this board's wiring, so 12.5MHz is the real
    // practical ceiling here, not just a conservative guess.
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;                     // SD_CLOCK_EN=0
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(1); // 12.5MHz
    if (sdh_spin_until_set16(SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE))
        return sdh_fail(16, 0);
    *reg16(SDH_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;

    // CMD16: SET_BLOCKLEN = 512
    r = sdh_cmd(16, 512,
        SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) return sdh_fail(17, r);

    return rca;
}
