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

// Print key registers: NINTR, EINTR, PRESENT_STATE (all %x, no %s)
static void print_regs(void) {
    uint16_t n  = *reg16(SDHCI_BASE, SDHC_NINTR_STATUS);
    uint16_t e  = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
    uint32_t ps = *reg32(SDHCI_BASE, SDHC_PRESENT_STATE);
    printf("NINTR=%x EINTR=%x PS=%x\n", (uint32_t)n, (uint32_t)e, ps);
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
    if (caps == 0 || caps == 0xffffffff) {
        printf("FAIL: caps\n");
        uart_write_flush();
        return 1;
    }

    // -----------------------------------------------------------------------
    // 1. Controller reset and clock setup
    // -----------------------------------------------------------------------
    *reg8(SDHCI_BASE, SDHC_SOFTWARE_RESET) = SDHC_RESET_ALL;
    if (spin_until_clear8(SDHCI_BASE, SDHC_SOFTWARE_RESET, SDHC_RESET_ALL)) {
        printf("FAIL: reset\n");
        uart_write_flush();
        return 2;
    }

    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: clk\n");
        uart_write_flush();
        return 3;
    }

    // Program the identification-speed divider (sys/128) with SD_CLOCK_EN
    // still 0 -- sd_clk_generator only latches a new divider while the SD
    // clock is disabled -- then wait for it to load before starting the
    // clock. Safe init frequency per SD spec.
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(128);
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: sdclk div\n");
        uart_write_flush();
        return 3;
    }
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
    printf("Setup OK\n");

    // -----------------------------------------------------------------------
    // 2. CMD0 – GO_IDLE_STATE (no response)
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_CMD)) {
        printf("FAIL: CMD0 inh\n");
        uart_write_flush();
        return 4;
    }
    *reg32(SDHCI_BASE, SDHC_ARGUMENT) = 0;
    *reg16(SDHCI_BASE, SDHC_COMMAND)  = (0 << SDHC_COMMAND_INDEX_SHIFT) | SDHC_NO_RESPONSE;
    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE)) {
        printf("FAIL: CMD0 to\n");
        uart_write_flush();
        return 5;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    printf("CMD0 OK\n");

    // -----------------------------------------------------------------------
    // 3. CMD8 – SEND_IF_COND
    // -----------------------------------------------------------------------
    int r = sdhci_cmd(8, 0x000001AA,
                      SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD8=%x\n", r); uart_write_flush(); return 6; }
    uint32_t r7 = *reg32(SDHCI_BASE, SDHC_RESPONSE);
    if ((r7 & 0xFF) != 0xAA) {
        printf("FAIL: CMD8 echo=%x\n", r7);
        uart_write_flush();
        return 7;
    }
    printf("CMD8 OK r7=%x\n", r7);

    // -----------------------------------------------------------------------
    // 4. CMD55 + ACMD41 loop — wait for OCR[31]=1
    // -----------------------------------------------------------------------
    uint32_t ocr = 0;
    for (int i = 0; i < 200; i++) {
        r = sdhci_cmd(55, 0,
                      SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
        if (r) { printf("FAIL: CMD55=%x\n", r); uart_write_flush(); return 8; }
        r = sdhci_cmd(41, 0x40FF8000, SDHC_RESP_LEN_48);
        if (r) { printf("FAIL: ACMD41=%x\n", r); uart_write_flush(); return 9; }
        ocr = *reg32(SDHCI_BASE, SDHC_RESPONSE);
        if (ocr & (1u << 31)) break;
        clint_spin_ticks(5);
    }
    if (!(ocr & (1u << 31))) {
        printf("FAIL: OCR=%x\n", ocr);
        uart_write_flush();
        return 10;
    }
    printf("ACMD41 OK OCR=%x\n", ocr);

    // -----------------------------------------------------------------------
    // 5. CMD2 – ALL_SEND_CID
    // -----------------------------------------------------------------------
    r = sdhci_cmd(2, 0, SDHC_CRC_CHECK_ENABLE | SDHC_RESP_LEN_136);
    if (r) { printf("FAIL: CMD2=%x\n", r); uart_write_flush(); return 11; }

    // -----------------------------------------------------------------------
    // 6. CMD3 – SEND_RELATIVE_ADDR
    // -----------------------------------------------------------------------
    r = sdhci_cmd(3, 0,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD3=%x\n", r); uart_write_flush(); return 12; }
    uint32_t rca = (*reg32(SDHCI_BASE, SDHC_RESPONSE) >> 16) & 0xFFFF;
    printf("RCA=%x\n", rca);

    // -----------------------------------------------------------------------
    // 7. CMD7 – SELECT_CARD (R1b)
    // -----------------------------------------------------------------------
    r = sdhci_cmd(7, rca << 16,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48_CHK_BUSY);
    if (r) { printf("FAIL: CMD7=%x\n", r); uart_write_flush(); return 13; }
    printf("Card ready RCA=%x\n", rca);

    // -----------------------------------------------------------------------
    // Card is selected -- switch from identification speed to full
    // data-transport speed. SD_CLOCK_EN must go back to 0 while the divider
    // is reprogrammed, same rule as the initial clock setup above.
    // -----------------------------------------------------------------------
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE;                     // SD_CLOCK_EN=0
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) = SDHC_INTCLK_ENABLE | SDHC_SDCLK_DIV(0); // fastest divider
    if (spin_until_set16(SDHCI_BASE, SDHC_CLOCK_CTL, SDHC_INTCLK_STABLE)) {
        printf("FAIL: sdclk speedup\n");
        uart_write_flush();
        return 13;
    }
    *reg16(SDHCI_BASE, SDHC_CLOCK_CTL) |= SDHC_SDCLK_ENABLE;
    printf("SD clock switched to full speed\n");

    // -----------------------------------------------------------------------
    // 8. Switch host to 4-bit bus mode (sdModel always uses 4-bit DAT)
    // -----------------------------------------------------------------------
    *reg8(SDHCI_BASE, SDHC_HOST_CTL) = SDHC_4BIT_MODE;
    printf("4-bit mode set\n");

    // -----------------------------------------------------------------------
    // 9. CMD16 – SET_BLOCKLEN = 512
    //    sdModel never initializes blockSize, Verilator sets it to 0.
    //    Without CMD16, the card sends/receives 0 data bytes per transfer.
    // -----------------------------------------------------------------------
    r = sdhci_cmd(16, 512,
                  SDHC_CRC_CHECK_ENABLE | SDHC_INDEX_CHECK_ENABLE | SDHC_RESP_LEN_48);
    if (r) { printf("FAIL: CMD16=%x\n", r); uart_write_flush(); return 14; }
    printf("CMD16 OK blocklen=512\n");
    print_regs();

    // -----------------------------------------------------------------------
    // 10. Write 512 bytes (counter 0..127) to block 0  (CMD24 – WRITE_BLOCK)
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK)) {
        printf("FAIL: inh-wr\n");
        uart_write_flush();
        return 15;
    }

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

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE)) {
        printf("FAIL: CMD24 cmd-to\n");
        uart_write_flush();
        return 16;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    printf("CMD24 cmd ok\n");

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_BUFFER_WRITE_READY)) {
        printf("FAIL: wr-buf-to\n");
        print_regs();
        uart_write_flush();
        return 17;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_WRITE_READY;
    printf("Wr buf ready\n");

    for (int i = 0; i < 128; i++)
        *reg32(SDHCI_BASE, SDHC_DATA) = (uint32_t)i;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_TRANSFER_COMPLETE)) {
        printf("FAIL: wr-xfr-to\n");
        print_regs();
        uart_write_flush();
        return 18;
    }
    print_regs();
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;
    printf("Write OK\n");

    // -----------------------------------------------------------------------
    // 11. Read block 0 back  (CMD17 – READ_SINGLE_BLOCK) and verify
    // -----------------------------------------------------------------------
    if (spin_until_clear32(SDHCI_BASE, SDHC_PRESENT_STATE, SDHC_CMD_INHIBIT_MASK)) {
        printf("FAIL: inh-rd\n");
        uart_write_flush();
        return 19;
    }

    *reg16(SDHCI_BASE, SDHC_BLOCK_SIZE)    = 512;
    *reg16(SDHCI_BASE, SDHC_BLOCK_COUNT)   = 1;
    *reg32(SDHCI_BASE, SDHC_ARGUMENT)      = 0;
    *reg16(SDHCI_BASE, SDHC_TRANSFER_MODE) = SDHC_READ_MODE;
    *reg16(SDHCI_BASE, SDHC_COMMAND) =
        (17 << SDHC_COMMAND_INDEX_SHIFT) |
        SDHC_DATA_PRESENT_SELECT         |
        SDHC_CRC_CHECK_ENABLE            |
        SDHC_INDEX_CHECK_ENABLE          |
        SDHC_RESP_LEN_48;

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_COMMAND_COMPLETE)) {
        printf("FAIL: CMD17 cmd-to\n");
        uart_write_flush();
        return 20;
    }
    // Check for errors at command phase
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("CMD17 cmd err EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_COMMAND_COMPLETE;
    printf("CMD17 cmd ok\n");
    print_regs();

    if (spin_until_set16(SDHCI_BASE, SDHC_NINTR_STATUS, SDHC_BUFFER_READ_READY)) {
        printf("FAIL: rd-buf-to\n");
        print_regs();
        uart_write_flush();
        return 21;
    }
    // Check for CRC/end-bit errors before reading
    if (*reg16(SDHCI_BASE, SDHC_NINTR_STATUS) & SDHC_ERROR_INTERRUPT) {
        uint16_t e = *reg16(SDHCI_BASE, SDHC_EINTR_STATUS);
        printf("Read data err EINTR=%x\n", (uint32_t)e);
        *reg16(SDHCI_BASE, SDHC_EINTR_STATUS) = e;
        *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_ERROR_INTERRUPT;
    }
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_BUFFER_READ_READY;
    printf("Rd buf ready\n");
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
        printf("FAIL: rd-xfr-to\n");
        print_regs();
        uart_write_flush();
        return 22;
    }
    print_regs();
    *reg16(SDHCI_BASE, SDHC_NINTR_STATUS) = SDHC_TRANSFER_COMPLETE;

    if (errors) {
        printf("FAIL: %x mismatches\n", errors);
        uart_write_flush();
        return 23;
    }

    printf("Success\n");
    uart_write_flush();
    return 0;
}
