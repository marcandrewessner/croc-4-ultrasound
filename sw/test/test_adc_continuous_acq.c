// Copyright (c) 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner
//
// Functional test for CONF.MODE = CONTINUOUS_ACQ_F0_F1: verifies the F0/F1
// ping-pong alternates in the right order, the interrupt fires for each
// frame, the write head lands in the right bank each time, and each frame's
// data is internally consistent.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "config.h"
#include "adc_acquisition.h"

#define F0_BASE   0x10001000u
#define F1_BASE   0x10001800u
#define FRAME_N   16u                              // words per frame
#define F0_END    (F0_BASE + (FRAME_N - 1u) * 4u)
#define F1_END    (F1_BASE + (FRAME_N - 1u) * 4u)
#define N_CYCLES  3u                                // F0,F1 pairs to capture

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

// The ISR immediately parks the peripheral in IDLE and clears the Fx_FULL
// flag the instant a frame completes -- both are required for correctness,
// not just style:
//
//  - Clearing Fx_FULL inside the ISR (rather than after we've read the
//    frame back) is required because interrupt_frame_full_o is a level, not
//    an edge: leaving Fx_FULL set until the main loop gets around to
//    clearing it would let the still-pending condition re-trap the instant
//    this handler returns, double-counting the same frame.
//  - Forcing MODE=IDLE is required because CONTINUOUS mode keeps ping-
//    ponging the instant a frame is marked full, and reading a whole frame
//    back over OBI from the main loop is *slower* than hardware refilling
//    the other bank at these clock ratios (12 MHz ADC / 2 samples-per-word
//    vs. the CPU's per-word OBI read overhead) -- without pausing, the bank
//    we're still verifying could start getting overwritten underneath us.
//
// Because the CDC FIFO is drained unconditionally every cycle
// (dst_ready_i=1 in adc_acquisition_top), pausing does drop a few samples
// across the boundary. So this test checks progression *within* each frame,
// not across the F0->F1 splice -- that splice is exercised functionally
// (address handoff, alternation order) but not for sample-level continuity.
static volatile int      frame_full_which = -1; // 0 or 1
static volatile uint32_t frame_full_count = 0;

void croc_interrupt_handler(uint32_t cause) {
    if (cause != ADC_ACQ_INTERRUPT) return;
    uint32_t status = ADC_ACQ->STATUS;
    if (status & ADC_ACQ_STATUS_F0_FULL) {
        ADC_ACQ->CONF  = ADC_ACQ_MODE_IDLE;
        ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);
        frame_full_which = 0;
        frame_full_count++;
    } else if (status & ADC_ACQ_STATUS_F1_FULL) {
        ADC_ACQ->CONF  = ADC_ACQ_MODE_IDLE;
        ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);
        frame_full_which = 1;
        frame_full_count++;
    }
}

static int wait_for_frame(uint32_t expect_count) {
    for (volatile uint32_t i = 0; i < 300000u; i++) {
        if (frame_full_count >= expect_count) return 1;
    }
    return 0;
}

int main(void) {
    uart_init();
    printf("test_adc_continuous_acq\n");

    set_interrupt_enable(1, ADC_ACQ_INTERRUPT);
    set_global_irq_enable(1);

    ADC_ACQ->F0_START_ADDR = F0_BASE;
    ADC_ACQ->F0_END_ADDR   = F0_END;
    ADC_ACQ->F1_START_ADDR = F1_BASE;
    ADC_ACQ->F1_END_ADDR   = F1_END;
    ADC_ACQ->CNTRL         = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF          = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1;

    for (uint32_t cycle = 0; cycle < N_CYCLES; cycle++) {
        // --- F0 fills, hands off to F1 --------------------------------
        CHECK_ASSERT(1 + cycle * 10, wait_for_frame(2 * cycle + 1));
        CHECK_ASSERT(2 + cycle * 10, frame_full_which == 0);
        CHECK_ASSERT(3 + cycle * 10, ADC_ACQ->WRITE_HEAD == F1_BASE);
        CHECK_ASSERT(4 + cycle * 10, check_progression((volatile uint32_t *)F0_BASE, FRAME_N));
        ADC_ACQ->CONF = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1; // resume into F1

        // --- F1 fills, hands off back to F0 -----------------------------
        CHECK_ASSERT(5 + cycle * 10, wait_for_frame(2 * cycle + 2));
        CHECK_ASSERT(6 + cycle * 10, frame_full_which == 1);
        CHECK_ASSERT(7 + cycle * 10, ADC_ACQ->WRITE_HEAD == F0_BASE);
        CHECK_ASSERT(8 + cycle * 10, check_progression((volatile uint32_t *)F1_BASE, FRAME_N));
        ADC_ACQ->CONF = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1; // resume into F0
    }

    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;
    set_interrupt_enable(0, ADC_ACQ_INTERRUPT);
    set_global_irq_enable(0);

    printf("test_adc_continuous_acq OK\n");
    uart_write_flush();
    return 0;
}
