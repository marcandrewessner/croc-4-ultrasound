// SDCard acquisition program -- repeated pulse ("burst") capture benchmark.
//
// Same SDCARD_PULSE mechanism as sdcard_acquisition_pulse.c (see that file
// for the full mode explanation) -- this variant exists to characterise how
// fast pulses can actually be repeated back to back, i.e. the achievable
// Pulse Repetition Frequency (PRF), not just to demonstrate one pulse
// working. It runs N_PULSES back to back, times the whole run with a single
// clint_get_mtime() pair around the loop, and reports PRF = N_PULSES /
// elapsed_time at the end.
//
// "Optimized" here specifically means: minimize software-imposed overhead
// between the moment one pulse's SDCARD_DONE lands and the next pulse's
// CONF write goes out, since every cycle spent there is a cycle PRF pays
// for that the hardware itself didn't need. Two concrete changes from
// sdcard_acquisition_pulse.c's loop:
//   1. The re-arm sequence's two CNTRL writes (clear status/Fx_FULL, then
//      reset write head) are combined into one. adc_acquisition_top.sv's
//      control-logic always_comb block (rtl/adc_acquisition/
//      adc_acquisition_top.sv) handles CNTRL.RESET_WRITE_HEAD,
//      CNTRL.CLEAR_F0_FULL, CNTRL.CLEAR_F1_FULL and CNTRL.CLEAR_STATUS as
//      independent if-blocks writing disjoint hw2reg fields (WRITE_HEAD vs.
//      STATUS.F0_FULL vs. STATUS.F1_FULL vs. STATUS.{ADC_OVERFLOW,
//      SDCARD_DONE,SDCARD_OVERFLOW}) -- there is no ordering hazard between
//      them, so setting all four bits in one write is equivalent to two
//      sequential writes, minus one full OBI round trip per pulse.
//   2. The terminal-status poll only calls clint_get_mtime() (a second OBI
//      read, on top of the STATUS read every loop needs anyway) once every
//      256 iterations instead of every iteration, cutting the poll loop's
//      register traffic roughly in half without materially loosening the
//      timeout bound (250000-tick/~7.5s budget vs. a few hundred iterations
//      of slack -- negligible).
// Nothing else changes: same init, same static configuration hoisted out of
// the loop, same re-arm semantics (SDCARD_BLOCK_ADDR left advancing so
// consecutive pulses land back to back on the card), same terminal
// conditions. No printf happens inside the timed region -- UART transmit
// time is real wall-clock time and would otherwise leak into the PRF
// measurement.

#include "uart.h"
#include "clint.h"
#include "util.h"
#include "print.h"
#include "adc_acquisition.h"
#include "sdhci_helpers.h"

// Number of back-to-back pulses to time. Override on the command line like
// N_BLOCKS/SIM_CARD, e.g. `make N_PULSES=1000 sdcard_acquisition_pulse_rep...`.
#ifndef N_PULSES
#define N_PULSES  100
#endif

#define TIMEOUT       250000U  // CLINT ticks (~7.5 s at 32 kHz) -- bounds one pulse
#define POLL_MTIME_EVERY_MASK 0xFFu  // check the clock every 256 poll iterations

// RTC frequency feeding clint_get_mtime() -- see sw/lib/src/clint.c's
// clint_sleep_ms() ("RTC frequency: 32.768 kHz"). Used to convert elapsed
// ticks into a PRF in Hz.
#define RTC_HZ  32768u

// ADC SRAM layout -- identical to sdcard_acquisition_pulse.c, see there for
// the full derivation and the static_assert this mirrors.
#define SD_BLOCK_BYTES      512u
#define N_WORDS             ADC_ACQ_BANK_WORDS               // 512 words = 2 KiB per bank
#define FRAME_BYTES         (N_WORDS * 4u)
#define PULSE_BYTES         (2u * FRAME_BYTES)               // both banks = 4 KiB
#define BLOCKS_PER_PULSE    (PULSE_BYTES / SD_BLOCK_BYTES)   // 8
#define F0_START_ADDR_BYTE  ADC_ACQ_F0_BASE
#define F0_END_ADDR_BYTE    ADC_ACQ_FRAME_END(F0_START_ADDR_BYTE, N_WORDS)
#define F1_START_ADDR_BYTE  ADC_ACQ_F1_BASE
#define F1_END_ADDR_BYTE    ADC_ACQ_FRAME_END(F1_START_ADDR_BYTE, N_WORDS)

ADC_ACQ_ASSERT_FRAME_FITS(N_WORDS);
_Static_assert(BLOCKS_PER_PULSE * SD_BLOCK_BYTES == PULSE_BYTES,
               "a pulse must be a whole number of SD blocks");

#define SDCARD_START_ADDR    0u

// Combined re-arm mask for the single CNTRL write -- see file header comment
// point 1. RST_WRITE_HEAD is a bit position (like the two CLR_* bits),
// CLEAR_STATUS is already pre-shifted (see adc_acquisition.h), matching how
// sdcard_acquisition_pulse.c uses each individually.
#define REARM_CNTRL_MASK \
    (ADC_ACQ_CTRL_CLEAR_STATUS               | \
     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT)    | \
     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT)    | \
     (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT))

// printf (sw/lib/src/print.c) only implements %x -- no %d/%u/%f. PRF reads
// far more usefully in decimal, so convert by hand rather than dumping a hex
// Hz value.
static void print_dec_u32(uint32_t v) {
    char buf[10];
    int  i = 0;
    if (v == 0) {
        putchar('0');
        return;
    }
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0)
        putchar(buf[--i]);
}

// Prints a hundredths-fixed-point value, e.g. centi=12345 -> "123.45".
static void print_dec_centi(uint32_t centi) {
    print_dec_u32(centi / 100);
    putchar('.');
    uint32_t frac = centi % 100;
    if (frac < 10)
        putchar('0');
    print_dec_u32(frac);
}

int main(void) {
    uart_init();
    printf("SDCard pulse-rate benchmark, N_PULSES=%x\n", N_PULSES);

    // 1. Initialise SD card
    if (!sdh_init()) {
        printf("SD init failed\n");
        uart_write_flush();
        return 1;
    }

    // 2. Static configuration, identical for every pulse -- hoisted out of
    //    the loop, same as sdcard_acquisition_pulse.c.
    ADC_ACQ->SDCARD_BLOCK_SIZE  = SD_BLOCK_BYTES;
    ADC_ACQ->SDCARD_BLOCK_COUNT = BLOCKS_PER_PULSE;
    ADC_ACQ->SDCARD_BLOCK_ADDR  = SDCARD_START_ADDR;
    const uint32_t addr_is_blocks = sdh_card_is_block_addressed();
    ADC_ACQ->SDCARD_ADDR_MODE   = addr_is_blocks;
    ADC_ACQ->F0_START_ADDR      = F0_START_ADDR_BYTE;
    ADC_ACQ->F0_END_ADDR        = F0_END_ADDR_BYTE;
    ADC_ACQ->F1_START_ADDR      = F1_START_ADDR_BYTE;
    ADC_ACQ->F1_END_ADDR        = F1_END_ADDR_BYTE;

    uint32_t pulses_done = 0;
    uint32_t status      = 0;
    int      timed_out   = 0;

    // 3. Timed region starts here -- nothing above this point is part of
    //    the rate measurement (SD init and static config are one-time
    //    setup cost, not per-pulse cost).
    uint64_t t_start = clint_get_mtime();

    for (uint32_t p = 0; p < N_PULSES; p++) {
        ADC_ACQ->CNTRL = REARM_CNTRL_MASK;
        ADC_ACQ->CONF  = ADC_ACQ_MODE_SDCARD_PULSE;

        uint64_t end        = clint_get_mtime() + TIMEOUT;
        uint32_t poll_count  = 0;
        timed_out = 0;
        for (;;) {
            status = ADC_ACQ->STATUS;
            if (status & (ADC_ACQ_STATUS_SDCARD_DONE | ADC_ACQ_STATUS_SDCARD_OVERFLOW))
                break;
            if ((poll_count++ & POLL_MTIME_EVERY_MASK) == 0 &&
                clint_get_mtime() >= end) {
                timed_out = 1;
                break;
            }
        }

        ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

        if (timed_out || (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW))
            break;
        pulses_done++;
    }

    uint64_t t_end = clint_get_mtime();
    // 3b. Timed region ends here.

    // Diagnostic for the timeout path -- same reasoning as
    // sdcard_acquisition_pulse.c: F0_FULL/F1_FULL tell us whether the ADC
    // ever finished capturing this pulse, i.e. whether the stall is
    // upstream (ADC capture) or in the SD copy engine. Printed once, after
    // the timed region, so it cannot skew the measurement.
    if (timed_out)
        printf("STATUS=%x F0_FULL=%x F1_FULL=%x\n", status,
               (status & ADC_ACQ_STATUS_F0_FULL) != 0,
               (status & ADC_ACQ_STATUS_F1_FULL) != 0);

    uint32_t addr_advanced  = ADC_ACQ->SDCARD_BLOCK_ADDR - SDCARD_START_ADDR;
    uint32_t addr_per_pulse = addr_is_blocks ? BLOCKS_PER_PULSE : PULSE_BYTES;
    uint32_t pulses_on_card = addr_advanced / addr_per_pulse;

    if (timed_out)
        printf("FAIL: pulse timeout after %x/%x pulses\n", pulses_done, N_PULSES);
    if (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW)
        printf("FAIL: SDCARD_OVERFLOW after %x/%x pulses\n", pulses_done, N_PULSES);
    if (status & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");

    // 4. Report: elapsed time, and PRF = pulses_done / elapsed_time.
    //    Deliberately 32-bit-only arithmetic: this target is rv32i_zicsr,
    //    no M extension, so even 32-bit multiply/divide are libgcc soft
    //    routines (__mulsi3/__udivsi3) -- already paid for elsewhere in
    //    this codebase (e.g. addr_per_pulse above). uint64_t multiply/
    //    divide (__muldi3/__udivdi3/__umoddi3) are a different, much
    //    larger set of routines and were the actual cause of a 1804-byte
    //    SRAM overflow when this was first written with them -- avoided
    //    by splitting every scaled ratio into a whole part and a
    //    remainder part *before* multiplying by 100, which keeps every
    //    intermediate value within 32 bits for any realistic N_PULSES/
    //    runtime instead of computing the wide product directly.
    uint32_t elapsed_ticks = (uint32_t)(t_end - t_start);

    if (pulses_done > 0 && elapsed_ticks > 0) {
        // elapsed_ms = elapsed_ticks * 1000 / RTC_HZ, split so the *1000
        // never sees more than one RTC_HZ period's worth of ticks.
        uint32_t whole_s   = elapsed_ticks / RTC_HZ;
        uint32_t rem_ticks = elapsed_ticks % RTC_HZ;
        uint32_t elapsed_ms_centi = whole_s * 100000u + (rem_ticks * 100000u) / RTC_HZ;
        printf("elapsed=");
        print_dec_centi(elapsed_ms_centi);
        printf(" ms\n");

        // PRF = pulses_done * RTC_HZ / elapsed_ticks, same split: divide
        // first, then scale only the remainder by 100.
        uint32_t scaled    = pulses_done * RTC_HZ;
        uint32_t prf_whole = scaled / elapsed_ticks;
        uint32_t prf_rem   = scaled % elapsed_ticks;
        uint32_t prf_centihz = prf_whole * 100u + (prf_rem * 100u) / elapsed_ticks;
        printf("PRF=");
        print_dec_centi(prf_centihz);
        printf(" Hz\n");
    } else {
        printf("PRF: not enough completed pulses to measure\n");
    }

    // SDCARD_BLOCK_ADDR is set once before the loop and never touched again
    // in software -- every completed pulse advances it by one session's
    // worth in hardware (adc_acquisition_sdcard_controller.sv's CE_DONE,
    // block_addr_incr_o), so consecutive pulses land back to back with no
    // gap. addr_advanced (already computed above) is exactly the span all
    // pulses_on_card pulses cover, in whatever unit SDCARD_ADDR_MODE
    // selected -- print it explicitly as [start, end) instead of leaving
    // the contiguity implicit.
    // print.c's printf only implements %x (no %s) -- addr_is_blocks is
    // printed as the raw 0/1 flag rather than a "blocks"/"bytes" word.
    if (pulses_done == N_PULSES) {
        uint32_t end_addr = SDCARD_START_ADDR + addr_advanced;
        printf("OK: %x pulses x %x blocks x %x B = one continuous region, "
               "addr %x..%x (addr_is_blocks=%x)\n",
               pulses_on_card, BLOCKS_PER_PULSE, SD_BLOCK_BYTES,
               SDCARD_START_ADDR, end_addr, addr_is_blocks);
    }

    // Clear all ADC status flags
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);

    uart_write_flush();
    return 0;
}
