// ADC acquisition showcase -- single-shot capture, no SD card.
//
// Simplest possible use of the ADC acquisition peripheral: capture one
// frame into SRAM Bank2 (F0) and dump it over UART. No SD card / SDHCI
// involved at all -- see sdcard_acquisition_Nx.c for that.
//
// Flow:
//   1. Arm SINGLE_ACQ_F0: configure the F0 frame span, reset the write
//      head, enable the mode. Hardware fills F0 word-by-word as ADC
//      samples arrive and autonomously reverts CONF.MODE to IDLE the
//      instant the frame is full -- no software polling loop needed for
//      that part.
//   2. Wait for the completion interrupt (interrupt_frame_full_o, wired to
//      ADC_ACQ_INTERRUPT) instead of polling STATUS -- this mode supports
//      both, this example showcases the interrupt path. The line is level-
//      sensitive with no hardware ack, so the ISR clears the STATUS flags
//      that raised it; see croc_interrupt_handler below.
//   3. Dump the captured words: each 32-bit word packs two consecutive
//      14-bit ADC samples, {2'b00, sample[2i+1], 2'b00, sample[2i]}.
//   4. Re-arm once more to show the peripheral isn't one-shot -- CONF.MODE
//      reverting to IDLE only stops the *current* capture, SINGLE_ACQ_F0
//      can be written again right away.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "config.h"
#include "adc_acquisition.h"

// Fill the whole of Bank2: 512 words = 2 KiB = 1024 ADC samples, the largest
// frame a single bank can hold (see ADC_ACQ_BANK_WORDS in adc_acquisition.h
// for where that number comes from).
#define F0_START_ADDR_BYTE  ADC_ACQ_F0_BASE
#define N_WORDS             ADC_ACQ_BANK_WORDS
#define F0_END_ADDR_BYTE    ADC_ACQ_FRAME_END(F0_START_ADDR_BYTE, N_WORDS)

ADC_ACQ_ASSERT_FRAME_FITS(N_WORDS);

static inline uint32_t lo14(uint32_t w) { return w & 0x3FFFu; }
static inline uint32_t hi14(uint32_t w) { return (w >> 16) & 0x3FFFu; }

static volatile int frame_ready;
static volatile uint32_t frame_status;   // STATUS snapshot the ISR acted on

void croc_interrupt_handler(uint32_t cause) {
    if (cause != ADC_ACQ_INTERRUPT) return;

    uint32_t status = ADC_ACQ->STATUS;
    frame_status = status;
    if (status & ADC_ACQ_STATUS_F0_FULL) frame_ready = 1;

    // ADC_ACQ_INTERRUPT is level-sensitive: interrupt_frame_full_o is a plain
    // OR of every STATUS flag, held until software clears them, and CVE2 wires
    // it straight into mip.irq_fast with no edge detect or hardware ack. So
    // the ISR *must* deassert it before returning -- leave a flag set and mret
    // walks straight back into the trap handler, so main() retires only one
    // instruction per round trip. That looks exactly like "the interrupt never
    // fired". test_interrupts.c does the same thing via obi_timer_clear_expired().
    //
    // One store clears everything: CLEAR_F0_FULL/CLEAR_F1_FULL have their own
    // bits and CLEAR_STATUS covers ADC_OVERFLOW plus the two SD-card flags.
    // All CNTRL bits are singlepulse and RESET_WRITE_HEAD stays 0, so the
    // write head is untouched.
    //
    // Clearing from the ISR is race-free in SINGLE_ACQ_F0 specifically:
    // hardware reverted CONF.MODE to IDLE in the same cycle it raised F0_FULL
    // (adc_acquisition_top.sv), so nothing can re-raise a flag between the
    // read above and this write, and nothing writes F0 while main() dumps it.
    ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT)
                   | (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT)
                   | ADC_ACQ_CTRL_CLEAR_STATUS;
}

// Arms F0 and blocks (via IRQ, not polling) until it's full.
static void run_acquisition(void) {
    frame_ready  = 0;
    frame_status = 0;

    ADC_ACQ->F0_START_ADDR = F0_START_ADDR_BYTE;
    ADC_ACQ->F0_END_ADDR   = F0_END_ADDR_BYTE;
    ADC_ACQ->CNTRL         = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF          = ADC_ACQ_MODE_SINGLE_ACQ_F0;

    while (!frame_ready) { }
}

static void dump_frame(void) {
    printf("BEGIN DUMP\n");
    for (uint32_t i = 0; i < N_WORDS; i++) {
        uint32_t w = *reg32(F0_START_ADDR_BYTE, 4u * i);
        printf("%x: lo=%x hi=%x\n", i, lo14(w), hi14(w));
    }
    printf("END DUMP\n");
}

int main(void) {
    uart_init();
    printf("acquisition_single\n");

    set_interrupt_enable(1, ADC_ACQ_INTERRUPT);
    set_global_irq_enable(1);

    // --- run 1 ---
    run_acquisition();
    printf("F0 full\n");
    dump_frame();

    // --- run 2: same peripheral, re-armed -------------------------------
    run_acquisition();
    printf("F0 full (re-armed)\n");
    dump_frame();

    // The ISR clears ADC_OVERFLOW along with everything else, so the only
    // place it is still visible is the STATUS snapshot it took.
    if (frame_status & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");

    printf("acquisition_single done\n");
    uart_write_flush();
    return 0;
}
