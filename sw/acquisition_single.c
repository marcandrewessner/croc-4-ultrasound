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
//      both, this example showcases the interrupt path.
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

#define F0_START_ADDR_BYTE  0x10001000u
#define N_WORDS              64u   // 128 samples
#define F0_END_ADDR_BYTE    (F0_START_ADDR_BYTE + (N_WORDS - 1u) * 4u)

static inline uint32_t lo14(uint32_t w) { return w & 0x3FFFu; }
static inline uint32_t hi14(uint32_t w) { return (w >> 16) & 0x3FFFu; }

static volatile int frame_ready;

void croc_interrupt_handler(uint32_t cause) {
    if (cause == ADC_ACQ_INTERRUPT && (ADC_ACQ->STATUS & ADC_ACQ_STATUS_F0_FULL)) {
        frame_ready = 1;
        // Leave F0_FULL set for main() to consume; clearing it here would
        // race main()'s own read of STATUS below.
    }
}

// Arms F0 and blocks (via IRQ, not polling) until it's full.
static void run_acquisition(void) {
    frame_ready = 0;

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
    ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);

    // --- run 2: same peripheral, re-armed -------------------------------
    run_acquisition();
    printf("F0 full (re-armed)\n");
    dump_frame();
    ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);

    printf("acquisition_single done\n");
    uart_write_flush();
    return 0;
}
