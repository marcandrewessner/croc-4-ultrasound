// ADC acquisition showcase -- continuous capture, no SD card.
//
// CONTINUOUS_ACQ_F0_F1 ping-pongs F0/F1 in SRAM: while one bank is being
// filled, the other (already full) is waiting for the CPU to read it out.
// This is the same double-buffering the SD-card path uses, except the
// consumer here is software (this program), not the HW copy engine -- see
// sdcard_acquisition_Nx.c for that variant. No SD card / SDHCI involved.
//
// Unlike SINGLE_ACQ_F0 (and ACQ_SDCARD), this mode never auto-reverts to
// IDLE: it runs until software decides it has enough frames and switches
// CONF.MODE back to IDLE itself.
//
// Flow:
//   1. Configure both F0 and F1 frame spans, reset the write head, enable
//      CONTINUOUS_ACQ_F0_F1.
//   2. Block on the completion interrupt (not polling) for each frame;
//      the ISR just records which bank(s) finished, main() does the
//      actual draining -- keep interrupt handlers short.
//   3. Dump whichever bank just filled, clear its Fx_FULL flag (this is
//      what lets hardware reuse that bank -- if software falls behind,
//      target_frame_full backpressure raises ADC_OVERFLOW and drops
//      samples rather than letting the ADC overwrite unread data).
//   4. After NUM_FRAMES, stop by writing CONF.MODE = IDLE.

#include "uart.h"
#include "print.h"
#include "util.h"
#include "config.h"
#include "adc_acquisition.h"

#define NUM_FRAMES  6u   // how many frames to capture before stopping

// ADC SRAM frame layout (croc_pkg.sv: Bank2=F0 @ 0x1000_1000, Bank3=F1 @ 0x1000_1800)
#define F0_START_ADDR_BYTE  0x10001000u
#define N_WORDS              128u  // 256 samples per frame
#define F0_END_ADDR_BYTE    (F0_START_ADDR_BYTE + (N_WORDS - 1u) * 4u)
#define F1_START_ADDR_BYTE  0x10001800u
#define F1_END_ADDR_BYTE    (F1_START_ADDR_BYTE + (N_WORDS - 1u) * 4u)

static inline uint32_t lo14(uint32_t w) { return w & 0x3FFFu; }
static inline uint32_t hi14(uint32_t w) { return (w >> 16) & 0x3FFFu; }

static volatile int f0_ready;
static volatile int f1_ready;

void croc_interrupt_handler(uint32_t cause) {
    if (cause != ADC_ACQ_INTERRUPT) return;
    uint32_t status = ADC_ACQ->STATUS;
    if (status & ADC_ACQ_STATUS_F0_FULL) f0_ready = 1;
    if (status & ADC_ACQ_STATUS_F1_FULL) f1_ready = 1;
}

static void dump_frame(uint32_t base_addr, const char *tag) {
    printf("BEGIN DUMP %s\n", tag);
    for (uint32_t i = 0; i < N_WORDS; i++) {
        uint32_t w = *reg32(base_addr, 4u * i);
        printf("%x: lo=%x hi=%x\n", i, lo14(w), hi14(w));
    }
    printf("END DUMP %s\n", tag);
}

int main(void) {
    uart_init();
    printf("acquisition_cont\n");

    set_interrupt_enable(1, ADC_ACQ_INTERRUPT);
    set_global_irq_enable(1);

    f0_ready = 0;
    f1_ready = 0;

    ADC_ACQ->F0_START_ADDR = F0_START_ADDR_BYTE;
    ADC_ACQ->F0_END_ADDR   = F0_END_ADDR_BYTE;
    ADC_ACQ->F1_START_ADDR = F1_START_ADDR_BYTE;
    ADC_ACQ->F1_END_ADDR   = F1_END_ADDR_BYTE;
    ADC_ACQ->CNTRL         = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF          = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1;

    for (uint32_t frames_done = 0; frames_done < NUM_FRAMES; ) {
        while (!f0_ready && !f1_ready) { }

        if (f0_ready) {
            f0_ready = 0;
            dump_frame(F0_START_ADDR_BYTE, "F0");
            ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT);
            frames_done++;
        }
        if (f1_ready) {
            f1_ready = 0;
            dump_frame(F1_START_ADDR_BYTE, "F1");
            ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);
            frames_done++;
        }
    }

    // Stop acquisition -- this mode never stops itself.
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

    if (ADC_ACQ->STATUS & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");

    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS;
    printf("acquisition_cont done\n");
    uart_write_flush();
    return 0;
}
