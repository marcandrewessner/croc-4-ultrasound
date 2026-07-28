// SDCard acquisition program -- one-shot burst ("pulse") capture.
//
// SDCARD_PULSE is the non-overlapped counterpart to SDCARD_CONTINUOUS (see
// rtl/adc_acquisition/DATAPATH.md). The ADC fills F0, rolls straight on into
// F1, and stops. Only once *both* banks hold a frame does the HW copy engine
// run, streaming them out back to back as a *single* CMD25
// (WRITE_MULTIPLE_BLOCK) session covering all 4 KiB -- 8 x 512 B blocks --
// closed by AUTO_CMD12. Nothing is streamed while the ADC is capturing.
//
// The trade against sdcard_acquisition_Nx.c (SDCARD_CONTINUOUS):
//   - continuous: unlimited capture length, but the card must sustain the
//     ADC's data rate in real time -- one session must finish within one
//     bank fill or the ADC overruns a bank that is still draining
//     (SDCARD_OVERFLOW).
//   - pulse: no throughput requirement whatsoever, since capture and
//     streaming never overlap; the capture is bounded at both banks (4 KiB
//     = 2048 samples) instead.
// So a pulse captures a short burst at the full ADC rate regardless of how
// slow the card is, which is what an event/pulse-echo capture wants.
//
// Software drives nothing on the SD side here either: no CMD25 to open, no
// BLOCK_COUNT to poke, no BUFFER_WRITE_READY to pre-arm, no CMD12 to send.
//
// Flow:
//   1. Full SD card init (reset -> identify -> select -> 4-bit -> block size).
//   2. Configure the F0/F1 frame addresses (one whole bank each),
//      SDCARD_BLOCK_SIZE / SDCARD_BLOCK_COUNT -- note BLOCK_COUNT covers
//      *both* banks here, since one session spans both -- SDCARD_BLOCK_ADDR
//      (starting address) and SDCARD_ADDR_MODE (addressing units), then
//      enable SDCARD_PULSE. SDCARD_FRAME_COUNT is not set: the mode ignores
//      it, its capture length being fixed at the two banks.
//   3. Poll SDCARD_DONE (fires once the whole 8-block session is physically
//      committed and the card is idle again) / SDCARD_OVERFLOW.
//   4. Hardware has already reverted MODE to IDLE. To fire another pulse,
//      re-arm: clear the status flags, RESET_WRITE_HEAD, set MODE again --
//      that is the loop below. SDCARD_BLOCK_ADDR is deliberately *not*
//      reset, so consecutive pulses land back to back on the card.
//
// Note what the gap between pulses costs: samples arriving while software
// re-arms are not captured. That is inherent to the mode -- a pulse is a
// burst, not a stream.

#include "uart.h"
#include "clint.h"
#include "util.h"
#include "print.h"
#include "adc_acquisition.h"
#include "sdhci_helpers.h"

// Number of back-to-back pulses to capture. 1 is a single burst; >1 also
// exercises the software re-arm path (and shows consecutive pulses landing
// at consecutive LBAs).
#define NUM_PULSES  1

#define TIMEOUT  250000U   // CLINT ticks (~7.5 s at 32 kHz) -- bounds one pulse

// ADC SRAM layout: one whole bank per frame, both banks per pulse, so a
// pulse is all 4 KiB the SoC dedicates to ADC capture.
//
// The SD block size stays 512 B -- SDHC/SDXC fix the write block length
// there, and sdModel.v hardcodes it -- but the *session* is both banks, so
// SDCARD_BLOCK_COUNT is the blocks in F0 plus F1 (8), not the 4 that
// sdcard_acquisition_Nx.c uses for its per-bank sessions. The 1 KiB SDHCI
// DAT buffer does not have to hold any of that: the copy engine streams
// through it and is backpressured by the OBI grant when it fills.
//
// These must stay consistent -- the static assert below enforces it:
//   SDCARD_BLOCK_SIZE * SDCARD_BLOCK_COUNT == F0 bytes + F1 bytes
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

// Starting SDCARD_BLOCK_ADDR value. Its *units* are not a constant: block
// vs. byte addressing is the card's own answer, read out of the OCR's CCS
// bit during init -- see sdh_card_is_block_addressed(), which also documents
// the simulation model's override.
#define SDCARD_START_ADDR    0u

int main(void) {
    uart_init();
    printf("SDCard pulse acquisition, NUM_PULSES=%x\n", NUM_PULSES);

    // 1. Initialise SD card
    if (!sdh_init()) {
        printf("SD init failed\n");
        uart_write_flush();
        return 1;
    }

    // 2. Static configuration -- identical for every pulse, so it is set up
    //    once outside the loop. SDHCI's own BLOCK_SIZE/BLOCK_COUNT are not
    //    touched here: the copy engine writes them itself at the start of
    //    the session, from the two registers below.
    ADC_ACQ->SDCARD_BLOCK_SIZE  = SD_BLOCK_BYTES;
    ADC_ACQ->SDCARD_BLOCK_COUNT = BLOCKS_PER_PULSE;   // both banks in one session
    ADC_ACQ->SDCARD_BLOCK_ADDR  = SDCARD_START_ADDR;
    // Negotiated during sdh_init(), so this must come after it.
    const uint32_t addr_is_blocks = sdh_card_is_block_addressed();
    ADC_ACQ->SDCARD_ADDR_MODE   = addr_is_blocks;
    ADC_ACQ->F0_START_ADDR      = F0_START_ADDR_BYTE;
    ADC_ACQ->F0_END_ADDR        = F0_END_ADDR_BYTE;
    ADC_ACQ->F1_START_ADDR      = F1_START_ADDR_BYTE;
    ADC_ACQ->F1_END_ADDR        = F1_END_ADDR_BYTE;

    uint32_t pulses_done = 0;
    uint32_t status      = 0;
    int      timed_out   = 0;

    for (uint32_t p = 0; p < NUM_PULSES; p++) {
        // Re-arm: discard the previous pulse's terminal flags and put the
        // write head back at F0. RESET_WRITE_HEAD is also what clears the
        // hardware's "capture finished" latch, so it is what re-opens the
        // ADC-fill side. Clearing Fx_FULL is belt and braces -- the copy
        // engine already released both banks when its session committed --
        // but it makes a stale flag from a *failed* pulse impossible to
        // carry into the next one, where it would trip SDCARD_OVERFLOW
        // immediately.
        ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                         (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                         (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);
        ADC_ACQ->CNTRL = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
        ADC_ACQ->CONF  = ADC_ACQ_MODE_SDCARD_PULSE;

        // 3. Poll for the pulse's terminal status. SDCARD_DONE fires only
        //    after the whole 8-block session is physically on the card and
        //    the card has released DAT0.
        uint64_t end = clint_get_mtime() + TIMEOUT;
        timed_out = 0;
        while (!(ADC_ACQ->STATUS & (ADC_ACQ_STATUS_SDCARD_DONE |
                                     ADC_ACQ_STATUS_SDCARD_OVERFLOW))) {
            if (clint_get_mtime() >= end) { timed_out = 1; break; }
        }
        status = ADC_ACQ->STATUS;

        // 4. Hardware already reverted MODE to IDLE on the terminal event;
        //    this is only for the timeout path, where it did not.
        ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

        if (timed_out || (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW))
            break;
        pulses_done++;
    }

    // SDCARD_BLOCK_ADDR advances once per completed session, i.e. once per
    // pulse, by one whole session's worth in whichever unit SDCARD_ADDR_MODE
    // selects -- so it independently confirms how many pulses actually
    // landed, provided the divisor matches the unit the card negotiated.
    uint32_t addr_advanced  = ADC_ACQ->SDCARD_BLOCK_ADDR - SDCARD_START_ADDR;
    uint32_t addr_per_pulse = addr_is_blocks ? BLOCKS_PER_PULSE : PULSE_BYTES;
    uint32_t pulses_on_card = addr_advanced / addr_per_pulse;

    if (timed_out)
        printf("FAIL: pulse timeout after %x/%x pulses\n", pulses_done, NUM_PULSES);
    if (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW)
        printf("FAIL: SDCARD_OVERFLOW after %x/%x pulses\n", pulses_done, NUM_PULSES);
    if (status & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");
    if (pulses_done == NUM_PULSES)
        printf("OK: %x pulses x %x blocks x %x B written to SDCard starting at LBA %x\n",
               pulses_on_card, BLOCKS_PER_PULSE, SD_BLOCK_BYTES, SDCARD_START_ADDR);

    // Clear all ADC status flags
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);

    uart_write_flush();
    return 0;
}
