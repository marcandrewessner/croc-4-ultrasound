// SDCard acquisition program -- N-frame capture, N set by NUM_FRAMES below.
//
// ACQ_SDCARD is a single mechanism parametrized by frame count (see
// rtl/adc_acquisition/DATAPATH.md): set NUM_FRAMES to 1 for a single-block
// capture (what used to be a separate SINGLE_SDCARD mode), or to any N>1
// for an F0/F1 ping-pong multi-block capture (what used to be
// CONTINUOUS_SDCARD). Same code path either way.
//
// The HW copy engine (adc_acquisition_sdcard_controller) issues its own
// CMD24 (WRITE_BLOCK) per frame, back to back -- there is no CMD25
// stream for software to open or close, no BUFFER_WRITE_READY to
// pre-arm, no CMD12 to send. Software's job is just: SD card init,
// configure the ADC_ACQ registers below, enable ACQ_SDCARD, poll for
// completion.
//
// Flow, identical for any N:
//   1. Full SD card init (reset -> identify -> select -> 4-bit -> block size).
//   2. Configure F0 (and F1, harmless if unused when NUM_FRAMES == 1) frame
//      addresses, SDCARD_BLOCK_ADDR (starting address), SDCARD_ADDR_MODE
//      (addressing units), SDCARD_FRAME_COUNT = NUM_FRAMES (this is what
//      lets the ADC-fill side stop itself exactly at frame NUM_FRAMES
//      instead of overrunning it, DATAPATH.md §2a), and enable ACQ_SDCARD
//      mode.
//   3. Poll SDCARD_DONE (fires once the whole NUM_FRAMES-block capture is
//      physically written and the card is idle again) / SDCARD_OVERFLOW.
//   4. Stop acquisition.

#include "uart.h"
#include "clint.h"
#include "util.h"
#include "print.h"
#include "adc_acquisition.h"
#include "sdhci_helpers.h"

// ---------------------------------------------------------------------------
// The one thing to change to exercise a different mode/regime:
//   NUM_FRAMES == 1  -> single-block capture, no bank reuse
//   NUM_FRAMES == 2  -> two-block capture, no bank reuse either (each of
//                       F0/F1 written exactly once)
//   NUM_FRAMES  > 2  -> streaming capture, F0/F1 reused -- exercises
//                       target_frame_full/SDCARD_OVERFLOW if the card
//                       can't keep up with T_fill
// ---------------------------------------------------------------------------
#define NUM_FRAMES  1u

#define TIMEOUT  250000U   // CLINT ticks (~7.5 s at 32 kHz) -- bounds the whole capture

// ADC SRAM frame layout (croc_pkg.sv: Bank2=F0 @ 0x1000_1000, Bank3=F1 @ 0x1000_1800)
#define F0_START_ADDR_BYTE  0x10001000u
#define F0_END_ADDR_BYTE    (F0_START_ADDR_BYTE + 512u - 4u)  // address of last word
#define F1_START_ADDR_BYTE  0x10001800u
#define F1_END_ADDR_BYTE    (F1_START_ADDR_BYTE + 512u - 4u)

// Starting SDCARD_BLOCK_ADDR value and its units -- 1 = block addressing
// (SDHC/SDXC-style, argument is a raw block number, advances by 1 per
// block), 0 = byte addressing (standard-capacity-style, argument is a
// byte address, advances by this frame's byte count per block).
//
// 1, because real Genesys2 hardware testing (2026-07-28) confirmed the
// physical card is high-capacity (ACMD41's OCR reported CCS=1, bit 30 of
// 0xC0FF8000), which per SD spec means CMD17/CMD24 take a block number, not
// a byte address. Only the simulation model wants 0: rtl/test/sdcard/model/
// sdModel.v advertises CCS=1 in its OCR too but its CMD24 handler then does
// `BlockAddr = inCmd[39:8]` and indexes FLASHmem with it directly -- a raw
// *byte* address, checked with `if (BlockAddr % 512 != 0) $display("**Block
// Misalign Error")`, which is only meaningful for byte addressing. So set
// this back to 0 if testing against sdModel.v instead of real hardware.
#define SDCARD_START_ADDR    0u
#define SDCARD_ADDR_IS_BLOCKS 1u

int main(void) {
    uart_init();
    printf("SDCard acquisition, NUM_FRAMES=%x\n", NUM_FRAMES);

    // 1. Initialise SD card
    if (!sdh_init()) {
        printf("SD init failed\n");
        uart_write_flush();
        return 1;
    }

    // 2. Configure both frame banks (F1 goes unused but harmlessly
    //    configured when NUM_FRAMES == 1), the starting SD address and its
    //    units, the frame budget, and start the copy engine.
    ADC_ACQ->SDCARD_BLOCK_ADDR  = SDCARD_START_ADDR;
    ADC_ACQ->SDCARD_ADDR_MODE   = SDCARD_ADDR_IS_BLOCKS;
    ADC_ACQ->SDCARD_FRAME_COUNT = NUM_FRAMES;
    ADC_ACQ->F0_START_ADDR      = F0_START_ADDR_BYTE;
    ADC_ACQ->F0_END_ADDR        = F0_END_ADDR_BYTE;
    ADC_ACQ->F1_START_ADDR      = F1_START_ADDR_BYTE;
    ADC_ACQ->F1_END_ADDR        = F1_END_ADDR_BYTE;
    ADC_ACQ->CNTRL              = (1u << ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT);
    ADC_ACQ->CONF               = ADC_ACQ_MODE_ACQ_SDCARD;

    printf("Acquiring %x frames...\n", NUM_FRAMES);

    // 3. Poll SDCARD_DONE -- fires once the whole NUM_FRAMES-block capture
    //    is physically written and the card is idle again (the copy engine
    //    only sets it on the capture's last frame, see is_last_frame_i).
    uint64_t end = clint_get_mtime() + TIMEOUT;
    int timed_out = 0;
    while (!(ADC_ACQ->STATUS & (ADC_ACQ_STATUS_SDCARD_DONE |
                                 ADC_ACQ_STATUS_SDCARD_OVERFLOW))) {
        if (clint_get_mtime() >= end) { timed_out = 1; break; }
    }
    uint32_t status = ADC_ACQ->STATUS;

    // 4. Stop acquisition
    ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;
    uint32_t frames_done = ADC_ACQ->SDCARD_BLOCK_ADDR - SDCARD_START_ADDR;
    if (!SDCARD_ADDR_IS_BLOCKS)
        frames_done /= 512u;

    if (timed_out)
        printf("FAIL: capture timeout after %x/%x frames\n", frames_done, NUM_FRAMES);
    if (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW)
        printf("FAIL: SDCARD_OVERFLOW after %x/%x frames (SDHCI was not ready)\n",
               frames_done, NUM_FRAMES);
    if (status & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");
    if (status & ADC_ACQ_STATUS_SDCARD_DONE)
        printf("OK: %x x 512 B written to SDCard starting at LBA %x\n",
               frames_done, SDCARD_START_ADDR);

    // Clear all ADC status flags
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);

    uart_write_flush();
    return 0;
}
