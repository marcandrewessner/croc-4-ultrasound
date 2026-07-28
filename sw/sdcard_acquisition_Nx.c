// SDCard acquisition program -- N-frame capture, N set by NUM_FRAMES below.
//
// ACQ_SDCARD is a single mechanism parametrized by frame count (see
// rtl/adc_acquisition/DATAPATH.md): set NUM_FRAMES to 1 for a single-block
// capture (what used to be a separate SINGLE_SDCARD mode), or to any N>1
// for an F0/F1 ping-pong multi-block capture (what used to be
// CONTINUOUS_SDCARD). Same code path either way.
//
// A frame is one entire SRAM bank (2 KiB = 4 x 512 B blocks), and the HW
// copy engine (adc_acquisition_sdcard_controller) streams each filled frame
// out as one CMD25 (WRITE_MULTIPLE_BLOCK) session closed by AUTO_CMD12. The
// ADC fills one bank while the engine streams the other, so all 4 KiB of ADC
// SRAM is in use and capture is continuous across sessions.
//
// There is still nothing for software to drive on the SD side: no CMD25 to
// open, no BLOCK_COUNT to poke, no BUFFER_WRITE_READY to pre-arm, no CMD12
// to send. Software's job is just: SD card init, configure the ADC_ACQ
// registers below, enable ACQ_SDCARD, poll for completion.
//
// Flow, identical for any N:
//   1. Full SD card init (reset -> identify -> select -> 4-bit -> block size).
//   2. Configure the F0/F1 frame addresses (one whole bank each),
//      SDCARD_BLOCK_SIZE / SDCARD_BLOCK_COUNT (the CMD25 session geometry --
//      hardware writes these into SDHCI itself at the start of every
//      session), SDCARD_BLOCK_ADDR (starting address), SDCARD_ADDR_MODE
//      (addressing units), SDCARD_FRAME_COUNT = NUM_FRAMES (this is what
//      lets the ADC-fill side stop itself exactly at frame NUM_FRAMES
//      instead of overrunning it, DATAPATH.md §2a), and enable ACQ_SDCARD
//      mode.
//   3. Poll SDCARD_DONE (fires once the whole capture is physically written
//      and the card is idle again) / SDCARD_OVERFLOW.
//   4. Stop acquisition.

#include "uart.h"
#include "clint.h"
#include "util.h"
#include "print.h"
#include "adc_acquisition.h"
#include "sdhci_helpers.h"

// ---------------------------------------------------------------------------
// The one thing to change to exercise a different mode/regime. A frame is a
// whole bank / one CMD25 session of BLOCKS_PER_FRAME blocks:
//   NUM_FRAMES == 1  -> one session, no bank reuse
//   NUM_FRAMES == 2  -> two sessions, no bank reuse either (each of F0/F1
//                       written exactly once)
//   NUM_FRAMES  > 2  -> streaming capture, F0/F1 reused -- exercises
//                       target_frame_full/SDCARD_OVERFLOW if the card
//                       can't keep up with T_fill
// ---------------------------------------------------------------------------
#define NUM_FRAMES  4u

#define TIMEOUT  250000U   // CLINT ticks (~7.5 s at 32 kHz) -- bounds the whole capture

// ADC SRAM frame layout: one whole bank per frame, so both banks together
// use all 4 KiB the SoC dedicates to ADC capture.
//
// The SD block size stays 512 B -- SDHC/SDXC fix the write block length
// there, and sdModel.v hardcodes it (`BlockAddr % 512`, `BlockAddr += 512`,
// CSD advertising sector size 512) -- but a frame is no longer one block.
// The copy engine opens a CMD25 session of BLOCKS_PER_FRAME blocks per
// frame, so the frame can be as large as the bank while every block on the
// wire is still a spec-legal 512 B. The 1 KiB SDHCI DAT buffer
// (BufferNumWords=256) does not have to hold a whole frame either: the
// engine streams through it and is backpressured by the OBI grant when it
// fills.
//
// These three must stay consistent -- the static assert below enforces it:
//   SDCARD_BLOCK_SIZE * SDCARD_BLOCK_COUNT == frame byte count
#define SD_BLOCK_BYTES      512u
#define N_WORDS             ADC_ACQ_BANK_WORDS               // 512 words = 2 KiB
#define FRAME_BYTES         (N_WORDS * 4u)
#define BLOCKS_PER_FRAME    (FRAME_BYTES / SD_BLOCK_BYTES)   // 4
#define F0_START_ADDR_BYTE  ADC_ACQ_F0_BASE
#define F0_END_ADDR_BYTE    ADC_ACQ_FRAME_END(F0_START_ADDR_BYTE, N_WORDS)
#define F1_START_ADDR_BYTE  ADC_ACQ_F1_BASE
#define F1_END_ADDR_BYTE    ADC_ACQ_FRAME_END(F1_START_ADDR_BYTE, N_WORDS)

ADC_ACQ_ASSERT_FRAME_FITS(N_WORDS);
_Static_assert(BLOCKS_PER_FRAME * SD_BLOCK_BYTES == FRAME_BYTES,
               "frame must be a whole number of SD blocks");

// Starting SDCARD_BLOCK_ADDR value and its units -- 1 = block addressing
// (SDHC/SDXC-style, argument is a raw block number, advances by
// SDCARD_BLOCK_COUNT per session), 0 = byte addressing
// (standard-capacity-style, argument is a byte address, advances by the
// frame's byte count per session). Only the session's *starting* address is
// ever sent: within a CMD25 session the card advances internally per block.
// Most cards in use today are SDHC/SDXC, so 1 is what real hardware wants.
//
// This is 0 because of the simulation card model, not because of the
// design: rtl/test/sdcard/model/sdModel.v advertises CCS=1 in its OCR
// (32'h40ff8000, i.e. "I am high capacity, address me in blocks") but its
// CMD25 handler then does `BlockAddr = inCmd[39:8]` and indexes FLASHmem
// with it directly -- a raw *byte* address, and its per-block advance is
// likewise `BlockAddr += 512`. It even checks
// `if (BlockAddr % 512 != 0) $display("**Block Misalign Error")`, which is
// only meaningful for byte addressing. So against this model, block N must
// be addressed as N*512. Set this back to 1 for a real SDHC/SDXC card.
#define SDCARD_START_ADDR    0u
#define SDCARD_ADDR_IS_BLOCKS 0u

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
    //    configured when NUM_FRAMES == 1), the CMD25 session geometry, the
    //    starting SD address and its units, the frame budget, and start the
    //    copy engine. Note SDHCI's own BLOCK_SIZE/BLOCK_COUNT are not
    //    touched here: the copy engine writes them itself at the start of
    //    every session, from the two registers below.
    ADC_ACQ->SDCARD_BLOCK_SIZE  = SD_BLOCK_BYTES;
    ADC_ACQ->SDCARD_BLOCK_COUNT = BLOCKS_PER_FRAME;
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
    // SDCARD_BLOCK_ADDR advances once per completed session, by one frame's
    // worth in whichever unit SDCARD_ADDR_MODE selects -- so recovering the
    // frame count means dividing by that same per-frame amount.
    uint32_t frames_done = ADC_ACQ->SDCARD_BLOCK_ADDR - SDCARD_START_ADDR;
    frames_done /= (SDCARD_ADDR_IS_BLOCKS ? BLOCKS_PER_FRAME : FRAME_BYTES);

    if (timed_out)
        printf("FAIL: capture timeout after %x/%x frames\n", frames_done, NUM_FRAMES);
    if (status & ADC_ACQ_STATUS_SDCARD_OVERFLOW)
        printf("FAIL: SDCARD_OVERFLOW after %x/%x frames (SDHCI was not ready)\n",
               frames_done, NUM_FRAMES);
    if (status & ADC_ACQ_STATUS_ADC_OVERFLOW)
        printf("WARN: ADC_OVERFLOW (CDC FIFO full, samples dropped)\n");
    if (status & ADC_ACQ_STATUS_SDCARD_DONE)
        printf("OK: %x frames x %x blocks x %x B written to SDCard starting at LBA %x\n",
               frames_done, BLOCKS_PER_FRAME, SD_BLOCK_BYTES, SDCARD_START_ADDR);

    // Clear all ADC status flags
    ADC_ACQ->CNTRL = ADC_ACQ_CTRL_CLEAR_STATUS |
                     (1u << ADC_ACQ_CTRL_CLR_F0_FULL_BIT) |
                     (1u << ADC_ACQ_CTRL_CLR_F1_FULL_BIT);

    uart_write_flush();
    return 0;
}
