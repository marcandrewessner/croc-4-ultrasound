
#pragma once

#include "util.h"

#include "adc_acquisition_reg.h"

#define ADC_ACQ_BASE       0x0300C000UL
#define ADC_ACQ_INTERRUPT  (16+4)

// export an accessable register
#define ADC_ACQ ((volatile adc_acquisition_reg_t*)ADC_ACQ_BASE)

// ---------------------------------------------------------------------------
// ADC SRAM bank geometry -- single source of truth for frame layout.
//
// Mirrors rtl/croc_pkg.sv: NumSramBanks=4 banks of SramBankNumWords=512
// 32-bit words. ihp13/tc_sram_impl.sv's gen_512x32xBx1 arm realises each as
// one RM_IHPSG13_1P_256x64_c2_bm_bist macro (256 x 64 bit, with the address
// LSB bit-interleaving 32-bit accesses into the 64-bit word), i.e. exactly
// 2 KiB of silicon per bank. CrocAddrMap dedicates the top two banks to the
// ADC, and the crossbar connectivity matrix restricts the adc_data_write and
// adc_copy_read manager ports to those two banks only:
//
//   Bank2 = F0 @ 0x1000_1000 .. 0x1000_17FF   2 KiB / 512 words / 1024 samples
//   Bank3 = F1 @ 0x1000_1800 .. 0x1000_1FFF   2 KiB / 512 words / 1024 samples
//
// 4 KiB of ADC capture buffer in total. Separate banks are what make the
// ping-pong free: each has its own OBI port, so filling one never contends
// with draining the other. link.ld caps the linker's SRAM region at 4K from
// 0x1000_0000 -- exactly where Bank2 starts -- so no .data/.bss/stack can
// ever land in these banks.
#define ADC_ACQ_BANK_WORDS   512u
#define ADC_ACQ_BANK_BYTES   (ADC_ACQ_BANK_WORDS * 4u)
#define ADC_ACQ_F0_BASE      0x10001000u
#define ADC_ACQ_F1_BASE      0x10001800u

// Fx_END_ADDR holds the address of the frame's *last* word, inclusive -- that
// is what the RTL's frame-boundary comparison tests the write head against
// (adc_acquisition_top.sv), hence the -1. Off-by-one here costs a whole word.
#define ADC_ACQ_FRAME_END(base, words)  ((base) + ((words) - 1u) * 4u)

// The RTL has no concept of a bank boundary -- it only compares the write head
// against Fx_END_ADDR -- so a frame longer than its bank would silently spill
// into the neighbouring bank's address range instead of failing loudly. Catch
// it at compile time wherever a frame length is chosen.
#define ADC_ACQ_ASSERT_FRAME_FITS(words)                              \
    _Static_assert((words) > 0u && (words) <= ADC_ACQ_BANK_WORDS,     \
                   "ADC frame must fit in one 512-word (2 KiB) bank")

// Each 32-bit word packs two consecutive 14-bit ADC samples, laid out as
// {2'b00, sample[2i+1], 2'b00, sample[2i]} by the ADC-domain packer.
#define ADC_ACQ_SAMPLES_PER_WORD  2u

// CONFIG.MODE
#define ADC_ACQ_MODE_MASK                 ADC_ACQUISITION_REG__CONF__MODE_bm
#define ADC_ACQ_MODE_IDLE                 0x00
#define ADC_ACQ_MODE_SINGLE_ACQ_F0        0x04
#define ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1 0x10
// Ping-pong F0/F1 with HW copy to SDCard, capture overlapped with streaming:
// the ADC fills one bank while the copy engine streams the other out as its
// own CMD25 session. Set SDCARD_FRAME_COUNT to the number of frames (banks)
// to capture and SDCARD_BLOCK_COUNT to the blocks in *one* bank. Because
// streaming runs while capturing, the card must keep up with the ADC in real
// time -- one session must complete within one bank fill, else
// SDCARD_OVERFLOW.
#define ADC_ACQ_MODE_SDCARD_CONTINUOUS    0x18
// One-shot burst ("pulse") capture: the ADC fills F0 then F1 with nothing
// streaming alongside, and only once both are full does the copy engine run,
// writing both banks out as a single CMD25 session (8 blocks of 512 B for two
// full banks). No real-time throughput requirement at all -- the capture is
// bounded at two banks instead. Set SDCARD_BLOCK_COUNT to the blocks in
// *both* banks; SDCARD_FRAME_COUNT is ignored. SDCARD_DONE fires when the
// session is physically committed and MODE auto-reverts to IDLE; re-arm
// (CLEAR_STATUS + CLEAR_Fx_FULL, RESET_WRITE_HEAD, set MODE) per pulse.
#define ADC_ACQ_MODE_SDCARD_PULSE         0x1C

// STATUS bits (use as bitmasks on STATUS register value)
#define ADC_ACQ_STATUS_F0_FULL_BIT      ADC_ACQUISITION_REG__STATUS__F0_FULL_bp
#define ADC_ACQ_STATUS_F1_FULL_BIT      ADC_ACQUISITION_REG__STATUS__F1_FULL_bp
#define ADC_ACQ_STATUS_F0_FULL          (1u << ADC_ACQUISITION_REG__STATUS__F0_FULL_bp)
#define ADC_ACQ_STATUS_F1_FULL          (1u << ADC_ACQUISITION_REG__STATUS__F1_FULL_bp)
#define ADC_ACQ_STATUS_ADC_OVERFLOW     (1u << ADC_ACQUISITION_REG__STATUS__ADC_OVERFLOW_bp)
#define ADC_ACQ_STATUS_SDCARD_DONE      (1u << ADC_ACQUISITION_REG__STATUS__SDCARD_DONE_bp)
#define ADC_ACQ_STATUS_SDCARD_OVERFLOW  (1u << ADC_ACQUISITION_REG__STATUS__SDCARD_OVERFLOW_bp)

// CNTRL bits
#define ADC_ACQ_CTRL_RST_WRITE_HEAD_BIT ADC_ACQUISITION_REG__CNTRL__RESET_WRITE_HEAD_bp
#define ADC_ACQ_CTRL_CLR_F0_FULL_BIT    ADC_ACQUISITION_REG__CNTRL__CLEAR_F0_FULL_bp
#define ADC_ACQ_CTRL_CLR_F1_FULL_BIT    ADC_ACQUISITION_REG__CNTRL__CLEAR_F1_FULL_bp
#define ADC_ACQ_CTRL_CLEAR_STATUS       (1u << ADC_ACQUISITION_REG__CNTRL__CLEAR_STATUS_bp)