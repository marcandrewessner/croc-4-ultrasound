// SDCard copy controller for ADC acquisition pipeline.
//
// Software is responsible for SD card initialisation (CMD0..CMD16: reset,
// identify, select, 4-bit bus, block length) before enabling one of the
// SDCARD_* modes. From that point on, this module owns the SD protocol by
// itself: per job it opens one CMD25 (WRITE_MULTIPLE_BLOCK) session
// covering the whole job -- SDCARD_BLOCK_COUNT blocks of SDCARD_BLOCK_SIZE
// bytes -- streams every word of it into the SDHCI data port back to back,
// and lets AUTO_CMD12 close the session.
//
// A "frame" here is one entire SRAM bank (2 KiB = 4 x 512 B blocks), not
// one SD block: adc_acquisition_top ping-pongs the two banks, so the ADC
// fills one while this engine streams the other out.
//
// A job covers either one bank or both, selected by copy_both_i, and that
// is the *only* difference between the two SDCARD_* modes as far as this
// module is concerned:
//   - copy_both_i = 0 (SDCARD_CONTINUOUS): one job per filled bank,
//     copy_f0_i picking which. Capture and streaming overlap.
//   - copy_both_i = 1 (SDCARD_PULSE): one job for the whole capture, run
//     only once *both* banks are full, streaming F0's words and then F1's
//     into a single session (8 blocks for two full banks instead of 4).
//     Nothing overlaps -- the ADC has already stopped -- so the card never
//     has to keep up with the ADC in real time; the price is a capture
//     bounded at two banks.
// A both-banks job does not assume F1 is contiguous with F0 in the address
// map (it happens to be): the read address switches base explicitly once
// F0's word count is exhausted, so the two Fx_START_ADDR/Fx_END_ADDR pairs
// stay independent. On the SD side it is one uninterrupted session either
// way -- the card sees 8 blocks with its own internally advancing write
// pointer and cannot tell the bank switch happened.
//
// Relation to the earlier per-block CMD24 design (DATAPATH.md §1): that
// design existed because banks had to be released one block at a time,
// which needed a per-block notion of "physically committed", and
// BUFFER_WRITE_READY -- the only per-block event a CMD25 session offers --
// does not mean that (it reflects SDHCI-internal buffer occupancy). Here
// nothing needs per-block completion: the bank is released once, at the
// end of the session, on the session's single real TRANSFER_COMPLETE.
// BWR is therefore used only for what it actually is, flow control, and
// only once, to confirm the card is ready for the session's first data.
// The ambiguity §1 walked away from is designed out rather than managed.
//
// Backpressure: the engine does *not* pace itself per block. It pushes the
// frame's words continuously and relies on the SDHCI data port withholding
// the OBI grant when its 1 KiB buffer is full (dat_buffer.sv's
// buffer_data_port_write_ready_o -> reg_ready -> gnt, routed
// unconditionally in sdhci_top.sv). That is what makes streaming a 2 KiB
// frame through a 1 KiB buffer correct. Note this only works because the
// block-boundary logic in dat_buffer.sv forces buffer_write_enable_o (the
// status/interrupt bit) low for a cycle without touching
// buffer_data_port_write_ready_o, so the data path itself never closes
// mid-session. If the SD side falls behind far enough, the grant simply
// stalls this FSM; if it falls behind while the SDHCI buffer is empty,
// dat_wrap.sv pauses the SD clock instead of underrunning the block.
//
// This module is a passive subordinate to adc_acquisition_top's control
// logic: it does not watch CONF.MODE or STATUS.Fx_FULL itself, it only
// reacts to an explicit start_i pulse (with copy_f0_i selecting which
// frame, copy_both_i widening the job to both frames, is_last_frame_i
// marking whether this is the capture's final
// frame) and reports back via busy_o plus the existing *_set_o/*_clear_o
// pulses. It never re-arms itself -- every copy job is initiated from the
// outside, one at a time, and the module parks in CE_IDLE (busy_o low)
// between jobs. busy_o spans the *entire* session, from issuing CMD25
// through AUTO_CMD12 and the card's final program completing; there is no
// state where the module reports idle while data is still in flight.
//
// Per job:
//   1. On start_i, latch which frame to copy (copy_f0_i) or that both are
//      to be copied (copy_both_i), the job's total word count (one frame's
//      Fx_END_ADDR - Fx_START_ADDR + 1, or both frames' added together),
//      and whether this is the capture's last frame (is_last_frame_i).
//   2. CE_CLEAR_STALE_STATUS: W1C-clear every NINTR_STATUS bit this FSM
//      polls (TRANSFER_COMPLETE, BUFFER_WRITE_READY, COMMAND_COMPLETE) and
//      every EINTR_STATUS bit, unconditionally, before this job's own CMD25
//      is even issued. BWR in particular *must* be discarded here: a CMD25
//      session leaves it set from its last block boundary and nothing else
//      clears it, so the next session would otherwise match a leftover on
//      its first poll -- see the state body.
//      TRANSFER_COMPLETE fires on any 1->0 edge of
//      command_inhibit_dat, which includes busy-checked commands with no
//      data phase at all (e.g. CMD7 during sd_init()) -- a real event
//      that nothing else ever clears. Skipping this step means the next
//      wait for TRANSFER_COMPLETE (step 9) can trivially "succeed" on
//      that leftover bit without the block's data having gone anywhere
//      -- confirmed via waveform: busy_o dropped while dat_wrap.sv's
//      write_state_q was still WRITING.
//      Separately, EINTR_STATUS.data_timeout_error latches once on
//      *every* block, harmlessly, as a side effect of a fixed one-cycle
//      race in the SDHCI IP itself: dat_write.sv's STATUS_START_BIT
//      state samples dat0_i exactly once, two sd_clk cycles after
//      END_BIT (the hardcoded BUS_SWITCH wait), to check whether the
//      card has already pulled DAT0 low to start its CRC status token.
//      Against this repo's sdModel.v, DAT0 was not yet low at that exact
//      sample, so data_timeout_d latched on every block.
//
//      That sampling race is FIXED at its source now (dat_write.sv's
//      STATUS_START_BIT searches for the start bit over a window instead
//      of sampling once) -- it had to be, because it is not the harmless
//      passenger the single-block design could treat it as. It made
//      dat_wrap.sv's write_state_q detour through TIMEOUT_WRITING instead
//      of DONE_WRITING_BLOCK, and DONE_WRITING_BLOCK is the only state
//      that loops back for the next block of a CMD25 session -- so every
//      multi-block session was silently truncated to one block
//      (DATAPATH.md §0a). This step's clearing of the resulting
//      EINTR_STATUS bit is kept anyway: it costs nothing, and the flag can
//      still be set legitimately by a genuinely slow card.
//
//      EINTR_STATUS.data_timeout_error is set where write_state_q reaches
//      TIMEOUT_WRITING. Like TRANSFER_COMPLETE, nothing clears it
//      afterward, so without this step it survives into the next job:
//      confirmed via waveform at N=2, where job 1's leftover
//      data_timeout_error (and the ERROR_INTERRUPT bit it drives) was
//      still set when job 2's CE_WAIT_CMD_COMPLETE_RSP ran its error
//      check, sending job 2 straight to CE_OVERFLOW despite job 2's own
//      CMD24 never having been given a chance to fail.
//   3. CE_WAIT_CMD_INHIBIT: poll SDHCI PRESENT_STATE until both
//      CMD_INHIBIT_CMD and CMD_INHIBIT_DAT are clear -- the card must be
//      idle before a new command can be issued.
//   4. CE_SET_BLOCK_SIZE / CE_SET_TRANSFER_MODE / CE_SET_ARGUMENT /
//      CE_SUBMIT_CMD25: write BLOCK_SIZE and BLOCK_COUNT (one OBI write --
//      they are the low and high halves of the same 32-bit SDHCI register
//      at offset 0x04, so a be=4'hf write sets both), TRANSFER_MODE
//      (multi-block, block-count-enable, AUTO_CMD12, write direction),
//      ARGUMENT (the current SDCARD_BLOCK_ADDR value, used as-is
//      regardless of addressing mode -- see block_addr_advance below),
//      then COMMAND itself (index 25, R1 response, DATA_PRESENT_SELECT).
//      block-count-enable is what makes SDHCI end the session -- and raise
//      TRANSFER_COMPLETE -- after exactly BLOCK_COUNT blocks
//      (dat_wrap.sv's transmitted_block_counter). TRANSFER_MODE and
//      COMMAND are bits of the *same* physical 32-bit SDHCI register
//      (byte offsets 0x0c/0x0e) but are written as two separate OBI
//      transactions, matching every existing example in this codebase,
//      rather than one combined write -- the marginal cycles saved by
//      combining them aren't worth introducing an unexercised write
//      pattern for.
//   5. CE_WAIT_CMD_COMPLETE: poll NINTR_STATUS for COMMAND_COMPLETE (bit
//      0); if ERROR_INTERRUPT (bit 15) is set instead, treat it the same
//      as a timeout (-> overflow) rather than proceeding on a command
//      that may not have actually been accepted. Full EINTR_STATUS
//      decoding is still not implemented -- same documented limitation
//      as before, just no longer silently ignored at the command level.
//
//      Do not be tempted to skip this wait and go straight to BWR: it is
//      what keeps the session's block count correct. dat_wrap.sv latches
//      transmitted_block_counter from BLOCK_COUNT while write_state_q is
//      WAIT_FOR_RSP, leaving it only on sd_rsp_done_i -- but the buffer
//      goes live (and BWR can rise) as soon as dat_state becomes WRITE,
//      which is at command *start*, before the response. Meanwhile
//      dat_buffer.sv decrements the very same BLOCK_COUNT register on each
//      block *pushed*. So a copier that started pushing before the response
//      landed could decrement BLOCK_COUNT below its intended value before
//      dat_wrap ever sampled it, and the session would end one or more
//      blocks early. Waiting for COMMAND_COMPLETE -- which is derived from
//      the same response event as sd_rsp_done_i -- plus the ack round trip
//      puts the first push comfortably after the latch.
//   6. CE_ACK_CMD_COMPLETE: W1C-clear COMMAND_COMPLETE.
//   7. CE_WAIT_BUFFER_READY: poll for BUFFER_WRITE_READY (bit 4) once --
//      the card is ready for the session's data. Not repeated per block:
//      see the backpressure note above.
//   8. CE_COPY_WORD: pipelined SRAM -> SDHCI BUFFER_DATA_PORT copy of the
//      whole frame, one word in flight at a time. The next SRAM read is
//      only issued once the previous word has actually been written
//      (copy_write grant seen), since copy_write is backpressured by
//      SDHCI for an arbitrary number of cycles whenever its buffer fills
//      -- which, streaming a 2 KiB frame through a 1 KiB buffer, is the
//      normal case rather than an exception.
//   9. CE_ACK_BUFFERED: W1C-clear BWR -- the session's words are all in
//      SDHCI's hands.
//  10. CE_WAIT_TRANSFER_COMPLETE: poll for TRANSFER_COMPLETE (bit 1).
//      With block-count-enable set, SDHCI raises this exactly once, when
//      the last of BLOCK_COUNT blocks has been framed on the bus
//      (dat_wrap.sv: DONE_WRITING_BLOCK loops back per block and only
//      reaches DONE_WRITING at transmitted_block_counter == 1, dropping
//      command_inhibit_dat, which is what sets the bit). There is no
//      per-block event to race against here because none is used.
//  11. CE_ACK_TRANSFER_COMPLETE: W1C-clear TRANSFER_COMPLETE.
//  12. CE_WAIT_CARD_READY: poll PRESENT_STATE until the *card* says it is
//      done, not just the host controller: CMD_INHIBIT_CMD/DAT clear AND
//      DAT0's line level (bit 20) back high. TRANSFER_COMPLETE only means
//      SDHCI's own write FSM finished framing the block on the bus; the
//      card then holds DAT0 low for its entire internal
//      program-to-flash time (SD spec busy signalling), and until that
//      ends the card is in PRG state and will not accept another
//      command. Confirmed via waveform at N=2 -- see the CE_WAIT_CARD_READY
//      state body for the full trace. This wait is inside busy_o and
//      before CE_DONE on purpose: SDCARD_DONE and the Fx_FULL release must
//      not fire until the block is physically committed.
//      This also covers AUTO_CMD12 -- there is deliberately no separate
//      state for it, because SDHCI never reports COMMAND_COMPLETE for an
//      auto command. See the state body for the masking that causes that
//      and for why the pass condition is required on two consecutive
//      polls rather than one.
//  13. CE_CHECK_ERRORS: one NINTR_STATUS read for ERROR_INTERRUPT before
//      committing. With no COMMAND_COMPLETE for AUTO_CMD12 there is
//      nowhere else an AUTO_CMD12 failure (EINTR_STATUS bit 8) could be
//      noticed; it also catches late data CRC/end-bit errors.
//  14. CE_DONE: release the bank -- both banks for a copy_both_i job --
//      (Fx_FULL clear), advance
//      SDCARD_BLOCK_ADDR by block_addr_advance_o (+SDCARD_BLOCK_COUNT for
//      block addressing, +this frame's byte count for byte addressing --
//      see SDCARD_ADDR_MODE; note the card advances internally *within*
//      the session, so this register is written once per session, not per
//      block), and -- only if this was the capture's last frame
//      (is_last_frame_q, latched from is_last_frame_i at step 1) -- set
//      SDCARD_DONE. Every other frame releases its bank and advances the
//      address exactly the same way, just without SDCARD_DONE.
//
// Error handling: command-level errors (step 5) and any error latched
// during the session (step 13) fall through to
// CE_OVERFLOW, as does a copy that stalls longer than POLL_TIMEOUT on the
// SDHCI data port (step 8). Data/CRC errors on the blocks themselves are
// still not decoded. SDHCI EINTR_STATUS remains visible to software for
// diagnostics. A future revision can add a proper CE_ERROR sink state.

`include "common_cells/registers.svh"

module adc_acquisition_sdcard_controller
  import adc_acquisition_pkg::*;
  import adc_acquisition_reg_pkg::*;
#(
  parameter type mgr_obi_req_t = logic,
  parameter type mgr_obi_rsp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,

  // Job control: top-level explicitly starts a copy and selects the frame;
  // this module never decides to start on its own.
  input  logic start_i,          // pulse: begin copying one job
  input  logic copy_f0_i,        // 1 = copy F0, 0 = copy F1 (sampled on start_i)
  input  logic copy_both_i,      // 1 = one session covering F0 then F1 (sampled on start_i)
  input  logic is_last_frame_i,  // 1 = this is the capture's final frame (sampled on start_i)
  output logic busy_o,           // low iff parked in CE_IDLE, ready for start_i

  // copy_read: SRAM Bank2/3 reads
  output mgr_obi_req_t copy_read_req_o,
  input  mgr_obi_rsp_t copy_read_rsp_i,

  // copy_write: SDHCI reads/writes (command issuance, NINTR_STATUS polls,
  // BUFFER_DATA_PORT writes)
  output mgr_obi_req_t copy_write_req_o,
  input  mgr_obi_rsp_t copy_write_rsp_i,

  // Read-only view of register outputs (config + current status)
  input  hw2reg_t reg2hw,

  // Status signals driven back to adc_acquisition_top's single always_comb
  output logic        sdcard_done_set_o,      // pulse: set STATUS.SDCARD_DONE (last frame only)
  output logic        sdcard_overflow_set_o,  // pulse: set STATUS.SDCARD_OVERFLOW
  output logic        f0_full_clear_o,        // pulse: clear STATUS.F0_FULL after copy
  output logic        f1_full_clear_o,        // pulse: clear STATUS.F1_FULL after copy
  output logic        block_addr_incr_o,      // pulse: advance SDCARD_BLOCK_ADDR
  output logic [31:0] block_addr_advance_o    // amount to advance by, valid when block_addr_incr_o pulses
);

  // -------------------------------------------------------------------------
  // SDHCI address constants (XbarSDHC base = 0x1001_0000)
  // -------------------------------------------------------------------------
  localparam logic [31:0] SDHCI_BASE          = 32'h1001_0000;
  // BLOCK_SIZE occupies bits [11:0] and BLOCK_COUNT bits [31:16] of this one
  // 32-bit register (sdhci_reg_top.sv), so a single be=4'hf write sets both.
  localparam logic [31:0] SDHCI_BLOCK_SIZE    = SDHCI_BASE + 32'h04;
  localparam logic [31:0] SDHCI_ARGUMENT      = SDHCI_BASE + 32'h08;
  localparam logic [31:0] SDHCI_XFER_CMD      = SDHCI_BASE + 32'h0c; // TRANSFER_MODE (lo16) + COMMAND (hi16)
  localparam logic [31:0] SDHCI_DATA          = SDHCI_BASE + 32'h20; // BUFFER_DATA_PORT
  localparam logic [31:0] SDHCI_PRESENT_STATE = SDHCI_BASE + 32'h24;
  localparam logic [31:0] SDHCI_NINTR         = SDHCI_BASE + 32'h30; // NINTR_STATUS (W1C)

  localparam logic [31:0] CMD_INHIBIT_MASK    = 32'h0000_0003; // PRESENT_STATE bits 0 (CMD) + 1 (DAT)
  // PRESENT_STATE bits [23:20] mirror the raw DAT[3:0] pad levels
  // (sdhci_top.sv: hw2reg.present_state.dat_line_signal_level = sd_dat_i),
  // so bit 20 is DAT0 -- the line the card pulls low to signal "busy,
  // still programming". See CE_WAIT_CARD_READY.
  localparam logic [31:0] DAT0_LEVEL_MASK     = 32'h0010_0000; // PRESENT_STATE bit 20

  localparam logic [15:0] CMD_COMPLETE_BIT    = 16'h0001; // NINTR_STATUS bit 0
  localparam logic [15:0] XFER_BIT            = 16'h0002; // NINTR_STATUS bit 1 (TRANSFER_COMPLETE)
  localparam logic [15:0] BWR_BIT             = 16'h0010; // NINTR_STATUS bit 4 (BUFFER_WRITE_READY)
  localparam logic [15:0] ERROR_BIT           = 16'h8000; // NINTR_STATUS bit 15 (ERROR_INTERRUPT)

  // COMMAND register (upper 16 bits of SDHCI_XFER_CMD): CMD25
  // (WRITE_MULTIPLE_BLOCK), R1 response (no inherent busy-check -- the
  // write's busy periods happen later, on DAT0, handled internally by SDHCI's
  // own write FSM), CRC/index checked, data phase present.
  localparam logic [7:0]  CMD25_INDEX             = 8'd25;
  localparam int unsigned CMD_INDEX_SHIFT         = 8;
  localparam logic [15:0] CMD_DATA_PRESENT_SELECT = 16'h0020; // bit 5
  localparam logic [15:0] CMD_CRC_CHECK_ENABLE    = 16'h0008; // bit 3
  localparam logic [15:0] CMD_INDEX_CHECK_ENABLE  = 16'h0010; // bit 4
  localparam logic [15:0] CMD_RESP_LEN_48         = 16'h0002; // bits [1:0]
  localparam logic [15:0] CMD25_VALUE             =
    (16'(CMD25_INDEX) << CMD_INDEX_SHIFT) | CMD_DATA_PRESENT_SELECT |
    CMD_CRC_CHECK_ENABLE | CMD_INDEX_CHECK_ENABLE | CMD_RESP_LEN_48;

  // TRANSFER_MODE (lower 16 bits of SDHCI_XFER_CMD), sdhci_reg_top.sv:552-555:
  //   bit 1 block_count_enable  -- honour BLOCK_COUNT, so SDHCI ends the
  //                                session after exactly that many blocks
  //   bit 2 auto_cmd12_enable   -- issue CMD12 automatically at session end
  //   bit 4 data_transfer_direction_select, 0 = write (left clear)
  //   bit 5 multi_single_block_select, 1 = multi-block
  localparam logic [15:0] XFER_MODE_BLOCK_COUNT_EN = 16'h0002;
  localparam logic [15:0] XFER_MODE_AUTO_CMD12     = 16'h0004;
  localparam logic [15:0] XFER_MODE_MULTI_BLOCK    = 16'h0020;
  localparam logic [15:0] XFER_MODE_VALUE          =
    XFER_MODE_MULTI_BLOCK | XFER_MODE_AUTO_CMD12 | XFER_MODE_BLOCK_COUNT_EN;

  // Upper bound (in cycles) on how long any single poll in this FSM retries
  // before giving up and declaring SDCARD_OVERFLOW -- shared by every
  // CE_WAIT_* state and by CE_COPY_WORD's stall window, none of which are ever
  // concurrent. The card needs real time to become idle, accept a command,
  // ready its buffer, or finish programming flash, so every wait depends on
  // this actually waiting instead of checking once and giving up immediately.
  //
  // Sized for a *real* card, not the simulation model: the SD spec permits up
  // to 250 ms for a write, so 25M cycles at 100 MHz. The model needs ~5 us,
  // so this is enormously oversized there -- which costs nothing, since every
  // wait exits on its real condition and only a genuine fault ever runs the
  // counter out.
  //
  // Two of the waits bounded by this genuinely need the full 250 ms, and both
  // are new with the CMD25 session, which is why the previous 500k (5 ms) is
  // no longer adequate:
  //   - CE_WAIT_TRANSFER_COMPLETE now spans every block of the session plus
  //     all the card's inter-block programming, not one block.
  //   - CE_COPY_WORD stalls on the OBI grant for as long as the card stays
  //     busy between blocks, because the SDHCI buffer cannot drain meanwhile.
  //     At 5 ms a perfectly healthy transfer to a slow card would be aborted
  //     as an overflow.
  localparam int unsigned POLL_TIMEOUT = 25'd25_000_000;

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  // 6 bits: the session flow is 34 states, past what [4:0] holds.
  typedef enum logic [5:0] {
    CE_IDLE,                                            // parked: wait for start_i
    CE_CLEAR_STALE_STATUS, CE_CLEAR_STALE_STATUS_RSP,    // discard leftover TRANSFER_COMPLETE/EINTR_STATUS
    CE_WAIT_CMD_INHIBIT, CE_WAIT_CMD_INHIBIT_RSP,        // PRESENT_STATE: card idle?
    CE_SET_BLOCK_SIZE, CE_SET_BLOCK_SIZE_RSP,            // BLOCK_SIZE + BLOCK_COUNT, one write
    CE_SET_TRANSFER_MODE, CE_SET_TRANSFER_MODE_RSP,
    CE_SET_ARGUMENT, CE_SET_ARGUMENT_RSP,
    CE_SUBMIT_CMD25, CE_SUBMIT_CMD25_RSP,                // this is what actually issues CMD25
    CE_WAIT_CMD_COMPLETE, CE_WAIT_CMD_COMPLETE_RSP,
    CE_ACK_CMD_COMPLETE, CE_ACK_CMD_COMPLETE_RSP,
    CE_WAIT_BUFFER_READY, CE_WAIT_BUFFER_READY_RSP,      // card ready for session data?
    CE_COPY_WORD,                                        // pipelined SRAM->SDHCI copy, whole frame
    CE_ACK_BUFFERED, CE_ACK_BUFFERED_RSP,                // W1C-clear BWR
    CE_WAIT_TRANSFER_COMPLETE, CE_WAIT_TRANSFER_COMPLETE_RSP, // one per session
    CE_ACK_TRANSFER_COMPLETE, CE_ACK_TRANSFER_COMPLETE_RSP,
    CE_WAIT_CARD_READY, CE_WAIT_CARD_READY_RSP,          // card idle, incl. AUTO_CMD12?
    CE_CHECK_ERRORS, CE_CHECK_ERRORS_RSP,                // any error latched during the session?
    CE_DONE,                                             // release bank, advance address, maybe SDCARD_DONE
    CE_OVERFLOW                                          // timeout or command error
  } ce_state_e;

  ce_state_e state_d, state_q;
  `FF(state_q, state_d, CE_IDLE, clk_i, rst_ni)

  // Latched job parameters
  logic copying_f0_d, copying_f0_q;
  `FF(copying_f0_q, copying_f0_d, 1'b1, clk_i, rst_ni)

  // 1 = this job streams F0 *and* F1 in one session (SDCARD_PULSE).
  logic copying_both_d, copying_both_q;
  `FF(copying_both_q, copying_both_d, 1'b0, clk_i, rst_ni)

  logic is_last_frame_d, is_last_frame_q;
  `FF(is_last_frame_q, is_last_frame_d, 1'b0, clk_i, rst_ni)

  // Total word count of the job, latched at job start: one frame's
  // (Fx_END_ADDR - Fx_START_ADDR + 1), or both frames' added together for a
  // both-banks job -- see header comment. 12 bits: a both-banks job spans
  // two whole SRAM banks (4 KiB = 1024 words), and the width is one bit
  // wider than that needs so larger banks do not become a silent truncation.
  logic [11:0] frame_words_d, frame_words_q;
  `FF(frame_words_q, frame_words_d, 12'd512, clk_i, rst_ni)

  // Words contributed by F0, i.e. the index at which a both-banks job
  // switches its read base from F0 to F1. Only meaningful when
  // copying_both_q; latched alongside frame_words_q.
  logic [11:0] first_bank_words_d, first_bank_words_q;
  `FF(first_bank_words_q, first_bank_words_d, 12'd512, clk_i, rst_ni)

  // Copy counters
  logic [11:0] rd_idx_d, rd_idx_q; // SRAM reads issued (0..frame_words_q)
  logic [11:0] wr_idx_d, wr_idx_q; // SDHCI writes done (0..frame_words_q)
  `FF(rd_idx_q, rd_idx_d, '0, clk_i, rst_ni)
  `FF(wr_idx_q, wr_idx_d, '0, clk_i, rst_ni)

  // Pipeline register between SRAM read and SDHCI write
  logic [31:0] pipe_data_d,  pipe_data_q;
  logic        pipe_valid_d, pipe_valid_q;
  `FF(pipe_data_q,  pipe_data_d,  '0,   clk_i, rst_ni)
  `FF(pipe_valid_q, pipe_valid_d, 1'b0, clk_i, rst_ni)

  // Tracks a SRAM read that has been granted but whose rvalid has not yet
  // landed. Needed because pipe_valid_q is registered: right after a read is
  // granted there is a 1-cycle window where pipe_valid_q still reads 0 even
  // though the pipe slot is about to be filled. Without this flag, a slow
  // copy_write grant (SDHCI backpressure) lets a second SRAM read land in
  // that window and silently overwrite pipe_data_q before it is written out.
  logic rd_pending_d, rd_pending_q;
  `FF(rd_pending_q, rd_pending_d, 1'b0, clk_i, rst_ni)

  // Cycles spent in the current poll. Reset on entry to each CE_WAIT_*
  // state (CE_WAIT_CMD_INHIBIT, CE_WAIT_CMD_COMPLETE, CE_WAIT_BUFFER_READY,
  // CE_WAIT_TRANSFER_COMPLETE,
  // CE_WAIT_CARD_READY); checked against POLL_TIMEOUT in each corresponding
  // *_RSP state. Also reused by CE_COPY_WORD as its stall counter -- see
  // there. One counter suffices since these waits are never concurrent.
  // 25 bits: POLL_TIMEOUT is 25M, past what the previous 20 bits (1.05M) held.
  logic [24:0] poll_timeout_cnt_d, poll_timeout_cnt_q;
  `FF(poll_timeout_cnt_q, poll_timeout_cnt_d, '0, clk_i, rst_ni)

  // CE_WAIT_CARD_READY requires its pass condition on two *consecutive* polls;
  // this remembers whether the previous poll passed. See that state for why one
  // poll is not sufficient.
  logic card_ready_once_d, card_ready_once_q;
  `FF(card_ready_once_q, card_ready_once_d, 1'b0, clk_i, rst_ni)

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  assign busy_o = (state_q != CE_IDLE);

  // Byte address of the bank the job *starts* in. For a both-banks job this
  // is always F0 (adc_acquisition_top drives copy_f0_i high for it, and
  // copying_f0_d forces it anyway), and sram_rd_addr below switches to F1
  // partway through.
  logic [31:0] sram_copy_base;
  assign sram_copy_base = copying_f0_q
    ? {reg2hw.F0_START_ADDR.WORD_ADDRESS.value, 2'b00}
    : {reg2hw.F1_START_ADDR.WORD_ADDRESS.value, 2'b00};

  // Address of the word CE_COPY_WORD reads next. A single-bank job walks
  // sram_copy_base linearly; a both-banks job walks F0 for its
  // first_bank_words_q words and then restarts from F1_START_ADDR, so F1
  // does not have to abut F0 in the address map for the session to be one
  // contiguous stream on the card.
  logic [11:0] rd_idx_in_bank;
  logic [31:0] sram_rd_addr;
  always_comb begin
    if (copying_both_q && rd_idx_q >= first_bank_words_q) begin
      rd_idx_in_bank = rd_idx_q - first_bank_words_q;
      sram_rd_addr   = {reg2hw.F1_START_ADDR.WORD_ADDRESS.value, 2'b00}
                       + {rd_idx_in_bank, 2'b00};
    end else begin
      rd_idx_in_bank = rd_idx_q;
      sram_rd_addr   = sram_copy_base + {rd_idx_in_bank, 2'b00};
    end
  end

  // Word counts of the two frames, and of the job start_i is about to
  // select, sampled combinatorially off copy_f0_i/copy_both_i (same timing
  // as copying_f0_d/copying_both_d) so they're valid the cycle start_i fires.
  // Fx_END_ADDR is the address of the frame's *last* word (inclusive, matches
  // f0_frame_just_filled in adc_acquisition_top.sv and every sw example's
  // "F0_END_ADDR_BYTE = F0_START_ADDR_BYTE + 512 - 4" convention), so the
  // word count is the START/END word-address span plus 1, not the raw
  // difference -- omitting the +1 undercounts by exactly one word.
  logic [29:0] f0_words, f1_words;
  assign f0_words =
    (reg2hw.F0_END_ADDR.WORD_ADDRESS.value - reg2hw.F0_START_ADDR.WORD_ADDRESS.value) + 30'd1;
  assign f1_words =
    (reg2hw.F1_END_ADDR.WORD_ADDRESS.value - reg2hw.F1_START_ADDR.WORD_ADDRESS.value) + 30'd1;

  logic [29:0] frame_words_next;
  assign frame_words_next = copy_both_i ? (f0_words + f1_words)
                                        : (copy_f0_i ? f0_words : f1_words);

  // CMD25's argument is SDCARD_BLOCK_ADDR's current value, used as-is
  // regardless of addressing mode -- the mode only changes how much gets
  // added to it afterward (see block_addr_advance_o below). Only the
  // session's *starting* address is ever sent: within a CMD25 session the
  // card advances its own write pointer per block.
  logic [31:0] cmd25_argument;
  assign cmd25_argument = reg2hw.SDCARD_BLOCK_ADDR.BLOCK_ADDR.value;

  // How far to advance SDCARD_BLOCK_ADDR once this *session* completes:
  // block addressing (BLOCK_UNITS=1) advances by the session's block count;
  // byte addressing (BLOCK_UNITS=0) advances by the session's byte count,
  // derived from frame_words_q. The two agree by construction as long as
  // software keeps SDCARD_BLOCK_SIZE * SDCARD_BLOCK_COUNT equal to the
  // session size, since a session's byte count is exactly
  // blocks * block_size -- note "session" is one frame in SDCARD_CONTINUOUS
  // and both frames in SDCARD_PULSE, and frame_words_q already accounts for
  // that, so this is unchanged for either mode.
  assign block_addr_advance_o = reg2hw.SDCARD_ADDR_MODE.BLOCK_UNITS.value
    ? {16'b0, reg2hw.SDCARD_BLOCK_COUNT.BLOCK_COUNT.value}
    : {18'b0, frame_words_q, 2'b00};

  // Builds a single-word OBI request. Every state below assigns
  // copy_read_req_o/copy_write_req_o = obi_read(...)/obi_write(...) instead
  // of repeating the request struct's field list by hand.
  function automatic mgr_obi_req_t obi_read(input logic [31:0] addr);
    obi_read          = '0;
    obi_read.req      = 1'b1;
    obi_read.a.addr   = addr;
    obi_read.a.be     = 4'hf;
  endfunction

  function automatic mgr_obi_req_t obi_write(
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] be
  );
    obi_write          = '0;
    obi_write.req      = 1'b1;
    obi_write.a.addr   = addr;
    obi_write.a.we     = 1'b1;
    obi_write.a.be     = be;
    obi_write.a.wdata  = data;
  endfunction

  // -------------------------------------------------------------------------
  // Combinational FSM
  // -------------------------------------------------------------------------
  always_comb begin : sdcard_fsm
    state_d            = state_q;
    copying_f0_d        = copying_f0_q;
    copying_both_d     = copying_both_q;
    is_last_frame_d    = is_last_frame_q;
    frame_words_d      = frame_words_q;
    first_bank_words_d = first_bank_words_q;
    rd_idx_d           = rd_idx_q;
    wr_idx_d           = wr_idx_q;
    pipe_data_d        = pipe_data_q;
    pipe_valid_d       = pipe_valid_q;
    rd_pending_d       = rd_pending_q;
    poll_timeout_cnt_d = poll_timeout_cnt_q;
    card_ready_once_d  = card_ready_once_q;

    // Default outputs: idle / no-op
    copy_read_req_o  = '0;
    copy_write_req_o = '0;
    sdcard_done_set_o     = 1'b0;
    sdcard_overflow_set_o = 1'b0;
    f0_full_clear_o       = 1'b0;
    f1_full_clear_o       = 1'b0;
    block_addr_incr_o     = 1'b0;

    unique case (state_q)

      // --------------------------------------------------------------------
      CE_IDLE: begin
        // Parked: wait for the top-level coordinator to explicitly start a
        // job. No self-triggering on CONF.MODE/STATUS here -- the decision
        // of *whether* and *which* frame to copy next lives entirely in
        // adc_acquisition_top, including whether SDCARD_OVERFLOW or the
        // per-session frame budget should block further jobs.
        if (start_i) begin
          // A both-banks job always starts in F0, independently of what
          // copy_f0_i happens to say -- stated here rather than relied upon
          // from the caller, since sram_copy_base keys off copying_f0_q.
          copying_f0_d       = copy_both_i | copy_f0_i;
          copying_both_d     = copy_both_i;
          is_last_frame_d    = is_last_frame_i;
          frame_words_d      = frame_words_next[11:0];
          first_bank_words_d = f0_words[11:0];
          state_d            = CE_CLEAR_STALE_STATUS;
        end
      end

      // --------------------------------------------------------------------
      // W1C-clear TRANSFER_COMPLETE and every EINTR_STATUS bit
      // unconditionally before this job's own CMD25 is even issued. Both
      // halves live in the same 32-bit SDHCI_NINTR word (NINTR_STATUS in
      // the low 16 bits, EINTR_STATUS in the high 16), so one OBI write
      // covers both.
      //
      // TRANSFER_COMPLETE fires on any 1->0 edge of command_inhibit_dat
      // (sdhci_reg_logic.sv), which includes busy-checked commands with no
      // data phase at all -- e.g. CMD7 (SELECT_CARD, R1b) during
      // sd_init(), via sd_cmd_dat_busy_i. That's a real event, but nothing
      // clears it afterward (sdh_cmd() only ever clears COMMAND_COMPLETE),
      // so it sits set until *something* polls it. Confirmed via waveform
      // (croc.fst): without this step, the first-ever
      // CE_WAIT_TRANSFER_COMPLETE poll saw a TRANSFER_COMPLETE left over
      // from CMD7 and returned immediately, while dat_wrap.sv's
      // write_state_q was still WRITING -- i.e. done_o/busy_o dropped
      // while the card was still genuinely streaming data.
      //
      // EINTR_STATUS.data_timeout_error latches on *every* block as a
      // harmless side effect of a one-cycle dat0_i sampling race in
      // dat_write.sv (see header comment) -- it does not indicate lost or
      // corrupt data, but it is just as sticky, and ERROR_INTERRUPT
      // (checked in CE_WAIT_CMD_COMPLETE_RSP) is a same-cycle,
      // continuously-recomputed OR of the EINTR_STATUS bits, so clearing
      // the sub-bits alone is enough to bring it back down. Confirmed via
      // waveform at N=2: without this, job 1's leftover data_timeout_error
      // held ERROR_INTERRUPT set into job 2, and job 2's very first
      // CE_WAIT_CMD_COMPLETE_RSP check sent it straight to CE_OVERFLOW.
      //
      // BUFFER_WRITE_READY is cleared here too, and the multi-block design
      // is what makes that mandatory rather than merely tidy. In a CMD25
      // session BWR re-asserts at *every* block boundary
      // (dat_buffer.sv pulses buffer_write_enable low per block to raise it),
      // but this FSM only acks it once, at CE_ACK_BUFFERED. So a session
      // reliably ends with BWR set again from its final block boundary, and
      // nothing else in the system clears it. Without clearing it here, the
      // *next* session's CE_WAIT_BUFFER_READY would match that leftover on
      // its first poll and start streaming before the card had asked for
      // data -- structurally the same stale-sticky-status bug as the
      // TRANSFER_COMPLETE and data_timeout_error cases above, which is
      // exactly why all three bits this FSM ever polls are discarded in one
      // write. COMMAND_COMPLETE is included for the same reason even though
      // both of a session's occurrences (CMD25's and AUTO_CMD12's) are
      // individually acked.
      //
      // Clearing here happens before this job's own CMD25 can possibly
      // cause a fresh edge or a fresh error, so there's no race with a
      // real completion or a real error from *this* job.
      CE_CLEAR_STALE_STATUS: begin
        copy_write_req_o = obi_write(
          SDHCI_NINTR, {16'hffff, XFER_BIT | BWR_BIT | CMD_COMPLETE_BIT}, 4'hf);
        if (copy_write_rsp_i.gnt)
          state_d = CE_CLEAR_STALE_STATUS_RSP;
      end

      CE_CLEAR_STALE_STATUS_RSP: begin
        if (copy_write_rsp_i.rvalid) begin
          poll_timeout_cnt_d = '0;
          state_d            = CE_WAIT_CMD_INHIBIT;
        end
      end

      // --------------------------------------------------------------------
      // Card must be idle (no command or data transfer in flight) before a
      // new command can be issued.
      CE_WAIT_CMD_INHIBIT: begin
        copy_write_req_o   = obi_read(SDHCI_PRESENT_STATE);
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.gnt)
          state_d = CE_WAIT_CMD_INHIBIT_RSP;
      end

      CE_WAIT_CMD_INHIBIT_RSP: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.rvalid) begin
          if ((copy_write_rsp_i.r.rdata & CMD_INHIBIT_MASK) == 32'h0) begin
            state_d = CE_SET_BLOCK_SIZE;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_CMD_INHIBIT; // retry
          end
        end
      end

      // --------------------------------------------------------------------
      // Command setup: BLOCK_SIZE, then TRANSFER_MODE, then ARGUMENT, then
      // the COMMAND write itself (CE_SUBMIT_CMD25) that actually issues
      // CMD25. TRANSFER_MODE and COMMAND share one physical 32-bit SDHCI
      // register (different byte lanes) but are written separately -- see
      // header comment.
      CE_SET_BLOCK_SIZE: begin
        // One write covers both halves of the 0x04 register:
        // BLOCK_SIZE.transfer_block_size in bits [11:0] and BLOCK_COUNT in
        // [31:16]. Unlike the per-block design, the block size comes from
        // SDCARD_BLOCK_SIZE rather than the frame's byte count -- a frame is
        // now SDCARD_BLOCK_COUNT blocks, not one.
        copy_write_req_o = obi_write(
          SDHCI_BLOCK_SIZE,
          {reg2hw.SDCARD_BLOCK_COUNT.BLOCK_COUNT.value,
           4'b0, reg2hw.SDCARD_BLOCK_SIZE.BLOCK_SIZE.value},
          4'hf);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_BLOCK_SIZE_RSP;
      end

      CE_SET_BLOCK_SIZE_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SET_TRANSFER_MODE;
      end

      CE_SET_TRANSFER_MODE: begin
        // Multi-block + block-count-enable + AUTO_CMD12, write direction
        // (direction bit left clear). block-count-enable is what bounds the
        // session to BLOCK_COUNT blocks and therefore what makes
        // TRANSFER_COMPLETE fire exactly once, at the end of the frame.
        copy_write_req_o = obi_write(SDHCI_XFER_CMD, {16'h0, XFER_MODE_VALUE}, 4'h3);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_TRANSFER_MODE_RSP;
      end

      CE_SET_TRANSFER_MODE_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SET_ARGUMENT;
      end

      CE_SET_ARGUMENT: begin
        copy_write_req_o = obi_write(SDHCI_ARGUMENT, cmd25_argument, 4'hf);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_ARGUMENT_RSP;
      end

      CE_SET_ARGUMENT_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SUBMIT_CMD25;
      end

      // This write is what actually issues CMD25.
      CE_SUBMIT_CMD25: begin
        copy_write_req_o = obi_write(SDHCI_XFER_CMD, {CMD25_VALUE, 16'h0}, 4'hc);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SUBMIT_CMD25_RSP;
      end

      CE_SUBMIT_CMD25_RSP: begin
        if (copy_write_rsp_i.rvalid) begin
          poll_timeout_cnt_d = '0;
          state_d            = CE_WAIT_CMD_COMPLETE;
        end
      end

      // --------------------------------------------------------------------
      CE_WAIT_CMD_COMPLETE: begin
        copy_write_req_o   = obi_read(SDHCI_NINTR);
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.gnt)
          state_d = CE_WAIT_CMD_COMPLETE_RSP;
      end

      // An ERROR_INTERRUPT here means the command wasn't cleanly accepted
      // (CRC/index/timeout on the command itself) -- treat it the same as
      // a timeout rather than proceeding into a data phase for a command
      // that may not have landed.
      CE_WAIT_CMD_COMPLETE_RSP: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.rvalid) begin
          if (copy_write_rsp_i.r.rdata[15]) begin // ERROR_INTERRUPT
            state_d = CE_OVERFLOW;
          end else if (copy_write_rsp_i.r.rdata[0]) begin // COMMAND_COMPLETE
            state_d = CE_ACK_CMD_COMPLETE;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_CMD_COMPLETE; // retry
          end
        end
      end

      CE_ACK_CMD_COMPLETE: begin
        copy_write_req_o = obi_write(SDHCI_NINTR, {16'h0, CMD_COMPLETE_BIT}, 4'h3);
        if (copy_write_rsp_i.gnt)
          state_d = CE_ACK_CMD_COMPLETE_RSP;
      end

      CE_ACK_CMD_COMPLETE_RSP: begin
        if (copy_write_rsp_i.rvalid) begin
          poll_timeout_cnt_d = '0;
          state_d            = CE_WAIT_BUFFER_READY;
        end
      end

      // --------------------------------------------------------------------
      CE_WAIT_BUFFER_READY: begin
        copy_write_req_o   = obi_read(SDHCI_NINTR);
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.gnt)
          state_d = CE_WAIT_BUFFER_READY_RSP;
      end

      CE_WAIT_BUFFER_READY_RSP: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.rvalid) begin
          if (copy_write_rsp_i.r.rdata[4]) begin // BWR set
            rd_idx_d           = '0;
            wr_idx_d           = '0;
            pipe_valid_d       = 1'b0;
            rd_pending_d       = 1'b0;
            poll_timeout_cnt_d = '0; // CE_COPY_WORD reuses this as its stall window
            state_d            = CE_COPY_WORD;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_BUFFER_READY; // retry
          end
        end
      end

      // --------------------------------------------------------------------
      // Copy the whole job, one word at a time, straight through every
      // block boundary in the session -- there is no per-block handshake --
      // and, for a both-banks job, straight through the F0 -> F1 bank
      // switch too (sram_rd_addr), which is invisible on the SD side.
      //
      // READ side (copy_read -> SRAM): issue next read only once the pipe
      //   slot is fully free -- no read outstanding (rd_pending_q) and no
      //   unwritten word waiting (pipe_valid_q). SRAM grants combinatorially
      //   and returns rvalid 1 cycle later, but rd_pending_q is still needed:
      //   pipe_valid_q is registered, so without it a read could be issued
      //   into the 1-cycle window before pipe_valid_q reflects the previous
      //   read's result.
      //
      // WRITE side (copy_write -> SDHCI DATA): drains the pipeline register.
      //   SDHCI withholds the grant whenever its 1 KiB DAT buffer is full
      //   (dat_buffer.sv's buffer_data_port_write_ready_o -> reg_ready ->
      //   gnt), which streaming a 2 KiB frame through it makes routine, not
      //   exceptional. The read side waiting on pipe_valid_q/rd_pending_q is
      //   what makes the copy correct under that backpressure instead of
      //   silently overwriting unwritten data.
      //
      // Stall timeout: poll_timeout_cnt_q counts cycles since the last word
      // was accepted and is cleared on every accepted word, so it measures a
      // continuous stall rather than the copy's total duration (which
      // legitimately exceeds POLL_TIMEOUT for a large frame on a slow card).
      // Without it a mis-sized session hangs the FSM forever rather than
      // reporting: once SDHCI has taken BLOCK_COUNT blocks,
      // accepts_data_port_chunk (dat_buffer.sv) goes low permanently and no
      // further word is ever granted.
      CE_COPY_WORD: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;

        // --- READ side ---
        if (rd_idx_q < frame_words_q && !pipe_valid_q && !rd_pending_q) begin
          copy_read_req_o = obi_read(sram_rd_addr);
          if (copy_read_rsp_i.gnt) begin
            rd_idx_d     = rd_idx_q + 1;
            rd_pending_d = 1'b1;
          end
        end
        if (copy_read_rsp_i.rvalid) begin
          pipe_data_d  = copy_read_rsp_i.r.rdata;
          pipe_valid_d = 1'b1;
          rd_pending_d = 1'b0;
        end

        // --- WRITE side ---
        if (pipe_valid_q) begin
          copy_write_req_o = obi_write(SDHCI_DATA, pipe_data_q, 4'hf);
          if (copy_write_rsp_i.gnt) begin
            pipe_valid_d       = 1'b0;
            wr_idx_d           = wr_idx_q + 1;
            poll_timeout_cnt_d = '0; // progress: restart the stall window
          end
        end

        if (wr_idx_q == frame_words_q) begin
          poll_timeout_cnt_d = '0;
          state_d            = CE_ACK_BUFFERED;
        end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
          state_d = CE_OVERFLOW;
        end
      end

      // --------------------------------------------------------------------
      // W1C-clear BUFFER_WRITE_READY now that every word of the frame has
      // been handed to SDHCI.
      CE_ACK_BUFFERED: begin
        copy_write_req_o = obi_write(SDHCI_NINTR, {16'h0, BWR_BIT}, 4'h3);
        if (copy_write_rsp_i.gnt)
          state_d = CE_ACK_BUFFERED_RSP;
      end

      CE_ACK_BUFFERED_RSP: begin
        if (copy_write_rsp_i.rvalid) begin
          poll_timeout_cnt_d = '0;
          state_d            = CE_WAIT_TRANSFER_COMPLETE;
        end
      end

      // --------------------------------------------------------------------
      // One condition, and it fires exactly once per session. With
      // block-count-enable set, dat_wrap.sv's write FSM loops
      // DONE_WRITING_BLOCK -> WAIT_FOR_WRITE_BUFFER for each block and only
      // reaches DONE_WRITING when transmitted_block_counter hits 1; that is
      // what returns dat_state to READY, drops command_inhibit_dat, and sets
      // TRANSFER_COMPLETE (sdhci_reg_logic.sv). BUFFER_WRITE_READY does
      // re-assert per block in a multi-block session, but nothing here reads
      // it -- the old design's BWR-vs-TRANSFER_COMPLETE race does not exist
      // because per-block events are not used for anything.
      CE_WAIT_TRANSFER_COMPLETE: begin
        copy_write_req_o   = obi_read(SDHCI_NINTR);
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.gnt)
          state_d = CE_WAIT_TRANSFER_COMPLETE_RSP;
      end

      CE_WAIT_TRANSFER_COMPLETE_RSP: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.rvalid) begin
          if (copy_write_rsp_i.r.rdata[1]) begin // TRANSFER_COMPLETE
            state_d = CE_ACK_TRANSFER_COMPLETE;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_TRANSFER_COMPLETE; // retry
          end
        end
      end

      CE_ACK_TRANSFER_COMPLETE: begin
        copy_write_req_o = obi_write(SDHCI_NINTR, {16'h0, XFER_BIT}, 4'h3);
        if (copy_write_rsp_i.gnt)
          state_d = CE_ACK_TRANSFER_COMPLETE_RSP;
      end

      CE_ACK_TRANSFER_COMPLETE_RSP: begin
        if (copy_write_rsp_i.rvalid) begin
          poll_timeout_cnt_d = '0;
          card_ready_once_d  = 1'b0;
          state_d            = CE_WAIT_CARD_READY;
        end
      end

      // --------------------------------------------------------------------
      // TRANSFER_COMPLETE is a *host controller* event, not a *card* event.
      // It means SDHCI's own write FSM finished framing the session's last
      // block on the
      // bus (data, CRC, CRC-status token); the card only starts its
      // internal program-to-flash cycle at that point, and signals it by
      // holding DAT0 low for the whole duration (SD spec busy signalling).
      // While DAT0 is low the card sits in PRG state and rejects any new
      // command outright. So the block is *not* committed when
      // TRANSFER_COMPLETE fires -- it is committed when DAT0 comes back up.
      //
      // Confirmed via waveform (croc.fst) at N=2, which is exactly what the
      // "flash dump only shows one block" symptom was:
      //   cyc 874077  card enters PRG, sdModel dataState = WRITE_FLASH
      //   cyc 874095  TRANSFER_COMPLETE rises -- with DAT0 already *low*
      //   cyc 874099  job 1 reached CE_DONE (~500 cycles too early)
      //   cyc 874111  job 2 issued its CMD24 ...
      //   cyc 874200  ... which reached a card still in PRG and was
      //               dropped on the floor -- sdModel.v's `24:` handler
      //               only runs its body `if (CardStatus[12:9] == TRAN)`,
      //               and the else branch just returns an empty response
      //               with no error bit, so nothing on the host side
      //               noticed. Job 2 then streamed a full block of data
      //               into a card that had never been told to write it,
      //               and still saw a TRANSFER_COMPLETE at the end.
      //   cyc 874605  card finally leaves PRG (DAT0 released) -- long after
      //               job 2 had already come and gone.
      // Net effect: exactly one WRITE_FLASH episode for the whole run, so
      // block 0 landed and block 1 silently vanished. Note the SDCARD_BLOCK_ADDR
      // advance was never the problem: the argument did go 0 -> 1 correctly,
      // the card just never accepted the command carrying it.
      //
      // Polling DAT0's level here is race-free by protocol construction:
      // the busy period starts contiguously with the CRC-status token that
      // TRANSFER_COMPLETE itself is derived from, so by the time this state
      // can issue its first read (a full OBI round trip after the
      // TRANSFER_COMPLETE poll plus the W1C ack), DAT0 is already low if
      // the card intends to be busy at all. A card that does not go busy
      // simply reads back high on the first poll and falls straight through
      // -- no hang. CMD_INHIBIT_CMD/DAT are folded into the same check so a
      // single read covers both "host controller idle" and "card idle".
      //
      // This wait also covers AUTO_CMD12, which is why there is no separate
      // state for it. There cannot be one: SDHCI never reports COMMAND_COMPLETE
      // for an auto command, because autocmd_wrap.sv deliberately masks the
      // status bit the interrupt is derived from --
      //   command_inhibit_cmd_o.d = driver_cmd_queued_q |
      //                             (cmd_inhibit_logic && ~running_autocmd12_q)
      // -- so present_state.command_inhibit_cmd never rises for CMD12 and its
      // 1->0 edge, which is what sets COMMAND_COMPLETE, never happens. (Per
      // spec: Auto CMD12 does not generate Command Complete. An earlier
      // revision waited for one here and hung until POLL_TIMEOUT.) What *does*
      // cover CMD12 is CMD_INHIBIT_DAT, via dat_state entering BUSY for its
      // R1b response.
      //
      // Hence the two-consecutive-poll requirement. TRANSFER_COMPLETE fires on
      // command_inhibit_dat's 1->0 edge, and AUTO_CMD12 re-raises it one cycle
      // later when the command is accepted -- so there is a 1-cycle window in
      // which everything reads idle while CMD12 has not even started. A single
      // passing poll landing there would release the bank mid-session.
      // Requiring two consecutive passes closes it without depending on
      // arithmetic about OBI round-trip length: the CMD12 busy window is ~107
      // cycles wide against a poll interval of ~3, so a poll that lands in the
      // gap is always followed by one that does not.
      //
      // Do not be tempted to lean on DAT0 here instead. It happens to stay
      // high across that window against sdModel.v, whose CMD12 asserts no R1b
      // busy at all -- checked on the waveform -- so DAT0 provides no coverage
      // of this race even though it is the right signal for the card's own
      // programming busy afterwards.
      //
      // Note on POLL_TIMEOUT: this wait's natural duration is set by flash
      // programming rather than by bus turnaround, which is what drove
      // POLL_TIMEOUT's sizing -- 25M cycles = 250 ms at 100 MHz, the SD spec's
      // permitted worst case for a write. Against this testbench model (~500
      // cycles) that is vastly oversized and costs nothing.
      CE_WAIT_CARD_READY: begin
        copy_write_req_o   = obi_read(SDHCI_PRESENT_STATE);
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.gnt)
          state_d = CE_WAIT_CARD_READY_RSP;
      end

      CE_WAIT_CARD_READY_RSP: begin
        poll_timeout_cnt_d = poll_timeout_cnt_q + 1'b1;
        if (copy_write_rsp_i.rvalid) begin
          if (((copy_write_rsp_i.r.rdata & CMD_INHIBIT_MASK) == 32'h0) &&
              ((copy_write_rsp_i.r.rdata & DAT0_LEVEL_MASK) != 32'h0)) begin
            // Idle this poll -- but only act on it if the previous poll agreed.
            card_ready_once_d = 1'b1;
            if (card_ready_once_q) begin
              poll_timeout_cnt_d = '0;
              state_d            = CE_CHECK_ERRORS;
            end else begin
              state_d = CE_WAIT_CARD_READY; // confirm once more
            end
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            card_ready_once_d = 1'b0; // not idle: restart the confirmation
            state_d           = CE_WAIT_CARD_READY;
          end
        end
      end

      // --------------------------------------------------------------------
      // Final gate before the bank is released: one NINTR_STATUS read to catch
      // anything the session latched but no earlier state polled for. Notably
      // AUTO_CMD12 failures, which set EINTR_STATUS.auto_cmd12_error (bit 8)
      // and through it ERROR_INTERRUPT -- with no COMMAND_COMPLETE to check,
      // this is the only place an AUTO_CMD12 problem can be noticed at all.
      // Also catches data CRC/end-bit errors raised late in the data phase.
      CE_CHECK_ERRORS: begin
        copy_write_req_o = obi_read(SDHCI_NINTR);
        if (copy_write_rsp_i.gnt)
          state_d = CE_CHECK_ERRORS_RSP;
      end

      CE_CHECK_ERRORS_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = copy_write_rsp_i.r.rdata[15] ? CE_OVERFLOW : CE_DONE;
      end

      // --------------------------------------------------------------------
      CE_OVERFLOW: begin
        sdcard_overflow_set_o = 1'b1;
        state_d = CE_IDLE;
      end

      // --------------------------------------------------------------------
      // Every job releases the bank(s) it consumed and advances the address
      // the same way; only the capture's actual last frame (is_last_frame_q,
      // latched at job start) also asserts SDCARD_DONE. The advance is one
      // whole session's worth (SDCARD_BLOCK_COUNT blocks / the session's byte
      // count), not one block -- see block_addr_advance_o.
      //
      // A both-banks job releases *both* banks at once, and only here: F0
      // stays flagged full for the whole session even though its words were
      // streamed first, because the ADC must not start refilling it while
      // the session is still in flight. (In SDCARD_PULSE the ADC has already
      // parked on capture_done_q, so this is belt and braces -- but the
      // release is what the mode's software re-arm sees, and releasing F0
      // early would make Fx_FULL mean something different for a job that has
      // not physically committed yet.)
      CE_DONE: begin
        block_addr_incr_o = 1'b1;
        if (copying_both_q) begin
          f0_full_clear_o = 1'b1;
          f1_full_clear_o = 1'b1;
        end else if (copying_f0_q) begin
          f0_full_clear_o = 1'b1;
        end else begin
          f1_full_clear_o = 1'b1;
        end
        if (is_last_frame_q)
          sdcard_done_set_o = 1'b1;
        state_d = CE_IDLE;
      end

      default: state_d = CE_IDLE;
    endcase
  end

endmodule
