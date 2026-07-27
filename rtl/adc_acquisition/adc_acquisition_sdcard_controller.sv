// SDCard copy controller for ADC acquisition pipeline.
//
// Software is responsible for SD card initialisation (CMD0..CMD16: reset,
// identify, select, 4-bit bus, block length) before enabling ACQ_SDCARD
// mode. From that point on, this module owns the SD protocol for every
// block by itself: it issues its own CMD24 (WRITE_BLOCK) per frame, back
// to back, one at a time -- there is no multi-block CMD25/AUTO_CMD12
// session to open or close. This is a deliberate departure from an
// earlier multi-block design: with CMD25(BLOCK_COUNT=N), only the
// session's *last* block gets a real TRANSFER_COMPLETE, every other block
// is judged done via BUFFER_WRITE_READY re-asserting, which reflects
// SDHCI-internal double-buffer occupancy, not physical completion of the
// specific block just copied. Per-block CMD24 sidesteps that: SDHCI
// forces its internal block counter to 1 for single-block mode
// (dat_wrap.sv's new_block_count computation) regardless of BLOCK_COUNT,
// so TRANSFER_COMPLETE fires directly off *this* block's real completion,
// every time, with no double-buffering ambiguity.
//
// This module is a passive subordinate to adc_acquisition_top's control
// logic: it does not watch CONF.MODE or STATUS.Fx_FULL itself, it only
// reacts to an explicit start_i pulse (with copy_f0_i selecting which
// frame, is_last_frame_i marking whether this is the capture's final
// frame) and reports back via busy_o plus the existing *_set_o/*_clear_o
// pulses. It never re-arms itself -- every copy job is initiated from the
// outside, one at a time, and the module parks in CE_IDLE (busy_o low)
// between jobs. busy_o now spans the *entire* per-block transaction. from
// issuing CMD24 through the card's physical write completing. there is no
// state where the module reports idle while a block is still in flight.
//
// Per job:
//   1. On start_i, latch which frame to copy (copy_f0_i), its word count
//      (Fx_END_ADDR - Fx_START_ADDR + 1), and whether this is the
//      capture's last frame (is_last_frame_i).
//   2. CE_CLEAR_STALE_STATUS: W1C-clear TRANSFER_COMPLETE and every
//      EINTR_STATUS bit unconditionally, before this job's own CMD24 is
//      even issued. TRANSFER_COMPLETE fires on any 1->0 edge of
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
//      Against this repo's sdModel.v, DAT0 is not yet low at that exact
//      sample, so data_timeout_d latches -- confirmed via waveform
//      (croc.fst): dat_tx_state_q enters STATUS_START_BIT for exactly
//      one cycle with dat0_i=1, data_timeout_q latches the next cycle,
//      and the block still completes correctly two cycles later
//      (write_done and TRANSFER_COMPLETE both fire on schedule,
//      unaffected). The flag is purely a passenger: dat_write.sv's own
//      FSM does not stall or branch away from DONE because of it, it
//      only causes dat_wrap.sv's outer write_state_q to detour through
//      TIMEOUT_WRITING (instead of DONE_WRITING_BLOCK) on its way to
//      DONE_WRITING, which is where EINTR_STATUS.data_timeout_error
//      actually gets set. Like TRANSFER_COMPLETE, nothing clears it
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
//      CE_SUBMIT_CMD24: write BLOCK_SIZE (this frame's byte count),
//      TRANSFER_MODE (all-zero: single-block, write direction, no
//      AUTO_CMD12, no block-count-enable -- none of that machinery is
//      needed for a plain single-block write), ARGUMENT (the current
//      SDCARD_BLOCK_ADDR value, used as-is regardless of addressing
//      mode -- see block_addr_advance below), then COMMAND itself
//      (index 24, R1 response, DATA_PRESENT_SELECT). TRANSFER_MODE and
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
//   6. CE_ACK_CMD_COMPLETE: W1C-clear COMMAND_COMPLETE.
//   7. CE_WAIT_BUFFER_READY: poll for BUFFER_WRITE_READY (bit 4) -- the
//      card signals it's ready for this block's data.
//   8. CE_COPY_WORD: pipelined SRAM -> SDHCI BUFFER_DATA_PORT copy, one
//      word in flight at a time. The next SRAM read is only issued once
//      the previous word has actually been written (copy_write grant
//      seen), since copy_write can be backpressured by SDHCI for an
//      arbitrary number of cycles.
//   9. CE_ACK_BUFFERED: W1C-clear BWR -- tells SDHCI this block's words
//      are all in its buffer.
//  10. CE_WAIT_TRANSFER_COMPLETE: poll for TRANSFER_COMPLETE (bit 1)
//      only -- unlike the old multi-block design there is no second
//      condition to race against: BUFFER_WRITE_READY structurally cannot
//      re-assert for a single-block command (SDHCI's internal block
//      counter is pinned to 1), so TRANSFER_COMPLETE is the only outcome
//      that ever arrives here, and (now that step 2 discards any stale
//      leftover) it genuinely means this specific block is physically
//      written and the card is idle again.
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
//  13. CE_DONE: release the bank (Fx_FULL clear), advance
//      SDCARD_BLOCK_ADDR by block_addr_advance_o (+1 for block
//      addressing, +this frame's byte count for byte addressing -- see
//      SDCARD_ADDR_MODE), and -- only if this was the capture's last
//      frame (is_last_frame_q, latched from is_last_frame_i at step 1) --
//      set SDCARD_DONE. Every other frame releases its bank and advances
//      the address exactly the same way, just without SDCARD_DONE.
//
// Error handling: command-level errors (step 5) fall through to
// CE_OVERFLOW; data/CRC errors on the block itself are still not
// decoded. SDHCI EINTR_STATUS remains visible to software for
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
  input  logic start_i,          // pulse: begin copying one frame
  input  logic copy_f0_i,        // 1 = copy F0, 0 = copy F1 (sampled on start_i)
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

  // COMMAND register (upper 16 bits of SDHCI_XFER_CMD): CMD24 (WRITE_BLOCK),
  // R1 response (no inherent busy-check -- the write's busy period happens
  // later, on DAT0, handled internally by SDHCI's own write FSM), CRC/index
  // checked, data phase present.
  localparam logic [7:0]  CMD24_INDEX             = 8'd24;
  localparam int unsigned CMD_INDEX_SHIFT         = 8;
  localparam logic [15:0] CMD_DATA_PRESENT_SELECT = 16'h0020; // bit 5
  localparam logic [15:0] CMD_CRC_CHECK_ENABLE    = 16'h0008; // bit 3
  localparam logic [15:0] CMD_INDEX_CHECK_ENABLE  = 16'h0010; // bit 4
  localparam logic [15:0] CMD_RESP_LEN_48         = 16'h0002; // bits [1:0]
  localparam logic [15:0] CMD24_VALUE             =
    (16'(CMD24_INDEX) << CMD_INDEX_SHIFT) | CMD_DATA_PRESENT_SELECT |
    CMD_CRC_CHECK_ENABLE | CMD_INDEX_CHECK_ENABLE | CMD_RESP_LEN_48;

  // Upper bound (in cycles) on how long any single poll in this FSM retries
  // before giving up and declaring SDCARD_OVERFLOW -- shared by every
  // CE_WAIT_* state, none of which are ever concurrent. The card needs real
  // time to become idle, accept a command, ready its buffer, or finish
  // programming flash, so every wait depends on this actually waiting
  // instead of checking once and giving up immediately.
  localparam int unsigned POLL_TIMEOUT = 20'd500_000;

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    CE_IDLE,                                            // parked: wait for start_i
    CE_CLEAR_STALE_STATUS, CE_CLEAR_STALE_STATUS_RSP,    // discard leftover TRANSFER_COMPLETE/EINTR_STATUS
    CE_WAIT_CMD_INHIBIT, CE_WAIT_CMD_INHIBIT_RSP,        // PRESENT_STATE: card idle?
    CE_SET_BLOCK_SIZE, CE_SET_BLOCK_SIZE_RSP,
    CE_SET_TRANSFER_MODE, CE_SET_TRANSFER_MODE_RSP,
    CE_SET_ARGUMENT, CE_SET_ARGUMENT_RSP,
    CE_SUBMIT_CMD24, CE_SUBMIT_CMD24_RSP,                // this is what actually issues CMD24
    CE_WAIT_CMD_COMPLETE, CE_WAIT_CMD_COMPLETE_RSP,
    CE_ACK_CMD_COMPLETE, CE_ACK_CMD_COMPLETE_RSP,
    CE_WAIT_BUFFER_READY, CE_WAIT_BUFFER_READY_RSP,      // room for this block's data?
    CE_COPY_WORD,                                        // pipelined SRAM->SDHCI copy
    CE_ACK_BUFFERED, CE_ACK_BUFFERED_RSP,                // W1C-clear BWR
    CE_WAIT_TRANSFER_COMPLETE, CE_WAIT_TRANSFER_COMPLETE_RSP, // single condition now, no race
    CE_ACK_TRANSFER_COMPLETE, CE_ACK_TRANSFER_COMPLETE_RSP,
    CE_WAIT_CARD_READY, CE_WAIT_CARD_READY_RSP,          // card off DAT0-busy (PRG -> TRAN)?
    CE_DONE,                                             // release bank, advance address, maybe SDCARD_DONE
    CE_OVERFLOW                                          // timeout or command error
  } ce_state_e;

  ce_state_e state_d, state_q;
  `FF(state_q, state_d, CE_IDLE, clk_i, rst_ni)

  // Latched job parameters
  logic copying_f0_d, copying_f0_q;
  `FF(copying_f0_q, copying_f0_d, 1'b1, clk_i, rst_ni)

  logic is_last_frame_d, is_last_frame_q;
  `FF(is_last_frame_q, is_last_frame_d, 1'b0, clk_i, rst_ni)

  // Word count of the frame being copied, latched at job start from
  // (Fx_END_ADDR - Fx_START_ADDR + 1) -- see header comment.
  logic [9:0] frame_words_d, frame_words_q;
  `FF(frame_words_q, frame_words_d, 10'd128, clk_i, rst_ni)

  // Copy counters
  logic [9:0] rd_idx_d, rd_idx_q; // SRAM reads issued (0..frame_words_q)
  logic [9:0] wr_idx_d, wr_idx_q; // SDHCI writes done (0..frame_words_q)
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
  // CE_WAIT_TRANSFER_COMPLETE); checked against POLL_TIMEOUT in each
  // corresponding *_RSP state. One counter suffices since these waits are
  // never concurrent.
  logic [19:0] poll_timeout_cnt_d, poll_timeout_cnt_q;
  `FF(poll_timeout_cnt_q, poll_timeout_cnt_d, '0, clk_i, rst_ni)

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  assign busy_o = (state_q != CE_IDLE);

  // Byte address of the frame being copied
  logic [31:0] sram_copy_base;
  assign sram_copy_base = copying_f0_q
    ? {reg2hw.F0_START_ADDR.WORD_ADDRESS.value, 2'b00}
    : {reg2hw.F1_START_ADDR.WORD_ADDRESS.value, 2'b00};

  // Word count of the frame start_i is about to select, sampled combinatorially
  // off copy_f0_i (same timing as copying_f0_d) so it's valid the cycle start_i fires.
  // Fx_END_ADDR is the address of the frame's *last* word (inclusive, matches
  // f0_frame_just_filled in adc_acquisition_top.sv and every sw example's
  // "F0_END_ADDR_BYTE = F0_START_ADDR_BYTE + 512 - 4" convention), so the
  // word count is the START/END word-address span plus 1, not the raw
  // difference -- omitting the +1 undercounts by exactly one word.
  logic [29:0] frame_words_next;
  assign frame_words_next = copy_f0_i
    ? (reg2hw.F0_END_ADDR.WORD_ADDRESS.value - reg2hw.F0_START_ADDR.WORD_ADDRESS.value) + 30'd1
    : (reg2hw.F1_END_ADDR.WORD_ADDRESS.value - reg2hw.F1_START_ADDR.WORD_ADDRESS.value) + 30'd1;

  // CMD24's argument is SDCARD_BLOCK_ADDR's current value, used as-is
  // regardless of addressing mode -- the mode only changes how much gets
  // added to it afterward (see block_addr_advance_o below).
  logic [31:0] cmd24_argument;
  assign cmd24_argument = reg2hw.SDCARD_BLOCK_ADDR.BLOCK_ADDR.value;

  // How far to advance SDCARD_BLOCK_ADDR once this block completes:
  // block addressing (BLOCK_UNITS=1) advances by one block; byte addressing
  // (BLOCK_UNITS=0) advances by this frame's byte count, derived from
  // frame_words_q so block size has a single source of truth.
  assign block_addr_advance_o = reg2hw.SDCARD_ADDR_MODE.BLOCK_UNITS.value
    ? 32'd1
    : {20'b0, frame_words_q, 2'b00};

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
    is_last_frame_d    = is_last_frame_q;
    frame_words_d      = frame_words_q;
    rd_idx_d           = rd_idx_q;
    wr_idx_d           = wr_idx_q;
    pipe_data_d        = pipe_data_q;
    pipe_valid_d       = pipe_valid_q;
    rd_pending_d       = rd_pending_q;
    poll_timeout_cnt_d = poll_timeout_cnt_q;

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
          copying_f0_d       = copy_f0_i;
          is_last_frame_d    = is_last_frame_i;
          frame_words_d      = frame_words_next[9:0];
          state_d            = CE_CLEAR_STALE_STATUS;
        end
      end

      // --------------------------------------------------------------------
      // W1C-clear TRANSFER_COMPLETE and every EINTR_STATUS bit
      // unconditionally before this job's own CMD24 is even issued. Both
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
      // Clearing here happens before this job's own CMD24 can possibly
      // cause a fresh edge or a fresh error, so there's no race with a
      // real completion or a real error from *this* job.
      CE_CLEAR_STALE_STATUS: begin
        copy_write_req_o = obi_write(SDHCI_NINTR, {16'hffff, XFER_BIT}, 4'hf);
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
      // the COMMAND write itself (CE_SUBMIT_CMD24) that actually issues
      // CMD24. TRANSFER_MODE and COMMAND share one physical 32-bit SDHCI
      // register (different byte lanes) but are written separately -- see
      // header comment.
      CE_SET_BLOCK_SIZE: begin
        // BLOCK_SIZE.transfer_block_size is bits [11:0] of the low
        // halfword; frame_words_q*4 (this frame's byte count) fits there
        // for any realistic frame size.
        copy_write_req_o = obi_write(SDHCI_BLOCK_SIZE, {20'b0, frame_words_q, 2'b00}, 4'h3);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_BLOCK_SIZE_RSP;
      end

      CE_SET_BLOCK_SIZE_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SET_TRANSFER_MODE;
      end

      CE_SET_TRANSFER_MODE: begin
        // All-zero: single-block, write direction, no AUTO_CMD12, no
        // block-count-enable -- none of that is needed for CMD24.
        copy_write_req_o = obi_write(SDHCI_XFER_CMD, 32'h0, 4'h1);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_TRANSFER_MODE_RSP;
      end

      CE_SET_TRANSFER_MODE_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SET_ARGUMENT;
      end

      CE_SET_ARGUMENT: begin
        copy_write_req_o = obi_write(SDHCI_ARGUMENT, cmd24_argument, 4'hf);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SET_ARGUMENT_RSP;
      end

      CE_SET_ARGUMENT_RSP: begin
        if (copy_write_rsp_i.rvalid)
          state_d = CE_SUBMIT_CMD24;
      end

      // This write is what actually issues CMD24.
      CE_SUBMIT_CMD24: begin
        copy_write_req_o = obi_write(SDHCI_XFER_CMD, {CMD24_VALUE, 16'h0}, 4'hc);
        if (copy_write_rsp_i.gnt)
          state_d = CE_SUBMIT_CMD24_RSP;
      end

      CE_SUBMIT_CMD24_RSP: begin
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
            rd_idx_d     = '0;
            wr_idx_d     = '0;
            pipe_valid_d = 1'b0;
            rd_pending_d = 1'b0;
            state_d      = CE_COPY_WORD;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_BUFFER_READY; // retry
          end
        end
      end

      // --------------------------------------------------------------------
      // Copy, one word at a time.
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
      //   SDHCI's grant is not guaranteed same-cycle and can be backpressured
      //   for an arbitrary number of cycles; the read side waiting on
      //   pipe_valid_q/rd_pending_q is what makes the copy correct under
      //   that backpressure instead of silently overwriting unwritten data.
      CE_COPY_WORD: begin
        // --- READ side ---
        if (rd_idx_q < frame_words_q && !pipe_valid_q && !rd_pending_q) begin
          copy_read_req_o = obi_read(sram_copy_base + {rd_idx_q, 2'b00});
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
            pipe_valid_d = 1'b0;
            wr_idx_d     = wr_idx_q + 1;
          end
        end

        if (wr_idx_q == frame_words_q)
          state_d = CE_ACK_BUFFERED;
      end

      // --------------------------------------------------------------------
      // W1C-clear BUFFER_WRITE_READY so SDHCI knows this block's words are
      // all in its buffer.
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
      // Single condition, not a race: BUFFER_WRITE_READY structurally
      // cannot re-assert for a single-block command (SDHCI's internal
      // block counter is pinned to 1 -- see header comment), so
      // TRANSFER_COMPLETE is the only outcome that ever arrives here.
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
          state_d            = CE_WAIT_CARD_READY;
        end
      end

      // --------------------------------------------------------------------
      // TRANSFER_COMPLETE is a *host controller* event, not a *card* event.
      // It means SDHCI's own write FSM finished framing this block on the
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
      // Note on POLL_TIMEOUT: this is the one wait whose natural duration is
      // set by flash programming rather than by bus turnaround. 500k cycles
      // is ~5 ms at 100 MHz, comfortable against this testbench model (~500
      // cycles) but short of the ~250 ms a real card may legitimately take
      // for a single-block write -- raise POLL_TIMEOUT before running
      // against real silicon.
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
            state_d = CE_DONE;
          end else if (poll_timeout_cnt_q >= POLL_TIMEOUT) begin
            state_d = CE_OVERFLOW;
          end else begin
            state_d = CE_WAIT_CARD_READY; // retry
          end
        end
      end

      // --------------------------------------------------------------------
      CE_OVERFLOW: begin
        sdcard_overflow_set_o = 1'b1;
        state_d = CE_IDLE;
      end

      // --------------------------------------------------------------------
      // Every frame releases its bank and advances the address the same
      // way; only the capture's actual last frame (is_last_frame_q,
      // latched at job start) also asserts SDCARD_DONE.
      CE_DONE: begin
        block_addr_incr_o = 1'b1;
        if (copying_f0_q) f0_full_clear_o = 1'b1;
        else               f1_full_clear_o = 1'b1;
        if (is_last_frame_q)
          sdcard_done_set_o = 1'b1;
        state_d = CE_IDLE;
      end

      default: state_d = CE_IDLE;
    endcase
  end

endmodule
