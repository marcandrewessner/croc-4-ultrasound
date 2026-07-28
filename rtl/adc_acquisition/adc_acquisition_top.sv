
`include "common_cells/registers.svh"

module adc_acquisition_top import adc_acquisition_pkg::*; #(
  parameter obi_pkg::obi_cfg_t ObiSbrCfg = obi_pkg::ObiDefaultConfig,
  parameter type mgr_obi_req_t = logic,
  parameter type mgr_obi_rsp_t = logic,
  parameter type sbr_obi_req_t = logic,
  parameter type sbr_obi_rsp_t = logic
) (
  input logic clk_i,
  input logic rst_ni,

  // 3 OBI manager ports
  output mgr_obi_req_t adc_data_write_req_o,  // ADC samples → SRAM Bank2/3
  input  mgr_obi_rsp_t adc_data_write_rsp_i,
  output mgr_obi_req_t copy_read_req_o,        // copy engine reads from SRAM Bank2/3
  input  mgr_obi_rsp_t copy_read_rsp_i,
  output mgr_obi_req_t copy_write_req_o,       // copy engine writes to SDHCI buffer
  input  mgr_obi_rsp_t copy_write_rsp_i,

  // 1 OBI subordinate port (register config)
  input  sbr_obi_req_t sbr_obi_req_i,
  output sbr_obi_rsp_t sbr_obi_rsp_o,

  // Interrupt: any status flag raised
  output logic interrupt_frame_full_o,

  // ADC input (unsynchronised, ADC clock domain)
  input adc_input_signals_t adc_input_signals
);

  //////////////////////////////////////
  // Register block (OBI subordinate) //
  //////////////////////////////////////
  logic [adc_acquisition_reg_pkg::ADC_ACQUISITION_REG_MIN_ADDR_WIDTH-1:0] sbr_obi_req_relative_addr;
  reg2hw_t hw2reg;
  hw2reg_t reg2hw;

  adc_acquisition_reg #(
    .ID_WIDTH(ObiSbrCfg.IdWidth)
  ) i_adc_acquisition_reg (
    .clk( clk_i ),
    .rst( ~rst_ni ),
    .hwif_in  ( hw2reg ),
    .hwif_out ( reg2hw ),
    .s_obi_req    ( sbr_obi_req_i.req ),
    .s_obi_gnt    ( sbr_obi_rsp_o.gnt ),
    .s_obi_addr   ( sbr_obi_req_relative_addr ),
    .s_obi_we     ( sbr_obi_req_i.a.we ),
    .s_obi_be     ( sbr_obi_req_i.a.be ),
    .s_obi_wdata  ( sbr_obi_req_i.a.wdata ),
    .s_obi_aid    ( sbr_obi_req_i.a.aid ),
    .s_obi_rvalid ( sbr_obi_rsp_o.rvalid ),
    .s_obi_rready ( 1'b1 ),
    .s_obi_rdata  ( sbr_obi_rsp_o.r.rdata ),
    .s_obi_err    ( sbr_obi_rsp_o.r.err ),
    .s_obi_rid    ( sbr_obi_rsp_o.r.rid )
  );

  assign sbr_obi_req_relative_addr = (sbr_obi_req_i.a.addr - ADC_REGISTER_BASE_ADDRESS);
  assign sbr_obi_rsp_o.r.r_optional = '0;

  //////////////////////////////////////
  // ADC-domain 2-sample packer       //
  // Runs on adc_input_signals.clk    //
  //////////////////////////////////////
  logic        adc_pack_sel_d, adc_pack_sel_q;
  logic [13:0] adc_pack_lo_d,  adc_pack_lo_q;
  logic [31:0] adc_pack_word;
  logic        adc_pack_valid;
  logic        adc_cdc_src_ready;

  `FF(adc_pack_sel_q, adc_pack_sel_d, 1'b0, adc_input_signals.clk, adc_input_signals.rst_n)
  `FF(adc_pack_lo_q,  adc_pack_lo_d,  '0,   adc_input_signals.clk, adc_input_signals.rst_n)

  always_comb begin
    adc_pack_sel_d = adc_pack_sel_q;
    adc_pack_lo_d  = adc_pack_lo_q;
    adc_pack_valid = 1'b0;
    adc_pack_word  = '0;
    if (adc_input_signals.valid) begin
      if (!adc_pack_sel_q) begin
        adc_pack_lo_d  = adc_input_signals.data;
        adc_pack_sel_d = 1'b1;
      end else begin
        // {2'b00, hi[13:0], 2'b00, lo[13:0]}
        adc_pack_word  = {2'b00, adc_input_signals.data, 2'b00, adc_pack_lo_q};
        adc_pack_valid = 1'b1;
        adc_pack_sel_d = 1'b0;
      end
    end
  end

  //////////////////////////////////////
  // CDC FIFO (ADC clk → sys clk)     //
  // WIDTH=32: transfers packed words  //
  //////////////////////////////////////
  logic [31:0] adc_data_word;
  logic        adc_data_word_ready;

  cdc_fifo_gray #(
    .WIDTH( 32 )
  ) i_cdc_fifo_gray (
    .src_clk_i   ( adc_input_signals.clk   ),
    .src_rst_ni  ( adc_input_signals.rst_n ),
    .src_valid_i ( adc_pack_valid          ),
    .src_data_i  ( adc_pack_word           ),
    .src_ready_o ( adc_cdc_src_ready       ),
    .dst_clk_i   ( clk_i    ),
    .dst_rst_ni  ( rst_ni   ),
    .dst_ready_i ( 1'b1     ),
    .dst_data_o  ( adc_data_word           ),
    .dst_valid_o ( adc_data_word_ready     )
  );

  // CDC overflow: packer produced a word but FIFO was full
  logic adc_cdc_overflow;
  assign adc_cdc_overflow = adc_pack_valid && !adc_cdc_src_ready;

  //////////////////////////////////////
  // SDCard copy controller           //
  //////////////////////////////////////
  logic        sdcard_done_set;
  logic        sdcard_overflow_set;
  logic        f0_full_clear;
  logic        f1_full_clear;
  logic        block_addr_incr;
  logic [31:0] block_addr_advance;

  // Coordinator: adc_acquisition_sdcard_controller is a passive subordinate
  // -- it never decides on its own to start a copy. This is the single
  // place that decides "is there a job to do right now", so it's also the
  // single place a stuck retry-forever-after-overflow bug could hide:
  // once SDCARD_OVERFLOW is latched, block starting any further job (the
  // frame that overflowed keeps Fx_FULL set forever, since a failed copy
  // never clears it, and it must stay blocked until software clears
  // SDCARD_OVERFLOW and the Fx_FULL flags via CNTRL).
  // The two SD modes differ only in *when* a job is dispatched and how much
  // of the SRAM one job covers -- the copy engine's SD-side sequence is
  // identical for both:
  //   SDCARD_CONTINUOUS: dispatch as soon as either bank fills, one session
  //     per bank, so capture and streaming overlap (and the card must keep
  //     up with the ADC).
  //   SDCARD_PULSE: dispatch only once *both* banks have filled, one session
  //     covering both, so nothing overlaps and there is no throughput
  //     requirement -- the capture is bounded at two banks instead.
  logic sd_mode_continuous, sd_mode_pulse, sd_mode;
  assign sd_mode_continuous =
    reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__SDCARD_CONTINUOUS;
  assign sd_mode_pulse =
    reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__SDCARD_PULSE;
  assign sd_mode = sd_mode_continuous || sd_mode_pulse;

  logic sdcard_busy;
  logic sdcard_start;
  logic sdcard_copy_f0;
  logic sdcard_copy_both;

  // SDCARD_CONTINUOUS: F0 takes priority; F1 only starts once F0 isn't
  // pending. SDCARD_PULSE: nothing starts until both banks are full, and
  // then exactly one job covers both. Combinational
  // on sdcard_busy (itself derived from the controller's registered state),
  // so this naturally pulses for exactly one cycle per job: the cycle
  // start_i fires, busy_o (still reflecting the *previous* state) is low,
  // and the very next cycle busy_o goes high and gates this back off.
  //
  // In pulse mode the one job is also structurally the last: F1_FULL and
  // capture_done_q (which feeds is_last_frame_i) are set in the same cycle
  // by the F1 boundary, so capture_done_q is already high the first time
  // this can fire.
  logic sdcard_frames_ready;
  assign sdcard_frames_ready = sd_mode_pulse
    ? (reg2hw.STATUS.F0_FULL.value && reg2hw.STATUS.F1_FULL.value)
    : (reg2hw.STATUS.F0_FULL.value || reg2hw.STATUS.F1_FULL.value);
  assign sdcard_start   = sd_mode && !sdcard_busy && !reg2hw.STATUS.SDCARD_OVERFLOW.value &&
                           sdcard_frames_ready;
  assign sdcard_copy_f0   = reg2hw.STATUS.F0_FULL.value;
  assign sdcard_copy_both = sd_mode_pulse;

  adc_acquisition_sdcard_controller #(
    .mgr_obi_req_t ( mgr_obi_req_t ),
    .mgr_obi_rsp_t ( mgr_obi_rsp_t )
  ) i_sdcard_ctrl (
    .clk_i,
    .rst_ni,
    .start_i                ( sdcard_start        ),
    .copy_f0_i              ( sdcard_copy_f0      ),
    .copy_both_i            ( sdcard_copy_both    ),
    // The dispatched job's "is this the capture's last frame" flag is
    // exactly capture_done_q (§2a): it's guaranteed already settled by the
    // time sdcard_start can fire for the corresponding job (see
    // DATAPATH.md), so no extra latch is needed here.
    .is_last_frame_i        ( capture_done_q      ),
    .busy_o                 ( sdcard_busy         ),
    .copy_read_req_o,
    .copy_read_rsp_i,
    .copy_write_req_o,
    .copy_write_rsp_i,
    .reg2hw                 ( reg2hw              ),
    .sdcard_done_set_o      ( sdcard_done_set     ),
    .sdcard_overflow_set_o  ( sdcard_overflow_set ),
    .f0_full_clear_o        ( f0_full_clear       ),
    .f1_full_clear_o        ( f1_full_clear       ),
    .block_addr_incr_o      ( block_addr_incr     ),
    .block_addr_advance_o   ( block_addr_advance  )
  );

  //////////////////////////////////////
  // Control logic (single driver of hw2reg)
  //////////////////////////////////////
  typedef enum logic {
    CURRENT_FRAME_0, CURRENT_FRAME_1
  } current_frame_e;

  logic        dma_push;
  logic [31:0] dma_data;
  logic [31:0] dma_address;

  current_frame_e current_frame_d, current_frame_q;
  `FF(current_frame_q, current_frame_d, CURRENT_FRAME_0, clk_i, rst_ni)

  // SDCARD_CONTINUOUS frame budget (DATAPATH.md §2a): frames_started_q
  // counts how
  // many frames have been claimed for filling so far, F0's initial fill
  // counting as frame 1 -- incremented at the exact ping-pong boundary
  // crossing, not on the copy engine's (slower, dispatch-lag) start_i, so
  // the ADC never claims an (N+1)th frame in the first place, closing the
  // fast-dispatch/slow-completion race described there. capture_done_q
  // latches once the Nth frame's boundary is reached, so the ADC-fill side
  // parks (no more pushes) until software moves MODE away from the SD mode,
  // instead of re-evaluating target_frame_full against a bank
  // it deliberately isn't going to reuse.
  //
  // frames_started_q is SDCARD_CONTINUOUS-only. SDCARD_PULSE has no frame
  // budget to track -- its length is fixed at both banks -- so it sets
  // capture_done_q directly at the F1 boundary and never touches this
  // counter.
  logic [31:0] frames_started_d, frames_started_q;
  `FF(frames_started_q, frames_started_d, 32'd1, clk_i, rst_ni)

  // Sticky "the ADC has captured everything it was asked for": parks the
  // fill side, and doubles as the copy engine's is_last_frame_i.
  logic capture_done_d, capture_done_q;
  `FF(capture_done_q, capture_done_d, 1'b0, clk_i, rst_ni)

  // Shared by the mode-specific case below (to raise F0_FULL) and the
  // auto-stop section (to know a single-shot fill just completed), so both
  // places agree on the same frame-boundary condition.
  //
  // "just filled" means *the last word is being written this cycle*, not
  // merely "the head points at the last word". The distinction is the whole
  // bug this qualifier fixes: the head lands on F0_END_ADDR one cycle after
  // the second-to-last word is pushed, and sits there until the CDC FIFO
  // produces the next sample -- which, with the ADC running far slower than
  // the system clock, is many cycles later. Without the
  // adc_data_word_ready term every consumer of this signal fired during
  // that gap, i.e. one word early: SINGLE_ACQ_F0 reverted MODE to IDLE and
  // the ping-pong modes redirected the write head to the other bank,
  // both before the word at F0_END_ADDR was ever written. Confirmed via
  // waveform: the ADC DMA wrote 127 distinct words per frame, with
  // 0x1000_11fc (F0) and 0x1000_19fc (F1) never written at all, so every
  // 512 B SD block came out as 127 real words plus one uninitialised zero
  // -- visible as the 0x00000000 at offset 0x1fc of every block in the SD
  // card flash dump (scripts/check_flash_dump.py flags it). No ADC sample
  // was lost, the frame just ended one word short.
  //
  // adc_data_word_ready (the CDC FIFO output) rather than dma_push, even
  // though every branch below computes dma_push from it: dma_push is
  // assigned inside the always_comb that reads this signal, so depending on
  // it here would make the block's result order-sensitive. The branches
  // that inline this same comparison qualify it with dma_push directly,
  // which is safe there because it is already assigned at that point.
  logic f0_frame_just_filled;
  assign f0_frame_just_filled = adc_data_word_ready &&
    (reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F0_END_ADDR.WORD_ADDRESS.value);

  // True if the bank the write head currently targets still has its
  // Fx_FULL flag set, i.e. its previous contents haven't been consumed
  // yet. Every ping-pong mode uses this to stop the ADC from
  // wrapping around and overwriting a frame its consumer hasn't read yet
  // when that consumer is slower than one frame's fill time. In
  // SDCARD_PULSE, where no bank is ever reused, it degenerates into a
  // start-of-pulse check that both banks were released.
  logic target_frame_full;
  assign target_frame_full = (current_frame_q == CURRENT_FRAME_0)
    ? reg2hw.STATUS.F0_FULL.value
    : reg2hw.STATUS.F1_FULL.value;

  always_comb begin : adc_control_logic
    dma_push    = '0;
    dma_data    = 'x;
    dma_address = 'x;
    current_frame_d   = current_frame_q;
    frames_started_d  = frames_started_q;
    capture_done_d    = capture_done_q;

    // Default: hold all register values
    hw2reg.CONF.MODE.next               = reg2hw.CONF.MODE.value;
    hw2reg.STATUS.F0_FULL.next          = reg2hw.STATUS.F0_FULL.value;
    hw2reg.STATUS.F1_FULL.next          = reg2hw.STATUS.F1_FULL.value;
    hw2reg.STATUS.ADC_OVERFLOW.next     = reg2hw.STATUS.ADC_OVERFLOW.value;
    hw2reg.STATUS.SDCARD_DONE.next      = reg2hw.STATUS.SDCARD_DONE.value;
    hw2reg.STATUS.SDCARD_OVERFLOW.next  = reg2hw.STATUS.SDCARD_OVERFLOW.value;
    hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value;
    hw2reg.SDCARD_BLOCK_ADDR.BLOCK_ADDR.next = reg2hw.SDCARD_BLOCK_ADDR.BLOCK_ADDR.value;

    // --- Controller status signals ---
    if (sdcard_done_set)
      hw2reg.STATUS.SDCARD_DONE.next = 1'b1;
    if (sdcard_overflow_set)
      hw2reg.STATUS.SDCARD_OVERFLOW.next = 1'b1;
    if (f0_full_clear)
      hw2reg.STATUS.F0_FULL.next = 1'b0;
    if (f1_full_clear)
      hw2reg.STATUS.F1_FULL.next = 1'b0;
    if (block_addr_incr)
      hw2reg.SDCARD_BLOCK_ADDR.BLOCK_ADDR.next =
        reg2hw.SDCARD_BLOCK_ADDR.BLOCK_ADDR.value + block_addr_advance;

    // --- ADC CDC overflow (ADC-domain signal, safe to latch combinatorially) ---
    if (adc_cdc_overflow)
      hw2reg.STATUS.ADC_OVERFLOW.next = 1'b1;

    // --- CNTRL register pulses ---
    if (reg2hw.CNTRL.RESET_WRITE_HEAD.value) begin
      hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F0_START_ADDR.WORD_ADDRESS.value;
      current_frame_d  = CURRENT_FRAME_0;
      frames_started_d = 32'd1;
      capture_done_d   = 1'b0;
    end
    if (reg2hw.CNTRL.CLEAR_F0_FULL.value)
      hw2reg.STATUS.F0_FULL.next = 1'b0;
    if (reg2hw.CNTRL.CLEAR_F1_FULL.value)
      hw2reg.STATUS.F1_FULL.next = 1'b0;
    if (reg2hw.CNTRL.CLEAR_STATUS.value) begin
      hw2reg.STATUS.ADC_OVERFLOW.next    = 1'b0;
      hw2reg.STATUS.SDCARD_DONE.next     = 1'b0;
      hw2reg.STATUS.SDCARD_OVERFLOW.next = 1'b0;
    end

    // --- Auto-stop: bounded-length modes revert to IDLE once their capture
    // is done, so the ADC-fill logic below can't re-arm itself afterwards.
    // All of them are handled here so there is exactly one place that
    // answers "when does MODE auto-revert to IDLE". Both SD modes qualify:
    // SDCARD_CONTINUOUS is continuous only across its SDCARD_FRAME_COUNT
    // frames and ends on the last one's sdcard_done_set, SDCARD_PULSE ends
    // on its single job's. CONTINUOUS_ACQ_F0_F1 is deliberately absent -- it
    // is the one genuinely unbounded mode and only stops via software.
    if (reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__SINGLE_ACQ_F0
        && f0_frame_just_filled)
      hw2reg.CONF.MODE.next = adc_acquisition_reg_pkg::adc_mode__IDLE;
    if (sd_mode && (sdcard_done_set || sdcard_overflow_set))
      hw2reg.CONF.MODE.next = adc_acquisition_reg_pkg::adc_mode__IDLE;

    // --- Mode-specific ADC write control ---
    case (reg2hw.CONF.MODE.value)
      adc_acquisition_reg_pkg::adc_mode__IDLE: begin
        // No writes; CDC FIFO drains naturally
      end

      adc_acquisition_reg_pkg::adc_mode__SINGLE_ACQ_F0: begin
        dma_push    = adc_data_word_ready;
        dma_address = {reg2hw.WRITE_HEAD.WORD_ADDRESS.value, 2'b00};
        dma_data    = adc_data_word;
        if (dma_push)
          hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value + 1;
        if (f0_frame_just_filled)
          hw2reg.STATUS.F0_FULL.next = 1'b1;
      end

      adc_acquisition_reg_pkg::adc_mode__CONTINUOUS_ACQ_F0_F1: begin
        // Same target_frame_full guard as the SD modes below: without it, the
        // ADC silently overwrites a bank software hasn't read out yet, with
        // no flag raised anywhere to notice by. Reuses ADC_OVERFLOW (rather
        // than SDCARD_OVERFLOW, which is worded SD-specific) since it's
        // already the generic "a sample was dropped" flag.
        if (target_frame_full) begin
          hw2reg.STATUS.ADC_OVERFLOW.next = 1'b1;
        end else begin
          dma_push    = adc_data_word_ready;
          dma_address = {reg2hw.WRITE_HEAD.WORD_ADDRESS.value, 2'b00};
          dma_data    = adc_data_word;
          if (dma_push)
            hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value + 1;
          // dma_push qualifier: only declare the frame full on the cycle its
          // last word is actually written, never while the head merely points
          // at F0_END_ADDR waiting for the next sample -- see
          // f0_frame_just_filled above for why that gap loses the last word.
          if (dma_push
              && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F0_END_ADDR.WORD_ADDRESS.value
              && current_frame_q == CURRENT_FRAME_0) begin
            hw2reg.STATUS.F0_FULL.next          = 1'b1;
            hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F1_START_ADDR.WORD_ADDRESS.value;
            current_frame_d = CURRENT_FRAME_1;
          end
          if (dma_push
              && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F1_END_ADDR.WORD_ADDRESS.value
              && current_frame_q == CURRENT_FRAME_1) begin
            hw2reg.STATUS.F1_FULL.next          = 1'b1;
            hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F0_START_ADDR.WORD_ADDRESS.value;
            current_frame_d = CURRENT_FRAME_0;
          end
        end
      end

      adc_acquisition_reg_pkg::adc_mode__SDCARD_CONTINUOUS: begin
        // Ping-pong F0/F1, HW copy engine drains each bank to the SD card
        // with its own CMD25 session per frame. Works for any
        // configured frame count N (software sets SDCARD_FRAME_COUNT to N
        // before enabling this mode): N=1 behaves like the old
        // SINGLE_SDCARD, N>1 like the old CONTINUOUS_SDCARD -- same
        // mechanism, the only mode-internal branching is the frame-budget
        // stop below.
        //
        // Two independent stop conditions:
        //   - target_frame_full: the bank about to be entered still holds
        //     an undrained frame -- the card hasn't kept up in real time.
        //   - capture_done_q: the Nth (last wanted) frame's boundary has
        //     already been reached -- stop for good, regardless of
        //     target_frame_full, instead of re-evaluating it against a
        //     bank that's deliberately not going to be reused. Without
        //     this the ADC has no concept of N at all and keeps claiming
        //     frames N+1, N+2, ... while the real last job is still
        //     physically draining (fast dispatch vs. slow completion,
        //     DATAPATH.md §2a), wasting frames and tripping a spurious
        //     SDCARD_OVERFLOW on what should be a clean N-frame capture.
        if (!reg2hw.STATUS.SDCARD_OVERFLOW.value && !capture_done_q) begin
          if (target_frame_full) begin
            hw2reg.STATUS.SDCARD_OVERFLOW.next = 1'b1;
          end else begin
            dma_push    = adc_data_word_ready;
            dma_address = {reg2hw.WRITE_HEAD.WORD_ADDRESS.value, 2'b00};
            dma_data    = adc_data_word;
            if (dma_push)
              hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value + 1;

            // dma_push qualifier: see f0_frame_just_filled above -- without
            // it the head is redirected to the other bank while still
            // pointing at this frame's unwritten last word.
            if (dma_push
                && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F0_END_ADDR.WORD_ADDRESS.value
                && current_frame_q == CURRENT_FRAME_0) begin
              hw2reg.STATUS.F0_FULL.next = 1'b1;
              if (frames_started_q >= reg2hw.SDCARD_FRAME_COUNT.FRAME_COUNT.value) begin
                capture_done_d = 1'b1; // F0 was the last wanted frame
              end else begin
                hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F1_START_ADDR.WORD_ADDRESS.value;
                current_frame_d   = CURRENT_FRAME_1;
                frames_started_d  = frames_started_q + 1;
              end
            end
            if (dma_push
                && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F1_END_ADDR.WORD_ADDRESS.value
                && current_frame_q == CURRENT_FRAME_1) begin
              hw2reg.STATUS.F1_FULL.next = 1'b1;
              if (frames_started_q >= reg2hw.SDCARD_FRAME_COUNT.FRAME_COUNT.value) begin
                capture_done_d = 1'b1; // F1 was the last wanted frame
              end else begin
                hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F0_START_ADDR.WORD_ADDRESS.value;
                current_frame_d   = CURRENT_FRAME_0;
                frames_started_d  = frames_started_q + 1;
              end
            end
          end
        end
      end

      adc_acquisition_reg_pkg::adc_mode__SDCARD_PULSE: begin
        // One-shot burst: fill F0, roll straight on into F1, then stop. The
        // copy engine is not dispatched until *both* banks are full (see
        // sdcard_frames_ready above), and then runs once for both of them
        // as a single CMD25 session -- 8 blocks for two full banks.
        //
        // Why this is a separate case rather than SDCARD_CONTINUOUS with
        // SDCARD_FRAME_COUNT = 2: the difference is not the frame budget,
        // it is that no copy overlaps the capture at all. That removes the
        // real-time constraint the continuous mode lives under
        // (T_session <= T_fill), so the whole ping-pong frame-budget
        // apparatus has nothing to do here -- the capture length is fixed
        // at the two banks, hence no frames_started_q and no
        // SDCARD_FRAME_COUNT (that register is ignored in this mode).
        //
        // capture_done_q is still what parks the ADC, and it is set at the
        // F1 boundary in the same cycle as F1_FULL, which is also the cycle
        // the job becomes dispatchable -- so the job's is_last_frame_i
        // (wired to capture_done_q) is high for it, and SDCARD_DONE fires
        // when the session is physically committed. MODE then auto-reverts
        // to IDLE; software re-arms (clear status, RESET_WRITE_HEAD, set
        // MODE) for the next pulse.
        //
        // target_frame_full cannot fire from bank reuse here -- each bank is
        // written exactly once -- but it is kept as the guard against
        // starting a pulse on top of a bank whose Fx_FULL software never
        // cleared, which would otherwise capture into a bank the engine is
        // about to stream from, or has already streamed.
        if (!reg2hw.STATUS.SDCARD_OVERFLOW.value && !capture_done_q) begin
          if (target_frame_full) begin
            hw2reg.STATUS.SDCARD_OVERFLOW.next = 1'b1;
          end else begin
            dma_push    = adc_data_word_ready;
            dma_address = {reg2hw.WRITE_HEAD.WORD_ADDRESS.value, 2'b00};
            dma_data    = adc_data_word;
            if (dma_push)
              hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value + 1;

            // dma_push qualifier: see f0_frame_just_filled above -- without
            // it the head is redirected to F1 while still pointing at F0's
            // unwritten last word.
            if (dma_push
                && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F0_END_ADDR.WORD_ADDRESS.value
                && current_frame_q == CURRENT_FRAME_0) begin
              // F0_FULL is raised here even though nothing consumes it yet:
              // it is half of the both-full dispatch condition, and it keeps
              // Fx_FULL's meaning ("this bank holds a captured frame")
              // uniform across modes.
              hw2reg.STATUS.F0_FULL.next          = 1'b1;
              hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F1_START_ADDR.WORD_ADDRESS.value;
              current_frame_d = CURRENT_FRAME_1;
            end
            if (dma_push
                && reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F1_END_ADDR.WORD_ADDRESS.value
                && current_frame_q == CURRENT_FRAME_1) begin
              hw2reg.STATUS.F1_FULL.next = 1'b1;
              capture_done_d             = 1'b1; // pulse captured, hand off to the engine
            end
          end
        end
      end

      default: ;
    endcase
  end

  //////////////////////////////////////
  // ADC DMA OBI (adc_data_write port) //
  //////////////////////////////////////
  always_comb begin : adc_dma_obi
    adc_data_write_req_o.req          = dma_push;
    adc_data_write_req_o.a.addr       = dma_address;
    adc_data_write_req_o.a.we         = 1'b1;
    adc_data_write_req_o.a.be         = 4'hf;
    adc_data_write_req_o.a.wdata      = dma_data;
    adc_data_write_req_o.a.aid        = '0;
    adc_data_write_req_o.a.a_optional = '0;
  end

  //////////////////////////////////////
  // Interrupt                        //
  //////////////////////////////////////
  assign interrupt_frame_full_o = reg2hw.STATUS.F0_FULL.value
                                | reg2hw.STATUS.F1_FULL.value
                                | reg2hw.STATUS.SDCARD_DONE.value
                                | reg2hw.STATUS.SDCARD_OVERFLOW.value
                                | reg2hw.STATUS.ADC_OVERFLOW.value;

endmodule
