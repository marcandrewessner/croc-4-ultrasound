
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
  // ILA debug taps (Xilinx synthesis only; ignored elsewhere)
  //////////////////////////////////////
  // adc_input_signals.clk is a struct field, not a standalone net -- Vivado
  // needs an actual net to hang a MARK_DEBUG probe off of, hence the mirror
  // signal instead of tagging the field directly. Sampled by the ILA's own
  // capture clock (soc_clk, see xilinx/scripts/common.tcl insert_ilas), so
  // this just shows adc_clk's edges relative to soc_clk, not a real
  // clock-domain probe.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic dbg_adc_clk;
  assign dbg_adc_clk = adc_input_signals.clk;

  // Trigger-friendly probe: high for the whole time CONF.MODE != IDLE, i.e.
  // exactly the window an acquisition is running. Set the ILA trigger
  // condition to this probe's rising edge to start capture the instant a
  // mode is armed, instead of building a multi-bit compare against
  // CONF.MODE by hand in the Hardware Manager.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic dbg_mode_active;
  assign dbg_mode_active = reg2hw.CONF.MODE.value != adc_acquisition_reg_pkg::adc_mode__IDLE;

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
  logic sd_mode;
  assign sd_mode = reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__ACQ_SDCARD;

  logic sdcard_busy;
  logic sdcard_start;
  logic sdcard_copy_f0;

  // F0 takes priority; F1 only starts once F0 isn't pending. Combinational
  // on sdcard_busy (itself derived from the controller's registered state),
  // so this naturally pulses for exactly one cycle per job: the cycle
  // start_i fires, busy_o (still reflecting the *previous* state) is low,
  // and the very next cycle busy_o goes high and gates this back off.
  assign sdcard_start   = sd_mode && !sdcard_busy && !reg2hw.STATUS.SDCARD_OVERFLOW.value &&
                           (reg2hw.STATUS.F0_FULL.value || reg2hw.STATUS.F1_FULL.value);
  assign sdcard_copy_f0 = reg2hw.STATUS.F0_FULL.value;

  adc_acquisition_sdcard_controller #(
    .mgr_obi_req_t ( mgr_obi_req_t ),
    .mgr_obi_rsp_t ( mgr_obi_rsp_t )
  ) i_sdcard_ctrl (
    .clk_i,
    .rst_ni,
    .start_i                ( sdcard_start        ),
    .copy_f0_i              ( sdcard_copy_f0      ),
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

  // ACQ_SDCARD frame budget (DATAPATH.md §2a): frames_started_q counts how
  // many frames have been claimed for filling so far, F0's initial fill
  // counting as frame 1 -- incremented at the exact ping-pong boundary
  // crossing, not on the copy engine's (slower, dispatch-lag) start_i, so
  // the ADC never claims an (N+1)th frame in the first place, closing the
  // fast-dispatch/slow-completion race described there. capture_done_q
  // latches once the Nth frame's boundary is reached, so the ADC-fill side
  // parks (no more pushes) until software moves MODE away from
  // ACQ_SDCARD, instead of re-evaluating target_frame_full against a bank
  // it deliberately isn't going to reuse.
  logic [31:0] frames_started_d, frames_started_q;
  `FF(frames_started_q, frames_started_d, 32'd1, clk_i, rst_ni)

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
  // ACQ_SDCARD/CONTINUOUS redirected the write head to the other bank,
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
  // yet. ACQ_SDCARD and CONTINUOUS_ACQ_F0_F1 use this to stop the ADC from
  // wrapping around and overwriting a frame its consumer hasn't read yet
  // when that consumer is slower than one frame's fill time.
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

    // --- Auto-stop: single-shot modes revert to IDLE once their one job is
    // done, so the ADC-fill logic below can't re-arm itself afterwards. Both
    // single-shot modes are handled here so there is exactly one place that
    // answers "when does MODE auto-revert to IDLE" -- CONTINUOUS_* modes are
    // deliberately absent, they only stop via SDCARD_OVERFLOW or software.
    if (reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__SINGLE_ACQ_F0
        && f0_frame_just_filled)
      hw2reg.CONF.MODE.next = adc_acquisition_reg_pkg::adc_mode__IDLE;
    if (reg2hw.CONF.MODE.value == adc_acquisition_reg_pkg::adc_mode__ACQ_SDCARD
        && (sdcard_done_set || sdcard_overflow_set))
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
        // Same target_frame_full guard as ACQ_SDCARD below: without it, the
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

      adc_acquisition_reg_pkg::adc_mode__ACQ_SDCARD: begin
        // Ping-pong F0/F1, HW copy engine drains each bank to the SD card
        // with its own CMD24 (WRITE_BLOCK) per frame. Works for any
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
  // ILA debug taps (Xilinx synthesis only; ignored elsewhere), continued
  //////////////////////////////////////
  // dma_push qualifies dma_address/dma_data: both read as 'x (default in the
  // control-logic case statement) whenever a write isn't actually happening
  // this cycle, so without this the bus values are meaningless noise.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic        dbg_dma_push;
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [31:0] dbg_dma_address;
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [31:0] dbg_dma_data;
  assign dbg_dma_push    = dma_push;
  assign dbg_dma_address = dma_address;
  assign dbg_dma_data    = dma_data;

  // Raw ADC sample (ADC clock domain, mirrored) and the packed 32-bit word
  // (system clock domain) that dma_data above is ultimately sourced from.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [13:0] dbg_adc_data;
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [31:0] dbg_adc_data_word;
  assign dbg_adc_data      = adc_input_signals.data;
  assign dbg_adc_data_word = adc_data_word;

  // Write head pointer (word address) and full mode encoding, to correlate
  // capture progress and control-flow state against the buses above.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [29:0] dbg_write_head;
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [7:0]  dbg_mode;
  assign dbg_write_head = reg2hw.WRITE_HEAD.WORD_ADDRESS.value;
  assign dbg_mode       = reg2hw.CONF.MODE.value;

  // STATUS flags packed into one bus: {SDCARD_OVERFLOW, SDCARD_DONE,
  // ADC_OVERFLOW, F1_FULL, F0_FULL}.
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [4:0] dbg_status;
  assign dbg_status = {reg2hw.STATUS.SDCARD_OVERFLOW.value,
                       reg2hw.STATUS.SDCARD_DONE.value,
                       reg2hw.STATUS.ADC_OVERFLOW.value,
                       reg2hw.STATUS.F1_FULL.value,
                       reg2hw.STATUS.F0_FULL.value};

  //////////////////////////////////////
  // Interrupt                        //
  //////////////////////////////////////
  assign interrupt_frame_full_o = reg2hw.STATUS.F0_FULL.value
                                | reg2hw.STATUS.F1_FULL.value
                                | reg2hw.STATUS.SDCARD_DONE.value
                                | reg2hw.STATUS.SDCARD_OVERFLOW.value
                                | reg2hw.STATUS.ADC_OVERFLOW.value;

endmodule
