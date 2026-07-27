# ADC → SD Datapath — Architecture Plan

Working doc for planning the datapath, not a tutorial. Current-state facts
are cited to file:line where it matters for a decision; skip otherwise.

## Modes — current vs target

| Mode | Banks | Consumer | N source | Throughput req | Status |
|---|---|---|---|---|---|
| `IDLE` | — | — | — | — | unchanged |
| `SINGLE_ACQ_F0` | F0 | CPU read | fixed 1 | none | unchanged |
| `CONTINUOUS_ACQ_F0_F1` | F0+F1 | CPU read | unbounded | `T_read < T_fill` | **bug**: no `target_frame_full` check (top.sv:294-312), silent overwrite |
| `SINGLE_SDCARD` | F0 | HW copy + card | `BLOCK_COUNT=1` | none | to merge → `ACQ_SDCARD` |
| `CONTINUOUS_SDCARD` | F0+F1 | HW copy + card | `BLOCK_COUNT=N` | `T_block ≤ T_fill` | to merge → `ACQ_SDCARD` |

Target: collapse `SINGLE_SDCARD`+`CONTINUOUS_SDCARD` → one `ACQ_SDCARD`
mode. Both are the same mechanism at different `BLOCK_COUNT`; the
ADC-fill side needs zero knowledge of N once §2 lands. N lives in exactly
one place: SDHCI `BLOCK_COUNT`, already sw-configured pre-`CMD25` today.

Bank-reuse is what matters, not the literal N:
- N≤2: each bank written once, no reuse → no throughput requirement, only
  latency. Existing ping-pong/overflow code degrades into this safely,
  no special-casing.
- N>2: bank reuse required → `T_block ≤ T_fill` must hold or
  `target_frame_full` (correctly) trips `SDCARD_OVERFLOW`.

## §1 — SUPERSEDED: multi-block `CMD25` abandoned in favor of per-block `CMD24`

The original §1/§1b design (BWR-vs-`TRANSFER_COMPLETE` race inside one
`CMD25(BLOCK_COUNT=N)` session) was implemented, then abandoned after
real N=2 testing kept surfacing timing concerns tied to the SDHCI's
double-buffered pipelining (`BufferNumWords=256` = 2 blocks) — the design
relied on that pipelining behaving correctly in an SDHCI IP whose own
comments describe it as not fully exercised, and proving that safe or
unsafe by static analysis alone hit a wall. Rather than keep spending
effort on that, the copy engine now issues its **own `CMD24`
(WRITE_BLOCK) per frame, back to back**, eliminating the double-buffering
dependency entirely: `dat_wrap.sv`'s `new_block_count` computation forces
the SDHCI's internal block counter to 1 for single-block mode regardless
of `BLOCK_COUNT`'s value, so `TRANSFER_COMPLETE` fires directly off
*this* block's real completion, every time — no multi-block session, no
`AUTO_CMD12`, no BWR-vs-`TRANSFER_COMPLETE` race, no "intermediate vs.
last frame" distinction at the SD-protocol level at all. Every frame
takes the identical, always-wait-for-real-completion path.

Consequence: `busy_o` now spans the *entire* per-block transaction (issue
command → data phase → physical completion) for every frame, not just
the session's last one — this was the actual fix for the "copy engine
signals ready for more while the card is still writing" concern.

## §1b — Copy engine FSM (`ce_state_e`)

Software no longer opens/closes anything — no `CMD25`, no
`BUFFER_WRITE_READY` pre-arm, no `AUTO_CMD12`/manual `CMD12`. The copy
engine owns the full per-block SD sequence itself:

```
CE_IDLE
CE_CLEAR_STALE_STATUS, CE_CLEAR_STALE_STATUS_RSP          // discard leftover TRANSFER_COMPLETE/EINTR_STATUS (§1c)
CE_WAIT_CMD_INHIBIT, CE_WAIT_CMD_INHIBIT_RSP              // PRESENT_STATE: card idle?
CE_SET_BLOCK_SIZE, CE_SET_BLOCK_SIZE_RSP
CE_SET_TRANSFER_MODE, CE_SET_TRANSFER_MODE_RSP
CE_SET_ARGUMENT, CE_SET_ARGUMENT_RSP
CE_SUBMIT_CMD24, CE_SUBMIT_CMD24_RSP                      // this write issues CMD24
CE_WAIT_CMD_COMPLETE, CE_WAIT_CMD_COMPLETE_RSP            // + ERROR_INTERRUPT check → CE_OVERFLOW
CE_ACK_CMD_COMPLETE, CE_ACK_CMD_COMPLETE_RSP
CE_WAIT_BUFFER_READY, CE_WAIT_BUFFER_READY_RSP            // unchanged from the old design
CE_COPY_WORD                                               // unchanged
CE_ACK_BUFFERED, CE_ACK_BUFFERED_RSP                       // unchanged
CE_WAIT_TRANSFER_COMPLETE, CE_WAIT_TRANSFER_COMPLETE_RSP  // single condition now, no race
CE_ACK_TRANSFER_COMPLETE, CE_ACK_TRANSFER_COMPLETE_RSP
CE_WAIT_CARD_READY, CE_WAIT_CARD_READY_RSP                // card off DAT0-busy, i.e. PRG → TRAN (§1d)
CE_DONE                                                    // release bank, advance address, maybe SDCARD_DONE
CE_OVERFLOW
```

Key details:
- `TRANSFER_MODE` (byte offset `0x0c`) and `COMMAND` (`0x0e`) are the
  *same* physical 32-bit SDHCI register, split by byte-enable
  (`sdhci_reg_top.sv:550-577`) — written as two separate OBI transactions
  anyway, matching every existing sw example, rather than one combined
  write (technically safe given the byte-enable split, but not worth
  introducing an unexercised write pattern for a few cycles).
  `BLOCK_SIZE`/`BLOCK_COUNT` (`0x04`) share a register the same way;
  `BLOCK_COUNT` is never written since single-block mode force-overrides
  it internally anyway.
- `CE_WAIT_TRANSFER_COMPLETE_RSP` only checks bit1 now — bit4 (BWR)
  structurally cannot re-assert for a single-block command, so there's no
  second condition to race.
- `TRANSFER_COMPLETE` is *not* the end of the block, though — it is a host
  controller event, not a card event. `CE_WAIT_CARD_READY` (§1d) is what
  actually ends the job, and it is inside `busy_o` and before `CE_DONE`
  precisely so `SDCARD_DONE` and the `Fx_FULL` release mean "physically
  committed".
- `is_last_frame_i` (new input, sampled at `start_i` like `copy_f0_i`):
  wired straight to `top.sv`'s `capture_done_q` (§2a) — `CE_DONE` only
  asserts `sdcard_done_set_o` when this was the capture's actual last
  frame, preserving `SDCARD_DONE`'s original meaning even though every
  frame now completes the same way mechanically.
- `block_addr_advance_o` (new output): `+1` or `+`this frame's byte count,
  per `SDCARD_ADDR_MODE.BLOCK_UNITS` — see §2b.
- Boilerplate stays factored through `obi_read`/`obi_write` helpers
  (unchanged from the original §1b rationale): states stay
  explicit/intent-named rather than collapsed into a generic reusable
  REQ/RSP pair, which would need a side "what am I waiting for" tag and
  reintroduce the exact ambiguity this redesign was for.

### §1c — Bugs found via waveform (`pywellen` on `verilator/croc.fst`): stale `TRANSFER_COMPLETE`, stale `EINTR_STATUS`

**Bug 1 — stale `TRANSFER_COMPLETE` (found at N=1).** `busy_o` was
observed dropping while `dat_wrap.sv`'s `write_state_q` was still
`WRITING` — i.e. `SDCARD_DONE`/bank-release fired while the card was
genuinely still streaming the block. Root cause, confirmed by reading
`transfer_complete_q`'s full transition history off the waveform: it was
already `1` from ~28,600 cycles *before* the job even started. Why:
`TRANSFER_COMPLETE` fires on any 1→0 edge of `command_inhibit_dat`
(`sdhci_reg_logic.sv`), which fires for *any* busy-checked command, not
just data transfers — `CMD7` (`SELECT_CARD`, R1b) in `sw/lib/inc/
sdhci_helpers.h`'s `sdh_init()` triggers a real one via
`sd_cmd_dat_busy_i`. Nothing ever clears it afterward (`sdh_cmd()` only
clears `COMMAND_COMPLETE`), so it sits set until the copy engine's
*first-ever* `CE_WAIT_TRANSFER_COMPLETE` poll sees it and returns
immediately, having never waited for anything.

**Bug 2 — stale `EINTR_STATUS.data_timeout_error` (found at N=2).**
`SDCARD_OVERFLOW` fired after frame 1/2, at job 2's very *first*
`CE_WAIT_CMD_COMPLETE_RSP` check — before job 2's own `CMD24` could
plausibly have failed yet. Waveform trace of every `error_interrupt_status`
sub-field showed exactly one bit ever moves: `data_timeout_error`, set
at a timestamp falling inside *job 1's* `CE_WAIT_TRANSFER_COMPLETE`
bounce, a couple of cycles before job 1's own completion. Root cause,
confirmed by tracing `dat_write.sv`'s `dat_tx_state_q`/`dat0_i` and
`dat_wrap.sv`'s `write_state_q` cycle-by-cycle around that timestamp:
`dat_write.sv`'s `STATUS_START_BIT` state samples `dat0_i` exactly once,
a fixed two cycles after `END_BIT` (`BUS_SWITCH`'s hardcoded wait), to
check whether the card has already pulled DAT0 low to start its CRC
status token (`STATUS_START_BIT: if (dat0_i != '0) data_timeout_d = '1;`).
Against this repo's `sdModel.v`, DAT0 is not yet low at that exact
sample — every single block, deterministically, not just occasionally —
so `data_timeout_d` latches. This flag is a passenger: `dat_write.sv`'s
own FSM does not stall or divert because of it, and the block still
completes correctly two cycles later (`write_done` and
`TRANSFER_COMPLETE` both fire right on schedule). But `dat_wrap.sv`'s
outer `write_state_q` *does* react to it, detouring through
`TIMEOUT_WRITING` (instead of `DONE_WRITING_BLOCK`) on the way to
`DONE_WRITING` — and it's specifically `write_state_q == TIMEOUT_WRITING`
that pulses `EINTR_STATUS.data_timeout_error`'s hardware set (`dat_wrap.sv`'s
`error_reporting` block). Structurally the same bug class as Bug 1: a real
W1C status bit sets, for a legitimate hardware reason, but harmlessly with
respect to the block that just completed — and nothing ever clears it, so
it survives into the *next* job, where `CE_WAIT_CMD_COMPLETE_RSP`'s
`ERROR_INTERRUPT` check (added alongside the `CMD24` redesign, §1b) reads
it and wrongly attributes it to that job's own command.

Fix (covers both bugs): `CE_CLEAR_STALE_XFER`/`_RSP` renamed to
`CE_CLEAR_STALE_STATUS`/`_RSP`, still the first state after `CE_IDLE`,
now W1C-clearing `TRANSFER_COMPLETE` *and* every `EINTR_STATUS` bit in one
write — `NINTR_STATUS` (low 16 bits) and `EINTR_STATUS` (high 16 bits) are
the same 32-bit SDHCI register (`SDHCI_ERROR_INTERRUPT_STATUS_OFFSET ==
SDHCI_NORMAL_INTERRUPT_STATUS_OFFSET == 0x30`), so this is a `be`/`wdata`
widening of the existing write, not a new OBI transaction. `ERROR_INTERRUPT`
(bit 15, what `CE_WAIT_CMD_COMPLETE_RSP` actually checks) needs no write of
its own: its hardware `.de` is unconditionally `1`, so it's a
continuously-recomputed OR of the `EINTR_STATUS` sub-bits
(`sdhci_reg_logic.sv`: `error_interrupt_o.d = !clear_i & (|visible_error_status)`)
— clearing the sub-bits makes it fall on its own the next cycle. Clearing
happens before this job's own `CMD24` can cause a fresh edge or a fresh
error, so there's no race with a real completion or a real error from
*this* job. `COMMAND_COMPLETE` doesn't have either problem — `sdh_cmd()`
already clears it after every init command, confirmed by the same
waveform traces showing it set fresh at each job's own command response,
never stale.

**Methodology note for next time**: waveform analysis via `pywellen`
(`uv pip install --python .venv pywellen`, loads FST directly, no VCD
conversion needed) resolved both of these in a few queries each, after
static-analysis speculation had gone in circles — `Waveform.all_vars()`
to find hierarchical signal paths by keyword, `var.signal[i]` for
`(time_ps, value)` transition tuples. Reach for this earlier next time a
"which signal is actually driving this" or "why did this status bit end up
set" question comes up, rather than re-deriving RTL behavior from reading
source alone. Bug 2 specifically was found by first narrowing which of the
~8 `EINTR_STATUS` sub-fields actually moved (only `data_timeout_error`
did), then pulling a tight time window around that exact transition
timestamp across both `dat_write.sv`'s and `dat_wrap.sv`'s internal FSM
signals — cheaper than reasoning about `dat_write.sv`'s state timing from
source alone. Bug 3 (§1d) needed one step further out: correlating the
copy engine's `state_q` against the *testbench card model's* internal
`dataState`/`CardStatus`/`flash_write_cnt` in a single merged timeline.
That's what made "the host thinks it's done, the card doesn't" visible in
one glance, and it's the query to reach for whenever both sides of a
protocol are in the same waveform — don't only instrument the DUT. That
query is now checked in as `scripts/analyze_sdcard_wave.py` (run
`.venv/bin/python scripts/analyze_sdcard_wave.py verilator/croc.fst`); it
prints the per-job timeline, the card's own `WRITE_FLASH` episodes, every
`CMD24` the card decoded and whether it dropped it, and the DAT0-vs-
`TRANSFER_COMPLETE` ordering that `CE_WAIT_CARD_READY` depends on. It
reads FSM state names out of the FST's enum metadata rather than
hardcoding them, so it keeps working when states are added.

### §1d — Bug 3: `TRANSFER_COMPLETE` ≠ block committed (found at N=2, `CE_WAIT_CARD_READY`)

**Symptom.** N=2 ran clean end to end — no `SDCARD_OVERFLOW`, sw printed
`OK: 2 x 512 B written to SDCard starting at LBA 0`, `Simulation finished:
SUCCESS` — but `verilator/sdcard/flash_dump.hex` had real data only in
block 0 (`0x000`–`0x1FB`); block 1 (`0x200`+) was all zeros. Two blocks
reported written, one block actually written.

**First hypothesis (wrong): the LBA isn't advancing.** Ruled out directly
from the waveform. `SDCARD_BLOCK_ADDR` / `cmd24_argument` go `0 → 1 → 2`
exactly as designed, and `adc_acquisition_top.sv`'s hold/advance
`always_comb` plus the PeakRDL-generated `hw=rw, sw=rw` field logic both
behave correctly. The address was fine; the *command carrying it* never
landed.

**Root cause.** The copy engine treated `TRANSFER_COMPLETE` as "this block
is on the card", but it only means SDHCI's own write FSM finished framing
the block on the bus (data → CRC → CRC-status token). The card *starts*
its internal program-to-flash cycle at that point and signals it by
holding DAT0 low for the whole duration (SD spec busy signalling); while
DAT0 is low it sits in `PRG` and rejects new commands. So the engine was
declaring the job done ~500 cycles early and firing the next `CMD24`
straight into a busy card. Cycle-accurate trace correlating the copy
engine's `state_q` against `sdModel.v`'s *internal* FSMs:

```
cyc 874077  card enters PRG, sdModel dataState = WRITE_FLASH
cyc 874095  TRANSFER_COMPLETE rises — with DAT0 already low
cyc 874099  job 1 reaches CE_DONE                    ← ~500 cycles too early
cyc 874111  job 2 issues its CMD24 (argument = 1, correct)
cyc 874200  ...which arrives at a card still in PRG  ← dropped on the floor
cyc 874605  card finally leaves PRG, DAT0 released   ← job 2 long gone
```

The drop is silent by construction: `sdModel.v`'s `24:` handler only runs
its body `if (CardStatus[12:9] == TRAN)`, and its `else` branch just
returns an empty response with *no error bit set*. So the host saw
nothing wrong, streamed a full block of data into a card that had never
been told to write it, and collected a perfectly good
`TRANSFER_COMPLETE` at the end of that too. Exactly one `WRITE_FLASH`
episode for the whole run — hence one block in the dump, and hence sw
happily reporting 2/2.

**Fix.** New `CE_WAIT_CARD_READY`/`_RSP` pair between
`CE_ACK_TRANSFER_COMPLETE_RSP` and `CE_DONE`, polling `PRESENT_STATE`
until `CMD_INHIBIT_CMD`/`CMD_INHIBIT_DAT` are clear *and* DAT0's line
level (bit 20) is back high. `PRESENT_STATE[23:20]` mirrors the raw
DAT[3:0] pads (`sdhci_top.sv:186`:
`hw2reg.present_state.dat_line_signal_level = sd_dat_i`), so bit 20 *is*
the card's busy signal — no extra command, no new register, no `RCA`
needed. Race-free by protocol construction: the busy period starts
contiguously with the CRC-status token that `TRANSFER_COMPLETE` is
derived from, so by the time this state issues its first read (a full OBI
round trip after the `TRANSFER_COMPLETE` poll plus the W1C ack) DAT0 is
already low if the card intends to be busy at all — confirmed on the
waveform, DAT0 was already low when `TRANSFER_COMPLETE` rose. A card that
never goes busy just reads high on the first poll and falls through, so
there's no hang either.

`CMD13`/`SEND_STATUS` polling was the other candidate (what real drivers
typically use) and is strictly more informative, but it needs the card's
`RCA` — which software owns, since software does the init — so it would
have meant a new RDL field plus a register regen. DAT0-level polling gets
the same guarantee from a register the engine already reads.

**Caveat carried forward — `POLL_TIMEOUT`.** 500k cycles (~5 ms at
100 MHz) is comfortable against this model's ~500-cycle busy, but a real
card may legitimately take ~250 ms for a single-block write. This is now
the one wait whose duration is set by flash programming rather than bus
turnaround — raise `POLL_TIMEOUT` before running against real silicon.

**Throughput headroom at N > 2 — checked, fine.** `busy_o` now honestly
covers the card's program time, so a job is ~500 cycles longer, and the
first estimate here was that N > 2 would therefore overflow. It does not.
Measured on the N=4 run after the fix: jobs run ~2850 cycles and start
every ~3175, so there is real headroom. The earlier ~2117-cycle figure
came from the N=2 run's `F0_FULL` → `F1_FULL` gap, which included capture
startup and was too pessimistic. Worth re-measuring if the SD clock
divider or frame size changes; if it ever does go negative, the honest
options are a faster SD clock, overlapping the next block's SRAM→SDHCI
copy with the current block's program time, or `CMD25` multi-block (which
§1 deliberately walked away from). Do not "solve" it by shortening
`busy_o` again — that is precisely the bug this section is about.

**Second, independent bug in the same symptom (`SDCARD_ADDR_MODE` vs the
model).** `sdModel.v` advertises `CCS=1` in its OCR (`32'h40ff8000`, "I am
high capacity, address me in blocks") but its `CMD24` handler then does
`BlockAddr = inCmd[39:8]` and indexes `FLASHmem` with it directly — a raw
*byte* address, and it even checks `if (BlockAddr % 512 != 0)
$display("**Block Misalign Error")`, which is only meaningful for byte
addressing. So against this model, block N must be addressed as `N*512`.
`sw/sdcard_acquisition_Nx.c` had `SDCARD_ADDR_IS_BLOCKS = 1`, so even with
the busy-wait fixed, job 2's `CMD24` argument of `1` would have written at
byte offset 1 — off by one byte and stomping block 0's tail. Changed to
`0` for this testbench, with a comment saying to set it back to `1` for a
real SDHC/SDXC card. The RTL is right either way; this is purely a
what-does-the-model-expect config choice.

### §1e — Bug 4: frames ended one word short (found by `scripts/check_flash_dump.py`)

**Symptom.** With §1d fixed, all N blocks land at the right LBAs and the
ADC sawtooth reads back continuous — but every 512 B block ends with a
`0x00000000` at offset `0x1fc`, so each block carries 127 real words plus
one filler and the payload drifts 4 B per block.

**Not a lost sample.** `0x1e8d → [zero] → 0x1e8e` across the boundary:
the zero is *inserted*, not overwriting anything. That distinction is what
`scripts/check_flash_dump.py` reports separately, and it is what points at
the fill side rather than the copy engine — the copy engine faithfully
copied all 128 words of the frame it was given, one of which had never
been written. Waveform confirms: the ADC DMA wrote 127 distinct words per
frame, with `0x1000_11fc` (F0) and `0x1000_19fc` (F1) never written at all.

**Root cause.** The frame-boundary conditions tested only *where the write
head points*, not *whether the last word was written*:

```systemverilog
if (dma_push)
  hw2reg.WRITE_HEAD.WORD_ADDRESS.next = ...value + 1;

if (reg2hw.WRITE_HEAD.WORD_ADDRESS.value == reg2hw.F0_END_ADDR.WORD_ADDRESS.value
    && current_frame_q == CURRENT_FRAME_0) begin        // ← no dma_push term
  hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F1_START_ADDR...;  // overrides the +1
```

The head lands on `F0_END_ADDR` one cycle after the 127th word is pushed
and then *sits there* until the CDC FIFO produces the next sample — many
cycles, since the ADC runs far slower than the system clock. The ungated
condition fires immediately in that gap and redirects the head to the
other bank, so the word at `F0_END_ADDR` is skipped entirely.

**Fix.** Qualify every frame-boundary test with "a word is actually being
written this cycle" — five sites in `adc_acquisition_top.sv`:
`f0_frame_just_filled` (used by `SINGLE_ACQ_F0`'s `F0_FULL` *and* by the
auto-stop that reverts `MODE` to `IDLE`, which had the same one-word-early
problem), plus the inlined F0/F1 comparisons in `CONTINUOUS_ACQ_F0_F1` and
`ACQ_SDCARD`. The inlined ones use `dma_push` directly, which is already
assigned at that point in the block; `f0_frame_just_filled` is a
continuous assign *outside* that `always_comb`, so it uses
`adc_data_word_ready` (the CDC FIFO output `dma_push` is derived from)
instead — depending on `dma_push` there would make the block's result
order-sensitive, since it is read at the auto-stop before the `case`
assigns it.

**Note the interaction with §1d.** All three of these bugs (`§1c` ×2,
`§1d`, `§1e`) shared one shape: a condition that was *nearly* the right
event — a status bit that really did set, a completion that really did
happen, a head that really did reach the end — used as a proxy for the
event actually wanted. Worth the reflex on the next one: ask what
distinguishes the proxy from the real thing, and whether anything can slip
into the gap.

## §2b — SD address configuration (`SDCARD_ADDR_MODE`, new)

New register, one bit, `BLOCK_UNITS` (sw=rw, hw=r, default 1): `1` =
block addressing (SDHC/SDXC-style — `SDCARD_BLOCK_ADDR` is a raw block
number, CMD24 argument used as-is, advances by `+1`); `0` = byte
addressing (standard-capacity-style — argument still used as-is,
advances by this frame's byte count, derived from the same
`frame_words_q` used for `BLOCK_SIZE` so block size has one source of
truth). `SDCARD_BLOCK_ADDR` itself already existed but was previously
inert (an informational counter only) — it's now the actual CMD24
argument.

## §2 — Mode collapse: `ACQ_SDCARD`

- `adc_acquisition_reg_definition.rdl`: single `ACQ_SDCARD` enum value
  (decide: drop `SINGLE_SDCARD` value, or alias for compat — open q).
- `adc_acquisition_top.sv`: `CONTINUOUS_SDCARD`'s case
  (ping-pong + `target_frame_full`) becomes the only SD branch, run for
  any N. Generalize the auto-revert-to-`IDLE` rule (top.sv:263-265,
  today `SINGLE_SDCARD`-only) to fire on
  `sdcard_done_set || sdcard_overflow_set` unconditionally — safe once §1
  makes `sdcard_done_set` session-scoped for any N.
- sw: `sdcard_acquisition_single.c`/`_cont.c` converge to one flow
  differing only in `BLOCK_COUNT`/`F1_*` setup. Drop the manual
  `SDCARD_BLOCK_ADDR >= N_FRAMES` poll loop in `_cont.c` in favor of
  polling `SDCARD_DONE` (now real for any N).

### §2a — Required: the ADC-fill side needs its own N, period

Without this, §2 is broken for any N, not just N=2. The underlying reason
holds regardless of how the SD side is implemented (multi-block session,
§1's original design; or per-block `CMD24`, current design): the
ADC-fill/ping-pong logic has **no hardware concept of N at all** on its
own — `target_frame_full` only ever detects *reuse* of a bank that's
still full, which structurally cannot happen until frame N+2 (§ "Modes"
table above, bank-reuse framing). Nothing stops it from happily starting
frame N+1 into an already-vacated bank the instant that bank frees up,
regardless of whether that freed up fast (the original BWR-based
shortcut for intermediate blocks) or, as now, at the same uniform speed
as every other frame (`CMD24`'s full per-block completion, §1b) — wasting
frame(s) and then tripping a **spurious** `SDCARD_OVERFLOW` once the ADC
catches up to a bank the copy engine hasn't released yet.

Fix — stop the ADC at the ping-pong boundary itself, zero-latency, not on
any signal downstream of it. (First-pass design gated on `sdcard_start`
instead — rejected during implementation: `sdcard_start` still lags the
boundary crossing by a cycle, which reopens the exact race being closed.
The boundary decision has to gate itself.)

- New register `SDCARD_FRAME_COUNT` (sw=rw, hw=r,
  `adc_acquisition_reg_definition.rdl`): software sets to the same N as
  SDHCI `BLOCK_COUNT`.
- New counter `frames_started_q` (`adc_acquisition_top.sv`), reset to 1
  (F0's initial fill already counts as frame 1), incremented *at* the
  ping-pong boundary crossing itself (same `always_comb` block, same
  cycle `Fx_FULL` is raised and `current_frame_d` would flip) — not on
  `sdcard_start`, which fires later once the copy engine notices.
- New sticky `capture_done_q`: at a boundary crossing, if
  `frames_started_q >= SDCARD_FRAME_COUNT`, the frame just completed was
  the last wanted one — set `Fx_FULL` as usual but do **not** flip
  `current_frame_d`/advance `WRITE_HEAD`/increment `frames_started_q`;
  latch `capture_done_q` instead. Both reset alongside
  `CNTRL.RESET_WRITE_HEAD`.
- `ACQ_SDCARD` case in `adc_control_logic`: gate the whole push/ping-pong
  body on `!capture_done_q`, in addition to the existing
  `target_frame_full` check — once set, the ADC-fill side parks until
  software moves `MODE` away from `ACQ_SDCARD`, rather than re-evaluating
  `target_frame_full` against a bank it deliberately won't reuse (which
  would otherwise itself look like an overflow on a clean capture).

Originally accepted a tradeoff here (N configured in two places: SDHCI
`BLOCK_COUNT` and `SDCARD_FRAME_COUNT`) — moot now that §1's `CMD24`
pivot removed `BLOCK_COUNT`/multi-block sessions entirely.
`SDCARD_FRAME_COUNT` is the *only* place N is configured.

## §3 — Other fixes queued (independent, land separately)

- **`CONTINUOUS_ACQ_F0_F1` overflow gap** (top.sv:294-312): add the same
  `target_frame_full` check `CONTINUOUS_SDCARD` already has.
- **`WORDS_PER_BLOCK` hardcode** (`adc_acquisition_sdcard_controller.sv:88`):
  copy length is a fixed 128, ignores `Fx_END_ADDR` entirely. Derive from
  `(Fx_END_ADDR - Fx_START_ADDR) + 1` at job-start instead — `Fx_END_ADDR`
  is the *last* word's address (inclusive), so the `+1` is required, not
  optional (caught as a real bug post-implementation: omitting it silently
  copies one word short, which then stalls the SDHCI's own block-completion
  tracking forever since it independently expects `BLOCK_SIZE`/4 words
  before it'll ever consider the block done).

## §4 — Concepts parked, not scoped yet

- ~~Expose `dat_write.sv`'s internal `write_done` as a new SDHCI status
  bit~~ — moot, §1's `CMD24` pivot achieves true per-block completion
  without touching the `sdhci` IP at all.
- ~~Chunked sessions (auto close/reopen `CMD25` every K frames)~~ — moot,
  per-block `CMD24` already gives a durability checkpoint every single
  frame, finer-grained than any chunk size could.
- 3rd SRAM bank: more burst slack, doesn't raise sustained throughput
  ceiling.
- Truly unbounded capture (no upper bound on `SDCARD_FRAME_COUNT`,
  capture until software decides to stop): §2a still requires *some*
  frame count to gate the ADC-fill side, so this isn't free even with
  per-block `CMD24` — would need `SDCARD_FRAME_COUNT` to support a
  sentinel ("no limit") the same way `CONTINUOUS_ACQ_F0_F1` is unbounded
  today, relying purely on `target_frame_full` backpressure and a
  software-driven stop.
- Unify `SINGLE_ACQ_F0`/`CONTINUOUS_ACQ_F0_F1` the same way — no SDHCI
  `BLOCK_COUNT` to anchor N to, needs a new local frame-count register.
  Separate scope from §2.
- Interrupt-driven CPU drain (`interrupt_frame_full_o` exists, unused by
  example sw) instead of polling, for the CPU-drained modes.

## Implementation plan

Decisions (resolving the prior open questions):

1. **Breaking removal** of `SINGLE_SDCARD`, no compat alias. Pre-production
   RTL, single consumer (this repo's own sw), no external users — an alias
   just leaves permanent dead encoding for no benefit.
2. Update all sw referencing `ADC_ACQ_MODE_SINGLE_SDCARD` in the same
   change (build breaks otherwise; no shim).
3. Bundle §3's two fixes into this pass — both are small, mechanically
   similar to code already being touched, and reviewing the same function
   twice is wasted motion. Kept as separate commits within the change for
   reviewability.

### Step 1 — `adc_acquisition_sdcard_controller.sv`
- Replace `wr_state_e` wholesale with `ce_state_e` per §1b (not a patch —
  `poll_after_copy_q` and `bwr_wait_cnt_q` go away, renamed/restructured
  as described there).
- Derive copy length from `(Fx_END_ADDR - Fx_START_ADDR) + 1` (word count —
  `Fx_END_ADDR` is the last word's address, inclusive, so the `+1` is
  required) at job-start instead of `localparam WORDS_PER_BLOCK = 128`;
  latch into a register alongside `copying_f0_q`. Note: this only changes
  what the copy engine copies — it does not cross-check against SDHCI
  `BLOCK_SIZE`; a mismatch there is still a software configuration error,
  not something this step adds detection for.

### Step 2 — Mode collapse (incl. §2a)
- `adc_acquisition_reg_definition.rdl`: drop `SINGLE_SDCARD`, keep single
  `ACQ_SDCARD` value (pick either existing encoding, e.g. `0x18`). Add
  `SDCARD_FRAME_COUNT` register (§2a).
- Regenerate via `rtl/adc_acquisition/reg/generate.sh`
  (`peakrdl regblock` / `peakrdl html` / `peakrdl c-header`) — do not
  hand-edit the generated `adc_acquisition_reg_pkg.sv`, `adc_acquisition_reg.sv`,
  `doc/`, or `sw/lib/inc/adc_acquisition_reg.h`.
- `adc_acquisition_top.sv`: remove the `SINGLE_SDCARD` case; `sd_mode`
  and the `ACQ_SDCARD` case become the old `CONTINUOUS_SDCARD` logic
  unconditionally (ping-pong + `target_frame_full`, any N), plus the new
  `frames_started_q`/`capture_done_q` counter and gate from §2a.
  Generalize the auto-revert-to-`IDLE` condition (currently
  `SINGLE_SDCARD`-only, top.sv:263-265) to `MODE==ACQ_SDCARD &&
  (sdcard_done_set || sdcard_overflow_set)`.
- `sw/lib/inc/adc_acquisition.h`: `ADC_ACQ_MODE_SINGLE_SDCARD` /
  `ADC_ACQ_MODE_CONTINUOUS_SDCARD` → single `ADC_ACQ_MODE_ACQ_SDCARD`.
- `sw/sdcard_acquisition_single.c` and `_cont.c`: converge to one flow
  (`sdcard_acquisition.c`?) parametrized by `BLOCK_COUNT`/frame count —
  now also sets `SDCARD_FRAME_COUNT` to the same N before enabling the
  mode; or keep as two thin examples calling a shared helper — pick when
  writing it, not architecturally significant. Drop the manual
  `SDCARD_BLOCK_ADDR >= N_FRAMES` poll loop, poll `SDCARD_DONE` instead.
- `sw/test/test_adc_sdcard.c`, `test_adc_continuous_acq.c`,
  `test_adc_single_acq.c`, `test_adc_regs.c`: update mode references and
  set `SDCARD_FRAME_COUNT`.

### Step 3 — Bundled fixes (§3)
- `adc_acquisition_top.sv`, `CONTINUOUS_ACQ_F0_F1` case: add the same
  `target_frame_full` check `ACQ_SDCARD`'s case has.
- Already covered by Step 1 (`WORDS_PER_BLOCK`).

### Step 4 — Verification (updated for the `CMD24` design, §1/§1b)
- `verilator/` sim (`rtl/test/tb_croc_pkg.sv` + `sdModel.v`) against:
  N=1, N=2, N=3+ continuous — every frame now takes the identical path
  (issue `CMD24` → data phase → real `TRANSFER_COMPLETE`), so there's no
  more race to distinguish between frames; what actually needs checking
  per-N is the §2a boundary behavior (does capture stop exactly at N,
  no N+1 frame ever started) and that `SDCARD_BLOCK_ADDR` advances
  correctly across repeated `CMD24`s.
  - `CE_WAIT_CMD_INHIBIT`: confirm it actually polls (not just passes
    through) between back-to-back frames — `PRESENT_STATE` should read
    idle quickly after the previous frame's `TRANSFER_COMPLETE`, but this
    is the one new wait with no prior art in this codebase to lean on.
  - Command-level error path: force `ERROR_INTERRUPT` (e.g. via
    `sdModel.v`) during `CE_WAIT_CMD_COMPLETE` and confirm it routes to
    `CE_OVERFLOW` rather than proceeding into a data phase.
  - Both `SDCARD_ADDR_MODE` settings: block addressing (`+1` per frame)
    and byte addressing (`+`frame byte count per frame) — confirm
    `SDCARD_BLOCK_ADDR` advances correctly and matches what actually
    landed on the card at that LBA/byte offset.
  - N=2 specifically (§2a regression, still applicable): confirm no 3rd
    frame is ever started and `SDCARD_OVERFLOW` stays clear on a clean
    2-frame capture.
  - Forced-slow-card injection (via `sdModel.v`): still trips
    `SDCARD_OVERFLOW` through `target_frame_full`/`POLL_TIMEOUT`, not a
    stuck FSM.
- `sw/sdcard_acquisition_Nx.c`: exercise via direct run (no more manual
  CMD25/CMD12 for it to get subtly wrong) across N=1, 2, and a streaming
  N (e.g. 8).
- New: a `CONTINUOUS_ACQ_F0_F1` overflow test exercising Step 3's fix
  (currently no test covers this path since the bug means there was
  nothing to assert on).
