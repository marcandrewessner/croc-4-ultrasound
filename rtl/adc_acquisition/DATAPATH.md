# ADC → SD Datapath — Architecture Plan

Working doc for planning the datapath, not a tutorial. Current-state facts
are cited to file:line where it matters for a decision; skip otherwise.

## Modes — as implemented

| Mode | Banks | Session covers | Consumer | N source | Throughput req |
|---|---|---|---|---|---|
| `IDLE` | — | — | — | — | — |
| `SINGLE_ACQ_F0` | F0 | sw-set span | CPU read | fixed 1 | none |
| `CONTINUOUS_ACQ_F0_F1` | F0+F1 | sw-set span | CPU read | unbounded | `T_read < T_fill` |
| `SDCARD_CONTINUOUS` | F0+F1 | one whole bank (2 KiB, 4 blocks) | HW copy + card | `SDCARD_FRAME_COUNT` | `T_session ≤ T_fill` |
| `SDCARD_PULSE` | F0+F1 | **both** banks (4 KiB, 8 blocks) | HW copy + card | fixed 2 (both banks) | **none** |

`SINGLE_SDCARD`/`CONTINUOUS_SDCARD` were collapsed into what is now
`SDCARD_CONTINUOUS` (§2, landed): one mechanism, parametrized by frame
count. The ADC-fill side knows N only through `SDCARD_FRAME_COUNT`; the SD
side's per-session geometry is
`SDCARD_BLOCK_SIZE`/`SDCARD_BLOCK_COUNT` (§0). `SDCARD_PULSE` (§5) is the
same SD mechanism run once over both banks after capture has stopped.

Bank-reuse is what matters, not the literal N:
- N≤2: each bank written once, no reuse → no throughput requirement, only
  latency. The ping-pong/overflow code degrades into this safely,
  no special-casing.
- N>2: bank reuse required → `T_session ≤ T_fill` must hold or
  `target_frame_full` (correctly) trips `SDCARD_OVERFLOW`. `T_fill` is now
  4× longer than in the per-block design (a whole bank, not one block), which
  is most of why the larger session is affordable.
- `SDCARD_PULSE` sits outside this entirely: no bank is reused *and* no
  session overlaps a fill, so the throughput requirement is not merely
  slack, it is absent by construction (§5).

## §0 — CURRENT DESIGN: full-SRAM double buffering, `CMD25` per bank

> Supersedes §1/§1b below. §1's per-block `CMD24` decision was reversed once
> the buffer geometry changed; §1c–§1e remain live, they are bugs in code
> that still exists. Read this section first — §1 is kept for the reasoning
> trail, not as a description of the implementation.

A frame is now **one entire SRAM bank** (2 KiB = 4 × 512 B blocks), not one
SD block. Both ADC banks are used, F0/F1 ping-pong as before, and each filled
frame is streamed out as a **single `CMD25` (WRITE_MULTIPLE_BLOCK) session
closed by `AUTO_CMD12`**. The ADC fills one bank while the copy engine
streams the other, so capture stays continuous while each SD transaction
carries 2 KiB instead of 512 B.

**Why §1's objection does not apply here.** §1 abandoned `CMD25` because
`BUFFER_WRITE_READY` was being used as a proxy for *per-block physical
completion* — which it is not, it reflects SDHCI-internal buffer occupancy —
and that proxy was needed only because banks had to be released one block at
a time. With a whole bank per session, nothing needs per-block completion:
the bank is released once, at the end of the session, on its single real
`TRANSFER_COMPLETE`. BWR goes back to meaning only "you may push more words".
The hazard is designed out rather than managed. (Note this is *not* the
"shorten `busy_o`" mistake §1d warns against — `busy_o` still spans the whole
session through `AUTO_CMD12` and the card's final `PRG`→`TRAN`.)

**Backpressure is what makes it work.** Streaming a 2 KiB frame through the
1 KiB SDHCI DAT buffer requires the data port to be able to say "not now".
That path existed but was disabled: `sdhci_top.sv` tied the register
interface's `buffer_data_port_*_ready_i` to `1'b1` unless
`AllowNoncompliantBufferSizes`, so `dat_buffer.sv` still gated its internal
`reg_push` on a ready the bus never honoured and over-pushed words were
**silently dropped**. Those inputs are now routed unconditionally, so
`reg_ready` → `reg_rsp.ready` → OBI `gnt` (`sdhci_obi_to_reg.sv`) genuinely
stalls the copy engine. Done this way rather than by setting
`AllowNoncompliantBufferSizes`, which reaches the same signal but also
disables the buffer-size assertions and weakens BWR from "room for a whole
block" to "room for one word".

Consequences worth knowing:

- **The engine does not pace per block.** It waits for BWR once, then pushes
  the whole frame, stalling on the grant. Safe because `dat_buffer.sv`'s
  block-boundary logic pulses `buffer_write_enable` (the status bit) low
  without touching `buffer_data_port_write_ready_o` (the flow-control
  signal), so the data path never closes mid-session.
- **Inter-block card busy is the IP's problem, not ours.** `dat_write.sv`
  already ends every block with `STATUS_END_BIT → BUSY: if (dat0_i) → DONE`,
  so §1d's "host thinks it's done, card doesn't" cannot recur *between*
  blocks of a session. `CE_WAIT_CARD_READY` still guards the session end.
- **`AUTO_CMD12` cannot be waited on directly** — see §0c. It produces no
  `COMMAND_COMPLETE`; `CE_WAIT_CARD_READY` covers it via `CMD_INHIBIT_DAT`,
  requiring its pass condition on two consecutive polls.
- **BWR must be cleared at session start.** BWR re-asserts at every block
  boundary but is acked only once, so a session reliably *ends* with it set
  and nothing else clears it. `CE_CLEAR_STALE_STATUS` now discards it along
  with `TRANSFER_COMPLETE` and `COMMAND_COMPLETE` — same stale-sticky-status
  class as §1c, caught by asking what §1c's lesson implied for the new flow.
- **`CE_COPY_WORD` needs a timeout.** The grant can now stall indefinitely,
  and `accepts_data_port_chunk` goes low permanently once SDHCI has taken
  `BLOCK_COUNT` blocks — so a mis-sized session would hang forever. The copy
  loop counts cycles since the last *accepted* word (not total duration, which
  legitimately exceeds `POLL_TIMEOUT` on a slow card) and bails to
  `CE_OVERFLOW`.
- **N is configured in two more places.** `SDCARD_BLOCK_SIZE` and
  `SDCARD_BLOCK_COUNT` are new registers; hardware writes them into SDHCI at
  session start. Software must keep
  `SDCARD_BLOCK_SIZE × SDCARD_BLOCK_COUNT == frame byte count` — not checked
  in hardware (a mismatch surfaces as the `CE_COPY_WORD` timeout above).
  `SDCARD_FRAME_COUNT` is unchanged in meaning: frames, i.e. bank-fills.
- **`SDCARD_BLOCK_ADDR` advances per session, not per block** — by
  `SDCARD_BLOCK_COUNT` (block units) or the frame's byte count (byte units).
  Within a session the card advances its own write pointer.

### §0a — Bug 5: §1c's "harmless passenger" is fatal to a multi-block session

First `CMD25` run (N=1) failed with `SDCARD_OVERFLOW` after 0/1 frames. The
copy engine reached `CE_COPY_WORD`, pushed **384 of 512 words — exactly 3
blocks** — and then stalled until the new stall timeout fired 5.02 ms later.

The stall was the symptom; the session had already died on block 1:

```
8847.38 us  dat_wrap write_state_q = WRITING
8868.39 us  ... = TIMEOUT_WRITING        ← block 1 aborts
8868.40 us  ... = DONE_WRITING           ← whole session over, after ONE block
8868.41 us  dat_state_q leaves WRITE     ← buffer stops draining for good
```

With the DAT side out of `WRITE`, the 1 KiB SDHCI buffer never drained again:
384 pushed − 128 transmitted = 256 words resident = full, grant withheld
forever. `wr_idx_q` froze at 384.

Root cause is the §1c Bug 2 sampling race, unchanged and still firing on every
block: `dat_write.sv`'s `STATUS_START_BIT` sampled `dat0_i` exactly once at the
nominal `N_CRC` = 2-clock turnaround, and `sdModel.v` starts its CRC-status
token ~1.15 SD clocks later (measured: sample at 8868260 ns reads 1, DAT0 falls
at 8868283 ns). The whole 5-bit token was therefore read one clock early —
`status_q` came out `001` instead of `010`, `end_bit_err_q` set as well, and
`data_timeout_q` latched.

What changed is only the *consequence*. §1c called this flag "purely a
passenger" because under single-block `CMD24` the `TIMEOUT_WRITING` detour
still reached `DONE_WRITING` on a session that was one block anyway — the block
had already landed and the only residue was a stale `EINTR_STATUS` bit. But
`DONE_WRITING_BLOCK` is the **only** state that loops back to
`WAIT_FOR_WRITE_BUFFER` for the next block, and `TIMEOUT_WRITING` bypasses it
(`dat_wrap.sv:313-322`). So the flag silently truncates *any* multi-block
session to one block, at any `BLOCK_COUNT`.

Fix in `dat_write.sv`: `STATUS_START_BIT` now searches for the start bit over a
`StatusStartWindow` (default 8 SD clocks) instead of sampling once, latching
`data_timeout` only if it never arrives. The zero-margin sample was fragile
independent of this model — real pad and board round-trip delay at 50 MHz eats
a full clock on its own.

Lesson, and it is the same one §1c and §1e already stated in a different form:
a defect written off as harmless was only harmless *against the surrounding
design*. When that design changes, re-derive the "harmless" conclusion instead
of inheriting it. Both §1c bugs were re-read during this work and their
*mechanism* was correctly carried over — what was not re-checked was whether
their **consequence** still held.

### §0c — Bug 6: `AUTO_CMD12` produces no `COMMAND_COMPLETE`

With §0a fixed, the session ran correctly end to end — `status_q` = `010`,
`data_timeout_q` never set, `write_state_q` looping
`WRITING → DONE_WRITING_BLOCK → WAIT_FOR_WRITE_BUFFER` per block,
`wr_idx_q` reaching 512, `AUTO_CMD12` issued and completed — and then the copy
engine sat in `CE_WAIT_AUTOCMD12_COMPLETE` polling forever.

That state was waiting for an event that structurally cannot occur.
`autocmd_wrap.sv:196-198` masks the status bit the interrupt is derived from:

```systemverilog
// autocmd12 execution should not inhibit the driver
assign command_inhibit_cmd_o.d = driver_cmd_queued_q |
                                 (cmd_inhibit_logic && ~running_autocmd12_q);
```

So `present_state.command_inhibit_cmd` never rises for an auto command, and
`COMMAND_COMPLETE` is generated from that field's 1→0 edge
(`sdhci_reg_logic.sv:201-205`) — no rise, no edge, no interrupt. This matches
the SDHCI spec, which does not generate Command Complete for Auto CMD12. The
internal `cmd_inhibit_logic` *does* toggle (measured rising at 10146.50 µs,
falling at 10148.58 µs), which is what makes the mistake easy to make from a
waveform: the signal that moves is not the signal the interrupt watches.

**Fix.** The four `CE_*_AUTOCMD12_*` states are gone. `CE_WAIT_CARD_READY`
covers CMD12 via `CMD_INHIBIT_DAT`, which *does* assert for its R1b response
(`dat_state` → `BUSY`). Two additions:

- **Two-consecutive-poll requirement.** The original worry was real:
  `TRANSFER_COMPLETE` fires on `command_inhibit_dat`'s 1→0 edge and CMD12
  re-raises it one cycle later, so there is a 1-cycle window where everything
  reads idle with CMD12 not yet started. Requiring two consecutive passes
  closes it without depending on arithmetic about OBI round-trip length — the
  CMD12 busy window is ~107 cycles against a poll interval of ~3.
- **`CE_CHECK_ERRORS`.** One `NINTR_STATUS` read before `CE_DONE`. With no
  `COMMAND_COMPLETE` to inspect, this is the only place an `AUTO_CMD12` failure
  (`EINTR_STATUS` bit 8 → `ERROR_INTERRUPT`) can be detected at all.

**Do not use DAT0 for this race.** It reads high across the whole window
against `sdModel.v`, whose CMD12 asserts no R1b busy — verified on the
waveform. DAT0 is the right signal for the card's own programming busy
afterwards and useless for this.

Same lesson as §0a from the other direction: there I inherited a conclusion
without rechecking it; here I invented a wait from a plausible mechanism
without checking whether the IP actually generates the event. Both are the
"nearly the right signal" failure this file keeps recording.

### §0d — Bug 7: the second session's `CMD25` went out as a second `CMD12`

With §0a and §0c fixed, N=4 got session 1 fully onto the card and then failed
with `SDCARD_OVERFLOW after 1/4 frames`. The overflow was *not* the copy
engine's: `sdcard_overflow_set` never pulses in the whole run. The copy engine
hung in `CE_COPY_WORD` (`wr_idx_q` frozen at 384 = 3 blocks, exactly the §0a
signature of a session that died on block 1), the ADC kept filling, and
`adc_acquisition_top`'s own `target_frame_full` raised the status bit at
12982 µs. Note the sw message reads "SDHCI was not ready", which is what the
ADC side concluded — the actual fault is two levels down.

Session 2's `CMD25` never reached the card. Decoding the CMD line bit by bit
off the waveform (sample `i_sd_card.cmd_i` on each `sd_clk_i` posedge while
`cmd_en_i`):

```
job 1  10934.15 us  0 1 011001 00000000...  -> CMD25, arg 0        (correct)
job 2  11958.15 us  0 1 001100 00000000...  -> CMD12, arg 0        (!!)
```

The card answered R1 with index 12 and stayed in `TRAN`; it never entered
`RCV`, `dataState`/`BlockAddr`/`flash_write_cnt` never move again for the rest
of the run. But `data_present_select` was still set in the COMMAND register, so
SDHCI ran the data phase anyway — streaming block 1 at a card that had not been
told to receive it, getting no CRC status token back, and so tripping
`dat_write.sv`'s (now windowed, §0a) start-bit search into a real
`data_timeout` → `TIMEOUT_WRITING` → session over after one block → the 1 KiB
DAT buffer never drains → the grant is withheld forever → `CE_COPY_WORD` hangs.
Every downstream symptom follows from the wrong command index.

**Root cause, `autocmd_wrap.sv`.** `running_autocmd12_q` means "the command
executing right now is the auto CMD12", but it is only ever reassigned at
`command_started`, so it stays set through the entire idle gap after the auto
CMD12 completes. The command mux keyed off it:

```systemverilog
assign is_autocmd12 = autocmd12_queued_q | running_autocmd12_q;
assign current_cmd  = is_autocmd12 ? 6'd12 : reg2hw.command.command_index.q;
assign current_arg  = is_autocmd12 ? '0    : reg2hw.argument.q;
```

and `accepted_cmd_q`/`accepted_arg_q`/`accepted_rsp_type_q` latch `current_*`
*at* `command_started` — the one cycle where the stale 1 is still visible
(`running_autocmd12_q` clears in the same cycle's `_d`). Confirmed directly:
`accepted_cmd_q` has exactly one transition in the whole run, to 12 at
11041.24 µs, and no transition at all at 11958.15 µs when job 2's command was
accepted. So *every* driver command issued after an `AUTO_CMD12` is rewritten
into another `CMD12` — session 2 onward, forever.

**Fix.** Key the mux on `autocmd12_queued_q` alone (`starting_autocmd12`),
which is precisely the arbitration `request_commands` itself applies at
`command_started`. The stability argument in the old comment is obsolete: the
values are re-latched into `accepted_*_q`, `cmd_logic` reads those and not the
mux, and `cmd_needs_busy_o` is sampled by `dat_wrap.sv` only under
`cmd_started_i` — all three consumers sample exactly at `command_started`.
`cmd_data_present_o` and `active_transfer_direction_q` in the same module
already key off `autocmd12_queued_q` for this reason. `running_autocmd12_q` is
left as is for the status/response/error paths, which all qualify it with an
event that can only occur during execution, plus a comment saying what it must
not be used for.

This is `AUTO_CMD12`'s first use in this repo, so nothing had exercised the
"driver command after an auto command" path before §0. Same family as §0a and
§0c and worth stating in the general form the file has been circling: **a flag
that names a state is only as good as the event that clears it.** §1c's bits
were never cleared, §0c's was never set, this one is cleared a command too
late. When reading an IP's status flag, find its clear edge before trusting its
level.

### §0b — Real-card readiness (the model hides four things)

Everything above was validated against `sdModel.v`, which is permissive in
ways that matter. These were fixed together, and only the last is a
consequence of the `CMD25` work:

1. **`ACMD6` was never sent** — `sdh_init()` set the *host* to 4-bit
   (`HOST_CTL`) but never told the card, leaving it listening on DAT0 alone
   while the host spread each byte over four lines. The model hides this
   completely: its data path hardcodes 4-bit reception
   (`bitBlockRec = blockSize * 2`) regardless of its own `BusWidth` register,
   which `ACMD6` is the only thing that sets. On a real card this corrupts the
   very first block. Now sent right after `CMD7`, before `HOST_CTL`.
2. **No `CMD6` high-speed switch** while clocking the bus at 50 MHz — out of
   spec for a card still in default speed (25 MHz ceiling). Now done via
   `sdh_switch_func()` (check, then set, reading the mandatory 64-byte status
   both times so the DAT lines don't stay busy into the next command), and a
   failed switch is a hard init failure.

   **It is skipped entirely when `SIM_CARD` is set, and must be.** `sdModel.v`
   has no `SWITCH_FUNC` handler — its `6:` case only accepts the `ACMD6` form —
   and the first attempt to issue it anyway hung the run. The failure is not
   cheap: `CMD6` carries a data phase, so an unanswered one parks `dat_state_q`
   in `READ`/`READING` waiting for a 64-byte block that never comes, released
   only by the SDHCI data timeout — 1.34 s of simulated time at
   `SDHC_TIMEOUT_MAX`, i.e. hours of wall clock under Verilator. The
   software-side spin timeouts (~300 ms) give up long before the hardware does,
   so every later command inherits a hung DAT line. Worth remembering as a
   general shape: *a software timeout does not undo the hardware state the
   timed-out operation left behind.* On the real-card path a failed `CMD6` now
   issues a CMD/DAT software reset before returning, for the same reason.
3. **High speed is required, not optional.** At a spec-legal 25 MHz the 4-bit
   bus gives 12.5 MB/s against the ADC's 16 MB/s (8 MSa/s × 2 B), so
   `SDCARD_CONTINUOUS` would overflow by construction. Only 50 MHz
   (25 MB/s) closes, which is why a failed `CMD6` is treated as fatal rather
   than a fallback. `SDCARD_PULSE` (§5) is exempt — it does not stream while
   capturing — but high speed is still what makes its dump time reasonable.
4. **`POLL_TIMEOUT` was 5 ms** against an SD-spec worst case of 250 ms per
   write. Now 25M cycles (250 ms at 100 MHz), counter widened 20 → 25 bits.
   This became urgent with `CMD25`: `CE_WAIT_TRANSFER_COMPLETE` now spans the
   whole session including every inter-block program cycle, and `CE_COPY_WORD`
   stalls on the grant for as long as the card is busy between blocks — so at
   5 ms a healthy transfer to a slow card would have been aborted as an
   overflow. The model's ~5 µs busy would never have shown it.

Also `SDCARD_ADDR_IS_BLOCKS`, which must be 0 for the model and 1 for a real
SDHC/SDXC card, is now behind a `SIM_CARD` build switch instead of a comment
telling you to remember.

**Not verified in simulation:** the `CMD6` path. The model cannot answer it,
so it is exercised only on real hardware — the sim run proves the *warning*
branch, not the switch itself.

### §0e — Real-card audit (what passing against `sdModel.v` does not prove)

A clean N=4 run against the model says the *mechanism* works. It does not say
the design is spec-compliant, because the model is permissive in exactly the
places compliance matters. Audit of the whole SD-facing path against the SD
Physical Layer / SDHCI specs rather than against the model. Fixed here:

- **`CMD6` status decode was indexing the wrong words** (`sdhci_helpers.h`).
  The bit offsets from the spec were right; the mapping to
  `BUFFER_DATA_PORT` words was not. `dat_read.sv:118-136` enters each received
  byte at `[31:24]` and shifts the build-up register right by 8, so the
  **first** byte of every group of four ends up in `[7:0]` — little-endian, per
  the SDHCI spec's byte-stream port, the opposite of the big-endian reading
  the code assumed. Correct offsets: group 1 support bit for High Speed is
  `sw[3]` bit **9** (was 17), selected function is `sw[4][3:0]` (was
  `[23:20]`). This is a hard init failure on every real card — high speed is
  mandatory (§0b.3) and a failed switch returns 0 — and it is structurally
  unreachable in simulation, since the model has no `SWITCH_FUNC` handler. It
  had been flagged in-code as `UNVERIFIED INDEXING`; the verification it needed
  was of the RTL that packs the words, not of a real card.
- **No power-up delay before `CMD0`.** The spec requires ≥74 SD clocks after
  the supply is stable, and allows 1 ms for the ramp. Bus power now goes on
  before the clock, followed by ~2 ms of clocking before the first command.
- **`ACMD41` gave the card ~31 ms.** Fixed iteration count, sized by nothing in
  particular; the spec allows the card 1 s and real cards commonly take
  100 ms+. Now a 1 s deadline. The model answers on the first poll, so this
  could never have shown up in simulation.
- **Addressing mode was a build switch.** `SDCARD_ADDR_MODE` is negotiated, not
  a property of the design: OCR bit 30 (CCS) is the card's own answer. It is
  now read during init (`sdh_card_is_block_addressed()`), with the `SIM_CARD`
  override kept for the model, which advertises CCS=1 and then behaves like a
  standard-capacity card. A real SDSC card would previously have been written
  at 512× the intended offset.

Known and deliberately not fixed here — each is a design decision, not a typo:

- **The card's R1 response is never inspected.** `CE_WAIT_CMD_COMPLETE_RSP`
  checks `COMMAND_COMPLETE`/`ERROR_INTERRUPT`, which cover CRC/index/timeout of
  the *response frame*, not the card's own status bits inside it. A real card
  rejecting `CMD25` (`WP_VIOLATION`, `ADDRESS_ERROR`, wrong state) answers with
  a perfectly well-formed R1 carrying an error bit, and the engine streams the
  frame at it anyway, then hangs until `POLL_TIMEOUT` (250 ms) and reports
  `SDCARD_OVERFLOW`. Fails safe, diagnoses badly. Fix would be one more
  `RESPONSE0` read in `CE_WAIT_CMD_COMPLETE_RSP` against the R1 error mask.
- **A CRC-status token of `101`/`110` does not stop the session.**
  `dat_wrap.sv:305-312` leaves `WRITING` on `write_done` regardless of
  `write_crc_err`, which is only *reported* into `EINTR_STATUS`. So a block the
  card rejected is followed by the session's remaining blocks, and the failure
  surfaces only at `CE_CHECK_ERRORS`, one session late. Against the model this
  never happens; on a real card it is the normal signalling for a bad block.
  There is also no retry anywhere — a failed frame is lost, by design.
- **Nothing recovers the card after `CE_OVERFLOW`.** The engine sets the status
  bit and parks, leaving the card wherever it was — possibly mid-`CMD25` in
  `RCV`, or in `PRG`. `AUTO_CMD12` does still fire on the truncated-session
  path (`dat_wrap.sv:371-379` triggers on `WRITE -> READY` regardless of how
  `WRITE` was left), so the card is usually stopped, but nothing verifies it.
  A `CMD12` + `CMD13` + CMD/DAT software reset before any retry is what a real
  driver would do.
- **`dat_wrap.sv:191-200` gives the card 4 SD clocks to assert R1b busy** and
  calls it done otherwise. That is 2 clocks of margin over the spec's 2, at
  50 MHz, before any pad or board round-trip delay — the same zero-margin
  sampling shape as §0a, in a different state machine. The copy engine happens
  to be immune (`CE_WAIT_CARD_READY` re-checks DAT0's *level*, so a missed busy
  costs nothing), but software using `CMD_INHIBIT_DAT` after an R1b command is
  not. Same fix as §0a: make the window a parameter and widen it.
- **`N_WR` (response end bit → data start bit) is not counted anywhere.** The
  path `WAIT_FOR_RSP → WAIT_FOR_WRITE_BUFFER → START_WRITING` lands the start
  bit roughly 2 SD clocks after the response, which is the spec minimum with
  no margin. The model does not police it. Worth a scope check on first
  bring-up.

And the one that is not a compliance question at all but is the most likely
thing to break on real hardware — see §0f.

### §0f — Throughput: the model is not a card, and bus rate is not write rate

§0b.3 concluded "only 50 MHz (25 MB/s) closes" against the ADC's demand. That
is the *bus* rate. What has to keep up is the card's **sustained write** rate,
and more importantly its **worst-case pause**, because the design's entire
elasticity is one SRAM bank.

Measured on the N=4 run (`ADC_SAMPLING_FREQ_MHZ = 9`), all off the waveform:

| | |
|---|---|
| ADC fill rate | 450 `WRITE_HEAD` advances per 100 µs = 4.5 MW/s = **18 MB/s** (= 9 MSa/s × 2 B, so no samples are being dropped) |
| Time to fill one bank | 512 words = **113.8 µs** |
| Session (`CE_CLEAR_STALE_STATUS` → `CE_DONE`) | **108.4 µs** |
| of which data phase | 4 × 26.2 µs = 104.8 µs (19.6 MB/s effective on a 25 MB/s bus) |
| **Slack per frame** | **~5 µs, i.e. ~95% utilisation** |

That is against `sdModel.v`, whose program cycle is ~5 µs per block and which
never pauses. There is essentially no headroom left for a card that does. A
real card's `CMD25` stream is paced by its internal write: sustained rates for
a decent UHS-I card are fine, but garbage-collection and wear-levelling pauses
of **1–100 ms** are normal and permitted (the spec's own worst case for a
single write is 250 ms — which is exactly why `POLL_TIMEOUT` is 250 ms). Any
pause beyond ~5 µs overflows, because there is nowhere to put the samples:
total buffering is 2 KiB, i.e. **114 µs** at this rate.

**The passing run does not exercise this.** In that same waveform, frames were
*started* 1024 µs apart (`frames_started_q` at 10934 / 11958 / 12982 µs) while
a bank fills in 114 µs — so the ADC side was parked ~90% of the time and the
datapath never ran back to back. Whatever the cause (startup transient, or the
fill side waiting on banks), the consequence is what matters: **an N=4 sim
passing is not evidence of sustained operation at 9 MSa/s.** Re-measure the
frame-start interval on the passing run before believing the rate; if it is not
~114 µs, the sim is running the datapath at a duty cycle the hardware will not
have.

This is not fixable by protocol compliance. The options are the ones §1d
already listed — more banks (deeper elastic buffer, which is what this branch
is about), a lower sample rate, or accepting dropped frames — plus one that
matters more now: a card qualified for the worst-case pause, not the average
rate. Whatever is chosen, the N=4 sim passing is not evidence about it: the
testbench ADC and the model card are both far away from the real operating
point, in opposite directions.

## §5 — CURRENT DESIGN: `SDCARD_PULSE`, capture first then stream

> Live, alongside §0. §0 describes the SD-side session mechanism; this
> section only changes *when* sessions are dispatched and how much SRAM one
> session covers. Nothing in §0 is invalidated.

`SDCARD_CONTINUOUS` (§0) buys unbounded capture length by overlapping
capture with streaming, and pays for it with a hard real-time constraint:
one `CMD25` session must complete within one bank fill, or the ADC catches
up with a bank the engine has not released and `target_frame_full` trips
`SDCARD_OVERFLOW`. §0b item 3 is the sharp end of that — at a spec-legal
25 MHz the 4-bit bus gives 12.5 MB/s against the ADC's 16 MB/s, so the mode
is only viable at all because `CMD6` high-speed puts the card at 25 MB/s.

`SDCARD_PULSE` makes the opposite trade. The ADC fills F0, rolls straight
on into F1, and stops; **only once both banks are full** does the copy
engine run, streaming both of them out as a *single* `CMD25` session of 8 ×
512 B blocks. Capture and streaming never overlap, so:

- **There is no throughput requirement at all.** Not "more slack" — none.
  The card can be arbitrarily slow; the pulse is already fully captured in
  SRAM before the first block goes out. High speed becomes a
  time-to-completion question rather than a correctness one.
- **The capture is bounded at two banks** (4 KiB = 2048 samples). That is
  the whole cost, and it is what makes the mode a *burst* rather than a
  stream.
- **Between pulses, samples are lost** while software re-arms. Inherent,
  not a defect: a pulse is an event capture.

### What is actually new in hardware

Very little, deliberately — the SD-side sequence is byte-for-byte the §0
one, including every fix §0a–§0d records.

- **Dispatch condition** (`adc_acquisition_top.sv`): `sdcard_frames_ready`
  is `F0_FULL && F1_FULL` in pulse mode instead of `F0_FULL || F1_FULL`.
- **Job width** (`adc_acquisition_sdcard_controller.sv`): new `copy_both_i`,
  latched at `start_i` like `copy_f0_i`. It widens `frame_words` to both
  frames' word counts added together, and makes `CE_DONE` release both
  banks.
- **Read-base switch**: a both-banks job walks F0 for `first_bank_words_q`
  words, then restarts from `F1_START_ADDR`. Explicitly, rather than
  relying on F1 abutting F0 in the address map (it does — Bank2/Bank3 are
  contiguous — but the two `Fx_*_ADDR` pairs stay independent registers, so
  the engine should not silently depend on their values lining up). On the
  wire it is one uninterrupted session: the card advances its own write
  pointer per block and cannot tell the switch happened.
- **Counter widths**: `frame_words`/`rd_idx`/`wr_idx` 11 → 12 bits (1024
  words per job, plus the usual bit of headroom), and
  `block_addr_advance_o`'s byte-mode concatenation retuned accordingly.
- **No frame budget.** `frames_started_q` and `SDCARD_FRAME_COUNT` are
  `SDCARD_CONTINUOUS`-only. Pulse mode's length is fixed at both banks, so
  it sets `capture_done_q` directly at the F1 boundary. `SDCARD_FRAME_COUNT`
  is *ignored* in this mode — `sw/test/test_adc_sdcard_pulse.c` sets it to 1
  (a value that would stop a continuous capture after F0) specifically to
  keep that true.

### Why a separate mode and not `SDCARD_CONTINUOUS` with N=2

They look adjacent — both fill exactly two banks — and they are not. Under
`SDCARD_CONTINUOUS` with `SDCARD_FRAME_COUNT=2` the engine dispatches F0 the
moment it fills, *while the ADC is still filling F1*, and writes two 4-block
sessions. That is a different thing on the card (two sessions, two
`AUTO_CMD12`s, two program cycles interleaved with capture) and it puts the
card back in the real-time path for the F0 session. The distinguishing
property of pulse mode is not the frame count, it is that **nothing is in
flight while the ADC is running**. Sharing the ping-pong frame-budget
machinery to express that would mean gating dispatch on a condition the
budget does not encode.

### Sequencing details worth knowing

- **`is_last_frame_i` needs no special case.** `capture_done_q` is set at
  the F1 boundary in the same cycle `F1_FULL` is raised — which is the same
  cycle the job first becomes dispatchable — so it is already high when the
  engine latches it. `SDCARD_DONE` therefore means what it always meant:
  the capture's last frame is physically committed.
- **Both banks are released only at `CE_DONE`**, not F0 when its words have
  been read. `Fx_FULL` keeps meaning "this bank holds a captured frame the
  consumer has not finished with", so an aborted session leaves both flags
  set for software to see.
- **`target_frame_full` cannot fire from reuse** here (each bank is written
  once). It is kept as the guard against *starting* a pulse on top of a
  bank whose `Fx_FULL` software never cleared.
- **Re-arm is software's job**: `MODE` auto-reverts to `IDLE` on
  `sdcard_done_set || sdcard_overflow_set` (that rule now covers both SD
  modes), and `RESET_WRITE_HEAD` is what clears `capture_done_q`. See
  `sw/sdcard_acquisition_pulse.c`'s loop. `SDCARD_BLOCK_ADDR` is
  deliberately not reset between pulses, so consecutive pulses land back to
  back on the card.

### Not yet verified

Written and reviewed, not simulated (no toolchain in the authoring
environment). What to look at first in a run:

- `test_adc_sdcard_pulse.c` end to end — its CMD17 readback of all 8 blocks
  against SRAM is the real check on the F0→F1 base switch: a switch off by
  one word misaligns everything from block 4 on, a switch that never
  happened writes F0 twice.
- That exactly *one* `CMD25` goes out per pulse (not two), with
  `BLOCK_COUNT=8`, and that `SDCARD_BLOCK_ADDR` advances once per pulse.
  `scripts/analyze_sdcard_wave.py` prints per-job timelines and the card's
  decoded commands.
- `NUM_PULSES > 1` in `sw/sdcard_acquisition_pulse.c` for the re-arm path —
  the one sequence with no prior art here, since every earlier SD flow ran
  a capture exactly once per program.

**One number to revisit on real silicon: `POLL_TIMEOUT`.** It is left
unchanged at 25M cycles (250 ms at 100 MHz), but note what pulse mode does
to the wait it bounds. `CE_WAIT_TRANSFER_COMPLETE`'s counter is not reset
per block — it spans the *whole* session including every inter-block
program cycle — and a pulse session is 8 blocks where the continuous mode's
is 4. If the SD spec's 250 ms is read as per-block rather than per-write,
even the 4-block session was already under-provisioned against a
pathological card; pulse mode doubles that exposure. Not changed here
because it is not specific to this mode and the failure it would cause
(a healthy slow transfer aborting as `SDCARD_OVERFLOW`) has not been
observed — but it is the first thing to suspect if a real card reports
overflow on a transfer that visibly completed.

## §1 — SUPERSEDED (see §0): multi-block `CMD25` abandoned in favor of per-block `CMD24`

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

**Caveat carried forward — `POLL_TIMEOUT`.** ~~500k cycles (~5 ms at
100 MHz) is comfortable against this model's ~500-cycle busy, but a real
card may legitimately take ~250 ms for a single-block write. This is now
the one wait whose duration is set by flash programming rather than bus
turnaround — raise `POLL_TIMEOUT` before running against real silicon.~~
**Resolved** in §0b: `POLL_TIMEOUT` is now 25M cycles (250 ms at 100 MHz)
and the counter widened to 25 bits.

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
the SD modes. The inlined ones use `dma_push` directly, which is already
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
number, `CMD25` argument used as-is, advances by `SDCARD_BLOCK_COUNT`);
`0` = byte addressing (standard-capacity-style — argument still used
as-is, advances by this frame's byte count, derived from
`frame_words_q`). `SDCARD_BLOCK_ADDR` itself already existed but was
previously inert (an informational counter only) — it's now the actual
`CMD25` argument.

Updated for §0: the advance is per *session*, not per block. Within a
`CMD25` session the card advances its own write pointer, so only the
session's starting address is ever sent.

## §2 — Mode collapse: `ACQ_SDCARD`

> Historical. `ACQ_SDCARD` was later renamed `SDCARD_CONTINUOUS` when
> `SDCARD_PULSE` joined it (§5); the names below are the ones in use at the
> time.

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
  bit~~ — moot either way: §1's `CMD24` pivot achieved per-block
  completion without touching the `sdhci` IP, and §0 no longer needs
  per-block completion at all.
- Chunked sessions (close/reopen `CMD25` every K blocks): reopened by §0.
  The durability checkpoint is now one per *session* (one bank), not one
  per block as under §1's `CMD24`. If a shorter checkpoint interval is
  ever wanted, it is a `SDCARD_BLOCK_COUNT` reduction plus more sessions
  per bank — not a new mechanism.
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

### Step 4 — Verification (written for the `CMD24` design, §1/§1b; see §0 for what the `CMD25` session changes)
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
