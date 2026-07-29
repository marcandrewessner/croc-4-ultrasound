# ADC acquisition / SD-card subsystem: cleanup plan

## Why

`rtl/adc_acquisition/` and the software that drives it (`sw/sdcard_test.c`,
`sw/sdcard_acquisition_Nx.c`, `sw/sdcard_acquisition_pulse.c`,
`sw/lib/inc/sdhci_helpers.h`, `sw/lib/inc/adc_acquisition*.h`,
`sw/test/test_adc_sdcard*.c`) just went through several rounds of rapid,
organic growth: the original single-block `ACQ_SDCARD`/`CMD24` design, a
~650-line rework to `CMD25`/`AUTO_CMD12` multi-block streaming, a new
`SDCARD_PULSE` mode, an `ACQ_SDCARD`→`SDCARD_CONTINUOUS` rename, real-card
bring-up fixes (`ACMD6`, negotiated CCS addressing, `SIM_CARD` build switch),
and several rounds of hard-won hardware-validated bugfixes (see the "more
SRAM" merge commit and the SD debugging session before it). All of that is
now working on real hardware. But it accumulated the debris that kind of
history always leaves: stale comments, dead code, copy-pasted boilerplate
across five different `.c` files, register field names that no longer match
what they hold, and a 1100-line design-notes file that's become a
chronological lab notebook rather than a reference.

This is the plan for cleaning that up **without breaking anything that's
already validated on real hardware**. It's split into phases ordered by risk,
cheapest/safest first, each independently buildable and testable — nothing
here should be done as one big-bang rewrite.

## Ground rules for every phase

- **Every phase gets its own commit(s)**, buildable and testable in isolation.
  Never bundle a rename/removal with a behavior change in the same commit.
- **Regression ladder after every phase that touches RTL or `sdhci_helpers.h`**
  (cheapest/fastest first, stop and investigate on the first failure):
  1. `oseda -2026.04 make -C sw SIM_CARD=1 bin/<touched>.hex` + Verilator sim.
  2. `riscv64-unknown-elf-size` on every SD-touching binary against the
     3584-byte `link.ld` budget (this has already silently broken twice this
     project's history — once as garbled `printf` output, once as a hard
     link error. Don't skip this check).
  3. On real hardware: `sdcard_test.c` first (the most-validated, simplest
     regression sentinel), then `sdcard_acquisition_Nx.c` (`NUM_FRAMES=1`,
     `2`), then `sdcard_acquisition_pulse.c`.
- **Register renames (Phase 2) require a full regenerate-and-grep pass**,
  not hand-editing the generated `.sv`/`.h` files — grep for every macro/field
  name being renamed across `rtl/` and `sw/` before considering a rename done.
- If a phase turns out to be bigger or riskier than it looks once started,
  stop and re-scope rather than pushing through — this doc should be updated
  as reality corrects the plan.

## Phase 0 — Zero-risk fixes (do first, do together, one commit)

These are either genuinely dead code or comments that are factually wrong
about current behavior. None of them change what the hardware does.

- **Remove `sdh_switch_func()`** (`sw/lib/inc/sdhci_helpers.h`) — the CMD6
  `SWITCH_FUNC`/High-Speed helper. Confirmed orphaned: not called anywhere,
  already stripped from every linked binary by `--gc-sections`. Keeping it
  "for future work" is fine as an idea but dead code in a shared header
  invites exactly the kind of confusion this cleanup is trying to remove —
  if High Speed is revisited later, it can come back with the RTL changes
  that would actually be needed alongside it (see Phase 5).
- **Fix the stale `SIM_CARD` comment in `sw/Makefile`** — it says "the
  header's own default (`SIM_CARD=1`)" but the header's fallback default is
  actually `0` (already fixed in code, comment never caught up). Exactly the
  same class of bug as the already-fixed `SDCARD_ADDR_IS_BLOCKS` comment —
  worth noting because it means this merge left at least two of these, so
  treat Phase 0 as "grep for any other comment referencing a removed/changed
  name" rather than just this one instance.
- **Fix the FSM width comment** in `adc_acquisition_sdcard_controller.sv`
  (`ce_state_e`): says "6 bits: the session flow is 34 states, past what
  `[4:0]` holds" — the enum actually has 32 states, which fits exactly in
  `[4:0]` (0-31). Either correct the comment to say what's actually true, or
  fold into Phase 5 (shrink to 5 bits) if that's in scope.
- **Remove unused macros**: `ADC_ACQ_STATUS_F0_FULL_BIT` /
  `_F1_FULL_BIT` (`sw/lib/inc/adc_acquisition.h`) and `ADC_ACQ_SAMPLES_PER_WORD`
  (defined, never used anywhere — every call site still hand-unpacks via
  `lo14`/`hi14`). Either use `ADC_ACQ_SAMPLES_PER_WORD` where the unpacking
  happens or remove it; don't leave it decorative.
- **Investigate and remove `ADC_ACQUISITION_REG__ADDRESS_CONFIG_R__*`**
  (`sw/lib/inc/adc_acquisition_reg.h`) — corresponds to no field in the
  current `adc_acquisition_reg_t`, looks like a generator artifact from a
  renamed/removed register that never got cleanly regenerated. Confirm
  against the `.rdl` and regenerate rather than hand-deleting, in case it's a
  hint that the reg-gen step needs re-running project-wide (see Phase 2).
- **Trim the stale historical prose in `adc_acquisition_sdcard_controller.sv`**
  that still describes the retired per-block `CMD24` design in present tense
  ("job 2's own CMD24...", "job 2 issues its CMD24..." — check the current
  file for exact locations) — either mark clearly as "historical, compare to
  current CMD25 design" or move into `DATAPATH.md`'s history section
  (Phase 4), since live source comments describing removed behavior are easy
  to mistake for current behavior on a skim.

## Phase 1 — `sdcard_test.c` → `sdhci_helpers.h` consolidation

**This is the single biggest win and the single riskiest change**, so it's
its own phase, done carefully, alone.

`sdcard_test.c` is the one SD program that does *not* use
`sdhci_helpers.h` — it independently reimplements
`spin_until_clear8/set16/clear32`, its own `sdhci_cmd()` (nearly identical to
`sdh_cmd()`), and the entire bring-up sequence inline (~155 lines
duplicating `sdh_init()`'s ~140 lines). It's also the one binary sitting
at 94.6% of the SRAM budget (3392/3584 bytes) while every other SD program
that already shares the header sits at 700-870 bytes of headroom. The
duplication and the tight budget are the same problem.

**Why this is risky**: `sdcard_test.c` is the most deeply validated,
hardware-debugged file in this codebase — the ACMD6 fix, the 12.5MHz clock
speed, the memory-corruption/`link.ld` investigation all happened against
this exact file. Migrating it to `sdh_init()` must be a **pure refactor**:
same register writes, same order, same clock speed, same timeouts, verified
by diffing the two init sequences line-by-line before touching anything, not
just "looks similar."

Approach:
1. Diff `sdcard_test.c`'s inline sequence against `sdh_init()` field-by-field
   (register, value, order, timeout). Resolve any discrepancy as a deliberate
   decision, not an oversight, before writing code.
2. Switch `sdcard_test.c` to call `sdh_init()`. Its own `ok()`/`fail()`
   step-coded diagnostics can stay — they're `sdcard_test.c`'s own numbering,
   independent of `sdh_init()`'s internal `sdh_fail()` codes.
3. Full regression ladder (see Ground rules). This is the one place a size
   *increase* would be a red flag, not just a decrease — if it doesn't shrink
   substantially, the consolidation didn't actually happen.

## Phase 2 — Register field naming and description audit

Source-of-truth fix in `rtl/adc_acquisition/reg/adc_acquisition_reg_definition.rdl`,
then regenerate `adc_acquisition_reg_pkg.sv`, `adc_acquisition_reg.sv`,
`sw/lib/inc/adc_acquisition_reg.h`, and the `reg/doc/` output — don't
hand-edit the generated files.

- **`STATUS.F0_FULL`/`F1_FULL`** (field `fx_full_f`, display name "Block Fx
  Full"): rename to something frame-based (e.g. `frame_full_f` / "Frame Fx
  Full" / desc "High if the Fx frame (SRAM bank) was fully written"). This
  register file uses "block" everywhere else to mean the 512B SD block
  (`SDCARD_BLOCK_SIZE`, `SDCARD_BLOCK_COUNT`, `SDCARD_BLOCK_ADDR`) — the one
  field using "block" to mean "SRAM bank" is a real collision, not just a
  style nit.
- **`STATUS.SDCARD_OVERFLOW` description**: currently describes only the
  original ADC-side trip condition ("A frame became full while
  BUFFER_WRITE_READY was not set"). The same bit is now also set by
  `adc_acquisition_sdcard_controller.sv`'s `CE_OVERFLOW` state on `POLL_TIMEOUT`
  expiry (six different wait states), command-level `ERROR_INTERRUPT`, or a
  late data/CRC/`AUTO_CMD12` error — none of which involve
  `BUFFER_WRITE_READY`. Rewrite the description to cover both trigger
  classes, or split into two status bits if software needs to distinguish
  "ADC side dropped a frame" from "SD session itself failed" (worth deciding
  explicitly — right now they're indistinguishable from software's view,
  which may or may not be intended).
- While in the `.rdl`: sweep every other field's `desc` for the same kind of
  drift (a description written for the design at the time, not updated when
  behavior changed) — the two found above were located by targeted checking,
  not an exhaustive pass, so treat this as a floor, not a ceiling.
- Confirm/resolve the orphaned `ADDRESS_CONFIG_R` fields from Phase 0 here,
  since this is where the `.rdl` gets touched anyway.

## Phase 3 — Consolidate common SD-program boilerplate

Across `sdcard_acquisition_Nx.c`, `sdcard_acquisition_pulse.c`,
`test/test_adc_sdcard.c`, `test/test_adc_sdcard_pulse.c`: the same
terminal-status poll (`while (!(STATUS & (DONE|OVERFLOW))) { timeout-check }`,
4 near-identical instances), the same status-clear trailer (`CNTRL =
CLEAR_STATUS | CLR_F0_FULL | CLR_F1_FULL`, 3 instances), and the same
FAIL/WARN/OK status-decode `printf` block (2 instances, differing only in
"frames" vs "pulses" wording) are each copy-pasted with minor variable-name
differences.

- Extract the poll-and-decode pattern into `sdhci_helpers.h` (or a new
  small shared header if it doesn't belong next to the SDHCI-only helpers —
  decide based on whether it's SDHCI-generic or ADC-acquisition-specific;
  it's the latter, so it more likely belongs in `sw/lib/inc/adc_acquisition.h`
  instead) as one or two small functions, parameterized on the
  status/overflow bitmasks and the "frames"/"pulses" noun.
- Smaller win than Phase 1 (all four target binaries already have
  700+ bytes of headroom), but real: do it, just don't over-invest — check
  size before/after and confirm the win is real, not offset by function-call
  overhead eating the savings on such small functions.
- The two `test_adc_sdcard*.c` files' near-identical `check_progression()`
  and CMD17 readback-and-diff loop are candidates too, but lower priority
  (~45 lines total, test-only code, not shipped).

## Phase 4 — `DATAPATH.md` restructuring

1104 lines, non-monotonic section numbering (§5 appears before §1), and
roughly half of it (the "Implementation plan" at the end, plus §1/§1b-§1e)
is explicitly marked historical/superseded in the text but still there at
full length. This makes it hard to use as a "what does this subsystem do
right now" reference, which is what a file living next to the RTL should be.

- Split into two files: a **current-state reference** (what
  `SDCARD_CONTINUOUS`/`SDCARD_PULSE` actually do today, the register
  interface, the FSM, the real-card bring-up requirements that are still
  live — e.g. §0b's `ACMD6`/CMD6 material, which was checked and found *not*
  stale) and a **history/design-log** file (the retired `CMD24` design, the
  `ACQ_SDCARD`→`SDCARD_CONTINUOUS` collapse discussion, the implementation
  plan) for anyone who wants the archaeology later.
- Any move-worthy historical prose currently living in `.sv` file comments
  (Phase 0's last bullet) can land in the history file too, rather than
  needing a new home.
- Renumber sections monotonically in whichever file(s) result.

## Phase 5 — FSM/state-encoding cleanup (optional, lowest priority)

`ce_state_e` is `[5:0]` (6 bits) for 32 actual states, which fits `[4:0]`
(5 bits). Shrinking it is a one-line, mechanical, functionally-inert change
(same states, same transitions, one fewer bit of storage) — low value on
its own, but bundle it with Phase 0's comment fix rather than doing it as
a separate change, since fixing the comment without fixing the actual width
just relocates the staleness.

## Explicitly out of scope for this cleanup

- Anything about `SDCARD_CONTINUOUS`'s throughput ceiling (needs ≥16MB/s,
  current system clock caps the SD bus at 12.5MB/s) — that's a system-clock
  or protocol-capability question, not an architecture/naming cleanup.
- Re-adding CMD6 High-Speed negotiation — `sdh_switch_func()` is being
  removed as dead code (Phase 0), not preserved as a stub; if High Speed
  comes back, it comes back as new, deliberate work informed by the actual
  clock-generator constraint, not by resurrecting orphaned code.
