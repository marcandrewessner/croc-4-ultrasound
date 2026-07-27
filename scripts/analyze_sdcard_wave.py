#!/usr/bin/env python3
"""Correlate the ADC->SDCard copy engine against the testbench SD card model.

Both sides of the SD protocol live in the same waveform, so the useful view is
a single merged timeline: what the copy engine (`i_sdcard_ctrl`) thought it was
doing, next to what `sdModel.v` was actually doing internally. That is what
made the "host thinks the block is done, card is still programming it" class of
bug visible -- see rtl/adc_acquisition/DATAPATH.md sections 1c and 1d.

Usage:
    .venv/bin/python scripts/analyze_sdcard_wave.py [verilator/croc.fst]

Needs pywellen (reads FST directly, no VCD conversion):
    uv pip install --python .venv pywellen
"""

import sys

import pywellen

CLK_PS = 10_000  # 100 MHz core clock

CTRL = "tb_croc_soc.i_croc_soc.i_croc.i_adc_acquisition_top.i_sdcard_ctrl."
TOP = "tb_croc_soc.i_croc_soc.i_croc.i_adc_acquisition_top."
SDH = "tb_croc_soc.i_croc_soc.i_croc.i_sdhci_top_obi.i_sdhci_impl."
MOD = "tb_croc_soc.i_sd_card.i_model."

# sdModel.v `ifdef SYSTEMVERILOG branch (the one this build compiles) --
# fallback only; enum names are read out of the FST when it carries them.
DATA_STATES = ["DATA_IDLE", "READ_WAITS", "READ_DATA", "WRITE_FLASH", "WRITE_DATA"]

# CardStatus[12:9], SD spec Table 4-35. Not an enum in the model, so hardcoded.
CARD_STATES = {0: "IDLE", 1: "READY", 2: "IDENT", 3: "STBY", 4: "TRAN",
               5: "DATA", 6: "RCV", 7: "PRG", 8: "DIS"}


class Wave:
    def __init__(self, path):
        self.w = pywellen.Waveform(path)
        self._vars = {v.full_name: v for v in self.w.all_vars()}

    def _var(self, name):
        try:
            return self._vars[name]
        except KeyError:
            sys.exit(f"signal not in waveform: {name}\n"
                     f"(was the design rebuilt with tracing on?)")

    def tr(self, name):
        """[(time_ps, value)] -- transitions only, VCD-style."""
        return list(self._var(name).tv)

    def enum(self, name, fallback=None):
        """Value -> label map, read from the FST's own enum metadata.

        Verilator emits the `typedef enum` names into the FST, so state
        decoding survives someone adding a state without this script having to
        be kept in sync -- which matters, since that is exactly what happened
        when CE_WAIT_CARD_READY was added mid-debug.
        """
        et = self._var(name).enum_type
        if et:
            return {int(bits, 2): label for bits, label in et[1]}
        if fallback is None:
            sys.exit(f"no enum metadata for {name} and no fallback given")
        return dict(enumerate(fallback))


def at(trs, t):
    """Value of a transition list at time t (last change <= t)."""
    lo, hi, best = 0, len(trs) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if trs[mid][0] <= t:
            best = trs[mid][1]
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def cyc(t):
    return t // CLK_PS


def card_state(cs, t):
    return CARD_STATES.get((int(at(cs, t) or 0) >> 9) & 0xF, "?")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "verilator/croc.fst"
    w = Wave(path)

    ce_names = w.enum(CTRL + "state_q")
    ce_val = {v: k for k, v in ce_names.items()}
    data_names = w.enum(MOD + "dataState", DATA_STATES)

    ce = w.tr(CTRL + "state_q")
    arg = w.tr(CTRL + "cmd24_argument")
    blk = w.tr(TOP + "reg2hw.SDCARD_BLOCK_ADDR.BLOCK_ADDR.value")
    dat = w.tr(SDH + "i_dat_wrap.dat_i")  # == sd_dat_i == PRESENT_STATE[23:20]
    tc = w.tr(SDH + "i_dat_wrap.reg2hw_i.normal_interrupt_status.transfer_complete.q")
    cs = w.tr(MOD + "CardStatus")
    ds = w.tr(MOD + "dataState")
    ba = w.tr(MOD + "BlockAddr")
    fwc = w.tr(MOD + "flash_write_cnt")
    incmd = w.tr(MOD + "inCmd")

    idle = ce_val["CE_IDLE"]
    submit = ce_val["CE_SUBMIT_CMD24"]
    done = ce_val["CE_DONE"]
    overflow = ce_val["CE_OVERFLOW"]

    # ---------------------------------------------------------------- jobs
    jobs, prev = [], None
    for t, v in ce:
        v = int(v)
        if prev == idle and v != idle:
            jobs.append({"start": t})
        if jobs:
            if v == submit:
                jobs[-1].setdefault("cmd24", t)
            elif v == done:
                jobs[-1]["done"] = t
            elif v == overflow:
                jobs[-1]["overflow"] = t
        prev = v

    print("=" * 76)
    print("1. Copy-engine jobs, each against the card model's state at that instant")
    print("=" * 76)
    for i, j in enumerate(jobs, 1):
        print(f"\n--- job {i} ---")
        print(f"  start           cyc {cyc(j['start']):>8}")
        if "cmd24" in j:
            t = j["cmd24"]
            print(f"  CMD24 issued    cyc {cyc(t):>8}  arg={at(arg, t)}  "
                  f"(SDCARD_BLOCK_ADDR={at(blk, t)})")
            print(f"     card: {card_state(cs, t):<5} "
                  f"dataState={data_names[int(at(ds, t))]:<11} "
                  f"flash_write_cnt={at(fwc, t)}  BlockAddr={at(ba, t)}")
            if card_state(cs, t) != "TRAN":
                print("     >> card NOT in TRAN: sdModel.v's `24:` handler will "
                      "silently drop this command")
        for k in ("done", "overflow"):
            if k in j:
                t = j[k]
                print(f"  {k:<15} cyc {cyc(t):>8}  card={card_state(cs, t)} "
                      f"DAT0={int(at(dat, t)) & 1}")
                if k == "done" and card_state(cs, t) == "PRG":
                    print("     >> job completed while the card was still "
                          "programming -- block not committed yet")

    # ----------------------------------------------- card-side ground truth
    if jobs:
        t0 = jobs[0]["start"]
        t1 = (jobs[-1].get("done") or jobs[-1].get("overflow") or ce[-1][0]) + 2000 * CLK_PS
    else:
        t0, t1 = 0, ce[-1][0]

    print()
    print("=" * 76)
    print("2. Card-side ground truth: every WRITE_FLASH episode + BlockAddr move")
    print("=" * 76)
    events, last = [], {}
    for t, v in ds:
        if t0 <= t <= t1:
            events.append((t, "dataState", data_names[int(v)]))
    for t, v in cs:
        if t0 <= t <= t1:
            events.append((t, "CardState", card_state(cs, t)))
    for t, v in ba:
        if t0 <= t <= t1:
            events.append((t, "BlockAddr", int(v)))
    for t, key, val in sorted(events):
        if last.get(key) == val:  # CardStatus has bits other than the state field
            continue
        last[key] = val
        print(f"  cyc {cyc(t):>8}  {key:<10} = {val}")

    flashes = sum(1 for t, v in ds
                  if t0 <= t <= t1 and data_names[int(v)] == "WRITE_FLASH")
    print(f"\n  WRITE_FLASH episodes: {flashes}   (expected: one per job, "
          f"{len(jobs)} job(s) ran)")
    if flashes < len(jobs):
        print("  >> fewer commits than jobs: some block(s) never reached flash")

    # ------------------------------------------------- CMD24s the card saw
    print()
    print("=" * 76)
    print("3. CMD24s as decoded from the model's inCmd shift register")
    print("=" * 76)
    seen = set()
    for t, v in incmd:
        if t < t0:
            continue
        v = int(v)
        if (v >> 40) & 0x3F != 24:
            continue
        a = (v >> 8) & 0xFFFFFFFF
        st = card_state(cs, t)
        if (a, st) in seen:  # inCmd shifts, so the same command matches repeatedly
            continue
        seen.add((a, st))
        print(f"  cyc {cyc(t):>8}  arg=0x{a:08x} ({a})  card={st}  "
              f"free_wr_buf={(int(at(cs, t)) >> 8) & 1}")
        if st != "TRAN":
            print("      >> DROPPED (handler requires TRAN; else branch returns an "
                  "empty response with no error bit, so the host never notices)")

    # --------------------------------------- DAT0 busy vs TRANSFER_COMPLETE
    print()
    print("=" * 76)
    print("4. DAT0 busy signalling vs TRANSFER_COMPLETE")
    print("=" * 76)
    print("   CE_WAIT_CARD_READY polls PRESENT_STATE bit 20 (== DAT0 level) to")
    print("   wait out the card's program cycle. That is only race-free if DAT0")
    print("   is already low when TRANSFER_COMPLETE rises:")
    for t, v in tc:
        if not (t0 <= t <= t1 and int(v) == 1):
            continue
        d0 = int(at(dat, t)) & 1
        st = card_state(cs, t)
        if d0 == 0:
            verdict = "OK -- busy already asserted, the poll cannot miss it"
        elif st == "PRG":
            verdict = "RACE -- card is programming but has not pulled DAT0 low yet"
        else:
            # Not a race: a card that never enters PRG has nothing to wait for,
            # so CE_WAIT_CARD_READY falls straight through. Note this is also
            # what a *dropped* CMD24 looks like -- cross-check against section 3.
            verdict = "card never went busy for this block (see section 3)"
        print(f"   cyc {cyc(t):>8}  TRANSFER_COMPLETE^  DAT0={d0}  "
              f"card={st:<5}  {verdict}")


if __name__ == "__main__":
    main()
