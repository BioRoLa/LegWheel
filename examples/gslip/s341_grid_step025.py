"""Log s341 (modelling check): the full Stage 2a turning-envelope grid at
landing-angle (beta) step 0.25 deg -- the step the shipped v070 template
exporter uses -- instead of the 0.5 deg step of the s84 cache of record.

What this script does:
  - Solves exactly the s84 grid: the v_td and lambda sets and the law list
    are READ FROM the paper's vendored s84 cache
    (corgi-abad-icra2027/figures/stage2a_grid.npz, 21 v_td x 13 lambda x 2
    laws = 546 cells), with the stage script's own run_grid at step 0.25.
  - One task = one run_grid call for one (step, law, v_td) over all 13
    lambdas in grid order. run_grid's beta warm start is keyed per law and
    speed and flows only from lower to higher lambda, so every cell sees the
    warm-start chain it would see in a single full-grid run.
  - Control (--solve control): two tasks re-solved at step 0.5 and compared
    field by field with the s84 cache (and, for the geometric law, with
    stage2a_grid_s336.npz, the s336 re-solve on the corrected rolling radius,
    which the current code carries).
  - Reuse of s340's step-0.25 records (stage2a_lowspeed_step025.jsonl, v_td
    0.50-0.75 x lambda 0-5 deg): only for speeds whose 7 s340 cells ALL have
    no fixed point. Then run_grid's warm start is still empty at lambda 7.5,
    so s340's 7 cells + a fresh run_grid over the remaining 6 lambdas is the
    same chain as a fresh 13-lambda solve. The reuse is refused until the
    VERIFY tasks (fresh 13-lambda solves at speeds s340 also solved, both
    laws, with and without fixed points) are in the jsonl and their first 7
    cells equal s340's cells field for field.
  - Tasks run in a multiprocessing pool; each finished task is appended to
    stage2a_figs/stage2a_grid_step025.jsonl and a rerun skips tasks already
    there (resume).
  - --check prints the control and the reuse verification.
  - --finalize writes stage2a_figs/stage2a_grid_step025.npz (cells in the s84
    cache's order and key set, plus c4_lever = "profile" as every post-s335
    cell carries; 'gates' as the stage script records them; the step-0.5
    control cells) and s341_grid_step025.out.txt: control, reuse
    verification, per-law existence / beta* / field changes against s84
    (and against s336 for the geometric law), lost cells, and warm-start
    diagnostics.

Nothing here changes a number of record, a gate, or a default: the stage
script is imported, not edited, and the paper's regate module is only
imported (main not run) for abad_hold_profile.

    # in WSL, from the LegWheel root; at most 4 workers (the machine is shared)
    .venv/bin/python examples/gslip/s341_grid_step025.py --solve control,verify --workers 4
    .venv/bin/python examples/gslip/s341_grid_step025.py --check
    .venv/bin/python examples/gslip/s341_grid_step025.py --solve rest --workers 4
    .venv/bin/python examples/gslip/s341_grid_step025.py --finalize
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")   # one core per pool worker

import argparse                                  # noqa: E402
import json                                      # noqa: E402
import math                                      # noqa: E402
import runpy                                     # noqa: E402
import subprocess                                # noqa: E402
import sys                                       # noqa: E402
import time                                      # noqa: E402
from collections import Counter                  # noqa: E402
from multiprocessing import Pool                 # noqa: E402
from pathlib import Path                         # noqa: E402

import numpy as np                               # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stage2a_turning_envelope as env           # noqa: E402

# LegWheel/examples/gslip -> parents[2] is the directory holding both repos
PAPER_FIGS = HERE.parents[2] / "corgi-abad-icra2027" / "figures" / "make_stage2a_figs.py"
CACHE_S84 = PAPER_FIGS.parent / "stage2a_grid.npz"      # byte-identical to LegWheel's s84 cache (log s339)
CACHE_S336 = env.FIG_DIR / "stage2a_grid_s336.npz"
S340_JSONL = env.FIG_DIR / "stage2a_lowspeed_step025.jsonl"
JSONL = env.FIG_DIR / "stage2a_grid_step025.jsonl"
NPZ_OUT = env.FIG_DIR / "stage2a_grid_step025.npz"
OUT_TXT = HERE / "s341_grid_step025.out.txt"

STEP = 0.25
CONTROL_STEP = 0.5


def _cache_grid():
    with np.load(CACHE_S84, allow_pickle=True) as z:
        cells = list(z["cells"])
    laws = tuple(dict.fromkeys(c["law"] for c in cells))            # cache order
    v = np.array(sorted({float(c["v_td"]) for c in cells}))
    lam = np.array(sorted({float(c["lam_deg"]) for c in cells}))
    assert len(cells) == len(laws) * len(v) * len(lam), (len(cells), laws, v, lam)
    order = [(c["law"], float(c["v_td"]), float(c["lam_deg"])) for c in cells]
    assert order == [(law, vv, ll) for law in laws for vv in v for ll in lam], "cache order"
    return cells, laws, v, lam


S84_CELLS, LAWS, V, LAM = _cache_grid()
N_LAM = len(LAM)


def i_v_of(v_td: float) -> int:
    i = int(np.argmin(np.abs(V - v_td)))
    assert abs(V[i] - v_td) < 1e-9, v_td
    return i


TASKS = [(STEP, law, i) for law in LAWS for i in range(len(V))]
CONTROL = [(CONTROL_STEP, "empirical", i_v_of(0.75)),    # loses existence at lam 10: fallback path
           (CONTROL_STEP, "geometric", i_v_of(1.45))]    # s84 and s336 beta* differ here
VERIFY = [(STEP, "empirical", i_v_of(0.60)), (STEP, "geometric", i_v_of(0.55)),   # s340: no FP
          (STEP, "empirical", i_v_of(0.70)), (STEP, "geometric", i_v_of(0.70)),   # s340: FP
          (STEP, "empirical", i_v_of(0.75)), (STEP, "geometric", i_v_of(0.75))]
ALL_TASKS = TASKS + CONTROL


def _jsonable(c: dict) -> dict:
    out = {}
    for k, val in c.items():
        if isinstance(val, np.bool_):
            val = bool(val)
        elif isinstance(val, np.floating):
            val = float(val)
        elif isinstance(val, np.integer):
            val = int(val)
        out[k] = val
    return out


# --- s340 reuse -----------------------------------------------------------------

def load_s340() -> dict:
    """(law, i_v) -> s340 step-0.25 record (7 lambdas, the prefix of LAM)."""
    out = {}
    if not S340_JSONL.exists():
        return out
    for line in S340_JSONL.read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if r["step"] != STEP:
            continue
        lam = [float(c["lam_deg"]) for c in r["cells"]]
        assert lam == [float(x) for x in LAM[:len(lam)]], lam
        assert all(abs(c["v_td"] - V[r["i_v"]]) < 1e-12 for c in r["cells"])
        out[(r["law"], r["i_v"])] = r
    return out


def reuse_candidates(s340: dict) -> list:
    """Step-0.25 tasks that may be built from s340's prefix: every prefix cell
    without a fixed point (so the warm start is empty at the first new lambda),
    and not a VERIFY task."""
    return [(STEP, law, i) for (law, i), r in sorted(s340.items())
            if not any(c["exists"] for c in r["cells"]) and (STEP, law, i) not in VERIFY]


def _same(a, b) -> bool:
    if isinstance(a, float) and isinstance(b, float):
        return a == b or (math.isnan(a) and math.isnan(b))
    return a == b


def verify_reuse(runs: dict, s340: dict) -> tuple[bool, list[str]]:
    lines, ok = [], True
    for t in VERIFY:
        _, law, i = t
        if t not in runs:
            lines.append(f"  {law[:3]} v_td {V[i]:.2f}: fresh solve not in jsonl yet")
            ok = False
            continue
        if runs[t].get("source") != "fresh":
            lines.append(f"  {law[:3]} v_td {V[i]:.2f}: record is not a fresh solve")
            ok = False
            continue
        new = runs[t]["cells"][:len(s340[(law, i)]["cells"])]
        old = s340[(law, i)]["cells"]
        bad = []
        for cn, co in zip(new, old):
            keys = set(cn) | set(co)
            diff = sorted(k for k in keys if k not in cn or k not in co or not _same(cn[k], co[k]))
            if diff:
                bad.append((co["lam_deg"], diff))
        n_ex = sum(c["exists"] for c in old)
        lines.append(f"  {law[:3]} v_td {V[i]:.2f} ({n_ex}/{len(old)} s340 cells exist): "
                     f"{len(old)} cells compared on every field, "
                     f"{'IDENTICAL' if not bad else 'DIFFER ' + str(bad)}")
        ok = ok and not bad
    return ok, lines


# --- solve ----------------------------------------------------------------------

def solve_task(item):
    step, law, i_v, mode, prefix = item
    t0 = time.time()
    lam = LAM if mode == "fresh" else LAM[len(prefix):]
    cells = env.run_grid(V[i_v:i_v + 1], lam, step, laws=(law,), verbose=False)
    cells = list(prefix) + [_jsonable(c) for c in cells]
    assert [float(c["lam_deg"]) for c in cells] == [float(x) for x in LAM]
    return {"step": step, "law": law, "i_v": i_v, "v_td": float(V[i_v]),
            "source": "fresh" if mode == "fresh" else "s340 prefix (lam 0-5) + fresh suffix",
            "seconds": time.time() - t0, "t_end": time.time(), "cells": cells}


def load_runs() -> dict:
    runs = {}
    if JSONL.exists():
        for line in JSONL.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                runs[(r["step"], r["law"], r["i_v"])] = r
    return runs


def parse_tasks(spec: str) -> list:
    s340 = load_s340()
    out = []
    for part in spec.split(","):
        if part == "control":
            out.extend(CONTROL)
        elif part == "verify":
            out.extend(VERIFY)
        elif part == "rest":
            out.extend(t for t in TASKS if t not in VERIFY)
        elif "-" in part:
            a, b = part.split("-")
            out.extend(ALL_TASKS[k] for k in range(int(a), int(b) + 1))
        else:
            out.append(ALL_TASKS[int(part)])
    return list(dict.fromkeys(out))


def solve(tasks: list, workers: int) -> None:
    assert workers <= 4, "at most 4 workers: the machine is shared"
    done = load_runs()
    todo = [t for t in tasks if t not in done]
    s340 = load_s340()
    reuse = set(reuse_candidates(s340))
    items = []
    for t in todo:
        if t in reuse:
            ok, lines = verify_reuse(done, s340)
            if not ok:
                print("\n".join(lines))
                raise SystemExit(f"refusing s340 reuse for {t}: verification incomplete or failed "
                                 "(run --solve verify first)")
            items.append((*t, "reuse", s340[(t[1], t[2])]["cells"]))
        else:
            items.append((*t, "fresh", []))
    # longest first (no fixed point expected -> full beta scans), so the pool's
    # long tasks start early
    s84_ex = {(c["law"], round(float(c["v_td"]), 3)) for c in S84_CELLS if c["exists"]}
    items.sort(key=lambda it: ((it[1], round(float(V[it[2]]), 3)) in s84_ex, it[3] == "reuse"))
    print(f"{len(tasks)} tasks requested, {len(todo)} not yet in {JSONL.name} "
          f"({sum(it[3] == 'reuse' for it in items)} from the s340 prefix); {workers} workers; "
          f"start {time.strftime('%Y-%m-%d %H:%M:%S')}", flush=True)
    if not items:
        return
    t0 = time.time()
    with Pool(processes=min(workers, len(items))) as pool, JSONL.open("a") as fh:
        for r in pool.imap_unordered(solve_task, items):
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            n_ex = sum(c["exists"] for c in r["cells"])
            print(f"  step {r['step']:4.2f} {r['law'][:3]} v_td {r['v_td']:.2f} [{r['source'][:5]}]: "
                  f"{n_ex}/{len(r['cells'])} cells exist, {r['seconds']:.0f} s "
                  f"(wall {time.time() - t0:.0f} s)", flush=True)
    print(f"done {time.strftime('%Y-%m-%d %H:%M:%S')}, wall {time.time() - t0:.0f} s", flush=True)


# --- comparison helpers ------------------------------------------------------------

def load_paper_figs() -> dict:
    return runpy.run_path(str(PAPER_FIGS), run_name="s341_import")   # not __main__


def key(c) -> tuple:
    return (c["law"], round(float(c["v_td"]), 3), float(c["lam_deg"]))


FIELDS = ("beta_deg", "alpha_deg", "v_fwd", "tau_leg", "tau_abad_profile", "psi", "R",
          "duty", "slope", "flight_s", "scrub", "lam_out_deg")


def with_profile(c, fig):
    c = dict(c)
    if c["exists"]:
        c["tau_abad_profile"] = float(fig["abad_hold_profile"](c))
    return c


def compare(new: dict, old: dict, keys, label: str, say, detail=True) -> dict:
    """Existence, beta* shifts, and max |d| on cells existing in both."""
    ex_gain = [k for k in keys if new[k]["exists"] and not old[k]["exists"]]
    ex_lost = [k for k in keys if old[k]["exists"] and not new[k]["exists"]]
    both = [k for k in keys if new[k]["exists"] and old[k]["exists"]]
    shift = {k: round(float(new[k]["beta_deg"]) - float(old[k]["beta_deg"]), 6) for k in both}
    moved = [k for k in both if shift[k] != 0.0]
    say(f"  [{label}] cells {len(keys)}: exist new {sum(new[k]['exists'] for k in keys)}, "
        f"old {sum(old[k]['exists'] for k in keys)}; existence gained {len(ex_gain)}, LOST {len(ex_lost)}")
    for tag, lst in (("gained", ex_gain), ("LOST", ex_lost)):
        for k in lst:
            c = new[k] if tag == "gained" else old[k]
            say(f"      {tag}: {k[0][:3]} v_td {k[1]:.2f} lam {k[2]:4.1f} "
                f"(beta {c['beta_deg']:.2f}, alpha {c['alpha_deg']:.2f}, v_fwd {c['v_fwd']:.4f})")
    hist = Counter(shift[k] for k in moved)
    say(f"    beta* changed in {len(moved)} of {len(both)} cells existing in both; "
        f"shift histogram (new - old, deg): "
        f"{dict(sorted(hist.items())) if hist else '{}'}")
    res = {"gain": ex_gain, "lost": ex_lost, "both": both, "moved": moved, "hist": hist}
    for subset_name, subset in (("all both-exist", both),
                                ("beta* unchanged", [k for k in both if shift[k] == 0.0]),
                                ("beta* changed", moved)):
        if not subset:
            continue
        parts = []
        for f in FIELDS:
            ds = []
            for k in subset:
                a, b = new[k].get(f), old[k].get(f)
                if a is None or b is None:
                    continue
                a, b = float(a), float(b)
                if math.isinf(a) and math.isinf(b):
                    continue          # R at lambda = 0
                ds.append((abs(a - b), k))
            if ds:
                d, kk = max(ds)
                parts.append(f"{f} {d:.4g}" + (f" @{kk[0][:3]} {kk[1]:.2f}/{kk[2]:g}" if d > 0 else ""))
        say(f"    max |d| over {len(subset)} cells ({subset_name}): " + "; ".join(parts))
    if detail and moved:
        say("    beta*-changed cells:")
        for k in sorted(moved):
            a, b = new[k], old[k]
            say(f"      {k[0][:3]} v_td {k[1]:.2f} lam {k[2]:4.1f}: beta {b['beta_deg']:.2f} -> "
                f"{a['beta_deg']:.2f}, alpha {b['alpha_deg']:.2f} -> {a['alpha_deg']:.2f}, v_fwd "
                f"{b['v_fwd']:.4f} -> {a['v_fwd']:.4f}, tau_leg {b['tau_leg']:.2f} -> "
                f"{a['tau_leg']:.2f}, psi {b['psi']:.4f} -> {a['psi']:.4f}, R "
                f"{b['R']:.3f} -> {a['R']:.3f}, |slope| {abs(b['slope']):.4f} -> {abs(a['slope']):.4f}")
    return res


def warm_start_diagnostics(cells: list, step: float, say) -> dict:
    """Per (law, v_td) chain, reconstruct run_grid's warm-start centre (the
    last existing beta* at lower lambda) and flag: beta* on the +-3 deg window
    edge; beta* outside the window (only the cold fallback can return it);
    cells with a centre but no fixed point (fallback ran and failed);
    existence holes along lambda; beta* on the 60/86 scan bounds; alpha on
    its 1/45 bounds; beta* reversals along lambda."""
    flags = {"edge": [], "fallback_found": [], "fallback_failed": [], "holes": [],
             "beta_bound": [], "alpha_bound": [], "reversal": []}
    chains = {}
    for c in cells:
        chains.setdefault((c["law"], round(float(c["v_td"]), 3)), []).append(c)
    for (law, v), ch in sorted(chains.items()):
        ch = sorted(ch, key=lambda c: c["lam_deg"])
        center, prev_d = None, 0.0
        ex = [c["exists"] for c in ch]
        if True in ex:
            first, last = ex.index(True), len(ex) - 1 - ex[::-1].index(True)
            for j in range(first, last + 1):
                if not ex[j]:
                    flags["holes"].append((law, v, ch[j]["lam_deg"]))
        for c in ch:
            k = (law, v, c["lam_deg"])
            if not c["exists"]:
                if center is not None:
                    flags["fallback_failed"].append(k)
                continue
            b = float(c["beta_deg"])
            if center is not None:
                d = b - center
                lo, hi = max(60.0, center - 3.0), min(86.0, center + 3.0)
                if b < lo - 1e-9 or b > hi + 1e-9:
                    flags["fallback_found"].append((k, center, b))
                elif abs(b - lo) < 1e-9 or abs(b - hi) < 1e-9:
                    flags["edge"].append((k, center, b))
                if d != 0.0:
                    if prev_d != 0.0 and np.sign(d) != np.sign(prev_d):
                        flags["reversal"].append((k, center, b))
                    prev_d = d
            if b <= 60.0 + 1e-9 or b >= 86.0 - 1e-9:
                flags["beta_bound"].append((k, b))
            a = float(c["alpha_deg"])
            if a >= 45.0 - 0.1 or a <= 1.0 + 0.1:
                flags["alpha_bound"].append((k, a))
            center = b
    for name, lst in flags.items():
        say(f"    {name}: {len(lst)}" + ("" if not lst else "  " + "; ".join(
            _fmt_flag(x) for x in lst[:40]) + (" ..." if len(lst) > 40 else "")))
    return flags


def _fmt_flag(x) -> str:
    if isinstance(x[0], tuple):
        k = x[0]
        rest = " -> ".join(f"{v:.2f}" for v in x[1:])
        return f"{k[0][:3]} {k[1]:.2f}/{k[2]:g} ({rest})"
    return f"{x[0][:3]} {x[1]:.2f}/{x[2]:g}"


def _git(*args) -> str:
    try:
        return subprocess.run(["git", "-C", str(HERE), *args], capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except Exception as exc:
        return f"unavailable ({exc})"


def _restore_types(c: dict, template: dict) -> dict:
    """JSON round trip -> the s84 cache's per-field Python/numpy types."""
    out = {}
    for k, val in c.items():
        t = template.get(k)
        if val is None or t is None:
            out[k] = val
        elif isinstance(t, np.floating):
            out[k] = np.float64(val)
        elif isinstance(t, (bool, np.bool_)):
            out[k] = type(t)(val)
        elif isinstance(t, float):
            out[k] = float(val)
        else:
            out[k] = val
    return out


# --- check / finalize ------------------------------------------------------------------

def check(say, fig=None) -> None:
    fig = fig or load_paper_figs()
    runs = load_runs()
    s84 = {key(c): with_profile(c, fig) for c in S84_CELLS}
    with np.load(CACHE_S336, allow_pickle=True) as z:
        s336 = {key(c): with_profile(c, fig) for c in z["cells"]}
    say("--- control: step 0.5 on current code vs the caches ---")
    for t in CONTROL:
        if t not in runs:
            say(f"  {t}: not solved yet")
            continue
        r = runs[t]
        new = {key(c): with_profile(c, fig) for c in r["cells"]}
        keys = sorted(new)
        say(f"  {r['law']} v_td {r['v_td']:.2f} ({r['seconds']:.0f} s):")
        compare(new, s84, keys, "vs s84", say)
        if r["law"] == "geometric":
            compare(new, s336, keys, "vs s336 (corrected rolling radius)", say)
        ex_same = all(new[k]["exists"] == s84[k]["exists"] for k in keys)
        exact = ex_same and all(
            all(_same(float(new[k][f]), float(s84[k][f])) for f in
                ("beta_deg", "alpha_deg", "v_fwd", "tau_leg", "psi", "duty", "slope"))
            for k in keys if new[k]["exists"])
        say(f"    exact reproduction of s84 (exists, beta, alpha, v_fwd, tau_leg, psi, duty, slope): {exact}")
        if r["law"] == "geometric":
            exact336 = all(new[k]["exists"] == s336[k]["exists"] for k in keys) and all(
                all(_same(float(new[k][f]), float(s336[k][f])) for f in
                    ("beta_deg", "alpha_deg", "v_fwd", "tau_leg", "psi", "duty", "slope",
                     "tau_abad"))
                for k in keys if new[k]["exists"])
            say(f"    exact reproduction of s336 (same fields + raw tau_abad, both profile lever): {exact336}")
    say("\n--- s340 reuse verification (fresh step-0.25 solves vs s340's cells) ---")
    ok, lines = verify_reuse(runs, load_s340())
    for ln in lines:
        say(ln)
    say(f"  reuse allowed: {ok}; reuse candidates "
        f"{[(law[:3], round(float(V[i]), 2)) for _, law, i in reuse_candidates(load_s340())]}")


def finalize() -> None:
    lines: list[str] = []

    def say(s=""):
        print(s, flush=True)
        lines.append(s)

    env._selftest()
    say("stage2a_turning_envelope selftest: PASS")
    fig = load_paper_figs()
    say(f"paper module imported (run_name != __main__) from {PAPER_FIGS}")
    say(f"stage run_grid gates (verdicts as solved): {env.gate_constants()}")
    say(f"LegWheel HEAD {_git('rev-parse', '--short', 'HEAD')}")
    say(f"grid read from {CACHE_S84}: laws {LAWS}, {len(V)} v_td {V[0]:.2f}..{V[-1]:.2f}, "
        f"lambda {[float(x) for x in LAM]}")
    say(f"  equals the stage module's V_GRID_FULL / LAM_GRID_DEG: "
        f"{np.array_equal(V, env.V_GRID_FULL)} / {np.array_equal(LAM, env.LAM_GRID_DEG)}")

    runs = load_runs()
    missing = [t for t in ALL_TASKS if t not in runs]
    if missing:
        raise SystemExit(f"missing tasks (run --solve first): {missing}")

    # runtime
    recs = [runs[t] for t in ALL_TASKS]
    src = Counter(r["source"] for r in recs if r["step"] == STEP)
    say(f"\nruntime: step-0.25 tasks {len(TASKS)} ({dict(src)}); summed task seconds "
        f"{sum(runs[t]['seconds'] for t in TASKS):.0f} s (reused s340 prefix time not included), "
        f"task range {min(runs[t]['seconds'] for t in TASKS):.0f}-"
        f"{max(runs[t]['seconds'] for t in TASKS):.0f} s; control tasks "
        f"{[round(runs[t]['seconds']) for t in CONTROL]} s; first/last record end "
        f"{time.strftime('%m-%d %H:%M:%S', time.localtime(min(r['t_end'] for r in recs)))} / "
        f"{time.strftime('%m-%d %H:%M:%S', time.localtime(max(r['t_end'] for r in recs)))}")

    say("")
    check(say, fig)

    # assemble in cache order
    tmpl = next(c for c in S84_CELLS if c["exists"])
    tmpl_none = next(c for c in S84_CELLS if not c["exists"])
    cells = []
    for law in LAWS:
        for i in range(len(V)):
            for c in runs[(STEP, law, i)]["cells"]:
                c = _restore_types(c, tmpl if c["exists"] else tmpl_none)
                c["v_td"], c["lam_deg"] = np.float64(V[i]), np.float64(c["lam_deg"])
                cells.append(c)
    assert [key(c) for c in cells] == [key(c) for c in S84_CELLS]
    s84_keys = {k for c in S84_CELLS for k in c}
    extra = sorted({k for c in cells for k in c} - s84_keys)
    missing_keys = sorted(s84_keys - {k for c in cells for k in c})
    say(f"\nassembled {len(cells)} cells in the s84 order; keys beyond s84: {extra}; "
        f"s84 keys absent: {missing_keys}")
    say(f"run_grid verdicts at the stage gates: binding {dict(Counter(c['binding'] for c in cells))}")

    new = {key(c): with_profile(c, fig) for c in cells}
    s84 = {key(c): with_profile(c, fig) for c in S84_CELLS}
    with np.load(CACHE_S336, allow_pickle=True) as z:
        s336 = {key(c): with_profile(c, fig) for c in z["cells"]}

    say("\n=== step 0.25 vs the s84 cache (step 0.5), per law ===")
    say("(tau_abad_profile = the C4 demand at the profile contact, the paper's "
        "abad_hold_profile applied to both; R compared on lambda > 0)")
    summary = {}
    for law in LAWS:
        keys = sorted(k for k in new if k[0] == law)
        say(f"\n{law}:")
        summary[(law, "s84")] = compare(new, s84, keys, "step0.25 vs s84", say)
        if law == "geometric":
            say("  (the s84 geometric arm predates the s336 rolling-radius fix; s336 is the "
                "same step 0.5 on the current law, so this isolates the step)")
            summary[(law, "s336")] = compare(new, s336, keys, "step0.25 vs s336", say)
        say("  s84 vs s336 for reference (step 0.5 both):")
        compare(s336, s84, keys, "s336 vs s84", say, detail=False)

    say("\n=== warm-start diagnostics ===")
    for law in LAWS:
        for label, grid, step in (("step 0.25", [c for c in cells if c["law"] == law], STEP),
                                  ("s84 step 0.5", [c for c in S84_CELLS if c["law"] == law], 0.5)):
            say(f"  {law} {label}:")
            warm_start_diagnostics(grid, step, say)

    say("\n=== beta* along lambda per speed (step 0.25; '--' no fixed point; "
        "'|' marks a cell whose beta* differs from s84) ===")
    for law in LAWS:
        say(f"  {law}")
        for i, v in enumerate(V):
            row = []
            for lam in LAM:
                k = (law, round(float(v), 3), float(lam))
                c, o = new[k], s84[k]
                if not c["exists"]:
                    row.append("   --" + ("*" if o["exists"] else " "))
                else:
                    moved = (not o["exists"]) or float(o["beta_deg"]) != float(c["beta_deg"])
                    row.append(f"{c['beta_deg']:5.2f}" + ("|" if moved else " "))
            say(f"    {v:.2f} " + " ".join(row))
    say("  ('*' after -- marks a cell that exists in s84 but not at step 0.25)")

    meta = {"script": Path(__file__).name, "beta_step": STEP, "control_step": CONTROL_STEP,
            "grid_from": str(CACHE_S84), "v_td": [float(v) for v in V],
            "lam_deg": [float(x) for x in LAM], "laws": list(LAWS),
            "legwheel_head": _git("rev-parse", "HEAD"),
            "sources": dict(src),
            "note": "raw run_grid cells at beta step 0.25 in the s84 cache's order and schema "
                    "(+ c4_lever='profile'); verdicts at the stage gates in 'gates'. Cells of "
                    "v_td 0.50-0.65 with no s340 fixed point take lambda 0-5 from s340's "
                    "step-0.25 jsonl (verified identical to fresh solves; see the out.txt)."}
    np.savez_compressed(NPZ_OUT, cells=np.array(cells, dtype=object),
                        cells_step05_control=np.array(
                            [c for t in CONTROL for c in runs[t]["cells"]], dtype=object),
                        gates=np.array(env.gate_constants(), dtype=object),
                        meta=np.array(meta, dtype=object))
    say(f"\nwrote {NPZ_OUT}")
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_TXT}")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--solve", help="control, verify, rest, or task indices into "
                    f"TASKS+CONTROL (0-{len(TASKS) - 1} step 0.25, "
                    f"{len(TASKS)}-{len(ALL_TASKS) - 1} control), comma-separated")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--finalize", action="store_true")
    a = ap.parse_args(argv)
    if a.solve:
        solve(parse_tasks(a.solve), a.workers)
    if a.check:
        check(print)
    if a.finalize:
        finalize()
    if not (a.solve or a.check or a.finalize):
        ap.print_help()


if __name__ == "__main__":
    main(sys.argv[1:])
