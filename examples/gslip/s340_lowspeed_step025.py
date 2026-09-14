"""Log s340 (modelling check): does a finer landing-angle step find
torque-feasible CAMBERED turns below the Stage 2a grid's 0.53 m/s edge?

Background (log s338 s3): at lambda = 0, v_td 0.70, solve_existence finds no
fixed point at beta step 0.5 but finds beta 84.25, alpha 44.11, v_fwd 0.497 at
step 0.25, inside the production window (alpha <= 45, beta <= 86). s338 s8:
no cambered cell was re-solved and no torque gate was applied to that point.

What this script does:
  - Re-solves the low-speed block of the Stage 2a grid, v_td = V_GRID_FULL[1:7]
    (0.50 ... 0.75, the grid's own floats) x lambda = LAM_GRID_DEG[:7]
    (0, 0.5, 1, 1.5, 2, 2.5, 5 deg) x both radius laws, with the stage script's
    own run_grid, at beta step 0.25 (the check) and 0.5 (a control on current
    code, because the cache of record is the s84 grid and the geometric
    law's rolling_radius has changed since, log s336).
  - One task = one run_grid call for one (step, law, v_td) over the 7 lambdas
    in grid order. run_grid's beta warm start is per speed and flows only
    from lower to higher lambda, and these 7 lambdas are the prefix of
    LAM_GRID_DEG, so every cell sees the same warm-start chain it would see
    in a full-grid run at that step.
  - Tasks run in a multiprocessing pool; each finished task is appended to a
    jsonl, and a rerun skips tasks already there (resume).
  - --finalize regates every cell exactly as the paper does, by importing
    regate / _rmin_curve and the gate constants from
    corgi-abad-icra2027/figures/make_stage2a_figs.py (its main is not run):
    C2 psi <= 0.29 rad/s, C3 eroded tau_leg <= 29.5 (6:1) and <= 44.25 (9:1),
    C4 at the profile contact <= 29.5 via abad_hold_profile. It compares with
    the paper's vendored s84 cache figures/stage2a_grid.npz and writes
    stage2a_figs/stage2a_lowspeed_step025.npz and s340_lowspeed_step025.out.txt.

Nothing here changes a number of record, a gate, or a default: the stage
script is imported, not edited.

    # in WSL, from the LegWheel root. The resume unit is one task, and a task
    # at a speed with no fixed point scans every beta for all 7 lambdas: on
    # 8 cores (2026-09-14) such tasks took 780-850 s at step 0.25 in the
    # 8-task chunk below and up to 1300 s in the 16-task chunk (8 workers
    # each), so run chunks in the background.
    .venv/bin/python examples/gslip/s340_lowspeed_step025.py --solve 0-7
    .venv/bin/python examples/gslip/s340_lowspeed_step025.py --solve 8-23
    .venv/bin/python examples/gslip/s340_lowspeed_step025.py --finalize
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")   # one core per pool worker

import argparse                                  # noqa: E402
import importlib.util                            # noqa: E402
import json                                      # noqa: E402
import subprocess                                # noqa: E402
import sys                                       # noqa: E402
import time                                      # noqa: E402
from multiprocessing import Pool                 # noqa: E402
from pathlib import Path                         # noqa: E402

import numpy as np                               # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stage2a_turning_envelope as env           # noqa: E402

# LegWheel/examples/gslip -> parents[2] is the directory holding both repos
PAPER_FIGS = HERE.parents[2] / "corgi-abad-icra2027" / "figures" / "make_stage2a_figs.py"
JSONL = env.FIG_DIR / "stage2a_lowspeed_step025.jsonl"
NPZ_OUT = env.FIG_DIR / "stage2a_lowspeed_step025.npz"
OUT_TXT = HERE / "s340_lowspeed_step025.out.txt"

I_V = tuple(range(1, 7))                 # V_GRID_FULL[1:7] = 0.50 ... 0.75
N_LAM = 7                                # LAM_GRID_DEG[:7] = 0 ... 5 deg
LAWS = ("empirical", "geometric")
STEPS = (0.25, 0.5)
TASKS = [(step, law, i_v) for step in STEPS for law in LAWS for i_v in I_V]
EDGE_V_FWD = 0.526                       # below the grid's lowest cambered-cell v_fwd


def _check_block() -> None:
    v = env.V_GRID_FULL[list(I_V)]
    assert np.allclose(v, [0.50, 0.55, 0.60, 0.65, 0.70, 0.75], atol=1e-12), v
    lam = env.LAM_GRID_DEG[:N_LAM]
    assert np.array_equal(lam, [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 5.0]), lam


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


def solve_task(task):
    step, law, i_v = task
    t0 = time.time()
    cells = env.run_grid(env.V_GRID_FULL[i_v:i_v + 1], env.LAM_GRID_DEG[:N_LAM],
                         step, laws=(law,), verbose=False)
    return {"step": step, "law": law, "i_v": i_v,
            "v_td": float(env.V_GRID_FULL[i_v]),
            "seconds": time.time() - t0,
            "cells": [_jsonable(c) for c in cells]}


def load_runs() -> dict:
    runs = {}
    if JSONL.exists():
        for line in JSONL.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                runs[(r["step"], r["law"], r["i_v"])] = r
    return runs


def parse_indices(spec: str) -> list[int]:
    idx = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            idx.extend(range(int(a), int(b) + 1))
        else:
            idx.append(int(part))
    return idx


def solve(indices: list[int], workers: int) -> None:
    done = load_runs()
    todo = [TASKS[i] for i in indices if TASKS[i] not in done]
    print(f"{len(indices)} tasks requested, {len(todo)} not yet in {JSONL.name}; "
          f"{workers} workers", flush=True)
    if not todo:
        return
    t0 = time.time()
    with Pool(processes=min(workers, len(todo))) as pool, JSONL.open("a") as fh:
        for r in pool.imap_unordered(solve_task, todo):
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            n_ex = sum(c["exists"] for c in r["cells"])
            print(f"  step {r['step']:4.2f} {r['law'][:3]} v_td {r['v_td']:.2f}: "
                  f"{n_ex}/{len(r['cells'])} cells exist, {r['seconds']:.0f} s "
                  f"(wall {time.time() - t0:.0f} s)", flush=True)


# --- finalize -----------------------------------------------------------------

def load_paper_figs():
    spec = importlib.util.spec_from_file_location("make_stage2a_figs", PAPER_FIGS)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # module level only; main() not called
    return mod


def key(c) -> tuple:
    return (c["law"], round(float(c["v_td"]), 3), float(c["lam_deg"]))


def fails(c, leg_limit, abad_limit, psi_limit) -> str:
    """Every gate a cell fails (the paper's `binding` names only the first in
    its scrub > leg-torque > abad priority)."""
    if not c["exists"]:
        return "C1"
    f = [n for n, bad in (("C2", c["psi"] > psi_limit),
                          ("C3", c["tau_leg"] > leg_limit),
                          ("C4", c["tau_abad"] > abad_limit)) if bad]
    return "+".join(f) if f else "-"


def _git(*args) -> str:
    try:
        return subprocess.run(["git", "-C", str(HERE), *args], capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except Exception as exc:          # record, never fail the summary on git
        return f"unavailable ({exc})"


def finalize() -> None:
    lines: list[str] = []

    def say(s=""):
        print(s)
        lines.append(s)

    env._selftest()
    say("stage2a_turning_envelope selftest: PASS")
    fig = load_paper_figs()
    say(f"paper regate imported from {PAPER_FIGS}")
    say(f"  gates: C2 psi <= {fig.PSI_DOT_MAX}, C3 leg <= {fig.LEG_TAU_PAPER} (6:1) / "
        f"{fig.LEG_TAU_9TO1} (9:1), C4 profile contact <= {fig.ABAD_TAU_MAX}")
    say(f"  stage run_grid gates (verdicts as solved): {env.gate_constants()}")
    # HEAD only: git run from WSL on the /mnt/c checkout lists most tracked
    # files as modified (line endings), so a status line would be
    # noise
    say(f"LegWheel HEAD {_git('rev-parse', '--short', 'HEAD')}")

    runs = load_runs()
    missing = [t for t in TASKS if t not in runs]
    if missing:
        raise SystemExit(f"missing tasks (run --solve first): {missing}")
    cells = {s: [c for law in LAWS for i_v in I_V for c in runs[(s, law, i_v)]["cells"]]
             for s in STEPS}
    secs = {s: [runs[(s, law, i_v)]["seconds"] for law in LAWS for i_v in I_V] for s in STEPS}
    for s in STEPS:
        say(f"step {s}: {len(cells[s])} cells, task time {min(secs[s]):.0f}-"
            f"{max(secs[s]):.0f} s (summed task wall time / cells = "
            f"{sum(secs[s]) / len(cells[s]):.1f} s, with tasks sharing the machine)")

    keys = {key(c) for c in cells[0.25]}
    cache_all = list(np.load(fig.DEFAULT_CACHE, allow_pickle=True)["cells"])
    cache = {key(c): c for c in cache_all if key(c) in keys}
    assert len(cache) == len(keys) == 84, (len(cache), len(keys))
    say(f"cache of record: {fig.DEFAULT_CACHE} ({len(cache_all)} cells; {len(cache)} in this block)")

    LEG = {"6:1": fig.LEG_TAU_PAPER, "9:1": fig.LEG_TAU_9TO1}
    rg = {}   # (source, gear) -> {key: regated cell}
    for src, cs in (("cache", list(cache.values())), ("s0.5", cells[0.5]), ("s0.25", cells[0.25])):
        for gear, leg in LEG.items():
            rg[(src, gear)] = {key(c): c for c in fig.regate(cs, leg)}

    # sanity 1: run_grid's own verdicts (stage gates) vs the paper regate at 6:1
    same = (env.MOTOR_TORQUE_LIMIT == fig.LEG_TAU_PAPER and env.ABAD_TAU_MAX == fig.ABAD_TAU_MAX
            and env.PSI_DOT_MAX == fig.PSI_DOT_MAX)
    for s in STEPS:
        mism = sum(c["binding"] != rg[(f"s{s}", "6:1")][key(c)]["binding"] for c in cells[s])
        say(f"sanity: step {s} run_grid verdicts vs paper regate @6:1 (gates identical: {same}): "
            f"{mism} binding mismatches")

    # sanity 2 (control): step 0.5 on current code vs the s84 cache
    say("\n--- control: step 0.5, current code, vs the s84 cache (same block) ---")
    ex_diff = [k for k in keys if bool(cache[k]["exists"]) != bool(rg[("s0.5", "6:1")][k]["exists"])]
    say(f"existence differs in {len(ex_diff)} of 84 cells {sorted(ex_diff)}")
    both = [k for k in sorted(keys) if cache[k]["exists"] and rg[("s0.5", "6:1")][k]["exists"]]
    if both:
        new05 = rg[("s0.5", "6:1")]
        for fld in ("v_fwd", "beta_deg", "alpha_deg", "tau_leg"):
            d = max(abs(float(cache[k][fld]) - float(new05[k][fld])) for k in both)
            say(f"  max |d {fld}| over {len(both)} cells existing in both: {d:.4g}")
        d_abad = max(abs(float(rg[("cache", "6:1")][k]["tau_abad"]) - float(new05[k]["tau_abad"]))
                     for k in both)
        say(f"  max |d tau_abad| (profile contact, both regated): {d_abad:.4g}")
    for gear in LEG:
        m = sum(rg[("cache", gear)][k]["binding"] != rg[("s0.5", gear)][k]["binding"] for k in keys)
        say(f"  binding mismatches after regate @{gear}: {m}")

    # full table at step 0.25
    say("\n--- step 0.25 block, regated (binding = paper's first-failing gate; "
        "fails = every failing gate) ---")
    say(f"{'law':3} {'v_td':>4} {'lam':>4} | {'cache':>6} {'s0.5':>6} | {'beta':>5} {'alpha':>5} "
        f"{'duty':>5} {'slope':>6} {'v_fwd':>6} {'psi':>5} {'R':>6} {'tauL':>5} {'tauA':>5} | "
        f"{'bind 6:1':>10} {'fails':>5} | {'bind 9:1':>10} {'fails':>5} | {'cache 6:1':>10} {'cache 9:1':>10}")

    def vf(c):
        return f"{c['v_fwd']:6.3f}" if c["exists"] else "    --"

    for k in sorted(keys):
        c6, c9 = rg[("s0.25", "6:1")][k], rg[("s0.25", "9:1")][k]
        k6, k9 = rg[("cache", "6:1")][k], rg[("cache", "9:1")][k]
        n5 = rg[("s0.5", "6:1")][k]
        if c6["exists"]:
            mid = (f"{c6['beta_deg']:5.2f} {c6['alpha_deg']:5.2f} {c6['duty']:5.3f} "
                   f"{c6['slope']:+6.3f} {c6['v_fwd']:6.3f} {c6['psi']:5.3f} {c6['R']:6.2f} "
                   f"{c6['tau_leg']:5.2f} {c6['tau_abad']:5.2f}")
        else:
            mid = f"{'no fixed point':>61}"
        say(f"{k[0][:3]} {k[1]:4.2f} {k[2]:4.1f} | {vf(k6)} {vf(n5)} | {mid} | "
            f"{c6['binding']:>10} {fails(c6, fig.LEG_TAU_PAPER, fig.ABAD_TAU_MAX, fig.PSI_DOT_MAX):>5} | "
            f"{c9['binding']:>10} {fails(c9, fig.LEG_TAU_9TO1, fig.ABAD_TAU_MAX, fig.PSI_DOT_MAX):>5} | "
            f"{k6['binding']:>10} {k9['binding']:>10}")

    say("\n--- cells that exist at step 0.25 but not in the cache ---")
    new = [k for k in sorted(keys) if rg[("s0.25", "6:1")][k]["exists"] and not cache[k]["exists"]]
    for k in new:
        c6, c9 = rg[("s0.25", "6:1")][k], rg[("s0.25", "9:1")][k]
        at05 = rg[("s0.5", "6:1")][k]["exists"]
        say(f"  {k[0][:3]} v_td {k[1]:.2f} lam {k[2]:3.1f}: v_fwd {c6['v_fwd']:.3f}, beta "
            f"{c6['beta_deg']:.2f}, alpha {c6['alpha_deg']:.2f}; 6:1 {c6['binding']} "
            f"(fails {fails(c6, fig.LEG_TAU_PAPER, fig.ABAD_TAU_MAX, fig.PSI_DOT_MAX)}), 9:1 "
            f"{c9['binding']} (fails {fails(c9, fig.LEG_TAU_9TO1, fig.ABAD_TAU_MAX, fig.PSI_DOT_MAX)}); "
            f"psi {c6['psi']:.3f}, tau_leg {c6['tau_leg']:.2f}, tau_abad {c6['tau_abad']:.2f}; "
            f"exists at step 0.5 on current code: {at05}")
    say(f"  total {len(new)}")
    lost = [k for k in sorted(keys) if cache[k]["exists"] and not rg[("s0.25", "6:1")][k]["exists"]]
    say(f"cells in the cache that do not exist at step 0.25: {len(lost)} {lost}")
    resel = [k for k in sorted(keys) if cache[k]["exists"] and rg[("s0.25", "6:1")][k]["exists"]
             and abs(float(cache[k]["beta_deg"]) - rg[("s0.25", "6:1")][k]["beta_deg"]) > 1e-9]
    say(f"cells existing in both whose selected beta* changed: {len(resel)}")
    for k in resel:
        a, b = cache[k], rg[("s0.25", "6:1")][k]
        say(f"  {k[0][:3]} v_td {k[1]:.2f} lam {k[2]:3.1f}: beta {a['beta_deg']:.2f} -> "
            f"{b['beta_deg']:.2f}, v_fwd {a['v_fwd']:.4f} -> {b['v_fwd']:.4f}, tau_leg "
            f"{a['tau_leg']:.2f} -> {b['tau_leg']:.2f}, 6:1 {rg[('cache', '6:1')][k]['binding']} -> "
            f"{b['binding']}, 9:1 {rg[('cache', '9:1')][k]['binding']} -> "
            f"{rg[('s0.25', '9:1')][k]['binding']}")

    say(f"\n--- QUESTION: any feasible cambered (lam > 0) cell below v_fwd {EDGE_V_FWD} m/s? ---")
    for gear in LEG:
        for law in LAWS:
            f = [c for c in rg[("s0.25", gear)].values()
                 if c["law"] == law and c["feasible"] and c["lam_deg"] > 0]
            low = sorted((c for c in f if c["v_fwd"] < EDGE_V_FWD), key=key)
            lo = min((c["v_fwd"] for c in f), default=None)
            say(f"  {gear} {law:9s}: {'YES' if low else 'NO'} -- {len(f)} feasible cambered "
                f"cells in the block, lowest v_fwd {lo if lo is None else round(lo, 4)}"
                + "".join(f"\n      v_td {c['v_td']:.2f} lam {c['lam_deg']:.1f}: v_fwd "
                          f"{c['v_fwd']:.4f}, psi {c['psi']:.3f}, tau_leg {c['tau_leg']:.2f}, "
                          f"tau_abad {c['tau_abad']:.2f}, R {c['R']:.2f}" for c in low))

    say("\n--- the paper's II-D grid numbers: cache vs cache with this block at step 0.25 ---")
    say("(the block replaces its 84 cells; v_td >= 0.80 stays the s84 step-0.5 cache, "
        "so the second row is a mixed-step, mixed-code grid; R_min here is the "
        "grid's own -- the paper's R_min values also merge the 0.05-deg fine sliver, "
        "which has no cell below v_td 0.75)")
    s25 = {key(c): c for c in cells[0.25]}
    merged = [s25.get(key(c), c) for c in cache_all]
    for label, grid in (("cache (paper)", cache_all), ("cache + block@0.25", merged)):
        for gear, leg in LEG.items():
            reg = fig.regate(grid, leg)
            for law in LAWS:
                f = [c for c in reg if c["law"] == law and c["feasible"] and c["lam_deg"] > 0]
                vs, rs = fig._rmin_curve(reg, law)
                say(f"  {label:18s} {gear} {law:9s}: cambered v_fwd {min(c['v_fwd'] for c in f):.4f}-"
                    f"{max(c['v_fwd'] for c in f):.4f} m/s (v_td {min(c['v_td'] for c in f):.2f}-"
                    f"{max(c['v_td'] for c in f):.2f}), {len(f)} cells, per-speed R_min "
                    f"{min(rs):.2f}-{max(rs):.2f} m")

    meta = {"script": Path(__file__).name, "beta_step": 0.25, "control_step": 0.5,
            "v_td": [float(v) for v in env.V_GRID_FULL[list(I_V)]],
            "lam_deg": [float(x) for x in env.LAM_GRID_DEG[:N_LAM]],
            "legwheel_head": _git("rev-parse", "HEAD"),
            "note": "raw run_grid cells (verdicts at the stage gates in 'gates'); "
                    "the paper regate is in s340_lowspeed_step025.out.txt"}
    np.savez_compressed(NPZ_OUT, cells=np.array(cells[0.25], dtype=object),
                        cells_step05_control=np.array(cells[0.5], dtype=object),
                        gates=np.array(env.gate_constants(), dtype=object),
                        meta=np.array(meta, dtype=object))
    say(f"\nwrote {NPZ_OUT}")
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_TXT}")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--solve", help="task indices, e.g. 0-7 or 0,3,5 "
                    f"(0-11 step 0.25, 12-23 step 0.5; {len(TASKS)} tasks)")
    ap.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    ap.add_argument("--finalize", action="store_true")
    a = ap.parse_args(argv)
    _check_block()
    if a.solve:
        solve(parse_indices(a.solve), a.workers)
    if a.finalize:
        finalize()
    if not (a.solve or a.finalize):
        ap.print_help()


if __name__ == "__main__":
    main(sys.argv[1:])
