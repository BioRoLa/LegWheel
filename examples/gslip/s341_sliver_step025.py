"""Log s341 (modelling check): the Stage 2a fine lambda sliver re-solved at
landing-angle (beta) step 0.25.

The paper's sliver (corgi-abad-icra2027/figures/stage2a_fine_sliver.npz, 135
cells, produced by figures/run_fine_sliver.py) runs the stage script's run_grid
at beta step 0.5, empirical law only, v_td 0.75-1.45, on a 0.05-deg lambda
window [-0.30, +0.10] deg around each speed's scrub limit placed at
v_fwd = 0.73 v_td. This script keeps that construction and changes:

  1. beta step 0.25 (the step the shipped v070 template exporter uses);
  2. v_td 0.70 added (log s340 s5: at step 0.25 it has a scrub-feasible
     cambered cell; 0.50-0.65 have no fixed point at step 0.25);
  3. both radius laws (so E3 can be checked on the sliver);
  4. the window's TOP is extended upward on the same 0.05 lattice.
     Log s337 s5: against each cell's own v_fwd the original window runs from
     -0.25/+0.15 (v_td 1.00) to -0.60/-0.20 (v_td 1.45), and from v_td 1.30 up
     its top cell is still scrub-feasible, so it does not bracket the limit.
     Here each window starts at the original's first lambda (lam73 - 0.30,
     so every original lambda float is kept and the first, cold-started cell
     is the same) and runs up to at least max(lam73 + 0.10, lam_lim + 0.25),
     where lam_lim = atan(0.29 v_fwd_est / g) and v_fwd_est is the speed's
     step-0.5 sliver v_fwd (step-0.25 s340 value 0.4971 at v_td 0.70).
     The 0.25 deg margin absorbs a beta* reselection at step 0.25;
     --finalize checks, per speed and law, that the bottom cell is
     scrub-feasible and the top cell scrub-infeasible (both existing), and
     prints FAIL otherwise.

One task = one run_grid call for one (law, v_td) over that speed's lambdas in
increasing order, exactly as run_fine_sliver.py does (warm start starts cold at
the first lambda and flows upward). Two control tasks re-solve the ORIGINAL
9-lambda windows at v_td 0.75 and 1.45 at step 0.5 (empirical) on current code
and are compared with the record sliver.

Outputs (all new files): stage2a_figs/stage2a_fine_sliver_step025.npz (key
`cells` = step-0.25 cells, both laws, same schema as the stage cache plus
c4_lever "profile"; `cells_step05_control`, `windows`, `gates`, `meta`),
stage2a_figs/stage2a_fine_sliver_step025.jsonl (resume) and
s341_sliver_step025.out.txt (paper regate, bracket check, E3, R_min).

    # in WSL, from the LegWheel root
    .venv/bin/python examples/gslip/s341_sliver_step025.py --plan
    .venv/bin/python examples/gslip/s341_sliver_step025.py --solve all --workers 2
    .venv/bin/python examples/gslip/s341_sliver_step025.py --finalize
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")   # one core per pool worker

import argparse                                  # noqa: E402
import importlib.util                            # noqa: E402
import json                                      # noqa: E402
import math                                      # noqa: E402
import subprocess                                # noqa: E402
import sys                                       # noqa: E402
import time                                      # noqa: E402
from multiprocessing import Pool                 # noqa: E402
from pathlib import Path                         # noqa: E402

import numpy as np                               # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stage2a_turning_envelope as env           # noqa: E402

PAPER_DIR = HERE.parents[2] / "corgi-abad-icra2027" / "figures"
PAPER_FIGS = PAPER_DIR / "make_stage2a_figs.py"
RECORD_SLIVER = PAPER_DIR / "stage2a_fine_sliver.npz"
RECORD_GRID = PAPER_DIR / "stage2a_grid.npz"
S340_NPZ = env.FIG_DIR / "stage2a_lowspeed_step025.npz"
JSONL = env.FIG_DIR / "stage2a_fine_sliver_step025.jsonl"
NPZ_OUT = env.FIG_DIR / "stage2a_fine_sliver_step025.npz"
OUT_TXT = HERE / "s341_sliver_step025.out.txt"

BETA_STEP = 0.25
LAWS = ("empirical", "geometric")
V_FWD_RATIO = 0.73            # run_fine_sliver.py
LAM_WINDOW = (-0.30, 0.10)    # run_fine_sliver.py
LAM_STEP = 0.05               # run_fine_sliver.py
TOP_MARGIN = 0.25             # deg above the estimated own-v_fwd scrub limit

# v_td floats: 0.70 is V_GRID_FULL's; 0.75-1.45 are run_fine_sliver.py's own
# np.arange(0.75, ...) floats, so every lambda of the record sliver recurs.
V_SLIVER = [float(env.V_GRID_FULL[5])] + [float(v) for v in np.arange(0.75, 1.45 + 1e-9, 0.05)]
V_KEYS = [f"{v:.2f}" for v in V_SLIVER]

# v_fwd estimates used ONLY to place each window's top (not a result):
# record sliver (step 0.5, empirical) min v_fwd per speed; 0.70 from s340 s5
# (step 0.25). Printed by scratchpad inspect_sliver.py, 2026-09-14.
V_FWD_EST = {"0.70": 0.4971, "0.75": 0.5278, "0.80": 0.5778, "0.85": 0.6284,
             "0.90": 0.6404, "0.95": 0.6919, "1.00": 0.7028, "1.05": 0.7553,
             "1.10": 0.8082, "1.15": 0.8615, "1.20": 0.9153, "1.25": 0.9696,
             "1.30": 1.0243, "1.35": 1.0795, "1.40": 1.1351, "1.45": 1.2333}

CONTROL = [("ctrl0.5", "empirical", "0.75"), ("ctrl0.5", "empirical", "1.45")]


def lam73(v_td: float) -> float:
    return float(np.degrees(np.arctan(env.PSI_DOT_MAX * V_FWD_RATIO * v_td / env.G)))


def lam_limit(v_fwd: float) -> float:
    return float(np.degrees(np.arctan(env.PSI_DOT_MAX * v_fwd / env.G)))


def original_window(v_td: float) -> np.ndarray:
    """run_fine_sliver.lam_window, verbatim."""
    lam_scrub = lam73(v_td)
    lo = lam_scrub + LAM_WINDOW[0]
    hi = lam_scrub + LAM_WINDOW[1]
    return np.round(np.arange(lo, hi + 1e-9, LAM_STEP), 3)


def window(vk: str) -> np.ndarray:
    v = V_SLIVER[V_KEYS.index(vk)]
    lo = lam73(v) + LAM_WINDOW[0]
    target = max(lam73(v) + LAM_WINDOW[1], lam_limit(V_FWD_EST[vk]) + TOP_MARGIN)
    n = int(math.ceil((target - lo) / LAM_STEP - 1e-9))
    lams = np.round(np.arange(lo, lo + n * LAM_STEP + 1e-9, LAM_STEP), 3)
    orig = original_window(v)
    assert np.array_equal(lams[:len(orig)], orig), (vk, lams, orig)
    return lams


TASKS = [("s0.25", law, vk) for vk in V_KEYS[::-1] for law in LAWS] + CONTROL


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


def task_lams(task) -> np.ndarray:
    kind, law, vk = task
    return window(vk) if kind == "s0.25" else original_window(V_SLIVER[V_KEYS.index(vk)])


def solve_task(task):
    kind, law, vk = task
    step = BETA_STEP if kind == "s0.25" else 0.5
    lams = task_lams(task)
    t0 = time.time()
    cells = env.run_grid([V_SLIVER[V_KEYS.index(vk)]], lams, step, laws=(law,), verbose=False)
    return {"kind": kind, "step": step, "law": law, "v_key": vk,
            "lams": [float(x) for x in lams], "seconds": time.time() - t0,
            "cells": [_jsonable(c) for c in cells]}


def _tkey(r) -> tuple:
    return (r["kind"], r["law"], r["v_key"])


def load_runs() -> dict:
    runs = {}
    if JSONL.exists():
        for line in JSONL.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                t = _tkey(r)
                # a task whose window changed is a different task: keep only matches
                if t in TASKS and r["lams"] == [float(x) for x in task_lams(t)]:
                    runs[t] = r
    return runs


def parse_indices(spec: str) -> list[int]:
    if spec == "all":
        return list(range(len(TASKS)))
    idx = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            idx.extend(range(int(a), int(b) + 1))
        else:
            idx.append(int(part))
    return idx


def solve(indices, workers) -> None:
    done = load_runs()
    todo = [TASKS[i] for i in indices if TASKS[i] not in done]
    print(f"{len(indices)} tasks requested, {len(todo)} not yet in {JSONL.name}; "
          f"{workers} workers", flush=True)
    if not todo:
        return
    t0 = time.time()
    with Pool(processes=max(1, min(workers, len(todo)))) as pool, JSONL.open("a") as fh:
        for r in pool.imap_unordered(solve_task, todo):
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            cs = r["cells"]
            n_ex = sum(c["exists"] for c in cs)
            print(f"  {r['kind']} {r['law'][:3]} v_td {r['v_key']}: {n_ex}/{len(cs)} exist, "
                  f"lam {r['lams'][0]:.3f}-{r['lams'][-1]:.3f}, {r['seconds']:.0f} s "
                  f"(wall {time.time() - t0:.0f} s)", flush=True)


# --- finalize -----------------------------------------------------------------

def load_paper_figs():
    spec = importlib.util.spec_from_file_location("make_stage2a_figs", PAPER_FIGS)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # module level only; main() not called
    return mod


def _git(*args) -> str:
    try:
        return subprocess.run(["git", "-C", str(HERE), *args], capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except Exception as exc:
        return f"unavailable ({exc})"


def vkey(c) -> str:
    return f"{float(c['v_td']):.2f}"


def finalize() -> None:
    lines: list[str] = []

    def say(s=""):
        print(s)
        lines.append(s)

    env._selftest()
    say("Log s341 fine sliver at beta step 0.25. stage2a_turning_envelope selftest: PASS")
    fig = load_paper_figs()
    T = fig.PSI_DOT_MAX
    say(f"paper regate imported from {PAPER_FIGS}")
    say(f"  gates: C2 psi <= {T}, C3 leg <= {fig.LEG_TAU_PAPER} (6:1) / {fig.LEG_TAU_9TO1} (9:1), "
        f"C4 profile contact <= {fig.ABAD_TAU_MAX}")
    say(f"LegWheel HEAD {_git('rev-parse', '--short', 'HEAD')}")
    runs = load_runs()
    missing = [t for t in TASKS if t not in runs]
    if missing:
        raise SystemExit(f"missing tasks (run --solve first): {missing}")
    cells = [c for t in TASKS if t[0] == "s0.25" for c in runs[t]["cells"]]
    ctrl = [c for t in CONTROL for c in runs[t]["cells"]]
    secs = [runs[t]["seconds"] for t in TASKS if t[0] == "s0.25"]
    say(f"step 0.25: {len(cells)} cells ({len(V_KEYS)} speeds x 2 laws), task time "
        f"{min(secs):.0f}-{max(secs):.0f} s, summed {sum(secs) / 3600:.2f} h CPU")
    LEG = {"6:1": fig.LEG_TAU_PAPER, "9:1": fig.LEG_TAU_9TO1}
    rg = {g: fig.regate(cells, leg) for g, leg in LEG.items()}

    # --- windows and bracket check
    say("\n--- windows: construction and bracket of the 0.29 rad/s limit at each cell's own v_fwd ---")
    say("orig = run_fine_sliver.py's [-0.30, +0.10] around lam73 (v_fwd = 0.73 v_td); "
        "ext = this script's top")
    windows = {}
    all_ok = True
    for vk in V_KEYS:
        v = V_SLIVER[V_KEYS.index(vk)]
        lams = window(vk)
        orig = original_window(v)
        windows[vk] = {"lams": [float(x) for x in lams], "n_orig": len(orig),
                       "lam73": lam73(v), "v_fwd_est": V_FWD_EST[vk]}
        for law in LAWS:
            cs = [c for c in rg["6:1"] if c["law"] == law and vkey(c) == vk]
            ex = [c for c in cs if c["exists"]]
            if not ex:
                say(f"  {vk} {law[:3]}: NO fixed point in the window -- FAIL")
                all_ok = False
                continue
            lim = [lam_limit(c["v_fwd"]) for c in ex]
            bot, top = cs[0], cs[-1]
            ok = (len(ex) == len(cs) and bot["psi"] <= T and top["psi"] > T)
            all_ok &= ok
            orig_top = cs[len(orig) - 1]
            orig_br = orig_top["exists"] and orig_top["psi"] > T
            say(f"  {vk} {law[:3]}: lam {lams[0]:.3f}-{lams[-1]:.3f} ({len(lams)} cells, orig "
                f"{len(orig)} to {orig[-1]:.3f}); own limit {min(lim):.3f}-{max(lim):.3f} -> window "
                f"{lams[0] - max(lim):+.2f}/{lams[-1] - min(lim):+.2f} "
                f"(orig top {orig[-1] - min(lim):+.2f}, brackets: {orig_br}); exist "
                f"{len(ex)}/{len(cs)}; bottom psi {bot['psi']:.3f}, top psi {top['psi']:.3f} -> "
                f"{'BRACKETS' if ok else 'FAIL'}")
    say(f"bracket check over all speeds and laws: {'PASS' if all_ok else 'FAIL'}")

    # --- per-speed table
    say("\n--- per speed (paper regate) ---")
    say(f"{'v_td':>4} {'law':3} | {'beta*':>12} {'v_fwd':>13} {'v/vtd':>5} {'tauL':>11} "
        f"{'tauA max':>8} | {'max scrub-ok lam':>16} {'R there':>7} {'v/0.29':>6} {'ratio':>5} "
        f"{'analytic lam':>12} | {'6:1':>5} {'9:1':>5}")
    per = {}
    for law in LAWS:
        for vk in V_KEYS:
            cs6 = [c for c in rg["6:1"] if c["law"] == law and vkey(c) == vk]
            cs9 = [c for c in rg["9:1"] if c["law"] == law and vkey(c) == vk]
            ex = [c for c in cs6 if c["exists"]]
            if not ex:
                continue
            sok = [c for c in ex if c["psi"] <= T and c["lam_deg"] > 0]
            best = max(sok, key=lambda c: c["lam_deg"]) if sok else None
            f6 = [c for c in cs6 if c["feasible"] and c["lam_deg"] > 0]
            f9 = [c for c in cs9 if c["feasible"] and c["lam_deg"] > 0]
            betas = sorted({round(c["beta_deg"], 3) for c in ex})
            vf = [c["v_fwd"] for c in ex]
            tl = [c["tau_leg"] for c in ex]
            per[(law, vk)] = {"best": best, "f6": f6, "f9": f9, "betas": betas, "ex": ex}
            line = (f"{vk} {law[:3]} | {str(betas):>12} {min(vf):.4f}-{max(vf):.4f} "
                    f"{np.mean(vf) / float(vk):5.3f} {min(tl):5.2f}-{max(tl):5.2f} "
                    f"{max(c['tau_abad'] for c in ex):8.2f} | ")
            if best:
                line_v = best["v_fwd"] / T
                line += (f"{best['lam_deg']:16.3f} {best['R']:7.3f} {line_v:6.3f} "
                         f"{best['R'] / line_v:5.3f} {lam_limit(best['v_fwd']):12.3f} | ")
            else:
                line += f"{'none':>16} {'':>7} {'':>6} {'':>5} {'':>12} | "
            line += f"{'yes' if f6 else 'no':>5} {'yes' if f9 else 'no':>5}"
            say(line)

    # --- R_min curves (sliver only)
    say("\n--- R_min per speed from the step-0.25 sliver alone (tightest feasible lambda>0 cell) ---")
    rmin = {}
    for gear in LEG:
        for law in LAWS:
            vs, rs = fig._rmin_curve(rg[gear], law)
            f = [c for c in rg[gear] if c["law"] == law and c["feasible"] and c["lam_deg"] > 0]
            rmin[(gear, law)] = (vs, rs)
            if not f:
                say(f"  {gear} {law:9s}: no feasible cambered cell")
                continue
            vtd = sorted({vkey(c) for c in f}, key=float)
            ratio = [r / (v / T) for v, r in zip(vs, rs)]
            say(f"  {gear} {law:9s}: feasible v_td {vtd[0]}-{vtd[-1]} ({len(vtd)} speeds: {vtd}); "
                f"v_fwd {min(vs):.4f}-{max(vs):.4f}; R_min {min(rs):.3f}-{max(rs):.3f} m; "
                f"R_min/(v/0.29) {min(ratio):.4f}-{max(ratio):.4f}; max feasible lambda "
                f"{max(c['lam_deg'] for c in f):.3f}; C4 max {max(c['tau_abad'] for c in f):.2f}; "
                f"largest leg demand {max(c['tau_leg'] for c in f):.2f}")
            say("      " + ", ".join(f"({v:.3f}, {r:.3f})" for v, r in zip(vs, rs)))

    # --- E3 on the sliver
    say("\n--- E3 on the sliver: empirical vs geometric ---")
    for gear in LEG:
        fe = {(vkey(c), c["lam_deg"]) for c in rg[gear] if c["law"] == "empirical" and c["feasible"]}
        fg = {(vkey(c), c["lam_deg"]) for c in rg[gear] if c["law"] == "geometric" and c["feasible"]}
        say(f"  {gear}: feasible cells emp {len(fe)}, geo {len(fg)}, only-emp "
            f"{sorted(fe - fg)}, only-geo {sorted(fg - fe)}")
        (ve, re_), (vg, rg_) = rmin[(gear, "empirical")], rmin[(gear, "geometric")]
        if ve and vg and len(ve) == len(vg):
            dv = max(abs(a - b) for a, b in zip(ve, vg))
            dr = max(abs(a - b) / a for a, b in zip(re_, rg_))
            # envelope area proxy: trapezoid under the R_min curve over forward speed
            ae, ag = np.trapezoid(re_, ve), np.trapezoid(rg_, vg)
            say(f"      R_min curves: same speeds; max |dv_fwd| {dv:.4f}, max |dR|/R {100 * dr:.2f} %; "
                f"area under R_min(v) emp {ae:.4f}, geo {ag:.4f} ({100 * (ag - ae) / ae:+.2f} %)")
        else:
            say(f"      R_min curves on different speed sets: emp {len(ve)}, geo {len(vg)}")

    # --- E1 / measured band
    say(f"\n--- measured band v_fwd {fig.MEASURED_V[0]}-{fig.MEASURED_V[1]} m/s (sliver, empirical) ---")
    band = [c for c in rg["6:1"] if c["law"] == "empirical" and c["exists"]
            and fig.MEASURED_V[0] <= c["v_fwd"] <= fig.MEASURED_V[1]]
    sok = [c for c in band if c["psi"] <= T and c["lam_deg"] > 0]
    if band and sok:
        say(f"  {len(band)} existing cells at v_td {sorted({vkey(c) for c in band}, key=float)}; "
            f"scrub-feasible max lambda {max(c['lam_deg'] for c in sok):.3f} deg "
            f"(analytic limit at the band's cells {min(lam_limit(c['v_fwd']) for c in band):.3f}-"
            f"{max(lam_limit(c['v_fwd']) for c in band):.3f})")
        for vk in sorted({vkey(c) for c in band}, key=float):
            s_v = [c for c in sok if vkey(c) == vk]
            say(f"    v_td {vk}: v_fwd {band[[vkey(c) for c in band].index(vk)]['v_fwd']:.4f}, "
                f"max scrub-feasible lambda {max((c['lam_deg'] for c in s_v), default=float('nan')):.3f}")
    else:
        say(f"  {len(band)} existing cells in the band, {len(sok)} scrub-feasible")
    for gear in LEG:
        bg = [c for c in rg[gear] if c["law"] == "empirical" and c["exists"]
              and fig.MEASURED_V[0] <= c["v_fwd"] <= fig.MEASURED_V[1]]
        f = [c for c in bg if c["feasible"] and c["lam_deg"] > 0]
        split = {k: [c["lam_deg"] for c in bg if c["binding"] == k] for k in ("scrub", "leg-torque", "abad")}
        say(f"  {gear}: {len(f)} feasible cambered cells"
            + (f", max lambda {max(c['lam_deg'] for c in f):.3f}" if f else "")
            + "; binding " + ", ".join(f"{k} {len(v)}" + (f" (lam {min(v):.3f}-{max(v):.3f})" if v else "")
                                       for k, v in split.items()))
    top = [c for c in rg["9:1"] if c["law"] == "empirical" and c["feasible"] and c["lam_deg"] > 0]
    if top:
        say(f"  9:1 largest feasible lambda over the whole sliver {max(c['lam_deg'] for c in top):.3f} "
            f"deg at v_fwd {max(top, key=lambda c: c['lam_deg'])['v_fwd']:.4f}")

    # --- E4 entry slew on the sliver (feasible cells, 6:1 no-load / rated)
    say("\n--- E4: worst entry slew / flight over feasible sliver cells ---")
    for gear in LEG:
        f = [c for c in rg[gear] if c["feasible"]]
        if not f:
            say(f"  {gear}: no feasible sliver cell")
            continue
        for name, w in (("no-load 34.6", fig.ABAD_SPEED_NOLOAD), ("rated 14.66", fig.ABAD_SPEED_RATED)):
            wc = max(f, key=lambda c: np.deg2rad(c["lam_deg"]) / w / c["flight_s"])
            say(f"  {gear} {name} rad/s: {100 * np.deg2rad(wc['lam_deg']) / w / wc['flight_s']:.2f} % "
                f"(lambda {wc['lam_deg']:.3f}, flight {1e3 * wc['flight_s']:.0f} ms, {wc['law'][:3]} "
                f"v_td {vkey(wc)})")

    # --- comparison with the record sliver (step 0.5) on common cells
    say("\n--- vs the record sliver (step 0.5, empirical) on common (v_td, lambda) ---")
    rec_raw = list(np.load(RECORD_SLIVER, allow_pickle=True)["cells"])
    rec = {g: {(vkey(c), float(c["lam_deg"])): c for c in fig.regate(rec_raw, leg)} for g, leg in LEG.items()}
    new = {g: {(vkey(c), float(c["lam_deg"])): c for c in rg[g] if c["law"] == "empirical"} for g in LEG}
    common = sorted(set(rec["6:1"]) & set(new["6:1"]), key=lambda k: (float(k[0]), k[1]))
    say(f"  record {len(rec_raw)} cells; common {len(common)}; record cells not in the new sliver "
        f"{len(set(rec['6:1']) - set(new['6:1']))}")
    for vk in V_KEYS:
        ks = [k for k in common if k[0] == vk]
        if not ks:
            continue
        a = [rec["6:1"][k] for k in ks]
        b = [new["6:1"][k] for k in ks]
        exd = sum(x["exists"] != y["exists"] for x, y in zip(a, b))
        both = [(x, y) for x, y in zip(a, b) if x["exists"] and y["exists"]]
        flips = {g: [(k[1], rec[g][k]["binding"], new[g][k]["binding"]) for k in ks
                     if rec[g][k]["binding"] != new[g][k]["binding"]] for g in LEG}
        if both:
            say(f"  {vk}: beta* {sorted({round(float(x['beta_deg']), 3) for x, _ in both})} -> "
                f"{sorted({round(float(y['beta_deg']), 3) for _, y in both})}; v_fwd "
                f"{both[0][0]['v_fwd']:.4f} -> {both[0][1]['v_fwd']:.4f}; tau_leg "
                f"{max(x['tau_leg'] for x, _ in both):.2f} -> {max(y['tau_leg'] for _, y in both):.2f}; "
                f"existence changes {exd}; binding flips 6:1 {flips['6:1']} 9:1 {flips['9:1']}")
        else:
            say(f"  {vk}: existence changes {exd}; no cell exists in both")

    say("\n--- control: step 0.5 on current code vs the record sliver (original windows) ---")
    rc = {(vkey(c), float(c["lam_deg"])): c for c in rec_raw}
    for kind, law, vk in CONTROL:
        cs = runs[(kind, law, vk)]["cells"]
        d = {}
        exd = 0
        for c in cs:
            r = rc[(vk, float(c["lam_deg"]))]
            exd += bool(r["exists"]) != bool(c["exists"])
            if r["exists"] and c["exists"]:
                for fld in ("v_fwd", "beta_deg", "alpha_deg", "tau_leg", "psi"):
                    d[fld] = max(d.get(fld, 0.0), abs(float(r[fld]) - float(c[fld])))
        reg_c = fig.regate(cs, fig.LEG_TAU_PAPER)
        reg_r = fig.regate([rc[(vk, float(c["lam_deg"]))] for c in cs], fig.LEG_TAU_PAPER)
        mism = sum(a["binding"] != b["binding"] for a, b in zip(reg_c, reg_r))
        say(f"  {law} v_td {vk}: existence differs {exd}/{len(cs)}; max |diff| "
            + ", ".join(f"{k} {v:.3g}" for k, v in d.items()) + f"; 6:1 binding mismatches {mism}")

    # --- II-D style numbers, sliver merged into the record grid + s340 block
    say("\n--- II-D-style readout: record grid (step 0.5) with the s340 low-speed block "
        "(step 0.25) substituted, plus this sliver ---")
    say("(MIXED STEP: grid cells at v_td >= 0.80 stay step 0.5; only the sliver and the "
        "v_td <= 0.75 block are step 0.25. The step-0.25 full grid is a separate task.)")
    grid = list(np.load(RECORD_GRID, allow_pickle=True)["cells"])
    blk = {(c["law"], round(float(c["v_td"]), 3), float(c["lam_deg"])): c
           for c in np.load(S340_NPZ, allow_pickle=True)["cells"]}
    merged = [blk.get((c["law"], round(float(c["v_td"]), 3), float(c["lam_deg"])), c) for c in grid]
    for gear, leg in LEG.items():
        for label, cs in (("record grid + record sliver", grid + rec_raw),
                          ("grid w/ s340 block + s341 sliver", merged + cells)):
            reg = fig.regate(cs, leg)
            f = [c for c in reg if c["law"] == "empirical" and c["feasible"] and c["lam_deg"] > 0]
            vs, rs = fig._rmin_curve(reg, "empirical")
            say(f"  {gear} {label:33s}: cambered v_fwd {min(c['v_fwd'] for c in f):.3f}-"
                f"{max(c['v_fwd'] for c in f):.3f} (v_td {min(float(c['v_td']) for c in f):.2f}-"
                f"{max(float(c['v_td']) for c in f):.2f}); per-speed R_min {min(rs):.3f}-{max(rs):.3f} m")

    meta = {"script": Path(__file__).name, "beta_step": BETA_STEP, "laws": list(LAWS),
            "v_td": V_SLIVER, "v_fwd_ratio_for_window": V_FWD_RATIO, "lam_window": LAM_WINDOW,
            "lam_step": LAM_STEP, "top_margin_deg": TOP_MARGIN, "v_fwd_est": V_FWD_EST,
            "bracket_check": "PASS" if all_ok else "FAIL",
            "legwheel_head": _git("rev-parse", "HEAD"),
            "note": "raw run_grid cells (verdicts at the stage gates); paper regate in "
                    "s341_sliver_step025.out.txt"}
    np.savez_compressed(NPZ_OUT, cells=np.array(cells, dtype=object),
                        cells_step05_control=np.array(ctrl, dtype=object),
                        windows=np.array(windows, dtype=object),
                        gates=np.array(env.gate_constants(), dtype=object),
                        meta=np.array(meta, dtype=object))
    say(f"\nwrote {NPZ_OUT}")
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_TXT}")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--solve", help=f"task indices, e.g. all, 0-5 ({len(TASKS)} tasks)")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--finalize", action="store_true")
    a = ap.parse_args(argv)
    if a.plan:
        done = load_runs()
        n = 0
        for i, t in enumerate(TASKS):
            lams = task_lams(t)
            n += len(lams)
            print(i, t, f"{len(lams)} lams {lams[0]:.3f}-{lams[-1]:.3f}", "DONE" if t in done else "")
        print(f"{n} cells total")
    if a.solve:
        solve(parse_indices(a.solve), a.workers)
    if a.finalize:
        finalize()
    if not (a.plan or a.solve or a.finalize):
        ap.print_help()


if __name__ == "__main__":
    main(sys.argv[1:])
