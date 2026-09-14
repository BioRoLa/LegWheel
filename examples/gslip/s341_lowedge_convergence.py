"""Log s341 (modelling check): is the lower edge of the Stage 2a envelope
converged in the landing-angle (beta) step?

Background: log s338 s3 / s340 s5 -- at beta step 0.5 the lowest fixed point in
the production window (alpha 1-45 deg, beta 60-86 deg, duty <= 0.55, apex >=
10 mm) is v_td 0.75 (v_fwd 0.528); at step 0.25 it is v_td 0.70 (v_fwd 0.497,
beta 84.25, alpha 44.14, duty 0.545). No finer step and no v_td between the
0.05 grid speeds was tried.

What this script does:
  --solve   v_td in {0.60, 0.625, 0.65, 0.675, 0.70, 0.725, 0.75} x lambda
            {0, 0.5, 1.0} (LAM_GRID_DEG[:3]) x both radius laws, at beta steps
            0.5, 0.25 and 0.125, with the stage script's own run_grid
            (unmodified). One task = one run_grid call for one (step, law,
            v_td) over the 3 lambdas in grid order; these are the prefix of
            LAM_GRID_DEG, so the warm-start chain is the full grid's. Speeds on
            the 0.05 grid use V_GRID_FULL's own floats.
            REUSE: steps 0.5 and 0.25 at v_td 0.60/0.65/0.70/0.75 are the first
            3 cells of the s340 tasks (stage2a_lowspeed_step025.jsonl), which
            ran the same run_grid over a 7-lambda list with the same prefix;
            they are read, not re-solved. Two "repro" tasks re-solve (0.25,
            empirical, 0.70) and (0.5, empirical, 0.70) fresh and --finalize
            compares them with the reused cells.
  --diag    dense beta scan at lambda 0.5 deg, empirical law (the cambered
            cell the edge question is about): for each (v_td, beta) calls
            find_fixed_points exactly as solve_existence does (alpha 1-45 deg,
            n_samples 20) and records every fixed point with its duty, apex and
            slope, plus a widened call (alpha 40-65 deg, n_samples 26) that
            shows what lies just outside alpha 45. This gives the continuous
            beta interval that passes the production filters at each speed,
            i.e. what a beta step of h can hit.
  --finalize  regates with the paper's regate (corgi-abad-icra2027/figures/
            make_stage2a_figs.py, imported; main not run) and writes
            s341_lowedge_convergence.out.txt and
            stage2a_figs/s341_lowedge_convergence.npz.

Resume: every finished task / diag point is appended to a jsonl and skipped on
rerun. Nothing here changes a number of record, a gate or a default.

    # in WSL, from the LegWheel root
    .venv/bin/python examples/gslip/s341_lowedge_convergence.py --plan
    .venv/bin/python examples/gslip/s341_lowedge_convergence.py --solve all --workers 2
    .venv/bin/python examples/gslip/s341_lowedge_convergence.py --diag 0.650,0.675,0.680,0.685,0.690,0.695,0.700,0.725,0.750:83.0:85.5:0.025 --workers 2
    .venv/bin/python examples/gslip/s341_lowedge_convergence.py --confirm --workers 2
    .venv/bin/python examples/gslip/s341_lowedge_convergence.py --finalize

  --confirm runs three fresh solves at speeds between the grid points (v_td 0.690
  at steps 0.125 and 0.25, v_td 0.685 at step 0.0625; lambda 0 and 0.5,
  empirical), chosen from the diag scan's prediction of which beta lattice hits
  the passing interval, to check that prediction with the production solver.
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
from functools import partial                    # noqa: E402
from multiprocessing import Pool                 # noqa: E402
from pathlib import Path                         # noqa: E402

import numpy as np                               # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stage2a_turning_envelope as env           # noqa: E402
from legwheel.models.gslip_fixed_point import find_fixed_points   # noqa: E402

PAPER_FIGS = HERE.parents[2] / "corgi-abad-icra2027" / "figures" / "make_stage2a_figs.py"
JSONL = env.FIG_DIR / "s341_lowedge_convergence.jsonl"
DIAG_JSONL = env.FIG_DIR / "s341_lowedge_diag.jsonl"
S340_JSONL = env.FIG_DIR / "stage2a_lowspeed_step025.jsonl"
NPZ_OUT = env.FIG_DIR / "s341_lowedge_convergence.npz"
OUT_TXT = HERE / "s341_lowedge_convergence.out.txt"

LAWS = ("empirical", "geometric")
STEPS = (0.5, 0.25, 0.125)
N_LAM = 3                                        # LAM_GRID_DEG[:3] = 0, 0.5, 1.0
# v_td label -> float actually solved (grid speeds use V_GRID_FULL's floats)
V_EDGE = {"0.600": float(env.V_GRID_FULL[3]), "0.625": 0.625,
          "0.650": float(env.V_GRID_FULL[4]), "0.675": 0.675,
          "0.700": float(env.V_GRID_FULL[5]), "0.725": 0.725,
          "0.750": float(env.V_GRID_FULL[6])}
S340_IV = {"0.600": 3, "0.650": 4, "0.700": 5, "0.750": 6}   # reused at 0.5 / 0.25
REUSED = {(s, law, vk) for s in (0.5, 0.25) for law in LAWS for vk in S340_IV}
DIAG_LAM = 0.5


def _build_tasks():
    order = ("0.675", "0.700", "0.725", "0.750", "0.650", "0.625", "0.600")
    t = [("solve", 0.125, law, vk) for vk in order for law in LAWS]
    t += [("solve", s, law, vk) for s in (0.25, 0.5) for vk in ("0.675", "0.725", "0.625")
          for law in LAWS]
    t += [("repro", 0.25, "empirical", "0.700"), ("repro", 0.5, "empirical", "0.700")]
    assert not any((s, law, vk) in REUSED for kind, s, law, vk in t if kind == "solve")
    return t


TASKS = _build_tasks()

# --confirm: speeds between the grid points, at steps whose beta lattice the
# diag scan predicts to hit (or miss) the passing interval. run_grid over
# lambda (0, 0.5), empirical law, production window, fresh cold start.
CONFIRM = [("confirm", 0.125, "empirical", "0.690"),     # diag: 84.375 inside -> FP
           ("confirm", 0.0625, "empirical", "0.685"),    # diag: 84.4375 inside -> FP
           ("confirm", 0.25, "empirical", "0.690")]      # diag: no lattice point -> none
CONFIRM_LAMS = (0.0, 0.5)


def _check() -> None:
    assert np.allclose([V_EDGE[k] for k in S340_IV], [0.60, 0.65, 0.70, 0.75], atol=1e-12)
    assert np.array_equal(env.LAM_GRID_DEG[:N_LAM], [0.0, 0.5, 1.0])
    for k, v in V_EDGE.items():
        assert abs(float(k) - v) < 1e-9, (k, v)


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
    kind, step, law, vk = task
    t0 = time.time()
    if kind == "confirm":
        v, lams = float(vk), np.array(CONFIRM_LAMS)
    else:
        v, lams = V_EDGE[vk], env.LAM_GRID_DEG[:N_LAM]
    cells = env.run_grid([v], lams, step, laws=(law,), verbose=False)
    return {"kind": kind, "step": step, "law": law, "v_key": vk, "v_td": v,
            "seconds": time.time() - t0, "cells": [_jsonable(c) for c in cells]}


def _tkey(r) -> tuple:
    return (r["kind"], r["step"], r["law"], r["v_key"])


def load_jsonl(path: Path, keyfn) -> dict:
    runs = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                runs[keyfn(r)] = r
    return runs


def parse_indices(spec: str, n: int) -> list[int]:
    if spec == "all":
        return list(range(n))
    idx = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            idx.extend(range(int(a), int(b) + 1))
        else:
            idx.append(int(part))
    return idx


def run_pool(fn, todo, workers, path, describe):
    t0 = time.time()
    with Pool(processes=max(1, min(workers, len(todo)))) as pool, path.open("a") as fh:
        for r in pool.imap_unordered(fn, todo):
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            print(f"  {describe(r)} (wall {time.time() - t0:.0f} s)", flush=True)


def solve(indices, workers, tasks=None) -> None:
    tasks = TASKS if tasks is None else tasks
    done = load_jsonl(JSONL, _tkey)
    todo = [tasks[i] for i in indices if tasks[i] not in done]
    print(f"{len(indices)} tasks requested, {len(todo)} not yet in {JSONL.name}; "
          f"{workers} workers", flush=True)
    if todo:
        run_pool(solve_task, todo, workers, JSONL,
                 lambda r: (f"{r['kind']} step {r['step']:5.3f} {r['law'][:3]} v_td {r['v_key']}: "
                            f"{sum(c['exists'] for c in r['cells'])}/{len(r['cells'])} exist, "
                            f"{r['seconds']:.0f} s"))


# --- diag: dense beta scan ------------------------------------------------------

def diag_point(task):
    vk, beta_deg = task
    v = V_EDGE[vk] if vk in V_EDGE else float(vk)
    lam = np.deg2rad(DIAG_LAM)
    p = env.base_params()
    stride_fn = partial(env.stride_const_r, lam=lam, r_const=env.R_EMPIRICAL)
    t0 = time.time()
    out = {"v_key": vk, "v_td": v, "beta": beta_deg, "lam_deg": DIAG_LAM, "law": "empirical"}
    for name, arange_deg, n in (("prod", (1.0, 45.0), 20), ("wide", (40.0, 65.0), 26)):
        fps = []
        try:
            found = find_fixed_points(p, v, np.deg2rad(beta_deg),
                                      alpha_range=tuple(np.deg2rad(arange_deg)),
                                      n_samples=n, stride_fn=stride_fn)
        except Exception as exc:            # record, never lose the point
            out[name + "_error"] = repr(exc)
            found = []
        for fp in found:
            try:
                apex = env.apex_mm(stride_fn(p, v, fp.alpha, fp.beta))
            except Exception:
                apex = float("nan")
            fps.append({"alpha": float(np.rad2deg(fp.alpha)), "duty": float(fp.duty_factor),
                        "apex_mm": float(apex), "slope": float(fp.slope),
                        "v_fwd": float(fp.mean_speed),
                        "pass": bool(fp.duty_factor <= env.MAX_DUTY and apex >= env.MIN_APEX_MM)})
        out[name] = fps
    out["seconds"] = time.time() - t0
    return out


def _dkey(r) -> tuple:
    return (r["v_key"], round(float(r["beta"]), 4))


def diag(spec: str, workers: int) -> None:
    # spec "v1,v2,...:lo:hi:step"
    vs, lo, hi, st = spec.split(":")
    vkeys = vs.split(",")
    betas = np.round(float(lo) + np.arange(int(round((float(hi) - float(lo)) / float(st))) + 1)
                     * float(st), 4)
    todo_all = [(vk, float(b)) for vk in vkeys for b in betas]
    done = load_jsonl(DIAG_JSONL, _dkey)
    todo = [t for t in todo_all if (t[0], round(t[1], 4)) not in done]
    print(f"diag: {len(todo_all)} points requested, {len(todo)} not yet in {DIAG_JSONL.name}; "
          f"{workers} workers", flush=True)
    if todo:
        n_done = [0]

        def desc(r):
            n_done[0] += 1
            ok = [f for f in r["prod"] if f["pass"]]
            return (f"[{n_done[0]}/{len(todo)}] v_td {r['v_key']} beta {r['beta']:.3f}: "
                    f"prod {len(r['prod'])} fp, {len(ok)} pass; wide {len(r['wide'])} fp; "
                    f"{r['seconds']:.1f} s")
        run_pool(diag_point, todo, workers, DIAG_JSONL, desc)


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


def apex_of(c) -> float:
    return 1000.0 * env.G * float(c["flight_s"]) ** 2 / 8.0


def finalize() -> None:
    lines: list[str] = []

    def say(s=""):
        print(s)
        lines.append(s)

    env._selftest()
    say("Log s341 low-edge convergence. stage2a_turning_envelope selftest: PASS")
    fig = load_paper_figs()
    say(f"paper regate imported from {PAPER_FIGS}")
    say(f"  gates: C2 psi <= {fig.PSI_DOT_MAX}, C3 leg <= {fig.LEG_TAU_PAPER} (6:1) / "
        f"{fig.LEG_TAU_9TO1} (9:1), C4 profile contact <= {fig.ABAD_TAU_MAX}")
    say(f"  solver window: beta 60-86 deg at the step, alpha 1-45 deg (n_samples 20), "
        f"duty <= {env.MAX_DUTY}, apex >= {env.MIN_APEX_MM} mm, min |slope| selected")
    say(f"LegWheel HEAD {_git('rev-parse', '--short', 'HEAD')}")

    runs = load_jsonl(JSONL, _tkey)
    missing = [t for t in TASKS if t not in runs]
    if missing:
        raise SystemExit(f"missing tasks (run --solve first): {missing}")
    s340 = load_jsonl(S340_JSONL, lambda r: (r["step"], r["law"], r["i_v"]))

    cells = {}          # (step, law, vk) -> list of 3 cells
    source = {}
    secs = {}
    for s in STEPS:
        for law in LAWS:
            for vk in V_EDGE:
                if (s, law, vk) in REUSED:
                    r = s340[(s, law, S340_IV[vk])]
                    cs = r["cells"][:N_LAM]
                    assert [c["lam_deg"] for c in cs] == [0.0, 0.5, 1.0]
                    assert abs(float(cs[0]["v_td"]) - V_EDGE[vk]) < 1e-12
                    cells[(s, law, vk)], source[(s, law, vk)] = cs, "s340"
                else:
                    r = runs[("solve", s, law, vk)]
                    cells[(s, law, vk)], source[(s, law, vk)] = r["cells"], "s341"
                    secs.setdefault(s, []).append(r["seconds"])
    for s in STEPS:
        if s in secs:
            say(f"step {s}: s341 task time {min(secs[s]):.0f}-{max(secs[s]):.0f} s over "
                f"{len(secs[s])} tasks (2 workers, machine shared with another solve)")

    say("\n--- reproduction of reused s340 cells (fresh solve, first 3 lambdas) ---")
    for kind, s, law, vk in TASKS:
        if kind != "repro":
            continue
        new = runs[(kind, s, law, vk)]["cells"]
        old = cells[(s, law, vk)]
        worst = 0.0
        same_ex = all(a["exists"] == b["exists"] for a, b in zip(new, old))
        for a, b in zip(new, old):
            if a["exists"] and b["exists"]:
                for fld in ("v_fwd", "beta_deg", "alpha_deg", "slope", "duty", "tau_leg", "tau_abad"):
                    worst = max(worst, abs(float(a[fld]) - float(b[fld])))
        say(f"  step {s} {law} v_td {vk}: existence identical {same_ex}; "
            f"max |diff| over v_fwd/beta/alpha/slope/duty/tau {worst:.3g}")

    LEG = {"6:1": fig.LEG_TAU_PAPER, "9:1": fig.LEG_TAU_9TO1}
    rg = {}
    for k, cs in cells.items():
        for gear, leg in LEG.items():
            rg[(gear,) + k] = fig.regate(cs, leg)

    say("\n--- all cells (binding = paper regate's first-failing gate) ---")
    say(f"{'step':>5} {'law':3} {'v_td':>5} {'lam':>3} {'src':>4} | {'beta':>6} {'alpha':>6} "
        f"{'duty':>5} {'apex':>5} {'slope':>6} {'v_fwd':>6} {'psi':>5} {'tauL':>5} {'tauA':>5} | "
        f"{'6:1':>10} {'9:1':>10}")
    for s in STEPS:
        for law in LAWS:
            for vk in V_EDGE:
                for i, c in enumerate(rg[("6:1", s, law, vk)]):
                    c9 = rg[("9:1", s, law, vk)][i]
                    if c["exists"]:
                        mid = (f"{c['beta_deg']:6.3f} {c['alpha_deg']:6.2f} {c['duty']:5.3f} "
                               f"{apex_of(c):5.1f} {c['slope']:+6.3f} {c['v_fwd']:6.4f} "
                               f"{c['psi']:5.3f} {c['tau_leg']:5.2f} {c['tau_abad']:5.2f}")
                    else:
                        mid = f"{'no fixed point':>62}"
                    say(f"{s:5.3f} {law[:3]} {vk} {c['lam_deg']:3.1f} {source[(s, law, vk)]:>4} | "
                        f"{mid} | {c['binding']:>10} {c9['binding']:>10}")

    say("\n--- lower edge per step ---")
    summary = {}
    for law in LAWS:
        for s in STEPS:
            ex = [vk for vk in V_EDGE if any(c["exists"] for c in cells[(s, law, vk)])]
            ex0 = [vk for vk in V_EDGE if cells[(s, law, vk)][0]["exists"]]
            feas = [vk for vk in V_EDGE
                    if any(c["feasible"] and c["lam_deg"] > 0 for c in rg[("6:1", s, law, vk)])]
            lo_ex = min(ex, key=float) if ex else None
            lo_f = min(feas, key=float) if feas else None
            summary[(law, s)] = (lo_ex, lo_f, ex, feas)
            say(f"{law:9s} step {s:5.3f}: v_td with a non-grazing FP (any lambda): "
                f"{ex or 'none'}; at lambda 0: {ex0 or 'none'}; with a 6:1-feasible cambered "
                f"cell: {feas or 'none'}")
            for label, vk in (("lowest FP", lo_ex), ("lowest 6:1-feasible cambered", lo_f)):
                if vk is None:
                    say(f"    {label}: none")
                    continue
                cs = rg[("6:1", s, law, vk)]
                c = next((x for x in cs if x["exists"] and (label == "lowest FP" or
                                                           (x["feasible"] and x["lam_deg"] > 0))))
                say(f"    {label}: v_td {vk}, v_fwd {c['v_fwd']:.4f} (lambda {c['lam_deg']:.1f}); "
                    f"beta {c['beta_deg']:.3f}, alpha {c['alpha_deg']:.2f}, slope {c['slope']:+.4f}, "
                    f"duty {c['duty']:.4f}, apex {apex_of(c):.1f} mm, tau_leg {c['tau_leg']:.2f}, "
                    f"tau_abad {c['tau_abad']:.2f}, psi {c['psi']:.3f}, 6:1 {c['binding']}")
        a, b = summary[(law, 0.25)], summary[(law, 0.125)]
        say(f"  {law}: 0.25 -> 0.125 lowest-FP v_td {a[0]} -> {b[0]} "
            f"({'MOVES' if a[0] != b[0] else 'same'}); lowest 6:1-feasible cambered v_td "
            f"{a[1]} -> {b[1]} ({'MOVES' if a[1] != b[1] else 'same'})")
        a5 = summary[(law, 0.5)]
        say(f"  {law}: 0.5 -> 0.25 lowest-FP v_td {a5[0]} -> {a[0]}; lowest 6:1-feasible "
            f"cambered {a5[1]} -> {a[1]}")

    # β* selected per step at the speeds that exist everywhere, to show reselection
    say("\n--- selected beta* and v_fwd at lambda 0.5 by step (empirical) ---")
    for vk in V_EDGE:
        row = []
        for s in STEPS:
            c = cells[(s, "empirical", vk)][1]
            row.append(f"{s}: " + (f"beta {c['beta_deg']:.3f} v_fwd {c['v_fwd']:.4f} slope "
                                   f"{c['slope']:+.4f}" if c["exists"] else "--"))
        say(f"  v_td {vk}: " + " | ".join(row))

    exist_by = {(s, vk): bool(cells[(s, "empirical", vk)][1]["exists"])
                for s in STEPS for vk in V_EDGE}
    preds = diag_summary(say, exist_by)

    say("\n--- confirm: fresh run_grid solves between grid speeds, lambda (0, 0.5), empirical, "
        "production window ---")
    for t in CONFIRM:
        if t not in runs:
            say(f"  {t}: not run")
            continue
        r = runs[t]
        hit = preds.get((t[3], t[1]))
        say(f"  step {t[1]} v_td {t[3]}: diag predicts lattice betas inside the interval "
            f"{hit if hit is not None else '(no diag row)'}; {r['seconds']:.0f} s")
        for c in fig.regate(r["cells"], fig.LEG_TAU_PAPER):
            if c["exists"]:
                say(f"    lambda {c['lam_deg']:.1f}: FP beta {c['beta_deg']:.4f}, alpha {c['alpha_deg']:.3f}, "
                    f"duty {c['duty']:.4f}, apex {apex_of(c):.1f} mm, slope {c['slope']:+.4f}, v_fwd "
                    f"{c['v_fwd']:.4f}, psi {c['psi']:.3f}, tau_leg {c['tau_leg']:.2f}, tau_abad "
                    f"{c['tau_abad']:.2f}; 6:1 {c['binding']}")
            else:
                say(f"    lambda {c['lam_deg']:.1f}: no fixed point")

    meta = {"script": Path(__file__).name, "steps": list(STEPS), "v_edge": V_EDGE,
            "lam_deg": [float(x) for x in env.LAM_GRID_DEG[:N_LAM]],
            "legwheel_head": _git("rev-parse", "HEAD"),
            "reused_from_s340": sorted([list(k) for k in REUSED]),
            "note": "raw run_grid cells keyed step/law/v_key; paper regate in the .out.txt"}
    np.savez_compressed(
        NPZ_OUT,
        cells=np.array([dict(c, step=s) for (s, law, vk), cs in cells.items() for c in cs],
                       dtype=object),
        gates=np.array(env.gate_constants(), dtype=object),
        diag=np.array(list(load_jsonl(DIAG_JSONL, _dkey).values()), dtype=object),
        meta=np.array(meta, dtype=object))
    say(f"\nwrote {NPZ_OUT}")
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_TXT}")


def _crossings(b, y, level):
    """Linear-interpolated beta values where y crosses `level` between
    consecutive scan points."""
    out = []
    for i in range(len(b) - 1):
        y0, y1 = y[i] - level, y[i + 1] - level
        if y0 == 0.0:
            out.append(float(b[i]))
        elif y0 * y1 < 0.0:
            out.append(float(b[i] + (b[i + 1] - b[i]) * (-y0) / (y1 - y0)))
    return out


def diag_edge(say, d, exist_by=None) -> None:
    """The passing family's beta interval at each scanned speed, from the
    interpolated alpha = 45 deg and duty = 0.55 crossings (the two filters
    that bound it, log s338 s3), and the speed where the interval closes."""
    say("\n--- diag: passing interval by interpolation (duty = 0.55 on the shallow side, "
        "alpha = 45 on the steep side) ---")
    say("family point per beta = the fixed point (production or widened call) with alpha nearest 45")
    by_v = {}
    for (vk, _), r in d.items():
        by_v.setdefault(vk, []).append(r)
    rows = []
    preds = {}
    for vk in sorted(by_v, key=float):
        rs = sorted(by_v[vk], key=lambda r: r["beta"])
        fam, multi = [], 0
        for r in rs:
            cand = r["prod"] + r["wide"]
            if not cand:
                continue
            if len({round(f["alpha"], 2) for f in cand}) > 1:
                multi += 1
            f = min(cand, key=lambda f: abs(f["alpha"] - 45.0))
            fam.append((r["beta"], f["alpha"], f["duty"], f["apex_mm"], f["v_fwd"], f["slope"]))
        if len(fam) < 3:
            say(f"  v_td {vk}: {len(fam)} family points, skipped")
            continue
        b = np.array([x[0] for x in fam])
        a = np.array([x[1] for x in fam])
        du = np.array([x[2] for x in fam])
        ap = np.array([x[3] for x in fam])
        vf = np.array([x[4] for x in fam])
        sl = np.array([x[5] for x in fam])
        gaps = int(np.sum(np.diff(b) > 0.0375))
        ba, bd = _crossings(b, a, 45.0), _crossings(b, du, 0.55)
        if len(ba) != 1 or len(bd) != 1:
            say(f"  v_td {vk}: alpha=45 crossings {ba}, duty=0.55 crossings {bd} "
                f"(beta {b[0]:.3f}-{b[-1]:.3f}, {len(fam)} pts, {gaps} gaps) -- not a single pair, skipped")
            continue
        ba, bd = ba[0], bd[0]
        w = ba - bd
        mid = 0.5 * (ba + bd)
        v_mid = float(np.interp(mid, b, vf))
        apex_min = float(np.interp(bd, b, ap)), float(np.interp(ba, b, ap))
        rows.append((float(vk), w, v_mid, bd, ba))
        pred = []
        for h in STEPS + (0.0625, 0.03125):
            k_lo = math.ceil((bd - 60.0) / h - 1e-9)
            hit = [60.0 + k * h for k in range(k_lo, k_lo + 64) if bd <= 60.0 + k * h <= ba]
            preds[(vk, h)] = [round(x, 5) for x in hit]
            solver = "" if not exist_by or (h, vk) not in exist_by else \
                f" [solver: {'FP' if exist_by[(h, vk)] else 'none'}]"
            pred.append(f"{h}: {[round(x, 5) for x in hit] or 'none'}{solver}")
        say(f"  v_td {vk}: duty 0.55 at beta {bd:.4f}, alpha 45 at beta {ba:.4f} -> width {w:+.4f} deg"
            f" ({'OPEN' if w > 0 else 'CLOSED'}); v_fwd at mid {v_mid:.4f}; apex {apex_min[0]:.1f}/"
            f"{apex_min[1]:.1f} mm, slope {float(np.interp(mid, b, sl)):+.4f} (scan {b[0]:.3f}-{b[-1]:.3f}, "
            f"{len(fam)} pts, {gaps} gaps, {multi} betas with >1 FP)")
        say("      beta-lattice points inside (60 + k h): " + "; ".join(pred))
    rows.sort()
    for (v0, w0, f0, *_), (v1, w1, f1, *_) in zip(rows, rows[1:]):
        if w0 <= 0.0 < w1:
            vs = v0 + (v1 - v0) * (-w0) / (w1 - w0)
            fs = f0 + (f1 - f0) * (-w0) / (w1 - w0)
            say(f"  interval closes between v_td {v0:.3f} (width {w0:+.4f}) and {v1:.3f} ({w1:+.4f}): "
                f"linear estimate v_td* = {vs:.4f}, v_fwd* ~ {fs:.4f} m/s")
    if rows:
        say(f"  width grows by ~{np.polyfit([r[0] for r in rows], [r[1] for r in rows], 1)[0]:.2f} "
            f"deg per m/s of v_td over the scanned speeds (linear fit)")
    return preds


def diag_summary(say, exist_by=None) -> dict:
    d = load_jsonl(DIAG_JSONL, _dkey)
    if not d:
        say("\n(no diag scan in this run)")
        return {}
    preds = diag_edge(say, d, exist_by)
    say(f"\n--- diag: dense beta scan, lambda {DIAG_LAM} deg, empirical law, "
        f"find_fixed_points as solve_existence calls it ---")
    say("pass = alpha in [1, 45] (found by the production call), duty <= 0.55, apex >= 10 mm")
    by_v = {}
    for (vk, b), r in d.items():
        by_v.setdefault(vk, []).append(r)
    for vk in sorted(by_v, key=float):
        rs = sorted(by_v[vk], key=lambda r: r["beta"])
        db = np.min(np.diff([r["beta"] for r in rs])) if len(rs) > 1 else float("nan")
        passing = [(r["beta"], f) for r in rs for f in r["prod"] if f["pass"]]
        say(f"v_td {vk}: beta {rs[0]['beta']:.3f}-{rs[-1]['beta']:.3f} at {db:.3f}, "
            f"{len(rs)} points; passing betas: {len(passing)}")
        if not passing:
            # what is closest: show the fixed points in the scan's middle and their failing filter
            near = [(r["beta"], f, src) for r in rs for src in ("prod", "wide") for f in r[src]]
            if near:
                best_duty = min((x for x in near if x[1]["alpha"] <= 45.0),
                                key=lambda x: x[1]["duty"], default=None)
                if best_duty:
                    b, f, src = best_duty
                    say(f"    alpha<=45 FP with the lowest duty: beta {b:.3f} alpha {f['alpha']:.2f} "
                        f"duty {f['duty']:.4f} apex {f['apex_mm']:.1f} v_fwd {f['v_fwd']:.4f} ({src})")
                lo_a = [x for x in near if x[1]["duty"] <= env.MAX_DUTY and x[1]["apex_mm"] >= env.MIN_APEX_MM]
                if lo_a:
                    b, f, src = min(lo_a, key=lambda x: x[1]["alpha"])
                    say(f"    duty/apex-passing FP with the lowest alpha: beta {b:.3f} alpha "
                        f"{f['alpha']:.2f} duty {f['duty']:.4f} apex {f['apex_mm']:.1f} v_fwd "
                        f"{f['v_fwd']:.4f} ({src})")
            continue
        bs = sorted({b for b, _ in passing})
        gaps = [(a, b) for a, b in zip(bs, bs[1:]) if b - a > 1.5 * db]
        say(f"    passing beta interval(s): {bs[0]:.3f}-{bs[-1]:.3f}"
            + (f", gaps {[(round(a, 3), round(b, 3)) for a, b in gaps]}" if gaps else " (contiguous)"))
        for b in (bs[0], bs[-1]):
            f = next(f for bb, f in passing if bb == b)
            say(f"      at beta {b:.3f}: alpha {f['alpha']:.3f}, duty {f['duty']:.4f}, apex "
                f"{f['apex_mm']:.1f} mm, slope {f['slope']:+.4f}, v_fwd {f['v_fwd']:.4f}")
        # what fails just outside each end
        for side, b in (("below", bs[0] - db), ("above", bs[-1] + db)):
            r = next((r for r in rs if abs(r["beta"] - b) < 1e-6), None)
            if r is None:
                say(f"      {side}: end of scan")
                continue
            fl = [f"prod a {f['alpha']:.2f} duty {f['duty']:.4f} apex {f['apex_mm']:.1f}" for f in r["prod"]]
            fl += [f"wide a {f['alpha']:.2f} duty {f['duty']:.4f} apex {f['apex_mm']:.1f}"
                   for f in r["wide"] if f["alpha"] > 45.0]
            say(f"      just {side} (beta {r['beta']:.3f}): {fl or 'no fixed point found'}")
        # which production steps can hit the interval (lattice 60 + k*h)
        for h in STEPS + (0.0625,):
            lat = np.arange(60.0, 86.0 + 1e-9, h)
            hit = [float(x) for x in lat if bs[0] - 1e-9 <= x <= bs[-1] + 1e-9]
            say(f"      lattice step {h}: grid betas inside [{bs[0]:.3f}, {bs[-1]:.3f}]: "
                f"{[round(x, 4) for x in hit] or 'none'} (interval known to +-{db:.3f})")
    return preds


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--solve", help=f"task indices, e.g. all, 0-13, 3,5 ({len(TASKS)} tasks)")
    ap.add_argument("--diag", help="v1,v2,...:beta_lo:beta_hi:beta_step")
    ap.add_argument("--confirm", action="store_true",
                    help=f"run the {len(CONFIRM)} between-grid-speed confirmation solves")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--finalize", action="store_true")
    a = ap.parse_args(argv)
    _check()
    if a.plan:
        done = load_jsonl(JSONL, _tkey)
        for i, t in enumerate(TASKS + CONFIRM):
            print(i, t, "DONE" if t in done else "")
        print("reused from s340:", sorted(REUSED))
    if a.solve:
        solve(parse_indices(a.solve, len(TASKS)), a.workers)
    if a.diag:
        diag(a.diag, a.workers)
    if a.confirm:
        solve(list(range(len(CONFIRM))), a.workers, tasks=CONFIRM)
    if a.finalize:
        finalize()
    if not (a.plan or a.solve or a.diag or a.confirm or a.finalize):
        ap.print_help()


if __name__ == "__main__":
    main(sys.argv[1:])
