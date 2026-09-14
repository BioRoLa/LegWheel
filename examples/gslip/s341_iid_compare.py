"""Log s341 R3: every Sec. II-D number re-read on the beta-step-0.25 Stage 2a
inputs, side by side with the s84 cache of record the paper uses.

Inputs (all read only):
  s84    corgi-abad-icra2027/figures/stage2a_grid.npz (546 cells, beta step 0.5)
         + figures/stage2a_fine_sliver.npz (135 cells, empirical law only).
         These are the paper's inputs.
  s025   stage2a_figs/stage2a_grid_step025.npz (546 cells, beta step 0.25, s341 R1)
         + stage2a_figs/stage2a_fine_sliver_step025.npz (432 cells, both laws,
         extended window tops, s341 R2).
  s025w  the same step-0.25 grid + the step-0.25 sliver CUT to
         run_fine_sliver.py's original lambda windows (the first n_orig lambdas
         of each speed, v_td 0.70 included), so the effect of R2's extended
         window tops can be told apart from the beta step.
  s336   stage2a_figs/stage2a_grid_s336.npz (beta step 0.5, current geometric
         rolling-radius law); grid-only rows. The geometric law's step-only
         reference: s84's geometric arm predates the s336 radius fix.

Every gate and readout is the paper's own: regate(), _rmin_curve(),
cache_gates() and print_sec2d() from corgi-abad-icra2027/figures/
make_stage2a_figs.py, loaded with runpy.run_path(run_name="x") so its main()
(which writes tracked PDFs) never runs. Nothing is solved, nothing is drawn,
nothing is written to the paper repo.

    # in WSL, from the LegWheel root
    .venv/bin/python examples/gslip/s341_iid_compare.py

Writes examples/gslip/s341_iid_compare.out.txt (a copy of stdout).
"""
from __future__ import annotations

import hashlib
import math
import runpy
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
PAPER = HERE.parents[2] / "corgi-abad-icra2027"
FIGS_PY = PAPER / "figures" / "make_stage2a_figs.py"
FIG_DIR = HERE / "stage2a_figs"
INPUTS = {
    "s84 grid": PAPER / "figures" / "stage2a_grid.npz",
    "s84 sliver": PAPER / "figures" / "stage2a_fine_sliver.npz",
    "s025 grid": FIG_DIR / "stage2a_grid_step025.npz",
    "s025 sliver": FIG_DIR / "stage2a_fine_sliver_step025.npz",
    "s336 grid": FIG_DIR / "stage2a_grid_s336.npz",
}
OUT_TXT = HERE / "s341_iid_compare.out.txt"
INF = float("inf")
LAWS = ("empirical", "geometric")
SCOPE = {"grid": "grid", "fine": "grid+sliver"}
DS_NOTE = {"s84": "paper inputs, step 0.5", "s025": "step 0.25, R2 extended sliver",
           "s025w": "step 0.25, sliver cut to original windows",
           "s336": "step 0.5, s336 geometric radius law (grid only)"}

fig = None          # make_stage2a_figs namespace, set in run()
T0 = None
GEAR = {}
BAND = None
DS = {}
_cache = {}


class Tee:
    def __init__(self, stream):
        self.stream, self.buf = stream, []

    def write(self, s):
        self.stream.write(s)
        self.buf.append(s)
        return len(s)

    def flush(self):
        self.stream.flush()


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:12]


def git(repo: Path, *args) -> str:
    try:
        return subprocess.run(["git", "-C", str(repo), *args], capture_output=True,
                              text=True, timeout=60).stdout.strip()
    except Exception as exc:
        return f"unavailable ({exc})"


def vk(c) -> float:
    return round(float(c["v_td"]), 3)


def lk(c) -> float:
    return round(float(c["lam_deg"]), 3)


def phimax(v: float, T: float | None = None) -> float:
    return math.degrees(math.atan((T0 if T is None else T) * v / fig.GRAV))


def R(ds: str, scope: str, gear: str, T: float | None = None):
    """Paper regate of dataset ds (scope grid|fine) at a gear's leg gate and C2 limit T."""
    T = T0 if T is None else T
    key = (ds, scope, gear, T)
    if key not in _cache:
        _cache[key] = fig.regate(DS[ds][scope], GEAR[gear], fig.ABAD_TAU_MAX, T)
    return _cache[key]


def feas_camb(cs, law):
    return [c for c in cs if c["law"] == law and c["feasible"] and c["lam_deg"] > 0]


def rmin_cells(cs, law):
    """_rmin_curve, returning the cells (checked against the paper's function)."""
    out = []
    for v in sorted({vk(c) for c in cs if c["law"] == law}):
        f = [c for c in cs if c["law"] == law and vk(c) == v and c["feasible"] and c["lam_deg"] > 0]
        if f:
            out.append(min(f, key=lambda c: c["R"]))
    vs, rs = fig._rmin_curve(cs, law)
    assert [float(c["v_fwd"]) for c in out] == vs and [float(c["R"]) for c in out] == rs
    return out


def band_cells(cs, law=None):
    return [c for c in cs if (law is None or c["law"] == law) and c["exists"]
            and BAND[0] <= c["v_fwd"] <= BAND[1]]


def each(scopes=("grid", "fine"), dss=("s84", "s025", "s025w", "s336")):
    for scope in scopes:
        for ds in dss:
            if ds not in DS or DS[ds][scope] is None:
                continue
            if scope == "grid" and ds == "s025w":
                continue             # identical to s025
            yield scope, ds


def head(title):
    print(f"\n{'=' * 100}\n{title}\n{'=' * 100}")


def rng(xs, fmt="{:.2f}"):
    xs = list(xs)
    return (fmt.format(min(xs)) + "-" + fmt.format(max(xs))) if xs else "none"


def slew(c, w):
    return np.deg2rad(abs(c["lam_deg"])) / w / max(1e-9, c["flight_s"])


def trapz(y, x):
    f = getattr(np, "trapezoid", None) or np.trapz
    return float(f(y, x))


# ----------------------------------------------------------------------------------------------
def run():
    global fig, T0, GEAR, BAND
    fig = SimpleNamespace(**runpy.run_path(str(FIGS_PY), run_name="x"))
    T0 = fig.PSI_DOT_MAX
    GEAR.update({"35 stage": fig.LEG_TAU_STAGE, "6:1": fig.LEG_TAU_PAPER, "9:1": fig.LEG_TAU_9TO1})
    BAND = fig.MEASURED_V

    head("s341 R3: Sec. II-D numbers, s84 cache of record vs beta step 0.25")
    print(f"paper figure module: {FIGS_PY} (sha256 {sha(FIGS_PY)}; imported via runpy, run_name 'x')")
    print(f"  gates: C2 psi <= {T0}, C3 leg {fig.LEG_TAU_PAPER} (6:1) / {fig.LEG_TAU_9TO1} (9:1) "
          f"[stage {fig.LEG_TAU_STAGE}], C4 profile contact <= {fig.ABAD_TAU_MAX}; measured band "
          f"v_fwd {BAND[0]}-{BAND[1]} m/s")
    print(f"paper HEAD {git(PAPER, 'rev-parse', '--short', 'HEAD')}; LegWheel HEAD "
          f"{git(HERE, 'rev-parse', '--short', 'HEAD')}")
    npz = {}
    for k, p in INPUTS.items():
        if p.exists():
            npz[k] = np.load(p, allow_pickle=True)
            print(f"  {k:12s}: {p} sha256 {sha(p)}, {len(npz[k]['cells'])} cells, keys {npz[k].files}")
        else:
            print(f"  {k:12s}: MISSING {p}")
    cells = {k: list(z["cells"]) for k, z in npz.items()}
    win = npz["s025 sliver"]["windows"].item()

    def in_orig(c):
        w = win[f"{float(c['v_td']):.2f}"]
        return lk(c) in {round(x, 3) for x in w["lams"][:w["n_orig"]]}

    sliver_w = [c for c in cells["s025 sliver"] if in_orig(c)]
    DS.update({
        "s84": {"grid": cells["s84 grid"], "fine": cells["s84 grid"] + cells["s84 sliver"]},
        "s025": {"grid": cells["s025 grid"], "fine": cells["s025 grid"] + cells["s025 sliver"]},
        "s025w": {"grid": cells["s025 grid"], "fine": cells["s025 grid"] + sliver_w},
    })
    if "s336 grid" in cells:
        DS["s336"] = {"grid": cells["s336 grid"], "fine": None}
    for ds, d in DS.items():
        for scope, cs in d.items():
            if cs is None:
                continue
            n = {law: sum(c["law"] == law for c in cs) for law in LAWS}
            print(f"  dataset {ds:5s} {SCOPE[scope]:11s}: {len(cs)} cells ({n}); {DS_NOTE[ds]}")
    for law in LAWS:
        print(f"  sliver cells, {law}: s84 {sum(c['law'] == law for c in cells['s84 sliver'])}, "
              f"s025 {sum(c['law'] == law for c in cells['s025 sliver'])}, "
              f"s025 cut to original windows {sum(c['law'] == law for c in sliver_w)}")

    # --- provenance: regating at each cache's own gates reproduces its stored verdicts ------------
    head("0. Provenance: regate at each cache's own gates vs its stored verdicts")
    for k in ("s84 grid", "s84 sliver", "s025 grid", "s025 sliver", "s336 grid"):
        if k not in npz:
            continue
        g = fig.cache_gates(npz[k])
        if k == "s84 sliver":
            g = dict(g, leg=fig.LEG_TAU_STAGE)
        rg = fig.regate(cells[k], g["leg"], g["abad"], g["psi"])
        mism = sum(a["binding"] != b["binding"] for a, b in zip(cells[k], rg))
        print(f"  {k:12s}: gates leg {g['leg']} / abad {g['abad']} / psi {g['psi']} ({g['src']}): "
              f"binding mismatches {mism} / {len(rg)}")
    for scope, ds in each(scopes=("grid",)):
        a, b = R(ds, "grid", "35 stage"), R(ds, "grid", "6:1")
        fl = [(x, y) for x, y in zip(a, b) if x["binding"] != y["binding"]]
        kinds = sorted({f"{x['binding']}->{y['binding']}" for x, y in fl})
        print(f"  {ds:5s} grid: binding flips 35 -> 29.5 N.m leg gate: {len(fl)} ({kinds}); "
              f"eroded tau_leg of flipped {rng([x['tau_leg'] for x, _ in fl], '{:.2f}')}")

    # --- 1. the ~0.73 v_td ratio -------------------------------------------------------------------
    head("1. '(~0.73 v_td here)': v_fwd / v_td")
    for scope, ds in each():
        parts = []
        for law in LAWS:
            ex = [c for c in DS[ds][scope] if c["law"] == law and c["exists"]]
            if not ex:
                continue
            r = [c["v_fwd"] / c["v_td"] for c in ex]
            f6 = feas_camb(R(ds, scope, "6:1"), law)
            f9 = feas_camb(R(ds, scope, "9:1"), law)
            parts.append(f"{law[:3]} existing {min(r):.3f}-{max(r):.3f} (mean {np.mean(r):.3f}, n {len(r)}); "
                         f"feasible cambered 6:1 {rng([c['v_fwd'] / c['v_td'] for c in f6], '{:.3f}')}, "
                         f"9:1 {rng([c['v_fwd'] / c['v_td'] for c in f9], '{:.3f}')}")
        print(f"  {SCOPE[scope]:11s} {ds:5s}: " + " | ".join(parts))

    # --- 2. feasible cambered spans and R_min ------------------------------------------------------
    head("2. Feasible cambered span, per-speed R_min and its ratio to the scrub line (II-D p.4, p.5)")
    for gear in ("6:1", "9:1"):
        for law in LAWS:
            print(f"\n  [{gear} {law}]")
            for scope, ds in each():
                cs = R(ds, scope, gear)
                f = feas_camb(cs, law)
                if not f:
                    print(f"    {SCOPE[scope]:11s} {ds:5s}: no feasible cambered cell")
                    continue
                rm = rmin_cells(cs, law)
                vf = [c["v_fwd"] for c in f]
                vt = sorted({vk(c) for c in f})
                ratio = [c["R"] / (c["v_fwd"] / T0) for c in rm]
                gap = [phimax(c["v_fwd"]) - c["lam_deg"] for c in rm]
                lm = max(f, key=lambda c: c["lam_deg"])
                print(f"    {SCOPE[scope]:11s} {ds:5s}: v_fwd {min(vf):.4f}-{max(vf):.4f} "
                      f"[{min(vf):.2f}-{max(vf):.2f}] v_td {vt[0]:.2f}-{vt[-1]:.2f} ({len(vt)} speeds, "
                      f"{len(f)} cells); R_min {min(c['R'] for c in rm):.3f}-{max(c['R'] for c in rm):.3f} "
                      f"[{min(c['R'] for c in rm):.1f}-{max(c['R'] for c in rm):.1f}] m; line v/0.29 at "
                      f"those speeds {min(c['v_fwd'] for c in rm) / T0:.3f}-{max(c['v_fwd'] for c in rm) / T0:.3f}; "
                      f"R_min/line {min(ratio):.3f}-{max(ratio):.3f}; phi_max - lam(R_min) "
                      f"{min(gap):.3f}-{max(gap):.3f} deg; max feasible lam {lm['lam_deg']:.3f} "
                      f"(v_td {vk(lm):.2f}, v_fwd {lm['v_fwd']:.4f})")
    print("\n  per-speed R_min cells, empirical law, grid+sliver (v_td: v_fwd, lam, R, R/line, beta*):")
    for gear in ("6:1", "9:1"):
        for ds in ("s84", "s025", "s025w"):
            rm = rmin_cells(R(ds, "fine", gear), "empirical")
            print(f"    {gear} {ds:5s}: " + "; ".join(
                f"{vk(c):.2f}: {c['v_fwd']:.4f} {c['lam_deg']:.3f} {c['R']:.3f} "
                f"{c['R'] / (c['v_fwd'] / T0):.3f} {float(c['beta_deg']):.2f}" for c in rm))

    head("2b. 'the coarse grid's 2.3-3.3 m R_min values were grid artifacts' (grid only) and "
         "'135 added cells'")
    for gear in ("6:1", "9:1"):
        for scope, ds in each(scopes=("grid",)):
            for law in LAWS:
                rm = rmin_cells(R(ds, "grid", gear), law)
                print(f"  {gear} {ds:5s} {law[:3]} grid-only per-speed R_min: "
                      f"{rng([c['R'] for c in rm], '{:.3f}')} m over {len(rm)} speeds; lams "
                      f"{sorted({lk(c) for c in rm})}")

    # --- 3. measured band ---------------------------------------------------------------------------
    head(f"3. Measured band v_fwd {BAND[0]}-{BAND[1]} m/s: binding split, divides at 1 deg (HEAD) "
         "and at phi_max (working tree), torque feasibility")
    print(f"  analytic phi_max = atan(0.29 v/g) over the band: {phimax(BAND[0]):.3f}-{phimax(BAND[1]):.3f} deg")
    for gear in ("6:1", "9:1"):
        for law in LAWS:
            print(f"\n  [{gear} {law}]")
            for scope, ds in each():
                b = band_cells(R(ds, scope, gear), law)
                if not b:
                    print(f"    {SCOPE[scope]:11s} {ds:5s}: no existing cell in the band")
                    continue
                split = {k: [c["lam_deg"] for c in b if c["binding"] == k]
                         for k in ("none", "scrub", "leg-torque", "abad")}
                fc = [c for c in b if c["feasible"] and c["lam_deg"] > 0]
                sok = [c for c in b if c["psi"] <= T0 and c["lam_deg"] > 0]
                out = []
                for name, div in (("1deg", lambda c: 1.0), ("phi_max", lambda c: phimax(c["v_fwd"]))):
                    above = [c for c in b if c["lam_deg"] > div(c) + 1e-9]
                    below = [c for c in b if c["lam_deg"] <= div(c) + 1e-9]
                    va = [c for c in above if c["binding"] != "scrub"]
                    vb = [c for c in below if c["binding"] != "leg-torque"]
                    ex = "; ".join(f"v_td {vk(c):.2f} lam {c['lam_deg']:.3f} {c['binding']}" for c in (va + vb)[:4])
                    out.append(f"divide {name}: above {len(above)} (not scrub {len(va)}), below {len(below)} "
                               f"(not leg-torque {len(vb)})" + (f" e.g. {ex}" if ex else ""))
                speeds = sorted({vk(c) for c in b})
                print(f"    {SCOPE[scope]:11s} {ds:5s}: {len(b)} cells at v_td {speeds}; binding "
                      + ", ".join(f"{k} {len(v)}" + (f" (lam {min(v):.3f}-{max(v):.3f})" if v else "")
                                  for k, v in split.items())
                      + f"; feasible cambered {len(fc)}; min tau_leg {min(c['tau_leg'] for c in b):.2f}; "
                      f"max scrub-feasible lam {max((c['lam_deg'] for c in sok), default=float('nan')):.3f}"
                      f"; all scrub-feasible <= own phi_max: "
                      f"{all(c['lam_deg'] <= phimax(c['v_fwd']) + 1e-9 for c in sok)}")
                for o in out:
                    print(f"        {o}")

    # --- 4. 9:1 coverage of the band -------------------------------------------------------------
    head("4. 9:1: 'the sweep's top', 'covering the measured band', in-band R_min '2.4-2.9 m'")
    vmax_grid = max(vk(c) for c in DS["s84"]["grid"])
    print(f"  scrub line at the band edges: {BAND[0] / T0:.3f}-{BAND[1] / T0:.3f} m")
    for scope, ds in each():
        for law in LAWS:
            cs = R(ds, scope, "9:1")
            f = feas_camb(cs, law)
            rm = rmin_cells(cs, law)
            rin = [c for c in rm if BAND[0] <= c["v_fwd"] <= BAND[1]]
            top = max(f, key=lambda c: c["v_fwd"])
            print(f"  {SCOPE[scope]:11s} {ds:5s} {law[:3]}: fastest feasible cambered cell v_td {vk(top):.2f} "
                  f"(sweep top {vmax_grid:.2f}: {vk(top) == vmax_grid}) v_fwd {top['v_fwd']:.4f} beta* "
                  f"{float(top['beta_deg']):.2f} tau_leg {top['tau_leg']:.2f}; covers band: "
                  f"{min(c['v_fwd'] for c in f) <= BAND[0] and max(c['v_fwd'] for c in f) >= BAND[1]}; "
                  f"in-band per-speed R_min {rng([c['R'] for c in rin], '{:.3f}')} at v_fwd "
                  f"{[round(c['v_fwd'], 3) for c in rin]}")

    # --- 5. leg torque and C4 ---------------------------------------------------------------------
    head("5. Largest eroded leg demand among feasible cells ('37.2 N.m') and C4 ('21.5' / '27.9', never binds)")
    for gear in ("6:1", "9:1"):
        print(f"\n  [{gear}]")
        for scope, ds in each():
            cs = R(ds, scope, gear)
            parts = []
            for law in LAWS + ("both",):
                f = [c for c in cs if c["feasible"] and (law == "both" or c["law"] == law)]
                fc = [c for c in f if c["lam_deg"] > 0]
                if not f:
                    continue
                w = max(fc, key=lambda c: c["tau_leg"]) if fc else None
                parts.append(f"{law[:4]} all {max(c['tau_leg'] for c in f):.2f} / cambered "
                             + (f"{w['tau_leg']:.2f} (v_td {vk(w):.2f} lam {w['lam_deg']:.3f})" if w else "none"))
            fb = [c for c in cs if c["feasible"]]
            ex = [c for c in cs if c["exists"]]
            wa = max(fb, key=lambda c: c["tau_abad"])
            print(f"    {SCOPE[scope]:11s} {ds:5s}: leg demand " + "; ".join(parts))
            print(f"    {'':11s} {'':5s}  C4 max feasible demand {wa['tau_abad']:.2f} N.m ({wa['law'][:3]} v_td "
                  f"{vk(wa):.2f} lam {wa['lam_deg']:.3f}); all existing cells: max {max(c['tau_abad'] for c in ex):.2f}, "
                  f"> {fig.ABAD_TAU_MAX}: {sum(c['tau_abad'] > fig.ABAD_TAU_MAX for c in ex)}, binding abad "
                  f"{sum(c['binding'] == 'abad' for c in cs)}")

    # --- 6. E1 --------------------------------------------------------------------------------------
    head("6. E1: bank in the band and at the top; 'feasible cells bank at most 2 deg' (C4 paragraph)")
    for scope, ds in each():
        for law in LAWS:
            b = band_cells(R(ds, scope, "6:1"), law)
            sok = [c for c in b if c["psi"] <= T0 and c["lam_deg"] > 0]
            f9b = [c for c in band_cells(R(ds, scope, "9:1"), law) if c["feasible"] and c["lam_deg"] > 0]
            f9 = feas_camb(R(ds, scope, "9:1"), law)
            f6 = feas_camb(R(ds, scope, "6:1"), law)
            t9 = max(f9, key=lambda c: c["lam_deg"])
            hs = [c for c in f9 if c["lam_deg"] >= 1.5]
            print(f"  {SCOPE[scope]:11s} {ds:5s} {law[:3]}: band max scrub-feasible lam "
                  f"{max((c['lam_deg'] for c in sok), default=float('nan')):.3f} (9:1 band feasible max "
                  f"{max((c['lam_deg'] for c in f9b), default=float('nan')):.3f}); max feasible lam 6:1 "
                  f"{max(c['lam_deg'] for c in f6):.3f}, 9:1 {t9['lam_deg']:.3f} at v_fwd {t9['v_fwd']:.4f} "
                  f"(v_td {vk(t9):.2f}); 9:1 feasible lam >= 1.5 only at v_fwd >= "
                  f"{min((c['v_fwd'] for c in hs), default=float('nan')):.3f}")

    # --- 7. E3 ---------------------------------------------------------------------------------------
    head("7. E3: feasible sets empirical vs geometric (the stage script's metric |sym diff| / |union|) "
         "and an R_min-curve area proxy")
    for gear in ("35 stage", "6:1", "9:1"):
        for scope, ds in each():
            if scope == "fine" and ds == "s84":
                print(f"  {gear:8s} {SCOPE[scope]:11s} {ds:5s}: n/a (the record sliver is empirical-only)")
                continue
            cs = R(ds, scope, gear)
            fe = {(vk(c), lk(c)) for c in cs if c["law"] == "empirical" and c["feasible"]}
            fg = {(vk(c), lk(c)) for c in cs if c["law"] == "geometric" and c["feasible"]}
            metric = len(fe ^ fg) / max(1, len(fe | fg))
            ce, cg = rmin_cells(cs, "empirical"), rmin_cells(cs, "geometric")
            extra = ""
            if ce and [vk(c) for c in ce] == [vk(c) for c in cg]:
                ae = trapz([c["R"] for c in ce], [c["v_fwd"] for c in ce])
                ag = trapz([c["R"] for c in cg], [c["v_fwd"] for c in cg])
                dr = max(abs(a["R"] - b["R"]) / a["R"] for a, b in zip(ce, cg))
                extra = (f"; R_min curves same speeds, max |dR|/R {100 * dr:.2f} %, area under R_min(v) "
                         f"{100 * (ag - ae) / ae if len(ce) > 1 else 0.0:+.2f} %")
            elif ce or cg:
                extra = f"; R_min curves on different speed sets ({len(ce)} vs {len(cg)})"
            print(f"  {gear:8s} {SCOPE[scope]:11s} {ds:5s}: emp {len(fe)} geo {len(fg)} identical {fe == fg}; "
                  f"metric {100 * metric:.1f} %; only-emp {sorted(fe - fg)[:4]} only-geo {sorted(fg - fe)[:4]}"
                  + extra)

    # --- 8. E4 ---------------------------------------------------------------------------------------
    head("8. E4: worst entry slew / flight (lambda / omega / flight_s)")
    W = {f"no-load {fig.ABAD_SPEED_NOLOAD:.1f}": fig.ABAD_SPEED_NOLOAD,
         f"rated {fig.ABAD_SPEED_RATED:.2f}": fig.ABAD_SPEED_RATED}
    for scope, ds in each():
        for gear in ("6:1", "9:1"):
            cs = R(ds, scope, gear)
            f = [c for c in cs if c["feasible"]]
            parts = []
            for name, w in W.items():
                wc = max(f, key=lambda c: slew(c, w))
                parts.append(f"{name}: {100 * slew(wc, w):.2f} % (lam {wc['lam_deg']:.3f}, flight "
                             f"{1e3 * wc['flight_s']:.0f} ms, {wc['law'][:3]} v_td {vk(wc):.2f})")
            print(f"  {SCOPE[scope]:11s} {ds:5s} {gear} feasible: " + "; ".join(parts))
        ex = [c for c in DS[ds][scope] if c["exists"]]
        parts = []
        for name, w in W.items():
            wc = max(ex, key=lambda c: slew(c, w))
            parts.append(f"{name}: {100 * slew(wc, w):.2f} % (lam {wc['lam_deg']:.1f}, {wc['law'][:3]} v_td {vk(wc):.2f})")
        print(f"  {SCOPE[scope]:11s} {ds:5s} all existing (context): " + "; ".join(parts))

    # --- 9. threshold insensitivity --------------------------------------------------------------
    head("9. 'from 0.21 rad/s up, or without C2, the feasible speed range under either gearbox is unchanged'")
    print("  Feasible(T) = feasible at T=inf and psi <= T (regate's C2 is the only T-dependent gate), so the")
    print("  span can only widen with T. Breakpoint = smallest cell psi from which the readout equals T=inf.")
    bps = {}
    for scope, ds in each():
        print(f"\n  [{SCOPE[scope]} {ds}]")
        for gear in ("6:1", "9:1"):
            for law in LAWS:
                fe = feas_camb(R(ds, scope, gear, INF), law)
                if not fe:
                    print(f"    {gear} {law[:3]}: no feasible cambered cell at T = inf")
                    continue
                psis = sorted({float(c["psi"]) for c in fe})

                def exact(T):
                    f = [c["v_fwd"] for c in fe if c["psi"] <= T]
                    return (min(f), max(f)) if f else None

                def shown(T):
                    e = exact(T)
                    return None if e is None else (f"{e[0]:.2f}", f"{e[1]:.2f}")

                def tdset(T):
                    return frozenset(vk(c) for c in fe if c["psi"] <= T)

                res = {}
                for name, fn in (("v_fwd exact", exact), ("v_fwd 2-dec", shown), ("v_td set", tdset)):
                    target, bp, below = fn(INF), None, None
                    for T in reversed(psis):
                        if fn(T) == target:
                            bp = T
                        else:
                            below = fn(T)
                            break
                    setter = [c for c in fe if c["psi"] == bp]
                    res[name] = bp
                    bps.setdefault((scope, ds, name), []).append((bp, gear, law, setter, below))
                    sdesc = "; ".join(f"v_td {vk(c):.2f} lam {c['lam_deg']:.3f} v_fwd {c['v_fwd']:.4f} "
                                      f"beta* {float(c['beta_deg']):.2f}" for c in setter[:2])
                    btxt = (sorted(below) if isinstance(below, frozenset) else below)
                    print(f"    {gear} {law[:3]} {name:11s}: T=inf {sorted(target) if isinstance(target, frozenset) else target}; "
                          f"breakpoint psi {bp:.4f} ({sdesc}); just below it: {btxt}")
                spans = ", ".join(f"T {T:g}: {shown(T)}" for T in (0.16, 0.18, 0.20, 0.21, 0.29, 0.36, 0.40, INF))
                print(f"    {gear} {law[:3]} 2-dec span by T: {spans}")
        for name in ("v_fwd exact", "v_fwd 2-dec", "v_td set"):
            worst = max(bps[(scope, ds, name)], key=lambda x: x[0])
            print(f"    => {name}: unchanged for every T >= {worst[0]:.4f} rad/s (set by {worst[1]} {worst[2]}), "
                  f"and T=0.29 equals T=inf: {worst[0] <= T0}")

    # --- 10. the paper's own printout -------------------------------------------------------------
    head("10. make_stage2a_figs.print_sec2d() on each input (the paper script's own Sec. II-D printout)")
    for ds, gk in (("s84", "s84 grid"), ("s025", "s025 grid"), ("s025w", "s025 grid")):
        g = fig.cache_gates(npz[gk])
        stage = fig.regate(DS[ds]["grid"], g["leg"], g["abad"], g["psi"])
        print(f"\n  >>> {ds} ({DS_NOTE[ds]})")
        fig.print_sec2d(R(ds, "grid", "6:1"), R(ds, "grid", "9:1"), R(ds, "fine", "6:1"),
                        R(ds, "fine", "9:1"), stage, g)

    # --- 11. summary --------------------------------------------------------------------------------
    head("11. Summary: each II-D statement, paper value, s84 read, step-0.25 reads (empirical law unless noted)")

    def span_s(ds, scope, gear, law="empirical", td=True):
        f = feas_camb(R(ds, scope, gear), law)
        s = f"{min(c['v_fwd'] for c in f):.2f}-{max(c['v_fwd'] for c in f):.2f}"
        if td:
            s += f" (v_td {min(vk(c) for c in f):.2f}-{max(vk(c) for c in f):.2f})"
        return s

    def rmin_s(ds, gear, band_only=False):
        rm = rmin_cells(R(ds, "fine", gear), "empirical")
        if band_only:
            rm = [c for c in rm if BAND[0] <= c["v_fwd"] <= BAND[1]]
        rs = [c["R"] for c in rm]
        return f"{min(rs):.1f}-{max(rs):.1f} ({min(rs):.2f}-{max(rs):.2f})"

    def band_zero(ds):
        return str(all(not (c["feasible"] and c["lam_deg"] > 0)
                       for c in band_cells(R(ds, "fine", "6:1")))).lower()

    def divide(ds, which):
        b = band_cells(R(ds, "fine", "6:1"))
        div = (lambda c: 1.0) if which == "1deg" else (lambda c: phimax(c["v_fwd"]))
        ok = all((c["binding"] == "scrub") if c["lam_deg"] > div(c) + 1e-9 else (c["binding"] == "leg-torque")
                 for c in b)
        bg = band_cells(R(ds, "grid", "6:1"))
        okg = all((c["binding"] == "scrub") if c["lam_deg"] > div(c) + 1e-9 else (c["binding"] == "leg-torque")
                  for c in bg)
        return f"{str(okg).lower()} grid / {str(ok).lower()} +sliver"

    def top_is_sweep(ds):
        f = feas_camb(R(ds, "fine", "9:1"), "empirical")
        return str(max(vk(c) for c in f) == vmax_grid).lower()

    def covers(ds):
        f = feas_camb(R(ds, "fine", "9:1"), "empirical")
        return str(min(c["v_fwd"] for c in f) <= BAND[0] and max(c["v_fwd"] for c in f) >= BAND[1]).lower()

    def leg37(ds):
        f = feas_camb(R(ds, "fine", "9:1"), "empirical")
        fg = [c for c in R(ds, "grid", "9:1") if c["feasible"]]
        return (f"{max(c['tau_leg'] for c in f):.1f} (grid all feasible, both laws "
                f"{max(c['tau_leg'] for c in fg):.1f})")

    def c4(ds, gear):
        f = [c for c in R(ds, "grid", gear) if c["feasible"]]
        return f"{max(c['tau_abad'] for c in f):.1f}"

    def c4never(ds):
        ex = [c for c in R(ds, "fine", "6:1") if c["exists"]]
        return f"{str(all(c['tau_abad'] <= fig.ABAD_TAU_MAX for c in ex)).lower()} (max {max(c['tau_abad'] for c in ex):.1f})"

    def e1head(ds):
        out = []
        for scope in ("grid", "fine"):
            b = band_cells(R(ds, scope, "6:1"), "empirical")
            out.append(f"{max(c['lam_deg'] for c in b if c['psi'] <= T0 and c['lam_deg'] > 0):.2f}")
        return " grid / ".join(out) + " +sliver"

    def e1top(ds):
        f = feas_camb(R(ds, "fine", "9:1"), "empirical")
        t = max(f, key=lambda c: c["lam_deg"])
        return f"{t['lam_deg']:.2f} at {t['v_fwd']:.2f} m/s"

    def e1wt(ds):
        b = band_cells(R(ds, "fine", "6:1"))
        sok = [c for c in b if c["psi"] <= T0 and c["lam_deg"] > 0]
        return (f"{str(all(c['lam_deg'] <= phimax(c['v_fwd']) + 1e-9 for c in sok)).lower()} "
                f"(realised max {max(c['lam_deg'] for c in sok):.2f})")

    def at_most2(ds):
        f = [c for g in ("6:1", "9:1") for c in R(ds, "fine", g) if c["feasible"]]
        m = max(c["lam_deg"] for c in f)
        return f"{str(m <= 2.0 + 1e-9).lower()} (max {m:.3f})"

    def abad_never(ds):
        return str(all(c["binding"] != "abad" for g in ("6:1", "9:1") for s in ("grid", "fine")
                       for c in R(ds, s, g))).lower()

    def e3(ds):
        out = []
        for scope in ("grid", "fine"):
            if scope == "fine" and ds == "s84":
                out.append("n/a")
                continue
            ok, worst = True, 0.0
            for gear in ("35 stage", "6:1", "9:1"):
                cs = R(ds, scope, gear)
                fe = {(vk(c), lk(c)) for c in cs if c["law"] == "empirical" and c["feasible"]}
                fg = {(vk(c), lk(c)) for c in cs if c["law"] == "geometric" and c["feasible"]}
                ok &= fe == fg
                worst = max(worst, len(fe ^ fg) / max(1, len(fe | fg)))
            out.append(f"{str(ok).lower()} {100 * worst:.1f} %")
        return " grid / ".join(out) + " +sliver"

    def e4(ds):
        f = [c for g in ("6:1", "9:1") for c in R(ds, "fine", g) if c["feasible"]]
        return " / ".join(f"{100 * max(slew(c, w) for c in f):.2f} %" for w in W.values())

    def online(ds):
        out = []
        for gear in ("6:1", "9:1"):
            rm = rmin_cells(R(ds, "fine", gear), "empirical")
            r = [c["R"] / (c["v_fwd"] / T0) for c in rm]
            out.append(f"{gear} {min(r):.3f}-{max(r):.3f}")
        return ", ".join(out)

    def coarse(ds):
        rm = rmin_cells(R(ds, "grid", "9:1"), "empirical")
        return f"{min(c['R'] for c in rm):.1f}-{max(c['R'] for c in rm):.1f}"

    def nsliver(ds):
        n = len(DS[ds]["fine"]) - len(DS[ds]["grid"])
        ne = sum(c["law"] == "empirical" for c in DS[ds]["fine"]) - sum(c["law"] == "empirical" for c in DS[ds]["grid"])
        return f"{ne} emp ({n} both laws)"

    def thr(ds, scope):
        if scope == "grid" and ds == "s025w":
            ds = "s025"              # same grid
        return f"{max(x[0] for x in bps[(scope, ds, 'v_fwd 2-dec')]):.4f}"

    def ratio(ds):
        ex = [c for c in DS[ds]["grid"] if c["law"] == "empirical" and c["exists"]]
        r = [c["v_fwd"] / c["v_td"] for c in ex]
        return f"mean {np.mean(r):.3f} ({min(r):.3f}-{max(r):.3f})"

    rows = [
        ("6:1 span, grid (II-D p.5)", "0.53-0.58 (v_td 0.75-0.80)", lambda d: span_s(d, "grid", "6:1")),
        ("6:1 span, grid+sliver", "0.53-0.58 (v_td 0.75-0.80)", lambda d: span_s(d, "fine", "6:1")),
        ("6:1 span, geometric law, grid+sliver", "0.53-0.58", lambda d: span_s(d, "fine", "6:1", "geometric", False)),
        ("6:1 per-speed R_min, grid+sliver", "1.9-2.0 m", lambda d: rmin_s(d, "6:1")),
        ("band 6:1: no torque-feasible cambered cell", "true", band_zero),
        ("band 6:1: scrub above 1 deg, torque below (HEAD)", "true", lambda d: divide(d, "1deg")),
        ("band 6:1: scrub above phi_max, torque below (WT)", "true", lambda d: divide(d, "phi_max")),
        ("9:1 span, grid+sliver", "0.53-1.23", lambda d: span_s(d, "fine", "9:1", td=False)),
        ("9:1 span, geometric law, grid+sliver", "0.53-1.23", lambda d: span_s(d, "fine", "9:1", "geometric", False)),
        ("9:1 upper edge is the sweep's top", "true", top_is_sweep),
        ("9:1 covers the measured band", "true", covers),
        ("9:1 in-band per-speed R_min", "2.4-2.9 m", lambda d: rmin_s(d, "9:1", True)),
        ("9:1 largest eroded demand (feasible cambered)", "37.2", leg37),
        ("C4 max feasible demand 6:1 (grid, both laws)", "21.5", lambda d: c4(d, "6:1")),
        ("C4 max feasible demand 9:1 (grid, both laws)", "27.9", lambda d: c4(d, "9:1")),
        ("C4 never binds (existing cells <= 29.5)", "true", c4never),
        ("E1 (HEAD) band scrub-feasible bank <= 1.0", "1.0", e1head),
        ("E1 (HEAD) rising to ~2 deg at the 1.2 m/s edge", "~2 at 1.2", e1top),
        ("E1 (WT) band bank <= phi_max 1.2-1.4 (< 1.5)", "true", e1wt),
        ("C4 para (WT) feasible cells bank at most 2 deg", "true", at_most2),
        ("E2 AB/AD never binds", "true", abad_never),
        ("E3 identical feasible sets; area 0.0 %", "true 0.0 %", e3),
        ("E4 worst slew/flight, 34.6 / 14.66 rad/s", "well inside", e4),
        ("R_min on v/0.29 within resolution (R/line)", "~1", online),
        ("coarse-grid R_min artifacts (9:1 grid only)", "2.3-3.3 m", coarse),
        ("added sliver cells", "135", nsliver),
        ("threshold breakpoint, 2-dec v_fwd span, grid", "0.21", lambda d: thr(d, "grid")),
        ("threshold breakpoint, 2-dec v_fwd span, grid+sliver", "0.21", lambda d: thr(d, "fine")),
        ("v_fwd / v_td (existing grid cells)", "~0.73", ratio),
    ]
    w0 = max(len(r[0]) for r in rows)
    print(f"  {'statement':{w0}s} | {'paper':26s} | {'s84':34s} | {'s025':34s} | s025w")
    for name, paper, fn in rows:
        vals = [fn(d) for d in ("s84", "s025", "s025w")]
        print(f"  {name:{w0}s} | {paper:26s} | {vals[0]:34s} | {vals[1]:34s} | {vals[2]}")


def main():
    tee = Tee(sys.stdout)
    sys.stdout = tee
    try:
        run()
    finally:
        sys.stdout = tee.stream
        OUT_TXT.write_text("".join(tee.buf))
        print(f"wrote {OUT_TXT}")


if __name__ == "__main__":
    main()
