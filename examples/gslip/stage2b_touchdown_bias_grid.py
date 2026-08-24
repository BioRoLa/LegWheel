"""Does the plant's caudal touchdown bias change the survival verdict? The
Stage 2b touchdown-bias campaign, at the PLANT's orbit.

WHY THIS EXISTS. Issue #20: the plant lands ~70% through the commanded sweep --
a caudal touchdown offset the controller never commanded and never sees. The
model campaigns so far scored the orbit with the EXECUTED input equal to the
COMMANDED one. This driver sweeps a deliberate execution-side bias
dbeta in {0, +1.6, +3.2, +4.75, +6.3, +7.92} deg (caudal = +dbeta; +4.75 is
issue #20's physical landing offset, +7.92 the tracking lag) over the S184/S185
NEAR and FAR survival grids with the S135 controller set, at the plant orbit
RE-SOLVED under the current radius law of record. The S135 table predates the
S183/S184 crown fix and the S185 law flip, so the dbeta = 0 column doubles as
the first post-fix re-run of that gate.

Injection semantics of record: EXECUTION-side (the controller computes u about
the unbiased orbit, gain and clamp included; the leg lands at u + [dbeta,0,0]).
--command-side runs the S57 pattern instead (u* itself shifted, deadbeat trims
about the biased nominal) as the cross-check arm.

Outputs are CSVs only (figures come from the vault-side generator):
  survival_summary.csv  k/16 per bias x grid x controller
  cells.csv             per-cell steps/peak/dlam (heatmap raw data)
  growth.csv            max|eig(J_x)| at the biased executed input + measured
                        rho-seed growth factor, per bias level
  basin_polar.csv       r(theta) rays at the best bias level, PD+deadbeat

Run:
    uv run python examples/gslip/stage2b_touchdown_bias_grid.py \
        [--quick] [--command-side] [--out DIR]
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models import coronal_bip as bip
from legwheel.models.cambered_return_map import PairParams, RollPD
from legwheel.models.gslip import GSlipFailure

MAX_STEPS = 12
ABAD_CEILING = 44.25          # N.m per joint, S37

# The continuation seed -- stage2b_deadbeat_at_plant's reference point.
REF_VT, REF_V, REF_BETA = 0.70, 1.19, 80.75
REF_ALPHA_DEG = 40.74
L0_CORGI = 0.2931
PLANT_VTD = 0.41

# S135's plant-orbit numbers, for the side-by-side (pre-crown-fix; the
# re-solved values below are the values of record, these are the reference).
S135_JU_SV = (2.893, 0.332, 1.88e-5)

# S184/S185 grids, unchanged for comparability.
GRIDS = {
    "NEAR": (np.deg2rad([-3.0, -1.5, 1.5, 3.0]), [-0.15, -0.08, 0.08, 0.15]),
    "FAR": (np.deg2rad([-12.0, -6.0, 6.0, 12.0]), [-0.6, -0.3, 0.3, 0.6]),
}
QUICK_GRIDS = {
    "NEAR": (np.deg2rad([-3.0, 3.0]), [-0.15, 0.15]),
    "FAR": (np.deg2rad([-12.0, 12.0]), [-0.6, 0.6]),
}

# Bias axis (deg, caudal = +dbeta): 0 = baseline re-run of S135's gate;
# +4.75 = issue #20 physical landing offset; +7.92 = tracking lag.
BIAS_LEVELS_DEG = (0.0, 1.6, 3.2, 4.75, 6.3, 7.92)
QUICK_BIAS_LEVELS_DEG = (0.0, 4.75)

# S57 clamp discipline for the deadbeat command (stage2b_budget_gain_sweep).
U_LO = np.array([np.deg2rad(40.0), -np.deg2rad(30.0), -np.deg2rad(30.0)])
U_HI = np.array([np.deg2rad(89.0), +np.deg2rad(30.0), +np.deg2rad(30.0)])
U_LIMITS = (U_LO, U_HI)

# Basin rays, stage2b_budget_gain_sweep's scales (r = 1 is the FAR inner ring).
N_RAYS = 16
RAY_RHO_SCALE = np.deg2rad(6.0)
RAY_DRHO_SCALE = 0.3
RAY_R_MAX = 3.0

RHO_SEED = 1e-3               # rad, the measured-growth seed
GROWTH_STRIDES = 5

CONTROLLERS = ("passive", "roll_pd", "pd_deadbeat")


def repo_commit() -> str:
    """HEAD commit by reading .git files directly -- never invokes git
    (the working tree carries a CRLF trap; read-only file access is safe)."""
    root = Path(__file__).resolve().parents[2]
    try:
        head = (root / ".git" / "HEAD").read_text().strip()
        if not head.startswith("ref: "):
            return head[:12]
        ref = head[5:].strip()
        ref_file = root / ".git" / ref
        if ref_file.exists():
            return f"{ref_file.read_text().strip()[:12]} ({ref})"
        packed = root / ".git" / "packed-refs"
        if packed.exists():
            for line in packed.read_text().splitlines():
                if line.endswith(" " + ref):
                    return f"{line.split()[0][:12]} ({ref})"
        return f"unresolved ({ref})"
    except OSError:
        return "unknown"


def resolve_orbit(pp: PairParams):
    """Re-solve the plant orbit under the CURRENT radius law of record.

    Continuation block reused verbatim from stage2b_deadbeat_at_plant.py
    (GATE 1): v~0.70 reference seed walked down to v_td 0.41, each point
    screened to apex h in [0.27, 0.40] m to stay off the ballistic-hop branch.
    """
    x_seed = [REF_V * np.cos(np.deg2rad(REF_ALPHA_DEG)), 0.0, 0.32, 0.0, 0.0]
    u_beta = np.deg2rad(REF_BETA)
    chain = []
    for v in np.arange(REF_V, PLANT_VTD - 1e-9, -0.03):
        seed = list(x_seed)
        seed[0] = v * np.cos(np.deg2rad(REF_ALPHA_DEG))
        try:
            x, u = crm.solve_periodic(pp, seed, [u_beta, 0.0, 0.0])
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            continue
        if not (0.27 <= x[2] <= 0.40):
            continue
        x_seed, u_beta = list(x), float(u[0])
        chain.append((float(v), np.array(x), np.array(u)))
    if not chain:
        raise SystemExit("continuation never held the physical branch -- "
                         "no orbit to bias; stop.")
    return chain


def measured_growth(pp: PairParams, x_star, u_exec,
                    rho_seed: float = RHO_SEED,
                    n_strides: int = GROWTH_STRIDES) -> float:
    """Geometric growth factor of |rho| per stride for a small seed roll,
    open loop at the EXECUTED input -- the S42/S45 roll_growth_per_bounce
    pattern on the apex map. nan when fewer than two ratios complete."""
    x = np.asarray(x_star, float).copy()
    x[3] += rho_seed
    mags = [abs(x[3])]
    for _ in range(n_strides):
        try:
            x = crm.apex_map(pp, x, u_exec)
        except GSlipFailure:
            break
        mags.append(abs(x[3]))
    mags = [m for m in mags if m > 0.0]
    if len(mags) < 3:
        return float("nan")
    factors = np.array(mags[1:]) / np.array(mags[:-1])
    return float(np.exp(np.mean(np.log(factors))))


def scan(pp, x_star, u_star, rho_g, drho_g, controller, k_db, bias_vec,
         command_side: bool):
    """One basin_scan cell-grid for one (bias, grid, controller) triple."""
    ctrl = RollPD() if controller in ("roll_pd", "pd_deadbeat") else None
    gain = k_db if controller == "pd_deadbeat" else None
    limits = U_LIMITS if gain is not None else None
    if command_side:
        u_b = u_star.copy()
        u_b[0] += bias_vec[0]
        return crm.basin_scan(pp, x_star, u_b, rho_g, drho_g,
                              ctrl_proto=ctrl, gain=gain,
                              max_steps=MAX_STEPS, u_limits=limits)
    return crm.basin_scan(pp, x_star, u_star, rho_g, drho_g,
                          ctrl_proto=ctrl, gain=gain,
                          max_steps=MAX_STEPS, u_limits=limits,
                          u_exec_bias=bias_vec)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true",
                    help="smoke run: 2x2 grids, bias levels {0, 4.75} deg, "
                         "8 basin rays")
    ap.add_argument("--command-side", action="store_true",
                    help="S57 cross-check arm: bias the COMMANDED u* "
                         "(deadbeat trims about it) instead of the executed "
                         "input")
    ap.add_argument("--out", default="./stage2b_bias_out",
                    help="output directory for the CSVs")
    args = ap.parse_args(argv)

    grids = QUICK_GRIDS if args.quick else GRIDS
    bias_levels = QUICK_BIAS_LEVELS_DEG if args.quick else BIAS_LEVELS_DEG
    n_rays = 8 if args.quick else N_RAYS
    mode = "command-side" if args.command_side else "execution-side"

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(__doc__.split("Run:")[0].rstrip())
    print()
    print("=" * 74)
    print("ENVIRONMENT AND ORBIT OF RECORD")
    print("=" * 74)
    print(f"  LegWheel commit: {repo_commit()}")
    print(f"  radius law of record: {bip.RADIUS_LAW_DEFAULT!r} "
          f"(LEGWHEEL_RADIUS_LAW="
          f"{os.environ.get('LEGWHEEL_RADIUS_LAW', '<unset>')})")
    print(f"  injection semantics: {mode}"
          + ("  [cross-check arm]" if args.command_side else "  [primary]"))
    print(f"  bias levels (deg, caudal=+): {list(bias_levels)}")
    print(f"  quick: {args.quick}   out: {out_dir}")
    print()

    pp = PairParams()
    chain = resolve_orbit(pp)
    v_pt, x_star, u_star = chain[-1]
    print("  continuation held %d points down to v_td %.3f m/s (v~ %.3f)"
          % (len(chain), v_pt, v_pt / np.sqrt(pp.g * L0_CORGI)))
    print("  x* = vx %.4f  vy %+.4f  h %.4f  rho %+.4f deg  drho %+.4f"
          % (x_star[0], x_star[1], x_star[2], np.rad2deg(x_star[3]),
             x_star[4]))
    print("  u* = beta %.2f deg  lam_l %.2f  lam_r %.2f"
          % tuple(np.rad2deg(u_star)))

    jx, ju = crm.jacobians(pp, x_star, u_star)
    sv = np.linalg.svd(ju, compute_uv=False)
    print("  J_u singular values: " + np.array2string(sv, precision=4))
    print("  S135 (pre-crown-fix) reference: %s" % (S135_JU_SV,))
    kept = int(np.sum(sv / sv[0] >= 1e-2))
    print("  rcond=1e-2 keeps %d of 3 input directions" % kept)
    k_db = crm.deadbeat_gain(jx, ju)         # rcond 1e-2
    print("  |K| row norms (beta, lam_l, lam_r): "
          + np.array2string(np.linalg.norm(k_db, axis=1), precision=3))
    print("  unbiased max|eig(J_x)|: %.4f"
          % float(np.max(np.abs(np.linalg.eigvals(jx)))))
    print()

    # -- (a) survival grids ------------------------------------------------
    print("=" * 74)
    print("SURVIVAL k/n (%d strides), %s bias" % (MAX_STEPS, mode))
    print("=" * 74)
    summary_rows = []
    cell_rows = []
    totals = {}                  # bias_deg -> total PD+deadbeat k over grids
    n_cells = {g: len(rg) * len(dg) for g, (rg, dg) in grids.items()}
    head = f"{'bias':>7} |"
    for g in grids:
        head += f"  {g}: " + " ".join(f"{c:>4}" for c in
                                      ("pas", "pd", "db")) + " |"
    print(head)
    for bias_deg in bias_levels:
        bias_vec = np.array([np.deg2rad(bias_deg), 0.0, 0.0])
        line = f"{bias_deg:+6.2f}d |"
        for gname, (rho_g, drho_g) in grids.items():
            for cname in CONTROLLERS:
                res = scan(pp, x_star, u_star, rho_g, drho_g, cname, k_db,
                           bias_vec, args.command_side)
                k = int(np.count_nonzero(res.steps >= MAX_STEPS))
                summary_rows.append(dict(
                    mode=mode, bias_deg=bias_deg, grid=gname,
                    controller=cname, k=k, n=n_cells[gname],
                    survival_frac=res.survival_fraction,
                    mean_steps=float(res.steps.mean()),
                    peak_max=res.peak_max,
                    peak_max_surviving=res.peak_max_surviving,
                    dlam_max_deg=float(np.rad2deg(res.dlam.max()))))
                for i, r in enumerate(rho_g):
                    for j, dr in enumerate(drho_g):
                        cell_rows.append(dict(
                            mode=mode, bias_deg=bias_deg, grid=gname,
                            controller=cname,
                            rho_deg=float(np.rad2deg(r)), drho=float(dr),
                            steps=int(res.steps[i, j]),
                            peak=float(res.peak[i, j]),
                            dlam_deg=float(np.rad2deg(res.dlam[i, j]))))
                line += f" {k:>4}"
                if cname == "pd_deadbeat":
                    totals[bias_deg] = totals.get(bias_deg, 0) + k
                    if res.peak_max_surviving > ABAD_CEILING:
                        line += "!"     # survivor peak over the ABAD ceiling
            line += " |"
        print(line)
    print("  (counts are k of %s cells; '!' marks a surviving-cell peak over "
          "the %.2f N.m ABAD ceiling)" % (dict(n_cells), ABAD_CEILING))
    print()

    with open(out_dir / "survival_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary_rows[0]))
        w.writeheader()
        w.writerows(summary_rows)
    with open(out_dir / "cells.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(cell_rows[0]))
        w.writeheader()
        w.writerows(cell_rows)

    # -- (b) growth per bounce vs bias -------------------------------------
    print("=" * 74)
    print("GROWTH PER BOUNCE vs bias (open loop at the executed input)")
    print("=" * 74)
    growth_rows = []
    print(f"{'bias':>7} {'max|eig(J_x)|':>14} {'measured rho growth':>20}")
    for bias_deg in bias_levels:
        u_exec = u_star + np.array([np.deg2rad(bias_deg), 0.0, 0.0])
        try:
            jx_b, _ = crm.jacobians(pp, x_star, u_exec)
            eig_max = float(np.max(np.abs(np.linalg.eigvals(jx_b))))
        except GSlipFailure:
            eig_max = float("nan")
        g_meas = measured_growth(pp, x_star, u_exec)
        growth_rows.append(dict(bias_deg=bias_deg, max_abs_eig_jx=eig_max,
                                measured_rho_growth=g_meas))
        print(f"{bias_deg:+6.2f}d {eig_max:>14.4f} {g_meas:>20.4f}")
    print("  (J_x by central difference about x* at u*+bias; measured factor "
          "is the geometric mean |rho| ratio over %d strides from a %.4f rad "
          "seed)" % (GROWTH_STRIDES, RHO_SEED))
    print()
    with open(out_dir / "growth.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(growth_rows[0]))
        w.writeheader()
        w.writerows(growth_rows)

    # -- (c) basin rays at the best bias level ------------------------------
    best_bias = max(totals, key=lambda b: (totals[b], -b))
    print("=" * 74)
    print("BASIN r(theta) at the best bias level (PD + deadbeat, clamped)")
    print("=" * 74)
    print("  best bias by total PD+deadbeat survival: %+.2f deg "
          "(total k %d)" % (best_bias, totals[best_bias]))
    print("  scales: rho %.1f deg / drho %.2f rad/s per unit r, r_max %.1f"
          % (np.rad2deg(RAY_RHO_SCALE), RAY_DRHO_SCALE, RAY_R_MAX))
    angles = np.linspace(0.0, 2 * np.pi, n_rays, endpoint=False)
    bias_vec = np.array([np.deg2rad(best_bias), 0.0, 0.0])
    if args.command_side:
        u_b = u_star.copy()
        u_b[0] += bias_vec[0]
        r = crm.basin_radius(pp, x_star, u_b, angles,
                             RAY_RHO_SCALE, RAY_DRHO_SCALE,
                             ctrl_proto=RollPD(), gain=k_db,
                             max_steps=MAX_STEPS, r_max=RAY_R_MAX,
                             u_limits=U_LIMITS)
    else:
        r = crm.basin_radius(pp, x_star, u_star, angles,
                             RAY_RHO_SCALE, RAY_DRHO_SCALE,
                             ctrl_proto=RollPD(), gain=k_db,
                             max_steps=MAX_STEPS, r_max=RAY_R_MAX,
                             u_limits=U_LIMITS, u_exec_bias=bias_vec)
    print("  r(theta) = " + " ".join(f"{v:4.2f}" for v in r)
          + f"   min {r.min():.2f} mean {r.mean():.2f}")
    with open(out_dir / "basin_polar.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mode", "bias_deg", "theta_rad", "r",
                    "rho_scale_deg", "drho_scale", "r_max"])
        for th, rv in zip(angles, r):
            w.writerow([mode, best_bias, f"{th:.6f}", f"{rv:.4f}",
                        f"{np.rad2deg(RAY_RHO_SCALE):.1f}",
                        f"{RAY_DRHO_SCALE:.2f}", f"{RAY_R_MAX:.1f}"])

    print()
    print("=" * 74)
    print("CSVs written to %s:" % out_dir)
    for name in ("survival_summary.csv", "cells.csv", "growth.csv",
                 "basin_polar.csv"):
        print("  " + name)
    print("Figures come from the vault-side generator, not this driver.")
    print("=" * 74)


if __name__ == "__main__":
    main()
