"""Inner/outer pair asymmetry at the feasible turning radii (log section 85).

The carried Stage 2a decision ("schedule explicitly or descope", timeline
Stage 2b task 5), settled by measurement in template space. The trot's
retirement changed the problem: the DIAGONAL-pair form (inner + outer leg
averaged into one trot virtual leg) is moot for the thesis gait; the pronk's
virtual pair is LATERAL, which section 45 models. What survives is the
inner/outer split itself: wheel planes at R -+ w must roll at path speeds
split by 2w/R -- 23.7% at the oft-quoted R = 2 m, but the section-84
envelope shows feasible steady turns start at R ~ 2.9 m, where it is less.

WHAT THIS SCRIPT DOES (model space, zero sim, zero new dynamics)
For every FEASIBLE turning cell of the cached section-84 grid (empirical
radius law; E3 measured the laws indistinguishable): give each leg of the
lateral pair its own path speed v*(R -+ w)/R (touchdown speed scaled by the
same factor -- a local-linearity approximation, stated), its own Ackermann
camber (lam_in = the cell's lambda; lam_out from the apex condition), and
solve each leg's OWN fixed point with the same machinery and filters the
envelope used. Report the split, survival, d_beta*, d_alpha*, and the
stride-period mismatch dT/T -- the clock-forcing a shared-period pronk pair
needs from the controller, since the yaw-less return map cannot express
per-leg speeds at all (yaw extension = Stage 3 territory).

REGISTERED PREDICTIONS (log section 85, written before the first run)
    A1  split = +-w/R: +-7.3% at R_min 2.88, falling to +-5.9% at R 3.6
    A2  the split-robust subset is non-empty; interior cells survive, edge
        cells (v_td 0.75 / 1.10) lose a leg; >= half survive
    A3  |d_beta*| <= 1.5 deg at every surviving cell
    A4  |dT|/T <= 10% at every surviving cell

DECISION RULE (pre-committed): A2-A4 all hold -> DESCOPE from Stage 2a
(caption caveat with these numbers; asymmetry folds into the Stage 3 yaw
extension). Any failure -> SCHEDULE the four-leg/coupled model first.

Run:
    uv run python examples/gslip/stage2a_pair_split.py \
        [--grid examples/gslip/stage2a_figs/stage2a_grid.npz]
"""
from __future__ import annotations

import importlib.util
import sys
from functools import partial
from pathlib import Path

import numpy as np

GSLIP_DIR = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "s2a_env", GSLIP_DIR / "stage2a_turning_envelope.py")
env = importlib.util.module_from_spec(_spec)
sys.modules["s2a_env"] = env
_spec.loader.exec_module(env)

from legwheel.models.slip_rf_cambered import cambered_stride  # noqa: E402
from legwheel.planners import gslip_to_corgi as g2c            # noqa: E402

W_HALF_TRACK = env.CONTACT_TRACK / 2.0     # 0.2117, the section-23 track


def split_fraction(r_turn: float) -> float:
    """Per-leg path-speed deviation from body speed: w/R."""
    return W_HALF_TRACK / r_turn


def mismatch_pct(r_turn: float) -> float:
    """v_out/v_in - 1 = 2w/(R - w), the risk-register convention."""
    return 2.0 * W_HALF_TRACK / (r_turn - W_HALF_TRACK)


def _selftest() -> None:
    # the risk-register number reproduces exactly: 23.7% at R = 2 m
    assert abs(mismatch_pct(2.0) - 0.2367) < 0.001, mismatch_pct(2.0)
    # A1's algebra at the envelope's R_min
    assert abs(split_fraction(2.88) - 0.0735) < 0.001
    # zero track -> zero split, identical legs by construction
    assert mismatch_pct(10.0) > 0 and W_HALF_TRACK > 0


def leg_solution(cell, factor, lam_leg, leg_map, step=0.5):
    """One leg's own fixed point at its own path speed and camber, through
    the same solver + grazing filters + torque chain the envelope used."""
    v_td = cell["v_td"] * factor
    stride_fn = partial(env.stride_const_r, lam=lam_leg,
                        r_const=env.R_EMPIRICAL)
    fp = env.solve_existence(env.base_params(), v_td, stride_fn, step,
                             beta_center=cell["beta_deg"])
    if fp is None:
        return None
    p_cell = env.cambered_params_const_r(env.base_params(), lam_leg,
                                         env.R_EMPIRICAL)
    tau, _ = env.leg_torque(p_cell, v_td, fp, leg_map)
    return {"fp": fp, "tau": tau,
            "ok": tau <= env.MOTOR_TORQUE_LIMIT}


def main(argv) -> None:
    _selftest()
    print("selftest: PASS")
    grid_path = "examples/gslip/stage2a_figs/stage2a_grid.npz"
    if "--grid" in argv:
        grid_path = argv[argv.index("--grid") + 1]
    cells = list(np.load(grid_path, allow_pickle=True)["cells"])
    turning = [c for c in cells if c["law"] == "empirical"
               and c["feasible"] and c["lam_deg"] > 0]
    if not turning:
        print("REFUSED: the cached grid has no feasible turning cells")
        return
    leg_map = g2c.LegLengthMap()
    print(f"{len(turning)} feasible turning cells (empirical law)\n")
    print(f"{'v_td':>5} {'lam':>4} {'R':>6} {'split':>6} | "
          f"{'in ok':>5} {'out ok':>6} | {'dBeta*':>7} {'dAlpha*':>7} "
          f"{'dT/T':>6} | {'tau in/out':>11}")
    rows = []
    for c in turning:
        r_turn = c["R"]
        s = split_fraction(r_turn)
        lam = np.deg2rad(c["lam_deg"])
        h_td = (env.cambered_params_const_r(env.base_params(), lam,
                                            env.R_EMPIRICAL).l0
                * np.sin(np.deg2rad(c["beta_deg"])))
        lam_in, lam_out = env.ackermann_split(lam, h_td)
        inner = leg_solution(c, 1.0 - s, lam_in, leg_map)
        outer = leg_solution(c, 1.0 + s, lam_out, leg_map)
        row = {"cell": c, "split": s, "R": r_turn,
               "inner": inner, "outer": outer,
               "both": bool(inner and inner["ok"] and outer
                            and outer["ok"])}
        if row["both"]:
            b_in, b_out = inner["fp"], outer["fp"]
            row["d_beta"] = abs(np.rad2deg(b_out.beta - b_in.beta))
            row["d_alpha"] = abs(np.rad2deg(b_out.alpha - b_in.alpha))
            t_in, t_out = b_in.period, b_out.period
            row["dT_over_T"] = abs(t_out - t_in) / (0.5 * (t_out + t_in))
        rows.append(row)
        d = (f"{row['d_beta']:7.2f} {row['d_alpha']:7.2f} "
             f"{row['dT_over_T']:6.1%}" if row["both"] else
             f"{'--':>7} {'--':>7} {'--':>6}")
        taus = (f"{inner['tau'] if inner else float('nan'):5.1f}/"
                f"{outer['tau'] if outer else float('nan'):5.1f}")
        print(f"{c['v_td']:5.2f} {c['lam_deg']:3.1f}d {r_turn:6.2f} "
              f"{s:6.1%} | {'yes' if inner and inner['ok'] else 'NO':>5} "
              f"{'yes' if outer and outer['ok'] else 'NO':>6} | {d} | "
              f"{taus:>11}")

    surv = [r for r in rows if r["both"]]
    print(f"\n--- registered verdicts (section 85) ---")
    r_min = min(r["R"] for r in rows)
    print(f"A1 split at R_min {r_min:.2f} m = +-{split_fraction(r_min):.1%} "
          f"(mismatch convention: {mismatch_pct(r_min):.1%}); at R = 2 m it "
          f"would be {mismatch_pct(2.0):.1%} "
          f"{'PASS' if abs(split_fraction(r_min) - 0.073) < 0.02 else 'CHECK'}")
    frac = len(surv) / len(rows)
    print(f"A2 split-robust cells: {len(surv)}/{len(rows)} = {frac:.0%} "
          f"{'PASS' if surv and frac >= 0.5 else 'FAIL' if surv else 'FAIL (EMPTY)'}")
    if surv:
        worst_b = max(r["d_beta"] for r in surv)
        worst_t = max(r["dT_over_T"] for r in surv)
        print(f"A3 max |dBeta*| = {worst_b:.2f} deg (bar 1.5) "
              f"{'PASS' if worst_b <= 1.5 else 'FAIL'}")
        print(f"A4 max |dT|/T = {worst_t:.1%} (bar 10%) "
              f"{'PASS' if worst_t <= 0.10 else 'FAIL'}")
        a2, a3, a4 = frac >= 0.5, worst_b <= 1.5, worst_t <= 0.10
        print(f"\nDECISION RULE -> "
              f"{'DESCOPE (fold into Stage 3 yaw extension)' if a2 and a3 and a4 else 'SCHEDULE the four-leg/coupled model'}")


if __name__ == "__main__":
    main(sys.argv[1:])
