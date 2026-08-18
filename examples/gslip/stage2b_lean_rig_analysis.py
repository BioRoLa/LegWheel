"""Close the coronal statics validation bracket with PER-LEG achieved leans.

stage2b_coronal_statics.py validated the contact mechanism against section 33's
measured body roll at a CONSTANT 0.80 (achieved lean) to 0.94 (commanded lean)
-- constant ratio, exact mirror, so the mechanism is right and one input is
off. The suspect input is the achieved lean: section 39 showed the tracking
error is a LEFT/RIGHT SPLIT, which a symmetric solve cannot represent, and
section 33 logged only the pooled undershoot.

This script consumes the re-run dumps (camber_cycle.sh, lr pattern, per-leg
gamma recorded) and, for each run, WITHIN THE SAME RUN:

  measured : body roll and ride drop between the pre-lean window (wheeled,
             lambda = 0, after the fold settles) and the lean-hold window
             (after the lean settles, before rolling starts)
  predicted: solve_pose_asym fed the same windows' per-side achieved leans
             (left = +mean(gamma_A, gamma_D), right = -mean(gamma_B, gamma_C)
             for the lr pattern)

Zero free parameters, no cross-run baseline. If per-side leans land the ratio
near 1.0 the seam is validated outright; a surviving residual is a real
modelling gap to know about before Stage 1 leans on the same geometry.

Windows come from camber_roll.py's schedule (settle 1 + fold 3 + lean 2 +
settle 1.5): pre-lean [3.4, 3.9] s, hold [6.3, 7.4] s after the trigger.

Run:
    uv run python examples/gslip/stage2b_lean_rig_analysis.py \
        /home/alexc/camber_dumps/camber_lean10_pl.npz:10 [dump:lam ...]
"""

from __future__ import annotations

import sys

import numpy as np

from stage2b_coronal_statics import solve_pose_asym

PRE_WINDOW = (3.4, 3.9)
HOLD_WINDOW = (6.3, 7.4)
# lr pattern signs (A, B, C, D); left pair {A, D}, right pair {B, C}.
LR_SIGNS = np.array([+1.0, -1.0, -1.0, +1.0])


def _window_mean(t: np.ndarray, v: np.ndarray, lo: float, hi: float):
    m = (t >= lo) & (t <= hi)
    if not np.any(m):
        raise SystemExit(f"no samples in window [{lo}, {hi}] -- truncated run?")
    return v[m].mean(axis=0)


def _roll_from_quat(q: np.ndarray) -> np.ndarray:
    """Roll angle (rad) about x from [qx, qy, qz, qw] rows."""
    qx, qy, qz, qw = q.T
    return np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))


def analyse(path: str, lam_cmd_deg: float, fold_settle: float = 0.0) -> dict:
    """fold_settle: the run's --fold-settle dwell (s). Shifts both windows so
    they track the schedule; with fold_settle > 0 the pre window sits in the
    SETTLED folded-unleaned dwell rather than the fold transient the original
    schedule left it in (log section 62/63)."""
    d = np.load(path)
    mt, motor = d["motor_t"], d["motor_deg"]        # (n, 4, 3): theta beta gamma
    ot, odom = d["odom_t"], d["odom"]               # (n, 10): xyz ... quat

    if fold_settle > 0.0:
        # Settled dwell is [t_fold + ~1.5, t_fold + fold_settle]; hold shifts
        # rigidly with the schedule.
        pre = (4.0 + max(fold_settle - 1.5, 0.4), 4.0 + fold_settle - 0.1)
        hold = (HOLD_WINDOW[0] + fold_settle, HOLD_WINDOW[1] + fold_settle)
    else:
        pre, hold = PRE_WINDOW, HOLD_WINDOW

    gam_pre = _window_mean(mt, motor[:, :, 2], *pre)
    gam_hold = _window_mean(mt, motor[:, :, 2], *hold)

    z_pre = _window_mean(ot, odom[:, 2], *pre)
    z_hold = _window_mean(ot, odom[:, 2], *hold)
    roll = _roll_from_quat(odom[:, 6:10])
    roll_pre = _window_mean(ot, roll, *pre)
    roll_hold = _window_mean(ot, roll, *hold)

    # World-sense per-side achieved leans (deg).
    world = gam_hold * LR_SIGNS
    left = 0.5 * (world[0] + world[3])
    right = 0.5 * (world[1] + world[2])

    z0, _ = solve_pose_asym(0.0, 0.0)
    z, rho = solve_pose_asym(np.deg2rad(left), np.deg2rad(right))

    return {
        "lam_cmd": lam_cmd_deg,
        "gam_pre": gam_pre, "gam_hold": gam_hold,
        "left": left, "right": right, "split": left - right,
        "roll_meas": np.rad2deg(roll_hold - roll_pre),
        "drop_meas": (z_pre - z_hold) * 1e3,
        "roll_pred": np.rad2deg(rho),
        "drop_pred": (z0 - z) * 1e3,
    }


def main(argv) -> None:
    if not argv:
        raise SystemExit(__doc__)
    print(f"{'cmd':>5} {'left ach':>9} {'right ach':>9} {'split':>7} | "
          f"{'roll pred':>9} {'roll meas':>9} {'ratio':>6} | "
          f"{'drop pred':>9} {'drop meas':>9} {'ratio':>6}")
    for arg in argv:
        parts = arg.split(":")
        if len(parts) >= 3 and parts[-2].replace(".", "").isdigit():
            path, lam, fs = ":".join(parts[:-2]), parts[-2], parts[-1]
        else:
            path, lam = arg.rsplit(":", 1)
            fs = "0"
        r = analyse(path, float(lam), fold_settle=float(fs))
        rr = r["roll_pred"] / r["roll_meas"] if r["roll_meas"] else float("nan")
        dr = r["drop_pred"] / r["drop_meas"] if r["drop_meas"] else float("nan")
        print(f"{r['lam_cmd']:4.0f}d {r['left']:8.2f}d {r['right']:8.2f}d "
              f"{r['split']:6.2f}d | {r['roll_pred']:8.3f}d {r['roll_meas']:8.3f}d "
              f"{abs(rr):6.2f} | {r['drop_pred']:8.2f} {r['drop_meas']:8.2f} "
              f"{abs(dr):6.2f}")
        print(f"      per-leg gamma at hold (A B C D): "
              + " ".join(f"{g:+.2f}" for g in r["gam_hold"])
              + f"   pre-lean residual: "
              + " ".join(f"{g:+.2f}" for g in r["gam_pre"]))


if __name__ == "__main__":
    main(sys.argv[1:])
