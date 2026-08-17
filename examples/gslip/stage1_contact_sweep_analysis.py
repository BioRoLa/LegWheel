"""Stage 1 lambda x alpha sweep analysis -- built BEFORE the sweep, on rails.

Stage 1 (opens Aug 20) validates the contact model: does the contact height
follow h = R_tread cos(lambda) - w_flat sin(lambda) + r_corner? This script is
the day-one analysis path, built from the section 47 lean-rig patterns so the
sweep starts with its carry-ins already enforced:

1. ACHIEVED, NOT COMMANDED. Commanded lambda predicts NEITHER side's achieved
   lambda to better than ~3 deg at kp 90, and the error is a variable
   left/right split, not a fixed undershoot (sections 39/47) -- a fixed-bias
   correction would be wrong in sign on one side. Per-leg gamma is read from
   every dump and both per-side achieved leans are reported and used.
2. THE DIAGONAL RESIDUAL IS SUBTRACTED IN THE OPEN. At zero command the legs
   sit at gamma = {-1.77, -1.04, -1.76, -1.04} (A/B/C/D) -- an {A,C}/{B,D}
   diagonal pattern, identical across runs (section 47). It is reported raw
   AND subtracted, so the correction is auditable, not silent.
3. UNFIT DUMPS ARE REFUSED. Section 39's contaminated capture (81.8% duplicate
   timestamps, 6 backward steps, from orphan recorders) read as plausible
   data. Gates: duplicate fraction <= 1%, zero backward steps -- a failing
   dump gets a verdict, not an analysis.
4. THE ANALYSER SELF-TESTS on synthetic known answers at startup and refuses
   to run if its own arithmetic is off (diag README rule).

The gate formula is imported from slip_rf_cambered.rolling_radius -- the SAME
function the models use, so a Stage 1 correction lands everywhere at once.
Section 42's warning stands: the rolling-radius drop (1 - cos lambda) is NOT
the measured body-height drop (sin lambda); this script never compares the
two.

Run (dumps land Aug 20; run with no args to see the expected schema):
    uv run python examples/gslip/stage1_contact_sweep_analysis.py \
        <dump.npz>:<lam_cmd_deg>:<alpha_cmd_deg> [...]
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np

from legwheel.models.slip_rf_cambered import rolling_radius

# lr pattern signs (A, B, C, D); left pair {A, D}, right pair {B, C}.
LR_SIGNS = np.array([+1.0, -1.0, -1.0, +1.0])
# Section 47's zero-command diagonal residual (deg), identical to 0.01 deg
# across all three lean-rig runs. Reported and subtracted, never silently.
DIAG_RESIDUAL_REF = np.array([-1.77, -1.04, -1.76, -1.04])

MAX_DUP_FRACTION = 0.01
MAX_BACKWARD_STEPS = 0

# Windows inherited from the lean rig schedule; override per sweep schedule.
PRE_WINDOW = (3.4, 3.9)
HOLD_WINDOW = (6.3, 7.4)


@dataclass
class HealthReport:
    dup_fraction: float
    n_backward: int

    @property
    def ok(self) -> bool:
        return (self.dup_fraction <= MAX_DUP_FRACTION
                and self.n_backward <= MAX_BACKWARD_STEPS)


def timestamp_health(t: np.ndarray) -> HealthReport:
    """Section 39's dump gate: orphan recorders interleave into one file and
    produce duplicate-heavy, backward-stepping clocks that otherwise read as
    plausible data."""
    t = np.asarray(t, float)
    if len(t) < 2:
        return HealthReport(dup_fraction=1.0, n_backward=0)
    dup = 1.0 - len(np.unique(t)) / len(t)
    back = int(np.sum(np.diff(t) < 0.0))
    return HealthReport(dup_fraction=float(dup), n_backward=back)


def per_side_achieved(gam_deg: np.ndarray,
                      subtract_residual: bool = True):
    """(left, right) achieved lean in world sense from per-leg gamma (A B C D,
    deg), lr pattern. Optionally subtracts the section 47 diagonal residual."""
    g = np.asarray(gam_deg, float)
    if subtract_residual:
        g = g - DIAG_RESIDUAL_REF
    world = g * LR_SIGNS
    left = 0.5 * (world[0] + world[3])
    right = 0.5 * (world[1] + world[2])
    return float(left), float(right)


def diagonal_residual(gam_pre_deg: np.ndarray) -> dict:
    """Zero-command residual split into the {A,C} / {B,D} diagonals, with the
    distance from the section 47 reference -- if this drifts, the residual is
    not the stable systematic it was and must be re-explained, not subtracted."""
    g = np.asarray(gam_pre_deg, float)
    return {
        "ac": float(0.5 * (g[0] + g[2])),
        "bd": float(0.5 * (g[1] + g[3])),
        "vs_ref": float(np.max(np.abs(g - DIAG_RESIDUAL_REF))),
    }


def gate_height(lam_deg: float) -> float:
    """Expected contact height (m) at achieved lean lam -- THE Stage 1 gate,
    from the shared seam function."""
    return float(rolling_radius(np.deg2rad(lam_deg)))


def _window_mean(t, v, lo, hi):
    m = (t >= lo) & (t <= hi)
    if not np.any(m):
        raise SystemExit(f"no samples in window [{lo}, {hi}] -- truncated run?")
    return v[m].mean(axis=0)


def _roll_from_quat(q: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = q.T
    return np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))


def analyse(path: str, lam_cmd_deg: float, alpha_cmd_deg: float,
            pre=PRE_WINDOW, hold=HOLD_WINDOW) -> dict:
    d = np.load(path)
    mt, motor = d["motor_t"], d["motor_deg"]        # (n, 4, 3): theta beta gamma
    ot, odom = d["odom_t"], d["odom"]               # (n, 10): xyz ... quat

    health_m, health_o = timestamp_health(mt), timestamp_health(ot)
    if not (health_m.ok and health_o.ok):
        return {"lam_cmd": lam_cmd_deg, "alpha_cmd": alpha_cmd_deg,
                "refused": f"UNFIT DUMP: motor dup {health_m.dup_fraction:.1%}"
                           f"/back {health_m.n_backward}, odom dup "
                           f"{health_o.dup_fraction:.1%}/back "
                           f"{health_o.n_backward}"}

    gam_pre = _window_mean(mt, motor[:, :, 2], *pre)
    gam_hold = _window_mean(mt, motor[:, :, 2], *hold)
    left, right = per_side_achieved(gam_hold)
    resid = diagonal_residual(gam_pre)

    z_pre = _window_mean(ot, odom[:, 2], *pre)
    z_hold = _window_mean(ot, odom[:, 2], *hold)
    roll = _roll_from_quat(odom[:, 6:10])

    return {
        "lam_cmd": lam_cmd_deg, "alpha_cmd": alpha_cmd_deg,
        "gam_pre": gam_pre, "gam_hold": gam_hold,
        "left": left, "right": right, "split": left - right,
        "residual": resid,
        "h_gate_left": gate_height(left), "h_gate_right": gate_height(right),
        "drop_meas_mm": (z_pre - z_hold) * 1e3,
        "roll_meas_deg": np.rad2deg(
            _window_mean(ot, roll, *hold) - _window_mean(ot, roll, *pre)),
    }


def _selftest() -> None:
    """Known answers, run before any dump is touched (diag README rule)."""
    clean = np.linspace(0.0, 10.0, 1001)
    assert timestamp_health(clean).ok
    dirty = clean.copy()
    dirty[100:300] = dirty[100]                     # 20% duplicates
    assert not timestamp_health(dirty).ok
    backward = clean.copy()
    backward[500] = backward[499] - 0.1
    assert not timestamp_health(backward).ok

    # lr pattern, exact 20-deg lean both sides, residual pre-added: recovers
    # (20, 20) after subtraction.
    gam = 20.0 * LR_SIGNS + DIAG_RESIDUAL_REF
    left, right = per_side_achieved(gam)
    assert abs(left - 20.0) < 1e-12 and abs(right - 20.0) < 1e-12

    assert abs(gate_height(0.0) - 0.145) < 1e-15   # the lambda = 0 anchor
    r = diagonal_residual(DIAG_RESIDUAL_REF)
    assert r["vs_ref"] == 0.0


def main(argv) -> None:
    _selftest()
    if not argv:
        print(__doc__)
        print("selftest: PASS. Expected npz schema (same as the lean rig): "
              "motor_t (n,), motor_deg (n, 4, 3), odom_t (n,), odom (n, 10).")
        return
    print(f"{'lam':>5} {'alp':>5} {'left':>7} {'right':>7} {'split':>6} | "
          f"{'h_gate L':>8} {'h_gate R':>8} | {'drop':>7} {'roll':>7} | "
          f"residual {{A,C}}/{{B,D}} (drift)")
    for arg in argv:
        path, lam, alpha = arg.rsplit(":", 2)
        r = analyse(path, float(lam), float(alpha))
        if "refused" in r:
            print(f"{r['lam_cmd']:4.0f}d {r['alpha_cmd']:4.0f}d  {r['refused']}")
            continue
        res = r["residual"]
        print(f"{r['lam_cmd']:4.0f}d {r['alpha_cmd']:4.0f}d "
              f"{r['left']:6.2f}d {r['right']:6.2f}d {r['split']:5.2f}d | "
              f"{r['h_gate_left'] * 1e3:7.2f}m {r['h_gate_right'] * 1e3:7.2f}m"
              f" | {r['drop_meas_mm']:6.2f} {r['roll_meas_deg']:6.2f}d | "
              f"{res['ac']:+.2f}/{res['bd']:+.2f} ({res['vs_ref']:.2f})")


if __name__ == "__main__":
    main(sys.argv[1:])
