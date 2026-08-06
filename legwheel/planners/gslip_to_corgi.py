"""Map the SLIP-RF template onto Corgi joint commands (template -> anchor).

Phase 0 established that the Corgi's foot-arc center O_r lies exactly on the
leg axis for every theta, so the mapping is two independent scalar inversions
rather than a 2-D solve:

    leg length   l  ->  theta_c   by inverting the hip-to-arc-center distance
    leg angle  phi  ->  beta_c    = -phi

The sign follows from the geometry: SLIP-RF puts the foot center at
(l - r)*(sin phi, cos phi) from the mass, so the mass-to-center bearing is
atan2(-cos phi, -sin phi), while the Corgi's is -90deg + beta_c. Equating
gives beta_c = -phi, and `test_gslip_to_corgi.py` checks it by round-tripping
through forward kinematics.

CONTACT RADIUS -- unresolved between the two codebases:
  * this package's `LegModel.rim_point` models the tyre envelope as three arcs
    of radius foot_radius = 0.145 m (tread 0.130 + corner 0.015)
  * corgi_utils/src/leg_model.cpp `contact_map` models five linkage arcs, with
    the bottom one of radius r = 0.019 m about G
Those place the ground 26 mm apart. Everything from Phase 0 onward uses the
tyre model, so it is the default here, exposed as `contact_radius` so the
alternative can be tested without touching the rest of the pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from legwheel.config import RobotParams
from legwheel.models.leg_model import LegModel
from legwheel.planners.gslip_template import StrideTemplate

# Foot-arc angular half-span (LegModel.rim_point switches arcs at +/-40 deg).
FOOT_ARC_HALF_SPAN_DEG = 40.0


class WorkspaceViolation(Exception):
    """A commanded pose falls outside the Corgi's reachable range."""


@dataclass
class JointTrajectory:
    """Per-leg joint commands over one stride, plus the guard results."""

    t: np.ndarray
    theta: np.ndarray  # leg extension (rad)
    beta: np.ndarray  # sagittal swing (rad)
    gamma: np.ndarray  # ABAD, held at zero for planar work
    in_stance: np.ndarray
    contact_alpha: np.ndarray  # rolling angle on the foot arc (deg)

    @property
    def period(self) -> float:
        return float(self.t[-1])

    def guard_report(self) -> dict:
        """Every constraint the trajectory has to satisfy, with margins."""
        theta_deg = np.rad2deg(self.theta)
        beta_deg = np.rad2deg(self.beta)
        arc_deg = np.abs(self.contact_alpha)
        return {
            "theta_min_deg": float(theta_deg.min()),
            "theta_max_deg": float(theta_deg.max()),
            "theta_ok": bool(
                theta_deg.min() >= RobotParams.MIN_THETA_DEG
                and theta_deg.max() <= RobotParams.MAX_THETA_DEG
            ),
            "beta_abs_max_deg": float(np.abs(beta_deg).max()),
            "beta_ok": bool(np.abs(beta_deg).max() <= RobotParams.BETA_MAX_DEG),
            "arc_max_deg": float(arc_deg.max()),
            "stays_on_foot_arc": bool(arc_deg.max() <= FOOT_ARC_HALF_SPAN_DEG),
        }

    def assert_feasible(self) -> None:
        r = self.guard_report()
        problems = []
        if not r["theta_ok"]:
            problems.append(
                f"theta {r['theta_min_deg']:.1f}-{r['theta_max_deg']:.1f} deg outside "
                f"[{RobotParams.MIN_THETA_DEG}, {RobotParams.MAX_THETA_DEG}]"
            )
        if not r["beta_ok"]:
            problems.append(
                f"|beta| peaks at {r['beta_abs_max_deg']:.1f} deg, limit "
                f"{RobotParams.BETA_MAX_DEG}"
            )
        if not r["stays_on_foot_arc"]:
            problems.append(
                f"contact reaches {r['arc_max_deg']:.1f} deg on the foot arc, past the "
                f"{FOOT_ARC_HALF_SPAN_DEG} deg half-span: the telescoping (SLIP-RF) "
                f"reduction no longer holds"
            )
        if problems:
            raise WorkspaceViolation("; ".join(problems))


class LegLengthMap:
    """Bidirectional map between theta_c and the hip-to-arc-center distance."""

    def __init__(self, leg: LegModel | None = None) -> None:
        self.leg = leg or LegModel()

    def length(self, theta: float) -> float:
        """Hip-to-foot-arc-center distance at a given extension (m)."""
        self.leg.forward(theta, 0.0, vector=False)
        return abs(complex(self.leg.O_r))

    def theta_for(self, length: float) -> float:
        """Extension giving a target hip-to-arc-center distance (rad)."""
        lo = np.deg2rad(RobotParams.MIN_THETA_DEG + 0.5)
        hi = np.deg2rad(RobotParams.MAX_THETA_DEG)
        f_lo, f_hi = self.length(lo) - length, self.length(hi) - length
        if np.sign(f_lo) == np.sign(f_hi):
            raise WorkspaceViolation(
                f"hip-to-arc-center {length:.4f} m unreachable; theta spans "
                f"{self.length(lo):.4f}-{self.length(hi):.4f} m"
            )
        return float(brentq(lambda t: self.length(t) - length, lo, hi, xtol=1e-12))

    def slope(self, theta: float, delta: float = np.deg2rad(0.5)) -> float:
        """dl/dtheta (m/rad); the factor converting leg force to motor torque."""
        return (self.length(theta + delta) - self.length(theta - delta)) / (2 * delta)


def map_template(
    template: StrideTemplate,
    n: int = 200,
    contact_radius: float | None = None,
    leg_map: LegLengthMap | None = None,
) -> JointTrajectory:
    """Convert a stride template into Corgi joint commands.

    Args:
        template: the fixed-point stride from `gslip_template.build_template`
        n: samples across the stride
        contact_radius: foot radius; defaults to the template's own r, which
            should be the tyre value the rest of the pipeline assumes
        leg_map: reusable theta <-> length map
    """
    leg_map = leg_map or LegLengthMap()
    r = template.params.r if contact_radius is None else contact_radius

    s = template.sample(n)
    theta = np.empty(n)
    for i, length in enumerate(s["leg_length"]):
        theta[i] = leg_map.theta_for(float(length) - r)

    beta = -s["leg_angle"]

    # Rolling angle on the foot arc. The contact point is directly below the
    # arc center, so its offset from the leg axis is exactly the leg's swing.
    contact_alpha = np.rad2deg(s["leg_angle"])

    return JointTrajectory(
        t=s["t"],
        theta=theta,
        beta=beta,
        gamma=np.zeros(n),
        in_stance=s["in_stance"],
        contact_alpha=contact_alpha,
    )


def pronk(traj: JointTrajectory) -> dict[str, JointTrajectory]:
    """All four legs in phase: one virtual leg, identical commands.

    Module order matches corgi_msgs (A = front-left, B = front-right,
    C = rear-right, D = rear-left).
    """
    return {name: traj for name in ("A", "B", "C", "D")}


def samples_for_rate(period: float, rate_hz: float = 1000.0) -> int:
    """Sample count so consecutive frames land one control tick apart.

    The Corgi control loop runs at 1 kHz (corgi_driver.py, 1 ms basicTimeStep),
    and corgi_csv_control replays one CSV row per tick, so the row spacing has
    to match the tick or the trajectory plays back at the wrong speed.
    """
    return int(round(period * rate_hz)) + 1


def to_csv(traj: JointTrajectory, path, cycles: int = 1) -> None:
    """Write hardware-ready 12-DOF commands in the corgi_csv_control format.

    Columns are theta/beta/gamma per module, repeated for `cycles` strides.
    The final sample of each stride is the wrap-around of the first, so it is
    dropped on all but the last cycle to avoid a duplicated frame (which would
    read as a one-tick stall at every stride boundary).
    """
    import csv

    legs = ("A", "B", "C", "D")
    header = [f"{leg}_{q}" for leg in legs for q in ("theta", "beta", "gamma")]
    n = len(traj.t)

    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for cycle in range(cycles):
            last = n if cycle == cycles - 1 else n - 1
            for i in range(last):
                row = []
                for _leg in legs:
                    row += [traj.theta[i], traj.beta[i], traj.gamma[i]]
                writer.writerow([f"{value:.6f}" for value in row])


def to_template_csv(traj: JointTrajectory, path) -> None:
    """Write the stride reference for the G-SLIP controller node.

    Distinct from `to_csv`, which emits the 12-DOF format corgi_csv_control
    replays open-loop. The controller additionally needs to know which samples
    are stance, because the virtual spring is only active there, so this
    carries one leg's reference plus the phase flag. Pronk applies the same
    row to all four modules, so one leg is enough.

    Columns: t, theta, beta, gamma, in_stance
    """
    import csv

    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "theta", "beta", "gamma", "in_stance"])
        for i in range(len(traj.t)):
            writer.writerow([
                f"{traj.t[i]:.6f}",
                f"{traj.theta[i]:.6f}",
                f"{traj.beta[i]:.6f}",
                f"{traj.gamma[i]:.6f}",
                int(bool(traj.in_stance[i])),
            ])


def motor_torque_for(leg_force: float, theta: float,
                     leg_map: LegLengthMap | None = None) -> float:
    """Motor torque realizing a radial leg force (N.m).

    Virtual work through the Corgi's motor coupling
    (phi_R = beta + theta - 17deg, phi_L = beta - theta + 17deg) gives
    d(phi_R) = -d(phi_L) = d(theta) for radial motion, so
    F_leg * dl/dtheta = tau_R - tau_L, split evenly between the two motors.
    """
    leg_map = leg_map or LegLengthMap()
    return leg_force * leg_map.slope(theta) / 2.0
