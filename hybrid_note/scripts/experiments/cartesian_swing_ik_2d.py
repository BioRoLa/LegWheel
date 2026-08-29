"""Day 8--9 Step 4: generalized rim-point FK / IK (2D, gamma = 0).

The swing planner asks a different question from ordinary foot-tip IK::

    (contact position, rim, alpha)  ->  (theta, beta)

that is, "put *this* point of the tyre at *that* place in the world", not
"put the foot somewhere".

Which geometry is the truth
---------------------------
Two rim parameterisations exist in this repository and they are **not**
identical:

* ``LegModel.rim_point(alpha_deg, w)`` -- the physical model.  Every rim uses
  ``WHEEL_RADIUS_OUTER = 0.145 m``.
* the arcs drawn by ``PlotLeg.leg_shape`` -- what
  ``sample_contact_geometry_points`` samples, and therefore what the whole
  Day 3--7 contact/collision pipeline treats as the leg surface.  The foot rim
  is also 0.145 m, but the two upper tyres carry a 1.2 mm drawing clearance
  (``c = 0.0012`` in ``LegShape._update_geometry``), so their outer arcs sit at
  0.1438 m.

The angular convention is identical in both (foot rim spans alpha in
[-40, 40] deg about the ``G - O_r`` direction, the upper tyres continue to
+-180 deg); only the upper-rim radius differs, by a constant 1.2 mm.

This module builds FK on the **drawn arcs**, because Steps 6--7 validate the
swing with ``query_contact`` against exactly that geometry.  Targeting the
0.145 m model instead would leave a 1.2 mm gap at an upper-rim touchdown --
larger than the 1 mm contact tolerance -- and Step 7 would reject swings that
its own IK considered exact.  :func:`rim_point_model_gap_2d` measures the
discrepancy so it stays visible rather than becoming folded-in error.

Reused, not reimplemented: the arcs come from ``PlotLeg``; the alpha ranges
from ``LEGACY_SURFACE_ALPHA_LIMITS_DEG``; the joint limits from
``RobotParams``; the damped pseudo-inverse from ``legwheel.utils``.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.planners.hybrid import HipPose2D, RimId
from legwheel.planners.hybrid.geometry_2d import LEGACY_SURFACE_ALPHA_LIMITS_DEG
from legwheel.utils import pseudo_inverse_dls
from legwheel.visualization.plot_leg import PlotLeg

from .cartesian_swing_contract_2d import (
    legacy_surface_name_for_rim,
    rim_alpha_limits_rad,
)

THETA_MIN_RAD = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
THETA_MAX_RAD = float(np.deg2rad(RobotParams.MAX_THETA_DEG))

#: Finite-difference step for the theta column of the Jacobian.  The beta
#: column is analytic, so only theta needs one.
DEFAULT_THETA_DIFF_RAD = 1e-6


@dataclass(frozen=True)
class RimArc2D:
    """One drawn tyre arc at ``beta = 0``, in hip coordinates."""

    rim: RimId
    center_hip_xz_m: NDArray[np.float64]
    radius_m: float
    angle_at_alpha_min_rad: float
    angle_span_rad: float

    def point_at_alpha(self, alpha_rad: float) -> NDArray[np.float64]:
        alpha_min_rad, alpha_max_rad = rim_alpha_limits_rad(self.rim)
        fraction = (float(alpha_rad) - alpha_min_rad) / (alpha_max_rad - alpha_min_rad)
        angle = self.angle_at_alpha_min_rad + fraction * self.angle_span_rad
        return self.center_hip_xz_m + self.radius_m * np.array(
            [np.cos(angle), np.sin(angle)], dtype=float
        )


_SHAPE_ATTRIBUTE_BY_RIM = {
    RimId.FOOT: "foot_rim",
    RimId.LEFT: "upper_rim_l_f",
    RimId.RIGHT: "upper_rim_r_f",
}

#: Which model joints define each drawn arc: (centre, arc start, arc end).
#: Read off ``LegShape._update_geometry``; verified against the drawn arcs in
#: the Step 4 tests rather than trusted.
_ARC_JOINTS_BY_RIM = {
    RimId.FOOT: ("O_r", "I_l", "I_r"),
    RimId.LEFT: ("U_l", "H_extend_l", "J_l"),
    RimId.RIGHT: ("U_r", "J_r", "H_extend_r"),
}


def _scratch_leg() -> PlotLeg:
    """One reusable leg instance; constructing a fresh one costs 6 ms."""

    global _SCRATCH_LEG
    if _SCRATCH_LEG is None:
        _SCRATCH_LEG = PlotLeg()
    return _SCRATCH_LEG


_SCRATCH_LEG: PlotLeg | None = None


def _joint_xz_m(leg: PlotLeg, name: str) -> NDArray[np.float64]:
    value = np.asarray(getattr(leg, name)).ravel()
    if np.iscomplexobj(value):
        return np.array([float(value[0].real), float(value[0].imag)], dtype=float)
    return value[:2].astype(float)


@lru_cache(maxsize=1)
def _drawn_arc_radii_m() -> tuple[tuple[RimId, float], ...]:
    """Measure each drawn arc's outer radius once, from the shape module.

    The upper tyres carry a 1.2 mm drawing clearance that the foot rim does
    not.  Measuring it here keeps that number owned by
    ``LegShape._update_geometry``; hard-coding it in this module would be one
    more place to drift.  The radii are rigid distances, so they do not depend
    on theta -- a Step 4 test pins that down.
    """

    leg = PlotLeg()
    leg.forward(float(np.deg2rad(60.0)), 0.0, vector=False)
    leg.leg_shape.get_shape(np.array([0.0, 0.0]))
    return tuple(
        (rim, float(getattr(leg.leg_shape, attribute).arc[1].width / 2.0))
        for rim, attribute in _SHAPE_ATTRIBUTE_BY_RIM.items()
    )


def _normalised_span_rad(start_rad: float, end_rad: float) -> float:
    """Wrap an arc sweep into ``(-pi, pi]``, matching the existing sampler."""

    span = end_rad - start_rad
    while span > np.pi:
        span -= 2.0 * np.pi
    while span < -np.pi:
        span += 2.0 * np.pi
    return float(span)


@lru_cache(maxsize=4096)
def _rim_arcs_at_theta(theta_rad: float) -> tuple[RimArc2D, ...]:
    """Build the three tyre arcs at ``(theta, beta = 0)`` in hip coordinates.

    The centres and arc endpoints are read from the model's own joints, so
    only ``LegModel.forward`` runs here.  Rebuilding the full drawing shape
    would give bit-identical arcs (the tests check exactly that) at roughly
    seventy times the cost, which matters because a numerical Jacobian
    re-evaluates nearby thetas thousands of times per trajectory.
    """

    leg = _scratch_leg()
    leg.forward(float(theta_rad), 0.0, vector=True)
    radii = dict(_drawn_arc_radii_m())

    arcs = []
    for rim, (centre_name, start_name, end_name) in _ARC_JOINTS_BY_RIM.items():
        centre = _joint_xz_m(leg, centre_name)
        start_vector = _joint_xz_m(leg, start_name) - centre
        end_vector = _joint_xz_m(leg, end_name) - centre
        start_rad = float(np.arctan2(start_vector[1], start_vector[0]))
        end_rad = float(np.arctan2(end_vector[1], end_vector[0]))
        centre.setflags(write=False)
        arcs.append(
            RimArc2D(
                rim=rim,
                center_hip_xz_m=centre,
                radius_m=radii[rim],
                angle_at_alpha_min_rad=start_rad,
                angle_span_rad=_normalised_span_rad(start_rad, end_rad),
            )
        )
    return tuple(arcs)


def rim_arc_2d(theta_rad: float, rim: RimId) -> RimArc2D:
    """Return one drawn tyre arc at ``(theta, beta = 0)`` in hip coordinates."""

    wanted = RimId(rim)
    for arc in _rim_arcs_at_theta(float(theta_rad)):
        if arc.rim is wanted:
            return arc
    raise KeyError(f"no drawn arc for rim {wanted!r}.")


def _rotation(angle_rad: float) -> NDArray[np.float64]:
    cosine, sine = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[cosine, -sine], [sine, cosine]], dtype=float)


def rim_contact_point_hip_xz_m(
    theta_rad: float,
    beta_rad: float,
    rim: RimId,
    alpha_rad: float,
) -> NDArray[np.float64]:
    """Forward kinematics of one rim point, in hip coordinates.

    ``beta`` is a pure rotation of the whole leg about the hip origin in this
    model, so the pose factors as ``R(beta) @ f(theta)``.  Keeping that
    factorisation is not just an optimisation: it makes the beta column of the
    IK Jacobian analytic, and lets the theta-dependent shape be cached.
    """

    point_at_zero_beta = rim_arc_2d(float(theta_rad), rim).point_at_alpha(alpha_rad)
    return _rotation(float(beta_rad)) @ point_at_zero_beta


def rim_contact_point_world_xz_m(
    theta_rad: float,
    beta_rad: float,
    rim: RimId,
    alpha_rad: float,
    hip_pose: HipPose2D,
) -> NDArray[np.float64]:
    """Forward kinematics of one rim point, in world coordinates."""

    if not isinstance(hip_pose, HipPose2D):
        raise TypeError("hip_pose must be a HipPose2D.")
    if not np.isclose(hip_pose.pitch_world_hip_rad, 0.0, atol=1e-12):
        raise ValueError("The Day 8--9 first version only supports zero hip pitch.")
    return rim_contact_point_hip_xz_m(theta_rad, beta_rad, rim, alpha_rad) + (
        hip_pose.position_world_xz_m
    )


def rim_point_model_gap_2d(
    theta_rad: float,
    beta_rad: float,
    rim: RimId,
    alpha_rad: float,
) -> float:
    """Distance between the drawn-arc FK and ``LegModel.rim_point``, in metres.

    Zero on the foot rim; a constant 1.2 mm on the two upper tyres, which is
    the drawing clearance the contact pipeline inherited.  Reported rather than
    absorbed, because it is larger than the 1 mm contact tolerance the same
    pipeline uses.
    """

    leg = PlotLeg()
    leg.forward(float(theta_rad), float(beta_rad), vector=True)
    model_point = leg.rim_point(float(np.rad2deg(alpha_rad)), 0.0)
    model_point = np.asarray(model_point, dtype=float).ravel()[:2]
    drawn_point = rim_contact_point_hip_xz_m(theta_rad, beta_rad, rim, alpha_rad)
    return float(np.linalg.norm(drawn_point - model_point))


@dataclass(frozen=True)
class ContactIkSolution2D:
    """One rim-point IK result, including why it should or should not be used.

    ``converged`` is about the residual only.  ``joint_limits_ok`` is reported
    separately and never silently repaired, so a caller can tell "unreachable"
    apart from "reachable but outside the joint range".
    """

    theta_rad: float
    beta_rad: float
    converged: bool
    residual_m: float
    iterations: int
    theta_at_limit: bool
    joint_limits_ok: bool
    initial_guess_rad: NDArray[np.float64]
    solved_point_world_xz_m: NDArray[np.float64]
    desired_point_world_xz_m: NDArray[np.float64]

    @property
    def joint_angles_rad(self) -> NDArray[np.float64]:
        return np.array([self.theta_rad, self.beta_rad], dtype=float)

    @property
    def usable(self) -> bool:
        """Converged *and* inside the joint range; the only safe combination."""

        return bool(self.converged and self.joint_limits_ok)

    def as_dict(self) -> dict:
        return {
            "theta_deg": float(np.rad2deg(self.theta_rad)),
            "beta_deg": float(np.rad2deg(self.beta_rad)),
            "converged": self.converged,
            "residual_mm": self.residual_m * 1e3,
            "iterations": self.iterations,
            "theta_at_limit": self.theta_at_limit,
            "joint_limits_ok": self.joint_limits_ok,
            "usable": self.usable,
        }


def solve_contact_ik_2d(
    desired_contact_point_world_xz_m,
    rim: RimId,
    alpha_rad: float,
    hip_pose: HipPose2D,
    initial_guess_rad,
    *,
    position_tolerance_m: float = 1e-5,
    max_iterations: int = 60,
    damping: float = 1e-3,
    max_step_rad: float = 0.35,
    max_backtracks: int = 8,
    theta_min_rad: float = THETA_MIN_RAD,
    theta_max_rad: float = THETA_MAX_RAD,
    theta_diff_rad: float = DEFAULT_THETA_DIFF_RAD,
) -> ContactIkSolution2D:
    """Solve ``(p, rim, alpha) -> (theta, beta)`` for a fixed hip pose.

    Two equations, two unknowns, solved by damped Gauss-Newton from
    ``initial_guess_rad``.  The caller supplies that guess; Step 5 passes the
    previous sample's solution so the joint trajectory stays on one branch.

    On theta clamping: ``LegModel.forward`` silently clips theta into its
    limits, so FK outside the range does not describe the pose it claims to.
    The iteration therefore stays inside the box on purpose, and reports
    ``theta_at_limit`` when it ends up pinned there -- an unreachable target is
    returned as an unconverged solution with its true residual, never as a
    quietly clamped "answer".
    """

    target = np.asarray(desired_contact_point_world_xz_m, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("desired_contact_point_world_xz_m must be a finite [x, z] pair.")
    guess = np.asarray(initial_guess_rad, dtype=float)
    if guess.shape != (2,) or not np.all(np.isfinite(guess)):
        raise ValueError("initial_guess_rad must be a finite (theta, beta) pair.")
    if not isinstance(hip_pose, HipPose2D):
        raise TypeError("hip_pose must be a HipPose2D.")
    if position_tolerance_m <= 0.0 or max_iterations < 1:
        raise ValueError("position_tolerance_m must be positive and max_iterations >= 1.")
    if theta_max_rad <= theta_min_rad:
        raise ValueError("theta_max_rad must exceed theta_min_rad.")

    rim = RimId(rim)
    alpha_rad = float(alpha_rad)
    # Fail loudly if alpha does not belong to the rim it is paired with.
    rim_alpha_limits = rim_alpha_limits_rad(rim)
    if not rim_alpha_limits[0] - 1e-12 <= alpha_rad <= rim_alpha_limits[1] + 1e-12:
        raise ValueError(
            f"alpha={np.rad2deg(alpha_rad):.3f} deg is outside the "
            f"{legacy_surface_name_for_rim(rim)} arc."
        )

    theta = float(np.clip(guess[0], theta_min_rad, theta_max_rad))
    beta = float(guess[1])
    hip = hip_pose.position_world_xz_m

    def world_point(theta_value: float, beta_value: float) -> NDArray[np.float64]:
        local_point = rim_arc_2d(theta_value, rim).point_at_alpha(alpha_rad)
        return _rotation(beta_value) @ local_point + hip

    iterations = 0
    residual_m = float(np.linalg.norm(target - world_point(theta, beta)))
    for iterations in range(1, int(max_iterations) + 1):
        if residual_m <= position_tolerance_m:
            break

        local = rim_arc_2d(theta, rim).point_at_alpha(alpha_rad)
        rotation = _rotation(beta)
        error = target - (rotation @ local + hip)

        # d/dbeta R(beta) v = R(beta + pi/2) v -- exact, no differencing.
        beta_column = _rotation(beta + 0.5 * np.pi) @ local
        theta_probe = float(np.clip(theta + theta_diff_rad, theta_min_rad, theta_max_rad))
        step_rad = theta_probe - theta
        if abs(step_rad) < 1e-15:
            theta_probe = float(np.clip(theta - theta_diff_rad, theta_min_rad, theta_max_rad))
            step_rad = theta_probe - theta
        if abs(step_rad) < 1e-15:
            theta_column = np.zeros(2, dtype=float)
        else:
            probe_local = rim_arc_2d(theta_probe, rim).point_at_alpha(alpha_rad)
            theta_column = rotation @ (probe_local - local) / step_rad

        jacobian = np.column_stack([theta_column, beta_column])
        delta = pseudo_inverse_dls(jacobian, damping_factor=damping) @ error

        # Trust region.  Without it a near-singular Jacobian can throw beta
        # most of a turn and land the solver on a different branch, which for
        # a swing means a joint trajectory that teleports mid-flight.
        step_norm = float(np.linalg.norm(delta))
        if step_norm > max_step_rad:
            delta = delta * (max_step_rad / step_norm)

        # Backtracking: accept the first step that actually reduces the
        # residual.  A pure Gauss-Newton step can overshoot near the theta
        # limit and stall while still reporting progress.
        improved = False
        for _ in range(int(max_backtracks) + 1):
            trial_theta = float(np.clip(theta + delta[0], theta_min_rad, theta_max_rad))
            trial_beta = float(beta + delta[1])
            trial_residual = float(np.linalg.norm(target - world_point(trial_theta, trial_beta)))
            if trial_residual < residual_m:
                theta, beta, residual_m = trial_theta, trial_beta, trial_residual
                improved = True
                break
            delta = delta * 0.5
        if not improved:
            # No downhill step exists from here; stop and report the truth.
            break

    solved = rim_contact_point_world_xz_m(theta, beta, rim, alpha_rad, hip_pose)
    residual_m = float(np.linalg.norm(target - solved))
    at_limit = bool(
        np.isclose(theta, theta_min_rad, atol=1e-9) or np.isclose(theta, theta_max_rad, atol=1e-9)
    )
    initial = guess.copy()
    initial.setflags(write=False)
    solved.setflags(write=False)
    desired = target.copy()
    desired.setflags(write=False)
    return ContactIkSolution2D(
        theta_rad=theta,
        beta_rad=beta,
        converged=bool(residual_m <= position_tolerance_m),
        residual_m=residual_m,
        iterations=iterations,
        theta_at_limit=at_limit,
        joint_limits_ok=bool(theta_min_rad - 1e-12 <= theta <= theta_max_rad + 1e-12),
        initial_guess_rad=initial,
        solved_point_world_xz_m=solved,
        desired_point_world_xz_m=desired,
    )
