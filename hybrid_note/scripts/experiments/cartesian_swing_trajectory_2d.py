"""Day 8--9 Step 5: turn a Cartesian swing path into a joint trajectory.

Step 2--3 produce ``p(t)``; Step 4 solves one ``(p, rim, alpha) -> (theta,
beta)``.  Step 5 runs the second along the first::

    p[0] -> q[0], initial_guess = the start ContactState's own joints
    p[1] -> q[1], initial_guess = q[0]
    ...
    p[N] -> q[N], initial_guess = q[N-1]

and records, per sample, whether the IK converged, its residual, whether the
joints stay inside their limits, and how far the joints moved since the
previous sample.

Warm starting is not an optimisation here.  Step 4 measured the solver at
100% convergence when the initial guess is within +-1 deg and 93% at +-25 deg,
and a cold start can land on a different solution branch -- which for a swing
means a joint trajectory that teleports mid-flight.  Feeding the previous
solution forward keeps the whole trajectory on one branch.

Nothing here writes ``valid``: collision (Step 6) and touchdown (Steps 7--8)
have still not run, so a trajectory that passes every check in this module is
reported as ``NOT_EVALUATED``, not as a usable swing.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from numpy.typing import NDArray

from legwheel.planners.hybrid import HipPose2D

from .cartesian_swing_contract_2d import (
    SwingFailure,
    SwingRequest2D,
    SwingResult2D,
    SwingSample2D,
)
from .cartesian_swing_ik_2d import ContactIkSolution2D, solve_contact_ik_2d


def _wrapped(angle_rad: float) -> float:
    """Map an angle difference into ``(-pi, pi]``."""

    return float((float(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi)


def joint_step_rad(
    previous_rad: NDArray[np.float64],
    current_rad: NDArray[np.float64],
) -> float:
    """Distance between two ``(theta, beta)`` poses, wrapping beta.

    Beta wraps because a solver branch differing by a full turn is the same
    physical configuration; theta does not, because it is a bounded linkage
    coordinate rather than an angle the leg can spin through.
    """

    delta_theta = float(current_rad[0] - previous_rad[0])
    delta_beta = _wrapped(float(current_rad[1] - previous_rad[1]))
    return float(np.hypot(delta_theta, delta_beta))


def _hip_pose_for_sample(request: SwingRequest2D, sample: SwingSample2D) -> HipPose2D:
    fraction = float(sample.time_s / request.swing_duration_s)
    return request.hip_trajectory.pose_at(float(np.clip(fraction, 0.0, 1.0)))


def _first_failure(
    samples: tuple[SwingSample2D, ...],
    solutions: tuple[ContactIkSolution2D, ...],
    request: SwingRequest2D,
) -> tuple[SwingFailure, int | None, str | None]:
    """Return the earliest sample-level problem, in a fixed priority order.

    Ordering matters: an unconverged solve makes its residual, limits and step
    meaningless, so it must be reported instead of the symptoms it causes.
    """

    limits = request.constraints
    for sample, solution in zip(samples, solutions):
        if not solution.converged:
            # A target the leg cannot reach because theta ran out shows up as a
            # non-convergence, not as a limit violation: the solve stays inside
            # the box on purpose, so the joints it reports are always legal.
            # Say which of the two it was.
            pinned = (
                " theta is pinned at its limit, so the target is out of reach."
                if solution.theta_at_limit
                else ""
            )
            return (
                SwingFailure.IK_NOT_CONVERGED,
                sample.index,
                f"IK stopped at residual {solution.residual_m * 1e3:.3f} mm after "
                f"{solution.iterations} iterations.{pinned}",
            )
        if solution.residual_m > limits.ik_position_tolerance_m:
            return (
                SwingFailure.IK_RESIDUAL_TOO_LARGE,
                sample.index,
                f"residual {solution.residual_m * 1e3:.3f} mm exceeds "
                f"{limits.ik_position_tolerance_m * 1e3:.3f} mm.",
            )
        if not sample.joint_limits_ok:
            return (
                SwingFailure.JOINT_LIMIT_VIOLATION,
                sample.index,
                f"theta {np.rad2deg(solution.theta_rad):.2f} deg is outside "
                f"[{np.rad2deg(limits.theta_min_rad):.2f}, "
                f"{np.rad2deg(limits.theta_max_rad):.2f}] deg.",
            )
        if sample.joint_step_rad is not None and sample.joint_step_rad > limits.max_joint_step_rad:
            return (
                SwingFailure.JOINT_DISCONTINUITY,
                sample.index,
                f"joint step {np.rad2deg(sample.joint_step_rad):.2f} deg exceeds "
                f"{np.rad2deg(limits.max_joint_step_rad):.2f} deg.",
            )
    return SwingFailure.NOT_EVALUATED, None, "Joints solved; Steps 6-8 have not run."


def solve_swing_joint_trajectory_2d(
    result: SwingResult2D,
    *,
    initial_guess_rad=None,
    position_tolerance_m: float = 1e-5,
    max_iterations: int = 60,
) -> tuple[SwingResult2D, tuple[ContactIkSolution2D, ...]]:
    """Solve every sample of a Cartesian path into ``(theta, beta)``.

    The first guess defaults to the start ContactState's own joints, which the
    Step 1 contract already carries -- so the trajectory starts from the pose
    the swing is actually leaving, not from an arbitrary seed.

    Every sample is solved even after one fails, so the failure can be located
    and looked at; the returned result reports the *earliest* problem.  The
    raw solutions are returned alongside for diagnostics that do not belong in
    the frozen sample schema.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; generate a Cartesian path first.")

    request = result.request
    limits = request.constraints
    if initial_guess_rad is None:
        guess = np.array([request.start.theta_rad, request.start.beta_rad], dtype=float)
    else:
        guess = np.asarray(initial_guess_rad, dtype=float)
        if guess.shape != (2,):
            raise ValueError("initial_guess_rad must be a (theta, beta) pair.")

    solved_samples = []
    solutions = []
    previous_joints = None
    for sample in result.samples:
        solution = solve_contact_ik_2d(
            sample.position_world_xz_m,
            sample.rim,
            sample.alpha_rad,
            _hip_pose_for_sample(request, sample),
            guess,
            position_tolerance_m=position_tolerance_m,
            max_iterations=max_iterations,
            theta_min_rad=limits.theta_min_rad,
            theta_max_rad=limits.theta_max_rad,
        )
        step_rad = (
            None if previous_joints is None else joint_step_rad(previous_joints, solution.joint_angles_rad)
        )
        solved_samples.append(
            replace(
                sample,
                theta_rad=solution.theta_rad,
                beta_rad=solution.beta_rad,
                gamma_rad=0.0,
                ik_converged=solution.converged,
                ik_residual_m=solution.residual_m,
                joint_step_rad=step_rad,
                joint_limits_ok=solution.joint_limits_ok,
            )
        )
        solutions.append(solution)
        # Keep warm-starting from the last solve even when it failed: a cold
        # restart mid-trajectory would swap branches and hide the real problem
        # behind a discontinuity.
        guess = solution.joint_angles_rad
        previous_joints = solution.joint_angles_rad

    samples = tuple(solved_samples)
    solutions = tuple(solutions)
    failure, failure_index, detail = _first_failure(samples, solutions, request)
    return (
        SwingResult2D(
            request=request,
            samples=samples,
            valid=False,
            failure=failure,
            failure_sample_index=failure_index,
            failure_detail=detail,
            minimum_terrain_clearance_m=result.minimum_terrain_clearance_m,
        ),
        solutions,
    )


def joint_trajectory_report_2d(result: SwingResult2D) -> dict:
    """Summarise what Step 5's completion criterion asks about.

    "A continuous joint trajectory" is four measurable things: every solve
    converged, every residual is small, every sample is inside the joint
    limits, and no two consecutive samples are far apart.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    samples = result.samples
    if not samples or any(sample.theta_rad is None for sample in samples):
        raise ValueError("result has no joint solution; run Step 5 first.")

    thetas = np.array([sample.theta_rad for sample in samples], dtype=float)
    betas = np.array([sample.beta_rad for sample in samples], dtype=float)
    residuals = np.array([sample.ik_residual_m for sample in samples], dtype=float)
    steps = np.array(
        [sample.joint_step_rad for sample in samples if sample.joint_step_rad is not None],
        dtype=float,
    )
    request = result.request

    # Does the first solve reproduce the pose the swing is leaving?  If the
    # contract, the path and the FK/IK disagree anywhere, this is where it
    # shows up first.
    start_theta_error_deg = float(np.rad2deg(abs(thetas[0] - request.start.theta_rad)))
    start_beta_error_deg = float(
        np.rad2deg(abs(_wrapped(betas[0] - request.start.beta_rad)))
    )

    # The per-sample step depends on how densely the swing was sampled; the
    # rate does not, so both are reported and only the rate is comparable
    # between trajectories of different sample_count.
    duration_s = float(request.swing_duration_s)
    sample_interval_s = duration_s / max(len(samples) - 1, 1)

    return {
        "samples": len(samples),
        "ik_converged_count": int(sum(bool(sample.ik_converged) for sample in samples)),
        "max_ik_residual_um": float(np.max(residuals) * 1e6),
        "joint_limits_ok_count": int(sum(bool(sample.joint_limits_ok) for sample in samples)),
        "theta_min_deg": float(np.rad2deg(np.min(thetas))),
        "theta_max_deg": float(np.rad2deg(np.max(thetas))),
        "beta_span_deg": float(np.rad2deg(np.max(betas) - np.min(betas))),
        "max_joint_step_deg": float(np.rad2deg(np.max(steps))) if steps.size else 0.0,
        "max_joint_step_limit_deg": float(np.rad2deg(request.constraints.max_joint_step_rad)),
        "max_joint_speed_deg_s": (
            float(np.rad2deg(np.max(steps)) / sample_interval_s) if steps.size else 0.0
        ),
        "sample_interval_ms": sample_interval_s * 1e3,
        "start_theta_error_deg": start_theta_error_deg,
        "start_beta_error_deg": start_beta_error_deg,
        "failure": result.failure.value,
        "failure_sample_index": result.failure_sample_index,
    }
