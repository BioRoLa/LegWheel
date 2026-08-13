"""Regression tests for toroidal tire contact and gait parameter guards."""

import argparse
import io
import subprocess
import sys
from contextlib import redirect_stdout

import numpy as np

from legwheel.cli import cmd_check
from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics


def _sample_lowest_contact(kin, theta, beta, gamma, alpha, n_samples=401):
    """Return the sampled lowest toroidal contact along wheel width in {B}."""
    half_w = RobotParams.WHEEL_THICKNESS / 2.0
    samples = np.linspace(-half_w, half_w, n_samples)
    points = np.array(
        [kin.forward_kinematics(theta, beta, gamma, alpha=alpha, w=w) for w in samples]
    )
    idx = int(np.argmin(points[:, 2]))  # {B} has +Z up, so lower means smaller z.
    return samples[idx], points[idx]


def test_toroidal_contact_selector_returns_sampled_lowest_width_point():
    """`foot_rim_contact_fk` should find the toroidal lowest point, not force edge contact."""
    kin = CorgiLegKinematics(0)
    theta = np.deg2rad(140.0)
    beta = 0.0
    gamma = np.deg2rad(5.0)

    alpha, w_contact = kin.foot_rim_contact_fk(theta, beta, gamma)
    _, p_expected = _sample_lowest_contact(kin, theta, beta, gamma, alpha)
    p_contact = kin.forward_kinematics(theta, beta, gamma, alpha=alpha, w=w_contact)

    assert abs(p_contact[2] - p_expected[2]) < 1e-4
    assert abs(w_contact) < RobotParams.WHEEL_THICKNESS / 2.0 - 1e-3


def _planner_v_y_limit(height: float, period: float, gait: str = "Trot") -> float:
    """The planner's own one-sided lateral limit, from GaitGenerator3D.

    Mirrors gait_generator_3d.py: D_y_max = H_hip * sin(GAMMA_GUARD), because the
    contact rolls from the touchdown extreme down to a floor near zero rather
    than sweeping symmetrically about upright.

    Deliberately recomputed here rather than imported: the point of the tests
    below is that the CLI and the planner agree, and a shared helper would make
    them agree by construction whatever either one did.
    """
    from legwheel.planners.gait_generator_3d import GAIT_LIBRARY

    stance_duty = GAIT_LIBRARY[gait]["stance_duty"]
    H_hip = height + RobotParams.ABAD_AXIS_OFFSET
    d_y_max = H_hip * np.sin(np.deg2rad(RobotParams.GAMMA_GUARD_DEG))
    return d_y_max / (period * stance_duty)


def test_lateral_velocity_guard_is_not_looser_than_the_geometric_limit():
    """The velocity guard may not permit more lateral tilt than the leg has.

    GAMMA_GUARD_DEG is meant to be a *conservative* guard on GAMMA_MAX_DEG, the
    ABAD sweep the hardware actually has, so exceeding it would let the planner
    command poses the leg cannot reach.

    Note the two are currently EQUAL (both 70.0), so the guard is not
    conservative -- it binds exactly when the geometry does, never before. This
    asserts only the invariant that must hold regardless; whether the guard
    should sit below the geometric limit, and what that limit really is, is a
    hardware question. See the note on GAMMA_MAX_DEG in the config.
    """
    assert RobotParams.GAMMA_GUARD_DEG <= RobotParams.GAMMA_MAX_DEG


def test_cli_lateral_guard_matches_one_sided_planner_limit():
    """CLI check must warn above the one-sided lateral guard used by GaitGenerator3D.

    The command is derived from GAMMA_GUARD_DEG rather than hard-coded. The
    previous version asked for vy = 0.45 and asserted "downscaled to 66.",
    numbers that were correct only while GAMMA_GUARD_DEG was 30.27 -- sized, per
    a since-deleted comment, so that vy = 0.6 at h = 0.30 / T = 1.0 sat exactly
    on the boundary. When the constant moved to 70.0 the guard stopped binding at
    0.45 and both checker tests failed, which looked like a message mismatch and
    was really a changed limit.
    """
    height, period, overshoot = 0.30, 1.0, 1.5
    v_y_limit = _planner_v_y_limit(height, period)
    vy = v_y_limit * overshoot

    args = argparse.Namespace(
        gait="Trot", height=height, vx=0.0, vy=vy, wz=0.0, period=period, step=0.04
    )
    buf = io.StringIO()
    with redirect_stdout(buf):
        cmd_check(args)
    output = buf.getvalue()

    assert "Velocity Guard (Y)" in output, output
    # Commanding 1.5x the limit must scale back to 1/1.5 = 66.7%, and the CLI's
    # own limit must therefore equal the planner's.
    assert f"downscaled to {100 / overshoot:.1f}%" in output, output


def test_legacy_checker_lateral_guard_matches_one_sided_planner_limit():
    """Legacy example checker should report the same one-sided lateral guard warning."""
    height, period, overshoot = 0.30, 1.0, 1.5
    vy = _planner_v_y_limit(height, period) * overshoot

    result = subprocess.run(
        [
            sys.executable,
            "examples/kinematics/check_parameters.py",
            "--gait",
            "Trot",
            "--height",
            f"{height}",
            "--vx",
            "0.0",
            "--vy",
            f"{vy:.6f}",
            "--period",
            f"{period}",
            "--step",
            "0.04",
        ],
        cwd=".",
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert "Velocity Guard (Y)" in result.stdout, result.stdout
    assert f"downscaled to {100 / overshoot:.1f}%" in result.stdout, result.stdout
