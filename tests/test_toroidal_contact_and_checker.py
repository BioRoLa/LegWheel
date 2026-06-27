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


def test_cli_lateral_guard_matches_one_sided_planner_limit():
    """CLI check must warn above the one-sided lateral guard used by GaitGenerator3D."""
    args = argparse.Namespace(
        gait="Trot", height=0.30, vx=0.0, vy=0.45, wz=0.0, period=1.0, step=0.04
    )
    buf = io.StringIO()
    with redirect_stdout(buf):
        cmd_check(args)
    output = buf.getvalue()

    assert "Velocity Guard (Y)" in output
    assert "Twist downscaled to 66." in output


def test_legacy_checker_lateral_guard_matches_one_sided_planner_limit():
    """Legacy example checker should report the same one-sided lateral guard warning."""
    result = subprocess.run(
        [
            sys.executable,
            "examples/kinematics/check_parameters.py",
            "--gait",
            "Trot",
            "--height",
            "0.30",
            "--vx",
            "0.0",
            "--vy",
            "0.45",
            "--period",
            "1.0",
            "--step",
            "0.04",
        ],
        cwd=".",
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert "Velocity Guard (Y)" in result.stdout
    assert "Twist downscaled to 66." in result.stdout
