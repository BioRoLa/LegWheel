"""Tests for the SLIP-RF template -> Corgi joint mapping."""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.models import slip_rf
from legwheel.models.leg_model import LegModel
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

CORGI = slip_rf.SlipRfParams(m=30.0, l0=0.230, k=62322.0, r=0.145)
V = 1.2 * np.sqrt(9.81 * 0.230)
ALPHA = np.deg2rad(20.0)
BETA = np.deg2rad(70.0)


@pytest.fixture(scope="module")
def template():
    return tpl.build_template(CORGI, V, ALPHA, BETA)


@pytest.fixture(scope="module")
def leg_map():
    return g2c.LegLengthMap()


def test_leg_length_map_round_trips(leg_map) -> None:
    for theta_deg in (25.0, 45.0, 65.89, 90.0, 130.0):
        theta = np.deg2rad(theta_deg)
        assert leg_map.theta_for(leg_map.length(theta)) == pytest.approx(theta, abs=1e-9)


def test_leg_length_map_matches_phase_0(leg_map) -> None:
    """theta = 65.89 deg gives the Phase 0 nominal hip-to-arc-center length."""
    assert leg_map.length(np.deg2rad(65.89)) == pytest.approx(0.0850, abs=5e-5)


def test_leg_length_slope_matches_phase_2(leg_map) -> None:
    """dl/dtheta at the nominal pose, the leg-force to motor-torque factor."""
    assert leg_map.slope(np.deg2rad(65.89)) == pytest.approx(0.10324, rel=1e-3)


def test_unreachable_length_is_rejected(leg_map) -> None:
    with pytest.raises(g2c.WorkspaceViolation):
        leg_map.theta_for(0.5)


def test_mapping_round_trips_through_forward_kinematics(template, leg_map) -> None:
    """The commanded (theta, beta) must place the foot arc where SLIP-RF wants it.

    This is the check that catches a sign error in beta_c = -phi: it compares
    the Corgi's actual hip-to-arc-center vector against the model's, in both
    magnitude and bearing.
    """
    traj = g2c.map_template(template, n=40)
    leg = LegModel()
    s = template.sample(40)

    for i in range(40):
        leg.forward(traj.theta[i], traj.beta[i], vector=False)
        actual = complex(leg.O_r)
        # Model: mass -> foot center is (l - r)*(-sin phi, -cos phi).
        length = s["leg_length"][i] - CORGI.r
        phi = s["leg_angle"][i]
        expected = complex(-length * np.sin(phi), -length * np.cos(phi))
        assert abs(actual - expected) < 1e-6, (
            f"sample {i}: got {actual}, expected {expected}"
        )


def test_beta_is_negated_leg_angle(template) -> None:
    traj = g2c.map_template(template, n=30)
    s = template.sample(30)
    assert traj.beta == pytest.approx(-s["leg_angle"], abs=1e-12)


def test_gamma_is_zero_for_planar_work(template) -> None:
    traj = g2c.map_template(template, n=20)
    assert np.all(traj.gamma == 0.0)


def test_trajectory_is_feasible_at_the_target_speed(template) -> None:
    """The chosen v~ = 1.2 fixed point must respect every hardware limit."""
    traj = g2c.map_template(template, n=200)
    report = traj.guard_report()
    traj.assert_feasible()
    assert RobotParams.MIN_THETA_DEG < report["theta_min_deg"]
    assert report["theta_max_deg"] < RobotParams.MAX_THETA_DEG
    assert report["stays_on_foot_arc"]


def test_guard_rejects_an_out_of_range_trajectory(template) -> None:
    """A trajectory pushed past the beta limit must be reported, not silently passed."""
    traj = g2c.map_template(template, n=50)
    traj.beta = traj.beta + np.deg2rad(RobotParams.BETA_MAX_DEG + 5.0)
    with pytest.raises(g2c.WorkspaceViolation, match="beta"):
        traj.assert_feasible()


def test_guard_rejects_leaving_the_foot_arc(template) -> None:
    traj = g2c.map_template(template, n=50)
    traj.contact_alpha = traj.contact_alpha + 60.0
    with pytest.raises(g2c.WorkspaceViolation, match="foot arc"):
        traj.assert_feasible()


def test_pronk_gives_all_four_legs_the_same_command(template) -> None:
    traj = g2c.map_template(template, n=20)
    legs = g2c.pronk(traj)
    assert set(legs) == {"A", "B", "C", "D"}
    for leg in legs.values():
        assert leg.theta is traj.theta


def test_csv_export_shape(tmp_path, template) -> None:
    traj = g2c.map_template(template, n=25)
    path = tmp_path / "pronk.csv"
    g2c.to_csv(traj, path, cycles=2)
    lines = path.read_text().strip().splitlines()
    # Header + one dropped wrap-around frame on all but the final cycle.
    assert len(lines) == 1 + (25 - 1) + 25
    assert lines[0].split(",")[:3] == ["A_theta", "A_beta", "A_gamma"]
    assert len(lines[1].split(",")) == 12


def test_csv_has_no_duplicated_frame_at_the_stride_boundary(tmp_path, template) -> None:
    """A repeated wrap-around frame would read as a one-tick stall each stride."""
    traj = g2c.map_template(template, n=30)
    path = tmp_path / "pronk.csv"
    g2c.to_csv(traj, path, cycles=3)
    rows = [r.split(",") for r in path.read_text().strip().splitlines()[1:]]
    assert not any(rows[i] == rows[i + 1] for i in range(len(rows) - 1))


def test_samples_for_rate_matches_the_control_tick(template) -> None:
    """Row spacing must equal the 1 ms loop period."""
    n = g2c.samples_for_rate(template.period, rate_hz=1000.0)
    dt = template.period / (n - 1)
    assert dt == pytest.approx(1e-3, rel=0.01)


def test_motor_torque_conversion(leg_map) -> None:
    """tau = F * (dl/dtheta)/2; 678 N per leg should sit at the 35 N.m limit."""
    theta = np.deg2rad(65.89)
    assert g2c.motor_torque_for(678.0, theta, leg_map) == pytest.approx(35.0, rel=0.01)
