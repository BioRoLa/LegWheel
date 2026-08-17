"""Tests for the 3D cambered-pair return map (Stage 2b Modules 3-4)."""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import cambered_return_map as crm
from legwheel.models.cambered_return_map import PairParams, RollPD

V_OP = 1.19          # ~ the v~0.70 operating point, m/s forward at apex
BETA0 = np.deg2rad(71.75)


@pytest.fixture(scope="module")
def fixed_point():
    p = PairParams()
    x_star, u_star = crm.solve_periodic(
        p, [V_OP, 0.0, 0.31, 0.0, 0.0], [BETA0, 0.0, 0.0])
    return p, x_star, u_star


def test_symmetric_map_stays_symmetric() -> None:
    p = PairParams()
    x = crm.apex_map(p, [V_OP, 0.0, 0.31, 0.0, 0.0], [BETA0, 0.0, 0.0])
    assert abs(x[1]) < 1e-10   # vy
    assert abs(x[3]) < 1e-10   # rho
    assert abs(x[4]) < 1e-9    # drho


def test_periodic_gait_exists_and_closes(fixed_point) -> None:
    p, x_star, u_star = fixed_point
    x_next = crm.apex_map(p, x_star, u_star)
    assert np.linalg.norm(x_next - x_star) < 1e-5
    assert 0.05 < x_star[2] < 0.6          # a physical apex height
    assert np.deg2rad(40) < u_star[0] < np.deg2rad(89)


def test_mirror_symmetry_of_the_map() -> None:
    p = PairParams()
    mirror = np.array([1.0, -1.0, 1.0, -1.0, -1.0])
    xa = crm.apex_map(p, [V_OP, 0.05, 0.31, 0.03, 0.1], [BETA0, 0.0, 0.0])
    xb = crm.apex_map(p, [V_OP, -0.05, 0.31, -0.03, -0.1], [BETA0, 0.0, 0.0])
    assert np.allclose(xa, mirror * xb, atol=1e-8)


def test_roll_perturbation_grows_passively(fixed_point) -> None:
    """Chang Fig. 13's expectation, on our own model: the periodic gait is
    unstable in roll without actuation. A seeded roll rate must grow."""
    p, x_star, u_star = fixed_point
    x = x_star.copy()
    x[4] += 0.05
    growth = []
    for _ in range(3):
        x = crm.apex_map(p, x, u_star)
        growth.append(abs(x[3]) + abs(x[4]))
    assert growth[-1] > growth[0]


def test_deadbeat_gain_reduces_one_step_error(fixed_point) -> None:
    p, x_star, u_star = fixed_point
    jx, ju = crm.jacobians(p, x_star, u_star)
    k = crm.deadbeat_gain(jx, ju)
    assert np.all(np.isfinite(k))
    dx = np.array([0.02, 0.0, 0.005, 0.0, 0.0])
    passive = np.linalg.norm(crm.apex_map(p, x_star + dx, u_star) - x_star)
    steered = np.linalg.norm(
        crm.apex_map(p, x_star + dx, u_star + k @ dx) - x_star)
    assert steered < passive


def test_roll_pd_extends_steps_to_fail(fixed_point) -> None:
    """Module 4's claim in miniature: in-stance roll torque must beat passive
    survival for a roll perturbation, at a torque the ABAD budget allows."""
    p, x_star, u_star = fixed_point
    x0 = x_star.copy()
    x0[4] += 0.25
    n_passive = crm.steps_to_fail(p, x_star, u_star, x0, max_steps=12)
    ctrl = RollPD()
    n_pd = crm.steps_to_fail(p, x_star, u_star, x0, ctrl=ctrl, max_steps=12)
    assert n_pd > n_passive
    assert ctrl.peak_used > 0.0


def test_ackermann_pair_helper() -> None:
    assert crm.ackermann_pair(0.0, 0.28) == (0.0, 0.0)
    lam_in, lam_out = crm.ackermann_pair(np.deg2rad(15.0), 0.28)
    assert 0.0 < lam_out < lam_in          # outer wheel leans less
    _, lam_out_neg = crm.ackermann_pair(-np.deg2rad(15.0), 0.28)
    assert lam_out_neg == pytest.approx(-lam_out)
