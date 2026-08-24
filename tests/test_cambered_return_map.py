"""Tests for the 3D cambered-pair return map (Stage 2b Modules 3-4)."""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import cambered_return_map as crm
from legwheel.models import coronal_bip as bip
from legwheel.models.cambered_return_map import PairParams, RollPD

V_OP = 1.19          # ~ the v~0.70 operating point, m/s forward at apex
BETA0 = np.deg2rad(71.75)

# The budget-sweep winner (stage2b_budget_gain_sweep, 2026-08-18), valid at
# the CANONICAL orbit below -- on this file's other fixture orbit the same
# gains survive only 12.5% of the NEAR grid. Gains are per-orbit facts.
SWEEP_KP, SWEEP_KD, SWEEP_TAU = 100.0, 12.5, 40.0
U_LIMITS = (np.array([np.deg2rad(40.0), -np.deg2rad(30.0), -np.deg2rad(30.0)]),
            np.array([np.deg2rad(89.0), +np.deg2rad(30.0), +np.deg2rad(30.0)]))


@pytest.fixture(scope="module")
def fixed_point():
    p = PairParams()
    x_star, u_star = crm.solve_periodic(
        p, [V_OP, 0.0, 0.31, 0.0, 0.0], [BETA0, 0.0, 0.0])
    return p, x_star, u_star


@pytest.fixture(scope="module")
def fixed_point_canonical():
    """The section 45 gate orbit (stage2b_clocked_torque's seed) -- the one
    every recorded Stage 2b number lives on."""
    p = PairParams()
    x_star, u_star = crm.solve_periodic(
        p, [V_OP * np.cos(np.deg2rad(40.74)), 0.0, 0.32, 0.0, 0.0],
        [np.deg2rad(80.75), 0.0, 0.0])
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


def test_basin_scan_uses_a_fresh_controller_so_per_cell_peaks_are_independent(
        fixed_point) -> None:
    """The section 45 gate shared one RollPD across a whole grid, so its
    printed peak was a grid-wide max. basin_scan clones the prototype per
    cell: a mild perturbation must report a smaller peak than a harsh one,
    and the prototype itself must come back untouched."""
    p, x_star, u_star = fixed_point
    proto = RollPD()
    res = crm.basin_scan(p, x_star, u_star,
                         rho_vals=np.deg2rad([0.5, 8.0]), drho_vals=[0.0],
                         ctrl_proto=proto, max_steps=3)
    assert res.peak[0, 0] < res.peak[1, 0]
    assert proto.peak_used == 0.0


def test_basin_radius_shrinks_when_the_torque_clamp_tightens(
        fixed_point) -> None:
    """The clamp is the budget: starving the PD cannot enlarge the connected
    basin. r(theta) under a 15 N.m clamp must not exceed r(theta) at 40."""
    p, x_star, u_star = fixed_point
    jx, ju = crm.jacobians(p, x_star, u_star)
    k = crm.deadbeat_gain(jx, ju)
    angles = np.linspace(0.0, 2 * np.pi, 4, endpoint=False)
    kwargs = dict(rho_scale=np.deg2rad(6.0), drho_scale=0.3, gain=k,
                  max_steps=4, r_max=2.0, tol=0.25)
    r_full = crm.basin_radius(p, x_star, u_star, angles,
                              ctrl_proto=RollPD(tau_max=40.0), **kwargs)
    r_starved = crm.basin_radius(p, x_star, u_star, angles,
                                 ctrl_proto=RollPD(tau_max=15.0), **kwargs)
    assert np.all(r_starved <= r_full + 1e-9)


def test_exec_bias_zero_is_bit_identical_to_no_bias(fixed_point) -> None:
    """u_exec_bias is an execution-side offset the controller never sees;
    at zero it must be the SAME computation as passing None -- bit-identical
    survival, peak, and slew numbers, not merely close ones. Guards the
    touchdown-bias campaign's baseline column against the plumbing itself."""
    p, x_star, u_star = fixed_point
    jx, ju = crm.jacobians(p, x_star, u_star)
    k = crm.deadbeat_gain(jx, ju)
    zero = np.zeros(3)

    # steps_to_fail, deadbeat arm, off-orbit start.
    x0 = x_star.copy()
    x0[3] += np.deg2rad(2.0)
    x0[4] += 0.10
    n_ref = crm.steps_to_fail(p, x_star, u_star, x0, gain=k, max_steps=5)
    assert crm.steps_to_fail(p, x_star, u_star, x0, gain=k, max_steps=5,
                             u_exec_bias=None) == n_ref
    assert crm.steps_to_fail(p, x_star, u_star, x0, gain=k, max_steps=5,
                             u_exec_bias=zero) == n_ref

    # basin_scan, PD + clamped deadbeat -- the campaign's scored path.
    kwargs = dict(rho_vals=np.deg2rad([-2.0, 2.0]), drho_vals=[0.1],
                  ctrl_proto=RollPD(), gain=k, max_steps=4,
                  u_limits=U_LIMITS)
    ref = crm.basin_scan(p, x_star, u_star, **kwargs)
    biased = crm.basin_scan(p, x_star, u_star, u_exec_bias=zero, **kwargs)
    assert np.array_equal(ref.steps, biased.steps)
    assert np.array_equal(ref.peak, biased.peak)
    assert np.array_equal(ref.dlam, biased.dlam)


def test_sweep_gains_hold_the_near_grid_inside_the_abad_budget(
        fixed_point_canonical) -> None:
    """The claim section 45 could only print, now asserted: at the canonical
    orbit the budget-sweep gains (kp 100, kd 12.5, clamp 40) plus the clamped
    deadbeat survive the entire NEAR grid with UNCLIPPED peak demand inside
    the 40 N.m ABAD budget (measured 31.2). If this fails after a model
    change, the budget conversation with the hardware changed too.

    ⚠ IT DID, 2026-08-23 (S183/S184). The S183 fix to `side_geometry` -- the
    crown term was R_CORNER where the ROLLING radius belongs, ~20x -- changed
    the lateral coupling this controller fights, and the answer is now
    LAW-DEPENDENT:

        radius_law="measured"  survival 1.000, peak inside budget  (holds)
        radius_law="torus"     survival 0.625                      (does NOT)

    Both are PINNED below rather than relaxed to whichever passes. The torus
    arm is not a bug to be tuned away in the test -- it is a real statement
    that S57's budget-sweep gains do not transfer to the smooth-torus geometry
    and would need re-tuning there. Which law the thesis stands behind is an
    open decision (Stage 2a's E3 sensitivity check); this test records both
    outcomes so that decision is made on numbers rather than on a green suite.
    """
    p, x_star, u_star = fixed_point_canonical
    jx, ju = crm.jacobians(p, x_star, u_star)
    k = crm.deadbeat_gain(jx, ju)
    res = crm.basin_scan(
        p, x_star, u_star, np.deg2rad([-3.0, -1.5, 1.5, 3.0]),
        [-0.15, -0.08, 0.08, 0.15],
        ctrl_proto=RollPD(kp=SWEEP_KP, kd=SWEEP_KD, tau_max=SWEEP_TAU),
        gain=k, max_steps=12, u_limits=U_LIMITS)
    if bip.RADIUS_LAW_DEFAULT == "measured":
        assert res.survival_fraction == 1.0
        assert res.peak_max <= SWEEP_TAU
    else:
        # Pinned, not excused. If this number moves, say why.
        assert res.survival_fraction == pytest.approx(0.625, abs=1e-9)


def test_roll_pd_reset_zeroes_the_peak_recorder() -> None:
    ctrl = RollPD()
    ctrl.torque(0.1, 0.0)
    assert ctrl.peak_used > 0.0
    ctrl.reset()
    assert ctrl.peak_used == 0.0


def test_ackermann_pair_helper() -> None:
    assert crm.ackermann_pair(0.0, 0.28) == (0.0, 0.0)
    lam_in, lam_out = crm.ackermann_pair(np.deg2rad(15.0), 0.28)
    assert 0.0 < lam_out < lam_in          # outer wheel leans less
    _, lam_out_neg = crm.ackermann_pair(-np.deg2rad(15.0), 0.28)
    assert lam_out_neg == pytest.approx(-lam_out)
