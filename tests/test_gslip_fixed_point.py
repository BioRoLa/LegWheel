"""Tests for the fixed-point machinery, and for the step counts built on it.

`steps_to_converge` exists because the literature's headline stability number
is "recovers in N strides", and the boolean `converges_to` threw N away. Its
correctness rests on one identity: for a one-dimensional map the count is
log(tol / err_0) / log|dP/dalpha|. If the simulated and analytic counts ever
part company, either the map or the eq-13 error metric is wrong, so that
agreement is asserted here rather than assumed.

The anchor is the Corgi's one genuinely self-stable, non-grazing fixed point:
theta_nom = 100 deg, k_rel = 7, v~ = 2.25, beta = 46 deg -> alpha* = 12.05 deg,
slope +0.7214, apex 32 mm, duty 0.42. It sits well outside the torque envelope
(the gait saturates at v~ = 2.0), which is exactly why the shipped templates
are unstable -- but it is a real fixed point of this model and it makes a
deterministic test anchor.
"""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import (
    FixedPoint,
    basin_in_alpha,
    basin_of_attraction,
    converges_to,
    find_fixed_points,
    steps_to_converge,
)
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
NOMINAL_THETA_DEG = 100.0
TOL = 0.01


def params(k_rel: float) -> slip_rf.SlipRfParams:
    leg_map = g2c.LegLengthMap()
    r = leg_map.leg.foot_radius
    hip = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    return slip_rf.SlipRfParams(m=MASS, l0=hip + r, k=k_rel * MASS * G / hip, r=r)


def _solve(p, v_tilde: float, beta_deg: float, stable: bool):
    """The stable or unstable root at this (speed, landing angle)."""
    v = v_tilde * np.sqrt(G * p.l0)
    roots = find_fixed_points(
        p, v, np.deg2rad(beta_deg),
        alpha_range=(np.deg2rad(1.0), np.deg2rad(60.0)),
        n_samples=60, stride_fn=slip_rf.stride,
    )
    picked = [fp for fp in roots if fp.stable is stable]
    assert picked, f"no {'stable' if stable else 'unstable'} root at v~{v_tilde}, beta {beta_deg}"
    return v, picked[0]


@pytest.fixture(scope="module")
def stable_fp():
    p = params(7.0)
    v, fp = _solve(p, 2.25, 46.0, stable=True)
    return p, v, fp


@pytest.fixture(scope="module")
def unstable_fp():
    """The config of record's orbit, v~0.70 -- slope +1.28, i.e. 28%/stride growth."""
    p = params(18.0)
    v, fp = _solve(p, 0.70, 80.75, stable=False)
    return p, v, fp


def test_stable_anchor_matches_the_recorded_fixed_point(stable_fp) -> None:
    _, _, fp = stable_fp
    assert np.rad2deg(fp.alpha) == pytest.approx(12.05, abs=0.05)
    assert fp.slope == pytest.approx(+0.7214, abs=0.002)
    assert fp.stable


def test_unstable_anchor_is_the_config_of_record(unstable_fp) -> None:
    _, _, fp = unstable_fp
    assert np.rad2deg(fp.beta) == pytest.approx(80.75, abs=0.01)
    assert np.rad2deg(fp.alpha) == pytest.approx(40.74, abs=0.05)
    assert fp.slope > 1.0


@pytest.mark.parametrize("d_alpha_deg", [-1.0, -0.5, 0.5, 1.0])
def test_step_count_matches_the_analytic_prediction(stable_fp, d_alpha_deg) -> None:
    """The identity that makes the step count trustworthy."""
    p, v, fp = stable_fp
    alpha0 = fp.alpha + np.deg2rad(d_alpha_deg)
    n = steps_to_converge(p, fp, v, alpha0, tol=TOL, stride_fn=slip_rf.stride)
    assert n is not None

    err0 = abs(alpha0 - fp.alpha) / fp.alpha
    analytic = np.log(TOL / err0) / np.log(abs(fp.slope))
    assert n == pytest.approx(np.ceil(analytic), abs=1.0)


def test_unstable_orbit_never_converges(unstable_fp) -> None:
    p, v, fp = unstable_fp
    assert steps_to_converge(p, fp, v, fp.alpha + np.deg2rad(0.5),
                             stride_fn=slip_rf.stride) is None


def test_start_inside_tolerance_costs_zero_steps(stable_fp) -> None:
    p, v, fp = stable_fp
    assert steps_to_converge(p, fp, v, fp.alpha, stride_fn=slip_rf.stride) == 0


def test_converges_to_agrees_with_the_step_count(stable_fp, unstable_fp) -> None:
    """The boolean is now a wrapper; it must not have changed meaning."""
    for p, v, fp in (stable_fp, unstable_fp):
        for d in (-1.0, -0.1, 0.0, 0.1, 1.0):
            alpha = fp.alpha + np.deg2rad(d)
            n = steps_to_converge(p, fp, v, alpha, stride_fn=slip_rf.stride)
            assert converges_to(p, fp, v, alpha, stride_fn=slip_rf.stride) is (n is not None)


def test_v_is_invariant_so_a_v_perturbation_can_never_converge(stable_fp) -> None:
    """Why basin_of_attraction's v axis is a mask, not an axis.

    The map conserves v, so the v term of the eq-13 error is a floor fixed by
    the initial offset. Anything at or beyond `tol` is unreachable whatever
    alpha does -- which is a property of the reduction, not a solver artifact.
    """
    p, v, fp = stable_fp
    res = slip_rf.stride(p, v, fp.alpha, fp.beta)
    assert res["v"] == pytest.approx(v, abs=1e-8)

    v_off = fp.v * (1.0 + 5.0 * TOL)
    grid = basin_of_attraction(p, fp, np.deg2rad([10.0, 12.0, 14.0]),
                               v_values=np.array([v_off]),
                               stride_fn=slip_rf.stride)
    assert not grid.any()


def test_basin_in_alpha_reports_counts_not_just_membership(stable_fp) -> None:
    p, _, fp = stable_fp
    alphas = fp.alpha + np.deg2rad(np.linspace(-2.0, 2.0, 9))
    out = basin_in_alpha(p, fp, alphas, stride_fn=slip_rf.stride)

    assert out["converged"].any()
    inside = out["steps"][out["converged"]]
    assert np.all(inside >= 0)
    # Nearer the fixed point is never slower to reach it.
    centre = int(np.argmin(np.abs(alphas - fp.alpha)))
    assert out["steps"][centre] == np.nanmin(out["steps"])
