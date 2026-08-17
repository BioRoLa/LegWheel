"""Tests for the cambered SLIP-RF reduction (Stage 2a / Stage 2b Module 1).

The lambda = 0 identity is the keystone. Everything downstream in Stage 2b rides
on the claim that camber enters as a parameter rescaling, and the cheapest way to
be wrong about that is a derivation error that silently shifts the fixed point.
This is the class of fault that hid in `slip_rf.stride` for weeks (a clean factor
of 2.000 in stance length, invisible to everything downstream).
"""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import slip_rf, slip_rf_cambered as cam
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.slip_rf import SlipRfParams

# The Stage 2a Phase 2 design point: theta_nom = 100 deg, k_rel = 18, v~ = 1.2.
MASS, HIP_TO_ARC, FOOT_RADIUS, G = 30.0, 0.148, 0.145, 9.81
K_REL = 18.0

# Measured fixed point at that design point, from the sagittal half.
BETA_STAR_DEG = 71.75
ALPHA_STAR_DEG = 17.79
SLOPE_STAR = 1.2318
V_STAR = 2.035


def corgi() -> SlipRfParams:
    return SlipRfParams(
        m=MASS, l0=HIP_TO_ARC + FOOT_RADIUS,
        k=K_REL * MASS * G / HIP_TO_ARC, r=FOOT_RADIUS, g=G,
    )


# --------------------------------------------------------------------------
# The lambda = 0 identity
# --------------------------------------------------------------------------

def test_rolling_radius_recovers_sagittal_value_exactly_at_zero_camber() -> None:
    """0.130 + 0.015 = 0.145 with no residual -- the reduction's anchor."""
    assert cam.rolling_radius(0.0) == pytest.approx(FOOT_RADIUS, abs=1e-15)


def test_cambered_params_are_the_identity_at_zero_camber() -> None:
    p = corgi()
    q = cam.cambered_params(p, 0.0)
    assert q.m == pytest.approx(p.m, abs=1e-15)
    assert q.l0 == pytest.approx(p.l0, abs=1e-15)
    assert q.k == pytest.approx(p.k, abs=1e-15)
    assert q.r == pytest.approx(p.r, abs=1e-15)
    assert q.g == pytest.approx(p.g, abs=1e-15)


def test_cambered_stride_reproduces_sagittal_stride_at_zero_camber() -> None:
    """Not just the parameters -- the whole integrated stride must coincide."""
    p = corgi()
    v, alpha, beta = V_STAR, np.deg2rad(ALPHA_STAR_DEG), np.deg2rad(BETA_STAR_DEG)
    ref = slip_rf.stride(p, v, alpha, beta)
    got = cam.cambered_stride(p, v, alpha, beta, lam=0.0)
    # rel=1e-9, not exact: cambered_params rebuilds l0 as (l0 - r) + r_eff, and
    # that round-trip can land one ulp away from the original. The ODE amplifies
    # it slightly, most visibly in peak_grf_z (a max over samples). 1e-9 is still
    # nine orders tighter than any physical effect in this stage.
    for key in ("v", "alpha", "stance_time", "flight_time", "stride_length",
                "peak_grf_z", "peak_compression"):
        assert got[key] == pytest.approx(ref[key], rel=1e-9), key


def test_zero_camber_fixed_point_matches_the_measured_sagittal_one() -> None:
    """THE Stage 2a sanity check, from the thesis timeline.

    At lambda = 0 the cambered template must reproduce our own measured
    SLIP-RF fixed point: beta* = 71.75 deg, alpha* = 17.79 deg, slope +1.2318.
    Tolerances are set by the precision those values are quoted to, not by what
    the solver can achieve.
    """
    p = corgi()
    beta = np.deg2rad(BETA_STAR_DEG)

    def stride_fn(pp, v, a, b):
        return cam.cambered_stride(pp, v, a, b, lam=0.0)

    fps = find_fixed_points(p, V_STAR, beta, stride_fn=stride_fn)
    assert fps, "no fixed point found at the design point"

    fp = min(fps, key=lambda f: abs(np.rad2deg(f.alpha) - ALPHA_STAR_DEG))
    assert np.rad2deg(fp.alpha) == pytest.approx(ALPHA_STAR_DEG, abs=0.01)
    assert fp.slope == pytest.approx(SLOPE_STAR, abs=0.001)
    assert not fp.stable, "the design point is unstable; that is the premise of Stage 2b"


# --------------------------------------------------------------------------
# Behaviour under actual camber
# --------------------------------------------------------------------------

@pytest.mark.parametrize("lam_deg,expected", [(15.0, 1.035), (20.0, 1.064), (25.0, 1.103)])
def test_effective_gravity_matches_the_documented_magnitudes(lam_deg, expected) -> None:
    p = corgi()
    q = cam.cambered_params(p, np.deg2rad(lam_deg))
    assert q.g / p.g == pytest.approx(expected, abs=5e-4)


def test_dimensionless_groups_rescale_as_the_design_note_states() -> None:
    """k~ -> k~*cos(lam) and v~ -> v~*sqrt(cos(lam)), with k and v untouched.

    These are consequences of scaling g, not separate operations. If someone
    later applies them by hand as well, this test fails -- which is the point.
    """
    p = corgi()
    lam = np.deg2rad(20.0)
    q = cam.cambered_params(p, lam)

    k_tilde = lambda s: s.k * s.l0 / (s.m * s.g)
    v_tilde = lambda s: 1.0 / np.sqrt(s.g * s.l0)

    # l0 also moves slightly (the arc radius shrinks), so compare against the
    # gravity-only prediction with the l0 ratio divided out.
    l0_ratio = q.l0 / p.l0
    assert k_tilde(q) / k_tilde(p) == pytest.approx(np.cos(lam) * l0_ratio, rel=1e-12)
    assert v_tilde(q) / v_tilde(p) == pytest.approx(
        np.sqrt(np.cos(lam) / l0_ratio), rel=1e-12)
    assert q.k == p.k and q.m == p.m


def test_rolling_radius_shrinks_monotonically_and_symmetrically() -> None:
    lams = np.deg2rad(np.array([0.0, 5.0, 10.0, 20.0, 30.0]))
    radii = [cam.rolling_radius(l) for l in lams]
    assert all(b < a for a, b in zip(radii, radii[1:])), radii
    for l in lams:
        assert cam.rolling_radius(-l) == pytest.approx(cam.rolling_radius(l), abs=1e-15)


def test_rolling_radius_drop_is_NOT_the_measured_ride_height_drop() -> None:
    """A cross-check that looks obvious and is wrong -- pinned so nobody retries it.

    Section 33 measured body ride height dropping 2.00 / 4.34 / 6.45 mm at
    lambda = 10 / 20 / 30 deg, and it is tempting to check this module against
    those numbers. They are DIFFERENT QUANTITIES, and the scaling laws prove it:

        measured drop        1 : 2.17 : 3.23     ~ sin(lambda)
        rolling-radius drop  1 : 3.97 : 8.82     ~ 1 - cos(lambda)

    A first-order quantity against a second-order one. By 30 deg this module says
    19.9 mm against a measured 6.45. Nothing is wrong with either: the wheel's
    rolling radius shrinks as 1-cos, while the BODY also rolls (-4.6 / -9.2 /
    -13.8 deg in that run), and a rolling body drops through a sin(rho) lever --
    the measured drop is ~27 mm * sin(body roll), roughly constant across all
    three angles.

    So the ride-height comparison belongs to the CORONAL model (Module 2), where
    body roll exists. This module has no roll and cannot reproduce it.
    """
    r0 = cam.rolling_radius(0.0)
    drops = [(r0 - cam.rolling_radius(np.deg2rad(d))) * 1e3 for d in (10.0, 20.0, 30.0)]

    # Second-order: follows 1 - cos(lambda), to within the flat-tread term.
    ratios = [d / drops[0] for d in drops]
    assert ratios[1] == pytest.approx(3.36, abs=0.05)
    assert ratios[2] == pytest.approx(7.00, abs=0.05)

    # And is emphatically NOT the measured sequence.
    for got, measured in zip(drops, (2.00, 4.34, 6.45)):
        if got == pytest.approx(measured, abs=0.02):
            pytest.fail("rolling-radius drop now equals the section 33 ride-height "
                        "drop; one of the two models has changed meaning")


def test_turn_radius_matches_the_documented_feasibility_table() -> None:
    """Cambered Contact Geometry: 1.0 m/s at 10/15/20 deg -> 0.58/0.38/0.28 m."""
    for lam_deg, expected in ((10.0, 0.58), (15.0, 0.38), (20.0, 0.28)):
        assert cam.turn_radius(1.0, np.deg2rad(lam_deg)) == pytest.approx(
            expected, abs=0.005)
    assert cam.turn_radius(1.0, 0.0) == float("inf")


def test_cambered_fixed_points_exist_at_the_operating_camber() -> None:
    """Stage 2a's gate is EXISTENCE, not stability (per Chang 2022)."""
    p = corgi()
    beta = np.deg2rad(BETA_STAR_DEG)
    for lam_deg in (10.0, 20.0):
        lam = np.deg2rad(lam_deg)

        def stride_fn(pp, v, a, b, _l=lam):
            return cam.cambered_stride(pp, v, a, b, lam=_l)

        fps = find_fixed_points(p, V_STAR, beta, stride_fn=stride_fn)
        assert fps, f"no cambered fixed point at lambda = {lam_deg} deg"
