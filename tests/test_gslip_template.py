"""Tests for the parametrized fixed-point stride template."""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import slip_rf
from legwheel.planners import gslip_template as tpl

CORGI = slip_rf.SlipRfParams(m=30.0, l0=0.230, k=62322.0, r=0.145)
V = 1.2 * np.sqrt(9.81 * 0.230)
ALPHA = np.deg2rad(20.0)
BETA = np.deg2rad(70.0)


def test_quintic_matches_boundary_conditions() -> None:
    """Six boundary conditions determine the quintic exactly."""
    start, end, duration = (0.1, -2.0, 5.0), (-0.3, 1.5, -4.0), 0.12
    c = tpl.quintic(duration, start, end)
    for order, (s, e) in enumerate(zip(start, end)):
        assert tpl.polyval(c, 0.0, order) == pytest.approx(s, abs=1e-9)
        assert tpl.polyval(c, duration, order) == pytest.approx(e, abs=1e-7)


def test_polyval_derivative_matches_finite_difference() -> None:
    c = tpl.quintic(0.2, (0.0, 1.0, 0.0), (0.5, -1.0, 0.0))
    t, h = 0.11, 1e-6
    numeric = (tpl.polyval(c, t + h) - tpl.polyval(c, t - h)) / (2 * h)
    assert tpl.polyval(c, t, order=1) == pytest.approx(numeric, abs=1e-6)


def test_trapezoid_endpoints_and_span() -> None:
    duration, start, end = 0.2, -0.35, 0.35
    pos, vel = tpl.trapezoid(duration, start, end)
    assert float(pos(0.0)) == pytest.approx(start, abs=1e-12)
    assert float(pos(duration)) == pytest.approx(end, abs=1e-12)
    # Velocity ramps from zero and returns to zero.
    assert float(vel(0.0)) == pytest.approx(0.0, abs=1e-12)
    assert float(vel(duration)) == pytest.approx(0.0, abs=1e-12)


def test_trapezoid_velocity_integrates_to_span() -> None:
    duration, start, end = 0.2, -0.35, 0.35
    _, vel = tpl.trapezoid(duration, start, end)
    t = np.linspace(0.0, duration, 20001)
    assert np.trapezoid(vel(t), t) == pytest.approx(end - start, rel=1e-6)


def test_trapezoid_rejects_bad_ramp() -> None:
    for bad in (0.0, -0.1, 0.75):
        with pytest.raises(ValueError):
            tpl.trapezoid(0.2, 0.0, 1.0, ramp_fraction=bad)


def test_template_phase_durations() -> None:
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    res = slip_rf.stride(CORGI, V, ALPHA, BETA)
    assert t.stance_time == pytest.approx(res["stance_time"])
    assert t.flight_time == pytest.approx(res["flight_time"])
    assert t.period == pytest.approx(t.stance_time + t.flight_time)
    assert 0.0 < t.duty_factor < 1.0


def test_template_endpoints_match_the_fixed_point() -> None:
    """Stance starts at touchdown and ends at the rest length."""
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    assert t.leg_angle(0.0) == pytest.approx(CORGI.phi_touchdown(BETA), abs=1e-9)
    assert t.leg_length(0.0) == pytest.approx(CORGI.l0, abs=1e-9)
    assert t.leg_length(t.stance_time) == pytest.approx(CORGI.l0, abs=1e-6)


def test_template_returns_to_touchdown_angle_by_end_of_flight() -> None:
    """The flight sweep must reposition the leg for the next touchdown."""
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    assert t.leg_angle(t.period - 1e-12) == pytest.approx(
        CORGI.phi_touchdown(BETA), abs=1e-6
    )


def test_template_leg_extends_forward_through_stance() -> None:
    """Leg angle sweeps from negative (protracted) to positive (retracted)."""
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    assert t.leg_angle(0.0) < 0.0
    assert t.leg_angle(t.stance_time) > t.leg_angle(0.0)


def test_quintic_reproduces_the_stance_trajectory() -> None:
    """Interior deviation of the quintic from the true stance motion is small."""
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    err = tpl.tracking_error(t)
    # Leg angle sweeps ~0.7 rad and length ~0.02 m over stance.
    assert err["max_angle_error"] < 0.02, err
    assert err["max_length_error"] < 2e-3, err


def test_sample_shapes_and_stance_mask() -> None:
    t = tpl.build_template(CORGI, V, ALPHA, BETA)
    s = t.sample(n=50)
    assert all(len(s[key]) == 50 for key in ("t", "leg_angle", "leg_length"))
    assert s["in_stance"][0] and not s["in_stance"][-1]
    assert np.all(np.diff(s["t"]) > 0)
