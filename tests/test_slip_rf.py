"""Tests for the SLIP-RF model, the Corgi's reduced-order template."""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.models.slip_rf import SlipRfParams

CORGI = dict(m=30.0, l0=0.230, k=62322.0, r=0.145)


def corgi() -> SlipRfParams:
    return SlipRfParams(**CORGI)


def test_rest_length_must_exceed_foot_radius() -> None:
    with pytest.raises(ValueError):
        SlipRfParams(m=30.0, l0=0.1, k=1000.0, r=0.145)


@pytest.mark.parametrize("beta_deg", [50.0, 65.0, 80.0, 89.0])
def test_touchdown_height_consistency(beta_deg: float) -> None:
    """position() at touchdown agrees with the touchdown event (eq 33)."""
    p = corgi()
    beta = np.deg2rad(beta_deg)
    _, z = slip_rf.position(p, p.l0, p.phi_touchdown(beta))
    assert z == pytest.approx(p.touchdown_height(beta), abs=1e-14)


@pytest.mark.parametrize("beta_deg", [50.0, 65.0, 80.0])
def test_mass_sits_rest_length_from_foot_center(beta_deg: float) -> None:
    """The mass is exactly (l0 - r) from the foot center, for any landing angle.

    This is the property the general G-SLIP parametrization fails to hold, and
    the reason the Corgi work uses SLIP-RF.
    """
    p = corgi()
    beta = np.deg2rad(beta_deg)
    phi = p.phi_touchdown(beta)
    x, z = slip_rf.position(p, p.l0, phi)
    # Foot center rolls along z = r, at x = r*phi.
    assert np.hypot(x - p.r * phi, z - p.r) == pytest.approx(p.l0 - p.r, rel=1e-14)


def test_touchdown_leg_is_protracted() -> None:
    """The foot lands ahead of the mass, so the leg compresses on contact."""
    p = corgi()
    beta = np.deg2rad(70.0)
    assert p.phi_touchdown(beta) < 0
    x, _ = slip_rf.position(p, p.l0, p.phi_touchdown(beta))
    assert x < 0
    dl, _ = slip_rf.touchdown_rates(p, 1.8, np.deg2rad(10.0), beta)
    assert dl < 0


def test_jacobian_matches_finite_difference() -> None:
    p = corgi()
    length, phi, h = 0.21, -0.25, 1e-7
    analytic = slip_rf.jacobian(p, length, phi)
    numeric = np.zeros((2, 2))
    for j, (dl, dp) in enumerate([(h, 0.0), (0.0, h)]):
        plus = np.array(slip_rf.position(p, length + dl, phi + dp))
        minus = np.array(slip_rf.position(p, length - dl, phi - dp))
        numeric[:, j] = (plus - minus) / (2 * h)
    assert numeric == pytest.approx(analytic, abs=1e-6)


def test_jacobian_determinant_closed_form() -> None:
    p = corgi()
    for length in (0.19, 0.21, 0.23):
        for phi in (-0.3, 0.0, 0.3):
            assert slip_rf.jacobian_det(p, length, phi) == pytest.approx(
                np.linalg.det(slip_rf.jacobian(p, length, phi)), abs=1e-14
            )


def test_hessians_match_finite_difference() -> None:
    p = corgi()
    length, phi, h = 0.21, -0.25, 1e-7
    h_x, h_z = slip_rf.hessians(p, length, phi)
    num_x, num_z = np.zeros((2, 2)), np.zeros((2, 2))
    for j, (dl, dp) in enumerate([(h, 0.0), (0.0, h)]):
        plus = slip_rf.jacobian(p, length + dl, phi + dp)
        minus = slip_rf.jacobian(p, length - dl, phi - dp)
        num_x[:, j] = (plus[0] - minus[0]) / (2 * h)
        num_z[:, j] = (plus[1] - minus[1]) / (2 * h)
    assert num_x == pytest.approx(h_x, abs=1e-6)
    assert num_z == pytest.approx(h_z, abs=1e-6)


@pytest.mark.parametrize("v_tilde", [0.8, 1.2, 1.6])
def test_stance_conserves_energy(v_tilde: float) -> None:
    p = corgi()
    v = v_tilde * np.sqrt(p.g * p.l0)
    sol = slip_rf.simulate_stance(
        p, v, np.deg2rad(20.0), np.deg2rad(70.0), rtol=1e-12, atol=1e-14
    )
    e = [sum(slip_rf.energy(p, *sol.y[:, i]).values()) for i in range(sol.y.shape[1])]
    assert (max(e) - min(e)) / abs(e[0]) < 1e-9


def test_stride_conserves_speed() -> None:
    """Conservative model returning to the touchdown height keeps its speed."""
    p = corgi()
    v = 1.2 * np.sqrt(p.g * p.l0)
    res = slip_rf.stride(p, v, np.deg2rad(20.0), np.deg2rad(70.0))
    assert res["v"] == pytest.approx(v, rel=1e-6)
    assert res["stance_time"] > 0
    assert res["flight_time"] > 0


def test_stance_length_is_measured_from_the_touchdown_MASS_position() -> None:
    """stance_length must subtract the touchdown offset, not report x_lo raw.

    `position()` has its horizontal origin at the touchdown CONTACT POINT, so
    the mass already sits at x_td != 0 when stance begins. Reporting x_lo alone
    understated the stance displacement -- by exactly a factor of two at a
    symmetric fixed point, where phi_td = -phi_lo makes x_td = -x_lo.

    The bug was silent: nothing downstream could detect it, and it propagated
    into stride_length and therefore into every design speed derived from
    stride_length / period. This pins the definition.
    """
    p = corgi()
    v = 1.2 * np.sqrt(p.g * p.l0)
    res = slip_rf.stride(p, v, np.deg2rad(20.0), np.deg2rad(70.0))

    sol = res["solution"]
    x_td, _ = slip_rf.position(p, float(sol.y[0, 0]), float(sol.y[1, 0]))
    x_lo, _ = slip_rf.position(p, *sol.y_events[0][0][:2])

    assert x_td != pytest.approx(0.0, abs=1e-6), (
        "the touchdown offset must be non-zero, or this test proves nothing"
    )
    assert res["stance_length"] == pytest.approx(x_lo - x_td, rel=1e-12)
    assert res["stance_length"] != pytest.approx(x_lo, rel=1e-6)


def test_stride_length_is_stance_plus_flight() -> None:
    """stride_length must be built on the corrected stance displacement."""
    p = corgi()
    v = 1.2 * np.sqrt(p.g * p.l0)
    res = slip_rf.stride(p, v, np.deg2rad(20.0), np.deg2rad(70.0))

    vx_lo = (res["stride_length"] - res["stance_length"]) / res["flight_time"]
    assert vx_lo > 0
    assert res["stride_length"] == pytest.approx(
        res["stance_length"] + vx_lo * res["flight_time"], rel=1e-12
    )


def test_arc_roll_is_the_rim_distance_not_the_body_displacement() -> None:
    """arc_roll = r*d_phi, which the foot-arc budget wants.

    These two coincided numerically while the bug was present, because
    r = 0.145 is within 2% of l - r = 0.148 at the Corgi's nominal stance --
    so the arc-budget guard was right for the wrong reason. Keep them distinct.
    """
    p = corgi()
    v = 1.2 * np.sqrt(p.g * p.l0)
    res = slip_rf.stride(p, v, np.deg2rad(20.0), np.deg2rad(70.0))

    sol = res["solution"]
    d_phi = float(sol.y_events[0][0][1]) - float(sol.y[1, 0])
    assert res["arc_roll"] == pytest.approx(p.r * d_phi, rel=1e-12)
    assert abs(res["arc_roll"]) < abs(res["stance_length"])


def test_peak_grf_is_reported_and_positive() -> None:
    p = corgi()
    v = 1.2 * np.sqrt(p.g * p.l0)
    res = slip_rf.stride(p, v, np.deg2rad(20.0), np.deg2rad(70.0))
    assert res["peak_grf_z"] > p.m * p.g  # must exceed body weight while running
    assert 0 < res["peak_compression"] < p.l0 - p.r


def test_non_compressing_touchdown_is_rejected() -> None:
    """A retracted leg (mirrored phi) extends on contact and must be refused."""
    p = corgi()
    beta = np.deg2rad(70.0)

    class Mirrored(SlipRfParams):
        @staticmethod
        def phi_touchdown(b: float) -> float:
            return float(np.pi / 2 - b)

    with pytest.raises(GSlipFailure, match="not compressing"):
        slip_rf.simulate_stance(Mirrored(**CORGI), 0.5, np.deg2rad(1.0), beta)
