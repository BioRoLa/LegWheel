"""Regression tests for the lateral contact model in foot_rim_contact_fk.

A flat-tread wheel previously snapped its contact edge ``w`` from +half_w to
-half_w the instant gamma crossed zero. That ~40 mm lateral jump in the predicted
ground-contact point injected a velocity impulse into the Rolling Jacobian and
produced spurious body yaw during lateral motion. The contact model now slides the
center of pressure continuously across gamma = 0 (pressure-center ramp + R4 fillet),
so the contact point and its gamma-derivative stay bounded and continuous.
"""
import numpy as np

from legwheel.models.corgi_leg import CorgiLegKinematics


def _contact_y(leg, theta, beta, gamma):
    alpha, w = leg.foot_rim_contact_fk(theta, beta, gamma)
    return leg.forward_kinematics(theta, beta, gamma, alpha=alpha, w=w)[1]


def test_lateral_contact_is_continuous_across_gamma_zero():
    """Contact Y must not jump as gamma sweeps through zero."""
    leg = CorgiLegKinematics(0)
    theta, beta = leg.theta0, -0.001
    gammas = np.deg2rad(np.linspace(-5.0, 5.0, 401))
    ys = np.array([_contact_y(leg, theta, beta, g) for g in gammas])

    # Step between adjacent samples (~0.025 deg apart) must stay tiny.
    # Old hard-edge model produced a single ~40 mm jump here.
    max_step = np.max(np.abs(np.diff(ys)))
    assert max_step < 1e-3, f"contact Y discontinuity {max_step*1000:.2f} mm at gamma=0"


def test_lateral_contact_is_centered_at_zero_tilt():
    """With no lateral tilt the contact sits on the tread center (w = 0)."""
    leg = CorgiLegKinematics(0)
    _, w = leg.foot_rim_contact_fk(leg.theta0, 0.0, 0.0)
    assert abs(w) < 1e-9


def test_lateral_contact_saturates_at_half_width():
    """Contact magnitude never exceeds the physical half-thickness."""
    leg = CorgiLegKinematics(0)
    half_w = leg.wheel_thickness / 2.0
    for gd in (-60.0, -30.0, 30.0, 60.0):
        _, w = leg.foot_rim_contact_fk(leg.theta0, -0.001, np.deg2rad(gd))
        assert abs(w) <= half_w + 1e-12


def test_contact_edge_sign_follows_tilt_direction():
    """Positive/negative tilt must pick opposite tread edges (no sign flip bug)."""
    leg = CorgiLegKinematics(0)
    _, w_pos = leg.foot_rim_contact_fk(leg.theta0, -0.001, np.deg2rad(5.0))
    _, w_neg = leg.foot_rim_contact_fk(leg.theta0, -0.001, np.deg2rad(-5.0))
    assert np.sign(w_pos) == -np.sign(w_neg)
    assert w_pos != 0.0
