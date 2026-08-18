"""Tests for the closed-wheel contact envelope (Stage 1, log sections 66-72)."""

from __future__ import annotations

import numpy as np

from legwheel.models.leg_kinematics import closed_wheel_eccentricity


def test_the_designed_closure_angle_is_concentric() -> None:
    """The linkage closes the wheel at theta = 17.00 deg with every rim-arc
    center on the axle -- the envelope's k=1 eccentricity vanishes there."""
    assert closed_wheel_eccentricity(np.deg2rad(17.0)) < 1e-6


def test_closure_error_costs_about_a_millimetre_per_degree() -> None:
    """The open-side envelope slope, pinned so the calibration story stays
    checkable: ~0.89 mm of k=1 eccentricity per degree of closure error
    (measured on the sim's V-curve: ~0.5 -- the envelope is the documented
    upper bound, factor ~1.8 of contact-patch smoothing between them)."""
    e18 = closed_wheel_eccentricity(np.deg2rad(18.0))
    e19 = closed_wheel_eccentricity(np.deg2rad(19.0))
    assert 0.0007 < e18 < 0.0011          # ~0.865 mm at +1 deg
    slope = (e19 - e18) * 1e3             # mm per degree
    assert 0.7 < slope < 1.1
