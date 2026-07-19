"""Unit tests for the phase-locked attitude oscillation waveform (Plan doc V2).

Standalone checks on legwheel.planners.attitude_oscillation only -- no gait
generator / solver involved yet (that coupling is Step 2/3 of the plan).
"""
import numpy as np
import pytest

from legwheel.planners.attitude_oscillation import (
    attitude_reference,
    ds_dphi,
    oscillation_rate,
    s,
)


def test_waveform_bounded():
    phi = np.linspace(0.0, 1.0, 1001)
    assert np.all(s(phi) >= -1.0 - 1e-12)
    assert np.all(s(phi) <= 1.0 + 1e-12)


def test_waveform_zero_mean_over_one_period():
    phi = np.linspace(0.0, 1.0, 100001, endpoint=False)
    assert np.abs(np.mean(s(phi))) < 1e-9


def test_waveform_half_period_antisymmetry():
    phi = np.linspace(0.0, 0.5, 501, endpoint=False)
    assert np.allclose(s(phi + 0.5), -s(phi), atol=1e-12)


def test_waveform_peaks_at_stance_transition_phases():
    # Bound/Pace GAIT_LIBRARY phase_offsets transition at phi_g = 0.0, 0.5.
    assert np.isclose(s(0.0), 1.0)
    assert np.isclose(s(0.5), -1.0)
    assert np.isclose(s(0.25), 0.0, atol=1e-12)
    assert np.isclose(s(0.75), 0.0, atol=1e-12)


def test_derivative_matches_numerical_gradient():
    # np.gradient uses one-sided differences at the array edges, which are not
    # representative for a periodic signal; compare on the interior only.
    phi = np.linspace(0.0, 1.0, 2001, endpoint=False)
    numerical = np.gradient(s(phi), phi)
    assert np.allclose(ds_dphi(phi)[5:-5], numerical[5:-5], atol=5e-3)


@pytest.mark.parametrize("phase_lead", [0.0, 0.1, -0.15])
def test_attitude_reference_matches_amplitude_at_phase_lead(phase_lead):
    amplitude = np.deg2rad(6.0)
    # s peaks at argument 0, so phi_g - phase_lead == 0 -> phi_g == phase_lead
    phi_peak = phase_lead % 1.0
    value = attitude_reference(phi_peak, amplitude, phase_lead)
    assert np.isclose(value, amplitude, atol=1e-9)


def test_attitude_reference_zero_amplitude_is_always_zero():
    phi = np.linspace(0.0, 1.0, 50)
    assert np.allclose(attitude_reference(phi, 0.0), 0.0)


def test_oscillation_rate_bounded_and_scales_with_period():
    amplitude = np.deg2rad(5.0)
    period = 0.8
    phi = np.linspace(0.0, 1.0, 1001)
    rate = np.array([oscillation_rate(p, amplitude, period) for p in phi])

    peak_rate = amplitude * 2.0 * np.pi / period
    assert np.all(np.abs(rate) <= peak_rate + 1e-9)

    # Halving the period doubles the peak rate for the same amplitude.
    rate_half_period = np.array(
        [oscillation_rate(p, amplitude, period / 2.0) for p in phi]
    )
    assert np.isclose(np.max(np.abs(rate_half_period)), 2.0 * np.max(np.abs(rate)))


def test_oscillation_rate_zero_amplitude_is_always_zero():
    phi = np.linspace(0.0, 1.0, 50)
    for p in phi:
        assert oscillation_rate(p, 0.0, period=1.0) == 0.0
