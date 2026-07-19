"""
Phase-locked pitch/roll oscillation reference for line-support gaits (Bound, Pace).

Kinematics-only, feedforward reference-shaping utilities. No dynamic model,
force, or feedback is used here -- see:
    Biorola Notes/06_Research/Drafts/CH4_Legged_Locomotion/Theory/CH4_Attitude_Oscillation_Compensation.md
    Biorola Notes/06_Research/Drafts/CH4_Legged_Locomotion/TODO_AND_PLANNING/CH4_Attitude_Oscillation_Compensation_Plan.md

The waveform is a single cosine, chosen so that its peaks land exactly on the
Bound/Pace stance-pair transition phases (phi_g = 0.0, 0.5), matching the
GAIT_LIBRARY phase_offsets convention in gait_generator_3d.py.
"""

import numpy as np


def s(phi_g: float | np.ndarray) -> float | np.ndarray:
    """
    Periodic compensation waveform, normalized to [-1, 1].

    s(phi_g) = cos(2*pi*phi_g). Peaks at phi_g = 0.0 (value +1) and
    phi_g = 0.5 (value -1) -- the stance-pair transition phases for Bound
    and Pace -- and is zero at the stance/swing midpoints (0.25, 0.75).

    Args:
        phi_g (float | np.ndarray): Normalized gait phase, wraps at period 1.

    Returns:
        float | np.ndarray: Waveform value in [-1, 1].
    """
    return np.cos(2.0 * np.pi * phi_g)


def ds_dphi(phi_g: float | np.ndarray) -> float | np.ndarray:
    """
    Closed-form derivative of ``s`` with respect to ``phi_g``.

    Args:
        phi_g (float | np.ndarray): Normalized gait phase.

    Returns:
        float | np.ndarray: d(s)/d(phi_g).
    """
    return -2.0 * np.pi * np.sin(2.0 * np.pi * phi_g)


def attitude_reference(
    phi_g: float | np.ndarray,
    amplitude: float,
    phase_lead: float = 0.0,
) -> float | np.ndarray:
    """
    Commanded attitude offset (position form), Theory doc Section 3.2.

    phi^cmd(phi_g) = amplitude * s(phi_g - phase_lead)

    Used for Bound's pitch channel or Pace's roll channel. Position form is
    for logging/visualization and for the attitude-tracking ablation check
    (Plan doc V4) -- it is not injected directly during active gait (Theory
    doc Section 4.1); ``oscillation_rate`` below is the velocity form
    actually injected into the rolling-Jacobian stance solver.

    Args:
        phi_g (float | np.ndarray): Normalized gait phase.
        amplitude (float): Oscillation amplitude (rad). Signed so a positive
            value tilts the currently-unsupported end upward at phi_g=0.
        phase_lead (float): Anticipatory phase shift in [0, 1) (Theory doc
            Section 3.2, delta_phi). Default 0.0 = in phase with the
            stance-pair transition.

    Returns:
        float | np.ndarray: Commanded attitude offset (rad).
    """
    return amplitude * s(phi_g - phase_lead)


def oscillation_rate(
    phi_g: float,
    amplitude: float,
    period: float,
    phase_lead: float = 0.0,
) -> float:
    """
    Instantaneous attitude rate (velocity form), Theory doc Section 4.1.

    d(phi^cmd)/dt = amplitude * ds_dphi(phi_g - phase_lead) / period

    This is the value injected as omega_y (Bound) or omega_x (Pace) into the
    per-leg hip velocity before the rolling-Jacobian stance solve -- never as
    a generic CoM twist field (Theory doc Section 4.3).

    Args:
        phi_g (float): Normalized gait phase at the current instant.
        amplitude (float): Oscillation amplitude (rad), see
            ``attitude_reference``.
        period (float): Gait period T (s). Converts the phase-domain
            derivative into a time-domain rate.
        phase_lead (float): Anticipatory phase shift in [0, 1).

    Returns:
        float: Commanded attitude rate (rad/s).
    """
    return amplitude * ds_dphi(phi_g - phase_lead) / period
