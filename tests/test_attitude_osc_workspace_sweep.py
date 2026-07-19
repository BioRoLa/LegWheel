"""V5 (Attitude Oscillation Compensation plan): workspace feasibility sweep.

Confirms that the existing Workspace Guard / step-scaling logic in
GaitGenerator3D.__init__ either passes cleanly or scales gracefully as
attitude_osc_amplitude is swept 0-8deg on Bound and Pace -- it must not
produce IK failures (RuntimeError from CorgiLegKinematics.inverse_kinematics,
see corgi_leg.py:579).

Important asymmetry this test is designed to catch: the Workspace Guard
(gait_generator_3d.py:186-233) and the Dynamic Step-Height Scaling
(gait_generator_3d.py:235-278) are both computed from the BASE hip
velocities (calc_hip_vels on the raw twist) BEFORE the attitude-oscillation
hip_velocity_fn is added in the stance loop (trajectory_planning_3d.py). So
neither guard "sees" the oscillation's contribution to beta/gamma usage --
a large enough amplitude could in principle push a leg's realized beta/gamma
past the geometric limit without the guard ever tripping. This sweep checks
the REALIZED beta/gamma (from generate_full_gait's actual IK output) against
the geometric limits directly, independent of whether the (amplitude-blind)
guard fired.
"""
import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.planners.gait_generator_3d import GaitGenerator3D

COMMON_KW = dict(stand_height=0.25, twist=[0.0, 0.10, 0.0], period=1.0, dt=0.004)

AMPLITUDES_DEG = [0.0, 2.0, 4.0, 6.0, 8.0]
BETA_MAX_DEG = RobotParams.BETA_MAX_DEG
GAMMA_MAX_DEG = RobotParams.GAMMA_MAX_DEG


@pytest.mark.parametrize("gait_type", ["Bound", "Pace"])
@pytest.mark.parametrize("amplitude_deg", AMPLITUDES_DEG)
def test_sweep_does_not_raise_ik_failure(gait_type, amplitude_deg):
    """No IK RuntimeError anywhere in the 0-8deg amplitude grid."""
    gen = GaitGenerator3D(
        gait_type=gait_type,
        attitude_osc_amplitude=np.deg2rad(amplitude_deg),
        **COMMON_KW,
    )
    cmds = gen.generate_full_gait(n_cycles=1)
    assert np.all(np.isfinite(cmds))


@pytest.mark.parametrize("gait_type", ["Bound", "Pace"])
def test_sweep_reports_realized_beta_gamma_usage(gait_type):
    """Track realized beta/gamma usage across the amplitude grid.

    Not a hard pass/fail gate on usage ratio (the guard is explicitly
    allowed to let usage climb toward the limit as amplitude grows -- V5
    only requires it "passes cleanly or scales gracefully", not that usage
    stays flat). This asserts the harder invariant: realized beta/gamma
    must never exceed the geometric limit itself, since exceeding it is
    exactly the failure mode the (amplitude-blind) guard could miss.
    """
    usage_by_amplitude = {}
    for amplitude_deg in AMPLITUDES_DEG:
        gen = GaitGenerator3D(
            gait_type=gait_type,
            attitude_osc_amplitude=np.deg2rad(amplitude_deg),
            **COMMON_KW,
        )
        cmds = gen.generate_full_gait(n_cycles=1)
        # cmds columns: [FL_Theta, FL_Beta, FL_Gamma, FR_..., RR_..., RL_...]
        beta_cols = cmds[:, [1, 4, 7, 10]]
        gamma_cols = cmds[:, [2, 5, 8, 11]]
        max_beta_deg = np.rad2deg(np.max(np.abs(beta_cols)))
        max_gamma_deg = np.rad2deg(np.max(np.abs(gamma_cols)))
        usage_by_amplitude[amplitude_deg] = (max_beta_deg, max_gamma_deg)

        assert max_beta_deg <= BETA_MAX_DEG + 1e-6, (
            f"{gait_type} amplitude={amplitude_deg}deg: realized beta "
            f"{max_beta_deg:.2f}deg exceeds geometric limit {BETA_MAX_DEG}deg "
            f"-- amplitude-blind Workspace Guard failed to catch this."
        )
        assert max_gamma_deg <= GAMMA_MAX_DEG + 1e-6, (
            f"{gait_type} amplitude={amplitude_deg}deg: realized gamma "
            f"{max_gamma_deg:.2f}deg exceeds geometric limit {GAMMA_MAX_DEG}deg "
            f"-- amplitude-blind Workspace Guard failed to catch this."
        )

    # Usage should be monotonically non-decreasing-ish as amplitude grows
    # (informational -- printed for the V6 handoff, not asserted strictly
    # since step-height scaling can interact nonlinearly).
    print(f"\n{gait_type} realized beta/gamma usage by amplitude (deg):")
    for amplitude_deg, (max_beta_deg, max_gamma_deg) in usage_by_amplitude.items():
        print(
            f"  A={amplitude_deg:4.1f}deg -> beta={max_beta_deg:5.2f}/{BETA_MAX_DEG}deg, "
            f"gamma={max_gamma_deg:5.2f}/{GAMMA_MAX_DEG}deg"
        )
