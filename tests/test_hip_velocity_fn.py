"""Regression tests for TrajectoryPlanner3D's optional hip_velocity_fn hook
(Step 2 of the Attitude Oscillation Compensation plan, V1 in the verify plan).

hip_velocity_fn=None (the default) must reproduce the pre-existing constant-
velocity stance behavior exactly; a nonzero callback must actually perturb
the resulting joint trajectory, confirming the hook is wired into the stance
loop.
"""
import numpy as np

from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D

STAND_H = 0.25
VELOCITY = [0.15, 0.0, 0.0]


def _planner(**kwargs):
    return TrajectoryPlanner3D(
        stand_height=STAND_H, velocity=VELOCITY, leg_index=0, dt=0.001, **kwargs
    )


def test_default_hip_velocity_fn_matches_explicit_none():
    cmd_default = np.array(_planner().generate_trajectory())
    cmd_explicit_none = np.array(_planner(hip_velocity_fn=None).generate_trajectory())
    assert np.array_equal(cmd_default, cmd_explicit_none)


def test_zero_hip_velocity_fn_matches_no_op_default():
    cmd_default = np.array(_planner().generate_trajectory())
    cmd_zero_fn = np.array(
        _planner(hip_velocity_fn=lambda t: np.zeros(3)).generate_trajectory()
    )
    assert cmd_default.shape == cmd_zero_fn.shape
    assert np.allclose(cmd_default, cmd_zero_fn, atol=1e-12)


def test_nonzero_hip_velocity_fn_perturbs_trajectory():
    cmd_default = np.array(_planner().generate_trajectory())
    cmd_perturbed = np.array(
        _planner(hip_velocity_fn=lambda t: np.array([0.05, 0.0, 0.0])).generate_trajectory()
    )
    assert cmd_default.shape == cmd_perturbed.shape
    assert not np.allclose(cmd_default, cmd_perturbed, atol=1e-6)


def test_hip_velocity_fn_receives_local_stance_time():
    """The callback argument should be local stance time starting near 0, not
    a global/wrapped gait phase -- confirms the calling convention documented
    in the constructor docstring."""
    seen_times = []

    def _record(t):
        seen_times.append(t)
        return np.zeros(3)

    _planner(hip_velocity_fn=_record).generate_trajectory()

    assert len(seen_times) > 0
    assert seen_times[0] == 0.001  # first call is at t = dt (touchdown sample is appended separately)
    assert all(t2 > t1 for t1, t2 in zip(seen_times, seen_times[1:]))
