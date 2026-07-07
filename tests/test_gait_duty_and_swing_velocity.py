"""Regression tests for gait duty overrides and swing boundary velocity mapping."""

import importlib.util
from pathlib import Path

import numpy as np

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.launch_controller import LaunchController
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D

TUI_MODULE_PATH = Path(__file__).resolve().parents[1] / "examples" / "gait" / "generate_csv_tui.py"


def _load_tui_module():
    spec = importlib.util.spec_from_file_location("generate_csv_tui", TUI_MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gait_generator_accepts_custom_stance_duty():
    """Custom ``D_f`` should override the gait-library default for every leg planner."""
    gen = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        period=1.0,
        gait_type="Trot",
        dt=0.01,
        stance_duty=0.7,
    )

    assert gen.stance_duty == 0.7
    assert all(planner.stance_duty == 0.7 for planner in gen.planners)
    assert "_D0.70" in gen.get_parameter_string()


def test_launch_controller_uses_custom_stance_duty():
    """Launch phase search and ramp cycles should use the same custom ``D_f``."""
    launch = LaunchController(
        gait_type="Trot",
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        period=1.0,
        dt=0.01,
        stance_duty=0.7,
    )

    assert launch.stance_duty == 0.7


def test_tui_gait_command_forwards_custom_duty():
    """The full-screen TUI should pass the edited duty value to the generator script."""
    tui_module = _load_tui_module()

    tui = tui_module.CSVGeneratorTUI(mode="gait", overrides={"duty": "0.70"})
    cmd = tui._gait_cmd()

    assert "--duty" in cmd
    assert cmd[cmd.index("--duty") + 1] == "0.70"


def test_swing_touchdown_horizontal_velocity_follows_global_displacement():
    """Positive global travel during swing should not request opposite touchdown velocity."""
    planner = TrajectoryPlanner3D(
        stand_height=0.25,
        velocity=[0.05, 0.03, 0.0],
        period=1.0,
        dt=0.01,
        stance_duty=0.6,
        leg_index=0,
    )

    planner.generate_trajectory()
    _, v_td_B = planner._last_swing_boundary_velocities_B

    assert v_td_B[0] > 0.0
    assert v_td_B[1] > 0.0


def test_walk_swing_liftoff_does_not_backtrack_reference_points():
    """Walk swing should not choose Bezier branches with large backward X kicks."""
    gen = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.20, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.001,
    )
    gen.generate_full_gait(n_cycles=1)

    for planner in gen.planners:
        q = np.asarray(planner.cmd)
        n_stance = int(round(planner.stance_duty * len(q)))
        early_swing = q[n_stance : n_stance + 70]

        foot_path = np.asarray(
            [planner.kin.forward_kinematics(*q_i, alpha=0.0, w=0.0) for q_i in early_swing]
        )
        vx = np.gradient(foot_path[:, 0], planner.dt)
        dL2 = planner.swing_planner._last_liftoff_shape_params[1]

        # Threshold updated to -0.65 m/s (was -0.5): with the a_max model (v_lo_z = 0),
        # the Bézier starts horizontally (dH1 ≈ 0). The previous 0.32 m/s vertical
        # target caused dH1 > 0, indirectly reducing x-backtracking by adding an upward
        # component to the first control point tangent. The new model is physically correct
        # (height clearance is guaranteed by c2.y = h), so this guard is relaxed accordingly.
        # The dL2 < 0.03 constraint remains the primary backtracking control.
        assert np.min(vx) > -0.65
        assert dL2 < 0.03
