import numpy as np
import pytest

from legwheel.planners.hybrid import (
    ContactCandidate,
    ContactState,
    GaitConstraints,
    HybridPlanningRequest,
    HybridTrajectory,
    InitialRobotState,
    MotionMode,
    RectangularObstacle,
    RimId,
    SwingTarget,
    TerrainProfile,
    TraversalGoal,
    write_trajectory_csv,
)


def test_flat_and_rectangular_terrain_contract():
    obstacle = RectangularObstacle("box_1", [0.4, -0.2], [0.6, 0.2], 0.05)
    terrain = TerrainProfile(obstacles=[obstacle])

    assert terrain.ground_height_m == 0.0
    assert terrain.obstacles == (obstacle,)
    assert not obstacle.min_xy_world_m.flags.writeable


def test_contact_and_swing_positions_are_world_frame_immutable_vectors():
    state = ContactState(RimId.FOOT, 0.1, [0.5, 0.2, 0.05], "box_top")
    candidate = ContactCandidate(
        RimId.LEFT, 0.2, [0.4, 0.1, 0.0], 0.0, 0.01, True, "ground"
    )
    target = SwingTarget([0.7, -0.2, 0.08], RimId.RIGHT, -0.3, 0.04)

    assert state.point_world_m.shape == (3,)
    assert candidate.edge_margin_m == pytest.approx(0.01)
    assert target.target_position_world_m[2] == pytest.approx(0.08)
    with pytest.raises(ValueError):
        state.point_world_m[0] = 1.0


@pytest.mark.parametrize(
    "factory",
    [
        lambda: SwingTarget([0, 0, 0], RimId.FOOT, 0.0, -0.01),
        lambda: ContactCandidate(RimId.FOOT, 0.0, [0, 0, 0], 0.0, -0.01, True, "ground"),
        lambda: RectangularObstacle("box", [1, 0], [0, 1], 0.1),
        lambda: ContactState(RimId.FOOT, 0.0, [0, 0], "ground"),
    ],
)
def test_invalid_planning_contracts_are_rejected(factory):
    with pytest.raises(ValueError):
        factory()


def test_terrain_surface_ids_are_unique():
    obstacle = RectangularObstacle("ground", [0, 0], [1, 1], 0.1)
    with pytest.raises(ValueError, match="unique"):
        TerrainProfile(obstacles=(obstacle,))


def test_offline_planning_request_contains_complete_pre_execution_input():
    initial = InitialRobotState([0, 0, 0.3], [0, 0, 0], np.zeros((4, 3)))
    goal = TraversalGoal([[0, 0, 0.3], [1.0, 0, 0.3]])
    request = HybridPlanningRequest(
        TerrainProfile(), initial, goal, GaitConstraints(sample_period_s=0.02)
    )

    assert request.goal.body_path_world_m[-1, 0] == pytest.approx(1.0)
    assert request.constraints.sample_period_s == pytest.approx(0.02)
    assert not request.initial_state.joint_position_rad.flags.writeable


def _two_sample_trajectory():
    modes = [
        [MotionMode.ROLL] * 4,
        [MotionMode.ROLL, MotionMode.SWING, MotionMode.ROLL, MotionMode.ROLL],
    ]
    rims = [[RimId.FOOT] * 4, [RimId.FOOT] * 4]
    return HybridTrajectory(
        time_s=[0.0, 0.02],
        body_pose_world=np.zeros((2, 6)),
        joint_position_rad=np.zeros((2, 4, 3)),
        modes=modes,
        rims=rims,
        alpha_rad=np.zeros((2, 4)),
        foothold_world_m=np.zeros((2, 4, 3)),
        contact_active=[[True] * 4, [True, False, True, True]],
        swing_phase=[[0.0] * 4, [0.0, 0.25, 0.0, 0.0]],
        stability_margin_m=[0.03, 0.02],
    )


def test_full_trajectory_has_synchronized_four_leg_metadata():
    trajectory = _two_sample_trajectory()

    assert trajectory.sample_count == 2
    assert trajectory.modes[1][1] is MotionMode.SWING
    assert not trajectory.contact_active[1, 1]
    assert trajectory.joint_position_rad.shape == (2, 4, 3)
    assert not trajectory.body_pose_world.flags.writeable


def test_trajectory_csv_uses_deterministic_shared_schema(tmp_path):
    output = write_trajectory_csv(_two_sample_trajectory(), tmp_path / "trajectory.csv")
    rows = output.read_text().splitlines()

    assert len(rows) == 3
    assert rows[0].startswith("time_s,body_x,body_y,body_z,body_roll,body_pitch,body_yaw")
    assert "leg1_mode" in rows[0]
    assert "stability_margin" in rows[0]
    assert "SWING" in rows[2]


def test_full_trajectory_rejects_unsynchronized_or_invalid_samples():
    trajectory = _two_sample_trajectory()
    values = dict(trajectory.__dict__)
    values["time_s"] = [0.0, 0.0]
    with pytest.raises(ValueError, match="strictly increasing"):
        HybridTrajectory(**values)

    values = dict(trajectory.__dict__)
    values["swing_phase"] = np.full((2, 4), 1.1)
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        HybridTrajectory(**values)
