"""Day 12 Step 2: four-leg world-frame initialisation and terrain registration.

Plan §9's nine requirements, one test each where they are testable, plus the
two things this step exists to catch: a left/right or front/hind mapping error,
and an experimental terrain size leaking into planner logic.
"""

import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.hybrid import InitialRobotState, RimId

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    FlatRunExtent2D,
    FourLegState2D,
    LegId,
    four_leg_rows,
    initialize_four_leg_state_2d,
    leg_mounts_2d,
    sagittal_reach_agreement_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import NominalPosture2D


def terrain(height_m=0.04, top_length_m=0.40, x_start_m=1.00) -> SharedTerrainSpec2D:
    """A platform built from parameters.

    The values here are a **test fixture**, not planner constants; every test
    that depends on a size passes its own.
    """

    return SharedTerrainSpec2D(
        height_m=height_m, top_length_m=top_length_m, x_start_m=x_start_m,
        obstacle_id="day12_platform",
    )


@pytest.fixture(scope="module")
def state() -> FourLegState2D:
    return initialize_four_leg_state_2d(terrain())


# --------------------------------------------------------------------------
# §9(1): the mounting comes from existing geometry
# --------------------------------------------------------------------------


def test_there_are_four_mounts_one_per_leg():
    mounts = leg_mounts_2d()
    assert len(mounts) == 4
    assert {m.leg for m in mounts} == set(LEG_ORDER)
    assert {m.leg.index for m in mounts} == {0, 1, 2, 3}


def test_the_leg_names_keep_the_projects_own_indices():
    """``CorgiLegKinematics`` documents 0:FL 1:FR 2:RR 3:RL.  Renaming to
    LF/RF/LH/RH must not renumber, or every ``joint_position_rad`` row moves."""

    assert (LegId.LF.index, LegId.RF.index, LegId.RH.index, LegId.LH.index) == (
        0, 1, 2, 3
    )
    for leg in LEG_ORDER:
        kinematics = CorgiLegKinematics(leg.index)
        assert kinematics.is_left == leg.is_left
        assert kinematics.is_front == leg.is_front


def test_the_mount_offsets_match_the_existing_robot_dimensions():
    """Not invented: front/hind from ``WHEEL_BASE``, lateral from
    ``BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET``, height from ``ABAD_AXIS_OFFSET``."""

    mounts = {m.leg: m for m in leg_mounts_2d()}
    half_length = RobotParams.WHEEL_BASE / 2.0
    lateral = RobotParams.BODY_WIDTH / 2.0 + RobotParams.WHEEL_AXIAL_OFFSET
    for leg in LEG_ORDER:
        x, y, z = (float(v) for v in mounts[leg].offset_body_xyz_m)
        assert x == pytest.approx(half_length if leg.is_front else -half_length)
        assert y == pytest.approx(lateral if leg.is_left else -lateral)
        assert z == pytest.approx(RobotParams.ABAD_AXIS_OFFSET)


def test_the_lateral_offset_is_not_merely_half_the_body_width():
    """The easy mistake this step must not make: the sagittal plane sits at the
    wheel mid-plane, 91.675 mm outboard of the ABAD axis."""

    y = float(leg_mounts_2d()[0].offset_body_xyz_m[1])
    assert y != pytest.approx(RobotParams.BODY_WIDTH / 2.0)
    assert y - RobotParams.BODY_WIDTH / 2.0 == pytest.approx(
        RobotParams.WHEEL_AXIAL_OFFSET
    )


def test_gamma_may_not_be_freed_here():
    with pytest.raises(ValueError, match="gamma = 0"):
        leg_mounts_2d(np.deg2rad(10.0))


# --------------------------------------------------------------------------
# The 2D/3D agreement that licenses mounting Step 1 on this frame
# --------------------------------------------------------------------------


def test_the_two_models_agree_on_how_far_the_leg_reaches():
    """The 3D hip-to-foot drop and the 2D flat stance height are the same
    distance computed by different code.  If they diverge, mounting the Step 1
    trajectories on the four-leg frame is invalid."""

    report = sagittal_reach_agreement_2d()
    assert report["hip_to_foot_drop_3d_mm"] == pytest.approx(219.4486, abs=1e-3)
    # The residual is the 2D scene's deliberate surface offset (1e-9 m), not a
    # modelling disagreement.
    assert abs(report["difference_mm"]) == pytest.approx(1e-6, abs=1e-9)


def test_the_agreement_holds_at_another_posture():
    report = sagittal_reach_agreement_2d(
        NominalPosture2D(theta_rad=np.deg2rad(90.0))
    )
    assert abs(report["difference_mm"]) < 1e-5


# --------------------------------------------------------------------------
# §9(2)(3)(4)(5): the state
# --------------------------------------------------------------------------


def test_the_body_is_level_and_gamma_is_zero(state):
    assert np.allclose(state.body_rpy_rad, 0.0)
    for leg in state.legs:
        assert leg.gamma_rad == 0.0
    assert np.allclose(state.joint_position_rad[:, 2], 0.0)


def test_a_non_level_body_is_refused(state):
    from dataclasses import replace

    with pytest.raises(ValueError, match="roll/pitch/yaw"):
        replace(state, body_rpy_rad=np.array([0.0, np.deg2rad(2.0), 0.0]))


def test_every_leg_carries_the_four_requested_quantities(state):
    """§9(6): hip pose in world, contact point in world, surface id, joints."""

    for leg in state.legs:
        assert leg.hip_pose_world_xyz_m.shape == (3,)
        assert leg.contact_point_world_xyz_m.shape == (3,)
        assert leg.surface_id
        assert leg.rim == RimId.FOOT.value
        assert np.isfinite([leg.theta_rad, leg.beta_rad, leg.gamma_rad]).all()


def test_all_four_legs_stand_on_the_ground_before_the_platform(state):
    assert state.all_in_contact
    for leg in state.legs:
        assert leg.surface_id == "ground"
        assert not leg.collision
        assert leg.surface_gap_m == pytest.approx(0.0, abs=1e-6)


def test_the_hips_are_the_body_plus_the_mount_offsets(state):
    mounts = {m.leg: m for m in leg_mounts_2d()}
    for leg in state.legs:
        expected = state.body_position_world_m + mounts[leg.leg].offset_body_xyz_m
        assert leg.hip_pose_world_xyz_m == pytest.approx(expected)


def test_the_front_hips_lead_the_hind_hips(state):
    """A front/hind sign error would put the hind legs ahead."""

    front = state.leg(LegId.LF).hip_pose_world_xyz_m[0]
    hind = state.leg(LegId.LH).hip_pose_world_xyz_m[0]
    assert front - hind == pytest.approx(RobotParams.WHEEL_BASE)
    assert front > hind


def test_the_left_hips_are_on_positive_y(state):
    """A left/right sign error would mirror the robot."""

    for leg in state.legs:
        y = float(leg.hip_pose_world_xyz_m[1])
        assert (y > 0.0) == leg.leg.is_left


def test_the_state_converts_to_the_day1_contract(state):
    contract = state.as_initial_robot_state()
    assert isinstance(contract, InitialRobotState)
    assert contract.joint_position_rad.shape == (4, 3)
    assert np.allclose(contract.body_rpy_rad, 0.0)


def test_joint_rows_are_indexed_by_the_project_order_not_the_reading_order():
    """``LEG_ORDER`` is LF, RF, LH, RH for reading; the contract's rows are
    0..3 as the project numbers them.  Emitting the reading order would swap
    the two hind legs."""

    posture = NominalPosture2D()
    state = initialize_four_leg_state_2d(terrain(), posture=posture)
    rows = state.joint_position_rad
    for leg in LEG_ORDER:
        assert rows[leg.index][0] == pytest.approx(state.leg(leg).theta_rad)
    assert LEG_ORDER[2] is LegId.LH and LegId.LH.index == 3


# --------------------------------------------------------------------------
# §9(9): symmetry
# --------------------------------------------------------------------------


def test_the_symmetric_state_passes_every_symmetry_check(state):
    checks = state.symmetry_checks()
    assert checks
    failed = [c.as_dict() for c in checks if not c.symmetric]
    assert not failed, failed
    assert state.is_symmetric


def test_the_symmetry_check_compares_y_as_mirrored_not_as_equal(state):
    """Comparing ``y`` for equality would pass a robot with both legs on one
    side, which is exactly the mapping error this check is for."""

    mirrored = [c for c in state.symmetry_checks() if "mirrored" in c.quantity]
    assert mirrored
    for check in mirrored:
        assert check.left_value == pytest.approx(-check.right_value)
        assert check.difference == pytest.approx(0.0, abs=1e-9)


def test_a_mirrored_leg_would_fail_the_symmetry_check(state):
    """The checks must be able to fail, or passing them says nothing."""

    from dataclasses import replace

    broken_leg = replace(
        state.leg(LegId.RF),
        hip_pose_world_xyz_m=state.leg(LegId.LF).hip_pose_world_xyz_m,
    )
    broken = replace(
        state,
        legs=tuple(broken_leg if l.leg is LegId.RF else l for l in state.legs),
    )
    assert not broken.is_symmetric
    failed = {c.quantity for c in broken.symmetry_checks() if not c.symmetric}
    assert "hip_y_m (mirrored)" in failed


# --------------------------------------------------------------------------
# §0.1: the terrain is a parameter, not a branch
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "height_m,top_length_m",
    [(0.001, 0.40), (0.04, 0.40), (0.10, 0.40), (0.19, 0.40), (0.07, 0.22)],
)
def test_the_same_code_registers_any_platform(height_m, top_length_m):
    """Plan §0.1: only the terrain parameters change.  The initial stance on
    the lower ground is independent of the platform, so the body height must
    come out identical for every one of these."""

    state = initialize_four_leg_state_2d(terrain(height_m, top_length_m))
    assert state.all_in_contact
    assert state.is_symmetric
    assert state.terrain.height_m == height_m
    assert state.terrain.top_length_m == top_length_m
    assert float(state.body_position_world_m[2]) == pytest.approx(0.162283, abs=1e-5)


def test_the_platform_is_registered_where_the_parameters_put_it():
    spec = terrain(height_m=0.11, top_length_m=0.33, x_start_m=2.5)
    state = initialize_four_leg_state_2d(spec)
    assert state.terrain.x_start_m == 2.5
    assert state.terrain.x_max_m == pytest.approx(2.83)
    assert state.terrain.top_z_m == pytest.approx(0.11)


def test_the_flat_run_extent_is_an_input_not_a_terrain_edge():
    """``TerrainProfile2D``'s ground is unbounded, so these are the plan's
    statement of where the nominal cycles happen."""

    extent = FlatRunExtent2D(flat_before_m=0.8, flat_after_m=0.3)
    spec = terrain(x_start_m=2.0)
    state = initialize_four_leg_state_2d(spec, extent=extent)
    lo, hi = extent.x_limits_m(spec)
    assert lo == pytest.approx(1.2)
    assert hi == pytest.approx(2.7)
    # The default body placement puts the front hip one flat_before back.
    assert float(state.leg(LegId.LF).hip_pose_world_xyz_m[0]) == pytest.approx(1.2)


def test_the_body_can_be_placed_explicitly():
    state = initialize_four_leg_state_2d(terrain(), body_x_m=0.5)
    assert float(state.body_position_world_m[0]) == pytest.approx(0.5)


# --------------------------------------------------------------------------
# The state that is not a valid stance, reported rather than repaired
# --------------------------------------------------------------------------


def test_a_leg_over_the_platform_is_reported_not_silently_lowered():
    """A level body on the lower-ground stance height puts a leg standing over
    the platform *inside* it.  Step 2 must say so; moving the body to fix it is
    Step 5's job."""

    spec = terrain(height_m=0.04, x_start_m=1.00)
    state = initialize_four_leg_state_2d(spec, body_x_m=0.90)
    assert not state.all_in_contact
    for leg in (LegId.LF, LegId.RF):
        front = state.leg(leg)
        assert not front.in_contact
        assert front.collision
        # Penetrating by exactly the platform height, not floating above it.
        assert front.surface_gap_m == pytest.approx(-spec.height_m, abs=1e-6)
    for leg in (LegId.LH, LegId.RH):
        assert state.leg(leg).in_contact
    # ...and it is still left/right symmetric, which is the point of the checks
    # being separate from the contact validity.
    assert state.is_symmetric


def test_a_posture_and_terrain_that_disagree_about_the_ground_are_refused():
    with pytest.raises(ValueError, match="where the ground is"):
        initialize_four_leg_state_2d(
            terrain(), posture=NominalPosture2D(ground_height_m=0.05)
        )


# --------------------------------------------------------------------------
# §9(8): the debug output
# --------------------------------------------------------------------------


def test_the_debug_table_carries_body_terrain_legs_and_symmetry(state):
    rows = four_leg_rows(state)
    kinds = [row["row_kind"] for row in rows]
    assert kinds.count("body") == 1
    assert kinds.count("terrain") == 1
    assert kinds.count("leg") == 4
    assert kinds.count("symmetry") == len(state.symmetry_checks())


def test_the_visualization_draws_without_a_display(state):
    import matplotlib
    matplotlib.use("Agg")
    from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
        plot_four_leg_state_2d,
    )

    figure = plot_four_leg_state_2d(state)
    assert len(figure.axes) >= 2
    import matplotlib.pyplot as plt
    plt.close(figure)
