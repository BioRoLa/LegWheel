"""Day 6--7 Step 10R: the full right-up / left-down traversal, end to end."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    PHASE_FRONT_CONTACT,
    STAGE_APPROACH,
    STAGE_ROLL_DOWN,
    STAGE_WHEEL_TRANSITION,
    TRAVERSAL_PHASE_ORDER,
    ObstacleSpec2D,
    RollingTraversalResult2D,
    TraversalConstraints2D,
    TraversalInitialState2D,
    check_right_up_left_down_traversal,
    plot_traversal_key_frames_2d,
    run_approach_to_front_face_2d,
    traversal_frame_rows,
    traversal_summary_row,
    write_traversal_csv,
)

THETA_CLIMB_RAD = np.deg2rad(60.0)
ARC_SAMPLES = 61
# The alpha = 180 deg seam is a sampling gap, not a notch, so a coarse test
# fixture needs a matching bridge allowance; see the Step 8R seam test.
SEAM_BRIDGE_M = 0.013
FEASIBLE_WIDTH_M = 0.35
# Beyond the verified top-length window the left rim is spent before the
# corner, so this width is expected to fail -- it is the upper-bound probe.
TOO_WIDE_M = 0.60


def _constraints(**overrides) -> TraversalConstraints2D:
    settings = dict(max_seam_bridge_m=SEAM_BRIDGE_M, ground_roll_distance_m=0.005)
    settings.update(overrides)
    return TraversalConstraints2D(**settings)


@pytest.fixture(scope="module")
def obstacle() -> ObstacleSpec2D:
    return ObstacleSpec2D(
        x_start_m=0.10, width_m=FEASIBLE_WIDTH_M, height_m=0.10,
        arc_samples=ARC_SAMPLES,
    )


@pytest.fixture(scope="module")
def initial_state() -> TraversalInitialState2D:
    return TraversalInitialState2D(hip_x_m=-0.10, beta_rad=0.0)


@pytest.fixture(scope="module")
def approach(obstacle, initial_state):
    return run_approach_to_front_face_2d(
        obstacle, initial_state, THETA_CLIMB_RAD, _constraints()
    )


@pytest.fixture(scope="module")
def traversal(obstacle, initial_state) -> RollingTraversalResult2D:
    return check_right_up_left_down_traversal(
        obstacle=obstacle,
        initial_state=initial_state,
        theta_climb=THETA_CLIMB_RAD,
        constraints=_constraints(),
    )


# --------------------------------------------------------------------------
# APPROACH
# --------------------------------------------------------------------------


def test_approach_starts_on_the_ground_and_ends_at_the_front_face(approach, obstacle):
    """The traversal must begin standing on the lower ground, not hovering."""

    assert approach.success, approach.failure_reason
    first, last = approach.frames[0], approach.final_frame

    assert first.terrain_surface_id == first.scene.terrain.ground_surface_id
    assert first.contact_point_world_xz_m[0] < obstacle.x_start_m
    # Standing, not hanging: the support contact is on the ground plane.
    assert first.contact_point_world_xz_m[1] == pytest.approx(
        obstacle.ground_height_m, abs=1e-6
    )
    # And it stopped just short of the face rather than inside it.
    assert 0.0 < approach.front_face_clearance_m <= 1e-3
    assert approach.rolled_distance_m > 0.0
    assert not any(frame.collision for frame in approach.frames)
    assert last.accepted


def test_approach_reaches_a_dual_contact_the_roll_up_accepts(approach, obstacle):
    """The hand-off pose touches the ground *and* the front face at once.

    That dual contact is what lets Step 4.5 start from a ground-supported pose
    instead of the hand-placed hovering one.
    """

    final = approach.final_frame
    surfaces = {
        candidate.terrain_surface_id for candidate in final.query_result.candidates
    }
    assert final.scene.terrain.ground_surface_id in surfaces
    assert any(name.endswith("_front") for name in surfaces)


def test_approach_contact_advances_monotonically_along_the_ground(approach):
    """Rolling, not sliding or hopping: the contact only ever moves forward."""

    contact_x = [
        frame.contact_point_world_xz_m[0]
        for frame in approach.frames
        if frame.contact_point_world_xz_m is not None
    ]
    assert np.all(np.diff(contact_x) > 0.0)


def test_approach_refuses_a_start_that_is_already_at_the_face(obstacle):
    """Starting inside the obstacle is rejected, not quietly rolled out of."""

    result = run_approach_to_front_face_2d(
        obstacle,
        TraversalInitialState2D(hip_x_m=0.20, beta_rad=0.0),
        THETA_CLIMB_RAD,
        _constraints(),
    )
    assert not result.success
    assert result.failure_reason in {
        "APPROACH_START_ALREADY_AT_THE_FRONT_FACE",
        "APPROACH_START_NOT_ON_LOWER_GROUND",
    }


# --------------------------------------------------------------------------
# Full traversal
# --------------------------------------------------------------------------


def test_full_traversal_succeeds_on_a_known_feasible_obstacle(traversal):
    assert traversal.feasible, (traversal.failure_stage, traversal.failure_reason)
    assert traversal.full_success
    assert traversal.approach_success
    assert traversal.roll_up_success
    assert traversal.retract_success
    assert traversal.left_rim_ready_success
    assert traversal.roll_down_success
    assert traversal.failure_stage is None
    assert traversal.failure_reason is None


def test_every_phase_appears_exactly_once_and_in_the_specified_order(traversal):
    """The phase sequence is the spec's, monotone, with no phase revisited."""

    visited = traversal.phases_visited
    assert visited == TRAVERSAL_PHASE_ORDER


def test_stages_hand_over_the_previous_final_state_without_teleporting(traversal):
    """Each stage starts from the pose the previous one ended in.

    Measured rather than assumed: a hand-placed 'ideal' start pose would show
    up here as a jump in hip position or joint angles.  Two of the three
    boundaries are exact.  The third is not, and that is a real property of
    the pipeline rather than slack in the test: Step 9R pins the contact
    sample exactly on the trailing corner, while the corner-arrival frame is
    only within the corner tolerance of it, so entering the descent snaps the
    leg by that much.
    """

    jumps = dict(
        ((source, target), value)
        for source, target, value in traversal.stage_handoff_discontinuities_m
    )
    assert jumps[(STAGE_APPROACH, "ROLL_UP")] == pytest.approx(0.0, abs=1e-12)
    assert jumps[("ROLL_UP", STAGE_WHEEL_TRANSITION)] == pytest.approx(0.0, abs=1e-12)

    corner_snap = jumps[(STAGE_WHEEL_TRANSITION, STAGE_ROLL_DOWN)]
    assert 0.0 <= corner_snap <= 5e-3
    assert traversal.maximum_handoff_discontinuity_m == pytest.approx(corner_snap)


def test_success_requires_landing_on_the_ground_behind_the_obstacle(traversal):
    final = traversal.final_state
    assert final is not None
    assert final.terrain_surface_id == "ground"
    assert final.contact_point_world_xz_m[0] > traversal.obstacle.x_max_m
    assert traversal.reached_lower_ground_behind_obstacle


def test_a_successful_traversal_has_no_rejected_or_colliding_frame(traversal):
    assert all(frame.accepted for frame in traversal.trajectory)
    assert not any(frame.collision for frame in traversal.trajectory)
    assert not any(frame.penetration for frame in traversal.trajectory)
    assert not any(frame.link_collision for frame in traversal.trajectory)
    assert not any(frame.vertical_face_collision for frame in traversal.trajectory)


def test_collision_margin_is_positive_and_reported(traversal):
    """The margin measures distance to surfaces the leg is *not* resting on.

    It must stay positive on a feasible traversal; a zero or negative value
    would mean the leg reached something it was not supposed to touch.
    """

    assert traversal.minimum_collision_margin_m is not None
    assert traversal.minimum_collision_margin_m > 0.0
    assert traversal.minimum_collision_margin == traversal.minimum_collision_margin_m


def test_l_transition_comes_from_the_handover_and_is_aliased(traversal):
    assert traversal.l_transition_m is not None
    assert traversal.l_transition_m > 0.0
    assert traversal.L_transition == traversal.l_transition_m
    assert traversal.l_transition_m == pytest.approx(
        traversal.transition_result.l_transition_m
    )


def test_all_four_stages_contribute_frames(traversal):
    counts = dict(traversal.stage_frame_counts)
    assert set(counts) == {
        STAGE_APPROACH, "ROLL_UP", STAGE_WHEEL_TRANSITION, STAGE_ROLL_DOWN
    }
    assert all(count > 0 for count in counts.values())
    assert sum(counts.values()) == len(traversal.trajectory)


def test_front_contact_phase_is_the_last_approach_frame(traversal):
    front = [
        frame for frame in traversal.trajectory if frame.phase == PHASE_FRONT_CONTACT
    ]
    assert front
    assert all(frame.stage == STAGE_APPROACH for frame in front)
    approach_frames = [
        frame for frame in traversal.trajectory if frame.stage == STAGE_APPROACH
    ]
    assert front[-1] is approach_frames[-1]


# --------------------------------------------------------------------------
# Failure reporting -- no recovery, just an answer
# --------------------------------------------------------------------------


def test_a_top_that_is_too_long_fails_in_the_wheel_transition(initial_state):
    """Past the verified window the left rim is spent before the corner.

    The run must stop there and say so, not fall through to a descent.
    """

    result = check_right_up_left_down_traversal(
        obstacle=ObstacleSpec2D(
            x_start_m=0.10, width_m=TOO_WIDE_M, height_m=0.10,
            arc_samples=ARC_SAMPLES,
        ),
        initial_state=initial_state,
        theta_climb=THETA_CLIMB_RAD,
        constraints=_constraints(),
    )
    assert not result.feasible
    assert not result.full_success
    assert result.failure_stage == STAGE_WHEEL_TRANSITION
    assert result.failure_reason is not None
    assert result.roll_down_success is False
    assert result.descent_result is None
    # Earlier stages still report what they did achieve.
    assert result.approach_success
    assert result.roll_up_success
    assert not any(
        frame.stage == STAGE_ROLL_DOWN for frame in result.trajectory
    )


def test_failure_before_the_transition_leaves_l_transition_unset(obstacle):
    result = check_right_up_left_down_traversal(
        obstacle=obstacle,
        initial_state=TraversalInitialState2D(hip_x_m=0.20, beta_rad=0.0),
        theta_climb=THETA_CLIMB_RAD,
        constraints=_constraints(),
    )
    assert result.failure_stage == STAGE_APPROACH
    assert result.l_transition_m is None
    assert result.final_state is not None      # still reports where it stopped
    assert not result.full_success


# --------------------------------------------------------------------------
# Export and figures
# --------------------------------------------------------------------------


def test_frame_rows_summary_and_csv_round_trip(traversal, tmp_path):
    rows = traversal_frame_rows(traversal)
    assert len(rows) == len(traversal.trajectory)
    assert {"phase", "stage", "active_rim", "contact_surface",
            "collision_margin_m", "theta_deg", "beta_deg"} <= set(rows[0])

    summary = traversal_summary_row(traversal)
    assert summary["full_success"] is True
    assert summary["frame_count"] == len(rows)
    assert " -> ".join(TRAVERSAL_PHASE_ORDER) == summary["phases_visited"]

    summary_path, trajectory_path = write_traversal_csv(
        traversal, tmp_path / "summary.csv", tmp_path / "trajectory.csv"
    )
    assert summary_path.exists()
    assert len(
        trajectory_path.read_text(encoding="utf-8").splitlines()
    ) == len(rows) + 1


def test_key_frame_plot_draws_one_panel_per_phase(traversal):
    figure, axes = plot_traversal_key_frames_2d(traversal)
    visible = [ax for row in axes for ax in row if ax.get_visible()]
    assert len(visible) == len(TRAVERSAL_PHASE_ORDER)
    plt.close(figure)
