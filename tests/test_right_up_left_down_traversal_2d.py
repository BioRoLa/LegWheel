"""Day 6--7 revised traversal tests: Step 7R retract-to-wheel on the obstacle top."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    build_single_leg_rolling_scene_2d,
    run_forward_right_rim_roll_up_2d,
    run_retract_and_reset_branch_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    roll_down_start_state_from_rolling_result,
)
from hybrid_note.scripts.experiments.right_up_left_down_traversal_2d import (
    RetractToWheelOnTopResult2D,
    corner_states_along_transition,
    roll_up_result_from_top_contact_state,
    run_left_rim_roll_down_2d,
    run_wheel_mode_transition_to_corner_2d,
    wheel_mode_transition_summary_row,
    plot_left_rim_roll_down_key_frames_2d,
    plot_retract_to_wheel_key_frames_2d,
    plot_retract_to_wheel_on_top_2d,
    retract_to_wheel_on_top_frame_rows,
    retract_to_wheel_on_top_summary_row,
    run_retract_to_wheel_on_top_2d,
    write_retract_to_wheel_on_top_csv,
)

THETA_TARGET_RAD = np.deg2rad(17.0)
STEP_RAD = np.deg2rad(1.0)
TOP_LENGTH_M = 0.60
ARC_SAMPLES = 61
# The alpha = 180 deg seam is a *sampling* gap, not a physical notch: at
# theta = 17 deg both endpoint samples sit at the same radius as the rim.  Its
# measured width therefore scales as 1 / arc_samples (11.7 mm at 61 samples,
# 2.9 mm at 241), so a coarse test fixture needs a matching bridge allowance.
SEAM_BRIDGE_M = 0.013


@pytest.fixture(scope="module")
def roll_up():
    """A cheap but genuinely successful Step 4.5 roll-up onto the obstacle top."""

    result = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.002,
        max_forward_steps=10,
        beta_step_rad=np.deg2rad(-5.0),
        top_roll_distance_m=0.001,
        arc_samples=ARC_SAMPLES,
    )
    assert result.success
    return result


@pytest.fixture(scope="module")
def step7r(roll_up):
    return run_retract_to_wheel_on_top_2d(
        roll_up,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        obstacle_top_length_m=TOP_LENGTH_M,
    )


def test_stop_at_rejects_unknown_values(roll_up):
    with pytest.raises(ValueError, match="stop_at must be one of"):
        run_retract_and_reset_branch_2d(
            roll_up, branch="forward_continuation", stop_at="left_rim"
        )


def test_retract_reaches_the_wheel_state_while_still_on_the_obstacle_top(step7r):
    assert isinstance(step7r, RetractToWheelOnTopResult2D)
    assert step7r.success
    assert step7r.failure_reason is None

    final = step7r.final_frame
    assert final.theta_rad == pytest.approx(THETA_TARGET_RAD, abs=1e-12)
    assert final.terrain_surface_id.endswith("_top")
    assert final.valid_contact
    assert not any(frame.collision for frame in step7r.frames)
    assert all(frame.accepted for frame in step7r.frames)
    assert all(frame.joint_limits_ok for frame in step7r.frames)

    # theta decreases monotonically and never overshoots the target.
    thetas = np.asarray([frame.theta_rad for frame in step7r.frames])
    assert np.all(np.diff(thetas) <= 1e-12)
    assert np.all(thetas >= THETA_TARGET_RAD - 1e-12)


def test_retract_stops_at_the_first_frame_that_reaches_the_target(step7r):
    at_target = [
        frame for frame in step7r.frames
        if np.isclose(frame.theta_rad, THETA_TARGET_RAD, atol=1e-12)
    ]
    assert len(at_target) == 1
    assert at_target[0] is step7r.final_frame
    assert step7r.theta_reached_at_step == step7r.final_frame.step


def test_retract_holds_signed_no_slip_and_moves_forward(step7r):
    residuals = [abs(frame.no_slip_tangent_residual_m) for frame in step7r.frames]
    assert max(residuals) < 1e-9

    contact = np.asarray(
        [frame.contact_forward_displacement_m for frame in step7r.frames]
    )
    hip = np.asarray([frame.hip_forward_displacement_m for frame in step7r.frames])
    assert np.all(np.diff(contact) >= -1e-12)
    assert np.all(np.diff(hip) >= -1e-12)
    assert contact[-1] > 0.0
    # Contact travel and hip travel are different quantities, not interchangeable.
    assert contact[-1] != pytest.approx(hip[-1], abs=1e-4)


def test_retract_does_not_require_or_reach_the_foot_rim(step7r):
    """Step 7R targets the wheel-like configuration, not a foot-rim reset."""

    assert step7r.final_frame.active_rim == "right_rim"
    assert not any(frame.foot_rim_ready for frame in step7r.frames)


def test_step7r_is_the_left_rim_trajectory_cut_earlier(roll_up, step7r):
    """The milestones are one motion, not two independent simulations."""

    left_ready = run_retract_and_reset_branch_2d(
        roll_up,
        branch="forward_continuation",
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        stop_at="left_rim_ready",
        max_seam_bridge_m=SEAM_BRIDGE_M,
        obstacle_top_length_m=TOP_LENGTH_M,
    )
    assert left_ready.success
    assert left_ready.final_frame.active_rim == "left_rim"
    assert len(left_ready.frames) > len(step7r.frames)

    for early, later in zip(step7r.frames, left_ready.frames):
        assert early.step == later.step
        assert early.theta_rad == pytest.approx(later.theta_rad, abs=1e-12)
        assert early.beta_rad == pytest.approx(later.beta_rad, abs=1e-12)
        assert early.active_sample_index == later.active_sample_index

    # The contact crosses the alpha = 180 deg seam only after theta is retracted.
    seam_crossings = [
        (before, after)
        for before, after in zip(left_ready.frames, left_ready.frames[1:])
        if before.active_rim == "right_rim" and after.active_rim == "left_rim"
    ]
    assert len(seam_crossings) == 1
    before, after = seam_crossings[0]
    assert before.theta_rad == pytest.approx(THETA_TARGET_RAD, abs=1e-12)
    assert np.rad2deg(before.alpha_rad) > 170.0
    assert np.rad2deg(after.alpha_rad) < -170.0


def test_default_stop_at_still_runs_past_the_wheel_state(roll_up):
    """The Step 6.5 default must not silently inherit the new early stop."""

    result = run_retract_and_reset_branch_2d(
        roll_up,
        branch="forward_continuation",
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        obstacle_top_length_m=TOP_LENGTH_M,
        max_steps=60,
    )
    assert not result.success
    assert result.failure_reason == "MAX_COUPLED_RESET_STEPS_REACHED"
    at_target = [
        frame for frame in result.frames
        if np.isclose(frame.theta_rad, THETA_TARGET_RAD, atol=1e-12)
    ]
    assert len(at_target) > 1, "the default branch stopped at the wheel state"


def test_frame_rows_summary_and_csv(step7r, tmp_path):
    rows = retract_to_wheel_on_top_frame_rows(step7r)
    assert len(rows) == len(step7r.frames)
    assert {"theta_deg", "alpha_deg", "no_slip_tangent_residual_m"} <= set(rows[0])

    summary = retract_to_wheel_on_top_summary_row(step7r)
    assert summary["success"] is True
    assert summary["theta_final_deg"] == pytest.approx(17.0)
    assert summary["any_collision"] is False
    assert summary["frame_count"] == len(rows)

    summary_path, frames_path = write_retract_to_wheel_on_top_csv(
        step7r, tmp_path / "summary.csv", tmp_path / "frames.csv"
    )
    assert summary_path.exists()
    assert len(frames_path.read_text(encoding="utf-8").splitlines()) == len(rows) + 1


def test_plot_helper_builds_axes(step7r):
    axes = plot_retract_to_wheel_on_top_2d(step7r)
    assert len(axes) == 2


def test_step7r_key_frames_span_the_whole_retract(step7r):
    """The three poses must be start, middle and the wheel state -- not three
    frames that happen to look alike."""

    figure, axes = plot_retract_to_wheel_key_frames_2d(step7r)
    assert axes.shape == (1, 3)
    titles = [ax.get_title() for ax in axes[0]]
    assert "60.0 deg" in titles[0]
    assert "17.0 deg" in titles[-1]
    plt.close(figure)


def test_seam_bridge_limit_is_coupled_to_rim_sampling(roll_up):
    """The right-to-left handover is gated by sampling density, not geometry.

    At theta = 17 deg the leg is effectively a closed circle, so the seam at
    alpha = 180 deg carries no real notch.  What the guard measures is the
    chord between the last sampled right-rim point and the first sampled
    left-rim point, which shrinks as arc_samples grows.  A sweep that lowers
    arc_samples to run faster would therefore reject a handover that is
    physically fine, so this coupling is pinned down here.
    """

    common = dict(
        branch="forward_continuation",
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        stop_at="left_rim_ready",
        obstacle_top_length_m=TOP_LENGTH_M,
    )
    too_tight = run_retract_and_reset_branch_2d(
        roll_up, max_seam_bridge_m=5e-3, **common
    )
    assert not too_tight.success
    assert too_tight.failure_reason == "RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET"

    matched = run_retract_and_reset_branch_2d(
        roll_up, max_seam_bridge_m=SEAM_BRIDGE_M, **common
    )
    assert matched.success
    assert matched.final_frame.active_rim == "left_rim"


def test_rebuilt_end_state_reproduces_the_retract_exactly(roll_up, step7r):
    """The notebook cache must be a faithful stand-in for the real roll-up.

    Downstream stages only read the roll-up final frame, so caching that state
    and rebuilding the pose must give a bit-for-bit identical retract; a cache
    that quietly perturbed the entry pose would be worse than no cache.
    """

    state = roll_down_start_state_from_rolling_result(roll_up)
    rebuilt = roll_up_result_from_top_contact_state(state)

    original = roll_up.final_frame
    copy = rebuilt.final_frame
    assert rebuilt.success
    assert copy.top_roll_complete
    assert copy.theta_rad == pytest.approx(original.theta_rad, abs=1e-15)
    assert copy.beta_rad == pytest.approx(original.beta_rad, abs=1e-15)
    assert copy.alpha_rad == pytest.approx(original.alpha_rad, abs=1e-15)
    assert copy.contact_point_world_xz_m == pytest.approx(
        original.contact_point_world_xz_m, abs=1e-15
    )

    replayed = run_retract_to_wheel_on_top_2d(
        rebuilt,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        obstacle_top_length_m=TOP_LENGTH_M,
    )
    assert replayed.success == step7r.success
    assert len(replayed.frames) == len(step7r.frames)
    for cached, direct in zip(replayed.frames, step7r.frames):
        assert cached.theta_rad == pytest.approx(direct.theta_rad, abs=1e-15)
        assert cached.beta_rad == pytest.approx(direct.beta_rad, abs=1e-15)
        assert cached.active_sample_index == direct.active_sample_index
        assert cached.contact_forward_displacement_m == pytest.approx(
            direct.contact_forward_displacement_m, abs=1e-15
        )


# --------------------------------------------------------------------------
# Step 8R: wheel-mode forward roll to the trailing corner
# --------------------------------------------------------------------------

LONG_TOP_M = 0.50


@pytest.fixture(scope="module")
def wheel_mode(roll_up):
    """One long wheel-mode run; the corner sweep is derived from it."""

    return run_wheel_mode_transition_to_corner_2d(
        roll_up,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        max_seam_bridge_m=SEAM_BRIDGE_M,
        obstacle_top_length_m=LONG_TOP_M,
    )


def test_trailing_corner_run_does_not_stop_at_the_rim_handover(wheel_mode):
    """Regression: ``trailing_corner`` must not inherit the left-rim stop rule.

    The handover and the corner are different events; stopping at the first
    one would silently report a corner-arrival state that is nowhere near the
    corner.
    """

    assert wheel_mode.reached_trailing_corner
    assert wheel_mode.left_rim_handover_step is not None
    assert wheel_mode.final_frame.step > wheel_mode.left_rim_handover_step

    corner_x = wheel_mode.trailing_corner_world_xz_m[0]
    contact_x = wheel_mode.final_frame.contact_point_world_xz_m[0]
    assert contact_x < corner_x
    # The stop really is at the corner, within one no-slip step of it.
    assert wheel_mode.corner_gap_m == pytest.approx(corner_x - contact_x, abs=1e-12)
    assert 0.0 < wheel_mode.corner_gap_m < 0.02


def test_wheel_mode_records_both_milestones_of_one_motion(wheel_mode):
    assert wheel_mode.theta_reached_step is not None
    assert wheel_mode.left_rim_handover_step is not None
    assert wheel_mode.theta_reached_step < wheel_mode.left_rim_handover_step
    assert wheel_mode.l_transition_m is not None
    assert wheel_mode.l_transition_m > 0.0

    # The handover happens only after the leg is folded into the wheel state.
    handover = wheel_mode.frames[wheel_mode.left_rim_handover_step]
    before = wheel_mode.frames[wheel_mode.left_rim_handover_step - 1]
    assert handover.theta_rad == pytest.approx(THETA_TARGET_RAD, abs=1e-12)
    assert before.active_rim == "right_rim"
    assert handover.active_rim == "left_rim"

    assert not any(frame.collision for frame in wheel_mode.frames)
    assert all(frame.accepted for frame in wheel_mode.frames)


def test_left_rim_ready_is_a_precondition_evaluated_at_the_corner(roll_up, wheel_mode):
    """A top that is too short reaches the corner but is not ready to descend."""

    short = run_wheel_mode_transition_to_corner_2d(
        roll_up,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        max_seam_bridge_m=SEAM_BRIDGE_M,
        obstacle_top_length_m=0.18,
    )
    assert short.reached_trailing_corner, "reaching the corner is a terrain property"
    assert not short.left_rim_ready, "readiness is a leg property and must differ"
    assert short.readiness_failure == "LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER"
    assert short.final_frame.active_rim == "right_rim"
    # Sufficiency must not be claimed from a non-left rim's leftover arc.
    assert short.descent_budget_sufficient is None


def test_descent_budget_is_only_reported_for_the_left_rim(wheel_mode):
    assert wheel_mode.left_rim_ready
    assert wheel_mode.rim_budget_at_corner_rad is not None
    assert wheel_mode.predicted_pivot_rotation_rad is not None
    assert wheel_mode.descent_budget_sufficient is not None


def test_corner_sweep_matches_direct_runs(roll_up, wheel_mode):
    """One long run must stand in exactly for a per-length sweep."""

    lengths = (0.20, 0.30, 0.40)
    derived = {
        row["obstacle_top_length_m"]: row
        for row in corner_states_along_transition(wheel_mode, lengths)
    }
    for length in lengths:
        direct = run_wheel_mode_transition_to_corner_2d(
            roll_up,
            theta_target_rad=THETA_TARGET_RAD,
            theta_step_rad=STEP_RAD,
            beta_step_rad=STEP_RAD,
            max_seam_bridge_m=SEAM_BRIDGE_M,
            obstacle_top_length_m=length,
        )
        row = derived[length]
        assert row["reached_trailing_corner"] == direct.reached_trailing_corner
        assert row["left_rim_ready"] == direct.left_rim_ready
        assert row["readiness_failure"] == direct.readiness_failure
        assert row["step"] == direct.final_frame.step
        assert row["active_rim"] == direct.final_frame.active_rim
        assert row["contact_x_m"] == pytest.approx(
            direct.final_frame.contact_point_world_xz_m[0], abs=1e-12
        )


def test_corner_sweep_refuses_to_extrapolate(wheel_mode):
    rows = corner_states_along_transition(wheel_mode, (LONG_TOP_M + 0.20,))
    assert rows[0]["reached_trailing_corner"] is False
    assert rows[0]["readiness_failure"] == "TRAILING_CORNER_BEYOND_SIMULATED_RUN"
    assert rows[0]["contact_x_m"] is None


def test_top_length_feasibility_is_a_window_not_a_lower_bound(wheel_mode):
    """Both a minimum and a maximum usable top length exist.

    Too short and the left rim has not taken over; too long and the left rim
    is already spent before the descent can start.
    """

    rows = corner_states_along_transition(
        wheel_mode, np.arange(0.16, LONG_TOP_M, 0.02)
    )
    feasible = [
        row["obstacle_top_length_m"]
        for row in rows
        if row["left_rim_ready"] and row["descent_budget_sufficient"]
    ]
    assert feasible, "no usable top length found"
    assert min(feasible) > rows[0]["obstacle_top_length_m"], "no lower bound"
    assert max(feasible) < rows[-1]["obstacle_top_length_m"], "no upper bound"

    too_short = [r for r in rows if r["obstacle_top_length_m"] < min(feasible)]
    too_long = [r for r in rows if r["obstacle_top_length_m"] > max(feasible)]
    assert all(
        r["readiness_failure"] == "LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER"
        for r in too_short
    )
    assert all(r["descent_budget_sufficient"] is not True for r in too_long)


def test_wheel_mode_summary_row(wheel_mode):
    summary = wheel_mode_transition_summary_row(wheel_mode)
    assert summary["reached_trailing_corner"] is True
    assert summary["left_rim_ready"] is True
    assert summary["final_active_rim"] == "left_rim"
    assert summary["L_transition_m"] == pytest.approx(wheel_mode.l_transition_m)
    assert summary["final_theta_deg"] == pytest.approx(17.0)
    assert summary["wheel_radius_at_target_m"] > 0.0


# --------------------------------------------------------------------------
# Step 9R: left-rim trailing-edge roll-down
# --------------------------------------------------------------------------

STEP9R_TOP_LENGTH_M = 0.35


@pytest.fixture(scope="module")
def corner_state(roll_up):
    """A LEFT_RIM_READY corner-arrival state for the descent tests."""

    result = run_wheel_mode_transition_to_corner_2d(
        roll_up,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        max_seam_bridge_m=SEAM_BRIDGE_M,
        obstacle_top_length_m=STEP9R_TOP_LENGTH_M,
    )
    assert result.left_rim_ready, result.readiness_failure
    return result


@pytest.fixture(scope="module")
def descent_held(corner_state):
    return run_left_rim_roll_down_2d(
        corner_state, release_theta=False, ground_roll_distance_m=0.005
    )


@pytest.fixture(scope="module")
def descent_released(corner_state):
    return run_left_rim_roll_down_2d(
        corner_state, release_theta=True, theta_release_clearance_m=0.02,
        ground_roll_distance_m=0.005,
    )


def test_descent_requires_a_left_rim_ready_corner_state(roll_up):
    """A top too short for the handover must not be silently descended from."""

    too_short = run_wheel_mode_transition_to_corner_2d(
        roll_up,
        theta_target_rad=THETA_TARGET_RAD,
        theta_step_rad=STEP_RAD,
        beta_step_rad=STEP_RAD,
        max_seam_bridge_m=SEAM_BRIDGE_M,
        obstacle_top_length_m=0.18,
    )
    assert not too_short.left_rim_ready
    with pytest.raises(ValueError, match="LEFT_RIM_READY"):
        run_left_rim_roll_down_2d(too_short)


def test_descent_reaches_lower_ground_on_the_left_rim(descent_held):
    assert descent_held.success
    assert descent_held.corner_pivot_success
    assert descent_held.ground_contact_success
    assert descent_held.ground_contact_rim == "left_rim"
    assert descent_held.ground_contact_point_world_xz_m[1] == pytest.approx(0.0, abs=1e-6)
    assert descent_held.ground_contact_point_world_xz_m[0] > (
        descent_held.trailing_corner_world_xz_m[0]
    )
    assert not any(frame.collision for frame in descent_held.frames)
    assert all(frame.accepted for frame in descent_held.frames)
    assert min(frame.ground_clearance_m for frame in descent_held.frames) >= 0.0
    assert descent_held.minimum_back_face_clearance_m > 0.0


def test_descent_phases_appear_in_order(descent_held):
    order = [
        "TRAILING_CORNER_TRANSITION", "LEFT_RIM_ROLL_DOWN",
        "LOWER_GROUND_CONTACT", "GROUND_ROLL",
    ]
    seen = [frame.phase for frame in descent_held.frames]
    assert set(order) <= set(seen)
    ranks = [order.index(phase) for phase in seen if phase in order]
    assert ranks == sorted(ranks)


def test_corner_pivot_pins_the_contact(descent_held):
    """Same physics as the right-rim study: a corner is a point, so this is a pivot."""

    pivot = [
        frame for frame in descent_held.frames
        if frame.phase in ("TRAILING_CORNER_TRANSITION", "LEFT_RIM_ROLL_DOWN")
    ]
    assert len(pivot) > 5
    assert all(frame.corner_pinned for frame in pivot)
    assert len({frame.active_sample_index for frame in pivot}) == 1
    assert len({round(frame.contact_advance_m, 12) for frame in pivot}) == 1

    hips = [frame.hip_x_m for frame in pivot]
    assert hips == sorted(hips)
    drops = [frame.hip_z_m for frame in pivot]
    assert drops == sorted(drops, reverse=True)


def test_theta_is_held_until_the_late_descent(descent_released):
    release_step = descent_released.theta_release_step
    assert descent_released.theta_released
    assert release_step is not None
    early = [f for f in descent_released.frames if f.step <= release_step]
    assert len({round(f.theta_deg, 9) for f in early}) == 1, (
        "theta moved before the release point"
    )
    late = [f for f in descent_released.frames
            if release_step < f.step and f.phase == "LEFT_RIM_ROLL_DOWN"]
    thetas = [f.theta_rad for f in late]
    assert thetas == sorted(thetas)
    assert descent_released.theta_final_rad > THETA_TARGET_RAD


def test_theta_release_trades_rotation_for_a_gentler_hip_drop(
    descent_held, descent_released
):
    assert descent_held.success and descent_released.success
    # Cheaper on the body...
    assert descent_released.hip_drop_at_touchdown_m > descent_held.hip_drop_at_touchdown_m
    # ...and paid for in extra rotation.
    assert descent_released.pivot_rotation_rad > descent_held.pivot_rotation_rad


def test_theta_release_is_paid_for_in_post_touchdown_rolling(
    corner_state, descent_held, descent_released
):
    """The real cost of releasing theta is not rim budget, it is rolling away.

    A more extended leg carries its foot rim closer to the ground, so although
    the left rim still lands, the leg cannot keep rolling on the lower ground
    afterwards.  The saving and the cost are both monotone in theta, so the
    release is a knob rather than a free improvement, and the baseline is what
    the traversal primitive should use.
    """

    # Ask for more ground roll than either branch can deliver, so what is
    # compared is capacity rather than whichever target happened to be set.
    beyond_reach = 0.06
    held = run_left_rim_roll_down_2d(
        corner_state, release_theta=False, ground_roll_distance_m=beyond_reach
    )
    released = run_left_rim_roll_down_2d(
        corner_state, release_theta=True, ground_roll_distance_m=beyond_reach
    )
    assert not held.ground_roll_success and not released.ground_roll_success
    assert (
        released.ground_roll_distance_achieved_m
        < held.ground_roll_distance_achieved_m
    )
    assert released.hip_drop_at_touchdown_m > held.hip_drop_at_touchdown_m
    assert released.theta_final_rad > held.theta_final_rad

    # Both still land on the descent rim; that part the guard does deliver.
    assert descent_held.descent_rim_preserved
    assert descent_released.descent_rim_preserved

    # Dropping the margin buys more hip-drop saving and less rolling room.
    generous = run_left_rim_roll_down_2d(
        corner_state, release_theta=True, descent_rim_margin_m=0.0,
        ground_roll_distance_m=beyond_reach,
    )
    assert generous.theta_ceiling_rad > released.theta_ceiling_rad
    assert generous.hip_drop_at_touchdown_m > released.hip_drop_at_touchdown_m
    assert (
        generous.ground_roll_distance_achieved_m
        <= released.ground_roll_distance_achieved_m
    )


def test_step9r_key_frames_cover_every_phase_that_ran(descent_held):
    """One panel per phase the descent actually entered, in order."""

    figure, axes = plot_left_rim_roll_down_key_frames_2d(descent_held)
    phases = [frame.phase for frame in descent_held.frames]
    expected = [
        phase for phase in dict.fromkeys(phases) if phase != "FAILED"
    ]
    titles = [ax.get_title() for ax in axes[0]]
    assert len(titles) >= len(expected)
    for phase, title in zip(expected, titles):
        assert title.startswith(phase)
    plt.close(figure)


def test_same_rim_chord_is_theta_invariant():
    """The geometric fact the previous test relies on, checked directly."""

    scene_kwargs = dict(
        gamma_rad=0.0, ground_height_m=0.0, obstacle_x_start_m=0.10,
        obstacle_width_m=0.30, obstacle_height_m=0.10,
        obstacle_id="chord_probe", arc_samples=ARC_SAMPLES,
    )
    chords = []
    for theta_deg in (17.0, 25.0, 41.0, 60.0):
        geometry = build_single_leg_rolling_scene_2d(
            np.deg2rad(theta_deg), 0.0, 0.0, 0.0, **scene_kwargs
        ).geometry
        left = np.flatnonzero(
            np.asarray(geometry.contact_regions) == "left_rim"
        )
        first, second = int(left[10]), int(left[-10])
        points = geometry.points_hip_xz_m
        chords.append(float(np.linalg.norm(points[second] - points[first])))
        # The radii themselves are *not* invariant, which is why this matters.
    assert np.ptp(chords) < 1e-9
