"""Day 12 appendix: the drawing of the assembled whole-body trajectory.

The viewer computes exactly one number -- a body height to draw at -- and the
danger with that number is not that it is wrong but that it looks like an
answer.  So the tests pin what it is (the maximum of the instant's hard
demands), what it costs (feet left above the ground, reported), and above all
that it stays out of the data it is drawn from.
"""

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_animation_2d import (
    FROM_HARD_DEMANDS,
    GAP_TOLERANCE_M,
    VIEWING_BASIS,
    demands_at_2d,
    hip_world_xz_m,
    plot_frame_strip_2d,
    sagittal_limits_2d,
    viewing_height_rows,
    viewing_heights_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    build_single_leg_rolling_scene_2d,
)

#: The known rim-model difference (Day 8-11: ``LegModel.rim_point`` uses
#: 0.145 m, the drawn arc 0.1438 m).  A geometric cross-check cannot be
#: tighter than the geometry it compares.
RIM_MODEL_GAP_M = 1.5e-3


@pytest.fixture(scope="module")
def drawn():
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    body = body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]),
        samples=61)
    stability = swing_stability_2d(four, body)
    whole = assemble_whole_body_2d(four, body, stability, samples=61,
                                   reposition_unresolved=2,
                                   use_generator_frames=True)
    heights = viewing_heights_2d(four, whole,
                                 nominal_body_z_m=body.nominal_body_z_m)
    return four, body, whole, heights


# --------------------------------------------------------------------------
# What the height is
# --------------------------------------------------------------------------


def test_the_height_is_the_maximum_hard_demand(drawn):
    four, _, whole, heights = drawn
    for sample, height in zip(whole.samples, heights):
        hard = [d.body_z_m for d in demands_at_2d(four, sample.time_s)
                if d.is_hard]
        assert hard, "the flat run always has a stance leg"
        assert height.source == FROM_HARD_DEMANDS
        assert height.body_z_m == pytest.approx(max(hard))


def test_no_stance_foot_is_pushed_into_the_ground(drawn):
    """The whole reason for taking the maximum rather than the minimum."""

    _, _, _, heights = drawn
    for height in heights:
        assert min(height.float_gap_m.values(), default=0.0) >= 0.0


def test_the_leg_that_sets_the_height_stands_on_the_ground(drawn):
    _, _, _, heights = drawn
    for height in heights:
        assert min(height.float_gap_m.values()) == pytest.approx(0.0)


def test_the_spread_is_step_5s_disagreement_and_it_is_not_smoothed(drawn):
    """A sample with no gap would mean Step 5 had nothing to refuse."""

    _, body, _, heights = drawn
    worst = max(h.worst_gap_m for h in heights)
    conflict = max(c.disagreement_m for c in body.conflicts)
    assert worst == pytest.approx(conflict, abs=1e-6)
    assert any(h.legs_off_the_ground for h in heights)


# --------------------------------------------------------------------------
# That it stays a viewing quantity
# --------------------------------------------------------------------------


def test_the_trajectory_is_left_infeasible(drawn):
    """Plan §12 refused to pick a body height; drawing one does not un-refuse
    it.  If this ever fails, a viewer has written into the planned data."""

    four, body, whole, _ = drawn
    assert not body.is_feasible
    assert all(not np.isfinite(s.body_position_world_m[2])
               for s in whole.samples)

    before = np.asarray(body.body_z_m, dtype=float)
    viewing_heights_2d(four, whole, nominal_body_z_m=body.nominal_body_z_m)
    assert np.array_equal(np.asarray(body.body_z_m, dtype=float), before,
                          equal_nan=True)


def test_every_row_carries_the_basis_and_the_per_leg_cost(drawn):
    _, _, _, heights = drawn
    rows = viewing_height_rows(heights)
    assert len(rows) == len(heights)
    for row in rows:
        assert row["basis"] == VIEWING_BASIS
        assert row["row_kind"] == "viewing_height"
        for leg in LEG_ORDER:
            assert f"{leg.value}_float_gap_mm" in row


def test_airborne_floors_are_reported_not_clipped(drawn):
    """A lower bound is a clearance requirement; the viewer reports its slack
    rather than lifting the body to satisfy it."""

    _, _, whole, heights = drawn
    airborne = [(s, h) for s, h in zip(whole.samples, heights)
                if s.swing_leg is not None]
    assert airborne, "the nominal cycle has swings"
    assert all(h.lower_bound_slack_m is not None for _, h in airborne)


# --------------------------------------------------------------------------
# That the drawing means what the numbers say
# --------------------------------------------------------------------------


def test_the_drawn_foot_is_where_the_gap_says_it_is(drawn):
    """Cross-check against the leg geometry itself, not against the same
    arithmetic: build each leg's scene at the drawn hip and measure its lowest
    point."""

    _, _, whole, heights = drawn
    mounts = {m.leg: m.offset_body_xyz_m
              for m in __import__(
                  "hybrid_note.scripts.experiments.day12_four_leg_state_2d",
                  fromlist=["leg_mounts_2d"]).leg_mounts_2d(0.0)}
    for index in (0, len(whole.samples) // 2):
        sample, height = whole.samples[index], heights[index]
        for leg, state in sample.legs.items():
            if state.mode is LegMode.AIRBORNE:
                continue
            hip = hip_world_xz_m(sample, height, mounts[leg])
            scene = build_single_leg_rolling_scene_2d(
                state.theta_rad, state.beta_rad, hip[0], hip[1],
                obstacle_x_start_m=None, arc_samples=241)
            lowest = float(scene.geometry.points_world_xz_m[:, 1].min())
            assert lowest == pytest.approx(height.float_gap_m[leg],
                                           abs=RIM_MODEL_GAP_M)


def test_the_limits_clear_the_wheel_not_just_the_hip(drawn):
    """Padding sized for the hip cuts the leg-wheel in half."""

    _, _, whole, heights = drawn
    (x_lo, x_hi), (z_lo, z_hi) = sagittal_limits_2d(whole, heights)
    mounts = {m.leg: m.offset_body_xyz_m
              for m in __import__(
                  "hybrid_note.scripts.experiments.day12_four_leg_state_2d",
                  fromlist=["leg_mounts_2d"]).leg_mounts_2d(0.0)}
    sample, height = whole.samples[0], heights[0]
    for leg, state in sample.legs.items():
        hip = hip_world_xz_m(sample, height, mounts[leg])
        scene = build_single_leg_rolling_scene_2d(
            state.theta_rad, state.beta_rad, hip[0], hip[1],
            obstacle_x_start_m=None, arc_samples=121)
        points = scene.geometry.points_world_xz_m
        assert points[:, 1].max() <= z_hi
        assert x_lo <= points[:, 0].min() and points[:, 0].max() <= x_hi
    assert z_lo < 0.0


def test_the_strip_draws(drawn, tmp_path):
    _, _, whole, heights = drawn
    path = tmp_path / "strip.png"
    plot_frame_strip_2d(whole, heights, path, indices=(0, len(whole.samples) // 2))
    assert path.exists() and path.stat().st_size > 0


def test_a_gap_below_the_contact_tolerance_is_not_called_a_gap(drawn):
    """1 mm is the contact tolerance the whole pipeline uses; anything under it
    is the same foot, not a floating one."""

    _, _, _, heights = drawn
    for height in heights:
        for leg, gap in height.float_gap_m.items():
            assert (leg in height.legs_off_the_ground) == (gap > GAP_TOLERANCE_M)
