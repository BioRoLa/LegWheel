"""Day 8--9 showcase: "given a step this tall, can the leg swing onto it?"."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (  # noqa: E402
    StepSwingShowcase2D,
    animate_step_swing_2d,
    swing_off_step_2d,
    swing_onto_step_2d,
)

FAST = {"sample_count": 21, "arc_samples": 61}


def test_a_low_step_needs_no_adjustment_at_all():
    result = swing_onto_step_2d(0.02, **FAST)

    assert result.feasible is True
    assert result.plan.valid is True
    assert result.adjustments == ()
    assert result.hip_lift_m == 0.0
    assert result.liftoff_rise_m == 0.0
    assert result.swing_duration_s == result.original_duration_s
    assert "no adjustment needed" in result.summary()


def test_a_tall_step_reports_every_adjustment_it_needed():
    """Nothing is folded in silently -- least of all raising the body."""

    result = swing_onto_step_2d(0.14, **FAST)

    assert result.feasible is True
    assert result.hip_lift_m > 0.0
    assert any("hip raised" in item for item in result.adjustments)
    # Raising the hip shifts the whole reachable range up, so theta relaxes.
    assert result.theta_min_deg > np.rad2deg(0.29)  # ~17 deg limit, comfortably clear
    assert result.minimum_clearance_m > 0.0


def test_forbidding_a_hip_lift_changes_the_answer():
    """The hip trajectory is an input; refusing to move it is a real constraint."""

    with_lift = swing_onto_step_2d(0.14, **FAST)
    without_lift = swing_onto_step_2d(0.14, allow_hip_lift=False, **FAST)

    assert with_lift.feasible is True
    assert without_lift.feasible is False
    assert without_lift.hip_lift_m == 0.0
    assert "out of reach" in without_lift.reason


def test_an_impossible_step_is_refused_with_an_actionable_reason():
    result = swing_onto_step_2d(0.20, **FAST)

    assert result.feasible is False
    assert result.reason
    # It says which input to change, not just that it failed.
    assert "approach geometry" in result.reason or "hip" in result.reason
    assert "NO swing up" in result.summary()


def test_a_refused_step_can_still_be_watched():
    """A failure you can play back is a different answer from the word 'no'."""

    result = swing_onto_step_2d(0.20, **FAST)

    assert result.plan is not None            # a best-effort plan is kept
    animation = animate_step_swing_2d(result, frame_stride=4, arc_samples=61)
    assert animation is not None
    matplotlib.pyplot.close(animation._fig)


def test_the_animation_title_states_the_verdict():
    feasible = swing_onto_step_2d(0.04, **FAST)

    animation = animate_step_swing_2d(feasible, frame_stride=4, arc_samples=61)
    title = animation._fig.axes[0].get_title()

    assert "40 mm step" in title
    assert "FEASIBLE" in title
    matplotlib.pyplot.close(animation._fig)


def test_approach_distance_decides_more_than_height_does():
    """The Day 8--9 headline, as a test: the same step, two approaches."""

    close = swing_onto_step_2d(0.04, approach_distance_m=0.10, allow_repair=False, **FAST)
    far = swing_onto_step_2d(0.04, approach_distance_m=0.20, allow_repair=False, **FAST)

    assert far.feasible is True
    assert close.feasible is False
    # Both poses are legal; it is the swing between them that hits the step.
    assert close.minimum_clearance_m < 0.0 < far.minimum_clearance_m


def test_standing_too_close_is_refused_before_any_swing_is_planned():
    """Closer still, and there is no start pose to swing from at all."""

    result = swing_onto_step_2d(0.04, approach_distance_m=0.06, **FAST)

    assert result.feasible is False
    assert result.plan is None
    assert "cannot even stand at the start pose" in result.reason
    assert "approach_distance_m" in result.reason


def test_a_non_positive_height_is_rejected():
    for bad in (0.0, -0.05):
        with pytest.raises(ValueError, match="finite and positive"):
            swing_onto_step_2d(bad)


def test_showcase_rows_are_table_ready():
    result = swing_onto_step_2d(0.06, **FAST)

    row = result.as_dict()

    assert row["obstacle_mm"] == pytest.approx(60.0)
    assert row["feasible"] is True
    for key in ("liftoff_rise_mm", "hip_lift_mm", "duration_s", "theta_min_deg",
                "min_clearance_mm", "adjustments"):
        assert key in row
    assert isinstance(result, StepSwingShowcase2D)


# ---------------------------------------------------------------------------
# Stepping down is not climbing with the sign flipped
# ---------------------------------------------------------------------------


def test_a_low_step_can_be_stepped_down_without_adjustment():
    result = swing_off_step_2d(0.04, **FAST)

    assert result.feasible is True
    assert result.direction == "off"
    assert result.adjustments == ()
    assert "descent found" in result.summary()
    assert result.plan.result.request.start.contact_point_world_xz_m[1] > (
        result.plan.result.request.target.target_point_world_xz_m[1]
    )


#: The 160 mm descent sits close to the joint-continuity limit, and that limit
#: is per sample -- so this case carries its own resolution rather than
#: inheriting the fast one, exactly as the regression cases do.
DESCENT = {"sample_count": 31, "arc_samples": 61}


def test_descending_a_tall_step_needs_the_body_held_back():
    """The opposite knob from climbing: the hip must not fall with the foot."""

    result = swing_off_step_2d(0.16, **DESCENT)

    assert result.feasible is True
    assert result.hip_lift_m > 0.0
    assert any("held above" in item for item in result.adjustments)


def test_letting_the_body_drop_with_the_foot_fails_where_holding_it_succeeds():
    """The measurement the descent showcase exists to encode."""

    held = swing_off_step_2d(0.16, **DESCENT)
    dropped = swing_off_step_2d(0.16, allow_hip_hold=False, **DESCENT)

    assert held.feasible is True
    assert dropped.feasible is False
    assert dropped.hip_lift_m == 0.0
    assert "out of reach" in dropped.reason


def test_the_two_directions_are_not_mirror_images():
    """Same height, opposite direction, different body knob."""

    up = swing_onto_step_2d(0.16, **DESCENT)
    down = swing_off_step_2d(0.16, **DESCENT)

    assert up.feasible and down.feasible
    assert up.direction == "onto"
    assert down.direction == "off"
    assert any("hip raised" in item for item in up.adjustments)
    assert any("held above" in item for item in down.adjustments)


def test_a_descent_can_be_watched_too():
    result = swing_off_step_2d(0.12, **FAST)

    animation = animate_step_swing_2d(result, frame_stride=4, arc_samples=61)
    title = animation._fig.axes[0].get_title()

    assert "120 mm step" in title
    matplotlib.pyplot.close(animation._fig)


def test_hold_fractions_must_be_fractions():
    with pytest.raises(ValueError, match="fractions in"):
        swing_off_step_2d(0.04, hip_hold_ladder=(0.0, 1.5), **FAST)
