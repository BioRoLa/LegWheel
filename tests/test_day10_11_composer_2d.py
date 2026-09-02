"""Day 10--11 Step 7: the composer's contract.

Cheap tests only.  The three composed sequences and their hand-over numbers are
produced by ``day10_11_step7_driver.py``, which has to run a full rolling
traversal to make them.

What is pinned here is the part Step 7 changed: that a decision now carries
**enough** to rebuild the motion it chose, that a refuted strategy answers with
its refutation instead of being absent, and that the two extra hand-over checks
the spec asks for measure what they claim to.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    TransitionKind,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    NOMINAL_RIM_GAP_M,
    RIM_SEAMS_DEG,
    ComposedSequence2D,
    compose_2d,
    compose_swing_swing_2d,
    seam_margin_deg,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    REFUTATIONS,
    Verdict,
    DecisionTables2D,
    RollThetaRow2D,
    StrategyId,
    SwingDownRow2D,
    SwingUpRow2D,
    swing_swing_cell_2d,
)


# --------------------------------------------------------------------------
# What Step 7 found missing from the decision
# --------------------------------------------------------------------------


def _tables() -> DecisionTables2D:
    """One height, with a repair on each half -- the case that exposed the gap."""

    return DecisionTables2D(
        roll=(RollThetaRow2D(0.10, 40.0, True, 0.114, 0.004, 0.270),),
        swing_up=(
            SwingUpRow2D(0.10, 0.02, True, 0.0, 0.005,
                         min_liftoff_rise_m=0.03, min_touchdown_drop_m=0.0,
                         duration_scale=1.0),
        ),
        swing_down=(
            SwingDownRow2D(0.10, 0.08, True, 0.0, 0.005,
                           min_liftoff_rise_m=0.0, min_touchdown_drop_m=0.02,
                           duration_scale=1.5),
        ),
    )


def test_a_decision_carries_the_repairs_its_sweep_needed():
    """Step 7's first finding, and why it is not cosmetic.

    Step 2 and Step 3 reach a feasible cell by walking repair ladders --
    ``liftoff_rise``, ``touchdown_drop``, ``duration_scale`` -- on top of the
    body knob.  The first composer omitted them and reproduced a
    ``TERRAIN_COLLISION`` the sweep had already repaired away.  A decision that
    reports only the body knob is not enough to regenerate what it chose, which
    is what the spec's third completion criterion asks the rule to be.
    """

    cell = swing_swing_cell_2d(0.10, 0.35, _tables())
    parameters = dict(cell.parameters)
    assert parameters["ascent_liftoff_rise_m"] == pytest.approx(0.03)
    assert parameters["descent_touchdown_drop_m"] == pytest.approx(0.02)
    assert parameters["descent_duration_scale"] == pytest.approx(1.5)


def test_the_body_knob_is_still_what_gets_priced():
    """The repairs are trajectory shaping, not body concession.

    Step 3 fixed the ladder order for exactly this reason; the repairs must not
    leak into ``body_deviation_m`` or Step 5's comparison changes meaning.
    """

    cell = swing_swing_cell_2d(0.10, 0.35, _tables())
    assert cell.body_deviation_m == pytest.approx(0.10)


def test_the_rolling_composer_reuses_day_6_7s_own_settings_bundle():
    """Step 7's third finding: two coupled settings, and both defaults wrong.

    ``ObstacleSpec2D.arc_samples`` defaults to 241 but Day 6--7's sweep ran at
    121, and ``max_seam_bridge_m`` has to be **paired** with whatever that is
    (implementation log trap 2) -- ``seam_bridge_for_sampling_m(121)`` is
    17.5 mm against ``TraversalConstraints2D``'s 5 mm default.

    The first composer set them by hand and the traversal failed where the
    sweep says it succeeds, with ``COUPLED_RESET_COLLISION_BLOCKED``.  Building
    through ``SweepSettings2D`` makes that impossible; this pins the pairing
    the composer now depends on.
    """

    from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (
        ObstacleSpec2D,
        TraversalConstraints2D,
    )
    from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (
        SweepSettings2D,
        seam_bridge_for_sampling_m,
    )

    settings = SweepSettings2D()
    assert settings.constraints.max_seam_bridge_m == pytest.approx(
        seam_bridge_for_sampling_m(settings.arc_samples)
    )
    # Both defaults the hand-built version picked up are wrong for this sweep.
    assert settings.arc_samples != ObstacleSpec2D().arc_samples
    assert settings.constraints.max_seam_bridge_m != pytest.approx(
        TraversalConstraints2D().max_seam_bridge_m
    )


# --------------------------------------------------------------------------
# The two hand-over checks the spec adds
# --------------------------------------------------------------------------


def test_the_seam_margin_measures_distance_to_the_nearest_rim_boundary():
    """Spec 6.3: a hand-over near a seam is one where the contact can jump."""

    assert seam_margin_deg(np.deg2rad(0.0)) == pytest.approx(40.0)
    assert seam_margin_deg(np.deg2rad(35.0)) == pytest.approx(5.0)
    assert seam_margin_deg(np.deg2rad(-38.0)) == pytest.approx(2.0)
    assert seam_margin_deg(np.deg2rad(179.0)) == pytest.approx(1.0)


def test_the_seams_are_the_four_the_rim_segmentation_actually_has():
    assert RIM_SEAMS_DEG == (-180.0, -40.0, 40.0, 180.0)


def test_the_rim_geometry_gap_is_zero_on_the_foot_rim_and_1_2_mm_above():
    """Step 0 measured it; Step 7 only has to check it does not accumulate."""

    from legwheel.planners.hybrid import RimId
    from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
        rim_point_model_gap_2d,
    )

    theta, beta = np.deg2rad(60.0), 0.0
    foot = rim_point_model_gap_2d(theta, beta, RimId.FOOT, 0.0)
    upper = rim_point_model_gap_2d(theta, beta, RimId.RIGHT, np.deg2rad(90.0))
    assert foot == pytest.approx(0.0, abs=1e-9)
    assert upper == pytest.approx(NOMINAL_RIM_GAP_M, abs=5e-5)


# --------------------------------------------------------------------------
# Refusals are outputs
# --------------------------------------------------------------------------


def test_a_blocked_pair_answers_with_its_reason_and_its_route_back():
    """Step 7's task list names two pairs whose primitives will not chain.

    A reader who finds only three sequences must be able to see why without
    going back through two earlier steps, so the composer answers for all five
    -- and spec 5.5 requires the answer to carry the route back, not just the
    refusal.
    """

    for strategy in BLOCKED_PAIRS:
        result = compose_2d(0.10, 0.35, _tables(), strategy=strategy)
        assert not result.composed
        assert result.sequence is None
        assert result.verdict is Verdict.DIRECT_HANDOFF_INFEASIBLE
        assert BLOCKED_PAIRS[strategy].evidence in result.refusal


def test_a_blocked_pair_never_claims_the_robot_cannot_do_it():
    """Spec 5.5: ``PHYSICALLY_INFEASIBLE`` needs evidence nobody here has."""

    for strategy in BLOCKED_PAIRS:
        result = compose_2d(0.10, 0.35, _tables(), strategy=strategy)
        assert result.as_dict()["verdict"] != Verdict.PHYSICALLY_INFEASIBLE.value


def test_a_blocked_pair_records_the_transition_it_would_need():
    """Spec 5.6, as data rather than prose.

    A reader of the CSV alone has to be able to tell "unsolved" from
    "absent", which is the whole reason the record exists.
    """

    result = compose_2d(0.10, 0.35, _tables(), strategy=StrategyId.SWING_ROLL)
    assert len(result.unresolved) == 1
    requirement = result.unresolved[0]
    assert requirement.kind is TransitionKind.TOP_REPOSITION
    assert requirement.requires_external_support
    assert not requirement.resolved
    assert "LEFT_RIM_READY" in requirement.target_condition
    assert result.as_dict()["unresolved_transitions"] == 1


def test_a_composed_strategy_carries_no_unresolved_transition():
    result = ComposedSequence2D(
        strategy=StrategyId.SWING_OVER, height_m=0.06, top_length_m=0.075,
        sequence=_Sequence(), refusal=None,
    )
    assert result.as_dict()["verdict"] == Verdict.COMPOSED.value
    assert result.as_dict()["unresolved_transitions"] == 0


def test_a_cell_with_no_winner_says_so_rather_than_composing_something():
    result = compose_2d(0.10, 0.05, _tables())
    assert not result.composed
    assert "no strategy" in result.refusal


def test_an_infeasible_strategy_reports_which_limiter_stopped_it():
    result = compose_2d(0.10, 0.19, _tables(), strategy=StrategyId.ROLL_ROLL)
    assert not result.composed
    assert "top length" in result.refusal


def test_the_swing_pair_refuses_a_top_too_short_before_planning_anything():
    """No planner runs when the geometry already rules the pair out."""

    result = compose_swing_swing_2d(
        0.10, 0.20, approach_clearance_m=0.02, min_hip_lift_m=0.0,
        takeoff_distance_m=0.08, min_hip_hold_fraction=0.0,
        landing_distance_m=0.16,
    )
    assert not result.composed
    assert "too short" in result.refusal
    assert result.seconds < 1.0


# --------------------------------------------------------------------------
# Partial sequences -- Step 8's need
# --------------------------------------------------------------------------


class _Sequence:
    """Only what ``as_dict`` reads; the schema itself is tested elsewhere."""

    segments = ("a", "b")
    frame_indices = (0, 1, 2)


def _partial() -> ComposedSequence2D:
    return ComposedSequence2D(
        strategy=StrategyId.ROLL_ROLL, height_m=0.16, top_length_m=0.35,
        sequence=_Sequence(), refusal="the traversal failed at ROLL_DOWN",
        partial=True,
    )


def test_a_partial_sequence_is_not_a_composed_plan():
    """It is what the leg managed before it stopped, not a plan.

    Step 8 needs it to price ``ROLL_UP`` at h = 160 mm, where Day 6--7 has the
    ascent succeeding at all ten thetas and the descent at none.  That ascent
    is real work, and the pair it belongs to is refused for a reason that has
    nothing to do with it -- but a caller must not mistake it for a crossing.
    """

    result = _partial()
    assert result.sequence is not None
    assert not result.composed
    assert result.as_dict()["partial"] is True
    assert result.as_dict()["composed"] is False


def test_a_complete_sequence_is_not_marked_partial():
    result = ComposedSequence2D(
        strategy=StrategyId.SWING_OVER, height_m=0.06, top_length_m=0.075,
        sequence=_Sequence(), refusal=None,
    )
    assert result.composed
    assert result.as_dict()["partial"] is False


# --------------------------------------------------------------------------
# The report
# --------------------------------------------------------------------------


def test_an_uncomposed_result_reports_no_counts_rather_than_zeros_that_read_as_data():
    result = ComposedSequence2D(
        strategy=StrategyId.ROLL_ROLL, height_m=0.1, top_length_m=0.35,
        sequence=None, refusal="nope",
    )
    row = result.as_dict()
    assert row["composed"] is False
    assert row["max_theta_jump_deg"] is None
    assert row["min_seam_margin_deg"] is None
    assert row["refusal"] == "nope"
