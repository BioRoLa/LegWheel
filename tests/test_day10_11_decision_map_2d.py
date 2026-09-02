"""Day 10--11 Step 5: the decision function's contract.

Cheap tests only, on synthetic tables.  The real map is made by
``day10_11_step5_driver.py``, which needs Steps 2/3/4's CSVs and a
``SWING_OVER`` sweep to make it.

What is pinned here is the structure of the decision rather than its answers:
that ``theta_climb`` is chosen by ``L_top`` and not freely, that "never
measured" cannot masquerade as "infeasible", that a refuted strategy stays
refuted, and that the lexicographic order is a parameter -- because the spec's
last completion criterion is a sensitivity check on exactly that order.
"""

import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    DEFAULT_ORDER,
    Verdict,
    REFUTATIONS,
    Availability,
    DecisionTables2D,
    Limiter,
    RollThetaRow2D,
    StrategyId,
    SwingDownRow2D,
    SwingOverRow2D,
    SwingUpRow2D,
    decide_2d,
    roll_roll_cell_2d,
    swing_over_cell_2d,
    swing_swing_cell_2d,
)


# --------------------------------------------------------------------------
# Synthetic tables, shaped like the measured ones
# --------------------------------------------------------------------------


def _tables(**overrides) -> DecisionTables2D:
    """Rolling that is cheap when extended-slowly, swing that is flat.

    The two thetas mirror the measured opposition: 40 deg needs a longer top
    (0.27 m) but moves the hip less (0.114 m); 85 deg fits on 0.205 m but costs
    0.150 m.  That is the whole mechanism the decision rule turns on.
    """

    base = dict(
        roll=(
            RollThetaRow2D(0.10, 40.0, True, 0.114, 0.004, 0.270),
            RollThetaRow2D(0.10, 85.0, True, 0.150, 0.006, 0.205),
        ),
        swing_up=(
            SwingUpRow2D(0.10, 0.04, True, 0.0, 0.005),
            SwingUpRow2D(0.10, 0.06, True, 0.02, 0.008),
        ),
        swing_down=(
            SwingDownRow2D(0.10, 0.08, True, 0.0, 0.005),
            SwingDownRow2D(0.10, 0.24, True, 0.5, 0.005),
        ),
        swing_over=(
            SwingOverRow2D(0.10, 0.05, 70.0, True, 0.04, 0.30, 0.09, 0.004, None),
            SwingOverRow2D(0.10, 0.30, 70.0, False, 0.04, None, None, None,
                           "IK_NOT_CONVERGED"),
        ),
    )
    base.update(overrides)
    return DecisionTables2D(**base)


# --------------------------------------------------------------------------
# The mechanism: L_top chooses theta_climb
# --------------------------------------------------------------------------


def test_the_top_length_demand_is_a_function_of_theta_alone():
    """Measured, and the reason ``L_top`` can pick theta without a search.

    Day 6--7 Step 12R's 70 cells agree across heights to under 3 um at eight of
    the ten thetas, so the table collapses to one number per theta.
    """

    required = _tables().required_top_length_for_theta()
    assert set(required) == {40.0, 85.0}
    assert required[85.0] < required[40.0]


def test_a_long_top_buys_the_cheapest_climb():
    cell = roll_roll_cell_2d(0.10, 0.35, _tables())
    assert cell.feasible
    assert dict(cell.parameters)["theta_climb_deg"] == 40.0
    assert cell.body_deviation_m == pytest.approx(0.114)


def test_a_short_top_forces_a_more_extended_climb_and_costs_more():
    """The staircase.  Neither half of theta is free; ``L_top`` resolves them."""

    cell = roll_roll_cell_2d(0.10, 0.21, _tables())
    assert cell.feasible
    assert dict(cell.parameters)["theta_climb_deg"] == 85.0
    assert cell.body_deviation_m == pytest.approx(0.150)


def test_an_unmeasured_theta_carries_the_requirement_and_says_it_is_unvalidated():
    """Step 7's second finding, and why it is **not** a flat margin.

    ``required_top_length_m`` is what a traversal that succeeded consumed, not
    a precondition, so it is not a bound on its own.  But the two direct
    brackets disagree about how wrong it is: at ``theta = 40`` Step 12R's sweep
    failed at exactly the requirement and succeeded 10 mm later, while at
    ``theta = 70, h = 140`` Step 7 bracketed the boundary to ``(220, 225]`` and
    the requirement -- 222.5 mm -- lies **inside** it.

    Adding a flat 10 mm would fit one sweep's grid spacing to every theta, and
    the second measurement says that overshoots.  So an unmeasured theta keeps
    the requirement and is marked unvalidated.
    """

    cell = roll_roll_cell_2d(0.10, 0.35, _tables())
    parameters = dict(cell.parameters)
    assert parameters["required_top_length_m"] == pytest.approx(0.270)
    assert parameters["top_length_bound_measured"] == 0.0
    assert "inferred" in cell.reason


def test_a_measured_bound_is_used_as_it_is_and_labelled():
    tables = _tables(roll=(
        RollThetaRow2D(0.10, 40.0, True, 0.114, 0.004, 0.270,
                       minimum_feasible_top_length_m=0.2794),
    ))
    cell = roll_roll_cell_2d(0.10, 0.35, tables)
    parameters = dict(cell.parameters)
    assert parameters["required_top_length_m"] == pytest.approx(0.2794)
    assert parameters["top_length_bound_measured"] == 1.0
    assert "measured" in cell.reason


def test_a_top_shorter_than_every_theta_needs_names_the_cheapest_demand():
    cell = roll_roll_cell_2d(0.10, 0.19, _tables())
    assert not cell.feasible
    assert cell.limiter is Limiter.TOP_LENGTH
    assert "205" in cell.reason


def test_a_feasible_rolling_cell_demands_a_trajectory():
    assert roll_roll_cell_2d(0.10, 0.35, _tables()).requirement_kind is (
        BodyRequirementKind.TRACK
    )


# --------------------------------------------------------------------------
# "Never measured" is not "infeasible"
# --------------------------------------------------------------------------


def test_a_height_nobody_swept_is_not_reported_as_infeasible():
    """Otherwise the map grows a region boundary out of a gap in the experiments."""

    cell = roll_roll_cell_2d(0.40, 0.35, _tables())
    assert cell.availability is Availability.NOT_MEASURED
    assert cell.limiter is Limiter.NO_DATA
    assert not cell.feasible


def test_a_height_that_was_swept_and_failed_is_infeasible():
    tables = _tables(roll=(RollThetaRow2D(0.10, 40.0, False, None, None, 0.270),))
    cell = roll_roll_cell_2d(0.10, 0.35, tables)
    assert cell.availability is Availability.INFEASIBLE
    assert cell.limiter is Limiter.ASCENT


# --------------------------------------------------------------------------
# The swing pair's geometric coupling
# --------------------------------------------------------------------------


def test_the_pair_needs_room_for_the_landing_and_the_takeoff():
    """``takeoff <= L_top - landing_distance`` -- the two must not overlap."""

    tables = _tables()
    assert swing_swing_cell_2d(0.10, 0.25, tables, landing_distance_m=0.16).feasible
    tight = swing_swing_cell_2d(0.10, 0.20, tables, landing_distance_m=0.16)
    assert not tight.feasible
    assert tight.limiter is Limiter.TOP_LENGTH
    assert "240" in tight.reason


def test_the_pairs_demand_is_the_item_wise_maximum_of_its_two_halves():
    """Spec task 3.  The ascent's ``h + lift`` dominates a hold, which is <= h."""

    cell = swing_swing_cell_2d(0.10, 0.35, _tables())
    assert cell.body_deviation_m == pytest.approx(0.10)
    assert cell.requirement_kind is BodyRequirementKind.LOWER_BOUND


def test_the_pair_takes_the_cheapest_clearance_and_takeoff():
    cell = swing_swing_cell_2d(0.10, 0.35, _tables())
    parameters = dict(cell.parameters)
    assert parameters["min_hip_lift_m"] == pytest.approx(0.0)
    assert parameters["min_hip_hold_fraction"] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# SWING_OVER
# --------------------------------------------------------------------------


def test_the_over_swing_moves_the_body_vertically_not_at_all():
    """Both ends stand on the lower ground, so the straight hip line is flat."""

    cell = swing_over_cell_2d(0.10, 0.05, _tables())
    assert cell.feasible
    assert cell.body_deviation_m == pytest.approx(0.0)


def test_the_over_swings_stance_price_is_reported_but_not_charged():
    """Whether it is already paid depends on the gait between obstacles.

    Folding it into ``body_deviation_m`` would answer a Day 15--16 question
    inside a Day 10--11 comparison.
    """

    cell = swing_over_cell_2d(0.10, 0.05, _tables())
    assert dict(cell.parameters)["stance_hip_above_min_m"] == pytest.approx(0.09)
    assert cell.body_deviation_m == pytest.approx(0.0)


def test_a_top_too_wide_to_span_is_limited_by_stride():
    cell = swing_over_cell_2d(0.10, 0.30, _tables())
    assert not cell.feasible
    assert cell.limiter is Limiter.STRIDE
    assert "50 mm" in cell.reason


def test_between_swept_tops_the_over_swing_closes_over_monotonically():
    """Licensed by the sweep: its feasible set is a strict prefix per height.

    Without the closure the map shows ``#5`` as isolated dots at the ten swept
    columns, which reads as a scattered result rather than the region it is.
    The interpolated cell must say so, and must not claim a more retracted
    stance than a measured wider top needed.
    """

    cell = swing_over_cell_2d(0.10, 0.04, _tables())
    assert cell.feasible
    parameters = dict(cell.parameters)
    assert parameters["measured_exactly"] == 0.0
    assert parameters["theta_deg"] == 70.0
    assert "from the 50 mm column" in cell.reason


def test_an_exactly_swept_top_is_not_labelled_as_interpolated():
    cell = swing_over_cell_2d(0.10, 0.05, _tables())
    assert dict(cell.parameters)["measured_exactly"] == 1.0


def test_a_height_the_over_sweep_skipped_is_still_not_measured():
    """The closure covers top lengths, never heights."""

    assert swing_over_cell_2d(0.40, 0.05, _tables()).availability is (
        Availability.NOT_MEASURED
    )


# --------------------------------------------------------------------------
# The decision
# --------------------------------------------------------------------------


def test_the_blocked_pairs_stay_in_the_map_and_carry_their_reason():
    """A silent omission would let a later reader think they were never tried."""

    decision = decide_2d(0.10, 0.35, _tables())
    blocked = [c for c in decision.cells
               if c.availability is Availability.HANDOFF_BLOCKED]
    assert {c.strategy for c in blocked} == set(BLOCKED_PAIRS)
    assert all(c.reason for c in blocked)
    assert all(c.strategy not in decision.feasible_strategies for c in blocked)


def test_a_blocked_pair_claims_only_that_the_primitives_will_not_chain():
    """Spec 5.5.  The label is about the primitive set, not about the robot.

    ``PHYSICALLY_INFEASIBLE`` needs reach / joint-limit / collision / support
    evidence, and nothing in Day 10--11 has it.  A cell that claimed it would
    mislead the multi-leg stage -- the one place these are most likely to be
    resolved.
    """

    decision = decide_2d(0.10, 0.35, _tables())
    for cell in decision.cells:
        assert cell.effective_verdict is not Verdict.PHYSICALLY_INFEASIBLE
    blocked = [c for c in decision.cells
               if c.availability is Availability.HANDOFF_BLOCKED]
    assert all(
        c.effective_verdict is Verdict.DIRECT_HANDOFF_INFEASIBLE for c in blocked
    )


def test_an_ordinary_refusal_is_out_of_envelope_not_a_handover_failure():
    """The over-correction this label exists to stop.

    The first pass derived ``DIRECT_HANDOFF_INFEASIBLE`` for *every* refusal,
    which made ``#1`` on a short top look like a hand-over problem.  It is not:
    the strategy works elsewhere and this terrain is outside its range.
    Spec 5.5's original four were written about ``#2`` / ``#3``; stretching them
    over the whole map is the same over-generalisation pointed the other way.
    """

    short_top = roll_roll_cell_2d(0.10, 0.19, _tables())
    assert not short_top.feasible
    assert short_top.limiter is Limiter.TOP_LENGTH
    assert short_top.effective_verdict is Verdict.OUT_OF_ENVELOPE


def test_an_unmeasured_cell_keeps_its_own_verdict():
    """"Nobody looked" is not a kind of "no"."""

    cell = roll_roll_cell_2d(0.40, 0.35, _tables())
    assert cell.availability is Availability.NOT_MEASURED
    assert cell.effective_verdict is Verdict.NOT_MEASURED


def test_every_blocked_pair_keeps_a_route_back():
    """Neither is a dead end, and the record has to say which route.

    ``#2`` has a single-leg fix (a retract that stops above 35 deg) as well as
    a multi-leg one; ``#3`` has only the multi-leg route.  Losing that
    distinction is how a conditional result becomes a permanent one.
    """

    roll_swing = BLOCKED_PAIRS[StrategyId.ROLL_SWING]
    swing_roll = BLOCKED_PAIRS[StrategyId.SWING_ROLL]
    assert roll_swing.single_leg_fix and "35 deg" in roll_swing.single_leg_fix
    assert swing_roll.single_leg_fix is None
    assert all(b.multileg_route for b in BLOCKED_PAIRS.values())
    assert all(
        b.verdict is Verdict.DIRECT_HANDOFF_INFEASIBLE
        for b in BLOCKED_PAIRS.values()
    )


def test_a_feasible_cell_is_labelled_composed():
    decision = decide_2d(0.10, 0.35, _tables())
    feasible = [c for c in decision.cells if c.feasible]
    assert feasible
    assert all(c.effective_verdict is Verdict.COMPOSED for c in feasible)


def test_the_decision_is_a_pure_function_of_its_arguments():
    tables = _tables()
    first = decide_2d(0.10, 0.35, tables)
    second = decide_2d(0.10, 0.35, tables)
    assert first.winner is second.winner
    assert first.as_dict() == second.as_dict()


def test_the_lexicographic_order_is_a_parameter_not_an_assumption():
    """Spec completion criterion 5 is a sensitivity check on this order."""

    tables = _tables()
    body_first = decide_2d(0.10, 0.35, tables, order=DEFAULT_ORDER)
    roll_first = decide_2d(
        0.10, 0.35, tables,
        order=("feasible", "roll_preference", "body", "margin"),
    )
    # At this top length the over-swing has no data, so the contest is #4's
    # 100 mm bound against #1's 114 mm trajectory.  Body cost picks #4; the
    # roll preference picks #1 -- from the same table, which is the point.
    assert body_first.winner is StrategyId.SWING_SWING
    assert roll_first.winner is StrategyId.ROLL_ROLL


def test_an_unknown_ordering_key_is_refused():
    with pytest.raises(ValueError):
        decide_2d(0.10, 0.35, _tables(), order=("feasible", "vibes"))


def test_the_cost_gap_is_what_the_runner_up_would_have_demanded():
    decision = decide_2d(0.10, 0.35, _tables())
    winner = decision.winner_cell
    assert decision.cost_gap_m is not None
    assert decision.cost_gap_m >= 0.0
    assert winner.body_deviation_m + decision.cost_gap_m == pytest.approx(
        min(c.body_deviation_m for c in decision.cells
            if c.feasible and c.strategy is not decision.winner)
    )


def test_a_cell_under_the_margin_floor_is_not_treated_as_feasible():
    """Spec 6.2: the clearances in play are millimetres and move with sampling."""

    tables = _tables()
    generous = decide_2d(0.10, 0.35, tables, margin_floor_m=0.0)
    strict = decide_2d(0.10, 0.35, tables, margin_floor_m=0.010)
    assert len(strict.feasible_strategies) < len(generous.feasible_strategies)


def test_a_cell_with_no_option_reports_no_winner_rather_than_guessing():
    decision = decide_2d(0.10, 0.05, _tables(swing_over=()))
    assert decision.winner is None
    assert decision.feasible_strategies == ()
    assert decision.as_dict()["winner"] is None
