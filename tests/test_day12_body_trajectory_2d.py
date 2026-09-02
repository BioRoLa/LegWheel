"""Day 12 Step 5: the merged body trajectory.

Plan §12, whose requirement 9 names the four cases the merge has to get right:
no concession, one active lower bound, several compatible lower bounds, and
conflicting hard requirements.  Those four are tested directly on
:func:`merge_demands` -- the whole rule lives there, so testing it there tests
it without a five-minute composition in the way -- and the end-to-end path is
then checked once on a real four-leg plan.
"""

import numpy as np
import pytest

from legwheel.config import RobotParams

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    BodyRequirementKind,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    RecoveryShaping2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BODY_BASIS,
    HARD_AGREEMENT_M,
    HIP_TO_BODY_Z_M,
    BodyDriver,
    LegDemand2D,
    body_rows,
    body_trajectory_2d,
    leg_demand_at,
    merge_demands,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    ScheduledSegment2D,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId

NOMINAL_BODY_Z_M = 0.162283


def _demand(leg: LegId, kind: BodyRequirementKind, body_z_m: float,
            mode: LegMode = LegMode.STANCE) -> LegDemand2D:
    return LegDemand2D(
        leg=leg, segment_index=0, segment_kind="FOOT_RIM_ROLL", phase=None,
        kind=kind, body_z_m=body_z_m, mode=mode,
    )


# --------------------------------------------------------------------------
# Requirement 9's four cases
# --------------------------------------------------------------------------


def test_no_concession_leaves_the_body_at_nominal():
    body_z, driver, winner, conflicts = merge_demands([], NOMINAL_BODY_Z_M, 0.0)
    assert body_z == pytest.approx(NOMINAL_BODY_Z_M)
    assert driver is BodyDriver.NOMINAL
    assert winner is None and conflicts == ()


def test_one_active_lower_bound_raises_the_body_to_it():
    demand = _demand(LegId.LF, BodyRequirementKind.LOWER_BOUND,
                     NOMINAL_BODY_Z_M + 0.020)
    body_z, driver, winner, conflicts = merge_demands(
        [demand], NOMINAL_BODY_Z_M, 0.0)
    assert body_z == pytest.approx(NOMINAL_BODY_Z_M + 0.020)
    assert driver is BodyDriver.LOWER_BOUND
    assert winner is demand and conflicts == ()


def test_a_lower_bound_below_nominal_does_not_lower_the_body():
    """A bound is a floor, not a target: it must never pull the body down."""

    demand = _demand(LegId.LF, BodyRequirementKind.LOWER_BOUND,
                     NOMINAL_BODY_Z_M - 0.030)
    body_z, driver, _, _ = merge_demands([demand], NOMINAL_BODY_Z_M, 0.0)
    assert body_z == pytest.approx(NOMINAL_BODY_Z_M)
    assert driver is BodyDriver.NOMINAL


def test_several_compatible_lower_bounds_take_the_highest_not_the_sum():
    demands = [
        _demand(LegId.LF, BodyRequirementKind.LOWER_BOUND, NOMINAL_BODY_Z_M + 0.010),
        _demand(LegId.RF, BodyRequirementKind.LOWER_BOUND, NOMINAL_BODY_Z_M + 0.025),
        _demand(LegId.LH, BodyRequirementKind.LOWER_BOUND, NOMINAL_BODY_Z_M + 0.005),
    ]
    body_z, driver, winner, conflicts = merge_demands(
        demands, NOMINAL_BODY_Z_M, 0.0)
    assert body_z == pytest.approx(NOMINAL_BODY_Z_M + 0.025)
    assert driver is BodyDriver.LOWER_BOUND
    assert winner.leg is LegId.RF
    assert conflicts == ()


def test_conflicting_hard_requirements_are_refused_not_averaged():
    """Plan §12 requirement 5.  An average would satisfy neither leg."""

    demands = [
        _demand(LegId.LF, BodyRequirementKind.TRACK, 0.20),
        _demand(LegId.RH, BodyRequirementKind.TRACK, 0.24),
    ]
    body_z, driver, winner, conflicts = merge_demands(
        demands, NOMINAL_BODY_Z_M, 1.5)
    assert np.isnan(body_z)
    assert driver is BodyDriver.INFEASIBLE
    assert winner is None
    assert len(conflicts) == 1
    assert conflicts[0].disagreement_m == pytest.approx(0.04)
    assert body_z != pytest.approx(0.22), "the midpoint must never be produced"


# --------------------------------------------------------------------------
# The rest of the merge rule
# --------------------------------------------------------------------------


def test_two_hard_requirements_that_agree_are_one_requirement():
    demands = [
        _demand(LegId.LF, BodyRequirementKind.TRACK, 0.20),
        _demand(LegId.RH, BodyRequirementKind.PINNED,
                0.20 + HARD_AGREEMENT_M / 2),
    ]
    body_z, driver, _, conflicts = merge_demands(demands, NOMINAL_BODY_Z_M, 0.0)
    assert conflicts == ()
    assert driver is BodyDriver.HARD
    assert body_z == pytest.approx(0.20)


def test_a_hard_requirement_beats_a_higher_lower_bound():
    """Hard means hard: a bound cannot lift the body off a tracked profile."""

    demands = [
        _demand(LegId.LF, BodyRequirementKind.TRACK, 0.20),
        _demand(LegId.RF, BodyRequirementKind.LOWER_BOUND, 0.30),
    ]
    body_z, driver, winner, _ = merge_demands(demands, NOMINAL_BODY_Z_M, 0.0)
    assert driver is BodyDriver.HARD
    assert body_z == pytest.approx(0.20)
    assert winner.leg is LegId.LF


def test_track_and_pinned_are_both_hard_and_lower_bound_is_not():
    assert _demand(LegId.LF, BodyRequirementKind.TRACK, 0.2).is_hard
    assert _demand(LegId.LF, BodyRequirementKind.PINNED, 0.2).is_hard
    assert not _demand(LegId.LF, BodyRequirementKind.LOWER_BOUND, 0.2).is_hard


# --------------------------------------------------------------------------
# hip_z -> body_z is Step 2's relation, not a new one
# --------------------------------------------------------------------------


def test_the_hip_to_body_offset_is_the_project_constant():
    assert HIP_TO_BODY_Z_M == pytest.approx(RobotParams.ABAD_AXIS_OFFSET)
    assert HIP_TO_BODY_Z_M > 0.0, "the leg plane hangs above the body origin"


def _scheduled(leg: LegId, kind: SegmentKind, start_s: float, end_s: float,
               mode: LegMode) -> ScheduledSegment2D:
    return ScheduledSegment2D(
        leg=leg, segment_index=0, segment_kind=kind,
        phase_label=kind.value, mode=mode, start_s=start_s, end_s=end_s,
        frame_count=5, window_index=0, duration_is_assigned=True,
    )


def _point(x: float, hip_z: float) -> PointContact2D:
    return PointContact2D(
        rim="foot_rim", alpha_rad=0.0, point_world_xz_m=(x, 0.0),
        surface_id="ground", theta_rad=np.deg2rad(60.0), beta_rad=0.0,
        hip_xz_m=(x, hip_z),
    )


def _track_segment(profile) -> MotionSegment2D:
    profile = np.asarray(profile, dtype=float)
    return MotionSegment2D(
        kind=SegmentKind.FOOT_RIM_ROLL, phase_label="FOOT_RIM_ROLL",
        start_contact=_point(0.0, float(profile[0])),
        end_contact=_point(0.2, float(profile[-1])),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-1.0),
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.2),
            hip_z_profile_m=profile),
        frames=FrameRef2D(source_id="synthetic",
                          indices=tuple(range(len(profile)))),
        rolling=RollingContact2D(
            rim="foot_rim", surface_ids=("ground",),
            alpha_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            beta_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            theta_range_rad=(np.deg2rad(60.0), np.deg2rad(60.0)),
            contact_start_xz_m=(0.0, 0.0), contact_end_xz_m=(0.2, 0.0)),
    )


def test_a_tracked_profile_is_read_at_the_right_place_in_its_window():
    """Step 3 gave the window in proportion to frames, so this is a read."""

    segment = _track_segment([0.20, 0.21, 0.22, 0.23, 0.24])
    scheduled = _scheduled(LegId.LF, SegmentKind.FOOT_RIM_ROLL, 0.0, 1.0,
                           LegMode.STANCE)
    for time_s, hip_z in ((0.0, 0.20), (0.5, 0.22), (1.0, 0.24)):
        demand = leg_demand_at(scheduled, segment, time_s)
        assert demand.body_z_m == pytest.approx(hip_z - HIP_TO_BODY_Z_M)


def test_a_demand_is_a_body_height_not_a_hip_height():
    segment = _track_segment([0.22, 0.22])
    scheduled = _scheduled(LegId.LF, SegmentKind.FOOT_RIM_ROLL, 0.0, 1.0,
                           LegMode.STANCE)
    demand = leg_demand_at(scheduled, segment, 0.5)
    assert demand.body_z_m == pytest.approx(0.22 - RobotParams.ABAD_AXIS_OFFSET)


def test_a_pinned_segment_speaks_only_at_its_endpoints():
    """Day 10-11 Step 9 section C: the interior of a swing is not hard."""

    segment = MotionSegment2D(
        kind=SegmentKind.RECOVERY_SWING, phase_label="RECOVERY_SWING",
        start_contact=_point(0.0, 0.22), end_contact=_point(0.3, 0.23),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-4.0),
                                theta_step_rad=np.deg2rad(2.0)),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.PINNED, x_range_m=(0.0, 0.3),
            hip_z_min_m=0.225),
        frames=FrameRef2D(source_id="synthetic", indices=(0, 1, 2)),
        recovery_shaping=RecoveryShaping2D(
            theta_compact_rad=np.deg2rad(17.0),
            theta_touchdown_rad=np.deg2rad(60.0),
            airborne_rotation_rad=np.deg2rad(280.3125),
            min_clearance_m=0.05, hip_advance_m=0.0),
    )
    scheduled = _scheduled(LegId.LF, SegmentKind.RECOVERY_SWING, 0.0, 1.0,
                           LegMode.AIRBORNE)
    assert leg_demand_at(scheduled, segment, 0.0).body_z_m == pytest.approx(
        0.225 - HIP_TO_BODY_Z_M)
    assert leg_demand_at(scheduled, segment, 1.0).body_z_m == pytest.approx(
        0.225 - HIP_TO_BODY_Z_M)
    assert leg_demand_at(scheduled, segment, 0.5) is None


# --------------------------------------------------------------------------
# End to end on a real four-leg plan
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def nominal_trajectory():
    """Four legs on flat ground, no crossing at all.

    The simplest possible case -- and it is *not* trivially feasible; see
    :func:`test_the_flat_four_leg_run_is_reported_INFEASIBLE`.
    """

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing",
    )
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    return body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]),
        samples=121)


def test_the_trajectory_covers_the_interval_all_four_legs_share(nominal_trajectory):
    assert len(nominal_trajectory.samples) == 121
    assert nominal_trajectory.time_s[0] < nominal_trajectory.time_s[-1]
    assert np.all(np.diff(nominal_trajectory.time_s) > 0)


def test_the_vertical_variation_is_still_reported_over_the_feasible_samples(
        nominal_trajectory):
    """An infeasible instant must not erase the record of the others."""

    assert np.isfinite(nominal_trajectory.body_z_travel_m)
    assert np.isfinite(nominal_trajectory.max_body_z_step_m)


def test_body_x_is_monotone_and_continuous(nominal_trajectory):
    """The body never walks backwards, and never jumps."""

    steps = np.diff(nominal_trajectory.body_x_m)
    assert np.all(steps >= -1e-12)
    assert steps.max() < 0.05


def test_body_y_and_the_three_angles_are_left_alone(nominal_trajectory):
    """Plan §12 requirement 6, checkable rather than assumed."""

    assert nominal_trajectory.body_y_m == 0.0
    assert nominal_trajectory.body_rpy_rad == (0.0, 0.0, 0.0)


def test_the_flat_four_leg_run_is_reported_INFEASIBLE(nominal_trajectory):
    """The headline Step 5 result, and it is not a defect in the merge.

    The foot-rim stroke's hip height is an **arc**: 202.161 mm at both ends and
    219.448 mm at its middle (Step 1).  Walk's phase offsets put the three
    stance legs at different points on that arc, so on a rigid body with
    ``rpy = 0`` they demand three different body heights at the same instant.
    Every one of them is a TRACK requirement, and TRACK is hard.

    Plan §12 requirement 5 says to return infeasible rather than average, so
    this is the merge doing exactly what it was asked -- and the disagreement
    is the real cost of the nominal cycle, not an artefact to be tuned away.
    """

    assert not nominal_trajectory.is_feasible
    assert nominal_trajectory.conflicts


def test_every_body_z_change_can_be_named(nominal_trajectory):
    """Plan §12's acceptance: no unexplained motion.

    An infeasible instant has no height to explain -- but it does name the two
    legs that could not agree, which is the same obligation in the other
    direction.
    """

    for sample in nominal_trajectory.samples:
        if sample.driver is BodyDriver.NOMINAL:
            assert sample.body_z_m == pytest.approx(
                nominal_trajectory.nominal_body_z_m)
        elif sample.driver is BodyDriver.INFEASIBLE:
            assert np.isnan(sample.body_z_m)
            assert sample.driver_leg is None
        else:
            assert sample.driver_leg is not None
            assert sample.driver_segment_kind


def test_the_disagreement_is_bounded_by_the_stroke_s_own_hip_z_travel(
        nominal_trajectory):
    """The mechanism, pinned: it is the arc, not an accumulating error.

    Step 1 measured the stroke's hip_z travel at 17.287 mm.  Two legs on the
    same arc cannot differ by more than that, and the conflicts must not
    exceed it -- if they did, something other than the phase offset would be
    moving the body.
    """

    worst = max(c.disagreement_m for c in nominal_trajectory.conflicts)
    assert worst <= 17.287e-3 + 1e-6
    assert worst > 10e-3, "the disagreement is large, not a rounding artefact"


def test_an_infeasible_instant_produces_no_height_at_all(nominal_trajectory):
    """Not a midpoint, not the nominal, not the last good value: nothing."""

    infeasible = [s for s in nominal_trajectory.samples
                  if s.driver is BodyDriver.INFEASIBLE]
    assert infeasible
    assert all(np.isnan(s.body_z_m) for s in infeasible)


def test_the_summary_reports_how_much_of_the_run_survived(nominal_trajectory):
    summary = nominal_trajectory.as_dict()
    assert summary["feasible_samples"] < len(nominal_trajectory.samples)
    assert summary["max_disagreement_mm"] > 0.0
    assert summary["is_feasible"] is False


def test_the_vertical_variation_is_recorded(nominal_trajectory):
    """Plan §12 requirement 8."""

    assert nominal_trajectory.body_z_travel_m >= 0.0
    assert nominal_trajectory.as_dict()["body_z_travel_mm"] == pytest.approx(
        nominal_trajectory.body_z_travel_m * 1e3)


def test_the_rows_say_what_the_number_is_and_is_not(nominal_trajectory):
    """A body-frame approximation must not be readable as a whole-robot CoM."""

    rows = body_rows(nominal_trajectory)
    assert all(len(r) == len(rows[0]) for r in rows)
    assert rows[0]["basis"] == BODY_BASIS
    assert "no whole-robot CoM model" in BODY_BASIS
    assert not any("com" in key.lower() for key in rows[0])


def test_the_table_carries_the_concessions_it_found(nominal_trajectory):
    rows = body_rows(nominal_trajectory)
    kinds = {r["row_kind"] for r in rows}
    assert {"summary", "sample"} <= kinds
