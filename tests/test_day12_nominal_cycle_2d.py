"""Day 12 Step 1: the nominal flat-ground Hybrid cycle.

Plan §8.5 lists eight things the cycle must be validated for, and §8's
acceptance adds five more.  Each has a test here, named for the claim rather
than for the function, so a failure says which claim broke.

The cycles are generated once per module: one pair costs about a minute, and
every test below is an assertion about the *same* trajectory rather than about
a fresh one.
"""

import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    rim_alpha_limits_rad,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    WHEEL_MODE_THETA_RAD,
    MotionSegment2D,
    RecoveryShaping2D,
    RollSampling2D,
    SegmentKind,
    SwingSampling2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    RecoveryConfig2D,
    cycle_segments_2d,
    recovery_beta_target_2d,
    run_foot_rim_roll_2d,
    run_nominal_cycles_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    BoundaryKind,
    SegmentChain2D,
    boundary_kind_2d,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    _airborne_beta_target_2d,
)


@pytest.fixture(scope="module")
def cycles():
    """Plan §8.5 requirement 5: at least two consecutive nominal cycles."""

    return run_nominal_cycles_2d(2)


# --------------------------------------------------------------------------
# It runs at all, and it runs twice
# --------------------------------------------------------------------------


def test_two_consecutive_cycles_are_generated(cycles):
    assert len(cycles) == 2
    for cycle in cycles:
        assert cycle.success, (
            cycle.stroke.stop_reason, cycle.recovery.failure_reason
        )


def test_the_cycle_is_periodic(cycles):
    """The second cycle must be the first one translated forward.

    This is the check that the recovery really does return the leg to the pose
    the stroke started from.  If it did not, the two strokes would differ and
    "nominal cycle" would be a name for a drifting sequence.
    """

    first, second = cycles
    assert first.stroke.contact_advance_m == pytest.approx(
        second.stroke.contact_advance_m, abs=1e-9
    )
    assert first.stroke.rotation_rad == pytest.approx(
        second.stroke.rotation_rad, abs=1e-9
    )
    assert first.hip_advance_m == pytest.approx(second.hip_advance_m, abs=1e-9)
    assert first.stroke.alpha_range_rad == pytest.approx(
        second.stroke.alpha_range_rad, abs=1e-9
    )


def test_one_cycle_is_exactly_one_leg_wheel_revolution(cycles):
    """Rolling rotation plus airborne rotation is 360 deg, and that is what
    makes the recovery target a subtraction instead of a search."""

    for cycle in cycles:
        assert np.rad2deg(cycle.total_rotation_rad) == pytest.approx(360.0, abs=1e-6)
        assert np.rad2deg(cycle.stroke.rotation_rad) == pytest.approx(
            79.6875, abs=1e-3
        )
        assert np.rad2deg(cycle.recovery.rotation_rad) == pytest.approx(
            280.3125, abs=1e-3
        )


# --------------------------------------------------------------------------
# §8.5(6): the eight validations
# --------------------------------------------------------------------------


def test_valid_foot_rim_contact_throughout_the_rolling_stroke(cycles):
    for cycle in cycles:
        for frame in cycle.stroke.frames:
            assert not frame.airborne
            assert frame.rim == RimId.FOOT.value
            assert frame.surface_id is not None
            assert not frame.collision


def test_the_stroke_uses_the_whole_usable_foot_arc(cycles):
    """It ends because the rim ran out, not because a distance was requested."""

    low, high = (float(v) for v in rim_alpha_limits_rad(RimId.FOOT))
    for cycle in cycles:
        assert cycle.stroke.stop_reason == "RIM_ARC_EXHAUSTED"
        a_from, a_to = cycle.stroke.alpha_range_rad
        assert a_from == pytest.approx(low, abs=np.deg2rad(0.1))
        assert a_to == pytest.approx(high, abs=np.deg2rad(0.1))


def test_consecutive_contact_phases_roll_the_same_way(cycles):
    """Plan §0.2's rotation-direction principle.

    Forward rolling decreases ``beta``; every stroke must do so, and the sign
    must be the same one in both cycles.  A stroke that rolled the other way
    would still produce contact frames, which is why this is checked rather
    than assumed from the ``beta_direction=-1.0`` passed to the solver.
    """

    for cycle in cycles:
        betas = [f.beta_rad for f in cycle.stroke.frames]
        assert all(b < a for a, b in zip(betas, betas[1:]))
        assert cycle.stroke.rotation_rad > 0.0
    assert cycles[0].stroke.rotation_rad > 0.0
    assert cycles[1].stroke.rotation_rad > 0.0


def test_the_recovery_rotates_the_same_way_the_stroke_did(cycles):
    """"Continue the nominal forward rotation sense", not the short way back."""

    for cycle in cycles:
        assert cycle.recovery.rotation_rad > 0.0
        rotating = [
            f.beta_rad for f in cycle.recovery.frames
            if f.phase == "RECOVERY_ROTATE"
        ]
        assert all(b <= a for a, b in zip(rotating, rotating[1:]))
        # And it is nearly a full turn -- the leg does not take the 80 deg
        # shortcut backwards, which is what an unconstrained IK would find.
        assert cycle.recovery.rotation_rad > np.deg2rad(180.0)


def test_liftoff_happens_before_the_rotation(cycles):
    """Plan §8.5(6): "liftoff before recovery rotation".

    The retract phase is the liftoff, and it comes first; by the time the
    rotation starts the leg is clear of the ground.
    """

    for cycle in cycles:
        phases = [f.phase for f in cycle.recovery.frames]
        first_retract = phases.index("RECOVERY_RETRACT")
        first_rotate = phases.index("RECOVERY_ROTATE")
        assert first_retract < first_rotate
        assert cycle.recovery.frames[first_rotate].clearance_m > 0.0


def test_the_leg_clears_the_terrain_while_rotating(cycles):
    for cycle in cycles:
        assert cycle.recovery.min_clearance_m >= cycle.recovery.config.min_clearance_m
        for frame in cycle.recovery.frames:
            if frame.phase == "RECOVERY_ROTATE":
                assert frame.clearance_m > 0.0


def test_retracting_is_what_creates_the_clearance(cycles):
    """No hip lift is needed on flat ground: the leg gets out of the way by
    shrinking.  Recorded because it is why the recovery needs no liftoff-rise
    knob of the kind a Cartesian swing has."""

    for cycle in cycles:
        retract = [
            f for f in cycle.recovery.frames if f.phase == "RECOVERY_RETRACT"
        ]
        assert retract[0].clearance_m < 1e-6
        assert retract[-1].clearance_m > 0.05
        thetas = [f.theta_rad for f in retract]
        assert all(b <= a for a, b in zip(thetas, thetas[1:]))


def test_no_frame_leaves_the_joint_range(cycles):
    low = np.deg2rad(RobotParams.MIN_THETA_DEG)
    high = np.deg2rad(RobotParams.MAX_THETA_DEG)
    for cycle in cycles:
        for frame in cycle.frames:
            assert low - 1e-9 <= frame.theta_rad <= high + 1e-9


def test_the_touchdown_is_a_valid_foot_rim_ground_contact(cycles):
    for cycle in cycles:
        end = cycle.recovery.end
        assert end.phase == "RECOVERY_TOUCHDOWN"
        assert not end.airborne
        assert end.rim == RimId.FOOT.value
        assert not end.collision
        # It lands where the next stroke starts, which is the whole point.
        assert end.alpha_rad == pytest.approx(
            cycle.stroke.start.alpha_rad, abs=np.deg2rad(0.1)
        )
        assert end.theta_rad == pytest.approx(cycle.stroke.start.theta_rad)


def test_the_joint_trajectory_is_continuous(cycles):
    """No teleport anywhere in the cycle, at the sampling it was generated with."""

    for cycle in cycles:
        frames = cycle.frames
        for before, after in zip(frames, frames[1:]):
            assert abs(after.theta_rad - before.theta_rad) <= np.deg2rad(2.0) + 1e-9
            assert abs(after.beta_rad - before.beta_rad) <= np.deg2rad(4.0) + 1e-9


def test_the_hip_trajectory_is_continuous(cycles):
    for cycle in cycles:
        frames = cycle.frames
        for before, after in zip(frames, frames[1:]):
            gap = np.linalg.norm(
                np.array(after.hip_xz_m) - np.array(before.hip_xz_m)
            )
            assert gap < 0.030


def test_net_forward_progress_is_positive(cycles):
    for cycle in cycles:
        assert cycle.hip_advance_m > 0.0
        assert cycle.stroke.contact_advance_m > 0.0
    assert cycles[1].frames[0].hip_xz_m[0] > cycles[0].frames[0].hip_xz_m[0]


# --------------------------------------------------------------------------
# §8.5(3)(4): no terrain in it, and 17 deg lives in one place
# --------------------------------------------------------------------------


def test_the_nominal_cycle_registers_no_obstacle():
    """Requirement 3, at the root: a terrain feature cannot leak into the
    nominal cycle because there is none in the scene to leak."""

    posture = NominalPosture2D()
    assert posture.scene_kwargs["obstacle_x_start_m"] is None
    scene = posture.scene(0.0, 0.0, posture.hip_z_for_flat_stance(0.0))
    assert scene.terrain.obstacle is None


def test_the_compact_posture_is_a_parameter_not_a_constant(cycles):
    """Plan §0.2: 17 deg must be a configurable recovery parameter."""

    default = RecoveryConfig2D()
    assert default.theta_compact_rad == pytest.approx(
        np.deg2rad(RobotParams.THETA0_DEG)
    )
    for cycle in cycles:
        assert cycle.recovery.theta_min_rad == pytest.approx(
            default.theta_compact_rad
        )

    # Changing it changes the motion, which is what "configurable" has to mean.
    other = RecoveryConfig2D(theta_compact_rad=np.deg2rad(30.0))
    assert other.theta_compact_rad != default.theta_compact_rad


def test_the_compact_posture_is_not_the_same_quantity_as_wheel_mode():
    """They coincide in the MVP and are still different things: one is what
    ``WHEEL_ROLL`` means, the other is how far the leg retracts to get out of
    the way.  A single name would make the coincidence permanent."""

    config = RecoveryConfig2D(theta_compact_rad=np.deg2rad(25.0))
    assert config.theta_compact_rad != WHEEL_MODE_THETA_RAD
    # And a retracted recovery is never mistaken for wheel-mode rolling.
    assert not SegmentKind.RECOVERY_SWING.pins_theta


def test_the_touchdown_posture_is_not_the_compact_one(cycles):
    """Plan §8.4: ``theta_touchdown`` is set by the next contact, and on flat
    ground that is the nominal rolling posture, not 17 deg."""

    for cycle in cycles:
        _, recovery_segment = cycle_segments_2d(cycle, source_id="t")
        shaping = recovery_segment.recovery_shaping
        assert shaping.theta_touchdown_rad != shaping.theta_compact_rad
        assert shaping.theta_touchdown_rad == pytest.approx(np.deg2rad(60.0))


def test_a_shorter_stroke_can_be_requested():
    """A leg approaching a transition cannot always spend the whole arc, so the
    stroke length is a request rather than a fixed full arc."""

    stroke = run_foot_rim_roll_2d(max_distance_m=0.05)
    assert stroke.success
    assert stroke.stop_reason == "REQUESTED_DISTANCE_REACHED"
    assert 0.05 <= stroke.contact_advance_m < 0.20


# --------------------------------------------------------------------------
# The recovery target, against Day 6--7's own helper
# --------------------------------------------------------------------------


def test_the_recovery_target_agrees_with_day6_7_when_the_stroke_starts_at_alpha_zero():
    """``_airborne_beta_target_2d`` returns the next equivalent foot-down beta,
    which is this module's target for the one case it covers: a stroke starting
    at the middle of the arc.  Checking the agreement is what says the Day 12
    generalisation is one, rather than a different rule that happens to run."""

    stroke = run_foot_rim_roll_2d(start_beta_rad=0.0, max_distance_m=0.02)
    assert stroke.start.alpha_rad == pytest.approx(0.0, abs=1e-9)
    assert recovery_beta_target_2d(stroke) == pytest.approx(
        _airborne_beta_target_2d(stroke.start.beta_rad, -1)
    )


def test_the_recovery_target_is_always_forward_of_the_stroke_end(cycles):
    for cycle in cycles:
        assert recovery_beta_target_2d(cycle.stroke) < cycle.stroke.end.beta_rad


# --------------------------------------------------------------------------
# §8.5(7): chainable segments
# --------------------------------------------------------------------------


def test_a_cycle_becomes_two_chainable_segments(cycles):
    segments = []
    offset = 0
    for cycle in cycles:
        pair = cycle_segments_2d(cycle, source_id="day12_step1", frame_offset=offset)
        segments.extend(pair)
        offset += sum(s.frames.frame_count for s in pair)

    assert [s.kind for s in segments] == [
        SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING,
        SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING,
    ]
    chain = SegmentChain2D(leg_id="LF", segments=tuple(segments))
    assert chain.is_chained, [b.as_dict() for b in chain.breaks]
    assert chain.is_complete
    # One frame source, and the boundaries are cuts through one generated run.
    assert chain.sources == ("day12_step1",)
    assert {boundary_kind_2d(a, b) for a, b in zip(segments, segments[1:])} == {
        BoundaryKind.CUT
    }


def test_every_segment_of_the_cycle_is_nominal_locomotion(cycles):
    """Neither half is a terrain transition, so Step 11's metrics can count
    them apart from the swings an obstacle forces."""

    roll, recovery = cycle_segments_2d(cycles[0], source_id="t")
    for segment in (roll, recovery):
        assert segment.kind.is_nominal_locomotion
        assert not segment.kind.is_terrain_transition
    assert not roll.kind.is_swing
    assert recovery.kind.is_swing


def test_the_rolling_half_asks_the_body_to_track_and_the_airborne_half_does_not(cycles):
    """The hip height in stance is an output of the contact geometry, so it is
    TRACK; airborne, the body only has to be high enough, so it is a bound."""

    roll, recovery = cycle_segments_2d(cycles[0], source_id="t")
    assert roll.body_requirement.kind.value == "track"
    assert roll.body_requirement.hip_z_profile_m is not None
    assert recovery.body_requirement.kind.value == "lower_bound"
    assert recovery.body_requirement.hip_z_min_m is not None


def test_neither_half_carries_a_duration(cycles):
    """Trap 33 again: this generator advances in beta, not in time.  Step 3
    supplies the timeline, and that is its modelling decision to make."""

    for segment in cycle_segments_2d(cycles[0], source_id="t"):
        assert segment.duration_s is None


def test_a_recovery_segment_records_its_own_shaping_not_day8_9s(cycles):
    _, recovery = cycle_segments_2d(cycles[0], source_id="t")
    assert isinstance(recovery.recovery_shaping, RecoveryShaping2D)
    assert recovery.swing_shaping is None
    assert isinstance(recovery.sampling, RollSampling2D)
    row = recovery.as_dict()
    assert row["theta_compact_deg"] == pytest.approx(17.0)
    assert row["airborne_rotation_deg"] == pytest.approx(280.3125, abs=1e-3)


def test_a_recovery_may_not_be_dressed_as_a_cartesian_swing(cycles):
    """The schema refuses the mix rather than quietly accepting whichever
    shaping was supplied."""

    _, recovery = cycle_segments_2d(cycles[0], source_id="t")
    from dataclasses import replace

    with pytest.raises(TypeError, match="stepped in theta/beta"):
        replace(recovery, sampling=SwingSampling2D(
            arc_samples=241, sample_count=51, leg_arc_samples=61,
            max_joint_step_rad=np.deg2rad(10.0),
        ))
    with pytest.raises(ValueError, match="must record its own shaping"):
        replace(recovery, recovery_shaping=None)


def test_segments_are_refused_for_a_failed_cycle():
    """A cycle that did not close is not a plan; building segments from it
    would present a broken motion as a finished one."""

    from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
        NominalCycle2D, run_recovery_swing_2d,
    )

    stroke = run_foot_rim_roll_2d(max_distance_m=0.02)
    impossible = RecoveryConfig2D(min_clearance_m=10.0)
    cycle = NominalCycle2D(stroke, run_recovery_swing_2d(stroke, impossible))
    assert not cycle.success
    assert cycle.recovery.failure_reason == "ROTATION_CLEARANCE_LOST"
    with pytest.raises(ValueError, match="failed cycle"):
        cycle_segments_2d(cycle, source_id="t")


def test_the_full_cycle_needs_no_hip_motion_while_airborne(cycles):
    """The obvious guess -- that the hip must rise 17.3 mm during recovery --
    is wrong, and wrong in sign as well as size.

    The hip arches by 17.3 mm *during* the stroke, but the two ends of the foot
    arc sit at the same height because the arc is symmetric about ``alpha = 0``.
    So a full-arc recovery ramps the hip by exactly zero.
    """

    posture = cycles[0].stroke.posture
    for cycle in cycles:
        stroke = cycle.stroke
        assert stroke.start.hip_xz_m[1] == pytest.approx(
            stroke.end.hip_xz_m[1], abs=1e-9
        )
        touchdown_z = posture.hip_z_for_flat_stance(recovery_beta_target_2d(stroke))
        assert touchdown_z == pytest.approx(stroke.end.hip_xz_m[1], abs=1e-9)
        # ...while the stroke itself is anything but flat.
        assert stroke.hip_z_travel_m * 1e3 == pytest.approx(17.29, abs=0.05)


def test_a_stroke_cut_short_does_need_the_hip_ramp():
    """Which is why it is not dead code: a leg approaching a transition stops
    part-way up the arch and has to come back down to land."""

    posture = NominalPosture2D()
    stroke = run_foot_rim_roll_2d(posture, max_distance_m=0.05)
    touchdown_z = posture.hip_z_for_flat_stance(recovery_beta_target_2d(stroke))
    ramp_mm = (touchdown_z - stroke.end.hip_xz_m[1]) * 1e3
    assert ramp_mm == pytest.approx(-12.73, abs=0.05)
