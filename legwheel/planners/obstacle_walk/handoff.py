"""Splice the periodic flat Walk into the quasi-static obstacle crawl.

Why a splice is possible at all
-------------------------------
``traversal`` documents that a periodic-Walk segment "cannot be spliced into
this trajectory without a joint velocity step".  That statement holds for an
*arbitrary* sample, and for two reasons:

1. The Walk at ``stance_duty = 0.75`` has **no all-stance sample**.  Its four
   swings tile the cycle exactly, so at every sample precisely one leg carries
   a swing flag, while ``generate_stance_segment`` requires four stance legs.
2. The Walk rolls its wheels through stance while the crawl holds every contact
   world-fixed, so the two stance laws disagree on joint velocity.

Both are avoidable at one specific sample: the **liftoff instant**.  At the
sample where a leg is flagged into swing its wheel has not left the ground yet,
so all four rim contacts are still on the surface, inside the same sub-0.1 mm
band the Walk's own IK leaves behind.  Relabelling that one row as all-stance
is bookkeeping, not a physical claim.  Reason 2 is then handled by
``generate_stance_segment``'s existing quintic-Hermite mode, which starts at a
supplied incoming body velocity and decelerates to rest, so the crawl absorbs
the Walk's motion instead of stepping away from it.

The liftoff sample used here is the one at a cycle boundary, because
``walk_swing_order`` derives the crawl's leg order from the same phase offsets
and therefore names that leg first.  The splice hands over with the crawl's
first swing leg already correct; a mismatch is raised, never reordered.

Starting from rest
------------------
``corgi_csv_control`` plays the 5000 transform rows and then **waits** for the
operator's trigger, so the robot is physically stationary at the trigger row no
matter what the CSV stores there.  A trajectory whose first frame already moves
at the Walk's joint rate therefore demands a velocity step that no prep-ramp
shaping can remove -- the prep is over before the trigger arrives.

The launch fixes that by **re-timing** the Walk rather than replacing it: the
first ``launch_cycles`` periods are replayed against a monotone time warp that
starts at zero rate and reaches exactly nominal rate, with zero second
derivative at both ends.  The joint *path* is the Walk's own, sample for
sample; only the clock differs.  The warp is built in index space so its end
lands on an integer Walk sample, which makes the join exact rather than
interpolated.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import PchipInterpolator

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.launch_controller import LaunchController
from legwheel.planners.obstacle_walk.flat_adapter import (
    _foot_points_world,
    flat_walk_segment_from_generator,
)
from legwheel.planners.obstacle_walk.types import (
    LEG_ORDER,
    LegId,
    SegmentType,
    TrajectorySegment,
    WalkState,
)


class FlatHandoffError(ValueError):
    """The flat Walk cannot be cut at a usable handover sample."""


def cycle_sample_count(generator: GaitGenerator3D) -> int:
    """Samples in one Walk period, as the generator itself discretizes it."""

    count = int(round(generator.T / generator.dt))
    if count < 8:
        raise FlatHandoffError(
            f"one Walk period holds only {count} samples at dt={generator.dt:g} s; "
            "the handover needs a finely sampled cycle"
        )
    return count


def _liftoff_leg_at(phase_row: NDArray[np.int8]) -> LegId:
    """The single leg flagged into swing on ``phase_row``."""

    swinging = np.flatnonzero(np.asarray(phase_row) == 1)
    if len(swinging) != 1:
        raise FlatHandoffError(
            "the handover row must carry exactly one swing flag; it carries "
            f"{len(swinging)}, so this Walk does not hand over one leg at a time"
        )
    return LEG_ORDER[int(swinging[0])]


def handover_leg_at(
    phase: NDArray[np.int8], row: int, samples_per_cycle: int
) -> tuple[LegId, bool]:
    """``(leg the Walk swings next, whether the row needs relabelling)``.

    Which rows can carry a handover depends on the duty factor, and the two
    cases are physically different:

    * ``stance_duty = 0.75`` tiles the four swings across the whole cycle, so
      **no** sample has four legs down.  The best available row is a liftoff
      instant, where the departing wheel has not left the ground yet; that row
      is relabelled all-stance, which is bookkeeping rather than a physical
      claim.
    * ``stance_duty = 0.85`` leaves a genuine four-leg overlap between swings.
      A row inside it needs no relabelling at all -- the Walk really does have
      all four feet down there, which is exactly what the crawl's world-fixed
      stance assumes.

    Returning the flag rather than always relabelling keeps the honest case
    honest instead of hiding it behind the same fixup.
    """

    row_phase = np.asarray(phase[row])
    swinging = np.flatnonzero(row_phase == 1)
    if len(swinging) == 1:
        return LEG_ORDER[int(swinging[0])], True
    if len(swinging) == 0:
        for step in range(1, samples_per_cycle + 1):
            here = row + step
            if here >= len(phase):
                break
            starts = np.flatnonzero((phase[here - 1] == 0) & (phase[here] == 1))
            if len(starts):
                return LEG_ORDER[int(starts[0])], False
        raise FlatHandoffError(
            f"no leg lifts within one cycle after the all-stance handover row {row}"
        )
    raise FlatHandoffError(
        f"handover row {row} carries {len(swinging)} swing flags; this Walk does not "
        "hand over one leg at a time"
    )


def launch_sample_index(warp_samples: int) -> NDArray[np.float64]:
    """Walk-sample index as a function of real sample index during the launch.

    ``warp_samples`` real samples consume ``warp_samples / 2`` Walk samples, so
    the returned index runs ``0 -> warp_samples / 2`` while its slope runs
    ``0 -> 1``.  The slope is the cubic smoothstep, whose integral normalised to
    unit end value is ``P(s) = 2 s^3 - s^4``; ``P'(1) = 2`` is exactly the
    factor that makes the final slope nominal, and ``P''(0) = P''(1) = 0`` makes
    both ends free of a joint-acceleration step.
    """

    if warp_samples % 2 != 0 or warp_samples < 2:
        raise FlatHandoffError("warp_samples must be a positive even number of samples.")
    half = warp_samples // 2
    s = np.arange(warp_samples + 1, dtype=float) / float(warp_samples)
    index = half * (2.0 * s**3 - s**4)
    # The join must land on an integer Walk sample so the shared boundary is a
    # knot rather than an interpolated value.
    index[-1] = float(half)
    index[0] = 0.0
    return index


def landing_sample_index(warp_samples: int) -> NDArray[np.float64]:
    """Mirror of :func:`launch_sample_index`: nominal rate down to rest.

    Used at the end of the file so the trajectory stops the way the pure crawl
    always did, instead of ending mid-stride at full joint rate.
    """

    if warp_samples % 2 != 0 or warp_samples < 2:
        raise FlatHandoffError("warp_samples must be a positive even number of samples.")
    half = warp_samples // 2
    s = np.arange(warp_samples + 1, dtype=float) / float(warp_samples)
    reverse = 1.0 - s
    index = half * (1.0 - (2.0 * reverse**3 - reverse**4))
    index[0] = 0.0
    index[-1] = float(half)
    return index


def _warped_index_grid(
    span: int, launch_samples: int = 0, landing_samples: int = 0
) -> NDArray[np.float64]:
    """Walk-sample index for every real sample of a re-timed Walk stretch.

    ``span`` Walk samples are traversed.  The first ``launch_samples`` real
    samples ramp the rate up from rest and the last ``landing_samples`` ramp it
    back down; whatever lies between runs at nominal rate.  Each warp consumes
    half its real samples' worth of Walk, which is why both ends land on integer
    Walk samples and the joins stay exact.
    """

    launch_half = launch_samples // 2
    landing_half = landing_samples // 2
    if launch_half + landing_half > span:
        raise FlatHandoffError(
            f"the launch and landing warps would consume {launch_half + landing_half} "
            f"of the {span} Walk samples available; use fewer warp cycles or a longer "
            "flat stretch"
        )
    pieces: list[NDArray[np.float64]] = []
    if launch_samples:
        pieces.append(launch_sample_index(launch_samples))
    else:
        pieces.append(np.zeros(1, dtype=float))
    nominal_end = span - landing_half
    pieces.append(np.arange(launch_half + 1, nominal_end + 1, dtype=float))
    if landing_samples:
        # Drop the first entry: it repeats the nominal stretch's final sample.
        pieces.append(nominal_end + landing_sample_index(landing_samples)[1:])
    return np.concatenate(pieces)


def _name_final_next_swing_leg(
    segment: TrajectorySegment, next_swing_leg: LegId
) -> TrajectorySegment:
    """Record the crawl's first swing leg on an already-all-stance final row."""

    if np.any(segment.phase[-1] != 0):
        raise FlatHandoffError("the final row is not four-leg stance.")
    final_state = WalkState(
        joint_position_rad=segment.commands_rad[-1],
        previous_joint_position_rad=segment.commands_rad[-2],
        body_pose_world=segment.body_pose_world[-1],
        foot_contact_points_world_m=segment.foot_contact_points_world_m[-1],
        phase=segment.phase[-1],
        contact_active=segment.contact_active[-1],
        surface_ids=segment.surface_ids[-1],
        gait_cycle_phase=float(segment.gait_cycle_phase[-1]),
        next_swing_leg=next_swing_leg,
    )
    return TrajectorySegment(
        time_s=segment.time_s,
        commands_rad=segment.commands_rad,
        phase=segment.phase,
        body_pose_world=segment.body_pose_world,
        foot_contact_points_world_m=segment.foot_contact_points_world_m,
        contact_active=segment.contact_active,
        gait_cycle_phase=segment.gait_cycle_phase,
        surface_ids=segment.surface_ids,
        start_state=segment.start_state,
        final_state=final_state,
        dt_s=segment.dt_s,
        segment_type=SegmentType.FLAT,
        command_order=segment.command_order,
    )


def _relabel_final_row_all_stance(
    segment: TrajectorySegment, next_swing_leg: LegId
) -> TrajectorySegment:
    """Return ``segment`` with its last row re-flagged as four-leg stance.

    Only the phase bookkeeping changes.  Commands, body pose and rim contact
    points are copied through untouched, so the relabelled row still is the
    Walk's own liftoff sample.
    """

    phase = segment.phase.copy()
    phase[-1] = 0
    active = phase == 0
    final_state = WalkState(
        joint_position_rad=segment.commands_rad[-1],
        previous_joint_position_rad=segment.commands_rad[-2],
        body_pose_world=segment.body_pose_world[-1],
        foot_contact_points_world_m=segment.foot_contact_points_world_m[-1],
        phase=phase[-1],
        contact_active=active[-1],
        surface_ids=segment.surface_ids[-1],
        gait_cycle_phase=float(segment.gait_cycle_phase[-1]),
        next_swing_leg=next_swing_leg,
    )
    return TrajectorySegment(
        time_s=segment.time_s,
        commands_rad=segment.commands_rad,
        phase=phase,
        body_pose_world=segment.body_pose_world,
        foot_contact_points_world_m=segment.foot_contact_points_world_m,
        contact_active=active,
        gait_cycle_phase=segment.gait_cycle_phase,
        surface_ids=segment.surface_ids,
        start_state=segment.start_state,
        final_state=final_state,
        dt_s=segment.dt_s,
        segment_type=SegmentType.FLAT,
        command_order=segment.command_order,
    )


def _resample_walk(
    generator: GaitGenerator3D,
    full: TrajectorySegment,
    index_grid: NDArray[np.float64],
    ground_surface_id: str,
) -> TrajectorySegment:
    """Evaluate the Walk at fractional sample indices, keeping knots exact.

    PCHIP is used for the same reason the exporter uses it: it reproduces every
    knot exactly and is shape preserving, so the launch's final sample -- which
    sits on an integer index by construction -- is the Walk's own sample rather
    than an interpolated approximation.  Foot points are recomputed by forward
    kinematics from the resampled commands, never interpolated, so they stay
    consistent with the joints that produced them.
    """

    knots = np.arange(full.sample_count, dtype=float)
    commands = np.asarray(
        PchipInterpolator(knots, full.commands_rad, axis=0)(index_grid), dtype=float
    )
    body_pose = np.asarray(
        PchipInterpolator(knots, full.body_pose_world, axis=0)(index_grid), dtype=float
    )
    # A phase is a discrete interval label; the sample the warp currently sits
    # on owns it, which keeps liftoff and touchdown rows aligned with the joints.
    phase = full.phase[np.floor(index_grid + 1e-9).astype(int)]
    active = phase == 0
    foot_points = _foot_points_world(generator, commands, body_pose)
    count = len(index_grid)
    surfaces = tuple((ground_surface_id,) * 4 for _ in range(count))
    samples_per_cycle = cycle_sample_count(generator)
    gait_cycle_phase = (index_grid % samples_per_cycle) / samples_per_cycle

    def state_at(index: int, previous: int | None) -> WalkState:
        return WalkState(
            joint_position_rad=commands[index],
            previous_joint_position_rad=None if previous is None else commands[previous],
            body_pose_world=body_pose[index],
            foot_contact_points_world_m=foot_points[index],
            phase=phase[index],
            contact_active=active[index],
            surface_ids=surfaces[index],
            gait_cycle_phase=float(gait_cycle_phase[index]),
            next_swing_leg=None,
        )

    return TrajectorySegment(
        time_s=np.arange(count, dtype=float) * generator.dt,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_pose,
        foot_contact_points_world_m=foot_points,
        contact_active=active,
        gait_cycle_phase=gait_cycle_phase,
        surface_ids=surfaces,
        start_state=state_at(0, None),
        final_state=state_at(count - 1, count - 2),
        dt_s=generator.dt,
        segment_type=SegmentType.FLAT,
    )


def flat_approach_segment(
    generator: GaitGenerator3D,
    *,
    cycles: int,
    launch_cycles: int,
    handover_body_x_m: float,
    body_y_m: float,
    stand_height_m: float,
    ground_height_m: float,
    ground_surface_id: str,
    expected_first_swing_leg: LegId,
) -> TrajectorySegment:
    """``cycles`` periods of the existing flat Walk, ending at a liftoff sample.

    The segment is positioned so its **last** sample sits at
    ``handover_body_x_m``; the Walk therefore starts ``cycles * v_x * T`` metres
    further back and the crawl still begins exactly where it did before.  The
    final row is relabelled all-stance so the crawl's first world-fixed stance
    can take it as a start state.

    ``launch_cycles`` of those periods are replayed under the starting time warp
    so the trajectory leaves the trigger row at rest.  The warp changes only the
    clock, so the body still advances ``cycles * v_x * T`` in total; it just
    takes ``launch_cycles * T`` seconds longer to do it.
    """

    if not isinstance(cycles, int) or isinstance(cycles, bool) or cycles <= 0:
        raise FlatHandoffError("cycles must be a positive integer.")
    if not isinstance(launch_cycles, int) or isinstance(launch_cycles, bool):
        raise FlatHandoffError("launch_cycles must be an integer.")
    if not 0 <= launch_cycles <= cycles:
        raise FlatHandoffError(
            f"launch_cycles must lie in [0, cycles]; got {launch_cycles} with "
            f"cycles={cycles}"
        )
    samples_per_cycle = cycle_sample_count(generator)
    handover_index = cycles * samples_per_cycle

    velocity_x = float(generator.v_com[0])
    start_x = handover_body_x_m - velocity_x * cycles * generator.T
    initial_pose = np.array(
        [start_x, float(body_y_m), ground_height_m + stand_height_m, 0.0, 0.0, 0.0]
    )

    # One extra cycle so the cut sample itself exists: n_cycles=cycles yields
    # indices 0..cycles*samples_per_cycle - 1, which stops one short.
    generator.generate_full_gait(n_cycles=cycles + 1)
    full = flat_walk_segment_from_generator(
        generator,
        initial_body_pose_world=initial_pose,
        ground_surface_id=ground_surface_id,
    )
    if full.sample_count <= handover_index:
        raise FlatHandoffError(
            f"the flat Walk produced {full.sample_count} samples, which does not "
            f"reach the handover sample {handover_index}"
        )

    handover_leg, needs_relabel = handover_leg_at(
        full.phase, handover_index, samples_per_cycle
    )
    if handover_leg is not expected_first_swing_leg:
        raise FlatHandoffError(
            f"the flat Walk swings {handover_leg.value} next at the cycle boundary but "
            f"the crawl's leg order starts with {expected_first_swing_leg.value}; the "
            "handover would reorder the gait"
        )

    grid = _warped_index_grid(
        handover_index, launch_samples=2 * launch_cycles * samples_per_cycle
    )
    walked = _resample_walk(generator, full, grid, ground_surface_id)
    if not needs_relabel:
        # A genuine four-leg overlap: nothing to relabel, only the successor leg
        # to record for the crawl.
        return _name_final_next_swing_leg(walked, expected_first_swing_leg)
    return _relabel_final_row_all_stance(walked, expected_first_swing_leg)


def legacy_launch_flat_approach_segment(
    generator: GaitGenerator3D,
    *,
    steady_cycles: int,
    launch_cycles: int,
    ramp_floor: float,
    handover_body_x_m: float,
    body_y_m: float,
    stand_height_m: float,
    ground_height_m: float,
    ground_surface_id: str,
    expected_first_swing_leg: LegId,
) -> TrajectorySegment:
    """Use the repository's original LaunchController + steady Walk pipeline.

    Unlike :func:`flat_approach_segment`, this function does not time-warp a
    full-speed Walk.  It calls the same ``LaunchController`` used by
    ``examples/gait/generate_hardware_csv.py`` and then appends the existing
    generator's steady Walk commands.  Only one extra cycle-boundary row is
    appended so the obstacle crawl has an exact liftoff state to inherit.
    """

    if steady_cycles < 0 or launch_cycles <= 0:
        raise FlatHandoffError(
            "legacy flat approach requires non-negative steady_cycles and "
            "positive launch_cycles"
        )
    if not 0.0 < ramp_floor <= 1.0:
        raise FlatHandoffError("ramp_floor must lie in (0, 1].")
    samples_per_cycle = cycle_sample_count(generator)

    launcher = LaunchController(
        gait_type="Walk",
        stand_height=stand_height_m,
        twist=generator.twist,
        step_height=generator.step_height,
        period=generator.T,
        dt=generator.dt,
        n_ramp=launch_cycles,
        ramp_floor=ramp_floor,
        stability_margin=0.0,
        stance_duty=generator.stance_duty,
    )
    launch_commands_flat, launch_phase = launcher._generate_launch_sequence_with_phase()

    # Generate one extra steady cycle so the exact next cycle-boundary sample
    # exists.  The original hardware pipeline ends one row before that sample;
    # the added row is identical to steady row 0 and is shared with the crawl.
    generator.generate_full_gait(n_cycles=max(steady_cycles, 1) + 1)
    steady_count = steady_cycles * samples_per_cycle
    steady_commands_flat = np.asarray(generator.CMDS[: steady_count + 1], dtype=float)
    steady_phase = np.asarray(generator.PHASE[: steady_count + 1], dtype=np.int8)
    commands_flat = np.vstack([launch_commands_flat, steady_commands_flat])
    phase = np.vstack([launch_phase.astype(np.int8), steady_phase])

    boundary_leg = _liftoff_leg_at(phase[-1])
    if boundary_leg is not expected_first_swing_leg:
        raise FlatHandoffError(
            f"legacy Walk boundary lifts {boundary_leg.value}, but crawl starts with "
            f"{expected_first_swing_leg.value}"
        )

    commands = commands_flat.reshape(-1, 4, 3)
    count = len(commands)
    scales = np.linspace(ramp_floor, 1.0, launch_cycles)
    velocity = np.concatenate(
        [
            np.repeat(float(generator.v_com[0]) * scales, samples_per_cycle),
            np.full(steady_count + 1, float(generator.v_com[0])),
        ]
    )
    distance = np.zeros(count, dtype=float)
    distance[1:] = np.cumsum(velocity[:-1]) * generator.dt
    body_pose = np.zeros((count, 6), dtype=float)
    body_pose[:, 0] = float(handover_body_x_m) - distance[-1] + distance
    body_pose[:, 1] = float(body_y_m)
    body_pose[:, 2] = float(ground_height_m + stand_height_m)
    foot_points = _foot_points_world(generator, commands, body_pose)

    # The boundary is the Walk's own liftoff pose.  Only its bookkeeping is
    # changed to all-stance, exactly as in the previous handoff contract.
    phase[-1] = 0
    active = phase == 0
    surfaces = tuple((ground_surface_id,) * 4 for _ in range(count))
    gait_cycle_phase = (np.arange(count) % samples_per_cycle) / samples_per_cycle

    def state_at(index: int, previous: int | None, next_leg: LegId | None) -> WalkState:
        return WalkState(
            joint_position_rad=commands[index],
            previous_joint_position_rad=(
                None if previous is None else commands[previous]
            ),
            body_pose_world=body_pose[index],
            foot_contact_points_world_m=foot_points[index],
            phase=phase[index],
            contact_active=active[index],
            surface_ids=surfaces[index],
            gait_cycle_phase=float(gait_cycle_phase[index]),
            next_swing_leg=next_leg,
        )

    return TrajectorySegment(
        time_s=np.arange(count, dtype=float) * generator.dt,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_pose,
        foot_contact_points_world_m=foot_points,
        contact_active=active,
        gait_cycle_phase=gait_cycle_phase,
        surface_ids=surfaces,
        start_state=state_at(0, None, None),
        final_state=state_at(count - 1, count - 2, expected_first_swing_leg),
        dt_s=generator.dt,
        segment_type=SegmentType.FLAT,
    )


def handover_contact_height_error_m(
    segment: TrajectorySegment, ground_height_m: float
) -> float:
    """Largest ``|z - ground|`` among the four rim contacts at the handover row.

    The crawl's stance rejects a start contact that misses its surface by more
    than ``contact_height_tolerance_m``, so this is the quantity that decides
    whether a given Walk discretization can hand over at all.
    """

    heights = np.asarray(segment.foot_contact_points_world_m[-1, :, 2], dtype=float)
    return float(np.max(np.abs(heights - float(ground_height_m))))


def _entry_row_for(
    phase: NDArray[np.int8], leg: LegId, samples_per_cycle: int
) -> int:
    """Row the recovery Walk is entered on so that ``leg`` swings next.

    At a duty with a genuine four-leg overlap the entry moves back to the start
    of the all-stance block before that leg lifts, which hands the crawl a real
    four-leg support to settle into instead of a relabelled liftoff instant.
    """

    liftoff = _liftoff_row(phase, leg, samples_per_cycle)
    row = liftoff
    while row - 1 >= 0 and not np.any(phase[row - 1] == 1):
        row -= 1
    return row


def _relabel_first_row_all_stance(segment: TrajectorySegment) -> TrajectorySegment:
    """Mirror of ``_relabel_final_row_all_stance`` for the segment's first row."""

    if not np.any(np.asarray(segment.phase[0]) == 1):
        return segment  # already a genuine four-leg overlap
    liftoff_leg = _liftoff_leg_at(segment.phase[0])
    phase = segment.phase.copy()
    phase[0] = 0
    active = phase == 0
    start_state = WalkState(
        joint_position_rad=segment.commands_rad[0],
        previous_joint_position_rad=None,
        body_pose_world=segment.body_pose_world[0],
        foot_contact_points_world_m=segment.foot_contact_points_world_m[0],
        phase=phase[0],
        contact_active=active[0],
        surface_ids=segment.surface_ids[0],
        gait_cycle_phase=float(segment.gait_cycle_phase[0]),
        next_swing_leg=liftoff_leg,
    )
    return TrajectorySegment(
        time_s=segment.time_s,
        commands_rad=segment.commands_rad,
        phase=phase,
        body_pose_world=segment.body_pose_world,
        foot_contact_points_world_m=segment.foot_contact_points_world_m,
        contact_active=active,
        gait_cycle_phase=segment.gait_cycle_phase,
        surface_ids=segment.surface_ids,
        start_state=start_state,
        final_state=segment.final_state,
        dt_s=segment.dt_s,
        segment_type=SegmentType.FLAT,
        command_order=segment.command_order,
    )


def _liftoff_row(phase: NDArray[np.int8], leg: LegId, samples_per_cycle: int) -> int:
    """First row inside one cycle where ``leg`` is flagged into swing.

    The crawl hands the gait back at whichever liftoff belongs to the leg it was
    about to swing, so the recovery Walk is entered at that leg's own row rather
    than always at the cycle origin.  That removes any need to pad the crawl
    with filler events just to line the leg order up.
    """

    index = LEG_ORDER.index(leg)
    for row in range(1, samples_per_cycle + 1):
        if phase[row - 1, index] == 0 and phase[row, index] == 1:
            return row
    raise FlatHandoffError(
        f"the Walk cycle contains no liftoff row for {leg.value}; its phase offsets "
        "do not give one swing per leg per cycle"
    )


def flat_recovery_segment(
    generator: GaitGenerator3D,
    *,
    cycles: int,
    launch_cycles: int,
    landing_cycles: int,
    start_body_x_m: float,
    body_y_m: float,
    stand_height_m: float,
    ground_height_m: float,
    ground_surface_id: str,
    first_swing_leg: LegId,
) -> TrajectorySegment:
    """``cycles`` periods of the flat Walk, entered after the obstacle.

    Mirror of :func:`flat_approach_segment`.  It begins at ``first_swing_leg``'s
    own liftoff row -- relabelled all-stance, so the crawl's final stance can
    meet it -- and is re-timed at both ends: up from rest, because the crawl
    exits at rest, and back down to rest, because the file ends here and the
    pure-crawl trajectory always finished stationary.
    """

    if not isinstance(cycles, int) or isinstance(cycles, bool) or cycles <= 0:
        raise FlatHandoffError("cycles must be a positive integer.")
    samples_per_cycle = cycle_sample_count(generator)
    generator.generate_full_gait(n_cycles=cycles + 2)
    probe = flat_walk_segment_from_generator(
        generator, ground_surface_id=ground_surface_id
    )
    entry_row = _entry_row_for(probe.phase, first_swing_leg, samples_per_cycle)

    velocity_x = float(generator.v_com[0])
    initial_pose = np.array(
        [
            start_body_x_m - velocity_x * entry_row * generator.dt,
            float(body_y_m),
            ground_height_m + stand_height_m,
            0.0,
            0.0,
            0.0,
        ]
    )
    full = flat_walk_segment_from_generator(
        generator,
        initial_body_pose_world=initial_pose,
        ground_surface_id=ground_surface_id,
    )
    span = cycles * samples_per_cycle
    if full.sample_count <= entry_row + span:
        raise FlatHandoffError(
            f"the flat Walk produced {full.sample_count} samples, which does not reach "
            f"the recovery exit sample {entry_row + span}"
        )
    grid = entry_row + _warped_index_grid(
        span,
        launch_samples=2 * launch_cycles * samples_per_cycle,
        landing_samples=2 * landing_cycles * samples_per_cycle,
    )
    walked = _resample_walk(generator, full, grid, ground_surface_id)
    return _relabel_first_row_all_stance(walked)


def blend_final_row_to(
    generator: GaitGenerator3D,
    segment: TrajectorySegment,
    target: WalkState,
) -> tuple[TrajectorySegment, float, float]:
    """Settle a stance onto ``target``'s exact joints over its whole length.

    The crawl and the periodic Walk are generated independently, so the crawl's
    exit into the recovery Walk is the one boundary in the trajectory where two
    separately solved poses have to agree; everywhere else a segment copies the
    previous segment's final state verbatim, which is why the boundary joint
    error is otherwise exactly zero.

    The residual is not small.  The swing places a foot to within
    ``tracking_tolerance_m`` -- a millimetre -- and a millimetre of foot error is
    worth about a hundredth of a radian at the joints, so forcing it onto the
    last row alone would jump the standing feet by that millimetre in a single
    sample.  Instead the correction is distributed across the segment by a cubic
    smoothstep: zero at the first row, so continuity with the crawl stays exact
    in both position and velocity; one at the last row, so the Walk is entered on
    its own joints; and zero slope at both ends, so neither boundary gains a
    velocity step.

    What the blend spends is contact drift -- the standing feet slide by the
    reconciliation distance over the settle -- which is the same budget the
    world-fixed stance already accounts for.  Both the joint correction and the
    resulting drift are returned so the caller can bound and publish them.
    """

    commands = np.array(segment.commands_rad, dtype=float, copy=True)
    delta = np.asarray(target.joint_position_rad, dtype=float) - commands[-1]
    joint_correction = float(np.max(np.abs(delta)))
    count = segment.sample_count
    progress = np.arange(count, dtype=float) / float(count - 1)
    blend = 3.0 * progress**2 - 2.0 * progress**3
    commands += blend[:, None, None] * delta[None, :, :]
    # Snap out the smoothstep's own rounding so the shared row is bit-exact.
    commands[-1] = target.joint_position_rad
    commands[0] = segment.commands_rad[0]

    # Contacts follow the joints that were actually commanded, so the last row's
    # points come out as the Walk's own f* = p* + FK(q*) without a second snap.
    contacts = _foot_points_world(generator, commands, segment.body_pose_world)
    contacts[0] = segment.foot_contact_points_world_m[0]
    contacts[-1] = target.foot_contact_points_world_m
    settle_drift = float(
        np.max(
            np.linalg.norm(
                contacts - np.asarray(segment.foot_contact_points_world_m[0])[None],
                axis=2,
            )[:, np.asarray(segment.contact_active[0], dtype=bool)]
        )
    )

    final_state = WalkState(
        joint_position_rad=commands[-1],
        previous_joint_position_rad=commands[-2],
        body_pose_world=segment.body_pose_world[-1],
        foot_contact_points_world_m=contacts[-1],
        phase=segment.phase[-1],
        contact_active=segment.contact_active[-1],
        surface_ids=segment.surface_ids[-1],
        gait_cycle_phase=float(segment.gait_cycle_phase[-1]),
        next_swing_leg=target.next_swing_leg,
    )
    settled = TrajectorySegment(
        time_s=segment.time_s,
        commands_rad=commands,
        phase=segment.phase,
        body_pose_world=segment.body_pose_world,
        foot_contact_points_world_m=contacts,
        contact_active=segment.contact_active,
        gait_cycle_phase=segment.gait_cycle_phase,
        surface_ids=segment.surface_ids,
        start_state=segment.start_state,
        final_state=final_state,
        dt_s=segment.dt_s,
        segment_type=segment.segment_type,
        swing_leg=segment.swing_leg,
        command_order=segment.command_order,
    )
    return settled, joint_correction, settle_drift


def resample_segment(segment: TrajectorySegment, target_dt_s: float) -> TrajectorySegment:
    """Re-sample one segment onto a finer uniform grid, keeping its knots exact.

    The flat sections must be the Walk's *own* 1 kHz samples, because that is
    what the flat-Walk hardware runs play and what the operator compares
    against.  The crawl cannot be planned at 1 ms -- its lowest-rim solve
    changes branch there -- so the two halves are planned on different clocks
    and the coarse one is brought up here, before assembly, instead of the
    exporter resampling everything afterwards.

    PCHIP reproduces every original knot exactly, so the first and last rows
    are untouched and the boundaries the crawl shares with the flat Walk stay
    exact.  Phase is a discrete interval label and is repeated, never
    interpolated; ``gait_cycle_phase`` is regenerated rather than interpolated
    because it wraps, and interpolating across a wrap would run the cycle
    backwards through the middle of the segment.
    """

    source_dt = float(segment.dt_s)
    ratio_float = source_dt / float(target_dt_s)
    ratio = int(round(ratio_float))
    if ratio < 1 or not np.isclose(ratio_float, ratio, rtol=0.0, atol=1e-9):
        raise FlatHandoffError(
            f"segment dt {source_dt:g} s is not an integer multiple of the target "
            f"{target_dt_s:g} s"
        )
    if ratio == 1:
        return segment

    knots = np.arange(segment.sample_count, dtype=float)
    count = (segment.sample_count - 1) * ratio + 1
    grid = np.arange(count, dtype=float) / ratio
    grid[-1] = knots[-1]

    commands = np.asarray(
        PchipInterpolator(knots, segment.commands_rad, axis=0)(grid), dtype=float
    )
    body_pose = np.asarray(
        PchipInterpolator(knots, segment.body_pose_world, axis=0)(grid), dtype=float
    )
    foot_points = np.asarray(
        PchipInterpolator(knots, segment.foot_contact_points_world_m, axis=0)(grid),
        dtype=float,
    )
    # Exactness at the shared rows matters more than interpolation: force the
    # endpoints back onto the values the neighbouring segments were built from.
    for array, source in (
        (commands, segment.commands_rad),
        (body_pose, segment.body_pose_world),
        (foot_points, segment.foot_contact_points_world_m),
    ):
        array[0] = source[0]
        array[-1] = source[-1]

    phase = np.vstack(
        [np.repeat(segment.phase[:-1], ratio, axis=0), segment.phase[-1:]]
    ).astype(np.int8, copy=False)
    active = phase == 0
    surfaces = tuple(
        [row for row in segment.surface_ids[:-1] for _ in range(ratio)]
        + [segment.surface_ids[-1]]
    )
    period_samples = max(len(segment.gait_cycle_phase) - 1, 1) * source_dt
    del period_samples
    start_phase = float(segment.gait_cycle_phase[0])
    span = float(segment.gait_cycle_phase[-1]) - start_phase
    # The crawl advances its bookkeeping phase linearly, so regenerating it from
    # the endpoints reproduces it without ever crossing a wrap incorrectly.
    gait_cycle_phase = (start_phase + span * (grid / knots[-1])) % 1.0

    def state_at(index: int, previous: int | None) -> WalkState:
        return WalkState(
            joint_position_rad=commands[index],
            previous_joint_position_rad=None if previous is None else commands[previous],
            body_pose_world=body_pose[index],
            foot_contact_points_world_m=foot_points[index],
            phase=phase[index],
            contact_active=active[index],
            surface_ids=surfaces[index],
            gait_cycle_phase=float(gait_cycle_phase[index]),
            next_swing_leg=(
                segment.start_state.next_swing_leg
                if index == 0
                else segment.final_state.next_swing_leg
            ),
        )

    return TrajectorySegment(
        time_s=np.arange(count, dtype=float) * float(target_dt_s),
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_pose,
        foot_contact_points_world_m=foot_points,
        contact_active=active,
        gait_cycle_phase=gait_cycle_phase,
        surface_ids=surfaces,
        start_state=state_at(0, None),
        final_state=state_at(count - 1, count - 2),
        dt_s=float(target_dt_s),
        segment_type=segment.segment_type,
        swing_leg=segment.swing_leg,
        command_order=segment.command_order,
    )
