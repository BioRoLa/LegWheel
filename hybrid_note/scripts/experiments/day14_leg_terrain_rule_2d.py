"""Day 14 Step 3: what one leg does next, from where its contact actually is.

This is the per-leg terrain rule the gait-first planner consults.  It never
sees the other three legs and does not need to: the planner serialises the
liftoffs.  What the rule decides is, for *this* leg, standing where it stands:

* how far the next stroke may roll (``StanceAction2D``), and
* where the next swing lands and on which surface (``SwingAction2D``).

The rule of thumb is **lift as late as the terrain allows**.  A stroke on the
ground before the obstacle is generated with the obstacle in its scene, so it
stops on its own when the next roll step would put the leg into the face
(``OBSTACLE_AHEAD``); that frame *is* the latest legal takeoff, measured rather
than tabulated.  A stroke on the top is asked for exactly the contact distance
left before the trailing edge.  Landings are searched from the earliest
position that puts the contact on the target surface, outward, until the
transition generator accepts one -- so every landing in the plan is one the
terrain-aware swing actually flew.

Why the two legs of a pair come out staggered
---------------------------------------------

Same ``mount_x``, so the same hip x at every instant -- but not the same
contact.  Half a cycle apart in phase, one leg's contact leads the other's by
roughly 60--100 mm, so their standing poses stop being legal at *different*
hip positions and they take off at different body positions (measured on the
40 mm platform: the mid-arc leg at ~100 mm before the face, the arc-end leg at
~40 mm).  That stagger, not any scheduling, is what lets the planner put them
in the air one at a time.  It also sets the limit: the second leg must still
be standing legally when the first has landed on the top, and the first can
land no earlier than one contact lead (61.7 mm) before the face.  At 40 mm the
standing limit is ~100 mm behind the face and the two fit; at 100 mm it is
~180 mm and they do not -- the pair then needs one leg to *roll* up instead
(Step 4), because a rolling leg is meant to touch the face.

Body height in this step
------------------------

Constant.  The axle stays at the flat gait's held height and a leg standing
on the top crouches by the obstacle height (theta ~47 deg at 40 mm).  That is
the simplest policy that keeps all four hips on one plane without a body
profile, and it is only available while the top is lower than the crouch can
absorb (about 75 mm: the compact leg is 144 mm long).  Taller obstacles raise
the axle instead, which is Step 4's business.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LegId
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    THETA_MAX_RAD,
    CycleFrame2D,
    HipZProfile2D,
    NominalPosture2D,
    RecoveryConfig2D,
    RollStroke2D,
    _translate_frame,
    nominal_stroke_2d,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    TransitionPhase,
)
from hybrid_note.scripts.experiments.day13_b5_nominal_ascent_2d import (
    obstacle_posture_2d,
)
from hybrid_note.scripts.experiments.day14_rolling_crossing_2d import (
    APPROACH_WINDOW_M,
    ROLL_UP_STOP_REASON,
    roll_up_landing_beta_rad,
    roll_up_stroke_2d,
)
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
    DEFAULT_LANDING_EDGE_MARGIN_M,
    SEGMENT_KIND_OF_TRANSITION,
    NominalTransition2D,
    TransitionKind2D,
    run_nominal_transition_2d,
)
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
    LegProgress2D,
    PlannerRefusal2D,
    StanceAction2D,
    SwingAction2D,
)

__all__ = [
    "Surface2D",
    "AxleProfiles2D",
    "SwingSwingRule2D",
    "LegTerrainState2D",
    "terrain_axle_profiles_2d",
    "axle_profiles_from_events_2d",
    "plan_swing_swing_crossing_2d",
    "ideal_origins_2d",
    "origin_for_arc_end_contact_2d",
    "origin_for_landing_hip_2d",
]


# --------------------------------------------------------------------------
# The axle heights
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class AxleProfiles2D:
    """Absolute hip height of each axle, as a function of **hip x**.

    Both legs of a pair hang from one axle, so they share one profile; the
    front and rear axles have their own because they cross at different
    times.  A leg standing on the ground reaches down from the profile, a leg
    on the top reaches down less -- the profile is the body's, not the leg's.
    """

    front: HipZProfile2D
    rear: HipZProfile2D

    def for_leg(self, leg: LegId) -> HipZProfile2D:
        return self.front if leg.is_front else self.rear

    @classmethod
    def constant(cls, hip_z_m: float) -> "AxleProfiles2D":
        return cls(HipZProfile2D.constant(hip_z_m), HipZProfile2D.constant(hip_z_m))

    def as_rows(self) -> list[dict]:
        rows = []
        for name, profile in (("front", self.front), ("rear", self.rear)):
            for x, z in zip(profile.hip_x_m, profile.hip_z_m):
                rows.append({"row_kind": "axle_profile", "axle": name,
                             "hip_x_mm": x * 1e3, "hip_z_mm": z * 1e3})
        return rows


def _ramp_profile(hold_m: float, height_m: float, up: tuple[float, float],
                  down: tuple[float, float]) -> HipZProfile2D:
    """Hold, rise by ``height_m`` over ``up``, hold, fall over ``down``, hold."""

    (u0, u1), (d0, d1) = up, down
    if not (u0 < u1 <= d0 < d1):
        raise ValueError(f"ramps must be ordered: up {up}, down {down}")
    return HipZProfile2D(
        (u0, u1, d0, d1),
        (hold_m, hold_m + height_m, hold_m + height_m, hold_m))


def axle_rise_m(spec: SharedTerrainSpec2D, posture: NominalPosture2D, *,
                margin_m: float = 0.005) -> float:
    """How far the axle rises for this block: the block's height, unless the
    leg still on the ground at its arc end cannot reach that high (measured:
    331.5 mm at the arc-end beta with theta 160 deg, so 140 mm of block gets
    107 mm of rise and the leg on the top crouches the rest)."""

    flat_stroke = nominal_stroke_2d(posture)
    beta_end = float(flat_stroke.end.beta_rad)
    reach = float(replace(posture, theta_rad=THETA_MAX_RAD, hold_hip_z_m=None,
                          hold_hip_z_profile=None, obstacle_xwh_m=None,
                          ground_height_m=0.0).hip_z_for_flat_stance(beta_end))
    return float(min(float(spec.height_m), reach - float(posture.hold_hip_z_m) - margin_m))


def terrain_axle_profiles_2d(spec: SharedTerrainSpec2D, hold_m: float, *,
                             rise_before_m: float = 0.22, rise_until_m: float = 0.04,
                             fall_from_m: float = 0.03, fall_until_m: float = 0.20,
                             rise_m: float | None = None,
                             ) -> AxleProfiles2D:
    """A first guess from the terrain alone: rise on the way in, fall on the way out.

    Used for the first planning pass; the second pass replaces the ramps with
    the swing windows the first pass actually produced
    (:func:`axle_profiles_from_events_2d`), so that a swing's own linear hip
    path and its partner's levelled stroke agree.
    """

    x0, x1 = float(spec.x_start_m), float(spec.x_max_m)
    profile = _ramp_profile(float(hold_m), float(spec.height_m) if rise_m is None else float(rise_m),
                            (x0 - rise_before_m, x0 - rise_until_m),
                            (x1 + fall_from_m, x1 + fall_until_m))
    return AxleProfiles2D(profile, profile)


def _recoveries_before_climb(plan, leg: LegId):
    """``(climb_kind, [(x0, x1), ...])``: this leg's climb segment kind
    (``"ROLL_UP"`` or ``"SWING_UP"``, ``None`` without one) and the hip x
    windows of its recovery swings before it, in order."""

    leg_plan = plan.plans.get(leg)
    if leg_plan is None:
        return None, []
    segments = list(leg_plan.chain.segments)
    windows, run, climb = [], None, None
    for seg in segments:
        kind = seg.kind.value
        if kind in ("ROLL_UP", "SWING_UP"):
            climb = kind
            break
        if kind == "RECOVERY_SWING":
            frames = leg_plan.frames[seg.frames.source_id]
            xs = [float(frames[i].hip_xz_m[0]) for i in seg.frames.indices]
            if run is None:
                run = [min(xs), max(xs)]
            else:
                run[1] = max(run[1], max(xs))
        else:
            if run is not None:
                windows.append(tuple(run))
                run = None
    if run is not None:
        windows.append(tuple(run))
    return climb, windows


def _swing_hip_path(plan, leg: LegId, kind: str, *, first: bool):
    """``(hip_x, hip_z)`` knots of this leg's first (or last) swing of ``kind``,
    read from the swing's own frames, or ``None`` when it has none."""

    leg_plan = plan.plans.get(leg)
    if leg_plan is None:
        return None
    # A transition is written as consecutive pieces of one kind (fold,
    # rotation, extension): one swing is the run of them.  Measured: taking
    # the first piece alone gave the fold's flat path and pinned the rise
    # onto a single x -- a 50 mm step in the axle profile.
    segments = list(leg_plan.chain.segments)
    runs = []
    for k, seg in enumerate(segments):
        if seg.kind.value != kind:
            continue
        if runs and runs[-1][-1] == k - 1:
            runs[-1].append(k)
        else:
            runs.append([k])
    if not runs:
        return None
    run = runs[0] if first else runs[-1]
    path = []
    for k in run:
        seg = segments[k]
        frames = leg_plan.frames[seg.frames.source_id]
        path += [(float(frames[i].hip_xz_m[0]), float(frames[i].hip_xz_m[1]))
                 for i in seg.frames.indices]
    return path


def _ramp_knots(path, hold_m: float, height_m: float, min_ramp_m: float, *,
                rising: bool = True):
    """The knots of one axle move, from the swing's hip path.

    The generator moves the hip linearly in x over the whole swing and in z
    only over the rotation phase, so the ramp is the frames where z changes.
    Read off the frames rather than assumed linear over the window: a ramp
    assumed linear over the window put the stance leg 11 mm below its
    airborne partner (measured: coplanarity residual 5.7 mm).  A swing
    flown in place has no x extent; its ramp is spread over ``min_ramp_m``
    from its takeoff, which the stance legs follow over the next few cm.
    """

    xs = [x for x, _ in path]
    raw = [z for _, z in path]
    # The swing was flown from whatever height the *previous* profile put its
    # takeoff at; only its shape is kept.  The ends are pinned to the held
    # height and the held height plus the block (measured: a swing that took
    # off from a half-raised ramp gave a profile 34 mm high everywhere before
    # the face, and every approach stroke grew by 33 mm).
    z0 = float(hold_m) if rising else float(hold_m) + float(height_m)
    z1 = float(hold_m) + float(height_m) if rising else float(hold_m)
    span = raw[-1] - raw[0]
    if abs(span) <= 1e-9:
        # The swing flew level: the previous profile had the axle already
        # at its landing height at the takeoff, so the swing's path says
        # nothing about where the ramp is.  The caller keeps the old ramp.
        return None
    zs = [z0 + (z - raw[0]) / span * (z1 - z0) for z in raw]
    moving = [i for i, z in enumerate(zs) if abs(z - z0) > 1e-9 and abs(z - z1) > 1e-9]
    if not moving:
        return [(xs[0], z0), (max(xs[-1], xs[0] + min_ramp_m), z1)]
    lo, hi = moving[0] - 1, moving[-1] + 1
    knots = [(xs[lo], z0)]
    for i in range(lo + 1, hi + 1):
        if xs[i] > knots[-1][0] + 1e-6:
            knots.append((xs[i], zs[i]))
    if knots[-1][0] - knots[0][0] < min_ramp_m:
        # Too short in x for the stance legs to follow: linear over the minimum.
        return [(knots[0][0], z0), (knots[0][0] + min_ramp_m, z1)]
    if abs(knots[-1][1] - z1) > 1e-9:
        knots.append((knots[-1][0] + 1e-6, z1))
    return knots


def axle_profiles_from_events_2d(plan, spec: SharedTerrainSpec2D, hold_m: float,
                                 previous: AxleProfiles2D, *,
                                 min_ramp_m: float = 0.06,
                                 rise_m: float | None = None,
                                 fall_after_hip_x_m: dict | None = None,
                                 fall_ramp_m: float = 0.10,
                                 pre_drop_m: float = 0.0,
                                 pre_drop_ramp_m: float = 0.10,
                                 late_rise_m: float = 0.0,
                                 late_rise_ramp_m: float = 0.10,
                                 late_rise_rear_m: float | None = None,
                                 rear_prelift_m: float = 0.0,
                                 rear_prelift_roller_m: float = 0.0) -> AxleProfiles2D:
    """Ramps per pair, in hip x: the rise follows the pair's first UP's own
    hip path (see :func:`_ramp_knots`); the fall is a linear ramp that
    starts once the pair's last DOWN has landed *and* the pair's hip x is
    past ``fall_after_hip_x_m[axle]`` (where the trailing leg stands at the
    held height with the block behind it), so the first leg down lands
    extended, rolls on, and both legs crouch together afterwards."""

    out = {}
    for axle in ("front", "rear"):
        legs = [LegId.LF, LegId.RF] if axle == "front" else [LegId.LH, LegId.RH]
        climb = None
        for leg in legs:
            leg_plan = plan.plans.get(leg)
            if leg_plan is None:
                continue
            segs = list(leg_plan.chain.segments)
            for k, seg in enumerate(segs):
                if seg.kind.value == "ROLL_UP":
                    frames = leg_plan.frames[seg.frames.source_id]
                    path = [(float(frames[i].hip_xz_m[0]), float(frames[i].hip_xz_m[1]))
                            for i in seg.frames.indices]
                    # ... then the swing to the nominal pose on the top: every
                    # consecutive piece of it (fold, rotation, extension).
                    started = False
                    for later in segs[k + 1:]:
                        if later.kind.value == "TOP_REPOSITION_SWING":
                            started = True
                            path += [(float(frames[i].hip_xz_m[0]), float(frames[i].hip_xz_m[1]))
                                     for i in later.frames.indices]
                        elif started:
                            break
                    climb = path
                    break
            if climb is not None:
                break
        ups = [p for p in (_swing_hip_path(plan, leg, "SWING_UP", first=True) for leg in legs)
               if p]
        downs = [p for p in (_swing_hip_path(plan, leg, "SWING_DOWN", first=False) for leg in legs)
                 if p]
        if (not ups and climb is None) or not downs:
            out[axle] = previous.front if axle == "front" else previous.rear
            continue
        rise = float(spec.height_m) if rise_m is None else float(rise_m)
        # The owner's call (2026-09-09): the legs on the top stand shorter
        # for most of the top -- the body pitches less -- and the axle only
        # completes its rise over the last ``late_rise_ramp_m`` before the
        # pair's first descent, because a descent from the trailing edge
        # needs the full rise (measured: at +50/+60 the leg must land 74 mm
        # past the edge, at +100 it lands in place).
        late_here = (float(late_rise_m) if axle == "front" or late_rise_rear_m is None
                     else float(late_rise_rear_m))
        late = min(max(0.0, late_here), rise)
        plateau = rise - late
        old_profile = previous.front if axle == "front" else previous.rear
        if climb is not None:
            # The climb's own heights, as they are: the geometry of rolling
            # over the corner decides them.  Then the top swing's rise to
            # the nominal height above the top.
            up = []
            for x, z in climb:
                if not up or x > up[-1][0] + 1e-6:
                    up.append((x, z))
            # The climb's own heights end at the right-rim-top pose (hip at
            # hold + 50.1 mm, measured at 100 mm); the previous pass's top
            # swing then rose to the full height.  Clip that to the plateau,
            # or the knots spike above it and fall back (measured: the
            # partner's levelled stroke reversed at hip 1044 and was refused).
            up = [(x, min(z, float(hold_m) + plateau)) for x, z in up]
            up[0] = (up[0][0], float(hold_m))
            up[-1] = (up[-1][0], float(hold_m) + plateau)
        else:
            up = _ramp_knots(min(ups, key=lambda p: p[0][0]), hold_m, plateau, min_ramp_m, rising=True)
        if up is None:
            # Keep the previous rise: its knots up to the first at the top.
            up = []
            for x, z in zip(old_profile.hip_x_m, old_profile.hip_z_m):
                up.append((float(x), float(z)))
                if abs(float(z) - (float(hold_m) + plateau)) <= 1e-9:
                    break
            if len(up) < 2:
                up = [(float(spec.x_start_m) - 0.22, float(hold_m)),
                      (float(spec.x_start_m) - 0.04, float(hold_m) + plateau)]
        if axle == "rear" and (rear_prelift_m > 1e-9 or rear_prelift_roller_m > 1e-9):
            # Hardware 2026-09-09 (Vicon, log 4.10): when a rear leg swings
            # while the fronts stand on the block, its own hip sags 45-58 mm
            # (20-39 mm on flat ground) and the closed wheel's 74 mm of
            # clearance is gone at the fold -- the leg folds on the ground
            # and rolls.  So the rear axle rises over the last 100 mm of the
            # stance before each rear leg's last recovery before its climb,
            # takes off that much higher, and lands at the held height (the
            # airborne hip descends through the swing; the roll-up's start
            # pose needs the held height, and a fall inside the *next*
            # stance shortened it and moved the climb 10 mm earlier, onto
            # the fronts' descents -- measured).
            # Keep the bumps where the first event-based pass put them: a
            # bump makes the stance before the recovery longer (the leg
            # extends on the rise), so re-anchoring on the recovery moved it
            # ~10 mm later every pass and never converged (measured: pass 2
            # planned, pass 3 refused with the climb 5 mm early).
            climb_x0 = up[0][0]
            prev = [(float(x), float(z)) for x, z in zip(old_profile.hip_x_m, old_profile.hip_z_m)
                    if float(x) < climb_x0 - 1e-9]
            keep = any(float(hold_m) + 1e-9 < z <= float(hold_m) + 0.08 for _, z in prev)
            # The first pass runs on the terrain-only guess (4 knots) with a
            # 100 mm top swing that makes the rear swinger pause in place:
            # its "last recovery" is that pause (measured: a zero-width
            # spike at hip 526, and the delivered plan was byte-identical to
            # the one without a bump).  Bumps are read off the first plan
            # made on an event-based profile.
            generic = len(old_profile.hip_x_m) <= 4
            bumps = []
            for leg in ([] if (keep or generic) else legs):
                climb_kind, windows = _recoveries_before_climb(plan, leg)
                if climb_kind is None or not windows:
                    continue
                lift = float(rear_prelift_roller_m if climb_kind == "ROLL_UP" else rear_prelift_m)
                if lift > 1e-9:
                    bumps.append((windows[-1][0], windows[-1][1], lift))
            bumps.sort()
            pre, last_x1 = [], None
            for x0, x1, lift in bumps:
                if x1 - x0 < 0.02:
                    continue   # a pause in place, not a recovery
                start = x0 - 0.10 if last_x1 is None else max(x0 - 0.10, last_x1)
                if start < x0 - 1e-6:
                    pre.append((start, float(hold_m)))
                pre.append((x0, float(hold_m) + lift))
                # A short plateau past the takeoff: the stance before the
                # recovery grows on the rise, so the next pass takes off a
                # few mm later and must still be at the top.
                pre.append((min(x0 + 0.03, x1 - 0.01), float(hold_m) + lift))
                pre.append((x1, float(hold_m)))
                last_x1 = x1
            if keep:
                pre = prev
                while pre and pre[-1][1] <= float(hold_m) + 1e-9 and len(pre) > 1 \
                        and pre[-2][1] <= float(hold_m) + 1e-9:
                    pre.pop()
            if pre:
                up = pre + [k for k in up if k[0] > pre[-1][0] + 1e-9]
        last_landing = max(p[-1][0] for p in downs)
        fall_from = last_landing
        if fall_after_hip_x_m is not None and axle in fall_after_hip_x_m:
            fall_from = max(fall_from, float(fall_after_hip_x_m[axle]))
        top_z = float(hold_m) + rise
        down = []
        if late > 1e-9:
            first_takeoff = min(p[0][0] for p in downs)
            start = max(first_takeoff - float(late_rise_ramp_m), up[-1][0] + 1e-6)
            down += [(start, float(hold_m) + plateau), (first_takeoff, top_z)]
            fall_from = max(fall_from, first_takeoff + 1e-6)
        if pre_drop_m > 1e-9:
            # Part of the fall happens on the top, before the first descent:
            # the legs on the top crouch by ``pre_drop_m`` over
            # ``pre_drop_ramp_m`` ending at the first descent's takeoff, so
            # the descending leg lands less extended (the simulation lost
            # three contacts when a leg landed at theta 145 deg with the
            # axle still 100 mm up).
            first_takeoff = min(p[0][0] for p in downs)
            drop = min(float(pre_drop_m), rise)
            down += [(first_takeoff - float(pre_drop_ramp_m), top_z),
                     (first_takeoff, top_z - drop)]
            top_z = top_z - drop
            fall_from = max(fall_from, first_takeoff + 1e-6)
        down += [(fall_from, top_z), (fall_from + float(fall_ramp_m), float(hold_m))]
        if up[-1][0] >= down[0][0] - 1e-9:
            raise PlannerRefusal2D(
                f"the {axle} axle's rise (to hip x {up[-1][0] * 1e3:.1f} mm) does not end "
                f"before its fall begins ({down[0][0] * 1e3:.1f} mm).")
        xs = [x for x, _ in up] + [x for x, _ in down]
        zs = [z for _, z in up] + [z for _, z in down]
        out[axle] = HipZProfile2D(tuple(xs), tuple(zs))
    return AxleProfiles2D(out["front"], out["rear"])


class Surface2D(str, Enum):
    GROUND_BEFORE = "GROUND_BEFORE"
    TOP = "TOP"
    GROUND_AFTER = "GROUND_AFTER"


@dataclass
class LegTerrainState2D:
    surface: Surface2D = Surface2D.GROUND_BEFORE
    swings_after: int = 0
    #: The recovery advance the edge plan asked for after the current stroke,
    #: or ``None`` when the stroke is to be followed by the transition itself.
    planned_advance: float | None = None
    #: Every transition this leg was asked for, accepted or refused.
    attempts: list = field(default_factory=list)
    #: Hip x where the leg landed on the top, once it has.
    top_landing_hip_m: float | None = None
    #: Rolling legs: the roll-up's start landing has been flown / the climb
    #: has been rolled / the leg stands on the right rim on the top.
    pending_roll_up: bool = False
    roll_up_done: bool = False
    on_right_rim: bool = False
    rephased: bool = False


@dataclass
class SwingSwingRule2D:
    """Swing up, roll on the top, swing down; nominal cycles either side."""

    spec: SharedTerrainSpec2D
    posture: NominalPosture2D
    config: RecoveryConfig2D
    #: A landing contact keeps this far from any edge.
    edge_margin_m: float = DEFAULT_LANDING_EDGE_MARGIN_M
    #: A stroke on the top may roll its contact this close to the trailing
    #: edge (the rim curves away from the edge); an in-place descent needs
    #: the hip within ~10 mm of it (measured: 10 short lands, 12 does not),
    #: and with the 50 mm landing margin the hip stopped 18 mm short.
    trailing_edge_margin_m: float = 0.035
    #: The landing search steps the hip this far each try.
    landing_step_m: float = 0.010
    #: ...and gives up after this much.
    landing_search_m: float = 0.30
    #: A top recovery is only worth it if at least this much stroke follows.
    min_top_stroke_m: float = 0.020
    #: A recovery before the face is only worth it if at least this much
    #: stroke can be rolled from its landing before the face stops it.
    min_stroke_after_recovery_m: float = 0.050
    #: An approach stroke stops with its contact this far short of the face,
    #: so that the climb can still retract (measured at 40 mm: 40 mm works;
    #: at 60 mm a mid-arc pose 74 mm short could not retract).  ``None``
    #: means the block's height, at least 40 mm.
    climb_margin_m: float | None = None
    #: A swing in place before the face lands this much short of the
    #: arc-start standing limit.
    pause_margin_m: float = 0.010
    #: Legs that climb by rolling (Day 6--7 right-rim roll-up as a stroke),
    #: their partners swing up once the axle is raised.  Empty: all swing.
    rolling_legs: frozenset = frozenset()
    #: Rolling legs that roll *with* their same-axle partner (bound roll-up):
    #: their stroke stops at the partner's roll-start landing, they recover
    #: in place into the same start pose, and the pair rolls up as one stance
    #: stroke -- no leg is airborne over the corner.  Hardware 2026-09-13:
    #: the stepping partner's folded wheel skimmed the ground and cleared
    #: the corner by 11-16 mm, and the rear stepper dragged for a second.
    #: After the descent such a leg's first stroke is shortened to restore
    #: the half-cycle offset to its partner.
    bound_partner_legs: frozenset = frozenset()
    #: The climb's theta.
    theta_climb_rad: float = float(np.deg2rad(60.0))
    #: The roll-start landing aims this far past the approach window's far
    #: end (the window is 200..125 mm short of the face; measured at 140 mm:
    #: 132 mm short, the extended leg touched the block).
    roll_start_back_off_m: float = 0.030
    #: The roller's partner must stand through the climb and the top swing:
    #: its last recovery lands so that the following stroke reaches past the
    #: roller's top landing by this margin.
    partner_stance_margin_m: float = 0.010
    #: Off: landing the partner late put it in the dead zone before the
    #: face (it could neither roll on nor climb, measured at 100 mm).  The
    #: partner keeps its nominal landing and its whole stroke instead.
    land_partner_late: bool = False
    #: A rolling leg may wait by landing in its roll-start pose anywhere in
    #: the approach window (tried for 140 mm; off by default).
    roller_waits_in_window: bool = False
    #: The rear pair waits before the face (swinging in place where that
    #: is still legal) until both front legs are down.
    rear_waits_for_front: bool = False
    #: The leg of a pair that lands on the top later descends first, from a
    #: cut stroke, so both descents fit before the rear pair arrives.
    second_lander_descends_first: bool = False
    #: The first of a pair to descend shortens its top stroke so the partner
    #: can wait on the top (needed at 140 mm; off by default).
    partner_room_on_top: bool = False
    #: On the top, end a stroke where a swing in place still lands on the top
    #: when its natural end would fall in the dead zone short of the edge --
    #: too far for an in-place descent, too close for a hop (measured at
    #: 100 mm on a +50 plateau: RH's stroke ended 50 mm short of the edge,
    #: LH's descent covered its liftoff, and neither a hop nor a capped
    #: descent was legal).
    hop_early_on_top: bool = False
    #: The rear pair's first descent lands its contact no further than this
    #: past the back face (``None``: the arc-start landing, whose contact
    #: leads the hip by 145 mm with the axle at +100).  Measured on the 0913
    #: v4 trials 2 and 5: the pair's second descent (LH) tips about LF--RH
    #: because RH's contact leads its hip by 145 mm (support margin -68 mm;
    #: LH's hip fell 173 mm and kicked the block).  The front pair has the
    #: mirror geometry and is stable (+44 mm), so only the rear is capped.
    rear_first_descent_contact_cap_m: float | None = None
    #: A front leg whose ground stroke after its own descent is cut so that
    #: its next stroke ends just after the rear pair's second descent lands:
    #: its contact then trails its hip while that descent is airborne, which
    #: moves the LF--RH support edge back under the CoM (same measurement).
    early_recovery_front_leg: LegId | None = None
    early_recovery_margin_m: float = 0.030
    #: The rear pair's first descender ends its top stroke this much before
    #: the in-place descent point and descends *with* that much body travel:
    #: an in-place descent can only land fully extended (145 mm ahead of the
    #: hip; any landing further along the arc extends into the block's back
    #: corner, measured), while a descent with 57.5 mm of travel can land up
    #: to 20% of the arc later, 77 mm ahead of the hip.  The partner rolls on
    #: to the edge meanwhile and descends in place.
    rear_first_descender: LegId | None = None
    rear_first_descent_travel_m: float = 0.0
    #: An in-place descent lands from this close to the trailing edge
    #: (measured at 100 mm, full rise: 6 mm short lands, 20 mm short needs
    #: 10 mm of travel).
    descend_in_place_margin_m: float = 0.010
    #: A stroke on the top is ended with its hip this far short of the
    #: trailing edge (inside the in-place limit, so a partner's descent
    #: capped at this leg's liftoff can still land: measured, the landing
    #: must be within 10 mm of the edge, 12 fails).
    top_stroke_end_short_m: float = 0.006
    #: The theta a terrain transition folds to while airborne; ``None`` is
    #: wheel mode's 17 deg (the nominal recovery always uses that).
    transition_theta_compact_rad: float | None = None
    #: The steepest axle rise (per metre of hip travel) a levelled stroke
    #: can follow without its hip moving backwards.
    max_rise_slope: float = 0.5
    #: A climb may land further along the top so the descent is nominal;
    #: off, it lands as early as it can and the descent takes the extra.
    prefer_nominal_descent: bool = True
    #: Where a descent should put its contact past the back face for the
    #: landing to stand once the axle is back down (measured at 40 mm).
    descent_landing_past_face_m: float = 0.100
    #: Nominal cycles to run past the obstacle before the leg is finished.
    cycles_after: int = 1
    #: A stroke whose leg stays this far from the block in x is the flat stroke
    #: translated (the flat geometry is translation-invariant to 0.0000 um,
    #: Day 12 Step 1), which saves a ~40 s generation per stroke.  Anything
    #: closer is generated with the obstacle in its scene.
    far_from_obstacle_m: float = 0.30
    #: Where each axle is, as a function of hip x.  ``None`` keeps the held
    #: height everywhere (a leg on the top then crouches by h).
    axles: AxleProfiles2D | None = None
    states: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.posture.hold_hip_z_m is None:
            raise ValueError("this rule levels the axle; the posture must hold a hip height.")
        self.hold_m = float(self.posture.hold_hip_z_m)
        if self.climb_margin_m is None:
            self.climb_margin_m = max(0.040, float(self.spec.height_m))
        if self.axles is None:
            self.axles = AxleProfiles2D.constant(self.hold_m)
        ground = obstacle_posture_2d(self.posture, self.spec)
        top = replace(self.posture, ground_height_m=float(self.spec.top_z_m),
                      obstacle_xwh_m=None)
        # One posture per (surface, axle): the profile is the axle's, and the
        # stroke generator levels theta against it step by step.
        self._postures = {
            (surface, axle): replace(base, hold_hip_z_profile=profile)
            for surface, base in ((Surface2D.GROUND_BEFORE, ground),
                                  (Surface2D.GROUND_AFTER, ground),
                                  (Surface2D.TOP, top))
            for axle, profile in (("front", self.axles.front), ("rear", self.axles.rear))
        }
        self._flat_stroke = nominal_stroke_2d(self.posture)
        start = self._flat_stroke.frames[0]
        #: How far ahead of the hip the contact sits at a landing (the arc start).
        self.contact_lead_m = float(start.contact_xz_m[0] - start.hip_xz_m[0])
        #: One full stroke's contact advance.
        self.full_stroke_m = float(self._flat_stroke.contact_advance_m)
        #: How much a recovery from the arc end moves the contact relative to
        #: the hip: the lead at landing minus the lag at liftoff.
        self.arc_end_lag_m = float(self._flat_stroke.end.hip_xz_m[0]
                                   - self._flat_stroke.end.contact_xz_m[0])
        self.transitions: list[NominalTransition2D] = []
        self.generated_strokes = 0
        self.translated_strokes = 0
        self.cached_strokes = 0
        self.climb_cuts = 0
        self.early_recovery_cuts = 0
        self.descent_cap_choices: list = []
        #: The edge each surface is planned towards, as a **contact** x: the
        #: trailing edge less the margin on the top; before the face, the
        #: furthest contact at which the arc-end pose still stands legally,
        #: measured by bisection per axle (the standing leg reaches past its
        #: contact, by an amount that depends on the height).
        self._edge_contact = {
            ("front", Surface2D.TOP): float(self.spec.x_max_m) - self.trailing_edge_margin_m,
            ("rear", Surface2D.TOP): float(self.spec.x_max_m) - self.trailing_edge_margin_m,
        }
        self._arc_start_limit_hip_x = {}
        for axle in ("front", "rear"):
            hip_limit = self._arc_end_standing_limit_hip_x(axle)
            self._edge_contact[(axle, Surface2D.GROUND_BEFORE)] = (
                hip_limit - self.arc_end_lag_m)
            self._arc_start_limit_hip_x[axle] = self._arc_start_standing_limit_hip_x(axle)

    def _arc_start_standing_limit_hip_x(self, axle: str) -> float:
        """The furthest hip x before the face at which the arc-*start* pose
        stands: where a swing may still land, and so where a leg may still
        swing in place.  Bisection like the arc-end limit."""

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        posture = self._postures[(Surface2D.GROUND_BEFORE, axle)]
        profile = self.axles.front if axle == "front" else self.axles.rear
        beta = float(self._flat_stroke.start.beta_rad)

        def legal(hip_x: float) -> bool:
            try:
                theta, hip_z = held_landing_pose_2d(self.posture, beta, 0.0, profile.at(hip_x))
            except ValueError:
                return False
            return standing_stroke_2d(posture, theta, beta, hip_x, hip_z).success

        far, near = float(self.spec.x_start_m) - 0.60, float(self.spec.x_start_m)
        if not legal(far):
            raise PlannerRefusal2D(
                f"the arc-start pose does not stand even {0.6 * 1e3:.0f} mm before the face.")
        if legal(near):
            return near
        for _ in range(12):
            mid = 0.5 * (far + near)
            if legal(mid):
                far = mid
            else:
                near = mid
        return far

    def stroke_is_uncuttable(self, progress: LegProgress2D) -> bool:
        """Whether cutting this leg's pending stroke for another leg's swing
        can never work: a climb (the corner's geometry), or a stroke on the
        top that already ends at the trailing edge -- a pause there just
        restarts the same stroke to the same edge (measured at 100 mm: LH,
        22 mm from the edge, paused for RF's recovery and was still inside
        it).  The other leg yields instead."""

        stroke = progress.stroke
        if stroke is None:
            return False
        if stroke.stop_reason == ROLL_UP_STOP_REASON:
            return True
        state = self.states.get(progress.leg)
        if state is None or state.surface is not Surface2D.TOP:
            return False
        edge = self._edge_for(progress.leg, Surface2D.TOP)
        hip_limit = float(self.spec.x_max_m) - self.descend_in_place_margin_m
        return (float(stroke.end.contact_xz_m[0]) >= edge - self.min_top_stroke_m
                or float(stroke.end.hip_xz_m[0]) >= hip_limit - self.min_top_stroke_m)

    def latest_pause_hip_x_m(self, progress: LegProgress2D) -> float | None:
        """The furthest hip x at which this leg could swing in place and land
        legally, or ``None`` when its surface puts no such limit.

        Before the face a swing in place lands with its contact one lead
        ahead of the hip, at the arc-start pose, and that pose needs
        ~100-180 mm of room to the face (Day 14 §2.2).  A leg squeezed to
        pause right at another leg's takeoff can therefore be too close to
        the face to pause there at all (measured at 80 and 100 mm: LF at hip
        859 had no legal landing under its own cap).  The loop cuts it back
        to here instead.
        """

        state = self.states.get(progress.leg)
        if state is None or state.surface is not Surface2D.GROUND_BEFORE:
            return None
        if (self.roller_waits_in_window and progress.leg in self.rolling_legs
                and not state.roll_up_done):
            # A rolling leg waits by landing in the roll-up's start pose,
            # anywhere in its approach window (the pose stands right up to
            # the window's near end, measured at 100 and 140 mm).
            return float(self.spec.x_start_m) - APPROACH_WINDOW_M[1]
        axle = "front" if progress.leg.is_front else "rear"
        return self._arc_start_limit_hip_x[axle] - self.pause_margin_m

    def _arc_end_standing_limit_hip_x(self, axle: str) -> float:
        """The furthest hip x before the face at which the arc-end pose stands.

        Bisection on the standing legality of the levelled arc-end pose with
        the obstacle in the scene, between half a metre out and the face.
        """

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        posture = self._postures[(Surface2D.GROUND_BEFORE, axle)]
        profile = self.axles.front if axle == "front" else self.axles.rear
        end = self._flat_stroke.end
        beta = float(end.beta_rad)

        def legal(hip_x: float) -> bool:
            try:
                theta, hip_z = held_landing_pose_2d(self.posture, beta, 0.0, profile.at(hip_x))
            except ValueError:
                return False   # the leg cannot reach the axle height there
            return standing_stroke_2d(posture, theta, beta, hip_x, hip_z).success

        # The hip leads the contact by 62 mm (flat) to 95 mm (axle raised) at
        # the arc end, so the limit can lie *past* the face in hip x
        # (measured: a levelled stroke stood at hip x = face + 7 mm).  The
        # near bound is therefore 100 mm past the face, not 20 mm short of it.
        far, near = float(self.spec.x_start_m) - 0.50, float(self.spec.x_start_m) + 0.10
        if not legal(far):
            raise PlannerRefusal2D(
                f"the arc-end pose does not stand even {0.5 * 1e3:.0f} mm before the face.")
        if legal(near):
            return near
        for _ in range(12):
            mid = 0.5 * (far + near)
            if legal(mid):
                far = mid
            else:
                near = mid
        return far

    def _generate(self, posture: NominalPosture2D, *, start_beta_rad, hip_x_m,
                  max_distance_m=None) -> RollStroke2D:
        """``run_foot_rim_roll_2d`` behind an on-disk cache.

        Generating a levelled stroke costs ~40 s and the same request recurs
        across planning passes and driver reruns.  The cache is keyed on the
        posture (frozen, hashable), the request, and the generator source's
        modification time, so an edited generator never serves a stale
        stroke.  Off unless ``DAY14_STROKE_CACHE`` names a directory.
        """

        import hashlib
        import os
        import pickle

        self.generated_strokes += 1
        cache_dir = os.environ.get("DAY14_STROKE_CACHE")
        if not cache_dir:
            return run_foot_rim_roll_2d(posture, start_beta_rad=start_beta_rad,
                                        hip_x_m=float(hip_x_m), max_distance_m=max_distance_m)
        import hybrid_note.scripts.experiments.day12_nominal_cycle_2d as generator
        stamp = os.path.getmtime(generator.__file__)
        key = hashlib.sha1(repr((posture, start_beta_rad, float(hip_x_m), max_distance_m,
                                 stamp)).encode()).hexdigest()
        path = Path(cache_dir) / f"stroke_{key}.pkl"
        if path.exists():
            with open(path, "rb") as handle:
                self.cached_strokes += 1
                return pickle.load(handle)
        stroke = run_foot_rim_roll_2d(posture, start_beta_rad=start_beta_rad,
                                      hip_x_m=float(hip_x_m), max_distance_m=max_distance_m)
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as handle:
            pickle.dump(stroke, handle)
        return stroke

    def _ground_stroke(self, leg: LegId, surface: Surface2D, hip_x_m: float,
                       beta_rad: float | None) -> RollStroke2D:
        """A full stroke on the ground, translated when the block is out of reach."""

        posture = self._posture_on(surface, leg)
        if (surface is Surface2D.GROUND_BEFORE and leg in self.rolling_legs
                and not self._state(leg).roll_up_done):
            # The rolling leg's own approach stays at the held height: the
            # axle's rise *is* its climb, which begins where it lands, so a
            # profile read here would move that landing between passes
            # (measured: the pass-2 landing sat 6 mm up the rise and the
            # traversal refused it as not on the ground).
            posture = replace(posture, hold_hip_z_profile=None, hold_hip_z_m=self.hold_m)
        flat = self._flat_stroke
        arc_start = float(flat.start.beta_rad)
        at_arc_start = beta_rad is None or abs(
            (float(beta_rad) - arc_start) % (2.0 * np.pi)) < 1e-9 or abs(
            (float(beta_rad) - arc_start) % (2.0 * np.pi) - 2.0 * np.pi) < 1e-9
        span = float(flat.end.hip_xz_m[0] - flat.start.hip_xz_m[0])
        far = (float(hip_x_m) + span + self.far_from_obstacle_m < float(self.spec.x_start_m)
               or float(hip_x_m) - self.far_from_obstacle_m > float(self.spec.x_max_m))
        # Translation is only the flat stroke where the axle is at its flat
        # height for the whole stroke.
        level = (abs(self._axle_z_at(leg, hip_x_m) - self.hold_m) < 1e-9
                 and abs(self._axle_z_at(leg, hip_x_m + span) - self.hold_m) < 1e-9)
        max_distance = None
        if surface is Surface2D.GROUND_BEFORE:
            # Stop with the contact a climb's margin short of the face: the
            # arc-end pose stands with its contact right at the face, but a
            # climb from there retracts into the block (measured at 60 mm:
            # RETRACT_PENETRATES_TERRAIN from hip x = face + 117 mm).
            contact0 = float(hip_x_m) + self.contact_lead_m
            room = float(self.spec.x_start_m) - self.climb_margin_m - contact0
            if room < self.full_stroke_m - 1e-9:
                max_distance = max(0.0, room)
        if at_arc_start and far and level and max_distance is None:
            dx = float(hip_x_m) - float(flat.start.hip_xz_m[0])
            dbeta = 0.0 if beta_rad is None else float(beta_rad) - arc_start
            frames = tuple(_translate_frame(f, dx, dbeta) for f in flat.frames)
            end = frames[-1]
            scene = posture.scene(float(end.beta_rad), float(end.hip_xz_m[0]),
                                  float(end.hip_xz_m[1]), theta_rad=float(end.theta_rad))
            self.translated_strokes += 1
            return replace(flat, frames=frames, posture=posture, final_scene=scene)
        stroke = self._generate(posture, start_beta_rad=beta_rad, hip_x_m=float(hip_x_m),
                                max_distance_m=max_distance)
        stroke = self._end_before_hip_reverses(stroke)
        if (surface is Surface2D.GROUND_BEFORE and leg in self.rolling_legs
                and not self._state(leg).roll_up_done and stroke.success and stroke.frames):
            # A rolling leg stops where the roll-up starts (the window's far
            # end plus the back-off, where every roller has landed) and
            # recovers in place into the start pose; a bound partner thus
            # meets its roller there.  (A stroke ending short of it still
            # recovers forward to it, as before.)
            from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
                truncate_stroke_2d)
            start_x = float(self.spec.x_start_m) - APPROACH_WINDOW_M[0] + self.roll_start_back_off_m
            if float(stroke.end.hip_xz_m[0]) > start_x + 1e-6 >= float(stroke.frames[0].hip_xz_m[0]):
                stroke = truncate_stroke_2d(stroke, start_x, frame_at=self.frame_at_hip_x)
            return stroke
        if (surface is Surface2D.GROUND_AFTER and self.early_recovery_front_leg is not None
                and leg is self.early_recovery_front_leg and stroke.success and stroke.frames
                and not all(self.states.get(r) is not None
                            and self.states[r].surface is Surface2D.GROUND_AFTER
                            for r in (LegId.LH, LegId.RH))):
            # Cut so that the next stroke (a nominal recovery, then a full
            # stroke) ends ``early_recovery_margin_m`` after the rear pair's
            # second descent lands: takeoff at the top-stroke end, one
            # nominal advance of travel, and the front hip 510 mm ahead.
            from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
                truncate_stroke_2d)
            rear_landing = (float(self.spec.x_max_m) - self.top_stroke_end_short_m
                            + float(self.config.hip_advance_m))
            cut = (rear_landing + 0.510 + float(self.early_recovery_margin_m) - span
                   - float(self.config.hip_advance_m))
            if float(stroke.frames[0].hip_xz_m[0]) + 0.02 < cut < float(stroke.end.hip_xz_m[0]) - 1e-6:
                import os
                if os.environ.get("DAY14_TRACE"):
                    print(f"    early recovery: {leg.value} ground stroke cut at hip {cut * 1e3:.1f} mm "
                          f"(was {float(stroke.end.hip_xz_m[0]) * 1e3:.1f})")
                stroke = truncate_stroke_2d(stroke, cut, frame_at=self.frame_at_hip_x)
                self.early_recovery_cuts += 1
        partner = {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                   LegId.LH: LegId.RH, LegId.RH: LegId.LH}[leg]
        waits_for_climb = (partner in self.rolling_legs and leg not in self.rolling_legs
                           and not (self.states.get(partner) is not None
                                    and self.states[partner].roll_up_done
                                    and not self.states[partner].on_right_rim))
        if (surface is Surface2D.GROUND_BEFORE and stroke.success and stroke.frames
                and float(stroke.end.hip_xz_m[0]) > float(self.spec.x_start_m) - 0.30):
            # (Also for a roller's partner: without it the partner's stroke
            # ran to the face and no climb could retract, measured at 100 mm.)
            # Any stroke that ends near the face, capped or not: measured at
            # 100 mm, an uncapped full stroke ended with its hip at the face
            # and no climb could retract from there.
            stroke = self._end_where_a_climb_is_possible(leg, stroke)
        return stroke

    def _end_before_hip_reverses(self, stroke: RollStroke2D) -> RollStroke2D:
        """A stance stroke whose levelled hip moves backwards is cut before
        that frame: the body does not reverse (measured on a stroke levelled
        up a rise of 1.5 mm per mm: hip steps of -8 mm)."""

        if not stroke.success or len(stroke.frames) < 2:
            return stroke
        hips = [float(f.hip_xz_m[0]) for f in stroke.frames]
        for i in range(1, len(hips)):
            if hips[i] < hips[i - 1] - 1e-9:
                return replace(stroke, frames=stroke.frames[:max(1, i)], success=True,
                               stop_reason="LEVELLING_MOVES_HIP_BACKWARD")
        return stroke

    def _end_where_a_climb_is_possible(self, leg: LegId, stroke: RollStroke2D) -> RollStroke2D:
        """The same stroke, ended at its last frame from which a climb can
        still retract.

        Standing legality is not climb legality: on a raised axle the hip
        leads the contact by up to 180 mm at the arc end, so a stroke that
        stops with its contact a margin short of the face has its hip far
        past the face, and retracting from there drags the wheel into the
        block (measured at 60 mm: every climb from hip x = face + 117 mm was
        RETRACT_PENETRATES_TERRAIN, while the other session measured a
        climb in place from hip x = face + 12 mm).  The generator is the
        judge: walk the frames back from the end until a climb flies.
        """

        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            run_nominal_transition_2d)
        earliest = float(self.spec.x_start_m) + self.edge_margin_m - self.contact_lead_m
        frames = list(stroke.frames)
        for i in range(len(frames) - 1, 0, -1):
            frame = frames[i]
            hip_x = float(frame.hip_xz_m[0])
            landing = max(earliest, hip_x)
            out = run_nominal_transition_2d(
                self.spec, self.posture, self.config, kind=TransitionKind2D.UP,
                takeoff_theta_rad=float(frame.theta_rad), takeoff_beta_rad=float(frame.beta_rad),
                takeoff_hip_xz_m=(hip_x, float(frame.hip_xz_m[1])),
                landing_hip_x_m=landing,
                landing_hip_z_above_surface_m=self._axle_z_at(leg, landing) - float(self.spec.top_z_m),
                landing_edge_margin_m=self.edge_margin_m)
            if out.success or not str(out.refusal).startswith("RETRACT"):
                if i == len(frames) - 1:
                    return stroke
                cut = replace(stroke, frames=tuple(frames[:i + 1]), success=True,
                              stop_reason="CLIMB_NEEDS_EARLIER_TAKEOFF")
                self.climb_cuts += 1
                return cut
        return stroke

    # -- state -------------------------------------------------------------

    def _state(self, leg: LegId) -> LegTerrainState2D:
        return self.states.setdefault(leg, LegTerrainState2D())

    def _surface_of(self, landing: CycleFrame2D) -> Surface2D:
        if landing.surface_id and landing.surface_id.endswith("_top"):
            return Surface2D.TOP
        if float(landing.contact_xz_m[0]) > float(self.spec.x_max_m):
            return Surface2D.GROUND_AFTER
        return Surface2D.GROUND_BEFORE

    def _posture_on(self, surface: Surface2D, leg: LegId) -> NominalPosture2D:
        return self._postures[(surface, "front" if leg.is_front else "rear")]

    def _axle_z_at(self, leg: LegId, hip_x_m: float) -> float:
        return float(self.axles.for_leg(leg).at(float(hip_x_m)))

    # -- planning backwards from an edge ------------------------------------

    def _plan_to_edge(self, distance_m: float, lag_now_m: float) -> tuple[float, float | None]:
        """``(stroke contact length, recovery advance or None)`` so that the leg
        reaches the edge **at its arc end**.

        ``distance_m`` is the contact distance to the edge; ``lag_now_m`` is
        how far the contact sits *behind* the hip now (0 at the arc start,
        61.7 mm at the arc end).  A recovery from a pose with lag ``l`` moves
        the contact by ``l + lead + advance``; a stroke of length ``s`` moves
        it by ``s`` and grows the lag by about ``s * (lead + lag_end) / full``.
        The last stroke must be a full one, because the descent (or the climb)
        is shortest from the arc end, and a short window is what the other
        legs' liftoffs need.

        Returns the stroke to roll now and, if a recovery should follow it,
        that recovery's hip advance.  ``None`` means: roll and then leave.
        """

        full = self.full_stroke_m
        lead, lag_end = self.contact_lead_m, self.arc_end_lag_m
        nominal = float(self.config.hip_advance_m)
        d = float(distance_m)
        if d < full - 1e-9:
            return max(0.0, d), None
        # Room for a recovery *and* a full stroke after it?
        gain = (lead + lag_end) / full          # lag grown per metre of stroke
        room = d - full - lead - float(lag_now_m)
        if room < 1e-4:
            return full, None
        # s * (1 + gain) + a = room, with a nominal advance if it fits.  The
        # advance is never more than nominal: a longer recovery is a longer
        # airborne window, and that is what swallows the other legs' liftoffs
        # (measured: a 2x advance, 115 mm of body travel, did exactly that).
        # Distance that does not fit is left for the next landing to re-plan,
        # which then costs one more short recovery rather than a long one.
        s = (room - nominal) / (1.0 + gain)
        if s < 0.0:
            return 0.0, max(1e-4, room)
        if s > full:
            return full, nominal
        return s, nominal

    def _edge_for(self, leg: LegId, surface: Surface2D) -> float | None:
        return self._edge_contact.get(("front" if leg.is_front else "rear", surface))

    # -- strokes -----------------------------------------------------------

    def first_stroke(self, progress: LegProgress2D, *, hip_x_m: float) -> StanceAction2D:
        state = self._state(progress.leg)
        state.surface = Surface2D.GROUND_BEFORE
        stroke = self._ground_stroke(progress.leg, state.surface, float(hip_x_m), None)
        return StanceAction2D(stroke=stroke, phase=TransitionPhase.NOMINAL_BEFORE)

    def stroke_after(self, progress: LegProgress2D,
                     landing: CycleFrame2D) -> StanceAction2D | None:
        state = self._state(progress.leg)
        state.surface = self._surface_of(landing)
        hip_x = float(landing.hip_xz_m[0])
        beta = float(landing.beta_rad)
        contact = float(landing.contact_xz_m[0])
        lag = hip_x - contact
        if state.surface is Surface2D.TOP:
            if state.top_landing_hip_m is None:
                state.top_landing_hip_m = hip_x
            # A full stroke unless the trailing edge is closer; the descent
            # then flies from wherever that ends.  Planning the stroke so the
            # leg reached the edge exactly at its arc end (Day 14 first
            # attempt) produced a 4 mm stroke followed by a top recovery --
            # a leg swinging twice in a row, which the other legs paid for
            # with pauses.  A descent from a full stroke's arc end lands
            # 110 mm past the back face on a 400 mm top, which is legal.
            length = min(self.full_stroke_m,
                         self._edge_for(progress.leg, Surface2D.TOP) - contact)
            mine, other = state, self.states.get(
                {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                 LegId.LH: LegId.RH, LegId.RH: LegId.LH}[progress.leg])
            if (self.second_lander_descends_first and other is not None
                    and other.surface is Surface2D.TOP and other.top_landing_hip_m is not None
                    and mine.top_landing_hip_m is not None
                    and mine.top_landing_hip_m > other.top_landing_hip_m + 1e-9):
                # The leg that landed on the top *later* descends *first*:
                # its stroke is cut so that a nominal descent lands before
                # the partner's arc end, and the partner then descends from
                # its own arc end.  Otherwise both descents end after the
                # rear pair reaches the face and nobody can wait (measured
                # at 100 mm: 20 mm short, every pass).
                stroke_hip = float(self._flat_stroke.hip_advance_m)
                partner_arc_end = other.top_landing_hip_m + stroke_hip
                my_arc_end_wanted = partner_arc_end - float(self.config.hip_advance_m) - 0.005
                hip_room = my_arc_end_wanted - hip_x
                contact_room = hip_room * self.full_stroke_m / stroke_hip
                if contact_room >= self.min_top_stroke_m:
                    length = min(length, contact_room)
            elif self.partner_room_on_top and self._partner_still_on_top_behind(progress.leg):
                # The first of a pair to descend leaves its partner room to
                # wait on the top: the partner's swing in place lands one
                # lead ahead of the hip, and the hip is one lag ahead of
                # this stroke's end.  Measured at 140 mm: RF taking off with
                # its contact 111 mm short of the edge left LF's in-place
                # landing 13 mm past it, and the plan was refused.
                room = (self._edge_for(progress.leg, Surface2D.TOP)
                        - self.arc_end_lag_m - self.contact_lead_m - 0.005) - contact
                length = min(length, max(0.0, room))
            state.planned_advance = None
            stroke = self._generate(self._posture_on(Surface2D.TOP, progress.leg),
                                    start_beta_rad=beta, hip_x_m=hip_x,
                                    max_distance_m=max(0.0, length))
            stroke = self._end_before_hip_reverses(stroke)
            if stroke.success and stroke.frames:
                # End where an in-place descent still lands: the hip at most
                # ``descend_in_place_margin_m`` short of the trailing edge.
                # The contact-based end alone left the hip 18 mm short on a
                # 900 mm top (mid-arc lag 32 mm) and both fronts, at the edge
                # together, had no descent (measured).
                from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
                    truncate_stroke_2d)
                hip_limit = float(self.spec.x_max_m) - self.top_stroke_end_short_m
                if self._descends_first_with_travel(progress.leg):
                    hip_limit -= float(self.rear_first_descent_travel_m)
                if float(stroke.end.hip_xz_m[0]) > hip_limit + 1e-9 >= float(stroke.frames[0].hip_xz_m[0]):
                    stroke = truncate_stroke_2d(stroke, hip_limit, frame_at=self.frame_at_hip_x)
                    if self._descends_first_with_travel(progress.leg):
                        import os
                        if os.environ.get("DAY14_TRACE"):
                            print(f"    early descent: {progress.leg.value} top stroke ends at hip "
                                  f"{hip_limit * 1e3:.1f} mm, {self.rear_first_descent_travel_m * 1e3:.1f} mm "
                                  "before the in-place point")
            if self.hop_early_on_top and stroke.success and stroke.frames:
                # A stroke whose natural end lies between the last legal hop
                # landing and the descent zone is ended at that landing: the
                # leg then hops in place there and its next stroke reaches
                # the zone.
                from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
                    truncate_stroke_2d)
                h_star, zone_hip = self._last_hop_landing_hip_x()
                if self._descends_first_with_travel(progress.leg):
                    h_star -= float(self.rear_first_descent_travel_m)
                    zone_hip -= float(self.rear_first_descent_travel_m)
                end_hip = float(stroke.end.hip_xz_m[0])
                if (h_star + 1e-6 < end_hip < zone_hip - 1e-6
                        and h_star >= hip_x + self.min_top_stroke_m):
                    stroke = truncate_stroke_2d(stroke, h_star, frame_at=self.frame_at_hip_x)
            return StanceAction2D(stroke=stroke, phase=TransitionPhase.ON_TOP,
                                  phase_label="TOP_FOOT_RIM_ROLL")
        if state.surface is Surface2D.GROUND_AFTER and state.swings_after >= self.cycles_after:
            return None
        if (state.surface is Surface2D.GROUND_AFTER and progress.leg in self.bound_partner_legs
                and not state.rephased):
            # The pair landed in phase; shorten this leg's first ground
            # stroke so its liftoff comes half a cycle before the partner's
            # (the nominal offset of the two legs of one axle).
            from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
                truncate_stroke_2d)
            state.rephased = True
            stroke = self._ground_stroke(progress.leg, state.surface, hip_x, beta)
            half = 0.5 * float(self._flat_stroke.hip_advance_m)
            if stroke.success and stroke.frames and float(stroke.end.hip_xz_m[0]) > hip_x + half + 1e-6:
                stroke = truncate_stroke_2d(stroke, hip_x + half, frame_at=self.frame_at_hip_x)
            return StanceAction2D(stroke=stroke, phase=TransitionPhase.NOMINAL_AFTER)
        if state.pending_roll_up:
            # The leg landed in the roll-up's start pose: the climb is its
            # next stroke, stance from here to the top's right-rim pose.
            climb = self._roll_up(progress.leg, hip_x, beta)
            if not climb.success:
                raise PlannerRefusal2D(
                    f"{progress.leg.value}: the roll-up from hip x {hip_x * 1e3:.1f} mm is "
                    f"refused: {climb.refusal}")
            state.pending_roll_up = False
            state.roll_up_done = True
            state.on_right_rim = True
            state.surface = Surface2D.TOP
            return StanceAction2D(stroke=climb.stroke, kind=SegmentKind.ROLL_UP,
                                  phase_label="RIGHT_RIM_ROLL_UP", phase=TransitionPhase.ASCENT)
        # On the ground before the face the strokes stay full: the phase the
        # pair arrives at the face with is set by where the walk *starts*
        # (``ideal_origins_2d``), not by cutting strokes on the way, because a
        # cut stroke lifts early and lands in the previous leg's window --
        # measured: the cuts cascaded through all four legs until two of them
        # were due at the same body x.  The stroke itself still stops at the
        # last legal pose (``OBSTACLE_AHEAD``).
        stroke = self._ground_stroke(progress.leg, state.surface, hip_x, beta)
        phase = (TransitionPhase.NOMINAL_AFTER if state.surface is Surface2D.GROUND_AFTER
                 else TransitionPhase.NOMINAL_BEFORE)
        return StanceAction2D(stroke=stroke, phase=phase)

    # -- swings ------------------------------------------------------------

    def swing_after(self, progress: LegProgress2D, *,
                    landing_cap_body_x_m: float | None = None,
                    next_liftoffs_body_x_m: Sequence[float] = ()) -> SwingAction2D:
        state = self._state(progress.leg)
        stroke = progress.stroke
        assert stroke is not None
        end = stroke.end
        hip_x = float(end.hip_xz_m[0])
        # A cut leg lands no later than the cap: nominal advance if the room
        # allows, less if not, down to swinging in place (the body stands).
        cap_hip = (None if landing_cap_body_x_m is None
                   else float(landing_cap_body_x_m) + progress.mount_x_m)
        nominal_hip = hip_x + float(self.config.hip_advance_m)
        recovery_hip = nominal_hip if cap_hip is None else max(hip_x, min(nominal_hip, cap_hip))
        # The other legs' liftoffs ahead, in this leg's hip x: a landing past
        # one of them swallows that leg's slot.
        slots = tuple(float(x) + progress.mount_x_m for x in next_liftoffs_body_x_m)
        contact = float(end.contact_xz_m[0])
        lag = hip_x - contact
        if (state.surface is Surface2D.GROUND_BEFORE and self.rear_waits_for_front
                and not progress.leg.is_front and not self._front_pair_is_down()
                and cap_hip is None):
            # The rear pair reaches the face while the front pair is still
            # descending (the wheelbase is 510 mm, a 400 mm top plus the
            # descents is longer), and near the face there is no legal
            # place to wait.  So a rear leg waits *here*, swinging in place
            # while it still can, until both front legs are down.  Its
            # roll-start landing (a rolling leg) is that pause when it is in
            # the window.
            axle_limit = self._arc_start_limit_hip_x["rear"] - self.pause_margin_m
            if hip_x <= axle_limit and progress.leg not in self.rolling_legs:
                pause = self._fly(progress.leg, TransitionKind2D.RECOVERY, end, hip_x)
                if pause.success:
                    return self._action(pause, TransitionPhase.NOMINAL_BEFORE)
            if progress.leg in self.rolling_legs and not state.roll_up_done:
                lo = float(self.spec.x_start_m) - APPROACH_WINDOW_M[0]
                hi = float(self.spec.x_start_m) - APPROACH_WINDOW_M[1]
                if lo <= hip_x <= hi:
                    start = self._fly_to_roll_start(progress.leg, end, hip_x)
                    if start.success:
                        state.pending_roll_up = True
                        return self._action(start, TransitionPhase.NOMINAL_BEFORE)
                elif hip_x < lo and hip_x <= axle_limit:
                    pause = self._fly(progress.leg, TransitionKind2D.RECOVERY, end, hip_x)
                    if pause.success:
                        return self._action(pause, TransitionPhase.NOMINAL_BEFORE)
        if (state.surface is Surface2D.GROUND_BEFORE and progress.leg in self.rolling_legs
                and not state.roll_up_done):
            lo = float(self.spec.x_start_m) - APPROACH_WINDOW_M[0]
            hi = float(self.spec.x_start_m) - APPROACH_WINDOW_M[1]
            if hip_x + 1e-4 > hi:
                raise PlannerRefusal2D(
                    f"{progress.leg.value} rolls, but at hip x {hip_x * 1e3:.1f} mm it is past "
                    f"its approach window [{lo * 1e3:.0f}, {hi * 1e3:.0f}] mm; choose the origin "
                    "so that a recovery lands in it.")
            if nominal_hip >= lo - 1e-9 or hip_x + float(self.config.hip_advance_m) * 2.0 >= lo:
                # Within a recovery of the window: land in it, at (theta
                # climb, beta 0) -- the roll-up's own start pose.
                target = min(max(lo + self.roll_start_back_off_m, hip_x), hi)
                if cap_hip is not None:
                    target = min(target, cap_hip)
                if target < hip_x - 1e-9 or target < lo - 1e-9:
                    raise PlannerRefusal2D(
                        f"{progress.leg.value} rolls, but its landing in the approach window "
                        f"would be capped at hip x {cap_hip * 1e3:.1f} mm, behind it.")
                start = self._fly_to_roll_start(progress.leg, end, target)
                if not start.success:
                    raise PlannerRefusal2D(
                        f"{progress.leg.value}: the landing in the roll-up's start pose at hip x "
                        f"{target * 1e3:.1f} mm is refused: {start.refusal}")
                state.pending_roll_up = True
                return self._action(start, TransitionPhase.NOMINAL_BEFORE)
        if state.surface is Surface2D.TOP and state.on_right_rim:
            # From the top of the climb (right rim, theta climb) to the
            # nominal pose on the top; measured: lands at every hip tried
            # with 25--28 mm of rotation clearance.
            # The axle still has to rise from the climb's end height to the
            # held height above the top; that rise happens over this swing's
            # rotation, and the partner's stroke is levelled on it.  Steeper
            # than ``max_rise_slope`` the levelled stroke moves the hip
            # *backwards* (measured: 50 mm of rise over 32 mm of travel gave
            # hip steps of -8 mm), so the swing is at least long enough.
            # Looked ahead 300 mm before the late rise existed; the plateau
            # is what this swing lands on, and the late rise (>= 150 mm
            # further, over the levelled strokes) is not this swing's.
            ahead = max(self._axle_z_at(progress.leg, hip_x + d)
                        for d in (0.02, 0.05, 0.10, 0.15))
            rise_left = max(0.0, ahead - float(end.hip_xz_m[1]))
            spread = rise_left / self.max_rise_slope
            earliest = max(float(self.spec.x_start_m) + self.edge_margin_m - self.contact_lead_m,
                           hip_x + spread)
            # The far landing for a nominal descent only when asked for
            # (as for the stepping climb): on a 900 mm top it is 1554 mm,
            # and capped at the next liftoff it made this swing 259 mm long
            # and the partner could not stand through it (measured).
            ideal_top = max(nominal_hip, hip_x + spread)
            if self.prefer_nominal_descent:
                ideal_top = max(ideal_top, self._top_landing_for_nominal_descent())
            top = self._search(progress.leg, TransitionKind2D.TOP, end, earliest,
                               cap_hip_x_m=cap_hip, slots_hip_x_m=slots,
                               ideal_hip_x_m=ideal_top)
            state.on_right_rim = False
            return self._action(top, TransitionPhase.ON_TOP)
        if state.surface is Surface2D.GROUND_BEFORE:
            earliest_up = (float(self.spec.x_start_m) + self.edge_margin_m
                           - self.contact_lead_m)
            climb_fits = cap_hip is None or earliest_up <= cap_hip + 1e-9
            nominal = None
            stopped = stroke.stop_reason in ("OBSTACLE_AHEAD", "CLIMB_NEEDS_EARLIER_TAKEOFF")
            partner_floor = (self._partner_stance_landing_floor(progress.leg)
                             if self.land_partner_late else None)
            if (partner_floor is not None and cap_hip is None and not stopped
                    and nominal_hip < partner_floor
                    and partner_floor <= self._arc_start_limit_hip_x[
                        "front" if progress.leg.is_front else "rear"] - self.pause_margin_m):
                # The partner of a rolling leg: land late enough that the
                # next stroke stands through the roller's climb and top swing
                # (the axle is too low mid-climb for this leg to land on the
                # top, measured at 100 mm), a longer swing the loop makes
                # room for.
                late = self._fly(progress.leg, TransitionKind2D.RECOVERY, end, partner_floor)
                if late.success:
                    return self._action(late, TransitionPhase.NOMINAL_BEFORE)
            if not stopped or not climb_fits:
                # Recover if a recovery still lands legally before the face
                # (the transition checks the landing against the terrain)
                # *and* leaves a stroke worth rolling before the face stops
                # it -- a landing 111 mm short of a 40 mm face stood, then
                # rolled 0 mm and had to climb at once (measured), which is
                # two swings of one leg back to back.  A leg cut to land
                # before another leg's takeoff, whose climb would land past
                # that, recovers under the cap if it legally can.
                if cap_hip is None:
                    # Slot-aware like every other landing: a nominal landing
                    # 0.2 mm past the next leg's liftoff put two legs in the
                    # air for 1 ms (measured at 60 mm, RH then LH).
                    try:
                        nominal = self._search(progress.leg, TransitionKind2D.RECOVERY, end,
                                               hip_x, slots_hip_x_m=slots)
                    except PlannerRefusal2D:
                        nominal = self._fly(progress.leg, TransitionKind2D.RECOVERY, end,
                                            recovery_hip)
                else:
                    # Under a cap the nominal landing may sit too close to the
                    # face (its arc-start pose reaches ~100 mm ahead of the
                    # contact); a shorter recovery, down to one in place,
                    # lands further back and stands.  Measured: RH capped at
                    # hip 851 was refused at 851 and had nothing else tried.
                    try:
                        nominal = self._search(progress.leg, TransitionKind2D.RECOVERY, end,
                                               hip_x, cap_hip_x_m=cap_hip)
                    except PlannerRefusal2D as error:
                        nominal = NominalTransition2D.refused(
                            TransitionKind2D.RECOVERY, str(error)) if hasattr(
                            NominalTransition2D, "refused") else None
                if nominal is not None and nominal.success and climb_fits:
                    landing = nominal.swing.end
                    following = self._ground_stroke(
                        progress.leg, Surface2D.GROUND_BEFORE,
                        float(landing.hip_xz_m[0]), float(landing.beta_rad))
                    if (following.contact_advance_m < self.min_stroke_after_recovery_m
                            and following.stop_reason == "OBSTACLE_AHEAD"):
                        state.attempts[-1] = (TransitionKind2D.RECOVERY.value, recovery_hip,
                                              False, "RECOVERY_LEAVES_NO_STROKE")
                        nominal = replace(nominal, swing=None,
                                          refusal="RECOVERY_LEAVES_NO_STROKE")
                if nominal is not None and nominal.success:
                    return self._action(nominal, TransitionPhase.NOMINAL_BEFORE)
            if not climb_fits:
                raise PlannerRefusal2D(
                    f"{progress.leg.value}: at hip {hip_x * 1e3:.1f} mm before the face it "
                    f"must land by hip {cap_hip * 1e3:.1f} mm, but the earliest landing on "
                    f"the top is at hip {earliest_up * 1e3:.1f} mm and no recovery under "
                    f"the cap stands: {None if nominal is None else nominal.refusal}")
            # The second leg of a pair to land on the top must reach its own
            # arc end no earlier than its partner's descent lands: the two
            # share a hip x, the partner descends first (it landed first,
            # and a descent lands no closer than ~100 mm past the back
            # face), and a descent window cannot be shortened -- measured:
            # RF's descent [1106, 1186] mm swallowed LF's arc end at 1163.5
            # and LF, cut, had no legal landing under the cap.
            partner_floor = self._landing_floor_after_partner_descent(progress.leg)
            if partner_floor is not None and partner_floor > earliest_up:
                if (partner_floor + self.contact_lead_m + self.full_stroke_m
                        > float(self.spec.x_max_m) - self.trailing_edge_margin_m + 1e-9):
                    raise PlannerRefusal2D(
                        f"{progress.leg.value}: to reach its arc end after its partner's "
                        f"descent lands it would have to land at hip {partner_floor * 1e3:.1f} mm, "
                        "and a full stroke from there runs past the trailing edge.")
                earliest_up = partner_floor
            # The climb lands as early as it legally can (nominal advance,
            # or the first landing on the top): a far leg's climb is then
            # ~110 mm of body travel instead of ~210, and the extra goes to
            # its descent -- two medium swings, not one long one (the
            # simulation sagged 11 deg during a 1.4 s climb).
            up = self._search(progress.leg, TransitionKind2D.UP, end, earliest_up,
                              cap_hip_x_m=cap_hip, slots_hip_x_m=slots,
                              ideal_hip_x_m=(max(nominal_hip, self._top_landing_for_nominal_descent())
                                             if self.prefer_nominal_descent else nominal_hip))
            return self._action(up, TransitionPhase.ASCENT)
        if state.surface is Surface2D.TOP:
            # Descend, landing as close to the nominal advance as the ground
            # past the back face allows.  A capped leg that cannot fit a
            # descent under its cap lands on the top again if any top is left
            # -- rolling to the edge is how it waits for the previous descent.
            edge = self._edge_for(progress.leg, Surface2D.TOP)
            earliest_down = (float(self.spec.x_max_m) + self.edge_margin_m
                             - self.contact_lead_m)
            # A long top: a nominal recovery on the top (its landing one
            # lead ahead of the hip) when enough top remains for it and a
            # stroke worth rolling after it -- a descent from here would
            # be as long as the top left (measured on a 650 mm top: 346 mm).
            recovery_contact = lag + self.contact_lead_m + float(self.config.hip_advance_m)
            h_star, zone_hip = self._last_hop_landing_hip_x()
            if hip_x <= h_star + 1e-9 and hip_x < zone_hip - self.descend_in_place_margin_m:
                # Not yet in the descent zone and a hop can still land where
                # the stroke after it reaches the zone: hop, at most to
                # ``h_star`` (in place when already there).  Beyond
                # ``h_star`` the leg is contact-limited short of the zone
                # and can only descend with travel (measured on a 900 mm
                # top: 57 mm short, neither hop nor in-place descent).
                hop_cap = h_star if cap_hip is None else min(cap_hip, h_star)
                nominal_top = hip_x + float(self.config.hip_advance_m)
                top = self._search(progress.leg, TransitionKind2D.TOP, end, hip_x,
                                   cap_hip_x_m=hop_cap, slots_hip_x_m=slots,
                                   ideal_hip_x_m=min(nominal_top, h_star))
                if top.success:
                    return self._action(top, TransitionPhase.ON_TOP)
            if cap_hip is None or earliest_down <= cap_hip + 1e-9:
                down = self._search(progress.leg, TransitionKind2D.DOWN, end,
                                    earliest_down, cap_hip_x_m=cap_hip, slots_hip_x_m=slots)
                return self._action(down, TransitionPhase.DESCENT)
            room_after = edge - (recovery_hip + self.contact_lead_m)
            if room_after >= 0.0:
                top = self._fly(progress.leg, TransitionKind2D.TOP, end, recovery_hip)
                if top.success:
                    return self._action(top, TransitionPhase.ON_TOP)
            raise PlannerRefusal2D(
                f"{progress.leg.value}: on the top at hip {hip_x * 1e3:.1f} mm it can "
                f"neither descend under its cap ({cap_hip * 1e3:.1f} mm) nor land on the "
                f"top again ({room_after * 1e3:.1f} mm of top would remain).")
        # GROUND_AFTER: nominal recoveries, counted.  Searched forward from the
        # nominal landing: just past the back face a nominal landing can still
        # put the leg's rear into the block.
        nominal = self._search(progress.leg, TransitionKind2D.RECOVERY, end, recovery_hip,
                               cap_hip_x_m=cap_hip, slots_hip_x_m=slots)
        state.swings_after += 1
        return self._action(nominal, TransitionPhase.NOMINAL_AFTER)

    # -- helpers -----------------------------------------------------------

    def _fly(self, leg: LegId, kind: TransitionKind2D, takeoff: CycleFrame2D,
             landing_hip_x_m: float) -> NominalTransition2D:
        surface_z = (float(self.spec.top_z_m)
                     if kind in (TransitionKind2D.UP, TransitionKind2D.TOP)
                     else float(self.spec.ground_height_m))
        config = self.config
        if kind is not TransitionKind2D.RECOVERY and self.transition_theta_compact_rad is not None:
            # A terrain transition need not fold all the way to wheel mode:
            # on a raised axle there is room to spare (74 mm at 17 deg), and
            # a leg that folds less snaps less.  The generator still checks
            # the clearance of every frame.
            config = replace(config, theta_compact_rad=float(self.transition_theta_compact_rad))
        landing_beta = None
        if (kind is TransitionKind2D.DOWN and self.rear_first_descent_contact_cap_m is not None
                and leg in (LegId.LH, LegId.RH) and not self._partner_is_down(leg)):
            landing_beta = self._capped_descent_landing_beta(leg, config, takeoff,
                                                             float(landing_hip_x_m), surface_z)
        out = run_nominal_transition_2d(
            self.spec, self.posture, config, kind=kind,
            takeoff_theta_rad=float(takeoff.theta_rad),
            takeoff_beta_rad=float(takeoff.beta_rad),
            takeoff_hip_xz_m=(float(takeoff.hip_xz_m[0]), float(takeoff.hip_xz_m[1])),
            landing_hip_x_m=float(landing_hip_x_m),
            landing_beta_rad=landing_beta,
            landing_hip_z_above_surface_m=self._axle_z_at(leg, landing_hip_x_m) - surface_z,
            landing_edge_margin_m=self.edge_margin_m)
        self._state(leg).attempts.append((kind.value, landing_hip_x_m, out.success,
                                          out.refusal))
        self.transitions.append(out)
        return out

    def _capped_descent_landing_beta(self, leg: LegId, config, takeoff: CycleFrame2D,
                                     landing_hip_x_m: float, surface_z: float) -> float | None:
        """The landing beta, further along the arc than the arc start, at which
        the descent's contact lands no further than the cap past the back face
        (``None`` when the arc-start landing already does, or no such landing
        flies).  Stepped in twentieths of the arc up to 60% of it: past that
        the leg has too little arc left for the partner's descent to roll on."""

        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            arc_start_landing_beta_rad)
        cap_x = float(self.spec.x_max_m) + float(self.rear_first_descent_contact_cap_m)
        arc_start = arc_start_landing_beta_rad(self.posture, float(takeoff.beta_rad))
        sweep = float(self._flat_stroke.end.beta_rad) - float(self._flat_stroke.start.beta_rad)
        best = None
        for k in range(0, 13):
            beta = arc_start + (k / 20.0) * sweep
            out = run_nominal_transition_2d(
                self.spec, self.posture, config, kind=TransitionKind2D.DOWN,
                takeoff_theta_rad=float(takeoff.theta_rad),
                takeoff_beta_rad=float(takeoff.beta_rad),
                takeoff_hip_xz_m=(float(takeoff.hip_xz_m[0]), float(takeoff.hip_xz_m[1])),
                landing_hip_x_m=float(landing_hip_x_m), landing_beta_rad=float(beta),
                landing_hip_z_above_surface_m=self._axle_z_at(leg, landing_hip_x_m) - surface_z,
                landing_edge_margin_m=self.edge_margin_m)
            if not out.success or out.landing_contact_xz_m is None:
                continue
            contact_x = float(out.landing_contact_xz_m[0])
            if best is None or contact_x < best[1]:
                best = (beta, contact_x, k)
            if contact_x <= cap_x + 1e-9:
                self.descent_cap_choices.append((leg.value, k, contact_x - float(self.spec.x_max_m)))
                import os
                if os.environ.get("DAY14_TRACE"):
                    print(f"    descent cap: {leg.value} lands {k}/20 of the arc past the arc start, "
                          f"contact {(contact_x - float(self.spec.x_max_m)) * 1e3:.1f} mm past the back face")
                return None if k == 0 else float(beta)
        if best is not None:
            self.descent_cap_choices.append((leg.value, best[2], best[1] - float(self.spec.x_max_m)))
            import os
            if os.environ.get("DAY14_TRACE"):
                print(f"    descent cap: {leg.value} no landing under the cap; closest {best[2]}/20 "
                      f"of the arc, contact {(best[1] - float(self.spec.x_max_m)) * 1e3:.1f} mm past the face")
            return None if best[2] == 0 else float(best[0])
        return None

    def _partner_stance_landing_floor(self, leg: LegId) -> float | None:
        """For the partner of a rolling leg, before the face: the lowest
        landing hip x from which a full stroke reaches past the roller's
        top landing, or ``None`` when this leg's partner does not roll."""

        partner = {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                   LegId.LH: LegId.RH, LegId.RH: LegId.LH}[leg]
        if partner not in self.rolling_legs or leg in self.rolling_legs:
            return None
        other = self.states.get(partner)
        if other is not None and other.top_landing_hip_m is not None:
            top_landing = float(other.top_landing_hip_m)
        else:
            # Not planned yet: the climb ends ~40 mm past the face and the
            # top swing lands ~30-70 mm on (measured at 100 mm: 1068-1094).
            top_landing = float(self.spec.x_start_m) + 0.095
        stroke_hip = float(self._flat_stroke.hip_advance_m)
        return top_landing + self.partner_stance_margin_m - stroke_hip

    def _partner_still_on_top_behind(self, leg: LegId) -> bool:
        """Whether this leg's same-axle partner is on the top and has not
        descended (so this leg descends first and the partner must wait)."""

        partner = {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                   LegId.LH: LegId.RH, LegId.RH: LegId.LH}[leg]
        other = self.states.get(partner)
        mine = self.states.get(leg)
        if other is None or other.surface is not Surface2D.TOP:
            return False
        # Both on the top: the one that landed first reaches its arc end
        # first and descends first; only *it* leaves room.  The other must
        # keep its whole stroke -- it rolls on while the first descends
        # (measured: capping both left the second with a 0 mm stroke).
        if (mine is not None and mine.top_landing_hip_m is not None
                and other.top_landing_hip_m is not None):
            return mine.top_landing_hip_m <= other.top_landing_hip_m
        return other.top_landing_hip_m is None

    def _front_pair_is_down(self) -> bool:
        """Both front legs have descended (or never climbed)."""

        for leg in (LegId.LF, LegId.RF):
            st = self.states.get(leg)
            if st is not None and st.surface in (Surface2D.TOP,):
                return False
            if st is not None and st.surface is Surface2D.GROUND_BEFORE and (
                    st.pending_roll_up or (leg in self.rolling_legs and not st.roll_up_done)
                    or True):
                # still before the face: not down yet
                return False
        return True

    def _descends_first_with_travel(self, leg: LegId) -> bool:
        """Whether this leg is the rear pair's first descender that takes off
        ``rear_first_descent_travel_m`` early and descends with travel."""

        return (self.rear_first_descender is not None and leg is self.rear_first_descender
                and float(self.rear_first_descent_travel_m) > 0.0
                and not self._partner_is_down(leg))

    def _partner_is_down(self, leg: LegId) -> bool:
        """Whether this leg's same-axle partner has already descended, so
        that this leg is the pair's last: the axle comes down after *its*
        landing, and it must stand the drop where it lands.  The first leg
        to descend lands with the axle still raised and rolls on before the
        axle falls (the fall ramp starts no earlier than where it stands at
        the held height, see :func:`axle_profiles_from_events_2d`)."""

        partner = {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                   LegId.LH: LegId.RH, LegId.RH: LegId.LH}[leg]
        other = self.states.get(partner)
        return other is not None and other.surface is Surface2D.GROUND_AFTER

    def ground_after_stand_limit_hip_x(self, axle: str) -> float:
        """The smallest hip x past the block at which the arc-end pose stands
        at the held height with the block behind it (the trailing wheel is
        the leg's rearmost point).  Bisection; the axle may not fall before
        both legs of the pair are past it."""

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        posture = self._postures[(Surface2D.GROUND_AFTER, axle)]
        beta = float(self._flat_stroke.end.beta_rad)

        def legal(hip_x: float) -> bool:
            try:
                theta, hip_z = held_landing_pose_2d(self.posture, beta, 0.0, self.hold_m)
            except ValueError:
                return False
            return standing_stroke_2d(posture, theta, beta, hip_x, hip_z).success

        near, far = float(self.spec.x_max_m), float(self.spec.x_max_m) + 0.60
        if not legal(far):
            raise PlannerRefusal2D(
                f"the arc-end pose does not stand even {0.6 * 1e3:.0f} mm past the back face.")
        if legal(near):
            return near
        for _ in range(12):
            mid = 0.5 * (near + far)
            if legal(mid):
                far = mid
            else:
                near = mid
        return far

    def fall_start_hip_x(self, axle: str, landing_hips_m: Sequence[float],
                         top_z_m: float, ramp_m: float) -> float:
        """The earliest hip x at which the axle's fall ramp (``top_z_m`` down
        to the held height over ``ramp_m``) is legal along the pair's actual
        stance behind the block.

        The arc-end pose at the held height (``ground_after_stand_limit_hip_x``)
        stands only 200 mm past the back face (measured at 100 mm), and
        waiting for it keeps the landed legs at theta 145 deg for a whole
        stroke.  The leg lands at the arc *start*, leaning forward, and rolls
        back over its stroke; the trailing wheel is lowest at the arc end.
        Here beta is taken linear in hip x over the flat advance from each
        landing, the pose is solved at the ramp's height there, and stood
        with the block behind it.  Legality is monotone in the ramp's start
        (a later ramp is higher everywhere), so bisection.  Measured at
        100 mm from a landing at hip 1394: +80 stands at 1474, +60 at 1494,
        +27 at 1514, held height at 1534 -> a 100 mm ramp may start at 1454.
        """

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        posture = self._postures[(Surface2D.GROUND_AFTER, axle)]
        flat = self._flat_stroke
        b0, b1 = float(flat.frames[0].beta_rad), float(flat.end.beta_rad)
        adv = float(flat.end.hip_xz_m[0]) - float(flat.frames[0].hip_xz_m[0])
        top_z, hold, ramp = float(top_z_m), self.hold_m, max(float(ramp_m), 1e-3)

        def legal(x: float, beta: float, z: float) -> bool:
            try:
                theta, hip_z = held_landing_pose_2d(self.posture, beta, 0.0, z)
            except ValueError:
                return False
            return standing_stroke_2d(posture, theta, beta, x, hip_z).success

        def ramp_legal(x_land: float, x_s: float) -> bool:
            for k in range(6):
                x = x_s + ramp * k / 5.0
                z = top_z - (top_z - hold) * k / 5.0
                beta = b0 + (b1 - b0) * min(1.0, max(0.0, (x - x_land) / adv))
                if not legal(x, beta, z):
                    return False
            return True

        start = None
        for x_land in landing_hips_m:
            lo, hi = float(x_land), float(x_land) + 0.40
            if not ramp_legal(x_land, hi):
                hi = self.ground_after_stand_limit_hip_x(axle)
                if not ramp_legal(x_land, hi):
                    raise PlannerRefusal2D(
                        f"the {axle} axle's fall ramp is not legal even from hip x {hi * 1e3:.1f} mm.")
            if ramp_legal(x_land, lo):
                hi = lo
            else:
                for _ in range(7):
                    mid = 0.5 * (lo + hi)
                    if ramp_legal(x_land, mid):
                        hi = mid
                    else:
                        lo = mid
            start = hi if start is None else max(start, hi)
        return float(start)

    def _fly_to_roll_start(self, leg: LegId, takeoff: CycleFrame2D,
                           landing_hip_x_m: float) -> NominalTransition2D:
        """A recovery that lands in the roll-up's start pose: beta 0 one turn
        on, the held height (theta solves to the climb's 60 deg there)."""

        out = run_nominal_transition_2d(
            self.spec, self.posture, self.config, kind=TransitionKind2D.RECOVERY,
            takeoff_theta_rad=float(takeoff.theta_rad),
            takeoff_beta_rad=float(takeoff.beta_rad),
            takeoff_hip_xz_m=(float(takeoff.hip_xz_m[0]), float(takeoff.hip_xz_m[1])),
            landing_hip_x_m=float(landing_hip_x_m),
            landing_beta_rad=roll_up_landing_beta_rad(float(takeoff.beta_rad)),
            landing_hip_z_above_surface_m=self.hold_m - float(self.spec.ground_height_m),
            landing_edge_margin_m=self.edge_margin_m)
        self._state(leg).attempts.append(("ROLL_START", landing_hip_x_m, out.success, out.refusal))
        self.transitions.append(out)
        return out

    def _roll_up(self, leg: LegId, landing_hip_x_m: float, landing_beta_rad: float):
        """The climb from the landing, behind an on-disk cache (the Day 6--7
        traversal takes ~140 s and runs its descent too)."""

        import hashlib
        import os
        import pickle
        import hybrid_note.scripts.experiments.day14_rolling_crossing_2d as module
        import hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d as generator

        posture = self._postures[(Surface2D.GROUND_BEFORE, "front" if leg.is_front else "rear")]
        cache_dir = os.environ.get("DAY14_STROKE_CACHE")
        key = None
        if cache_dir:
            stamp = (os.path.getmtime(module.__file__), os.path.getmtime(generator.__file__))
            key = hashlib.sha1(repr((float(self.spec.height_m), float(self.spec.top_length_m),
                                     float(self.spec.x_start_m), round(float(landing_hip_x_m), 7),
                                     round(float(landing_beta_rad), 9), float(self.theta_climb_rad),
                                     stamp)).encode()).hexdigest()
            path = Path(cache_dir) / f"rollup_{key}.pkl"
            if path.exists():
                with open(path, "rb") as handle:
                    return pickle.load(handle)
        out = roll_up_stroke_2d(self.spec, posture, landing_hip_x_m=float(landing_hip_x_m),
                                landing_beta_rad=float(landing_beta_rad),
                                theta_climb_rad=float(self.theta_climb_rad))
        if key is not None:
            Path(cache_dir).mkdir(parents=True, exist_ok=True)
            with open(Path(cache_dir) / f"rollup_{key}.pkl", "wb") as handle:
                pickle.dump(out, handle)
        return out

    def _stands_at_hold(self, leg: LegId, landing: CycleFrame2D) -> str | None:
        """Why the landing pose would not stand once the axle is back at the
        flat height, or ``None`` when it would.

        The first leg of a pair to descend lands while the axle is still
        raised; the axle comes down later, with that leg standing next to the
        back face.  A landing that is legal only while extended is refused
        here, so the drop cannot push its wheel into the block (measured: a
        landing 9.7 mm past the back face stood at theta 95.7 and collided
        four roll steps later, when the axle started down).
        """

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        surface_z = float(landing.contact_xz_m[1])
        theta, hip_z = held_landing_pose_2d(self.posture, float(landing.beta_rad),
                                            surface_z, self.hold_m - surface_z)
        ground = obstacle_posture_2d(self.posture, self.spec)
        stand = standing_stroke_2d(ground, theta, float(landing.beta_rad),
                                   float(landing.hip_xz_m[0]), hip_z)
        return None if stand.success else f"NOT_STANDABLE_AT_FLAT_AXLE:{stand.stop_reason}"

    def _landing_floor_after_partner_descent(self, leg: LegId) -> float | None:
        """The lowest landing hip x on the top from which this leg's arc end
        comes after its same-axle partner's descent has landed, or ``None``
        when the partner is not on the top yet."""

        partner = {LegId.LF: LegId.RF, LegId.RF: LegId.LF,
                   LegId.LH: LegId.RH, LegId.RH: LegId.LH}[leg]
        other = self.states.get(partner)
        if other is None or other.surface is not Surface2D.TOP or other.top_landing_hip_m is None:
            return None
        stroke_hip = float(self._flat_stroke.hip_advance_m)
        partner_arc_end = other.top_landing_hip_m + stroke_hip
        h_star, _zone = self._last_hop_landing_hip_x()
        if partner_arc_end < h_star - 1e-9:
            # A long top: the partner hops again before it descends, so its
            # descent is not the next thing after its first stroke (measured
            # on a 900 mm top: this floor asked for a landing at hip 1617
            # and refused every origin).
            return None
        descent_landing = max(partner_arc_end + float(self.config.hip_advance_m),
                              float(self.spec.x_max_m) + self.descent_landing_past_face_m
                              - self.contact_lead_m)
        return descent_landing + 0.005 - stroke_hip

    def _last_hop_landing_hip_x(self) -> tuple[float, float]:
        """``(h_star, zone_hip)``: the latest hip x a hop on the top may land
        at so that the stroke after it reaches the in-place descent zone
        (``zone_hip``, the hip ``top_stroke_end_short_m`` short of the
        trailing edge) before its contact reaches the trailing margin.
        Contact-minus-hip runs from +lead to -lag over a stroke's hip
        advance (flat numbers; a plateau stroke is shorter, which only makes
        this conservative).  Measured on a 900 mm top: a hop landing later
        than this left the leg 30-60 mm short, able neither to hop nor to
        descend in place."""

        lead, lag_end = self.contact_lead_m, self.arc_end_lag_m
        advance = float(self._flat_stroke.hip_advance_m)
        zone_hip = float(self.spec.x_max_m) - self.top_stroke_end_short_m
        edge_contact = float(self.spec.x_max_m) - self.trailing_edge_margin_m
        need = advance * (lead + (zone_hip - edge_contact)) / max(lead + lag_end, 1e-6)
        return zone_hip - need, zone_hip

    def _top_landing_for_nominal_descent(self) -> float:
        """The landing hip x on the top from which a full stroke and a nominal
        descent put the contact ``descent_landing_past_face_m`` past the back
        face, where a landing next to the block stands once the axle is down
        (measured: 100 mm past a 40 mm block's back face stood, 90 did not).
        """

        return (float(self.spec.x_max_m) + self.descent_landing_past_face_m
                - self.contact_lead_m - float(self.config.hip_advance_m)
                - self.arc_end_lag_m - self.full_stroke_m - self.contact_lead_m)

    def _search(self, leg: LegId, kind: TransitionKind2D, takeoff: CycleFrame2D,
                earliest_landing_hip_x_m: float, *,
                cap_hip_x_m: float | None = None,
                slots_hip_x_m: Sequence[float] = (),
                ideal_hip_x_m: float | None = None) -> NominalTransition2D:
        """The legal landing nearest the preferred one, stepping outward both ways.

        ``cap_hip_x_m`` bounds the search from above: a leg that was cut to
        lift before another leg's swing must have landed by that swing's
        takeoff, so a landing past the cap is not a landing it can use.

        The preferred landing is the nominal advance (or ``ideal_hip_x_m``),
        but never past the first of the other legs' liftoffs
        (``slots_hip_x_m``) that a legal landing can stay short of: a swing
        that lands past another leg's liftoff makes that leg swing in place
        first, and the body stand still for it.
        """

        takeoff_hip = float(takeoff.hip_xz_m[0])
        start = max(float(earliest_landing_hip_x_m), takeoff_hip)
        # The nominal advance first, then outward from it on both sides: a
        # leg standing past the earliest landing must not be flown in place
        # when the nominal swing lands legally (measured: LF at hip x =
        # face + 7 mm was).
        preferred = takeoff_hip + float(self.config.hip_advance_m)
        if ideal_hip_x_m is not None:
            preferred = max(preferred, float(ideal_hip_x_m))
        for slot in sorted(float(v) for v in slots_hip_x_m):
            if slot >= start - 1e-9:
                preferred = min(preferred, slot)
                break
        if cap_hip_x_m is not None:
            preferred = min(preferred, float(cap_hip_x_m))
        preferred = max(preferred, start)
        grid = [start + step * self.landing_step_m
                for step in range(int(np.ceil(self.landing_search_m / self.landing_step_m)) + 1)]
        if cap_hip_x_m is not None:
            grid = [g for g in grid if g <= float(cap_hip_x_m) + 1e-9]
        if preferred >= start and all(abs(g - preferred) > 1e-9 for g in grid):
            grid.append(preferred)
        if not grid:
            raise PlannerRefusal2D(
                f"{leg.value}: no {kind.value} landing can lie between the earliest legal "
                f"landing at hip x {start * 1e3:.1f} mm and the cap at "
                f"{float(cap_hip_x_m) * 1e3:.1f} mm.")
        grid.sort(key=lambda g: (abs(g - preferred), g))
        last = None
        for candidate in grid:
            out = self._fly(leg, kind, takeoff, candidate)
            # A descent lands with the axle still raised, and the axle only
            # falls once the pair's hips are past where the trailing pose
            # stands at the held height (the fall ramp, see
            # axle_profiles_from_events_2d) -- so no descent has to stand
            # the drop where it lands.  Measured at 100 mm: requiring it of
            # the pair's last descent made that descent 120 mm long and
            # swallowed the rear leg's climb.
            must_stand_at_hold = kind is TransitionKind2D.RECOVERY
            if out.success and must_stand_at_hold:
                why = self._stands_at_hold(leg, out.swing.end)
                if why is not None:
                    self._state(leg).attempts[-1] = (kind.value, candidate, False, why)
                    out = replace(out, swing=None, refusal=why)
            if out.success:
                return out
            last = out
        raise PlannerRefusal2D(
            f"{leg.value}: no {kind.value} landing within {self.landing_search_m * 1e3:.0f} mm "
            f"of hip x {start * 1e3:.1f} mm"
            f"{'' if cap_hip_x_m is None else f' (capped at {cap_hip_x_m * 1e3:.1f} mm)'}; "
            f"last refusal: {None if last is None else last.refusal}")

    def frame_at_hip_x(self, stroke: RollStroke2D, hip_x_m: float) -> CycleFrame2D:
        """The stance pose of ``stroke`` at exactly ``hip_x_m``, between two frames.

        The loop cuts strokes where *other* legs' swings need them cut, and
        the previous swing's landing is not on this leg's 6.4 mm roll grid
        (measured: windows of 5 mm and of 0 mm had no frame in them).  Beta
        is interpolated between the neighbouring frames -- a rim rolling
        without slip has hip x nearly linear in beta over one step -- and
        theta is then *solved* for the axle height at that x, so the pose
        is an exact stance, checked as one against the terrain.  Frames made
        here carry ``index == -1`` so the write-out keeps them as their own
        two-frame segments (a segment's frames are read as uniform in beta).
        """

        from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import standing_stroke_2d
        from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
            held_landing_pose_2d)
        x = float(hip_x_m)
        hips = [float(f.hip_xz_m[0]) for f in stroke.frames]
        for frame, hip in zip(stroke.frames, hips):
            if abs(hip - x) < 1e-9:
                return frame
        if stroke.stop_reason == ROLL_UP_STOP_REASON:
            # A climb's poses are the corner's geometry, not a levelled
            # roll, so there is no solve to make one; but its frames are 2 mm
            # apart (5 mm on the approach), and a pose interpolated between
            # two of them is within the tolerance the standing check allows
            # (+0.5 mm of hip height stood, measured).  Without an exact
            # frame here the body's stops fell inside a climb segment and
            # the climbing leg's hip was read 22-58 mm from the body's.
            if x < hips[0] - 1e-9 or x > hips[-1] + 1e-9:
                raise PlannerRefusal2D(
                    f"hip x {x * 1e3:.3f} mm is outside the climb "
                    f"[{hips[0] * 1e3:.3f}, {hips[-1] * 1e3:.3f}] mm.")
            j = next(k for k, hip in enumerate(hips) if hip > x)
            a, b = stroke.frames[j - 1], stroke.frames[j]
            w = (x - hips[j - 1]) / (hips[j] - hips[j - 1])

            def mix(u, v):
                return None if u is None or v is None else float(u) + w * (float(v) - float(u))

            contact = (None if a.contact_xz_m is None or b.contact_xz_m is None
                       else (mix(a.contact_xz_m[0], b.contact_xz_m[0]),
                             mix(a.contact_xz_m[1], b.contact_xz_m[1])))
            return replace(a, index=-1, theta_rad=mix(a.theta_rad, b.theta_rad),
                           beta_rad=mix(a.beta_rad, b.beta_rad),
                           hip_xz_m=(x, mix(a.hip_xz_m[1], b.hip_xz_m[1])),
                           alpha_rad=mix(a.alpha_rad, b.alpha_rad), contact_xz_m=contact)
        if x < hips[0] - 1e-9 or x > hips[-1] + 1e-9:
            raise PlannerRefusal2D(
                f"hip x {x * 1e3:.3f} mm is outside the stroke "
                f"[{hips[0] * 1e3:.3f}, {hips[-1] * 1e3:.3f}] mm.")
        j = next(k for k, hip in enumerate(hips) if hip > x)
        a, b = stroke.frames[j - 1], stroke.frames[j]
        w = (x - hips[j - 1]) / (hips[j] - hips[j - 1])
        beta = float(a.beta_rad) + w * (float(b.beta_rad) - float(a.beta_rad))
        surface_z = float(a.contact_xz_m[1])
        z = stroke.posture.held_hip_z_at(x)
        if z is None:
            z = float(a.hip_xz_m[1]) + w * (float(b.hip_xz_m[1]) - float(a.hip_xz_m[1]))
        theta, hip_z = held_landing_pose_2d(self.posture, beta, surface_z, float(z) - surface_z)
        stand = standing_stroke_2d(stroke.posture, theta, beta, x, hip_z)
        if not stand.success:
            raise PlannerRefusal2D(
                f"the stroke's pose at hip x {x * 1e3:.3f} mm does not stand: "
                f"{stand.stop_reason}")
        return replace(stand.frames[0], index=-1, phase=a.phase)

    def _action(self, out: NominalTransition2D, phase: TransitionPhase) -> SwingAction2D:
        kind = SEGMENT_KIND_OF_TRANSITION[out.kind]
        label = ("NOMINAL_RECOVERY_SWING" if kind is SegmentKind.RECOVERY_SWING
                 else kind.value)
        return SwingAction2D(kind=kind, phase_label=label, phase=phase, swing=out.swing)

    def rows(self) -> list[dict]:
        return [{"row_kind": "transition", **t.as_dict()} for t in self.transitions]


# --------------------------------------------------------------------------
# The crossing, in two passes
# --------------------------------------------------------------------------


def origin_for_arc_end_contact_2d(rule: SwingSwingRule2D, clock, leg: LegId,
                                  contact_x_m: float) -> float:
    """The walk origin that puts one of ``leg``'s arc ends at ``contact_x_m``.

    A leg's landings (arc starts) fall at hip ``origin + v * t_leg + mount +
    k * cycle``; the contact at an arc start leads the hip by the contact
    lead, and a full stroke moves it by the full stroke.  That is in
    **contact** x, so it does not depend on the axle profile the stroke is
    levelled against (a stroke over the rising ramp moves the hip 33 mm
    further than a flat one, but the contact the same 202.5 mm).
    """

    from hybrid_note.scripts.experiments.day12_four_leg_state_2d import leg_mounts_2d
    mounts = {m.leg: float(m.offset_body_xyz_m[0]) for m in leg_mounts_2d(0.0)}
    cycle = float(clock.speed_m_s) * float(clock.timing.cycle_period_s)
    landing_hip = float(contact_x_m) - rule.contact_lead_m - rule.full_stroke_m
    base = clock.speed_m_s * clock.chain_start_time_s(leg) + mounts[leg]
    return float((landing_hip - base) % cycle)


def origin_for_landing_hip_2d(rule: SwingSwingRule2D, clock, leg: LegId, hip_x_m: float) -> float:
    """The walk origin that puts one of ``leg``'s landings (arc starts) at hip
    ``hip_x_m``: the roll-up's start pose must be landed in its window."""

    from hybrid_note.scripts.experiments.day12_four_leg_state_2d import leg_mounts_2d
    mounts = {m.leg: float(m.offset_body_xyz_m[0]) for m in leg_mounts_2d(0.0)}
    cycle = float(clock.speed_m_s) * float(clock.timing.cycle_period_s)
    base = clock.speed_m_s * clock.chain_start_time_s(leg) + mounts[leg]
    return float((float(hip_x_m) - base) % cycle)


def ideal_origins_2d(rule: SwingSwingRule2D, clock, *, face_margin_m: float | None = None) -> list[dict]:
    """One origin per leg: that leg reaches the face at its arc end with its
    contact ``face_margin_m`` short of the face.

    Measured on the 40 mm block: the arc-end pose stands with its contact
    right up to the face (nothing of the leg reaches past a trailing
    contact), but the climb needs about 40 mm of room to retract and
    rotate, and a full stroke whose *mid-arc* poses pass within ~100 mm of
    the face is stopped early (``OBSTACLE_AHEAD``) -- so the margin is on
    the contact at the arc end, and the last recovery before the face lands
    a full stroke short of it.  The four candidates differ by the gait's
    own phase offsets; the pairs (LF, RH) and (RF, LH) reach the face at
    nearly the same contact phase, half a cycle from the other pair.
    """

    from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER
    if face_margin_m is None:
        face_margin_m = float(rule.climb_margin_m)
    target = float(rule.spec.x_start_m) - float(face_margin_m)
    out = []
    if rule.rolling_legs:
        # A rolling leg's landing goes to the middle of its approach window.
        landing = float(rule.spec.x_start_m) - 0.5 * (APPROACH_WINDOW_M[0] + APPROACH_WINDOW_M[1])
        for leg in LEG_ORDER:
            if leg in rule.rolling_legs:
                out.append({"leg": leg.value, "axle": "front" if leg.is_front else "rear",
                            "origin_x_m": origin_for_landing_hip_2d(rule, clock, leg, landing),
                            "target_arc_end_contact_m": landing})
        return out
    for leg in LEG_ORDER:
        out.append({"leg": leg.value, "axle": "front" if leg.is_front else "rear",
                    "origin_x_m": origin_for_arc_end_contact_2d(rule, clock, leg, target),
                    "target_arc_end_contact_m": target})
    return out


def plan_swing_swing_crossing_2d(
    spec: SharedTerrainSpec2D,
    *,
    timing=None,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    samples: int = 301,
    cycles_after: int = 1,
    passes: int = 3,
    window_tolerance_m: float = 0.005,
    origins: Sequence[float] | None = None,
    rolling_legs=frozenset(),
    fall_ramp_m: float = 0.10,
    rule_overrides: dict | None = None,
    long_swing_speed_scale: float = 1.0,
    crossing_speed_scale: float = 1.0,
    pre_drop_m: float = 0.0,
    phase_dwell_min_s: float = 0.0,
    axle_rise_override_m: float | None = None,
    late_rise_m: float = 0.0,
    late_rise_ramp_m: float = 0.10,
    late_rise_rear_m: float | None = None,
    rear_prelift_m: float = 0.0,
    rear_prelift_roller_m: float = 0.0,
    fast_fall: bool = False,
    pre_rise_before_m: float = 0.22,
    pre_rise_until_m: float = 0.04,
    log=print,
):
    """Plan the SWING_SWING crossing with the axle profile converged.

    Pass 1 raises the axle over ramps guessed from the terrain.  Each later
    pass sets the ramps to the first UP and last DOWN windows the previous
    pass produced, so a climbing leg's own hip path and its partner's levelled
    stroke describe the same axle.  Stops when the windows move by less than
    ``window_tolerance_m``.  Returns ``(run, plan, rule, passes_used)``; the
    run's body z and pitch are then read off the four hips
    (``refit_body_plane_2d``), and the coplanarity residual says how well the
    two agree.
    """

    from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
    from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
        hybrid_posture_2d, hybrid_timing_2d)
    from hybrid_note.scripts.experiments.day12_world_registration_2d import (
        swing_hip_advance_m)
    from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
        plan_terrain_gait_first_2d)

    timing = hybrid_timing_2d() if timing is None else timing
    posture = hybrid_posture_2d() if posture is None else posture
    if config is None:
        config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import hybrid_clock_2d

    hold = float(posture.hold_hip_z_m)
    rise = axle_rise_m(spec, posture)
    if axle_rise_override_m is not None:
        # The owner's call: the legs on the top may stand crouched (the top
        # leg needs >= 145 mm, the partner's climb needs the axle >= 255 mm
        # above the ground at 100 mm), and a smaller rise pitches the body
        # less and shortens the descents.
        rise = min(rise, float(axle_rise_override_m))
    log(f"  axle rise {rise * 1e3:.1f} mm for a {spec.height_m * 1e3:.0f} mm block")
    # The first pass's terrain-only guess: the rise ramp before the face.
    # Its default slope (100 mm over 180 mm) is steeper than a levelled
    # stroke tolerates, so a stepping leg's stroke was cut at the ramp's
    # start and it stepped from 230 mm before the face -- a 1.3 s swing
    # (measured on the all-step baseline).  A one-pass plan keeps this
    # profile, so the ramp is a parameter.
    first_axles = terrain_axle_profiles_2d(spec, hold, rise_m=rise,
                                           rise_before_m=float(pre_rise_before_m),
                                           rise_until_m=float(pre_rise_until_m))
    if origins is None:
        probe = SwingSwingRule2D(spec=spec, posture=posture, config=config,
                                 cycles_after=cycles_after, axles=first_axles,
                                 rolling_legs=frozenset(rolling_legs), **(rule_overrides or {}))
        clock, _, _ = hybrid_clock_2d(timing, posture, config)
        candidates = ideal_origins_2d(probe, clock)
        for c in candidates:
            log(f"  origin candidate {c['origin_x_m'] * 1e3:7.1f} mm  ({c['leg']} at arc end "
                f"with contact at {c['target_arc_end_contact_m'] * 1e3:.1f} mm)")
        # Front-pair candidates first: the front pair meets the face first,
        # and the rear pair's residual is what the loop's cuts are for.
        origins = [c["origin_x_m"] for c in candidates if c["axle"] == "front"] + \
                  [c["origin_x_m"] for c in candidates if c["axle"] == "rear"]
    last_error = None
    for origin in origins:
        axles = first_axles
        log(f"  origin {origin * 1e3:.1f} mm")
        result = None
        try:
            for index in range(int(passes)):
                rule = SwingSwingRule2D(spec=spec, posture=posture, config=config,
                                        cycles_after=cycles_after, axles=axles,
                                        rolling_legs=frozenset(rolling_legs), **(rule_overrides or {}))
                try:
                    run, plan = plan_terrain_gait_first_2d(
                        rule, terrain=spec, strategy=StrategyId.SWING_SWING, timing=timing,
                        posture=posture, config=config, samples=samples,
                        origin_x_m=float(origin), long_swing_speed_scale=long_swing_speed_scale,
                        crossing_speed_scale=crossing_speed_scale,
                        phase_dwell_min_s=phase_dwell_min_s)
                except PlannerRefusal2D:
                    log(f"  pass {index + 1} refused; the legs' attempts so far:")
                    for leg, state in rule.states.items():
                        log(f"    {leg.value}: on {state.surface.value}; " + "; ".join(
                            f"{k}@{x * 1e3:.0f}{'+' if ok else '-'}"
                            f"{'' if ok else '(' + str(why)[:40] + ')'}"
                            for k, x, ok, why in state.attempts))
                    for name, profile in (("front", axles.front), ("rear", axles.rear)):
                        log(f"    axle {name}: " + ", ".join(
                            f"{x * 1e3:.1f}->{z * 1e3:.1f}"
                            for x, z in zip(profile.hip_x_m, profile.hip_z_m)))
                    raise
                fall_after = {}
                for axle in ("front", "rear"):
                    if fast_fall:
                        pair = [LegId.LF, LegId.RF] if axle == "front" else [LegId.LH, LegId.RH]
                        landings = [p[-1][0] for p in
                                    (_swing_hip_path(plan, leg, "SWING_DOWN", first=False)
                                     for leg in pair) if p]
                        if landings:
                            fall_after[axle] = rule.fall_start_hip_x(
                                axle, landings, hold + rise, float(fall_ramp_m))
                            log(f"  {axle} axle falls from hip x {fall_after[axle] * 1e3:.1f} mm "
                                f"(landings {[round(v * 1e3, 1) for v in landings]} mm; "
                                f"arc-end limit {rule.ground_after_stand_limit_hip_x(axle) * 1e3:.1f} mm)")
                            continue
                    fall_after[axle] = rule.ground_after_stand_limit_hip_x(axle)
                refreshed = axle_profiles_from_events_2d(
                    plan, spec, hold, axles, rise_m=rise, fall_ramp_m=float(fall_ramp_m),
                    pre_drop_m=float(pre_drop_m),
                    late_rise_m=float(late_rise_m), late_rise_ramp_m=float(late_rise_ramp_m),
                    late_rise_rear_m=None if late_rise_rear_m is None else float(late_rise_rear_m),
                    rear_prelift_m=float(rear_prelift_m),
                    rear_prelift_roller_m=float(rear_prelift_roller_m),
                    fall_after_hip_x_m=fall_after)

                def profile_gap(a: HipZProfile2D, b: HipZProfile2D) -> float:
                    grid = sorted(set(a.hip_x_m) | set(b.hip_x_m))
                    return max(abs(float(a.at(x)) - float(b.at(x))) for x in grid)

                moved = max(profile_gap(refreshed.front, axles.front),
                            profile_gap(refreshed.rear, axles.rear))
                log(f"  pass {index + 1}: axle profiles moved by {moved * 1e3:.2f} mm in z  "
                    f"(front knots {[round(v * 1e3, 1) for v in refreshed.front.hip_x_m]} mm, rear {[round(v * 1e3, 1) for v in refreshed.rear.hip_x_m]} mm)")
                result = (run, plan, rule, index + 1)
                if moved <= window_tolerance_m:
                    break
                axles = refreshed
        except PlannerRefusal2D as error:
            last_error = error
            log(f"  origin {origin * 1e3:.1f} mm refused: {str(error)[:160]}")
            continue
        return result
    raise PlannerRefusal2D(
        f"no origin among {[round(o * 1e3, 1) for o in origins]} mm plans the crossing; "
        f"last: {last_error}")
