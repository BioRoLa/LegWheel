"""Day 14 Step 4: the Day 6--7 climb as a **stance stroke** of the gait.

The right-rim roll-up (Step 4.5) keeps the leg on the ground and then on the
block's front corner the whole way up: it is stance, not a swing, so in the
whole-body loop it is one long stroke -- ``APPROACH`` on the foot rim, then
``RIGHT_RIM_ROLL_UP`` rolling on the corner, then 20 mm of ``RIGHT_RIM_TOP``.
Its hip height is an **output** of the geometry (measured at 100 mm, theta
60 deg: 219.4 -> 271.2 mm over 150 mm of hip travel) and becomes the axle's
profile that the leg's partner follows.

What is measured (2026-09-08, 100 mm block, theta_climb 60 deg):

* the roll-up's start pose is the flat stance at theta 60, beta 0 and the
  held hip height; a nominal-style recovery lands there exactly
  (``run_recovery_swing_2d`` with ``beta_target`` one turn on and its own
  touchdown);
* every frame of the climb is a valid Day 12 standing pose (to +0.5 mm of
  hip height, not +1 mm), so the frames are used as they are;
* from the top-of-climb pose a nominal-style swing lands in the nominal pose
  on the top with 25--28 mm of rotation clearance, at any landing tried
  (face + 80 .. + 250 mm hip).

So the rolling leg's crossing is: land at (60 deg, 0), roll up, swing to the
nominal pose on the top, and from there everything is Step 3.  The
wheel-mode top roll and the left-rim descent are **not** used: the descent
presses the axle to 143.8 mm, below the partner's shortest leg (145.0), and
the owner's direction is to descend by swinging.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    CycleFrame2D,
    NominalPosture2D,
    RollStroke2D,
    standing_stroke_2d,
)
from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (
    ObstacleSpec2D,
    TraversalConstraints2D,
    TraversalInitialState2D,
    check_right_up_left_down_traversal,
)

__all__ = [
    "RollUpStroke2D",
    "ROLL_UP_STOP_REASON",
    "APPROACH_WINDOW_M",
    "roll_up_landing_beta_rad",
    "roll_up_stroke_2d",
]

#: The stop reason a climb stroke carries; the rule keys on it.
ROLL_UP_STOP_REASON = "RIGHT_RIM_TOP"

#: Where the roll-up's start landing may be, as hip x short of the face: the
#: approach rolls forward at most 20 deg of beta (~75 mm) before the right rim
#: must touch the corner, and it touches with the hip 122.5 mm short of the
#: face (measured at 100 mm).
APPROACH_WINDOW_M: tuple[float, float] = (0.200, 0.105)


def roll_up_landing_beta_rad(takeoff_beta_rad: float) -> float:
    """The roll-up starts at beta = 0 (mod one turn), one forward turn on."""

    turns = np.floor(float(takeoff_beta_rad) / (2.0 * np.pi))
    target = turns * 2.0 * np.pi
    if target > float(takeoff_beta_rad) - 1e-9:
        target -= 2.0 * np.pi
    return float(target)


@dataclass(frozen=True)
class RollUpStroke2D:
    """The climb as a stroke, plus the numbers the rule wants from it."""

    stroke: RollStroke2D
    success: bool
    refusal: str | None
    stage_frame_counts: tuple
    hip_z_path: tuple  # (hip_x_m, hip_z_m) knots, the axle profile's rise
    theta_climb_rad: float


def roll_up_stroke_2d(spec: SharedTerrainSpec2D, posture: NominalPosture2D, *,
                      landing_hip_x_m: float, landing_beta_rad: float,
                      theta_climb_rad: float = float(np.deg2rad(60.0)),
                      constraints: TraversalConstraints2D | None = None) -> RollUpStroke2D:
    """Run the Day 6--7 traversal from the landing pose and keep its climb.

    The frames from the approach's first to the last ``RIGHT_RIM_TOP`` frame
    become one :class:`RollStroke2D`, world-registered (the traversal runs
    in world x directly).  Rims and contacts are the traversal's own.
    """

    obstacle = ObstacleSpec2D(x_start_m=float(spec.x_start_m), width_m=float(spec.top_length_m),
                              height_m=float(spec.height_m),
                              ground_height_m=float(spec.ground_height_m))
    if constraints is None:
        # The approach may rotate a little further than Day 6-7's 20 deg,
        # so a landing 105 mm short of the face still reaches the corner
        # (the rear pair's timing needs the front roller to land later).
        constraints = TraversalConstraints2D(approach_max_rotation_rad=float(np.deg2rad(26.0)))
    initial = TraversalInitialState2D(hip_x_m=float(landing_hip_x_m),
                                      beta_rad=float(landing_beta_rad), hip_z_m=None)
    result = check_right_up_left_down_traversal(
        obstacle, initial, theta_climb=float(theta_climb_rad), constraints=constraints)
    if not (result.approach_success and result.roll_up_success):
        return RollUpStroke2D(None, False,
                              f"{result.failure_stage}:{result.failure_reason}",
                              result.stage_frame_counts, (), float(theta_climb_rad))
    kept = [f for f in result.trajectory if f.stage in ("APPROACH", "ROLL_UP")]
    frames = []
    for k, f in enumerate(kept):
        frames.append(CycleFrame2D(
            index=k, phase=f.phase, theta_rad=float(f.theta_rad), beta_rad=float(f.beta_rad),
            hip_xz_m=(float(f.hip_position_world_xz_m[0]), float(f.hip_position_world_xz_m[1])),
            airborne=False, rim=f.active_rim, alpha_rad=f.alpha_rad,
            contact_xz_m=(None if f.contact_point_world_xz_m is None
                          else (float(f.contact_point_world_xz_m[0]),
                                float(f.contact_point_world_xz_m[1]))),
            surface_id=f.terrain_surface_id, clearance_m=0.0, collision=False))
    end = frames[-1]
    # The Day 12 scene of the end pose, for the stroke's final scene and
    # sample (the pose is a valid standing pose there, measured).
    stand = standing_stroke_2d(posture, float(end.theta_rad), float(end.beta_rad),
                               float(end.hip_xz_m[0]), float(end.hip_xz_m[1]))
    if not stand.success:
        return RollUpStroke2D(None, False,
                              f"CLIMB_END_POSE_NOT_A_DAY12_STANCE:{stand.stop_reason}",
                              result.stage_frame_counts, (), float(theta_climb_rad))
    stroke = replace(stand, frames=tuple(frames), success=True, stop_reason=ROLL_UP_STOP_REASON)
    path = tuple((float(f.hip_xz_m[0]), float(f.hip_xz_m[1])) for f in frames)
    return RollUpStroke2D(stroke, True, None, result.stage_frame_counts, path,
                          float(theta_climb_rad))
