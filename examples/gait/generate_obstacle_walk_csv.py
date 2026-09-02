"""Generate one offline obstacle-walk hardware CSV for a known rectangle.

Example
-------
    .venv/bin/python examples/gait/generate_obstacle_walk_csv.py \
      --obstacle-x 0.65 \
      --obstacle-length 0.35 \
      --obstacle-height 0.06 \
      --step-length 0.15 \
      --period 2.0 \
      --dt 0.001 \
      --output outputs/csv/obstacle_walk.csv

Writes the 12-column hardware CSV (no header), a row-aligned ``_phase.csv``,
``_metadata.json`` and ``_validation.json``.  The prep ramp appears once, at
the front of the main CSV only.

By default the flat approach and recovery use the existing periodic rolling
Walk, while the obstacle neighbourhood uses the quasi-static crawl and its
Cartesian Bezier swings.  ``--flat-approach-cycles 0`` or
``--flat-recovery-cycles 0`` keeps the corresponding section as crawl.
``--flat-launch-cycles`` calls the same velocity-scaled ``LaunchController``
cycles used by ``generate_hardware_csv.py``.  That legacy launch is preserved
exactly; it does not guarantee zero joint velocity immediately after trigger.

It is an offline geometric and kinematic result: no simulation, contact-force,
stability or hardware validation is claimed.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from pathlib import Path

from legwheel.planners.obstacle_walk.export import (
    CONTROLLER_DT_S,
    CONTROLLER_TRANSFORM_DURATION_S,
    CONTROLLER_TRANSFORM_ROWS,
    write_obstacle_walk_csv,
)
from legwheel.planners.obstacle_walk.traversal import (
    ObstacleTraversalError,
    ObstacleWalkRequest,
    generate_obstacle_walk,
)


DEFAULT_FLAT_WALK_VELOCITY_M_S = 0.1
DEFAULT_CRAWL_STEP_LENGTH_M = 0.15


def build_request(args: argparse.Namespace) -> ObstacleWalkRequest:
    launch_cycles = args.flat_launch_cycles
    if launch_cycles is None:
        # The time warp re-times cycles that are already counted in
        # --flat-approach-cycles, so it must not ask for more than exist.
        launch_cycles = (
            min(1, args.flat_approach_cycles)
            if args.flat_launch_mode == "timewarp"
            else 3
        )
    step_length = (
        DEFAULT_CRAWL_STEP_LENGTH_M
        if args.step_length is None
        else args.step_length
    )
    flat_walk_velocity = (
        DEFAULT_FLAT_WALK_VELOCITY_M_S if args.vx is None else args.vx
    )
    return ObstacleWalkRequest(
        obstacle_x_start_m=args.obstacle_x,
        obstacle_length_m=args.obstacle_length,
        obstacle_width_m=args.obstacle_width,
        obstacle_height_m=args.obstacle_height,
        edge_margin_m=args.edge_margin,
        stand_height_m=args.stand_height,
        step_length_m=step_length,
        period_s=args.period,
        flat_walk_velocity_m_s=flat_walk_velocity,
        flat_walk_step_height_m=args.flat_step_height,
        dt_s=args.dt,
        step_clearance_m=args.step_clearance,
        body_lift_ratio=args.body_lift_ratio,
        approach_distance_m=args.approach_distance,
        post_distance_m=args.post_distance,
        maximum_touchdown_bias_m=args.max_touchdown_bias,
        maximum_lateral_sway_m=args.max_lateral_sway,
        required_stability_margin_m=args.required_stability_margin,
        lateral_sway_candidates=args.lateral_sway_candidates,
        joint_velocity_limit_rad_s=args.joint_velocity_limit,
        joint_limit_margin_rad=args.joint_limit_margin,
        maximum_events=args.max_events,
        flat_approach_cycles=args.flat_approach_cycles,
        flat_launch_cycles=launch_cycles,
        flat_launch_ramp_floor=args.flat_launch_ramp_floor,
        flat_launch_mode=args.flat_launch_mode,
        flat_recovery_cycles=args.flat_recovery_cycles,
        flat_recovery_launch_cycles=args.flat_recovery_launch_cycles,
        flat_landing_cycles=args.flat_landing_cycles,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--obstacle-x", type=float, default=1.0,
                        help="world x of the obstacle front face (m)")
    parser.add_argument("--obstacle-length", type=float, default=0.4,
                        help="obstacle extent along the direction of travel (m)")
    parser.add_argument("--obstacle-width", type=float, default=0.8,
                        help="lateral obstacle extent (m).  The planner is sagittal "
                             "2-D, so this changes no joint command; it is recorded in "
                             "the metadata so the Webots/scene box can be built to match")
    parser.add_argument("--obstacle-height", type=float, default=0.04)
    parser.add_argument("--edge-margin", type=float, default=0.02,
                        help="top-surface support margin kept clear of both edges (m)")
    parser.add_argument("--stand-height", type=float, default=0.25)
    parser.add_argument("--step-length", type=float, default=None,
                        help="obstacle-crawl stride (m), independent of flat Walk "
                             "speed; default 0.15")
    parser.add_argument("--vx", type=float, default=None,
                        help="flat Walk forward velocity (m/s), independent of the "
                             "obstacle-crawl stride; default 0.03")
    parser.add_argument("--period", type=float, default=2.0,
                        help="Walk period (s); the known flat baseline uses 4 s and "
                             "the crawl reuses its stance/swing timing")
    parser.add_argument("--flat-step-height", type=float, default=0.04,
                        help="existing flat Walk swing height (m), independent of "
                             "the obstacle Bezier --step-clearance; default 0.04 "
                             "matches generate_hardware_csv.py")
    parser.add_argument("--dt", type=float, default=0.02,
                        help="planner sample period (s); must be an integer multiple "
                             "of the 1 ms controller period, which the exporter "
                             "resamples to")
    parser.add_argument("--step-clearance", type=float, default=0.03,
                        help="swing apex above the tallest terrain in the corridor; "
                             "0.03 is the geometry-checked nominal value; 0.02 was "
                             "measured to let the rim clip the top-front corner")
    parser.add_argument("--body-lift-ratio", type=float, default=0.6)
    parser.add_argument("--approach-distance", type=float, default=1.0,
                        help="body-origin distance to the obstacle front face at start "
                             "(m).  The flat Walk covers its front portion and the crawl "
                             "the rest, so the start pose does not move when the split "
                             "changes")
    parser.add_argument("--post-distance", type=float, default=0.30)
    parser.add_argument("--max-touchdown-bias", type=float, default=0.16)
    parser.add_argument(
        "--max-lateral-sway",
        type=float,
        default=0.09,
        help="maximum temporary body-y shift used to obtain three-leg support margin (m)",
    )
    parser.add_argument(
        "--required-stability-margin",
        type=float,
        default=0.02,
        help="required sampled support-polygon margin (m); 0 permits straight no-sway trials",
    )
    parser.add_argument(
        "--lateral-sway-candidates",
        type=int,
        default=33,
        help="number of body-y candidates; lower values give a faster coarse search",
    )
    parser.add_argument("--joint-velocity-limit", type=float, default=16.0,
                        help="planning guard on per-sample joint step, not a hardware limit")
    parser.add_argument("--joint-limit-margin", type=float, default=0.02,
                        help="joint-limit headroom the search keeps in hand (rad)")
    parser.add_argument("--max-events", type=int, default=400)
    parser.add_argument(
        "--flat-approach-cycles",
        type=int,
        default=3,
        help="whole periods of the existing periodic flat Walk walked before the "
             "crawl takes over.  They consume the front of --approach-distance, "
             "so each cycle covers v_x * period metres that the crawl no longer "
             "walks.  0 keeps the pure crawl the planner produced before",
    )
    parser.add_argument(
        "--flat-launch-cycles",
        type=int,
        default=None,
        help="how many of those periods are replayed under the starting time warp "
             "(legacy wording): now the number of original LaunchController "
             "velocity-ramp cycles prepended before the steady flat Walk; default 3",
    )
    parser.add_argument(
        "--flat-launch-mode",
        choices=("timewarp", "legacy"),
        default="timewarp",
        help="how the flat Walk starts from rest.  'timewarp' replays the Walk's own "
             "joint path against a clock that ramps up from rest, so it covers exactly "
             "--flat-approach-cycles * v_x * period and stays continuous.  'legacy' "
             "uses the repository LaunchController, which walks extra reduced-speed "
             "cycles and steps the joints at every ramp-cycle boundary",
    )
    parser.add_argument(
        "--flat-launch-ramp-floor",
        type=float,
        default=0.1,
        help="first LaunchController cycle velocity fraction; default 0.1",
    )
    parser.add_argument(
        "--flat-recovery-cycles",
        type=int,
        default=3,
        help="whole periods of the periodic flat Walk that replace the post-obstacle "
             "crawl.  When non-zero this sets the recovery distance instead of "
             "--post-distance, and the crawl only walks far enough past the obstacle "
             "for the Walk's footprint to be legal ground.  0 keeps the crawl",
    )
    parser.add_argument(
        "--flat-landing-cycles",
        type=int,
        default=1,
        help="periods at the very end replayed under the closing time warp, so the "
             "file finishes stationary the way the pure crawl always did",
    )
    parser.add_argument(
        "--flat-recovery-launch-cycles",
        type=int,
        default=1,
        help="custom recovery Walk ramp cycles; independent of the original "
             "three-cycle approach LaunchController",
    )
    parser.add_argument(
        "--prep-seconds",
        type=float,
        default=CONTROLLER_TRANSFORM_DURATION_S,
        help="fixed 5 s / 5000-row transform consumed before trigger by corgi_csv_control",
    )
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("outputs/csv/obstacle_walk.csv"))
    args = parser.parse_args(argv)

    request = build_request(args)
    print("Generating offline obstacle-walk trajectory")
    print(f"  obstacle      : x={request.obstacle_x_start_m:.3f} m "
          f"length={request.obstacle_length_m:.3f} m "
          f"height={request.obstacle_height_m:.3f} m")
    print(f"  stride        : {request.step_length_m:.4f} m "
          f"(Walk v_x {request.forward_velocity_m_s:.4f} m/s at T={request.period_s:.2f} s)")
    print(f"  dt            : {request.dt_s} s")
    try:
        result = generate_obstacle_walk(request)
    except ObstacleTraversalError as error:
        print(f"\n[REJECTED] {error}", file=sys.stderr)
        return 1

    try:
        paths = write_obstacle_walk_csv(
            result, args.output, prep_duration_s=args.prep_seconds
        )
    except ValueError as error:
        print(f"\n[EXPORT REJECTED] {error}", file=sys.stderr)
        return 1
    print("\n[RESULT]")
    print(f"  swing order            : {[leg.value for leg in result.swing_order]}")
    print(f"  wheel-face exclusion   : {result.wheel_face_exclusion_m*1e3:.1f} mm")
    print(f"  legs that reached top  : {[leg.value for leg in result.legs_that_reached_top]}")
    print(f"  legs back on ground    : "
          f"{[leg.value for leg in result.legs_that_returned_to_ground]}")
    print(f"  max simultaneous top   : {result.maximum_top_contact_count}")
    print(f"  traversal completed    : {result.traversal_completed}")
    for stage, passed in result.stage_results.items():
        print(f"    stage {stage:<12}: {passed}")
    print(f"  max boundary q error   : "
          f"{result.maximum_boundary_joint_position_error_rad:.3e} rad")
    print(f"  max boundary qd error  : "
          f"{result.maximum_boundary_joint_velocity_error_rad_s:.4f} rad/s")
    print(f"  max contact drift      : {result.maximum_contact_drift_m*1e3:.3f} mm")
    print(f"  max IK/FK swing error  : {result.maximum_tracking_error_m*1e3:.3f} mm")
    if result.recovery_handover_joint_snap_rad is not None:
        print(f"  recovery blend q       : "
              f"{result.recovery_handover_joint_snap_rad:.4f} rad")
        print(f"  recovery blend contact : "
              f"{result.recovery_handover_contact_snap_m*1e3:.3f} mm")
    print(f"  foot-rim beta bound    : "
          f"peak {np.rad2deg(result.maximum_abs_beta_rad):.1f} deg "
          f"of {np.rad2deg(result.beta_limit_rad):.1f} deg")
    print(f"  full geometry checked  : {result.full_geometry_collision_checked}")
    print("\n[FILES]")
    print(f"  prep rows       : {paths.prep_row_count}")
    print(f"  trajectory rows : {paths.trajectory_row_count}")
    print(f"  total rows      : {paths.total_row_count}")
    print(f"  controller dt   : {CONTROLLER_DT_S:.3f} s")
    print(f"  trigger boundary: row {CONTROLLER_TRANSFORM_ROWS}")
    print(f"  csv        : {paths.csv_path}")
    print(f"  phase      : {paths.phase_csv_path}")
    print(f"  metadata   : {paths.metadata_path}")
    print(f"  validation : {paths.validation_path}")
    print("\nOffline kinematic result only: no simulation, contact-force, stability")
    print("or hardware validation is claimed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
