"""Step 7 hardware export: one column reorder, one prep ramp, one CSV set.

The planner works in leg-major ``[FL, FR, RR, RL] x [theta, beta, gamma]``
order everywhere.  The hardware column order is applied exactly once, here, so
no other module reorders joints.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import PchipInterpolator

from legwheel.config import RobotParams
from legwheel.planners.obstacle_walk.traversal import ObstacleWalkResult


HARDWARE_COLUMN_NAMES = (
    "FL_theta",
    "FL_beta",
    "FR_theta",
    "FR_beta",
    "RR_theta",
    "RR_beta",
    "RL_theta",
    "RL_beta",
    "FL_gamma",
    "FR_gamma",
    "RR_gamma",
    "RL_gamma",
)
PHASE_COLUMN_NAMES = ("FL_Phase", "FR_Phase", "RR_Phase", "RL_Phase")
CONTROLLER_DT_S = 0.001
CONTROLLER_TRANSFORM_ROWS = 5000
CONTROLLER_TRANSFORM_DURATION_S = CONTROLLER_DT_S * CONTROLLER_TRANSFORM_ROWS
# planner column index -> hardware column index, matching
# examples/gait/generate_hardware_csv.py::_to_hw_order exactly.
_PLANNER_TO_HARDWARE = (0, 1, 8, 2, 3, 9, 4, 5, 10, 6, 7, 11)


def to_hardware_order(planner_commands_rad: NDArray[np.float64]) -> NDArray[np.float64]:
    """Reorder ``(N, 12)`` planner columns into the hardware CSV order."""

    commands = np.asarray(planner_commands_rad, dtype=float)
    if commands.ndim != 2 or commands.shape[1] != 12:
        raise ValueError(f"planner commands must have shape (N, 12); got {commands.shape}.")
    if not np.all(np.isfinite(commands)):
        raise ValueError("planner commands must contain only finite values.")
    hardware = np.empty_like(commands)
    for planner_column, hardware_column in enumerate(_PLANNER_TO_HARDWARE):
        hardware[:, hardware_column] = commands[:, planner_column]
    return hardware


def build_prep_rows(
    first_hardware_frame: NDArray[np.float64],
    *,
    dt_s: float,
    prep_duration_s: float = 5.0,
    home_theta_deg: float = RobotParams.THETA0_DEG,
) -> NDArray[np.float64]:
    """Cosine ramp from the folded home pose to the trajectory's first frame.

    This mirrors the prep block of ``generate_hardware_csv.py`` so downstream
    consumers see the same startup they already expect.  It appears once, at
    the very front of the file, never per segment.
    """

    frame = np.asarray(first_hardware_frame, dtype=float)
    if frame.shape != (12,) or not np.all(np.isfinite(frame)):
        raise ValueError("first_hardware_frame must contain twelve finite values.")
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive.")
    if not np.isfinite(prep_duration_s) or prep_duration_s < 0.0:
        raise ValueError("prep_duration_s must be finite and non-negative.")
    count = int(round(prep_duration_s / dt_s))
    if count == 0:
        return np.empty((0, 12), dtype=float)
    home = np.zeros(12, dtype=float)
    home[[0, 2, 4, 6]] = np.deg2rad(home_theta_deg)
    blend = (0.5 * (1.0 - np.cos(np.pi * np.linspace(0.0, 1.0, count))))[:, np.newaxis]
    return (1.0 - blend) * home + blend * frame


def resample_for_csv_controller(
    hardware_commands: NDArray[np.float64],
    phase: NDArray[np.int8],
    *,
    planner_dt_s: float,
    controller_dt_s: float = CONTROLLER_DT_S,
) -> tuple[NDArray[np.float64], NDArray[np.int8], int]:
    """Resample planner rows to the fixed-rate CSV controller contract.

    The ROS2 controller publishes one row every millisecond and has no way to
    read a sample period from the CSV.  Planning may use a coarser integral
    multiple of that period, but hardware export must therefore resample every
    joint command to 1 kHz.  PCHIP is component-wise shape preserving and
    reproduces every planner knot exactly without the overshoot of a generic
    cubic spline.

    Returns ``(commands, phase, ratio)`` where ``ratio`` is the integer number
    of controller intervals per planner interval.
    """

    commands = np.asarray(hardware_commands, dtype=float)
    phases = np.asarray(phase, dtype=np.int8)
    if commands.ndim != 2 or commands.shape[1] != 12 or len(commands) < 2:
        raise ValueError("hardware_commands must have shape (N, 12) with N >= 2.")
    if phases.shape != (len(commands), 4):
        raise ValueError("phase must have shape (N, 4) aligned with hardware_commands.")
    if not np.all(np.isfinite(commands)):
        raise ValueError("hardware_commands must contain only finite values.")
    if not np.isfinite(planner_dt_s) or planner_dt_s <= 0.0:
        raise ValueError("planner_dt_s must be finite and positive.")
    if not np.isfinite(controller_dt_s) or controller_dt_s <= 0.0:
        raise ValueError("controller_dt_s must be finite and positive.")

    ratio_float = planner_dt_s / controller_dt_s
    ratio = int(round(ratio_float))
    if ratio < 1 or not np.isclose(ratio_float, ratio, rtol=0.0, atol=1e-9):
        raise ValueError(
            "planner dt must be an integer multiple of the 1 ms CSV controller period; "
            f"got planner_dt_s={planner_dt_s:g}, controller_dt_s={controller_dt_s:g}."
        )
    if ratio == 1:
        return commands.copy(), phases.copy(), ratio

    planner_time = np.arange(len(commands), dtype=float) * planner_dt_s
    controller_count = (len(commands) - 1) * ratio + 1
    controller_time = np.arange(controller_count, dtype=float) * controller_dt_s
    controller_time[-1] = planner_time[-1]
    resampled_commands = np.asarray(
        PchipInterpolator(planner_time, commands, axis=0)(controller_time), dtype=float
    )
    # A phase is a discrete interval label, not an interpolated quantity.  Each
    # planner interval keeps its left-hand phase; the final knot is appended.
    resampled_phase = np.vstack(
        [np.repeat(phases[:-1], ratio, axis=0), phases[-1:]]
    ).astype(np.int8, copy=False)
    return resampled_commands, resampled_phase, ratio


@dataclass(frozen=True)
class ExportPaths:
    csv_path: Path
    phase_csv_path: Path
    metadata_path: Path
    validation_path: Path
    prep_row_count: int
    trajectory_row_count: int
    total_row_count: int


def _joint_rate_summary(commands_rad: NDArray[np.float64], dt_s: float) -> dict[str, object]:
    velocity = np.diff(commands_rad, axis=0) / dt_s
    acceleration = np.diff(velocity, axis=0) / dt_s if len(velocity) > 1 else velocity[:0]
    column = int(np.argmax(np.max(np.abs(velocity), axis=0))) if len(velocity) else 0
    return {
        "maximum_abs_joint_velocity_rad_s": float(np.max(np.abs(velocity)))
        if len(velocity)
        else 0.0,
        "maximum_abs_joint_velocity_column": HARDWARE_COLUMN_NAMES[column],
        "maximum_abs_joint_acceleration_rad_s2": float(np.max(np.abs(acceleration)))
        if len(acceleration)
        else 0.0,
    }


def _metadata(
    result: ObstacleWalkResult,
    prep_rows: int,
    *,
    planner_dt_s: float,
    controller_dt_s: float,
    resample_ratio: int,
    trajectory_rows: int,
) -> dict[str, object]:
    request = result.request
    obstacle = result.terrain.obstacle
    return {
        "generator": "legwheel.planners.obstacle_walk.traversal.generate_obstacle_walk",
        "status": "offline kinematic obstacle-walk trajectory; not hardware validated",
        "motion_model": {
            "gait_source": "GaitGenerator3D Walk phase offsets and leg kinematics",
            "swing_order": [leg.value for leg in result.swing_order],
            "stance_contact_model": result.stance_contact_model,
            "body_pose_policy": result.body_pose_policy,
            "body_is_level": True,
            "body_at_rest_at_every_stance_swing_boundary": (
                request.flat_approach_cycles == 0 and request.flat_recovery_cycles == 0
            ),
            "periodic_rolling_walk_spliced": (
                request.flat_approach_cycles > 0 or request.flat_recovery_cycles > 0
            ),
            "periodic_rolling_walk_cycles": request.flat_approach_cycles,
            "periodic_rolling_walk_recovery_cycles": request.flat_recovery_cycles,
            "recovery_handover_joint_snap_rad": result.recovery_handover_joint_snap_rad,
            "recovery_handover_contact_snap_m": result.recovery_handover_contact_snap_m,
            "recovery_handover_snap_note": (
                "the crawl's last four swings use the recovery Walk footholds.  A "
                "bounded all-stance blend then reconciles the remaining lowest-rim "
                "IK-branch difference over the whole settling segment; the value "
                "reported here is the largest temporary contact motion in that blend"
            ),
            "periodic_rolling_walk_launch_cycles": (
                request.flat_launch_cycles if request.flat_approach_cycles else 0
            ),
            "periodic_rolling_walk_note": (
                "the periodic Walk rolls its stance contact while this trajectory holds "
                "contacts world-fixed, so the two stance laws cannot share an arbitrary "
                "boundary without a joint velocity step"
                if request.flat_approach_cycles == 0
                else "the leading segment is the unmodified periodic Walk; it hands over "
                "at a liftoff sample, where all four rim contacts are still on the "
                "ground, and the crawl's first stance starts at the Walk's body "
                "velocity and brakes to rest.  Its first cycles come directly from "
                "the repository's original LaunchController (10%-to-100% velocity "
                "ramp by default), followed by the original steady Walk"
            ),
        },
        "request": {
            "obstacle_x_start_m": request.obstacle_x_start_m,
            "obstacle_length_m": request.obstacle_length_m,
            "obstacle_width_m": request.obstacle_width_m,
            "obstacle_width_note": (
                "lateral extent for the scene box only; the sagittal 2-D planner "
                "never reads it and no joint command depends on it"
            ),
            "obstacle_height_m": request.obstacle_height_m,
            "edge_margin_m": request.edge_margin_m,
            "stand_height_m": request.stand_height_m,
            "step_length_m": request.step_length_m,
            "flat_walk_velocity_m_s": request.forward_velocity_m_s,
            "flat_walk_step_height_m": request.flat_walk_step_height_m,
            "period_s": request.period_s,
            "stance_duty": request.stance_duty,
            "crawl_swing_seconds": request.crawl_swing_seconds,
            "flat_walk_dt_s": request.flat_walk_dt_s,
            "dt_s": request.dt_s,
            "step_clearance_m": request.step_clearance_m,
            "body_lift_ratio": request.body_lift_ratio,
            "approach_distance_m": request.approach_distance_m,
            "post_distance_m": request.post_distance_m,
            "maximum_touchdown_bias_m": request.maximum_touchdown_bias_m,
            "ground_face_safety_m": request.ground_face_safety_m,
            "top_edge_safety_m": request.top_edge_safety_m,
            "body_advance_fractions": list(request.body_advance_fractions),
            "joint_velocity_limit_rad_s": request.joint_velocity_limit_rad_s,
            "joint_limit_margin_rad": request.joint_limit_margin_rad,
            "maximum_lateral_sway_m": request.maximum_lateral_sway_m,
            "required_stability_margin_m": request.required_stability_margin_m,
            "lateral_sway_candidates": request.lateral_sway_candidates,
            "swing_duration_scales": list(request.swing_duration_scales),
            "contact_drift_tolerance_m": request.contact_drift_tolerance_m,
            "tracking_tolerance_m": request.tracking_tolerance_m,
            "maximum_events": request.maximum_events,
            "flat_approach_cycles": request.flat_approach_cycles,
            "flat_launch_cycles": request.flat_launch_cycles,
            "flat_launch_ramp_floor": request.flat_launch_ramp_floor,
            "flat_launch_mode": request.flat_launch_mode,
            "flat_recovery_cycles": request.flat_recovery_cycles,
            "flat_recovery_launch_cycles": request.flat_recovery_launch_cycles,
            "flat_landing_cycles": request.flat_landing_cycles,
        },
        "derived": {
            "forward_velocity_m_s": result.forward_velocity_m_s,
            "wheel_outer_radius_m": result.wheel_outer_radius_m,
            "wheel_face_exclusion_m": result.wheel_face_exclusion_m,
            "beta_limit_rad": result.beta_limit_rad,
            "beta_limit_meaning": (
                "sagittal range over which the foot rim can still be the contact; a "
                "Walk never rolls onto the upper tyre rims, so this is a hard "
                "feasibility bound for this gait"
            ),
            "minimum_stride_for_obstacle_m": result.wheel_face_exclusion_m
            + request.edge_margin_m,
            "legal_top_x_range_m": [obstacle.legal_top_x_min_m, obstacle.legal_top_x_max_m],
            "obstacle_x_end_m": obstacle.x_end_m,
            "obstacle_top_height_world_m": result.terrain.ground_height_m
            + obstacle.height_m,
        },
        "rows": {
            "prep_rows": prep_rows,
            "transform_rows_expected_by_controller": CONTROLLER_TRANSFORM_ROWS,
            "trajectory_start_row": prep_rows,
            "planner_trajectory_rows": result.segment.sample_count,
            "trajectory_rows": trajectory_rows,
            "total_rows": prep_rows + trajectory_rows,
            "planner_dt_s": planner_dt_s,
            "controller_dt_s": controller_dt_s,
            "dt_s": controller_dt_s,
            "resample_ratio": resample_ratio,
            "controller_rate_hz": 1.0 / controller_dt_s,
            "prep_appears_once": True,
            "trigger_waits_after_prep": True,
            "hardware_column_names": list(HARDWARE_COLUMN_NAMES),
            "phase_column_names": list(PHASE_COLUMN_NAMES),
        },
        "step6_reference_schedule": {
            "note": (
                "nominal periodic-Walk event prediction from Step 6, kept for "
                "comparison only; the executed crawl re-derives every touchdown "
                "from the previous actual final state and from the wheel-face "
                "foothold rule, so its footholds and timing differ by design"
            ),
            "error": result.reference_schedule_error,
            "schedule": result.reference_schedule,
        },
        "segments": [
            {
                "index": record.index,
                "kind": record.kind,
                "stage": record.stage.value,
                "event_index": record.event_index,
                "leg": None if record.leg is None else record.leg.value,
                "csv_start_row": record.start_row * resample_ratio + prep_rows,
                "csv_end_row": record.end_row * resample_ratio + prep_rows,
                "trajectory_start_row": record.start_row,
                "trajectory_end_row": record.end_row,
                "sample_count": record.sample_count,
                "csv_sample_count": (
                    (record.end_row - record.start_row) * resample_ratio + 1
                ),
                "body_pose_start_world": list(record.body_pose_start_world),
                "body_pose_end_world": list(record.body_pose_end_world),
                "body_advance_m": record.body_advance_m,
                "touchdown_world_m": None
                if record.touchdown_world_m is None
                else list(record.touchdown_world_m),
                "touchdown_surface_id": record.touchdown_surface_id,
                "touchdown_bias_x_m": record.touchdown_bias_x_m,
                "from_surface_id": record.from_surface_id,
                "maximum_contact_drift_m": record.maximum_contact_drift_m,
                "maximum_tracking_error_m": record.maximum_tracking_error_m,
                "requested_apex_height_world_m": record.requested_apex_height_world_m,
                "achieved_apex_height_world_m": record.achieved_apex_height_world_m,
                "rejected_candidates": list(record.rejected_candidates),
            }
            for record in result.records
        ],
    }


def _validation_report(
    result: ObstacleWalkResult,
    hardware_commands: NDArray[np.float64],
    dt_s: float,
) -> dict[str, object]:
    boundaries = [
        {
            "left_segment_index": report.left_segment_index,
            "right_segment_index": report.right_segment_index,
            "joint_position_max_rad": report.joint_position_max_rad,
            "joint_position_leg": report.joint_position_leg.value,
            "joint_position_joint": report.joint_position_joint,
            "joint_velocity_max_rad_s": report.joint_velocity_max_rad_s,
            "joint_velocity_leg": report.joint_velocity_leg.value,
            "joint_velocity_joint": report.joint_velocity_joint,
            "body_position_max_m": report.body_position_max_m,
            "body_orientation_max_rad": report.body_orientation_max_rad,
            "support_foot_position_max_m": report.support_foot_position_max_m,
            "phase_match": report.phase_match,
            "contact_active_match": report.contact_active_match,
            "active_surface_match": report.active_surface_match,
            "passed": report.passed,
            "violations": list(report.violations),
        }
        for report in result.boundary_reports
    ]
    # The crawl's stance and swing enforce their own per-sample joint step, but
    # a spliced flat-Walk segment passes through neither, so the only place the
    # whole trajectory is ever inspected is here.  Reporting a literal True was
    # a claim about a check that had not been made.
    planner_steps = np.abs(np.diff(result.segment.commands_rad, axis=0))
    planner_step_limit = (
        result.request.joint_velocity_limit_rad_s * result.segment.dt_s
    )
    worst_planner_step = float(planner_steps.max()) if len(planner_steps) else 0.0
    step_row = int(np.argmax(planner_steps.max(axis=(1, 2)))) + 1 if len(planner_steps) else 0
    return {
        "checked": {
            "segment_boundary_continuity": True,
            "terrain_touchdown_legality": True,
            "wheel_face_ground_exclusion": True,
            "world_fixed_contact_drift": True,
            "recovery_handover_blend_bounded": True,
            "single_leg_inverse_kinematics": True,
            "joint_position_limits": True,
            "per_sample_joint_step": worst_planner_step <= planner_step_limit,
            "joint_limit_margin_headroom": True,
            "tracked_material_point_vs_obstacle_top": True,
        },
        "not_checked": {
            "full_leg_wheel_body_collision_geometry": not result
            .full_geometry_collision_checked,
            "support_polygon_and_quasi_static_stability": True,
            "contact_forces_friction_and_slip": True,
            "actuator_torque_and_hardware_limits": True,
            "simulation_or_hardware_execution": True,
        },
        "stages": dict(result.stage_results),
        "traversal": {
            "legs_that_reached_top": [leg.value for leg in result.legs_that_reached_top],
            "legs_that_returned_to_ground": [
                leg.value for leg in result.legs_that_returned_to_ground
            ],
            "maximum_simultaneous_top_contacts": result.maximum_top_contact_count,
            "all_four_top_observed": result.all_four_top_observed,
            "traversal_completed": result.traversal_completed,
            "recovery_distance_m": result.recovery_distance_m,
        },
        "worst_case": {
            "planner_per_sample_joint_step_rad": worst_planner_step,
            "planner_per_sample_joint_step_limit_rad": planner_step_limit,
            "planner_per_sample_joint_step_row": step_row,
            "boundary_joint_position_error_rad": (
                result.maximum_boundary_joint_position_error_rad
            ),
            "boundary_joint_velocity_error_rad_s": (
                result.maximum_boundary_joint_velocity_error_rad_s
            ),
            "world_fixed_contact_drift_m": result.maximum_contact_drift_m,
            "recovery_handover_blend_contact_motion_m": (
                result.recovery_handover_contact_snap_m
            ),
            "recovery_handover_blend_joint_correction_rad": (
                result.recovery_handover_joint_snap_rad
            ),
            "maximum_abs_beta_rad": result.maximum_abs_beta_rad,
            "maximum_abs_beta_deg": float(np.rad2deg(result.maximum_abs_beta_rad)),
            "beta_limit_deg": float(np.rad2deg(result.beta_limit_rad)),
            "swing_cartesian_tracking_error_m": result.maximum_tracking_error_m,
            **_joint_rate_summary(hardware_commands, dt_s),
        },
        "boundary_reports": boundaries,
        "all_boundaries_passed": all(report.passed for report in result.boundary_reports),
    }


def write_obstacle_walk_csv(
    result: ObstacleWalkResult,
    output_path: str | Path,
    *,
    prep_duration_s: float = CONTROLLER_TRANSFORM_DURATION_S,
    home_theta_deg: float = RobotParams.THETA0_DEG,
) -> ExportPaths:
    """Write the 12-column CSV, the row-aligned phase CSV, metadata and report."""

    if not isinstance(result, ObstacleWalkResult):
        raise TypeError("result must be an ObstacleWalkResult.")
    path = Path(output_path)
    if path.suffix.lower() != ".csv":
        raise ValueError("output_path must end with .csv")
    path.parent.mkdir(parents=True, exist_ok=True)

    planner_dt_s = float(result.segment.dt_s)
    if not np.isclose(
        prep_duration_s, CONTROLLER_TRANSFORM_DURATION_S, rtol=0.0, atol=1e-12
    ):
        raise ValueError(
            "corgi_csv_control consumes exactly 5000 transform rows at 1 kHz; "
            f"prep_duration_s must be {CONTROLLER_TRANSFORM_DURATION_S:g}."
        )
    planner_trajectory = to_hardware_order(result.segment.to_planner_commands())
    trajectory, trajectory_phase, resample_ratio = resample_for_csv_controller(
        planner_trajectory,
        result.segment.to_phase_array(),
        planner_dt_s=planner_dt_s,
    )
    prep = build_prep_rows(
        trajectory[0],
        dt_s=CONTROLLER_DT_S,
        prep_duration_s=prep_duration_s,
        home_theta_deg=home_theta_deg,
    )
    if len(prep) != CONTROLLER_TRANSFORM_ROWS:
        raise RuntimeError(
            f"controller transform contract requires {CONTROLLER_TRANSFORM_ROWS} rows; "
            f"built {len(prep)}."
        )
    commands = np.vstack([prep, trajectory]) if len(prep) else trajectory
    phase = np.vstack(
        [np.zeros((len(prep), 4), dtype=np.int8), trajectory_phase]
    )
    if len(phase) != len(commands):
        raise RuntimeError("phase rows and command rows are not aligned.")

    np.savetxt(path, commands, delimiter=",", fmt="%.6f")
    phase_path = path.with_name(path.stem + "_phase.csv")
    np.savetxt(
        phase_path,
        phase,
        delimiter=",",
        fmt="%.0f",
        header=",".join(PHASE_COLUMN_NAMES),
        comments="",
    )
    metadata_path = path.with_name(path.stem + "_metadata.json")
    metadata_path.write_text(
        json.dumps(
            _metadata(
                result,
                len(prep),
                planner_dt_s=planner_dt_s,
                controller_dt_s=CONTROLLER_DT_S,
                resample_ratio=resample_ratio,
                trajectory_rows=len(trajectory),
            ),
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    validation_path = path.with_name(path.stem + "_validation.json")
    validation_path.write_text(
        json.dumps(_validation_report(result, commands, CONTROLLER_DT_S), indent=2) + "\n",
        encoding="utf-8",
    )
    return ExportPaths(
        csv_path=path,
        phase_csv_path=phase_path,
        metadata_path=metadata_path,
        validation_path=validation_path,
        prep_row_count=len(prep),
        trajectory_row_count=len(trajectory),
        total_row_count=len(commands),
    )
