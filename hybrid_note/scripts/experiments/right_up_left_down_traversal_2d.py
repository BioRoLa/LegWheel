"""Day 6--7 revised traversal: right-rim climb, wheel-mode handover, left-rim descent.

The target primitive is::

    GROUND
    -> RIGHT-RIM ROLL-UP
    -> RETRACT theta -> 17 deg          (Step 7R, this module)
    -> WHEEL-MODE FORWARD ROLL ON TOP   (Step 8R)
    -> LEFT-RIM READY
    -> LEFT-RIM ROLL-DOWN               (Step 9R)
    -> GROUND

Why the retract matters, in geometry rather than intent
-------------------------------------------------------
Each physical rim arc is a finite budget.  Forward rolling always migrates the
contact toward increasing global ``alpha``, so a traversal that stays on the
right rim has to pay for the leading-edge climb, the top roll and the trailing
corner out of one 136-degree arc, and runs out.

Retracting ``theta`` to 17 degrees folds the leg into a near-circular wheel:
every rim radius converges to about 0.1438 m and the unlabelled seam at
``alpha = 180 deg`` shrinks to roughly 3 mm of chord with a few micrometres of
dip.  Only then can the contact hand over from the right rim to the left rim
by rolling, which restores a fresh rim budget for the descent.

Reuse, not reimplementation
---------------------------
Step 6.5 already solved the hard part: coupled theta/beta continuation on the
obstacle top under a signed horizontal no-slip constraint, with an explicit
ordered rim contour and seam-bridge guard.  Step 7R is the *same* trajectory
cut at an earlier milestone, so this module drives
``run_retract_and_reset_branch_2d(..., stop_at="theta_target")`` instead of
simulating the motion a second time.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation

from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    ForwardRollingFrame2D,
    ForwardRollingResult2D,
    RetractResetBranchResult2D,
    RetractResetFrame2D,
    _translated_scene_with_sample_on_target_2d,
    animate_retract_and_reset_branch_2d,
    build_single_leg_rolling_scene_2d,
    candidate_status_2d,
    plot_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
    run_retract_and_reset_branch_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    RollDownFrame2D,
    RollDownStartState2D,
    _back_face_clearance,
    _candidate_for_sample,
    _ground_clearance,
    _lowest_contact_sample,
    _pivot_scene,
    _remaining_rim_arc_rad,
    _remaining_rim_budget_rad,
    _solve_flat_roll_rotation,
    animate_trailing_edge_roll_down_2d,
    load_or_build_roll_down_start_state,
    plot_roll_down_key_frames_2d,
    roll_down_frame_rows,
)

__all__ = [
    "RetractToWheelOnTopResult2D",
    "roll_up_result_from_top_contact_state",
    "load_or_build_roll_up_end_state",
    "run_retract_to_wheel_on_top_2d",
    "retract_to_wheel_on_top_frame_rows",
    "retract_to_wheel_on_top_summary_row",
    "write_retract_to_wheel_on_top_csv",
    "plot_retract_to_wheel_on_top_2d",
    "plot_retract_to_wheel_key_frames_2d",
    "animate_retract_to_wheel_on_top_2d",
    "WheelModeTransitionResult2D",
    "run_wheel_mode_transition_to_corner_2d",
    "wheel_mode_transition_summary_row",
    "wheel_mode_transition_frame_rows",
    "traversal_phase_label",
    "plot_left_rim_ready_frame_2d",
    "animate_wheel_mode_transition_2d",
    "corner_states_along_transition",
    "LeftRimRollDownResult2D",
    "run_left_rim_roll_down_2d",
    "left_rim_roll_down_frame_rows",
    "left_rim_roll_down_summary_row",
    "plot_left_rim_roll_down_key_frames_2d",
    "animate_left_rim_roll_down_2d",
]


def roll_up_result_from_top_contact_state(
    state: RollDownStartState2D,
) -> ForwardRollingResult2D:
    """Rebuild a one-frame roll-up result from a cached obstacle-top end state.

    Every downstream stage only reads the roll-up *final frame*, so the whole
    Step 4.5 simulation does not have to be replayed to continue from it.  The
    pose, the scene and the terrain-aware query are all regenerated here from
    the cached scalars rather than unpickled, which keeps the cache readable
    and stable across changes to the simulator.
    """

    if not isinstance(state, RollDownStartState2D):
        raise TypeError("state must be a RollDownStartState2D.")
    template = build_single_leg_rolling_scene_2d(
        state.theta_rad, state.beta_rad, 0.0, 0.0, **state.scene_kwargs
    )
    contact = np.asarray(state.contact_point_world_xz_m, dtype=float)
    scene = _translated_scene_with_sample_on_target_2d(
        template, state.active_sample_index, contact
    )
    query_result = query_single_leg_rolling_scene_2d(scene)
    candidates = [
        candidate
        for candidate in query_result.candidates
        if candidate.terrain_surface_id.endswith("_top")
    ]
    if not candidates:
        raise ValueError("the cached end state holds no obstacle-top contact.")
    candidate = min(
        candidates,
        key=lambda item: (
            abs(item.sample_index - state.active_sample_index),
            item.surface_distance_m,
        ),
    )
    if query_result.collision:
        raise ValueError("the cached end state is in collision.")
    hip = scene.hip_pose.position_world_xz_m
    frame = ForwardRollingFrame2D(
        step=0,
        hip_x_m=float(hip[0]),
        hip_forward_progress_m=0.0,
        theta_rad=float(scene.theta_rad),
        beta_rad=float(scene.beta_rad),
        active_rim=candidate.rim.value,
        alpha_rad=float(candidate.alpha_rad),
        contact_point_world_xz_m=tuple(
            float(value) for value in candidate.point_world_xz_m
        ),
        continuation_target_world_xz_m=tuple(float(value) for value in contact),
        continuation_error_m=0.0,
        leading_edge_clearance_m=None,
        top_roll_progress_m=0.0,
        top_contact_advance_m=0.0,
        top_roll_complete=True,
        top_roll_remaining_m=0.0,
        roll_phase="TOP_ROLL_COMPLETE",
        terrain_surface_id=candidate.terrain_surface_id,
        contact_phase="top",
        valid_contact=True,
        collision=False,
        status=candidate_status_2d(candidate, scene.terrain),
        accepted=True,
        failure_reason=None,
        scene=scene,
        query_result=query_result,
    )
    return ForwardRollingResult2D(
        candidate_theta_rad=float(scene.theta_rad),
        initial_beta_rad=float(scene.beta_rad),
        dx_m=0.0,
        top_roll_distance_m=0.0,
        frames=(frame,),
        success=True,
        failure_reason=None,
    )


def load_or_build_roll_up_end_state(
    path,
    *,
    force_rebuild: bool = False,
    **roll_up_kwargs,
) -> tuple[ForwardRollingResult2D, RollDownStartState2D, bool]:
    """Reuse a cached roll-up end state, running Step 4.5 only when needed.

    Returns a one-frame roll-up result ready for the retract stages, the
    cached state itself, and whether the Step 4.5 roll-up actually ran.
    """

    state, ran = load_or_build_roll_down_start_state(
        path, force_rebuild=force_rebuild, **roll_up_kwargs
    )
    return roll_up_result_from_top_contact_state(state), state, ran


@dataclass(frozen=True)
class RetractToWheelOnTopResult2D:
    """Step 7R: theta retracted to the wheel-like state while still on the top.

    ``branch_result`` is the underlying Step 6.5 coupled retract/roll result,
    kept so that the existing frame tables, plots and animations still apply.
    """

    branch_result: RetractResetBranchResult2D
    theta_target_rad: float
    success: bool
    failure_reason: str | None

    @property
    def frames(self) -> tuple[RetractResetFrame2D, ...]:
        return self.branch_result.frames

    @property
    def final_frame(self) -> RetractResetFrame2D:
        return self.frames[-1]

    @property
    def theta_target_deg(self) -> float:
        return float(np.rad2deg(self.theta_target_rad))

    @property
    def required_rotation_rad(self) -> float:
        return float(self.final_frame.accumulated_rotation_rad)

    @property
    def contact_forward_distance_m(self) -> float:
        """Top distance the *contact point* travelled during the retract."""

        return float(self.final_frame.contact_forward_displacement_m)

    @property
    def hip_forward_distance_m(self) -> float:
        """Top distance the *hip* travelled; not the same as the contact travel."""

        return float(self.final_frame.hip_forward_displacement_m)

    @property
    def theta_reached_at_step(self) -> int | None:
        for frame in self.frames:
            if np.isclose(frame.theta_rad, self.theta_target_rad, atol=1e-12):
                return int(frame.step)
        return None


def run_retract_to_wheel_on_top_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    theta_target_rad: float = np.deg2rad(17.0),
    theta_step_rad: float = np.deg2rad(1.0),
    beta_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(5.0),
    max_rotation_rad: float = np.deg2rad(400.0),
    max_forward_distance_m: float = 0.55,
    max_seam_bridge_m: float = 5e-3,
    obstacle_top_length_m: float | None = None,
    max_steps: int = 800,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> RetractToWheelOnTopResult2D:
    """Step 7R: retract to the wheel-like state without leaving the obstacle top.

    Starts from the successful roll-up final frame -- no approximate pose is
    rebuilt.  ``theta`` steps down toward ``theta_target_rad`` while ``beta``
    continues in the roll-up direction, the hip is reconstructed from a signed
    horizontal no-slip constraint on the previous material contact sample, and
    every frame is revalidated by the terrain-aware query.

    Success requires theta to reach its target while the active contact is
    still a legal obstacle-top contact.  The foot rim is deliberately not a
    termination condition here: the point of this step is the wheel-like
    configuration, not a foot-rim reset.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be a ForwardRollingResult2D.")
    branch = run_retract_and_reset_branch_2d(
        rolling_result,
        branch="forward_continuation",
        theta_target_rad=theta_target_rad,
        theta_step_rad=theta_step_rad,
        beta_step_rad=beta_step_rad,
        beta_search_window_rad=beta_search_window_rad,
        stop_at="theta_target",
        max_rotation_rad=max_rotation_rad,
        max_forward_distance_m=max_forward_distance_m,
        max_seam_bridge_m=max_seam_bridge_m,
        obstacle_top_length_m=obstacle_top_length_m,
        max_steps=max_steps,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )
    return RetractToWheelOnTopResult2D(
        branch_result=branch,
        theta_target_rad=float(theta_target_rad),
        success=bool(branch.success),
        failure_reason=branch.failure_reason,
    )


def traversal_phase_label(
    frame: RetractResetFrame2D,
    theta_target_rad: float,
    *,
    trailing_corner_x_m: float | None = None,
    corner_tolerance_m: float = 5e-3,
) -> str:
    """Name the traversal stage one coupled retract/roll frame belongs to.

    Steps 7R and 8R are one continuous motion, so the frames carry no stage of
    their own; the stage is read back from the configuration.  Keeping it
    derived rather than stored means the label can never disagree with the
    pose it describes.
    """

    at_wheel = bool(np.isclose(frame.theta_rad, theta_target_rad, atol=1e-12))
    if not at_wheel:
        return "RETRACT_TO_WHEEL"
    contact = frame.contact_point_world_xz_m
    if (
        trailing_corner_x_m is not None
        and contact is not None
        and abs(contact[0] - trailing_corner_x_m) <= corner_tolerance_m
    ):
        return "TRAILING_CORNER_ARRIVAL"
    if frame.active_rim == "left_rim":
        return "LEFT_RIM_TOP_ROLL"
    return "WHEEL_MODE_TOP_ROLL"


def _frame_row(
    frame: RetractResetFrame2D,
    theta_target_rad: float,
    *,
    trailing_corner_x_m: float | None = None,
) -> dict:
    """One frame of the coupled retract/roll motion as a flat row."""

    contact = frame.contact_point_world_xz_m
    return {
        "step": frame.step,
        "phase": traversal_phase_label(
            frame, theta_target_rad, trailing_corner_x_m=trailing_corner_x_m
        ),
        "theta_deg": float(np.rad2deg(frame.theta_rad)),
        "beta_deg": float(np.rad2deg(frame.beta_rad)),
        "accumulated_rotation_deg": float(
            np.rad2deg(frame.accumulated_rotation_rad)
        ),
        "active_rim": frame.active_rim,
        "active_sample_index": frame.active_sample_index,
        "alpha_deg": (
            None if frame.alpha_rad is None else float(np.rad2deg(frame.alpha_rad))
        ),
        "contact_x_m": None if contact is None else contact[0],
        "contact_z_m": None if contact is None else contact[1],
        "terrain_surface": frame.terrain_surface_id,
        "hip_x_m": frame.hip_position_world_xz_m[0],
        "hip_z_m": frame.hip_position_world_xz_m[1],
        "hip_forward_displacement_m": frame.hip_forward_displacement_m,
        "contact_forward_displacement_m": frame.contact_forward_displacement_m,
        "step_hip_displacement_m": frame.step_hip_displacement_m,
        "step_contact_displacement_m": frame.step_contact_displacement_m,
        "no_slip_tangent_residual_m": frame.no_slip_tangent_residual_m,
        "valid_contact": frame.valid_contact,
        "collision": frame.collision,
        # Split so a rejected frame says *what* was wrong, per the Day 6-7 spec.
        "penetration": bool(frame.query_result.geometry_penetrations),
        "link_collision": bool(frame.query_result.link_collisions),
        "vertical_face_collision": bool(frame.query_result.collisions),
        "joint_limits_ok": frame.joint_limits_ok,
        "accepted": frame.accepted,
        "failure_reason": frame.failure_reason,
    }


def retract_to_wheel_on_top_frame_rows(
    result: RetractToWheelOnTopResult2D,
) -> list[dict]:
    """One row per Step 7R frame."""

    if not isinstance(result, RetractToWheelOnTopResult2D):
        raise TypeError("result must be a RetractToWheelOnTopResult2D.")
    return [
        _frame_row(frame, result.theta_target_rad) for frame in result.frames
    ]


def retract_to_wheel_on_top_summary_row(
    result: RetractToWheelOnTopResult2D,
) -> dict:
    """One-row Step 7R summary."""

    if not isinstance(result, RetractToWheelOnTopResult2D):
        raise TypeError("result must be a RetractToWheelOnTopResult2D.")
    final = result.final_frame
    start = result.frames[0]
    contact = final.contact_point_world_xz_m
    return {
        "success": result.success,
        "failure_reason": result.failure_reason,
        "theta_start_deg": float(np.rad2deg(start.theta_rad)),
        "theta_target_deg": result.theta_target_deg,
        "theta_final_deg": float(np.rad2deg(final.theta_rad)),
        "beta_start_deg": float(np.rad2deg(start.beta_rad)),
        "beta_final_deg": float(np.rad2deg(final.beta_rad)),
        "required_rotation_deg": float(np.rad2deg(result.required_rotation_rad)),
        "contact_forward_distance_m": result.contact_forward_distance_m,
        "hip_forward_distance_m": result.hip_forward_distance_m,
        "final_active_rim": final.active_rim,
        "final_alpha_deg": (
            None if final.alpha_rad is None else float(np.rad2deg(final.alpha_rad))
        ),
        "final_contact_x_m": None if contact is None else contact[0],
        "final_terrain_surface": final.terrain_surface_id,
        "final_hip_x_m": final.hip_position_world_xz_m[0],
        "final_hip_z_m": final.hip_position_world_xz_m[1],
        "max_no_slip_residual_m": float(
            max(abs(frame.no_slip_tangent_residual_m) for frame in result.frames)
        ),
        "any_collision": bool(any(frame.collision for frame in result.frames)),
        "frame_count": len(result.frames),
    }


def write_retract_to_wheel_on_top_csv(
    result: RetractToWheelOnTopResult2D,
    summary_path,
    frames_path,
) -> tuple[Path, Path]:
    """Persist the Step 7R summary row and every frame."""

    summary_path = Path(summary_path)
    frames_path = Path(frames_path)
    payloads = (
        (summary_path, [retract_to_wheel_on_top_summary_row(result)]),
        (frames_path, retract_to_wheel_on_top_frame_rows(result)),
    )
    for path, rows in payloads:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    return summary_path, frames_path


def plot_retract_to_wheel_on_top_2d(
    result: RetractToWheelOnTopResult2D,
    *,
    axes=None,
):
    """Theta/alpha schedule and the hip-vs-contact travel during the retract."""

    if not isinstance(result, RetractToWheelOnTopResult2D):
        raise TypeError("result must be a RetractToWheelOnTopResult2D.")
    owns_figure = axes is None
    if owns_figure:
        _, axes = plt.subplots(1, 2, figsize=(13.0, 4.6))
    left, right = axes

    steps = [frame.step for frame in result.frames]
    thetas = [float(np.rad2deg(frame.theta_rad)) for frame in result.frames]
    alphas = [
        None if frame.alpha_rad is None else float(np.rad2deg(frame.alpha_rad))
        for frame in result.frames
    ]
    left.plot(steps, thetas, "-", color="#2563eb", linewidth=1.9, label="theta [deg]")
    left.axhline(result.theta_target_deg, color="#dc2626", linestyle="--",
                 linewidth=1.2, label=f"target {result.theta_target_deg:.0f} deg")
    twin = left.twinx()
    twin.plot(steps, alphas, "-", color="#16a34a", linewidth=1.6,
              label="contact alpha [deg]")
    twin.set_ylabel("contact alpha [deg]", color="#16a34a")
    left.set_xlabel("simulation step")
    left.set_ylabel("theta [deg]", color="#2563eb")
    left.set_title("Step 7R: retract to the wheel-like state")
    left.legend(loc="upper right", fontsize=8)
    left.grid(True, alpha=0.3)

    contact = [frame.contact_forward_displacement_m for frame in result.frames]
    hip = [frame.hip_forward_displacement_m for frame in result.frames]
    residual = [abs(frame.no_slip_tangent_residual_m) for frame in result.frames]
    right.plot(steps, contact, "-", color="#dc2626", linewidth=1.9,
               label="contact forward [m]")
    right.plot(steps, hip, "-", color="#7c3aed", linewidth=1.9,
               label="hip forward [m]")
    right.set_xlabel("simulation step")
    right.set_ylabel("forward displacement [m]")
    right.set_title(
        "Contact vs hip travel  (max no-slip residual "
        f"{max(residual):.2e} m)"
    )
    right.legend(loc="upper left", fontsize=8)
    right.grid(True, alpha=0.3)
    if owns_figure:
        left.figure.tight_layout()
    return axes


def plot_retract_to_wheel_key_frames_2d(
    result: RetractToWheelOnTopResult2D,
    *,
    labels=("START (theta = 60 deg)", "MID-RETRACT", "WHEEL STATE (theta = 17 deg)"),
):
    """Draw the retract at start, mid-point and the wheel-like end state.

    The schedule plot shows that theta falls while alpha advances; these three
    poses show *what that looks like*, in particular that the leg keeps a legal
    obstacle-top contact rather than lifting off and re-placing itself.
    """

    if not isinstance(result, RetractToWheelOnTopResult2D):
        raise TypeError("result must be a RetractToWheelOnTopResult2D.")
    frames = result.frames
    picks = [frames[0], frames[len(frames) // 2], result.final_frame]
    figure, axes = plt.subplots(
        1, len(picks), figsize=(4.8 * len(picks), 4.4), squeeze=False
    )
    for ax, label, frame in zip(axes[0], labels, picks):
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        alpha = (
            "n/a" if frame.alpha_rad is None
            else f"{np.rad2deg(frame.alpha_rad):.1f} deg"
        )
        ax.set_title(
            f"{label}  [step {frame.step}]\n"
            f"theta={np.rad2deg(frame.theta_rad):.1f} deg, "
            f"rim={frame.active_rim}, alpha={alpha}",
            fontsize=9,
        )
        ax.legend().set_visible(False)
    figure.tight_layout()
    return figure, axes


def animate_retract_to_wheel_on_top_2d(
    result: RetractToWheelOnTopResult2D,
    **kwargs,
):
    """Animate Step 7R by reusing the existing Step 6.5 branch animation."""

    if not isinstance(result, RetractToWheelOnTopResult2D):
        raise TypeError("result must be a RetractToWheelOnTopResult2D.")
    return animate_retract_and_reset_branch_2d(result.branch_result, **kwargs)


# --------------------------------------------------------------------------
# Step 8R: wheel-mode forward roll to the trailing corner
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class WheelModeTransitionResult2D:
    """Step 8R: roll at the wheel-like theta until the trailing corner.

    Reaching the corner and being *ready to descend* are deliberately two
    different flags.  How far the leg rolls on the top is set by the obstacle,
    not by the leg; whether the left rim has taken over by the time it gets
    there is set by the leg, not by the obstacle.  ``LEFT_RIM_READY`` is
    therefore evaluated as a precondition on arrival rather than used as the
    stopping rule.
    """

    branch_result: RetractResetBranchResult2D
    theta_target_rad: float
    trailing_corner_world_xz_m: tuple[float, float]
    reached_trailing_corner: bool
    left_rim_ready: bool
    readiness_failure: str | None
    failure_reason: str | None
    theta_reached_step: int | None
    left_rim_handover_step: int | None
    # The v4 spec asks for these three at LEFT_RIM_READY (the handover), not at
    # the corner, so they are measured on the handover frame specifically.
    l_transition_m: float | None
    required_beta_rotation_rad: float | None
    required_hip_forward_distance_m: float | None
    required_contact_forward_distance_m: float | None
    corner_gap_m: float | None
    # Remaining arc of whichever rim is active at the corner -- not necessarily
    # the left rim, which is exactly what makes it worth reporting.
    rim_budget_at_corner_rad: float | None
    wheel_radius_at_target_m: float
    predicted_pivot_rotation_rad: float | None

    @property
    def frames(self) -> tuple[RetractResetFrame2D, ...]:
        return self.branch_result.frames

    @property
    def final_frame(self) -> RetractResetFrame2D:
        return self.frames[-1]

    @property
    def descent_budget_sufficient(self) -> bool | None:
        """Is the left rim long enough for the predicted corner pivot?

        ``None`` unless the left rim is actually the rim at the corner: a
        healthy-looking budget on the right or foot rim says nothing about a
        left-rim descent, so it must not be reported as sufficiency.

        First-order only -- it treats the retracted leg as the circle it very
        nearly is at 17 degrees.  Step 9R is what actually verifies it.
        """

        if (
            not self.left_rim_ready
            or self.rim_budget_at_corner_rad is None
            or self.predicted_pivot_rotation_rad is None
        ):
            return None
        return bool(
            self.rim_budget_at_corner_rad >= self.predicted_pivot_rotation_rad
        )


def _rim_region_alpha_end_rad(geometry, sample_index: int) -> float:
    """Alpha at the far end of the rim region owning a sample."""

    regions = np.asarray(geometry.contact_regions)
    region = regions[sample_index]
    stop = sample_index
    while stop + 1 < len(regions) and regions[stop + 1] == region:
        stop += 1
    return float(geometry.alpha_rad[stop])


def _wheel_radius_at_theta(theta_rad: float, scene_kwargs: dict) -> float:
    """Radius of the retracted leg, measured at the lowest rim sample."""

    template = build_single_leg_rolling_scene_2d(
        theta_rad, 0.0, 0.0, 0.0, **scene_kwargs
    )
    points = template.geometry.points_hip_xz_m
    regions = np.asarray(template.geometry.contact_regions)
    physical = np.flatnonzero(regions != "non_contact_region")
    return float(np.max(np.linalg.norm(points[physical], axis=1)))


def _corner_readiness_failure(
    frame: RetractResetFrame2D,
    theta_target_rad: float,
) -> str | None:
    """Evaluate LEFT_RIM_READY as a precondition on the corner-arrival frame.

    Readiness is about the *leg*: retracted to the wheel state, with the left
    rim already carrying the contact, on a legal top contact.  How far the leg
    had to roll to get there is the obstacle's business, not this check's.
    """

    if not np.isclose(frame.theta_rad, theta_target_rad, atol=1e-12):
        return "THETA_NOT_AT_WHEEL_TARGET_AT_CORNER"
    if frame.active_rim != "left_rim":
        return "LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER"
    if frame.collision or not frame.valid_contact:
        return "INVALID_CONTACT_AT_CORNER"
    return None


def run_wheel_mode_transition_to_corner_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    theta_target_rad: float = np.deg2rad(17.0),
    theta_step_rad: float = np.deg2rad(1.0),
    beta_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(5.0),
    max_rotation_rad: float = np.deg2rad(600.0),
    max_forward_distance_m: float = 0.90,
    max_seam_bridge_m: float = 5e-3,
    obstacle_top_length_m: float | None = None,
    max_steps: int = 1200,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> WheelModeTransitionResult2D:
    """Step 8R: retract to the wheel state, then roll on to the trailing corner.

    This is Step 7R continued, not a second motion: the same coupled
    theta/beta no-slip trajectory simply runs on until the contact reaches the
    trailing corner.  Along the way the theta milestone and the right-to-left
    rim handover are recorded, and on arrival the ``LEFT_RIM_READY``
    precondition is evaluated.

    A run can reach the corner without being ready -- the top may be too short
    for the handover to have happened, or long enough that the left rim has
    already been partly spent.  Both are reported rather than collapsed into
    one boolean.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be a ForwardRollingResult2D.")
    branch = run_retract_and_reset_branch_2d(
        rolling_result,
        branch="forward_continuation",
        theta_target_rad=theta_target_rad,
        theta_step_rad=theta_step_rad,
        beta_step_rad=beta_step_rad,
        beta_search_window_rad=beta_search_window_rad,
        stop_at="trailing_corner",
        max_rotation_rad=max_rotation_rad,
        max_forward_distance_m=max_forward_distance_m,
        max_seam_bridge_m=max_seam_bridge_m,
        obstacle_top_length_m=obstacle_top_length_m,
        max_steps=max_steps,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )
    start_scene = branch.frames[0].scene
    obstacle = start_scene.terrain.obstacle
    if obstacle is None:
        raise ValueError("the traversal scene has no rectangular obstacle.")
    top_z = start_scene.terrain.ground_height_m + obstacle.height_m
    x_min = float(obstacle.x_min_m)
    top_length = (
        float(obstacle.width_m) if obstacle_top_length_m is None
        else float(obstacle_top_length_m)
    )
    corner = (x_min + top_length, top_z)

    scene_kwargs = {
        "gamma_rad": start_scene.gamma_rad,
        "ground_height_m": start_scene.terrain.ground_height_m,
        "obstacle_x_start_m": x_min,
        "obstacle_width_m": top_length,
        "obstacle_height_m": float(obstacle.height_m),
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": len(start_scene.geometry.points_hip_xz_m) // 3,
    }
    wheel_radius = _wheel_radius_at_theta(theta_target_rad, scene_kwargs)
    height = float(obstacle.height_m)
    cosine = 1.0 - height / wheel_radius
    predicted_pivot = (
        float(np.arccos(cosine)) if -1.0 <= cosine <= 1.0 else None
    )

    theta_reached_step = None
    handover_step = None
    handover_frame = None
    for previous, frame in zip(branch.frames, branch.frames[1:]):
        if theta_reached_step is None and np.isclose(
            frame.theta_rad, theta_target_rad, atol=1e-12
        ):
            theta_reached_step = int(frame.step)
        if (
            handover_step is None
            and previous.active_rim == "right_rim"
            and frame.active_rim == "left_rim"
        ):
            handover_step = int(frame.step)
            handover_frame = frame
    l_transition = (
        None if handover_frame is None
        else float(handover_frame.contact_forward_displacement_m)
    )
    required_rotation = (
        None if handover_frame is None
        else float(handover_frame.accumulated_rotation_rad)
    )
    required_hip_forward = (
        None if handover_frame is None
        else float(handover_frame.hip_forward_displacement_m)
    )

    reached_corner = bool(branch.success)
    final = branch.final_frame
    corner_gap = None
    budget = None
    readiness_failure = None
    if reached_corner and final.contact_point_world_xz_m is not None:
        corner_gap = float(corner[0] - final.contact_point_world_xz_m[0])
        if final.active_sample_index is not None:
            budget = float(
                abs(
                    _rim_region_alpha_end_rad(
                        final.scene.geometry, final.active_sample_index
                    )
                    - final.alpha_rad
                )
            )
        readiness_failure = _corner_readiness_failure(final, theta_target_rad)
    elif not reached_corner:
        readiness_failure = "TRAILING_CORNER_NOT_REACHED"

    return WheelModeTransitionResult2D(
        branch_result=branch,
        theta_target_rad=float(theta_target_rad),
        trailing_corner_world_xz_m=corner,
        reached_trailing_corner=reached_corner,
        left_rim_ready=bool(reached_corner and readiness_failure is None),
        readiness_failure=readiness_failure,
        failure_reason=branch.failure_reason,
        theta_reached_step=theta_reached_step,
        left_rim_handover_step=handover_step,
        l_transition_m=l_transition,
        required_beta_rotation_rad=required_rotation,
        required_hip_forward_distance_m=required_hip_forward,
        required_contact_forward_distance_m=l_transition,
        corner_gap_m=corner_gap,
        rim_budget_at_corner_rad=budget,
        wheel_radius_at_target_m=wheel_radius,
        predicted_pivot_rotation_rad=predicted_pivot,
    )


def wheel_mode_transition_summary_row(
    result: WheelModeTransitionResult2D,
) -> dict:
    """One-row Step 8R summary."""

    if not isinstance(result, WheelModeTransitionResult2D):
        raise TypeError("result must be a WheelModeTransitionResult2D.")
    final = result.final_frame
    contact = final.contact_point_world_xz_m
    return {
        "reached_trailing_corner": result.reached_trailing_corner,
        "left_rim_ready": result.left_rim_ready,
        "readiness_failure": result.readiness_failure,
        "failure_reason": result.failure_reason,
        "trailing_corner_x_m": result.trailing_corner_world_xz_m[0],
        "theta_reached_step": result.theta_reached_step,
        "left_rim_handover_step": result.left_rim_handover_step,
        "L_transition_m": result.l_transition_m,
        "required_beta_rotation_deg": (
            None if result.required_beta_rotation_rad is None
            else float(np.rad2deg(result.required_beta_rotation_rad))
        ),
        "required_hip_forward_distance_m": result.required_hip_forward_distance_m,
        "required_contact_forward_distance_m": (
            result.required_contact_forward_distance_m
        ),
        "corner_gap_m": result.corner_gap_m,
        "final_active_rim": final.active_rim,
        "final_alpha_deg": (
            None if final.alpha_rad is None else float(np.rad2deg(final.alpha_rad))
        ),
        "final_contact_x_m": None if contact is None else contact[0],
        "final_theta_deg": float(np.rad2deg(final.theta_rad)),
        "total_rotation_deg": float(np.rad2deg(final.accumulated_rotation_rad)),
        "contact_forward_distance_m": float(final.contact_forward_displacement_m),
        "hip_forward_distance_m": float(final.hip_forward_displacement_m),
        "wheel_radius_at_target_m": result.wheel_radius_at_target_m,
        "rim_budget_at_corner_deg": (
            None if result.rim_budget_at_corner_rad is None
            else float(np.rad2deg(result.rim_budget_at_corner_rad))
        ),
        "predicted_pivot_rotation_deg": (
            None if result.predicted_pivot_rotation_rad is None
            else float(np.rad2deg(result.predicted_pivot_rotation_rad))
        ),
        "descent_budget_sufficient": result.descent_budget_sufficient,
        "frame_count": len(result.frames),
    }


def corner_states_along_transition(
    result: WheelModeTransitionResult2D,
    top_lengths_m,
    *,
    corner_tolerance_m: float = 1e-3,
) -> list[dict]:
    """Derive the corner-arrival state for several top lengths from one run.

    The wheel-mode trajectory does not depend on where the trailing corner is;
    the corner only decides *where the run stops*.  So a single long run
    already contains the arrival state for every shorter top, and sweeping top
    length costs one simulation instead of one per length.  A regression test
    checks the derived states against direct runs.

    This holds only while the leg never touches the drop side before reaching
    the corner, which is the case for the retracted wheel configuration but is
    not assumed silently: every returned row carries the frame it came from so
    it can be re-validated.

    ``top_lengths_m`` longer than the simulated run are reported with
    ``reached_trailing_corner=False`` rather than extrapolated.
    """

    if not isinstance(result, WheelModeTransitionResult2D):
        raise TypeError("result must be a WheelModeTransitionResult2D.")
    start_scene = result.frames[0].scene
    obstacle = start_scene.terrain.obstacle
    if obstacle is None:
        raise ValueError("the traversal scene has no rectangular obstacle.")
    x_min = float(obstacle.x_min_m)
    usable = [
        frame for frame in result.frames
        if frame.accepted and frame.contact_point_world_xz_m is not None
    ]
    simulated_max_x = max(
        frame.contact_point_world_xz_m[0] for frame in usable
    )

    rows = []
    for top_length in top_lengths_m:
        top_length = float(top_length)
        corner_x = x_min + top_length
        reachable = [
            frame for frame in usable
            if frame.contact_point_world_xz_m[0] <= corner_x - corner_tolerance_m
        ]
        if not reachable or corner_x - corner_tolerance_m > simulated_max_x:
            rows.append(
                {
                    "obstacle_top_length_m": top_length,
                    "trailing_corner_x_m": corner_x,
                    "reached_trailing_corner": False,
                    "left_rim_ready": False,
                    "readiness_failure": "TRAILING_CORNER_BEYOND_SIMULATED_RUN"
                    if reachable else "NO_FRAME_BEFORE_TRAILING_CORNER",
                    "step": None,
                    "active_rim": None,
                    "alpha_deg": None,
                    "contact_x_m": None,
                    "corner_gap_m": None,
                    "rim_budget_at_corner_deg": None,
                    "descent_budget_sufficient": None,
                    "frame": None,
                }
            )
            continue
        frame = reachable[-1]
        failure = _corner_readiness_failure(frame, result.theta_target_rad)
        budget = float(
            abs(
                _rim_region_alpha_end_rad(
                    frame.scene.geometry, frame.active_sample_index
                )
                - frame.alpha_rad
            )
        )
        ready = failure is None
        sufficient = (
            None
            if not ready or result.predicted_pivot_rotation_rad is None
            else bool(budget >= result.predicted_pivot_rotation_rad)
        )
        rows.append(
            {
                "obstacle_top_length_m": top_length,
                "trailing_corner_x_m": corner_x,
                "reached_trailing_corner": True,
                "left_rim_ready": ready,
                "readiness_failure": failure,
                "step": int(frame.step),
                "active_rim": frame.active_rim,
                "alpha_deg": float(np.rad2deg(frame.alpha_rad)),
                "contact_x_m": float(frame.contact_point_world_xz_m[0]),
                "corner_gap_m": float(
                    corner_x - frame.contact_point_world_xz_m[0]
                ),
                "rim_budget_at_corner_deg": float(np.rad2deg(budget)),
                "descent_budget_sufficient": sufficient,
                "frame": frame,
            }
        )
    return rows


# --------------------------------------------------------------------------
# Step 9R: left-rim trailing-edge roll-down
# --------------------------------------------------------------------------

PHASE_LEFT_CORNER_PIVOT = "TRAILING_CORNER_TRANSITION"
PHASE_LEFT_ROLL_DOWN = "LEFT_RIM_ROLL_DOWN"
PHASE_LEFT_GROUND_CONTACT = "LOWER_GROUND_CONTACT"
PHASE_LEFT_GROUND_ROLL = "GROUND_ROLL"
PHASE_LEFT_FAILED = "FAILED"


@dataclass(frozen=True)
class LeftRimRollDownResult2D:
    """Step 9R: descend the trailing edge on the left rim.

    ``theta`` is held through the corner pivot and only released once the leg
    is close to the lower ground.  Releasing it does *not* reach for the
    ground: with a material point pinned on the corner, extending the leg
    pushes the hip away from that corner, so clearance grows.  What the
    release actually buys is a smaller hip drop, paid for with extra rotation
    and therefore extra rim arc.  Both sides of that trade are recorded.
    """

    frames: tuple
    success: bool
    failure_phase: str | None
    failure_reason: str | None
    corner_pivot_success: bool
    ground_contact_success: bool
    ground_roll_success: bool
    theta_released: bool
    theta_release_step: int | None
    theta_final_rad: float
    theta_ceiling_rad: float
    trailing_corner_world_xz_m: tuple[float, float]
    corner_landing_error_m: float
    pivot_rotation_rad: float | None
    hip_drop_at_touchdown_m: float | None
    ground_contact_point_world_xz_m: tuple[float, float] | None
    ground_contact_rim: str | None
    ground_contact_alpha_rad: float | None
    remaining_rim_arc_rad_at_touchdown: float | None
    ground_roll_distance_m: float
    ground_roll_distance_achieved_m: float
    minimum_back_face_clearance_m: float | None
    descent_rim: str
    theta_ceiling_hit: bool

    @property
    def descent_rim_preserved(self) -> bool | None:
        """Did the leg actually land on the rim it descended on?

        Extending theta makes the foot rim protrude, and past a ceiling it
        takes the touchdown away from the descent rim -- landing on the foot
        rim's boundary at alpha = -40 deg rather than on a rim surface.  That
        is a different motion from a left-rim descent, so it is reported
        rather than accepted silently.
        """

        if self.ground_contact_rim is None:
            return None
        return self.ground_contact_rim == self.descent_rim

    @property
    def final_frame(self):
        return self.frames[-1]

    def frames_in_phase(self, phase: str) -> tuple:
        return tuple(frame for frame in self.frames if frame.phase == phase)


def run_left_rim_roll_down_2d(
    transition: WheelModeTransitionResult2D,
    *,
    pivot_beta_step_rad: float = np.deg2rad(1.0),
    release_theta: bool = True,
    theta_release_clearance_m: float = 0.02,
    theta_step_rad: float = np.deg2rad(1.0),
    theta_max_rad: float = np.deg2rad(45.0),
    keep_descent_rim: bool = True,
    descent_rim_margin_m: float = 5e-3,
    ground_contact_step_m: float = 0.002,
    ground_roll_distance_m: float = 0.02,
    max_pivot_rotation_rad: float = np.deg2rad(180.0),
    max_flat_rotation_rad: float = np.deg2rad(30.0),
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    touchdown_tolerance_m: float = 5e-5,
    surface_offset_m: float = 1e-9,
    sample_match_tolerance: int = 3,
    corner_exclusion_m: float = 0.01,
    max_steps: int = 500,
    bisection_iterations: int = 24,
) -> LeftRimRollDownResult2D:
    """Step 9R: pivot the left rim over the trailing corner down to lower ground.

    Starts from the Step 8R corner-arrival state, which must satisfy the
    ``LEFT_RIM_READY`` precondition.  The material sample that arrived at the
    corner is pinned there and the leg turns about it, exactly as in the
    right-rim study: a sharp corner supports a point, so the corner transition
    is a rotation rather than a roll and the world contact point does not move.

    ``release_theta`` enables the late-descent theta freedom.  Set it to
    ``False`` for the theta-held baseline that the release is measured against.

    Theta cannot be extended without limit.  Past a ceiling the foot rim
    protrudes far enough to become the lowest point and take the touchdown,
    which turns a left-rim descent into a landing on the foot rim's boundary.
    ``keep_descent_rim`` (default) refuses any theta step that would do that,
    so the released run stays the same motion as the baseline, only gentler.
    Setting it to ``False`` allows the larger hip-drop saving at the cost of
    changing which rim lands.

    Merely keeping the descent rim lowest is not enough to stay usable: at the
    ceiling the foot rim can sit a fraction of a millimetre above the ground,
    which is legal at touchdown but blocks the very next rolling step.
    ``descent_rim_margin_m`` therefore requires every other rim to stay that
    far clear of the ground at touchdown, so the leg can still roll away.
    """

    if not isinstance(transition, WheelModeTransitionResult2D):
        raise TypeError("transition must be a WheelModeTransitionResult2D.")
    if not transition.left_rim_ready:
        raise ValueError(
            "Step 9R requires a LEFT_RIM_READY corner state; this one failed "
            f"with {transition.readiness_failure}."
        )
    start = transition.final_frame
    if start.active_sample_index is None or start.contact_point_world_xz_m is None:
        raise ValueError("the corner-arrival frame carries no active contact.")

    scene0 = start.scene
    obstacle = scene0.terrain.obstacle
    ground_height = float(scene0.terrain.ground_height_m)
    top_z = ground_height + float(obstacle.height_m)
    corner_x = float(transition.trailing_corner_world_xz_m[0])
    corner = np.array([corner_x, top_z], dtype=float)
    scene_kwargs = {
        "gamma_rad": scene0.gamma_rad,
        "ground_height_m": ground_height,
        "obstacle_x_start_m": float(obstacle.x_min_m),
        "obstacle_width_m": corner_x - float(obstacle.x_min_m),
        "obstacle_height_m": float(obstacle.height_m),
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": len(scene0.geometry.points_hip_xz_m) // 3,
    }
    pin = int(start.active_sample_index)
    descent_rim = str(np.asarray(scene0.geometry.contact_regions)[pin])
    theta = float(start.theta_rad)
    beta0 = float(start.beta_rad)
    corner_landing_error = float(corner_x - start.contact_point_world_xz_m[0])
    pivot_target = corner + np.array([0.0, surface_offset_m])

    frames: list = []
    state = {"rotation": 0.0, "advance": 0.0, "hip0": None, "min_back": None,
             "release_step": None, "theta_ceiling": False}

    def query(scene):
        return query_single_leg_rolling_scene_2d(
            scene, contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )

    def append(scene, query_result, candidate, *, phase, active_sample,
               corner_pinned=False, ground_supported=False, accepted=True,
               failure_reason=None):
        hip = np.asarray(scene.hip_pose.position_world_xz_m, dtype=float)
        if state["hip0"] is None:
            state["hip0"] = hip.copy()
        hip0 = state["hip0"]
        clearance, lowest = _ground_clearance(scene)
        back = _back_face_clearance(scene, corner_exclusion_m=corner_exclusion_m)
        if back is not None:
            state["min_back"] = back if state["min_back"] is None else min(
                state["min_back"], back
            )
        point = None if candidate is None else tuple(
            float(v) for v in candidate.point_world_xz_m
        )
        frame = RollDownFrame2D(
            step=len(frames),
            phase=phase,
            hip_x_m=float(hip[0]),
            hip_z_m=float(hip[1]),
            hip_forward_displacement_m=float(hip[0] - hip0[0]),
            hip_vertical_displacement_m=float(hip[1] - hip0[1]),
            theta_rad=float(scene.theta_rad),
            beta_rad=float(scene.beta_rad),
            accumulated_rotation_rad=float(state["rotation"]),
            active_rim=None if candidate is None else candidate.rim.value,
            active_sample_index=None if active_sample is None else int(active_sample),
            alpha_rad=None if candidate is None else float(candidate.alpha_rad),
            contact_point_world_xz_m=point,
            terrain_surface_id=(
                None if candidate is None else candidate.terrain_surface_id
            ),
            trailing_edge_distance_m=None if point is None else float(point[0] - corner_x),
            contact_advance_m=float(state["advance"]),
            lowest_point_world_xz_m=tuple(
                float(v) for v in scene.geometry.points_world_xz_m[lowest]
            ),
            ground_clearance_m=float(clearance),
            back_face_clearance_m=back,
            remaining_rim_arc_rad=_remaining_rim_budget_rad(scene.geometry),
            corner_pinned=bool(corner_pinned),
            ground_supported=bool(ground_supported),
            valid_contact=bool(accepted and candidate is not None),
            collision=bool(query_result.collision),
            status=query_result.primary_status,
            accepted=bool(accepted),
            failure_reason=failure_reason,
            scene=scene,
            query_result=query_result,
        )
        frames.append(frame)
        return frame

    def build(theta_value, beta_value):
        return _pivot_scene(theta_value, beta_value, pin, pivot_target, scene_kwargs)

    def result(*, success, failure_phase=None, failure_reason=None, **extra):
        payload = dict(
            frames=tuple(frames),
            success=success,
            failure_phase=failure_phase,
            failure_reason=failure_reason,
            corner_pivot_success=state.get("pivot_ok", False),
            ground_contact_success=state.get("ground_ok", False),
            ground_roll_success=state.get("roll_ok", False),
            theta_released=state["release_step"] is not None,
            theta_release_step=state["release_step"],
            theta_final_rad=float(frames[-1].theta_rad),
            trailing_corner_world_xz_m=(float(corner[0]), float(corner[1])),
            corner_landing_error_m=corner_landing_error,
            pivot_rotation_rad=state.get("pivot_rotation"),
            hip_drop_at_touchdown_m=state.get("hip_drop"),
            ground_contact_point_world_xz_m=state.get("ground_point"),
            ground_contact_rim=state.get("ground_rim"),
            ground_contact_alpha_rad=state.get("ground_alpha"),
            remaining_rim_arc_rad_at_touchdown=state.get("ground_budget"),
            ground_roll_distance_m=float(ground_roll_distance_m),
            ground_roll_distance_achieved_m=float(state.get("rolled", 0.0)),
            minimum_back_face_clearance_m=state["min_back"],
            descent_rim=descent_rim,
            theta_ceiling_hit=bool(state["theta_ceiling"]),
            theta_ceiling_rad=float(theta_ceiling),
        )
        payload.update(extra)
        return LeftRimRollDownResult2D(**payload)

    def fail(scene, query_result, phase, reason, sample=None):
        append(scene, query_result, None, phase=PHASE_LEFT_FAILED,
               active_sample=sample, accepted=False, failure_reason=reason)
        return result(success=False, failure_phase=phase, failure_reason=reason)

    # ---- corner arrival ---------------------------------------------------
    scene = build(theta, beta0)
    query_result = query(scene)
    candidate = _candidate_for_sample(query_result, pin)
    if query_result.collision or candidate is None:
        return fail(scene, query_result, PHASE_LEFT_CORNER_PIVOT,
                    "INVALID_CORNER_ARRIVAL_STATE", pin)
    append(scene, query_result, candidate, phase=PHASE_LEFT_CORNER_PIVOT,
           active_sample=pin, corner_pinned=True)
    pivot_hip_z0 = frames[0].hip_z_m

    # ---- how far theta may be extended without losing the descent rim -----
    def touchdown_is_usable(theta_value: float) -> bool:
        """Would this theta still land on the descent rim, with room to roll?

        Checks the *touchdown* configuration rather than the current step: the
        lowest sample partway through the pivot does not predict which rim
        finally lands.
        """

        low, high = 0.0, max_pivot_rotation_rad
        if _ground_clearance(build(theta_value, beta0 - high))[0] > 0.0:
            return False
        for _ in range(bisection_iterations):
            middle = 0.5 * (low + high)
            if _ground_clearance(build(theta_value, beta0 - middle))[0] > 0.0:
                low = middle
            else:
                high = middle
        scene_at = build(theta_value, beta0 - low)
        _, lowest = _ground_clearance(scene_at)
        regions = np.asarray(scene_at.geometry.contact_regions)
        if str(regions[lowest]) != descent_rim:
            return False
        points = scene_at.geometry.points_world_xz_m
        others = np.flatnonzero(
            (regions != "non_contact_region") & (regions != descent_rim)
        )
        if others.size == 0:
            return True
        clearance = float(
            np.min(points[others, 1]) - scene_at.terrain.ground_height_m
        )
        return clearance >= descent_rim_margin_m

    theta_ceiling = theta_max_rad
    if release_theta and keep_descent_rim:
        # Extending theta makes the foot rim protrude; past a ceiling it takes
        # the touchdown away from the descent rim.  The ceiling is found once,
        # by bisection on theta, rather than guessed per step -- the per-step
        # lowest sample does not predict which rim finally lands.  Monotonicity
        # in theta is assumed here and the landing rim is checked afterwards.
        if touchdown_is_usable(theta_max_rad):
            theta_ceiling = theta_max_rad
        else:
            low, high = theta, theta_max_rad
            for _ in range(bisection_iterations):
                middle = 0.5 * (low + high)
                if touchdown_is_usable(middle):
                    low = middle
                else:
                    high = middle
            theta_ceiling = low
            state["theta_ceiling"] = True

    # ---- corner pivot, theta held then optionally released ----------------
    rotation = 0.0
    touchdown = None
    while len(frames) < max_steps and rotation < max_pivot_rotation_rad:
        next_rotation = min(rotation + pivot_beta_step_rad, max_pivot_rotation_rad)
        released = state["release_step"] is not None
        theta_options = [theta]
        if release_theta and released and theta + theta_step_rad <= theta_ceiling:
            # Prefer extending: it is what trades rotation for a gentler hip drop.
            theta_options.insert(0, theta + theta_step_rad)
        elif release_theta and released and theta + theta_step_rad <= theta_max_rad:
            state["theta_ceiling"] = True
        chosen = None
        for theta_value in theta_options:
            probe = build(theta_value, beta0 - next_rotation)
            clearance, lowest = _ground_clearance(probe)
            if clearance <= touchdown_tolerance_m:
                chosen = (theta_value, probe, clearance)
                break
            probe_query = query(probe)
            probe_candidate = _candidate_for_sample(probe_query, pin)
            if probe_query.collision or probe_candidate is None or (
                abs(probe_candidate.sample_index - pin) > sample_match_tolerance
            ):
                continue
            chosen = (theta_value, probe, clearance, probe_query, probe_candidate)
            break
        if chosen is None:
            return fail(frames[-1].scene, frames[-1].query_result,
                        PHASE_LEFT_ROLL_DOWN, "NO_LEGAL_CORNER_PIVOT_CONTINUATION", pin)
        if len(chosen) == 3:
            theta_value, _, _ = chosen
            theta = theta_value
            low, high = rotation, next_rotation
            for _ in range(bisection_iterations):
                middle = 0.5 * (low + high)
                clearance, _ = _ground_clearance(build(theta, beta0 - middle))
                if clearance > 0.0:
                    low = middle
                else:
                    high = middle
            touchdown = low
            break
        theta_value, probe, clearance, probe_query, probe_candidate = chosen
        theta = theta_value
        state["rotation"] += next_rotation - rotation
        rotation = next_rotation
        if (
            release_theta
            and state["release_step"] is None
            and clearance <= theta_release_clearance_m
        ):
            state["release_step"] = len(frames)
        phase = (
            PHASE_LEFT_ROLL_DOWN
            if state["release_step"] is not None
            or probe.geometry.points_world_xz_m[
                _lowest_contact_sample(probe.geometry), 1
            ] < top_z - contact_tolerance_m
            else PHASE_LEFT_CORNER_PIVOT
        )
        append(probe, probe_query, probe_candidate, phase=phase,
               active_sample=pin, corner_pinned=True)
        state["pivot_ok"] = True

    if touchdown is None:
        return fail(frames[-1].scene, frames[-1].query_result, PHASE_LEFT_ROLL_DOWN,
                    "MAX_PIVOT_ROTATION_BEFORE_GROUND_CONTACT", pin)

    # ---- lower-ground touchdown -------------------------------------------
    scene = build(theta, beta0 - touchdown)
    query_result = query(scene)
    points = scene.geometry.points_world_xz_m
    lowest_any = int(np.argmin(points[:, 1]))
    ground_sample = _lowest_contact_sample(scene.geometry)
    regions = np.asarray(scene.geometry.contact_regions)
    if regions[lowest_any] == "non_contact_region" and (
        points[lowest_any, 1] < points[ground_sample, 1] - 1e-9
    ):
        return fail(scene, query_result, PHASE_LEFT_GROUND_CONTACT,
                    "RIM_SEAM_REACHES_GROUND_FIRST", pin)
    if query_result.collision:
        return fail(scene, query_result, PHASE_LEFT_GROUND_CONTACT,
                    "COLLISION_AT_LOWER_GROUND_TOUCHDOWN", ground_sample)
    ground_surface_id = scene.terrain.ground_surface_id
    ground_candidate = _candidate_for_sample(
        query_result, ground_sample, surface_ids=(ground_surface_id,)
    )
    if ground_candidate is None or (
        abs(ground_candidate.sample_index - ground_sample) > sample_match_tolerance
    ):
        return fail(scene, query_result, PHASE_LEFT_GROUND_CONTACT,
                    "NO_LEGAL_LOWER_GROUND_CONTACT", ground_sample)
    state["rotation"] += touchdown - rotation
    state["pivot_ok"] = True
    state["ground_ok"] = True
    state["pivot_rotation"] = float(touchdown)
    state["hip_drop"] = float(scene.hip_pose.position_world_xz_m[1] - pivot_hip_z0)
    state["ground_point"] = tuple(float(v) for v in ground_candidate.point_world_xz_m)
    state["ground_rim"] = ground_candidate.rim.value
    state["ground_alpha"] = float(ground_candidate.alpha_rad)
    state["ground_budget"] = _remaining_rim_arc_rad(scene.geometry, ground_sample)
    append(scene, query_result, ground_candidate, phase=PHASE_LEFT_GROUND_CONTACT,
           active_sample=ground_sample, corner_pinned=True, ground_supported=True)

    # ---- no-slip rolling away on lower ground ------------------------------
    beta = float(scene.beta_rad)
    sample = ground_sample
    contact = np.asarray(ground_candidate.point_world_xz_m, dtype=float)
    previous_points = build_single_leg_rolling_scene_2d(
        theta, beta, 0.0, 0.0, **scene_kwargs
    ).geometry.points_hip_xz_m
    rotation_resolution = np.deg2rad(180.0) / max(
        int(scene_kwargs["arc_samples"]), 2
    )
    state["rolled"] = 0.0
    roll_reason = None
    while state["rolled"] < ground_roll_distance_m - 1e-12:
        if len(frames) >= max_steps:
            roll_reason = "MAX_STEPS_DURING_GROUND_ROLL"
            break
        target_advance = min(
            ground_contact_step_m, ground_roll_distance_m - state["rolled"]
        )
        solved = _solve_flat_roll_rotation(
            theta, beta, sample, previous_points, scene_kwargs,
            target_advance_m=target_advance,
            beta_direction=-1.0,
            coarse_step_rad=pivot_beta_step_rad,
            max_rotation_rad=max_flat_rotation_rad,
            iterations=bisection_iterations,
            rotation_resolution_rad=rotation_resolution,
        )
        if solved is None:
            roll_reason = "RIM_ARC_EXHAUSTED_DURING_GROUND_ROLL"
            break
        template, next_sample, arc, step_rotation = solved
        target = np.array([contact[0] + arc, ground_height], dtype=float)
        roll_scene = _translated_scene_with_sample_on_target_2d(
            template, next_sample, target + np.array([0.0, surface_offset_m])
        )
        roll_query = query(roll_scene)
        roll_candidate = _candidate_for_sample(
            roll_query, next_sample, surface_ids=(ground_surface_id,)
        )
        if roll_query.collision or roll_candidate is None or (
            abs(roll_candidate.sample_index - next_sample) > sample_match_tolerance
        ):
            roll_reason = "NO_LEGAL_GROUND_ROLL_CONTINUATION"
            break
        state["rotation"] += step_rotation
        state["advance"] += arc
        state["rolled"] += arc
        beta = float(roll_scene.beta_rad)
        sample = next_sample
        contact = target
        previous_points = template.geometry.points_hip_xz_m
        append(roll_scene, roll_query, roll_candidate, phase=PHASE_LEFT_GROUND_ROLL,
               active_sample=sample, ground_supported=True)
    state["roll_ok"] = roll_reason is None
    return result(
        success=True,
        failure_phase=None if roll_reason is None else PHASE_LEFT_GROUND_ROLL,
        failure_reason=roll_reason,
    )


def left_rim_roll_down_frame_rows(result: LeftRimRollDownResult2D) -> list[dict]:
    """One row per Step 9R frame, sharing the Step 7R roll-down table shape."""

    if not isinstance(result, LeftRimRollDownResult2D):
        raise TypeError("result must be a LeftRimRollDownResult2D.")
    return roll_down_frame_rows(result)


def left_rim_roll_down_summary_row(result: LeftRimRollDownResult2D) -> dict:
    """One-row Step 9R summary."""

    if not isinstance(result, LeftRimRollDownResult2D):
        raise TypeError("result must be a LeftRimRollDownResult2D.")
    ground = result.ground_contact_point_world_xz_m
    return {
        "success": result.success,
        "failure_phase": result.failure_phase,
        "failure_reason": result.failure_reason,
        "corner_pivot_success": result.corner_pivot_success,
        "ground_contact_success": result.ground_contact_success,
        "ground_roll_success": result.ground_roll_success,
        "descent_rim": result.descent_rim,
        "ground_contact_rim": result.ground_contact_rim,
        "descent_rim_preserved": result.descent_rim_preserved,
        "theta_released": result.theta_released,
        "theta_release_step": result.theta_release_step,
        "theta_final_deg": float(np.rad2deg(result.theta_final_rad)),
        "theta_ceiling_deg": float(np.rad2deg(result.theta_ceiling_rad)),
        "theta_ceiling_hit": result.theta_ceiling_hit,
        "pivot_rotation_deg": (
            None if result.pivot_rotation_rad is None
            else float(np.rad2deg(result.pivot_rotation_rad))
        ),
        "hip_drop_at_touchdown_m": result.hip_drop_at_touchdown_m,
        "trailing_corner_x_m": result.trailing_corner_world_xz_m[0],
        "corner_landing_error_m": result.corner_landing_error_m,
        "ground_contact_x_m": None if ground is None else ground[0],
        "ground_contact_alpha_deg": (
            None if result.ground_contact_alpha_rad is None
            else float(np.rad2deg(result.ground_contact_alpha_rad))
        ),
        "remaining_rim_arc_deg_at_touchdown": (
            None if result.remaining_rim_arc_rad_at_touchdown is None
            else float(np.rad2deg(result.remaining_rim_arc_rad_at_touchdown))
        ),
        "ground_roll_distance_achieved_m": result.ground_roll_distance_achieved_m,
        "minimum_back_face_clearance_m": result.minimum_back_face_clearance_m,
        "frame_count": len(result.frames),
    }


def plot_left_rim_roll_down_key_frames_2d(
    result: LeftRimRollDownResult2D,
    *,
    phases=(PHASE_LEFT_CORNER_PIVOT, PHASE_LEFT_ROLL_DOWN,
            PHASE_LEFT_GROUND_CONTACT, PHASE_LEFT_GROUND_ROLL),
):
    """Last accepted frame of each Step 9R phase that actually occurred.

    Same renderer as the archived right-rim study -- only the phase names
    differ -- because the descent physics is the same problem.
    """

    if not isinstance(result, LeftRimRollDownResult2D):
        raise TypeError("result must be a LeftRimRollDownResult2D.")
    return plot_roll_down_key_frames_2d(result, phases=phases)


def animate_left_rim_roll_down_2d(result: LeftRimRollDownResult2D, **kwargs):
    """Animate Step 9R by reusing the trailing-edge roll-down animation."""

    if not isinstance(result, LeftRimRollDownResult2D):
        raise TypeError("result must be a LeftRimRollDownResult2D.")
    return animate_trailing_edge_roll_down_2d(result, **kwargs)


def wheel_mode_transition_frame_rows(
    result: WheelModeTransitionResult2D,
) -> list[dict]:
    """One row per Step 8R frame, sharing the Step 7R table shape."""

    if not isinstance(result, WheelModeTransitionResult2D):
        raise TypeError("result must be a WheelModeTransitionResult2D.")
    corner_x = result.trailing_corner_world_xz_m[0]
    return [
        _frame_row(frame, result.theta_target_rad, trailing_corner_x_m=corner_x)
        for frame in result.frames
    ]


def plot_left_rim_ready_frame_2d(
    result: WheelModeTransitionResult2D,
    *,
    axes=None,
):
    """Draw the right-to-left handover and the corner-arrival state side by side.

    The handover frame is where ``LEFT_RIM_READY`` first becomes true; the
    corner frame is where it is actually required.  Showing both makes visible
    how much left rim the top roll spends between them.
    """

    if not isinstance(result, WheelModeTransitionResult2D):
        raise TypeError("result must be a WheelModeTransitionResult2D.")
    selected = []
    if result.left_rim_handover_step is not None:
        selected.append(
            ("LEFT_RIM_READY (handover)", result.frames[result.left_rim_handover_step])
        )
    selected.append(("TRAILING_CORNER_ARRIVAL", result.final_frame))
    if axes is None:
        figure, axes = plt.subplots(
            1, len(selected), figsize=(6.4 * len(selected), 4.8), squeeze=False
        )
        axes = axes[0]
    else:
        figure = axes[0].figure
    for ax, (label, frame) in zip(axes, selected):
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        alpha = (
            "n/a" if frame.alpha_rad is None
            else f"{np.rad2deg(frame.alpha_rad):.1f} deg"
        )
        ax.set_title(
            f"{label}\ntheta={np.rad2deg(frame.theta_rad):.1f} deg, "
            f"rim={frame.active_rim}, alpha={alpha}",
            fontsize=10,
        )
        ax.legend().set_visible(False)
    figure.tight_layout()
    return figure, axes


def animate_wheel_mode_transition_2d(
    result: WheelModeTransitionResult2D,
    *,
    interval_ms: int = 110,
    frame_stride: int = 2,
    repeat: bool = False,
    show: bool = False,
):
    """Animate Step 7R + 8R: retract to the wheel state, then roll to the corner."""

    if not isinstance(result, WheelModeTransitionResult2D):
        raise TypeError("result must be a WheelModeTransitionResult2D.")
    if min(interval_ms, frame_stride) <= 0:
        raise ValueError("interval and stride must be positive.")
    frames = list(result.frames[::frame_stride])
    if frames[-1] is not result.final_frame:
        frames.append(result.final_frame)
    points = np.vstack([
        np.vstack((
            frame.scene.geometry.points_world_xz_m,
            np.asarray(frame.hip_position_world_xz_m)[None, :],
        ))
        for frame in frames
    ])
    x_pad = max(0.04, 0.08 * float(np.ptp(points[:, 0])))
    z_pad = max(0.04, 0.08 * float(np.ptp(points[:, 1])))
    x_limits = (float(np.min(points[:, 0]) - x_pad),
                float(np.max(points[:, 0]) + x_pad))
    z_limits = (
        min(frames[0].scene.terrain.ground_height_m - 0.03,
            float(np.min(points[:, 1]) - z_pad)),
        float(np.max(points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))
    corner = result.trailing_corner_world_xz_m

    def draw(index: int):
        frame = frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        hip = np.asarray(
            [item.hip_position_world_xz_m for item in frames[: index + 1]],
            dtype=float,
        )
        ax.plot(hip[:, 0], hip[:, 1], "--", color="#7c3aed",
                linewidth=1.6, label="hip path")
        contact = np.asarray(
            [
                item.contact_point_world_xz_m
                for item in frames[: index + 1]
                if item.contact_point_world_xz_m is not None
            ],
            dtype=float,
        )
        if contact.size:
            ax.plot(contact[:, 0], contact[:, 1], ".", color="#dc2626",
                    markersize=4.5, label="contact path")
        ax.plot([corner[0]], [corner[1]], "*", color="black",
                markersize=13, zorder=9)
        phase = traversal_phase_label(
            frame, result.theta_target_rad, trailing_corner_x_m=corner[0]
        )
        ax.set_title(
            f"Step 7R+8R {phase}: theta={np.rad2deg(frame.theta_rad):.1f} deg, "
            f"beta={np.rad2deg(frame.beta_rad):.1f} deg, rim={frame.active_rim}"
        )
        ax.text(
            0.01, 0.02,
            f"contact forward={frame.contact_forward_displacement_m:.4f} m\n"
            f"hip forward={frame.hip_forward_displacement_m:.4f} m\n"
            f"hip z={frame.hip_position_world_xz_m[1]:.4f} m "
            f"(contact-driven, not fixed)\n"
            f"rotation={np.rad2deg(frame.accumulated_rotation_rad):.1f} deg\n"
            f"no-slip residual={abs(frame.no_slip_tangent_residual_m):.2e} m\n"
            f"collision={frame.collision}",
            transform=ax.transAxes, fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(figure, draw, frames=len(frames),
                              interval=interval_ms, repeat=repeat, blit=False)
    draw(0)
    if show:
        plt.show()
    return animation
