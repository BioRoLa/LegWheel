"""Day 8--9 Step 9: the composed swing planner, and the regression case set.

Steps 2--8 are separate modules on purpose -- each one owns a question and can
be read, tested and blamed on its own.  But the planning note's final
criterion is about a *single* entry point::

    the same generate_swing(), changing only the SwingTarget, produces a
    collision-free, IK-feasible contact-state-to-contact-state swing for
    different touchdown heights, without switching gait logic on terrain height

so this module composes them into ``generate_swing_2d`` and pins that claim
with a fixed set of cases.

The case set deliberately contains **both** directions.  A regression suite
where everything passes only proves the happy path still works; these cases
also include requests the planner must reject, so a change that makes the
checks toothless fails just as loudly as one that breaks a working swing.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .cartesian_swing_contract_2d import (
    SwingFailure,
    SwingRequest2D,
    SwingResult2D,
    active_contact_candidate_2d,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
    validate_swing_request_2d,
)
from .cartesian_swing_path_2d import (
    DEFAULT_MID_FRACTIONS,
    ApexConstruction2D,
    generate_swing_path_2d,
    generate_terrain_aware_swing_path_2d,
    swing_path_endpoint_report,
    terrain_aware_apex_2d,
)
from .cartesian_swing_trajectory_2d import (
    joint_trajectory_report_2d,
    solve_swing_joint_trajectory_2d,
)
from .cartesian_swing_collision_2d import (
    DEFAULT_ARC_SAMPLES,
    SwingCollisionReport2D,
    check_swing_trajectory_collisions_2d,
)
from .cartesian_swing_touchdown_2d import TouchdownValidation2D, validate_swing_touchdown_2d
from .cartesian_swing_velocity_2d import (
    TouchdownVelocity2D,
    validate_swing_touchdown_velocity_2d,
)


@dataclass(frozen=True)
class SwingPlan2D:
    """One swing, planned end to end, with every step's own report kept.

    ``result`` is the answer; the rest is why.  Later steps are ``None`` when
    an earlier one made them meaningless -- an ill-posed request has no
    trajectory to collision-check.
    """

    result: SwingResult2D
    apex: ApexConstruction2D | None = None
    endpoint_report: dict | None = None
    joint_report: dict | None = None
    collision: SwingCollisionReport2D | None = None
    touchdown: TouchdownValidation2D | None = None
    velocity: TouchdownVelocity2D | None = None

    @property
    def valid(self) -> bool:
        return self.result.valid

    @property
    def failure(self) -> SwingFailure:
        return self.result.failure


def generate_swing_2d(
    request: SwingRequest2D,
    *,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    corridor_margin_m: float = 0.0,
    liftoff_fraction: float = 0.0,
    touchdown_fraction: float = 0.0,
    liftoff_rise_m: float = 0.0,
    touchdown_drop_m: float = 0.0,
    mid_fractions: tuple[float, float] = DEFAULT_MID_FRACTIONS,
) -> SwingPlan2D:
    """Plan one contact-state-to-contact-state swing, start to finish.

    Steps 3--8 in order, with no branch anywhere on terrain height: the
    touchdown height enters only as ``SwingTarget2D.target_point_world_xz_m``
    and as whatever surfaces the terrain query happens to find.

    Every step runs even after one fails, so the reports describe the whole
    trajectory rather than stopping at the first symptom; ``result.failure``
    names the earliest problem.  The single exception is an ill-posed request,
    which produces no trajectory to inspect.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")

    problems = validate_swing_request_2d(request)
    if problems:
        return SwingPlan2D(result=generate_swing_path_2d(request))

    apex = terrain_aware_apex_2d(request, corridor_margin_m=corridor_margin_m)
    path = generate_terrain_aware_swing_path_2d(
        request,
        corridor_margin_m=corridor_margin_m,
        liftoff_fraction=liftoff_fraction,
        touchdown_fraction=touchdown_fraction,
        liftoff_rise_m=liftoff_rise_m,
        touchdown_drop_m=touchdown_drop_m,
        mid_fractions=mid_fractions,
    )
    endpoint_report = swing_path_endpoint_report(path)

    solved, _ = solve_swing_joint_trajectory_2d(path)
    joint_report = joint_trajectory_report_2d(solved)

    checked, collision, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=arc_samples)
    validated, touchdown, _ = validate_swing_touchdown_2d(checked)
    final, velocity = validate_swing_touchdown_velocity_2d(validated)

    return SwingPlan2D(
        result=final,
        apex=apex,
        endpoint_report=endpoint_report,
        joint_report=joint_report,
        collision=collision,
        touchdown=touchdown,
        velocity=velocity,
    )


# ---------------------------------------------------------------------------
# The Day 8--9 regression case set
# ---------------------------------------------------------------------------

REGRESSION_THETA_RAD = float(np.deg2rad(60.0))
REGRESSION_CLEARANCE_M = 0.03
REGRESSION_ARC_SAMPLES = 241

#: How far the start contact sits from the obstacle's leading face.  This is
#: the parameter the Step 6 counter-example turned out to be about: the wheel
#: is 145 mm in radius, so a swing that starts closer to a step than that has
#: its tyre inside the vertical face before the contact point has gone
#: anywhere.  It is a property of the *approach*, not of the obstacle height.
REGRESSION_OBSTACLE_X_M = 0.20


def _scene(theta_rad, hip_x_m, support_z_m, **obstacle):
    return build_leg_on_surface_scene_2d(
        theta_rad, 0.0, hip_x_m, support_z_m, arc_samples=REGRESSION_ARC_SAMPLES, **obstacle
    )


def _step_terrain(height_m: float, *, width_m: float = 0.45) -> dict:
    return {
        "obstacle_x_start_m": REGRESSION_OBSTACLE_X_M,
        "obstacle_width_m": width_m,
        "obstacle_height_m": height_m,
        "obstacle_id": "day8_9_regression_obstacle",
    }


def _request(start_scene, target_scene, *, sample_count: int) -> SwingRequest2D:
    return replace(
        build_swing_request_2d(start_scene, target_scene, clearance_m=REGRESSION_CLEARANCE_M),
        sample_count=sample_count,
    )


def feasible_regression_cases_2d(*, sample_count: int = 51) -> dict[str, SwingRequest2D]:
    """The planning note's cases A--E, on geometry the leg can actually fly.

    Only the ``SwingTarget2D`` differs between them; nothing branches on the
    touchdown height.

    ``sample_count`` is part of the case, not a knob to vary underneath it:
    the joint-continuity limit is per sample, so a marginal trajectory can
    change its verdict with resolution.  These five are stable from 21 samples
    up; the default is the resolution they were characterised at.
    """

    theta = REGRESSION_THETA_RAD
    flat = {"obstacle_x_start_m": None}
    step_20 = _step_terrain(0.02)
    step_40 = _step_terrain(0.04)
    gap_40 = _step_terrain(0.04, width_m=0.15)

    ground_before_20 = _scene(theta, 0.00, 0.00, **step_20)
    ground_before_40 = _scene(theta, 0.00, 0.00, **step_40)
    return {
        "A flat -> flat": _request(
            _scene(theta, 0.00, 0.00, **flat),
            _scene(theta, 0.36, 0.00, **flat),
            sample_count=sample_count,
        ),
        "B flat -> 20 mm top": _request(
            ground_before_20,
            _scene(theta, 0.36, 0.02, **step_20),
            sample_count=sample_count,
        ),
        "C flat -> 40 mm top": _request(
            ground_before_40,
            _scene(theta, 0.36, 0.04, **step_40),
            sample_count=sample_count,
        ),
        "D 40 mm top -> flat": _request(
            _scene(theta, 0.36, 0.04, **step_40),
            ground_before_40,
            sample_count=sample_count,
        ),
        "E flat -> flat over a 40 mm step": _request(
            _scene(theta, 0.00, 0.00, **gap_40),
            _scene(theta, 0.60, 0.00, **gap_40),
            sample_count=sample_count,
        ),
    }


def rejected_regression_cases_2d(
    *, sample_count: int = 51
) -> dict[str, tuple[SwingRequest2D, SwingFailure]]:
    """Requests the planner must refuse, with the reason it must give.

    Without these, a change that quietly disabled the collision or touchdown
    checks would still pass the suite above.

    The reasons are stable from 31 samples up.  Below that, "apex beyond the
    theta limit" is reported as a joint discontinuity instead: at 21 samples
    its steps reach 12 deg, which trips the per-sample continuity limit before
    the IK gives up.  Both are true statements about the same bad request, and
    the pipeline reports the earliest one -- another reason the resolution
    belongs to the case.
    """

    theta = REGRESSION_THETA_RAD
    close_step = dict(_step_terrain(0.04), obstacle_x_start_m=0.10)
    gap_40 = _step_terrain(0.04, width_m=0.15)

    # The Step 6 counter-example: the contact point clears the step, the tyre
    # does not, because the start sits closer to it than the wheel radius.
    too_close = _request(
        _scene(theta, 0.00, 0.00, **close_step),
        _scene(theta, 0.20, 0.04, **close_step),
        sample_count=sample_count,
    )
    # Reaching too far across the step: the leg is extended over it at apex.
    too_far = _request(
        _scene(theta, 0.00, 0.00, **gap_40),
        _scene(theta, 0.72, 0.00, **gap_40),
        sample_count=sample_count,
    )
    # An apex the leg cannot reach: theta runs out against its 17 deg limit.
    unreachable = replace(
        _request(
            _scene(theta, 0.00, 0.00, **gap_40),
            _scene(theta, 0.60, 0.00, **gap_40),
            sample_count=sample_count,
        ),
    )
    unreachable = replace(
        unreachable, target=replace(unreachable.target, clearance_m=0.10)
    )
    return {
        "start closer to the step than the wheel radius": (
            too_close,
            SwingFailure.TERRAIN_COLLISION,
        ),
        "reaching too far across the step": (too_far, SwingFailure.TERRAIN_COLLISION),
        "apex beyond the theta limit": (unreachable, SwingFailure.IK_NOT_CONVERGED),
    }


def regression_row(name: str, plan: SwingPlan2D) -> dict:
    """The seven properties the planning note asks each case to be checked on."""

    endpoint = plan.endpoint_report or {}
    joints = plan.joint_report or {}
    collision = plan.collision
    touchdown = plan.touchdown
    velocity = plan.velocity
    samples = plan.result.sample_count
    return {
        "case": name,
        # 1. same planner logic -- structural, see generate_swing_2d
        "planner": "generate_swing_2d",
        # 2. endpoint correctness
        "endpoint_error_mm": max(
            endpoint.get("start_position_error_mm", np.nan),
            endpoint.get("target_position_error_mm", np.nan),
        ),
        "duration_error_s": endpoint.get("duration_error_s", np.nan),
        # 3. IK feasibility
        "ik_converged": (
            f"{joints.get('ik_converged_count', 0)}/{samples}" if joints else "n/a"
        ),
        "max_ik_residual_um": joints.get("max_ik_residual_um", np.nan),
        # 4. joint limits
        "joint_limits_ok": (
            joints.get("joint_limits_ok_count", 0) == samples if joints else False
        ),
        "max_joint_step_deg": joints.get("max_joint_step_deg", np.nan),
        # 5. collision-free
        "collision_free": None if collision is None else collision.collision_free,
        "min_clearance_mm": (
            None
            if collision is None or collision.minimum_clearance_m is None
            else collision.minimum_clearance_m * 1e3
        ),
        # 6. correct touchdown state
        "touchdown_state_ok": None if touchdown is None else touchdown.passed,
        "final_alpha_error_deg": None if touchdown is None else touchdown.alpha_error_deg,
        # 7. touchdown velocity
        "touchdown_normal_mps": None if velocity is None else velocity.normal_velocity_mps,
        "touchdown_velocity_ok": None if velocity is None else velocity.normal_ok,
        "VALID": plan.valid,
        "failure": plan.failure.value,
    }


# ---------------------------------------------------------------------------
# Interface inventory and animation
# ---------------------------------------------------------------------------

from dataclasses import fields as dataclass_fields  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

from legwheel.planners.hybrid import HipPose2D, plot_terrain_profile_2d  # noqa: E402

from .cartesian_swing_contract_2d import (  # noqa: E402
    HipTrajectory2D,
    SwingConstraints2D,
    SwingSample2D,
    SwingStartState2D,
    SwingTarget2D,
)
from .cartesian_swing_collision_2d import swing_sample_scene_2d  # noqa: E402


def _field_rows(role: str, group: str, dataclass_type) -> list[dict]:
    return [
        {
            "role": role,
            "group": group,
            "field": item.name,
            "type": item.type if isinstance(item.type, str) else str(item.type),
        }
        for item in dataclass_fields(dataclass_type)
    ]


#: Knobs of ``generate_swing_2d`` itself, which are not part of any dataclass.
GENERATE_SWING_OPTIONS = (
    ("arc_samples", "int", "tyre samples per arc when rebuilding leg geometry (Step 6)"),
    ("corridor_margin_m", "float", "widen the obstacle corridor used for the apex (Step 3)"),
    ("liftoff_fraction", "float", "move P1 along the span: horizontal lift-off velocity"),
    ("touchdown_fraction", "float", "move P4 along the span: horizontal touchdown velocity"),
    ("liftoff_rise_m", "float", "raise P1: vertical lift-off velocity"),
    ("touchdown_drop_m", "float", "raise P4: vertical touchdown velocity"),
    ("mid_fractions", "tuple[float, float]", "where P2 and P3 sit along the span"),
)


def swing_io_rows() -> list[dict]:
    """Enumerate the swing planner's interface, straight from the dataclasses.

    Generated rather than written out, so it cannot drift from the code: if a
    field is added or renamed, this table changes with it.
    """

    rows: list[dict] = []
    rows += _field_rows("input", "SwingStartState2D", SwingStartState2D)
    rows += _field_rows("input", "SwingTarget2D", SwingTarget2D)
    rows += _field_rows("input", "HipTrajectory2D", HipTrajectory2D)
    rows += _field_rows("input", "SwingConstraints2D", SwingConstraints2D)
    rows += [
        {"role": "input", "group": "SwingRequest2D", "field": item.name,
         "type": item.type if isinstance(item.type, str) else str(item.type)}
        for item in dataclass_fields(SwingRequest2D)
    ]
    rows += [
        {"role": "input", "group": "generate_swing_2d(...)", "field": name, "type": kind}
        for name, kind, _ in GENERATE_SWING_OPTIONS
    ]
    rows += _field_rows("output", "SwingResult2D", SwingResult2D)
    rows += _field_rows("output", "SwingSample2D (per sample)", SwingSample2D)
    rows += _field_rows("output", "SwingPlan2D", SwingPlan2D)
    return rows


def animate_swing_2d(
    plan: SwingPlan2D,
    *,
    interval_ms: int = 110,
    frame_stride: int = 1,
    arc_samples: int = 121,
    repeat: bool = False,
    title_prefix: str = "",
):
    """Animate one planned swing: the leg, the contact path, and the checks.

    Each frame rebuilds the full leg geometry for that sample, so what is drawn
    is the same geometry Step 6 checked -- not a stick figure standing in for
    it.  The per-frame panel shows the quantities that decide the verdict, so a
    failure can be watched happening rather than only read afterwards.
    """

    if not isinstance(plan, SwingPlan2D):
        raise TypeError("plan must be a SwingPlan2D.")
    if not plan.result.samples:
        raise ValueError("plan has no samples to animate.")
    if min(interval_ms, frame_stride) <= 0:
        raise ValueError("interval and stride must be positive.")

    result = plan.result
    request = result.request
    samples = list(result.samples[::frame_stride])
    if samples[-1] is not result.samples[-1]:
        samples.append(result.samples[-1])

    scenes = [swing_sample_scene_2d(result, sample, arc_samples=arc_samples)
              for sample in samples]
    path = result.positions_world_xz_m

    points = np.vstack(
        [np.vstack((scene.geometry.points_world_xz_m,
                    scene.hip_pose.position_world_xz_m[None, :])) for scene in scenes]
        + [path]
    )
    x_pad = max(0.04, 0.06 * float(np.ptp(points[:, 0])))
    z_pad = max(0.03, 0.08 * float(np.ptp(points[:, 1])))
    x_limits = (float(points[:, 0].min() - x_pad), float(points[:, 0].max() + x_pad))
    z_limits = (
        min(request.terrain.ground_height_m - 0.03, float(points[:, 1].min() - z_pad)),
        float(points[:, 1].max() + z_pad),
    )

    figure, ax = plt.subplots(figsize=(11, 5.6))
    failure_index = result.failure_sample_index

    def draw_frame(index: int):
        sample = samples[index]
        scene = scenes[index]
        ax.clear()
        plot_terrain_profile_2d(request.terrain, ax=ax, x_limits_m=x_limits)

        ax.plot(path[:, 0], path[:, 1], '-', color='#d1d5db', lw=1.4, zorder=3)
        travelled = path[: sample.index + 1]
        ax.plot(travelled[:, 0], travelled[:, 1], '-', color='#111827', lw=1.8, zorder=6)

        failed = sample.collision_free is False
        leg_colour = '#dc2626' if failed else '#111827'
        for segment in scene.geometry.link_segments_world_xz_m:
            ax.plot(segment[:, 0], segment[:, 1], color=leg_colour, lw=1.5, zorder=7)
        rim_points = scene.geometry.points_world_xz_m
        ax.plot(rim_points[:, 0], rim_points[:, 1], '.', ms=1.6,
                color=leg_colour, alpha=0.75, zorder=5)
        hip = scene.hip_pose.position_world_xz_m
        ax.plot(hip[0], hip[1], 'o', ms=6, color='#2563eb', zorder=8)
        ax.plot(sample.position_world_xz_m[0], sample.position_world_xz_m[1], 'X',
                ms=11, color='#dc2626' if failed else '#16a34a',
                mec='white', mew=0.9, zorder=10)

        fraction = sample.time_s / request.swing_duration_s
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.2)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('z [m]')
        ax.set_title(
            (f'{title_prefix}\n' if title_prefix else '')
            + f's = {fraction:.2f}   t = {sample.time_s:.3f} s   '
            f'rim = {sample.rim.value}, alpha = {np.rad2deg(sample.alpha_rad):.1f} deg',
            fontsize=10,
        )

        theta_text = 'n/a' if sample.theta_rad is None else f'{np.rad2deg(sample.theta_rad):.1f}'
        beta_text = 'n/a' if sample.beta_rad is None else f'{np.rad2deg(sample.beta_rad):+.1f}'
        residual_text = (
            'n/a' if sample.ik_residual_m is None else f'{sample.ik_residual_m * 1e6:.1f} um'
        )
        step_text = (
            'n/a' if sample.joint_step_rad is None
            else f'{np.rad2deg(sample.joint_step_rad):.2f} deg'
        )
        clearance_text = (
            'n/a' if sample.terrain_clearance_m is None
            else f'{sample.terrain_clearance_m * 1e3:+.2f} mm'
        )
        speed = float(np.linalg.norm(sample.velocity_world_xz_mps))
        ax.text(
            0.01, 0.02,
            f'theta = {theta_text} deg, beta = {beta_text} deg\n'
            f'IK residual = {residual_text},  |dq| = {step_text}\n'
            f'leg clearance = {clearance_text}\n'
            f'|v| = {speed:.3f} m/s\n'
            f'collision_free = {sample.collision_free}\n'
            f'verdict: valid={result.valid}, {result.failure.value}'
            + (f' @ {failure_index}' if failure_index is not None else ''),
            transform=ax.transAxes, fontsize=8,
            bbox={'facecolor': 'white', 'alpha': 0.87, 'edgecolor': '0.75'},
            zorder=15,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure, draw_frame, frames=len(samples),
        interval=interval_ms, repeat=repeat, blit=False,
    )
    draw_frame(0)
    return animation


# ---------------------------------------------------------------------------
# Repairing a colliding swing
# ---------------------------------------------------------------------------

#: How far to escalate an endpoint knob, in metres.  Chosen from the measured
#: Day 8--9 cases: the flat -> 40 mm step needs 20--30 mm of lift-off rise, the
#: swing over a 40 mm step needs 80 mm.
DEFAULT_REPAIR_LADDER_M = (0.01, 0.02, 0.03, 0.05, 0.08, 0.12)


@dataclass(frozen=True)
class SwingRepairAttempt2D:
    """One try, and what it changed."""

    liftoff_rise_m: float
    touchdown_drop_m: float
    collision_free: bool
    minimum_clearance_m: float | None
    first_collision_index: int | None
    collision_side: str | None

    def as_dict(self) -> dict:
        return {
            "liftoff_rise_mm": self.liftoff_rise_m * 1e3,
            "touchdown_drop_mm": self.touchdown_drop_m * 1e3,
            "collision_free": self.collision_free,
            "min_clearance_mm": (
                None
                if self.minimum_clearance_m is None
                else self.minimum_clearance_m * 1e3
            ),
            "first_collision_index": self.first_collision_index,
            "collision_side": self.collision_side,
        }


@dataclass(frozen=True)
class SwingRepairResult2D:
    """The outcome of trying to fly a colliding swing differently."""

    plan: SwingPlan2D
    request: SwingRequest2D
    repaired: bool
    liftoff_rise_m: float
    touchdown_drop_m: float
    swing_duration_s: float
    original_duration_s: float
    attempts: tuple[SwingRepairAttempt2D, ...]
    reason: str | None

    @property
    def duration_was_extended(self) -> bool:
        return self.swing_duration_s > self.original_duration_s + 1e-12

    def as_dict(self) -> dict:
        return {
            "repaired": self.repaired,
            "liftoff_rise_mm": self.liftoff_rise_m * 1e3,
            "touchdown_drop_mm": self.touchdown_drop_m * 1e3,
            "duration_s": self.swing_duration_s,
            "original_duration_s": self.original_duration_s,
            "duration_was_extended": self.duration_was_extended,
            "attempts": len(self.attempts),
            "valid": self.plan.valid,
            "failure": self.plan.failure.value,
            "reason": self.reason,
        }


def _collision_side(plan: SwingPlan2D) -> str | None:
    """Which end of the swing the first collision belongs to."""

    if plan.collision is None or plan.collision.first_collision_index is None:
        return None
    last_index = max(plan.result.sample_count - 1, 1)
    fraction = plan.collision.first_collision_index / last_index
    return "liftoff" if fraction < 0.5 else "touchdown"


def minimum_duration_for_touchdown_drop_s(
    touchdown_drop_m: float,
    normal_speed_limit_mps: float,
) -> float:
    """Shortest swing whose touchdown normal speed stays under the limit.

    The Bezier endpoint derivative is ``P'(1) = 5 (P5 - P4)``, so raising P4 by
    ``d`` lands at ``5 d / T``.  Approaching from higher up therefore costs
    time, and there is no way around it with one control point: the same P4
    sets both how steeply the leg comes in and how fast.
    """

    if touchdown_drop_m < 0.0 or normal_speed_limit_mps <= 0.0:
        raise ValueError("drop must be non-negative and the speed limit positive.")
    return 5.0 * float(touchdown_drop_m) / float(normal_speed_limit_mps)


def repair_swing_2d(
    request: SwingRequest2D,
    *,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    ladder_m: tuple[float, ...] = DEFAULT_REPAIR_LADDER_M,
    extend_duration: bool = True,
    max_duration_s: float = 4.0,
    **generate_kwargs,
) -> SwingRepairResult2D:
    """Try to fly a colliding swing differently, without changing the request.

    The strategy comes straight from what the Day 8--9 cases measured, and is
    deliberately not an optimiser:

    1. **Where** does it hit?  A collision in the first half is the leg arcing
       forward into the step it is standing next to; one in the second half is
       the leg reaching down into the surface it is leaving.  The two need
       opposite fixes, so the side selects the knob.
    2. **Escalate that knob** until the trajectory is collision-free.  Geometry
       only -- the control polygon does not depend on ``swing_duration_s``, so
       this phase can ignore time entirely.
    3. **Then buy back the velocity.**  Only a touchdown drop costs normal
       speed, and only through ``5 d / T``, so the minimum duration follows in
       closed form rather than by search.

    What this cannot repair: anything that is not a collision.  An
    ``IK_NOT_CONVERGED`` means the apex is out of the leg's reach, which is
    fixed by the hip trajectory or the approach geometry -- both *inputs* to
    this planner, not knobs inside it.

    Extending the duration changes the task, not just the path: a 2 s swing is
    a different gait timing decision than a 0.6 s one.  It is reported
    explicitly through ``duration_was_extended`` rather than folded in.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")

    baseline = generate_swing_2d(request, arc_samples=arc_samples, **generate_kwargs)
    attempts: list[SwingRepairAttempt2D] = []

    def record(plan: SwingPlan2D, rise_m: float, drop_m: float) -> SwingRepairAttempt2D:
        collision = plan.collision
        attempt = SwingRepairAttempt2D(
            liftoff_rise_m=rise_m,
            touchdown_drop_m=drop_m,
            collision_free=bool(collision is not None and collision.collision_free),
            minimum_clearance_m=None if collision is None else collision.minimum_clearance_m,
            first_collision_index=None if collision is None else collision.first_collision_index,
            collision_side=_collision_side(plan),
        )
        attempts.append(attempt)
        return attempt

    record(baseline, 0.0, 0.0)

    if baseline.valid:
        return SwingRepairResult2D(
            plan=baseline, request=request, repaired=False,
            liftoff_rise_m=0.0, touchdown_drop_m=0.0,
            swing_duration_s=request.swing_duration_s,
            original_duration_s=request.swing_duration_s,
            attempts=tuple(attempts), reason="the swing already passes; nothing to repair.",
        )
    if baseline.failure is not SwingFailure.TERRAIN_COLLISION:
        return SwingRepairResult2D(
            plan=baseline, request=request, repaired=False,
            liftoff_rise_m=0.0, touchdown_drop_m=0.0,
            swing_duration_s=request.swing_duration_s,
            original_duration_s=request.swing_duration_s,
            attempts=tuple(attempts),
            reason=(
                f"{baseline.failure.value} is not a collision; the endpoint knobs "
                "cannot repair it. Change the hip trajectory or the approach geometry."
            ),
        )

    # Phase 1 -- geometry.  Escalate the knob the collision side calls for, and
    # let the other one grow too if the problem moves to the far end.
    rise_m = drop_m = 0.0
    solved_plan: SwingPlan2D | None = None
    for rung_m in ladder_m:
        # The side is re-read every round: fixing one end can move the problem
        # to the other, and then the other knob is the one that has to grow.
        side = attempts[-1].collision_side or "liftoff"
        if side == "touchdown":
            drop_m = rung_m
        else:
            rise_m = rung_m
        plan = generate_swing_2d(
            request, arc_samples=arc_samples,
            liftoff_rise_m=rise_m, touchdown_drop_m=drop_m, **generate_kwargs
        )
        attempt = record(plan, rise_m, drop_m)
        if attempt.collision_free:
            solved_plan = plan
            break

    if solved_plan is None:
        return SwingRepairResult2D(
            plan=baseline, request=request, repaired=False,
            liftoff_rise_m=rise_m, touchdown_drop_m=drop_m,
            swing_duration_s=request.swing_duration_s,
            original_duration_s=request.swing_duration_s,
            attempts=tuple(attempts),
            reason=(
                f"still colliding at the largest endpoint offset tried "
                f"({max(ladder_m) * 1e3:.0f} mm); this request needs a different "
                "approach geometry, not a different trajectory."
            ),
        )

    # Phase 2 -- time.  Only the touchdown drop costs normal speed.
    duration_s = request.swing_duration_s
    if drop_m > 0.0:
        needed_s = minimum_duration_for_touchdown_drop_s(
            drop_m, request.constraints.touchdown_normal_speed_max_mps
        )
        if needed_s > duration_s:
            if not extend_duration:
                return SwingRepairResult2D(
                    plan=solved_plan, request=request, repaired=False,
                    liftoff_rise_m=rise_m, touchdown_drop_m=drop_m,
                    swing_duration_s=duration_s, original_duration_s=duration_s,
                    attempts=tuple(attempts),
                    reason=(
                        f"collision-free, but landing from {drop_m * 1e3:.0f} mm up needs "
                        f"{needed_s:.2f} s to stay under the touchdown speed limit."
                    ),
                )
            if needed_s > max_duration_s:
                return SwingRepairResult2D(
                    plan=solved_plan, request=request, repaired=False,
                    liftoff_rise_m=rise_m, touchdown_drop_m=drop_m,
                    swing_duration_s=duration_s, original_duration_s=duration_s,
                    attempts=tuple(attempts),
                    reason=(
                        f"would need {needed_s:.2f} s, beyond the {max_duration_s:.2f} s "
                        "allowed for one swing."
                    ),
                )
            duration_s = needed_s

    repaired_request = replace(request, swing_duration_s=duration_s)
    final_plan = generate_swing_2d(
        repaired_request, arc_samples=arc_samples,
        liftoff_rise_m=rise_m, touchdown_drop_m=drop_m, **generate_kwargs
    )
    return SwingRepairResult2D(
        plan=final_plan, request=repaired_request, repaired=bool(final_plan.valid),
        liftoff_rise_m=rise_m, touchdown_drop_m=drop_m,
        swing_duration_s=duration_s, original_duration_s=request.swing_duration_s,
        attempts=tuple(attempts),
        reason=None if final_plan.valid else final_plan.result.failure_detail,
    )


# ---------------------------------------------------------------------------
# Showcase: "here is a step this tall -- can the leg swing onto it?"
# ---------------------------------------------------------------------------

#: How much extra height the body may be asked to gain during the swing.  The
#: hip trajectory is an *input* to this planner, so raising it is a request to
#: the body planner, not something the swing decides on its own -- each rung
#: used is reported so the caller can accept or refuse it.
DEFAULT_HIP_LIFT_LADDER_M = (0.0, 0.02, 0.04, 0.06, 0.08, 0.12)


@dataclass(frozen=True)
class StepSwingShowcase2D:
    """Whether the leg can swing onto a step of a given height, and at what price."""

    obstacle_height_m: float
    feasible: bool
    reason: str | None
    plan: SwingPlan2D | None
    request: SwingRequest2D | None
    #: ``"onto"`` for climbing a step, ``"off"`` for stepping down from one.
    #: The two need different things from the body: climbing wants the hip
    #: *raised*, descending wants it *held back* from dropping with the foot.
    direction: str = "onto"
    liftoff_rise_m: float = 0.0
    touchdown_drop_m: float = 0.0
    hip_lift_m: float = 0.0
    swing_duration_s: float = 0.0
    original_duration_s: float = 0.0
    theta_min_deg: float | None = None
    minimum_clearance_m: float | None = None
    adjustments: tuple[str, ...] = ()

    def as_dict(self) -> dict:
        return {
            "obstacle_mm": self.obstacle_height_m * 1e3,
            "feasible": self.feasible,
            "liftoff_rise_mm": self.liftoff_rise_m * 1e3,
            "touchdown_drop_mm": self.touchdown_drop_m * 1e3,
            "direction": self.direction,
            "hip_lift_mm": self.hip_lift_m * 1e3,
            "duration_s": self.swing_duration_s,
            "theta_min_deg": self.theta_min_deg,
            "min_clearance_mm": (
                None if self.minimum_clearance_m is None else self.minimum_clearance_m * 1e3
            ),
            "adjustments": "; ".join(self.adjustments) if self.adjustments else "none",
            "reason": self.reason,
        }

    def summary(self) -> str:
        """One human-readable line, which is what a showcase actually wants."""

        height_mm = self.obstacle_height_m * 1e3
        verb = "swing up" if self.direction == "onto" else "descent"
        if not self.feasible:
            return f"{height_mm:.0f} mm step ({self.direction}): NO {verb} -- {self.reason}"
        extras = "; ".join(self.adjustments) if self.adjustments else "no adjustment needed"
        return (
            f"{height_mm:.0f} mm step ({self.direction}): {verb} found "
            f"({extras}); theta down to {self.theta_min_deg:.1f} deg, "
            f"clearance {self.minimum_clearance_m * 1e3:.2f} mm, "
            f"duration {self.swing_duration_s:.2f} s"
        )


def _step_swing_request(
    obstacle_height_m: float,
    *,
    obstacle_x_start_m: float,
    obstacle_width_m: float,
    approach_distance_m: float,
    landing_distance_m: float,
    clearance_m: float,
    swing_duration_s: float,
    sample_count: int,
    hip_lift_m: float,
    theta_rad: float,
    leg_arc_samples: int,
) -> SwingRequest2D:
    terrain = {
        "obstacle_x_start_m": obstacle_x_start_m,
        "obstacle_width_m": obstacle_width_m,
        "obstacle_height_m": obstacle_height_m,
        "obstacle_id": "showcase_step",
        "arc_samples": leg_arc_samples,
    }
    start_scene = build_leg_on_surface_scene_2d(
        theta_rad, 0.0, obstacle_x_start_m - approach_distance_m, 0.0, **terrain
    )
    target_scene = build_leg_on_surface_scene_2d(
        theta_rad, 0.0, obstacle_x_start_m + landing_distance_m, obstacle_height_m, **terrain
    )
    request = replace(
        build_swing_request_2d(start_scene, target_scene, clearance_m=clearance_m),
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
    )
    if hip_lift_m > 0.0:
        end_pose = HipPose2D(
            target_scene.hip_pose.position_world_xz_m + np.array([0.0, hip_lift_m])
        )
        request = replace(
            request, hip_trajectory=HipTrajectory2D(start_scene.hip_pose, end_pose)
        )
    return request


def _reachable(request: SwingRequest2D, **generate_kwargs) -> tuple[bool, dict, SwingFailure]:
    """Cheap Steps 3--5 probe: can the leg follow this path at all?

    Skipping the collision check here is what makes a height sweep affordable:
    reachability is decided by the IK, and only trajectories that pass it are
    worth expanding into full leg geometry.
    """

    path = generate_terrain_aware_swing_path_2d(request, **generate_kwargs)
    solved, _ = solve_swing_joint_trajectory_2d(path)
    report = joint_trajectory_report_2d(solved)
    return solved.failure is SwingFailure.NOT_EVALUATED, report, solved.failure


def _standing_fails(
    obstacle_height_m, obstacle_x_start_m, obstacle_width_m,
    hip_x_m, support_z_m, theta_rad, leg_arc_samples,
) -> bool:
    """Whether the leg cannot legally rest at one pose of the showcase scene.

    Building the scene is not enough: it places the leg but does not judge it.
    The contact query is what refuses a pose that is inside the terrain, so it
    is the thing to ask.
    """

    try:
        scene = build_leg_on_surface_scene_2d(
            theta_rad, 0.0, hip_x_m, support_z_m,
            obstacle_x_start_m=obstacle_x_start_m,
            obstacle_width_m=obstacle_width_m,
            obstacle_height_m=obstacle_height_m,
            obstacle_id="showcase_step",
            arc_samples=leg_arc_samples,
        )
        active_contact_candidate_2d(scene)
    except ValueError:
        return True
    return False


def _step_swing_scene_request(
    obstacle_height_m: float,
    *,
    obstacle_x_start_m: float,
    obstacle_width_m: float,
    start_hip_x_m: float,
    start_support_z_m: float,
    target_hip_x_m: float,
    target_support_z_m: float,
    clearance_m: float,
    swing_duration_s: float,
    sample_count: int,
    hip_offset_m: float,
    theta_rad: float,
    leg_arc_samples: int,
) -> SwingRequest2D:
    """Build one showcase request; ``hip_offset_m`` raises where the hip ends up.

    For a climb that means lifting the body higher than the landing pose needs;
    for a descent it means *not* letting the body fall all the way with the
    foot.  Both are the same number: how far above the naive end pose the hip
    finishes.
    """

    terrain = {
        "obstacle_x_start_m": obstacle_x_start_m,
        "obstacle_width_m": obstacle_width_m,
        "obstacle_height_m": obstacle_height_m,
        "obstacle_id": "showcase_step",
        "arc_samples": leg_arc_samples,
    }
    start_scene = build_leg_on_surface_scene_2d(
        theta_rad, 0.0, start_hip_x_m, start_support_z_m, **terrain
    )
    target_scene = build_leg_on_surface_scene_2d(
        theta_rad, 0.0, target_hip_x_m, target_support_z_m, **terrain
    )
    request = replace(
        build_swing_request_2d(start_scene, target_scene, clearance_m=clearance_m),
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
    )
    if hip_offset_m > 0.0:
        end_pose = HipPose2D(
            target_scene.hip_pose.position_world_xz_m + np.array([0.0, hip_offset_m])
        )
        request = replace(
            request, hip_trajectory=HipTrajectory2D(start_scene.hip_pose, end_pose)
        )
    return request


def _run_step_swing_showcase(
    obstacle_height_m: float,
    *,
    direction: str,
    start_hip_x_m: float,
    start_support_z_m: float,
    target_hip_x_m: float,
    target_support_z_m: float,
    obstacle_x_start_m: float,
    obstacle_width_m: float,
    clearance_m: float,
    swing_duration_s: float,
    sample_count: int,
    arc_samples: int,
    leg_arc_samples: int,
    theta_rad: float,
    hip_offset_ladder_m: tuple[float, ...],
    hip_offset_label: str,
    start_knob_name: str,
    landing_knob_name: str,
    allow_hip_offset: bool,
    allow_repair: bool,
    extend_duration: bool,
) -> StepSwingShowcase2D:
    """The two-ceiling search, shared by the climb and the descent showcases."""

    build = lambda offset_m: _step_swing_scene_request(
        obstacle_height_m,
        obstacle_x_start_m=obstacle_x_start_m,
        obstacle_width_m=obstacle_width_m,
        start_hip_x_m=start_hip_x_m,
        start_support_z_m=start_support_z_m,
        target_hip_x_m=target_hip_x_m,
        target_support_z_m=target_support_z_m,
        clearance_m=clearance_m,
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
        hip_offset_m=offset_m,
        theta_rad=theta_rad,
        leg_arc_samples=leg_arc_samples,
    )

    try:
        build(0.0)
    except ValueError as error:
        which = "start" if _standing_fails(
            obstacle_height_m, obstacle_x_start_m, obstacle_width_m,
            start_hip_x_m, start_support_z_m, theta_rad, leg_arc_samples,
        ) else "landing"
        knob = start_knob_name if which == "start" else landing_knob_name
        return StepSwingShowcase2D(
            obstacle_height_m=obstacle_height_m, feasible=False, plan=None, request=None,
            direction=direction,
            reason=(
                f"the leg cannot even stand at the {which} pose of a "
                f"{obstacle_height_m * 1e3:.0f} mm step, so no swing was attempted; "
                f"change {knob}. ({error})"
            ),
        )

    # --- ceiling 1: reach -------------------------------------------------
    offsets = hip_offset_ladder_m if allow_hip_offset else (0.0,)
    chosen_offset_m = None
    report: dict = {}
    last_failure = SwingFailure.NOT_EVALUATED
    for offset_m in offsets:
        ok, report, last_failure = _reachable(build(offset_m))
        if ok:
            chosen_offset_m = offset_m
            break
    if chosen_offset_m is None:
        best_effort_request = build(max(offsets))
        best_effort = generate_swing_2d(best_effort_request, arc_samples=arc_samples)
        return StepSwingShowcase2D(
            obstacle_height_m=obstacle_height_m, feasible=False,
            plan=best_effort, request=best_effort_request, direction=direction,
            reason=(
                f"out of reach: {last_failure.value} even with {hip_offset_label} "
                f"{max(offsets) * 1e3:.0f} mm (theta bottoms out at "
                f"{report.get('theta_min_deg', float('nan')):.1f} deg against its "
                f"{np.rad2deg(SwingConstraints2D().theta_min_rad):.0f} deg limit)."
            ),
            hip_lift_m=max(offsets),
            swing_duration_s=swing_duration_s,
            original_duration_s=swing_duration_s,
            theta_min_deg=report.get("theta_min_deg"),
            minimum_clearance_m=(
                None if best_effort.collision is None
                else best_effort.collision.minimum_clearance_m
            ),
        )

    adjustments: list[str] = []
    if chosen_offset_m > 0.0:
        adjustments.append(f"{hip_offset_label} {chosen_offset_m * 1e3:.0f} mm")

    # --- ceiling 2: fit ---------------------------------------------------
    request = build(chosen_offset_m)
    if allow_repair:
        repair = repair_swing_2d(
            request, arc_samples=arc_samples, extend_duration=extend_duration
        )
        plan, final_request = repair.plan, repair.request
        if repair.liftoff_rise_m > 0.0:
            adjustments.append(f"lift-off raised {repair.liftoff_rise_m * 1e3:.0f} mm")
        if repair.touchdown_drop_m > 0.0:
            adjustments.append(
                f"touchdown approached from {repair.touchdown_drop_m * 1e3:.0f} mm up"
            )
        if repair.duration_was_extended:
            adjustments.append(
                f"swing lengthened {repair.original_duration_s:.2f} -> "
                f"{repair.swing_duration_s:.2f} s"
            )
        liftoff_rise_m, touchdown_drop_m = repair.liftoff_rise_m, repair.touchdown_drop_m
        duration_s = repair.swing_duration_s
        reason = None if plan.valid else (repair.reason or plan.result.failure_detail)
    else:
        plan = generate_swing_2d(request, arc_samples=arc_samples)
        final_request = request
        liftoff_rise_m = touchdown_drop_m = 0.0
        duration_s = request.swing_duration_s
        reason = None if plan.valid else plan.result.failure_detail

    return StepSwingShowcase2D(
        obstacle_height_m=obstacle_height_m, feasible=bool(plan.valid), reason=reason,
        plan=plan, request=final_request, direction=direction,
        liftoff_rise_m=liftoff_rise_m, touchdown_drop_m=touchdown_drop_m,
        hip_lift_m=chosen_offset_m,
        swing_duration_s=duration_s, original_duration_s=swing_duration_s,
        theta_min_deg=(None if plan.joint_report is None else plan.joint_report["theta_min_deg"]),
        minimum_clearance_m=(None if plan.collision is None else plan.collision.minimum_clearance_m),
        adjustments=tuple(adjustments),
    )


def swing_onto_step_2d(
    obstacle_height_m: float,
    *,
    obstacle_x_start_m: float = 0.20,
    obstacle_width_m: float = 0.45,
    approach_distance_m: float = 0.20,
    landing_distance_m: float = 0.16,
    clearance_m: float = 0.03,
    swing_duration_s: float = 0.6,
    sample_count: int = 51,
    arc_samples: int = 61,
    leg_arc_samples: int = 241,
    theta_rad: float = REGRESSION_THETA_RAD,
    hip_lift_ladder_m: tuple[float, ...] = DEFAULT_HIP_LIFT_LADDER_M,
    allow_hip_lift: bool = True,
    allow_repair: bool = True,
    extend_duration: bool = True,
) -> StepSwingShowcase2D:
    """Given a step height, produce a swing **onto** it -- or say why there is none.

    The search follows the two ceilings Day 8--9 measured, in the order they
    bind:

    1. **Reach.**  The apex has to be within the leg's range, which is set by
       how high the hip goes.  Probed with Steps 3--5 only, because the IK
       decides it and the collision check is expensive.
    2. **Fit.**  Then the whole leg has to miss the step, which the endpoint
       control points fix (see :func:`repair_swing_2d`).

    Every adjustment it had to make is reported.  Raising the hip in
    particular is a request to the *body* planner -- the hip trajectory is an
    input here -- so it is never silently folded into the answer.
    """

    if not np.isfinite(obstacle_height_m) or obstacle_height_m <= 0.0:
        raise ValueError("obstacle_height_m must be finite and positive.")

    return _run_step_swing_showcase(
        obstacle_height_m,
        direction="onto",
        start_hip_x_m=obstacle_x_start_m - approach_distance_m,
        start_support_z_m=0.0,
        target_hip_x_m=obstacle_x_start_m + landing_distance_m,
        target_support_z_m=obstacle_height_m,
        obstacle_x_start_m=obstacle_x_start_m,
        obstacle_width_m=obstacle_width_m,
        clearance_m=clearance_m,
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
        arc_samples=arc_samples,
        leg_arc_samples=leg_arc_samples,
        theta_rad=theta_rad,
        hip_offset_ladder_m=hip_lift_ladder_m,
        hip_offset_label="hip raised",
        start_knob_name="approach_distance_m (start further from the step)",
        landing_knob_name="landing_distance_m (land further onto the top)",
        allow_hip_offset=allow_hip_lift,
        allow_repair=allow_repair,
        extend_duration=extend_duration,
    )


#: Fractions of the drop that the hip is allowed to *keep* while the foot goes
#: down.  Descending is not climbing with the sign flipped: the foot has to
#: reach below the surface it is leaving, and a body that falls with it takes
#: the leg's remaining range away exactly when it is needed.
DEFAULT_HIP_HOLD_LADDER = (0.0, 0.25, 0.5, 0.75, 1.0)


def swing_off_step_2d(
    obstacle_height_m: float,
    *,
    obstacle_x_start_m: float = 0.20,
    obstacle_width_m: float = 0.45,
    takeoff_distance_m: float = 0.16,
    landing_distance_m: float = 0.20,
    clearance_m: float = 0.03,
    swing_duration_s: float = 0.6,
    sample_count: int = 51,
    arc_samples: int = 61,
    leg_arc_samples: int = 241,
    theta_rad: float = REGRESSION_THETA_RAD,
    hip_hold_ladder: tuple[float, ...] = DEFAULT_HIP_HOLD_LADDER,
    allow_hip_hold: bool = True,
    allow_repair: bool = True,
    extend_duration: bool = True,
) -> StepSwingShowcase2D:
    """Given a step height, produce a swing **off** it -- or say why there is none.

    Same two ceilings as the climb, but the body knob is the opposite one.
    Climbing asks the hip to rise; descending asks it *not to fall* with the
    foot.  Measured on this leg: at 160 mm a hip that drops with the foot runs
    theta out and collides, while one that keeps half the drop clears with
    about 1.8 mm to spare.

    ``hip_hold_ladder`` is in fractions of the step height, since that is the
    whole distance the body would otherwise descend.
    """

    if not np.isfinite(obstacle_height_m) or obstacle_height_m <= 0.0:
        raise ValueError("obstacle_height_m must be finite and positive.")
    for fraction in hip_hold_ladder:
        if not 0.0 <= fraction <= 1.0:
            raise ValueError("hip_hold_ladder entries must be fractions in [0, 1].")

    obstacle_x_end_m = obstacle_x_start_m + obstacle_width_m
    return _run_step_swing_showcase(
        obstacle_height_m,
        direction="off",
        start_hip_x_m=obstacle_x_end_m - takeoff_distance_m,
        start_support_z_m=obstacle_height_m,
        target_hip_x_m=obstacle_x_end_m + landing_distance_m,
        target_support_z_m=0.0,
        obstacle_x_start_m=obstacle_x_start_m,
        obstacle_width_m=obstacle_width_m,
        clearance_m=clearance_m,
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
        arc_samples=arc_samples,
        leg_arc_samples=leg_arc_samples,
        theta_rad=theta_rad,
        hip_offset_ladder_m=tuple(f * obstacle_height_m for f in hip_hold_ladder),
        hip_offset_label="hip held above the landing pose by",
        start_knob_name="takeoff_distance_m (start further back on the top)",
        landing_knob_name="landing_distance_m (land further from the step)",
        allow_hip_offset=allow_hip_hold,
        allow_repair=allow_repair,
        extend_duration=extend_duration,
    )


def animate_step_swing_2d(showcase: StepSwingShowcase2D, **kwargs):
    """Animate what :func:`swing_onto_step_2d` came back with, feasible or not.

    A refused height is worth watching too: the leg visibly runs out of theta,
    or the tyre visibly enters the step, which is a different kind of answer
    from the word "infeasible".
    """

    if not isinstance(showcase, StepSwingShowcase2D):
        raise TypeError("showcase must be a StepSwingShowcase2D.")
    if showcase.plan is None:
        raise ValueError(
            f"nothing to animate: {showcase.reason}"
        )
    verdict = 'FEASIBLE' if showcase.feasible else 'NOT FEASIBLE'
    extras = '; '.join(showcase.adjustments) if showcase.adjustments else 'no adjustment'
    prefix = (
        f'{showcase.obstacle_height_m * 1e3:.0f} mm step -- {verdict} ({extras})'
    )
    kwargs.setdefault('title_prefix', prefix)
    return animate_swing_2d(showcase.plan, **kwargs)
