"""Day 8--9 Step 2: the Cartesian quintic Bezier swing path (2D, gamma = 0).

This module fills in the Cartesian half of ``SwingResult2D``: time, position
and velocity for every sample, plus the ``(rim, alpha)`` each sample is asking
the leg to place there.  It does **not** solve IK, check joint limits, or
query collisions -- those stay in Steps 4--8, and a result returned from here
is therefore still ``valid=False`` with ``failure=NOT_EVALUATED``.

Curve construction, following the planning note section 8::

    P0 = start contact position
    P1 = lift-off shaping        (P'(0) = 5 (P1 - P0))
    P2 = upper swing region
    P3 = upper swing region
    P4 = touchdown approach      (P'(1) = 5 (P5 - P4))
    P5 = target touchdown position

The first version keeps ``P1 = P0`` and ``P4 = P5`` so both endpoint
velocities are exactly zero, which is what Step 8 asks for as its starting
point.  ``liftoff_fraction`` / ``touchdown_fraction`` expose those two control
points for later work without changing any caller.

The Bernstein evaluation itself is the existing ``legwheel.bezier.Bezier``;
nothing here re-implements a Bezier basis.
"""

from __future__ import annotations

from math import comb

import numpy as np
from numpy.typing import NDArray

from legwheel.bezier import Bezier
from legwheel.planners.hybrid import RimId

from .cartesian_swing_contract_2d import (
    SwingFailure,
    SwingRequest2D,
    SwingResult2D,
    SwingSample2D,
    rim_for_alpha_rad,
    validate_swing_request_2d,
)

#: Where P2 and P3 sit along the start -> target span.  Symmetric fractions
#: keep the arch centred; the resulting horizontal control polygon is
#: monotone, so the contact point never travels backwards mid-swing.
DEFAULT_MID_FRACTIONS = (0.25, 0.75)

#: Grid used to place the apex.  It only affects how precisely the requested
#: apex height is met, not the curve family.
_APEX_GRID_SIZE = 4001


def _bernstein_basis(order: int, s_values: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the ``(len(s), order + 1)`` Bernstein basis matrix."""

    indices = np.arange(order + 1)
    coefficients = np.array([float(comb(order, int(i))) for i in indices], dtype=float)
    s = np.asarray(s_values, dtype=float)[:, None]
    return coefficients * (1.0 - s) ** (order - indices) * s**indices


def swing_apex_height_m(
    request: SwingRequest2D,
    *,
    apex_height_m: float | None = None,
) -> float:
    """Return the apex height the Bezier should reach.

    Step 2 uses the planning note's baseline definition::

        z_apex = max(z_start, z_target) + clearance

    which already handles a target higher than the start.  The obstacle term
    ``z_obstacle along path`` is Step 3's job, and arrives here through the
    ``apex_height_m`` override rather than by changing this rule.
    """

    if apex_height_m is not None:
        return float(apex_height_m)
    z_start = float(request.start.contact_point_world_xz_m[1])
    z_target = float(request.target.target_point_world_xz_m[1])
    return max(z_start, z_target) + float(request.target.clearance_m)


def quintic_swing_control_points_2d(
    start_point_world_xz_m,
    target_point_world_xz_m,
    apex_height_m: float,
    *,
    liftoff_fraction: float = 0.0,
    touchdown_fraction: float = 0.0,
    liftoff_rise_m: float = 0.0,
    touchdown_drop_m: float = 0.0,
    mid_fractions: tuple[float, float] = DEFAULT_MID_FRACTIONS,
) -> NDArray[np.float64]:
    """Build the six control points of one swing, as a ``(6, 2)`` array.

    ``P2`` and ``P3`` share one height, solved so the curve's maximum equals
    ``apex_height_m``.  Setting their height *to* the apex would undershoot it:
    a Bezier passes through its endpoints but only approaches its interior
    control points, so the height has to be solved for, not assigned.

    The four endpoint knobs are the two halves of what the planning note calls
    "control lift-off and touchdown velocity through P1 and P4": the
    ``_fraction`` pair moves them along the span, the ``_m`` pair moves them
    vertically.  Without the vertical pair the touchdown velocity is
    structurally horizontal, and Step 8's normal-speed check could never see a
    non-zero value.  ``touchdown_drop_m`` raises P4 above the target, so the
    curve arrives from above at ``5 * touchdown_drop_m / duration``.
    """

    start = np.asarray(start_point_world_xz_m, dtype=float)
    target = np.asarray(target_point_world_xz_m, dtype=float)
    if start.shape != (2,) or target.shape != (2,):
        raise ValueError("start and target must be world [x, z] pairs.")
    if not np.isfinite(apex_height_m):
        raise ValueError("apex_height_m must be finite.")
    first_fraction, second_fraction = (float(item) for item in mid_fractions)
    for name, value in (
        ("liftoff_fraction", liftoff_fraction),
        ("touchdown_fraction", touchdown_fraction),
        ("mid_fractions[0]", first_fraction),
        ("mid_fractions[1]", second_fraction),
    ):
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must lie in [0, 1].")
    for name, value in (
        ("liftoff_rise_m", liftoff_rise_m),
        ("touchdown_drop_m", touchdown_drop_m),
    ):
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative.")
    if first_fraction > second_fraction:
        raise ValueError("mid_fractions must be non-decreasing.")

    span_x = float(target[0] - start[0])
    x_controls = np.array(
        [
            start[0],
            start[0] + float(liftoff_fraction) * span_x,
            start[0] + first_fraction * span_x,
            start[0] + second_fraction * span_x,
            target[0] - float(touchdown_fraction) * span_x,
            target[0],
        ],
        dtype=float,
    )

    # Solve the shared P2/P3 height.  With z2 = z3 = h the curve height is
    # affine in h, so the exact h is a single minimum over the grid instead of
    # an iterative search.
    liftoff_z = start[1] + float(liftoff_rise_m)
    touchdown_z = target[1] + float(touchdown_drop_m)

    grid = np.linspace(0.0, 1.0, _APEX_GRID_SIZE)[1:-1]
    basis = _bernstein_basis(5, grid)
    fixed_z = np.array([start[1], liftoff_z, touchdown_z, target[1]], dtype=float)
    fixed_part = basis[:, [0, 1, 4, 5]] @ fixed_z
    apex_part = basis[:, 2] + basis[:, 3]
    required = (float(apex_height_m) - fixed_part) / apex_part
    apex_control_z = float(np.min(required))

    z_controls = np.array(
        [start[1], liftoff_z, apex_control_z, apex_control_z, touchdown_z, target[1]],
        dtype=float,
    )
    controls = np.column_stack([x_controls, z_controls])
    controls.setflags(write=False)
    return controls


def sample_quintic_swing_2d(
    control_points_world_xz_m,
    *,
    duration_s: float,
    sample_count: int,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Sample one control polygon into ``(time, position, velocity)``.

    The velocity comes from the analytic hodograph -- the degree-4 Bezier on
    ``5 (P_{i+1} - P_i)`` -- divided by the duration, not from differencing the
    sampled positions.  Step 8 has to judge touchdown speed against a
    threshold, so the endpoint velocity must be exact rather than a finite
    difference that depends on ``sample_count``.
    """

    controls = np.asarray(control_points_world_xz_m, dtype=float)
    if controls.shape != (6, 2):
        raise ValueError("control_points_world_xz_m must have shape (6, 2).")
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive.")
    if int(sample_count) < 2:
        raise ValueError("sample_count must be at least 2.")

    position_curve = Bezier([point for point in controls])
    velocity_curve = Bezier([5.0 * (controls[i + 1] - controls[i]) for i in range(5)])

    s_values = np.linspace(0.0, 1.0, int(sample_count))
    positions = np.array([position_curve.getBzPoint(float(s)) for s in s_values], dtype=float)
    velocities = (
        np.array([velocity_curve.getBzPoint(float(s)) for s in s_values], dtype=float)
        / float(duration_s)
    )
    times = s_values * float(duration_s)
    return times, positions, velocities


def swing_alpha_schedule_rad(
    request: SwingRequest2D,
    s_values: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Interpolate the contact parameter from the start alpha to the target.

    A swing is a *contact-state* relocation, so the point being placed at
    ``p(s)`` is not one fixed material point unless the two contact states
    happen to share a rim and an alpha.  Alpha is a single global rim
    parameter, so walking it from start to target also walks the rim
    identity -- ``FootRim -> LowerRim`` needs no special case.
    """

    s = np.asarray(s_values, dtype=float)
    start_alpha = float(request.start.alpha_rad)
    target_alpha = float(request.target.target_alpha_rad)
    return start_alpha + s * (target_alpha - start_alpha)


def generate_swing_path_2d(
    request: SwingRequest2D,
    *,
    apex_height_m: float | None = None,
    liftoff_fraction: float = 0.0,
    touchdown_fraction: float = 0.0,
    liftoff_rise_m: float = 0.0,
    touchdown_drop_m: float = 0.0,
    mid_fractions: tuple[float, float] = DEFAULT_MID_FRACTIONS,
) -> SwingResult2D:
    """Step 2: turn one request into a sampled Cartesian swing path.

    The returned result is deliberately still ``valid=False``: a path exists,
    but nothing has checked that the leg can follow it.  Steps 4--8 fill in the
    joint, collision and touchdown fields and only then may set ``valid``.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")

    problems = validate_swing_request_2d(request)
    if problems:
        return SwingResult2D(
            request=request,
            valid=False,
            failure=SwingFailure.INVALID_REQUEST,
            failure_detail=" / ".join(problems),
        )

    apex_m = swing_apex_height_m(request, apex_height_m=apex_height_m)
    controls = quintic_swing_control_points_2d(
        request.start.contact_point_world_xz_m,
        request.target.target_point_world_xz_m,
        apex_m,
        liftoff_fraction=liftoff_fraction,
        touchdown_fraction=touchdown_fraction,
        liftoff_rise_m=liftoff_rise_m,
        touchdown_drop_m=touchdown_drop_m,
        mid_fractions=mid_fractions,
    )
    times, positions, velocities = sample_quintic_swing_2d(
        controls,
        duration_s=request.swing_duration_s,
        sample_count=request.sample_count,
    )
    s_values = times / request.swing_duration_s
    alphas = swing_alpha_schedule_rad(request, s_values)

    # The rim is derived from alpha, except at the two endpoints, which are
    # pinned to the contact states the swing was asked to connect.  Alpha is
    # ambiguous exactly on an arc boundary -- the seam at +-40 deg belongs to
    # two arcs that are 45 mm apart in space -- so deriving it there could
    # silently plan a touchdown on a different rim than the target named.
    # Away from a boundary the override is a no-op: the arcs partition alpha.
    rims = [rim_for_alpha_rad(float(alpha)) for alpha in alphas]
    rims[0] = request.start.rim
    rims[-1] = request.target.target_rim

    samples = tuple(
        SwingSample2D(
            index=index,
            time_s=float(times[index]),
            position_world_xz_m=positions[index],
            velocity_world_xz_mps=velocities[index],
            alpha_rad=float(alphas[index]),
            rim=rims[index],
        )
        for index in range(len(times))
    )
    return SwingResult2D(
        request=request,
        samples=samples,
        valid=False,
        failure=SwingFailure.NOT_EVALUATED,
        failure_detail="Cartesian path only; Steps 4-8 have not run.",
    )


def swing_path_endpoint_report(result: SwingResult2D) -> dict:
    """Measure what Step 2's completion criterion actually asks about.

    Endpoint position error, duration, endpoint speed and achieved apex are
    the four things "same generator, correct endpoints and duration" means, so
    they are reported as numbers rather than asserted in a comment.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; run generate_swing_path_2d first.")

    request = result.request
    positions = result.positions_world_xz_m
    velocities = result.velocities_world_xz_mps
    times = result.time_s
    apex_index = int(np.argmax(positions[:, 1]))
    return {
        "start_position_error_mm": float(
            np.linalg.norm(positions[0] - request.start.contact_point_world_xz_m) * 1e3
        ),
        "target_position_error_mm": float(
            np.linalg.norm(positions[-1] - request.target.target_point_world_xz_m) * 1e3
        ),
        "t_first_s": float(times[0]),
        "t_last_s": float(times[-1]),
        "duration_error_s": float(times[-1] - request.swing_duration_s),
        "liftoff_speed_mps": float(np.linalg.norm(velocities[0])),
        "touchdown_speed_mps": float(np.linalg.norm(velocities[-1])),
        "requested_apex_z_m": swing_apex_height_m(request),
        "achieved_apex_z_m": float(positions[apex_index, 1]),
        "apex_at_s": float(times[apex_index] / request.swing_duration_s),
        "x_monotone": bool(
            np.all(np.diff(positions[:, 0]) >= -1e-12)
            or np.all(np.diff(positions[:, 0]) <= 1e-12)
        ),
        "start_alpha_deg": float(np.rad2deg(result.samples[0].alpha_rad)),
        "target_alpha_deg": float(np.rad2deg(result.samples[-1].alpha_rad)),
        "start_rim": result.samples[0].rim.value,
        "target_rim": result.samples[-1].rim.value,
    }


# ---------------------------------------------------------------------------
# Step 3: terrain-aware apex construction
# ---------------------------------------------------------------------------

from dataclasses import dataclass  # noqa: E402  (grouped with the Step 3 types)

from legwheel.planners.hybrid import (  # noqa: E402
    TerrainProfile2D,
    TerrainSurface2D,
    TerrainSurfaceKind,
    query_point_to_terrain_surfaces_2d,
)


def corridor_obstacle_top_surfaces_2d(
    terrain: TerrainProfile2D,
    x_from_m: float,
    x_to_m: float,
    *,
    margin_m: float = 0.0,
) -> tuple[TerrainSurface2D, ...]:
    """Return the obstacle tops whose span overlaps the start -> target corridor.

    Only obstacle tops are considered: a vertical face never rises above the
    top it belongs to, so including faces could not change the apex.  The
    corridor is the x interval between the two contact points, optionally
    widened by ``margin_m``.
    """

    if not isinstance(terrain, TerrainProfile2D):
        raise TypeError("terrain must be a TerrainProfile2D.")
    if not np.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("margin_m must be finite and non-negative.")
    lower = min(float(x_from_m), float(x_to_m)) - float(margin_m)
    upper = max(float(x_from_m), float(x_to_m)) + float(margin_m)
    return tuple(
        surface
        for surface in terrain.surfaces
        if surface.kind is TerrainSurfaceKind.OBSTACLE_TOP
        and surface.span_min_m <= upper
        and surface.span_max_m >= lower
    )


@dataclass(frozen=True)
class ApexConstruction2D:
    """How one apex height was arrived at, and what drove it.

    Recording the governing source matters because the same number can come
    from three different situations -- a high touchdown, a high lift-off, or an
    obstacle in between -- and only the third one is what Step 3 adds.
    """

    z_start_m: float
    z_target_m: float
    clearance_m: float
    corridor_x_min_m: float
    corridor_x_max_m: float
    obstacle_tops: tuple[tuple[str, float], ...]
    governing_source: str
    base_height_m: float
    apex_height_m: float

    @property
    def obstacle_governs(self) -> bool:
        return self.governing_source not in ("start", "target")

    def as_dict(self) -> dict:
        return {
            "z_start_mm": self.z_start_m * 1e3,
            "z_target_mm": self.z_target_m * 1e3,
            "corridor_x_min_m": self.corridor_x_min_m,
            "corridor_x_max_m": self.corridor_x_max_m,
            "corridor_obstacle_tops": ", ".join(
                f"{surface_id}={height_m * 1e3:.0f}mm"
                for surface_id, height_m in self.obstacle_tops
            ),
            "governing_source": self.governing_source,
            "base_height_mm": self.base_height_m * 1e3,
            "clearance_mm": self.clearance_m * 1e3,
            "apex_z_mm": self.apex_height_m * 1e3,
            "obstacle_governs": self.obstacle_governs,
        }


def terrain_aware_apex_2d(
    request: SwingRequest2D,
    *,
    corridor_margin_m: float = 0.0,
) -> ApexConstruction2D:
    """Build the planning note's full apex rule and record what drove it::

        z_apex = max(z_start, z_target, z_obstacle along path) + h_clear

    This is a **trajectory construction heuristic only**.  Raising the apex
    above every obstacle top in the corridor does not make the swing
    collision-free: the curve is below the apex almost everywhere, and the
    leg-wheel is not a point.  Step 6 owns that question.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")

    x_start, z_start = (float(value) for value in request.start.contact_point_world_xz_m)
    x_target, z_target = (float(value) for value in request.target.target_point_world_xz_m)
    surfaces = corridor_obstacle_top_surfaces_2d(
        request.terrain, x_start, x_target, margin_m=corridor_margin_m
    )
    obstacle_tops = tuple(
        (surface.surface_id, float(surface.position_m)) for surface in surfaces
    )

    candidates = [("start", z_start), ("target", z_target)]
    candidates.extend(obstacle_tops)
    governing_source, base_height_m = max(candidates, key=lambda item: item[1])
    return ApexConstruction2D(
        z_start_m=z_start,
        z_target_m=z_target,
        clearance_m=float(request.target.clearance_m),
        corridor_x_min_m=min(x_start, x_target) - float(corridor_margin_m),
        corridor_x_max_m=max(x_start, x_target) + float(corridor_margin_m),
        obstacle_tops=obstacle_tops,
        governing_source=governing_source,
        base_height_m=base_height_m,
        apex_height_m=base_height_m + float(request.target.clearance_m),
    )


def generate_terrain_aware_swing_path_2d(
    request: SwingRequest2D,
    *,
    corridor_margin_m: float = 0.0,
    liftoff_fraction: float = 0.0,
    touchdown_fraction: float = 0.0,
    liftoff_rise_m: float = 0.0,
    touchdown_drop_m: float = 0.0,
    mid_fractions: tuple[float, float] = DEFAULT_MID_FRACTIONS,
) -> SwingResult2D:
    """Step 3: the Step 2 generator with the terrain-aware apex supplied.

    The curve construction is unchanged -- only the apex number differs -- so
    a case with no obstacle in the corridor produces exactly the Step 2 path.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")
    if validate_swing_request_2d(request):
        return generate_swing_path_2d(request)
    construction = terrain_aware_apex_2d(request, corridor_margin_m=corridor_margin_m)
    return generate_swing_path_2d(
        request,
        apex_height_m=construction.apex_height_m,
        liftoff_fraction=liftoff_fraction,
        touchdown_fraction=touchdown_fraction,
        liftoff_rise_m=liftoff_rise_m,
        touchdown_drop_m=touchdown_drop_m,
        mid_fractions=mid_fractions,
    )


def path_point_clearance_samples_2d(
    result: SwingResult2D,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Per-sample clearance of the *contact point* to the terrain, in metres.

    Returns ``(clearance, obstacle_clearance)``; both are negative where the
    point is inside the terrain solid.  ``obstacle_clearance`` ignores the
    ground surface, which is what "did the path clear the obstacle" means when
    both contacts are themselves on the ground.

    This is deliberately a point query, not a leg query.  Step 6 owns the full
    leg-wheel geometry, so a non-negative result here is not a collision-free
    swing.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; generate a path first.")

    terrain = result.request.terrain
    clearances = []
    obstacle_clearances = []
    for sample in result.samples:
        query = query_point_to_terrain_surfaces_2d(sample.position_world_xz_m, terrain)
        if query.point_inside_terrain:
            depth_m = max(gap.penetration_depth_m for gap in query.penetrating_surface_gaps)
            clearances.append(-depth_m)
            obstacle_clearances.append(-depth_m)
            continue
        # Outside the solid: the distance to the nearest terrain boundary.  The
        # signed normal gap is the wrong measure here -- a point standing safely
        # on the ground is metres "behind" the far vertical face of an obstacle,
        # which says nothing about how close it is to hitting anything.
        relevant = query.relevant_surface_gaps
        clearances.append(
            min(gap.euclidean_distance_m for gap in relevant) if relevant else np.inf
        )
        obstacle_gaps = [
            gap for gap in relevant if gap.surface_kind is not TerrainSurfaceKind.GROUND
        ]
        obstacle_clearances.append(
            min(gap.euclidean_distance_m for gap in obstacle_gaps) if obstacle_gaps else np.inf
        )
    return (
        np.asarray(clearances, dtype=float),
        np.asarray(obstacle_clearances, dtype=float),
    )


def path_point_clearance_report_2d(result: SwingResult2D) -> dict:
    """Summarise :func:`path_point_clearance_samples_2d` into one row.

    It answers what Step 3 claims -- "raising the apex lifted the planned
    contact point out of the obstacle" -- and nothing more.
    """

    clearance, obstacle_clearance = path_point_clearance_samples_2d(result)
    worst_index = int(np.argmin(clearance))
    worst_obstacle_index = int(np.argmin(obstacle_clearance))
    return {
        # Always ~0: both endpoints are contacts, so they touch the terrain by
        # definition.  The useful signal here is the penetration count.
        "minimum_point_clearance_mm": float(clearance[worst_index] * 1e3),
        "worst_sample_index": worst_index,
        "worst_sample_x_m": float(result.samples[worst_index].position_world_xz_m[0]),
        "worst_sample_z_m": float(result.samples[worst_index].position_world_xz_m[1]),
        # Obstacle surfaces only; this is what "the path was raised over the
        # obstacle" actually means when both contacts are on the ground.
        "minimum_obstacle_clearance_mm": float(obstacle_clearance[worst_obstacle_index] * 1e3),
        "worst_obstacle_sample_x_m": float(
            result.samples[worst_obstacle_index].position_world_xz_m[0]
        ),
        "penetrating_sample_count": int(np.count_nonzero(clearance < 0.0)),
        "point_path_clears_terrain": bool(not np.any(clearance < 0.0)),
    }
