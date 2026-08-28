"""Day 6--7 Step 11R: obstacle-height x theta-climb full-traversal feasibility.

Every cell of this sweep runs the *whole* Step 10R traversal --
right-rim roll-up, retract to the wheel state, wheel-mode top roll, left-rim
handover, trailing-corner pivot, descent, lower ground -- through
``check_right_up_left_down_traversal``.  A cell is feasible only when the leg
actually finishes on the lower ground behind the obstacle.

This deliberately supersedes the early fixed-hip Step 4 theta sweep, which
only ever asked whether the *climb* worked.  Climbing is the easy half: the
left-rim budget spent on the top decides whether the leg can get back down,
and a climb-only sweep cannot see that.

What is held fixed
------------------
``gamma = 0``, the approach start (hip x and beta on the lower ground in front
of the obstacle), and the obstacle top length.  ``hip_z`` is fixed *as a
condition* -- the leg starts standing on the lower ground -- rather than as a
number, because one number cannot keep legs of different ``theta_climb``
standing on the same ground: the leg radius changes with theta, so a fixed
hip height would leave most of the sweep either floating or penetrating.

A note on "make the top long enough"
------------------------------------
Top length has an *upper* bound as well as a lower one (Step 8R): the left rim
is consumed while rolling along the top, so too long a top leaves too little
rim for the corner pivot.  Making the top longer therefore does not make the
traversal safer, and there is no length that is "long enough" for every cell.
The sweep fixes the top length at a value verified feasible at the reference
height, and reports ``failure_reason`` so that cells failing *because of* the
top length stay distinguishable from cells failing on height or theta.

Not included, deliberately: no swing fallback, no Step 6.75 airborne reset, no
energy optimisation.  A cell that fails is reported, never retried.
"""

from __future__ import annotations

import csv
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    STAGE_APPROACH,
    STAGE_ROLL_DOWN,
    STAGE_ROLL_UP,
    STAGE_WHEEL_TRANSITION,
    ObstacleSpec2D,
    TraversalConstraints2D,
    TraversalInitialState2D,
    _front_face_clearance_m,
    build_single_leg_rolling_scene_2d,
    check_right_up_left_down_traversal,
    traversal_frame_rows,
)

__all__ = [
    "SweepCell2D",
    "TraversalSweepResult2D",
    "run_height_theta_sweep_2d",
    "sweep_rows",
    "write_sweep_csv",
    "rerun_sweep_cell",
    "plot_feasibility_map_2d",
    "plot_feasible_theta_range_2d",
    "plot_minimum_feasible_theta_2d",
    "seam_bridge_for_sampling_m",
    "FAILURE_STAGES",
]

# The requested failure buckets.  ``WHEEL_TRANSITION`` covers two different
# things -- never reaching theta = 17 deg, and reaching it but arriving at the
# corner without the left rim -- so it splits into two labels here.
ROLL_UP_FAIL = "ROLL_UP_FAIL"
RETRACT_FAIL = "RETRACT_FAIL"
LEFT_RIM_TRANSITION_FAIL = "LEFT_RIM_TRANSITION_FAIL"
ROLL_DOWN_FAIL = "ROLL_DOWN_FAIL"
APPROACH_FAIL = "APPROACH_FAIL"

FAILURE_STAGES = (
    APPROACH_FAIL,
    ROLL_UP_FAIL,
    RETRACT_FAIL,
    LEFT_RIM_TRANSITION_FAIL,
    ROLL_DOWN_FAIL,
)

# Measured seam chord at the reference sampling; it scales as 1 / arc_samples.
_SEAM_CHORD_AT_241_M = 2.93e-3


def seam_bridge_for_sampling_m(arc_samples: int, *, safety: float = 3.0) -> float:
    """Seam allowance matched to the rim sampling density.

    The gap at ``alpha = 180 deg`` is a sampling artefact, not a notch: at
    ``theta = 17 deg`` both endpoint samples sit at the same radius.  Its
    measured width therefore scales as ``1 / arc_samples``, so a fixed
    allowance silently rejects the right-to-left handover on coarse grids.
    """

    if arc_samples < 3:
        raise ValueError("arc_samples must be at least 3.")
    chord = _SEAM_CHORD_AT_241_M * 241.0 / float(arc_samples)
    return float(max(5e-3, safety * chord))


def _failure_label(result) -> str | None:
    """Map the traversal's stage names onto the requested failure buckets."""

    stage = result.failure_stage
    if stage is None:
        return None if result.full_success else ROLL_DOWN_FAIL
    if stage == STAGE_APPROACH:
        return APPROACH_FAIL
    if stage == STAGE_ROLL_UP:
        return ROLL_UP_FAIL
    if stage == STAGE_WHEEL_TRANSITION:
        return RETRACT_FAIL if not result.retract_success else LEFT_RIM_TRANSITION_FAIL
    if stage == STAGE_ROLL_DOWN:
        return ROLL_DOWN_FAIL
    return stage


@dataclass(frozen=True)
class SweepCell2D:
    """One (height, theta_climb) cell: its verdict, and the trajectory behind it."""

    obstacle_height_m: float
    theta_climb_rad: float
    row: dict
    trajectory_rows: tuple[dict, ...] = ()
    result: object | None = None       # only populated when run serially

    @property
    def feasible(self) -> bool:
        return bool(self.row["feasible"])

    @property
    def theta_climb_deg(self) -> float:
        return float(np.rad2deg(self.theta_climb_rad))


def _hip_x_for_start_clearance(
    obstacle: ObstacleSpec2D,
    theta_rad: float,
    beta_rad: float,
    clearance_m: float,
    *,
    iterations: int = 60,
) -> float:
    """Hip x that puts the standing leg ``clearance_m`` short of the front face.

    The clearance falls monotonically as the hip advances, so a bisection on
    hip x is enough.  This is what makes "fixed initial approach condition"
    mean the same thing for every leg size in the sweep.
    """

    template = build_single_leg_rolling_scene_2d(
        theta_rad, beta_rad, 0.0, 0.0, **obstacle.scene_kwargs
    )
    hip_z = (
        obstacle.ground_height_m
        - float(template.geometry.points_hip_xz_m[:, 1].min())
    )

    def clearance_at(hip_x: float) -> float:
        scene = build_single_leg_rolling_scene_2d(
            theta_rad, beta_rad, hip_x, hip_z, **obstacle.scene_kwargs
        )
        return _front_face_clearance_m(scene)

    low = obstacle.x_start_m - 1.2       # far enough back to be clear
    high = obstacle.x_start_m            # hip over the leading edge: past the face
    if clearance_at(low) < clearance_m:
        raise ValueError(
            "the leg cannot start that far from the obstacle front face."
        )
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        if clearance_at(middle) > clearance_m:
            low = middle
        else:
            high = middle
    return float(0.5 * (low + high))


@dataclass(frozen=True)
class SweepSettings2D:
    """Everything held fixed across the sweep, in one picklable bundle."""

    obstacle_x_start_m: float = 0.10
    obstacle_width_m: float = 0.35
    arc_samples: int = 121
    gamma_rad: float = 0.0
    obstacle_id: str = "day6_7_obstacle"
    # The fixed approach condition is a *clearance*, not a hip x: the leg
    # radius grows with theta_climb, so one hip x would leave every theta a
    # different distance to roll, and the low-theta cells would run out of
    # foot rim before reaching the face.  Fixing the gap makes the approach
    # identical work in every cell.  Set ``approach_hip_x_m`` to override.
    approach_start_clearance_m: float = 0.04
    approach_hip_x_m: float | None = None
    approach_beta_rad: float = 0.0
    theta_wheel_rad: float = np.deg2rad(17.0)
    release_theta: bool = False
    ground_roll_distance_m: float = 0.02
    max_seam_bridge_m: float | None = None   # None -> matched to arc_samples

    def obstacle(self, height_m: float) -> ObstacleSpec2D:
        return ObstacleSpec2D(
            x_start_m=self.obstacle_x_start_m,
            width_m=self.obstacle_width_m,
            height_m=float(height_m),
            gamma_rad=self.gamma_rad,
            obstacle_id=self.obstacle_id,
            arc_samples=self.arc_samples,
        )

    def initial_state_for(
        self, height_m: float, theta_climb_rad: float
    ) -> TraversalInitialState2D:
        """The start pose for one cell: standing, a fixed gap from the face.

        ``hip_z`` stays ``None`` on purpose -- "standing on the lower ground"
        is the fixed condition, not a fixed number; see the module docstring.
        """

        hip_x = (
            self.approach_hip_x_m if self.approach_hip_x_m is not None
            else _hip_x_for_start_clearance(
                self.obstacle(height_m), theta_climb_rad,
                self.approach_beta_rad, self.approach_start_clearance_m,
            )
        )
        return TraversalInitialState2D(
            hip_x_m=hip_x, beta_rad=self.approach_beta_rad, hip_z_m=None
        )

    @property
    def constraints(self) -> TraversalConstraints2D:
        bridge = (
            seam_bridge_for_sampling_m(self.arc_samples)
            if self.max_seam_bridge_m is None else self.max_seam_bridge_m
        )
        return TraversalConstraints2D(
            theta_wheel_rad=self.theta_wheel_rad,
            max_seam_bridge_m=bridge,
            release_theta=self.release_theta,
            ground_roll_distance_m=self.ground_roll_distance_m,
        )


def _run_one_cell(task):
    """Worker entry point: one full traversal, returned without scene objects.

    Scenes and query results are deliberately not sent back across the process
    boundary -- they are large and the sweep only needs the verdict and the
    trajectory.  ``rerun_sweep_cell`` rebuilds the full object for any cell
    when the scenes themselves are wanted.
    """

    height_m, theta_climb_rad, settings, keep_trajectory = task
    started = time.perf_counter()
    result = check_right_up_left_down_traversal(
        obstacle=settings.obstacle(height_m),
        initial_state=settings.initial_state_for(height_m, theta_climb_rad),
        theta_climb=theta_climb_rad,
        constraints=settings.constraints,
    )
    elapsed = time.perf_counter() - started
    final = result.final_state
    row = {
        "obstacle_height_m": float(height_m),
        "theta_climb_deg": float(np.rad2deg(theta_climb_rad)),
        "theta_climb_rad": float(theta_climb_rad),
        # The requested definition: only a leg that finished on the lower
        # ground behind the obstacle counts as feasible.
        "feasible": bool(result.full_success),
        "all_stages_succeeded": bool(result.feasible),
        "approach_success": bool(result.approach_success),
        "roll_up_success": bool(result.roll_up_success),
        "retract_success": bool(result.retract_success),
        "left_rim_ready_success": bool(result.left_rim_ready_success),
        "roll_down_success": bool(result.roll_down_success),
        "reached_lower_ground_behind_obstacle":
            bool(result.reached_lower_ground_behind_obstacle),
        "failure_stage": _failure_label(result),
        "failure_reason": result.failure_reason,
        "L_transition_m": result.l_transition_m,
        "final_theta_deg": None if final is None else float(np.rad2deg(final.theta_rad)),
        "final_beta_deg": None if final is None else float(np.rad2deg(final.beta_rad)),
        "final_hip_x_m": None if final is None else final.hip_x_m,
        "final_contact_x_m": (
            None if final is None or final.contact_point_world_xz_m is None
            else final.contact_point_world_xz_m[0]
        ),
        "final_active_rim": None if final is None else final.active_rim,
        "final_contact_surface": None if final is None else final.terrain_surface_id,
        "minimum_collision_margin_m": result.minimum_collision_margin_m,
        "frame_count": len(result.trajectory),
        "phases_visited": " -> ".join(result.phases_visited),
        "obstacle_width_m": float(settings.obstacle_width_m),
        "approach_start_clearance_m": float(settings.approach_start_clearance_m),
        "approach_hip_x_m": float(
            settings.initial_state_for(height_m, theta_climb_rad).hip_x_m
        ),
        "arc_samples": int(settings.arc_samples),
        "runtime_s": float(elapsed),
    }
    trajectory = ()
    if keep_trajectory:
        trajectory = tuple(
            {"obstacle_height_m": float(height_m),
             "theta_climb_deg": float(np.rad2deg(theta_climb_rad)),
             **frame_row}
            for frame_row in traversal_frame_rows(result)
        )
    return row, trajectory


@dataclass(frozen=True)
class TraversalSweepResult2D:
    """The whole map, plus the per-cell detail behind every verdict."""

    cells: tuple[SweepCell2D, ...]
    heights_m: tuple[float, ...]
    theta_climb_rad: tuple[float, ...]
    settings: SweepSettings2D
    total_runtime_s: float

    def cell(self, height_m: float, theta_climb_rad: float) -> SweepCell2D:
        for item in self.cells:
            if np.isclose(item.obstacle_height_m, height_m) and np.isclose(
                item.theta_climb_rad, theta_climb_rad
            ):
                return item
        raise KeyError(f"no cell at height={height_m}, theta={theta_climb_rad}")

    @property
    def theta_climb_deg(self) -> tuple[float, ...]:
        return tuple(float(np.rad2deg(value)) for value in self.theta_climb_rad)

    def feasibility_matrix(self) -> np.ndarray:
        """``[height, theta]`` boolean map of full-traversal feasibility."""

        grid = np.zeros((len(self.heights_m), len(self.theta_climb_rad)), dtype=bool)
        for cell in self.cells:
            row = self.heights_m.index(cell.obstacle_height_m)
            column = self.theta_climb_rad.index(cell.theta_climb_rad)
            grid[row, column] = cell.feasible
        return grid

    def failure_stage_matrix(self) -> np.ndarray:
        """``[height, theta]`` map of failure labels; ``None`` where feasible."""

        grid = np.empty(
            (len(self.heights_m), len(self.theta_climb_rad)), dtype=object
        )
        for cell in self.cells:
            row = self.heights_m.index(cell.obstacle_height_m)
            column = self.theta_climb_rad.index(cell.theta_climb_rad)
            grid[row, column] = cell.row["failure_stage"]
        return grid

    def feasible_theta_range_deg(self, height_m: float) -> tuple[float, float] | None:
        """Smallest and largest feasible ``theta_climb`` at one height."""

        values = [
            cell.theta_climb_deg
            for cell in self.cells
            if np.isclose(cell.obstacle_height_m, height_m) and cell.feasible
        ]
        return (min(values), max(values)) if values else None

    def minimum_feasible_theta_deg(self, height_m: float) -> float | None:
        window = self.feasible_theta_range_deg(height_m)
        return None if window is None else window[0]

    def feasible_theta_ranges(self) -> list[dict]:
        """One row per height: the feasible theta window, or ``None``."""

        rows = []
        for height in self.heights_m:
            window = self.feasible_theta_range_deg(height)
            feasible = [
                cell for cell in self.cells
                if np.isclose(cell.obstacle_height_m, height) and cell.feasible
            ]
            rows.append(
                {
                    "obstacle_height_m": height,
                    "feasible_cell_count": len(feasible),
                    "minimum_feasible_theta_deg": None if window is None else window[0],
                    "maximum_feasible_theta_deg": None if window is None else window[1],
                    "feasible_theta_span_deg": (
                        None if window is None else window[1] - window[0]
                    ),
                }
            )
        return rows

    @property
    def feasible_cells(self) -> tuple[SweepCell2D, ...]:
        return tuple(cell for cell in self.cells if cell.feasible)


def run_height_theta_sweep_2d(
    heights_m,
    theta_climb_rad,
    *,
    settings: SweepSettings2D | None = None,
    keep_trajectories: bool = True,
    max_workers: int | None = None,
    progress: bool = True,
) -> TraversalSweepResult2D:
    """Run one full Step 10R traversal per (height, theta_climb) cell.

    Cells are independent, so they run in a process pool.  ``max_workers``
    defaults to a conservative share of the cores: each worker holds a whole
    trajectory in memory, so oversubscribing costs RAM rather than time.
    """

    settings = SweepSettings2D() if settings is None else settings
    heights = tuple(float(value) for value in heights_m)
    thetas = tuple(float(value) for value in theta_climb_rad)
    if not heights or not thetas:
        raise ValueError("the sweep needs at least one height and one theta.")
    if len(set(heights)) != len(heights) or len(set(thetas)) != len(thetas):
        raise ValueError("sweep axes must not repeat a value.")
    if any(value <= 0.0 for value in heights):
        raise ValueError("obstacle heights must be positive.")

    tasks = [
        (height, theta, settings, keep_trajectories)
        for height in heights
        for theta in thetas
    ]
    if max_workers is None:
        max_workers = max(1, min(8, (os.cpu_count() or 2) // 2))

    started = time.perf_counter()
    cells: list[SweepCell2D] = []
    if max_workers == 1:
        for index, task in enumerate(tasks, start=1):
            row, trajectory = _run_one_cell(task)
            cells.append(SweepCell2D(task[0], task[1], row, trajectory))
            if progress:
                _report(index, len(tasks), row)
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as pool:
            for index, ((height, theta, _, _), (row, trajectory)) in enumerate(
                zip(tasks, pool.map(_run_one_cell, tasks)), start=1
            ):
                cells.append(SweepCell2D(height, theta, row, trajectory))
                if progress:
                    _report(index, len(tasks), row)
    return TraversalSweepResult2D(
        cells=tuple(cells),
        heights_m=heights,
        theta_climb_rad=thetas,
        settings=settings,
        total_runtime_s=float(time.perf_counter() - started),
    )


def _report(index: int, total: int, row: dict) -> None:
    verdict = "FEASIBLE" if row["feasible"] else (row["failure_stage"] or "FAILED")
    print(
        f"[{index:3d}/{total}] h={row['obstacle_height_m']:.3f} "
        f"theta={row['theta_climb_deg']:5.1f} deg  {verdict:26s} "
        f"({row['runtime_s']:.1f} s)",
        flush=True,
    )


def rerun_sweep_cell(
    result: TraversalSweepResult2D,
    height_m: float,
    theta_climb_rad: float,
):
    """Rebuild the full traversal object for one cell, scenes included.

    The sweep keeps every cell's trajectory but not its scenes, so this is how
    a cell gets animated or replotted after the fact.  It reruns the identical
    call the worker made, so the result is the same one, not an approximation.
    """

    if not isinstance(result, TraversalSweepResult2D):
        raise TypeError("result must be a TraversalSweepResult2D.")
    settings = result.settings
    return check_right_up_left_down_traversal(
        obstacle=settings.obstacle(height_m),
        initial_state=settings.initial_state_for(height_m, theta_climb_rad),
        theta_climb=theta_climb_rad,
        constraints=settings.constraints,
    )


def sweep_rows(result: TraversalSweepResult2D) -> list[dict]:
    if not isinstance(result, TraversalSweepResult2D):
        raise TypeError("result must be a TraversalSweepResult2D.")
    return [cell.row for cell in result.cells]


def write_sweep_csv(
    result: TraversalSweepResult2D,
    sweep_path,
    *,
    theta_range_path=None,
    trajectory_path=None,
) -> list[Path]:
    """Write the feasibility table, and optionally the ranges and trajectories."""

    written = []

    def dump(path, payload):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload[0]))
            writer.writeheader()
            writer.writerows(payload)
        written.append(path)

    dump(sweep_path, sweep_rows(result))
    if theta_range_path is not None:
        dump(theta_range_path, result.feasible_theta_ranges())
    if trajectory_path is not None:
        rows = [row for cell in result.cells for row in cell.trajectory_rows]
        if rows:
            dump(trajectory_path, rows)
    return written


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------

# One colour per outcome.  Feasible is the only green; every failure keeps its
# own colour so the map shows *where* the traversal broke, not just that it did.
_OUTCOME_COLOURS = {
    "FEASIBLE": "#16a34a",
    APPROACH_FAIL: "#94a3b8",
    ROLL_UP_FAIL: "#2563eb",
    RETRACT_FAIL: "#c026d3",
    LEFT_RIM_TRANSITION_FAIL: "#ea580c",
    ROLL_DOWN_FAIL: "#dc2626",
}


def plot_feasibility_map_2d(result: TraversalSweepResult2D, *, ax=None):
    """Height x theta_climb map, coloured by outcome rather than by pass/fail."""

    if not isinstance(result, TraversalSweepResult2D):
        raise TypeError("result must be a TraversalSweepResult2D.")
    outcomes = list(_OUTCOME_COLOURS)
    index_by_outcome = {name: index for index, name in enumerate(outcomes)}
    grid = np.full((len(result.heights_m), len(result.theta_climb_rad)), np.nan)
    for cell in result.cells:
        row = result.heights_m.index(cell.obstacle_height_m)
        column = result.theta_climb_rad.index(cell.theta_climb_rad)
        label = "FEASIBLE" if cell.feasible else cell.row["failure_stage"]
        grid[row, column] = index_by_outcome.get(label, np.nan)

    if ax is None:
        _, ax = plt.subplots(figsize=(1.05 * len(result.theta_climb_rad) + 4.5,
                                      0.72 * len(result.heights_m) + 3.0))
    colours = [_OUTCOME_COLOURS[name] for name in outcomes]
    ax.imshow(
        grid, origin="lower", aspect="auto", interpolation="nearest",
        cmap=matplotlib.colors.ListedColormap(colours),
        vmin=-0.5, vmax=len(outcomes) - 0.5,
    )
    ax.set_xticks(range(len(result.theta_climb_rad)))
    ax.set_xticklabels([f"{value:.0f}" for value in result.theta_climb_deg])
    ax.set_yticks(range(len(result.heights_m)))
    ax.set_yticklabels([f"{value:.3f}" for value in result.heights_m])
    ax.set_xlabel("theta_climb [deg]")
    ax.set_ylabel("obstacle height [m]")
    ax.set_title(
        "Step 11R: full right-up / left-down traversal feasibility\n"
        f"(top length {result.settings.obstacle_width_m:.2f} m, "
        f"arc_samples {result.settings.arc_samples}, gamma 0)"
    )
    for cell in result.cells:
        row = result.heights_m.index(cell.obstacle_height_m)
        column = result.theta_climb_rad.index(cell.theta_climb_rad)
        ax.text(column, row, "o" if cell.feasible else "x",
                ha="center", va="center", color="white", fontsize=8)
    present = {
        "FEASIBLE" if cell.feasible else cell.row["failure_stage"]
        for cell in result.cells
    }
    ax.legend(
        handles=[
            plt.Line2D([], [], marker="s", linestyle="none", markersize=10,
                       color=_OUTCOME_COLOURS[name], label=name)
            for name in outcomes if name in present
        ],
        loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8.5,
        frameon=False,
    )
    ax.figure.tight_layout()
    return ax


def plot_feasible_theta_range_2d(result: TraversalSweepResult2D, *, ax=None):
    """The feasible theta_climb window at each obstacle height.

    The bar is only the *envelope* between the smallest and largest feasible
    theta.  Individual cells are drawn on top of it, because the feasible set
    is not always contiguous: near the height limit a theta inside the
    envelope can still fail, and a solid bar alone would hide that.
    """

    if not isinstance(result, TraversalSweepResult2D):
        raise TypeError("result must be a TraversalSweepResult2D.")
    rows = result.feasible_theta_ranges()
    if ax is None:
        _, ax = plt.subplots(figsize=(8.6, 0.62 * len(rows) + 2.8))
    span = max(result.heights_m) - min(result.heights_m)
    pad = 0.08 * span if span > 0 else 0.01
    for row in rows:
        height = row["obstacle_height_m"]
        cells = [
            cell for cell in result.cells
            if np.isclose(cell.obstacle_height_m, height)
        ]
        low = row["minimum_feasible_theta_deg"]
        if low is not None:
            ax.plot([low, row["maximum_feasible_theta_deg"]], [height, height],
                    "-", color="#86efac", linewidth=9, solid_capstyle="butt",
                    zorder=1)
        for cell in cells:
            ax.plot([cell.theta_climb_deg], [height],
                    "o" if cell.feasible else "x",
                    color="#15803d" if cell.feasible else "#dc2626",
                    markersize=6.5 if cell.feasible else 6.0,
                    markeredgewidth=1.8, zorder=3)
        label = (
            "no feasible theta" if low is None
            else f"{low:.0f}-{row['maximum_feasible_theta_deg']:.0f} deg "
                 f"({row['feasible_cell_count']}/{len(cells)} cells)"
        )
        ax.text(max(result.theta_climb_deg) + 1.6, height, label, va="center",
                fontsize=8.5,
                color="#dc2626" if low is None else "#111827")
    ax.set_xlabel("theta_climb [deg]")
    ax.set_ylabel("obstacle height [m]")
    ax.set_title("Feasible theta_climb range per obstacle height\n"
                 "(bar = envelope; o = feasible cell, x = infeasible cell)")
    ax.set_xlim(min(result.theta_climb_deg) - 3.0, max(result.theta_climb_deg) + 22.0)
    ax.set_ylim(min(result.heights_m) - pad, max(result.heights_m) + pad)
    ax.grid(True, alpha=0.3)
    ax.figure.tight_layout()
    return ax


def plot_minimum_feasible_theta_2d(result: TraversalSweepResult2D, *, ax=None):
    """Obstacle height against the smallest theta_climb that still works."""

    if not isinstance(result, TraversalSweepResult2D):
        raise TypeError("result must be a TraversalSweepResult2D.")
    rows = result.feasible_theta_ranges()
    heights = [row["obstacle_height_m"] for row in rows
               if row["minimum_feasible_theta_deg"] is not None]
    minima = [row["minimum_feasible_theta_deg"] for row in rows
              if row["minimum_feasible_theta_deg"] is not None]
    infeasible = [row["obstacle_height_m"] for row in rows
                  if row["minimum_feasible_theta_deg"] is None]
    if ax is None:
        _, ax = plt.subplots(figsize=(7.4, 4.6))
    if heights:
        ax.plot(heights, minima, "o-", color="#2563eb", linewidth=1.9,
                markersize=6, label="minimum feasible theta_climb")
    for height in infeasible:
        ax.axvline(height, color="#dc2626", linestyle=":", linewidth=1.4)
    if infeasible:
        ax.plot([], [], ":", color="#dc2626", linewidth=1.4,
                label="no feasible theta_climb")
    ax.set_xlabel("obstacle height [m]")
    ax.set_ylabel("minimum feasible theta_climb [deg]")
    ax.set_title("Obstacle height -> minimum feasible theta_climb\n"
                 "(full traversal back to the lower ground)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8.5)
    ax.figure.tight_layout()
    return ax
