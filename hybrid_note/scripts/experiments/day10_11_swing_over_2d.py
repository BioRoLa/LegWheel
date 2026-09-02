"""Day 10--11 Step 5: ``SWING_OVER`` -- the candidate nobody had measured.

Strategy ``#5`` of spec 2.6 crosses the obstacle in one swing without ever
touching the top.  Steps 1--4 never touched it: Step 1's showcases are
``onto`` and ``off`` only, and Steps 2/3 sweep the two halves of a landing.
So the Step 5 completion criterion "every candidate has a concession value"
cannot be met without measuring it here.

**Its internal freedom is ``theta``, and that is forced rather than chosen.**
The apex sits ``clearance`` above the obstacle top, so mid-flight the foot is
roughly ``h + clearance`` above the ground while the hip is at its standing
height.  The leg has to span that gap, and its *shortest* reach is at
``theta = MIN_THETA_DEG`` -- one wheel radius, 143.8 mm.  A probe at
``h = 60 mm, theta = 60 deg`` put the foot 130 mm below the hip at the apex and
failed ``IK_NOT_CONVERGED`` for exactly that reason.

``HipTrajectory2D`` is a straight line, so there is no knob that arches the hip
mid-flight -- the same limitation Step 2b hit.  The only way to raise the hip
over the obstacle is to raise it at **both** ends at once, and the consistent
way to do that is a more extended stance.  Hence: theta is the knob, and the
strategy's price is paid in stance height and stride, not in hip excursion.

**Why its hip excursion is genuinely zero.**  Both ends stand on the lower
ground at the same theta, so the straight-line hip trajectory is horizontal.
That is a real property, not an artefact -- and it is why ``#5`` wins the body
comparison wherever it is feasible.  What it costs instead is reported
separately as ``stance_hip_above_min_m``, because whether that is paid depends
on the gait *outside* the obstacle, which is Day 15--16's question.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    build_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
    generate_swing_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    approach_hip_x_for_clearance_2d,
    standing_scene_2d,
)

#: The hip height of the most retracted stance, ``theta = MIN_THETA_DEG``.
#: Measured once here so the reason a cell failed can name it.
MIN_STANCE_HIP_M = 0.1438


@dataclass(frozen=True)
class SwingOverSettings2D:
    """Everything frozen for the ``#5`` sweep."""

    x_start_m: float = 0.10
    arc_samples: int = 121
    collision_arc_samples: int = 61
    swing_duration_s: float = 0.6
    sample_count: int = 31

    #: The apex sits this far above the obstacle top.  Day 8--9's default, kept
    #: so ``#5`` is not quietly given an easier corridor than ``#4``.
    apex_clearance_m: float = 0.03

    #: theta is the strategy's own freedom, so it is a ladder, not a constant.
    #: It runs to the mechanical maximum because the whole point of ``#5`` is
    #: that a more extended stance is what buys the clearance underneath.
    theta_ladder_deg: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 80.0, 85.0)

    #: Front-face clearance for the take-off stance, using Step 0's shared
    #: definition.  The landing is mirrored about the trailing edge, so the
    #: swing is symmetric and the stride is what the geometry dictates.
    clearance_ladder_m: tuple[float, ...] = (0.04, 0.06, 0.10)

    #: Time is a repair knob here for the same reason as Step 3: the foot
    #: arrives on the lower ground with the whole apex height behind it.
    duration_scale_ladder: tuple[float, ...] = (1.0, 1.5)

    def spec_for(self, height_m: float, top_length_m: float) -> SharedTerrainSpec2D:
        return SharedTerrainSpec2D(
            height_m=float(height_m),
            top_length_m=float(top_length_m),
            x_start_m=self.x_start_m,
            arc_samples=self.arc_samples,
        )


@lru_cache(maxsize=4096)
def _approach_hip_x_m(
    height_m: float, x_start_m: float, arc_samples: int,
    theta_rad: float, clearance_m: float,
) -> float:
    """Cached front-face bisection.

    It costs 60 contact queries, and it does **not depend on the top length**:
    the stance sits in front of the leading edge and the bisection only ever
    looks at that face.  A ``(h, L_top)`` sweep therefore repeats the identical
    solve once per column, which is the whole runtime of this module.
    """

    spec = SharedTerrainSpec2D(
        height_m=height_m, top_length_m=0.35,
        x_start_m=x_start_m, arc_samples=arc_samples,
    )
    return approach_hip_x_for_clearance_2d(spec, theta_rad, clearance_m)


@dataclass(frozen=True)
class SwingOverCell2D:
    """One ``(h, L_top)`` of the over-swing map, at its cheapest theta."""

    height_m: float
    top_length_m: float
    feasible: bool
    theta_deg: float | None = None
    approach_clearance_m: float | None = None
    duration_scale: float = 1.0
    stride_m: float | None = None
    stance_hip_m: float | None = None
    min_clearance_m: float | None = None
    failure: str | None = None
    reason: str = ""
    evaluations: int = 0
    planner_refusals: int = 0
    seconds: float = 0.0

    @property
    def stance_hip_above_min_m(self) -> float | None:
        """How much higher than the most retracted stance the body must sit.

        **Not** folded into the body comparison.  ``#5``'s hip excursion during
        the crossing is zero; this is the price of being in the stance at all,
        and whether it is already paid depends on the gait between obstacles.
        """

        if self.stance_hip_m is None:
            return None
        return float(self.stance_hip_m - MIN_STANCE_HIP_M)

    def as_dict(self) -> dict:
        return {
            "obstacle_mm": self.height_m * 1e3,
            "top_length_m": self.top_length_m,
            "feasible": self.feasible,
            "theta_deg": self.theta_deg,
            "approach_clearance_mm": (
                None if self.approach_clearance_m is None
                else self.approach_clearance_m * 1e3
            ),
            "duration_scale": self.duration_scale,
            "stride_mm": None if self.stride_m is None else self.stride_m * 1e3,
            "stance_hip_mm": (
                None if self.stance_hip_m is None else self.stance_hip_m * 1e3
            ),
            "stance_hip_above_min_mm": (
                None if self.stance_hip_above_min_m is None
                else self.stance_hip_above_min_m * 1e3
            ),
            "min_clearance_mm": (
                None if self.min_clearance_m is None else self.min_clearance_m * 1e3
            ),
            "failure": self.failure,
            "reason": self.reason,
            "evaluations": self.evaluations,
            "planner_refusals": self.planner_refusals,
            "seconds": self.seconds,
        }


def minimum_swing_over_2d(
    height_m: float, top_length_m: float, settings: SwingOverSettings2D
) -> SwingOverCell2D:
    """The least extended stance that gets the leg over this obstacle.

    theta is walked from the most retracted upward and the first success wins,
    because a more extended stance is strictly more expensive: it holds the
    body higher for the whole crossing and lengthens the stride the gait has to
    supply.  ``clearance`` and ``duration`` are repairs inside each rung, not
    traded against theta.
    """

    import time

    started = time.perf_counter()
    evaluations = refusals = 0
    best_failure: str | None = None
    best_reason = ""

    for theta_deg in settings.theta_ladder_deg:
        spec = settings.spec_for(height_m, top_length_m)
        theta = float(np.deg2rad(theta_deg))
        for clearance_m in settings.clearance_ladder_m:
            try:
                hip_x = _approach_hip_x_m(
                    float(height_m), settings.x_start_m, settings.arc_samples,
                    theta, float(clearance_m),
                )
                trailing_x = spec.x_start_m + float(top_length_m)
                # Mirror the take-off stance about the trailing edge, so the
                # landing has the same clearance behind the obstacle that the
                # take-off has in front of it.
                landing_x = trailing_x + (spec.x_start_m - hip_x)
                start = standing_scene_2d(
                    spec, theta, hip_x_m=hip_x, support_height_m=0.0
                )
                target = standing_scene_2d(
                    spec, theta, hip_x_m=landing_x, support_height_m=0.0
                )
                base = build_swing_request_2d(
                    start, target,
                    clearance_m=settings.apex_clearance_m,
                    swing_duration_s=settings.swing_duration_s,
                    sample_count=settings.sample_count,
                )
            except (ValueError, KeyError) as error:
                # A stance that penetrates the front face is not a stance; it
                # is not the swing failing, so it must not be recorded as one.
                best_failure = best_failure or "STANCE_INVALID"
                best_reason = f"theta {theta_deg:.0f} deg, c {clearance_m * 1e3:.0f} mm: {error}"
                continue

            for scale in settings.duration_scale_ladder:
                request = base
                if scale != 1.0:
                    from dataclasses import replace

                    request = replace(
                        base, swing_duration_s=base.swing_duration_s * float(scale)
                    )
                try:
                    plan = generate_swing_2d(
                        request, arc_samples=settings.collision_arc_samples
                    )
                except ValueError:
                    # Day 8--9's planner refuses to judge a leg that finished in
                    # the air (log trap 18).  Counted, never silently dropped.
                    evaluations += 1
                    refusals += 1
                    continue
                evaluations += 1
                if plan.valid:
                    stance_hip = float(
                        start.hip_pose.position_world_xz_m[1]
                    )
                    return SwingOverCell2D(
                        height_m=float(height_m), top_length_m=float(top_length_m),
                        feasible=True, theta_deg=float(theta_deg),
                        approach_clearance_m=float(clearance_m),
                        duration_scale=float(scale),
                        stride_m=float(landing_x - hip_x),
                        stance_hip_m=stance_hip,
                        min_clearance_m=(
                            None if plan.collision is None
                            else plan.collision.minimum_clearance_m
                        ),
                        reason=(
                            f"theta {theta_deg:.0f} deg at c "
                            f"{clearance_m * 1e3:.0f} mm spans "
                            f"{(landing_x - hip_x) * 1e3:.0f} mm."
                        ),
                        evaluations=evaluations, planner_refusals=refusals,
                        seconds=time.perf_counter() - started,
                    )
                if best_failure is None or best_failure == "STANCE_INVALID":
                    best_failure = plan.failure.name
                    best_reason = (
                        f"theta {theta_deg:.0f} deg, c {clearance_m * 1e3:.0f} mm, "
                        f"duration x{scale:g}: {plan.failure.name}"
                    )

    return SwingOverCell2D(
        height_m=float(height_m), top_length_m=float(top_length_m), feasible=False,
        failure=best_failure, reason=best_reason or "no stance was even valid.",
        evaluations=evaluations, planner_refusals=refusals,
        seconds=time.perf_counter() - started,
    )


def _task(args):
    height_m, top_length_m, settings = args
    return minimum_swing_over_2d(height_m, top_length_m, settings)


def run_swing_over_cells_2d(
    tasks: Sequence[tuple[float, float, SwingOverSettings2D]],
    *,
    workers: int | None = None,
    on_result: Callable[[int, int, SwingOverCell2D], None] | None = None,
) -> list[SwingOverCell2D]:
    """Run an explicit task list in parallel, preserving order.

    ``on_result`` is called as each cell lands, with ``(done, total, cell)``.
    It exists because ``pool.map`` returns nothing until the whole list is
    finished, and a cell here can take minutes: without it a sweep of this size
    is a black box for its entire runtime, which is how the earlier steps'
    interrupted runs (implementation log trap 12) cost hours.
    """

    if workers is not None and workers <= 1:
        results = []
        for index, task in enumerate(tasks):
            cell = _task(task)
            results.append(cell)
            if on_result is not None:
                on_result(index + 1, len(tasks), cell)
        return results

    results: list[SwingOverCell2D | None] = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(_task, task): index for index, task in enumerate(tasks)
        }
        done = 0
        for future in as_completed(futures):
            index = futures[future]
            cell = future.result()
            results[index] = cell
            done += 1
            if on_result is not None:
                on_result(done, len(tasks), cell)
    return [cell for cell in results if cell is not None]


def swing_over_rows(cells: Sequence[SwingOverCell2D]) -> list[dict]:
    return [cell.as_dict() for cell in cells]
