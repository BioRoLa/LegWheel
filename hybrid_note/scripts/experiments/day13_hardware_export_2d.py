"""Day 13: write the Hybrid flat gait in the **Walk pipeline's** CSV contract.

``day13_motor_export_2d`` writes what ``create_command_csv_phi`` writes:
``(phi_r, phi_l)`` per leg, four trailing ``-1`` columns, and no sample rate at
all.  The ROS2 controller the Walk and trot CSVs are made for wants something
different, and the difference is not cosmetic -- feeding it the phi file plays
the gait 12.4x too fast.  Measured against
``examples/gait/generate_hardware_csv.py`` and
``legwheel/planners/obstacle_walk/export.py``:

===================  ==========================  =========================
                     Walk / trot                 phi export
===================  ==========================  =========================
columns 0-7          ``(theta, beta)`` per leg   ``(phi_r, phi_l)`` per leg
columns 8-11         gamma, ``0.0``              flags, ``-1.0``
rate                 1 kHz, PCHIP resampled      the planner's own, unstated
prep                 5000 rows, cosine, from     5000 rows, linear, from the
                     the folded home pose        motor zero, plus 2000 held
===================  ==========================  =========================

So this module exists to make the two interchangeable: drop a Hybrid CSV in
where a Walk CSV went and the controller does not have to know.

**It reuses the Walk pipeline's own constants and helpers** rather than
restating them -- ``CONTROLLER_DT_S``, ``build_prep_rows``,
``resample_for_csv_controller`` -- so the two exports cannot drift apart
silently.  If the controller contract changes, it changes in one place.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from scipy.interpolate import PchipInterpolator

from legwheel.planners.obstacle_walk.export import (
    CONTROLLER_DT_S,
    CONTROLLER_TRANSFORM_ROWS,
    build_prep_rows,
    resample_for_csv_controller,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodyTrajectory2D,
)

#: ``(theta, beta)`` column pair for each leg, by the **project's** leg index
#: -- ``CorgiLegKinematics`` documents ``0 FL, 1 FR, 2 RR, 3 RL``, and
#: :class:`LegId`'s own docstring says LF *is* FL and RH *is* RR.  So the hind
#: pair is 2 = RH and 3 = LH, which reads backwards and is exactly why it gets
#: written down: ``LEG_ORDER`` is LF RF LH RH, the *reading* order, and using
#: it here swaps the two hind legs while nothing complains (Day 12 trap 19).
_LEG_INDEX: dict[str, int] = {"LF": 0, "RF": 1, "RH": 2, "LH": 3}


class NotExportable(RuntimeError):
    """Raised rather than writing something the controller cannot execute."""


@dataclass(frozen=True)
class HardwareCommand2D:
    """The Walk contract's ``(N, 12)`` block, plus what it took to build it."""

    rows: NDArray[np.float64]
    phase: NDArray[np.int8]
    prep_rows: int
    trajectory_rows: int
    controller_dt_s: float
    planner_dt_s: float
    resample_ratio: int

    @property
    def duration_s(self) -> float:
        return float(len(self.rows) * self.controller_dt_s)

    @property
    def trajectory_duration_s(self) -> float:
        return float(self.trajectory_rows * self.controller_dt_s)

    def as_dict(self) -> dict:
        return {
            "rows": int(len(self.rows)),
            "prep_rows": self.prep_rows,
            "trajectory_rows": self.trajectory_rows,
            "controller_dt_s": self.controller_dt_s,
            "planner_dt_s": self.planner_dt_s,
            "resample_ratio": self.resample_ratio,
            "duration_s": self.duration_s,
            "trajectory_duration_s": self.trajectory_duration_s,
        }


def planner_rows_2d(trajectory: WholeBodyTrajectory2D,
                    *, reverse: bool = False) -> tuple[
        NDArray[np.float64], NDArray[np.int8], float]:
    """``(N, 12)`` hardware-ordered rows straight off the trajectory samples.

    Gamma is written as zero because Day 12 fixes it there -- and written
    rather than left out, because the controller reads twelve columns and a
    short row is a silent misalignment, not an error.

    ``reverse`` negates ``beta`` on every leg.  The 2D planner has one rolling
    direction -- ``rotation_rad`` is documented as "forward rotation, positive;
    beta decreases while rolling forward" -- so *forward* is a property of the
    sign convention, not of the plan, and the whole of it lives in this one
    negation.  ``theta`` is a leg *length* and is not touched; the phase column
    is a per-leg airborne flag and does not depend on direction either.

    Which sign is forward **on the robot** is a fact about the hardware that
    the 2D model does not contain, so it is not asserted here: measured against
    ``input_csv/demo_walk_real.csv``, that pipeline's stance beta ramps
    *positive* on FL/RL and *negative* on FR/RR, while this export ramps every
    leg the same way.  Reversing was confirmed by driving the robot.
    """

    samples = trajectory.samples
    if len(samples) < 2:
        raise NotExportable("a trajectory needs at least two samples to time.")

    rows = np.zeros((len(samples), 12), dtype=float)
    phase = np.zeros((len(samples), 4), dtype=np.int8)
    for row, sample in enumerate(samples):
        # A sample that does not carry all four legs cannot be exported.
        #
        # ``rows`` is pre-filled with zeros and only the legs present are
        # written, so a missing leg leaves ``theta = 0`` -- fully folded, and
        # 17 deg below the joint's own minimum.  It does not read as an error
        # anywhere downstream: it reads as a command, and the leg snaps to it.
        # Obstacle runs do produce such samples (three of 121 on the 40 mm
        # terrain, and the legs go missing in pairs because a pair shares its
        # timing), which is what made the legs "explode" in simulation.
        # Flat ground never triggers it, which is why it went unseen.
        #
        # Refused rather than filled in: the honest options are a real pose or
        # no file, and inventing a pose here would put a fabricated command
        # where a measurement has to be.
        if len(sample.legs) != len(_LEG_INDEX):
            present = ", ".join(sorted(l.value for l in sample.legs))
            raise NotExportable(
                f"sample {row} (t = {sample.time_s:.6f} s) carries "
                f"{len(sample.legs)} of {len(_LEG_INDEX)} legs [{present}]; "
                "exporting it would write theta = 0 for the missing ones, "
                "which the controller would execute as a fully folded leg."
            )
        for leg, leg_sample in sample.legs.items():
            index = _LEG_INDEX[leg.value]
            rows[row, 2 * index] = float(leg_sample.theta_rad)
            rows[row, 2 * index + 1] = float(
                -leg_sample.beta_rad if reverse else leg_sample.beta_rad)
            rows[row, 8 + index] = 0.0
            phase[row, index] = 1 if leg_sample.mode.value.lower() == "airborne" else 0

    times = [float(s.time_s) for s in samples]
    steps = np.diff(times)
    if not np.allclose(steps, steps[0], rtol=0.0, atol=1e-9):
        raise NotExportable(
            "the trajectory is not evenly sampled, so it has no single planner "
            f"dt to resample from (min {steps.min():.6f}, max {steps.max():.6f}).")
    return rows, phase, float(steps[0])


def _resample_to_controller(
    commands: NDArray[np.float64],
    phase: NDArray[np.int8],
    *,
    planner_dt_s: float,
) -> tuple[NDArray[np.float64], NDArray[np.int8], int]:
    """PCHIP onto the controller's 1 kHz grid, for a **non-integer** ratio.

    ``resample_for_csv_controller`` requires the planner period to be an exact
    integer multiple of 1 ms, and for the Walk planner it is -- it chooses its
    own dt.  This trajectory cannot: its sample times come from position
    scheduling, so the span is whatever hip travel divided by body speed says
    it is, and dividing that into a whole number of samples almost never lands
    on a millisecond.

    So this relaxes exactly that one precondition and nothing else -- same
    interpolant, same clamp of the last controller time onto the last planner
    time, same rule that a phase is a label carried by its interval rather than
    a quantity to interpolate.  ``test_the_two_resamplers_agree_on_an_integer_ratio``
    pins it against the original wherever both are defined.
    """

    planner_time = np.arange(len(commands), dtype=float) * planner_dt_s
    span = float(planner_time[-1])
    count = int(np.floor(span / CONTROLLER_DT_S)) + 1
    controller_time = np.arange(count, dtype=float) * CONTROLLER_DT_S
    controller_time[-1] = min(controller_time[-1], span)
    resampled = np.asarray(
        PchipInterpolator(planner_time, commands, axis=0)(controller_time),
        dtype=float)
    # Each controller instant takes the phase of the planner interval it falls
    # in; the final instant takes the last knot's.
    index = np.clip(np.searchsorted(planner_time, controller_time, side="right") - 1,
                    0, len(phase) - 1)
    return resampled, phase[index].astype(np.int8, copy=False), 1


def hardware_command_2d(
    trajectory: WholeBodyTrajectory2D,
    *,
    home_theta_deg: float | None = None,
    reverse: bool = False,
) -> HardwareCommand2D:
    """Resample to the controller's rate and put the Walk prep ramp in front.

    ``reverse`` is handed to :func:`planner_rows_2d`; see its docstring for why
    the direction is a sign convention rather than a different plan.
    """

    planner, planner_phase, planner_dt = planner_rows_2d(
        trajectory, reverse=reverse)
    ratio_float = planner_dt / CONTROLLER_DT_S
    if np.isclose(ratio_float, round(ratio_float), rtol=0.0, atol=1e-9):
        # The Walk pipeline's own resampler, whenever its precondition holds.
        resampled, phase, ratio = resample_for_csv_controller(
            planner, planner_phase, planner_dt_s=planner_dt)
    else:
        resampled, phase, ratio = _resample_to_controller(
            planner, planner_phase, planner_dt_s=planner_dt)

    kwargs = {} if home_theta_deg is None else {"home_theta_deg": home_theta_deg}
    prep = build_prep_rows(resampled[0], dt_s=CONTROLLER_DT_S, **kwargs)
    if len(prep) != CONTROLLER_TRANSFORM_ROWS:
        raise NotExportable(
            f"the controller consumes exactly {CONTROLLER_TRANSFORM_ROWS} "
            f"transform rows; built {len(prep)}.")

    rows = np.vstack((prep, resampled))
    # Prep is all four legs quasi-statically ramping to the first pose, which
    # the Walk exporter labels stance -- the same reading, for the same reason.
    full_phase = np.vstack((np.zeros((len(prep), 4), dtype=np.int8), phase))
    return HardwareCommand2D(
        rows=rows, phase=full_phase, prep_rows=len(prep),
        trajectory_rows=len(resampled), controller_dt_s=CONTROLLER_DT_S,
        planner_dt_s=planner_dt, resample_ratio=ratio)


def write_hardware_csv_2d(command: HardwareCommand2D,
                          path: str | Path) -> tuple[Path, Path]:
    """The 12-column CSV plus the row-aligned phase sidecar, as Walk writes them."""

    out = Path(path)
    if out.suffix.lower() != ".csv":
        raise ValueError("path must end with .csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out, command.rows, delimiter=",", fmt="%.6f")

    phase_path = out.with_name(out.stem + "_phase.csv")
    np.savetxt(phase_path, command.phase, delimiter=",", fmt="%.0f",
               header="FL(LF),FR(RF),RR(RH),RL(LH)", comments="")
    return out, phase_path
