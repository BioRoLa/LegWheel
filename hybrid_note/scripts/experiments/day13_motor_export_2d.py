"""Day 13: turn an assembled trajectory into motor commands.

The project already has the transform and the writer
(``legwheel/utils/utils.py``)::

    phi_r =  theta + beta - theta_0
    phi_l = -theta + beta + theta_0

so this module is only the adapter: it takes a
:class:`WholeBodyTrajectory2D` and produces the ``4 x n`` ``theta`` and
``beta`` arrays ``create_command_csv_phi`` expects, **in the project's leg
index order**.

**It refuses more than it does.**  Two refusals, both learned the hard way:

``endpoint interpolation``   A trajectory assembled from segment endpoints has
                             a ``RECOVERY_SWING`` whose two ends share a theta,
                             so the retraction to the compact posture is simply
                             not in it -- measured: theta sweeps 0.000 deg
                             instead of 55.486.  Commanding that would hold the
                             leg extended straight through the swing.  Export
                             requires ``from_generator_frames``.
``unvalidated``              Every failing Step 9 check has to be named by the
                             caller.  Exporting a trajectory whose support
                             margin is zero is a decision, and it should be
                             made in the caller's source where a reader can see
                             it, not by omission here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodyTrajectory2D,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
    CheckId,
    ValidationReport2D,
    motor_rates_rad_s,
)


class NotExportable(RuntimeError):
    """Raised rather than writing something that cannot be executed."""


@dataclass(frozen=True)
class MotorCommand2D:
    """``theta`` and ``beta`` as ``create_command_csv_phi`` wants them."""

    #: ``(4, n)``, rows in the **project's** leg index order (0 FL, 1 FR,
    #: 2 RR, 3 RL) -- not ``LEG_ORDER``, which is the reading order.  Getting
    #: this wrong swaps the two hind legs and nothing complains (trap 19).
    theta_rad: np.ndarray
    beta_rad: np.ndarray
    #: Uniform sample spacing of the source trajectory, in seconds.  The CSV
    #: has no time column, so playback rate has to come from here.
    dt_s: float
    accepted_failures: tuple[str, ...]

    @property
    def sample_count(self) -> int:
        return int(self.theta_rad.shape[1])

    @property
    def duration_s(self) -> float:
        return float((self.sample_count - 1) * self.dt_s)

    def peak_motor_rate_rad_s(self) -> float:
        """The worst motor rate the command actually asks for."""

        worst = 0.0
        for row in range(4):
            d_theta = np.diff(self.theta_rad[row]) / self.dt_s
            d_beta = np.diff(self.beta_rad[row]) / self.dt_s
            for theta_rate, beta_rate in zip(d_theta, d_beta):
                worst = max(worst,
                            *(abs(r) for r in motor_rates_rad_s(theta_rate,
                                                                beta_rate)))
        return float(worst)

    def as_dict(self) -> dict:
        peak = self.peak_motor_rate_rad_s()
        return {
            "samples": self.sample_count,
            "dt_s": self.dt_s,
            "duration_s": self.duration_s,
            "playback_hz": 1.0 / self.dt_s if self.dt_s > 0 else None,
            "peak_motor_rate_deg_s": float(np.rad2deg(peak)),
            "motor_limit_deg_s": float(np.rad2deg(MOTOR_MAX_RATE_RAD_S)),
            "motor_utilisation": float(peak / MOTOR_MAX_RATE_RAD_S),
            "accepted_failures": ",".join(self.accepted_failures),
        }


def motor_command_2d(
    trajectory: WholeBodyTrajectory2D,
    report: ValidationReport2D,
    *,
    accept_failures: tuple[CheckId, ...] = (),
) -> MotorCommand2D:
    """The commands, or a refusal naming what stands in the way."""

    if not trajectory.from_generator_frames:
        raise NotExportable(
            "this trajectory was assembled by interpolating segment endpoints, "
            "so a RECOVERY_SWING's retraction is not in it -- rebuild with "
            "assemble_whole_body_2d(..., use_generator_frames=True)."
        )

    outstanding = [c for c in report.failed_checks() if c not in accept_failures]
    if outstanding:
        raise NotExportable(
            "refusing to export a trajectory that fails "
            + ", ".join(c.value for c in outstanding)
            + ".  Pass them in accept_failures to say so on purpose."
        )

    times = np.array([s.time_s for s in trajectory.samples], dtype=float)
    if len(times) < 2:
        raise NotExportable("a command needs at least two samples.")
    steps = np.diff(times)
    if float(steps.max() - steps.min()) > 1e-9:
        raise NotExportable(
            "the samples are not uniformly spaced, and the CSV has no time "
            "column -- a fixed playback rate would distort the motion."
        )

    theta = np.zeros((4, len(times)), dtype=float)
    beta = np.zeros((4, len(times)), dtype=float)
    for leg in LEG_ORDER:
        row = leg.index
        for column, sample in enumerate(trajectory.samples):
            leg_sample = sample.legs.get(leg)
            if leg_sample is None:
                raise NotExportable(
                    f"sample at t = {sample.time_s:.4f} s has no state for "
                    f"{leg.value}; every leg needs a command at every instant."
                )
            theta[row, column] = leg_sample.theta_rad
            beta[row, column] = leg_sample.beta_rad

    return MotorCommand2D(
        theta_rad=theta, beta_rad=beta, dt_s=float(steps[0]),
        accepted_failures=tuple(c.value for c in accept_failures),
    )


def write_motor_csv_2d(command: MotorCommand2D, path: Path | str) -> Path:
    """Write through the project's own writer, so the transform is theirs."""

    from legwheel.utils.utils import create_command_csv_phi

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # The writer appends ".csv" itself and takes the stem.
    create_command_csv_phi(command.theta_rad, command.beta_rad,
                           str(path.with_suffix("")), transform=True)
    return path.with_suffix(".csv")


def command_rows(command: MotorCommand2D) -> list[dict]:
    """The summary, plus per-leg peaks."""

    rows: list[dict] = [{"row_kind": "command", **command.as_dict()}]
    for leg in LEG_ORDER:
        row = leg.index
        d_theta = np.diff(command.theta_rad[row]) / command.dt_s
        d_beta = np.diff(command.beta_rad[row]) / command.dt_s
        peak = max(abs(r) for t, b in zip(d_theta, d_beta)
                   for r in motor_rates_rad_s(t, b))
        rows.append({
            "row_kind": "leg",
            "leg": leg.value,
            "leg_index": row,
            "theta_min_deg": float(np.rad2deg(command.theta_rad[row].min())),
            "theta_max_deg": float(np.rad2deg(command.theta_rad[row].max())),
            "beta_min_deg": float(np.rad2deg(command.beta_rad[row].min())),
            "beta_max_deg": float(np.rad2deg(command.beta_rad[row].max())),
            "peak_motor_rate_deg_s": float(np.rad2deg(peak)),
        })

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
