"""Parametrized fixed-point leg trajectory: the running template.

Section 2.2 of Lu & Lin 2024. The fixed-point motion is turned into a
reference the robot can actually track: a fifth-order polynomial across
stance, a trapezoidal sweep across flight, concatenated into one stride.

The conservative model's leg can be repositioned instantly in flight; a real
leg cannot, so the flight segment is given a finite-acceleration profile. The
clocked-torque controller (eq 11) then tracks this reference in both phases.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.slip_rf import SlipRfParams


def quintic(duration: float, start: tuple[float, float, float],
            end: tuple[float, float, float]) -> np.ndarray:
    """Quintic coefficients matching position, velocity and acceleration at both ends.

    Six boundary conditions exactly determine a fifth-order polynomial, so this
    interpolates rather than least-squares fits. Returns coefficients in
    ascending powers of t.
    """
    p0, v0, a0 = start
    p1, v1, a1 = end
    t = duration
    c = np.array([p0, v0, a0 / 2.0])
    # Solve the remaining three coefficients from the terminal conditions.
    m = np.array([
        [t**3, t**4, t**5],
        [3 * t**2, 4 * t**3, 5 * t**4],
        [6 * t, 12 * t**2, 20 * t**3],
    ])
    rhs = np.array([
        p1 - (c[0] + c[1] * t + c[2] * t**2),
        v1 - (c[1] + 2 * c[2] * t),
        a1 - 2 * c[2],
    ])
    return np.concatenate([c, np.linalg.solve(m, rhs)])


def polyval(coeffs: np.ndarray, t: float | np.ndarray, order: int = 0):
    """Evaluate a polynomial (ascending powers), or its `order`-th derivative."""
    c = np.asarray(coeffs, dtype=float)
    for _ in range(order):
        c = c[1:] * np.arange(1, len(c))
    return sum(ci * np.asarray(t, dtype=float) ** i for i, ci in enumerate(c))


def trapezoid(duration: float, start: float, end: float, ramp_fraction: float = 0.25):
    """Trapezoidal-velocity sweep from `start` to `end` over `duration`.

    Accelerates for `ramp_fraction` of the duration, coasts, then decelerates
    symmetrically. Returns (position, velocity) callables of time.
    """
    if not 0 < ramp_fraction <= 0.5:
        raise ValueError("ramp_fraction must lie in (0, 0.5]")
    span = end - start
    t_r = ramp_fraction * duration
    # Area of the trapezoid must equal the span: v_max*(T - t_r) = span
    v_max = span / (duration - t_r)
    accel = v_max / t_r

    def position(t):
        t = np.clip(t, 0.0, duration)
        return np.where(
            t < t_r,
            start + 0.5 * accel * t**2,
            np.where(
                t <= duration - t_r,
                start + 0.5 * v_max * t_r + v_max * (t - t_r),
                end - 0.5 * accel * (duration - t) ** 2,
            ),
        )

    def velocity(t):
        t = np.clip(t, 0.0, duration)
        return np.where(
            t < t_r, accel * t,
            np.where(t <= duration - t_r, v_max, accel * (duration - t)),
        )

    return position, velocity


@dataclass
class StrideTemplate:
    """One stride of the fixed-point motion, as a time-parametrized reference.

    Attributes:
        stance_time, flight_time: phase durations (s)
        angle_coeffs: quintic for the leg angle phi(t) through stance
        length_coeffs: quintic for the leg length l(t) through stance
        beta: landing angle the template was built for
    """

    params: SlipRfParams
    stance_time: float
    flight_time: float
    angle_coeffs: np.ndarray
    length_coeffs: np.ndarray
    beta: float
    v: float
    alpha: float

    @property
    def period(self) -> float:
        return self.stance_time + self.flight_time

    @property
    def duty_factor(self) -> float:
        return self.stance_time / self.period

    def _flight(self):
        phi_lo = float(polyval(self.angle_coeffs, self.stance_time))
        dphi_lo = float(polyval(self.angle_coeffs, self.stance_time, order=1))
        phi_td = self.params.phi_touchdown(self.beta)
        return trapezoid(self.flight_time, phi_lo, phi_td), phi_lo, dphi_lo

    def leg_angle(self, t: float) -> float:
        """Reference leg angle at time t within the stride (s)."""
        t = float(t) % self.period
        if t <= self.stance_time:
            return float(polyval(self.angle_coeffs, t))
        (pos, _), _, _ = self._flight()
        return float(pos(t - self.stance_time))

    def leg_angle_rate(self, t: float) -> float:
        t = float(t) % self.period
        if t <= self.stance_time:
            return float(polyval(self.angle_coeffs, t, order=1))
        (_, vel), _, _ = self._flight()
        return float(vel(t - self.stance_time))

    def leg_length(self, t: float) -> float:
        """Reference leg length at time t; constant at rest length through flight."""
        t = float(t) % self.period
        if t <= self.stance_time:
            return float(polyval(self.length_coeffs, t))
        return self.params.l0

    def sample(self, n: int = 200) -> dict:
        """Uniformly sampled stride, for plotting or CSV export."""
        t = np.linspace(0.0, self.period, n)
        return {
            "t": t,
            "leg_angle": np.array([self.leg_angle(ti) for ti in t]),
            "leg_angle_rate": np.array([self.leg_angle_rate(ti) for ti in t]),
            "leg_length": np.array([self.leg_length(ti) for ti in t]),
            "in_stance": t <= self.stance_time,
        }


def build_template(
    p: SlipRfParams, v: float, alpha: float, beta: float
) -> StrideTemplate:
    """Build the stride template from a fixed point's stance trajectory."""
    res = slip_rf.stride(p, v, alpha, beta)
    sol = res["solution"]
    t_lo = res["stance_time"]

    length_td, phi_td, dl_td, dphi_td = sol.y[:, 0]
    length_lo, phi_lo, dl_lo, dphi_lo = sol.y_events[0][0]

    ddl_td, ddphi_td = slip_rf.accel(p, length_td, phi_td, dl_td, dphi_td)
    ddl_lo, ddphi_lo = slip_rf.accel(p, length_lo, phi_lo, dl_lo, dphi_lo)

    return StrideTemplate(
        params=p,
        stance_time=t_lo,
        flight_time=res["flight_time"],
        angle_coeffs=quintic(
            t_lo, (phi_td, dphi_td, ddphi_td), (phi_lo, dphi_lo, ddphi_lo)
        ),
        length_coeffs=quintic(
            t_lo, (length_td, dl_td, ddl_td), (length_lo, dl_lo, ddl_lo)
        ),
        beta=beta,
        v=v,
        alpha=alpha,
    )


def tracking_error(template: StrideTemplate, n: int = 200) -> dict:
    """How well the quintic reproduces the true stance trajectory.

    The quintic matches the endpoints exactly by construction; this reports
    the interior deviation, which is what the controller will actually see.
    """
    res = slip_rf.stride(template.params, template.v, template.alpha, template.beta)
    sol = res["solution"]
    t = np.linspace(0.0, template.stance_time, n)
    true = sol.sol(t)
    approx_angle = polyval(template.angle_coeffs, t)
    approx_length = polyval(template.length_coeffs, t)
    return {
        "max_angle_error": float(np.max(np.abs(true[1] - approx_angle))),
        "max_length_error": float(np.max(np.abs(true[0] - approx_length))),
        "rms_angle_error": float(np.sqrt(np.mean((true[1] - approx_angle) ** 2))),
        "rms_length_error": float(np.sqrt(np.mean((true[0] - approx_length) ** 2))),
    }
