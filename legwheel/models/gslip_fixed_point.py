"""Fixed points and basins of attraction of the G-SLIP model.

Section 2.2 of Lu & Lin 2024. The Poincare section is the touchdown event.
Touchdown is described by three states (v, alpha, beta); beta is held equal
across touchdowns because only period-one gaits are considered, and the
conservative model returns to the same height with the same speed, so v is
invariant too. The map is therefore one-dimensional in alpha:

    alpha_{n+1} = P(alpha_n)

A fixed point satisfies P(alpha*) = alpha* (eq 10), and is stable when
|dP/dalpha| < 1.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from legwheel.models.gslip import GSlipFailure, GSlipParams, stride


@dataclass
class FixedPoint:
    """A period-one running gait of the G-SLIP model."""

    v: float
    alpha: float
    beta: float
    slope: float
    stance_time: float
    flight_time: float
    stride_length: float

    @property
    def stable(self) -> bool:
        return abs(self.slope) < 1.0

    @property
    def period(self) -> float:
        return self.stance_time + self.flight_time

    @property
    def duty_factor(self) -> float:
        return self.stance_time / self.period

    @property
    def mean_speed(self) -> float:
        return self.stride_length / self.period


def poincare(p, v: float, alpha: float, beta: float, stride_fn=stride) -> float:
    """One application of the touchdown-to-touchdown map. Raises on failure.

    `stride_fn` selects the model: `gslip.stride` for the general model,
    `slip_rf.stride` for the Corgi's SLIP-RF reduction.
    """
    return stride_fn(p, v, alpha, beta)["alpha"]


def _map_residual(p, v: float, beta: float, stride_fn=stride):
    def residual(alpha: float) -> float:
        return poincare(p, v, alpha, beta, stride_fn) - alpha

    return residual


def find_fixed_points(
    p,
    v: float,
    beta: float,
    alpha_range: tuple[float, float] = (0.0, np.pi / 2),
    n_samples: int = 60,
    stride_fn=stride,
) -> list[FixedPoint]:
    """All period-one fixed points at a given speed and landing angle.

    Samples the map on a grid (skipping touchdown states where the model
    fails), brackets sign changes of P(alpha) - alpha, and refines with Brent.
    """
    residual = _map_residual(p, v, beta, stride_fn)

    alphas = np.linspace(alpha_range[0], alpha_range[1], n_samples)
    sampled: list[tuple[float, float]] = []
    for a in alphas:
        try:
            sampled.append((float(a), residual(float(a))))
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            sampled.append((float(a), np.nan))

    found: list[FixedPoint] = []
    for (a0, r0), (a1, r1) in zip(sampled, sampled[1:]):
        if np.isnan(r0) or np.isnan(r1) or np.sign(r0) == np.sign(r1):
            continue
        try:
            a_star = brentq(residual, a0, a1, xtol=1e-12, rtol=1e-12)
        except (GSlipFailure, ValueError):
            continue
        found.append(_describe(p, v, float(a_star), beta, stride_fn))
    return found


def _describe(p, v: float, alpha: float, beta: float, stride_fn=stride) -> FixedPoint:
    """Build a FixedPoint, taking the map slope by central difference."""
    result = stride_fn(p, v, alpha, beta)
    h = 1e-6
    slope = (
        poincare(p, v, alpha + h, beta, stride_fn)
        - poincare(p, v, alpha - h, beta, stride_fn)
    ) / (2 * h)
    return FixedPoint(
        v=v,
        alpha=alpha,
        beta=beta,
        slope=float(slope),
        stance_time=result["stance_time"],
        flight_time=result["flight_time"],
        stride_length=result["stride_length"],
    )


def sweep_landing_angle(
    p: GSlipParams,
    v: float,
    beta_values: np.ndarray,
    alpha_range: tuple[float, float] = (0.0, np.pi / 2),
) -> list[FixedPoint]:
    """Fixed points across landing angles at one speed."""
    out: list[FixedPoint] = []
    for beta in beta_values:
        out.extend(find_fixed_points(p, v, float(beta), alpha_range))
    return out


def steps_to_converge(
    p: GSlipParams,
    fp: FixedPoint,
    v: float,
    alpha: float,
    max_steps: int = 40,
    tol: float = 0.01,
    stride_fn=stride,
) -> int | None:
    """How many map applications until a perturbed touchdown state re-reaches fp.

    Returns the number of steps after which the eq-13 relative error first falls
    below `tol` (0 if the start state is already inside it), or None if it never
    does within `max_steps` -- which covers divergence and model failure alike.

    This is the quantity the step-to-fall literature reports and that
    `converges_to` used to throw away. For a 1-D map the count is predicted by
    log(tol / err_0) / log|dP/dalpha|; `tests/test_gslip_fixed_point.py` checks
    the two agree, because a silent disagreement would mean either the map or
    this error metric is wrong.

    Only the states this conservative model can vary are compared: the relative
    phase rho of eq 13 is a clocked-torque quantity and is zero here.

    NOTE ON `v`: the map holds v EXACTLY invariant (measured: 1.7e-10 over one
    stride), so the v term of the error is a constant floor set by the initial
    perturbation. A state started with |dv|/v >= tol can therefore never
    converge, whatever alpha does. Perturb alpha to probe this fixed point;
    changing v selects a different orbit family, not a perturbation of this one.
    """
    for n in range(max_steps):
        err = np.sqrt(((v - fp.v) / fp.v) ** 2 + ((alpha - fp.alpha) / fp.alpha) ** 2)
        if err < tol:
            return n
        try:
            alpha = poincare(p, v, alpha, fp.beta, stride_fn)
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            return None
    return None


def converges_to(
    p: GSlipParams,
    fp: FixedPoint,
    v: float,
    alpha: float,
    max_steps: int = 40,
    tol: float = 0.01,
    stride_fn=stride,
) -> bool:
    """Step-to-fall test: does this touchdown state converge to the fixed point?

    Thin wrapper over `steps_to_converge`, which is the same iteration with the
    step count kept. Prefer that one unless a bare boolean is genuinely all you
    need.
    """
    return steps_to_converge(p, fp, v, alpha, max_steps, tol, stride_fn) is not None


def basin_of_attraction(
    p: GSlipParams,
    fp: FixedPoint,
    alpha_values: np.ndarray,
    v_values: np.ndarray | None = None,
    **kwargs,
) -> np.ndarray:
    """Boolean grid over (v, alpha) of touchdown states inside the basin.

    WARNING: the v axis is very nearly inert, and not for a numerical reason.
    The map conserves v, so `steps_to_converge`'s error metric carries a
    permanent floor of |v - fp.v| / fp.v; every row with a relative v offset at
    or above `tol` comes back all-False regardless of alpha. Pass v_values only
    if you want that mask made explicit. The meaningful basin of a period-one
    G-SLIP fixed point in this reduction is one-dimensional -- see
    `basin_in_alpha`.
    """
    v_values = np.array([fp.v]) if v_values is None else v_values
    grid = np.zeros((len(v_values), len(alpha_values)), dtype=bool)
    for i, v in enumerate(v_values):
        for j, alpha in enumerate(alpha_values):
            grid[i, j] = converges_to(p, fp, float(v), float(alpha), **kwargs)
    return grid


def basin_in_alpha(
    p: GSlipParams,
    fp: FixedPoint,
    alpha_values: np.ndarray,
    **kwargs,
) -> dict:
    """The 1-D basin at the fixed point's own speed, with step counts.

    Returns `alpha` (the probed values), `steps` (per-state count, NaN where the
    state never converged) and `converged` (boolean). Reporting the counts is
    the point: "inside the basin" and "recovers in six strides" are different
    claims and only the second is comparable across operating points.
    """
    steps = np.full(len(alpha_values), np.nan)
    for j, alpha in enumerate(alpha_values):
        n = steps_to_converge(p, fp, fp.v, float(alpha), **kwargs)
        if n is not None:
            steps[j] = n
    return {
        "alpha": np.asarray(alpha_values, dtype=float),
        "steps": steps,
        "converged": ~np.isnan(steps),
    }
