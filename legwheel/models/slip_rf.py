"""SLIP with rolling foot (SLIP-RF) -- the Corgi's exact reduced-order model.

Appendix C of Lu & Lin 2024 (eqs 31-34). Four intrinsic parameters: mass (m),
rest leg length (l0), linear spring stiffness (k) and foot radius (r).

Phase 0 (examples/gslip/identify_corgi_params.py) established that the Corgi
belongs here rather than in the general G-SLIP parametrization: its foot-arc
center sits exactly on the leg axis for every theta, so the leg is telescoping
with a rolling circular foot. That is precisely SLIP-RF.

Preferring this over the general model is also a correctness decision. The
G-SLIP stance kinematics (eq 5) represent a linear spring with two long bars,
and while their z component is exact, the implied mass-to-rim-center distance
is sqrt(l1^2 + l3^2 + 2*l1*l3*cos(2*beta - (phi0 - eta))), which varies with
the landing angle beta -- a rest length cannot. SLIP-RF has no such ambiguity:
the mass sits (l - r) from the foot center by construction.

Generalized coordinates are q = [l, phi]: l is the spring length and phi the
leg angle from vertical.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from legwheel.models.gslip import GSlipFailure

G_DEFAULT = 9.81


@dataclass
class SlipRfParams:
    """SLIP-RF intrinsic parameters.

    Args:
        m: point mass (kg)
        l0: rest length, mass to ground along the leg (m)
        k: linear spring stiffness (N/m)
        r: foot radius (m)
        g: gravitational acceleration (m/s^2)
    """

    m: float
    l0: float
    k: float
    r: float
    g: float = G_DEFAULT

    def __post_init__(self) -> None:
        if self.l0 <= self.r:
            raise ValueError(f"rest length l0={self.l0} must exceed foot radius r={self.r}")

    @property
    def hip_height(self) -> float:
        """Mass-to-foot-center distance at rest; the Corgi's hip-to-arc-center l0."""
        return self.l0 - self.r

    def touchdown_height(self, beta: float) -> float:
        """Mass height at touchdown for landing angle beta (eq 33)."""
        return self.r + (self.l0 - self.r) * np.sin(beta)

    @staticmethod
    def phi_touchdown(beta: float) -> float:
        """Leg angle from vertical at touchdown, for landing angle beta.

        Negative: the leg is protracted, so the foot lands ahead of the mass
        and phi sweeps up through zero to +(pi/2 - beta) over the stance. The
        mirrored (positive) choice puts the mass ahead of the foot at
        touchdown, where the leg extends rather than compresses.
        """
        return float(beta - np.pi / 2)


def position(p: SlipRfParams, length: float, phi: float) -> tuple[float, float]:
    """Point-mass position relative to the touchdown contact point (eq 31)."""
    return (
        float(p.r * phi + (length - p.r) * np.sin(phi)),
        float(p.r + (length - p.r) * np.cos(phi)),
    )


def jacobian(p: SlipRfParams, length: float, phi: float) -> np.ndarray:
    """d(x, z)/d(l, phi)."""
    return np.array(
        [
            [np.sin(phi), p.r + (length - p.r) * np.cos(phi)],
            [np.cos(phi), -(length - p.r) * np.sin(phi)],
        ]
    )


def jacobian_det(p: SlipRfParams, length: float, phi: float) -> float:
    """det J = -(l - r) - r*cos(phi); vanishing means a singular mass matrix."""
    return float(-(length - p.r) - p.r * np.cos(phi))


def hessians(p: SlipRfParams, length: float, phi: float) -> tuple[np.ndarray, np.ndarray]:
    """Second derivatives of x and z w.r.t. (l, phi)."""
    h_x = np.array([[0.0, np.cos(phi)], [np.cos(phi), -(length - p.r) * np.sin(phi)]])
    h_z = np.array([[0.0, -np.sin(phi)], [-np.sin(phi), -(length - p.r) * np.cos(phi)]])
    return h_x, h_z


def accel(p: SlipRfParams, length: float, phi: float, dl: float, dphi: float) -> np.ndarray:
    """Generalized acceleration during stance.

    Same point-mass reduction as the G-SLIP model: the Christoffel terms
    cancel, leaving m*(J_x^T*xddot + J_z^T*zddot) + dV/dq = 0.
    """
    jac = jacobian(p, length, phi)
    h_x, h_z = hessians(p, length, phi)
    qd = np.array([dl, dphi])
    j_x, j_z = jac[0], jac[1]

    mass = p.m * (np.outer(j_x, j_x) + np.outer(j_z, j_z))
    vel = p.m * (j_x * (qd @ h_x @ qd) + j_z * (qd @ h_z @ qd))
    dv = np.array(
        [
            p.k * (length - p.l0) + p.m * p.g * jac[1, 0],
            p.m * p.g * jac[1, 1],
        ]
    )
    return np.linalg.solve(mass, -vel - dv)


def cartesian_accel(
    p: SlipRfParams, length: float, phi: float, dl: float, dphi: float
) -> tuple[float, float]:
    """Point-mass acceleration (xddot, zddot) during stance."""
    qdd = accel(p, length, phi, dl, dphi)
    jac = jacobian(p, length, phi)
    h_x, h_z = hessians(p, length, phi)
    qd = np.array([dl, dphi])
    return (
        float(jac[0] @ qdd + qd @ h_x @ qd),
        float(jac[1] @ qdd + qd @ h_z @ qd),
    )


def ground_reaction(
    p: SlipRfParams, length: float, phi: float, dl: float, dphi: float
) -> tuple[float, float]:
    """Ground reaction force (Fx, Fz) on the mass, in newtons."""
    ax, az = cartesian_accel(p, length, phi, dl, dphi)
    return p.m * ax, p.m * (az + p.g)


def energy(p: SlipRfParams, length: float, phi: float, dl: float, dphi: float) -> dict:
    vx, vz = jacobian(p, length, phi) @ np.array([dl, dphi])
    return {
        "kinetic": 0.5 * p.m * (vx**2 + vz**2),
        "gravity": p.m * p.g * position(p, length, phi)[1],
        "elastic": 0.5 * p.k * (length - p.l0) ** 2,
    }


def touchdown_rates(p: SlipRfParams, v: float, alpha: float, beta: float) -> tuple[float, float]:
    """Generalized velocities at touchdown from touchdown states (v, alpha)."""
    jac = jacobian(p, p.l0, p.phi_touchdown(beta))
    return tuple(np.linalg.solve(jac, np.array([v * np.cos(alpha), -v * np.sin(alpha)])))


def simulate_stance(
    p: SlipRfParams,
    v: float,
    alpha: float,
    beta: float,
    max_time: float = 3.0,
    rtol: float = 1e-10,
    atol: float = 1e-12,
    dense: bool = False,
):
    """Integrate stance from touchdown to liftoff (eq 34: l = l0)."""
    phi_td = p.phi_touchdown(beta)
    dl0, dphi0 = touchdown_rates(p, v, alpha, beta)
    if dl0 >= 0:
        raise GSlipFailure("leg is not compressing at touchdown")

    def rhs(_t, y):
        return [y[2], y[3], *accel(p, y[0], y[1], y[2], y[3])]

    def liftoff(_t, y):
        return y[0] - p.l0

    liftoff.terminal = True
    liftoff.direction = 1.0  # the spring compresses, then returns to l0

    def fell(_t, y):
        return position(p, y[0], y[1])[1]

    fell.terminal = True
    fell.direction = -1.0

    def singular(_t, y):
        return jacobian_det(p, y[0], y[1])

    singular.terminal = True
    singular.direction = 0.0

    sol = solve_ivp(
        rhs,
        (0.0, max_time),
        [p.l0, phi_td, dl0, dphi0],
        events=[liftoff, fell, singular],
        rtol=rtol,
        atol=atol,
        dense_output=dense,
        max_step=max_time / 300.0,
    )
    if len(sol.t_events[2]) > 0:
        raise GSlipFailure("singular configuration during stance")
    if len(sol.t_events[1]) > 0:
        raise GSlipFailure("mass reached the ground during stance")
    if len(sol.t_events[0]) == 0:
        raise GSlipFailure(f"no liftoff within {max_time} s")
    return sol


def next_touchdown(p: SlipRfParams, v: float, alpha: float, beta: float) -> tuple[float, float]:
    """Next touchdown states (v, alpha) only -- the Poincare map, cheaply.

    Skips the dense output and the peak-force pass that `stride` does, which
    together cost about 1.6x. Fixed-point root-finding calls this thousands of
    times, so the saving is worth the separate entry point.
    """
    sol = simulate_stance(p, v, alpha, beta, rtol=1e-8, atol=1e-10)
    length, phi, dl, dphi = sol.y_events[0][0]
    vx, vz = jacobian(p, length, phi) @ np.array([dl, dphi])
    _, z_lo = position(p, length, phi)

    disc = vz**2 + 2 * p.g * (z_lo - p.touchdown_height(beta))
    if disc < 0:
        raise GSlipFailure("apex never reaches the touchdown height")
    t_flight = (vz + np.sqrt(disc)) / p.g
    if t_flight <= 0:
        raise GSlipFailure("model does not leave the ground at liftoff")
    vz_td = vz - p.g * t_flight
    return float(np.hypot(vx, vz_td)), float(np.arctan2(-vz_td, vx))


def stride(p: SlipRfParams, v: float, alpha: float, beta: float) -> dict:
    """One stride: stance then ballistic flight, returning next touchdown states."""
    sol = simulate_stance(p, v, alpha, beta, dense=True)
    t_lo = float(sol.t_events[0][0])
    length, phi, dl, dphi = sol.y_events[0][0]

    vx, vz = jacobian(p, length, phi) @ np.array([dl, dphi])
    x_lo, z_lo = position(p, length, phi)

    # position() is measured from the TOUCHDOWN CONTACT POINT, not from the
    # mass's own touchdown position, so the mass already sits at x_td != 0 when
    # stance begins. The stance displacement is therefore x_lo - x_td, and
    # reporting x_lo alone halves it at a symmetric fixed point (phi_td =
    # -phi_lo makes x_td = -x_lo exactly).
    #
    # Measured before the fix at v~1.20: phi swept -0.3185 to +0.3185 rad,
    # x_td = -0.0926 m, x_lo = +0.0926, true displacement 0.1851 -- against a
    # reported stance_length of 0.0926, short by a factor of 2.000.
    #
    # This propagated into stride_length and therefore into every "design
    # speed" derived from stride_length / period. It does NOT affect the fixed
    # point itself: find_fixed_points iterates on (v, alpha), which come from
    # the Jacobian, not from x.
    l_td, phi_td = float(sol.y[0, 0]), float(sol.y[1, 0])
    x_td, _ = position(p, l_td, phi_td)
    stance_dx = x_lo - x_td

    z_td = p.touchdown_height(beta)
    disc = vz**2 + 2 * p.g * (z_lo - z_td)
    if disc < 0:
        raise GSlipFailure("apex never reaches the touchdown height")
    t_flight = (vz + np.sqrt(disc)) / p.g
    if t_flight <= 0:
        raise GSlipFailure("model does not leave the ground at liftoff")

    vz_td = vz - p.g * t_flight

    # Peak ground reaction and peak compression over the stance.
    grf, compression = [], []
    for i in range(sol.y.shape[1]):
        li, pi_, dli, dphii = sol.y[:, i]
        grf.append(ground_reaction(p, li, pi_, dli, dphii))
        compression.append(p.l0 - li)
    fz = [f[1] for f in grf]

    return {
        "v": float(np.hypot(vx, vz_td)),
        "alpha": float(np.arctan2(-vz_td, vx)),
        "stance_time": t_lo,
        "flight_time": float(t_flight),
        "period": t_lo + float(t_flight),
        "stride_length": float(stance_dx + vx * t_flight),
        "stance_length": float(stance_dx),
        # Rolling distance along the foot arc, r*d_phi. Kept separate because
        # the arc-budget guard wants THIS, not the body displacement, and the
        # two happened to coincide numerically while the bug was present
        # (r = 0.145 is within 2% of l - r = 0.148 at the nominal stance).
        "arc_roll": float(p.r * (phi - phi_td)),
        "peak_grf_z": float(max(fz)),
        "peak_grf_mag": float(max(np.hypot(*np.array(grf).T))),
        "peak_compression": float(max(compression)),
        "min_length": float(p.l0 - max(compression)),
        "solution": sol,
    }
