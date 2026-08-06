"""Generalized spring-loaded inverted pendulum (G-SLIP) model.

Implements the planar running template of Lu & Lin 2024,
"The generalized spring-loaded inverted pendulum model for analysis of various
planar reduced-order models and for optimal robot leg design",
Bioinspir. Biomim. 19 026017.

Equation numbers in comments refer to that paper.

The model has seven intrinsic parameters (m, l1, l2, r, k_t, phi0, psi) and
specializes into SLIP, TSL, SLIP-RF and R-SLIP by parameter choice; see
`legwheel.models.gslip_special_cases`.

Generalized coordinates are q = [theta, phi]: theta is the leg angle at the
point mass, phi the torsion-spring angle. Motion alternates stance (spring
compression, rolling contact) and flight (ballistic).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

G_DEFAULT = 9.81


@dataclass
class GSlipParams:
    """The seven intrinsic G-SLIP parameters, plus gravity.

    Args:
        m: point mass (kg)
        l1: upper bar length, mass to spring joint (m)
        l2: lower bar length, spring joint to rim (m)
        r: rim radius (m); r = 0 gives point contact
        k_t: torsion spring stiffness (N m / rad)
        phi0: natural angle of the torsion spring (rad)
        psi: angle between the lower bar and the rim (rad)
        g: gravitational acceleration (m/s^2)
    """

    m: float
    l1: float
    l2: float
    r: float
    k_t: float
    phi0: float
    psi: float
    g: float = G_DEFAULT

    # Derived constants (eqs 1, 2, 4). Intrinsic parameters are constant, so
    # these are too.
    l3: float = field(init=False)
    eta: float = field(init=False)
    l0: float = field(init=False)

    def __post_init__(self) -> None:
        # eq 1
        self.l3 = float(np.sqrt(self.l2**2 + self.r**2 - 2 * self.l2 * self.r * np.cos(self.psi)))
        # eq 2
        if self.l3 == 0.0 or self.l2 == 0.0:
            self.eta = 0.0
        else:
            self.eta = float(
                np.arccos(
                    np.clip(
                        (-(self.r**2) + self.l2**2 + self.l3**2) / (2 * self.l2 * self.l3),
                        -1.0,
                        1.0,
                    )
                )
            )
        # eq 4: distance from the point mass to the rim center at the spring's
        # natural configuration
        self.l0 = float(
            np.sqrt(
                self.l1**2
                + self.l3**2
                - 2 * self.l1 * self.l3 * np.cos(self.phi0 - self.eta)
            )
        )

    def theta0(self, beta: float) -> float:
        """Leg angle at touchdown, for landing angle beta (eq 3)."""
        cos_arg = (self.l1**2 + self.l0**2 - self.l3**2) / (2 * self.l1 * self.l0)
        return float(np.pi - beta - np.arccos(np.clip(cos_arg, -1.0, 1.0)))

    def touchdown_height(self, beta: float) -> float:
        """Mass height at which touchdown occurs (eq 8)."""
        return self.r + self.l0 * np.sin(beta)


# --------------------------------------------------------------------------
# Fitting a real leg to G-SLIP parameters
# --------------------------------------------------------------------------


def phi0_for_leg_length(l1: float, l2: float, r: float, psi: float, l0: float) -> float:
    """Natural spring angle giving a target rest leg length l0, inverting eq 4.

    l0 is the mass-to-rim-center distance, so for the Corgi it is the
    hip-to-arc-center distance measured in Phase 0. The remaining intrinsic
    parameters fix l3 and eta, and eq 4 then determines phi0.
    """
    probe = GSlipParams(m=1.0, l1=l1, l2=l2, r=r, k_t=1.0, phi0=0.0, psi=psi)
    cos_arg = (l1**2 + probe.l3**2 - l0**2) / (2 * l1 * probe.l3)
    if not -1.0 <= cos_arg <= 1.0:
        raise ValueError(
            f"leg length l0={l0} unreachable with l1={l1}, l3={probe.l3:.4f}; "
            f"needs |cos| <= 1 but got {cos_arg:.4f}"
        )
    return float(np.arccos(cos_arg) + probe.eta)


def linear_stiffness(p: GSlipParams) -> float:
    """Equivalent linear stiffness along the leg, at the natural spring angle.

    Differentiating eq 4 gives  dl0/dphi = l1*l3*sin(phi - eta)/l0, and virtual
    work maps the torsion spring onto a linear one:

        k_linear = k_t / (dl0/dphi)^2

    This is the number to compare against the Phase 0 estimate
    k_virtual = k_rel*m*g/l0, and against the impedance gains the robot
    actually commands.
    """
    dl0_dphi = p.l1 * p.l3 * np.sin(p.phi0 - p.eta) / p.l0
    return float(p.k_t / dl0_dphi**2)


def k_t_for_linear_stiffness(p: GSlipParams, k_linear: float) -> float:
    """Torsion stiffness k_t that realizes a given linear leg stiffness."""
    dl0_dphi = p.l1 * p.l3 * np.sin(p.phi0 - p.eta) / p.l0
    return float(k_linear * dl0_dphi**2)


# --------------------------------------------------------------------------
# Stance kinematics
# --------------------------------------------------------------------------


def stance_height(p: GSlipParams, theta: float, phi: float) -> float:
    """Point-mass height (eq 5, z component). Independent of theta0."""
    return float(p.r + p.l3 * np.sin(phi - theta - p.eta) + p.l1 * np.sin(theta))


def stance_position_raw(
    p: GSlipParams, theta: float, phi: float, theta0: float
) -> tuple[float, float]:
    """Point-mass position exactly as eq 5 writes it.

    Note eq 5's x carries an arbitrary additive constant: x never enters the
    potential, and only xdot enters the kinetic energy, so the dynamics are
    unaffected by the offset. Use `stance_position` when the horizontal origin
    must actually be the contact point (stride length, plotting).
    """
    s = phi - theta - p.eta
    x = p.r * (phi - p.phi0 - theta + theta0) + p.l3 * np.cos(s) + p.l1 * np.cos(theta)
    return float(x), stance_height(p, theta, phi)


def stance_position(
    p: GSlipParams, theta: float, phi: float, beta: float
) -> tuple[float, float]:
    """Point-mass position with the horizontal origin at the contact point.

    At touchdown the rim center sits directly above the contact point at
    height r, and the mass is a distance l0 from it at angle beta above
    horizontal, trailing the contact point. Anchoring eq 5 to that condition
    removes its arbitrary offset.
    """
    theta0 = p.theta0(beta)
    x_raw, z = stance_position_raw(p, theta, phi, theta0)
    x_td, _ = stance_position_raw(p, theta0, p.phi0, theta0)
    return float(x_raw - x_td - p.l0 * np.cos(beta)), z


def stance_jacobian(p: GSlipParams, theta: float, phi: float) -> np.ndarray:
    """d(x, z)/d(theta, phi), a 2x2 matrix. Rows are x then z."""
    s = phi - theta - p.eta
    sin_s, cos_s = np.sin(s), np.cos(s)
    return np.array(
        [
            [-p.r + p.l3 * sin_s - p.l1 * np.sin(theta), p.r - p.l3 * sin_s],
            [-p.l3 * cos_s + p.l1 * np.cos(theta), p.l3 * cos_s],
        ]
    )


def stance_jacobian_det(p: GSlipParams, theta: float, phi: float) -> float:
    """Determinant of the stance Jacobian, in closed form.

    Expanding the 2x2 determinant collapses to

        det J = l1 * (l3 * sin(phi - 2*theta - eta) - r * cos(theta))

    which vanishes at the linkage's dead point. For point contact (r = 0) that
    is simply theta = (phi - eta)/2; rolling contact shifts it. There the mass
    matrix m*(J^T J) is singular and the model is not integrable, so this
    doubles as the singularity guard.
    """
    return float(
        p.l1 * (p.l3 * np.sin(phi - 2 * theta - p.eta) - p.r * np.cos(theta))
    )


def stance_hessians(p: GSlipParams, theta: float, phi: float) -> tuple[np.ndarray, np.ndarray]:
    """Second derivatives of x and z w.r.t. (theta, phi). Each 2x2, symmetric."""
    s = phi - theta - p.eta
    sin_s, cos_s = np.sin(s), np.cos(s)
    h_x = np.array(
        [
            [-p.l3 * cos_s - p.l1 * np.cos(theta), p.l3 * cos_s],
            [p.l3 * cos_s, -p.l3 * cos_s],
        ]
    )
    h_z = np.array(
        [
            [-p.l3 * sin_s - p.l1 * np.sin(theta), p.l3 * sin_s],
            [p.l3 * sin_s, -p.l3 * sin_s],
        ]
    )
    return h_x, h_z


def stance_accel(
    p: GSlipParams, theta: float, phi: float, dtheta: float, dphi: float, tau: np.ndarray | None = None
) -> np.ndarray:
    """Generalized acceleration [ddtheta, ddphi] during stance (eq 7).

    For a point mass the Christoffel terms collapse: Lagrange's equations
    reduce exactly to  m*(J_x^T*xddot + J_z^T*zddot) + dV/dq = tau,  so only
    the Hessians of x and z are needed, not symbolic Coriolis matrices.

    Args:
        tau: optional generalized torque [tau_theta, tau_phi]. None means the
            conservative model (eq 7 with Pi = Delta = 0). The clocked-torque
            model of eq 11 passes tau_theta from the PD law and tau_phi = 0.
    """
    jac = stance_jacobian(p, theta, phi)
    h_x, h_z = stance_hessians(p, theta, phi)
    qd = np.array([dtheta, dphi])

    j_x, j_z = jac[0], jac[1]

    # Mass matrix M = m * (J_x^T J_x + J_z^T J_z)
    mass = p.m * (np.outer(j_x, j_x) + np.outer(j_z, j_z))

    # Velocity-product term: m * (J_x^T * qd^T H_x qd + J_z^T * qd^T H_z qd)
    vel = p.m * (j_x * (qd @ h_x @ qd) + j_z * (qd @ h_z @ qd))

    # dV/dq with V = 0.5*k_t*(phi0 - phi)^2 + m*g*z
    dv = np.array(
        [
            p.m * p.g * jac[1, 0],
            -p.k_t * (p.phi0 - phi) + p.m * p.g * jac[1, 1],
        ]
    )

    rhs = -vel - dv
    if tau is not None:
        rhs = rhs + np.asarray(tau, dtype=float)
    return np.linalg.solve(mass, rhs)


def touchdown_rates(
    p: GSlipParams, v: float, alpha: float, beta: float
) -> tuple[float, float]:
    """Generalized velocities at touchdown, from touchdown states (v, alpha).

    alpha is the angle between the horizontal and the velocity vector, with the
    mass moving forward and downward, so (vx, vz) = (v*cos(alpha), -v*sin(alpha)).
    """
    theta0 = p.theta0(beta)
    jac = stance_jacobian(p, theta0, p.phi0)
    cart = np.array([v * np.cos(alpha), -v * np.sin(alpha)])
    return tuple(np.linalg.solve(jac, cart))


class GSlipFailure(Exception):
    """The model failed to complete a stride (no liftoff, no flight, fell)."""


def simulate_stance(
    p: GSlipParams,
    v: float,
    alpha: float,
    beta: float,
    max_time: float = 5.0,
    rtol: float = 1e-10,
    atol: float = 1e-12,
    dense: bool = False,
):
    """Integrate one stance phase, from touchdown to liftoff (eq 9: phi = phi0).

    Returns the scipy solution object; `sol.t_events[0][0]` is the liftoff time.
    """
    theta0 = p.theta0(beta)
    dtheta0, dphi0 = touchdown_rates(p, v, alpha, beta)

    if dphi0 == 0.0:
        raise GSlipFailure("spring has zero initial rate; stance is degenerate")

    def rhs(_t, y):
        theta, phi, dtheta, dphi = y
        ddtheta, ddphi = stance_accel(p, theta, phi, dtheta, dphi)
        return [dtheta, dphi, ddtheta, ddphi]

    # Liftoff is the *return* of phi to phi0. The spring leaves phi0 at t=0, so
    # arm the event in the direction of the return crossing only; that way the
    # t=0 root (where phi - phi0 is exactly zero but moving away) cannot fire.
    def liftoff(_t, y):
        return y[1] - p.phi0

    liftoff.terminal = True
    liftoff.direction = 1.0 if dphi0 < 0 else -1.0

    def fell(_t, y):
        return stance_height(p, y[0], y[1])

    fell.terminal = True
    fell.direction = -1.0

    # The two-bar linkage has a dead point at theta = (phi - eta)/2 where the
    # mass matrix goes singular. Stop there with a clear diagnosis instead of
    # letting the integrator grind its step size to zero.
    def singular(_t, y):
        return stance_jacobian_det(p, y[0], y[1])

    singular.terminal = True
    singular.direction = 0.0

    sol = solve_ivp(
        rhs,
        (0.0, max_time),
        [theta0, p.phi0, dtheta0, dphi0],
        events=[liftoff, fell, singular],
        rtol=rtol,
        atol=atol,
        dense_output=dense,
        max_step=max_time / 200.0,
    )
    if len(sol.t_events[2]) > 0:
        raise GSlipFailure(
            "linkage reached its kinematic dead point (det J = 0) during stance"
        )
    if len(sol.t_events[1]) > 0:
        raise GSlipFailure("mass reached the ground during stance")
    if len(sol.t_events[0]) == 0:
        raise GSlipFailure(f"no liftoff within {max_time} s")
    return sol


def flight_to_touchdown(
    p: GSlipParams, z_liftoff: float, vx: float, vz: float, beta: float
) -> tuple[float, float, float]:
    """Ballistic flight from liftoff to the next touchdown.

    Returns (flight_time, v_touchdown, alpha_touchdown).
    """
    z_td = p.touchdown_height(beta)
    disc = vz**2 + 2 * p.g * (z_liftoff - z_td)
    if disc < 0:
        raise GSlipFailure("apex never reaches the touchdown height")
    # Descending root of  z_liftoff + vz*t - 0.5*g*t^2 = z_td
    t_flight = (vz + np.sqrt(disc)) / p.g
    if t_flight <= 0:
        raise GSlipFailure("model does not leave the ground at liftoff")

    vz_td = vz - p.g * t_flight
    v = float(np.hypot(vx, vz_td))
    alpha = float(np.arctan2(-vz_td, vx))
    return float(t_flight), v, alpha


def stride(p: GSlipParams, v: float, alpha: float, beta: float) -> dict:
    """One full stride: stance then flight. Returns the next touchdown states.

    This is the Poincare map used for fixed-point analysis (section 2.2). The
    Poincare section is the touchdown event; for the conservative model with a
    fixed landing angle beta, v is invariant and the map is one-dimensional
    in alpha.
    """
    sol = simulate_stance(p, v, alpha, beta)
    t_lo = sol.t_events[0][0]
    theta_lo, phi_lo, dtheta_lo, dphi_lo = sol.y_events[0][0]

    jac = stance_jacobian(p, theta_lo, phi_lo)
    vx, vz = jac @ np.array([dtheta_lo, dphi_lo])
    x_lo, z_lo = stance_position(p, theta_lo, phi_lo, beta)

    t_flight, v_next, alpha_next = flight_to_touchdown(p, z_lo, vx, vz, beta)

    return {
        "v": v_next,
        "alpha": alpha_next,
        "stance_time": float(t_lo),
        "flight_time": t_flight,
        "period": float(t_lo) + t_flight,
        "x_liftoff": x_lo,
        "z_liftoff": z_lo,
        "vx_liftoff": float(vx),
        "vz_liftoff": float(vz),
        "stride_length": x_lo + vx * t_flight,
        "solution": sol,
    }


def stance_energy(p: GSlipParams, theta: float, phi: float, dtheta: float, dphi: float) -> dict:
    """Kinetic, gravitational and elastic energy during stance."""
    jac = stance_jacobian(p, theta, phi)
    vx, vz = jac @ np.array([dtheta, dphi])
    return {
        "kinetic": 0.5 * p.m * (vx**2 + vz**2),
        "gravity": p.m * p.g * stance_height(p, theta, phi),
        "elastic": 0.5 * p.k_t * (p.phi0 - phi) ** 2,
    }
