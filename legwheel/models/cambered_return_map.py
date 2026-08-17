"""3D apex return map for the cambered lateral pair -- Sovukluk on Chang's bones.

Stage 2b Module 3 (this file's map machinery) and Module 4 (the clocked/PD roll
actuation and steps-to-fail evaluation that consume it).

THE MODEL BEING MAPPED

A point mass m with a roll DOF (J_roll) carrying a LATERAL PAIR of massless
spring legs -- the pronk's four legs reduced to left/right virtual legs, which
is the reduction the coronal question actually needs. DOFs (x, y, z, rho) and
rates; yaw and pitch are excluded, exactly the constraints RTL-CTR-SLIP keeps
(removing yaw is the later turning extension; this file is about ROLL stability
of the cambered gait, which Chang Fig. 13 says is the thing that fails).

Each leg's touchdown pose is commanded in the body frame: sagittal landing
angle beta (slip_rf convention: the foot lands ahead of the hip by l0*cos(beta))
and the side's cambered contact geometry from `coronal_bip.side_geometry` --
lateral offset d_out(lambda) outboard of the hip, rest length l0(lambda). At
touchdown the foot PINS at its world point (unlike coronal_bip's v1 sliding
contact); stance force is the spring along hip->foot, applied at the hip, so a
tilted leg makes lateral force and roll moment. Legs touch down and lift off
independently, so the four Chang phases (double/left/right/flight) arise from
the hybrid event sequence.

THE MAP AND ITS APEX STATE

Apex (flight, vz = 0) to next apex. State x = [vx, vy, h, rho, drho] -- the
first three are Sovukluk's trio; roll and roll rate ride along because they are
the states whose instability this stage exists to fix. Inputs
u = [beta, lam_left, lam_right] (stiffness available but held). Periodic gaits
solve min || x - E f(x, u) ||; for the pronk successive apexes repeat, so
E = I. Sovukluk's leg-swap trick E = diag(1,-1,1,-1,-1) is provided for
alternating gaits but is NOT the pronk's case. Deadbeat K = -pinv(J_u) J_x with
both Jacobians by central difference, matching gslip_fixed_point's convention.

An Ackermann pair helper couples lam_out to lam_in (cot lam_out - cot lam_in =
track/h), per camber_apex_geometry: uniform camber cannot roll drill-free on
multiple contacts, so the meaningful turning input is the pair, one dimension.

MODULE 4 -- ACTUATED STABILIZATION

`RollPD` supplies a stance-phase ABAD roll torque -tau(kp*rho + kd*drho),
clamped to a torque budget (default sized against the ABAD's 44.25 N.m ceiling
x 2 joints per side x a duty margin; section 37's warning applies -- the joint
already works near its limit in the running gait, so report the peak used).
`steps_to_fail` and `perturbation_grid` evaluate survival, not eigenvalues --
Chang's own headline (0.59% -> 100% surviving 7 steps) is a survival statistic.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, least_squares

from legwheel.models.gslip import GSlipFailure
from legwheel.models.coronal_bip import (
    HALF_TRACK_HIP, I_ROLL_DEFAULT, K_SIDE_DEFAULT, MASS_DEFAULT,
    SideGeometry, side_geometry,
)

G = 9.81
CONTACT_TRACK = 0.4234   # Stage 0: the wheel planes, not the hips


@dataclass
class PairParams:
    m: float = MASS_DEFAULT
    j_roll: float = I_ROLL_DEFAULT
    w_hip: float = HALF_TRACK_HIP
    k_side: float = K_SIDE_DEFAULT
    g: float = G
    rho_fail: float = np.deg2rad(30.0)   # beyond this the gait is lost
    h_min: float = 0.02                  # apex below this is a fall


@dataclass
class RollPD:
    """Stance-phase ABAD roll stabilization -- Module 4's actuation.

    tau = -(kp*rho + kd*drho), clamped to +/-tau_max. Defaults: kd critical
    against J_roll; tau_max = 2 joints/side * 44.25 N.m * 0.45 duty-ish margin,
    rounded down -- generous but not fantasy. peak_used records what the
    controller actually demanded, for the section 37 budget conversation.
    """

    kp: float = 400.0
    kd: float = 31.0
    tau_max: float = 40.0
    peak_used: float = field(default=0.0, compare=False)

    def torque(self, rho: float, drho: float) -> float:
        raw = -(self.kp * rho + self.kd * drho)
        self.peak_used = max(self.peak_used, abs(raw))
        return float(np.clip(raw, -self.tau_max, self.tau_max))


def ackermann_pair(lam_in: float, ride_height: float,
                   track: float = CONTACT_TRACK) -> tuple[float, float]:
    """(lam_in, lam_out) rolling-consistent pair; lam_out from the apex
    condition cot(lam_out) = cot(lam_in) + track/h. lam_in = 0 -> both zero."""
    if abs(lam_in) < 1e-12:
        return 0.0, 0.0
    cot_out = 1.0 / np.tan(abs(lam_in)) + track / ride_height
    lam_out = float(np.arctan(1.0 / cot_out))
    return float(abs(lam_in)), np.copysign(lam_out, lam_in)


def _foot_offset_body(geom: SideGeometry, beta: float, s: int) -> np.ndarray:
    """Foot position relative to the hip, body frame, at the touchdown pose."""
    fx = geom.l0 * np.cos(beta)
    fy = s * geom.d_out
    under = geom.l0**2 - fx**2 - fy**2
    if under <= 0.0:
        raise GSlipFailure("touchdown pose exceeds leg length")
    return np.array([fx, fy, -np.sqrt(under)])


def _hip_world(state: np.ndarray, w_hip: float, s: int) -> np.ndarray:
    x, y, z, rho = state[:4]
    return np.array([x, y + s * w_hip * np.cos(rho), z + s * w_hip * np.sin(rho)])


def _foot_world_flight(state: np.ndarray, p: PairParams, geom: SideGeometry,
                       beta: float, s: int) -> np.ndarray:
    """World foot position while the leg holds its touchdown pose in flight."""
    x, y, z, rho = state[:4]
    off = _foot_offset_body(geom, beta, s)
    hip_off_y = s * p.w_hip
    cy = hip_off_y + off[1]
    cz = off[2]
    return np.array([
        x + off[0],
        y + cy * np.cos(rho) - cz * np.sin(rho),
        z + cy * np.sin(rho) + cz * np.cos(rho),
    ])


def _stance_rhs(t, st, p: PairParams, legs: dict, ctrl: RollPD | None):
    x, y, z, rho, vx, vy, vz, drho = st
    fx = fy = fz = tau = 0.0
    for s, (geom, foot) in legs.items():
        hip = _hip_world(st, p.w_hip, s)
        vec = hip - foot
        length = float(np.linalg.norm(vec))
        if length >= geom.l0 or length < 1e-9:
            continue
        f = p.k_side * (geom.l0 - length)
        ux = vec / length
        fx += f * ux[0]
        fy += f * ux[1]
        fz += f * ux[2]
        ry = s * p.w_hip * np.cos(rho)
        rz = s * p.w_hip * np.sin(rho)
        tau += ry * f * ux[2] - rz * f * ux[1]
    if ctrl is not None and legs:
        tau += ctrl.torque(rho, drho)
    return [vx, vy, vz, drho,
            fx / p.m, fy / p.m, fz / p.m - p.g, tau / p.j_roll]


def _flight_touchdown_time(state, p: PairParams, geom: SideGeometry,
                           beta: float, s: int, t_max: float) -> float | None:
    """Earliest t at which side s's held foot reaches the ground, or None."""

    def foot_z(t):
        st = state.copy()
        st[0] += state[4] * t
        st[1] += state[5] * t
        st[2] += state[6] * t - 0.5 * p.g * t**2
        st[3] += state[7] * t
        return _foot_world_flight(st, p, geom, beta, s)[2]

    ts = np.linspace(0.0, t_max, 240)
    vals = [foot_z(t) for t in ts]
    for a, b in zip(range(len(ts) - 1), range(1, len(ts))):
        if vals[a] > 0.0 >= vals[b]:
            return float(brentq(foot_z, ts[a], ts[b], xtol=1e-12))
    return None


def _pin_ties(state, p: PairParams, geoms: dict, beta: float,
              legs: dict, tol: float = 1e-9) -> None:
    """Pin any not-yet-landed leg whose foot is already at the ground.

    The pronk lands BOTH legs at the same instant. An event function that is
    zero at the start of an integration interval does not fire, so relying on
    the touchdown event for the second leg silently produces a one-legged
    stance -- which injects roll into a perfectly symmetric gait and reads as
    a physics result. Sweep for ties explicitly instead.
    """
    for s in (+1, -1):
        if s in legs:
            continue
        if _foot_world_flight(state, p, geoms[s], beta, s)[2] <= tol:
            legs[s] = (geoms[s], _foot_world_flight(state, p, geoms[s], beta, s))


def _release_ties(state, p: PairParams, legs: dict, tol: float = 1e-9) -> None:
    """Release any still-pinned leg already at or past its rest length.

    The liftoff mirror of `_pin_ties`: the pronk's legs lift TOGETHER, the
    event machinery reports one, and the other -- already past zero -- never
    fires again. Left pinned, it re-engages as a tether when the body comes
    back down, which reads as a physics result (a vault that ends with
    downward velocity) rather than the bookkeeping bug it is.
    """
    for s in list(legs):
        geom, foot = legs[s]
        length = float(np.linalg.norm(_hip_world(state, p.w_hip, s) - foot))
        if length >= geom.l0 - tol:
            del legs[s]


def apex_map(p: PairParams, apex, u, ctrl: RollPD | None = None) -> np.ndarray:
    """One stride: apex -> next apex.

    apex = [vx, vy, h, rho, drho]; u = [beta, lam_left, lam_right].
    Raises GSlipFailure on fall, roll blow-up, or a leg that never lands.
    """
    vx, vy, h, rho, drho = (float(v) for v in apex)
    beta, lam_l, lam_r = (float(v) for v in u)
    if h < p.h_min:
        raise GSlipFailure("apex too low")
    if abs(rho) > p.rho_fail:
        raise GSlipFailure("roll beyond failure limit at apex")
    geoms = {+1: side_geometry(lam_l), -1: side_geometry(lam_r)}

    state = np.array([0.0, 0.0, h, rho, vx, vy, 0.0, drho])
    legs: dict = {}

    # -- flight to first touchdown ---------------------------------------
    t_ball = 2.0 * np.sqrt(2.0 * h / p.g) + 0.3
    times = {}
    for s in (+1, -1):
        t_td = _flight_touchdown_time(state, p, geoms[s], beta, s, t_ball)
        if t_td is not None:
            times[s] = t_td
    if not times:
        raise GSlipFailure("no touchdown found")

    def advance_flight(st, t):
        out = st.copy()
        out[0] += st[4] * t
        out[1] += st[5] * t
        out[2] += st[6] * t - 0.5 * p.g * t**2
        out[3] += st[7] * t
        out[6] -= p.g * t
        return out

    s_first = min(times, key=times.get)
    state = advance_flight(state, times[s_first])
    legs[s_first] = (geoms[s_first],
                     _foot_world_flight(state, p, geoms[s_first], beta, s_first))
    _pin_ties(state, p, geoms, beta, legs)

    # -- hybrid stance loop ----------------------------------------------
    for _hop in range(8):
        pending = [s for s in (+1, -1) if s not in legs]

        def liftoff_events():
            evs = []
            for s in list(legs):
                def ev(t, st, s=s):
                    geom, foot = legs[s]
                    return float(np.linalg.norm(_hip_world(st, p.w_hip, s) - foot)
                                 - geom.l0)
                ev.terminal = True
                ev.direction = 1.0
                evs.append((("lift", s), ev))
            for s in pending:
                def ev(t, st, s=s):
                    return float(_foot_world_flight(st, p, geoms[s], beta, s)[2])
                ev.terminal = True
                ev.direction = -1.0
                evs.append((("touch", s), ev))

            def fell(t, st):
                return st[2] - p.h_min
            fell.terminal = True
            fell.direction = -1.0
            evs.append((("fell", 0), fell))

            def rolled(t, st):
                return p.rho_fail - abs(st[3])
            rolled.terminal = True
            rolled.direction = -1.0
            evs.append((("rolled", 0), rolled))
            return evs

        evs = liftoff_events()
        wrapped = []
        for _tag, ev in evs:
            def w(t, st, _p_, _l_, _c_, ev=ev):
                return ev(t, st)
            w.terminal = ev.terminal
            w.direction = ev.direction
            wrapped.append(w)

        sol = solve_ivp(_stance_rhs, (0.0, 1.0), state,
                        args=(p, legs, ctrl), events=wrapped,
                        rtol=1e-9, atol=1e-11, max_step=5e-4)
        hits = [i for i, te in enumerate(sol.t_events) if len(te)]
        if not hits:
            raise GSlipFailure("stance did not terminate")
        first = min(hits, key=lambda i: sol.t_events[i][0])
        tag, s_ev = evs[first][0]
        state = sol.y_events[first][0].copy()

        if tag == "fell":
            raise GSlipFailure("fell during stance")
        if tag == "rolled":
            raise GSlipFailure("roll beyond failure limit in stance")
        if tag == "touch":
            legs[s_ev] = (geoms[s_ev],
                          _foot_world_flight(state, p, geoms[s_ev], beta, s_ev))
            _pin_ties(state, p, geoms, beta, legs)
            continue
        del legs[s_ev]
        _release_ties(state, p, legs)
        if not legs:
            break
    else:
        raise GSlipFailure("stance phase sequence did not converge")

    # -- flight to apex ----------------------------------------------------
    vz = state[6]
    if vz <= 0.0:
        raise GSlipFailure("no upward velocity at final liftoff")
    t_ap = vz / p.g
    state = advance_flight(state, t_ap)
    if abs(state[3]) > p.rho_fail:
        raise GSlipFailure("roll beyond failure limit at next apex")
    return np.array([state[4], state[5], state[2], state[3], state[7]])


# ---------------------------------------------------------------------------
# Sovukluk machinery
# ---------------------------------------------------------------------------

E_PRONK = np.eye(5)
E_ALTERNATE = np.diag([1.0, -1.0, 1.0, -1.0, -1.0])


def solve_periodic(p: PairParams, x0, u0, e_matrix=E_PRONK,
                   free_x=(2,), free_u=(0,)) -> tuple[np.ndarray, np.ndarray]:
    """Periodic gait: adjust the listed apex-state and input components to
    minimise ||x - E f(x, u)||. Defaults solve (h, beta) at fixed speed --
    the fewest unknowns that close the pronk's symmetric gait."""
    x0, u0 = np.asarray(x0, float).copy(), np.asarray(u0, float).copy()

    def residual(q):
        x, u = x0.copy(), u0.copy()
        for i, idx in enumerate(free_x):
            x[idx] = q[i]
        for j, idx in enumerate(free_u):
            u[idx] = q[len(free_x) + j]
        try:
            return x - e_matrix @ apex_map(p, x, u)
        except GSlipFailure:
            return np.full(5, 10.0)

    q0 = np.concatenate([[x0[i] for i in free_x], [u0[j] for j in free_u]])
    res = least_squares(residual, q0, xtol=1e-12, ftol=1e-12, diff_step=1e-6)
    if not res.success or np.linalg.norm(res.fun) > 1e-5:
        raise GSlipFailure(f"no periodic gait (residual {np.linalg.norm(res.fun):.2e})")
    x, u = x0.copy(), u0.copy()
    for i, idx in enumerate(free_x):
        x[idx] = res.x[i]
    for j, idx in enumerate(free_u):
        u[idx] = res.x[len(free_x) + j]
    return x, u


def jacobians(p: PairParams, x_star, u_star, hx=1e-6, hu=1e-6):
    """J_x (5x5) and J_u (5x3) of the apex map, central difference."""
    x_star, u_star = np.asarray(x_star, float), np.asarray(u_star, float)
    jx = np.zeros((5, 5))
    for i in range(5):
        d = np.zeros(5)
        d[i] = hx
        jx[:, i] = (apex_map(p, x_star + d, u_star)
                    - apex_map(p, x_star - d, u_star)) / (2 * hx)
    ju = np.zeros((5, 3))
    for j in range(3):
        d = np.zeros(3)
        d[j] = hu
        ju[:, j] = (apex_map(p, x_star, u_star + d)
                    - apex_map(p, x_star, u_star - d)) / (2 * hu)
    return jx, ju


def deadbeat_gain(jx: np.ndarray, ju: np.ndarray, rcond: float = 1e-2) -> np.ndarray:
    """K = -pinv(J_u) J_x : u = u* + K (x - x*) cancels the linearised error
    in one stride, least-squares when J_u is not square.

    `rcond` matters physically, not just numerically. At the symmetric gait the
    SYMMETRIC camber combination (lam_l = lam_r) is a near-null input -- its
    singular value is ~500x below beta's -- so an unregularised pseudoinverse
    commands tens of RADIANS of camber to fix a percent-level speed error.
    The ANTISYMMETRIC combination (differential camber) has genuine authority
    over roll and survives the cut. Measured at the v~0.70 point: sigma(beta)
    ~ 3.4, sigma(differential lam) ~ 0.16, sigma(symmetric lam) ~ 0.0065;
    rcond = 1e-2 keeps the first two and drops the third, which is exactly the
    steering structure the apex geometry predicts.
    """
    return -np.linalg.pinv(ju, rcond=rcond) @ jx


def steps_to_fail(p: PairParams, x_star, u_star, x0,
                  ctrl: RollPD | None = None,
                  gain: np.ndarray | None = None,
                  max_steps: int = 20) -> int:
    """Strides survived from apex state x0. Chang's metric, not eigenvalues."""
    x = np.asarray(x0, float).copy()
    x_star, u_star = np.asarray(x_star, float), np.asarray(u_star, float)
    for n in range(max_steps):
        u = u_star.copy()
        if gain is not None:
            u = u_star + gain @ (x - x_star)
        try:
            x = apex_map(p, x, u, ctrl=ctrl)
        except GSlipFailure:
            return n
    return max_steps


def perturbation_grid(p: PairParams, x_star, u_star, rho_vals, drho_vals,
                      **kwargs) -> np.ndarray:
    """steps_to_fail over a (rho, drho) apex-perturbation grid."""
    out = np.zeros((len(rho_vals), len(drho_vals)), dtype=int)
    for i, r in enumerate(rho_vals):
        for j, dr in enumerate(drho_vals):
            x0 = np.asarray(x_star, float).copy()
            x0[3] += r
            x0[4] += dr
            out[i, j] = steps_to_fail(p, x_star, u_star, x0, **kwargs)
    return out
