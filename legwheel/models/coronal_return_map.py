"""Pinned-contact coronal dynamics and apex return map -- coronal_bip v2.

`coronal_bip` (v1) places each contact quasi-statically under the current hip:
the contact slides with the body and the leg-length rate ignores lateral CoM
motion, so the tilted-leg force is non-conservative whenever the body moves
laterally -- exactly the regime the roll-instability question lives in. v1
stays as built (its statics closed the section 47 validation at 0.99-1.01 and
its tests pin that); this module is the refinement its own docstring deferred:
feet PIN at touchdown, the way `cambered_return_map` already does it in 3D.

THE MODEL

Same body as v1 -- point mass m with roll inertia J_roll, hips at +/-w_hip,
per-side spring legs whose contact geometry comes from the SAME seam
(`coronal_bip.side_geometry`), states (y, z, rho, vy, vz, vrho). Differences:

  1. While unloaded, a leg holds its touchdown pose: foot s*d_out outboard of
     the hip, sqrt(l0^2 - d_out^2) below it. Touchdown is the event foot_z = 0;
     at that instant the foot pins at its world point.
  2. In stance the spring acts along hip->pinned-foot; the length rate is the
     true d|hip - foot|/dt, so the force is conservative by construction.
  3. Touchdown and liftoff are events, with explicit tie sweeps -- the
     symmetric bounce lands and lifts BOTH legs in the same instant, and an
     event function that is zero at the start of an interval never fires
     (`cambered_return_map._pin_ties`' lesson, ported).

THE APEX MAP (Module 2's dynamics, for the Chang 2022 reproduction)

Apex (flight, vz = 0) to next apex. Apex state x = [vy, h, rho, drho]; y is
cyclic (translation invariance) and carries no dynamics. There are no inputs:
the BIP has no clock and no touchdown command -- asymmetry enters through the
parameters (k_left != k_right, Chang's stiffness ratio). Periodic orbits solve
min || x - E f(x) ||; E = I for the bounce-in-place pronk. `jacobian` is
E-aware, unlike `cambered_return_map`'s (where only E = I is exercised):
alternating-contact orbits close through the mirror, so their stability must
be read off E J, not J.

NAMING: Chang's stiffness-ratio parameter is called gamma in the paper. In
this repo `gamma` is the ABAD joint angle, everywhere. To keep grep honest the
ratio is only ever `k_ratio` here, applied one-sided via
`with_stiffness_ratio` (k_left = ratio * k, k_right = k). If Chang's
definition turns out mean-preserving, change that ONE helper.
"""

from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, least_squares

from legwheel.models.gslip import GSlipFailure
from legwheel.models.coronal_bip import CoronalParams, SideGeometry

RHO_FAIL = np.deg2rad(30.0)   # beyond this the bounce is lost
H_MIN = 0.02                  # CoM below this is a fall

E_BOUNCE = np.eye(4)
# Alternating-contact orbits close through a left/right mirror of the reduced
# apex state [vy, h, rho, drho]:
E_MIRROR = np.diag([-1.0, 1.0, -1.0, -1.0])


def with_stiffness_ratio(p: CoronalParams, k_ratio: float) -> CoronalParams:
    """STATIC one-sided stiffness asymmetry: k_left = k_ratio * k_right.

    NOT Chang's gamma (read the PDF before assuming otherwise -- we did, the
    other way round, and section 59's first framing paid for it). Chang 2022
    Eq. 1-3: each side ALTERNATES between k0 and k1 through a "gamma phase"
    that exchanges the constants at every apex (the tripod's two leg sets:
    two legs vs one, so gamma = k0/k1 = 2 is the natural RHex value), with
    k_sum = k0 + k1 fixed. That convention lives in `with_gamma_phase` and
    `apex_map_two_step`. This helper models a PERMANENTLY stiffer side --
    the Ackermann inner/outer and inner/outer asymmetry cases -- which is a
    different physical object. k_ratio = 1 returns an equivalent symmetric p.
    """
    from dataclasses import replace
    return replace(p, k_left=k_ratio * p.k_right)


def with_gamma_phase(p: CoronalParams, gamma_ratio: float,
                     phase: int = 0) -> CoronalParams:
    """Chang 2022's stiffness convention (Eq. 1-3), mean-preserving.

    k0 = gamma/(1+gamma) * k_sum on one side, k1 = 1/(1+gamma) * k_sum on
    the other, k_sum = p.k_left + p.k_right held fixed; `phase` 0 puts k0 on
    the left, 1 exchanges them (the gamma-phase transition that happens at
    each apex). gamma_ratio = 1 returns a symmetric p with the same k_sum.
    """
    from dataclasses import replace
    k_sum = p.k_left + p.k_right
    k0 = gamma_ratio / (1.0 + gamma_ratio) * k_sum
    k1 = k_sum - k0
    if phase == 0:
        return replace(p, k_left=k0, k_right=k1)
    return replace(p, k_left=k1, k_right=k0)


def apex_map_two_step(p: CoronalParams, apex, gamma_ratio: float) -> np.ndarray:
    """Chang's Poincare map: two apex-to-apex steps with the gamma-phase
    exchange between them (his mapping is [rho_n, drho_n] ->
    [rho_n+2, drho_n+2] precisely because one step swaps the leg sets and
    two steps restore them). Our apex state keeps [vy, h, rho, drho]; note
    Chang's BIP pins the CoM laterally (z-only), so vy is a departure --
    report it, don't hide it."""
    x = apex_map(with_gamma_phase(p, gamma_ratio, phase=0), apex)
    return apex_map(with_gamma_phase(p, gamma_ratio, phase=1), x)


def _geom(p: CoronalParams, s: int) -> SideGeometry:
    return p.left if s > 0 else p.right


def _k(p: CoronalParams, s: int) -> float:
    return p.k_left if s > 0 else p.k_right


def _hip(state, p: CoronalParams, s: int) -> np.ndarray:
    y, z, rho = state[0], state[1], state[2]
    return np.array([y + s * p.w_hip * np.cos(rho),
                     z + s * p.w_hip * np.sin(rho)])


def _foot_held(state, p: CoronalParams, s: int) -> np.ndarray:
    """World foot position while side s holds its touchdown pose."""
    geom = _geom(p, s)
    hip = _hip(state, p, s)
    drop = np.sqrt(max(geom.l0**2 - geom.d_out**2, 1e-12))
    return np.array([hip[0] + s * geom.d_out, hip[1] - drop])


def _stance_rhs(t, st, p: CoronalParams, legs: dict):
    y, z, rho, vy, vz, vrho = st
    fy = fz = tau = 0.0
    for s, foot in legs.items():
        geom = _geom(p, s)
        hip = _hip(st, p, s)
        vec = hip - foot
        length = float(np.linalg.norm(vec))
        if length >= geom.l0 or length < 1e-9:
            continue
        f = _k(p, s) * (geom.l0 - length)
        u = vec / length
        fy += f * u[0]
        fz += f * u[1]
        ry = s * p.w_hip * np.cos(rho)
        rz = s * p.w_hip * np.sin(rho)
        tau += ry * f * u[1] - rz * f * u[0]
    return [vy, vz, vrho, fy / p.m, fz / p.m - p.g, tau / p.j_roll]


def _pin_ties(state, p: CoronalParams, legs: dict, tol: float = 1e-9) -> None:
    """Pin any not-yet-landed leg whose held foot is at (or through) the
    ground -- the symmetric bounce lands both legs in the same instant.

    Pin at the foot's ACTUAL held position, never projected to z = 0. A body
    that rolled hard in flight can carry the second leg's held pose well below
    ground at the instant the first leg lands; projecting that foot up to the
    ground plane manufactures instant spring compression -- measured at +89 J
    on a 89 J trajectory, a clean doubling -- where pinning at the held pose
    keeps length = l0 and injects nothing (`cambered_return_map._pin_ties`
    does the same, which is why the 3D map never showed this)."""
    for s in (+1, -1):
        if s in legs:
            continue
        foot = _foot_held(state, p, s)
        if foot[1] <= tol:
            legs[s] = foot.copy()


def _release_ties(state, p: CoronalParams, legs: dict,
                  tol: float = 1e-9) -> None:
    """Release any still-pinned leg at or past its rest length (the symmetric
    bounce lifts both together; the unfired one would re-engage as a tether)."""
    for s in list(legs):
        geom = _geom(p, s)
        if float(np.linalg.norm(_hip(state, p, s) - legs[s])) >= geom.l0 - tol:
            del legs[s]


def energy_pinned(p: CoronalParams, state, legs: dict | None = None) -> float:
    """Total mechanical energy under pinned contacts. `legs` maps side to its
    pinned world foot; None or empty means flight (no spring terms)."""
    y, z, rho, vy, vz, vrho = state
    e = 0.5 * p.m * (vy**2 + vz**2) + 0.5 * p.j_roll * vrho**2 + p.m * p.g * z
    for s, foot in (legs or {}).items():
        geom = _geom(p, s)
        length = float(np.linalg.norm(_hip(state, p, s) - foot))
        if length < geom.l0:
            e += 0.5 * _k(p, s) * (geom.l0 - length) ** 2
    return float(e)


def _segment_events(p: CoronalParams, legs: dict):
    """Terminal events for one hybrid segment: liftoffs for pinned legs,
    touchdowns for held legs, fall, roll blow-up. Returns (tags, funcs)."""
    tags, funcs = [], []
    # Event signatures absorb solve_ivp's args=(p, legs) via *_ -- the same
    # trap cambered_return_map handles by wrapping.
    for s in list(legs):
        def lift(t, st, *_, s=s):
            return float(np.linalg.norm(_hip(st, p, s) - legs[s])
                         - _geom(p, s).l0)
        lift.terminal = True
        lift.direction = 1.0
        tags.append(("lift", s))
        funcs.append(lift)
    for s in (+1, -1):
        if s in legs:
            continue

        def touch(t, st, *_, s=s):
            return float(_foot_held(st, p, s)[1])
        touch.terminal = True
        touch.direction = -1.0
        tags.append(("touch", s))
        funcs.append(touch)

    def fell(t, st, *_):
        return st[1] - H_MIN
    fell.terminal = True
    fell.direction = -1.0
    tags.append(("fell", 0))
    funcs.append(fell)

    def rolled(t, st, *_):
        return RHO_FAIL - abs(st[2])
    rolled.terminal = True
    rolled.direction = -1.0
    tags.append(("rolled", 0))
    funcs.append(rolled)
    return tags, funcs


def simulate_pinned(p: CoronalParams, state0, t_final: float,
                    rtol: float = 1e-10, atol: float = 1e-12,
                    max_segments: int = 400):
    """Hybrid trajectory under pinned contacts.

    Returns (t, states, legs) -- concatenated sample times and 6-state columns
    across all hybrid segments, and the final legs dict. Terminates early on a
    fall or roll blow-up (like v1's `simulate`, without raising).
    """
    state = np.asarray(state0, float).copy()
    legs: dict = {}
    _pin_ties(state, p, legs)
    t_now = 0.0
    ts, ys = [np.array([0.0])], [state.reshape(6, 1)]
    for _seg in range(max_segments):
        if t_now >= t_final:
            break
        tags, funcs = _segment_events(p, legs)
        sol = solve_ivp(_stance_rhs, (t_now, t_final), state,
                        args=(p, legs), events=funcs,
                        rtol=rtol, atol=atol, max_step=1e-3)
        ts.append(sol.t[1:])
        ys.append(sol.y[:, 1:])
        hits = [i for i, te in enumerate(sol.t_events) if len(te)]
        if not hits:
            break                      # ran to t_final
        first = min(hits, key=lambda i: sol.t_events[i][0])
        tag, s_ev = tags[first]
        t_now = float(sol.t_events[first][0])
        state = sol.y_events[first][0].copy()
        if tag in ("fell", "rolled"):
            break
        if tag == "touch":
            legs[s_ev] = _foot_held(state, p, s_ev).copy()
            _pin_ties(state, p, legs)
        else:
            del legs[s_ev]
            _release_ties(state, p, legs)
    return np.concatenate(ts), np.concatenate(ys, axis=1), legs


# ---------------------------------------------------------------------------
# Apex return map -- the Chang 2022 reproduction machinery
# ---------------------------------------------------------------------------


def apex_map(p: CoronalParams, apex) -> np.ndarray:
    """One bounce: apex [vy, h, rho, drho] -> next apex.

    Raises GSlipFailure on a fall, roll blow-up, or an orbit that never
    returns to a flight apex (e.g. grounded oscillation).
    """
    vy, h, rho, drho = (float(v) for v in apex)
    if h < H_MIN:
        raise GSlipFailure("apex too low")
    if abs(rho) > RHO_FAIL:
        raise GSlipFailure("roll beyond failure limit at apex")
    state = np.array([0.0, h, rho, vy, 0.0, drho])

    # -- flight to first touchdown (ballistic; roll rate constant) ----------
    def first_touchdown(st) -> float | None:
        t_max = 2.0 * np.sqrt(2.0 * max(st[1], 1e-9) / p.g) + 0.3

        def foot_z(t, s):
            fl = st.copy()
            fl[0] += st[3] * t
            fl[1] += st[4] * t - 0.5 * p.g * t**2
            fl[2] += st[5] * t
            return _foot_held(fl, p, s)[1]

        best = None
        ts = np.linspace(0.0, t_max, 240)
        for s in (+1, -1):
            vals = [foot_z(t, s) for t in ts]
            for a in range(len(ts) - 1):
                if vals[a] > 0.0 >= vals[a + 1]:
                    root = float(brentq(foot_z, ts[a], ts[a + 1], args=(s,),
                                        xtol=1e-12))
                    if best is None or root < best[0]:
                        best = (root, s)
                    break
        return best

    td = first_touchdown(state)
    if td is None:
        raise GSlipFailure("no touchdown found")
    t_td, s_td = td
    state[0] += state[3] * t_td
    state[1] += state[4] * t_td - 0.5 * p.g * t_td**2
    state[2] += state[5] * t_td
    state[4] -= p.g * t_td
    legs: dict = {}
    legs[s_td] = _foot_held(state, p, s_td).copy()
    _pin_ties(state, p, legs)

    # -- hybrid stance loop -------------------------------------------------
    for _hop in range(8):
        tags, funcs = _segment_events(p, legs)
        sol = solve_ivp(_stance_rhs, (0.0, 2.0), state, args=(p, legs),
                        events=funcs, rtol=1e-10, atol=1e-12, max_step=1e-3)
        hits = [i for i, te in enumerate(sol.t_events) if len(te)]
        if not hits:
            raise GSlipFailure("stance did not terminate")
        first = min(hits, key=lambda i: sol.t_events[i][0])
        tag, s_ev = tags[first]
        state = sol.y_events[first][0].copy()
        if tag == "fell":
            raise GSlipFailure("fell during stance")
        if tag == "rolled":
            raise GSlipFailure("roll beyond failure limit in stance")
        if tag == "touch":
            legs[s_ev] = _foot_held(state, p, s_ev).copy()
            _pin_ties(state, p, legs)
            continue
        del legs[s_ev]
        _release_ties(state, p, legs)
        if not legs:
            break
    else:
        raise GSlipFailure("stance phase sequence did not converge")

    # -- flight to apex -----------------------------------------------------
    vz = state[4]
    if vz <= 0.0:
        raise GSlipFailure("no upward velocity at final liftoff")
    t_ap = vz / p.g
    state[1] += vz * t_ap - 0.5 * p.g * t_ap**2
    state[2] += state[5] * t_ap
    if abs(state[2]) > RHO_FAIL:
        raise GSlipFailure("roll beyond failure limit at next apex")
    return np.array([state[3], state[1], state[2], state[5]])


def solve_periodic(p: CoronalParams, x0, e_matrix=E_BOUNCE,
                   free=(1,), max_nfev: int | None = None) -> np.ndarray:
    """Periodic bounce: adjust the listed apex components to minimise
    ||x - E f(x)||. No inputs to solve for -- the BIP is parameter-driven.

    NOTE the symmetric bounce is a CONTINUUM of periodic orbits (conservative
    springs: every drop height returns to itself), so with free=(1,) the
    residual is ~zero everywhere along h and the solver just polishes the
    seed. That degeneracy is real physics, not a solver fault; asymmetric
    parameters are where existence becomes a question worth asking, with
    free=(0, 1, 2, 3).
    """
    x0 = np.asarray(x0, float).copy()

    def residual(q):
        x = x0.copy()
        for i, idx in enumerate(free):
            x[idx] = q[i]
        try:
            return x - e_matrix @ apex_map(p, x)
        except GSlipFailure:
            return np.full(4, 10.0)

    res = least_squares(residual, [x0[i] for i in free],
                        xtol=1e-12, ftol=1e-12, diff_step=1e-6,
                        max_nfev=max_nfev)
    if not res.success or np.linalg.norm(res.fun) > 1e-6:
        raise GSlipFailure(
            f"no periodic bounce (residual {np.linalg.norm(res.fun):.2e})")
    x = x0.copy()
    for i, idx in enumerate(free):
        x[idx] = res.x[i]
    return x


def solve_periodic_gamma(p: CoronalParams, x0, gamma_ratio: float,
                         free=(0, 1, 2, 3),
                         max_nfev: int | None = None) -> np.ndarray:
    """Fixed point of Chang's two-step gamma-phase map, x = F(x).

    All four apex components free by default: for gamma != 1 the paper's
    fixed points carry nonzero rolling velocity (his Fig. 6 -- "the pronking
    orbit could not exist as gamma != 1"), so pinning rho or drho at zero
    would search for an orbit the paper says is not there."""
    x0 = np.asarray(x0, float).copy()

    def residual(q):
        x = x0.copy()
        for i, idx in enumerate(free):
            x[idx] = q[i]
        try:
            return x - apex_map_two_step(p, x, gamma_ratio)
        except GSlipFailure:
            return np.full(4, 10.0)

    res = least_squares(residual, [x0[i] for i in free],
                        xtol=1e-12, ftol=1e-12, diff_step=1e-6,
                        max_nfev=max_nfev)
    if not res.success or np.linalg.norm(res.fun) > 1e-6:
        raise GSlipFailure(
            f"no two-step orbit (residual {np.linalg.norm(res.fun):.2e})")
    x = x0.copy()
    for i, idx in enumerate(free):
        x[idx] = res.x[i]
    return x


def jacobian_gamma(p: CoronalParams, x_star, gamma_ratio: float,
                   h_step: float = 1e-6) -> np.ndarray:
    """dF/dx of the two-step gamma-phase map at x_star, central difference --
    Chang Eq. 9's finite-difference Jacobian, on the full 4-state."""
    x_star = np.asarray(x_star, float)
    j = np.zeros((4, 4))
    for i in range(4):
        d = np.zeros(4)
        d[i] = h_step
        j[:, i] = (apex_map_two_step(p, x_star + d, gamma_ratio)
                   - apex_map_two_step(p, x_star - d, gamma_ratio)) \
            / (2 * h_step)
    return j


def jacobian(p: CoronalParams, x_star, e_matrix=E_BOUNCE,
             h_step: float = 1e-6) -> np.ndarray:
    """d(E f)/dx at x_star, central difference -- the matrix whose eigenvalues
    decide stability of an orbit that closes through E. For E = I this is the
    plain apex-map Jacobian."""
    x_star = np.asarray(x_star, float)
    j = np.zeros((4, 4))
    for i in range(4):
        d = np.zeros(4)
        d[i] = h_step
        j[:, i] = (e_matrix @ apex_map(p, x_star + d)
                   - e_matrix @ apex_map(p, x_star - d)) / (2 * h_step)
    return j
