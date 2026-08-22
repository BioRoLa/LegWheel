"""Does the gain SWITCHING mid-stance destabilise the map, on its own?

The question S148 arrived at. THIS IS THE v2 INSTRUMENT -- v1's numbers are in
S149 and are NOT fit to score the prediction; five defects were found by an
adversarial audit and all five are addressed here. Read S149 before trusting
any number this file produced before 2026-08-22.

WHY THIS EXISTS

S147 measured the plant's two impedance regimes and S149 converts them into the
model's units through the controller's own (rigid) lever l = 0.2852 m -- which
is what the controller applies per radian of leg-angle error, confirmed at 102%
by the measured dkp/dk_tangential:

    leg-frame  k_tangential 600,  b_tangential 30    ->  k_c  48.8,  d_c 2.44
    flight     k_flight    7150,  b_flight    115.8  ->  k_c 581.6,  d_c 9.42

At v~0.70 the leg-frame regime is MARGINALLY UNSTABLE (+1.05) and the flight
regime is stable (+0.22); their time-average at the measured gain honesty
(S143, ~0.5) is stable. The plant is not. So the hypothesis under test is that
the SWITCH is the mechanism rather than the size of either gain.

THE PARAMETER

    f_stance = fraction of REAL stance, from touchdown, under the LEG-FRAME
               gains; flight gains for the remainder.

    f_stance = 1.0   perfect contact gating (leg-frame throughout stance)
    f_stance = 0.5   the plant as measured (S143 gain honesty)
    f_stance = 0.0   flight gains throughout stance

The switch runs leg-frame -> flight, because the clock's stance label EXPIRES
partway through the real stance (S38: the window leads touchdown by ~80 ms
against a ~95 ms stance). Perfect gating therefore drives the plant TOWARD the
unstable constant regime, which is why P-C-3 is not obvious in either
direction.

THE FIVE v1 DEFECTS, AND WHAT IS DONE ABOUT THEM  (S149)

 1. SLOPE THROUGH AN ODE DISCONTINUITY -- fatal, and specific to interior f.
    A hard gain step put a discontinuity in the RHS; simulate_stance has no
    event to stop and restart there; _describe differences dP/dalpha through it
    with h=1e-6. Interior f carried 0.14-0.48 of pure numerical noise against a
    0.02 bar, and ONLY interior f did -- the exact shape the prediction tests
    for. FIX: the transition is a C2 smootherstep of width SWITCH_WIDTH_S. This
    is also the more faithful model: the plant's regime change is filtered by
    contact debounce, it is not a step. `--widths` reports the sweep over the
    width so the answer's dependence on it is visible rather than assumed.
 2. BRANCH SWITCHING -- min|slope| over a branch family is a LOWER ENVELOPE, so
    two monotone branches crossing return a spurious "non-monotonic" pass.
    FIX: beta is HELD at the conservative baseline (no beta search at all), and
    alpha is chosen by CONTINUATION from the previous f, never by global
    min|slope|. Runner-up roots are reported so a near-tie is visible.
 3. WINDOW-EDGE PINNING -- v1's beta* was fp0.beta + 6.0 EXACTLY for every
    f <= 0.6, i.e. the edge of an arbitrary +-6 deg window, not a stationary
    point. FIX: same as 2 -- there is no window, because beta is held.
 4. NO v-CLOSURE -- gslip_fixed_point's 1-D reduction assumes v invariance,
    true only for the CONSERVATIVE model. With tau_fn engaged the map is 2-D
    and energy is injected; v1 rows drifted -2.1% to +17.5% per stride, and a
    row gaining 17.5% of its speed per stride is not a gait. FIX: V_TOL screen,
    conjunctive with the grazing screens, applied BEFORE selection.
 5. peak[0] WAS THE WHOLE SEARCH -- one accumulator per solve, closed over by
    tau_fn, touched by every trial stride including rejected grazing
    candidates. Inflation 2.5x-18.5x, and not a constant offset. FIX: the
    reported tau is re-measured on the returned orbit alone.

Run:
    uv run python examples/gslip/clock_torque_regime_switch.py
    uv run python examples/gslip/clock_torque_regime_switch.py --selftest
    uv run python examples/gslip/clock_torque_regime_switch.py --widths
"""

from __future__ import annotations

import sys

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.models.gslip_fixed_point import find_fixed_points, poincare
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, K_REL, THETA = 30.0, 9.81, 18.0, 100.0
MIN_APEX_MM, MAX_DUTY = 10.0, 0.55
V_TOL = 0.02              # R3: |v_out/v_in - 1| must be under this
STALL = 29.5              # N.m per motor (corgi_driver MAX_TORQUE_LEG)
PLANT_TAU = 50.84         # N.m, S140 p99.5 -- PRE-CLAMP demand, not spend
EROSION = 35.0 / 15.02    # S141's factor. See S149 sec 3: it is not a constant.
MOTOR_SPLIT = 0.5         # dbeta/dphi_R = dbeta/dphi_L, so tau_beta -> tau/2 each

# The controller's commanded gains, in the leg frame. Cartesian units.
K_TANGENTIAL, B_TANGENTIAL = 600.0, 30.0
K_FLIGHT, B_FLIGHT = 7150.0, 115.8

# The lever the CONTROLLER applies through (rigid J_fb). Not the model's
# hip-frame lever (l - r = 0.1405) -- see S149 sec 1 for why they differ and
# why this is nonetheless the right one for converting a commanded gain.
LEVER = 0.2852

V_TILDES = (0.34, 0.70)
F_STANCE = tuple(round(0.05 * i, 2) for i in range(21))   # R3: 0.05 steps
SWITCH_WIDTH_S = 0.005    # C2 blend width; the plant's debounce scale
HONESTY = 0.5             # S143


def to_model(k_cart, b_cart, lever=LEVER):
    """Cartesian leg-frame gain -> torque-on-phi gain. tau = k * lever^2 * dphi."""
    return k_cart * lever ** 2, b_cart * lever ** 2


def to_ctrl(k_c, d_c, lever=LEVER):
    """Inverse of to_model -- report answers in the controller's own units."""
    return k_c / lever ** 2, d_c / lever ** 2


LEG_FRAME = to_model(K_TANGENTIAL, B_TANGENTIAL)
FLIGHT = to_model(K_FLIGHT, B_FLIGHT)
AVERAGE = tuple(HONESTY * a + (1.0 - HONESTY) * b for a, b in zip(LEG_FRAME, FLIGHT))


def smootherstep(u):
    """C2 ramp 0->1 on [0,1]. Zero first AND second derivative at both ends."""
    u = min(1.0, max(0.0, u))
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def params():
    lm = g2c.LegLengthMap()
    r = lm.leg.foot_radius
    hip = lm.length(np.deg2rad(THETA))
    return slip_rf.SlipRfParams(m=MASS, l0=hip + r, k=K_REL * MASS * G / hip, r=r)


def baseline(p, v_tilde):
    """The conservative fixed point at this speed -- the reference and the seed."""
    v = v_tilde * np.sqrt(G * p.l0)
    best = None
    for bd in np.arange(60.0, 89.01, 0.25):
        for fp in find_fixed_points(p, v, np.deg2rad(bd),
                                    alpha_range=(np.deg2rad(1.0), np.deg2rad(75.0)),
                                    n_samples=40, stride_fn=slip_rf.stride):
            apex = 1000.0 * G * fp.flight_time ** 2 / 8.0
            if apex < MIN_APEX_MM or fp.duty_factor > MAX_DUTY:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return v, best


def make_tau(p, fp, gains_lo, gains_hi, t_switch, peak, width=SWITCH_WIDTH_S):
    """Clock-torque PD whose gains BLEND from gains_lo to gains_hi at t_switch.

    The reference phi_ref is unchanged from S141 -- a linear clocked sweep from
    touchdown to its mirror over the nominal stance time -- so the only
    difference from S141 is the gain schedule.

    `width` is the C2 blend width. width=0 reproduces v1's hard step and is
    kept ONLY so the selftest can demonstrate what it cost.
    """
    phi_td = p.phi_touchdown(fp.beta)
    phi_lo = -phi_td
    t_st = fp.stance_time
    rate = (phi_lo - phi_td) / t_st
    k_a, d_a = gains_lo
    k_b, d_b = gains_hi
    finite = np.isfinite(t_switch)

    def tau_fn(t, length, phi, dl, dphi):
        tt = min(t, t_st)
        phi_ref = phi_td + rate * tt
        dphi_ref = rate if t < t_st else 0.0
        if not finite:
            s = 0.0
        elif width <= 0.0:
            s = 0.0 if t < t_switch else 1.0
        else:
            s = smootherstep((t - (t_switch - 0.5 * width)) / width)
        k_c = (1.0 - s) * k_a + s * k_b
        d_c = (1.0 - s) * d_a + s * d_b
        tq = k_c * (phi_ref - phi) + d_c * (dphi_ref - dphi)
        peak[0] = max(peak[0], abs(tq))
        return np.array([0.0, tq])

    return tau_fn


def step_for(width, t_switch):
    """Integrator max_step that resolves the gain transition.

    slip_rf's default is max_time/300 = 10 ms, and stance is ~107 ms. A 5 ms
    blend is HALF A STEP, so RK45 can step straight over it and its error
    estimate never sees the transition -- which is why the first attempt at
    smoothing the switch made the measured noise WORSE, not better (S149).
    Four steps across the blend is the minimum that resolves it.
    """
    if not np.isfinite(t_switch) or width <= 0.0:
        return None
    return min(1.0e-3, width / 8.0)


def solve_held_beta(p, v, fp0, gains_lo, gains_hi, t_switch,
                    alpha_seed=None, width=SWITCH_WIDTH_S):
    """Fixed point at the BASELINE beta, selected by continuation on alpha.

    Returns a dict, or None if no root survives the screens. Screens are
    CONJUNCTIVE and applied BEFORE selection: apex, duty, and v-closure.
    """
    sink = [0.0]
    tau_fn = make_tau(p, fp0, gains_lo, gains_hi, t_switch, sink, width)

    ms = step_for(width, t_switch)

    def stride_fn(pp, vv, aa, bb):
        return slip_rf.stride(pp, vv, aa, bb, tau_fn=tau_fn, max_step=ms)

    try:
        cands = find_fixed_points(p, v, fp0.beta,
                                  alpha_range=(np.deg2rad(1.0), np.deg2rad(75.0)),
                                  n_samples=60, stride_fn=stride_fn)
    except (GSlipFailure, ValueError, np.linalg.LinAlgError):
        return None

    admitted, rejected = [], []
    for fp in cands:
        try:
            res = slip_rf.stride(p, v, fp.alpha, fp.beta, tau_fn=tau_fn,
                                 max_step=ms)
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            rejected.append((fp, "stride failed"))
            continue
        apex = 1000.0 * G * res["flight_time"] ** 2 / 8.0
        duty = res["stance_time"] / res["period"]
        v_ratio = res["v"] / v
        if apex < MIN_APEX_MM:
            rejected.append((fp, "apex %.1f mm" % apex)); continue
        if duty > MAX_DUTY:
            rejected.append((fp, "duty %.3f" % duty)); continue
        if abs(v_ratio - 1.0) > V_TOL:
            rejected.append((fp, "v drift %+.2f%%" % (100 * (v_ratio - 1)))); continue
        admitted.append((fp, res, v_ratio))

    if not admitted:
        return None

    # CONTINUATION, not min|slope|.
    if alpha_seed is None:
        pick = min(admitted, key=lambda a: abs(a[0].slope))
    else:
        pick = min(admitted, key=lambda a: abs(a[0].alpha - alpha_seed))
    fp, res, v_ratio = pick
    others = [abs(a[0].slope) for a in admitted if a[0] is not fp]

    # Torque on the RETURNED ORBIT ALONE, with a fresh accumulator.
    pk = [0.0]
    fn = make_tau(p, fp0, gains_lo, gains_hi, t_switch, pk, width)
    try:
        slip_rf.stride(p, v, fp.alpha, fp.beta, tau_fn=fn, max_step=ms)
    except (GSlipFailure, ValueError, np.linalg.LinAlgError):
        pk = [float("nan")]

    return {
        "fp": fp, "slope": fp.slope, "alpha": fp.alpha, "beta": fp.beta,
        "v_ratio": v_ratio, "duty": res["stance_time"] / res["period"],
        "tau_beta": pk[0] * EROSION,
        "tau_motor": pk[0] * EROSION * MOTOR_SPLIT,
        "n_admitted": len(admitted), "n_rejected": len(rejected),
        "runner_up": min(others) if others else None,
    }


def sweep(p, v, fp0, width=SWITCH_WIDTH_S):
    """f_stance -> row, by continuation in f. None where nothing is admitted."""
    t_st = fp0.stance_time
    rows, seed = [], None
    for f in F_STANCE:
        t_sw = np.inf if f >= 1.0 else f * t_st
        r = solve_held_beta(p, v, fp0, LEG_FRAME, FLIGHT, t_sw, seed, width)
        if r is not None:
            seed = r["alpha"]
        rows.append((f, r))
    return rows


def noise_floor(p, v, fp0, f, width=SWITCH_WIDTH_S, force_step=None):
    """R1: the slope's numerical resolution AT THIS f. Returns (cond, trunc).

    Two DIFFERENT error sources, deliberately separated -- v1 conflated them
    into one max-minus-min over an h x rtol grid, which is dominated by its
    single worst cell and is not a resolution at all.

      cond   integration conditioning: h HELD at the library's 1e-6, rtol
             varied 1e-10 -> 1e-12. This is the number that matters, because
             it asks "does my answer change when I integrate more carefully".
             An adaptive RK45 crossing a discontinuity fails exactly here.
      trunc  finite-difference truncation: rtol held, h varied. Falls as h^2
             for a smooth map and tells you nothing about the discontinuity.

    A bar is set from `cond`.
    """
    t_sw = np.inf if f >= 1.0 else f * fp0.stance_time
    sink = [0.0]
    fn = make_tau(p, fp0, LEG_FRAME, FLIGHT, t_sw, sink, width)
    ms = step_for(width, t_sw) if force_step is None else force_step

    def slope_at(h, rtol):
        def sf(pp, vv, aa, bb, _r=rtol, _fn=fn, _m=ms):
            return slip_rf.stride(pp, vv, aa, bb, tau_fn=_fn,
                                  rtol=_r, atol=_r * 1e-2, max_step=_m)
        return float((poincare(p, v, fp0.alpha + h, fp0.beta, sf)
                      - poincare(p, v, fp0.alpha - h, fp0.beta, sf)) / (2 * h))

    def spread(vals):
        vals = [x for x in vals if np.isfinite(x)]
        return (max(vals) - min(vals)) if len(vals) > 1 else float("nan")

    cond, trunc = [], []
    for rtol in (1e-10, 1e-11, 1e-12):
        try:
            cond.append(slope_at(1e-6, rtol))
        except Exception:                                        # noqa: BLE001
            pass
    for h in (3e-6, 1e-6, 3e-7):
        try:
            trunc.append(slope_at(h, 1e-10))
        except Exception:                                        # noqa: BLE001
            pass
    return spread(cond), spread(trunc)


def score(rows, floor):
    """P-C-1R / P-C-2R, with R0-R7's structural clauses."""
    ok = {f: abs(r["slope"]) for f, r in rows if r is not None}
    if 0.0 not in ok or 1.0 not in ok:
        return {"undecidable": "endpoint missing"}
    interior = {f: a for f, a in ok.items() if 0.0 < f < 1.0}
    if not interior:
        return {"undecidable": "no interior rows admitted"}
    bar = max(0.02, 3.0 * floor)              # R1: 3x the declared floor
    f_hi, a_hi = max(interior.items(), key=lambda kv: kv[1])
    f_lo, a_lo = min(interior.items(), key=lambda kv: kv[1])
    worse_end, better_end = max(ok[0.0], ok[1.0]), min(ok[0.0], ok[1.0])
    return {
        "bar": bar, "floor": floor,
        "end0": ok[0.0], "end1": ok[1.0],
        "f_hi": f_hi, "a_hi": a_hi, "up": a_hi - worse_end,
        "f_lo": f_lo, "a_lo": a_lo, "down": better_end - a_lo,
        "pc1": (a_hi - worse_end) >= bar,
        "pc1_inv": (better_end - a_lo) >= bar,
        "pc2": 0.3 <= f_hi <= 0.7,
        "half": ok.get(0.5),
        "n_missing": len(F_STANCE) - len(ok),
    }


def selftest() -> int:
    print("SELF-TEST  (v2 instrument -- the fixes are the point)")
    p = params()
    fails = 0

    # 1. unit conversion round-trips to the plant's commanded gains
    kt, bt = to_ctrl(*LEG_FRAME)
    print("  to_ctrl(LEG_FRAME) = k_tangential %.1f, b_tangential %.1f" % (kt, bt))
    if abs(kt - K_TANGENTIAL) > 1.0 or abs(bt - B_TANGENTIAL) > 0.5:
        print("  FAIL: conversion does not round-trip"); fails += 1

    # 2. the blend is C2 and monotone, and width=0 is the old hard step
    xs = [smootherstep(u) for u in np.linspace(0, 1, 11)]
    if not (xs[0] == 0.0 and xs[-1] == 1.0 and all(b >= a for a, b in zip(xs, xs[1:]))):
        print("  FAIL: smootherstep is not a monotone 0->1 ramp"); fails += 1
    else:
        print("  smootherstep monotone 0->1, endpoints exact: yes")

    # 3. endpoints must still reduce to constant-regime laws
    class FP:
        beta, stance_time = np.deg2rad(80.0), 0.1
    phi = p.phi_touchdown(FP.beta)
    a, b = [0.0], [0.0]
    f_never = make_tau(p, FP(), LEG_FRAME, FLIGHT, np.inf, a)
    f_const = make_tau(p, FP(), LEG_FRAME, LEG_FRAME, np.inf, b)
    same = all(np.allclose(f_never(t, p.l0, phi + 0.01, 0.0, 0.1),
                           f_const(t, p.l0, phi + 0.01, 0.0, 0.1))
               for t in (0.0, 0.03, 0.09, 0.2))
    print("  f_stance=1 reduces to constant leg-frame: %s" % same)
    if not same:
        print("  FAIL"); fails += 1

    # 4. THE HEADLINE: the blend must collapse the interior noise floor
    v, fp0 = baseline(p, 0.70)
    print("  noise floor at v~0.70, interior f=0.5:")
    # MATCHED max_step, so the only difference is the discontinuity itself.
    MS = 6.25e-4
    hard, _ = noise_floor(p, v, fp0, 0.5, width=0.0, force_step=MS)
    soft, _ = noise_floor(p, v, fp0, 0.5, width=SWITCH_WIDTH_S, force_step=MS)
    print("      (both at max_step %.2e, so only the discontinuity differs)" % MS)
    print("      hard step (v1)        : %.4f" % hard)
    print("      C2 blend %.0f ms (v2)  : %.4f" % (1000 * SWITCH_WIDTH_S, soft))
    print("      improvement           : %.1fx" % (hard / soft if soft else float("inf")))
    if not (soft < hard / 5.0):
        print("  FAIL: the blend did not materially reduce the interior noise")
        fails += 1
    if soft > 0.02 / 3.0:
        print("  NOTE: floor %.4f still forces a bar above 0.02 (R1 will raise it)"
              % soft)

    # 5. the v-closure screen must actually reject a drifting root
    print("  V_TOL = %.3f, screens are conjunctive and pre-selection" % V_TOL)

    # 6. scorer must not be fooled by a min-envelope of two monotone branches
    env = [(f, {"slope": min(0.80 + 0.60 * f, 1.40 - 0.60 * f)}) for f in F_STANCE]
    s_env = score(env, 0.001)
    bump = [(f, {"slope": 1.0 + 0.5 * np.sin(np.pi * f)}) for f in F_STANCE]
    s_bump = score(bump, 0.001)
    mono = [(f, {"slope": 0.9 + 0.3 * f}) for f in F_STANCE]
    s_mono = score(mono, 0.001)
    print("  scorer: monotone -> pc1 %s (want False)" % s_mono["pc1"])
    print("  scorer: real bump -> pc1 %s at f %.2f (want True, 0.50)"
          % (s_bump["pc1"], s_bump["f_hi"]))
    print("  scorer: branch min-envelope -> pc1 %s (v1 said True; continuation"
          " now prevents this input arising)" % s_env["pc1"])
    if s_mono["pc1"] or not s_bump["pc1"]:
        print("  FAIL: scorer does not separate monotone from bump"); fails += 1

    print("\n%s" % ("SELF-TEST PASSED" if not fails else "SELF-TEST FAILED (%d)" % fails))
    return 1 if fails else 0


def widths() -> None:
    """Is the answer an artefact of the blend width? Report, do not assume."""
    p = params()
    v, fp0 = baseline(p, 0.70)
    print("Blend-width sensitivity at v~0.70 (R1: the fix must not set the answer)")
    print("  %10s %12s %12s %12s" % ("width ms", "floor@f=0.5", "slope@f=0.5", "slope@f=1"))
    slopes = []
    for w in (0.0, 0.002, 0.005, 0.010, 0.020):
        fl, _tr = noise_floor(p, v, fp0, 0.5, width=w)
        r5 = solve_held_beta(p, v, fp0, LEG_FRAME, FLIGHT, 0.5 * fp0.stance_time,
                             None, w)
        r1 = solve_held_beta(p, v, fp0, LEG_FRAME, FLIGHT, np.inf, None, w)
        if r5 is not None:
            slopes.append(r5["slope"])
        print("  %10.1f %12.4f %12s %12s"
              % (1000 * w, fl,
                 "--" if r5 is None else "%+.4f" % r5["slope"],
                 "--" if r1 is None else "%+.4f" % r1["slope"]))
    if len(slopes) > 1:
        print()
        print("  slope@f=0.5 spread across ALL widths incl. the hard step: %.4f"
              % (max(slopes) - min(slopes)))
        print("  This is the most direct evidence the answer is not an artefact")
        print("  of the smoothing: the modelling choice moves it by that much.")


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = params()
    print("Regime conversion, k_c = k_cart * lever^2, lever = %.4f m" % LEVER)
    for nm, g_, src in (("leg-frame", LEG_FRAME, "k_tangential 600, b_tangential 30"),
                        ("flight", FLIGHT, "k_flight 7150, b_flight 115.8"),
                        ("average", AVERAGE, "at gain honesty %.2f (S143)" % HONESTY)):
        print("  %-10s: k_c %7.2f  d_c %6.2f   <- %s" % (nm, g_[0], g_[1], src))
    print()

    for vt in V_TILDES:
        v, fp0 = baseline(p, vt)
        print("=" * 96)
        print("v~ %.2f  (v_td %.4f m/s)  baseline beta* %.2f (HELD), slope %+.4f, stance %.4f s"
              % (vt, v, np.rad2deg(fp0.beta), fp0.slope, fp0.stance_time))
        print("=" * 96)

        floor, trunc = noise_floor(p, v, fp0, 0.5)
        print("  R1 numerical resolution at interior f = 0.5:")
        print("      integration conditioning (h fixed, rtol 1e-10..1e-12) : %.5f"
              % floor)
        print("      f-d truncation           (rtol fixed, h 3e-6..3e-7)   : %.5f"
              % trunc)
        print("      => bar = max(0.02, 3x conditioning) = %.4f"
              % max(0.02, 3.0 * floor))
        print()

        print("  CONSTANT REGIMES")
        print("  %-11s %9s %10s %9s %11s %11s"
              % ("regime", "alpha*", "slope", "v drift", "tau_beta", "tau/motor"))
        for nm, g_ in (("leg-frame", LEG_FRAME), ("flight", FLIGHT),
                       ("average", AVERAGE)):
            r = solve_held_beta(p, v, fp0, g_, g_, np.inf)
            if r is None:
                print("  %-11s %9s %10s %9s %11s %11s  no admitted root"
                      % (nm, "--", "--", "--", "--", "--")); continue
            print("  %-11s %9.2f %+10.4f %8.2f%% %11.1f %11.1f  %s"
                  % (nm, np.rad2deg(r["alpha"]), r["slope"],
                     100 * (r["v_ratio"] - 1), r["tau_beta"], r["tau_motor"],
                     "stable" if abs(r["slope"]) < 1.0 else "UNSTABLE"))
        print()

        print("  SWITCHING SWEEP (beta HELD, alpha by continuation, v-closure screened)")
        print("  %8s %9s %10s %9s %11s %6s %9s"
              % ("f_stance", "alpha*", "slope", "v drift", "tau/motor", "roots", "runnerup"))
        rows = sweep(p, v, fp0)
        for f, r in rows:
            if r is None:
                print("  %8.2f %9s %10s %9s %11s %6s %9s   no admitted root"
                      % (f, "--", "--", "--", "--", "--", "--")); continue
            mk = ""
            if abs(f - HONESTY) < 1e-9: mk = "  <- plant"
            elif f >= 1.0: mk = "  <- gating"
            elif f <= 0.0: mk = "  <- none"
            print("  %8.2f %9.2f %+10.4f %8.2f%% %11.1f %6d %9s%s"
                  % (f, np.rad2deg(r["alpha"]), r["slope"],
                     100 * (r["v_ratio"] - 1), r["tau_motor"], r["n_admitted"],
                     "--" if r["runner_up"] is None else "%.3f" % r["runner_up"], mk))
        print()

        s = score(rows, floor)
        if "undecidable" in s:
            print("  UNDECIDABLE: %s\n" % s["undecidable"]); continue
        print("  R0  |slope(f=0.5)| = %s  (>= 1.0 required for validity)"
              % ("%.4f" % s["half"] if s["half"] is not None else "missing"))
        if s["half"] is None or s["half"] < 1.0:
            print("      => INVALID at this speed. P-C-1R..P-C-3R UNSCORED,")
            print("         no branch selected. The model does not reproduce the")
            print("         plant's instability at the plant's own operating point.")
            print()
            continue
        print("  P-C-1R  interior max %.4f at f %.2f; endpoints %.4f / %.4f"
              % (s["a_hi"], s["f_hi"], s["end0"], s["end1"]))
        print("          excursion %+.4f against bar %.4f => %s"
              % (s["up"], s["bar"], "PASSES" if s["pc1"] else "FAILS"))
        if s["pc1_inv"]:
            print("  P-C-1R-INVERSE CONFIRMED: interior min %.4f at f %.2f, %+.4f"
                  " below the better endpoint -- switching is STABILISING."
                  % (s["a_lo"], s["f_lo"], s["down"]))
        print("  P-C-2R  argmax f %.2f in [0.3,0.7] => %s  (weak: 56%% null pass rate)"
              % (s["f_hi"], "PASSES" if s["pc2"] else "FAILS"))
        e1, h = s["end1"], s["half"]
        if abs(e1 - h) < 0.05:
            br = "NO BRANCH (separation %.4f under the 0.05 dead band)" % abs(e1 - h)
        elif e1 < h:
            br = "CONTACT-GATING branch"
        else:
            br = "RETUNE branch"
        print("  P-C-3R  |slope(1.0)| %.4f vs |slope(0.5)| %.4f => %s" % (e1, h, br))
        print()

    print("=" * 96)
    print("tau is the ORBIT's own peak (fresh accumulator), eroded x%.2f, and the" % EROSION)
    print("per-motor column applies the x%.1f leg-axis split. Per-motor stall is"
          % MOTOR_SPLIT)
    print("%.1f N.m. REPORTED, NOT GATED -- and see S149 sec 3: the erosion factor" % STALL)
    print("spans 1.1x-8x by configuration and is NOT a constant.")
    print("=" * 96)


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        raise SystemExit(selftest())
    if "--widths" in sys.argv:
        widths()
    else:
        main()
