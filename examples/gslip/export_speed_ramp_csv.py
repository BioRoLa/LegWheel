"""Export a speed-ramp template: hop in place, then accelerate to the pronk.

Solves the standing-start problem. The pronk template is the fixed point at
v~ = 1.2, i.e. 2.035 m/s of steady forward travel, but the robot is triggered
from rest. In Webots that mismatch showed up as the leg entering stance with far
more energy than the fixed point assumes: theta over-swept its command by
2.1-2.8x, all four legs saturated at the 35 N.m limit mid-stance, and flight
came out at 11-42% against 57% designed.

Softening k_radial does not fix it -- tested at 4900 and saturation got WORSE
(20 -> 37% on leg A), because the leg simply travelled further. The entry
condition is wrong, not the gain.

So ramp instead: start from the in-place hop, which is validated from standstill
(56.9% flight against 57% predicted at b_radial = 72), then walk up a ladder of
fixed points to the target speed. Each step is small enough that the robot is
never far from the fixed point it is currently tracking.

Two things this has to get right:

1. **Reject grazing solutions.** The stability sweep found apparently-stable
   fixed points at low speed that are really near-grazing gaits -- apex
   clearance 0.24-3.2 mm, duty 0.67-0.88, touchdown velocity almost horizontal.
   They are "stabilised" by the flight phase vanishing, and on a robot with a
   145 mm foot radius a 3 mm hop is indistinguishable from continuous contact.
   MIN_APEX_MM and MAX_DUTY filter them out.

2. **Keep the joins continuous.** Consecutive segments have different beta and
   different periods, so the concatenation is checked for jumps in theta and
   beta at every boundary and the worst is reported. A large jump would be a
   step command into an impedance controller, which is exactly the touchdown
   transient this is meant to avoid.

Run:
    uv run python examples/gslip/export_speed_ramp_csv.py
"""

import numpy as np

from legwheel.config import OUTPUT_CSV_DIR, RobotParams
from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
N_LEGS = 4
MOTOR_TORQUE_LIMIT = 35.0
NOMINAL_THETA_DEG = 100.0

# Hop segment: the validated standing start.
HOP_APEX_MM = 30.0

# Speed ladder, in v~. The gap from the hop (effectively v~ = 0) to the first
# forward step is the largest single jump in the ramp, so keep it modest.
V_TILDE_LADDER = (0.3, 0.5, 0.7, 0.9, 1.05, 1.2)

# Strides held at each rung. More strides = gentler acceleration but a longer
# CSV; the controller replays one row per 1 kHz tick.
STRIDES_PER_STEP = 6
HOP_STRIDES = 8

# Grazing filters -- see the module docstring.
MIN_APEX_MM = 10.0
MAX_DUTY = 0.55


def _params(leg_map):
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    return slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + leg_map.leg.foot_radius,
        k=K_REL * MASS * G / hip_to_arc, r=leg_map.leg.foot_radius,
    ), hip_to_arc


def _apex_mm(res):
    """Flight apex above touchdown height, from the ballistic flight time.

    Derived from flight_time rather than a liftoff velocity: an earlier version
    read res["liftoff_vz"], a key slip_rf.stride does not return, so .get()
    silently defaulted to 0.0 and the grazing filter rejected EVERY candidate at
    every speed -- including v~ = 1.2, which is the fixed point the shipped
    pronk template is built from. A filter that rejects everything looks exactly
    like "no fixed point exists".

    Ballistic: apex = g * (t_flight / 2)^2 / 2 = g * t_flight^2 / 8.
    """
    t_f = float(res["flight_time"])
    return 1000.0 * G * t_f * t_f / 8.0


def _best_fixed_point(p, v, beta_step=1.0):
    """Best-conditioned NON-GRAZING fixed point at this speed.

    Coarse beta step by default. The single-speed pronk exporter can afford
    0.25 deg over a 10 deg window; this sweeps a wider window at six speeds,
    which at that resolution is ~20x the integration work and takes long enough
    to be unusable. The stability sweep already established that refining beta
    20x moves the map slope by 2e-4, so the resolution buys nothing here.
    """
    best = None
    for beta_deg in np.arange(60.0, 86.01, beta_step):
        for fp in find_fixed_points(
            p, v, np.deg2rad(beta_deg),
            alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
            n_samples=20, stride_fn=slip_rf.stride,
        ):
            if fp.duty_factor > MAX_DUTY:
                continue
            res = slip_rf.stride(p, v, fp.alpha, fp.beta)
            if _apex_mm(res) < MIN_APEX_MM:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return best


def _segment(p, v, alpha, beta, leg_map):
    template = tpl.build_template(p, v, alpha, beta)
    n = g2c.samples_for_rate(template.period, rate_hz=1000.0)
    return g2c.map_template(template, n=n, leg_map=leg_map)


def main() -> None:
    leg_map = g2c.LegLengthMap()
    p, hip_to_arc = _params(leg_map)

    print()
    print("=" * 74)
    print(f"SPEED RAMP   theta_nom = {NOMINAL_THETA_DEG} deg, k_rel = {K_REL}")
    print("=" * 74)
    print(f"  standing hip height {p.l0:.4f} m,  k = {p.k:.0f} N/m "
          f"({p.k/N_LEGS:.0f} per leg)")

    segments = []   # (label, v_mps, traj, n_strides)

    # --- segment 0: the hop, which works from standstill -------------------
    v_hop = float(np.sqrt(2 * G * HOP_APEX_MM / 1000.0))
    hop = _segment(p, v_hop, np.deg2rad(90.0), np.deg2rad(90.0), leg_map)
    segments.append((f"hop {HOP_APEX_MM:.0f} mm", 0.0, hop, HOP_STRIDES))

    # --- forward rungs ------------------------------------------------------
    for vt in V_TILDE_LADDER:
        v = vt * np.sqrt(G * p.l0)
        fp = _best_fixed_point(p, v)
        if fp is None:
            print(f"  ! no non-grazing fixed point at v~ = {vt}; skipping")
            continue
        traj = _segment(p, v, fp.alpha, fp.beta, leg_map)
        segments.append((f"v~ {vt:.2f}", v, traj, STRIDES_PER_STEP))

    print()
    print(f"{'segment':>12} {'v (m/s)':>8} {'beta':>7} {'duty':>6} "
          f"{'period':>8} {'strides':>8} {'theta range':>16}")
    for label, v, traj, ns in segments:
        r = traj.guard_report()
        print(f"{label:>12} {v:8.3f} {np.rad2deg(traj.beta).max():7.2f} "
              f"{traj.in_stance.mean():6.3f} {traj.period:8.4f} {ns:8d} "
              f"{r['theta_min_deg']:7.2f}-{r['theta_max_deg']:.2f}")
        traj.assert_feasible()

    # --- concatenate --------------------------------------------------------
    # Drop each stride's wrap-around frame except at the very end, exactly as
    # to_csv does, so a stride boundary does not read as a one-tick stall.
    t, th, be, ga, st = [], [], [], [], []
    t_now = 0.0
    joins = []
    for si, (label, _v, traj, ns) in enumerate(segments):
        n = len(traj.t)
        dt = traj.period / (n - 1)
        for k in range(ns):
            last_overall = (si == len(segments) - 1) and (k == ns - 1)
            stop = n if last_overall else n - 1
            if th:
                joins.append((label if k == 0 else None,
                              abs(np.rad2deg(traj.theta[0] - th[-1])),
                              abs(np.rad2deg(traj.beta[0] - be[-1]))))
            for i in range(stop):
                t.append(t_now + i * dt)
                th.append(traj.theta[i])
                be.append(traj.beta[i])
                ga.append(traj.gamma[i])
                st.append(bool(traj.in_stance[i]))
            t_now += stop * dt

    th, be = np.array(th), np.array(be)
    print()
    print("=" * 74)
    print("CONCATENATION")
    print("=" * 74)
    print(f"  rows {len(th)}  ({t_now:.2f} s of gait at 1 kHz)")

    seg_joins = [(lbl, dth, dbe) for lbl, dth, dbe in joins if lbl]
    worst_th = max(j[1] for j in joins)
    worst_be = max(j[2] for j in joins)
    print(f"  worst jump at ANY stride boundary: "
          f"theta {worst_th:.3f} deg, beta {worst_be:.3f} deg")
    print("  segment transitions:")
    for lbl, dth, dbe in seg_joins:
        flag = "  <-- large" if (dth > 3.0 or dbe > 5.0) else ""
        print(f"    into {lbl:>10}: dtheta {dth:6.3f} deg, dbeta {dbe:6.3f} deg{flag}")

    # Whole-ramp guards, not just per segment.
    print()
    print(f"  theta  {np.rad2deg(th).min():.2f} to {np.rad2deg(th).max():.2f} deg"
          f"   (limits {RobotParams.MIN_THETA_DEG}-{RobotParams.MAX_THETA_DEG})")
    print(f"  |beta| peaks at {np.abs(np.rad2deg(be)).max():.2f} deg"
          f"   (limit {RobotParams.BETA_MAX_DEG})")
    ok = (np.rad2deg(th).min() >= RobotParams.MIN_THETA_DEG
          and np.rad2deg(th).max() <= RobotParams.MAX_THETA_DEG
          and np.abs(np.rad2deg(be)).max() <= RobotParams.BETA_MAX_DEG)
    print(f"  workspace: {'OK' if ok else 'VIOLATION'}")

    # --- write --------------------------------------------------------------
    import csv

    out = OUTPUT_CSV_DIR / "gslip_speed_ramp_template.csv"
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "theta", "beta", "gamma", "in_stance"])
        for i in range(len(th)):
            w.writerow([f"{t[i]:.6f}", f"{th[i]:.6f}", f"{be[i]:.6f}",
                        f"{ga[i]:.6f}", int(st[i])])
    print()
    print(f"  wrote {out}")
    print(f"    {len(th)} rows, {int(np.sum(st))} stance / "
          f"{int(len(st) - np.sum(st))} flight")
    print()
    print("  the controller replays this one row per tick, so the ramp is")
    print("  open-loop in speed: it does not measure whether the robot actually")
    print("  reached each rung. If a rung is missed the next one is a step, so")
    print("  check the measured flight fraction per segment before trusting it.")
    print()


if __name__ == "__main__":
    main()
