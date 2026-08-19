"""Stage 2a turning envelope -- the headline overlay, in model space.

Feasible steady coordinated (banked) turns for the cambered SLIP-RF template,
as a (v, lambda) constraint grid mapped into the (v, R) plane, overlaid on the
differential-steering evidence. NO new dynamics and NO yaw model: turning is
imposed by the algebraic coordinated-turn balance tan(lambda) = v^2/(g R)
(slip_rf_cambered.turn_radius, the seam), in-plane existence comes from the
existing reduction + 1-D Poincare machinery unchanged, and everything else is
a per-cell constraint check.

SPEED AXES -- the trap section 27 already documented: the solver's v is the
TOUCHDOWN speed; the measured 0.70-0.85 m/s band, the scrub bound, and the
coordinated-turn balance all live on MEAN FORWARD speed (stride/period).
v~0.70 is touchdown 1.19 m/s but forward 0.726. The grid therefore sweeps
touchdown speed (what the Poincare machinery needs), and every turning
quantity -- psi_dot, R, scrub -- is evaluated at THAT CELL'S OWN
fp.mean_speed. Cells with no fixed point get no turning numbers (they are
existence-blocked; nothing is fabricated for them).

CONSTRAINTS PER (v, lambda) CELL
    C1 existence   non-grazing cambered fixed point (duty <= 0.55, apex >=
                   10 mm -- pronk_operating_point's filters, same solver)
    C2 scrub       psi_dot = g*tan(lambda)/v <= 0.29 rad/s, the measured hard
                   geometric bound (Stage 3 Phase 5; scrub 13.8-17.2% of
                   rolling distance at the boundary). Geometric, not tunable.
    C3 leg torque  motor_torque_for(peak_grf/4, theta_min) * 2.33 erosion
                   <= 35 N.m (pronk_operating_point's exact chain)
    C4 ABAD hold   quasi-static lever estimate f_leg * (d_plane*cos(lambda)
                   + r_eff*sin(lambda)) <= 40 N.m. An ESTIMATE, not dynamics;
                   section 57's 31.2 N.m stabilization demand is annotated as
                   reserve on the same joint, never summed with this.
    C5 slew        NOT a gate: a steady turn holds camber, so the 2.56 rad/s
                   ABAD speed only bounds one-flight reachability (entry) and
                   the per-stride Ackermann re-trim. Reported, drawn dashed.

TWO RADIUS LAWS, BOTH RUN
    geometric   r_eff(lambda) = R_tread*cos - w_flat*sin + r_corner (the
                reduction's own law; exact 0.145 at lambda = 0)
    empirical   r = WHEEL_ROLL_RADIUS_SIM = 0.14482, lambda-independent
                (Stage 1.5 measured the sim's forward channel; knobby tread)
    The empirical law is primary for sim-facing claims (log section 75); the
    geometric curve is the dashed sensitivity. E3 predicts they differ by
    < 2% of envelope area, in which case the template-seam question is
    benign for this figure.

REGISTERED PREDICTIONS (written before the first full grid; log section 83)
    E1  Every scrub-feasible cell needs only tiny coordinated bank:
        tan(lambda) <= 0.29*v/g -> lambda <~ 1.5 deg at v <= 0.85. lambda >=
        10 deg maps to R < 0.6 m, an order of magnitude inside the scrub
        bound. The headline figure therefore shows the large-camber regime
        is NOT a banking regime -- camber's job at feasible radii is contact
        and roll authority, not centripetal balance.
    E2  The binding constraint over the measured band (0.70-0.85 m/s) is
        scrub, not torque.
    E3  Radius-law sensitivity < 2% of envelope area.
    E4  Entry slew never binds a steady cell (lambda_in/2.56 << t_flight).

WHAT THIS DOES NOT CLAIM
    - Turning MECHANISM: uniform camber steers ~0.02% of the geometric turn
      (section 33, spectral sigma 0.0065 vs 0.16 differential, section 45;
      Open Issue #10). Until camber-in-pronk curves a simulated path, this
      envelope is a model result, not a demonstrated mechanism.
    - Yaw dynamics: parked as Stage 3 territory. The return map expresses
      turning only as lateral drift; this script never integrates yaw.
    - Friction: no tyre model. The scrub bound is imported from wheeled-mode
      measurement via the section 22 geometry argument.
    - The ABAD check is a static lever estimate, labelled as such.

Run (no args = selftest + quick grid; --full for the paper grid):
    uv run python examples/gslip/stage2a_turning_envelope.py [--full]
        [--cache <out.npz>] [--from-cache <in.npz>] [--no-plots]
"""
from __future__ import annotations

import sys
from functools import partial
from pathlib import Path

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.slip_rf import SlipRfParams
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.slip_rf_cambered import (
    cambered_params, cambered_stride, rolling_radius, turn_radius,
)
from legwheel.models.cambered_return_map import CONTACT_TRACK, ackermann_pair
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c
from legwheel.config import RobotParams

# --- constants, every one with a named source --------------------------------
MASS, G = 30.0, 9.81                       # pronk_operating_point.py
K_REL = 18.0                               # pronk_operating_point.py
NOMINAL_THETA_DEG = 100.0                  # pronk_operating_point.py
MOTOR_TORQUE_LIMIT = 35.0                  # pronk_operating_point.py
TORQUE_EROSION = 35.0 / 15.02              # measured; trot_fixed_point.py
MIN_APEX_MM = 10.0                         # grazing filter
MAX_DUTY = 0.55                            # grazing filter
MEASURED_V = (0.70, 0.85)                  # measured operating band
V_OPERATING = 0.726                        # v~0.70, section 27
V_DESIGN = 2.035                           # v~1.20 design point (annotation)

PSI_DOT_MAX = 0.29                         # rad/s, hard bound (Phase 5, OI#6)
SCRUB_BAND = (0.138, 0.172)                # envelope-crossing band (Phase 5)
WHEEL_BASE = RobotParams.WHEEL_BASE        # 0.510
ABAD_TAU_MAX = 40.0                        # N.m practical (RollPD.tau_max)
ABAD_RESERVE = 31.2                        # N.m, section 57 winner NEAR peak
ABAD_JOINT_SPEED = 2.56                    # rad/s (stage2b_budget_gain_sweep)
D_PLANE = 0.091675                         # ABAD axis -> wheel plane (stage15)
R_EMPIRICAL = RobotParams.WHEEL_ROLL_RADIUS_SIM   # 0.14482, lambda-indep

# differential-steering overlay evidence
SLTL_CURVATURE = (0.28, 0.30)              # 1/m, lab SLTL (log section 38/40)
SLTL_NOTE = "SLTL achieves ~27% of predicted curvature"
WEBOTS_R = (1.35, 2.69)                    # m, own turning runs, v ~ 0.53
WEBOTS_V = 0.53

# grid: two-scale lambda so the scrub-feasible sliver (lambda <~ 2 deg at
# these speeds) is resolved AND the cambered family is solved at large lean.
# The v grid is TOUCHDOWN speed; the model's forward speeds come out ~0.73x
# of it, so 0.45-1.45 covers model-forward ~0.33-1.06, spanning the measured
# 0.70-0.85 band (which the ROBOT reaches at a template the MODEL runs
# faster -- the section-27 speed shortfall; the two axes are never equated).
LAM_GRID_DEG = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5,
                         5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0])
V_GRID_FULL = np.arange(0.45, 1.45 + 1e-9, 0.05)
V_GRID_QUICK = np.arange(0.45, 1.45 + 1e-9, 0.25)
TABLE_V_FWD = (0.60, V_OPERATING, 0.85)   # forward-speed table targets

FIG_DIR = Path(__file__).resolve().parent / "stage2a_figs"


# --- small algebra, kept as named functions so tests can pin them ------------

def psi_dot(v: float, lam: float) -> float:
    """Yaw rate implied by a coordinated turn: v/R = g*tan(lambda)/v."""
    return G * np.tan(lam) / v


def scrub_fraction(v: float, psi: float) -> float:
    """Lateral scrub per rolling distance at the worst foot,
    |psi_dot|*(wheelbase/2)/v -- turn_scrub_geometry's arithmetic."""
    return abs(psi) * (WHEEL_BASE / 2.0) / v


def ackermann_split(lam_in: float, h: float) -> tuple[float, float]:
    """(lam_in, lam_out) via the apex condition; must equal ackermann_pair."""
    if abs(lam_in) < 1e-12:
        return 0.0, 0.0
    cot_out = 1.0 / np.tan(lam_in) + CONTACT_TRACK / h
    return float(lam_in), float(np.arctan(1.0 / cot_out))


def cambered_params_const_r(p: SlipRfParams, lam: float,
                            r_const: float) -> SlipRfParams:
    """cambered_params with the radius held at a constant (the empirical
    lambda-independent law). g -> g/cos(lambda) is unchanged; feeding
    r_const = rolling_radius(lam) reproduces cambered_params exactly."""
    hip = p.l0 - p.r
    return SlipRfParams(m=p.m, l0=hip + r_const, k=p.k, r=r_const,
                        g=p.g / np.cos(lam))


def stride_const_r(p: SlipRfParams, v, alpha, beta, lam=0.0, r_const=R_EMPIRICAL):
    out = slip_rf.stride(cambered_params_const_r(p, lam, r_const), v, alpha, beta)
    out["lam"] = float(lam)
    return out


def base_params() -> SlipRfParams:
    """Exactly pronk_operating_point.main()'s parameter build."""
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = SlipRfParams(m=MASS, l0=hip_to_arc + foot_radius,
                     k=K_REL * MASS * G / hip_to_arc, r=foot_radius)
    return p


def apex_mm(res: dict) -> float:
    t_f = float(res["flight_time"])
    return 1000.0 * G * t_f * t_f / 8.0


def solve_existence(p: SlipRfParams, v: float, stride_fn, step: float = 0.5,
                    beta_center: float | None = None):
    """pronk_operating_point.solve, generalised over stride_fn, with an
    optional warm-start window (fixed points move slowly in lambda)."""
    if beta_center is None:
        betas = np.arange(60.0, 86.0 + 1e-9, step)
    else:
        betas = np.arange(max(60.0, beta_center - 3.0),
                          min(86.0, beta_center + 3.0) + 1e-9, step)
    best = None
    for beta_deg in betas:
        for fp in find_fixed_points(
            p, v, np.deg2rad(beta_deg),
            alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
            n_samples=20, stride_fn=stride_fn,
        ):
            if fp.duty_factor > MAX_DUTY:
                continue
            if apex_mm(stride_fn(p, v, fp.alpha, fp.beta)) < MIN_APEX_MM:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    if best is None and beta_center is not None:
        return solve_existence(p, v, stride_fn, step, beta_center=None)
    return best


def leg_torque(p_cell: SlipRfParams, v: float, fp, leg_map) -> float:
    """pronk_operating_point's exact torque chain, erosion applied."""
    res = slip_rf.stride(p_cell, v, fp.alpha, fp.beta)
    template = tpl.build_template(p_cell, v, fp.alpha, fp.beta)
    traj = g2c.map_template(
        template, n=g2c.samples_for_rate(template.period, rate_hz=1000.0))
    f_leg = res["peak_grf_mag"] / 4.0
    tau = g2c.motor_torque_for(
        f_leg, float(traj.theta[int(np.argmin(traj.theta))]), leg_map)
    return float(tau * TORQUE_EROSION), float(f_leg)


def abad_hold_torque(f_leg: float, lam: float, r_eff: float) -> float:
    """Quasi-static ABAD moment to hold the leg plane at `lam` under the
    peak leg force: lever = d_plane*cos + r_eff*sin. An estimate."""
    return float(f_leg * (D_PLANE * np.cos(lam) + r_eff * np.sin(lam)))


# --- selftest (refusal-gated; hand answers, not round-trips) ------------------

def _selftest() -> None:
    # coordinated-turn radius reproduces the documented feasibility table
    # (Cambered Contact Geometry, steady-turn feasibility, v = 1.0 m/s)
    for lam_deg, r_doc in ((10.0, 0.58), (15.0, 0.38), (20.0, 0.28)):
        assert abs(turn_radius(1.0, np.deg2rad(lam_deg)) - r_doc) < 0.01, \
            (lam_deg, turn_radius(1.0, np.deg2rad(lam_deg)))
    # scrub arithmetic reproduces the measured Phase 5 boundary row:
    # psi_dot 0.288 rad/s at v = 0.534 -> 13.8% of rolling distance
    assert abs(scrub_fraction(0.534, 0.288) - 0.1375) < 0.002
    # ackermann_split must agree with the return map's ackermann_pair
    for lam_deg in (5.0, 10.0, 15.0):
        a = ackermann_split(np.deg2rad(lam_deg), 0.30)
        b = ackermann_pair(np.deg2rad(lam_deg), 0.30)
        assert abs(a[1] - b[1]) < 1e-12, (a, b)
    # const-r params reproduce cambered_params when fed the geometric law
    p = base_params()
    lam = np.deg2rad(12.0)
    a, b = cambered_params_const_r(p, lam, rolling_radius(lam)), \
        cambered_params(p, lam)
    for f in ("m", "l0", "k", "r", "g"):
        assert abs(getattr(a, f) - getattr(b, f)) < 1e-15, f
    # REFUSAL: the lambda = 0 fixed point at the operating point (v~0.70,
    # touchdown 0.70*sqrt(g*l0)) must exist and land near the vault's
    # beta* = 80.75 deg; if not, the solver is broken and no envelope may
    # be drawn.
    v_td = 0.70 * np.sqrt(G * p.l0)
    fp = solve_existence(p, v_td, slip_rf.stride, step=1.0)
    assert fp is not None, "lambda=0 existence at v~0.70 failed -- REFUSING"
    assert 79.0 <= np.rad2deg(fp.beta) <= 83.0, np.rad2deg(fp.beta)
    # NOTE the model's v~0.70 fixed point runs ~0.87 m/s forward while the
    # ROBOT measures 0.726 at the same template -- the section-27/29 speed
    # shortfall. The model curve and the measured band are different
    # objects; the figure shows both and never equates them.
    assert 0.7 < fp.mean_speed < 1.0, fp.mean_speed


# --- grid ---------------------------------------------------------------------

def run_grid(v_grid, lam_grid_deg, step, laws=("empirical", "geometric"),
             verbose=True):
    p = base_params()
    leg_map = g2c.LegLengthMap()
    cells = []
    for law in laws:
        beta_warm = {}
        for v in v_grid:
            for lam_deg in lam_grid_deg:
                lam = np.deg2rad(lam_deg)
                if law == "geometric":
                    stride_fn = partial(cambered_stride, lam=lam)
                    p_cell = cambered_params(p, lam)
                    r_eff = rolling_radius(lam)
                else:
                    stride_fn = partial(stride_const_r, lam=lam,
                                        r_const=R_EMPIRICAL)
                    p_cell = cambered_params_const_r(p, lam, R_EMPIRICAL)
                    r_eff = R_EMPIRICAL
                fp = solve_existence(p, v, stride_fn, step,
                                     beta_center=beta_warm.get(v))
                cell = {"law": law, "v_td": v, "lam_deg": lam_deg,
                        "exists": fp is not None}
                if fp is not None:
                    beta_warm[v] = np.rad2deg(fp.beta)
                    v_fwd = fp.mean_speed      # turning lives on FORWARD speed
                    tau, f_leg = leg_torque(p_cell, v, fp, leg_map)
                    h_td = p_cell.l0 * np.sin(fp.beta)
                    lam_in, lam_out = ackermann_split(lam, h_td)
                    cell.update({
                        "v_fwd": v_fwd,
                        "psi": psi_dot(v_fwd, lam),
                        "scrub": scrub_fraction(v_fwd, psi_dot(v_fwd, lam)),
                        "R": turn_radius(v_fwd, lam),
                        "beta_deg": np.rad2deg(fp.beta),
                        "alpha_deg": np.rad2deg(fp.alpha),
                        "slope": fp.slope, "duty": fp.duty_factor,
                        "flight_s": fp.flight_time,
                        "tau_leg": tau,
                        "tau_abad": abad_hold_torque(f_leg, lam, r_eff),
                        "lam_out_deg": np.rad2deg(lam_out),
                        "entry_slew_ms": 1e3 * lam / ABAD_JOINT_SPEED,
                        "retrim_ms": 1e3 * (lam - lam_out) / ABAD_JOINT_SPEED,
                    })
                    c1 = True
                    c2 = cell["psi"] <= PSI_DOT_MAX
                    c3 = tau <= MOTOR_TORQUE_LIMIT
                    c4 = cell["tau_abad"] <= ABAD_TAU_MAX
                    cell["feasible"] = c1 and c2 and c3 and c4
                    cell["binding"] = ("none" if cell["feasible"] else
                                       "scrub" if not c2 else
                                       "leg-torque" if not c3 else "abad")
                else:
                    cell.update({"v_fwd": None, "psi": None, "scrub": None,
                                 "R": None, "feasible": False,
                                 "binding": "existence"})
                cells.append(cell)
                if verbose:
                    psi_txt = (f"psi {cell['psi']:5.2f} scrub "
                               f"{cell['scrub']:5.1%}" if fp else
                               "psi   --  scrub    --")
                    print(f"  {law[:3]} v_td {v:4.2f} lam {lam_deg:5.1f}d "
                          f"{psi_txt} {'FP' if cell['exists'] else '--'} "
                          f"{'FEASIBLE' if cell['feasible'] else cell['binding']}")
    return cells


def _nearest_vtd(cells, law, v_fwd_target):
    """The touchdown-speed row whose lambda=0 forward speed is nearest the
    target forward speed (or None if that law found no fixed points)."""
    straight = [c for c in cells if c["law"] == law and c["lam_deg"] == 0.0
                and c["exists"]]
    if not straight:
        return None
    return min(straight, key=lambda c: abs(c["v_fwd"] - v_fwd_target))["v_td"]


def print_table(cells):
    print(f"\n{'v_td':>5} {'v_fwd':>6} {'law':>4} | feasible lam | "
          f"R range [m] | binding | beta*/alpha* (at max-lam FP) | duty | "
          f"tau_leg | tau_abad | lam in/out | slew entry/retrim ms "
          f"(flight ms)")
    for v_fwd_target in TABLE_V_FWD:
        for law in ("empirical", "geometric"):
            v_td = _nearest_vtd(cells, law, v_fwd_target)
            if v_td is None:
                continue
            sub = [c for c in cells if c["law"] == law and c["v_td"] == v_td]
            feas = [c for c in sub if c["feasible"]]
            lam_max = max((c["lam_deg"] for c in feas), default=None)
            ex = [c for c in sub if c["exists"]]
            big = max(ex, key=lambda c: c["lam_deg"], default=None)
            if lam_max is not None and lam_max > 0:
                r_min = min(c["R"] for c in feas if c["lam_deg"] > 0)
                r_txt = f"{r_min:6.2f}..inf"
            else:
                r_txt = "straight only" if feas else "none"
            first_block = min((c for c in sub if not c["feasible"]),
                              key=lambda c: c["lam_deg"], default=None)
            v_fwd0 = next(c["v_fwd"] for c in sub if c["lam_deg"] == 0.0)
            print(f"{v_td:5.2f} {v_fwd0:6.3f} {law[:4]:>4} | lam <= "
                  f"{lam_max if lam_max is not None else float('nan'):4.1f}d "
                  f"| {r_txt:>13} | "
                  f"{first_block['binding'] if first_block else '--':>9}",
                  end="")
            if big:
                print(f" | {big['beta_deg']:5.2f}/{big['alpha_deg']:5.2f} "
                      f"(lam {big['lam_deg']:.0f}d) | {big['duty']:.3f} | "
                      f"{big['tau_leg']:5.1f} | {big['tau_abad']:5.1f} | "
                      f"{big['lam_deg']:.1f}/{big['lam_out_deg']:.1f} | "
                      f"{big['entry_slew_ms']:.0f}/{big['retrim_ms']:.0f} "
                      f"({1e3 * big['flight_s']:.0f})")
            else:
                print(" | no fixed point at any lambda")


def make_figures(cells):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    FIG_DIR.mkdir(exist_ok=True)
    # ---- headline: (v, R) overlay -------------------------------------------
    fig, ax = plt.subplots(figsize=(8.5, 6))
    v_line = np.linspace(0.45, 1.25, 200)
    # scrub bound: R >= v / PSI_DOT_MAX (since psi = v/R)
    ax.plot(v_line, v_line / PSI_DOT_MAX, "k-", lw=2,
            label=f"scrub bound R = v/{PSI_DOT_MAX} (hard, geometric)")
    band_lo = v_line / (PSI_DOT_MAX * SCRUB_BAND[1] / SCRUB_BAND[0])
    ax.fill_between(v_line, band_lo, v_line / PSI_DOT_MAX, color="k",
                    alpha=0.08, label="measured envelope-crossing band")
    for law, style in (("empirical", "-"), ("geometric", "--")):
        vs, rs = [], []
        for v_td in sorted({c["v_td"] for c in cells if c["law"] == law}):
            feas = [c for c in cells if c["law"] == law and c["v_td"] == v_td
                    and c["feasible"] and c["lam_deg"] > 0]
            if feas:
                vs.append(float(np.mean([c["v_fwd"] for c in feas])))
                rs.append(min(c["R"] for c in feas))
        if vs:
            ax.plot(vs, rs, style, color="tab:blue", lw=1.8,
                    label=f"cambered feasible R_min ({law})")
    # differential-steering overlay
    ax.fill_between([0.45, 1.25], 1 / SLTL_CURVATURE[1], 1 / SLTL_CURVATURE[0],
                    color="tab:orange", alpha=0.25,
                    label=f"SLTL differential steering ({SLTL_NOTE})")
    ax.scatter([WEBOTS_V, WEBOTS_V], WEBOTS_R, marker="x", c="tab:red",
               label=f"own Webots turns, R {WEBOTS_R[0]}-{WEBOTS_R[1]} m "
                     f"(non-repeatable)")
    ax.axvspan(*MEASURED_V, color="tab:green", alpha=0.12,
               label=f"measured operating band {MEASURED_V[0]}-{MEASURED_V[1]} m/s")
    # where large camber would sit (E1 annotation)
    for lam_deg in (10.0, 20.0):
        ax.plot(v_line, [turn_radius(v, np.deg2rad(lam_deg)) for v in v_line],
                ":", color="gray", lw=1)
        ax.annotate(f"lam = {lam_deg:.0f} deg", (1.02, turn_radius(
            1.02, np.deg2rad(lam_deg))), fontsize=8, color="gray")
    ax.set_yscale("log")
    ax.set_xlabel("mean forward speed [m/s]")
    ax.set_ylabel("turn radius R [m] (log)")
    ax.set_title("Stage 2a: coordinated-turn envelope vs differential steering"
                 "\n(model result, not demonstrated mechanism -- see docstring)")
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "stage2a_turning_envelope.png", dpi=160)
    print(f"wrote {FIG_DIR / 'stage2a_turning_envelope.png'}")

    # ---- constraint map: (v, lambda) colored by binding constraint ----------
    fig2, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
    order = ["none", "scrub", "existence", "leg-torque", "abad"]
    colors = {"none": "tab:green", "scrub": "tab:gray",
              "existence": "tab:red", "leg-torque": "tab:orange",
              "abad": "tab:purple"}
    for ax2, law in zip(axes, ("empirical", "geometric")):
        sub = [c for c in cells if c["law"] == law]
        for c in sub:
            ax2.scatter(c["v_td"], c["lam_deg"], s=42,
                        c=colors[c["binding"]], marker="s")
        ax2.set_title(f"{law} radius law")
        ax2.set_xlabel("touchdown speed v_td [m/s]")
        ax2.set_yscale("symlog", linthresh=3.0)
    axes[0].set_ylabel("lambda [deg] (symlog)")
    handles = [plt.Line2D([], [], marker="s", ls="", color=colors[k],
                          label=("feasible" if k == "none" else k))
               for k in order]
    axes[1].legend(handles=handles, fontsize=8)
    fig2.suptitle("binding constraint per cell (C5 slew reported, not gated)")
    fig2.tight_layout()
    fig2.savefig(FIG_DIR / "stage2a_constraint_map.png", dpi=160)
    print(f"wrote {FIG_DIR / 'stage2a_constraint_map.png'}")


def check_predictions(cells):
    print("\n--- registered predictions (section 83) ---")
    feas = [c for c in cells if c["feasible"] and c["lam_deg"] > 0]
    lam_max = max((c["lam_deg"] for c in feas), default=0.0)
    print(f"E1 max feasible coordinated-bank lambda = {lam_max:.1f} deg "
          f"(prediction: <~ 1.5-2 deg) "
          f"{'PASS' if lam_max <= 2.5 else 'FAIL'}")
    band = [c for c in cells if c["law"] == "empirical" and c["exists"]
            and MEASURED_V[0] <= c["v_fwd"] <= MEASURED_V[1]
            and not c["feasible"]]
    n_scrub = sum(1 for c in band if c["binding"] == "scrub")
    print(f"E2 binding constraint in the measured band: "
          f"{n_scrub}/{len(band)} blocked cells are scrub-blocked "
          f"{'PASS' if band and n_scrub == len(band) else 'CHECK'}")
    f_emp = {(c['v_td'], c['lam_deg']) for c in cells
             if c['law'] == 'empirical' and c['feasible']}
    f_geo = {(c['v_td'], c['lam_deg']) for c in cells
             if c['law'] == 'geometric' and c['feasible']}
    denom = max(1, len(f_emp | f_geo))
    diff = len(f_emp ^ f_geo) / denom
    print(f"E3 radius-law envelope-area difference = {diff:.1%} "
          f"(prediction < 2%) {'PASS' if diff < 0.02 else 'FAIL'}")
    def slew_frac(c):
        return c["entry_slew_ms"] / max(1e-9, 1e3 * c["flight_s"])

    steady = [c for c in cells if c["feasible"] and c["exists"]]
    worst_steady = max(steady, key=slew_frac, default=None)
    worst_any = max((c for c in cells if c["exists"]), key=slew_frac,
                    default=None)
    if worst_steady:
        frac = slew_frac(worst_steady)
        print(f"E4 worst entry slew / flight over STEADY (feasible) cells = "
              f"{frac:.2f} (lam {worst_steady['lam_deg']:.1f}d) "
              f"{'PASS' if frac < 1.0 else 'FAIL'}")
    if worst_any:
        print(f"   context: over all existence cells the worst is "
              f"{slew_frac(worst_any):.2f} (lam {worst_any['lam_deg']:.0f}d, "
              f"v_td {worst_any['v_td']:.2f}) -- large-lam entry does NOT "
              f"fit one flight (the section-57 rate budget, re-found), but "
              f"those cells are scrub-infeasible as turns anyway")


def main(argv):
    _selftest()
    print("selftest: PASS")
    full = "--full" in argv
    plots = "--no-plots" not in argv
    cache_out = cache_in = None
    if "--cache" in argv:
        cache_out = argv[argv.index("--cache") + 1]
    if "--from-cache" in argv:
        cache_in = argv[argv.index("--from-cache") + 1]

    if cache_in:
        cells = list(np.load(cache_in, allow_pickle=True)["cells"])
        print(f"loaded {len(cells)} cells from {cache_in}")
    else:
        v_grid = V_GRID_FULL if full else V_GRID_QUICK
        step = 0.5 if full else 2.0
        print(f"grid: {len(v_grid)} speeds x {len(LAM_GRID_DEG)} lambdas x "
              f"2 laws, beta step {step}")
        cells = run_grid(v_grid, LAM_GRID_DEG, step, verbose=not full)
        if cache_out:
            np.savez_compressed(cache_out, cells=np.array(cells, dtype=object))
            print(f"cached {len(cells)} cells to {cache_out}")

    print_table(cells)
    check_predictions(cells)
    if plots:
        make_figures(cells)


if __name__ == "__main__":
    main(sys.argv[1:])
