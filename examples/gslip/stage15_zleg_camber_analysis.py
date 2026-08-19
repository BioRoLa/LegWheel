"""Stage 1.5 z_leg camber analysis -- the offline measurement-model validator.

The ESEKF's no-slip velocity constraint turns encoder data into an "observed
body velocity" z_leg = -(omega x r_c + r_dot_leg + omega_rim x r_rim). The
sagittal estimator builds r_c with no camber (y hardcoded to the hip offset,
rim spin axis hardcoded body-Y) and rolls on the legacy radius 0.119 m. This
script replays that measurement model -- and its camber-corrected variant --
sample by sample against supervisor ground truth on the banked Stage 1 wheel
mode dumps, which contain everything the correction touches (measured gamma,
contact flags, GT pose/velocity/quat) and no IMU (so the FULL filter cannot
replay; the measurement model can, which is exactly what Stage 1.5 changes).

Carry-ins enforced, per the Stage 1 analyser:
1. ACHIEVED, NOT COMMANDED: per-leg gamma is re-baselined against the batch's
   own lambda = 0 run AT THE SAME kp (roll-state window -- the residual is
   state- and kp-dependent, no cross-kp scaling law).
2. UNFIT DUMPS ARE REFUSED: timestamp health gates, schema check, wheel-mode
   gate (achieved theta in the closed-wheel band; achieved theta is REPORTED
   because r0 depends on closure error -- pool radius fits over same-theta
   runs only), contact fraction sanity.
3. SELF-TEST FIRST on synthetic known answers (independently hand-derived,
   not round-tripped through the code under test) -- refuses to run if its
   own arithmetic is off.
4. SIGN CONVENTIONS ARE PARAMETERS, ADJUDICATED IN THE OPEN: gamma local
   signs and wheel spin signs default to the odometry/controller conventions
   and every run reports per-leg sign diagnostics; a wrong default shows up
   as a named verdict, never a silent flip.

Geometry: the corrected contact point replicates corgi_utils leg_model.cpp
contact_map_3d (ABAD/gslip): d_wheel = ABAD_AXIS_TO_WHEEL_PLANE + edge offset
(lower wheel edge, -+ half width by sign of sin gamma), rotated about the
body-x-parallel ABAD axis. The design effective radius comes from the shared
seam rolling_radius(lambda) (slip_rf_cambered) -- the same function the
models and the Stage 1 gate use. The ABAD axis |y| = 0.12 is read from the
Webots proto hinge anchor and is PENDING verification V1 (vs CAD); the legacy
sagittal frame (y = 0.193, r = 0.119) is never mixed with the ABAD frame, so
there is no double-count path.

State naming: all numbers here are ROLL-state, and the corpus is kp-labelled;
calibrations produced here are (roll, kp) tagged sim values.

Run (no args = selftest + expected schema):
    uv run python examples/gslip/stage15_zleg_camber_analysis.py \
        <dump.npz>:<lam_cmd_deg>:<kp> [...] [--r0 <m>] [--fit]
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np

from legwheel.models.slip_rf_cambered import rolling_radius

# the seam function is scalar; the analyser feeds it per-sample arrays
rolling_radius_vec = np.vectorize(rolling_radius, otypes=[float])

# ---------------------------------------------------------------- constants

# Module order in every dump: A B C D == LF RF RH LH (odometry createLeg
# order; consistent with the Stage 1 LR pattern: left pair {A, D}, right
# pair {B, C}).
SX = np.array([+1.0, +1.0, -1.0, -1.0])   # front/hind
SY = np.array([+1.0, -1.0, -1.0, +1.0])   # left/right (== stage1 LR_SIGNS)
LEG_NAMES = ["A/LF", "B/RF", "C/RH", "D/LH"]

X_HIP = 0.222            # body-x hip offset [m] (odometry Config LEG_X_OFFSET)
Y_HIP_LEGACY = 0.193     # sagittal estimator's hip y [m] (LEG_Y_OFFSET)
R_LEGACY = 0.119         # sagittal rim contact radius [m] (0.10 + 0.019)

Y_ABAD = 0.12            # ABAD hinge |y| from CorgiRobotABAD.proto -- V1 pending
D_PLANE = 0.091675       # ABAD axis -> wheel plane [m] (leg_model.cpp)
W_HALF = 0.02            # half wheel width [m] (leg_model.cpp wheel_thickness/2)

# Sign parameters (adjudicated on data via the per-leg diagnostics, V2).
GAMMA_LOCAL_SIGNS = np.array([+1.0, +1.0, +1.0, +1.0])  # raw motor gamma -> local tilt
# ADJUDICATED 2026-08-19 on the kp500 corpus: with the odometry's leg-mode
# convention (right legs flipped, SY) the spin/GT-forward correlation came
# out exactly {+1, -1, -1, +1} per leg -- raw beta_dot from motor/state is
# forward-positive for ALL legs in sim wheel mode. The diagnostics column
# guards this; if it ever prints -1 on any leg, the convention changed.
SPIN_SIGNS = np.array([+1.0, +1.0, +1.0, +1.0])

THETA_CLOSURE_DEG = 18.04    # calibrated concentric closure (roll, kp 500)
# Wheel-mode gate: achieved theta must sit in the closed-wheel band. The
# banked sweep runs at the DEFAULT theta command (achieved ~16.2 deg,
# uncalibrated closure, e ~1.16 mm) -- deliberately scoreable: that
# eccentricity is what the k=1 fit measures. Radius fits must pool only
# same-theta runs (r0 depends on closure error); achieved theta is reported
# per run for exactly that grouping decision.
THETA_WHEEL_BAND_DEG = (15.0, 20.0)

ROLL_WINDOW = (13.0, 28.0)   # stage1 runner schedule, steady rolling
MAX_DUP_FRACTION = 0.01
MAX_BACKWARD_STEPS = 0
MIN_CONTACT_FRACTION = 0.5   # per leg, inside the roll window

SMOOTH_MOTOR_S = 0.051       # moving-average window on 1 kHz derivatives
SMOOTH_ODOM_S = 0.05         # on 100 Hz quaternion-derived omega

REQUIRED_KEYS = ("motor_t", "motor_deg", "contact_t", "contact",
                 "odom_t", "odom")


# ------------------------------------------------------------------ helpers

@dataclass
class HealthReport:
    dup_fraction: float
    n_backward: int

    @property
    def ok(self) -> bool:
        return (self.dup_fraction <= MAX_DUP_FRACTION
                and self.n_backward <= MAX_BACKWARD_STEPS)


def timestamp_health(t: np.ndarray) -> HealthReport:
    t = np.asarray(t, float)
    if len(t) < 2:
        return HealthReport(dup_fraction=1.0, n_backward=0)
    dup = 1.0 - len(np.unique(t)) / len(t)
    back = int(np.sum(np.diff(t) < 0.0))
    return HealthReport(dup_fraction=float(dup), n_backward=back)


def _moving_average(v: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return v
    kernel = np.ones(n) / n
    if v.ndim == 1:
        return np.convolve(v, kernel, mode="same")
    return np.apply_along_axis(lambda c: np.convolve(c, kernel, mode="same"),
                               0, v)


def _rate(t: np.ndarray, v: np.ndarray, smooth_s: float) -> np.ndarray:
    """d/dt via central differences + moving average. v: (n,) or (n, k).
    Uses index gradient over the MEDIAN dt: the dumps carry ~0.1% duplicate
    timestamps (within the health gate), and np.gradient over t divides by
    the zero step there, after which the smoothing kernel smears nan over
    the whole series."""
    dt = float(np.median(np.diff(t)))
    dv = np.gradient(v, axis=0) / dt
    return _moving_average(dv, max(1, int(round(smooth_s / dt)) | 1))


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """(n, 4) [x y z w] -> (n, 3, 3) body->world."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.stack
    return np.stack([
        n([1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        n([2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)], -1),
        n([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)], -1),
    ], axis=1)


def omega_body_from_quat(t: np.ndarray, q: np.ndarray,
                         smooth_s: float = SMOOTH_ODOM_S) -> np.ndarray:
    """Body angular velocity (n, 3) from a unit-quaternion series [x y z w]:
    omega_body = 2 * vec(conj(q) * q_dot)."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    # keep the series on one cover of S3 (sign flips would spike q_dot)
    flip = np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1.0, 1.0))
    q = np.vstack([q[:1], q[1:] * flip[:, None]])
    qd = np.gradient(q, axis=0) / float(np.median(np.diff(t)))  # dup-t safe
    x, y, z, w = q.T
    dx, dy, dz, dw = qd.T
    # conj(q) * q_dot, vector part (Hamilton, [x y z w] storage)
    ox = w * dx - x * dw - y * dz + z * dy
    oy = w * dy + x * dz - y * dw - z * dx
    oz = w * dz - x * dy + y * dx - z * dw
    omega = 2.0 * np.stack([ox, oy, oz], axis=-1)
    dt = float(np.median(np.diff(t)))
    return _moving_average(omega, max(1, int(round(smooth_s / dt)) | 1))


def _interp_cols(tq: np.ndarray, ts: np.ndarray, v: np.ndarray) -> np.ndarray:
    if v.ndim == 1:
        return np.interp(tq, ts, v)
    return np.stack([np.interp(tq, ts, v[:, k]) for k in range(v.shape[1])],
                    axis=-1)


# ------------------------------------------------------- measurement models

def zleg_sagittal(spin: np.ndarray, omega: np.ndarray, leg: int,
                  r_eff: float = R_LEGACY) -> np.ndarray:
    """The estimator's current model: contact at (sx X, sy Y_legacy, -r),
    rolling about body-Y. spin: (n,) signed wheel rate; omega: (n, 3).
    Returns z_leg (n, 3) = -(omega x r_c + v_roll)."""
    r_c = np.array([SX[leg] * X_HIP, SY[leg] * Y_HIP_LEGACY, -r_eff])
    v_rot = np.cross(omega, r_c[None, :])
    v_roll = np.stack([-spin * r_eff, np.zeros_like(spin),
                       np.zeros_like(spin)], axis=-1)
    return -(v_rot + v_roll)


def _d_wheel(gam_loc: np.ndarray) -> np.ndarray:
    """contact_map_3d convention: wheel-plane distance + lower-edge offset."""
    s = np.sin(gam_loc)
    edge = np.where(np.abs(s) < 1e-4, 0.0, np.where(s > 0, -W_HALF, W_HALF))
    return D_PLANE + edge


def zleg_cambered(gam_loc: np.ndarray, gam_loc_d: np.ndarray,
                  spin: np.ndarray, omega: np.ndarray, leg: int,
                  r_eff: np.ndarray | float,
                  r_dot: np.ndarray | float = 0.0) -> np.ndarray:
    """Camber-corrected model, replicating leg_model.cpp contact_map_3d:
    local (Y, Z) offsets from the ABAD axis, rotated by the LOCAL tilt
    gam_loc; mapped to body y via the leg's side sign. The rolling term is
    exactly -spin * r_eff * x_hat (the tilted-axis cross product collapses:
    n_hat x rho = r_eff * x_hat for contact radially below in the wheel
    plane). Returns z_leg (n, 3)."""
    r_eff = np.broadcast_to(np.asarray(r_eff, float), gam_loc.shape)
    z2 = -r_eff
    d = _d_wheel(gam_loc)
    cg, sg = np.cos(gam_loc), np.sin(gam_loc)
    y_c = d * cg - z2 * sg                      # local, from ABAD axis
    z_c = d * sg + z2 * cg
    z2_dot = -np.broadcast_to(np.asarray(r_dot, float), gam_loc.shape)
    yd = (-d * sg - z2 * cg) * gam_loc_d - z2_dot * sg   # d piecewise-const
    zd = (d * cg - z2 * sg) * gam_loc_d + z2_dot * cg
    n = len(gam_loc)
    r_c = np.empty((n, 3))
    r_c[:, 0] = SX[leg] * X_HIP
    r_c[:, 1] = SY[leg] * (Y_ABAD + y_c)
    r_c[:, 2] = z_c
    v_rot = np.cross(omega, r_c)
    v_off = np.stack([-spin * r_eff, SY[leg] * yd, zd], axis=-1)
    return -(v_rot + v_off)


# ------------------------------------------------------------- run analysis

def _load(d) -> dict:
    missing = [k for k in REQUIRED_KEYS if k not in d]
    if missing:
        raise KeyError(f"missing keys: {missing}")
    return {k: np.asarray(d[k]) for k in REQUIRED_KEYS}


def analyse(d, lam_cmd: float, kp: float,
            residual_deg: np.ndarray | None,
            r0: float | None = None,
            ecc: dict | None = None,
            window=ROLL_WINDOW) -> dict:
    """One dump -> per-model per-axis residual RMS + diagnostics.
    `d` is an npz file or dict with the recorder schema. `residual_deg` is
    the batch lambda=0 roll-window per-leg gamma (deg) at this kp."""
    try:
        s = _load(d)
    except KeyError as e:
        return {"lam_cmd": lam_cmd, "kp": kp, "refused": f"SCHEMA: {e}"}

    mt, motor = s["motor_t"], s["motor_deg"]
    hm, ho = timestamp_health(mt), timestamp_health(s["odom_t"])
    if not (hm.ok and ho.ok):
        return {"lam_cmd": lam_cmd, "kp": kp,
                "refused": f"UNFIT: motor dup {hm.dup_fraction:.1%}/back "
                           f"{hm.n_backward}, odom dup {ho.dup_fraction:.1%}"
                           f"/back {ho.n_backward}"}

    m = (mt >= window[0]) & (mt <= window[1])
    if not np.any(m):
        return {"lam_cmd": lam_cmd, "kp": kp,
                "refused": f"no samples in roll window {window} -- truncated?"}

    theta_ach = motor[m][:, :, 0].mean(axis=0)
    lo, hi = THETA_WHEEL_BAND_DEG
    if np.min(theta_ach) < lo or np.max(theta_ach) > hi:
        return {"lam_cmd": lam_cmd, "kp": kp,
                "refused": f"NOT WHEEL MODE: achieved theta "
                           f"{np.round(theta_ach, 2)} outside "
                           f"[{lo}, {hi}] deg"}

    if residual_deg is None:
        return {"lam_cmd": lam_cmd, "kp": kp,
                "refused": "no lambda=0 run at this kp in the batch -- the "
                           "gamma re-baseline is kp-dependent with no "
                           "scaling law; refusing to guess"}

    # contact on the motor clock
    if len(s["contact_t"]) == len(mt) and np.allclose(s["contact_t"], mt):
        contact = s["contact"].astype(bool)
    else:
        contact = _interp_cols(mt, s["contact_t"],
                               s["contact"].astype(float)) > 0.5
    cfrac = contact[m].mean(axis=0)
    if np.min(cfrac) < MIN_CONTACT_FRACTION:
        return {"lam_cmd": lam_cmd, "kp": kp,
                "refused": f"CONTACT: in-window contact fraction "
                           f"{np.round(cfrac, 2)} < {MIN_CONTACT_FRACTION}"}

    # kinematic inputs (1 kHz), rad
    beta = np.deg2rad(motor[:, :, 1])
    gam_raw = np.deg2rad(motor[:, :, 2])
    beta_d = _rate(mt, beta, SMOOTH_MOTOR_S)
    gam_d = _rate(mt, gam_raw, SMOOTH_MOTOR_S)
    gam_ach = gam_raw - np.deg2rad(residual_deg)[None, :]

    # GT on the motor clock
    ot, odom = s["odom_t"], s["odom"]
    Rq = quat_to_rotmat(odom[:, 6:10] /
                        np.linalg.norm(odom[:, 6:10], axis=1, keepdims=True))
    v_body_o = np.einsum("nij,nj->ni", np.transpose(Rq, (0, 2, 1)),
                         odom[:, 3:6])
    v_body = _interp_cols(mt, ot, v_body_o)
    omega = _interp_cols(mt, ot, omega_body_from_quat(ot, odom[:, 6:10]))

    lam_loc = GAMMA_LOCAL_SIGNS[None, :] * gam_ach
    world_lean = np.rad2deg(gam_ach[m].mean(axis=0)) * SY
    left, right = 0.5 * (world_lean[0] + world_lean[3]), \
        0.5 * (world_lean[1] + world_lean[2])

    models = {"sagittal": {}, "design": {}}
    if r0 is not None:
        models["calibrated"] = {}
    if r0 is not None and ecc is not None:
        models["ecc"] = {}
    resid_store = {}
    spin_corr = np.zeros(4)
    for leg in range(4):
        mask = m & contact[:, leg]
        spin = SPIN_SIGNS[leg] * beta_d[:, leg]
        gl, gld = lam_loc[:, leg], GAMMA_LOCAL_SIGNS[leg] * gam_d[:, leg]
        z_sag = zleg_sagittal(spin, omega, leg)
        r_des = rolling_radius_vec(np.abs(gl))
        z_des = zleg_cambered(gl, gld, spin, omega, leg, r_des)
        legs = {"sagittal": z_sag, "design": z_des}
        if r0 is not None:
            legs["calibrated"] = zleg_cambered(gl, gld, spin, omega, leg, r0)
        if r0 is not None and ecc is not None:
            e_leg, phi_leg = ecc[leg]
            r_b = r0 + e_leg * np.cos(beta[:, leg] + phi_leg)
            r_b_dot = -e_leg * np.sin(beta[:, leg] + phi_leg) * beta_d[:, leg]
            legs["ecc"] = zleg_cambered(gl, gld, spin, omega, leg, r_b,
                                        r_dot=r_b_dot)
        for name, z in legs.items():
            models[name][leg] = (z - v_body)[mask]
        resid_store[leg] = {"mask": mask, "spin": spin, "beta": beta[:, leg],
                            "r_des": r_des}
        vx = v_body[mask, 0]
        sp = spin[mask]
        denom = np.linalg.norm(sp) * np.linalg.norm(vx)
        spin_corr[leg] = float(sp @ vx / denom) if denom > 0 else 0.0

    def rms(name):
        r = np.vstack([models[name][leg] for leg in range(4)])
        return np.sqrt(np.mean(r**2, axis=0))

    out = {
        "lam_cmd": lam_cmd, "kp": kp,
        "left": float(left), "right": float(right),
        "theta_ach": theta_ach, "contact_frac": cfrac,
        "spin_gt_corr": spin_corr,          # ~+1 per leg if spin signs right
        "mean_speed": float(np.linalg.norm(v_body[m, :2], axis=1).mean()),
        # GT lateral drift in the window: if this tracks the y-residual, the
        # lateral error is true side-slip (camber thrust), not geometry.
        "gt_vy_rms": float(np.sqrt(np.mean(v_body[m, 1] ** 2))),
        "rms": {name: rms(name) for name in models},
        "per_leg_y_rms": {
            name: np.array([np.sqrt(np.mean(models[name][leg][:, 1]**2))
                            for leg in range(4)])
            for name in models},
        "_internals": {"models": models, "store": resid_store},
    }
    return out


def fit_r0(results: list[dict]) -> dict:
    """Effective rolling radius from lambda=0 runs: least squares of GT
    forward speed against signed wheel spin, all legs, contact samples.
    ROLL state, at the batch kp. Refuses outside [0.110, 0.150] (P3 gate)."""
    num = den = 0.0
    n = 0
    for r in results:
        if r.get("refused") or r["lam_cmd"] != 0.0:
            continue
        st = r["_internals"]["store"]
        des = r["_internals"]["models"]["design"]
        for leg in range(4):
            mask = st[leg]["mask"]
            sp = st[leg]["spin"][mask]
            r_des = st[leg]["r_des"][mask]
            # design model: z_x = spin * r_des - (omega x r_c)_x - (gdot)_x,
            # residual = z_x - v_bx. The r-independent part is z_x - spin *
            # r_des, so the least-squares target for r is
            # spin * r_des - residual_x -- the omega and gdot terms cancel.
            target = sp * r_des - des[leg][:, 0]
            num += float(sp @ target)
            den += float(sp @ sp)
            n += len(sp)
    if den == 0.0:
        return {"refused": "no lambda=0 contact samples to fit r0"}
    r0 = num / den
    out = {"r0": float(r0), "n_samples": n, "state": "roll"}
    if not (0.110 <= r0 <= 0.150):
        out["refused"] = (f"r0 fit {r0:.4f} outside [0.110, 0.150] -- not "
                          f"blessing a calibrated radius; escalate (P3)")
    return out


def fit_eccentricity(results: list[dict], r0: float) -> dict:
    """Per-leg k=1 phase-locked radius modulation from lambda=0 runs:
    residual_x ~= beta_dot * e * cos(beta + phi). Regress on
    [spin*cos(beta), spin*sin(beta)]. Returns per-leg (e, phi) + fit
    uncertainty (P5 needs e >= 3x sigma and phase stability)."""
    per_leg = {}
    for leg in range(4):
        X_rows, y_rows = [], []
        for r in results:
            if r.get("refused") or r["lam_cmd"] != 0.0:
                continue
            st = r["_internals"]["store"][leg]
            mask = st["mask"]
            sp, be = st["spin"][mask], st["beta"][mask]
            res_x = r["_internals"]["models"]["calibrated"][leg][:, 0]
            X_rows.append(np.stack([sp * np.cos(be), sp * np.sin(be)], -1))
            y_rows.append(res_x)
        if not X_rows:
            return {"refused": "no lambda=0 runs for eccentricity fit"}
        X = np.vstack(X_rows)
        y = np.concatenate(y_rows)
        coef, res_ss, *_ = np.linalg.lstsq(X, y, rcond=None)
        # The regression describes the RESIDUAL (the model's excess); the
        # returned (e, phi) are in the CONSTRAINT convention
        # r(beta) = r0 + e cos(beta + phi), i.e. negated, so applying them
        # reduces the residual. (First application run had this backwards:
        # x rose 4.9 -> 5.8 mm/s, exactly the fitted amplitude added twice.)
        a, b = -coef[0], -coef[1]
        dof = max(1, len(y) - 2)
        s2 = float(res_ss[0] / dof) if len(res_ss) else float(
            np.mean((y - X @ coef)**2))
        cov = s2 * np.linalg.inv(X.T @ X)
        sigma_e = float(np.sqrt(np.trace(cov)))
        per_leg[LEG_NAMES[leg]] = {
            "e_m": float(np.hypot(a, b)),
            "phi_rad": float(np.arctan2(-b, a)),
            "sigma_e_m": sigma_e,
        }
    return {"per_leg": per_leg, "state": "roll", "r0_used": r0}


# ----------------------------------------------------------------- selftest

def _synth(n=2001, t1=20.0, gamma_deg=0.0, gamma_dot=0.0, vx=0.4,
           r_true=0.13, omega_body=(0.0, 0.0, 0.0), model="cambered"):
    """Synthetic dump built from hand physics, NOT from the models under
    test: wheels spin at vx / r_true; GT velocity is vx plus, when the body
    rotates, the rigid-body velocity implied by a fixed world contact --
    which is exactly what no-slip means, derived independently."""
    t = np.linspace(10.0, 10.0 + t1, n)
    motor = np.zeros((n, 4, 3))
    motor[:, :, 0] = THETA_CLOSURE_DEG
    g0 = np.deg2rad(gamma_deg)
    gam_loc = (g0 + gamma_dot * (t - t[0]))
    for leg in range(4):
        motor[:, leg, 2] = np.rad2deg(GAMMA_LOCAL_SIGNS[leg] * gam_loc)
        motor[:, leg, 1] = np.rad2deg(
            SPIN_SIGNS[leg] * (vx / r_true) * (t - t[0]))
    odom = np.zeros((n, 10))
    odom[:, 0] = vx * (t - t[0])
    odom[:, 3] = vx
    odom[:, 9] = 1.0                                  # identity quat
    d = {"motor_t": t, "motor_deg": motor, "contact_t": t,
         "contact": np.ones((n, 4), bool), "odom_t": t[::10],
         "odom": odom[::10], "anchor": 0.0}
    return d, gam_loc, r_true


def _selftest() -> None:
    # timestamp gate
    clean = np.linspace(0, 10, 1001)
    assert timestamp_health(clean).ok
    dirty = clean.copy(); dirty[100:300] = dirty[100]
    assert not timestamp_health(dirty).ok
    back = clean.copy(); back[500] = back[499] - 0.1
    assert not timestamp_health(back).ok

    # omega from quaternion: constant yaw rate 0.3 rad/s
    t = np.linspace(0, 10, 1001)
    yaw = 0.3 * t
    q = np.stack([np.zeros_like(t), np.zeros_like(t),
                  np.sin(yaw / 2), np.cos(yaw / 2)], -1)
    om = omega_body_from_quat(t, q)
    assert abs(om[100:-100, 2].mean() - 0.3) < 1e-6
    assert np.max(np.abs(om[100:-100, :2])) < 1e-9

    # seam anchor
    assert abs(rolling_radius(0.0) - 0.145) < 1e-15

    # rolling collapse: cambered z_leg at constant gamma, no rotation,
    # equals spin * r_eff forward exactly (hand-derived: n_hat x rho =
    # r_eff x_hat) -- and has zero y/z components.
    sp = np.full(11, 0.4 / 0.13)
    z = zleg_cambered(np.full(11, 0.3), np.zeros(11), sp,
                      np.zeros((11, 3)), 0, 0.13)
    assert np.allclose(z[:, 0], 0.4, atol=1e-12)
    assert np.allclose(z[:, 1:], 0.0, atol=1e-12)

    # gamma_dot lever, hand-derived at gamma = 0: d/dt of (Y, Z) =
    # (-z2 * cg, d * cg) * gdot = (r_eff, d_plane - W_HALF... edge at g=0
    # is 0) -> y_dot = r_eff * gdot, z_dot = D_PLANE * gdot (leg A, sy=+1).
    gd = 0.2
    z = zleg_cambered(np.zeros(3), np.full(3, gd), np.zeros(3),
                      np.zeros((3, 3)), 0, 0.13)
    assert np.allclose(z[:, 1], -0.13 * gd, atol=1e-12)
    assert np.allclose(z[:, 2], -D_PLANE * gd, atol=1e-12)

    # full-path synthetic: pure roll, gamma = 0 -- BOTH models recover GT
    # with the true radius; sagittal with the legacy radius shows exactly
    # the (1 - 0.119/0.13) forward deficit.
    d, _, r_true = _synth()
    res = analyse(d, 0.0, 500.0, residual_deg=np.zeros(4), r0=r_true)
    assert "refused" not in res, res.get("refused")
    assert res["rms"]["calibrated"][0] < 2e-3, res["rms"]["calibrated"]
    assert np.all(res["rms"]["calibrated"] < 2e-3)
    expected_deficit = (1 - R_LEGACY / r_true) * 0.4
    assert abs(res["rms"]["sagittal"][0] - expected_deficit) < 5e-3
    assert np.all(res["spin_gt_corr"] > 0.99)
    r0fit = fit_r0([res])
    assert "refused" not in r0fit and abs(r0fit["r0"] - r_true) < 1e-3

    # full-path synthetic: constant camber 20 deg rolling -- calibrated
    # cambered model still exact (rolling collapse is gamma-independent);
    # design mode lands near rolling_radius(20 deg), not exact vs r_true.
    d20, _, _ = _synth(gamma_deg=20.0)
    res20 = analyse(d20, 20.0, 500.0, residual_deg=np.zeros(4), r0=r_true)
    assert "refused" not in res20
    assert np.all(res20["rms"]["calibrated"] < 2e-3)

    # refusals: schema, not-wheel-mode (folded leg), missing lambda=0 residual
    assert "refused" in analyse({"motor_t": clean}, 0, 500, np.zeros(4))
    d_off, _, _ = _synth()
    d_off["motor_deg"] = d_off["motor_deg"].copy()
    d_off["motor_deg"][:, :, 0] = 10.0
    assert "refused" in analyse(d_off, 0, 500, np.zeros(4))
    d_ok, _, _ = _synth()
    assert "refused" in analyse(d_ok, 0, 500, residual_deg=None)


# --------------------------------------------------------------------- main

def _fmt_rms(v):
    return "/".join(f"{x * 1e3:6.1f}" for x in v)


def main(argv) -> None:
    _selftest()
    if not argv:
        print(__doc__)
        print("selftest: PASS. Expected npz schema: motor_t (n,), motor_deg "
              "(n,4,3) [theta beta gamma] deg, contact_t (n,), contact "
              "(n,4) bool, odom_t (k,), odom (k,10) [p v_lin quat].")
        return

    r0 = None
    do_fit = False
    do_ecc = False
    runs = []
    it = iter(argv)
    for arg in it:
        if arg == "--r0":
            r0 = float(next(it))
        elif arg == "--fit":
            do_fit = True
        elif arg == "--ecc-apply":
            do_fit = True
            do_ecc = True
        else:
            path, lam, kp = arg.rsplit(":", 2)
            runs.append((path, float(lam), float(kp)))

    # per-kp gamma re-baseline from each kp group's own lambda=0 run
    residuals: dict[float, np.ndarray] = {}
    for path, lam, kp in runs:
        if lam == 0.0 and kp not in residuals:
            d = np.load(path)
            mt, motor = d["motor_t"], d["motor_deg"]
            m = (mt >= ROLL_WINDOW[0]) & (mt <= ROLL_WINDOW[1])
            residuals[kp] = motor[m][:, :, 2].mean(axis=0)
            print(f"kp {kp:g} roll-state gamma residual (A B C D deg): "
                  + " ".join(f"{v:+.2f}" for v in residuals[kp]))

    print(f"\n{'lam':>5} {'kp':>5} {'ach L/R':>13} | per-axis residual RMS "
          f"x/y/z [mm/s]")
    print(f"{'':>25} | {'sagittal':>22} | {'design':>22} | "
          f"{'calibrated' if r0 else '(no --r0)':>22} | spin corr")
    results = []
    for path, lam, kp in runs:
        res = analyse(np.load(path), lam, kp,
                      residual_deg=residuals.get(kp), r0=r0)
        results.append(res)
        if "refused" in res:
            print(f"{lam:4.0f}d {kp:5.0f}  {res['refused']}")
            continue
        cal = _fmt_rms(res["rms"]["calibrated"]) if r0 else "--"
        print(f"{lam:4.0f}d {kp:5.0f} {res['left']:+6.2f}/{res['right']:+6.2f}"
              f" | {_fmt_rms(res['rms']['sagittal'])} | "
              f"{_fmt_rms(res['rms']['design'])} | {cal} | "
              f"v {res['mean_speed']:.3f} vyRMS {res['gt_vy_rms'] * 1e3:5.1f} | "
              + " ".join(f"{c:+.2f}" for c in res["spin_gt_corr"]))

    if do_fit:
        print("\n--- fits (ROLL state, per-kp corpus) ---")
        fit = fit_r0(results)
        print(f"r0 fit: {fit}")
        if "refused" not in fit and r0 is not None:
            ecc = fit_eccentricity(results, r0)
            if "refused" in ecc:
                print(f"eccentricity: {ecc['refused']}")
            else:
                for name, e in ecc["per_leg"].items():
                    print(f"  ecc {name}: e = {e['e_m'] * 1e3:.2f} mm, phi = "
                          f"{np.rad2deg(e['phi_rad']):+7.1f} deg, sigma_e = "
                          f"{e['sigma_e_m'] * 1e3:.2f} mm")
                if do_ecc:
                    ecc_params = {leg: (ecc["per_leg"][LEG_NAMES[leg]]["e_m"],
                                        ecc["per_leg"][LEG_NAMES[leg]]["phi_rad"])
                                  for leg in range(4)}
                    print("\n--- ecc term APPLIED (calibrated | ecc) ---")
                    for path, lam, kp in runs:
                        r = analyse(np.load(path), lam, kp,
                                    residual_deg=residuals.get(kp), r0=r0,
                                    ecc=ecc_params)
                        if "refused" in r:
                            continue
                        print(f"{lam:4.0f}d {kp:5.0f} | "
                              f"{_fmt_rms(r['rms']['calibrated'])} | "
                              f"{_fmt_rms(r['rms']['ecc'])}")


if __name__ == "__main__":
    main(sys.argv[1:])
