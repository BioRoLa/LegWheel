"""Log s340 (unregistered post-hoc re-analysis, 2026-09-14): the s236
theta = 100 deg static holds re-scored with the in-plane leg's own swing.

Not registered: banked numbers re-scored after they were seen. No parameter is
fitted anywhere below.

DATA (WSL paths)
  ~/corgi_runs/contact_lateral_theta100/contact_lam{0,8..15}.csv
      Webots contact-debug captures written by sweep_contact_lateral_theta100.sh
      (registered in log s226, scored in s236). MEAS below is
      contact_lateral_fine.py's printed body-frame migration by(lam) - by(0),
      raw signed, mm, 2 dp; it is recomputed from the CSVs through
      contact_lateral.measure when that file is present.
  ~/camber_dumps/camber_clt100_lam*_kp90.npz
      record_camber.py dumps of the same holds (each file written within 0.5 s
      of its CSV). motor_deg / cmd_deg are (theta, beta, gamma) per module in
      deg; odom is [p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w] of
      sim/base_odom. Angles and roll are medians over the last TAIL_S = 15 s,
      the static tail contact_lateral.py scores.

PREDICTION
  phi is the achieved gamma, in contact_profile's per-side outboard convention
  (phi > 0 swings the contact outboard). A and D are the +y modules, B and C
  the -y modules, so a module's body-frame migration is SY * (y1 - y0) with
  SY = +1 for A/D and -1 for B/C, the mapping s335_geometry_c4_rescore.py
  applies to s190. The script counts predictions whose sign differs from the
  measurement's.

  contact_profile.contact_point(phi, D).y is Eq. (4) as implemented: the
  contact's lateral offset from the axle at the hip's axial station. The AB/AD
  joint also swings the in-plane leg. At theta = 100 deg the rim centre sits
  L = |LegModel.O_r| = 0.1481 m from the hip along the leg axis (log s21), so
  the hip-relative offset is y + L sin(phi). In CorgiRobotABAD.proto module
  A's AB/AD hinge (axis along body x, anchor y 0.12, z 0.057166) and its leg
  motor hinge (axis along body y, anchored 0.091675 m further outboard)
  intersect, so that swing is the only in-plane lever about the AB/AD axis.
  L cases: 0; 0.1481 m; L(theta) at each leg's achieved theta in each hold
  (all at beta = 0).

  The lam >= 8 contact is the lower shoulder's in every row. The lam = 0
  holds sit at gamma -0.36 to -0.70 deg, where the rigid flat band makes the
  reference a choice, so every convention is scored:
    exact     s335 s2's y_profile: D whenever |gamma0| < 1 deg (no L term)
    hybrid    D + L sin(gamma0)
    band      band centre (w = 0, rho = R_t + r_c) at gamma0, + L sin(gamma0)
    shoulder  contact_point(gamma0, D).y + L sin(gamma0)
  s335 s2 quotes its centred (exact) row as the conservative number.

  The 8 -> 15 deg secants do not depend on the reference, which cancels in
  the difference. They are printed per commanded degree (/7) and per achieved
  degree (/|gamma15 - gamma8| of that leg).

LEGACY FORM
  s190's y = D cos(phi) + 0.145 sin(phi), plus the same L sin(phi). Its excess
  over the profile contact is s w_c cos(phi) + r_c sin(phi), s = sgn(phi),
  which equals d_lat cos(phi) + r_c (1 - cos(phi)) sin(phi); both identities
  are asserted at every scored lean. In the body frame that excess adds
  w_c cos|phi| + r_c sin|phi| to every leg's migration against a common
  reference, so the secants separate the two forms with no reference choice.

NOT MODELLED: static body roll
  Every scored row selects the tread point at the body-frame lean. The holds
  roll (odom), and a rolled body changes which tread point touches flat ground.
  The CONTEXT block at the end prints the measured roll, a zero-parameter
  prediction of it from the four contact heights, and a sensitivity in which
  the tread point is selected at the ground-relative lean phi + SY * rho. That
  block is not a row of record.

    .venv/bin/python examples/gslip/s340_s236_hold_rescore.py
"""
import glob
import importlib.util
import math
import os
import re

import numpy as np

from legwheel.models import contact_profile as cp
from legwheel.models.leg_model import LegModel

D = 0.091675              # d_wheel, as in s335_geometry_c4_rescore.py
R_LEGACY = 0.145          # s190's legacy swing radius
TAIL_S = 15.0             # contact_lateral.py's static tail (s)
HIP_Y = 0.12              # |AB/AD anchor y| of all four modules, CorgiRobotABAD.proto (m)
MODS = "ABCD"
SY = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
LAMS = (8, 9, 10, 11, 12, 13, 14, 15)
# contact_lateral_fine.py --dir ~/corgi_runs/contact_lateral_theta100, verbatim
MEAS = {
    8: (33.30, 38.45, 43.20, 37.95), 9: (38.50, 44.15, 48.80, 43.15),
    10: (43.80, 49.65, 54.20, 48.45), 11: (49.00, 54.75, 58.35, 53.65),
    12: (54.00, 59.20, 64.00, 57.20), 13: (57.75, 64.80, 69.50, 62.30),
    14: (62.80, 70.30, 75.00, 67.40), 15: (67.80, 75.75, 80.40, 72.55),
}
CSV_DIR = os.path.expanduser("~/corgi_runs/contact_lateral_theta100")
DUMP_GLOB = os.path.expanduser("~/camber_dumps/camber_clt100_lam*_kp90.npz")
CONTACT_LATERAL = os.path.expanduser(
    "~/corgi_ws/corgi_ros2_ws/src/corgi_force_control/scripts/diag/contact_lateral.py")
TREAD = cp.CORGI_TREAD
RHO_BAND = TREAD.r_spine + TREAD.r_crown
PROFILE = "PROFILE contact, Eq. (4) as implemented (contact_profile)"
LEGACY = "LEGACY form D cos + 0.145 sin"

_LM = LegModel()


def L_of(theta_deg):
    """Hip -> rim-centre length |O_r| (m) at theta (deg), beta = 0."""
    _LM.forward(np.deg2rad(theta_deg), 0.0, vector=False)
    return abs(complex(_LM.O_r))


def meas(lam, m):
    return MEAS[lam][MODS.index(m)]


# ---- data -----------------------------------------------------------------------
def reproduce_meas():
    if not os.path.exists(CONTACT_LATERAL):
        print(f"MEAS reproduction skipped: {CONTACT_LATERAL} not found")
        return
    spec = importlib.util.spec_from_file_location("contact_lateral", CONTACT_LATERAL)
    cl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cl)
    base = cl.measure(os.path.join(CSV_DIR, "contact_lam0.csv"))
    worst = 0.0
    for lam in LAMS:
        got = cl.measure(os.path.join(CSV_DIR, f"contact_lam{lam}.csv"))
        for m in MODS:
            worst = max(worst, abs(1e3 * (got[m]["by_med"] - base[m]["by_med"]) - meas(lam, m)))
    assert worst < 0.0051, worst
    print(f"MEAS reproduced from the CSVs by contact_lateral.measure: max |diff| {worst:.4f} mm")


def roll_deg(q):
    x, y, z, w = q.T
    return np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def load_holds():
    holds = {}
    for f in sorted(glob.glob(DUMP_GLOB)):
        lam = int(re.search(r"lam(\d+)_", os.path.basename(f)).group(1))
        z = np.load(f)
        t, ct, ot = z["motor_t"], z["cmd_t"], z["odom_t"]
        holds[lam] = {
            "ach": np.median(z["motor_deg"][t >= t[-1] - TAIL_S], axis=0),
            "cmd": np.median(z["cmd_deg"][ct >= ct[-1] - TAIL_S], axis=0),
            "roll": float(np.median(roll_deg(z["odom"][ot >= ot[-1] - TAIL_S, 6:10]))),
        }
    missing = [lam for lam in (0,) + LAMS if lam not in holds]
    assert not missing, f"missing dumps for lambda {missing}"
    return holds


# ---- contact forms (m, per-side outboard) ----------------------------------------
def y_profile(phi, L):
    return cp.contact_point(phi, D).y + L * math.sin(phi)


def y_legacy(phi, L):
    return D * math.cos(phi) + R_LEGACY * math.sin(phi) + L * math.sin(phi)


def contact_yz_selected(phi, phi_sel, L):
    """(y, z) of the tread point selected at lean phi_sel, rotated by phi."""
    w, rho = cp.profile_contact(phi_sel)
    axial = D + w
    c, s = math.cos(phi), math.sin(phi)
    return axial * c + rho * s + L * s, axial * s - rho * c - L * c


def ref_exact(g0, L0):
    if abs(math.degrees(g0)) >= 1.0:
        raise ValueError("s335's centred reference is defined for |gamma0| < 1 deg")
    return D


REFS_PROFILE = (
    ("exact", ref_exact),
    ("hybrid", lambda g0, L0: D + L0 * math.sin(g0)),
    ("band", lambda g0, L0: D * math.cos(g0) + (RHO_BAND + L0) * math.sin(g0)),
    ("shoulder", y_profile),
)
REFS_LEGACY = (
    ("own", y_legacy),
    ("exact", ref_exact),
    ("hybrid", lambda g0, L0: D + L0 * math.sin(g0)),
)


# ---- scoring ----------------------------------------------------------------------
def predict(holds, y1fn, ref, Lfn):
    """-> {(lam, module): predicted body-frame migration, mm}."""
    a0 = holds[0]["ach"]
    out = {}
    for lam in LAMS:
        a1 = holds[lam]["ach"]
        for i, m in enumerate(MODS):
            g1, g0 = math.radians(a1[i, 2]), math.radians(a0[i, 2])
            out[(lam, m)] = 1e3 * SY[m] * (y1fn(g1, Lfn(a1[i, 0])) - ref(g0, Lfn(a0[i, 0])))
    return out


def stats(pred):
    res = {k: pred[k] - meas(*k) for k in pred}
    v = np.array(list(res.values()))
    worst = max(res, key=lambda k: abs(res[k]))
    sign_diff = sum(1 for k in pred if np.sign(pred[k]) != np.sign(meas(*k)))
    return {"mae": float(np.mean(np.abs(v))), "max": float(np.max(np.abs(v))),
            "bias": float(np.mean(v)), "worst": f"{worst[1]}@{worst[0]}",
            "sign_diff": sign_diff}


def secants(pred, holds):
    """-> {module: (pred/cmd deg, meas/cmd deg, pred/ach deg, meas/ach deg)}."""
    out = {}
    for i, m in enumerate(MODS):
        dg = abs(holds[15]["ach"][i, 2] - holds[8]["ach"][i, 2])
        p = pred[(15, m)] - pred[(8, m)]
        q = meas(15, m) - meas(8, m)
        out[m] = (p / 7.0, q / 7.0, p / dg, q / dg)
    return out


def side_split(per_module):
    """mean over B, C (phi < 0) minus mean over A, D (phi > 0), elementwise."""
    bc = np.mean([per_module["B"], per_module["C"]], axis=0)
    ad = np.mean([per_module["A"], per_module["D"]], axis=0)
    return bc - ad


def level_split(pred):
    rows = []
    for lams in ((8,), (15,), LAMS):
        p = {m: np.mean([pred[(lam, m)] for lam in lams]) for m in MODS}
        q = {m: np.mean([meas(lam, m) for lam in lams]) for m in MODS}
        rows.append((side_split(p), side_split(q)))
    return rows


def print_secants(sec, indent="  "):
    print(f"{indent}8->15 secants (reference-independent), mm/deg, pred / meas:")
    print(f"{indent}  per commanded deg: " + "  ".join(
        f"{m} {sec[m][0]:4.2f}/{sec[m][1]:4.2f}" for m in MODS))
    print(f"{indent}  per achieved deg:  " + "  ".join(
        f"{m} {sec[m][2]:4.2f}/{sec[m][3]:4.2f}" for m in MODS))
    sp = side_split(sec)
    print(f"{indent}  side split phi<0 (B,C) - phi>0 (A,D): per commanded deg "
          f"{sp[0]:+.3f} pred / {sp[1]:+.3f} meas; per achieved deg "
          f"{sp[2]:+.3f} pred / {sp[3]:+.3f} meas")


def score_block(title, holds, y1fn, refs, lmodes, summary):
    print(f"\n######## {title}")
    for lname, Lfn in lmodes:
        print(f"\n== {title}; {lname}")
        print(f"  {'lambda-0 reference':10s} {'mean|res|':>9s} {'max|res|':>9s} "
              f"{'mean(pred-meas)':>16s} {'worst':>6s}  sign!=  level split B,C-A,D "
              f"pred/meas at 8 | 15 | mean 8..15 (mm)")
        sec = None
        for rname, ref in refs:
            pred = predict(holds, y1fn, ref, Lfn)
            s = stats(pred)
            summary[(title, lname, rname)] = s
            ls = level_split(pred)
            print(f"  {rname:10s} {s['mae']:9.2f} {s['max']:9.2f} {s['bias']:+16.2f} "
                  f"{s['worst']:>6s}  {s['sign_diff']:5d}   " + " | ".join(
                      f"{p:+5.2f}/{q:+5.2f}" for p, q in ls))
            this_sec = secants(pred, holds)
            if sec is not None:
                assert all(abs(this_sec[m][0] - sec[m][0]) < 1e-9 for m in MODS)
            sec = this_sec
        summary[(title, lname, "secants")] = sec
        print_secants(sec)


# ---- main -----------------------------------------------------------------------
def main():
    print(" ".join(__doc__.split("\n")[:2]))
    print("Unregistered post-hoc re-analysis; no fitted parameter.\n")
    reproduce_meas()
    holds = load_holds()
    L100 = L_of(100.0)
    print(f"L = |O_r|: theta 17 -> {L_of(17.0):.6f} m, theta 100 -> {L100:.6f} m")

    print("\nachieved (tail median) per module: cmd gamma / achieved gamma / achieved theta"
          " / L(theta) mm; body roll (odom) deg")
    for lam in (0,) + LAMS:
        h = holds[lam]
        print(f"  lam {lam:2d}: " + "  ".join(
            f"{m} {h['cmd'][i, 2]:+6.2f}/{h['ach'][i, 2]:+7.3f}/{h['ach'][i, 0]:6.2f}/"
            f"{1e3 * L_of(h['ach'][i, 0]):5.1f}" for i, m in enumerate(MODS))
              + f"  roll {h['roll']:+6.3f}")
    ths = [holds[lam]["ach"][i, 0] for lam in (0,) + LAMS for i in range(4)]
    print(f"  achieved theta {min(ths):.2f}..{max(ths):.2f} deg -> L "
          f"{L_of(min(ths)):.4f}..{L_of(max(ths)):.4f} m")
    print("  achieved gamma span 8->15 per leg (deg): " + "  ".join(
        f"{m} {abs(holds[15]['ach'][i, 2] - holds[8]['ach'][i, 2]):.3f}"
        for i, m in enumerate(MODS)))

    lmodes = (("L = 0", lambda th: 0.0),
              (f"L = {L100:.4f} m (theta = 100 deg)", lambda th: L100),
              ("L at achieved theta", L_of))
    summary = {}

    # side split of the local slope: the d_wheel cos(phi) term, not L
    u, h = math.radians(12.0), 1e-7
    s_pos = (y_profile(u + h, 0.0) - y_profile(u - h, 0.0)) / (2 * h)
    s_neg = (y_profile(-u + h, 0.0) - y_profile(-u - h, 0.0)) / (2 * h)
    assert abs((s_neg - s_pos) - 2 * D * math.sin(u)) < 1e-6
    k = math.pi / 180 * 1e3
    print(f"\nlocal slope of the migration at |phi| = 12 deg (mm/deg): phi>0 {s_pos * k:.3f}, "
          f"phi<0 {s_neg * k:.3f}; difference {(s_neg - s_pos) * k:.3f} = 2 D sin|phi| "
          f"(w_c cancels; L sin(phi) adds L cos(phi) to both sides)")

    score_block(PROFILE, holds, y_profile, REFS_PROFILE, lmodes, summary)

    # legacy vs profile identities, at every scored lean
    wc, rc = TREAD.w_flat, TREAD.r_crown
    for lam in LAMS:
        for i in range(4):
            phi = math.radians(holds[lam]["ach"][i, 2])
            w, _ = cp.profile_contact(phi)
            dlat = cp.contact_point(phi, D).d_lat
            assert abs(dlat + w) < 1e-15
            diff = y_legacy(phi, 0.0) - y_profile(phi, 0.0)
            assert abs(diff - (math.copysign(wc, phi) * math.cos(phi) + rc * math.sin(phi))) < 1e-12
            assert abs(diff - (dlat * math.cos(phi) + rc * (1 - math.cos(phi)) * math.sin(phi))) < 1e-12
    print("\nlegacy - profile identities hold at all 32 scored leans. Body-frame excess "
          "SY*(legacy - profile), mm, at lam 8 / 15:")
    print("  " + "  ".join(
        f"{m} " + "/".join(
            f"{1e3 * SY[m] * (y_legacy(math.radians(holds[lam]['ach'][i, 2]), 0.0) - y_profile(math.radians(holds[lam]['ach'][i, 2]), 0.0)):.2f}"
            for lam in (8, 15)) for i, m in enumerate(MODS)))

    score_block(LEGACY, holds, y_legacy, REFS_LEGACY, lmodes, summary)

    # ---- context: static body roll (not modelled above) ----------------------------
    print("\n######## CONTEXT: static body roll, not modelled in any row above")
    print("  A roll rho about body +x (rho < 0 lowers the +y side) leans a module's leg")
    print("  plane phi + SY*rho from the world vertical; on level ground that is the lean")
    print("  that selects the tread point. Roll about x keeps the in-plane lowest point")
    print("  directly below the rim centre within the leg plane, so only the profile's")
    print("  (w, rho) selection changes.")
    for lam in (0,) + LAMS:
        for i in range(4):
            phi = math.radians(holds[lam]["ach"][i, 2])
            y, z = contact_yz_selected(phi, phi, 0.0)
            pt = cp.contact_point(phi, D)
            assert abs(y - pt.y) < 1e-15 and abs(z - pt.z) < 1e-15

    def predicted_roll(ach):
        rho = 0.0
        for _ in range(50):
            ys, zs = {}, {}
            for i, m in enumerate(MODS):
                g = math.radians(ach[i, 2])
                y, z = contact_yz_selected(g, g + SY[m] * rho, L_of(ach[i, 0]))
                ys[m], zs[m] = SY[m] * (HIP_Y + y), z
            dy = 0.5 * (ys["A"] + ys["D"] - ys["B"] - ys["C"])
            dz = 0.5 * (zs["A"] + zs["D"] - zs["B"] - zs["C"])
            new = math.atan2(-dz, dy)
            if abs(new - rho) < 1e-13:
                return math.degrees(new), True
            rho = new
        return math.degrees(rho), False

    print("\n  roll (deg): measured (odom tail median) vs predicted from the four contact"
          " heights\n  (side means, each side one front and one rear module; L at achieved theta;"
          " all four\n  AB/AD anchors at body z 0.057166 in the proto; tread point selected at"
          " phi + SY*rho,\n  iterated to a fixed point; no fitted parameter)")
    for lam in (0,) + LAMS:
        pr, ok = predicted_roll(holds[lam]["ach"])
        print(f"    lam {lam:2d}: measured {holds[lam]['roll']:+6.3f}  predicted {pr:+6.3f}"
              f"{'' if ok else '  (NOT converged)'}")

    print("\n  SENSITIVITY (not a row of record): tread point selected at phi + SY*rho with"
          " the\n  measured roll of each hold; exact reference y0 = D.")
    for lname, Lfn in lmodes[1:]:
        pred = {}
        a0 = holds[0]["ach"]
        for lam in LAMS:
            a1, rho = holds[lam]["ach"], math.radians(holds[lam]["roll"])
            for i, m in enumerate(MODS):
                g1, g0 = math.radians(a1[i, 2]), math.radians(a0[i, 2])
                y1, _ = contact_yz_selected(g1, g1 + SY[m] * rho, Lfn(a1[i, 0]))
                pred[(lam, m)] = 1e3 * SY[m] * (y1 - ref_exact(g0, Lfn(a0[i, 0])))
        s = stats(pred)
        unrolled = summary[(PROFILE, lname, "exact")]
        print(f"  {lname}: mean|res| {s['mae']:.2f}  max {s['max']:.2f}  mean(pred-meas) "
              f"{s['bias']:+.2f} mm  (unrolled exact row: {unrolled['mae']:.2f} / "
              f"{unrolled['max']:.2f} / {unrolled['bias']:+.2f})")
        print_secants(secants(pred, holds), indent="    ")

    # ---- summary ---------------------------------------------------------------------
    print("\n######## SUMMARY: mean|res| / max|res| over 32 leg-points (mm)")
    for title, refs in ((PROFILE, REFS_PROFILE), (LEGACY, REFS_LEGACY)):
        print(f"  {title}")
        print(f"    {'':30s}" + "".join(f"{r:>14s}" for r, _ in refs))
        for lname, _ in lmodes:
            print(f"    {lname:30s}" + "".join(
                f"   {summary[(title, lname, r)]['mae']:5.2f}/{summary[(title, lname, r)]['max']:5.2f}"
                for r, _ in refs))
        print(f"    8->15 secants, mm/deg{'':9s}  per cmd deg A B C D     per ach deg A B C D")
        for lname, _ in lmodes:
            sec = summary[(title, lname, "secants")]
            print(f"    {lname:30s}  " + " ".join(f"{sec[m][0]:.2f}" for m in MODS)
                  + "     " + " ".join(f"{sec[m][2]:.2f}" for m in MODS))
    sec = summary[(PROFILE, lmodes[0][0], "secants")]
    print(f"  {'measured':32s}  " + " ".join(f"{sec[m][1]:.2f}" for m in MODS)
          + "     " + " ".join(f"{sec[m][3]:.2f}" for m in MODS))


if __name__ == "__main__":
    main()
