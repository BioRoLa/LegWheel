"""Stage 3 task 1: does leg Jacobian conditioning degrade with camber?

Thesis Timeline Stage 3 task 1: "Virtual spring via force control -- already
built in the sagittal controller (leg-frame impedance: k_radial 8941,
k_tangential 600, k_lateral 7500). Now it has to work OUT OF PLANE. Check
whether leg Jacobian conditioning degrades with camber."

Offline, zero simulator time. Implementation log S172 (Stage 3 cambered
thread).

WHAT IS UNDER STUDY. `KinematicsHelper::calculate_jacobian_3d`,
corgi_force_estimation/src/force_estimation.cpp:163. It is used TWICE and the
two uses fail differently:

  * force_control.cpp:178 builds J_fb from it and maps a commanded leg-frame
    force to motor torques (tau = J^T f). Ill-conditioning here DISTORTS the
    commanded impedance -- the virtual spring stops being the spring that was
    asked for.
  * ForceEstimator::estimate INVERTS it (f = -J^-T tau). Ill-conditioning here
    AMPLIFIES torque noise into the force estimate.

So sigma_min matters in its own right, not only the ratio sigma_max/sigma_min:
a uniformly small J is a different failure from a skewed one.

THE PORT IS DELIBERATE, NOT A RE-DERIVATION. Every formula below mirrors the
C++ line for line, including its conventions, so that what is measured is the
conditioning of THE SHIPPED CONTROLLER and not of a tidier model someone wrote
afterwards. Where the C++ makes a modelling choice this file reproduces the
choice and then measures its consequence separately (see CONVENTIONS below).

CONVENTIONS, and the one that is a live open question:

  * The 2D contact point is P(theta, alpha) = O_r + (G - O_r)*(foot_radius/R)
    *exp(i*alpha), i.e. the rim point at contact angle alpha about the rim
    centre O_r -- calculate_P_poly_3d's rim-1 branch, with rims 2 and 3 for
    |alpha| > 40 deg.
  * contact_map_3d sets alpha = (slope - beta) in DEGREES, so at zero slope
    alpha = -beta.
  * calculate_jacobian then applies a rotation by +beta to P. It differentiates
    ONLY that explicit rotation: dP/dbeta = i*exp(i*beta)*P, treating P itself
    as constant in beta. That is the "contact rotates RIGIDLY with beta"
    convention Open Issue #3 flags -- against slip_rf, the model the templates
    are solved from, which uses a ROLLING contact. Both are computed here; see
    `rolling_vs_rigid()`.
  * LegKinematics.rotate() adds beta0 = 90 deg, which the C++ Jacobian does
    not. A constant rotation changes no singular value, so conditioning is
    unaffected; it does rotate the singular VECTORS, which is why directions
    below are reported in the C++ frame.

Run:  python examples/gslip/stage3_jacobian_camber.py --selftest
      python examples/gslip/stage3_jacobian_camber.py
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from legwheel.models.leg_kinematics import LegKinematics  # noqa: E402

# ---- constants, mirrored from corgi_utils/src/leg_model.cpp -----------------
R = 0.100                 # WHEEL_RADIUS_PITCH, linkage joint circle
FOOT_OFFSET = 0.030       # TIRE_TREAD_RADIUS - R
TYRE_THICKNESS = 0.015    # TIRE_CORNER_RADIUS
FOOT_RADIUS = R + FOOT_OFFSET + TYRE_THICKNESS          # 0.145
SCALED_RADIUS = FOOT_RADIUS / R                          # 1.45
WHEEL_THICKNESS = 0.04
ABAD_AXIS_TO_WHEEL_PLANE = 0.091675   # the 91.7 mm in the contact-migration term

# slip_rf's rolling contact radius -- the model the templates are solved from.
SLIP_RF_R = 0.145

# Both spellings: this runs under WSL (the venv is Linux-side) but the repo
# lives on the Windows filesystem, and the template lives in the WSL repo.
TEMPLATE_CANDIDATES = [
    "/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_force_control/config"
    "/gslip_pronk_template_v070.csv",
    r"\\wsl.localhost\Ubuntu-22.04\home\alexc\corgi_ws\corgi_ros2_ws\src"
    r"\corgi_force_control\config\gslip_pronk_template_v070.csv",
]


# ---- kinematics ------------------------------------------------------------

# THE SHIPPED FITTED POLYNOMIALS, copied verbatim from
# corgi_utils/include/corgi_utils/fitted_coefficient.hpp. Using these rather
# than LegKinematics is deliberate: force_estimation.cpp sets
#     G_coef_(0,i)   = 0;   G_coef_(1,i)   = G_y_coef[i];
#     O_r_coef_(0,i) = 0;   O_r_coef_(1,i) = O_y_coef[i];
# i.e. its unrotated frame puts the leg on the Y axis, where LegKinematics puts
# it on the real axis (LegKinematics.rotate() carries beta0 = 90 deg to undo
# exactly that). Porting through LegKinematics therefore lands 90 deg out and
# makes z_2D identically zero, which makes the 3x3 exactly singular -- a
# spectacular-looking result that is purely a frame error. Ask me how I know.
G_Y_COEF = [-0.08004472811678946, -0.04301096555457295, -0.10580886132752444,
            0.0888545682810313, -0.031030861225472762, -0.0011104867548842852,
            0.0030345590247493667, -0.00046519990417785516]
O_Y_COEF = [0.019955276873480345, -0.04301098231885723, -0.10580886055310057,
            0.08885462301612962, -0.03103094211108991, -0.0011104365298488666,
            0.0030345444432208963, -0.00046519828008510805]


def _poly(coef, x):
    return sum(c * x ** i for i, c in enumerate(coef))


def _dpoly(coef, x):
    return sum(i * c * x ** (i - 1) for i, c in enumerate(coef) if i > 0)


def p_contact_2d(theta: float, alpha_deg: float) -> complex:
    """-> 2D contact point in the C++ unrotated leg frame, as complex (x + i z).

    Mirrors calculate_P_poly_3d's rim-1 branch:
        P_poly = Rot(alpha) * (G_coef - O_r_coef) * scaled_radius + O_r_coef
    with G and O_r both purely on the y axis. (G_y - O_y) is -0.100 for every
    theta -- the rim centre sits exactly R above the foot point -- so the
    contact lands foot_radius from O_r, as the selftest checks.

    Rims 2 and 3 (|alpha| > 40 deg) are NOT implemented: alpha = -beta and the
    v070 template sweeps beta by at most +-9.3 deg, so the running gait never
    leaves rim 1. Refusing loudly beats silently extrapolating a branch this
    study was not validated on.
    """
    a_mod = np.fmod(alpha_deg + 180.0, 360.0) - 180.0
    if not (-40.0 <= a_mod <= 40.0):
        raise ValueError(
            f"alpha {alpha_deg:.2f} deg leaves rim 1 (|alpha| <= 40). Rims 2/3 "
            "are not ported; the running gait never reaches them.")
    Gy = _poly(G_Y_COEF, theta)
    Oy = _poly(O_Y_COEF, theta)
    rot = np.exp(1j * np.deg2rad(alpha_deg))
    return rot * (1j * (Gy - Oy)) * SCALED_RADIUS + 1j * Oy


def dP_dtheta(theta: float, alpha_deg: float) -> complex:
    """Analytic d/dtheta of the same polynomial -- exactly what the C++ does
    via P_poly_deriv, not a finite difference."""
    dGy = _dpoly(G_Y_COEF, theta)
    dOy = _dpoly(O_Y_COEF, theta)
    rot = np.exp(1j * np.deg2rad(alpha_deg))
    return rot * (1j * (dGy - dOy)) * SCALED_RADIUS + 1j * dOy


def d_wheel(gamma: float) -> float:
    """Mirrors contact_map_3d step 4.

    NOTE THE DISCONTINUITY. contact_edge_offset is -half_wheel_width for
    gamma > 0, +half_wheel_width for gamma < 0, and ZERO inside |sin gamma| <
    1e-4. So d_wheel steps 0.1117 -> 0.0917 -> 0.0717 across gamma = 0: a 40 mm
    jump, 44% of the nominal offset, exactly where the sagittal gait lives.
    This is physical in intent -- the contact hops from one wheel edge to the
    other as the wheel tips -- but it is a STEP, and it lands in the Jacobian's
    third column.
    """
    sin_g = np.sin(gamma)
    edge = 0.0
    if abs(sin_g) >= 1e-4:
        edge = -WHEEL_THICKNESS / 2.0 if sin_g > 0 else WHEEL_THICKNESS / 2.0
    return ABAD_AXIS_TO_WHEEL_PLANE + edge


def jacobian_planar(P: complex, dP: complex, beta: float) -> np.ndarray:
    """Mirrors calculate_jacobian (2x2, d(Px,Pz)/d(phi_L,phi_R))."""
    cb, sb = np.cos(beta), np.sin(beta)
    dtheta_dphiR, dtheta_dphiL = -0.5, 0.5
    dbeta_dphiR, dbeta_dphiL = 0.5, 0.5

    dPx_dtheta = dP.real * cb - dP.imag * sb
    dPy_dtheta = dP.real * sb + dP.imag * cb
    dPx_dbeta = P.real * (-sb) - P.imag * cb
    dPy_dbeta = P.real * cb + P.imag * (-sb)

    return np.array([
        [dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL,
         dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR],
        [dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL,
         dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR],
    ])


def jacobian_3d(P: complex, dP: complex, beta: float, gamma: float,
                dw: float | None = None, z_rotated: bool = False) -> np.ndarray:
    """Mirrors calculate_jacobian_3d (3x3, d(X,Y,Z)/d(phi_L,phi_R,gamma)).

    z_rotated=False reproduces THE SHIPPED CODE, which sets

        double z_2D = P_theta(1,0);          // force_estimation.cpp:189

    i.e. the UNROTATED leg-frame z. z_rotated=True uses Im(exp(i*beta)*P),
    the beta-ROTATED z that force_control.cpp's own forward model uses two
    lines before it calls this function:

        p_beta_z     = p_poly_x*sin_beta + p_poly_z*cos_beta;   // ROTATED
        p_expected_y = d_wheel*cos_gamma - p_beta_z*sin_gamma;  // :171

    The two agree only at beta = 0. See `z_convention_gap()` and log S173.
    """
    Jp = jacobian_planar(P, dP, beta)
    J11, J12 = Jp[0, 0], Jp[0, 1]
    J21, J22 = Jp[1, 0], Jp[1, 1]
    sg, cg = np.sin(gamma), np.cos(gamma)
    z_2D = (np.exp(1j * beta) * P).imag if z_rotated else P.imag
    if dw is None:
        dw = d_wheel(gamma)
    return np.array([
        [J11, J12, 0.0],
        [-J21 * sg, -J22 * sg, -dw * sg - z_2D * cg],
        [J21 * cg, J22 * cg, dw * cg - z_2D * sg],
    ])


def contact_3d(theta: float, beta: float, gamma: float,
               rolling: bool = False) -> np.ndarray:
    """-> (X, Y, Z) contact position, mirroring contact_map_3d.

    rolling=False reproduces the controller: alpha = -beta AND an explicit
    rotation by +beta. rolling=True is the same construction with the contact
    angle frozen, i.e. the contact carried rigidly -- used only to expose what
    the Jacobian's dP/dbeta actually assumes.
    """
    alpha_deg = np.rad2deg(-beta)
    P = p_contact_2d(theta, 0.0 if rolling else alpha_deg)
    Prot = np.exp(1j * beta) * P
    dw = d_wheel(gamma)
    sg, cg = np.sin(gamma), np.cos(gamma)
    return np.array([Prot.real, dw * cg - Prot.imag * sg, dw * sg + Prot.imag * cg])


# ---- metrics ---------------------------------------------------------------

def metrics(J: np.ndarray) -> dict:
    sv = np.linalg.svd(J, compute_uv=False)
    U, S, Vt = np.linalg.svd(J)
    return {
        "cond": float(sv[0] / sv[-1]) if sv[-1] > 0 else np.inf,
        "sigma_max": float(sv[0]),
        "sigma_min": float(sv[-1]),
        "det": float(abs(np.linalg.det(J))),
        # The input direction that the map shrinks most -- i.e. the commanded
        # force direction that costs the most torque to realise.
        "worst_in": Vt[-1, :].copy(),
        "worst_out": U[:, -1].copy(),
    }


# ---- self-test -------------------------------------------------------------

def selftest() -> int:
    """No analyser is trusted before it has failed to fool itself."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
        if not cond:
            ok = False

    print("stage3_jacobian_camber selftest")

    # 1. The rim construction must put the contact exactly foot_radius from the
    #    rim centre. This is the one number the whole port rests on.
    th = np.deg2rad(100.0)
    O_r = 1j * _poly(O_Y_COEF, th)
    P = p_contact_2d(th, 0.0)
    check("contact sits foot_radius from rim centre",
          abs(abs(P - O_r) - FOOT_RADIUS) < 1e-9,
          f"|P-O_r| = {abs(P - O_r):.6f} vs {FOOT_RADIUS}")

    # 2. Known answer from log S21: at theta = 100 deg the hip->contact
    #    distance is 0.293 m and hip->rim-centre is 0.1481 m.
    check("hip->contact reproduces S21's 0.293 m at theta=100",
          abs(abs(P) - 0.293) < 5e-4, f"|P| = {abs(P):.4f}")
    check("hip->rim-centre reproduces S21's 0.1481 m at theta=100",
          abs(abs(O_r) - 0.1481) < 5e-4, f"|O_r| = {abs(O_r):.4f}")

    # 2b. The fitted polynomials must agree with the independent Python
    #     kinematics, up to the 90 deg frame offset. Two implementations,
    #     one number -- this is what catches a mistyped coefficient.
    kin = LegKinematics()
    kin.theta = np.array(float(th))
    kin.beta = np.array(0.0)
    kin.n_elements = 0
    kin.calculate()
    check("fitted O_r matches LegKinematics (rotated 90 deg)",
          abs(abs(complex(kin.O_r)) - abs(O_r)) < 5e-4,
          f"LegKinematics |O_r| = {abs(complex(kin.O_r)):.4f}")
    check("z_2D is NOT zero (the frame error that would fake a singular J)",
          abs(P.imag) > 0.2, f"z_2D = {P.imag:+.4f}")

    # 2c. The rim-1 restriction must refuse rather than extrapolate.
    try:
        p_contact_2d(th, 55.0)
        check("rim-1 restriction refuses alpha > 40 deg", False)
    except ValueError:
        check("rim-1 restriction refuses alpha > 40 deg", True)

    # 3. gamma = 0 must recover the planar Jacobian in rows 1 and 3.
    #    (Row 2 becomes [0, 0, -z_2D]: at zero camber the ABAD moves the
    #    contact laterally and the leg motors cannot.)
    b = 0.1
    dP = dP_dtheta(th, np.rad2deg(-b))
    Pb = p_contact_2d(th, np.rad2deg(-b))
    Jp = jacobian_planar(Pb, dP, b)
    J3 = jacobian_3d(Pb, dP, b, 0.0)
    check("gamma=0: row 1 is the planar x-row",
          np.allclose(J3[0, :2], Jp[0, :], atol=1e-12))
    check("gamma=0: row 3 is the planar z-row",
          np.allclose(J3[2, :2], Jp[1, :], atol=1e-12))
    check("gamma=0: leg motors produce no lateral motion",
          np.allclose(J3[1, :2], [0.0, 0.0], atol=1e-12))

    # 4. FINITE-DIFFERENCE CHECK against something that does not share the
    #    algebra: force_control.cpp's OWN forward model, lines 165-172. That
    #    model rotates z by beta before the gamma rotation, so the FD of it is
    #    the Jacobian the controller's own kinematics implies.
    g = np.deg2rad(12.0)
    h = 1e-7

    def fk_controller(phiL, phiR, gam):
        """force_control.cpp:165-172 exactly, with alpha frozen (the
        Jacobian's own rigid-contact assumption)."""
        theta_ = (phiL - phiR) / 2.0 + np.deg2rad(17.0)
        beta_ = (phiL + phiR) / 2.0
        Pl = p_contact_2d(theta_, np.rad2deg(-b))
        Pr = np.exp(1j * beta_) * Pl        # p_beta_x, p_beta_z
        dw = d_wheel(gam)
        return np.array([Pr.real,
                         dw * np.cos(gam) - Pr.imag * np.sin(gam),
                         dw * np.sin(gam) + Pr.imag * np.cos(gam)])

    phiL0 = b + th - np.deg2rad(17.0)
    phiR0 = b - th + np.deg2rad(17.0)
    Jfd = np.zeros((3, 3))
    for k, pert in enumerate("LRG"):
        dL = h if pert == "L" else 0.0
        dR = h if pert == "R" else 0.0
        dG = h if pert == "G" else 0.0
        Jfd[:, k] = (fk_controller(phiL0 + dL, phiR0 + dR, g + dG)
                     - fk_controller(phiL0 - dL, phiR0 - dR, g - dG)) / (2 * h)

    J_rot = jacobian_3d(Pb, dP, b, g, z_rotated=True)
    err_rot = float(np.max(np.abs(Jfd - J_rot)))
    check("z_rotated=True matches the FD of force_control's own FK",
          err_rot < 1e-5, f"max|dJ| = {err_rot:.2e}")

    # 5. The shipped z_2D is formally the unrotated one where force_control's
    #    forward model uses the rotated one -- but because alpha = -beta, the
    #    rim contact counter-rotates and the two nearly cancel in the imaginary
    #    part. The residual is SECOND order in beta and inert at gait
    #    amplitudes. Asserted as a bound so it fires if a future template
    #    sweeps beta far enough to matter.
    #
    #    ⚠ An earlier version of this file asserted the OPPOSITE -- that the
    #    gap was large (2.9e-2) and a real defect. That was a 90 deg frame
    #    error in the port, not a defect in the controller. The number only
    #    became trustworthy once z_2D was checked against a known answer.
    J_ship = jacobian_3d(Pb, dP, b, g, z_rotated=False)
    err_ship = float(np.max(np.abs(Jfd - J_ship)))
    check("shipped vs rotated z_2D is negligible at gait beta (< 1e-3 m/rad)",
          err_ship < 1e-3, f"max|dJ| = {err_ship:.3e} m/rad at beta = {b:.3f}")
    check("...and the two conventions coincide at beta = 0 (so it is beta-driven)",
          float(np.max(np.abs(jacobian_3d(p_contact_2d(th, 0.0),
                                          dP_dtheta(th, 0.0), 0.0, g, z_rotated=True)
                              - jacobian_3d(p_contact_2d(th, 0.0),
                                            dP_dtheta(th, 0.0), 0.0, g,
                                            z_rotated=False)))) < 1e-12)

    # 6. THE TEST MUST BITE. A wrong d_wheel has to fail check 4.
    Jbad = jacobian_3d(Pb, dP, b, g, dw=d_wheel(g) + 0.01, z_rotated=True)
    check("a 10 mm wrong d_wheel breaks that match (the test bites)",
          float(np.max(np.abs(Jfd - Jbad))) > 1e-3)

    # 7. The 2x2 planar block's singular values are invariant to the beta0
    #    frame offset. (The FULL 3x3 is NOT: its gamma column mixes a
    #    frame-dependent z_2D with a frame-independent d_wheel, so rotating
    #    the frame genuinely changes its conditioning. An earlier version of
    #    this test asserted 3x3 invariance and was simply wrong.)
    r90 = np.exp(1j * np.deg2rad(90))
    sv_a = np.linalg.svd(jacobian_planar(Pb, dP, b), compute_uv=False)
    sv_b = np.linalg.svd(jacobian_planar(Pb * r90, dP * r90, b), compute_uv=False)
    check("planar 2x2 conditioning is invariant to the beta0 frame offset",
          np.allclose(sv_a, sv_b, atol=1e-12))

    print("selftest", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# ---- studies ---------------------------------------------------------------

def load_template(path):
    """-> list of (theta, beta) for the STANCE rows only."""
    rows = []
    with open(path, newline="") as fh:
        for rec in csv.DictReader(fh):
            if int(float(rec["in_stance"])):
                rows.append((float(rec["theta"]), float(rec["beta"])))
    return rows


def gamma_sweep(theta, beta, gammas):
    out = []
    alpha_deg = np.rad2deg(-beta)
    P = p_contact_2d(theta, alpha_deg)
    dP = dP_dtheta(theta, alpha_deg)
    for g in gammas:
        m = metrics(jacobian_3d(P, dP, beta, g))
        m["gamma"] = g
        out.append(m)
    return out


def rolling_vs_rigid(theta, beta, gamma):
    """Open Issue #3's live sub-question, priced.

    The controller's Jacobian assumes dP/dbeta = i*exp(i*beta)*P -- the contact
    carried RIGIDLY round with the leg, lever arm |P|. slip_rf, the model the
    templates are solved from, has a ROLLING contact advancing at r = 0.145.
    The tangential lever arms differ by |P| / 0.145.
    """
    alpha_deg = np.rad2deg(-beta)
    P = p_contact_2d(theta, alpha_deg)
    dP = dP_dtheta(theta, alpha_deg)
    J_rigid = jacobian_3d(P, dP, beta, gamma)

    # Rolling: the contact angle tracks beta (alpha = -beta), so the explicit
    # rotation is cancelled at the rim and only the rim centre swings. Build
    # dP/dbeta from that instead.
    h = 1e-7
    def rolled(bv):
        return np.exp(1j * bv) * p_contact_2d(theta, np.rad2deg(-bv))
    dPdbeta_roll = (rolled(beta + h) - rolled(beta - h)) / (2 * h)
    dPdtheta_rot = np.exp(1j * beta) * dP

    dtheta_dphiL, dtheta_dphiR = 0.5, -0.5
    dbeta_dphiL = dbeta_dphiR = 0.5
    J11 = dPdtheta_rot.real * dtheta_dphiL + dPdbeta_roll.real * dbeta_dphiL
    J12 = dPdtheta_rot.real * dtheta_dphiR + dPdbeta_roll.real * dbeta_dphiR
    J21 = dPdtheta_rot.imag * dtheta_dphiL + dPdbeta_roll.imag * dbeta_dphiL
    J22 = dPdtheta_rot.imag * dtheta_dphiR + dPdbeta_roll.imag * dbeta_dphiR
    sg, cg = np.sin(gamma), np.cos(gamma)
    dw, z2 = d_wheel(gamma), (np.exp(1j * beta) * P).imag
    J_roll = np.array([
        [J11, J12, 0.0],
        [-J21 * sg, -J22 * sg, -dw * sg - z2 * cg],
        [J21 * cg, J22 * cg, dw * cg - z2 * sg],
    ])
    return metrics(J_rigid), metrics(J_roll), abs(P) / SLIP_RF_R


def z_convention_gap(theta, beta, gamma):
    """-> (abs gap in the gamma column, relative gap, |z_rot - z_unrot|).

    The shipped Jacobian's gamma column is built from the UNROTATED leg-frame
    z; force_control.cpp's own forward model, two lines earlier, uses the
    beta-ROTATED z. Both entries that carry gamma are affected:
        J[1,2] = -d_wheel*sin g - z*cos g
        J[2,2] =  d_wheel*cos g - z*sin g
    and the whole discrepancy is (z_rot - z_unrot), which is O(|P|*beta).
    """
    alpha_deg = np.rad2deg(-beta)
    P = p_contact_2d(theta, alpha_deg)
    dP = dP_dtheta(theta, alpha_deg)
    Js = jacobian_3d(P, dP, beta, gamma, z_rotated=False)
    Jr = jacobian_3d(P, dP, beta, gamma, z_rotated=True)
    gap = float(np.max(np.abs(Js - Jr)))
    scale = float(np.max(np.abs(Jr[:, 2])))
    dz = abs((np.exp(1j * beta) * P).imag - P.imag)
    return gap, (gap / scale if scale > 0 else np.inf), dz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--template", default=None)
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    if selftest() != 0:
        print("\nREFUSING to report: selftest failed.")
        sys.exit(1)

    cands = [args.template] if args.template else TEMPLATE_CANDIDATES
    stance = None
    for c in cands:
        try:
            stance = load_template(c)
            print(f"\ntemplate: {c}")
            break
        except OSError:
            continue
    if stance is None:
        # REFUSE rather than silently substituting nominal poses. The first
        # version fell back to beta = 0 for every row, which made the
        # z-convention study report a flat 0.0% gap -- the gap is O(beta), so
        # the fallback quietly answered "no problem" to the question being
        # asked. A study that cannot read its input must say so.
        print("\n!! cannot read the v070 template from any candidate path:")
        for c in cands:
            print(f"     {c}")
        print("!! REFUSING to report -- the beta range IS the study (S173).")
        sys.exit(1)

    thetas = [r[0] for r in stance]
    betas = [r[1] for r in stance]
    print(f"\ntemplate stance rows: {len(stance)}, "
          f"theta {np.rad2deg(min(thetas)):.1f}-{np.rad2deg(max(thetas)):.1f} deg, "
          f"beta {np.rad2deg(min(betas)):+.1f}-{np.rad2deg(max(betas)):+.1f} deg")

    # --- 1. conditioning vs camber, at representative stance poses ----------
    print("\n=== 1. cond(J3) vs camber, at template stance poses ===")
    idx = [0, len(stance) // 2, len(stance) - 1]
    print(f"{'gamma':>7}", end="")
    for i in idx:
        print(f"   th={np.rad2deg(stance[i][0]):5.1f} b={np.rad2deg(stance[i][1]):+5.1f}", end="")
    print()
    for gd in (0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
        g = np.deg2rad(gd)
        print(f"{gd:7.1f}", end="")
        for i in idx:
            th, be = stance[i]
            alpha_deg = np.rad2deg(-be)
            P = p_contact_2d(th, alpha_deg)
            dP = dP_dtheta(th, alpha_deg)
            print(f"   {metrics(jacobian_3d(P, dP, be, g))['cond']:19.3f}", end="")
        print()

    # --- 2. the worst pose over the whole stance, per camber ----------------
    print("\n=== 2. worst-conditioned stance row at each camber ===")
    print(f"{'gamma':>7} {'max cond':>10} {'min sigma':>11} {'min |det|':>11} "
          f"{'at theta':>9} {'at beta':>8}")
    curve = []
    for gd in (0.0, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
        g = np.deg2rad(gd)
        worst_c, worst_s, worst_d, at = 0.0, np.inf, np.inf, None
        for th, be in stance:
            alpha_deg = np.rad2deg(-be)
            m = metrics(jacobian_3d(p_contact_2d(th, alpha_deg),
                                    dP_dtheta(th, alpha_deg), be, g))
            if m["cond"] > worst_c:
                worst_c, at = m["cond"], (th, be)
            worst_s = min(worst_s, m["sigma_min"])
            worst_d = min(worst_d, m["det"])
        curve.append((gd, worst_c, worst_s, worst_d))
        print(f"{gd:7.1f} {worst_c:10.3f} {worst_s:11.5f} {worst_d:11.6f} "
              f"{np.rad2deg(at[0]):9.1f} {np.rad2deg(at[1]):+8.1f}")

    # --- 3. the d_wheel step at gamma = 0 -----------------------------------
    print("\n=== 3. the d_wheel STEP at gamma = 0 (contact_map_3d step 4) ===")
    print("d_wheel jumps as the contact hops between wheel edges. The sagittal")
    print("gait sits AT gamma = 0, i.e. exactly on the step.")
    th, be = stance[len(stance) // 2]
    alpha_deg = np.rad2deg(-be)
    P, dP = p_contact_2d(th, alpha_deg), dP_dtheta(th, alpha_deg)
    print(f"{'gamma (deg)':>12} {'d_wheel':>9} {'cond':>10} {'sigma_min':>11}")
    for gd in (-1.0, -0.1, -0.001, 0.0, 0.001, 0.1, 1.0):
        g = np.deg2rad(gd)
        m = metrics(jacobian_3d(P, dP, be, g))
        print(f"{gd:12.3f} {d_wheel(g):9.6f} {m['cond']:10.3f} {m['sigma_min']:11.5f}")

    # --- 4. rigid vs rolling contact (Open Issue #3) ------------------------
    print("\n=== 4. contact convention: controller (RIGID) vs slip_rf (ROLLING) ===")
    print(f"{'gamma':>7} {'cond rigid':>11} {'cond roll':>11} "
          f"{'sig_min rigid':>14} {'sig_min roll':>13} {'lever ratio':>12}")
    for gd in (0.0, 5.0, 10.0, 15.0, 20.0, 30.0):
        g = np.deg2rad(gd)
        mr, mo, lever = rolling_vs_rigid(th, be, g)
        print(f"{gd:7.1f} {mr['cond']:11.3f} {mo['cond']:11.3f} "
              f"{mr['sigma_min']:14.5f} {mo['sigma_min']:13.5f} {lever:12.3f}")

    # --- 5. which commanded force direction is worst served -----------------
    print("\n=== 5. worst-served direction at gamma = 15 deg ===")
    g = np.deg2rad(15.0)
    m = metrics(jacobian_3d(P, dP, be, g))
    print(f"  sigma_min {m['sigma_min']:.5f}, sigma_max {m['sigma_max']:.5f}, "
          f"cond {m['cond']:.3f}")
    print(f"  worst input  (phi_L, phi_R, gamma) = "
          f"[{m['worst_in'][0]:+.4f} {m['worst_in'][1]:+.4f} {m['worst_in'][2]:+.4f}]")
    print(f"  worst output (X, Y, Z)             = "
          f"[{m['worst_out'][0]:+.4f} {m['worst_out'][1]:+.4f} {m['worst_out'][2]:+.4f}]")

    # --- 6. the z-convention defect, priced over the template's own beta ----
    print("\n=== 6. ⭐ the shipped z_2D convention vs force_control's own FK ===")
    print("calculate_jacobian_3d uses the UNROTATED leg-frame z in its gamma")
    print("column; force_control.cpp:167-172 uses the beta-ROTATED z in the")
    print("forward model it feeds. They agree only at beta = 0.")
    print(f"{'beta (deg)':>11} {'|dz| (m)':>10} {'gap (m/rad)':>12} {'% of col':>9}")
    bmin, bmax = min(betas), max(betas)
    for bd in sorted({round(np.rad2deg(b), 2) for b in
                      (0.0, bmin, bmin / 2, bmax / 2, bmax)}):
        bb = np.deg2rad(bd)
        gap, rel, dz = z_convention_gap(th, bb, np.deg2rad(10.0))
        print(f"{bd:11.2f} {dz:10.5f} {gap:12.5f} {100 * rel:8.1f}%")
    worst = max(z_convention_gap(t_, b_, np.deg2rad(10.0))[1]
                for t_, b_ in stance)
    print(f"  worst over all {len(stance)} stance rows at gamma=10 deg: "
          f"{100 * worst:.1f}% of the camber column")

    print("\nNumbers above are the analyser's output. Any figure built from")
    print("them hardcodes them; nothing is re-derived in a plot script.")


if __name__ == "__main__":
    main()
