"""Coronal roll subsystem -- BIP after Chang 2022, with cambered contacts.

Stage 2b Module 2. The lateral state exists to carry the ROLL INSTABILITY that
clocked torque must fix (Chang 2022 Fig. 13; Sovukluk 2024 agrees by a different
mechanism), not to model a second spring-mass system. Chang's BIP is the
precedent: a rigid bar with a spring leg per side, a handful of states, no
lateral SLIP anywhere.

WHERE THIS DEPARTS FROM CHANG, AND WHY THAT IS THE POINT

Chang's BIP applies VERTICAL spring forces at contacts pinned laterally at +/-w;
roll enters only through the 2*w*sin(rho) height difference (their eq 20/24).
That is exactly the assumption this thesis's gap lives in. Here:

  1. Contacts sit OUTBOARD of the hips (the wheel planes are at +/-0.2117 m on
     the contact track, hips at +/-0.12 m), so each leg is tilted and the spring
     force acts ALONG the leg, hip-to-contact. A tilted leg force has a lateral
     component and a roll moment that vertical springs cannot represent.
  2. The contact offset and the effective rolling radius are functions of the
     wheel's lean, supplied through one seam (`side_geometry`) so the Stage 1
     validation lands in a single function.

States: (y, z, rho, vy, vz, vrho) -- lateral position, height, roll, and rates.
y is free (Chang constrains the CoM to z; the plan keeps v_y because a turn is
lateral translation). Legs are unilateral springs: force engages when the
hip-to-contact distance is below the side's rest length, which makes the four
Chang phases (double / left / right / flight) emerge from the same RHS without
event bookkeeping.

CONTACT MODEL CAVEAT (v1, deliberate)

The contact point is placed quasi-statically at the current hip position plus
the outboard offset -- it slides with the hip rather than pinning at touchdown.
This under-constrains lateral motion (a real wheel resists lateral scrub) and
makes the tilted-leg force non-conservative under lateral CoM motion; for the
symmetric vertical bounce it is exactly conservative, which is what the energy
test exercises. Pinned-at-touchdown contact is a later refinement and belongs
with the Stage 1 geometry validation.

Parameters come from measured sources: m and the coronal half-track from Stage 0,
I_roll = 0.6119 kg m^2 from the proto walk (implementation log section 41,
J~ = 0.44), k per side = 2 legs * 8941 N/m.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, replace

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

from legwheel.models.slip_rf_cambered import rolling_radius

G_DEFAULT = 9.81

# Measured geometry (Stage 0 / config). Module constants, same convention as
# slip_rf_cambered: self-contained, overridable per instance.
HALF_TRACK_HIP = 0.120        # hip lateral offset from centreline (m)
WHEEL_AXIAL_OFFSET = 0.091675  # wheel plane outboard of the hip (m)
LEG_LENGTH_NOMINAL = 0.293    # hip-to-contact at the theta = 100 deg stance (m)
K_SIDE_DEFAULT = 2 * 8941.0   # two sagittal legs per coronal side (N/m)
I_ROLL_DEFAULT = 0.611906     # kg m^2, from the proto (section 41)
MASS_DEFAULT = 30.0           # kg, scale reading

# WHICH ROLLING-RADIUS LAW side_geometry USES. This is a plant choice, not a
# tuning constant -- changing it moves every cambered Stage 2b number.
#
#   "measured"  radius is lambda-INDEPENDENT at r0 = 0.14482 m. THE DEFAULT
#               since 2026-08-23 (S185). Stage 1.5 / S75 measured it flat over
#               0-40 deg in sim; the sim's tread is not a smooth torus.
#   "torus"     radius shrinks as rolling_radius(lean). Correct for a smooth
#               torus, which is the Stage 0 DESIGN geometry and may be the
#               right law for hardware -- kept for exactly that reason.
#
# WHY "measured" IS THE DEFAULT. It is the law the simulator actually exhibits,
# and sim is what Stage 3 validates against; this project's record is that
# measurement beats derivation every time the two have disagreed (erosion, the
# flight gate, the alpha spread). NOT because it makes a test pass -- it does,
# and that is a consequence, not the argument.
#
# WHAT IT COSTS, so nobody rediscovers it: under "measured" the Ackermann
# family closes at lam_in 5 and 10 deg and NOT at 15 (residual 1.1e-02). That
# is a model-space lean ceiling and it wants checking against Stage 4's
# lambda in {0,5,10,15,20} matrix before it is believed. S184, S185.
#
# BOTH ARE KEPT. Stage 2a registers the choice as sensitivity check E3, so the
# thesis reports the band rather than a point. Overridable by env var so the
# SAME code runs under both -- a sensitivity check whose two arms differ by a
# source edit is not a sensitivity check.
RADIUS_LAW_DEFAULT = os.environ.get("LEGWHEEL_RADIUS_LAW", "measured")
if RADIUS_LAW_DEFAULT not in ("torus", "measured"):
    raise ValueError(f"LEGWHEEL_RADIUS_LAW must be 'torus' or 'measured', "
                     f"got {RADIUS_LAW_DEFAULT!r}")


@dataclass
class SideGeometry:
    """Contact geometry for one side, as a function of that side's wheel lean.

    THE SEAM. Stage 1 validates (or corrects) the contact model; the correction
    lands here and nowhere else.

    Args:
        d_out: lateral offset, hip to contact, positive outboard (m)
        l0: leg rest length, hip to contact (m)
    """

    d_out: float
    l0: float


def side_geometry(lean: float,
                  d_out0: float = WHEEL_AXIAL_OFFSET,
                  l0_sagittal: float = LEG_LENGTH_NOMINAL,
                  radius_law: str = RADIUS_LAW_DEFAULT) -> SideGeometry:
    """Default cambered side geometry at wheel lean `lean` (rad, signed;
    positive leans the wheel top outboard for this side).

    d_out is the lateral offset HIP TO CONTACT, and it has two parts:

      * the wheel plane sits d_out0 along the hip axis, and leaning rotates
        that axis, so its lateral component goes as d_out0*cos(lean);
      * the contact sits a rolling radius BELOW the wheel centre, and the wheel
        pivots about its axle, so that ground point swings outboard by
        r*sin(lean).

    ⚠ CORRECTED 2026-08-23 (log S183). This function previously used
    `0.015 * sin(lean)` -- R_CORNER, the shoulder fillet -- where the rolling
    radius belongs, and the docstring's "crown migration" is what named the
    mistake: CROWN is the wheel's outer circumference (r = 0.145), CORNER is
    the 15 mm shoulder fillet. They differ by 9.7x, and the two coefficients
    answer different questions:

      r_corner*sin(lean)  migration of the contact ACROSS THE TREAD, bounded
                          by the tread half-width (~20 mm). Real, and small.
      r*sin(lean)         displacement of the contact IN SPACE relative to the
                          hip. NOT bounded by tread width -- 49.5 mm at 20 deg
                          on a 20 mm tread is not a contradiction, because it
                          is not measured across the tread.

    `d_out` is documented as hip-to-contact, i.e. the second quantity, so the
    second coefficient is the right one. The old form under-stated the lateral
    coupling ~20x at lambda = 10 deg and turned NEGATIVE by 20 deg -- a
    coupling channel that shrinks and reverses across the working band.

    RADIUS LAW -- the open half, deliberately a switch and not a decision:

      "torus"     effective rolling radius shrinks as rolling_radius(lean).
                  Correct for a smooth torus; the Stage 0 design geometry.
      "measured"  radius is lambda-INDEPENDENT. Stage 1.5 / S75 measured
                  r0 = 0.14482 m flat over 0-40 deg in sim, because the sim's
                  tread is not a smooth torus.

    Stage 2a already carries this as a registered sensitivity check (E3), not
    an assumption, so Stage 2b inherits it rather than silently picking one.
    Run both and report the band. THE SEAM (see the class docstring): Stage 1
    validates or corrects the contact model, and the correction lands here.
    """
    if radius_law not in ("torus", "measured"):
        raise ValueError(f"radius_law must be 'torus' or 'measured', "
                         f"got {radius_law!r}")
    r0 = rolling_radius(0.0)
    # One law drives both terms: the radius the contact sits at is the radius
    # it swings on. Mixing them would be a third geometry nobody validated.
    r_contact = rolling_radius(lean) if radius_law == "torus" else r0
    return SideGeometry(
        d_out=d_out0 * np.cos(lean) + r_contact * np.sin(lean),
        l0=l0_sagittal - (r0 - r_contact),
    )


@dataclass
class CoronalParams:
    """Coronal BIP parameters. Sides may differ (Chang's gamma ratio; our
    Ackermann pair and inner/outer asymmetry both need k_L != k_R support)."""

    m: float = MASS_DEFAULT
    j_roll: float = I_ROLL_DEFAULT
    w_hip: float = HALF_TRACK_HIP
    k_left: float = K_SIDE_DEFAULT
    k_right: float = K_SIDE_DEFAULT
    left: SideGeometry = None
    right: SideGeometry = None
    b: float = 0.0            # damping along the leg, N s/m
    g: float = G_DEFAULT

    def __post_init__(self) -> None:
        if self.left is None:
            self.left = side_geometry(0.0)
        if self.right is None:
            self.right = side_geometry(0.0)

    def cambered(self, lean_left: float, lean_right: float,
                 radius_law: str = RADIUS_LAW_DEFAULT) -> "CoronalParams":
        """Both sides leaned; signs are each side's own outboard convention.

        `radius_law` is threaded through to `side_geometry` so a caller can run
        the Stage 2b grids under both laws without reaching into the module
        default -- see S183 and Stage 2a's E3 sensitivity check.
        """
        return replace(self, left=side_geometry(lean_left,
                                                radius_law=radius_law),
                       right=side_geometry(lean_right, radius_law=radius_law))


def _leg_force(p: CoronalParams, s: int, y: float, z: float, rho: float,
               vy: float, vz: float, vrho: float):
    """Force on the body from side s (+1 left, -1 right), applied at the hip.

    Returns (F_y, F_z, tau_x) about the CoM. Zero when the leg is unloaded.
    """
    geom = p.left if s > 0 else p.right
    k = p.k_left if s > 0 else p.k_right

    # Hip in world coronal coordinates.
    yh = y + s * p.w_hip * np.cos(rho)
    zh = z + s * p.w_hip * np.sin(rho)
    if zh <= 0.0:
        return 0.0, 0.0, 0.0  # hip through the floor; the fall event handles it

    # Contact outboard of the hip on the ground plane (quasi-static; see
    # module docstring).
    dy = -s * geom.d_out          # contact -> hip lateral component
    length = float(np.hypot(geom.d_out, zh))
    if length >= geom.l0:
        return 0.0, 0.0, 0.0

    ux, uz = dy / length, zh / length   # unit vector contact -> hip
    # Rate of leg-length change, for damping: only zh varies the length here.
    dzh = vz + s * p.w_hip * np.cos(rho) * vrho
    dlength = (zh / length) * dzh

    f = k * (geom.l0 - length) - p.b * dlength
    if f <= 0.0:
        return 0.0, 0.0, 0.0
    fy, fz = f * ux, f * uz

    # Torque about the CoM; the force acts at the hip (massless leg).
    ry = s * p.w_hip * np.cos(rho)
    rz = s * p.w_hip * np.sin(rho)
    tau = ry * fz - rz * fy
    return fy, fz, tau


def rhs(t, state, p: CoronalParams):
    y, z, rho, vy, vz, vrho = state
    fy = fz = tau = 0.0
    for s in (+1, -1):
        a, b_, c = _leg_force(p, s, y, z, rho, vy, vz, vrho)
        fy += a
        fz += b_
        tau += c
    return [vy, vz, vrho, fy / p.m, fz / p.m - p.g, tau / p.j_roll]


def stance_state(p: CoronalParams, state) -> tuple[bool, bool]:
    """(left_loaded, right_loaded) for a state -- the Chang phase labels."""
    y, z, rho, vy, vz, vrho = state
    out = []
    for s in (+1, -1):
        f = _leg_force(p, s, y, z, rho, vy, vz, vrho)
        out.append(abs(f[1]) > 0.0)
    return tuple(out)


def energy(p: CoronalParams, state) -> float:
    """Total mechanical energy. Exact bookkeeping for the symmetric bounce;
    see the contact-model caveat for why lateral motion can leak."""
    y, z, rho, vy, vz, vrho = state
    e = 0.5 * p.m * (vy**2 + vz**2) + 0.5 * p.j_roll * vrho**2 + p.m * p.g * z
    for s in (+1, -1):
        geom = p.left if s > 0 else p.right
        k = p.k_left if s > 0 else p.k_right
        zh = z + s * p.w_hip * np.sin(rho)
        length = float(np.hypot(geom.d_out, zh))
        if length < geom.l0:
            e += 0.5 * k * (geom.l0 - length) ** 2
    return float(e)


def equilibrium_height(p: CoronalParams) -> float:
    """Static CoM height at rho = 0 (symmetric sides required)."""

    def net_fz(z):
        return rhs(0.0, [0.0, z, 0.0, 0.0, 0.0, 0.0], p)[4]

    z_top = float(np.sqrt(max(p.left.l0**2 - p.left.d_out**2, 1e-9)))
    return float(brentq(net_fz, 0.3 * z_top, z_top - 1e-9))


def simulate(p: CoronalParams, state0, t_final: float, rtol: float = 1e-10,
             atol: float = 1e-12, dense: bool = False):
    """Integrate the coronal dynamics; terminates if the CoM nears the floor."""

    def fell(t, s, _p):
        return s[1] - 0.02

    fell.terminal = True
    fell.direction = -1.0
    return solve_ivp(rhs, (0.0, t_final), state0, args=(p,),
                     events=[fell], rtol=rtol, atol=atol,
                     dense_output=dense, max_step=1e-3)


def roll_growth_per_bounce(p: CoronalParams, drop: float = 0.02,
                           n_bounce: int = 6, rho0: float = 1e-6) -> float:
    """Geometric growth factor of |rho| per bounce for a small seed roll.

    The Chang/Seipel expectation is growth (> 1) for a passive bounce; this is
    the number clocked torque has to beat. Returned as the mean factor over the
    bounces that completed before a fall.
    """
    z0 = equilibrium_height(p)
    sol = simulate(p, [0.0, z0 + drop, rho0, 0.0, 0.0, 0.0],
                   t_final=n_bounce * 2.0, dense=True)
    t = np.linspace(0.0, sol.t[-1], 4000)
    zz = sol.sol(t)
    # Apexes: local maxima of z in flight-ish regions.
    z = zz[1]
    apex_idx = [i for i in range(1, len(t) - 1)
                if z[i] > z[i - 1] and z[i] >= z[i + 1]]
    rho_at_apex = np.abs(zz[2][apex_idx])
    rho_at_apex = rho_at_apex[rho_at_apex > 0]
    if len(rho_at_apex) < 3:
        return float("nan")
    factors = rho_at_apex[1:] / rho_at_apex[:-1]
    return float(np.exp(np.mean(np.log(factors))))
