"""SLIP-RF under camber -- the coordinated-turn reduction.

Stage 2a's template, and Module 1 of Stage 2b. Deliberately thin: it adds NO
dynamics. In a steady coordinated turn the lateral side is an algebraic balance,

    tan(lambda) = v^2 / (g * R)

not a second oscillator, so the resultant of gravity and centrifugal force lies
along the leaned body axis with magnitude m*g/cos(lambda). If the leg plane leans
with the body, in-plane stance dynamics are planar SLIP-RF under an increased
effective gravity, and the existing 1-D Poincare machinery works unchanged.

THE WHOLE REDUCTION IS TWO SUBSTITUTIONS

    g -> g / cos(lambda)
    r -> r_eff(lambda)                (the rolling radius under lean)

Everything else follows automatically, which is worth stating because it is easy
to over-implement. G-SLIP's dimensionless groups are

    k~ ~ k * l0 / (m * g)             v~ ~ v / sqrt(g * l0)

so scaling g by 1/cos(lambda) *already* produces the rescalings the design note
writes out by hand -- k~ -> k~*cos(lambda) and v~ -> v~*sqrt(cos(lambda)). The
physical k and v are untouched. Implementing those separately would double-count.

Magnitudes are small: 1/cos(lambda) is 1.035 / 1.064 / 1.103 at 15 / 20 / 25 deg,
consistent with the geometric effect being first order and the dynamic effect
second order. The first-order camber physics lives in the CONTACT geometry (the
lateral migration handled in the coronal model), not here.

ROLLING RADIUS UNDER CAMBER

The Corgi's tread is not a full torus: it is flat across the middle with a
filleted shoulder. Once leaned, the contact rides the lower shoulder fillet, so
the rolling radius -- the contact's distance from the axle, which sets forward
progress per radian of wheel rotation -- is

    r_eff(lambda) = R_tread + r_corner*cos(lambda)

(contact_profile.contact_point(...).rho). At lambda = 0 this returns
R_tread + r_corner = 0.130 + 0.015 = 0.145 m EXACTLY, the sagittal foot radius --
what makes the lambda = 0 identity check below a real test of the reduction
rather than a test of rounding. Like the ideal torus it is second order (0.9 mm
at 20 deg), and it stays within 0.2 mm of the sim's lean-independent
0.14482 m to 10 deg.

CORRECTED 2026-09-13 (log s336). This used to be Stage 0's
R_tread*cos - w_flat*sin|lambda| + r_corner: a contact HEIGHT with the
flat-band term's sign reversed (the height of the shoulder that does NOT touch;
a tilted band rises), used here as a radius. Its "first-order term worth
1.71 mm at 20 deg" was that sign error. Kept as stage0_contact_height_legacy()
for the closed Stage 1 analyser only; heights now use contact_profile.axle_height.

WHAT THIS DOES NOT COVER

Roll dynamics, lateral contact migration, and the inner/outer asymmetry of a
lateral pair. Those are the coronal subsystem (Stage 2b Module 2) and they are
where the contribution actually is. This module is the sagittal half.

Also note the apex geometry (examples/gslip/camber_apex_geometry.py): UNIFORM
camber cannot roll drill-free on four contacts at all, so a physically meaningful
turning family is parameterized by an Ackermann lateral PAIR, not a single
lambda. This module models one leg of that pair; pairing is the caller's job.
"""

from __future__ import annotations

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.slip_rf import SlipRfParams

# Stage 0 tread geometry (legwheel.config.RobotParams). Kept as module constants
# rather than imported so the model stays a self-contained reduction that can be
# exercised with other tread profiles.
R_TREAD = 0.130    # torus major radius, tread arc centre
W_FLAT = 0.005     # half-width of the flat band across the tread
R_CORNER = 0.015   # torus minor radius, the shoulder fillet


def rolling_radius(lam: float, r_tread: float = R_TREAD,
                   w_flat: float = W_FLAT, r_corner: float = R_CORNER) -> float:
    """Rolling radius at camber `lam` (rad): the profile contact's distance
    from the axle, r_tread + r_corner*cos(lam) (log s336). `w_flat` moves the
    contact across the tread, not its radius; kept for signature compatibility.

    Returns r_tread + r_corner exactly at lam = 0.
    """
    return float(r_tread + r_corner * np.cos(lam))


def stage0_contact_height_legacy(lam: float, r_tread: float = R_TREAD,
                                 w_flat: float = W_FLAT,
                                 r_corner: float = R_CORNER) -> float:
    """Stage 0's contact-height formula, r_tread*cos - w_flat*sin|lam| + r_corner.

    Kept ONLY so closed-stage analysers (the Stage 1 gate) reproduce their
    record. The w_flat sign is wrong (log s336): the true axle height is
    contact_profile.axle_height = r_tread*cos + w_flat*sin|lam| + r_corner.
    """
    return float(r_tread * np.cos(lam) - w_flat * np.sin(abs(lam)) + r_corner)


def cambered_params(p: SlipRfParams, lam: float,
                    r_tread: float = R_TREAD, w_flat: float = W_FLAT,
                    r_corner: float = R_CORNER) -> SlipRfParams:
    """Rescale SLIP-RF parameters for a steady banked turn at camber `lam`.

    The mass-to-arc-centre distance (p.hip_height) is a physical leg extension
    and does not change with lean; the arc radius does, so l0 moves with it.

    At lam = 0 this returns parameters equal to `p` to within floating point --
    the identity the Stage 2a sanity check rests on.
    """
    hip = p.l0 - p.r
    r_new = rolling_radius(lam, r_tread, w_flat, r_corner)
    return SlipRfParams(
        m=p.m,
        l0=hip + r_new,
        k=p.k,                       # physical stiffness is untouched; see module docstring
        r=r_new,
        g=p.g / np.cos(lam),
    )


def cambered_stride(p: SlipRfParams, v: float, alpha: float, beta: float,
                    lam: float = 0.0) -> dict:
    """One stride of the cambered template. Signature-compatible with `stride`.

    Bind `lam` (e.g. with functools.partial) to get a `stride_fn` that drops
    straight into `gslip_fixed_point.find_fixed_points`.
    """
    out = slip_rf.stride(cambered_params(p, lam), v, alpha, beta)
    out["lam"] = float(lam)
    return out


def cambered_next_touchdown(p: SlipRfParams, v: float, alpha: float,
                            beta: float, lam: float = 0.0) -> tuple[float, float]:
    """Cheap Poincare map for root-finding; mirrors `slip_rf.next_touchdown`."""
    return slip_rf.next_touchdown(cambered_params(p, lam), v, alpha, beta)


def turn_radius(v: float, lam: float, g: float = 9.81) -> float:
    """Coordinated-turn radius R = v^2 / (g tan lambda), metres.

    Point-mass balance, no tyre model -- a feasibility check, not a prediction.
    Infinite at lam = 0.
    """
    t = np.tan(lam)
    return float("inf") if abs(t) < 1e-15 else float(v**2 / (g * t))
