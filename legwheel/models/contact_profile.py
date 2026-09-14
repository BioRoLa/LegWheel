"""Cambered contact point: tread profile -> contact in the leg frame -> hip frame.

ONE derivation of where a leaned rim touches flat ground. Used by
`coronal_bip.side_geometry` (the return map's lateral offset) and by the
Stage 2a AB/AD hold check C4. Added 2026-09-13 (log s335), after a review of
the ICRA paper's Eq. (4) found the finite crown counted in the wrong frame --
see WHAT WAS WRONG below.

FRAMES (front view, one side, coronal_bip's per-side outboard convention)

  hip frame   origin on the AB/AD axis at the hip; y outboard, z up.
  lean phi    rotation of the leg about the fore-aft AB/AD axis. phi > 0
              swings the contact OUTBOARD (log s274 s1, verified by eye).
  leg frame   a = (cos phi, sin phi): along the axle, outboard.
              n = (sin phi, -cos phi): in the leg plane, hip -> axle ->
              ground when upright.
  profile     the tread cross-section in the leg frame: a surface point at
              axial coordinate w (along a, from the wheel centre plane) and
              radial distance rho from the axle (along n).

The wheel centre plane sits d_wheel along a, and the axle sits L =
`hip_to_axle` along n from the hip (L = 0 at theta = 17 deg, where the rim
closes around the hip; 0.1481 m at the theta = 100 deg running stance, log
s21). A profile point sits at (d_wheel + w) a + (L + rho) n:

    y = (d_wheel + w) cos phi + (L + rho) sin phi
    z = (d_wheel + w) sin phi - (L + rho) cos phi

and the contact is the profile point with the lowest z. That minimisation is
the only place the profile enters, and L does not change which point wins.

L DEFAULTS TO 0, i.e. the pivot is the axle, and every caller in legwheel/
and examples/ uses that default. (y, z) is then the contact relative to the
axle's station on the AB/AD axis, which is the hip-relative contact only at
theta = 17 deg; at theta = 100 deg the hip-relative contact is
(y + L sin phi, z - L cos phi). coronal_bip.side_geometry(pivot="hip") adds
the L sin phi (log s339 s5o).

COROLLARIES, all exact (y and z written at L = 0; any L adds L sin phi to y
and -L cos phi to z)

  Ideal torus (spine R, crown r):  w = -r sin phi,  rho = R + r cos phi, so
      y = d_wheel cos phi + R sin phi,   z = d_wheel sin phi - R cos phi - r.
  The crown moves the contact r sin phi off the leg plane, and
  rho = R + r cos phi is the rolling radius, yet the crown drops out of y:
  the r sin phi cos phi in rho sin phi cancels the one in w cos phi.

  Corgi tread (flat band |w| <= w_c at radius R_t + r_c, shoulder arcs of
  radius r_c centred at w = +-w_c, rho = R_t): for phi != 0 the lower
  shoulder touches,
      w = -sgn(phi) (w_c + r_c sin|phi|),   rho = R_t + r_c cos phi,
      y = (d_wheel - sgn(phi) w_c) cos phi + R_t sin phi.
  At phi = 0 the whole band touches and the band centre (w = 0) is returned,
  so the contact steps by w_c either side of zero -- a rigid-profile fact.

  AB/AD moment of a force f along the leaned leg axis (-n): exactly
  f (d_wheel + w), at any L -- L n is parallel to the force. The finite
  profile's lever is w, i.e. d_lat.

WHAT WAS WRONG (before 2026-09-13)

  side_geometry and C4 used  y = d_wheel cos phi + r_eff sin phi: the rolling
  radius swung about the axle with the contact left ON the leg plane, which
  drops w cos phi. On the ideal torus the excess is r sin phi cos phi
  (0.35 / 3.4 / 5.0 mm at 1 / 10 / 15 deg for r = 20 mm).

RESOLVED 2026-09-13 (log s336): `slip_rf_cambered.rolling_radius` used to
return Stage 0's R_t cos phi - w_c sin|phi| + r_c, which is neither this
profile's rolling radius rho = R_t + r_c cos phi nor its axle height
rho cos phi - w sin phi = R_t cos phi + w_c sin|phi| + r_c (a tilted band
RISES; the old form is the height of the shoulder that does not touch). It
now returns rho; heights use `axle_height`; the old formula survives only as
`slip_rf_cambered.stage0_contact_height_legacy` for closed-stage analysers.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TreadProfile:
    """Rounded tread cross-section: a flat band of half-width `w_flat` at
    radius r_spine + r_crown, closed by shoulder arcs of radius `r_crown`
    centred at axial +-w_flat, radial r_spine. w_flat = 0 is the ideal torus.
    """

    r_spine: float
    r_crown: float
    w_flat: float = 0.0


# Corgi's tread: slip_rf_cambered's R_TREAD / R_CORNER / W_FLAT, the platform
# reference's R_t / r_c / w_c. Duplicated per this package's self-contained
# module convention; tests/test_contact_profile.py asserts they agree.
CORGI_TREAD = TreadProfile(r_spine=0.130, r_crown=0.015, w_flat=0.005)


@dataclass(frozen=True)
class ContactPoint:
    """The contact at one lean. (w, rho) in the leg frame, (y, z) in the hip
    frame; see the module docstring for both."""

    w: float
    rho: float
    y: float
    z: float

    @property
    def d_lat(self) -> float:
        """Displacement off the leg plane, the paper's d_lat (= r sin phi on
        the ideal torus): -w, positive toward the AB/AD axis for phi > 0."""
        return -self.w


def profile_contact(phi: float, profile: TreadProfile = CORGI_TREAD
                    ) -> tuple[float, float]:
    """(w, rho) of the lowest profile point at lean `phi` (rad, |phi| <= 90 deg)."""
    s = float(np.sign(phi))
    w = -s * (profile.w_flat + profile.r_crown * abs(float(np.sin(phi))))
    rho = profile.r_spine + profile.r_crown * float(np.cos(phi))
    return w, rho


def contact_point(phi: float, d_wheel: float,
                  profile: TreadProfile = CORGI_TREAD,
                  hip_to_axle: float = 0.0) -> ContactPoint:
    """The contact in the hip frame: one rotation of
    (d_wheel + w, hip_to_axle + rho). hip_to_axle = 0 (the default) puts the
    axle on the AB/AD axis; see the module docstring."""
    w, rho = profile_contact(phi, profile)
    c, s = float(np.cos(phi)), float(np.sin(phi))
    axial = d_wheel + w
    radial = hip_to_axle + rho
    return ContactPoint(w=w, rho=rho, y=axial * c + radial * s,
                        z=axial * s - radial * c)


def axle_height(phi: float, profile: TreadProfile = CORGI_TREAD) -> float:
    """Axle height above flat ground at lean `phi`: rho cos phi - w sin phi."""
    w, rho = profile_contact(phi, profile)
    return rho * float(np.cos(phi)) - w * float(np.sin(phi))


def leg_plane_force(phi: float, magnitude: float) -> tuple[float, float]:
    """(F_y, F_z) of a ground reaction of `magnitude` along the leaned leg
    axis, ground -> hip (-n). The coordinated-turn reduction puts the stance
    force here, so it is the force C4 holds."""
    return (-magnitude * float(np.sin(phi)), magnitude * float(np.cos(phi)))


def abad_moment(point: ContactPoint, force_y: float, force_z: float) -> float:
    """Moment about the AB/AD axis of a force applied at the contact,
    M_x = y F_z - z F_y (hip frame)."""
    return point.y * force_z - point.z * force_y
