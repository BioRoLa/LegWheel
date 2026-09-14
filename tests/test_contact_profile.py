"""contact_profile: hand-computed special cases, a brute-force lowest point, and
the pins that keep the pre-2026-09-13 lateral form from coming back (log s335).
"""

import numpy as np
import pytest

from legwheel.models import contact_profile as cp
from legwheel.models import slip_rf_cambered as src

D = 0.091675
TORUS = cp.TreadProfile(r_spine=0.125, r_crown=0.020)
COIN = cp.TreadProfile(r_spine=0.130, r_crown=0.0, w_flat=0.010)
LEANS_DEG = [-30.0, -15.0, -1.0, 1.0, 5.0, 10.0, 15.0, 30.0]


def _surface(profile, n=20001):
    """Sampled outer cross-section (w, rho): the band and both shoulder arcs."""
    t = np.linspace(0.0, np.pi / 2, n)
    arc_w = profile.w_flat + profile.r_crown * np.sin(t)
    arc_r = profile.r_spine + profile.r_crown * np.cos(t)
    band_w = np.linspace(-profile.w_flat, profile.w_flat, n)
    band_r = np.full(n, profile.r_spine + profile.r_crown)
    return (np.concatenate([arc_w, -arc_w, band_w]),
            np.concatenate([arc_r, arc_r, band_r]))


def test_constants_match_the_sagittal_module() -> None:
    assert cp.CORGI_TREAD.r_spine == src.R_TREAD
    assert cp.CORGI_TREAD.r_crown == src.R_CORNER
    assert cp.CORGI_TREAD.w_flat == src.W_FLAT


@pytest.mark.parametrize("deg", LEANS_DEG)
@pytest.mark.parametrize("profile", [cp.CORGI_TREAD, TORUS, COIN])
def test_closed_form_is_the_lowest_surface_point(deg, profile) -> None:
    """The closed form against brute force: no derivation in the loop."""
    phi = np.deg2rad(deg)
    w, rho = _surface(profile)
    z = (D + w) * np.sin(phi) - rho * np.cos(phi)
    i = int(np.argmin(z))
    c = cp.contact_point(phi, D, profile)
    assert c.z == pytest.approx(z[i], abs=1e-9)
    assert c.y == pytest.approx((D + w[i]) * np.cos(phi) + rho[i] * np.sin(phi),
                                abs=1e-5)


@pytest.mark.parametrize("deg", [1.0, 10.0, 15.0, 25.0])
def test_ideal_torus_crown_cancels_out_of_the_lateral_coordinate(deg) -> None:
    """The paper's Eqs. (1)-(3) all hold, and the crown still leaves y."""
    phi = np.deg2rad(deg)
    R, r = TORUS.r_spine, TORUS.r_crown
    c = cp.contact_point(phi, D, TORUS)
    assert c.y == pytest.approx(D * np.cos(phi) + R * np.sin(phi), abs=1e-15)
    assert c.z == pytest.approx(D * np.sin(phi) - R * np.cos(phi) - r, abs=1e-15)
    assert c.d_lat == pytest.approx(r * np.sin(phi), abs=1e-15)      # eq:dlat
    assert c.rho == pytest.approx(R + r * np.cos(phi), abs=1e-15)     # eq:reff
    assert cp.axle_height(phi, TORUS) == pytest.approx(
        R * np.cos(phi) + r, abs=1e-15)                                 # eq:h


def test_pre_fix_form_overstates_by_r_sin_cos() -> None:
    """The review's numbers: 0.35 / 3.4 / 5.0 mm at 1 / 10 / 15 deg, r = 20 mm."""
    R, r = TORUS.r_spine, TORUS.r_crown
    for deg, mm in ((1.0, 0.349), (10.0, 3.420), (15.0, 5.000)):
        phi = np.deg2rad(deg)
        legacy = D * np.cos(phi) + (R + r * np.cos(phi)) * np.sin(phi)
        excess = legacy - cp.contact_point(phi, D, TORUS).y
        assert excess == pytest.approx(r * np.sin(phi) * np.cos(phi), abs=1e-15)
        assert 1e3 * excess == pytest.approx(mm, abs=0.002)


def test_tilted_coin_touches_its_inboard_corner_and_rises() -> None:
    """r_crown = 0 is a bare flat band: the lowest point is the corner nearer
    the AB/AD axis, and the axle RISES by w_flat sin phi."""
    phi = np.deg2rad(20.0)
    c = cp.contact_point(phi, D, COIN)
    assert c.w == pytest.approx(-0.010, abs=1e-15)
    assert c.y == pytest.approx((D - 0.010) * np.cos(phi) + 0.130 * np.sin(phi),
                                abs=1e-15)
    assert cp.axle_height(phi, COIN) == pytest.approx(
        0.130 * np.cos(phi) + 0.010 * np.sin(phi), abs=1e-15)
    mirror = cp.contact_point(-phi, D, COIN)
    assert mirror.w == pytest.approx(+0.010, abs=1e-15)
    assert mirror.y == pytest.approx(
        (D + 0.010) * np.cos(phi) - 0.130 * np.sin(phi), abs=1e-15)


def test_corgi_tread_upright_identity_and_closed_form() -> None:
    c0 = cp.contact_point(0.0, D)
    assert c0.w == 0.0
    assert c0.y == pytest.approx(D, abs=1e-15)
    assert c0.z == pytest.approx(-(src.R_TREAD + src.R_CORNER), abs=1e-15)
    for deg in (1.0, 10.0, 15.0, 30.0):
        phi = np.deg2rad(deg)
        c = cp.contact_point(phi, D)
        assert c.y == pytest.approx(
            (D - src.W_FLAT) * np.cos(phi) + src.R_TREAD * np.sin(phi), abs=1e-15)


@pytest.mark.parametrize("big_l", [0.0, 0.1481, 0.3])
@pytest.mark.parametrize("deg", [-20.0, -5.0, 5.0, 12.5, 20.0])
def test_hip_to_axle_shifts_the_contact_by_L_n_and_not_the_leg_axis_moment(
        deg, big_l) -> None:
    """Log s339 s5o. The axle L from the hip along n adds exactly
    (L sin phi, -L cos phi), leaves the profile contact (w, rho) alone, and
    leaves the AB/AD moment of a leg-axis force at f (d_wheel + w): C4 does
    not depend on L. The default L = 0 is bit-identical to no argument."""
    phi = np.deg2rad(deg)
    c0 = cp.contact_point(phi, D)
    c = cp.contact_point(phi, D, hip_to_axle=big_l)
    assert (c.w, c.rho) == (c0.w, c0.rho)
    assert c.y == pytest.approx(c0.y + big_l * np.sin(phi), abs=1e-15)
    assert c.z == pytest.approx(c0.z - big_l * np.cos(phi), abs=1e-15)
    fy, fz = cp.leg_plane_force(phi, 100.0)
    assert cp.abad_moment(c, fy, fz) == pytest.approx(100.0 * (D + c.w),
                                                      abs=1e-12)
    assert cp.contact_point(phi, D, hip_to_axle=0.0) == c0


@pytest.mark.parametrize("deg", [-20.0, -5.0, 5.0, 12.5, 20.0])
def test_leg_plane_force_lever_is_the_axial_contact_offset(deg) -> None:
    """A force along the leaned leg axis has AB/AD moment f (d_wheel + w): the
    finite profile's lever is w = -d_lat. A vertical force sees y instead."""
    phi = np.deg2rad(deg)
    c = cp.contact_point(phi, D)
    fy, fz = cp.leg_plane_force(phi, 100.0)
    assert cp.abad_moment(c, fy, fz) == pytest.approx(100.0 * (D + c.w), abs=1e-12)
    assert cp.abad_moment(c, 0.0, 100.0) == pytest.approx(100.0 * c.y, abs=1e-12)


def test_rolling_radius_is_the_profile_contact_radius() -> None:
    """Log s336 (was pinned OPEN in s335). rolling_radius is rho at the profile
    contact, R_t + r_c cos -- second order, no cusp. The Stage 0 formula it
    replaced sits exactly 2 w_c sin|phi| below this profile's axle height:
    it is the height of the shoulder that does not touch."""
    for deg in (-15.0, 5.0, 10.0, 20.0):
        phi = np.deg2rad(deg)
        assert src.rolling_radius(phi) == pytest.approx(
            cp.contact_point(phi, D).rho, abs=1e-15)
        gap = cp.axle_height(phi) - src.stage0_contact_height_legacy(phi)
        assert gap == pytest.approx(2 * src.W_FLAT * abs(np.sin(phi)), abs=1e-15)
