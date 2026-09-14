"""Tests for the coronal BIP roll subsystem (Stage 2b Module 2)."""

from __future__ import annotations

import inspect
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from legwheel.models import contact_profile as cp
from legwheel.models import coronal_bip as bip
from legwheel.models.coronal_bip import CoronalParams


def params() -> CoronalParams:
    return CoronalParams()


def test_equilibrium_supports_the_weight() -> None:
    p = params()
    z = bip.equilibrium_height(p)
    # Below the geometric standing height, above a collapsed pose.
    z_top = np.sqrt(p.left.l0**2 - p.left.d_out**2)
    assert 0.8 * z_top < z < z_top
    deriv = bip.rhs(0.0, [0.0, z, 0.0, 0.0, 0.0, 0.0], p)
    assert deriv[3] == pytest.approx(0.0, abs=1e-8)   # no lateral force
    assert deriv[4] == pytest.approx(0.0, abs=1e-6)   # weight balanced
    assert deriv[5] == pytest.approx(0.0, abs=1e-8)   # no roll moment


def test_symmetric_bounce_conserves_energy_and_symmetry() -> None:
    """The vertical bounce is the one motion the quasi-static contact model is
    exactly conservative for -- so energy drift here is a genuine bug."""
    p = params()
    z0 = bip.equilibrium_height(p)
    s0 = [0.0, z0 + 0.03, 0.0, 0.0, 0.0, 0.0]
    sol = bip.simulate(p, s0, t_final=1.5, dense=True)
    t = np.linspace(0.0, sol.t[-1], 400)
    states = sol.sol(t)
    e = np.array([bip.energy(p, states[:, i]) for i in range(len(t))])
    assert np.max(np.abs(e - e[0])) / e[0] < 1e-6
    assert np.max(np.abs(states[2])) < 1e-10   # rho stays zero
    assert np.max(np.abs(states[0])) < 1e-10   # y stays zero


def test_mirror_symmetry() -> None:
    """A rolled initial state and its mirror produce mirrored trajectories."""
    p = params()
    z0 = bip.equilibrium_height(p)
    a = bip.simulate(p, [0.0, z0 + 0.02, +0.05, 0.0, 0.0, +0.2], 0.6, dense=True)
    b = bip.simulate(p, [0.0, z0 + 0.02, -0.05, 0.0, 0.0, -0.2], 0.6, dense=True)
    t = np.linspace(0.0, min(a.t[-1], b.t[-1]), 200)
    sa, sb = a.sol(t), b.sol(t)
    assert np.allclose(sa[1], sb[1], atol=1e-9)          # z identical
    assert np.allclose(sa[2], -sb[2], atol=1e-9)         # rho mirrored
    assert np.allclose(sa[0], -sb[0], atol=1e-9)         # y mirrored


def test_static_roll_is_restoring() -> None:
    """Tilting the standing robot must produce a restoring moment -- the outer
    leg compresses more. If this fails the force/torque signs are wrong."""
    p = params()
    z = bip.equilibrium_height(p)
    for rho in (0.02, 0.05, -0.02, -0.05):
        tau = bip.rhs(0.0, [0.0, z, rho, 0.0, 0.0, 0.0], p)[5] * p.j_roll
        assert np.sign(tau) == -np.sign(rho), (rho, tau)


def test_side_geometry_identity_and_lean_trends() -> None:
    for law in ("torus", "measured"):
        g0 = bip.side_geometry(0.0, radius_law=law)
        assert g0.d_out == pytest.approx(bip.WHEEL_AXIAL_OFFSET, abs=1e-15)
        assert g0.l0 == pytest.approx(bip.LEG_LENGTH_NOMINAL, abs=1e-15)
    # The rest length shrinks only under the smooth-torus law. Stage 1.5 / S75
    # measured the effective radius FLAT over 0-40 deg in sim, so "measured"
    # must NOT shrink -- that is the whole difference between the two laws and
    # it is asserted, not assumed.
    g_t = bip.side_geometry(np.deg2rad(20.0), radius_law="torus")
    g_m = bip.side_geometry(np.deg2rad(20.0), radius_law="measured")
    assert g_t.l0 < bip.LEG_LENGTH_NOMINAL
    assert g_m.l0 == pytest.approx(bip.LEG_LENGTH_NOMINAL, abs=1e-15)
    p0 = params()
    p1 = params().cambered(np.deg2rad(10), np.deg2rad(10), radius_law="torus")
    assert p1.left.l0 < p0.left.l0
    p2 = params().cambered(np.deg2rad(10), np.deg2rad(10),
                           radius_law="measured")
    assert p2.left.l0 == pytest.approx(p0.left.l0, abs=1e-15)
    with pytest.raises(ValueError):
        bip.side_geometry(0.0, radius_law="whatever")


def test_side_geometry_lateral_offset_uses_the_ROLLING_radius() -> None:
    """S183: the crown term must be the rolling radius, not R_CORNER.

    This is the coefficient the whole cambered coupling rides on and NOTHING
    pinned it before -- the old test asserted only the lambda = 0 identity and
    that l0 decreased, so a 9.7x error in the term that generates the roll
    moment passed every check for six days.

    d_out is a displacement in space (from the axle's station on the AB/AD
    axis under the default pivot="axle"; log s339 s5o). The wheel pivots
    about its axle, so the ground point a rolling radius below the centre
    swings outboard by r*sin(lean).
    R_CORNER (0.015, the shoulder fillet) governs migration ACROSS THE TREAD,
    which is a different quantity and ~20x smaller.
    """
    from legwheel.models.slip_rf_cambered import (R_TREAD, W_FLAT,
                                                  rolling_radius)

    r0 = rolling_radius(0.0)
    assert r0 == pytest.approx(0.145, abs=1e-9)

    for deg in (5.0, 10.0, 15.0, 20.0, 30.0):
        lam = np.deg2rad(deg)
        # The legacy form stays reachable, and pinned, for reproduction only.
        legacy = bip.side_geometry(lam, radius_law="measured", lateral="legacy")
        assert legacy.d_out == pytest.approx(
            bip.WHEEL_AXIAL_OFFSET * np.cos(lam) + r0 * np.sin(lam), abs=1e-12)
        # S335: the profile contact, hand-computed -- lower shoulder, crown
        # cancelled, and the SAME for both radius laws.
        expect = ((bip.WHEEL_AXIAL_OFFSET - W_FLAT) * np.cos(lam)
                  + R_TREAD * np.sin(lam))
        for law in ("measured", "torus"):
            g = bip.side_geometry(lam, radius_law=law, lateral="profile")
            assert g.d_out == pytest.approx(expect, abs=1e-12)

    # The migration must be MONOTONE and OUTBOARD across the working band.
    # The R_CORNER form failed both: it peaked near 10 deg and went negative
    # by 20, i.e. the coupling channel reversed sign inside the band the
    # thesis operates in.
    migr = [bip.side_geometry(np.deg2rad(d), radius_law="measured",
                              lateral="profile").d_out
            - bip.WHEEL_AXIAL_OFFSET for d in (5, 10, 15, 20, 30)]
    assert all(m > 0 for m in migr), migr
    assert all(b > a for a, b in zip(migr, migr[1:])), migr
    # Scale check: at 10 deg the profile contact is +16.3 mm outboard of
    # upright (S335; the legacy form said +24 mm, the R_CORNER form ~1.2).
    assert migr[1] == pytest.approx(0.01626, abs=0.0002)


PIVOT_ARMS = [(law, drop, lateral)
              for law in ("torus", "measured")
              for drop in (0.0, bip.WHEEL_AXIAL_OFFSET)
              for lateral in ("profile", "legacy")]
PIVOT_LEANS_DEG = (-20.0, -10.0, -1.0, 1.0, 5.0, 10.0, 15.0, 20.0)


def test_coronal_pivot_default_is_axle() -> None:
    """Log s339 s5o: pivot="hip" is an option. With LEGWHEEL_CORONAL_PIVOT
    unset the module default is "axle", the geometry every closed Stage 2b /
    s335 / s336 number was computed on."""
    params = inspect.signature(bip.side_geometry).parameters
    assert params["pivot"].default == bip.CORONAL_PIVOT_DEFAULT
    assert params["hip_to_axle"].default is None
    root = Path(__file__).resolve().parents[1]
    env = {k: v for k, v in os.environ.items()
           if k != "LEGWHEEL_CORONAL_PIVOT"}
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in (str(root), env.get("PYTHONPATH", "")) if p)
    code = ("from legwheel.models import coronal_bip as b; "
            "print(b.CORONAL_PIVOT_DEFAULT)")
    out = subprocess.run([sys.executable, "-c", code], env=env, cwd=root,
                         capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "axle"
    bad = subprocess.run([sys.executable, "-c", code], cwd=root,
                         env={**env, "LEGWHEEL_CORONAL_PIVOT": "knee"},
                         capture_output=True, text=True)
    assert bad.returncode != 0
    assert "LEGWHEEL_CORONAL_PIVOT" in bad.stderr
    with pytest.raises(ValueError):
        bip.side_geometry(0.1, pivot="knee")
    with pytest.raises(ValueError):
        bip.side_geometry(0.1, pivot="hip", hip_to_axle=-0.01)


@pytest.mark.parametrize("law,drop,lateral", PIVOT_ARMS)
def test_hip_pivot_with_the_axle_on_the_hip_is_the_axle_geometry_bit_for_bit(
        law, drop, lateral) -> None:
    """(i) L = 0 is the "axle" geometry, exactly, on every arm."""
    kw = dict(radius_law=law, axial_drop=drop, lateral=lateral)
    for deg in PIVOT_LEANS_DEG:
        lam = np.deg2rad(deg)
        a = bip.side_geometry(lam, pivot="axle", **kw)
        h = bip.side_geometry(lam, pivot="hip", hip_to_axle=0.0, **kw)
        assert (h.d_out, h.l0) == (a.d_out, a.l0), deg


@pytest.mark.parametrize("law,drop,lateral", PIVOT_ARMS)
def test_hip_pivot_at_zero_lean_is_the_straight_geometry_bit_for_bit(
        law, drop, lateral) -> None:
    """(ii) lean = 0 is the "axle" geometry, exactly, at any L -- so the
    straight orbit (beta* 80.91 deg, h 0.326 m) cannot move."""
    kw = dict(radius_law=law, axial_drop=drop, lateral=lateral)
    for lean in (0.0, -0.0):
        a = bip.side_geometry(lean, pivot="axle", **kw)
        for big_l in (None, 0.1481, 0.3):
            h = bip.side_geometry(lean, pivot="hip", hip_to_axle=big_l, **kw)
            assert (h.d_out, h.l0) == (a.d_out, a.l0), (lean, big_l)


@pytest.mark.parametrize("law,drop,lateral", PIVOT_ARMS)
def test_hip_pivot_adds_the_leg_swing_to_d_out_and_leaves_l0(
        law, drop, lateral) -> None:
    """The hip pivot moves the foot L*sin(lean) outboard and nothing else:
    the legs' sphere of radius l0 about the hip already carries a rigid
    leg's contact. L defaults to l0_sagittal - r0 = 0.148 m (theta = 100)."""
    from legwheel.models.slip_rf_cambered import rolling_radius

    big_l = bip.LEG_LENGTH_NOMINAL - rolling_radius(0.0)
    assert big_l == pytest.approx(0.148, abs=1e-12)
    kw = dict(radius_law=law, axial_drop=drop, lateral=lateral)
    for deg in PIVOT_LEANS_DEG:
        lam = np.deg2rad(deg)
        a = bip.side_geometry(lam, pivot="axle", **kw)
        h = bip.side_geometry(lam, pivot="hip", **kw)
        assert h.l0 == a.l0
        assert h.d_out == pytest.approx(a.d_out + big_l * np.sin(lam),
                                        abs=1e-15)
        if lateral == "profile":
            assert h.d_out == pytest.approx(cp.contact_point(
                lam, bip.WHEEL_AXIAL_OFFSET, hip_to_axle=big_l).y, abs=1e-15)
        if deg in (10.0, 15.0):   # the s5o shortfall: 25.7 / 38.3 mm
            assert 1e3 * (h.d_out - a.d_out) == pytest.approx(
                {10.0: 25.70, 15.0: 38.31}[deg], abs=0.01)


def test_hip_pivot_lateral_offset_matches_forward_kinematics_at_theta_100(
        ) -> None:
    """(iii) against CorgiLegKinematics, not against contact_profile.

    Theta = 100 deg with the leg straight down (the FK's beta = 0), AB/AD
    angle gamma = lean. The lowest rim point is searched on the FK itself,
    over the tread width at the bottom of the rim. At gamma = 0 the flat
    band touches along its width; side_geometry returns its centre (w = 0),
    so the FK is evaluated there. Spec: < 1 mm over 0-20 deg; measured
    0.039 mm at 20 deg, which is the model's L = 0.148 against the FK's
    0.1481147 (x sin 20 deg). The "axle" arm misses by L*sin(lean)."""
    from scipy.optimize import minimize_scalar

    from legwheel.models.corgi_leg import CorgiLegKinematics

    kin = CorgiLegKinematics(0)        # front-left: body +y is outboard
    theta = np.deg2rad(100.0)
    origin = kin.p_Mi_in_B             # the AB/AD axis at the hip
    half = kin.wheel_thickness / 2.0
    worst = 0.0
    axle_short = {}
    for deg in (0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0):
        gamma = np.deg2rad(deg)

        def foot_z(w, gamma=gamma):
            return kin.forward_kinematics(theta, 0.0, gamma, alpha=0.0, w=w)[2]

        w_star = 0.0 if deg == 0.0 else minimize_scalar(
            foot_z, bounds=(-half, 0.0), method="bounded",
            options={"xatol": 1e-10}).x
        lat = (kin.forward_kinematics(theta, 0.0, gamma, alpha=0.0, w=w_star)[1]
               - origin[1])
        for law in ("torus", "measured"):
            h = bip.side_geometry(gamma, radius_law=law, lateral="profile",
                                  pivot="hip")
            worst = max(worst, abs(h.d_out - lat))
        a = bip.side_geometry(gamma, lateral="profile", pivot="axle")
        axle_short[deg] = lat - a.d_out
    assert worst < 1e-4, worst
    assert 1e3 * axle_short[10.0] == pytest.approx(25.7, abs=0.1)
    assert 1e3 * axle_short[15.0] == pytest.approx(38.3, abs=0.1)


def test_asymmetric_stiffness_rolls_the_bounce() -> None:
    """Chang's gamma != 1 mechanism: unequal side stiffness turns a symmetric
    drop into roll. The qualitative claim only -- sign and nonzero growth."""
    p = CoronalParams(k_left=1.2 * bip.K_SIDE_DEFAULT)
    z0 = bip.equilibrium_height(CoronalParams())
    sol = bip.simulate(p, [0.0, z0 + 0.02, 0.0, 0.0, 0.0, 0.0], 0.5, dense=True)
    t = np.linspace(0.0, sol.t[-1], 200)
    rho = sol.sol(t)[2]
    assert np.max(np.abs(rho)) > 1e-4   # symmetry genuinely broken


def test_roll_growth_returns_a_finite_factor() -> None:
    """Machinery check, not a physics assertion: the growth factor is the
    number Stage 2b's clocked torque has to beat, measured not assumed."""
    factor = bip.roll_growth_per_bounce(params(), drop=0.02, n_bounce=5)
    assert np.isfinite(factor) and factor > 0.0
