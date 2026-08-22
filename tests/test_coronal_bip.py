"""Tests for the coronal BIP roll subsystem (Stage 2b Module 2)."""

from __future__ import annotations

import numpy as np
import pytest

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

    d_out is hip-to-CONTACT. The wheel pivots about its axle, so the ground
    point a rolling radius below the centre swings outboard by r*sin(lean).
    R_CORNER (0.015, the shoulder fillet) governs migration ACROSS THE TREAD,
    which is a different quantity and ~20x smaller.
    """
    from legwheel.models.slip_rf_cambered import rolling_radius

    r0 = rolling_radius(0.0)
    assert r0 == pytest.approx(0.145, abs=1e-9)

    for deg in (5.0, 10.0, 15.0, 20.0, 30.0):
        lam = np.deg2rad(deg)
        g = bip.side_geometry(lam, radius_law="measured")
        expect = bip.WHEEL_AXIAL_OFFSET * np.cos(lam) + r0 * np.sin(lam)
        assert g.d_out == pytest.approx(expect, abs=1e-12)

    # The migration must be MONOTONE and OUTBOARD across the working band.
    # The R_CORNER form failed both: it peaked near 10 deg and went negative
    # by 20, i.e. the coupling channel reversed sign inside the band the
    # thesis operates in.
    migr = [bip.side_geometry(np.deg2rad(d), radius_law="measured").d_out
            - bip.WHEEL_AXIAL_OFFSET for d in (5, 10, 15, 20, 30)]
    assert all(m > 0 for m in migr), migr
    assert all(b > a for a, b in zip(migr, migr[1:])), migr
    # Scale check against the term S40 quotes as the contribution: at 10 deg
    # the migration is ~24 mm, not the ~1.2 mm the old coefficient gave.
    assert migr[1] == pytest.approx(0.0240, abs=0.0015)


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
