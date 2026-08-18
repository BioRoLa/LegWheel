"""Tests for the pinned-contact coronal model and its apex map (coronal v2).

The lambda = 0 story here is energy: the pinned spring is conservative by
construction, and v1's documented caveat (quasi-static contact slides with the
hip, length rate ignores vy) is pinned below as a measured leak, not a
docstring claim.
"""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import coronal_bip as v1
from legwheel.models import coronal_return_map as v2
from legwheel.models.coronal_bip import CoronalParams


@pytest.fixture(scope="module")
def params_and_equilibrium():
    p = CoronalParams()
    return p, v1.equilibrium_height(p)


def test_pinned_bounce_conserves_energy_under_lateral_motion_where_sliding_contact_does_not(
        params_and_equilibrium) -> None:
    """v2's reason to exist. A bounce with roll seed and lateral velocity must
    conserve energy to integration tolerance under pinned contacts; v1's
    sliding contact, measured over 40 ms of rolled loaded stance with the same
    lateral speed, leaks percent-level energy (the leak needs UNEQUAL leg
    compression -- for the symmetric pair the two legs' lateral work terms
    cancel, which is why v1's own symmetric-bounce energy test passes)."""
    p, z_eq = params_and_equilibrium
    s2 = [0.0, z_eq + 0.03, 0.05, 0.30, 0.0, 0.4]
    t, ys, legs = v2.simulate_pinned(p, s2, 0.5)
    e0 = v2.energy_pinned(p, s2, None)
    e1 = v2.energy_pinned(p, ys[:, -1], legs)
    assert abs(e1 - e0) / e0 < 1e-8

    s1 = [0.0, z_eq - 0.015, 0.08, 0.5, 0.0, 0.0]
    sol = v1.simulate(p, s1, 0.04)
    ev0 = v1.energy(p, sol.y[:, 0])
    ev1 = v1.energy(p, sol.y[:, -1])
    assert abs(ev1 - ev0) / ev0 > 1e-3


def test_pinned_and_sliding_contact_agree_for_a_purely_vertical_symmetric_bounce(
        params_and_equilibrium) -> None:
    """Regression tie to the validated v1: with no lateral motion and no roll
    the contact never moves, so pinning it changes nothing."""
    p, z_eq = params_and_equilibrium
    s0 = [0.0, z_eq + 0.02, 0.0, 0.0, 0.0, 0.0]
    t, ys, _ = v2.simulate_pinned(p, s0, 0.25)
    sol = v1.simulate(p, s0, 0.25, dense=True)
    z_v1 = sol.sol(np.clip(t, 0.0, sol.t[-1]))[1]
    assert np.max(np.abs(ys[1] - z_v1)) < 1e-8
    assert np.max(np.abs(ys[2])) == 0.0     # roll never seeded, never appears


def test_simultaneous_pair_touchdown_is_pinned_without_missing_an_event(
        params_and_equilibrium) -> None:
    """The symmetric bounce lands both legs in the same instant; the event
    machinery reports one and the tie sweep must pin the other, or the stance
    is silently one-legged and injects roll into a symmetric drop."""
    p, z_eq = params_and_equilibrium
    s0 = [0.0, z_eq + 0.02, 0.0, 0.0, 0.0, 0.0]
    _, ys, legs = v2.simulate_pinned(p, s0, 0.09)   # mid-stance of bounce 1
    assert sorted(legs) == [-1, +1]
    assert abs(ys[2, -1]) == 0.0


def test_pinned_statics_match_the_quasi_static_equilibrium(
        params_and_equilibrium) -> None:
    """Given the same contact points, v2's stance force law reproduces v1's
    equilibrium: at v1's equilibrium height with feet pinned where v1 places
    them (outboard of each hip on the ground), all accelerations vanish."""
    p, z_eq = params_and_equilibrium
    legs = {s: np.array([s * (p.w_hip + p.left.d_out), 0.0]) for s in (+1, -1)}
    rhs = v2._stance_rhs(0.0, [0.0, z_eq, 0.0, 0.0, 0.0, 0.0], p, legs)
    assert np.max(np.abs(rhs)) < 1e-9


def test_symmetric_stiffness_admits_a_bounce_in_place_pronk_fixed_point(
        params_and_equilibrium) -> None:
    """Chang 2022's pronk at gamma = 1, on our geometry: the symmetric bounce
    closes apex-to-apex. (Every symmetric drop height closes -- conservative
    springs make a continuum of orbits -- so closure, not uniqueness, is the
    testable claim.)"""
    p, z_eq = params_and_equilibrium
    x = np.array([0.0, z_eq + 0.02, 0.0, 0.0])
    assert np.linalg.norm(v2.apex_map(p, x) - x) < 1e-8


def test_stiffness_asymmetry_breaks_the_symmetric_bounce_closure(
        params_and_equilibrium) -> None:
    """The other half of Chang's existence claim: at k_ratio != 1 the
    symmetric bounce no longer returns to itself -- one bounce through the
    asymmetric pair pumps roll."""
    p, z_eq = params_and_equilibrium
    x = np.array([0.0, z_eq + 0.02, 0.0, 0.0])
    p_asym = v2.with_stiffness_ratio(p, 1.1)
    assert np.linalg.norm(v2.apex_map(p_asym, x) - x) > 1e-3


def test_the_pronk_roll_mode_is_not_asymptotically_stable_passively(
        params_and_equilibrium) -> None:
    """Chang Fig. 13 on the pinned model: the apex-map Jacobian at the
    symmetric bounce has spectral radius >= 1 -- no passive damping exists to
    kill a roll seed, which is the premise clocked torque answers."""
    p, z_eq = params_and_equilibrium
    x = np.array([0.0, z_eq + 0.02, 0.0, 0.0])
    j = v2.jacobian(p, x)
    assert np.max(np.abs(np.linalg.eigvals(j))) > 1.0


def test_with_stiffness_ratio_only_touches_the_left_side() -> None:
    p = CoronalParams()
    p2 = v2.with_stiffness_ratio(p, 1.25)
    assert p2.k_left == pytest.approx(1.25 * p.k_right)
    assert p2.k_right == p.k_right
    assert p2.left is p.left and p2.right is p.right


def test_gamma_phase_preserves_total_stiffness_and_exchanges_sides() -> None:
    """Chang 2022 Eq. 1-3: gamma redistributes a FIXED k_sum between the leg
    sets, and the phase transition swaps which side carries which."""
    p = CoronalParams()
    a = v2.with_gamma_phase(p, 2.0, phase=0)
    b = v2.with_gamma_phase(p, 2.0, phase=1)
    assert a.k_left + a.k_right == pytest.approx(p.k_left + p.k_right)
    assert a.k_left == pytest.approx(2.0 * a.k_right)
    assert (a.k_left, a.k_right) == (b.k_right, b.k_left)
    sym = v2.with_gamma_phase(p, 1.0)
    assert sym.k_left == pytest.approx(sym.k_right)


def test_two_step_map_at_gamma_one_is_the_plain_map_composed_twice(
        params_and_equilibrium) -> None:
    p, z_eq = params_and_equilibrium
    x = np.array([0.0, z_eq + 0.02, 0.0, 0.0])
    twice = v2.apex_map(p, v2.apex_map(p, x))
    assert np.allclose(v2.apex_map_two_step(p, x, 1.0), twice, atol=1e-9)


def test_gamma_exchange_breaks_the_pronk_closure_only_off_ratio(
        params_and_equilibrium) -> None:
    """Chang section 2.2's existence half, in miniature: the symmetric bounce
    closes under the two-step exchange map at gamma = 1 (measured 7e-11) and
    does not at gamma = 1.25 (measured 0.63) -- the paper's 'the pronking
    orbit could not exist as gamma != 1'. The rolling fixed points that
    replace it (drho* rising 0.13 -> 0.59 rad/s over gamma 1.1 -> 2.0) are
    the validation script's job; too slow for a test."""
    p, z_eq = params_and_equilibrium
    x = np.array([0.0, z_eq + 0.02, 0.0, 0.0])
    assert np.linalg.norm(v2.apex_map_two_step(p, x, 1.0) - x) < 1e-8
    assert np.linalg.norm(v2.apex_map_two_step(p, x, 1.25) - x) > 1e-2
