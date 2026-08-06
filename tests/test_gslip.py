"""Validation gate for the G-SLIP model (Lu & Lin 2024).

Three independent checks:

1. Internal consistency of the derived constants (eqs 1-4) against the
   touchdown condition (eq 8): z(theta0, phi0) must equal r + l0*sin(beta).
2. Energy conservation through stance, which validates the equations of
   motion (eq 7) built from the Jacobian and Hessians.
3. Reduction of the general model (eq 5) to the appendix kinematics of the
   four special cases at the Table 1 parameters.

Phase 2 of the port does not start until these pass.
"""

from __future__ import annotations

import numpy as np
import pytest

from legwheel.models import gslip
from legwheel.models.gslip import GSlipParams
from legwheel.models.gslip_special_cases import (
    TABLE_1,
    rslip_position,
    table_1_params,
    tsl_l0,
    tsl_position,
)

CASES = list(TABLE_1)

# Touchdown states known to produce a complete stride for each special case,
# found by scanning (beta, alpha, v) over the paper's operating ranges. Speeds
# are dimensionless (eq 16): v = v_tilde * sqrt(g * (l0 + r)).
#
# The viable region sits near beta = 30 deg, not the steeper landing angles a
# point-contact SLIP tolerates: Table 1 represents the spring with two 1 m bars
# folded to a ~0.15 m leg, and that sliver linkage reaches its dead point
# (det J = 0) under a steep landing.
VIABLE_TOUCHDOWN = {
    "SLIP": dict(v_tilde=1.5, alpha_deg=10.0, beta_deg=30.0),
    "SLIP-RF": dict(v_tilde=1.5, alpha_deg=10.0, beta_deg=30.0),
    "TSL": dict(v_tilde=1.0, alpha_deg=15.0, beta_deg=30.0),
    "R-SLIP": dict(v_tilde=1.5, alpha_deg=5.0, beta_deg=30.0),
}


def touchdown_for(name: str) -> tuple[GSlipParams, float, float, float]:
    """(params, v, alpha, beta) for a known-viable stride of a special case."""
    p = table_1_params(name)
    spec = VIABLE_TOUCHDOWN[name]
    v = spec["v_tilde"] * np.sqrt(p.g * (p.l0 + p.r))
    return p, v, np.deg2rad(spec["alpha_deg"]), np.deg2rad(spec["beta_deg"])


# --------------------------------------------------------------------------
# 1. Derived constants vs the touchdown condition
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", CASES)
@pytest.mark.parametrize("beta_deg", [20.0, 40.0, 60.0, 75.0])
def test_touchdown_height_consistency(name: str, beta_deg: float) -> None:
    """z(theta0, phi0) == r + l0*sin(beta), tying eqs 1-5 to eq 8.

    This is the sharpest available check on l3, eta, l0 and theta0: the
    touchdown height computed from the full stance kinematics must agree with
    the touchdown event definition.
    """
    p = table_1_params(name)
    beta = np.deg2rad(beta_deg)
    z_from_kinematics = gslip.stance_height(p, p.theta0(beta), p.phi0)
    assert z_from_kinematics == pytest.approx(p.touchdown_height(beta), abs=1e-12)


@pytest.mark.parametrize("name", CASES)
@pytest.mark.parametrize("beta_deg", [30.0, 45.0, 70.0])
def test_l0_is_mass_to_rim_center_distance(name: str, beta_deg: float) -> None:
    """l0 (eq 4) is the mass-to-rim-center distance at the natural spring angle.

    Checks the anchoring of `stance_position`: at touchdown the rim center is
    directly above the contact point at height r, so the mass must be exactly
    l0 away from it.
    """
    p = table_1_params(name)
    beta = np.deg2rad(beta_deg)
    x, z = gslip.stance_position(p, p.theta0(beta), p.phi0, beta)
    assert np.hypot(x, z - p.r) == pytest.approx(p.l0, rel=1e-12)


# --------------------------------------------------------------------------
# 2. Equations of motion: energy conservation
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", CASES)
def test_stance_conserves_energy(name: str) -> None:
    """The conservative model must not gain or lose energy through stance."""
    p, v, alpha, beta = touchdown_for(name)

    sol = gslip.simulate_stance(p, v, alpha, beta, rtol=1e-12, atol=1e-14)

    energies = [
        sum(gslip.stance_energy(p, *sol.y[:, i]).values()) for i in range(sol.y.shape[1])
    ]
    drift = (max(energies) - min(energies)) / abs(energies[0])
    assert drift < 1e-9, f"{name}: relative energy drift {drift:.2e}"


@pytest.mark.parametrize("name", CASES)
def test_jacobian_matches_finite_difference(name: str) -> None:
    """Analytic Jacobian of eq 5 against central differences."""
    p = table_1_params(name)
    theta, phi, h = 1.1, p.phi0 - 0.05, 1e-6
    theta0 = p.theta0(np.deg2rad(60.0))

    analytic = gslip.stance_jacobian(p, theta, phi)
    numeric = np.zeros((2, 2))
    for j, (dt, dp) in enumerate([(h, 0.0), (0.0, h)]):
        plus = np.array(gslip.stance_position_raw(p, theta + dt, phi + dp, theta0))
        minus = np.array(gslip.stance_position_raw(p, theta - dt, phi - dp, theta0))
        numeric[:, j] = (plus - minus) / (2 * h)

    assert numeric == pytest.approx(analytic, abs=1e-6)


@pytest.mark.parametrize("name", CASES)
def test_jacobian_determinant_closed_form(name: str) -> None:
    """det J = l1*l3*sin(phi - 2*theta - eta), the dead-point condition."""
    p = table_1_params(name)
    for theta in (0.4, 0.9, 1.4):
        for phi in (p.phi0 - 0.2, p.phi0 - 0.05, p.phi0):
            expected = np.linalg.det(gslip.stance_jacobian(p, theta, phi))
            assert gslip.stance_jacobian_det(p, theta, phi) == pytest.approx(
                expected, abs=1e-12
            )


@pytest.mark.parametrize("name", CASES)
def test_hessians_match_finite_difference(name: str) -> None:
    """Analytic Hessians against central differences of the Jacobian."""
    p = table_1_params(name)
    theta, phi, h = 1.1, p.phi0 - 0.05, 1e-6

    h_x, h_z = gslip.stance_hessians(p, theta, phi)
    num_x, num_z = np.zeros((2, 2)), np.zeros((2, 2))
    for j, (dt, dp) in enumerate([(h, 0.0), (0.0, h)]):
        plus = gslip.stance_jacobian(p, theta + dt, phi + dp)
        minus = gslip.stance_jacobian(p, theta - dt, phi - dp)
        num_x[:, j] = (plus[0] - minus[0]) / (2 * h)
        num_z[:, j] = (plus[1] - minus[1]) / (2 * h)

    assert num_x == pytest.approx(h_x, abs=1e-6)
    assert num_z == pytest.approx(h_z, abs=1e-6)


# --------------------------------------------------------------------------
# 3. Reduction to the appendix special cases
# --------------------------------------------------------------------------


def test_point_contact_cases_have_zero_rim() -> None:
    """SLIP and TSL are the r = 0 (point contact) cases."""
    assert table_1_params("SLIP").r == 0.0
    assert table_1_params("TSL").r == 0.0


def test_derived_constants_collapse_when_psi_is_zero() -> None:
    """psi = 0 and r = 0 give l3 = l2 and eta = 0, so eq 5 loses the rim terms."""
    for name in ("SLIP", "TSL"):
        p = table_1_params(name)
        assert p.l3 == pytest.approx(p.l2, rel=1e-14)
        assert p.eta == pytest.approx(0.0, abs=1e-14)


@pytest.mark.parametrize("theta", [0.6, 1.0, 1.4, 2.0])
@pytest.mark.parametrize("phi", [1.30, 1.45, 1.571])
def test_tsl_reduction(theta: float, phi: float) -> None:
    """G-SLIP at the TSL parameters reproduces eq 25, up to the x convention.

    z agrees exactly. The appendix writes x with the lower-bar term negated
    relative to eq 5 (a mirrored horizontal convention between figures 2 and
    18b), so x is compared against the mirrored form.
    """
    p = table_1_params("TSL")
    x_g, z_g = gslip.stance_position_raw(p, theta, phi, theta0=0.0)
    x_a, z_a = tsl_position(p.l1, p.l2, theta, phi)

    assert z_g == pytest.approx(z_a, abs=1e-12)
    # eq 5 gives  +l2*cos(phi-theta); eq 25 gives  -l2*cos(phi-theta).
    x_a_unmirrored = x_a + 2 * p.l2 * np.cos(phi - theta)
    assert x_g == pytest.approx(x_a_unmirrored, abs=1e-12)


def test_tsl_rest_length_convention() -> None:
    """eq 4 and eq 29 differ by phi0 -> pi - phi0, the same convention offset
    that flips the sign of the lower-bar term between eq 5 and eq 25.

    eq 4 (general):  l0 = sqrt(l1^2 + l3^2 - 2*l1*l3*cos(phi0 - eta))
    eq 29 (TSL):     l0 = sqrt(l1^2 + l2^2 + 2*l1*l2*cos(phi0))

    With r = 0 and psi = 0 these reduce to the same expression only under
    cos(phi0) -> -cos(phi0), confirming the appendix measures the TSL spring
    angle from the opposite reference.
    """
    p = table_1_params("TSL")
    assert p.eta == pytest.approx(0.0, abs=1e-14)
    assert p.l0 == pytest.approx(
        np.sqrt(p.l1**2 + p.l2**2 - 2 * p.l1 * p.l2 * np.cos(p.phi0)), rel=1e-12
    )
    # The appendix form, evaluated at the supplementary angle, agrees.
    assert p.l0 == pytest.approx(tsl_l0(p.l1, p.l2, np.pi - p.phi0), rel=1e-12)


def rslip_exact_params() -> GSlipParams:
    """R-SLIP requires l3 = r exactly, which pins psi.

    l3 = r  <=>  l2^2 + r^2 - 2*l2*r*cos(psi) = r^2  <=>  cos(psi) = l2/(2r).

    Table 1 quotes psi = 0.576, a rounding of arccos(0.126/0.15) = 0.573513;
    at the quoted value l3 = 0.075170 rather than 0.075, so the reduction only
    holds to ~2e-4. Use the exact value here.
    """
    base = TABLE_1["R-SLIP"]
    return GSlipParams(**{**base, "psi": float(np.arccos(base["l2"] / (2 * base["r"])))})


def test_rslip_geometry_degenerates_correctly() -> None:
    """At the exact psi, the lower bar collapses onto the rim: l3 = r, eta = psi."""
    p = rslip_exact_params()
    assert p.l3 == pytest.approx(p.r, rel=1e-12)
    assert p.eta == pytest.approx(p.psi, rel=1e-12)
    # Table 1's rounded psi is close but not exact.
    assert table_1_params("R-SLIP").l3 == pytest.approx(p.r, abs=2e-4)


@pytest.mark.parametrize("theta", [0.6, 1.0, 1.4])
@pytest.mark.parametrize("phi", [1.30, 1.45, 1.571])
def test_rslip_reduction(theta: float, phi: float) -> None:
    """G-SLIP at the exact R-SLIP parameters reproduces eq 35.

    The appendix measures the R-SLIP spring angle from the lower bar rather
    than from the rim, so phi_appendix = phi_gslip - eta. The rolling term
    r*(phi - phi0 - ...) is unaffected because phi and phi0 shift together.
    """
    p = rslip_exact_params()
    theta0 = 0.0
    x_g, z_g = gslip.stance_position_raw(p, theta, phi, theta0)
    x_a, z_a = rslip_position(
        p.l1, p.r, theta, phi - p.eta, theta0, p.phi0 - p.eta
    )

    assert z_g == pytest.approx(z_a, abs=1e-12)
    assert x_g == pytest.approx(x_a, abs=1e-12)


# --------------------------------------------------------------------------
# 4. A full stride runs
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", CASES)
def test_stride_completes(name: str) -> None:
    """Stance, liftoff and flight complete and return sane touchdown states."""
    p, v, alpha, beta = touchdown_for(name)

    result = gslip.stride(p, v, alpha, beta)

    assert result["stance_time"] > 0
    assert result["flight_time"] > 0
    # Conservative model returning to the same height keeps its speed.
    assert result["v"] == pytest.approx(v, rel=1e-6)


@pytest.mark.parametrize("name", CASES)
def test_dead_point_is_reported_clearly(name: str) -> None:
    """A steep landing drives the Table 1 linkage into its dead point.

    Phase 2's basin-of-attraction search relies on failures being diagnosed
    rather than the integrator stalling, so check the guard actually fires.
    """
    p = table_1_params(name)
    with pytest.raises(gslip.GSlipFailure):
        gslip.stride(p, v=2.0, alpha=np.deg2rad(10.0), beta=np.deg2rad(85.0))


# Corgi, from Phase 0 (examples/gslip/identify_corgi_params.py):
# m = 30.0 kg, rolling radius r = 0.145 m, nominal hip-to-arc-center
# l0 = 0.0850 m at theta = 65.89 deg, k_virtual = 62286 N/m at k_rel = 18.
CORGI_MASS = 30.0
CORGI_R = 0.145
CORGI_L0 = 0.0850
CORGI_K_LINEAR = 62286.0
CORGI_BAR = 1.0  # long bars approximate a linear spring (paper section 2.5)
CORGI_PSI = 1.496


def corgi_params() -> GSlipParams:
    """G-SLIP parameters for the Corgi at its Phase 0 nominal pose."""
    phi0 = gslip.phi0_for_leg_length(
        CORGI_BAR, CORGI_BAR, CORGI_R, CORGI_PSI, CORGI_L0
    )
    p = GSlipParams(
        m=CORGI_MASS, l1=CORGI_BAR, l2=CORGI_BAR, r=CORGI_R,
        k_t=1.0, phi0=phi0, psi=CORGI_PSI,
    )
    return GSlipParams(
        m=CORGI_MASS, l1=CORGI_BAR, l2=CORGI_BAR, r=CORGI_R,
        k_t=gslip.k_t_for_linear_stiffness(p, CORGI_K_LINEAR),
        phi0=phi0, psi=CORGI_PSI,
    )


def test_corgi_leg_length_matches_phase_0() -> None:
    """phi0 solved from eq 4 reproduces the measured hip-to-arc-center length."""
    p = corgi_params()
    assert p.l0 == pytest.approx(CORGI_L0, rel=1e-9)


def test_corgi_linear_stiffness_round_trips() -> None:
    """k_t <-> equivalent linear stiffness conversion is self-consistent."""
    p = corgi_params()
    assert gslip.linear_stiffness(p) == pytest.approx(CORGI_K_LINEAR, rel=1e-9)


def test_corgi_parameters_admit_a_stride() -> None:
    """The Corgi's Phase 0 geometry admits at least one viable stride.

    SLIP-RF geometry: a telescoping leg (long bars approximate a linear
    spring) on a rolling foot of radius 0.145 m. Rather than hard-coding
    touchdown states, scan the paper's operating ranges and require the
    parameter set to support running somewhere in them.
    """
    p = corgi_params()
    v_scale = np.sqrt(p.g * (p.l0 + p.r))

    found = []
    for beta_deg in range(25, 61, 5):
        for alpha_deg in (0.0, 5.0, 10.0, 15.0):
            for v_tilde in (1.0, 1.25, 1.5):
                try:
                    gslip.stride(
                        p,
                        v_tilde * v_scale,
                        np.deg2rad(alpha_deg),
                        np.deg2rad(beta_deg),
                    )
                except gslip.GSlipFailure:
                    continue
                found.append((beta_deg, alpha_deg, v_tilde))

    assert found, "Corgi parameters produced no viable stride in the scanned range"
