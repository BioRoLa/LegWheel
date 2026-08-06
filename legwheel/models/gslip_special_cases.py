"""The four special cases of the G-SLIP model (Lu & Lin 2024, appendix).

Table 1 gives parameter sets that reduce the G-SLIP model to SLIP, SLIP-RF,
TSL and R-SLIP. The appendix gives each model's kinematics independently
(eqs 21-38). Both are reproduced here so `tests/test_gslip.py` can check the
reduction, which is the validation gate for `legwheel.models.gslip`.
"""

from __future__ import annotations

import numpy as np

from legwheel.models.gslip import GSlipParams

# Table 1: parameters of the four special cases, as G-SLIP parameters.
TABLE_1 = {
    "SLIP": dict(m=7.78, k_t=4221.82, l1=1.0, l2=1.0, phi0=0.150, psi=0.0, r=0.0),
    "SLIP-RF": dict(m=7.78, k_t=4221.82, l1=1.0, l2=1.0, phi0=0.150, psi=1.496, r=0.075),
    "TSL": dict(m=7.78, k_t=22.80, l1=0.082, l2=0.126, phi0=1.571, psi=0.0, r=0.0),
    "R-SLIP": dict(m=7.78, k_t=22.80, l1=0.082, l2=0.126, phi0=1.571, psi=0.576, r=0.075),
}


def table_1_params(name: str) -> GSlipParams:
    """G-SLIP parameters for one of the four special cases."""
    return GSlipParams(**TABLE_1[name])


# --------------------------------------------------------------------------
# Independent kinematics from the appendix.
#
# Each returns (x, z) of the point mass relative to the initial contact point,
# so it can be compared against `gslip.stance_position`.
# --------------------------------------------------------------------------


def tsl_position(l1: float, l2: float, theta: float, phi: float) -> tuple[float, float]:
    """TSL model, eq 25."""
    return (
        l1 * np.cos(theta) - l2 * np.cos(phi - theta),
        l1 * np.sin(theta) + l2 * np.sin(phi - theta),
    )


def tsl_l0(l1: float, l2: float, phi0: float) -> float:
    """TSL rest length, eq 29."""
    return float(np.sqrt(l1**2 + l2**2 + 2 * l1 * l2 * np.cos(phi0)))


def rslip_position(
    l: float, r: float, theta: float, phi: float, theta0: float, phi0: float
) -> tuple[float, float]:
    """R-SLIP model, eq 35."""
    return (
        r * (phi - phi0 - theta + theta0) + r * np.cos(phi - theta) + l * np.cos(theta),
        r + r * np.sin(phi - theta) + l * np.sin(theta),
    )


def slip_position(length: float, phi: float) -> tuple[float, float]:
    """SLIP model, eq 21. phi is measured from vertical."""
    return length * np.sin(phi), length * np.cos(phi)


def slip_rf_position(
    length: float, r: float, phi: float, phi0: float
) -> tuple[float, float]:
    """SLIP-RF model, eq 31. phi is measured from vertical."""
    return (
        r * (phi - phi0) + (length - r) * np.sin(phi),
        r + (length - r) * np.cos(phi),
    )
