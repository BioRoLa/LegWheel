"""Pins for the Stage 2a turning-envelope script (log section 83).

These pin the seams: the envelope must ride the SAME algebra as the models
(slip_rf_cambered.turn_radius, ackermann_pair, cambered_params) and the same
existence solver as pronk_operating_point -- not local reimplementations
that can drift.
"""
import importlib.util
import sys
from pathlib import Path

import numpy as np

from legwheel.models import slip_rf
from legwheel.models import slip_rf_cambered
from legwheel.models.cambered_return_map import ackermann_pair

GSLIP_DIR = Path(__file__).resolve().parents[1] / "examples" / "gslip"


def _load(name, fname):
    spec = importlib.util.spec_from_file_location(name, GSLIP_DIR / fname)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


env = _load("s2a_env", "stage2a_turning_envelope.py")
pop = _load("pronk_op", "pronk_operating_point.py")


def test_lambda0_existence_matches_pronk_operating_point():
    p = env.base_params()
    v = 0.70 * np.sqrt(env.G * p.l0)      # v~0.70, the operating point
    fp_env = env.solve_existence(p, v, slip_rf.stride, step=1.0)
    fp_pop = pop.solve(pop.slip_rf.SlipRfParams(
        m=p.m, l0=p.l0, k=p.k, r=p.r), v, step=1.0)
    assert fp_env is not None and fp_pop is not None
    assert abs(np.rad2deg(fp_env.beta) - np.rad2deg(fp_pop.beta)) < 0.01
    assert abs(np.rad2deg(fp_env.alpha) - np.rad2deg(fp_pop.alpha)) < 0.01
    # the model's forward speed at v~0.70 is ~0.87 -- faster than the
    # robot's measured 0.726 (the section-27 speed shortfall); pin the
    # model-side value so a silent change to the mapping is caught
    assert abs(fp_env.mean_speed - 0.870) < 0.01


def test_scrub_bound_reproduces_the_measured_phase5_row():
    # psi_dot 0.288 rad/s at v = 0.534 m/s -> 13.8% scrub per rolling distance
    assert abs(env.scrub_fraction(0.534, 0.288) - 0.1375) < 0.002
    # and the upper edge of the measured band
    assert 0.16 < env.scrub_fraction(0.45, 0.29 * 0.45 / 0.534 * 1.25) < 0.20


def test_turn_radius_is_the_model_seam_not_a_local_law():
    assert env.turn_radius is slip_rf_cambered.turn_radius


def test_ackermann_split_matches_the_return_map_pair():
    for lam_deg in (2.0, 5.0, 10.0, 15.0):
        for h in (0.25, 0.30, 0.35):
            a = env.ackermann_split(np.deg2rad(lam_deg), h)
            b = ackermann_pair(np.deg2rad(lam_deg), h)
            assert abs(a[1] - b[1]) < 1e-9


def test_const_r_params_reproduce_cambered_params_on_the_geometric_law():
    p = env.base_params()
    for lam_deg in (0.0, 7.0, 21.0):
        lam = np.deg2rad(lam_deg)
        a = env.cambered_params_const_r(
            p, lam, slip_rf_cambered.rolling_radius(lam))
        b = slip_rf_cambered.cambered_params(p, lam)
        for f in ("m", "l0", "k", "r", "g"):
            assert abs(getattr(a, f) - getattr(b, f)) < 1e-15


def test_coarse_grid_feasible_cells_respect_the_scrub_bound():
    cells = env.run_grid(np.array([0.65, 1.05]), np.array([0.0, 1.0, 20.0]),
                         step=2.0, laws=("empirical",), verbose=False)
    assert any(c["feasible"] for c in cells)          # straight cells at least
    for c in cells:
        if c["feasible"] and c["lam_deg"] > 0:
            assert c["psi"] <= env.PSI_DOT_MAX + 1e-12
        if c["lam_deg"] == 20.0 and c["exists"]:
            # lam 20 at these forward speeds is far past the scrub bound
            assert c["psi"] > env.PSI_DOT_MAX
