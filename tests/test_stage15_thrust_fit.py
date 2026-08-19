"""Pins for the two-parameter camber-thrust fit (log section 81).

The synthetic-recovery and refusal behaviour also runs inside the example
script's selftest; these tests repeat it under pytest and pin the
one-parameter fit's keys, which downstream tooling reads.
"""
import importlib.util
import sys
from pathlib import Path

import numpy as np

MODULE_PATH = (Path(__file__).resolve().parents[1] / "examples" / "gslip"
               / "stage15_zleg_camber_analysis.py")
SPEC = importlib.util.spec_from_file_location("s15", MODULE_PATH)
s15 = importlib.util.module_from_spec(SPEC)
sys.modules["s15"] = s15          # dataclasses resolve via sys.modules
SPEC.loader.exec_module(s15)

B0_TRUE = {500.0: 0.5e-3, 90.0: 1.5e-3}
C_TRUE, SAT_TRUE = 3.0e-3, 1.6e-3


def _fake(lam, kp, y_mean, n=64):
    arr = np.zeros((n, 3))
    arr[:, 1] = y_mean
    return {"lam_cmd": lam, "kp": kp,
            "lean_ach_rad": float(np.deg2rad(lam)), "yaw_mean": 0.0,
            "rms": {"calibrated": np.zeros(3)},
            "_internals": {"models": {"calibrated":
                                      {leg: arr for leg in range(4)}}}}


def _corpus():
    return [_fake(lam, kp,
                  B0_TRUE[kp] + min(C_TRUE * np.deg2rad(lam), SAT_TRUE))
            for kp in (500.0, 90.0)
            for lam in (0.0, 10.0, 20.0, 30.0, 35.0, 40.0)]


def test_two_parameter_fit_recovers_known_constants():
    fit = s15.fit_camber_thrust_2p(_corpus(), verbose=False)
    assert "refused" not in fit
    assert abs(fit["b0"][500.0] - B0_TRUE[500.0]) < 1e-9
    assert abs(fit["b0"][90.0] - B0_TRUE[90.0]) < 1e-9
    assert abs(fit["c2"] - C_TRUE) < 1e-6
    assert abs(fit["v_sat2"] - SAT_TRUE) < 1e-6
    assert fit["rms_2p"] < 1e-9


def test_kp_group_without_lambda0_is_excluded_and_does_not_bend_the_fit():
    clean = s15.fit_camber_thrust_2p(_corpus(), verbose=False)
    poisoned = _corpus() + [_fake(20.0, 250.0, 9.9e-3)]
    fit = s15.fit_camber_thrust_2p(poisoned, verbose=False)
    assert fit["dropped_kp"] == [250.0]
    assert abs(fit["c2"] - clean["c2"]) < 1e-12
    assert abs(fit["v_sat2"] - clean["v_sat2"]) < 1e-12


def test_fit_refuses_rather_than_guessing():
    # too few runs
    assert "refused" in s15.fit_camber_thrust_2p(_corpus()[:3],
                                                 verbose=False)
    # no kp group carries a lambda=0 anchor
    no_anchor = [_fake(lam, 250.0, 1e-3) for lam in (10.0, 20.0, 30.0, 40.0)]
    assert "refused" in s15.fit_camber_thrust_2p(no_anchor, verbose=False)


def test_one_parameter_fit_keys_survive(capsys):
    fit = s15.fit_camber_thrust(_corpus())
    capsys.readouterr()
    for key in ("c_tan", "rms_tan", "c_lin", "rms_lin"):
        assert key in fit
