"""Pins for the pair-split analysis (log section 85)."""
import importlib.util
import sys
from pathlib import Path

GSLIP_DIR = Path(__file__).resolve().parents[1] / "examples" / "gslip"

SPEC = importlib.util.spec_from_file_location(
    "s2a_pair", GSLIP_DIR / "stage2a_pair_split.py")
pair = importlib.util.module_from_spec(SPEC)
sys.modules["s2a_pair"] = pair
SPEC.loader.exec_module(pair)


def test_track_is_the_contact_track_not_the_hip_spacing():
    # w = 0.2117 comes from the 0.4234 m contact track (section 23), the
    # correction that turned ~7.5% into 23.7% at R = 2
    assert abs(pair.W_HALF_TRACK - 0.2117) < 1e-4


def test_mismatch_reproduces_the_risk_register_number():
    assert abs(pair.mismatch_pct(2.0) - 0.2367) < 0.001


def test_split_fraction_at_the_envelope_radii():
    assert abs(pair.split_fraction(2.88) - 0.0735) < 0.001
    assert abs(pair.split_fraction(3.6) - 0.0588) < 0.001
    # split shrinks with radius, vanishes in the limit
    assert pair.split_fraction(10.0) < pair.split_fraction(3.0)


def test_selftest_passes():
    pair._selftest()
