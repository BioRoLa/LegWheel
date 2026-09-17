"""Day 14 Step 1: the gait clock.

Cheap on purpose: the clock is arithmetic on a timing and a speed, so these
tests hand it a speed rather than generating a nominal cycle to measure one.
The regression against the frozen flat gait lives in
``test_day14_whole_body_planner_2d.py`` and does generate.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    LIFTOFF_SEQUENCES,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import SpeedZone2D
from hybrid_note.scripts.experiments.day14_gait_clock_2d import (
    GaitClock2D,
    SwingEvent2D,
    liftoff_order_2d,
    minimum_swing_duration_s,
)


@pytest.fixture
def clock():
    return GaitClock2D(timing=hybrid_timing_2d(), speed_m_s=0.16, origin_x_m=0.5)


def test_the_liftoff_order_is_the_gait_files_own(clock):
    """FL -> RR -> FR -> RL, read off the windows rather than tabulated."""

    assert liftoff_order_2d(clock.timing) == LIFTOFF_SEQUENCES["project_walk"]
    assert clock.liftoff_order == LIFTOFF_SEQUENCES["project_walk"]


def test_consecutive_liftoffs_are_a_quarter_cycle_apart(clock):
    starts = sorted(clock.timing.swing_window(leg)[0] for leg in LEG_ORDER)
    gaps = np.diff(starts + [starts[0] + 1.0])
    assert np.allclose(gaps, 0.25)
    assert clock.nominal_liftoff_spacing_s == pytest.approx(0.6)


def test_a_chain_starts_where_its_phase_says(clock):
    """A leg ``phi`` through its cycle at t = 0 began ``phi * T`` earlier."""

    for leg in LEG_ORDER:
        phi = clock.timing.phase_offsets[leg.index]
        assert clock.chain_start_time_s(leg) == pytest.approx(-phi * 2.4)
    # The frozen flat schedule's four chain starts, to the millisecond.
    starts = {leg.value: round(clock.chain_start_time_s(leg), 6) for leg in LEG_ORDER}
    assert starts == {"LF": -2.04, "RF": -0.84, "LH": -0.24, "RH": -1.44}


def test_time_and_position_are_exact_inverses_without_zones(clock):
    for x in np.linspace(-1.0, 3.0, 41):
        t = clock.time_at_body_x(x)
        assert clock.body_x_at_time(t) == pytest.approx(x, abs=1e-12)
    # Behind the origin the body ran at nominal speed: negative time, no clamp.
    assert clock.time_at_body_x(0.5 - 0.16) == pytest.approx(-1.0)


def test_a_zone_slows_exactly_its_own_stretch_and_nothing_else(clock):
    slow = clock.with_zone(SpeedZone2D(1.0, 1.2, 0.08))
    # Up to the zone: unchanged.
    assert slow.time_at_body_x(1.0) == pytest.approx(clock.time_at_body_x(1.0))
    # Through it: twice as long.
    assert slow.time_at_body_x(1.2) - slow.time_at_body_x(1.0) == pytest.approx(2.5)
    # After it: the same speed again, offset by the extra time spent.
    extra = 0.2 / 0.08 - 0.2 / 0.16
    assert slow.time_at_body_x(2.0) == pytest.approx(clock.time_at_body_x(2.0) + extra)
    for x in np.linspace(0.0, 2.5, 51):
        assert slow.body_x_at_time(slow.time_at_body_x(x)) == pytest.approx(x, abs=1e-12)
    assert slow.speed_at_body_x(1.1) == pytest.approx(0.08)
    assert slow.speed_at_body_x(1.3) == pytest.approx(0.16)


def test_a_dwell_holds_time_at_one_body_x_and_orders_the_pauses_there(clock):
    """The body stands while a leg swings in place; a second pause at the
    same x follows the first; a stroke ending there ends before them and
    the next one starts after; nothing off that x moves in time except by
    the dwell's whole duration."""

    with_zone = clock.with_zone(SpeedZone2D(0.5, 0.6, 0.08))
    first, s1, e1 = with_zone.with_dwell(0.55, 0.3)
    second, s2, e2 = first.with_dwell(0.55, 0.2)
    assert (s1, e1) == pytest.approx((with_zone.time_at_body_x(0.55), with_zone.time_at_body_x(0.55) + 0.3))
    assert (s2, e2) == pytest.approx((e1, e1 + 0.2))
    assert second.dwell_window_s(0) == pytest.approx((s1, e1))
    assert second.dwell_window_s(1) == pytest.approx((s2, e2))
    assert second.time_at_body_x(0.55) == pytest.approx(s1)
    assert second.time_at_body_x(0.55, side="leave") == pytest.approx(e2)
    # Inside the dwell the body is at its x; outside, the inverse is exact.
    assert second.body_x_at_time(0.5 * (s1 + e2)) == pytest.approx(0.55)
    for x in (0.2, 0.5, 0.54, 0.56, 0.9):
        assert second.body_x_at_time(second.time_at_body_x(x)) == pytest.approx(x)
        shift = 0.5 if x > 0.55 else 0.0
        assert second.time_at_body_x(x) == pytest.approx(with_zone.time_at_body_x(x) + shift)
    # A dwell placed later at an earlier x shifts the later ones, and the
    # windows are read off the final clock, not remembered.
    third, s3, e3 = second.with_dwell(0.3, 0.1)
    assert (s3, e3) == pytest.approx((with_zone.time_at_body_x(0.3), with_zone.time_at_body_x(0.3) + 0.1))
    assert third.dwell_window_s(0) == pytest.approx((s1 + 0.1, e1 + 0.1))
    assert third.dwell_time_s == pytest.approx(0.6)


def test_overlapping_zones_are_refused_not_merged(clock):
    slow = clock.with_zone(SpeedZone2D(1.0, 1.2, 0.08))
    with pytest.raises(ValueError):
        slow.with_zone(SpeedZone2D(1.1, 1.3, 0.08))


def test_swing_events_overlap_in_body_x_not_in_time():
    """Two swings whose body-x intervals meet do not overlap; crossing ones do."""

    a = SwingEvent2D(LegId.LF, 0.0, 0.05, 0.0, 0.3, "RECOVERY_SWING", 0.3)
    b = SwingEvent2D(LegId.RH, 0.05, 0.10, 0.3, 0.6, "RECOVERY_SWING", 0.3)
    c = SwingEvent2D(LegId.RF, 0.04, 0.09, 0.25, 0.55, "RECOVERY_SWING", 0.3)
    assert not a.overlaps(b)
    assert a.overlaps(c)
    assert a.hip_advance_m == pytest.approx(0.05)


class _Frame:
    def __init__(self, theta_deg, beta_deg):
        self.theta_rad = np.deg2rad(theta_deg)
        self.beta_rad = np.deg2rad(beta_deg)


def test_the_minimum_swing_duration_checks_each_motor_on_its_own():
    """``phi_r = theta' + beta'`` and ``phi_l = beta' - theta'`` separately.

    Joints moving oppositely cancel on one motor and add on the other; the
    shorthand ``|dtheta| + |dbeta|`` would overstate it.
    """

    frames = [_Frame(60.0, 0.0), _Frame(40.0, -20.0), _Frame(20.0, -40.0)]
    # Each step: dtheta = -20, dbeta = -20 -> phi_r step -40 deg, phi_l step 0.
    limit = np.deg2rad(1980.0)
    expected = 2 * np.deg2rad(40.0) / (limit * 1.0)
    assert minimum_swing_duration_s(frames, utilisation=1.0) == pytest.approx(expected)
    # With 95% utilisation the same motion needs a little longer.
    assert minimum_swing_duration_s(frames) == pytest.approx(expected / 0.95)
    assert minimum_swing_duration_s(frames[:1]) == 0.0
