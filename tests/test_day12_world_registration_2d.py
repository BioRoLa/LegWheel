"""Day 12 problems A5/C4/C5: the crossing, put where the obstacle is.

The module had no tests of its own.  The three properties locked here are the
ones the four-leg run depends on and that were each measured wrong first:

* time comes from **position**, and the position rule reproduces the gait's own
  stance and swing windows rather than replacing them (log 1.15);
* the recovery's hip advance is not a free parameter -- zero makes a recovery
  take no time at all once time is read from position (C5);
* the crossing hands the gait back a **phase**, not always the arc's start,
  because two legs that share a ``mount_x`` otherwise leave the crossing in
  lockstep (log 1.16, 1.19).

Nothing here builds a four-leg chain: that is minutes of generator time.  What
is checked is the arithmetic those chains are assembled from, plus one resumed
run, which is the piece that had no way to be wrong quietly.
"""

import math

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    RecoveryConfig2D,
    nominal_stroke_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import GAMMA_RAD
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    TWO_PI,
    body_speed_m_s,
    cycle_contact_advance_m,
    phase_start_beta_rad,
    phase_start_pose_2d,
    resume_phases_2d,
    run_resumed_cycles_2d,
    stroke_hip_per_contact_ratio,
    swing_hip_advance_m,
)


@pytest.fixture(scope="module")
def setup():
    timing = hybrid_timing_2d()
    posture = hybrid_posture_2d()
    config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    return timing, posture, config


# --------------------------------------------------------------------------
# C5: the recovery's hip advance is what makes the cycle add up
# --------------------------------------------------------------------------


def test_the_swing_hip_advance_is_what_closes_the_cycle(setup):
    """``stroke + swing = period * speed`` is the whole definition."""

    timing, posture, _ = setup
    stroke = nominal_stroke_2d(posture)
    duty = timing.stance_duty
    assert swing_hip_advance_m(timing, posture) == pytest.approx(
        stroke.hip_advance_m * (1.0 - duty) / duty)


def test_a_recovery_that_does_not_move_the_hip_takes_no_time(setup):
    """Why the generator's default of zero cannot be left alone.

    Position scheduling reads a segment's duration off the hip travel, so a
    recovery with ``hip_advance_m = 0`` is a swing window of zero length -- the
    gait's duty is silently 1.0 no matter what ``timing`` says.
    """

    timing, posture, config = setup
    default = RecoveryConfig2D()
    assert default.hip_advance_m == 0.0
    assert config.hip_advance_m > 0.0
    with_swing = body_speed_m_s(timing, posture, config)
    without = body_speed_m_s(timing, posture, default)
    assert with_swing > without


def test_position_scheduling_reproduces_the_gaits_own_windows(setup):
    """The rule is "hip travel / body speed" and nothing else, so it has to
    give back the stance and swing durations the gait already declares."""

    timing, posture, config = setup
    speed = body_speed_m_s(timing, posture, config)
    stroke = nominal_stroke_2d(posture)
    stance_s = stroke.hip_advance_m / speed
    swing_s = swing_hip_advance_m(timing, posture) / speed
    assert stance_s == pytest.approx(timing.stance_duration_s, abs=1e-9)
    assert swing_s == pytest.approx(timing.swing_duration_s, abs=1e-9)
    assert stance_s + swing_s == pytest.approx(timing.cycle_period_s, abs=1e-9)


def test_a_cycle_advances_the_contact_further_than_a_stroke_does(setup):
    """The recovery's touchdown puts the foot down ahead of where it lifted
    off, and counting cycles with the stroke's number is what put every leg
    past the platform on the first assembly."""

    _, posture, config = setup
    assert (cycle_contact_advance_m(posture, config)
            > nominal_stroke_2d(posture).contact_advance_m)
    assert stroke_hip_per_contact_ratio(posture) > 1.0


# --------------------------------------------------------------------------
# The phase the crossing has to hand back
# --------------------------------------------------------------------------


def test_a_phase_pose_is_a_pose_the_stroke_actually_passes_through(setup):
    """Beta alone is not enough to aim a transition at: the levelled posture
    solves theta against the held hip height at every beta."""

    timing, posture, _ = setup
    stroke = nominal_stroke_2d(posture)
    held = float(stroke.frames[0].hip_xz_m[1])
    for phase in (0.0, 0.25, 0.5, 0.85):
        theta, beta, hip_z = phase_start_pose_2d(phase, timing, posture)
        assert hip_z == pytest.approx(held)
        assert beta == pytest.approx(phase_start_beta_rad(phase, timing,
                                                          posture))
        lo = min(float(stroke.frames[0].beta_rad),
                 float(stroke.frames[-1].beta_rad))
        hi = max(float(stroke.frames[0].beta_rad),
                 float(stroke.frames[-1].beta_rad))
        assert lo - 1e-9 <= beta <= hi + 1e-9
    # And theta is not constant along it, which is the reason the pose is
    # asked for as a pose rather than assembled from the arc start's theta.
    thetas = [phase_start_pose_2d(p, timing, posture)[0]
              for p in (0.0, 0.4, 0.85)]
    assert max(thetas) - min(thetas) > np.deg2rad(1.0)


def test_phase_zero_is_the_arcs_own_start(setup):
    timing, posture, _ = setup
    theta, beta, _ = phase_start_pose_2d(0.0, timing, posture)
    start = nominal_stroke_2d(posture).frames[0]
    assert beta == pytest.approx(float(start.beta_rad))
    assert theta == pytest.approx(float(start.theta_rad), abs=1e-6)


def test_every_leg_can_reach_the_phase_it_is_asked_to_resume_at(setup):
    """A stroke can only be **shortened**, so a resume phase past the duty is
    not reachable.  The free constant is what avoids that band."""

    timing, posture, config = setup
    phases = resume_phases_2d(timing, posture, config)
    assert set(phases) == set(LEG_ORDER)
    for leg, phase in phases.items():
        assert 0.0 <= phase <= timing.stance_duty, leg.value


def test_the_resume_phases_put_the_four_legs_a_quarter_cycle_apart(setup):
    """The point of the exercise, stated as the arithmetic it rests on.

    A leg leaves the crossing at ``K - mount_x / speed`` for one common ``K``
    -- all four leave it at the same *hip* -- and then rolls the rest of its
    stroke.  Those two together have to land it on its gait slot.
    """

    timing, posture, config = setup
    phases = resume_phases_2d(timing, posture, config)
    speed = body_speed_m_s(timing, posture, config)
    mounts = {m.leg: float(m.offset_body_xyz_m[0])
              for m in leg_mounts_2d(GAMMA_RAD)}
    period = float(timing.cycle_period_s)

    swing_at = {}
    for leg, phase in phases.items():
        exit_s = -mounts[leg] / speed
        swing_at[leg] = math.fmod(
            exit_s + (timing.stance_duty - phase) * period + 4.0 * period,
            period)

    slots = sorted(swing_at.values())
    spacing = [b - a for a, b in zip(slots, slots[1:])]
    spacing.append(slots[0] + period - slots[-1])
    for gap in spacing:
        assert gap == pytest.approx(period / 4.0, abs=1e-9)


def test_the_gait_order_survives_the_crossing(setup):
    """Not just *some* quarter-cycle order: the one the flat gait walks in."""

    timing, posture, config = setup
    phases = resume_phases_2d(timing, posture, config)
    speed = body_speed_m_s(timing, posture, config)
    mounts = {m.leg: float(m.offset_body_xyz_m[0])
              for m in leg_mounts_2d(GAMMA_RAD)}
    period = float(timing.cycle_period_s)

    after = sorted(
        LEG_ORDER,
        key=lambda leg: math.fmod(
            -mounts[leg] / speed
            + (timing.stance_duty - phases[leg]) * period + 4.0 * period,
            period))
    before = sorted(LEG_ORDER, key=lambda leg: math.fmod(
        (1.0 - float(timing.phase_at(leg, 0.0))) + 2.0, 1.0))

    # Compare as cyclic orders: which leg follows which, not who is first.
    def successor(order):
        return {order[i]: order[(i + 1) % len(order)]
                for i in range(len(order))}

    assert successor(after) == successor(before)


# --------------------------------------------------------------------------
# Resuming part way through a stroke
# --------------------------------------------------------------------------


def test_a_resumed_run_starts_short_and_then_runs_full(setup):
    """The first stroke is the partial one; everything after it is a cycle.

    ``run_nominal_cycles_2d`` cannot do this -- it builds one cycle and
    translates it, so a partial first cycle would make every later one partial
    too -- which is the whole reason this function exists.
    """

    timing, posture, config = setup
    stroke = nominal_stroke_2d(posture)
    arc_start = float(stroke.frames[0].beta_rad)
    phases = resume_phases_2d(timing, posture, config)
    leg = LEG_ORDER[0]
    _, beta, _ = phase_start_pose_2d(phases[leg], timing, posture)

    turns = -3          # a leg that has been rolling: beta is a counter
    cycles = run_resumed_cycles_2d(
        2, posture, config, hip_x_m=1.0,
        start_beta_rad=beta + turns * TWO_PI,
        arc_start_beta_rad=arc_start + turns * TWO_PI)

    assert len(cycles) == 2
    assert all(c.recovery.success for c in cycles), [
        c.recovery.failure_reason for c in cycles]
    assert cycles[0].stroke.hip_advance_m < cycles[1].stroke.hip_advance_m
    assert cycles[1].stroke.hip_advance_m == pytest.approx(
        stroke.hip_advance_m, abs=1e-9)
    # The partial stroke is shorter *because* of the phase, not by accident.
    assert cycles[0].stroke.hip_advance_m == pytest.approx(
        stroke.hip_advance_m * (timing.stance_duty - phases[leg])
        / timing.stance_duty, rel=0.10)


def test_a_run_told_no_arc_start_is_a_plain_nominal_run(setup):
    """``arc_start_beta_rad=None`` means "this is not a partial start", and
    the recovery is then left to aim where it always did."""

    _, posture, config = setup
    stroke = nominal_stroke_2d(posture)
    cycles = run_resumed_cycles_2d(
        1, posture, config, hip_x_m=0.0,
        start_beta_rad=float(stroke.frames[0].beta_rad))
    assert cycles[0].recovery.success
    assert cycles[0].stroke.hip_advance_m == pytest.approx(
        stroke.hip_advance_m, abs=1e-9)
