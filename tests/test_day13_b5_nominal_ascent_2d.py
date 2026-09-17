"""Day 13 B5: the ascent is flown by the **nominal** swing, not a Cartesian one.

The project owner asked for the flat-ground swing style to be what climbs the
obstacle, keeping Day 8--9 only for the start and end positions.  These tests
pin the two facts that made it possible, because both were assumptions that
had been written down in more than one place and were wrong in each of them.
"""

import numpy as np
import pytest
from dataclasses import replace

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    RecoveryConfig2D,
    _standable_surface_ids,
    run_nominal_cycles_2d,
    standing_stroke_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    swing_hip_advance_m,
)
from hybrid_note.scripts.experiments.day13_b5_nominal_ascent_2d import (
    obstacle_posture_2d,
    run_nominal_ascent_2d,
)


@pytest.fixture(scope="module")
def nominal():
    """The flat nominal cycle every B5 number is measured against."""

    posture = hybrid_posture_2d()
    config = RecoveryConfig2D(
        hip_advance_m=swing_hip_advance_m(hybrid_timing_2d(), posture)
    )
    cycle = run_nominal_cycles_2d(1, posture, config)[0]
    return posture, config, cycle


def test_a_top_is_a_surface_a_leg_may_stand_on(nominal):
    """The ground and any ``_top``; never a vertical face.

    This rule lived in three places as ``(ground_surface_id,)`` and the B5
    landing had to pass all three.  Fixing only the touchdown check produced
    the contradiction of a landing that was accepted and then raised "has no
    ground contact" one frame later.
    """

    posture, _, _ = nominal
    spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40,
                               x_start_m=0.36, arc_samples=121)
    post = obstacle_posture_2d(posture, spec)
    scene = post.scene(0.0, 0.30, post.hip_z_for_flat_stance(0.0))
    ids = _standable_surface_ids(scene)

    assert scene.terrain.ground_surface_id in ids
    assert any(sid.endswith("_top") for sid in ids)
    assert not any(sid.endswith(("_front", "_back")) for sid in ids)


def test_the_obstacle_must_reach_the_swing_through_the_stroke(nominal):
    """``run_recovery_swing_2d`` reads ``posture`` from ``stroke.posture``.

    Three probes failed with ``TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT``
    because they put the obstacle on a posture the generator never consulted.
    That failure string reads like "the leg cannot land there"; it actually
    meant "there is no obstacle in this world".
    """

    posture, _, cycle = nominal
    spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40,
                               x_start_m=0.36, arc_samples=121)
    end = cycle.stroke.end
    flat_stroke = standing_stroke_2d(
        posture, float(end.theta_rad), float(end.beta_rad),
        float(end.hip_xz_m[0]), float(end.hip_xz_m[1]),
    )
    obstacle_stroke = standing_stroke_2d(
        obstacle_posture_2d(posture, spec), float(end.theta_rad),
        float(end.beta_rad), float(end.hip_xz_m[0]), float(end.hip_xz_m[1]),
    )

    assert flat_stroke.posture.obstacle_xwh_m is None
    assert obstacle_stroke.posture.obstacle_xwh_m == (0.36, 0.40, 0.04)


def test_the_nominal_swing_lands_on_the_obstacle_top(nominal):
    """B5's whole claim, as one number: the landing z is the top's z.

    No Day 8--9 Cartesian swing is involved -- the frames come from
    ``run_recovery_swing_2d``, the same generator flat locomotion uses.
    """

    posture, config, cycle = nominal
    end, land = cycle.stroke.end, cycle.recovery.frames[-1]
    advance = float(land.hip_xz_m[0]) - float(end.hip_xz_m[0])
    spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40,
                               x_start_m=0.36, arc_samples=121)

    out = run_nominal_ascent_2d(
        spec, posture, config,
        approach_hip_x_m=float(end.hip_xz_m[0]),
        landing_hip_x_m=float(end.hip_xz_m[0]) + advance,
        beta_takeoff_rad=float(end.beta_rad),
        beta_landing_rad=float(land.beta_rad),
        theta_takeoff_rad=float(end.theta_rad),
    )

    assert out.success, out.refusal
    landing_x, landing_z = out.landing_contact_xz_m
    assert landing_z == pytest.approx(spec.height_m, abs=1e-4)
    # and it landed ON the top, not short of the face or past the back
    assert spec.x_start_m < landing_x < spec.x_start_m + spec.top_length_m


def test_the_swing_retracts_rather_than_reaching_over_the_edge(nominal):
    """Why the owner wanted this style: the leg is shortest at the crossing.

    The Cartesian ascent's theta runs 60 -> 36.6 -> 58.7, so the leg is near
    full length as it passes the leading edge.  The nominal swing retracts to
    the compact posture and stays there through the rotation.
    """

    posture, config, cycle = nominal
    end, land = cycle.stroke.end, cycle.recovery.frames[-1]
    advance = float(land.hip_xz_m[0]) - float(end.hip_xz_m[0])
    spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40,
                               x_start_m=0.36, arc_samples=121)

    out = run_nominal_ascent_2d(
        spec, posture, config,
        approach_hip_x_m=float(end.hip_xz_m[0]),
        landing_hip_x_m=float(end.hip_xz_m[0]) + advance,
        beta_takeoff_rad=float(end.beta_rad),
        beta_landing_rad=float(land.beta_rad),
        theta_takeoff_rad=float(end.theta_rad),
    )
    assert out.success, out.refusal

    thetas = np.rad2deg([f.theta_rad for f in out.swing.frames])
    rotation = [f for f in out.swing.frames if f.phase == "RECOVERY_ROTATE"]
    assert rotation, "the nominal swing must have a rotation phase"
    assert np.allclose(
        np.rad2deg([f.theta_rad for f in rotation]),
        np.rad2deg(config.theta_compact_rad),
    ), "the leg must stay compact through the rotation"
    assert thetas.min() == pytest.approx(
        np.rad2deg(config.theta_compact_rad), abs=1e-6
    )


def test_a_refusal_still_reports_the_question_it_refused(nominal):
    """An ascent that cannot start records where it was standing.

    "It refused" is not a measurement without the question; a refusal that
    drops the geometry is what turned a 3 mm scan-window error into an
    apparent height limit (log 34.7).
    """

    posture, config, cycle = nominal
    end, land = cycle.stroke.end, cycle.recovery.frames[-1]
    # an obstacle placed underneath the approach pose itself
    spec = SharedTerrainSpec2D(height_m=0.19, top_length_m=0.40,
                               x_start_m=0.10, arc_samples=121)

    out = run_nominal_ascent_2d(
        spec, posture, config,
        approach_hip_x_m=float(end.hip_xz_m[0]),
        landing_hip_x_m=float(land.hip_xz_m[0]),
        beta_takeoff_rad=float(end.beta_rad),
        beta_landing_rad=float(land.beta_rad),
        theta_takeoff_rad=float(end.theta_rad),
    )

    assert not out.success
    assert out.refusal
    assert out.obstacle_xwh_m == (0.10, 0.40, 0.19)
    assert np.isfinite(out.start_contact_xz_m).all() or out.start_contact_xz_m
