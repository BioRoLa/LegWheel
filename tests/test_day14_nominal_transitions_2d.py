"""Day 14 Step 2: every terrain-transition swing flown the nominal way.

What these pin, measured 2026-09-07 on the 40 mm platform from the nominal
stroke's own end pose:

* an UP lands **on the top** at the held height, with the terrain-aware
  rotation clearance (40--71 mm against a 10 mm requirement);
* a DOWN lands on the ground past the trailing face;
* the airborne frames now see the obstacle: a takeoff standing in the face is
  refused, and a swing that would pass through the block is refused for the
  clearance it lost, not accepted with a clean 74 mm as before Day 14;
* the landing is the arc start at the held height, so the next stroke begins
  exactly where a nominal one would.
"""

import numpy as np
import pytest

from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    RecoveryConfig2D,
    _terrain_clearance_m,
    nominal_stroke_2d,
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
)
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
    SEGMENT_KIND_OF_TRANSITION,
    TransitionKind2D,
    arc_start_landing_beta_rad,
    held_landing_pose_2d,
    run_nominal_transition_2d,
    transition_segment_2d,
)


@pytest.fixture(scope="module")
def nominal():
    posture = hybrid_posture_2d()
    config = RecoveryConfig2D(
        hip_advance_m=swing_hip_advance_m(hybrid_timing_2d(), posture))
    stroke = nominal_stroke_2d(posture)
    return posture, config, stroke


def _spec(height_m, face_x_m, top_length_m=0.60):
    return SharedTerrainSpec2D(height_m=height_m, top_length_m=top_length_m,
                               x_start_m=face_x_m, arc_samples=121)


def _up(nominal, *, height_m, face_offset_m, landing_offset_m):
    posture, config, stroke = nominal
    end = stroke.end
    hx, hz = float(end.hip_xz_m[0]), float(end.hip_xz_m[1])
    spec = _spec(height_m, hx + face_offset_m)
    return spec, run_nominal_transition_2d(
        spec, posture, config, kind=TransitionKind2D.UP,
        takeoff_theta_rad=float(end.theta_rad), takeoff_beta_rad=float(end.beta_rad),
        takeoff_hip_xz_m=(hx, hz), landing_hip_x_m=spec.x_start_m + landing_offset_m)


def test_an_up_lands_on_the_top_at_the_held_height(nominal):
    posture, _, _ = nominal
    spec, out = _up(nominal, height_m=0.04, face_offset_m=0.06, landing_offset_m=0.06)
    assert out.success, out.refusal
    x, z = out.landing_contact_xz_m
    assert z == pytest.approx(spec.top_z_m, abs=1e-4)
    assert spec.x_start_m < x < spec.x_max_m
    end = out.swing.end
    assert end.hip_xz_m[1] == pytest.approx(posture.hold_hip_z_m + spec.height_m, abs=1e-6)
    assert end.surface_id.endswith("_top")
    assert end.rim == RimId.FOOT.value


def test_the_up_clearance_is_measured_against_the_obstacle(nominal):
    """Before Day 14 an airborne frame saw the ground plane only (74 mm)."""

    spec, out = _up(nominal, height_m=0.04, face_offset_m=0.06, landing_offset_m=0.06)
    assert out.success, out.refusal
    assert 0.010 < out.rotation_clearance_m < 0.074
    rotating = [f for f in out.swing.frames if f.phase == "RECOVERY_ROTATE"]
    assert all(not f.collision for f in out.swing.frames)
    assert min(f.clearance_m for f in rotating) == pytest.approx(
        out.rotation_clearance_m)


def test_a_takeoff_standing_in_the_face_is_refused(nominal):
    _, out = _up(nominal, height_m=0.04, face_offset_m=0.02, landing_offset_m=0.06)
    assert not out.success
    assert out.refusal.startswith("TAKEOFF_POSE_REFUSED")
    # The refusal still says what was asked.
    assert out.obstacle_xwh_m[2] == pytest.approx(0.04)
    assert np.isfinite(out.takeoff_hip_xz_m[0])


def test_a_swing_through_the_block_is_refused_for_its_clearance(nominal):
    """A nominal recovery landing on the ground with a 100 mm block in its path.

    The compact leg is a 145 mm wheel under a hip 219 mm up, so it clears
    75 mm of block and not 100.  Landing on the ground *behind* the block
    asks the swing to pass through it; the terrain-aware frames refuse it.
    """

    posture, config, stroke = nominal
    end = stroke.end
    hx, hz = float(end.hip_xz_m[0]), float(end.hip_xz_m[1])
    # 130 mm ahead the standing pose is legal (the scan needs >= 120 at 100 mm).
    spec = SharedTerrainSpec2D(height_m=0.10, top_length_m=0.05,
                               x_start_m=hx + 0.13, arc_samples=121)
    out = run_nominal_transition_2d(
        spec, posture, config, kind=TransitionKind2D.OVER,
        takeoff_theta_rad=float(end.theta_rad), takeoff_beta_rad=float(end.beta_rad),
        takeoff_hip_xz_m=(hx, hz), landing_hip_x_m=hx + 0.20)
    assert not out.success, out.as_dict()
    assert not out.refusal.startswith("TAKEOFF_POSE_REFUSED"), out.refusal
    assert "CLEARANCE" in out.refusal or "PENETRATES" in out.refusal


def test_a_down_lands_on_the_ground_past_the_trailing_face(nominal):
    posture, config, stroke = nominal
    end = stroke.end
    hx, hz = float(end.hip_xz_m[0]), float(end.hip_xz_m[1])
    height = 0.04
    spec = SharedTerrainSpec2D(height_m=height, top_length_m=0.26,
                               x_start_m=hx - 0.20, arc_samples=121)
    out = run_nominal_transition_2d(
        spec, posture, config, kind=TransitionKind2D.DOWN,
        takeoff_theta_rad=float(end.theta_rad), takeoff_beta_rad=float(end.beta_rad),
        takeoff_hip_xz_m=(hx, hz + height), landing_hip_x_m=spec.x_max_m + 0.06)
    assert out.success, out.refusal
    x, z = out.landing_contact_xz_m
    assert z == pytest.approx(spec.ground_height_m, abs=1e-4)
    assert x > spec.x_max_m
    assert out.swing.end.hip_xz_m[1] == pytest.approx(posture.hold_hip_z_m, abs=1e-6)


def test_the_landing_is_the_arc_start_forward_of_the_takeoff(nominal):
    posture, _, stroke = nominal
    start = float(stroke.start.beta_rad)
    # From the full stroke's end: one turn on from the arc start.
    assert arc_start_landing_beta_rad(posture, float(stroke.end.beta_rad)) == pytest.approx(
        start - 2.0 * np.pi)
    # From a stroke cut short: still forward, and further round.
    mid = float(stroke.frames[24].beta_rad)
    target = arc_start_landing_beta_rad(posture, mid)
    assert target < mid
    assert (target - start) / (2.0 * np.pi) == pytest.approx(round((target - start) / (2.0 * np.pi)))
    theta, hip_z = held_landing_pose_2d(posture, target, 0.04)
    assert hip_z == pytest.approx(posture.hold_hip_z_m + 0.04)
    assert theta == pytest.approx(float(stroke.start.theta_rad), abs=1e-6)


def test_the_terrain_clearance_is_signed_and_exact_for_a_box(nominal):
    posture, _, stroke = nominal
    end = stroke.end
    # Standing 100 mm short of a 100 mm block the leg's front is *inside* it
    # (the scan refuses that stance); 150 mm short it is legal, and the gap is
    # then the contact's own ~0.
    spec = _spec(0.10, float(end.hip_xz_m[0]) + 0.15)
    post = obstacle_posture_2d(posture, spec)
    scene = post.scene(end.beta_rad, end.hip_xz_m[0], end.hip_xz_m[1], theta_rad=end.theta_rad)
    assert abs(_terrain_clearance_m(scene)) < 2e-3
    close = _spec(0.10, float(end.hip_xz_m[0]) + 0.10)
    scene = obstacle_posture_2d(posture, close).scene(
        end.beta_rad, end.hip_xz_m[0], end.hip_xz_m[1], theta_rad=end.theta_rad)
    assert _terrain_clearance_m(scene) < -2e-3
    # Buried in the block: negative.
    inside = post.scene(0.0, spec.x_start_m + 0.20, 0.05, theta_rad=np.deg2rad(17.0))
    assert _terrain_clearance_m(inside) < -0.02


def test_a_transition_writes_as_the_terrain_kind_it_is(nominal):
    spec, out = _up(nominal, height_m=0.04, face_offset_m=0.06, landing_offset_m=0.06)
    assert out.success, out.refusal
    kind = SEGMENT_KIND_OF_TRANSITION[TransitionKind2D.UP]
    segment = transition_segment_2d(out.swing, kind=kind, source_id="t", frame_offset=7)
    assert segment.kind is SegmentKind.SWING_UP
    assert segment.kind.is_terrain_transition and segment.kind.is_swing
    assert segment.frames.indices[0] == 7
    assert segment.frames.frame_count == len(out.swing.frames)
    assert segment.end_contact.point_world_xz_m[1] == pytest.approx(spec.top_z_m, abs=1e-4)
    assert segment.swing_shaping.apex_clearance_m == 0.0
