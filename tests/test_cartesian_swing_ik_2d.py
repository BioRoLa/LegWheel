"""Day 8--9 Step 4 tests: generalized rim-point FK / IK."""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from legwheel.planners.hybrid import HipPose2D, RimId
from legwheel.planners.hybrid.geometry_2d import legacy_rim_sample_alpha_rad
from legwheel.visualization.plot_leg import PlotLeg

from hybrid_note.scripts.kinematics.ground_contact_single_pose import (
    sample_contact_geometry_points,
)
from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import rim_alpha_limits_rad
from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
    THETA_MAX_RAD,
    THETA_MIN_RAD,
    rim_arc_2d,
    rim_contact_point_hip_xz_m,
    rim_contact_point_world_xz_m,
    rim_point_model_gap_2d,
    solve_contact_ik_2d,
)

HIP = HipPose2D([0.13, 0.27])
RIM_BY_LEGACY_SURFACE = {
    "foot_rim": RimId.FOOT,
    "upper_tyre_l": RimId.LEFT,
    "upper_tyre_r": RimId.RIGHT,
}
SHAPE_ATTRIBUTE_BY_RIM = {
    RimId.FOOT: "foot_rim",
    RimId.LEFT: "upper_rim_l_f",
    RimId.RIGHT: "upper_rim_r_f",
}
POSES_DEG = [(17.0, 0.0), (60.0, -37.0), (60.0, 0.0), (110.0, 95.0), (160.0, -5.0)]


def _alpha_at(rim: RimId, fraction: float) -> float:
    minimum_rad, maximum_rad = rim_alpha_limits_rad(rim)
    return minimum_rad + fraction * (maximum_rad - minimum_rad)


@pytest.mark.parametrize("theta_deg,beta_deg", POSES_DEG)
def test_fk_reproduces_the_existing_contact_sampler_exactly(theta_deg, beta_deg):
    """The Step 4 completion criterion: one alpha convention, not two."""

    theta_rad, beta_rad = np.deg2rad(theta_deg), np.deg2rad(beta_deg)
    records = [
        record
        for record in sample_contact_geometry_points(theta_rad, beta_rad, arc_samples=121)
        if record["geometry_type"] == "rim_arc"
    ]
    counts = {}
    for record in records:
        surface = record["surface_name"]
        counts[surface] = max(counts.get(surface, 0), record["arc_sample_index"] + 1)

    worst_m = 0.0
    for record in records[::5]:
        surface = record["surface_name"]
        alpha_rad = legacy_rim_sample_alpha_rad(
            surface, record["arc_sample_index"], counts[surface]
        )
        point = rim_contact_point_hip_xz_m(
            theta_rad, beta_rad, RIM_BY_LEGACY_SURFACE[surface], alpha_rad
        )
        worst_m = max(
            worst_m,
            float(np.linalg.norm(point - np.array([record["x_m"], record["y_m"]]))),
        )
    assert worst_m < 1e-12


@pytest.mark.parametrize("theta_deg", [17.0, 60.0, 120.0, 160.0])
def test_joint_derived_arcs_match_the_drawn_arcs(theta_deg):
    """The fast path must equal the shape module it replaces."""

    theta_rad = np.deg2rad(theta_deg)
    leg = PlotLeg()
    leg.forward(theta_rad, 0.0, vector=False)
    leg.leg_shape.get_shape(np.array([0.0, 0.0]))

    for rim, attribute in SHAPE_ATTRIBUTE_BY_RIM.items():
        drawn = getattr(leg.leg_shape, attribute).arc[1]
        arc = rim_arc_2d(theta_rad, rim)
        assert arc.center_hip_xz_m == pytest.approx(np.asarray(drawn.center), abs=1e-12)
        assert arc.radius_m == pytest.approx(drawn.width / 2.0, abs=1e-12)
        assert arc.angle_at_alpha_min_rad == pytest.approx(np.deg2rad(drawn.theta1), abs=1e-12)


def test_drawn_arc_radii_do_not_depend_on_theta():
    """Measuring the radii once is only valid if the drawn ones are constant."""

    measured = {rim: [] for rim in SHAPE_ATTRIBUTE_BY_RIM}
    for theta_deg in (17.0, 60.0, 110.0, 160.0):
        leg = PlotLeg()
        leg.forward(np.deg2rad(theta_deg), 0.0, vector=False)
        leg.leg_shape.get_shape(np.array([0.0, 0.0]))
        for rim, attribute in SHAPE_ATTRIBUTE_BY_RIM.items():
            measured[rim].append(getattr(leg.leg_shape, attribute).arc[1].width / 2.0)

    for rim, values in measured.items():
        assert values == pytest.approx([values[0]] * len(values), abs=1e-12), rim
        assert rim_arc_2d(np.deg2rad(43.0), rim).radius_m == pytest.approx(values[0], abs=1e-12)


def test_beta_is_a_pure_rotation_about_the_hip():
    """The IK's analytic beta column depends on this; pin it down."""

    theta_rad, beta_rad = np.deg2rad(60.0), np.deg2rad(-37.0)
    cosine, sine = np.cos(beta_rad), np.sin(beta_rad)
    rotation = np.array([[cosine, -sine], [sine, cosine]])

    for rim, fraction in itertools.product(RimId, (0.1, 0.5, 0.9)):
        alpha_rad = _alpha_at(rim, fraction)
        rotated = rotation @ rim_contact_point_hip_xz_m(theta_rad, 0.0, rim, alpha_rad)
        direct = rim_contact_point_hip_xz_m(theta_rad, beta_rad, rim, alpha_rad)
        assert direct == pytest.approx(rotated, abs=1e-12)


def test_upper_rims_carry_the_drawing_clearance_and_the_foot_rim_does_not():
    """Two rim models exist here; keep the 1.2 mm difference visible."""

    theta_rad, beta_rad = np.deg2rad(60.0), np.deg2rad(-20.0)

    foot_gap_m = rim_point_model_gap_2d(theta_rad, beta_rad, RimId.FOOT, _alpha_at(RimId.FOOT, 0.5))
    assert foot_gap_m == pytest.approx(0.0, abs=1e-9)

    for rim in (RimId.LEFT, RimId.RIGHT):
        for fraction in (0.1, 0.5, 0.9):
            gap_m = rim_point_model_gap_2d(theta_rad, beta_rad, rim, _alpha_at(rim, fraction))
            assert gap_m == pytest.approx(0.0012, abs=1e-6)


@pytest.mark.parametrize("theta_deg,beta_deg", POSES_DEG)
def test_ik_reconstructs_a_configuration_that_fk_produced(theta_deg, beta_deg):
    """The plan's Step 4 completion criterion, over all three rims."""

    theta_rad, beta_rad = np.deg2rad(theta_deg), np.deg2rad(beta_deg)
    for rim, fraction in itertools.product(RimId, (0.1, 0.5, 0.9)):
        alpha_rad = _alpha_at(rim, fraction)
        target = rim_contact_point_world_xz_m(theta_rad, beta_rad, rim, alpha_rad, HIP)
        guess = np.array([theta_rad + np.deg2rad(3.0), beta_rad - np.deg2rad(3.0)])

        solution = solve_contact_ik_2d(target, rim, alpha_rad, HIP, guess)

        assert solution.converged, (rim, fraction, solution.residual_m)
        assert solution.residual_m < 1e-4
        assert solution.usable
        assert solution.theta_rad == pytest.approx(theta_rad, abs=np.deg2rad(0.5))
        wrapped_beta_error = (solution.beta_rad - beta_rad + np.pi) % (2 * np.pi) - np.pi
        assert abs(wrapped_beta_error) < np.deg2rad(0.5)


def test_warm_started_sequence_stays_on_one_branch():
    """How Step 5 will call this: previous solution as the next guess."""

    theta_rad = np.deg2rad(60.0)
    betas_rad = np.deg2rad(np.linspace(-40.0, 20.0, 40))
    alpha_rad = _alpha_at(RimId.RIGHT, 0.4)
    targets = [
        rim_contact_point_world_xz_m(theta_rad, beta_rad, RimId.RIGHT, alpha_rad, HIP)
        for beta_rad in betas_rad
    ]

    guess = np.array([theta_rad, betas_rad[0]])
    solved_betas = []
    for target in targets:
        solution = solve_contact_ik_2d(target, RimId.RIGHT, alpha_rad, HIP, guess)
        assert solution.converged
        guess = solution.joint_angles_rad
        solved_betas.append(solution.beta_rad)

    assert np.allclose(solved_betas, betas_rad, atol=np.deg2rad(0.5))
    assert np.all(np.abs(np.diff(solved_betas)) < np.deg2rad(5.0))


def test_an_unreachable_target_is_reported_not_clamped():
    alpha_rad = _alpha_at(RimId.FOOT, 0.5)
    far_away = HIP.position_world_xz_m + np.array([1.5, 0.0])

    solution = solve_contact_ik_2d(
        far_away, RimId.FOOT, alpha_rad, HIP, np.array([np.deg2rad(60.0), 0.0])
    )

    assert solution.converged is False
    assert solution.usable is False
    assert solution.residual_m > 1.0
    # The reported joints and the reported residual describe the same pose.
    achieved = rim_contact_point_world_xz_m(
        solution.theta_rad, solution.beta_rad, RimId.FOOT, alpha_rad, HIP
    )
    assert float(np.linalg.norm(far_away - achieved)) == pytest.approx(
        solution.residual_m, abs=1e-12
    )


def test_a_target_on_the_theta_limit_is_flagged():
    alpha_rad = _alpha_at(RimId.FOOT, 0.5)
    target = rim_contact_point_world_xz_m(THETA_MIN_RAD, 0.0, RimId.FOOT, alpha_rad, HIP)

    solution = solve_contact_ik_2d(
        target, RimId.FOOT, alpha_rad, HIP, np.array([THETA_MIN_RAD + 0.2, 0.1])
    )

    assert solution.converged
    assert solution.theta_at_limit
    assert solution.joint_limits_ok
    assert THETA_MIN_RAD <= solution.theta_rad <= THETA_MAX_RAD


def test_alpha_must_belong_to_the_rim_it_is_paired_with():
    with pytest.raises(ValueError, match="outside the"):
        solve_contact_ik_2d(
            [0.1, 0.0], RimId.FOOT, np.deg2rad(90.0), HIP, np.array([np.deg2rad(60.0), 0.0])
        )


def test_world_fk_is_the_hip_fk_shifted_by_the_hip_position():
    theta_rad, beta_rad = np.deg2rad(60.0), np.deg2rad(-20.0)
    alpha_rad = _alpha_at(RimId.LEFT, 0.3)

    hip_point = rim_contact_point_hip_xz_m(theta_rad, beta_rad, RimId.LEFT, alpha_rad)
    world_point = rim_contact_point_world_xz_m(theta_rad, beta_rad, RimId.LEFT, alpha_rad, HIP)

    assert world_point == pytest.approx(hip_point + HIP.position_world_xz_m, abs=1e-15)


def test_a_pitched_hip_is_refused_in_the_first_version():
    with pytest.raises(ValueError, match="zero hip pitch"):
        rim_contact_point_world_xz_m(
            np.deg2rad(60.0), 0.0, RimId.FOOT, 0.0,
            HipPose2D([0.0, 0.3], pitch_world_hip_rad=0.05),
        )
