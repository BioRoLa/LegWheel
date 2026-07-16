import numpy as np
import pytest

from legwheel.visualization.plotly_robot import build_figure, parse_gamma_list


def test_parse_gamma_list_converts_four_degree_values_to_radians():
    gamma_values = parse_gamma_list([0.0, 10.0, -10.0, 30.0])

    np.testing.assert_allclose(gamma_values, np.deg2rad([0.0, 10.0, -10.0, 30.0]))


def test_parse_gamma_list_rejects_non_four_value_lists():
    with pytest.raises(ValueError, match="four"):
        parse_gamma_list([0.0, 10.0])


def test_end_plane_rim_traces_match_toroidal_fk_geometry():
    """The rendered tyre outlines must meet rim-point FK at both wheel faces."""
    from legwheel.models.corgi_leg import CorgiLegKinematics
    from legwheel.visualization.plotly_robot import leg_mechanism_traces

    theta = np.deg2rad(75.0)
    beta = 0.0
    gamma = 0.0
    leg = CorgiLegKinematics(0)
    traces = leg_mechanism_traces(0, theta, beta, gamma, include_rim_thickness=False)
    rim_traces = [trace for trace in traces if trace.name == "FL rims"]

    for w in (leg.wheel_thickness / 2, -leg.wheel_thickness / 2):
        rim_point = leg.forward_kinematics(theta, beta, gamma, alpha=0.0, w=w)
        distances = [
            np.min(
                np.linalg.norm(
                    np.column_stack([trace.x, trace.y, trace.z]).astype(float) - rim_point,
                    axis=1,
                )
            )
            for trace in rim_traces
        ]
        assert min(distances) < 0.01


def test_rim_outlines_resolve_toroidal_fk_at_wheel_faces():
    """Dense rim outlines must track the FK profile at each wheel face."""
    from legwheel.models.corgi_leg import CorgiLegKinematics
    from legwheel.visualization.plotly_robot import leg_mechanism_traces

    theta = np.deg2rad(107.8)
    beta = np.deg2rad(13.1)
    gamma = np.deg2rad(-5.355)
    leg = CorgiLegKinematics(0)
    traces = leg_mechanism_traces(0, theta, beta, gamma, include_rim_thickness=True)
    rim_traces = [
        np.column_stack([trace.x, trace.y, trace.z]).astype(float)
        for trace in traces
        if trace.name == "FL rims"
    ]

    for w in (-leg.wheel_thickness / 2, leg.wheel_thickness / 2):
        for alpha in np.linspace(-180.0, 180.0, 25):
            rim_point = leg.forward_kinematics(theta, beta, gamma, alpha=alpha, w=w)
            closest_distance = min(
                np.linalg.norm(points - rim_point, axis=1).min() for points in rim_traces
            )
            assert closest_distance < 0.002


def test_tyre_profile_traces_follow_toroidal_fk_cross_section():
    """Each alpha profile samples the rounded tyre surface across its width."""
    from legwheel.models.corgi_leg import CorgiLegKinematics
    from legwheel.visualization.plotly_robot import leg_mechanism_traces

    theta = np.deg2rad(75.0)
    beta = 0.0
    gamma = 0.0
    leg = CorgiLegKinematics(0)
    traces = leg_mechanism_traces(0, theta, beta, gamma, include_rim_thickness=True)
    profile = next(trace for trace in traces if trace.name == "FL tyre width")

    w_samples = np.linspace(-leg.wheel_thickness / 2, leg.wheel_thickness / 2, 15)
    expected = np.array(
        [leg.forward_kinematics(theta, beta, gamma, alpha=-180.0, w=w) for w in w_samples]
    )
    actual = np.column_stack([profile.x, profile.y, profile.z]).astype(float)

    np.testing.assert_allclose(actual, expected)


def test_build_figure_returns_non_empty_plotly_figure():
    figure = build_figure(
        theta=np.deg2rad(75.0),
        beta=0.0,
        gamma=0.0,
        show_frames=False,
        include_rim_thickness=False,
    )

    assert figure.data
    assert "Body Frame {B}" in figure.layout.title.text
