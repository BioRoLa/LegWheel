import numpy as np
import pytest

from legwheel.visualization.plotly_robot import build_figure, parse_gamma_list


def test_parse_gamma_list_converts_four_degree_values_to_radians():
    gamma_values = parse_gamma_list([0.0, 10.0, -10.0, 30.0])

    np.testing.assert_allclose(gamma_values, np.deg2rad([0.0, 10.0, -10.0, 30.0]))


def test_parse_gamma_list_rejects_non_four_value_lists():
    with pytest.raises(ValueError, match="four"):
        parse_gamma_list([0.0, 10.0])


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
