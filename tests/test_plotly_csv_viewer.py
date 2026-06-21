import numpy as np
import pytest

from legwheel.visualization.plotly_csv_viewer import hw_to_kin, select_frame_indices


def test_hw_to_kin_converts_hardware_row_to_leg_order():
    row = np.arange(12, dtype=float)

    q_all = hw_to_kin(row)

    np.testing.assert_array_equal(
        q_all,
        np.array(
            [
                [0.0, 1.0, 8.0],
                [2.0, 3.0, 9.0],
                [4.0, 5.0, 10.0],
                [6.0, 7.0, 11.0],
            ]
        ),
    )


def test_hw_to_kin_rejects_non_12_column_rows():
    with pytest.raises(ValueError, match="12 columns"):
        hw_to_kin(np.arange(11, dtype=float))


def test_select_frame_indices_applies_stride_and_max_frame_limit():
    indices = select_frame_indices(n_rows=100, frame_step=5, max_frames=4)

    np.testing.assert_array_equal(indices, np.array([0, 30, 60, 95]))


def test_select_frame_indices_rejects_invalid_stride():
    with pytest.raises(ValueError, match=">= 1"):
        select_frame_indices(n_rows=10, frame_step=0, max_frames=10)
