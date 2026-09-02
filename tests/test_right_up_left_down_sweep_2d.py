"""Day 6--7 Step 11R: the height x theta_climb full-traversal sweep."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    FAILURE_STAGES,
    LEFT_RIM_TRANSITION_FAIL,
    SweepSettings2D,
    TraversalSweepResult2D,
    _hip_x_for_start_clearance,
    plot_feasibility_map_2d,
    plot_feasible_theta_range_2d,
    plot_minimum_feasible_theta_2d,
    rerun_sweep_cell,
    run_height_theta_sweep_2d,
    seam_bridge_for_sampling_m,
    sweep_rows,
    write_sweep_csv,
)
from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    _front_face_clearance_m,
    build_single_leg_rolling_scene_2d,
)

ARC_SAMPLES = 61
# One feasible height and one that the descent cannot survive, so the fixture
# contains both verdicts without paying for a full grid.
HEIGHTS_M = (0.10, 0.14)
THETAS_RAD = (np.deg2rad(50.0), np.deg2rad(65.0))


@pytest.fixture(scope="module")
def settings() -> SweepSettings2D:
    return SweepSettings2D(arc_samples=ARC_SAMPLES, obstacle_width_m=0.35)


@pytest.fixture(scope="module")
def sweep(settings) -> TraversalSweepResult2D:
    return run_height_theta_sweep_2d(
        heights_m=HEIGHTS_M,
        theta_climb_rad=THETAS_RAD,
        settings=settings,
        keep_trajectories=True,
        max_workers=4,
        progress=False,
    )


# --------------------------------------------------------------------------
# The fixed initial condition
# --------------------------------------------------------------------------


def test_seam_bridge_follows_the_sampling_density():
    """A fixed allowance would reject the handover on coarse grids."""

    coarse = seam_bridge_for_sampling_m(61)
    fine = seam_bridge_for_sampling_m(241)
    assert coarse > fine
    assert fine >= 5e-3
    assert coarse == pytest.approx(4.0 * fine, rel=0.02)
    with pytest.raises(ValueError):
        seam_bridge_for_sampling_m(2)


@pytest.mark.parametrize("theta_deg", [45.0, 60.0, 75.0])
def test_start_clearance_is_the_same_for_every_leg_size(settings, theta_deg):
    """The fixed approach condition is a gap, not a hip x.

    The leg radius grows with theta_climb, so a single hip x would give each
    theta a different distance to roll.  Fixing the gap is what makes the
    approach the same work in every cell.
    """

    obstacle = settings.obstacle(0.10)
    theta = np.deg2rad(theta_deg)
    hip_x = _hip_x_for_start_clearance(
        obstacle, theta, settings.approach_beta_rad,
        settings.approach_start_clearance_m,
    )
    template = build_single_leg_rolling_scene_2d(
        theta, settings.approach_beta_rad, 0.0, 0.0, **obstacle.scene_kwargs
    )
    hip_z = -float(template.geometry.points_hip_xz_m[:, 1].min())
    scene = build_single_leg_rolling_scene_2d(
        theta, settings.approach_beta_rad, hip_x, hip_z, **obstacle.scene_kwargs
    )
    assert _front_face_clearance_m(scene) == pytest.approx(
        settings.approach_start_clearance_m, abs=1e-4
    )


def test_bigger_legs_start_closer_to_the_obstacle(settings):
    """Same gap, different hip x -- which is the whole point of fixing the gap."""

    obstacle = settings.obstacle(0.06)
    hip_x = [
        _hip_x_for_start_clearance(
            obstacle, np.deg2rad(theta_deg), settings.approach_beta_rad,
            settings.approach_start_clearance_m,
        )
        for theta_deg in (45.0, 60.0, 75.0)
    ]
    assert hip_x == sorted(hip_x)


def test_start_that_cannot_clear_the_face_is_refused(settings):
    with pytest.raises(ValueError, match="cannot start that far"):
        _hip_x_for_start_clearance(
            settings.obstacle(0.10), np.deg2rad(60.0), 0.0, 5.0
        )


# --------------------------------------------------------------------------
# The sweep itself
# --------------------------------------------------------------------------


def test_sweep_covers_the_whole_grid(sweep):
    assert len(sweep.cells) == len(HEIGHTS_M) * len(THETAS_RAD)
    assert sweep.heights_m == HEIGHTS_M
    assert sweep.theta_climb_rad == THETAS_RAD
    for height in HEIGHTS_M:
        for theta in THETAS_RAD:
            assert sweep.cell(height, theta) is not None


def test_sweep_refuses_degenerate_axes(settings):
    with pytest.raises(ValueError, match="at least one height"):
        run_height_theta_sweep_2d((), THETAS_RAD, settings=settings, progress=False)
    with pytest.raises(ValueError, match="must not repeat"):
        run_height_theta_sweep_2d(
            (0.10, 0.10), THETAS_RAD, settings=settings, progress=False
        )
    with pytest.raises(ValueError, match="heights must be positive"):
        run_height_theta_sweep_2d(
            (-0.10,), THETAS_RAD, settings=settings, progress=False
        )


def test_feasible_means_back_on_the_lower_ground(sweep):
    """A cell is feasible only if the leg finished across, not merely on top."""

    for cell in sweep.cells:
        if cell.feasible:
            assert cell.row["reached_lower_ground_behind_obstacle"]
            assert cell.row["roll_down_success"]
            assert cell.row["final_contact_surface"] == "ground"
        else:
            assert not (
                cell.row["reached_lower_ground_behind_obstacle"]
                and cell.row["roll_down_success"]
            )


def test_every_infeasible_cell_names_one_of_the_required_stages(sweep):
    for cell in sweep.cells:
        label = cell.row["failure_stage"]
        if cell.feasible:
            assert label is None
        else:
            assert label in FAILURE_STAGES
            assert cell.row["failure_reason"] is not None


def test_stage_flags_are_consistent_with_the_failure_stage(sweep):
    """The flags and the label must not disagree about where it broke."""

    for cell in sweep.cells:
        row = cell.row
        if row["failure_stage"] == LEFT_RIM_TRANSITION_FAIL:
            # Reaching theta is what separates this from RETRACT_FAIL.
            assert row["retract_success"]
            assert not row["left_rim_ready_success"]
        if not row["roll_up_success"]:
            assert not row["retract_success"]
            assert not row["left_rim_ready_success"]
            assert not row["roll_down_success"]


def test_required_columns_are_present(sweep):
    required = {
        "obstacle_height_m", "theta_climb_deg", "feasible", "roll_up_success",
        "retract_success", "left_rim_ready_success", "roll_down_success",
        "failure_stage", "failure_reason", "L_transition_m", "final_theta_deg",
        "final_beta_deg", "minimum_collision_margin_m",
    }
    for row in sweep_rows(sweep):
        assert required <= set(row)


def test_feasible_cells_report_a_transition_length_and_margin(sweep):
    feasible = sweep.feasible_cells
    assert feasible, "the fixture grid must contain at least one feasible cell"
    for cell in feasible:
        assert cell.row["L_transition_m"] > 0.0
        assert cell.row["minimum_collision_margin_m"] > 0.0
        assert cell.row["final_theta_deg"] == pytest.approx(17.0)


def test_raw_trajectories_are_retained_for_every_cell(sweep):
    """The verdict alone is not the result; the trajectory behind it is kept."""

    for cell in sweep.cells:
        assert cell.trajectory_rows
        assert len(cell.trajectory_rows) == cell.row["frame_count"]
        first = cell.trajectory_rows[0]
        assert first["obstacle_height_m"] == cell.obstacle_height_m
        assert {"phase", "stage", "theta_deg", "beta_deg"} <= set(first)


def test_rerun_reproduces_a_cell_exactly(sweep):
    """Scenes are not shipped back from the workers, so this is how they return."""

    cell = sweep.feasible_cells[0]
    result = rerun_sweep_cell(sweep, cell.obstacle_height_m, cell.theta_climb_rad)
    assert result.full_success == cell.feasible
    assert len(result.trajectory) == cell.row["frame_count"]
    assert result.l_transition_m == pytest.approx(cell.row["L_transition_m"])
    assert result.minimum_collision_margin_m == pytest.approx(
        cell.row["minimum_collision_margin_m"]
    )


# --------------------------------------------------------------------------
# Derived maps
# --------------------------------------------------------------------------


def test_feasibility_matrix_matches_the_cells(sweep):
    grid = sweep.feasibility_matrix()
    assert grid.shape == (len(HEIGHTS_M), len(THETAS_RAD))
    for cell in sweep.cells:
        row = HEIGHTS_M.index(cell.obstacle_height_m)
        column = THETAS_RAD.index(cell.theta_climb_rad)
        assert bool(grid[row, column]) == cell.feasible


def test_theta_range_brackets_every_feasible_cell(sweep):
    for height in HEIGHTS_M:
        window = sweep.feasible_theta_range_deg(height)
        feasible = [
            cell.theta_climb_deg
            for cell in sweep.cells
            if np.isclose(cell.obstacle_height_m, height) and cell.feasible
        ]
        if not feasible:
            assert window is None
            assert sweep.minimum_feasible_theta_deg(height) is None
            continue
        assert window[0] == pytest.approx(min(feasible))
        assert window[1] == pytest.approx(max(feasible))
        assert sweep.minimum_feasible_theta_deg(height) == pytest.approx(min(feasible))


def test_theta_range_rows_cover_every_height(sweep):
    rows = sweep.feasible_theta_ranges()
    assert [row["obstacle_height_m"] for row in rows] == list(HEIGHTS_M)
    for row in rows:
        if row["feasible_cell_count"] == 0:
            assert row["minimum_feasible_theta_deg"] is None
        else:
            assert row["feasible_theta_span_deg"] >= 0.0


def test_csv_export_round_trips(sweep, tmp_path):
    paths = write_sweep_csv(
        sweep,
        tmp_path / "sweep.csv",
        theta_range_path=tmp_path / "ranges.csv",
        trajectory_path=tmp_path / "trajectories.csv",
    )
    assert len(paths) == 3
    sweep_lines = paths[0].read_text(encoding="utf-8").splitlines()
    assert len(sweep_lines) == len(sweep.cells) + 1
    range_lines = paths[1].read_text(encoding="utf-8").splitlines()
    assert len(range_lines) == len(HEIGHTS_M) + 1
    expected = sum(len(cell.trajectory_rows) for cell in sweep.cells)
    assert len(paths[2].read_text(encoding="utf-8").splitlines()) == expected + 1


@pytest.mark.parametrize(
    "plot", [plot_feasibility_map_2d, plot_feasible_theta_range_2d,
             plot_minimum_feasible_theta_2d]
)
def test_sweep_plots_build(sweep, plot):
    ax = plot(sweep)
    assert ax is not None
    plt.close(ax.figure)
