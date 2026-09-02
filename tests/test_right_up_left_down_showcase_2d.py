"""Day 6--7 showcase: parameter advice drawn from the measured evidence."""

from __future__ import annotations

import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_showcase_2d import (  # noqa: E402
    EVIDENCE_ARC_SAMPLES,
    MEASURED_TOP_LENGTH_MARGIN_M,
    SWEEP_CSV_NAME,
    TRANSITION_CSV_NAME,
    WHEEL_RADIUS_AT_17_DEG_M,
    Day67Evidence2D,
    advise_showcase_parameters,
    plot_day6_7_evidence_panel_2d,
    run_showcase_traversal_2d,
    showcase_summary_lines,
)

DAY6_7_DIR = "hybrid_note/notes/day6-7"


@pytest.fixture(scope="module")
def evidence() -> Day67Evidence2D:
    return Day67Evidence2D.load(DAY6_7_DIR)


# --------------------------------------------------------------------------
# Loading the measured evidence
# --------------------------------------------------------------------------


def test_missing_evidence_says_which_section_to_run(tmp_path):
    with pytest.raises(FileNotFoundError, match="Step 11R / 12R"):
        Day67Evidence2D.load(tmp_path)


def test_evidence_matches_the_csv_it_came_from(evidence):
    with open(f"{DAY6_7_DIR}/{SWEEP_CSV_NAME}", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(evidence.feasibility_rows) == len(rows)
    assert len(evidence.swept_heights_m) * len(evidence.swept_theta_deg) == len(rows)


def test_angles_survive_the_degree_round_trip(evidence):
    """Angles reach the CSVs via deg->rad->deg, so 60 can arrive as 59.999...

    If that leaked through, table lookups and membership tests would miss and
    the advice would contradict itself.
    """

    for theta in evidence.swept_theta_deg:
        assert theta == pytest.approx(round(theta), abs=1e-9)
    assert set(evidence.required_top_length_by_theta()) <= set(
        evidence.swept_theta_deg
    )
    feasible = evidence.feasible_theta_deg(0.10)
    assert 60.0 in feasible


def test_required_top_length_falls_with_theta(evidence):
    table = evidence.required_top_length_by_theta()
    thetas = sorted(table)
    values = [table[theta] for theta in thetas]
    assert values == sorted(values, reverse=True)


def test_required_top_length_interpolates_but_refuses_to_extrapolate(evidence):
    thetas = sorted(evidence.required_top_length_by_theta())
    low, high = thetas[0], thetas[-1]
    middle = 0.5 * (low + high)
    assert evidence.required_top_length_m(low) is not None
    assert evidence.required_top_length_m(high) is not None

    interpolated = evidence.required_top_length_m(middle)
    assert (
        evidence.required_top_length_m(high)
        < interpolated
        < evidence.required_top_length_m(low)
    )
    assert evidence.required_top_length_m(low - 1.0) is None
    assert evidence.required_top_length_m(high + 1.0) is None


def test_suggested_length_carries_the_measured_margin(evidence):
    theta = 60.0
    assert evidence.suggested_top_length_m(theta) == pytest.approx(
        evidence.required_top_length_m(theta) + MEASURED_TOP_LENGTH_MARGIN_M
    )


# --------------------------------------------------------------------------
# The advice
# --------------------------------------------------------------------------


def test_a_measured_feasible_choice_raises_no_warning(evidence):
    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=evidence.suggested_top_length_m(60.0),
    )
    assert advice.looks_promising
    assert advice.warnings == ()
    assert advice.top_length_is_long_enough
    assert advice.previously_measured_outcome[0] is True


def test_a_short_top_is_flagged_as_blocking_the_handover_not_the_climb(evidence):
    """The warning must not tell the user the obstacle cannot be climbed."""

    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=0.20,
    )
    assert not advice.looks_promising
    assert advice.top_length_is_long_enough is False
    message = " ".join(advice.warnings)
    assert "handover" in message
    assert "roll-up will still succeed" in message


def test_a_top_inside_the_margin_is_flagged_as_tight(evidence):
    required = evidence.required_top_length_m(60.0)
    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=required + 0.5 * MEASURED_TOP_LENGTH_MARGIN_M,
    )
    assert advice.top_length_is_long_enough        # past the requirement...
    assert not advice.looks_promising              # ...but inside the margin
    assert "margin" in " ".join(advice.warnings)


def test_coarse_sampling_is_flagged_as_making_the_advice_unreliable(evidence):
    """Below the sampling the evidence was measured at, the advice stops holding.

    On a coarse rim the APPROACH can stop short of the front face -- a sampling
    artefact, not an infeasible traversal -- so the warning has to say which it
    is rather than implying the parameters are wrong.
    """

    kwargs = dict(
        obstacle_height_m=0.10, theta_climb_deg=85.0,
        obstacle_top_length_m=evidence.suggested_top_length_m(85.0),
    )
    coarse = advise_showcase_parameters(
        evidence, arc_samples=EVIDENCE_ARC_SAMPLES - 60, **kwargs
    )
    assert not coarse.looks_promising
    message = " ".join(coarse.warnings)
    assert "arc_samples" in message
    assert "sampling artefact" in message

    assert advise_showcase_parameters(
        evidence, arc_samples=EVIDENCE_ARC_SAMPLES, **kwargs
    ).looks_promising
    # Omitting it means "not my concern", not "known fine".
    assert advise_showcase_parameters(evidence, **kwargs).looks_promising


def test_an_obstacle_taller_than_the_wheel_is_flagged(evidence):
    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.16, theta_climb_deg=60.0,
        obstacle_top_length_m=0.30,
    )
    assert not advice.looks_promising
    message = " ".join(advice.warnings)
    assert "wheel radius" in message
    assert f"{WHEEL_RADIUS_AT_17_DEG_M:.4f}" in message


def test_an_unswept_height_is_reported_as_untested_not_refused(evidence):
    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.07, theta_climb_deg=60.0,
        obstacle_top_length_m=0.30,
    )
    assert "not swept" in " ".join(advice.warnings)
    assert advice.previously_measured_outcome is None
    # Still a usable answer: the top-length requirement does not need the sweep.
    assert advice.required_top_length_m is not None


def test_infeasible_theta_at_a_height_names_the_alternatives(evidence):
    """h=0.04 at high theta failed in Step 11R; the advice should say so."""

    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.04, theta_climb_deg=85.0,
        obstacle_top_length_m=evidence.suggested_top_length_m(85.0),
    )
    assert not advice.looks_promising
    message = " ".join(advice.warnings)
    assert "did not complete at this height" in message
    for value in evidence.feasible_theta_deg(0.04):
        assert f"{value:.0f}" in message


def test_report_is_plain_text_and_mentions_every_input(evidence):
    advice = advise_showcase_parameters(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=0.25,
    )
    report = advice.report()
    assert "0.100" in report and "60.0" in report and "0.250" in report
    assert isinstance(report, str)


def test_advice_rejects_a_foreign_evidence_object():
    with pytest.raises(TypeError):
        advise_showcase_parameters(
            object(), obstacle_height_m=0.1, theta_climb_deg=60.0,
            obstacle_top_length_m=0.3,
        )


# --------------------------------------------------------------------------
# Running the showcase
# --------------------------------------------------------------------------


def test_default_top_length_uses_the_measured_suggestion(evidence):
    """The demo default should be the shortest obstacle known to work."""

    result = run_showcase_traversal_2d(
        evidence, obstacle_height_m=0.10, theta_climb_deg=85.0,
        obstacle_top_length_m=None, arc_samples=61, verbose=False,
    )
    assert result.traversal.obstacle.width_m == pytest.approx(
        evidence.suggested_top_length_m(85.0)
    )
    assert result.advice is not None


def test_unknown_theta_without_an_explicit_length_is_refused(evidence):
    with pytest.raises(ValueError, match="must be given"):
        run_showcase_traversal_2d(
            evidence, obstacle_height_m=0.10, theta_climb_deg=120.0,
            obstacle_top_length_m=None, arc_samples=61, verbose=False,
        )


def test_showcase_runs_without_evidence(evidence):
    """The evidence is advice, not a dependency of the simulation."""

    result = run_showcase_traversal_2d(
        None, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=0.35, arc_samples=61, verbose=False,
    )
    assert result.advice is None
    assert result.traversal is not None


@pytest.fixture(scope="module")
def showcase(evidence):
    return run_showcase_traversal_2d(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=0.35, arc_samples=61, verbose=False,
    )


def test_showcase_completes_and_measures_its_own_run(showcase):
    assert showcase.success
    assert showcase.traversal.full_success
    assert showcase.measurement.reached_left_rim_ready
    assert showcase.measurement.l_transition_m == pytest.approx(
        showcase.traversal.l_transition_m
    )


def test_summary_lines_describe_the_outcome(showcase):
    lines = showcase_summary_lines(showcase)
    text = "\n".join(lines)
    assert "FULL TRAVERSAL COMPLETED" in text
    assert "L_transition" in text
    assert "minimum collision margin" in text
    assert "corner pivot rotation" in text


def test_summary_of_a_failed_run_names_the_stage(evidence):
    result = run_showcase_traversal_2d(
        evidence, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_top_length_m=0.18, arc_samples=61, verbose=False,
    )
    assert not result.success
    text = "\n".join(showcase_summary_lines(result))
    assert "DID NOT COMPLETE" in text
    assert result.traversal.failure_stage in text
    # ...and the climb itself was fine, which is the point of the whole section.
    assert result.traversal.roll_up_success


def test_evidence_panel_builds(evidence):
    axes = plot_day6_7_evidence_panel_2d(evidence)
    assert len(axes) == 2
    plt.close(axes[0].figure)
