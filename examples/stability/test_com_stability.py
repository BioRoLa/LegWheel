"""
examples/test_com_stability.py
===============================

Demonstrates the COMStabilityPlanner on three representative scenarios where
the standard GaitGenerator3D produces unstable COM trajectories:

  Scenario A – Pace gait, lateral-only motion
      Same-side leg pairs in stance → support line on one side of the robot.
      The COM (body centre) is ≈ W/2 away from the line → maximum instability.

  Scenario B – Trot gait, lateral motion
      Diagonal leg pairs in stance → narrow diagonal support strip.
      Lateral velocity shifts the feet, pushing the COM outside the strip.

  Scenario C – Walk gait, combined forward + lateral motion
      Three feet in stance most of the time, but the large v_y can still
      pull the triangular support polygon away from the COM.

For each scenario the script:
  1. Creates a GaitGenerator3D (raw planner, no stability awareness).
  2. Wraps it in a COMStabilityPlanner.
  3. Generates raw and corrected gait commands.
  4. Prints a before / after summary.
  5. Plots the comprehensive stability analysis figure.
"""

import sys
import os

import numpy as np
import matplotlib
matplotlib.use("TkAgg")          # change to "Agg" for headless / CI
import matplotlib.pyplot as plt

# Make sure the package is importable from the repo root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.com_stability import COMStabilityPlanner


# ─────────────────────────────────────────────────────────────────────────────
#  Shared parameters
# ─────────────────────────────────────────────────────────────────────────────

STAND_HEIGHT = 0.30   # m
STEP_HEIGHT  = 0.04   # m
PERIOD       = 1.2    # s  (longer period → more time for sway to take effect)
N_CYCLES     = 2

SAFETY_MARGIN = 0.02  # m  desired minimum stability margin
SMOOTH_SIGMA  = 0.06  # fraction of gait period used as Gaussian σ


# ─────────────────────────────────────────────────────────────────────────────
#  Scenario definitions
# ─────────────────────────────────────────────────────────────────────────────

SCENARIOS = [
    {
        "label": "A – Pace, lateral-only",
        "gait":  "Pace",
        "twist": [0.0, 0.00, 0.10],    # [ωz, vx, vy]  →  pure lateral
    },
    {
        "label": "B – Trot, lateral motion",
        "gait":  "Trot",
        "twist": [0.0, 0.10, 0.15],    # forward + strong lateral
    },
    {
        "label": "C – Walk, forward + lateral",
        "gait":  "Walk",
        "twist": [0.0, 0.12, 0.10],
    },
]


# ─────────────────────────────────────────────────────────────────────────────
#  Run all scenarios
# ─────────────────────────────────────────────────────────────────────────────

def run_scenario(scenario: dict) -> None:
    label = scenario["label"]
    print()
    print("=" * 60)
    print(f"  SCENARIO  {label}")
    print("=" * 60)

    # 1. Build the raw gait generator
    gen = GaitGenerator3D(
        stand_height=STAND_HEIGHT,
        twist=scenario["twist"],
        step_height=STEP_HEIGHT,
        period=PERIOD,
        gait_type=scenario["gait"],
    )
    gen.print_summary()

    # 2. Wrap with the COM stability planner
    planner = COMStabilityPlanner(
        gait_generator=gen,
        safety_margin=SAFETY_MARGIN,
        smooth_sigma=SMOOTH_SIGMA,
        max_sway=0.14,        # clip sway at 14 cm for safety
    )

    # 3. Generate corrected commands
    cmds_stable = planner.generate_stable_gait(n_cycles=N_CYCLES)

    # 4. Print summary
    planner.print_summary()

    # 5. Plot
    fig = planner.plot_stability_analysis(show_correction=True)
    fig.canvas.manager.set_window_title(label)


def main():
    for scenario in SCENARIOS:
        run_scenario(scenario)

    print("\nAll scenarios complete.  Close the figure windows to exit.")
    plt.show()


if __name__ == "__main__":
    main()
