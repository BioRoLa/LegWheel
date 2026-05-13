#!/usr/bin/env python3
"""Benchmark hotspot functions to compare Python vs future C-based implementations."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import os
import sys

# Ensure local source tree has priority over an installed package.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D


def _stats(samples_s: np.ndarray) -> dict:
    arr_ms = np.asarray(samples_s, dtype=float) * 1000.0
    return {
        "count": int(arr_ms.size),
        "mean_ms": float(np.mean(arr_ms)),
        "p50_ms": float(np.percentile(arr_ms, 50)),
        "p95_ms": float(np.percentile(arr_ms, 95)),
        "max_ms": float(np.max(arr_ms)),
        "min_ms": float(np.min(arr_ms)),
    }


def benchmark_ik(samples: int, seed: int) -> dict:
    kin = CorgiLegKinematics(0)
    rng = np.random.default_rng(seed)

    durations = []
    pos_errors = []

    for _ in range(samples):
        q_true = np.array(
            [
                np.deg2rad(rng.uniform(30.0, 140.0)),
                np.deg2rad(rng.uniform(-25.0, 25.0)),
                np.deg2rad(rng.uniform(-12.0, 12.0)),
            ],
            dtype=float,
        )
        target = kin.forward_kinematics(*q_true)
        guess = q_true + np.deg2rad(np.array([5.0, -3.0, 2.0]))

        t0 = time.perf_counter()
        q_calc = kin.inverse_kinematics(target, guess_q=guess)
        t1 = time.perf_counter()

        durations.append(t1 - t0)
        p_calc = kin.forward_kinematics(*q_calc)
        pos_errors.append(np.linalg.norm(target - p_calc))

    out = {
        "ik_timing": _stats(np.array(durations)),
        "ik_error_m": {
            "mean": float(np.mean(pos_errors)),
            "p95": float(np.percentile(pos_errors, 95)),
            "max": float(np.max(pos_errors)),
        },
    }
    return out


def benchmark_stance_solver(steps: int) -> dict:
    planner = TrajectoryPlanner3D(
        leg_index=0,
        stand_height=0.31,
        velocity=[0.15, 0.05, 0.0],
        period=1.0,
        dt=0.005,
        stance_duty=0.75,
    )

    gamma_sign = 1.0 if planner.velocity[1] >= 0 else -1.0
    q = np.array([planner.theta0, -planner.beta0, gamma_sign * planner.gamma0], dtype=float)

    durations = []
    foot_delta_errors = []

    total_t0 = time.perf_counter()
    for _ in range(steps):
        t0 = time.perf_counter()
        q_next = planner.stance_rt_solver(v_hip=planner.velocity, q=q)
        t1 = time.perf_counter()
        durations.append(t1 - t0)

        contact_0 = planner.kin.foot_rim_contact_fk(*q)
        fk_0 = planner.kin.forward_kinematics(*q, alpha=contact_0[0], w=contact_0[1])
        contact_1 = planner.kin.foot_rim_contact_fk(*q_next)
        fk_1 = planner.kin.forward_kinematics(*q_next, alpha=contact_1[0], w=contact_1[1])

        foot_delta = fk_1 - fk_0
        expected_delta = -planner.velocity * planner.dt
        foot_delta_errors.append(float(np.linalg.norm(foot_delta - expected_delta)))

        q = q_next
    total_t1 = time.perf_counter()

    total_s = total_t1 - total_t0
    out = {
        "stance_solver_timing": _stats(np.array(durations)),
        "stance_solver_total_s": float(total_s),
        "stance_solver_effective_hz": float(steps / total_s if total_s > 0 else 0.0),
        "stance_foot_delta_error_m": {
            "mean": float(np.mean(foot_delta_errors)),
            "p95": float(np.percentile(foot_delta_errors, 95)),
            "max": float(np.max(foot_delta_errors)),
        },
    }
    return out


def benchmark_generate_trajectory(rounds: int) -> dict:
    planner = TrajectoryPlanner3D(
        leg_index=0,
        stand_height=0.31,
        velocity=[0.15, 0.05, 0.0],
        period=1.0,
        dt=0.005,
        stance_duty=0.75,
    )

    durations = []
    frames = []
    for _ in range(rounds):
        t0 = time.perf_counter()
        cmd = planner.generate_trajectory()
        t1 = time.perf_counter()
        durations.append(t1 - t0)
        frames.append(len(cmd))

    out = {
        "trajectory_timing": _stats(np.array(durations)),
        "trajectory_frames": {
            "mean": float(np.mean(frames)),
            "min": int(np.min(frames)),
            "max": int(np.max(frames)),
        },
    }
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark C-target hotspots in LegWheel")
    parser.add_argument("--ik-samples", type=int, default=100, help="Number of IK cases")
    parser.add_argument("--stance-steps", type=int, default=500, help="Stance RT solver steps")
    parser.add_argument("--traj-rounds", type=int, default=10, help="Number of trajectory generations")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for IK samples")
    parser.add_argument("--json-out", type=str, default="", help="Optional JSON output path")
    args = parser.parse_args()

    report = {
        "meta": {
            "ik_samples": args.ik_samples,
            "stance_steps": args.stance_steps,
            "traj_rounds": args.traj_rounds,
            "seed": args.seed,
            "timestamp_epoch_s": time.time(),
        }
    }
    report.update(benchmark_ik(args.ik_samples, args.seed))
    report.update(benchmark_stance_solver(args.stance_steps))
    report.update(benchmark_generate_trajectory(args.traj_rounds))

    print(json.dumps(report, indent=2, sort_keys=True))

    if args.json_out:
        out_path = Path(args.json_out).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        print(f"Saved JSON report to: {out_path}")


if __name__ == "__main__":
    main()
