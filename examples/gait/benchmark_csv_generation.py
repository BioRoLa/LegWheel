#!/usr/bin/env python3
"""Benchmark end-to-end hardware gait CSV generation.

This script measures the full ``generate_hardware_csv`` path used by the
hardware workflow.  It intentionally benchmarks the current production Python
path rather than the experimental C-base branch, so it can be used as a stable
baseline before and after CSV-generation optimizations.
"""

from __future__ import annotations

import argparse
import cProfile
import json
import os
import pstats
import shutil
import sys
import time
from contextlib import redirect_stdout
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from examples.gait.generate_hardware_csv import generate_hardware_csv


def _stats(samples_s: List[float]) -> Dict[str, float]:
    arr_ms = np.asarray(samples_s, dtype=float) * 1000.0
    return {
        "count": int(arr_ms.size),
        "mean_ms": float(np.mean(arr_ms)),
        "p50_ms": float(np.percentile(arr_ms, 50)),
        "p95_ms": float(np.percentile(arr_ms, 95)),
        "min_ms": float(np.min(arr_ms)),
        "max_ms": float(np.max(arr_ms)),
    }


def _count_csv_rows(path: Path) -> int:
    with path.open("r", encoding="utf-8") as handle:
        return sum(1 for _ in handle)


def _run_once(args: argparse.Namespace, out_dir: Path) -> Tuple[float, Path, int, int]:
    out_dir.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    with open(os.devnull, "w", encoding="utf-8") as devnull, redirect_stdout(devnull):
        filepath = generate_hardware_csv(
            twist=[args.wz, args.vx, args.vy],
            gait_type=args.gait,
            stand_height=args.height,
            step_height=args.step,
            period=args.period,
            dt=args.dt,
            n_cycles=args.cycles,
            output_dir=str(out_dir),
            with_launch=args.launch,
            n_ramp=args.ramp_cycles,
            ramp_floor=args.ramp_floor,
            stability_margin=args.stab_margin,
            stance_duty=args.duty,
        )
    t1 = time.perf_counter()

    csv_path = Path(filepath)
    return t1 - t0, csv_path, _count_csv_rows(csv_path), csv_path.stat().st_size


def _profile_once(args: argparse.Namespace, out_dir: Path, profile_path: Path) -> None:
    profile_path.parent.mkdir(parents=True, exist_ok=True)

    def target() -> None:
        _run_once(args, out_dir)

    profiler = cProfile.Profile()
    profiler.enable()
    target()
    profiler.disable()
    profiler.dump_stats(str(profile_path))


def _top_profile_entries(profile_path: Path, limit: int) -> List[Dict[str, object]]:
    stats = pstats.Stats(str(profile_path))
    stats.strip_dirs().sort_stats("cumtime")
    rows = []
    for func, values in list(stats.stats.items()):
        cc, nc, tt, ct, callers = values
        filename, line, name = func
        rows.append(
            {
                "function": f"{filename}:{line}({name})",
                "primitive_calls": int(nc),
                "total_calls": int(cc),
                "tottime_s": float(tt),
                "cumtime_s": float(ct),
            }
        )
    rows.sort(key=lambda item: item["cumtime_s"], reverse=True)
    return rows[:limit]


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark LegWheel hardware CSV generation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--gait", default="Trot", choices=["Walk", "Trot", "Pace", "Bound", "Pronk"]
    )
    parser.add_argument("--vx", type=float, default=0.05)
    parser.add_argument("--vy", type=float, default=0.03)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--height", type=float, default=0.25)
    parser.add_argument("--step", type=float, default=0.04)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--duty", type=float, default=None)
    parser.add_argument("--stab-margin", type=float, default=0.02)
    parser.add_argument("--launch", action="store_true")
    parser.add_argument("--ramp-cycles", type=int, default=3)
    parser.add_argument("--ramp-floor", type=float, default=0.1)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--outdir", default="outputs/bench_csv")
    parser.add_argument("--json-out", default="outputs/bench_csv/benchmark_report.json")
    parser.add_argument("--profile", action="store_true", help="Capture cProfile for one extra run")
    parser.add_argument("--profile-out", default="outputs/bench_csv/generate_csv.prof")
    parser.add_argument("--profile-top", type=int, default=20)
    parser.add_argument(
        "--keep-csv", action="store_true", help="Keep generated CSV files from prior runs"
    )
    return parser


def main(argv: Optional[List[str]] = None) -> None:
    args = _make_parser().parse_args(argv)
    out_dir = (REPO_ROOT / args.outdir).resolve()

    if out_dir.exists() and not args.keep_csv:
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    durations = []
    rows = []
    sizes = []
    last_csv = None
    for _ in range(args.repeats):
        duration_s, csv_path, row_count, size_bytes = _run_once(args, out_dir)
        durations.append(duration_s)
        rows.append(row_count)
        sizes.append(size_bytes)
        last_csv = csv_path

    profile_entries = []
    profile_path = None
    if args.profile:
        profile_path = (REPO_ROOT / args.profile_out).resolve()
        _profile_once(args, out_dir, profile_path)
        profile_entries = _top_profile_entries(profile_path, args.profile_top)

    mean_s = float(np.mean(durations))
    mean_rows = float(np.mean(rows))
    report = {
        "meta": {
            "timestamp_epoch_s": time.time(),
            "params": {
                "gait": args.gait,
                "twist": [args.wz, args.vx, args.vy],
                "height": args.height,
                "step": args.step,
                "period": args.period,
                "cycles": args.cycles,
                "dt": args.dt,
                "duty": args.duty,
                "stab_margin": args.stab_margin,
                "launch": args.launch,
                "ramp_cycles": args.ramp_cycles,
                "ramp_floor": args.ramp_floor,
                "repeats": args.repeats,
            },
            "output_dir": str(out_dir),
            "last_csv": str(last_csv) if last_csv else "",
            "profile_path": str(profile_path) if profile_path else "",
        },
        "timing": _stats(durations),
        "csv_rows": {
            "mean": mean_rows,
            "min": int(np.min(rows)),
            "max": int(np.max(rows)),
        },
        "csv_size_bytes": {
            "mean": float(np.mean(sizes)),
            "min": int(np.min(sizes)),
            "max": int(np.max(sizes)),
        },
        "throughput": {
            "rows_per_second_mean": mean_rows / mean_s if mean_s > 0.0 else 0.0,
            "seconds_per_1000_rows_mean": mean_s * 1000.0 / mean_rows if mean_rows > 0.0 else 0.0,
        },
        "profile_top_cumulative": profile_entries,
    }

    print(json.dumps(report, indent=2, sort_keys=True))

    json_out = (REPO_ROOT / args.json_out).resolve()
    json_out.parent.mkdir(parents=True, exist_ok=True)
    json_out.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Saved JSON report to: {json_out}")


if __name__ == "__main__":
    main()
