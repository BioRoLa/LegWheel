#!/usr/bin/env python3
"""Compare Python baseline vs optional C-base kernels on existing hotspot benchmarks."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from examples.gait.benchmark_c_hotspots import (
    benchmark_generate_trajectory,
    benchmark_ik,
    benchmark_stance_solver,
)
from legwheel.cbase.dls import is_c_backend_available


def _safe_ratio(a: float, b: float) -> float:
    if b == 0.0:
        return 0.0
    return a / b


def _build_c_backend() -> tuple[bool, str]:
    src = REPO_ROOT / "legwheel" / "cbase" / "dls_solver.c"
    out = REPO_ROOT / "legwheel" / "cbase" / "libdls_solver.so"

    if not src.exists():
        return False, f"source missing: {src}"

    cmd = [
        "gcc",
        "-O3",
        "-shared",
        "-fPIC",
        str(src),
        "-o",
        str(out),
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError:
        return False, "gcc not found"
    except subprocess.CalledProcessError as exc:
        return False, (exc.stderr or exc.stdout or "gcc failed").strip()

    return True, f"built: {out}"


def _run_suite(ik_samples: int, stance_steps: int, traj_rounds: int, seed: int) -> dict:
    report = {
        "meta": {
            "ik_samples": ik_samples,
            "stance_steps": stance_steps,
            "traj_rounds": traj_rounds,
            "seed": seed,
            "timestamp_epoch_s": time.time(),
            "use_cbase": os.getenv("LEGWHEEL_USE_CBASE", "0") == "1",
        }
    }
    report.update(benchmark_ik(ik_samples, seed))
    report.update(benchmark_stance_solver(stance_steps))
    report.update(benchmark_generate_trajectory(traj_rounds))
    return report


def _compare(py_report: dict, c_report: dict) -> dict:
    py_ik = py_report["ik_timing"]["mean_ms"]
    c_ik = c_report["ik_timing"]["mean_ms"]

    py_st = py_report["stance_solver_timing"]["mean_ms"]
    c_st = c_report["stance_solver_timing"]["mean_ms"]

    py_tr = py_report["trajectory_timing"]["mean_ms"]
    c_tr = c_report["trajectory_timing"]["mean_ms"]

    return {
        "speedup": {
            "ik_mean_ms": _safe_ratio(py_ik, c_ik),
            "stance_solver_mean_ms": _safe_ratio(py_st, c_st),
            "trajectory_mean_ms": _safe_ratio(py_tr, c_tr),
            "stance_effective_hz": _safe_ratio(
                c_report["stance_solver_effective_hz"],
                py_report["stance_solver_effective_hz"],
            ),
        },
        "delta": {
            "ik_mean_ms": c_ik - py_ik,
            "stance_solver_mean_ms": c_st - py_st,
            "trajectory_mean_ms": c_tr - py_tr,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Python vs C-base benchmark comparison")
    parser.add_argument("--ik-samples", type=int, default=200)
    parser.add_argument("--stance-steps", type=int, default=1000)
    parser.add_argument("--traj-rounds", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--skip-build", action="store_true", help="Skip gcc build step")
    parser.add_argument("--json-out", type=str, default="")
    args = parser.parse_args()

    build_status = {"attempted": False, "ok": False, "message": "skipped"}
    if not args.skip_build:
        build_status["attempted"] = True
        ok, msg = _build_c_backend()
        build_status["ok"] = ok
        build_status["message"] = msg

    os.environ["LEGWHEEL_USE_CBASE"] = "0"
    py_report = _run_suite(args.ik_samples, args.stance_steps, args.traj_rounds, args.seed)

    os.environ["LEGWHEEL_USE_CBASE"] = "1"
    c_report = _run_suite(args.ik_samples, args.stance_steps, args.traj_rounds, args.seed)

    final = {
        "meta": {
            "timestamp_epoch_s": time.time(),
            "c_backend_available": bool(is_c_backend_available()),
            "build": build_status,
        },
        "python": py_report,
        "cbase": c_report,
        "comparison": _compare(py_report, c_report),
    }

    print(json.dumps(final, indent=2, sort_keys=True))

    if args.json_out:
        out_path = Path(args.json_out).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(final, indent=2, sort_keys=True), encoding="utf-8")
        print(f"Saved JSON report to: {out_path}")


if __name__ == "__main__":
    main()
