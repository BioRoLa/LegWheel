#!/usr/bin/env python3
"""Benchmark end-to-end hardware gait CSV generation: Python vs C-base."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import time
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from examples.gait.generate_hardware_csv import generate_hardware_csv
from legwheel.cbase.dls import is_c_backend_available
from legwheel.cbase.kernels import is_kernel_backend_available


def _stats(samples_s: np.ndarray) -> dict:
    arr_ms = np.asarray(samples_s, dtype=float) * 1000.0
    return {
        "count": arr_ms.size,
        "mean_ms": float(np.mean(arr_ms)),
        "p50_ms": float(np.percentile(arr_ms, 50)),
        "p95_ms": float(np.percentile(arr_ms, 95)),
        "max_ms": float(np.max(arr_ms)),
        "min_ms": float(np.min(arr_ms)),
    }


def _safe_ratio(a: float, b: float) -> float:
    return a / b if b != 0.0 else 0.0


def _build_c_libs() -> dict:
    builds = {}
    jobs = [
        (
            "dls",
            REPO_ROOT / "legwheel" / "cbase" / "dls_solver.c",
            REPO_ROOT / "legwheel" / "cbase" / "libdls_solver.so",
        ),
        (
            "kernels",
            REPO_ROOT / "legwheel" / "cbase" / "kernels.c",
            REPO_ROOT / "legwheel" / "cbase" / "libcbase_kernels.so",
        ),
    ]

    for key, src, out in jobs:
        if not src.exists():
            builds[key] = {"ok": False, "message": f"missing source: {src}"}
            continue

        cmd = ["gcc", "-O3", "-shared", "-fPIC", str(src), "-o", str(out)]
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            builds[key] = {"ok": True, "message": f"built: {out}"}
        except FileNotFoundError:
            builds[key] = {"ok": False, "message": "gcc not found"}
        except subprocess.CalledProcessError as exc:
            builds[key] = {
                "ok": False,
                "message": (exc.stderr or exc.stdout or "gcc failed").strip(),
            }

    return builds


def _run_once(use_cbase: bool, out_dir: Path, args) -> tuple[float, int]:
    os.environ["LEGWHEEL_USE_CBASE"] = "1" if use_cbase else "0"
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
        )
    t1 = time.perf_counter()

    fpath = Path(filepath)
    size = fpath.stat().st_size if fpath.exists() else 0
    return t1 - t0, size


def _run_mode(use_cbase: bool, repeats: int, base_out_dir: Path, args) -> dict:
    mode = "cbase" if use_cbase else "python"
    out_dir = base_out_dir / mode
    if out_dir.exists():
        shutil.rmtree(out_dir)

    durations = []
    sizes = []
    for _ in range(repeats):
        dt_s, size = _run_once(use_cbase, out_dir, args)
        durations.append(dt_s)
        sizes.append(size)

    return {
        "timing": _stats(np.asarray(durations, dtype=float)),
        "csv_size_bytes": {
            "mean": float(np.mean(sizes)),
            "min": int(np.min(sizes)),
            "max": int(np.max(sizes)),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark one gait CSV generation")
    parser.add_argument("--gait", type=str, default="Walk")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.1)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--height", type=float, default=0.25)
    parser.add_argument("--step", type=float, default=0.04)
    parser.add_argument("--period", type=float, default=4.0)
    parser.add_argument("--cycles", type=int, default=6)
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--outdir", type=str, default="output/csv/bench_gait_csv")
    parser.add_argument("--json-out", type=str, default="")
    args = parser.parse_args()

    builds = {"dls": {"ok": True, "message": "skipped"}, "kernels": {"ok": True, "message": "skipped"}}
    if not args.skip_build:
        builds = _build_c_libs()

    base_out_dir = (REPO_ROOT / args.outdir).resolve()
    python_report = _run_mode(False, args.repeats, base_out_dir, args)
    cbase_report = _run_mode(True, args.repeats, base_out_dir, args)

    py_mean = python_report["timing"]["mean_ms"]
    c_mean = cbase_report["timing"]["mean_ms"]

    report = {
        "meta": {
            "timestamp_epoch_s": time.time(),
            "build": builds,
            "dls_backend_available": is_c_backend_available(),
            "kernels_backend_available": is_kernel_backend_available(),
            "params": {
                "gait": args.gait,
                "twist": [args.wz, args.vx, args.vy],
                "stand_height": args.height,
                "step_height": args.step,
                "period": args.period,
                "cycles": args.cycles,
                "dt": args.dt,
                "repeats": args.repeats,
            },
            "output_dir": str(base_out_dir),
        },
        "python": python_report,
        "cbase": cbase_report,
        "comparison": {
            "speedup": _safe_ratio(py_mean, c_mean),
            "delta_ms": c_mean - py_mean,
        },
    }

    print(json.dumps(report, indent=2, sort_keys=True))

    if args.json_out:
        out = Path(args.json_out).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        print(f"Saved JSON report to: {out}")


if __name__ == "__main__":
    main()
