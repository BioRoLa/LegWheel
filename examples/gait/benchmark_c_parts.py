#!/usr/bin/env python3
"""Function-level benchmarks for C-base candidate kernels."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from legwheel.bezier.bezier import Bezier
from legwheel.cbase.dls import dls_solve_3x3_c, is_c_backend_available
from legwheel.cbase.kernels import is_kernel_backend_available
from legwheel.utils.screw import Screw
from legwheel.utils.utils import pseudo_inverse_dls


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


def _batch_time(fn, repeat: int = 5) -> np.ndarray:
    samples = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn()
        t1 = time.perf_counter()
        samples.append(t1 - t0)
    return np.asarray(samples, dtype=float)


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


def bench_dls(samples: int, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    Js = rng.normal(size=(samples, 3, 3))
    Js += np.eye(3)[None, :, :] * 0.2
    vs = rng.normal(size=(samples, 3))
    damping = 0.05

    py_out = np.zeros((samples, 3), dtype=float)
    c_out = np.zeros((samples, 3), dtype=float)

    def run_py():
        for i in range(samples):
            py_out[i] = pseudo_inverse_dls(Js[i], damping_factor=damping) @ vs[i]

    def run_c():
        for i in range(samples):
            c_out[i] = dls_solve_3x3_c(Js[i], vs[i], damping)

    py_t = _batch_time(run_py)
    c_t = _batch_time(run_c)
    out_err = np.linalg.norm(py_out - c_out, axis=1)

    return {
        "python": _stats(py_t),
        "cbase": _stats(c_t),
        "per_call_mean_us": {
            "python": float(np.mean(py_t) * 1e6 / samples),
            "cbase": float(np.mean(c_t) * 1e6 / samples),
        },
        "speedup": _safe_ratio(np.mean(py_t), np.mean(c_t)),
        "max_output_diff": float(np.max(out_err)),
        "mean_output_diff": float(np.mean(out_err)),
    }


def bench_bezier(samples: int, seed: int) -> dict:
    rng = np.random.default_rng(seed + 1)
    cps = [rng.normal(size=3) for _ in range(12)]
    bz = Bezier(cps)
    ts = rng.random(size=samples)

    py_out = np.zeros((samples, 3), dtype=float)
    c_out = np.zeros((samples, 3), dtype=float)

    def run_py():
        os.environ["LEGWHEEL_USE_CBASE"] = "0"
        for i, t in enumerate(ts):
            py_out[i] = bz.getBzPoint(float(t), 0.1, -0.2, 0.3)

    def run_c():
        os.environ["LEGWHEEL_USE_CBASE"] = "1"
        for i, t in enumerate(ts):
            c_out[i] = bz.getBzPoint(float(t), 0.1, -0.2, 0.3)

    py_t = _batch_time(run_py)
    c_t = _batch_time(run_c)
    out_err = np.linalg.norm(py_out - c_out, axis=1)

    return {
        "python": _stats(py_t),
        "cbase": _stats(c_t),
        "per_call_mean_us": {
            "python": float(np.mean(py_t) * 1e6 / samples),
            "cbase": float(np.mean(c_t) * 1e6 / samples),
        },
        "speedup": _safe_ratio(np.mean(py_t), np.mean(c_t)),
        "max_output_diff": float(np.max(out_err)),
        "mean_output_diff": float(np.mean(out_err)),
    }


def bench_screw_exp6(samples: int, seed: int) -> dict:
    rng = np.random.default_rng(seed + 2)

    py_out = np.zeros((samples, 4, 4), dtype=float)
    c_out = np.zeros((samples, 4, 4), dtype=float)

    screws = []
    thetas = []
    for _ in range(samples):
        w = rng.normal(size=3)
        if np.linalg.norm(w) < 1e-6:
            w[0] += 1.0
        w = w / np.linalg.norm(w)
        v = rng.normal(size=3)
        s = Screw(np.hstack([w, v]))
        screws.append(s)
        thetas.append(rng.uniform(-1.0, 1.0))

    def run_py():
        os.environ["LEGWHEEL_USE_CBASE"] = "0"
        for i, (s, th) in enumerate(zip(screws, thetas)):
            py_out[i] = s.exp6(th)

    def run_c():
        os.environ["LEGWHEEL_USE_CBASE"] = "1"
        for i, (s, th) in enumerate(zip(screws, thetas)):
            c_out[i] = s.exp6(th)

    py_t = _batch_time(run_py)
    c_t = _batch_time(run_c)
    out_err = np.linalg.norm((py_out - c_out).reshape(samples, -1), axis=1)

    return {
        "python": _stats(py_t),
        "cbase": _stats(c_t),
        "per_call_mean_us": {
            "python": float(np.mean(py_t) * 1e6 / samples),
            "cbase": float(np.mean(c_t) * 1e6 / samples),
        },
        "speedup": _safe_ratio(np.mean(py_t), np.mean(c_t)),
        "max_output_diff": float(np.max(out_err)),
        "mean_output_diff": float(np.mean(out_err)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Function-level C-base benchmark")
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--json-out", type=str, default="")
    args = parser.parse_args()

    builds = {"dls": {"ok": True, "message": "skipped"}, "kernels": {"ok": True, "message": "skipped"}}
    if not args.skip_build:
        builds = _build_c_libs()

    report = {
        "meta": {
            "samples": args.samples,
            "seed": args.seed,
            "timestamp_epoch_s": time.time(),
            "build": builds,
            "dls_backend_available": is_c_backend_available(),
            "kernels_backend_available": is_kernel_backend_available(),
        },
        "parts": {},
    }

    if report["meta"]["dls_backend_available"]:
        report["parts"]["dls_solve_3x3"] = bench_dls(args.samples, args.seed)
    else:
        report["parts"]["dls_solve_3x3"] = {"error": "dls backend unavailable"}

    if report["meta"]["kernels_backend_available"]:
        report["parts"]["bezier_getBzPoint_3d_12"] = bench_bezier(args.samples, args.seed)
        report["parts"]["screw_exp6"] = bench_screw_exp6(args.samples, args.seed)
    else:
        report["parts"]["bezier_getBzPoint_3d_12"] = {"error": "kernels backend unavailable"}
        report["parts"]["screw_exp6"] = {"error": "kernels backend unavailable"}

    print(json.dumps(report, indent=2, sort_keys=True))

    if args.json_out:
        out_path = Path(args.json_out).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        print(f"Saved JSON report to: {out_path}")


if __name__ == "__main__":
    main()
