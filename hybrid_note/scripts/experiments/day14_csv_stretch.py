"""Stretch (or compress) time over one or more intervals of a 1 kHz command CSV.

The trajectory keeps every pose; only the playback time of the chosen
intervals changes, so joint rates there scale by 1/factor.  Use it to slow a
stretch the simulation or the robot found too fast, without replanning::

    python3 day14_csv_stretch.py in_hardware.csv out_hardware.csv 8.0:9.5:1.5 12.0:13.0:2.0

Each interval is ``t_start:t_end:factor`` in seconds of the *input* file
(factor 1.5 = that stretch takes 1.5x longer).  The phase CSV next to the
input (``*_phase.csv``) is stretched the same way when it exists.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def _load(path: Path):
    lines = path.read_text().splitlines()
    header = lines[0] if not lines[0][0].isdigit() and not lines[0][0] == "-" else None
    body = lines[1:] if header is not None else lines
    rows = np.array([[float(v) for v in ln.split(",")] for ln in body if ln.strip()])
    return header, rows


def _stretch(rows: np.ndarray, intervals, dt_s: float = 1e-3) -> np.ndarray:
    n = len(rows)
    t = np.arange(n) * dt_s
    # piecewise-linear map from output time to input time
    knots_in, knots_out = [0.0], [0.0]
    cursor_in, cursor_out = 0.0, 0.0
    for t0, t1, f in sorted(intervals):
        knots_in.append(t0); knots_out.append(cursor_out + (t0 - cursor_in))
        cursor_out = knots_out[-1] + (t1 - t0) * f
        cursor_in = t1
        knots_in.append(t1); knots_out.append(cursor_out)
    knots_in.append(t[-1]); knots_out.append(cursor_out + (t[-1] - cursor_in))
    total_out = knots_out[-1]
    t_out = np.arange(0.0, total_out + 1e-9, dt_s)
    t_in = np.interp(t_out, knots_out, knots_in)
    out = np.column_stack([np.interp(t_in, t, rows[:, k]) for k in range(rows.shape[1])])
    return out


def main(argv):
    src, dst = Path(argv[1]), Path(argv[2])
    intervals = []
    for spec in argv[3:]:
        a, b, f = spec.split(":")
        intervals.append((float(a), float(b), float(f)))
    header, rows = _load(src)
    out = _stretch(rows, intervals)
    fmt = ",".join(["%.6f"] * rows.shape[1])
    with open(dst, "w") as h:
        if header is not None:
            h.write(header + "\n")
        np.savetxt(h, out, fmt=fmt, delimiter=",")
    print(f"{src.name}: {len(rows)} rows -> {dst.name}: {len(out)} rows")
    phase_src = src.with_name(src.stem + "_phase.csv")
    if phase_src.exists():
        ph_header, ph = _load(phase_src)
        ph_out = np.rint(_stretch(ph, intervals)).astype(int)
        phase_dst = dst.with_name(dst.stem + "_phase.csv")
        with open(phase_dst, "w") as h:
            if ph_header is not None:
                h.write(ph_header + "\n")
            np.savetxt(h, ph_out, fmt="%d", delimiter=",")
        print(f"{phase_src.name} -> {phase_dst.name}: {len(ph_out)} rows")


if __name__ == "__main__":
    main(sys.argv)
