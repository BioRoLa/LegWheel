"""Raise the rear axle over a time window of a hardware CSV, in place.

    python3 day14_csv_rear_lift.py IN_hardware.csv OUT_hardware.csv t_a t_b t_c t_d lift_mm [lead_s]

Times are plan seconds (the csv's lead-in, default 5.32 s, is added).  The
rear hip height ramps from the held height to +lift over [t_a, t_b], holds
it to t_c, and ramps back over [t_c, t_d].  Stance rows of the rear legs are
re-solved (theta for the row's beta at the new height, the levelled-stance
solve the planner uses); swing rows keep their beta and rotation, and their
fold / extension are scaled to start / end at the new stance theta.  Nothing
else changes, so the schedule is the one that was validated.

Why (hardware 2026-09-09, log 4.10): a swinging rear leg's own hip sags
45-58 mm once the fronts stand on the block, and the closed wheel's 74 mm of
planned clearance is gone at the fold -- the leg rolls on the ground.
"""
import csv
import sys

import numpy as np

sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")  # noqa: E402
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d  # noqa: E402
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import held_landing_pose_2d  # noqa: E402

REAR = {"RH": (4, 5, 2), "LH": (6, 7, 3)}   # theta col, beta col, phase col
THETA0 = 17.0


def main(src, dst, t_a, t_b, t_c, t_d, lift_mm, lead_s=5.32):
    hw = np.array([[float(v) for v in r] for r in csv.reader(open(src))])
    phase_path = src.replace("_hardware.csv", "_hardware_phase.csv")
    ph = np.array([[int(v) for v in r] for r in list(csv.reader(open(phase_path)))[1:]])
    assert len(ph) == len(hw), (len(ph), len(hw))
    posture = hybrid_posture_2d()
    hold = float(posture.hold_hip_z_m)
    t = np.arange(len(hw)) / 1000.0 - lead_s

    def lift_at(tp):
        L = lift_mm * 1e-3
        if tp <= t_a or tp >= t_d:
            return 0.0
        if tp < t_b:
            return L * (tp - t_a) / (t_b - t_a)
        if tp <= t_c:
            return L
        return L * (t_d - tp) / (t_d - t_c)

    def solve(beta_deg, z):
        b = np.deg2rad(((beta_deg + 180.0) % 360.0) - 180.0)
        theta, _ = held_landing_pose_2d(posture, float(b), 0.0, hold + z)
        return float(np.degrees(theta))

    # self-consistency: the solve at the held height must give the csv's own theta
    errs = []
    for name, (jt, jb, jp) in REAR.items():
        for i in range(int((2.3 + lead_s) * 1000), int((3.0 + lead_s) * 1000), 50):
            if ph[i, jp] == 0:
                errs.append(solve(np.degrees(hw[i, jb]), 0.0) - np.degrees(hw[i, jt]))
    print(f"solve check on flat stance rows: theta error mean {np.mean(errs):+.2f} deg, max |{np.max(np.abs(errs)):.2f}|")

    out = hw.copy()
    report = []
    for name, (jt, jb, jp) in REAR.items():
        i0, i1 = int((t_a + lead_s) * 1000), int((t_d + lead_s) * 1000)
        # stance rows first: the solve is slow (~0.1 s), so every 10th
        # stance row and every row next to a swing are solved and the rest
        # interpolated over the row index (theta is smooth in beta and z).
        rows = [i for i in range(i0, i1 + 1) if ph[i, jp] == 0]
        grid = [i for k, i in enumerate(rows)
                if k % 10 == 0 or k == len(rows) - 1 or ph[i - 1, jp] == 1 or ph[i + 1, jp] == 1]
        vals = [solve(np.degrees(hw[i, jb]), lift_at(t[i])) for i in grid]
        interp = np.interp(rows, grid, vals)
        for i, v in zip(rows, interp):
            out[i, jt] = np.deg2rad(v)
        # swings inside the window: scale fold and extension
        i = i0
        while i <= i1:
            if ph[i, jp] == 1:
                j = i
                while j + 1 < len(hw) and ph[j + 1, jp] == 1:
                    j += 1
                th = np.degrees(hw[i:j + 1, jt])
                take_old = np.degrees(hw[i - 1, jt]); take_new = np.degrees(out[i - 1, jt])
                land_old = np.degrees(hw[j + 1, jt]); land_new = np.degrees(out[j + 1, jt])
                low = np.where(th <= THETA0 + 0.05)[0]
                k_f = (take_new - THETA0) / max(take_old - THETA0, 1e-6)
                k_e = (land_new - THETA0) / max(land_old - THETA0, 1e-6)
                new = th.copy()
                if len(low):
                    f_end, e_start = low[0], low[-1]
                    new[:f_end + 1] = THETA0 + (th[:f_end + 1] - THETA0) * k_f
                    new[e_start:] = THETA0 + (th[e_start:] - THETA0) * k_e
                else:
                    new = THETA0 + (th - THETA0) * k_f
                out[i:j + 1, jt] = np.deg2rad(new)
                report.append(f"  {name} swing plan t {t[i]:.2f}..{t[j]:.2f}: takeoff theta {take_old:.1f}->{take_new:.1f}, "
                              f"landing {land_old:.1f}->{land_new:.1f}, fold x{k_f:.2f}, extend x{k_e:.2f}")
                i = j + 1
            else:
                i += 1
    print("\n".join(report))
    rate_old = np.degrees(np.abs(np.diff(hw[:, :8], axis=0))).max()
    rate_new = np.degrees(np.abs(np.diff(out[:, :8], axis=0))).max()
    win = slice(int((t_a + lead_s) * 1000), int((t_d + lead_s) * 1000))
    rate_win = np.degrees(np.abs(np.diff(out[win, :8], axis=0))).max()
    print(f"max joint step: whole file {rate_old:.3f} -> {rate_new:.3f} deg/ms; inside the window {rate_win:.3f} deg/ms")
    th_out = np.degrees(out[:, [0, 2, 4, 6]])
    print(f"theta range {th_out.min():.1f}..{th_out.max():.1f} deg; rows below 17 deg {(th_out < 16.99).sum()}")
    with open(dst, "w", newline="") as fh:
        w = csv.writer(fh)
        for row in out:
            w.writerow([f"{v:.6f}" for v in row])
    with open(dst.replace("_hardware.csv", "_hardware_phase.csv"), "w") as fh:
        fh.write(open(phase_path).read())
    print("wrote", dst)


if __name__ == "__main__":
    a = sys.argv
    main(a[1], a[2], *[float(v) for v in a[3:8]], *([float(a[8])] if len(a) > 8 else []))
