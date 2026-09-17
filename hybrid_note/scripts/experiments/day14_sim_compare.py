"""Compare a simulation log against the Day 14 plan that produced its CSV.

    python3 day14_sim_compare.py SIM.csv PLAN_hardware.csv PLAN_events.csv

Aligns sim time to CSV time by matching the first fold (cmd theta < 20 deg)
of every leg, then prints, per planned swing, the sim body x against the
plan's body x (the lag), and a table of pitch / height / contacts through
the crossing.  Legs: a=LF, b=RF, c=RH, d=LH.
"""
import csv
import sys

import numpy as np

LEGS = {"a": "LF", "b": "RF", "c": "RH", "d": "LH"}


def fold_starts(theta_deg, t):
    out, inside = [], False
    for i in range(len(theta_deg)):
        if theta_deg[i] < 20.0 and not inside:
            out.append(t[i]); inside = True
        elif theta_deg[i] >= 20.0:
            inside = False
    return out


def main(sim_path, hw_path, events_path):
    rows = list(csv.DictReader(open(sim_path)))
    t = np.array([float(r["Time"]) for r in rows])
    x = np.array([float(r["sim_pos_x"]) for r in rows]) * 1e3
    z = np.array([float(r["sim_pos_z"]) for r in rows]) * 1e3
    q = {k: np.array([float(r["sim_orien_" + k]) for r in rows]) for k in "xyzw"}
    pitch = np.degrees(np.arcsin(np.clip(2 * (q["w"] * q["y"] - q["z"] * q["x"]), -1, 1)))
    th = {l: np.degrees(np.array([float(r[f"cmd_theta_{l}"]) for r in rows])) for l in "abcd"}
    con = {l: np.array([float(r[f"sim_contact_state_{l}"]) for r in rows]) for l in "abcd"}

    hw = np.array([[float(v) for v in r] for r in csv.reader(open(hw_path))
                   if r and not r[0].startswith("t")])
    hw_t = np.arange(len(hw)) / 1000.0
    hw_th = {l: np.degrees(hw[:, 2 * k]) for k, l in enumerate("abcd")}
    # The CSV opens in the folded pose, so its first "fold" is not a swing.
    csv_folds = {l: [v for v in fold_starts(hw_th[l], hw_t) if v > 0.05] for l in "abcd"}
    sim_folds = {l: [v for v in fold_starts(th[l], t) if v > t[0] + 0.5] for l in "abcd"}
    events = [r for r in csv.DictReader(open(events_path)) if r["row_kind"] == "swing"]
    per_leg = {}
    for r in events:
        per_leg.setdefault(r["leg"], []).append(r)
    for l, name in LEGS.items():
        n = len(per_leg.get(name, []))
        print(f"{name}: {n} planned swings, {len(csv_folds[l])} csv folds, {len(sim_folds[l])} sim folds")
    # Plan time -> csv time: the csv's lead-in is the first real fold of the
    # first swinging leg minus that swing's planned start.  Csv -> sim: the
    # median offset over the legs' k-th folds.  (A hand-stretched csv breaks
    # the first mapping after the stretch; the driver's own pairs are exact.)
    first = events[0]
    l_first = [k for k, v in LEGS.items() if v == first["leg"]][0]
    lead_in = csv_folds[l_first][0] - float(first["start_s"])
    offsets = [u - v for l in "abcd" for u, v in zip(sim_folds[l], csv_folds[l])]
    offset = float(np.median(offsets))
    print(f"plan t = csv t - {lead_in:.3f} s; sim t = csv t + {offset:.3f} s "
          f"(fold offsets spread {np.std(offsets):.3f} s)")
    x0 = None
    print("planned swing            plan x   sim x   lag(mm)   sim t")
    timeline = [(float(r["start_s"]) + lead_in + offset, r) for r in events]
    for ts, r in timeline:
        i = int(np.argmin(abs(t - ts)))
        px = float(r["body_x_start_mm"])
        if x0 is None:
            x0 = px - x[i]
        print(f"  {r['leg']} {r['kind']:22s} {px:8.1f} {x[i]:8.1f} {px - x[i] - x0:8.1f} {t[i]:8.2f}")
    t_lo = min(p[0] for p in timeline) - 0.5
    t_hi = max(p[0] for p in timeline) + 1.5
    print("\nsim t   x    z   pitch | cmd theta a b c d | contacts a b c d")
    for T in np.arange(t_lo, t_hi, 0.25):
        i = int(np.argmin(abs(t - T)))
        print(f"{t[i]:6.2f} {x[i]:6.0f} {z[i]:5.0f} {pitch[i]:6.1f} | "
              + " ".join(f"{th[l][i]:4.0f}" for l in "abcd") + " | "
              + "".join(str(int(con[l][i])) for l in "abcd"))


if __name__ == "__main__":
    main(*sys.argv[1:4])
