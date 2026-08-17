"""Does the leg's SPEED budget have room for a taller gearbox?

Section 32 of the implementation log established the leg runs the STOCK 6:1
reduction and concluded "the gearbox escape is closed". That conclusion answers
"is there hidden torque multiplication already present in the drivetrain?" --
no, there is not. It does NOT answer "could the ratio be changed?", which is a
different question and the one this script exists to price.

The precedent is already on the robot: the ABAD was moved 6:1 -> 9:1 for exactly
this reason (see the Banking Turns note). Doing the same to the leg gives, from
S32's own scaling table:

    leg 6:1 (stock)  stall 29.5 N.m,  ~25 N.m usable at running speed,  330 rpm
    leg 9:1          stall 44.25 N.m, ~37 N.m usable at running speed,  220 rpm

The torque side is arithmetic. The SPEED side is what has never been checked,
and it is what decides whether the swap is free: 330 -> 220 rpm at the output is
a real cut, and if the templates need more than 220 rpm the swap breaks the gait
instead of rescuing it.

WHAT IS COMPARED, AND WHY IT IS THE OUTPUT SHAFT

The datasheet's "max speed after reduce, 330 rpm" is already an OUTPUT figure --
S32 verified this against the 36 V curve (250 rpm no-load x 48/36 = 333). The
leg linkage is driven by that output shaft, so the motor angles phi_L / phi_R
formed below are output angles and compare directly against 330 (6:1) and
220 (9:1). No further ratio conversion belongs anywhere in this file.

THE COUPLING

From Controller_TB.IK, per the correction in S8 of the log:

    phi_L = beta + (theta - theta_0)      phi_R = beta - (theta - theta_0)

with theta_0 = 17 deg, so phi_dot_L = beta_dot + theta_dot and
phi_dot_R = beta_dot - theta_dot. Only |phi_dot| is wanted here, and the
expression is symmetric under swapping the labels -- which matters, because S8
records that the L/R labels in the log run opposite to the code. Do not read a
physical side off this script's output.

TWO TRAPS, BOTH HIT ON THE FIRST RUN OF THIS SCRIPT

1. The periodic CSVs already DUPLICATE the endpoint -- theta[0] == theta[-1] and
   beta[0] == beta[-1] to 0.0 exactly. So the loop seam is already inside the
   file and differentiating it as an open interval is correct. The first version
   of this script appended a synthetic last-to-first step, which was a spurious
   zero on every template. Now asserted rather than assumed: assert_closed()
   fails loudly if a template stops being written that way.

2. The speed ramp's peak is a COMMANDED STEP, not a gait requirement. Its worst
   interval is a 7.0000 deg beta jump in a single 1 ms sample -- 1166.7 rpm --
   at a rung join, and there are exactly five such joins (7, 4, 3, 2, 2 deg).
   Excluding them the ramp peaks at 142.4 rpm, in line with the pronk templates.
   The 7 deg step is already flagged in the weekly log as wanting to be split
   out of the hop; this script finds it independently and prices it.

   Joins are REPORTED INDIVIDUALLY rather than silently filtered, because "the
   number got better after I removed some samples" is exactly the shape of
   result this project has learned to distrust.

Run:
    .venv/bin/python examples/gslip/motor_rate_budget.py
"""

import csv
import pathlib

import numpy as np

# Output-shaft ceilings, from the HT-04 datasheet via S32. These are what the
# computed rates are compared against; see the module docstring.
RPM_CEILING = {"6:1 (stock)": 330.0, "9:1": 220.0, "12:1": 165.0}

# An interval faster than this is treated as a commanded discontinuity rather
# than a trajectory. Set well above the smooth templates' peak (~143 rpm) and
# well below the smallest observed join (~334 rpm), so the classification is not
# sensitive to where in that gap it sits.
JOIN_RPM = 250.0

CSV_DIR = pathlib.Path(__file__).resolve().parents[2] / "output" / "csv"

TEMPLATES = (
    ("hop 30 mm", "gslip_hop_template.csv", True),
    ("pronk v~0.45", "gslip_pronk_template_v045.csv", True),
    ("pronk v~0.70", "gslip_pronk_template_v070.csv", True),
    ("pronk v~1.20 (shipped)", "gslip_pronk_template.csv", True),
    ("speed ramp", "gslip_speed_ramp_template.csv", False),
)

RAD_S_TO_RPM = 60.0 / (2.0 * np.pi)


def load(path):
    t, theta, beta, in_stance = [], [], [], []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            t.append(float(row["t"]))
            theta.append(float(row["theta"]))
            beta.append(float(row["beta"]))
            in_stance.append(int(row["in_stance"]))
    return (np.array(t), np.array(theta), np.array(beta),
            np.array(in_stance, dtype=bool))


def assert_closed(label, theta, beta):
    """A periodic template must repeat its first row as its last.

    Trap 1 in the module docstring. If this ever stops holding, the loop seam
    leaves the file and every rate below silently misses the fastest commanded
    transition in the gait.
    """
    dth, dbe = abs(theta[0] - theta[-1]), abs(beta[0] - beta[-1])
    if max(dth, dbe) > 1e-9:
        raise AssertionError(
            f"{label}: template is not closed (d_theta={dth:.3e}, "
            f"d_beta={dbe:.3e}). The seam is outside the file -- fix the rate "
            f"computation before trusting any number here.")


def motor_rpm(t, theta, beta):
    """|phi_dot| in output rpm, per sample interval, both motors."""
    dt = np.diff(t)
    th_dot, be_dot = np.diff(theta) / dt, np.diff(beta) / dt
    return np.maximum(np.abs(be_dot + th_dot),
                      np.abs(be_dot - th_dot)) * RAD_S_TO_RPM


def phase_masks(in_stance, n):
    """Stance / flight masks over INTERVALS, not samples.

    An interval counts as stance only if both endpoints are stance, so the
    touchdown and liftoff transitions belong to neither bucket rather than
    being attributed to the wrong one.
    """
    a, b = in_stance[:n], in_stance[1:n + 1]
    return a & b, (~a) & (~b)


def main() -> None:
    print()
    print("=" * 78)
    print("LEG MOTOR RATE BUDGET  --  is a taller gearbox free on speed?")
    print("=" * 78)
    print("  rates are OUTPUT-SHAFT rpm, directly comparable to the datasheet's")
    print("  'max speed after reduce' figures:")
    for name, ceil in RPM_CEILING.items():
        print(f"    {name:<12} {ceil:6.0f} rpm")
    print()
    print(f"{'template':<24} {'peak':>7} {'stance pk':>10} {'stance med':>11} "
          f"{'flight pk':>10} {'joins':>6}")

    worst, worst_label = 0.0, ""
    all_joins = []
    for label, fname, periodic in TEMPLATES:
        path = CSV_DIR / fname
        if not path.exists():
            print(f"{label:<24}   -- {fname} not found --")
            continue
        t, theta, beta, in_stance = load(path)
        if periodic:
            assert_closed(label, theta, beta)

        rpm = motor_rpm(t, theta, beta)
        n = len(rpm)
        st, fl = phase_masks(in_stance, n)

        join = rpm > JOIN_RPM
        for j in np.flatnonzero(join):
            all_joins.append((label, float(t[j]),
                              float(np.rad2deg(beta[j + 1] - beta[j])),
                              float(rpm[j])))

        gait = rpm[~join]
        peak = float(gait.max())
        if peak > worst:
            worst, worst_label = peak, label

        s = rpm[st & ~join]
        f = rpm[fl & ~join]
        s_pk = float(s.max()) if s.size else float("nan")
        s_md = float(np.median(s)) if s.size else float("nan")
        f_pk = float(f.max()) if f.size else float("nan")

        print(f"{label:<24} {peak:7.1f} {s_pk:10.1f} {s_md:11.1f} "
              f"{f_pk:10.1f} {int(join.sum()):6d}")

    print()
    print(f"  peak / stance / flight EXCLUDE commanded steps (> {JOIN_RPM:.0f} rpm");
    print("  in one sample); those are listed below, not silently dropped.")
    print("  intervals spanning touchdown or liftoff are in neither phase bucket.")

    if all_joins:
        print()
        print("  COMMANDED STEPS FOUND -- these are template construction, not gait:")
        print(f"    {'template':<24} {'t (s)':>8} {'d beta (deg)':>13} {'rpm':>9}")
        for lbl, tj, dbe, r in all_joins:
            print(f"    {lbl:<24} {tj:8.4f} {dbe:+13.4f} {r:9.1f}")
        print()
        print("    These are rung joins in the ramp: a step in beta inside one")
        print("    1 ms sample. No gearbox can track a discontinuity, and no")
        print("    gearbox needs to -- the fix is to ease the join, which the")
        print("    weekly log already lists as a to-do for the 7 deg one.")

    print()
    print("=" * 78)
    print("VERDICT")
    print("=" * 78)
    print(f"  worst genuine gait requirement: {worst:.1f} rpm  ({worst_label})")
    print()
    for name, ceil in RPM_CEILING.items():
        head = ceil / worst
        verdict = "FITS" if head >= 1.3 else "MARGINAL" if head >= 1.0 \
            else "EXCEEDS"
        print(f"    {name:<12} ceiling {ceil:6.0f} rpm -> {verdict:<9}"
              f" {head:.2f}x headroom")
    print()
    print("  ASSUMPTION, and the thing to verify before this is load-bearing:")
    print("    equal gearbox efficiency at 6:1 and 9:1. S32 flags this as the")
    print("    check if the ratio scaling is ever relied on, and it is not")
    print("    something this script can see.")
    print()
    print("  This prices the SPEED side only. Whether 9:1's ~37 N.m usable is")
    print("  ENOUGH is the torque question, and it depends on the erosion")
    print("  decomposition, which has not been done. Do not read a feasible")
    print("  speed budget as a feasible trot.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
