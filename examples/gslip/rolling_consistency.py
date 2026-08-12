"""Does the commanded template actually satisfy the rolling constraint?

The measured friction ordering runs backwards -- mu 0.6 gives 0.794 m/s, mu 1.0
gives 0.510, mu 1.6 gives 0.415. Less grip is faster. For a legged robot that is
the wrong way round: grip normally buys traction. An inversion means the feet
are being commanded through a motion they cannot perform, and friction is what
charges for the discrepancy.

Rolling contact is a hard kinematic constraint. The foot rim is rigid, its
absolute angle is beta (the angle-bisecting gear train, see Lee/Yu/Lin 2026), so
while a foot is down the hip MUST advance at

    v_hip = L(theta) * beta_dot,      L = |hip - contact| = |OO_r|(theta) + r

at every instant. Any mismatch between that and the body's actual horizontal
velocity is slip that the template is REQUIRING, before any controller or
contact model gets involved.

The body's horizontal speed through a stance is set by the SLIP dynamics and
varies smoothly and modestly -- a spring-mass system does not change direction
or double its speed mid-stance. So if L*beta_dot swings wildly across the stance
phase, the template cannot be rolling throughout it, and the feet must be
sliding for part of every step.

    uv run python examples/gslip/rolling_consistency.py [template.csv ...]
"""
import os
import sys

import numpy as np

from legwheel.models.leg_model import LegModel

CFG = "/mnt/c/Users/alexc/code/LegWheel/output/csv"
DEFAULTS = [
    ("v~1.20 (shipped)", "gslip_pronk_template.csv"),
    ("v~0.70", "gslip_pronk_template_v070.csv"),
    ("v~0.45", "gslip_pronk_template_v045.csv"),
]


def rolling_radius(lm, theta):
    """|hip - contact| at this theta: hip->rim centre plus the rim radius."""
    lm.forward(theta, 0.0, vector=True)
    return float(np.hypot(lm.O_r[0], lm.O_r[1])) + lm.foot_radius


def analyse(name, path, lm):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    t, th, be = raw["t"], raw["theta"], raw["beta"]
    st = raw["in_stance"] > 0.5
    if st.sum() < 10:
        print(f"{name}: no stance samples")
        return

    dt = float(np.median(np.diff(t)))
    bdot = np.gradient(be, dt)
    L = np.array([rolling_radius(lm, x) for x in th])
    v_req = L * bdot                       # m/s the hip MUST move, to roll

    s = v_req[st]
    # Trim the first and last few samples: the stance boundary has a
    # discontinuity in beta_dot that is a differencing artefact, not a command.
    k = max(2, len(s) // 50)
    s = s[k:-k]

    print(f"{name}")
    print(f"  stance {100*st.mean():.1f}% of stride, "
          f"beta swept {np.rad2deg(be[st][-1] - be[st][0]):+.2f} deg, "
          f"L = {L[st].min():.4f}..{L[st].max():.4f} m")
    print(f"  required hip speed over stance: "
          f"min {s.min():+.3f}  mean {s.mean():+.3f}  max {s.max():+.3f} m/s")
    print(f"    swing = {s.max() - s.min():.3f} m/s, "
          f"{(s.max() - s.min())/abs(s.mean()):.1f}x the mean")
    neg = 100.0 * float((s < 0).mean())
    print(f"    fraction of stance requiring the hip to move BACKWARDS: "
          f"{neg:.1f}%")
    # The distance the template would cover if the feet rolled perfectly.
    roll_dist = float(np.trapezoid(v_req[st], dx=dt))
    print(f"  rolling would advance the hip {roll_dist:.4f} m per stride")
    print()
    return dict(name=name, swing=s.max() - s.min(), mean=s.mean(),
                neg=neg, roll=roll_dist)


def main():
    lm = LegModel()
    args = sys.argv[1:]
    items = ([(os.path.basename(a), a) for a in args] if args else
             [(n, os.path.join(CFG, f)) for n, f in DEFAULTS])

    print("Rolling constraint: while a foot is down, v_hip = L(theta)*beta_dot.")
    print("A spring-mass body's horizontal speed varies smoothly through")
    print("stance, so a large swing here means the command is not rollable.\n")
    for n, p in items:
        if os.path.exists(p):
            analyse(n, p, lm)
        else:
            print(f"{n}: not found at {p}\n")


if __name__ == "__main__":
    main()
