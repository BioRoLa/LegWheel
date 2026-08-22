"""Export the hip-to-contact length lookup table L(theta) for the Tier 3
velocity-slaved stance sweep (event-scheduler thread, fix-order step 4).

The controller drives beta-dot = v / L(theta) during stance. L must use THE
SAME length convention as the template generator or a constant scale error
appears as a body-speed error (the P11 falsifier) -- so this table is
generated from the same LegLengthMap the template comes from, by
construction:

    L(theta) = leg_map.length(theta) + foot_radius

which at theta = 100 deg reproduces the 0.2931 m standing hip height the
template's l0 is built from (export_pronk_csv.py).

Output: output/csv/gslip_ltheta_lut.csv with header theta_deg,L_m, 1-degree
resolution over the linkage's 17..140 deg range. The controller loads it at
startup and linear-interpolates; no linkage arithmetic in the loop.

Run:
    uv run python examples/gslip/export_ltheta_lut.py
"""

import numpy as np

from legwheel.config import OUTPUT_CSV_DIR
from legwheel.planners import gslip_to_corgi as g2c


def main() -> None:
    leg_map = g2c.LegLengthMap()
    fr = leg_map.leg.foot_radius
    out = OUTPUT_CSV_DIR / "gslip_ltheta_lut.csv"
    thetas = np.arange(17.0, 140.0 + 0.5, 1.0)
    with open(out, "w") as fh:
        fh.write("theta_deg,L_m\n")
        for th in thetas:
            L = leg_map.length(np.deg2rad(th)) + fr
            fh.write(f"{th:.1f},{L:.6f}\n")
    l100 = leg_map.length(np.deg2rad(100.0)) + fr
    print(f"wrote {out} ({len(thetas)} rows, foot_radius {fr:.4f})")
    print(f"convention check: L(100 deg) = {l100:.4f} m "
          f"(template l0, expect 0.2931)")


if __name__ == "__main__":
    main()
