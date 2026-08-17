"""Patch the four *_LEG solids' impossible inertiaMatrix in CorgiRobotABAD.proto.

Found by examples/gslip/body_inertia.py (implementation log section 41). Each of
A_LEG..D_LEG carries:

    mass 0.05
    inertiaMatrix [ 0.27748 1.1978 0.96762
                    <products> ]

which is a radius of gyration of sqrt(1.1978 / 0.05) = 4.89 m on a 50-gram part,
in a robot 0.51 m long. No mass makes that tensor possible: even at the real
0.68-0.9 kg leg mass it still implies 1.15 m. It is almost certainly the other
half of the "zero-mass leg links (1e-06 kg) -> 0.05 kg" fix -- the mass was
patched and the tensor was not.

Webots uses mass/centerOfMass/inertiaMatrix directly when all three are given, so
the simulator has been angularly accelerating four legs with ~60x too much
rotational inertia every stride. That is a candidate cause of the unattributed
5-8x torque erosion.

WHAT THIS DOES, AND WHY THIS SHAPE OF FIX

A_LEG has NO boundingObject -- it is a container Solid whose children are joined
by HingeJoints and are therefore separate bodies. So Webots cannot auto-derive a
tensor if the explicit one is deleted, and the 0.05 kg is itself an arbitrary
numerical-stability value rather than a measured mass. The body is effectively a
connector frame.

So: scale the whole 6-tuple by a single factor that brings the largest diagonal
term to a physically possible magnitude for a small connector, and leave
centerOfMass alone. Scaling rather than replacing preserves the tensor's
anisotropy in case the exporter got the shape right and only the scale wrong, and
it keeps each leg's own product-of-inertia SIGNS, which differ per leg.

  target radius of gyration : 0.05 m (a small connector, not a leg)
  => I_max = m * k^2 = 0.05 * 0.05^2 = 1.25e-4 kg m^2

Idempotent: a block already at or below the target is left untouched, so this can
be re-run safely.

REVERT

`corgi_sim` is a SUBMODULE (on ABAD/gslip) and the proto is a 103 MB Git LFS
object, so the parent repo shows no diff for it and `git checkout` must be run
from inside the submodule:

    git -C ~/corgi_ws/corgi_ros2_ws/src/corgi_sim checkout -- protos/CorgiRobotABAD.proto

Checking `git status` from the workspace root reports nothing either way, which
is exactly the shape of confirmation that would be misleading here.

Run (from WSL, where the workspace lives):
    python3 fix_leg_inertia_patch.py --dry-run
    python3 fix_leg_inertia_patch.py
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

PROTO = Path.home() / (
    "corgi_ws/corgi_ros2_ws/src/corgi_sim/protos/CorgiRobotABAD.proto"
)
TARGET_MASS = 0.05
TARGET_GYRATION = 0.05  # m
TARGET_I_MAX = TARGET_MASS * TARGET_GYRATION**2  # 1.25e-4 kg m^2

# mass 0.05, then centerOfMass [ ... ], then inertiaMatrix [ a b c \n d e f ]
BLOCK = re.compile(
    r"(mass\s+0\.05\s*\n"
    r"\s*centerOfMass\s*\[\s*\n[^\]]*\]\s*\n"
    r"\s*inertiaMatrix\s*\[\s*\n)"
    r"(\s*)([-\d.eE+ ]+)\n"
    r"(\s*)([-\d.eE+ ]+)\n",
    re.MULTILINE,
)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("WHAT THIS")[0])
    ap.add_argument("--proto", type=Path, default=PROTO)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not args.proto.exists():
        raise SystemExit(f"proto not found: {args.proto}")

    text = args.proto.read_text()
    hits = {"patched": 0, "skipped": 0}

    def repl(m: re.Match) -> str:
        head, ind1, row1, ind2, row2 = m.groups()
        diag = [float(x) for x in row1.split()]
        prod = [float(x) for x in row2.split()]
        i_max = max(abs(v) for v in diag)

        if i_max <= TARGET_I_MAX * 1.001:
            hits["skipped"] += 1
            return m.group(0)

        s = TARGET_I_MAX / i_max
        hits["patched"] += 1
        gyr_before = (i_max / TARGET_MASS) ** 0.5
        print(f"  patch: I_max {i_max:.5g} -> {TARGET_I_MAX:.5g} "
              f"(x{s:.4g}); radius of gyration "
              f"{gyr_before:.2f} m -> {TARGET_GYRATION:.2f} m")
        d = "  ".join(f"{v * s:.6g}" for v in diag)
        p = "  ".join(f"{v * s:.6g}" for v in prod)
        return f"{head}{ind1}{d}\n{ind2}{p}\n"

    out = BLOCK.sub(repl, text)

    print(f"blocks patched : {hits['patched']}")
    print(f"blocks skipped : {hits['skipped']} (already within target)")

    if hits["patched"] == 0 and hits["skipped"] == 0:
        raise SystemExit(
            "NO mass-0.05 physics blocks matched -- the proto's formatting may "
            "have changed. Refusing to claim success; check the regex."
        )

    if args.dry_run:
        print("\n--dry-run: nothing written.")
        return

    args.proto.write_text(out)
    print(f"\nwritten: {args.proto}")
    print("REBUILD before this takes effect -- the launch loads the world from")
    print("install/, which is a real file and not a symlink:")
    print("  colcon build --packages-select corgi_sim")


if __name__ == "__main__":
    main()
