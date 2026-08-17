"""Composite body inertia of the ABAD Corgi, from the Webots proto.

Stage 2b task 0. The coronal (BIP) model of Chang 2022 is parameterized by a
dimensionless body inertia J~, and their entire Fig. 13 stability result is swept
over it -- so a wrong roll inertia invalidates the stage. Nothing in this repo
recorded one: `legwheel/config/__init__.py` has masses and offsets but no inertia,
the workspace URDF (`corgi_sim/resource/corgi.urdf`) is a five-line stub with no
<inertial> blocks at all, and the vault flags the proto's inertias as unverified.

The proto is the only source. It carries a full `physics Physics { mass,
centerOfMass, inertiaMatrix }` per link, but it is a 2.4-million-line file that is
almost entirely IndexedFaceSet mesh data and it lives inside WSL, unreachable over
UNC from Windows. So the structural skeleton is extracted once into
`data/corgi_abad_massprops.skel` (1619 lines: node opens/closes, translations,
rotations, and the mass properties, in file order) and this script parses that.

Regenerate the skeleton with `--regen` if the proto changes.

CONVENTIONS

Webots `inertiaMatrix` is two triples, [Ixx Iyy Izz] then [Ixy Ixz Iyz], taken
about `centerOfMass` and expressed in the owning Solid's own axes. `rotation` is
axis-angle. A Solid's translation/rotation place it in its parent's frame, so the
world pose is the composition down the node stack.

Body axes: +x forward, +y left, +z up. So Ixx is ROLL -- the axis this stage cares
about -- Iyy is pitch and Izz is yaw.

CAVEAT WORTH CARRYING

Composite inertia is CONFIGURATION-DEPENDENT and the proto encodes the joints at
their zero pose, not the theta ~ 100 deg nominal stance. The dominant roll term is
m*dy^2 for the four 4.7 kg leg modules, whose y-offsets are set by the hip spacing
and do not move with theta, so the headline number is robust -- but the smaller
link contributions do shift. Treat the third significant figure as soft.

Run:
    uv run python examples/gslip/body_inertia.py
    uv run python examples/gslip/body_inertia.py --regen   # re-extract skeleton
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
SKEL = REPO / "data" / "corgi_abad_massprops.skel"

# Inside WSL. Only needed for --regen.
PROTO_WSL = (
    "~/corgi_ws/corgi_ros2_ws/src/corgi_sim/protos/CorgiRobotABAD.proto"
)

_AWK = r"""
  { line=$0; s=line; gsub(/^[ \t]+|[ \t]+$/,"",s) }
  grab>0 { print "VAL " s; grab--; next }
  s ~ /\{$/ { print "OPEN " s; next }
  s == "}" { print "CLOSE"; next }
  s ~ /^translation / { print "T " s; next }
  s ~ /^rotation / { print "R " s; next }
  s ~ /^mass / { print "M " s; next }
  s ~ /^centerOfMass \[$/ { print "COM"; grab=1; next }
  s ~ /^inertiaMatrix \[$/ { print "INE"; grab=2; next }
"""


def regenerate_skeleton() -> None:
    """Re-extract the skeleton from the proto inside WSL."""
    out = str(SKEL).replace("C:\\", "/mnt/c/").replace("\\", "/")
    cmd = f"awk '{_AWK}' {PROTO_WSL} > {out} && wc -l < {out}"
    r = subprocess.run(
        ["wsl.exe", "-e", "bash", "-lc", cmd],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise SystemExit(f"skeleton regeneration failed:\n{r.stderr}")
    print(f"skeleton regenerated: {r.stdout.strip()} lines -> {SKEL}")


def _nums(s: str) -> list[float]:
    """Every numeric token on a skeleton line, non-numeric tokens skipped.

    Lines carry their field name ("T translation 0.255 0.12 0.057166"), so the
    leading tokens are words and must be stepped over rather than treated as a
    parse failure. The root node carries `translation IS translation` -- a proto
    parameter reference, not a value -- which yields an empty list and is read as
    identity. That is right: it resolves to the robot's placement in the world,
    which a body-frame inertia does not depend on.
    """
    out = []
    for tok in s.replace(",", " ").split():
        try:
            out.append(float(tok))
        except ValueError:
            continue
    return out


def _rot(axis_angle: list[float]) -> np.ndarray:
    """Rodrigues rotation matrix from Webots axis-angle [x, y, z, theta]."""
    if len(axis_angle) != 4:
        return np.eye(3)
    axis = np.array(axis_angle[:3], dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    k = axis / n
    th = axis_angle[3]
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * kx + (1 - np.cos(th)) * (kx @ kx)


class Frame:
    """One node on the transform stack."""

    __slots__ = ("t", "r")

    def __init__(self) -> None:
        self.t = np.zeros(3)
        self.r = np.eye(3)


def parse(path: Path) -> list[dict]:
    """-> one record per Physics block: world CoM, world inertia tensor, mass."""
    stack: list[Frame] = []
    links: list[dict] = []
    pending: dict | None = None
    want: str | None = None

    for raw in path.read_text().splitlines():
        tag, _, rest = raw.partition(" ")

        if tag == "OPEN":
            stack.append(Frame())
            if rest.startswith("physics Physics"):
                pending = {}
            continue

        if tag == "CLOSE":
            if pending is not None and {"m", "com", "ine"} <= pending.keys():
                # The Physics node itself is on the stack; its owning Solid is
                # everything below it.
                rot = np.eye(3)
                pos = np.zeros(3)
                for f in stack[:-1]:
                    pos = pos + rot @ f.t
                    rot = rot @ f.r
                ixx, iyy, izz = pending["ine"][0]
                ixy, ixz, iyz = pending["ine"][1]
                i_local = np.array(
                    [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
                )
                links.append({
                    "m": pending["m"],
                    "com": pos + rot @ np.array(pending["com"]),
                    "I": rot @ i_local @ rot.T,
                })
                pending = None
            elif pending is not None:
                pending = None
            if stack:
                stack.pop()
            continue

        if tag == "T" and stack:
            v = _nums(raw)
            if v:
                stack[-1].t = np.array(v[:3])
        elif tag == "R" and stack:
            v = _nums(raw)
            if v:
                stack[-1].r = _rot(v[:4])
        elif tag == "M" and pending is not None:
            v = _nums(raw)
            if v:
                pending["m"] = v[0]
        elif tag == "COM" and pending is not None:
            want = "com"
        elif tag == "INE" and pending is not None:
            want = "ine"
            pending["ine"] = []
        elif tag == "VAL" and pending is not None:
            vals = [float(x) for x in rest.replace(",", " ").split()]
            if want == "com":
                pending["com"] = vals
                want = None
            elif want == "ine":
                pending["ine"].append(vals)
                if len(pending["ine"]) == 2:
                    want = None

    return links


def gyration(link: dict) -> float:
    """Largest radius of gyration implied by a link's own inertia, metres.

    sqrt(I_max / m). A rigid body cannot have one larger than its own physical
    extent, so this is a hard plausibility check on the proto's numbers.
    """
    return float(np.sqrt(max(np.diag(link["I"])) / link["m"]))


def implausible(links: list[dict], max_gyration: float) -> list[dict]:
    """Links whose own inertia is impossible for a body of this robot's size."""
    return [l for l in links if gyration(l) > max_gyration]


def composite(links: list[dict], drop_own: list[dict] | None = None) -> dict:
    """Total mass, CoM, and inertia tensor about that CoM (parallel axis).

    `drop_own` names links whose *own* inertia tensor is discarded while their
    mass and position are kept. That is the right treatment for a link with a
    corrupt inertiaMatrix: the mass and CoM are independently corroborated, and a
    small part's own inertia is a minor term anyway, so zeroing it costs far less
    accuracy than propagating a value that is orders of magnitude wrong.
    """
    dropped = {id(l) for l in (drop_own or [])}
    m_tot = sum(l["m"] for l in links)
    com = sum(l["m"] * l["com"] for l in links) / m_tot

    i_tot = np.zeros((3, 3))
    for l in links:
        d = l["com"] - com
        if id(l) not in dropped:
            i_tot += l["I"]
        i_tot += l["m"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return {"mass": m_tot, "com": com, "I": i_tot}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("CONVENTIONS")[0])
    ap.add_argument("--regen", action="store_true",
                    help="re-extract the skeleton from the proto inside WSL")
    ap.add_argument("--skel", type=Path, default=SKEL)
    ap.add_argument("--max-gyration", type=float, default=0.5,
                    help="radius-of-gyration plausibility bound, m "
                         "(default 0.5, generous for a 0.51 m wheelbase)")
    args = ap.parse_args()

    if args.regen:
        regenerate_skeleton()

    if not args.skel.exists():
        raise SystemExit(f"missing skeleton {args.skel}; run with --regen")

    links = parse(args.skel)
    bad = implausible(links, args.max_gyration)
    raw = composite(links)
    c = composite(links, drop_own=bad)
    ixx, iyy, izz = np.diag(c["I"])

    if bad:
        print("!" * 72)
        print(f"{len(bad)} link(s) carry a PHYSICALLY IMPOSSIBLE inertiaMatrix "
              f"and their own inertia has been dropped:")
        for l in bad:
            print(f"   m = {l['m']:.4f} kg,  I = "
                  f"{np.round(np.diag(l['I']), 5)},  "
                  f"radius of gyration = {gyration(l):.2f} m")
        print("A rigid body's radius of gyration cannot exceed its own extent.")
        print("Mass and position are kept -- both are corroborated by the")
        print("composite CoM; only the corrupt tensor is discarded.")
        print(f"Raw (uncorrected) diagonal was "
              f"{np.round(np.diag(raw['I']), 4)} kg m^2.")
        print("!" * 72)
        print()

    print(f"links with physics : {len(links)}")
    print(f"total mass         : {c['mass']:.4f} kg")
    print(f"center of mass     : "
          f"[{c['com'][0]:+.5f} {c['com'][1]:+.5f} {c['com'][2]:+.5f}] m")
    print()
    print("inertia tensor about the CoM, body axes (kg m^2):")
    for row in c["I"]:
        print("   " + "  ".join(f"{v:+.6f}" for v in row))
    print()
    print(f"  I_roll  (Ixx) = {ixx:.6f} kg m^2   <-- the Stage 2b number")
    print(f"  I_pitch (Iyy) = {iyy:.6f}")
    print(f"  I_yaw   (Izz) = {izz:.6f}")

    # Chang 2022 nondimensionalizes by mass and half-track. The CONTACT track is
    # 0.4234 m (all four wheel planes sit outboard of the hips by
    # WHEEL_AXIAL_OFFSET), not the 0.240 m hip spacing -- see Stage 0.
    half_track = 0.4234 / 2.0
    print()
    print(f"  J~ = I_roll / (m * w^2), w = half contact track = {half_track:.4f} m")
    print(f"     = {ixx / (c['mass'] * half_track**2):.4f}")

    heaviest = sorted(links, key=lambda l: -l["m"])[:6]
    print()
    print("heaviest links (mass, world CoM, roll contribution m*(dy^2+dz^2)):")
    for l in heaviest:
        d = l["com"] - c["com"]
        print(f"   {l['m']:7.4f} kg  "
              f"[{l['com'][0]:+.4f} {l['com'][1]:+.4f} {l['com'][2]:+.4f}]  "
              f"{l['m'] * (d[1]**2 + d[2]**2):.6f}")


if __name__ == "__main__":
    main()
