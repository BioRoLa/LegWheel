"""Day 3--5 Step 0 inventory of the existing LegWheel geometry APIs.

This module deliberately does not implement terrain queries.  It collects the
geometry already exposed by the legacy kinematics code so the next steps can
reuse it with explicit angle and frame conventions.
"""

from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[3]
for path in (SCRIPT_DIR, PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ground_contact_pose_3d import compute_ground_contact_pose_3d  # noqa: E402
from ground_contact_single_pose import compute_ground_contact  # noqa: E402


SURFACE_COLORS = {
    "foot_rim": "#2ca02c",
    "upper_tyre_l": "#1f77b4",
    "upper_tyre_r": "#ff7f0e",
}


def rim_classification_rows() -> list[dict]:
    """Return the semantic F/L/R/N taxonomy used by ContactMap_Note_ICRA."""

    return [
        {
            "state_id": "F",
            "semantic_class": "foot_rim",
            "valid_contact_rim": True,
            "query_contact role": "may produce ContactCandidate(rim=RimId.FOOT)",
        },
        {
            "state_id": "L",
            "semantic_class": "left_rim",
            "valid_contact_rim": True,
            "query_contact role": "may produce ContactCandidate(rim=RimId.LEFT)",
        },
        {
            "state_id": "R",
            "semantic_class": "right_rim",
            "valid_contact_rim": True,
            "query_contact role": "may produce ContactCandidate(rim=RimId.RIGHT)",
        },
        {
            "state_id": "N",
            "semantic_class": "non_contact_region",
            "valid_contact_rim": False,
            "query_contact role": "diagnostic/collision geometry only; never a contact candidate",
        },
    ]


def geometry_capability_rows() -> list[dict]:
    """Return the Step-0 API inventory and the remaining exposure gaps."""

    return [
        {
            "geometry/data": "2D linkage joints",
            "existing API": "PlotLeg.forward(theta, beta); attributes A_l...H_extend_r",
            "available now": "yes",
            "frame / unit": "legacy leg plane (x, y_height), theta/beta in rad",
            "Step 1+ action": "reuse; wrap in a stable geometry result",
        },
        {
            "geometry/data": "named rim centers + endpoints",
            "existing API": "PlotLeg.leg_shape.<surface>.arc[1]",
            "available now": "yes",
            "frame / unit": "legacy leg plane, metre; endpoints implicit in arc limits",
            "Step 1+ action": "expose as numeric arrays (not Matplotlib objects)",
        },
        {
            "geometry/data": "3 physically contactable tyre surfaces / samples",
            "existing API": "sample_contact_geometry_points[_3d](...) ",
            "available now": "yes",
            "frame / unit": "2D leg plane or display-like 3D, metre",
            "Step 1+ action": "map foot/left/right tyre to F/L/R and the open upper gap to N",
        },
        {
            "geometry/data": "global rim parameter alpha",
            "existing API": "LegModel.rim_point(alpha, w)",
            "available now": "partial",
            "frame / unit": "alpha is degree; w is metre; 0 deg is foot-rim centre",
            "Step 1+ action": "add explicit deg-to-rad semantic adapter",
        },
        {
            "geometry/data": "per-arc geometric angle",
            "existing API": "surface_records[*].arc_angle_deg",
            "available now": "yes",
            "frame / unit": "degree about each individual arc centre",
            "Step 1+ action": "do not treat this value as global contact alpha",
        },
        {
            "geometry/data": "lowest point / lowest region",
            "existing API": "compute_ground_contact[_pose_3d](...) ",
            "available now": "yes",
            "frame / unit": "2D y_min or display-like 3D z_min",
            "Step 1+ action": "reuse only as flat-ground regression oracle",
        },
        {
            "geometry/data": "3D body-frame contact point",
            "existing API": "CorgiLegKinematics.forward_kinematics(...) ",
            "available now": "yes",
            "frame / unit": "body frame {B}, metre; alpha input is degree",
            "Step 1+ action": "use for the authoritative body/module transform",
        },
        {
            "geometry/data": "3D linkage points",
            "existing API": "CorgiLegKinematics.get_joint_positions(...) ",
            "available now": "partial",
            "frame / unit": "body frame {B}, metre; returns O...G only",
            "Step 1+ action": "extend if collision tests require H/I/J/rim centres",
        },
        {
            "geometry/data": "world-frame geometry",
            "existing API": "none in the sampled-contact helpers",
            "available now": "no",
            "frame / unit": "public planner requires {W}: +x forward, +y left, +z up",
            "Step 1+ action": "Step 2 must apply T_WB/T_WH explicitly",
        },
        {
            "geometry/data": "whole-linkage collision surface",
            "existing API": "PlotLeg bars/joints are drawable primitives",
            "available now": "partial",
            "frame / unit": "legacy leg plane",
            "Step 1+ action": "expose numeric segments/circles before Step 5",
        },
    ]


def _surface_summary(records: list[dict]) -> list[dict]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for record in records:
        if record["geometry_type"] == "rim_arc":
            grouped[record["surface_name"]].append(record)

    rows = []
    for surface_name, samples in grouped.items():
        first, last = samples[0], samples[-1]
        states = sorted({sample["contact_state"] for sample in samples})
        rows.append(
            {
                "surface_name": surface_name,
                "semantic_states": ", ".join(states),
                "sample_count": len(samples),
                "center_x_m": first["surface_center_x_m"],
                "center_y_m": first["surface_center_y_m"],
                "endpoint_0_x_m": first["x_m"],
                "endpoint_0_y_m": first["y_m"],
                "endpoint_1_x_m": last["x_m"],
                "endpoint_1_y_m": last["y_m"],
                "arc_angle_0_deg": first["arc_angle_deg"],
                "arc_angle_1_deg": last["arc_angle_deg"],
            }
        )
    return rows


def build_leg_geometry_inventory(
    theta: float,
    beta: float,
    gamma: float,
    *,
    arc_samples: int = 121,
    lateral_samples: int = 9,
    contact_height_tol: float = 1e-3,
) -> dict:
    """Evaluate all existing rim geometry and the legacy lowest-region result."""

    if arc_samples < 2 or lateral_samples < 1:
        raise ValueError("arc_samples must be >= 2 and lateral_samples must be >= 1")

    contact_2d = compute_ground_contact(
        theta,
        beta,
        contact_height_tol=contact_height_tol,
        arc_samples=arc_samples,
        include_reference_points=True,
    )
    contact_3d = compute_ground_contact_pose_3d(
        theta,
        beta,
        gamma,
        contact_height_tol=contact_height_tol,
        arc_samples=arc_samples,
        lateral_samples=lateral_samples,
        include_reference_points=False,
    )
    return {
        "theta": float(theta),
        "beta": float(beta),
        "gamma": float(gamma),
        "surface_summary": _surface_summary(contact_2d["surface_records"]),
        "rim_classification_rows": rim_classification_rows(),
        "capability_rows": geometry_capability_rows(),
        "contact_2d": contact_2d,
        "contact_3d": contact_3d,
    }


def plot_leg_geometry_inventory(inventory: dict):
    """Plot all named rim geometry and highlight the legacy lowest region."""

    contact_2d = inventory["contact_2d"]
    contact_3d = inventory["contact_3d"]
    fig = plt.figure(figsize=(15, 6))
    ax_2d = fig.add_subplot(1, 2, 1)
    ax_3d = fig.add_subplot(1, 2, 2, projection="3d")

    for row in inventory["surface_summary"]:
        name = row["surface_name"]
        samples = [r for r in contact_2d["surface_records"] if r["surface_name"] == name]
        points = np.array([[r["x_m"], r["y_m"]] for r in samples])
        color = SURFACE_COLORS[name]
        ax_2d.plot(points[:, 0], points[:, 1], color=color, label=name)
        ax_2d.plot(row["center_x_m"], row["center_y_m"], "+", color=color)
        ax_2d.plot(points[[0, -1], 0], points[[0, -1], 1], "o", color=color, ms=3)

    low_2d = contact_2d["contact_points"]
    ax_2d.scatter(low_2d[:, 0], low_2d[:, 1], s=18, c="red", zorder=5, label="lowest region")
    ax_2d.set(
        xlabel="legacy leg-plane x [m]",
        ylabel="legacy leg-plane y (height) [m]",
        title="All named 2D rim arcs (centre='+', endpoints='o')",
    )
    ax_2d.axis("equal")
    ax_2d.grid(True, alpha=0.3)
    ax_2d.legend(fontsize=8, loc="best")

    records_3d = contact_3d["surface_records"]
    for name, color in SURFACE_COLORS.items():
        samples = [r for r in records_3d if r["surface_name"] == name]
        points = np.array([[r["x_m"], r["y_m"], r["z_m"]] for r in samples])
        ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], s=1.2, alpha=0.22, color=color)
    low_3d = contact_3d["contact_points"]
    ax_3d.scatter(low_3d[:, 0], low_3d[:, 1], low_3d[:, 2], s=18, c="red", label="lowest region")
    ax_3d.set(
        xlabel="x [m]",
        ylabel="lateral y [m]",
        zlabel="height z [m]",
        title="Existing sampled 3D rim surfaces (display-like local frame)",
    )
    ax_3d.legend(fontsize=8)
    fig.suptitle(
        "Day 3--5 Step 0: existing geometry is available independently of the lowest-point result",
        fontsize=12,
    )
    fig.tight_layout()
    return fig


__all__ = [
    "build_leg_geometry_inventory",
    "geometry_capability_rows",
    "plot_leg_geometry_inventory",
    "rim_classification_rows",
]
