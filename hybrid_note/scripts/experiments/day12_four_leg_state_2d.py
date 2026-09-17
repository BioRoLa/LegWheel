"""Day 12 Step 2: four-leg world-frame initialisation and terrain registration.

Plan §9.  This is where the single-leg 2D results get mounted on the real
four-leg robot.  It adds **no** geometry, **no** timing and **no** body
planning: it places four legs and one parameterised platform in the world
frame, asks the existing contact query what each leg is standing on, and says
whether the answer is left/right symmetric.

Where every number comes from
-----------------------------

Nothing here invents a robot dimension.  The leg mounting is read out of
:class:`CorgiLegKinematics` by transforming the sagittal leg-frame origin into
the body frame with the project's own matrices, rather than by re-deriving it
from ``WHEEL_BASE`` and ``BODY_WIDTH``:

============  =========================================
leg           sagittal plane origin in ``{B}``  [m]
============  =========================================
LF (index 0)  ``(+0.255, +0.211675, +0.057166)``
RF (index 1)  ``(+0.255, -0.211675, +0.057166)``
RH (index 2)  ``(-0.255, -0.211675, +0.057166)``
LH (index 3)  ``(-0.255, +0.211675, +0.057166)``
============  =========================================

The lateral value is ``BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET``: the sagittal plane
sits at the **wheel mid-plane**, not at the ABAD axis.  The vertical value is
``ABAD_AXIS_OFFSET``, so the leg planes hang from a point 57.166 mm *above* the
body origin.

The check that the 2D and 3D models are talking about the same leg
------------------------------------------------------------------

The 2D pipeline's "hip" and the 3D model's leg-plane origin are asserted to be
the same point, so it is worth measuring rather than assuming.  At
``theta = 60 deg``, ``beta = 0``, ``gamma = 0``:

* :meth:`CorgiLegKinematics.forward_kinematics` puts the foot 219.449 mm below
  the leg-plane origin;
* Step 1's :meth:`NominalPosture2D.hip_z_for_flat_stance` puts the hip 219.449
  mm above flat ground.

They agree to six decimals, which is what licenses mounting the 2D sagittal
results on the 3D frame at all.  :func:`sagittal_reach_agreement_2d` recomputes
it so the agreement is a test rather than a paragraph.

Terrain
-------

The platform is :class:`SharedTerrainSpec2D`, the parameterised rectangle Day
10--11 already uses: ``height_m``, ``top_length_m``, ``x_start_m``,
``ground_height_m``.  No experimental size appears anywhere in this module --
plan §0.1 -- and :class:`FlatRunExtent2D` carries ``flat_before_m`` /
``flat_after_m`` as explicit inputs because ``TerrainProfile2D``'s ground is
unbounded and would otherwise leave "how much flat ground is there" unstated.

What this module deliberately does **not** do
---------------------------------------------

A level body standing on the lower ground is the initial condition, and a leg
whose hip has already passed the platform will **not** reach its surface.  That
is reported (``in_contact = False``, with the gap) rather than repaired: fixing
it means moving the body, and the body trajectory is Step 5's.  Step 2 saying
"this state is not a valid four-leg stance, here is why" is the honest output;
silently lowering a leg would manufacture a stance nobody planned.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.hybrid import InitialRobotState, RimId

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import NominalPosture2D
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    build_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    _candidate_for_sample,
    _lowest_contact_sample,
)

__all__ = [
    "LegId",
    "LegMount2D",
    "FlatRunExtent2D",
    "LegState2D",
    "FourLegState2D",
    "SymmetryCheck2D",
    "leg_mounts_2d",
    "sagittal_reach_agreement_2d",
    "initialize_four_leg_state_2d",
    "four_leg_rows",
    "plot_four_leg_state_2d",
]


class LegId(str, Enum):
    """The four legs, named as the Day 12 plan names them.

    The **values are the project's existing leg indices**, not a new ordering:
    ``CorgiLegKinematics`` documents ``0: FL, 1: FR, 2: RR, 3: RL``, so LF is
    FL and RH is RR.  Renaming without renumbering is what keeps
    ``joint_position_rad`` rows meaning the same leg they always did.
    """

    LF = "LF"
    RF = "RF"
    RH = "RH"
    LH = "LH"

    @property
    def index(self) -> int:
        return {"LF": 0, "RF": 1, "RH": 2, "LH": 3}[self.value]

    @property
    def is_left(self) -> bool:
        return self in (LegId.LF, LegId.LH)

    @property
    def is_front(self) -> bool:
        return self in (LegId.LF, LegId.RF)

    @property
    def mirror(self) -> "LegId":
        """The same leg on the other side -- what a symmetry check compares."""

        return {
            LegId.LF: LegId.RF, LegId.RF: LegId.LF,
            LegId.LH: LegId.RH, LegId.RH: LegId.LH,
        }[self]


#: The plan's order, front pair then hind pair.
LEG_ORDER: tuple[LegId, ...] = (LegId.LF, LegId.RF, LegId.LH, LegId.RH)


# --------------------------------------------------------------------------
# Mounting
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegMount2D:
    """Where one leg's sagittal plane hangs from the body."""

    leg: LegId
    offset_body_xyz_m: NDArray[np.float64]

    def __post_init__(self) -> None:
        value = np.asarray(self.offset_body_xyz_m, dtype=float).reshape(3)
        value.setflags(write=False)
        object.__setattr__(self, "offset_body_xyz_m", value)

    @property
    def sagittal_offset_xz_m(self) -> NDArray[np.float64]:
        """The part the 2D pipeline sees: ``x`` and ``z`` only."""

        return np.array(
            [self.offset_body_xyz_m[0], self.offset_body_xyz_m[2]], dtype=float
        )

    def as_dict(self) -> dict:
        x, y, z = (float(v) for v in self.offset_body_xyz_m)
        return {
            "leg": self.leg.value,
            "leg_index": self.leg.index,
            "is_left": self.leg.is_left,
            "is_front": self.leg.is_front,
            "offset_x_mm": x * 1e3,
            "offset_y_mm": y * 1e3,
            "offset_z_mm": z * 1e3,
        }


def leg_mounts_2d(gamma_rad: float = 0.0) -> tuple[LegMount2D, ...]:
    """Read the four mounting offsets out of the existing 3D kinematics.

    The sagittal leg-frame origin is the point ``p_L = 0`` pushed through the
    project's own ``{Li} -> {Mi} -> {B}`` transforms.  Deriving it here from
    ``WHEEL_BASE`` and ``BODY_WIDTH`` instead would be a second definition of
    the same geometry, and the lateral term would be easy to get wrong: it is
    ``BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET``, not ``BODY_WIDTH/2``.
    """

    if not np.isclose(gamma_rad, 0.0, atol=1e-12):
        raise ValueError("Day 12 fixes gamma = 0; Day 13-14 is what frees it.")
    mounts = []
    for leg in LEG_ORDER:
        kinematics = CorgiLegKinematics(leg.index, gamma=float(gamma_rad))
        leg_to_module, module_to_body = kinematics._get_transformation_matrices(
            float(gamma_rad), type="pos"
        )
        origin = module_to_body @ (leg_to_module @ np.array([0.0, 0.0, 0.0, 1.0]))
        mounts.append(LegMount2D(leg=leg, offset_body_xyz_m=origin[:3]))
    return tuple(mounts)


def sagittal_reach_agreement_2d(
    posture: NominalPosture2D | None = None, *, beta_rad: float = 0.0
) -> dict:
    """Compare the 3D model's hip-to-foot drop with the 2D stance height.

    They describe the same distance from two different code paths.  If they
    ever disagree, mounting the 2D trajectories on the four-leg frame is
    invalid, and this is the number that says so.
    """

    posture = NominalPosture2D() if posture is None else posture
    kinematics = CorgiLegKinematics(LegId.LF.index, gamma=0.0)
    foot_body = kinematics.forward_kinematics(
        float(posture.theta_rad), float(beta_rad), 0.0
    )
    mount = leg_mounts_2d()[0]
    drop_3d = float(mount.offset_body_xyz_m[2] - foot_body[2])
    stance_2d = float(
        posture.hip_z_for_flat_stance(beta_rad) - posture.ground_height_m
    )
    return {
        "theta_deg": float(np.rad2deg(posture.theta_rad)),
        "beta_deg": float(np.rad2deg(beta_rad)),
        "hip_to_foot_drop_3d_mm": drop_3d * 1e3,
        "hip_stance_height_2d_mm": stance_2d * 1e3,
        "difference_mm": (drop_3d - stance_2d) * 1e3,
    }


# --------------------------------------------------------------------------
# Terrain extent
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class FlatRunExtent2D:
    """How much flat ground the plan intends either side of the platform.

    ``TerrainProfile2D``'s ground is unbounded, so these are **not** geometry --
    they are the plan's statement of where the nominal cycles happen, which
    plan §4 lists as planner inputs alongside the platform parameters.  Kept as
    a separate record so nobody reads them as a terrain edge.
    """

    flat_before_m: float = 0.60
    flat_after_m: float = 0.60

    def __post_init__(self) -> None:
        for name in ("flat_before_m", "flat_after_m"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
            object.__setattr__(self, name, value)

    def x_limits_m(self, terrain: SharedTerrainSpec2D) -> tuple[float, float]:
        return (
            float(terrain.x_start_m - self.flat_before_m),
            float(terrain.x_max_m + self.flat_after_m),
        )


# --------------------------------------------------------------------------
# State
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegState2D:
    """One leg placed in the world, with whatever it is actually touching."""

    leg: LegId
    hip_pose_world_xyz_m: NDArray[np.float64]
    theta_rad: float
    beta_rad: float
    gamma_rad: float
    in_contact: bool
    #: ``None`` when the leg is not touching anything.
    contact_point_world_xyz_m: NDArray[np.float64] | None
    surface_id: str | None
    rim: str | None
    alpha_rad: float | None
    #: Signed gap from the lowest leg point to the terrain: negative means the
    #: leg would be inside it.
    surface_gap_m: float
    collision: bool

    def __post_init__(self) -> None:
        hip = np.asarray(self.hip_pose_world_xyz_m, dtype=float).reshape(3)
        hip.setflags(write=False)
        object.__setattr__(self, "hip_pose_world_xyz_m", hip)
        if self.contact_point_world_xyz_m is not None:
            point = np.asarray(self.contact_point_world_xyz_m, dtype=float).reshape(3)
            point.setflags(write=False)
            object.__setattr__(self, "contact_point_world_xyz_m", point)
        if not np.isclose(self.gamma_rad, 0.0, atol=1e-12):
            raise ValueError("Day 12 fixes gamma = 0.")

    @property
    def joint_row_rad(self) -> NDArray[np.float64]:
        """``(theta, beta, gamma)`` -- one row of the Day 1 contract."""

        return np.array(
            [self.theta_rad, self.beta_rad, self.gamma_rad], dtype=float
        )

    def as_dict(self) -> dict:
        hx, hy, hz = (float(v) for v in self.hip_pose_world_xyz_m)
        point = self.contact_point_world_xyz_m
        return {
            "leg": self.leg.value,
            "leg_index": self.leg.index,
            "hip_x_mm": hx * 1e3, "hip_y_mm": hy * 1e3, "hip_z_mm": hz * 1e3,
            "theta_deg": float(np.rad2deg(self.theta_rad)),
            "beta_deg": float(np.rad2deg(self.beta_rad)),
            "gamma_deg": float(np.rad2deg(self.gamma_rad)),
            "in_contact": self.in_contact,
            "contact_x_mm": None if point is None else float(point[0]) * 1e3,
            "contact_y_mm": None if point is None else float(point[1]) * 1e3,
            "contact_z_mm": None if point is None else float(point[2]) * 1e3,
            "surface_id": self.surface_id,
            "rim": self.rim,
            "alpha_deg": (
                None if self.alpha_rad is None
                else float(np.rad2deg(self.alpha_rad))
            ),
            "surface_gap_mm": self.surface_gap_m * 1e3,
            "collision": self.collision,
        }


@dataclass(frozen=True)
class SymmetryCheck2D:
    """One left/right comparison, with the quantity that was compared."""

    quantity: str
    left_leg: LegId
    right_leg: LegId
    left_value: float | None
    right_value: float | None
    difference: float | None
    tolerance: float
    symmetric: bool
    note: str = ""

    def as_dict(self) -> dict:
        return {
            "quantity": self.quantity,
            "left_leg": self.left_leg.value,
            "right_leg": self.right_leg.value,
            "left_value": self.left_value,
            "right_value": self.right_value,
            "difference": self.difference,
            "tolerance": self.tolerance,
            "symmetric": self.symmetric,
            "note": self.note,
        }


@dataclass(frozen=True)
class FourLegState2D:
    """The whole-body initial state: body pose, four legs, one platform."""

    body_position_world_m: NDArray[np.float64]
    body_rpy_rad: NDArray[np.float64]
    terrain: SharedTerrainSpec2D
    extent: FlatRunExtent2D
    legs: tuple[LegState2D, ...]

    def __post_init__(self) -> None:
        for name in ("body_position_world_m", "body_rpy_rad"):
            value = np.asarray(getattr(self, name), dtype=float).reshape(3)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if not np.allclose(self.body_rpy_rad, 0.0, atol=1e-12):
            raise ValueError(
                "Day 12's first symmetric test keeps body roll/pitch/yaw at "
                "zero; a non-level body is not this step's to introduce."
            )
        legs = tuple(self.legs)
        if len(legs) != 4 or {l.leg for l in legs} != set(LEG_ORDER):
            raise ValueError("a four-leg state has exactly LF, RF, LH and RH.")
        object.__setattr__(self, "legs", legs)

    def leg(self, leg: LegId) -> LegState2D:
        return next(state for state in self.legs if state.leg is leg)

    @property
    def all_in_contact(self) -> bool:
        return all(state.in_contact for state in self.legs)

    @property
    def joint_position_rad(self) -> NDArray[np.float64]:
        """``(4, 3)`` in the **project's** leg index order, not the plan's.

        ``LEG_ORDER`` is front-pair-then-hind-pair for reading; the Day 1
        contract's rows are indexed 0..3 as ``CorgiLegKinematics`` numbers them.
        Emitting the reading order here would silently permute two legs.
        """

        rows = np.zeros((4, 3), dtype=float)
        for state in self.legs:
            rows[state.leg.index] = state.joint_row_rad
        return rows

    def as_initial_robot_state(self) -> InitialRobotState:
        """The Day 1 world-frame contract, so Step 3 has a typed handover."""

        return InitialRobotState(
            body_position_world_m=self.body_position_world_m,
            body_rpy_rad=self.body_rpy_rad,
            joint_position_rad=self.joint_position_rad,
        )

    # -- symmetry ----------------------------------------------------------

    def symmetry_checks(self, *, tolerance_m: float = 1e-9) -> list[SymmetryCheck2D]:
        """Left/right sanity checks for the symmetric first test.

        A left leg and its mirror face the same longitudinal terrain, so every
        quantity below must match **except** ``y``, which must be equal and
        opposite.  Comparing ``y`` for equality instead of for opposition is
        the mistake this list exists to make impossible.
        """

        checks: list[SymmetryCheck2D] = []
        for left in (LegId.LF, LegId.LH):
            right = left.mirror
            l, r = self.leg(left), self.leg(right)

            checks.append(SymmetryCheck2D(
                "hip_x_m", left, right,
                float(l.hip_pose_world_xyz_m[0]), float(r.hip_pose_world_xyz_m[0]),
                float(l.hip_pose_world_xyz_m[0] - r.hip_pose_world_xyz_m[0]),
                tolerance_m,
                abs(l.hip_pose_world_xyz_m[0] - r.hip_pose_world_xyz_m[0]) <= tolerance_m,
            ))
            checks.append(SymmetryCheck2D(
                "hip_z_m", left, right,
                float(l.hip_pose_world_xyz_m[2]), float(r.hip_pose_world_xyz_m[2]),
                float(l.hip_pose_world_xyz_m[2] - r.hip_pose_world_xyz_m[2]),
                tolerance_m,
                abs(l.hip_pose_world_xyz_m[2] - r.hip_pose_world_xyz_m[2]) <= tolerance_m,
            ))
            mirrored = float(
                l.hip_pose_world_xyz_m[1] + r.hip_pose_world_xyz_m[1]
            )
            checks.append(SymmetryCheck2D(
                "hip_y_m (mirrored)", left, right,
                float(l.hip_pose_world_xyz_m[1]), float(r.hip_pose_world_xyz_m[1]),
                mirrored, tolerance_m, abs(mirrored) <= tolerance_m,
                note="left and right y must sum to zero, not be equal",
            ))
            checks.append(SymmetryCheck2D(
                "in_contact", left, right,
                float(l.in_contact), float(r.in_contact),
                float(l.in_contact) - float(r.in_contact), 0.0,
                l.in_contact == r.in_contact,
            ))
            checks.append(SymmetryCheck2D(
                "surface_id", left, right, None, None, None, 0.0,
                l.surface_id == r.surface_id,
                note=f"{l.surface_id} vs {r.surface_id}",
            ))
            checks.append(SymmetryCheck2D(
                "surface_gap_m", left, right,
                l.surface_gap_m, r.surface_gap_m,
                float(l.surface_gap_m - r.surface_gap_m), tolerance_m,
                abs(l.surface_gap_m - r.surface_gap_m) <= tolerance_m,
            ))
            if l.contact_point_world_xyz_m is not None and r.contact_point_world_xyz_m is not None:
                checks.append(SymmetryCheck2D(
                    "contact_x_m", left, right,
                    float(l.contact_point_world_xyz_m[0]),
                    float(r.contact_point_world_xyz_m[0]),
                    float(l.contact_point_world_xyz_m[0]
                          - r.contact_point_world_xyz_m[0]),
                    tolerance_m,
                    abs(l.contact_point_world_xyz_m[0]
                        - r.contact_point_world_xyz_m[0]) <= tolerance_m,
                ))
        return checks

    @property
    def is_symmetric(self) -> bool:
        return all(check.symmetric for check in self.symmetry_checks())

    def rows(self) -> list[dict]:
        return [state.as_dict() for state in self.legs]


# --------------------------------------------------------------------------
# Initialisation
# --------------------------------------------------------------------------


def initialize_four_leg_state_2d(
    terrain: SharedTerrainSpec2D,
    *,
    posture: NominalPosture2D | None = None,
    extent: FlatRunExtent2D | None = None,
    body_x_m: float | None = None,
    beta_rad: float = 0.0,
    body_y_m: float = 0.0,
) -> FourLegState2D:
    """Place a level robot on the lower ground and ask what each leg touches.

    ``body_x_m`` defaults to standing entirely on the flat run before the
    platform: the **front** hip is put one ``flat_before`` fraction back from
    the leading edge, so the default state is the one the Day 12 story starts
    from.  It is a parameter, and nothing in this function branches on the
    platform's size.

    The body height is chosen so a leg on the **lower ground** stands at the
    nominal posture.  A leg whose hip has already passed the leading edge is
    then too high for the platform, and that is reported rather than fixed --
    see the module docstring.
    """

    posture = NominalPosture2D() if posture is None else posture
    extent = FlatRunExtent2D() if extent is None else extent
    if not np.isclose(posture.ground_height_m, terrain.ground_height_m, atol=1e-12):
        raise ValueError(
            "the posture and the terrain disagree about where the ground is "
            f"({posture.ground_height_m} vs {terrain.ground_height_m})."
        )

    mounts = {mount.leg: mount for mount in leg_mounts_2d()}
    front_offset_x = float(mounts[LegId.LF].offset_body_xyz_m[0])
    if body_x_m is None:
        body_x_m = float(
            terrain.x_start_m - extent.flat_before_m - front_offset_x
        )

    # The hip that stands at the nominal posture is one on the lower ground;
    # the body sits that far up minus the mount's own height above the body
    # origin.
    hip_stance_z = posture.hip_z_for_flat_stance(beta_rad)
    mount_z = float(mounts[LegId.LF].offset_body_xyz_m[2])
    body_z_m = float(hip_stance_z - mount_z)
    body = np.array([float(body_x_m), float(body_y_m), body_z_m], dtype=float)

    scene_kwargs = dict(terrain.surface_scene_kwargs)
    scene_kwargs["arc_samples"] = posture.arc_samples

    legs = []
    for leg in LEG_ORDER:
        mount = mounts[leg]
        hip = body + mount.offset_body_xyz_m
        scene = build_single_leg_rolling_scene_2d(
            posture.theta_rad, float(beta_rad),
            float(hip[0]), float(hip[2]), **scene_kwargs
        )
        result = query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=posture.contact_tolerance_m,
            collision_tolerance_m=posture.collision_tolerance_m,
        )
        sample = _lowest_contact_sample(scene.geometry)
        candidate = _candidate_for_sample(result, sample)
        gap = _surface_gap_m(scene, terrain)

        point = None
        if candidate is not None:
            point = np.array(
                [float(candidate.point_world_xz_m[0]), float(hip[1]),
                 float(candidate.point_world_xz_m[1])], dtype=float
            )
        legs.append(LegState2D(
            leg=leg,
            hip_pose_world_xyz_m=hip,
            theta_rad=float(posture.theta_rad),
            beta_rad=float(beta_rad),
            gamma_rad=0.0,
            in_contact=candidate is not None,
            contact_point_world_xyz_m=point,
            surface_id=None if candidate is None else str(candidate.terrain_surface_id),
            rim=None if candidate is None else RimId(candidate.rim).value,
            alpha_rad=None if candidate is None else float(candidate.alpha_rad),
            surface_gap_m=gap,
            collision=bool(result.collision),
        ))

    return FourLegState2D(
        body_position_world_m=body,
        body_rpy_rad=np.zeros(3),
        terrain=terrain,
        extent=extent,
        legs=tuple(legs),
    )


def _surface_gap_m(scene, terrain: SharedTerrainSpec2D) -> float:
    """Signed gap from the lowest leg point to whichever surface is under it.

    Negative means the leg is inside the terrain.  Computed against the
    platform top where the leg is over the platform and against the ground
    elsewhere, so a leg standing beside the platform is not reported as
    floating by the platform's height.
    """

    points = np.asarray(scene.geometry.points_world_xz_m, dtype=float)
    lowest = int(np.argmin(points[:, 1]))
    x, z = float(points[lowest, 0]), float(points[lowest, 1])
    over_platform = terrain.x_start_m <= x <= terrain.x_max_m
    surface_z = terrain.top_z_m if over_platform else terrain.ground_height_m
    return float(z - surface_z)


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def four_leg_rows(state: FourLegState2D) -> list[dict]:
    """One table: the body, the platform, and the four legs."""

    bx, by, bz = (float(v) for v in state.body_position_world_m)
    rows = [{
        "row_kind": "body",
        "leg": None, "leg_index": None,
        "body_x_mm": bx * 1e3, "body_y_mm": by * 1e3, "body_z_mm": bz * 1e3,
        "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
    }, {
        "row_kind": "terrain",
        "platform_height_mm": state.terrain.height_m * 1e3,
        "platform_top_length_mm": state.terrain.top_length_m * 1e3,
        "platform_x_start_mm": state.terrain.x_start_m * 1e3,
        "platform_x_end_mm": state.terrain.x_max_m * 1e3,
        "platform_top_z_mm": state.terrain.top_z_m * 1e3,
        "ground_height_mm": state.terrain.ground_height_m * 1e3,
        "flat_before_mm": state.extent.flat_before_m * 1e3,
        "flat_after_mm": state.extent.flat_after_m * 1e3,
    }]
    rows += [{"row_kind": "leg", **row} for row in state.rows()]
    rows += [
        {"row_kind": "symmetry", **check.as_dict()}
        for check in state.symmetry_checks()
    ]
    # Day 10--11 trap 21: ``write_rows_csv`` takes its header from the *first*
    # row, and these four row kinds carry different fields.  The union is
    # filled in here, where the table is built, rather than at every call site.
    fields: list[str] = []
    for row in rows:
        fields += [key for key in row if key not in fields]
    return [{key: row.get(key) for key in fields} for row in rows]


def plot_four_leg_state_2d(state: FourLegState2D, path=None, *, ax=None):
    """Body, four hips, four contact points and the platform, in one figure.

    Two views, because one is not enough to catch a mapping error: the sagittal
    view shows the platform and the standing height, and the top view shows
    left/right and front/hind, which is where an index permutation would show
    up as a mirrored robot.
    """

    import matplotlib.pyplot as plt

    created = ax is None
    if created:
        figure, axes = plt.subplots(
            2, 1, figsize=(11.0, 7.5),
            gridspec_kw={"height_ratios": [1.35, 1.0]},
        )
    else:
        figure, axes = ax.figure, np.atleast_1d(ax)

    terrain, extent = state.terrain, state.extent
    bx, by, bz = (float(v) for v in state.body_position_world_m)
    # The intended flat run, widened to whatever the robot actually occupies:
    # the first version used the terrain extent alone and drew the hind legs
    # off the edge of the figure.
    x_values = list(extent.x_limits_m(terrain)) + [
        float(s.hip_pose_world_xyz_m[0]) for s in state.legs
    ] + [
        float(s.contact_point_world_xyz_m[0]) for s in state.legs
        if s.contact_point_world_xyz_m is not None
    ]
    pad = 0.08
    x_lo, x_hi = min(x_values) - pad, max(x_values) + pad
    colors = {LegId.LF: "#2a6f4e", LegId.RF: "#1a4d8f",
              LegId.LH: "#b06000", LegId.RH: "#c5221f"}

    # -- sagittal ----------------------------------------------------------
    side = axes[0]
    side.plot([x_lo, terrain.x_start_m], [terrain.ground_height_m] * 2,
              color="#444", lw=2.0, label="ground")
    side.plot([terrain.x_max_m, x_hi], [terrain.ground_height_m] * 2,
              color="#444", lw=2.0)
    side.plot(
        [terrain.x_start_m, terrain.x_start_m, terrain.x_max_m, terrain.x_max_m],
        [terrain.ground_height_m, terrain.top_z_m, terrain.top_z_m,
         terrain.ground_height_m],
        color="#444", lw=2.0,
    )
    side.fill_between(
        [terrain.x_start_m, terrain.x_max_m],
        terrain.ground_height_m, terrain.top_z_m, color="#444", alpha=0.12,
    )

    hips = [state.leg(leg) for leg in LEG_ORDER]
    front_x = [s.hip_pose_world_xyz_m[0] for s in hips if s.leg.is_front]
    hind_x = [s.hip_pose_world_xyz_m[0] for s in hips if not s.leg.is_front]
    side.plot([min(hind_x), max(front_x)], [hips[0].hip_pose_world_xyz_m[2]] * 2,
              color="#111", lw=3.0, solid_capstyle="round", label="body (hip line)")
    side.plot([bx], [bz], marker="s", ms=8, color="#111", label="body origin")

    for s in hips:
        hx, _, hz = (float(v) for v in s.hip_pose_world_xyz_m)
        # Leg names are annotated rather than put in the legend: with an equal
        # aspect over a metre of x there is almost no vertical room, and a
        # six-entry legend sits exactly on top of the hips it describes.
        side.plot([hx], [hz], marker="o", ms=9, color=colors[s.leg])
        side.annotate(s.leg.value, xy=(hx, hz), xytext=(0, 10),
                      textcoords="offset points", ha="center", fontsize=8,
                      color=colors[s.leg])
        if s.contact_point_world_xyz_m is not None:
            cx, _, cz = (float(v) for v in s.contact_point_world_xyz_m)
            side.plot([hx, cx], [hz, cz], color=colors[s.leg], lw=1.2, ls="--",
                      alpha=0.8)
            side.plot([cx], [cz], marker="v", ms=9, color=colors[s.leg])
        else:
            side.annotate(
                f"{s.leg.value}\nno contact\ngap {s.surface_gap_m*1e3:+.1f} mm",
                xy=(hx, hz), xytext=(0, -34), textcoords="offset points",
                ha="center", fontsize=7.5, color=colors[s.leg],
            )
    side.set_xlim(x_lo, x_hi)
    side.set_aspect("equal", adjustable="box")
    side.set_ylabel("z  [m]")
    side.grid(alpha=0.25)
    side.legend(fontsize=7.5, ncol=3, loc="lower right")
    side.set_title(
        f"sagittal view -- platform {terrain.height_m*1e3:.0f} mm high x "
        f"{terrain.top_length_m*1e3:.0f} mm long, registered from parameters",
        fontsize=9,
    )

    # -- top ---------------------------------------------------------------
    top = axes[1]
    top.axvspan(terrain.x_start_m, terrain.x_max_m, color="#444", alpha=0.12)
    for x in (terrain.x_start_m, terrain.x_max_m):
        top.axvline(x, color="#444", lw=1.5)
    xs = [float(s.hip_pose_world_xyz_m[0]) for s in hips]
    ys = [float(s.hip_pose_world_xyz_m[1]) for s in hips]
    order = [LegId.LF, LegId.RF, LegId.RH, LegId.LH]
    loop = [state.leg(l) for l in order] + [state.leg(order[0])]
    top.plot([float(s.hip_pose_world_xyz_m[0]) for s in loop],
             [float(s.hip_pose_world_xyz_m[1]) for s in loop],
             color="#111", lw=1.6, alpha=0.7)
    for s in hips:
        hx, hy, _ = (float(v) for v in s.hip_pose_world_xyz_m)
        top.plot([hx], [hy], marker="o", ms=9, color=colors[s.leg])
        top.annotate(s.leg.value, xy=(hx, hy), xytext=(0, 9),
                     textcoords="offset points", ha="center", fontsize=9,
                     color=colors[s.leg])
        if s.contact_point_world_xyz_m is not None:
            cx, cy, _ = (float(v) for v in s.contact_point_world_xyz_m)
            top.plot([cx], [cy], marker="v", ms=9, color=colors[s.leg])
    top.plot([bx], [by], marker="s", ms=8, color="#111")
    top.set_xlim(x_lo, x_hi)
    top.set_aspect("equal", adjustable="box")
    top.set_xlabel("x  [m]   (+x forward)")
    top.set_ylabel("y  [m]   (+y left)")
    top.grid(alpha=0.25)
    top.set_title(
        "top view -- LF/LH on +y, RF/RH on -y, front pair on +x", fontsize=9
    )

    if created:
        figure.tight_layout()
        if path is not None:
            figure.savefig(path, dpi=150)
            plt.close(figure)
    return figure
