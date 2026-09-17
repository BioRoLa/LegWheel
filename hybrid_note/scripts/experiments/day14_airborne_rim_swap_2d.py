"""B5: swap rims **in the air** on the obstacle top, instead of while touching.

Why this exists
---------------

The verified crossing rolls the top in wheel mode at ``theta = 17 deg``.  That
17 is not a style choice and not a legacy value -- sweeping it shows it is the
only value that works::

    theta_wheel   result
       17 deg     the only one that completes
       20-40 deg  RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET
       45 deg +   theta_target must not exceed the roll-up's final theta

The reason is in ``single_leg_rolling_scene_2d``: handing the contact from the
right rim to the left rim **while touching the top** requires the seam between
the two rims to be geometrically closed, and the generator keeps retracting
theta until it is.  The more extended the leg, the wider the seam.  So the
retraction is not "becoming a wheel" -- it is what makes the hand-over legal.

**But the seam only constrains a hand-over that happens in contact.**  Swap the
rims in the air and the constraint does not apply: the leg lifts off, retracts,
carries the rotation forward, extends, and lands on the other rim.  That is the
same shape as the nominal recovery swing, which measures::

    lift off   foot_rim   theta 72.49 deg
    airborne   retract to 17.00 deg, carry 280.3 deg of forward rotation
    land       foot_rim   theta 72.49 deg

The only difference is landing on a *different* rim, and
``run_recovery_swing_2d`` already takes a ``beta_target_rad``.

Clearance was the thing that could have killed it, so it was measured before
any of this was written -- over the ``RECOVERY_ROTATE`` phase, which is where
the requirement applies (the frames either side of contact are zero by
construction)::

    standing on        rotation-phase clearance    required
    flat ground              74.448 mm              10 mm
    40 mm top                74.448 mm              10 mm
    100 mm top               74.448 mm              10 mm

Identical at every height, because the clearance depends on the hip's height
above *its own* standing surface, and raising the whole scene does not change
that.

What this module does and does not do
-------------------------------------

It produces a :class:`WheelModeTransitionResult2D` -- the **same type** the
in-contact transition produces -- so ``run_left_rim_roll_down_2d`` consumes it
unchanged.  The descent only reads ``left_rim_ready``, ``final_frame`` and
``trailing_corner_world_xz_m``, which is a narrow enough interface that the
swap can be substituted without touching the descent at all.

It does **not** decide *whether* to swap in the air.  That is a strategy
choice, and it belongs with the top-length reasoning in Day 10--11's decision
map, not here.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = [
    "AirborneRimSwap2D",
    "SWAP_PHASE",
]

#: The phase label the swap's frames carry.  Distinct from the nominal
#: ``RECOVERY_*`` labels so a reader of a chain can tell a crossing's rim swap
#: from a gait recovery, even though the motion is the same shape.
SWAP_PHASE = "TOP_AIRBORNE_RIM_SWAP"

#: Where the clearance requirement applies.  Not every airborne frame: the ones
#: either side of contact are zero by construction, and minimising over them
#: reports 0.000 mm for a swing that in fact clears by 74 mm.
CLEARANCE_PHASE = "RECOVERY_ROTATE"


@dataclass(frozen=True)
class AirborneRimSwap2D:
    """One rim hand-over performed in the air, on top of the obstacle."""

    frames: tuple
    from_rim: str
    to_rim: str
    success: bool
    failure_reason: str | None
    #: Smallest clearance over :data:`CLEARANCE_PHASE`, in metres.
    rotation_clearance_m: float | None
    #: How far the leg turned, unwrapped.
    rotation_rad: float | None

    @property
    def final_frame(self):
        return self.frames[-1] if self.frames else None

    def as_dict(self) -> dict:
        return {
            "from_rim": self.from_rim,
            "to_rim": self.to_rim,
            "frames": len(self.frames),
            "success": self.success,
            "failure_reason": self.failure_reason,
            "rotation_clearance_mm": (
                None if self.rotation_clearance_m is None
                else self.rotation_clearance_m * 1e3),
            "rotation_deg": (None if self.rotation_rad is None
                             else float(np.rad2deg(self.rotation_rad))),
        }


def rotation_clearance_m(frames) -> float | None:
    """Smallest clearance over the rotation phase; ``None`` when there is none.

    Deliberately not ``min`` over every airborne frame -- see
    :data:`CLEARANCE_PHASE`.
    """

    values = [f.clearance_m for f in frames
              if getattr(f, "phase", None) == CLEARANCE_PHASE]
    return min(values) if values else None
