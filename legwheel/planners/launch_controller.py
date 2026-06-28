"""
Launch controller for smooth gait initiation from rest.

Generates a linear velocity ramp sequence (N_ramp cycles) that precedes the
steady-state gait. Each ramp cycle is phase-shifted to begin at the widest
all-stance window of the gait, ensuring all four feet are on the ground at
the moment of first leg lift-off.

Sequence produced by generate_launch_sequence():
    ramp cycle 0 : v = v_target × ramp_floor           (e.g. 10%)
    ramp cycle 1 : v = v_target × (ramp_floor + step)
    ...
    ramp cycle k : v = v_target × linspace(ramp_floor, 1.0, N_ramp)[k]

The caller is responsible for prepending the hardware prep sequence and
appending the steady-state gait (see generate_hardware_csv.py).
"""

import numpy as np
from legwheel.planners.gait_generator_3d import GaitGenerator3D, GAIT_LIBRARY


def find_all_stance_phase(phase_offsets: list, stance_duty: float,
                          n_samples: int = 2000) -> float:
    """
    Find the normalized cycle time where all legs are simultaneously in stance.

    Scans one full cycle and returns the start of the widest all-stance window.
    If no such window exists (e.g. Walk at duty=0.75), falls back to the start
    of the window where the most legs are simultaneously grounded.

    Args:
        phase_offsets (list): Per-leg phase offsets from GAIT_LIBRARY.
        stance_duty (float): Fraction of cycle each leg spends in stance.
        n_samples (int): Temporal resolution for scanning.

    Returns:
        float: Normalized time in [0, 1) of the best all-stance window start.
    """
    t = np.linspace(0.0, 1.0, n_samples, endpoint=False)
    n_on_ground = np.zeros(n_samples, dtype=int)
    for phi in phase_offsets:
        in_stance = ((t - phi) % 1.0) < stance_duty
        n_on_ground += in_stance.astype(int)

    n_legs = len(phase_offsets)
    target = n_on_ground.max()   # 4 if all-stance exists, else 3 etc.

    mask = (n_on_ground == target).astype(int)
    # Detect rising edges (transitions into the window)
    padded = np.concatenate([[mask[-1]], mask])   # wrap-around for circular search
    rising = np.where(np.diff(padded) == 1)[0] % n_samples

    if len(rising) == 0:
        return 0.0

    # Find widest window
    best_start = rising[0]
    best_width = 0
    for start in rising:
        # Walk the window forward
        width = 0
        while mask[(start + width) % n_samples] == 1:
            width += 1
            if width >= n_samples:
                break
        if width > best_width:
            best_width = width
            best_start = start

    phase = t[best_start]

    if target < n_legs:
        print(f"  ⚠ Launch: gait has no all-4-stance window "
              f"(max {target}/{n_legs} legs). "
              f"Starting at max-ground-contact phase {phase:.3f}.")
    return float(phase)


class LaunchController:
    """
    Generates a velocity-ramp launch sequence for a given gait.

    Args:
        gait_type (str): One of the keys in GAIT_LIBRARY.
        stand_height (float): Body height above ground (m).
        twist (array-like): Target body twist [omega_z, v_x, v_y] (rad/s, m/s, m/s).
        step_height (float): Swing clearance (m).
        period (float): Gait cycle duration (s).
        dt (float): Time step (s).
        n_ramp (int): Number of ramp cycles (each one gait period long).
        ramp_floor (float): Velocity fraction for the first ramp cycle (0–1).
                            Default 0.1 (10% of target velocity).
    """

    def __init__(
        self,
        gait_type: str,
        stand_height: float,
        twist,
        step_height: float = 0.04,
        period: float = 1.0,
        dt: float = 0.001,
        n_ramp: int = 3,
        ramp_floor: float = 0.1,
        stability_margin: float = 0.02,
    ):
        if gait_type not in GAIT_LIBRARY:
            raise ValueError(
                f"Unknown gait '{gait_type}'. Choose from {list(GAIT_LIBRARY.keys())}")

        self.gait_type = gait_type
        self.stand_height = stand_height
        self.twist = np.array(twist, dtype=float)
        self.step_height = step_height
        self.period = period
        self.dt = dt
        self.n_ramp = n_ramp
        self.ramp_floor = ramp_floor
        self.stability_margin = stability_margin

        gait_def = GAIT_LIBRARY[gait_type]
        self.phase_offsets = gait_def["phase_offsets"]
        self.stance_duty = gait_def["stance_duty"]

        # Find optimal launch start phase once at construction
        self.start_phase = find_all_stance_phase(
            self.phase_offsets, self.stance_duty)

    # ------------------------------------------------------------------

    def _scale_twist(self, scale: float) -> np.ndarray:
        """Scale the velocity components of the twist, preserving direction."""
        scaled = self.twist.copy()
        scaled[0] *= scale   # omega_z
        scaled[1] *= scale   # v_x
        scaled[2] *= scale   # v_y
        return scaled

    def _generate_one_cycle(self, scale: float) -> np.ndarray:
        """
        Generate one gait cycle at `scale × target velocity`, phase-shifted
        to the all-stance window.

        Returns:
            np.ndarray: (n_points, 12) joint commands in planner column order.
        """
        twist_k = self._scale_twist(scale)
        gen = GaitGenerator3D(
            stand_height=self.stand_height,
            twist=twist_k,
            step_height=self.step_height,
            period=self.period,
            gait_type=self.gait_type,
            dt=self.dt,
            stability_margin=self.stability_margin,
        )
        cmds = gen.generate_full_gait(n_cycles=1)   # (n_pts, 12)

        # Apply all-stance phase shift: roll so cycle starts at start_phase
        n_pts = len(cmds)
        shift = int(round(self.start_phase * n_pts)) % n_pts
        return np.roll(cmds, -shift, axis=0)

    def generate_launch_sequence(self) -> np.ndarray:
        """
        Generate the full ramp sequence: N_ramp cycles from ramp_floor to 1.0.

        Returns:
            np.ndarray: (N_ramp × n_points_per_cycle, 12) commands.
        """
        scales = np.linspace(self.ramp_floor, 1.0, self.n_ramp)

        print(f"  Launch: {self.n_ramp} ramp cycles, "
              f"v = {self.ramp_floor*100:.0f}% → 100% of target, "
              f"start_phase = {self.start_phase:.3f}")

        cycles = []
        for k, s in enumerate(scales):
            v_pct = s * 100
            print(f"    Ramp cycle {k+1}/{self.n_ramp}: {v_pct:.0f}% v_target")
            cycles.append(self._generate_one_cycle(s))

        return np.vstack(cycles)

    def print_summary(self) -> None:
        print("=== LaunchController Summary ===")
        print(f"  Gait          : {self.gait_type}")
        print(f"  Target twist  : ω_z={self.twist[0]:.3f} rad/s, "
              f"v_x={self.twist[1]:.3f} m/s, v_y={self.twist[2]:.3f} m/s")
        print(f"  Ramp cycles   : {self.n_ramp}")
        print(f"  Velocity range: {self.ramp_floor*100:.0f}% → 100%")
        print(f"  All-stance φ  : {self.start_phase:.4f} ({self.start_phase*self.period*1000:.1f} ms into cycle)")
        print(f"  Ramp duration : {self.n_ramp * self.period:.2f} s")
