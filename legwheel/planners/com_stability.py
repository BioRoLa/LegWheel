"""
com_stability.py
================

COM Stability Analyser and Corrector for Quadruped Gait
--------------------------------------------------------

The standard GaitGenerator3D plans trajectories from **kinematic** constraints
(joint-angle limits, workspace bounds) but completely ignores **static stability**:
whether the robot's projected Centre-of-Mass (COM) lies inside the convex
support polygon formed by all currently-grounded feet.

This is especially critical during **lateral motion** (v_y ≠ 0):

  * **Trot**  – two diagonal feet in stance → narrow diagonal support strip.
    The COM sits exactly on this strip when v_y = 0; any lateral shift of the
    body (or asymmetric foot placement) pushes it outside.

  * **Pace**  – two same-side feet in stance → support is a single lateral
    line on one side of the robot.  The COM (~body centre) is ≈ W/2 away
    from that line → inherently statically unstable without body sway.

  * **Walk**  – three feet in stance → triangular support.  Usually adequate,
    but large v_y can push the triangle's centroid away from the COM.

Solution
--------
``COMStabilityPlanner`` is a **post-processor** that wraps a
``GaitGenerator3D`` instance:

1. For every time step compute the XY support polygon from stance-foot FK.
2. Compute the *signed stability margin* – how far (m) the COM is **inside**
   the polygon (negative → outside → tipping risk).
3. Determine the minimum lateral body-sway ``delta_y`` (m) needed to restore
   the desired safety margin.
4. Gaussian-smooth the raw sway signal to eliminate sharp discontinuities.
5. Apply the sway by shifting every foot target by ``[0, −Δy, 0]`` in the
   body frame and re-solving inverse kinematics.

Usage
-----
::

    from legwheel.planners.gait_generator_3d import GaitGenerator3D
    from legwheel.planners.com_stability import COMStabilityPlanner

    gen = GaitGenerator3D(gait_type="Pace",
                          twist=[0.0, 0.0, 0.10],   # lateral only
                          stand_height=0.30)

    planner = COMStabilityPlanner(gen, safety_margin=0.02)
    cmds_stable = planner.generate_stable_gait(n_cycles=2)
    planner.plot_stability_analysis()
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from scipy.spatial import ConvexHull
from scipy.ndimage import gaussian_filter1d

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams


# ─────────────────────────────────────────────────────────────────────────────
#  Low-level geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _hull_signed_margin(
    com_xy: np.ndarray,
    stance_pts: np.ndarray,
) -> tuple[float, np.ndarray]:
    """
    Signed stability margin and correction direction for the given stance config.

    Parameters
    ----------
    com_xy : (2,) array
        Projected COM position in the body-frame XY plane.
    stance_pts : (n, 2) array
        XY positions of all currently-grounded feet.

    Returns
    -------
    margin : float
        Signed distance from COM to the nearest support-polygon boundary.
        * > 0  →  COM is **inside** the polygon (stable).
        * ≤ 0  →  COM is **outside** or on the boundary (unstable / marginal).
        For line (n=2) or point (n=1) support, margin ≤ 0 by definition;
        the best achievable value is 0 (COM exactly on the line/point).
    inward_dir : (2,) array
        Unit vector in the XY plane pointing from the COM **toward** the interior
        of the support region.  Used to determine the correction sway direction.
    """
    com = np.asarray(com_xy, dtype=float)
    pts = np.asarray(stance_pts, dtype=float)
    n = len(pts)

    # ── 0 stance legs ─────────────────────────────────────────────────────────
    if n == 0:
        return -np.inf, np.array([0.0, 1.0])

    # ── 1 stance leg (point support) ──────────────────────────────────────────
    if n == 1:
        vec = pts[0] - com
        dist = np.linalg.norm(vec)
        direction = vec / (dist + 1e-12)
        return -dist, direction

    # ── 2 stance legs (line support) ──────────────────────────────────────────
    if n == 2:
        seg = pts[1] - pts[0]
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-9:
            return _hull_signed_margin(com, pts[[0]])
        # Unit normal to the line (left of pts[0]→pts[1])
        normal = np.array([-seg[1], seg[0]]) / seg_len
        d = float(np.dot(com - pts[0], normal))
        # Best possible margin for line support is 0 (COM on the line).
        # Sign: negative of |d| so the closer to the line the better.
        inward = -np.sign(d) * normal if abs(d) > 1e-9 else normal
        return -abs(d), inward

    # ── 3+ stance legs (convex polygon) ───────────────────────────────────────
    try:
        hull = ConvexHull(pts)
    except Exception:
        # Collinear points – degenerate; find the two extremes and fall back
        dists = np.linalg.norm(pts[:, None] - pts[None, :], axis=-1)
        i, j = np.unravel_index(np.argmax(dists), dists.shape)
        return _hull_signed_margin(com, pts[[i, j]])

    # hull.equations rows: [a, b, c] with *outward* normal [a,b], offset c
    # Signed distance of a point p to facet k:  s_k = -(a·px + b·py + c)
    # s_k > 0  →  p is on the inside of facet k.
    # Overall stability margin = min(s_k) over all facets.
    s = -(hull.equations[:, :2] @ com + hull.equations[:, 2])
    k = int(np.argmin(s))
    margin = float(s[k])
    inward = -hull.equations[k, :2]           # inward normal at tightest facet
    inward /= np.linalg.norm(inward) + 1e-12
    return margin, inward


# ─────────────────────────────────────────────────────────────────────────────
#  Main class
# ─────────────────────────────────────────────────────────────────────────────

class COMStabilityPlanner:
    """
    COM Stability Post-Processor for GaitGenerator3D.

    Analyses whether the robot's projected COM lies within the support polygon
    at every timestep and corrects instabilities via a smooth lateral body sway,
    realised as per-leg IK adjustments.

    Parameters
    ----------
    gait_generator : GaitGenerator3D
        A fully-constructed generator instance (``__init__`` already called).
    com_bias_xy : array-like of shape (2,), optional
        [dx, dy] offset of the COM from the body-frame origin (m).
        Defaults to ``[RobotParams.COM_BIAS_X, RobotParams.COM_BIAS_Y]``.
    safety_margin : float
        Minimum desired signed margin (m) from COM to the support polygon
        boundary.  Frames with margin < safety_margin receive a sway correction.
        Use 0.0 to only enforce "COM inside polygon".
    smooth_sigma : float
        Gaussian smoothing kernel width for the sway signal, expressed as a
        **fraction of the gait period** (e.g. 0.05 → 5 % of T).
        Set to 0 to disable smoothing (sharp transitions).
    max_sway : float
        Maximum body sway amplitude (m). Corrections exceeding this value are
        clipped and a warning is printed.
    """

    _LEG_LABELS = ["FL", "FR", "RR", "RL"]
    _LEG_COLORS = ["royalblue", "tomato", "firebrick", "dodgerblue"]

    # ── Construction ──────────────────────────────────────────────────────────

    def __init__(
        self,
        gait_generator,
        com_bias_xy=None,
        safety_margin: float = 0.02,
        smooth_sigma: float = 0.05,
        max_sway: float = 0.15,
    ):
        self.gen = gait_generator
        self.safety_margin = float(safety_margin)
        self.smooth_sigma = float(smooth_sigma)
        self.max_sway = float(max_sway)

        if com_bias_xy is None:
            self.com_xy = np.array(
                [RobotParams.COM_BIAS_X, RobotParams.COM_BIAS_Y], dtype=float
            )
        else:
            self.com_xy = np.asarray(com_bias_xy, dtype=float)

        # Independent kinematic models for each leg (FK / IK)
        self.legs = [CorgiLegKinematics(i) for i in range(4)]

        # Populated after the first call to generate_stable_gait / analyze
        self._n_points: int | None = None
        self._last_info: dict | None = None
        self._last_cmds_raw: np.ndarray | None = None
        self._last_cmds_stable: np.ndarray | None = None
        self._last_sway_raw: np.ndarray | None = None
        self._last_sway: np.ndarray | None = None
        self._last_n_cycles: int = 1

    # ── Phase / stance helpers ─────────────────────────────────────────────────

    def stance_mask(self, frame_idx: int, n_points: int) -> np.ndarray:
        """
        Return a bool[4] indicating which legs are in stance at *frame_idx*.

        The base trajectory for each leg has:
          * indices [0,  n_stance)  → stance phase
          * indices [n_stance, n_points) → swing phase

        With phase offset ``phase_offsets[i]``, leg i at global frame f uses
        base index ``(f + shift_i) % n_points``.
        """
        n_stance = int(round(self.gen.stance_duty * n_points))
        mask = np.zeros(4, dtype=bool)
        for i in range(4):
            shift = int(self.gen.phase_offsets[i] * n_points)
            mask[i] = (frame_idx + shift) % n_points < n_stance
        return mask

    # ── FK ────────────────────────────────────────────────────────────────────

    def _foot_xy(self, cmd_row: np.ndarray) -> np.ndarray:
        """Return (4, 2) foot XY positions in the body frame for one cmd row."""
        xy = np.zeros((4, 2))
        for i in range(4):
            p = self.legs[i].forward_kinematics(*cmd_row[i * 3: i * 3 + 3])
            xy[i] = p[:2]
        return xy

    # ── Stability / sway computation ──────────────────────────────────────────

    def _compute_margin(
        self, foot_xy: np.ndarray, mask: np.ndarray
    ) -> tuple[float, np.ndarray, int]:
        """Stability margin + inward direction + stance count for a single frame."""
        n = int(mask.sum())
        margin, inward = _hull_signed_margin(self.com_xy, foot_xy[mask])
        return margin, inward, n

    def _required_sway_y(
        self,
        margin: float,
        inward_dir: np.ndarray,
        n_stance: int,
    ) -> float:
        """
        Lateral body sway Δy (m, +Y = body shifts left) needed to restore the
        stability margin.

        Target margin depends on the support geometry:

        * **≥3 stance legs (polygon)** – target = ``self.safety_margin``.
          A positive signed distance from the COM to the boundary is achievable.

        * **2 stance legs (line)** – target = 0.  A line has no interior, so
          the best attainable margin is 0 (COM exactly on the support line).
          Setting a positive target would push the COM *past* the support line
          to the opposite unstable side (over-correction).

        * **≤1 stance leg (point)** – same as line: target = 0.
        """
        target = self.safety_margin if n_stance >= 3 else 0.0
        deficit = target - margin          # > 0 → correction needed
        if deficit <= 0.0:
            return 0.0

        iny = float(inward_dir[1])
        if abs(iny) < 0.08:   # edge mostly along Y – X-violation; Y sway barely helps
            return float(np.clip(0.005 * np.sign(iny + 1e-10),
                                 -self.max_sway, self.max_sway))

        delta_y = deficit * iny          # project deficit onto Y axis
        return float(np.clip(delta_y, -self.max_sway, self.max_sway))

    # ── Sway application ──────────────────────────────────────────────────────

    def _apply_sway(self, cmd_row: np.ndarray, sway_y: float) -> np.ndarray:
        """
        Apply lateral body sway (Δy metres, +Y = body shifts left) by adjusting
        the γ (ABAD / hip-roll) joint angle of every leg.

        Physical rationale
        ------------------
        A body sway of +Δy shifts the body origin leftward while all feet stay
        fixed in the world frame.  In the body frame every foot's Y coordinate
        decreases by Δy.  The γ joint is exactly the DOF that controls lateral
        foot placement; adjusting it is therefore the minimal, numerically safe
        way to implement the sway without touching θ or β (which govern sagittal
        extension and height).

        Method
        ------
        For each leg, the required Δγ is found via the numerical partial
        derivative ∂foot_y / ∂γ at the current joint state:

            Δγ_i  =  −Δy  /  (∂foot_y_i / ∂γ_i)

        The result is clamped to [−GAMMA_MAX, +GAMMA_MAX] to prevent workspace
        violations.  The tiny height change (≈ H·(1−cos Δγ)) is accepted as a
        second-order effect; it is always < 5 mm for the sway values required
        in practice.
        """
        if abs(sway_y) < 5e-5:           # below 0.05 mm → skip
            return cmd_row.copy()

        GAMMA_MAX = np.deg2rad(RobotParams.GAMMA_MAX_DEG)
        EPS = 1e-5   # finite-difference step for ∂foot_y/∂γ

        corrected = cmd_row.copy()
        for i in range(4):
            q = cmd_row[i * 3: i * 3 + 3].copy()

            # Numerical ∂foot_y / ∂γ  (central difference)
            q_p, q_m = q.copy(), q.copy()
            q_p[2] += EPS
            q_m[2] -= EPS
            dfy_dg = (
                self.legs[i].forward_kinematics(*q_p)[1]
                - self.legs[i].forward_kinematics(*q_m)[1]
            ) / (2.0 * EPS)

            if abs(dfy_dg) > 1e-6:
                delta_gamma = -sway_y / dfy_dg
                q[2] = float(np.clip(q[2] + delta_gamma, -GAMMA_MAX, GAMMA_MAX))

            corrected[i * 3: i * 3 + 3] = q
        return corrected

    # ── Smoothing ─────────────────────────────────────────────────────────────

    def _smooth_sway(self, sway_raw: np.ndarray, n_points: int) -> np.ndarray:
        """
        Gaussian-smooth the raw sway signal.

        ``smooth_sigma`` is expressed as a fraction of the gait period, so the
        kernel width (in frames) = ``smooth_sigma × n_points``.
        ``mode='wrap'`` prevents edge artefacts for periodic signals.
        """
        if self.smooth_sigma <= 0.0:
            return sway_raw.copy()
        sigma_frames = max(1.0, self.smooth_sigma * n_points)
        return gaussian_filter1d(sway_raw, sigma=sigma_frames, mode="wrap")

    # ── Full analysis pipeline ─────────────────────────────────────────────────

    def analyze(
        self,
        cmds: np.ndarray,
        n_cycles: int = 1,
    ) -> dict:
        """
        Run a stability analysis on an existing (N, 12) command array.

        Returns a dict with:

        ========================  ================================================
        Key                       Description
        ========================  ================================================
        ``margins``               (N,) signed stability margin per frame (m)
        ``sway_required``         (N,) raw Y sway needed per frame (m)
        ``inward_dirs``           (N, 2) inward direction per frame
        ``foot_xy``               (N, 4, 2) foot XY in body frame per frame
        ``stance_counts``         (N,) number of grounded legs per frame
        ``n_points``              number of frames in one base cycle
        ========================  ================================================
        """
        N = len(cmds)
        n_points = N // n_cycles

        margins = np.zeros(N)
        sway_raw = np.zeros(N)
        inward_dirs = np.zeros((N, 2))
        foot_xy_all = np.zeros((N, 4, 2))
        stance_counts = np.zeros(N, dtype=int)

        for f in range(N):
            mask = self.stance_mask(f % n_points, n_points)
            fxy = self._foot_xy(cmds[f])
            margin, inward, n_st = self._compute_margin(fxy, mask)
            sway = self._required_sway_y(margin, inward, n_st)

            margins[f] = margin
            sway_raw[f] = sway
            inward_dirs[f] = inward
            foot_xy_all[f] = fxy
            stance_counts[f] = int(mask.sum())

        return {
            "margins": margins,
            "sway_required": sway_raw,
            "inward_dirs": inward_dirs,
            "foot_xy": foot_xy_all,
            "stance_counts": stance_counts,
            "n_points": n_points,
        }

    # ── Main public method ─────────────────────────────────────────────────────

    def generate_stable_gait(self, n_cycles: int = 2) -> np.ndarray:
        """
        Generate stability-corrected gait commands.

        Pipeline
        --------
        1. ``GaitGenerator3D.generate_full_gait()`` → raw (N, 12) commands
        2. Per-frame stability analysis → raw sway signal
        3. Gaussian smoothing of sway signal
        4. Per-frame IK correction applying the smoothed sway
        5. Return corrected (N, 12) command array

        Parameters
        ----------
        n_cycles : int
            Number of gait cycles to generate.

        Returns
        -------
        np.ndarray
            (N, 12) array of corrected joint commands  ``[θ, β, γ] × 4 legs``.
        """
        print("  [COMStability] Generating raw gait …")
        cmds_raw = self.gen.generate_full_gait(n_cycles=n_cycles)
        N = len(cmds_raw)
        n_points = N // n_cycles
        self._n_points = n_points

        print("  [COMStability] Analysing per-frame stability …")
        info = self.analyze(cmds_raw, n_cycles=n_cycles)

        sway_raw = info["sway_required"]
        sway = self._smooth_sway(sway_raw, n_points)

        # Warn about clipping
        clipped = np.abs(sway_raw) > self.max_sway
        if clipped.any():
            pct = 100.0 * clipped.mean()
            print(
                f"  ⚠ [COMStability] {pct:.1f}% of frames need sway > "
                f"max_sway={self.max_sway:.3f} m — clipped.  "
                f"Consider reducing v_y or choosing a gait with wider support."
            )

        n_unstable_before = int((info["margins"] < 0).sum())
        pct_unstable = 100.0 * n_unstable_before / N
        print(
            f"  [COMStability] Before correction: {n_unstable_before}/{N} frames "
            f"({pct_unstable:.1f}%) have negative stability margin."
        )
        print(
            f"  [COMStability] Applying sway corrections "
            f"(peak |sway| = {np.max(np.abs(sway))*100:.2f} cm) …"
        )

        cmds_stable = np.empty_like(cmds_raw)
        for f in range(N):
            cmds_stable[f] = self._apply_sway(cmds_raw[f], sway[f])

        # Verify improvement
        info_stable = self.analyze(cmds_stable, n_cycles=n_cycles)
        n_unstable_after = int((info_stable["margins"] < 0).sum())
        print(
            f"  [COMStability] After  correction: {n_unstable_after}/{N} frames "
            f"({100.*n_unstable_after/N:.1f}%) have negative stability margin."
        )
        print("  [COMStability] Done ✓")

        # Cache results for plotting
        self._last_info = info
        self._last_info_stable = info_stable
        self._last_cmds_raw = cmds_raw
        self._last_cmds_stable = cmds_stable
        self._last_sway_raw = sway_raw
        self._last_sway = sway
        self._last_n_cycles = n_cycles

        return cmds_stable

    # ── Visualisation ─────────────────────────────────────────────────────────

    def plot_stability_analysis(
        self,
        n_cycles: int = 1,
        show_correction: bool = True,
    ) -> plt.Figure:
        """
        Comprehensive stability analysis figure.

        Panels
        ------
        **Row 1 – Stability margin vs time**
            Red curve (before) vs green curve (after correction).
            Shaded red region marks frames with negative margin.

        **Row 2 – Body sway signal**
            Grey: raw per-frame required sway.
            Blue: smoothed sway actually applied.

        **Row 3 – Support polygon snapshots (4 moments per cycle)**
            Shows foot positions, support polygon, and COM (uncorrected ★ red,
            corrected ★ lime-green) for phases 0, 0.25 T, 0.5 T, 0.75 T.

        **Row 4 – Foot Y trajectories + COM sway overlay**
            Solid line = stance; dashed = swing.
            Gold line = effective COM Y (body-centre + sway correction).

        Parameters
        ----------
        n_cycles : int
            If ``generate_stable_gait`` has not been called yet, it is invoked
            here with this cycle count.
        show_correction : bool
            If False, only the pre-correction data is shown.
        """
        if self._last_info is None:
            self.generate_stable_gait(n_cycles=n_cycles)

        info = self._last_info
        info_st = self._last_info_stable
        cmds_raw = self._last_cmds_raw
        sway_raw = self._last_sway_raw
        sway = self._last_sway
        n_cycles = self._last_n_cycles
        N = len(cmds_raw)
        n_points = self._n_points
        t = np.arange(N) * self.gen.dt

        # ── Figure layout ──────────────────────────────────────────────────────
        fig = plt.figure(figsize=(15, 12))
        title = (
            f"COM Stability Analysis  ─  {self.gen.gait_type}  "
            f"[ωz={self.gen.omega_z:.2f} rad/s,  "
            f"vx={self.gen.v_com[0]:.2f} m/s,  "
            f"vy={self.gen.v_com[1]:.2f} m/s]"
        )
        fig.suptitle(title, fontsize=13, fontweight="bold")
        gs = GridSpec(4, 4, figure=fig, hspace=0.50, wspace=0.35)

        # ── Row 1: Stability margin ────────────────────────────────────────────
        ax_m = fig.add_subplot(gs[0, :])
        ax_m.fill_between(
            t, info["margins"] * 100, 0,
            where=info["margins"] < 0,
            color="red", alpha=0.18, label="Unstable region (before)",
        )
        ax_m.plot(
            t, info["margins"] * 100,
            color="tomato", lw=1.5, label="Before correction",
        )
        if show_correction:
            ax_m.plot(
                t, info_st["margins"] * 100,
                color="seagreen", lw=1.5, label="After correction",
            )
        ax_m.axhline(
            self.safety_margin * 100, color="darkorange", ls="--", lw=1.2,
            label=f"Safety margin target ({self.safety_margin*100:.0f} cm)",
        )
        ax_m.axhline(0, color="k", lw=0.8, alpha=0.4)
        ax_m.set_ylabel("Stability Margin (cm)")
        ax_m.set_xlabel("Time (s)")
        ax_m.set_title("Stability Margin vs Time")
        ax_m.legend(fontsize=8, loc="upper right")
        ax_m.grid(True, alpha=0.3)

        # ── Row 2: Body sway ──────────────────────────────────────────────────
        ax_s = fig.add_subplot(gs[1, :])
        ax_s.plot(
            t, sway_raw * 100,
            color="silver", lw=0.9, alpha=0.7, label="Raw sway required",
        )
        ax_s.plot(
            t, sway * 100,
            color="steelblue", lw=2.0, label="Smoothed sway applied",
        )
        ax_s.axhline(self.max_sway * 100, color="red", ls=":", lw=1, alpha=0.6)
        ax_s.axhline(-self.max_sway * 100, color="red", ls=":", lw=1, alpha=0.6,
                     label=f"± max sway ({self.max_sway*100:.0f} cm)")
        ax_s.axhline(0, color="k", lw=0.8, alpha=0.4)
        ax_s.set_ylabel("Body Sway in Y (cm)")
        ax_s.set_xlabel("Time (s)")
        ax_s.set_title("Corrective Body Sway Signal")
        ax_s.legend(fontsize=8)
        ax_s.grid(True, alpha=0.3)

        # ── Row 3: Support-polygon snapshots at 4 phase fractions ─────────────
        snap_fracs = [0.0, 0.25, 0.50, 0.75]
        for col, frac in enumerate(snap_fracs):
            ax_p = fig.add_subplot(gs[2, col])
            f_idx = int(frac * n_points)
            mask = self.stance_mask(f_idx % n_points, n_points)
            fxy = info["foot_xy"][f_idx]
            self._draw_snapshot(
                ax=ax_p,
                foot_xy=fxy,
                mask=mask,
                com_xy=self.com_xy,
                sway_y=float(sway[f_idx]),
                title=f"phase = {frac:.2f} T",
            )

        # ── Row 4: Foot Y positions + COM overlay ─────────────────────────────
        ax_y = fig.add_subplot(gs[3, :])
        for i in range(4):
            foot_y = info["foot_xy"][:, i, 1] * 100
            mask_i = np.array([
                self.stance_mask(f % n_points, n_points)[i]
                for f in range(N)
            ])
            ax_y.plot(t, foot_y, color=self._LEG_COLORS[i], lw=0.8,
                      alpha=0.35, ls="--")
            stance_y = np.where(mask_i, foot_y, np.nan)
            ax_y.plot(
                t, stance_y,
                color=self._LEG_COLORS[i], lw=1.8,
                label=self._LEG_LABELS[i] + " (stance)",
            )

        # COM before correction: stays at COM_BIAS_Y (constant in body frame)
        ax_y.axhline(
            self.com_xy[1] * 100, color="red", lw=1.5, ls="-.",
            label=f"COM Y uncorrected ({self.com_xy[1]*100:.1f} cm)",
        )
        # COM after correction: COM_Y + sway (effective position over support)
        effective_com_y = (self.com_xy[1] + sway) * 100
        ax_y.plot(t, effective_com_y, color="gold", lw=2.2,
                  label="COM Y after sway correction")

        ax_y.set_ylabel("Y Position (cm)")
        ax_y.set_xlabel("Time (s)")
        ax_y.set_title(
            "Foot Y-Positions (solid = stance, dashed = swing) and COM Trajectory"
        )
        ax_y.legend(fontsize=7, ncol=3, loc="upper right")
        ax_y.grid(True, alpha=0.3)

        plt.tight_layout()
        return fig

    # ── Snapshot helper ───────────────────────────────────────────────────────

    def _draw_snapshot(
        self,
        ax: plt.Axes,
        foot_xy: np.ndarray,    # (4, 2)
        mask: np.ndarray,       # bool (4,)
        com_xy: np.ndarray,     # (2,)
        sway_y: float,
        title: str,
    ) -> None:
        """Draw a single support-polygon snapshot panel."""
        ax.set_aspect("equal")

        # Robot body outline (top-view rectangle)
        wb = self.gen.legs[0].l_body / 2
        tw = self.gen.legs[0].w_body / 2
        rect_x = [wb, wb, -wb, -wb, wb]
        rect_y = [tw, -tw, -tw, tw, tw]
        ax.plot(rect_x, rect_y, "k-", lw=1.2, alpha=0.45)

        # Foot positions
        for i in range(4):
            c = self._LEG_COLORS[i]
            mk = "o" if mask[i] else "^"
            ms = 55 if mask[i] else 35
            ax.scatter(foot_xy[i, 0], foot_xy[i, 1],
                       c=c, marker=mk, s=ms, zorder=5, edgecolors="k", linewidths=0.4)
            ax.annotate(
                self._LEG_LABELS[i],
                xy=foot_xy[i],
                xytext=(0, 5), textcoords="offset points",
                fontsize=7, ha="center", color=c, fontweight="bold",
            )

        # Support polygon
        stance_pts = foot_xy[mask]
        n = len(stance_pts)
        if n >= 3:
            try:
                hull = ConvexHull(stance_pts)
                verts = stance_pts[hull.vertices]
                verts_closed = np.vstack([verts, verts[0]])
                ax.fill(verts[:, 0], verts[:, 1], alpha=0.18, color="limegreen")
                ax.plot(verts_closed[:, 0], verts_closed[:, 1],
                        "g-", lw=1.8, label="Support polygon")
            except Exception:
                ax.plot(stance_pts[:, 0], stance_pts[:, 1], "g-", lw=2)
        elif n == 2:
            ax.plot(stance_pts[:, 0], stance_pts[:, 1],
                    "g-", lw=2.5, label="Support line")
        elif n == 1:
            ax.scatter(stance_pts[0, 0], stance_pts[0, 1],
                       c="limegreen", s=100, zorder=6, marker="s")

        # COM positions
        ax.scatter(
            com_xy[0], com_xy[1],
            c="red", s=90, marker="*", zorder=10, label="COM (uncorrected)",
        )
        com_corrected = np.array([com_xy[0], com_xy[1] + sway_y])
        ax.scatter(
            com_corrected[0], com_corrected[1],
            c="lime", s=90, marker="*", zorder=10, edgecolors="k",
            linewidths=0.5, label="COM (after sway)",
        )
        if abs(sway_y) > 1e-4:
            ax.annotate(
                "", xy=com_corrected, xytext=com_xy,
                arrowprops=dict(arrowstyle="->", color="darkgreen", lw=1.2),
            )

        # Safety margin circle around corrected COM (for ≥3 leg case)
        if n >= 3:
            circle = plt.Circle(
                com_corrected, self.safety_margin,
                color="darkgreen", fill=False, ls=":", lw=1.0, alpha=0.6,
            )
            ax.add_patch(circle)

        ax.set_title(title, fontsize=8)
        ax.set_xlabel("X (m)", fontsize=7)
        ax.set_ylabel("Y (m)", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=5, loc="lower right")
        ax.grid(True, alpha=0.2)

    # ── Convenience summary ───────────────────────────────────────────────────

    def print_summary(self) -> None:
        """Print a text summary of the stability analysis results."""
        if self._last_info is None:
            print("No analysis data available. Call generate_stable_gait() first.")
            return

        info = self._last_info
        info_st = self._last_info_stable
        N = len(info["margins"])
        n_line = int((info["stance_counts"] <= 2).sum())
        pct_line = 100.0 * n_line / N

        print("=== COMStabilityPlanner Summary ===")
        print(f"  Gait type  : {self.gen.gait_type}")
        print(f"  Body twist : ωz={self.gen.omega_z:.3f} rad/s, "
              f"vx={self.gen.v_com[0]:.3f} m/s, vy={self.gen.v_com[1]:.3f} m/s")
        print(f"  COM bias   : {self.com_xy}")
        print(f"  Safety margin target: {self.safety_margin*100:.1f} cm  "
              f"(polygon support only; line/point support targets margin=0)")
        print()
        print(f"  Support geometry:")
        print(f"    Frames with ≥2 stance legs (polygon support): "
              f"{N - n_line}/{N} ({100.0*(N-n_line)/N:.0f}%)")
        print(f"    Frames with ≤2 stance legs (line/point support): "
              f"{n_line}/{N} ({pct_line:.0f}%)")
        if pct_line > 50:
            print(f"    ⚠  This gait spends {pct_line:.0f}% of the time in line/point support, "
                  f"which is inherently statically unstable.")
            print(f"       'Unstable frame' count will never reach 0 for those frames —"
                  f" use the *margin improvement* as the key metric.")
        print()
        print(f"  BEFORE correction:")
        print(f"    Min margin : {info['margins'].min()*100:+.2f} cm")
        print(f"    Mean margin: {info['margins'].mean()*100:+.2f} cm")
        n_bad = int((info["margins"] < 0).sum())
        print(f"    Unstable frames: {n_bad}/{N} ({100.*n_bad/N:.1f}%)")
        print()
        print(f"  AFTER correction:")
        print(f"    Min margin : {info_st['margins'].min()*100:+.2f} cm")
        print(f"    Mean margin: {info_st['margins'].mean()*100:+.2f} cm")
        n_bad_st = int((info_st["margins"] < 0).sum())
        print(f"    Unstable frames: {n_bad_st}/{N} ({100.*n_bad_st/N:.1f}%)")
        margin_improvement = info_st["margins"].mean() - info["margins"].mean()
        print(f"    Mean margin improvement: {margin_improvement*100:+.2f} cm "
              f"({'better' if margin_improvement > 0 else 'worse'})")
        print()
        print(f"  Sway signal:")
        print(f"    Peak raw  sway: {np.max(np.abs(self._last_sway_raw))*100:.2f} cm")
        print(f"    Peak smooth sway: {np.max(np.abs(self._last_sway))*100:.2f} cm")
        print(f"    Max sway limit:   {self.max_sway*100:.2f} cm")
