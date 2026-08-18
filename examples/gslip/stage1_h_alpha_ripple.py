"""Stage 1's measured half: h(alpha) from the rim-crossing ripple.

The gate table (stage1_contact_sweep_analysis) computes the EXPECTED contact
height per achieved lean; this script extracts the MEASURED side from data
the sweep already banked: as the wheel rolls, the contact point sweeps the
toroidal rim (6.26 revolutions per roll), and the body's z ripples with the
rim geometry -- all four wheels share the same beta phase, so the body
ripple IS the contact-height profile h(alpha), mean-removed.

Model side: `CorgiLegKinematics.foot_rim_contact_fk` -- the toroidal lowest-
point selector -- evaluated at each leg's MEASURED theta and gamma over one
wheel revolution, averaged across the four legs. Measured side: odometry z
folded by the accumulated wheel angle beta mod 360, binned. The comparison
reports peak-to-peak amplitudes and the best circular-shift correlation
(the rig's beta zero and the model's are different conventions; the shift is
reported, not hidden).

Self-tests on synthetic known answers before touching a dump (diag rule).

Run:
    uv run python examples/gslip/stage1_h_alpha_ripple.py \
        <dump.npz>:<lam_cmd_deg> [...]
"""

from __future__ import annotations

import sys

import numpy as np

from legwheel.models.corgi_leg import CorgiLegKinematics

ROLL_WINDOW = (13.0, 28.0)     # steady rolling (fold_settle-3 schedule)
N_BINS = 72                    # 5 deg of wheel phase per bin


def fold_profile(phase_deg: np.ndarray, z: np.ndarray,
                 n_bins: int = N_BINS) -> np.ndarray:
    """Mean-removed z binned by (phase mod 360). NaN bins forbidden -- with
    6+ revolutions every bin is hit many times, so a hole means bad input."""
    ph = np.asarray(phase_deg, float) % 360.0
    idx = np.minimum((ph / 360.0 * n_bins).astype(int), n_bins - 1)
    prof = np.zeros(n_bins)
    for b in range(n_bins):
        m = idx == b
        if not m.any():
            raise SystemExit(f"empty phase bin {b} -- not enough rolling?")
        prof[b] = z[m].mean()
    return prof - prof.mean()


def best_shift_correlation(a: np.ndarray, b: np.ndarray):
    """(max correlation, shift in bins) of a vs b over circular shifts."""
    best, arg = -2.0, 0
    for s in range(len(a)):
        c = np.corrcoef(a, np.roll(b, s))[0, 1]
        if c > best:
            best, arg = float(c), s
    return best, arg


def model_profile(theta_rad: float, gammas_rad, n_bins: int = N_BINS):
    """Toroidal-model body ripple: per-leg lowest-contact height over one
    wheel revolution at the leg's measured (theta, gamma), averaged over the
    four legs (equal beta phase in the rig), mean-removed."""
    betas = np.deg2rad(np.linspace(0.0, 360.0, n_bins, endpoint=False)
                       + 360.0 / (2 * n_bins))
    prof = np.zeros(n_bins)
    for leg, gam in enumerate(gammas_rad):
        kin = CorgiLegKinematics(leg)
        h = np.zeros(n_bins)
        for i, b in enumerate(betas):
            alpha, w = kin.foot_rim_contact_fk(theta_rad, b, gam)
            h[i] = -kin.forward_kinematics(theta_rad, b, gam,
                                           alpha=alpha, w=w)[2]
        prof += h / len(gammas_rad)
    return prof - prof.mean()


def analyse(path: str, lam_cmd_deg: float) -> dict:
    d = np.load(path)
    mt, motor = d["motor_t"], d["motor_deg"]
    ot, odom = d["odom_t"], d["odom"]
    m = (mt >= ROLL_WINDOW[0]) & (mt <= ROLL_WINDOW[1])
    beta = motor[m, :, 1].mean(axis=1)          # accumulated deg, legs synced
    if beta.max() - beta.min() < 2 * 360.0:
        raise SystemExit(f"{path}: <2 revolutions in the roll window")
    z = np.interp(mt[m], ot, odom[:, 2]) * 1e3  # mm
    z = z - np.polyval(np.polyfit(mt[m], z, 1), mt[m])   # detrend the window

    measured = fold_profile(beta, z)
    # Phase-locked fraction: how much of the detrended z variance the folded
    # profile explains. Folding over 6+ revolutions averages out anything not
    # locked to the wheel angle, so a high fraction means the ripple IS a
    # wheel-geometry signature, not gait/controller noise.
    ph = beta % 360.0
    idx = np.minimum((ph / 360.0 * N_BINS).astype(int), N_BINS - 1)
    z_hat = measured[idx]
    locked = float(1.0 - np.var(z - z_hat) / np.var(z))
    # Dominant harmonic (cycles per wheel revolution) of the folded profile.
    spec = np.abs(np.fft.rfft(measured))[1:]
    k_dom = int(np.argmax(spec)) + 1

    theta = np.deg2rad(motor[m, :, 0].mean())
    gammas = np.deg2rad(motor[m, :, 2].mean(axis=0))
    model = model_profile(theta, gammas) * 1e3  # mm

    return {
        "lam": lam_cmd_deg,
        "meas_pp": float(measured.max() - measured.min()),
        "model_pp": float(model.max() - model.min()),
        "locked": locked,
        "k_dom": k_dom,
    }


def _selftest() -> None:
    # Folding recovers a known two-harmonic profile from a beta ramp.
    beta = np.linspace(0.0, 6.0 * 360.0, 20000)
    truth = lambda ph: (1.5 * np.cos(2 * np.deg2rad(ph))
                        + 0.5 * np.cos(np.deg2rad(ph) + 0.7))
    rng = np.random.default_rng(0)
    z = truth(beta) + 0.05 * rng.standard_normal(len(beta))
    prof = fold_profile(beta, z)
    centres = (np.arange(N_BINS) + 0.5) * 360.0 / N_BINS
    ref = truth(centres) - truth(centres).mean()
    assert np.corrcoef(prof, ref)[0, 1] > 0.99
    # Circular shift is found exactly.
    c, s = best_shift_correlation(ref, np.roll(ref, 7))
    assert c > 0.999 and (N_BINS - s) % N_BINS == 7


def main(argv) -> None:
    _selftest()
    if not argv:
        print(__doc__)
        print("selftest: PASS")
        return
    print(f"{'lam':>5} | {'meas p-p':>9} {'model p-p':>9} | "
          f"{'locked':>6} {'k_dom':>5}")
    for arg in argv:
        path, lam = arg.rsplit(":", 1)
        r = analyse(path, float(lam))
        print(f"{r['lam']:4.0f}d | {r['meas_pp']:7.2f}mm {r['model_pp']:7.2f}mm"
              f" | {r['locked']:6.2f} {r['k_dom']:5d}")


if __name__ == "__main__":
    main(sys.argv[1:])
