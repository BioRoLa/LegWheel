# s338 low-speed scans (implementation log §338)

These are the scripts and saved scans behind log §338. That entry finds that the
Stage 2a envelope's ≈0.53 m/s "existence edge" comes from the solver's search
window (β ≤ 86° at a 0.5° step, α ≤ 45°), not from SLIP-RF. Steeper landings give
non-grazing pronk fixed points at far lower forward speeds. Every scan is the
planar SLIP-RF return map at λ = 0, so no simulator time is involved.

They were moved here on 2026-09-14 from a session scratchpad. Each script's
paths are now derived from its own location, and its docstring records exactly
what changed.

## How to run

Run from the LegWheel root with the WSL venv. From Git Bash, add the
`MSYS_NO_PATHCONV=1` prefix:

    wsl.exe -d ubuntu-22.04 -- bash -lc "cd /mnt/c/Users/alexc/code/LegWheel && .venv/bin/python examples/gslip/s338_lowspeed/<script>.py [args]"

`verify_s338.py` and `extra.py` also read the paper repo's cache of record,
`figures/stage2a_grid.npz`. They expect a sibling checkout at
`../corgi-abad-icra2027`.

## Files

| file | what it is |
|---|---|
| `lowspeed_scan.py <mode>` | The scan. It records every fixed point unfiltered and prints one JSON line per (k_rel, v_td) job. The modes `prod`, `wide`, `floor`, `band` and `krel` are the windows tabulated in §338 §2. It re-solves with 8 processes. Redirect to a new file, never over the saved scans. |
| `scan_prod.jsonl`, `scan_wide.jsonl`, `scan_floor.jsonl`, `scan_band.jsonl`, `scan_krel.jsonl` | The saved scans that §338 reports. |
| `summarize_scan.py <modes...>` | Per-mode summary of the saved scans under the C1 filter (duty ≤ 0.55, apex ≥ 10 mm). Reads data only. |
| `verify_s338.py` | The §338 §2 re-derivation. It lists C1-passing points by v_td, split into inside and outside the production window, then tabulates the cache of record by v_td. Reads data only. |
| `verify_lowspeed_fp.py` | Independent spot check with `stage2a_turning_envelope.base_params()`: the production window against steep landings at v_td 0.60 and 0.70. Re-solves. |
| `extra.py` | §338 §3. It runs the production search at β 84.00 / 84.25 / 84.50° at v_td 0.70, calls `solve_existence` at β step 0.5 and 0.25, and checks the cache's identity (sha256). Re-solves; the slowest script here. |
| `v070.py` | Finds which forward speed the solver gives at the ṽ0.70 touchdown speed. Resolved in §339 §5m: the shipped v070 template (β step 0.25) runs 0.892 m/s, and 0.87 is `solve_existence` at β step 1.0. Re-solves. |
