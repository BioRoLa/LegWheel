# Hybrid Note Workspace

This folder is the working area for hybrid leg-wheel ICRA material. Keep the root
level clean: scripts, data, generated outputs, notes, and paper-ready assets each
have a dedicated place.

## hybrid_note/
  scripts/      可重跑的程式
  data/         raw / processed / metadata
  outputs/      程式產生的中間圖、表、log
  notes/        推導、想法、會議紀錄
  configs/      repeatable run 的設定
  paper/        最後 ICRA 要用的精修圖表

## Folder Layout

- `scripts/kinematics/`: kinematics checks, plots, and geometry inspection scripts.
- `scripts/analysis/`: analysis scripts that process experiment or simulation data.
- `scripts/experiments/`: scripts used to generate or organize experiment inputs.
- `data/raw/`: original data copied from experiments, simulation, or hardware logs.
- `data/processed/`: cleaned or transformed datasets.
- `data/metadata/`: parameter notes, run manifests, and dataset descriptions.
- `outputs/figures/`: generated figures that can be reproduced from scripts.
- `outputs/tables/`: generated CSVs or summary tables that can be reproduced.
- `outputs/logs/`: console logs or run diagnostics from generated outputs.
- `notes/`: free-form notes, derivations, TODOs, and meeting notes.
- `configs/`: local configuration files for repeatable runs.
- `paper/figures/`: curated paper-ready figures copied or exported from outputs.
- `paper/tables/`: curated paper-ready tables copied or exported from outputs.

## Conventions

- Do not put generated PNG, CSV, or log files directly in this root folder.
- Put reusable code in `scripts/` and make each script write to `outputs/`.
- Treat `outputs/` as reproducible intermediate material.
- Treat `paper/` as curated final material for the ICRA submission.
- Prefer descriptive filenames with the topic first, for example
  `kinematics_current_leg_xy.png` or `gait_hybrid_phase_summary.csv`.

## Current Scripts

Open the presentation notebook for the current hybrid note material:

```bash
jupyter notebook hybrid_note/notes/Hybrid_Note_ICRA.ipynb
```

Open the single-leg hybrid gait planning notebook:

```bash
jupyter notebook hybrid_note/hybrid_note.ipynb
```

Generate only the single-leg 2D hybrid stance/swing trajectory CSV:

```bash
.venv/bin/python hybrid_note/scripts/hybrid_gait/single_leg_hybrid_gait.py
```

Generate the CSV, overview figure, and separate stance/swing HTML animations.
This visualizer imports the planner above; gait math is not duplicated:

```bash
.venv/bin/python hybrid_note/scripts/hybrid_gait/visualize_single_leg_hybrid_gait.py
```

Generate a single-leg contact-mode swing between different rims. The default
example lifts off from `left_rim`, retracts through the `foot_rim` region, and
touches down on `right_rim`:

```bash
.venv/bin/python hybrid_note/scripts/hybrid_gait/multi_rim_hybrid_gait.py
.venv/bin/python hybrid_note/scripts/hybrid_gait/visualize_multi_rim_hybrid_gait.py
```

Supported endpoint names are `foot_rim`, `left_rim`, and `right_rim`. This
stage covers swing/contact-mode transitions; loaded no-slip stance rolling on
the left/right structural rims is not modelled yet.

The 2D planner keeps body pose and terrain targets separate. `hip_height_m`
replaces the ambiguous body-height use of `stand_height`, while
`current_ground_height_m` and `next_foothold_(x, height)_m` describe this leg's
terrain. Leave `next_foothold_x_m=None` for a periodic flat-ground cycle.

Run the current 2D leg kinematics plot:

```bash
python3 hybrid_note/scripts/kinematics/plot_leg_2d.py
```

Run the local 3D leg kinematics plot with `O` fixed at the origin:

```bash
.venv/bin/python hybrid_note/scripts/kinematics/plot_leg_3d.py
.venv/bin/python hybrid_note/scripts/kinematics/plot_leg_3d.py --gamma-deg 20
```

Run the 2D single-pose geometric ground-contact analysis:

```bash
.venv/bin/python hybrid_note/scripts/kinematics/ground_contact_single_pose.py
```

Run the 3D single-pose geometric ground-contact analysis:

```bash
.venv/bin/python hybrid_note/scripts/kinematics/ground_contact_pose_3d.py
.venv/bin/python hybrid_note/scripts/kinematics/ground_contact_pose_3d.py --theta-deg 17 --beta-deg 45 --gamma-deg 10
```

Run the rim contact parameter scan:

```bash
.venv/bin/python hybrid_note/scripts/analysis/rim_contact_parameter_scan.py
.venv/bin/python hybrid_note/scripts/analysis/rim_contact_parameter_scan.py --map-2d-theta-samples 81 --map-2d-beta-samples 181
```

Run the theta-beta ground-contact state map:

```bash
.venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map.py --theta-step-deg 1.0 --beta-step-deg 1.0
```

Run the fixed-gamma 3D theta-beta ground-contact state map:

```bash
.venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map_3d.py --gamma-deg 10 --theta-step-deg 2.0 --beta-step-deg 2.0
```

Run the accelerated state maps. These reuse each theta geometry across all
beta values and default to the full 0.1-degree grid. PNG and compact NPZ are
saved by default; add `--csv path/to/file.csv` only when a text table is needed:

```bash
.venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map_fast.py
.venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map_3d_fast.py --gamma-deg 10
```

Compare several fixed-gamma contact maps against a baseline gamma. The output
includes aligned maps, difference maps, state-area ratios, and changed-cell
ratios:

```bash
.venv/bin/python hybrid_note/scripts/analysis/compare_ground_contact_gamma.py \
  --gamma-deg -20 -10 0 10 20 \
  --baseline-gamma-deg 0
```

Default outputs:

- `outputs/figures/kinematics/leg_2d.png`
- `outputs/tables/kinematics/leg_2d_points.csv`
- `outputs/figures/kinematics/leg_3d.png`
- `outputs/tables/kinematics/leg_3d_points.csv`
- `outputs/tables/kinematics/ground_contact_single_pose_summary.csv`
- `outputs/tables/kinematics/ground_contact_single_pose_candidates.csv`
- `outputs/tables/kinematics/ground_contact_pose_3d_summary.csv`
- `outputs/tables/kinematics/ground_contact_pose_3d_candidates.csv`
- `outputs/tables/analysis/rim_contact_scan_pose_contacts.csv`
- `outputs/tables/analysis/rim_contact_scan_candidates.csv`
- `outputs/tables/analysis/rim_contact_scan_surface_ranges.csv`
- `outputs/tables/analysis/rim_contact_scan_group_ranges.csv`
- `outputs/tables/analysis/rim_contact_scan_state_ranges.csv`
- `outputs/figures/analysis/rim_contact_map_2d_theta_beta.png`
- `outputs/figures/analysis/rim_contact_map_3d_theta_beta_gamma.png`
- `outputs/tables/analysis/ground_contact_state_map_theta_beta.csv`
- `outputs/figures/analysis/ground_contact_state_map_theta_beta.png`
- `outputs/tables/analysis/ground_contact_state_map_3d_theta_beta_gamma.csv`
- `outputs/figures/analysis/ground_contact_state_map_3d_theta_beta_gamma.png`
- `outputs/data/analysis/ground_contact_state_map_theta_beta_fast.npz`
- `outputs/figures/analysis/ground_contact_state_map_theta_beta_fast.png`
- `outputs/data/analysis/ground_contact_state_map_3d_theta_beta_gamma_fast.npz`
- `outputs/figures/analysis/ground_contact_state_map_3d_theta_beta_gamma_fast.png`
- `outputs/data/analysis/ground_contact_gamma_comparison.npz`
- `outputs/tables/analysis/ground_contact_gamma_comparison_summary.csv`
- `outputs/figures/analysis/ground_contact_gamma_comparison.png`
- `outputs/tables/hybrid_gait/single_leg_hybrid_gait.csv`
- `outputs/figures/hybrid_gait/single_leg_hybrid_gait_overview.png`
- `outputs/figures/hybrid_gait/single_leg_hybrid_gait_stance.html`
- `outputs/figures/hybrid_gait/single_leg_hybrid_gait_swing.html`
- `outputs/figures/hybrid_gait/single_leg_hybrid_gait_swing.gif`
- `outputs/tables/hybrid_gait/single_leg_multi_rim_swing.csv`
- `outputs/figures/hybrid_gait/single_leg_multi_rim_swing_overview.png`
- `outputs/figures/hybrid_gait/single_leg_multi_rim_swing.gif`
- `outputs/figures/hybrid_gait/single_leg_multi_rim_swing.html`
