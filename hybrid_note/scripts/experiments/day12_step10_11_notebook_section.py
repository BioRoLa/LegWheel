"""Append the Step 10 and Step 11 sections to the Day 12 dashboard notebook.

Both at once, because executing the notebook is the slow part and there is no
reason to pay for it twice.  Idempotent per section (tagged cells are dropped
first).  Kept in the repository, not in a scratch directory (trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_step10_11_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKERS = ("day12-step10-section", "day12-step11-section")

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if not (set(MARKERS) & set(c.get("metadata", {}).get("tags", [])))]

for cell in nb.cells:
    if cell.cell_type != "markdown" or "| Step |" not in cell.source:
        continue
    cell.source = cell.source.replace(
        "| 10 | 40 mm × 100 mm 對稱障礙 simulation | ⬜ |",
        "| **10** | 參數化地形整合 + generalization gate | ✅ 完成 |")
    cell.source = cell.source.replace(
        "| 11 | Paper metrics | ⬜ |",
        "| **11** | Paper metrics | ✅ 完成 |")


def md(text, marker):
    return nbf.v4.new_markdown_cell(text.strip("\n"),
                                    metadata={"tags": [marker]})


def code(text, marker):
    return nbf.v4.new_code_cell(text.strip("\n"), metadata={"tags": [marker]})


# ---------------------------------------------------------------- Step 10
S10 = MARKERS[0]
cells = []

cells.append(md(r"""
---

# Step 10 — 參數化地形整合 + Generalization Gate

規格 §17。要驗的**不是**「planner 能不能過某一個特定尺寸」，而是：

> **同一套** whole-body planner，能不能在**不改核心 code** 的前提下
> 吃不同的矩形地形參數，產生合法軌跡**或**明確的 infeasible reason。

## 10.1 唯一入口，平地不是特例

`plan_terrain_2d(terrain, tables)`。`terrain=None` 就是平地——
那是「**沒有障礙**」，不是「尺寸為 0」。
平地跑的是每個地形在越障之間都會跑的同一個 nominal cycle。
""", S10))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import load_tables_2d
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    plan_terrain_2d, comparison_rows, planner_size_literals,
    nominal_body_height_m, PLANNER_MODULES,
)

tables10 = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")
print(f"nominal body height  {nominal_body_height_m()*1e3:.4f} mm")
print("  從腿的取樣幾何解出來，【不是】蓋一個平台量出來的（見 10.3）")

TERRAIN_QUERIES = [("flat", None, 0.0), ("40mm x 400mm", 0.04, 0.40),
                   ("100mm x 400mm", 0.10, 0.40), ("190mm x 400mm", 0.19, 0.40)]
runs = []
for label, h, l in TERRAIN_QUERIES:
    terrain = None if h is None else SharedTerrainSpec2D(
        height_m=h, top_length_m=l, x_start_m=1.00, obstacle_id="day12_platform")
    runs.append(plan_terrain_2d(terrain, tables10, samples=61))

t10 = pd.DataFrame(comparison_rows(runs))
display(t10[t10["row_kind"] == "terrain"][
    ["terrain", "feasible", "ascent_primitive", "descent_primitive",
     "nominal_recovery_swings", "terrain_transition_swings",
     "max_body_lift_mm", "min_stability_margin_mm"]].reset_index(drop=True))
""", S10))

cells.append(md(r"""
平地 / 4 cm / 10 cm **走同一條 pipeline**，差別只有 terrain-transition swing
從 0 變成 8——那正是規格要求 4 說的「平地跑 nominal cycle，
不要多出不必要的越障 swing」。兩個障礙的 primitive 都是 Day 10–11 的
`decide_2d` 選的，**Step 10 沒有選任何東西**。

## 10.2 19 cm：**資料沒有** ≠ **物理不可能**

challenge terrain 在 decision 階段就停住，理由是
`the rolling traversal was never swept at this height`。

把它寫成「19 cm 做不到」會是造假——它指出的待辦是「**去掃那個高度**」。
""", S10))

cells.append(code(r"""
challenge = runs[-1]
print(f"planned  {challenge.planned}     feasible  {challenge.feasible}")
print(f"first limiting constraint:")
first = challenge.first_limiting_constraint
print(f"  [{first.stage.value}] {first.detail}")
print(f"\n每個策略各自的理由（{len(challenge.failures)} 個）")
for f in challenge.failures:
    print(f"  {str(f.strategy):28s} {f.detail[:90]}")
print(f"\n沒有為了讓它過而放寬任何東西：trajectory={challenge.trajectory}")
""", S10))

cells.append(md(r"""
## 10.3 Generalization gate 抓到了**我自己寫的**違規

`planner_size_literals()` 掃 9 個 planner 模組裡有沒有出現評估尺寸的字面值。
第一版跑出來抓到我剛寫的那一行：

```python
# 為了拿 nominal body height 而蓋了一個 4 cm 的平台
reference = terrain if terrain is not None else SharedTerrainSpec2D(
    height_m=0.04, top_length_m=0.40, ...)
```

一個 planner **根本不需要**的尺寸，變成了 planner 裡的常數。

修法是回到 Step 2 早就講過的事實：nominal 站姿在**下層地面**上、與平台無關——
所以它也**不該用蓋平台的方式取得**。改成從腿的取樣幾何直接解。
""", S10))

cells.append(code(r"""
found = planner_size_literals()
print(f"掃了 {len(PLANNER_MODULES)} 個 planner 模組")
if not found:
    print("  沒有任何模組出現評估尺寸")
for name, lines in found.items():
    print(f"  {name}")
    for line in lines:
        print(f"    {line[:100]}")
print("\n唯一的 hit 是【誤判】：0.10 * cycle_period_s 是週期比例不是長度。")
print("測試用一個【寫明理由的 allowlist】接受它，並且另外斷言那一行還在——")
print("allowlist 活得比它的理由久，就變成一個被消音的檢查。")
""", S10))

# ---------------------------------------------------------------- Step 11
S11 = MARKERS[1]

cells.append(md(r"""
---

# Step 11 — Paper Metrics

規格 §18 的兩條紅線：

```text
1. body centre 的指標與 whole-robot CoM 的指標【必須分開】
2. 在還沒做 energy experiment 之前，【不得】從這些指標推出 COT
```

第 2 條做成**機器可檢查**：`energy_vocabulary()` 掃這個模組的**程式碼**
（跳過 docstring、註解，也跳過禁用字清單本身）。
而且另有一個測試用假檔案證明這個 guard **真的會叫**——
不會叫的 guard 只是一句寫在註解裡的期望。
""", S11))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_paper_metrics_2d import (
    COM_METRICS_ABSENT, ENERGY_WORDS, energy_vocabulary,
    trajectory_metrics_2d, metrics_rows,
)

print(f"程式碼裡的能量詞彙: {energy_vocabulary() or 'none'}")
print(f"禁用字清單: {', '.join(ENERGY_WORDS)}")

metrics = [trajectory_metrics_2d(r.as_dict()["terrain"], r.trajectory, r.plan,
                                 r.body, r.stability)
           for r in runs if r.planned]
m11 = pd.DataFrame(metrics_rows(metrics))
rows = m11[m11["row_kind"] == "metrics"]
display(rows[["terrain", "traversal_distance_mm", "traversal_duration_s",
              "total_swing_segments", "nominal_recovery_swings",
              "terrain_transition_swings", "foot_rim_roll_leg_seconds",
              "swing_leg_seconds"]].reset_index(drop=True))
""", S11))

cells.append(md(r"""
## 11.1 第一版的時間指標是廢的

原本 roll 與 swing 的時間**都等於整段時長**——因為判斷式是
「**任何**一隻腳在這個 kind」，而任何時刻都同時有腳在滾、有腳在空中。
那個數字永遠等於整段長度，量不到任何東西。

改成 **leg-seconds**（四隻腳分別計時再加總），意義明確而且**會加總**：

```text
roll 8.926 + swing 2.975 = 11.901 = 4 × 2.975
```

欄位名也一起改成 `*_leg_seconds`——叫 `time_s` 會被讀成牆上時鐘。
""", S11))

cells.append(code(r"""
m = metrics[0]
print(f"roll {m.foot_rim_roll_time_s:.3f} + swing {m.swing_time_s:.3f} "
      f"= {m.foot_rim_roll_time_s + m.swing_time_s:.3f} leg-seconds")
print(f"4 x duration = {4 * m.traversal_duration_s:.3f}")

display(rows[["terrain", "body_z_peak_to_peak_mm", "body_z_std_mm",
              "usable_body_samples", "total_samples",
              "com_z_peak_to_peak_mm", "com_z_std_mm",
              "minimum_stability_margin_mm", "mean_swing_stability_margin_mm",
              "max_hip_lift_mm", "max_joint_discontinuity_deg",
              "max_contact_handoff_gap_mm"]].reset_index(drop=True))
""", S11))

cells.append(md(r"""
## 11.2 三個「不是 0，是量不到 / 不存在」

- **`body_z p2p` 與 `std` 是 `None`**：Step 5 之後 121 個取樣裡**一個**可用的
  body 高度都沒有。照算會得到 0，而 0 會被讀成「body 完全不起伏」，
  意思**正好相反**。`usable_body_samples` 一起報出去。
- **`CoM_z` 是 `None`**：這個 pipeline **沒有** whole-robot 質量模型。
  填 0 或填 body centre 的數字，都是把近似值當成量測值發表。
- **`transition ROLL 距離` 是 0**：`#4 SWING_SWING` 的越障 primitive
  **是 swing 不是 roll**，所以那個量本來就**不存在**，不是沒量到。

## 11.3 Step 11 驗收（規格 §18）

| 要求 | 結果 |
|---|---|
| traversal distance / duration | ✅ 491.02 mm / 2.975 s |
| total swing count | ✅ 平地 8、越障 16 |
| nominal RECOVERY_SWING count | ✅ 一律 8（不受地形影響） |
| terrain-transition swing count | ✅ 平地 0、越障 8 |
| FOOT_RIM_ROLL 時間 / 距離 | ✅ leg-seconds ＋ 距離（標明重疊） |
| transition ROLL 時間 / 距離 | ✅ 0 —— 不存在，非未量 |
| body_z peak-to-peak / std | ✅ `None` ＋ `usable_body_samples` |
| CoM_z 指標 | ✅ **沒有 model 所以不報**，附完整理由 |
| 最小 / 平均 support margin | ✅ 0.0000 / 10.49–10.55 mm |
| 最大 body / hip lift | ✅ 0 / 40 / 100 mm（隨障礙高度） |
| 最大 joint 不連續、contact gap | ✅ 39.84° / 652.696 mm |
| body centre 與 CoM 分開 | ✅ 兩個 basis 欄位，有測試斷言不相等 |
| 不得推導 energy / COT | ✅ guard 回空，且 guard 本身有測試 |
""", S11))

cells.append(code(r"""
print(COM_METRICS_ABSENT)
print()
print("Day 12 到此的正式完成條件（規格 §17）是")
print("「產生同步四腳軌跡【或】結構化的不可行結果，且不含地形尺寸專用邏輯」")
print("-> 這一條【達成了】。")
print()
print("但【沒有任何一個地形是 feasible】，七個未解結論見 implementation log §2。")
print("這兩件事必須分開講——不要讓表格自己看起來像成功報告。")
""", S11))

cells.append(code(r"""
import subprocess
proc = subprocess.run(
    ["python3", "-m", "pytest",
     "tests/test_day12_segment_contract_2d.py",
     "tests/test_day12_nominal_cycle_2d.py",
     "tests/test_day12_four_leg_state_2d.py",
     "tests/test_day12_timing_skeleton_2d.py",
     "tests/test_day12_transition_mapping_2d.py",
     "tests/test_day12_body_trajectory_2d.py",
     "tests/test_day12_support_stability_2d.py",
     "tests/test_day12_whole_body_trajectory_2d.py",
     "tests/test_day12_whole_body_validation_2d.py",
     "tests/test_day12_terrain_generalization_2d.py",
     "tests/test_day12_paper_metrics_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
print("（Step 7 的測試另外跑：swing planner 很慢，見 implementation log）")
""", S11))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
