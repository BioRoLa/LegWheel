"""Append the Step 7 section to the Day 12 dashboard notebook.

Idempotent: previously appended Step 7 cells (tagged ``MARKER``) are dropped
before the new ones go on.  Kept in the repository, not in a scratch directory
(trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_step7_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-step7-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

for cell in nb.cells:
    if cell.cell_type == "markdown" and "| 7 | `TOP_REPOSITION`" in cell.source:
        cell.source = cell.source.replace(
            "| 7 | `TOP_REPOSITION` resolution | ⬜ |",
            "| **7** | `TOP_REPOSITION` resolution "
            "| ✅ 完成（planning floor 下 support 不足） |")

cells = []

cells.append(md(r"""
---

# Step 7 — Resolve `TOP_REPOSITION`

規格 §14。Day 10–11 **故意**把這兩個 case 留成未解：單腳 planner 不可能知道
另外三隻腳撐不撐得住 body。Day 12 是第一個能問這個問題的地方。

## 7.1 順序是「支撐先、動作後」

先生成軌跡再檢查支撐，會產生一條「看起來像答案、但那個問題根本還沒被允許問」的軌跡。
所以 gate 沒過就**完全不生成**（規格要求 4：也不准發明 ABAD 補償）。

而且**不寫第二套 swing generator**（規格明令）。重用鏈全是現成的：

```text
standing_scene_2d / left_rim_landing_scene_2d
        ↓
build_swing_request_2d      <- Day 8-9 的既有 builder
        ↓
generate_swing_2d           <- 內建碰撞與 touchdown 驗證（要求 7 靠重用滿足）
        ↓
segment_from_swing_plan_2d  <- Day 10-11 已有的寫入 schema 的函式
```

## 7.2 `resolved = True` 不能寫回 requirement

`TransitionRequirement2D` 自己擋：

> a resolved transition is a segment, not a requirement:
> replace the record with the motion that solves it.

所以產出是一個 **attempt 記錄**，同時放**原始 requirement**（要求 10 的追溯）
與**生成出來的 segment**。不是把旗標翻掉。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import BLOCKED_PAIRS
from hybrid_note.scripts.experiments.day12_top_reposition_2d import (
    REPOSITION_TARGETS, TARGET_THETA_HEADROOM_RAD, DEFAULT_LANDING_BETA_STEP_DEG,
    RepositionOutcome, resolve_top_reposition_2d, reposition_rows,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
)

BLOCKED_CELLS = {
    StrategyId.ROLL_SWING: (0.160, 0.350),
    StrategyId.SWING_ROLL: (0.140, 0.225),
}

print("Day 10-11 留下的兩個未解 case  (規格 §14 要求 9)")
for strategy, (h, l) in BLOCKED_CELLS.items():
    t = REPOSITION_TARGETS[strategy]
    bound = ""
    if t.theta_min_rad is not None:
        bound += f", theta >= {np.rad2deg(t.theta_min_rad):.0f} deg"
    if t.theta_max_rad is not None:
        bound += f", theta <= {np.rad2deg(t.theta_max_rad):.0f} deg"
    print(f"  {strategy.value}   h = {h*1e3:.0f} mm, L_top = {l*1e3:.0f} mm")
    print(f"    Day 10-11 verdict   {BLOCKED_PAIRS[strategy].verdict.value}")
    print(f"    target（可檢查版）   {t.rim.value}{bound}")
"""))

cells.append(md(r"""
## 7.3 planning floor 下的答案：**支撐不足**

Step 6 已經量到平地走路的 margin 最小只有 0.995 mm，門檻是 10 mm。
所以兩個 case 在 gate 就停住，**沒有生成任何軌跡**。

規格 §14 的驗收本來就接受兩種結果之一——`resolved = True`，
或**明確的 support-related failure reason**。這是後者，而且理由是真的。
"""))

cells.append(code(r"""
planning = [resolve_top_reposition_2d(four_flat, traj, strategy, h, l,
                                      leg=LegId.LF)
            for strategy, (h, l) in BLOCKED_CELLS.items()]

for a in planning:
    print(f"\n{a.strategy.value}")
    print(f"  outcome            {a.outcome.value}")
    print(f"  reason             {a.reason}")
    print(f"  生成了軌跡嗎        {a.segment is not None}   <- gate 沒過就不生成")
    print(f"  原始失敗原因保留    {a.original_evidence[:110]}...")
"""))

cells.append(md(r"""
## 7.4 把支撐放寬之後（**探索，不是答案**）

為了證明 gate 兩個方向都會動，用一個低到一定會過的 floor 再跑一次。
每一列都被標上 `relaxed_floor`，不會被讀成結論。

- **`#2`** 走完整條鏈：Day 8–9 的 swing 通過碰撞與 touchdown 驗證，
  落在 foot rim、θ ≈ 37°，滿足「≥ 35°」——**`resolved = True`**。
- **`#3`** 卡在 `JOINT_DISCONTINUITY`。這是一個**真發現**：
  `LEFT_RIM_READY` 要的是左輪緣承載，而那需要一個離起始姿態很遠的 β，
  一個直接的 swing 過不去。換句話說 `#3` 的 reposition
  **不是「支撐夠了就能做」，它還缺一段把 β 轉過去的動作。**
"""))

cells.append(code(r"""
relaxed = [resolve_top_reposition_2d(four_flat, traj, strategy, h, l,
                                     leg=LegId.LF, margin_floor_m=-1.0,
                                     landing_beta_step_deg=2.0)
           for strategy, (h, l) in BLOCKED_CELLS.items()]

for a in relaxed:
    row = a.as_dict()
    print(f"\n{a.strategy.value}   [relaxed_floor = {a.relaxed_floor}]")
    print(f"  outcome        {row['outcome']}")
    print(f"  swing valid    {row['swing_valid']}   failure = {row['swing_failure']}")
    print(f"  segment kind   {row['segment_kind']}")
    print(f"  touchdown      rim {row['touchdown_rim']}, "
          f"theta {row['touchdown_theta_deg']}")
    print(f"  reason         {a.reason}")

rows7 = pd.DataFrame(reposition_rows(planning + relaxed))
display(rows7[rows7["row_kind"] == "attempt"][
    ["strategy", "relaxed_floor", "outcome", "resolved",
     "support_minimum_margin_mm", "touchdown_rim", "touchdown_theta_deg"]
].reset_index(drop=True))
"""))

cells.append(md(r"""
## 7.5 兩個量錯又修正的地方

**1. 對著 floor 瞄準會失敗。** 第一版把 target θ 設成 35°（＝下限），
IK 解出來是 **34.99955°**，差 0.00045° 被判不合格。
touchdown θ 是 IK 的**輸出**、不是請求的輸入（Day 10–11 陷阱 16）。
→ 加 `TARGET_THETA_HEADROOM_RAD`：**請求**瞄在下限之上，**檢查**仍然用下限本身。

**2. `#3` 的 target 不能用 standing scene。** θ = 17° 的站姿仍然是**腳輪緣**接觸，
而 `#3` 要的是**左輪緣**。→ 改接 Day 10–11 已經有的
`left_rim_beta_window_2d` + `choose_landing_beta_2d` + `left_rim_landing_scene_2d`。
**不是新寫一套。**

## 7.6 這一步動到了既有檔案：新增一個 `SegmentKind`

`TOP_REPOSITION_SWING`。不加就得說謊：

```text
SWING_OVER      是「一個 primitive 跨過整個障礙」——不是這個
RECOVERY_SWING  是「步態本來就會做的」——這個是地形逼出來的
```

規格 §18 要求 terrain-transition swing 單獨計數，借用別的 kind 會直接污染那個統計。
改動是**純新增**，而兩個「三個 terrain-transition swing」的斷言因此失敗——
**那是它們在做事**：Step 0 就是設計成新增 kind 一定會撞到。

## 7.7 Step 7 驗收（規格 §14 的十項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 找出未解的 `TOP_REPOSITION` | ✅ 用 Day 10–11 自己的 `top_reposition_requirement_2d` |
| 2 | 目標腳 airborne、其餘三隻支撐 | ✅ 直接讀 Step 3 已排好的空中區間，不另外發明 |
| 3 | 跑 Step 6 的 margin 檢查 | ✅ `SupportGate2D` |
| 4 | 支撐不足 → unresolved，不發明 ABAD | ✅ gate 沒過就**完全不生成軌跡**，有測試擋住 |
| 5 | 重用 Day 8–9 swing generator | ✅ 沒有第二套 |
| 6 | touchdown 要是下一個 primitive 合法的狀態 | ✅ `RepositionTarget2D.accepts()` |
| 7 | 對整條軌跡重跑碰撞 / 接觸驗證 | ✅ `generate_swing_2d` 內建，靠重用滿足 |
| 8 | 成功則存下 segment | ✅ 存在 attempt 記錄；**不是**翻 requirement 的旗標 |
| 9 | 重測原本只因 direct handoff 失敗的 case | ✅ `#2` 與 `#3` 都跑了 |
| 10 | 保留原始失敗原因 | ✅ `original_evidence`，有測試確認沒被覆蓋 |

**Step 8 要做的**：組出完整的四腳 joint / contact trajectory（規格 §15）。
開始之前要帶著現在累積的**四個獨立未解結論**：

```text
Step 4  時間放不下   0.6 s 的 swing 窗口要裝 1.2 s 的已排動作（2.000 x）
Step 5  高度對不起來 三隻站立腳的 body 高度要求差 15.329 mm  -> INFEASIBLE
Step 6  沒有餘裕     margin 最小 0.000 mm，五個 swing 全部 unstable
Step 7  撐不住       兩個 TOP_REPOSITION 都在 support gate 就停住
```
"""))

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
     "tests/test_day12_support_stability_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
print("（Step 7 的測試另外跑：swing planner 很慢，見 implementation log）")
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
