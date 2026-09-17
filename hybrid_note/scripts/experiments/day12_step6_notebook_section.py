"""Append the Step 6 section to the Day 12 dashboard notebook.

Idempotent: any previously appended Step 6 cells (marked by ``MARKER``) are
dropped before the new ones go on, so re-running replaces rather than
duplicates.  Kept in the repository rather than in a scratch directory: the
Step 5 generator lived in ``/tmp`` and was cleared (trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_step6_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-step6-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

for cell in nb.cells:
    if cell.cell_type == "markdown" and "| 6 | 三腳 support" in cell.source:
        cell.source = cell.source.replace(
            "| 6 | 三腳 support triangle + CoM margin | ⬜ |",
            "| **6** | 三腳 support triangle + CoM margin "
            "| ✅ 完成（結論是 UNSTABLE） |")

cells = []

cells.append(md(r"""
---

# Step 6 — 三腳 Support Triangle + Stability Margin

規格 §13。一隻腳在空中時，另外三隻撐住機器人。這一步在**水平面**上建它們的
support triangle，把 body 中心投影上去，並且**沿整個 swing** 取樣 signed margin
——不是只看 liftoff 那一幀，因為滾動中的支撐腳接觸點會在 swing 期間移動。

## 6.1 三個不能弄丟的標籤

```text
contact，不是 hip     規格特別點名。用「接觸點【相對髖】的 x」，
                     所以與 chain 那個任意的 x 原點無關。
body 中心，不是 CoM   沒有 whole-robot 質量模型，每一列都標明這件事。
body_z 不可行         Step 5 的結論。margin 是【水平】問題、body_x 仍良定義，
                     所以算得下去——但這個前提要跟著答案一起走。
```

`gamma = 0`（要求 8），所以每隻腳的矢狀面固定在它自己的 y，
橫向座標直接用 Step 2 量到的 mounting offset，不需要新模型。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    COM_BASIS, GAMMA_RAD, BOUNDARY_TOLERANCE_S, DEGENERATE_AREA_M2,
    SupportTriangle2D, contact_offset_from_hip_m,
    support_triangle_at, swing_stability_2d, stability_rows, plot_stability_2d,
)

print(f"gamma       {GAMMA_RAD}   (Day 12 不調 ABAD)")
print(f"CoM basis   {COM_BASIS}")
print("\n橫向座標 = Step 2 的 mounting offset（不是新數字）")
for m in leg_mounts_2d(GAMMA_RAD):
    r = m.as_dict()
    print(f"  {r['leg']}  x {r['offset_x_mm']:+8.3f}  y {r['offset_y_mm']:+9.3f} mm")

# inside / boundary / outside，用手算的直角三角形（規格要求 10）
unit = np.array([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])
tri = SupportTriangle2D(time_s=0.0, swing_leg=None,
                        support_legs=(LegId.RF, LegId.LH, LegId.RH),
                        points_xy_m=unit)
for name, point in (("inside", (0.2, 0.2)), ("boundary", (0.5, 0.0)),
                    ("outside", (-0.3, 0.5))):
    print(f"  {name:9s} margin = {tri.signed_margin_m(point):+.4f} m")
"""))

cells.append(md(r"""
## 6.2 Step 6 的結果：**margin 最小 0.000 mm，五個 swing 全部不穩**

**為什麼剛好是 0：對稱，不是捨入誤差。**
LH / RH 這兩個 swing 起跳的瞬間，對角的兩隻支撐腳量到在
`(+239.2, -211.7)` 與 `(-239.2, +211.7)` mm——**對 body 中心完全對稱**，
所以連接它們的那條邊**正好通過 body 中心**，margin 因此正好是 0。

另外三個 swing 沒有那個對稱，但最好也只有 **0.995 mm**。
整趟從來沒有超過 1 mm 的餘裕。
"""))

cells.append(code(r"""
stability = swing_stability_2d(four_flat, traj)
summary = stability.as_dict()
print(f"body 假設     {summary['body_basis']}")
print(f"swings        {summary['swings']}   unstable = {summary['unstable_swings']}")
print(f"margin floor  {summary['margin_floor_mm']:.1f} mm   （規劃門檻，不是量到的物理極限）")
print(f"最小 margin   {summary['minimum_stability_margin_mm']:.3f} mm")
print(f"stable        {summary['stable']}")

rows6 = pd.DataFrame(stability_rows(stability))
display(rows6[rows6["row_kind"] == "swing"][
    ["swing_leg", "support_legs", "start_s", "end_s",
     "minimum_stability_margin_mm", "worst_time_s", "stable"]
].reset_index(drop=True))
"""))

cells.append(md(r"""
## 6.3 順便證明了規格要求 5 是對的

「只檢查 liftoff 那一幀」會怎樣？RF 的 swing 在**起跳時** margin 是 **20.897 mm**，
輕鬆通過 10 mm 門檻——但它在 swing 期間**線性衰減**到 0.995 mm。
只看第一幀會把一個不穩的 swing 判成穩的。

衰減是線性的原因也很直接：body 一直往前走，而支撐腳的接觸點留在原地。
"""))

cells.append(code(r"""
rf = next(s for s in stability.swings if s.swing_leg is LegId.RF)
margins = [s.margin_m * 1e3 for s in rf.samples]
print(f"RF swing [{rf.start_s:.3f}, {rf.end_s:.3f}] s")
print(f"  起跳時       {margins[0]:8.3f} mm   <- 只看這一幀就會判定通過")
print(f"  最小         {rf.minimum_margin_m*1e3:8.3f} mm")
print(f"  單調遞減     {all(a >= b - 1e-12 for a, b in zip(margins, margins[1:]))}")
print(f"  stable       {rf.is_stable}")
"""))

cells.append(code(r"""
plot_stability_2d(four_flat, traj, stability,
                  path=DAY12 / "day12_step6_stability.png")
display(Image(filename=str(DAY12 / "day12_step6_stability.png")))
"""))

cells.append(md(r"""
## 6.4 Step 6 驗收（規格 §13 的十項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 用實際 world contact point | ✅ 相對髖的偏移，與 chain 原點無關（有平移不變測試） |
| 2 | 水平面 support triangle | ✅ 橫向用 Step 2 的 mounting offset |
| 3 | CoM 投影 | ✅ body 中心，且每列都標明它不是 whole-robot CoM |
| 4 | signed margin | ✅ inside / boundary / outside 都有測試 |
| 5 | 整個 swing 都要算 | ✅ 而且證明了只看 liftoff 會誤判（20.897 → 0.995 mm） |
| 6 | 每個 swing 與整趟都存最小 margin | ✅ 含「最小發生在哪一刻」 |
| 7 | margin 不足 → 標記 infeasible | ✅ 5 個 swing 全部 unstable |
| 8 | gamma 維持 0 | ✅ `GAMMA_RAD = 0.0`，有測試 |
| 9 | 視覺化 | ✅ 最差瞬間的三角形 + CoM + swing leg，右側 margin 對時間 |
| 10 | inside / boundary / outside 單元測試 | ✅ 用手算的直角三角形，不用 planner 輸出 |

## 6.5 目前累積、**互相獨立**的三個未解結論

```text
Step 4  時間放不下   0.6 s 的 swing 窗口要裝 1.2 s 的已排動作（2.000 x）
Step 5  高度對不起來 三隻站立腳要求的 body 高度差 15.329 mm  -> INFEASIBLE
Step 6  沒有餘裕     margin 最小 0.000 mm，五個 swing 全部 unstable
```

三個都**不是 bug**，都是規格要求「要問出來」的東西。不要合成一個「還沒好」。
規格自己指的路是 **Day 13–14 調 gamma**（改變橫向支撐幾何）——
Day 12 只做 feasibility。
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
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
