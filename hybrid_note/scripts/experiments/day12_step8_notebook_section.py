"""Append the Step 8 section to the Day 12 dashboard notebook.

Idempotent: previously appended Step 8 cells (tagged ``MARKER``) are dropped
before the new ones go on.  Kept in the repository, not in a scratch directory
(trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_step8_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-step8-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

for cell in nb.cells:
    if cell.cell_type == "markdown" and "| 8 | 完整四腳" in cell.source:
        cell.source = cell.source.replace(
            "| 8 | 完整四腳 joint/contact trajectory | ⬜ |",
            "| **8** | 完整四腳 joint/contact trajectory "
            "| ✅ 完成（疊在三個未解結論上） |")

cells = []

cells.append(md(r"""
---

# Step 8 — 完整四腳 Joint / Contact Trajectory

規格 §15。到這裡為止的每一塊都已經算好了——Step 3 的時間軸、Step 4 的 per-leg
segment、Step 5 的 body、Step 6 的 margin、Step 7 的 reposition 判定——
Step 8 做的是**把它們對齊到同一組取樣上**。

**它不呼叫任何 planner。** 規格要求 7 禁止 runtime replanning，
而滿足它的方式是「這裡沒有任何東西可以 replan」：只在既有結果之間內插。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    FULL_TURN_RAD, assemble_whole_body_2d, whole_body_rows, assumptions_of,
)

whole = assemble_whole_body_2d(four_flat, traj, stability,
                               reposition_unresolved=2)
summary8 = whole.as_dict()

print("規格 §15 要的三個摘要數字")
print(f"  max joint jump           {summary8['max_joint_jump_deg']:.3f} deg"
      f"   （raw，含整圈）")
print(f"  max joint discontinuity  "
      f"{summary8['max_joint_discontinuity_deg']:.3f} deg"
      f"   <- wrapped；{summary8['whole_turn_handoffs']} 個交接是整圈，不是斷點")
print(f"  max contact gap          {summary8['max_contact_gap_mm']:.3f} mm")
print(f"  min stability margin     "
      f"{summary8['minimum_stability_margin_mm']:.3f} mm")
print(f"\n  max rim geometry gap     "
      f"{summary8['max_rim_geometry_gap_mm']:.4f} mm")
print(f"    {summary8['rim_gap_note']}")
print(f"  finite body_z samples    {summary8['finite_body_z_samples']} of "
      f"{summary8['samples']}   <- Step 5 的結論在這裡的樣子")
"""))

cells.append(md(r"""
## 8.1 360° 是**一整圈**，不是斷點

第一版自己算 joint jump，報出 360° 看起來像大災難。
但 Step 1 建 recovery 時就是 `beta_target = start.beta - 2π`——
**β 在這整棵樹裡是圈數計數器，從來不 wrap**。

只報 raw 會把整圈說成災難；只報 wrapped 會把整圈藏起來。**兩個都報。**

而 297 mm 的 contact gap 也不是新東西：那是**陷阱 25** —— 前後兩段 nominal run
各自獨立生成，天生接不起來。Step 8 **回報**它，不縫合它。
"""))

cells.append(code(r"""
rows8 = pd.DataFrame(whole_body_rows(whole))
display(rows8[rows8["row_kind"] == "handoff"][
    ["leg", "from_kind", "to_kind", "joint_jump_deg",
     "joint_jump_wrapped_deg", "is_whole_turn", "body_jump_mm",
     "contact_jump_mm", "rim_geometry_gap_mm"]].reset_index(drop=True))
"""))

cells.append(md(r"""
## 8.2 這條軌跡是**疊在什麼之上**的

`assumptions` 是**算出來的**，不是寫死的。平地這一趟沒有越障，
所以 Step 4 的 overrun **不在列**——那是因為它真的不存在，不是被漏掉
（有測試擋住「無中生有一條 Step 4 假設」）。
"""))

cells.append(code(r"""
for note in whole.assumptions:
    print(f"  - {note}")

print("\n對照：把 Step 7 的未解數改成 0，那一條就會消失")
for note in assumptions_of(four_flat, traj, stability, reposition_unresolved=0):
    print(f"  - {note}")
"""))

cells.append(code(r"""
sample = whole.samples[0]
row = sample.as_dict()
print(f"一個取樣的完整內容，t = {sample.time_s:.3f} s")
for key in ("body_x_mm", "body_z_mm", "body_roll_deg", "swing_leg",
            "support_legs", "stability_margin_mm"):
    print(f"  {key:22s} {row[key]}")
for leg in LEG_ORDER:
    s = sample.legs[leg].as_dict()
    print(f"  {leg.value}  theta {s['theta_deg']:7.2f}  beta {s['beta_deg']:8.2f}"
          f"  gamma {s['gamma_deg']:5.2f}  {s['mode']:8s} {s['rim']:9s}"
          f"  alpha {s['alpha_deg']:7.2f}"
          f"  contact ({s['contact_x_mm']:8.1f}, {s['contact_y_mm']:8.1f}) mm"
          f"  {s['segment_kind']}")
"""))

cells.append(md(r"""
## 8.3 兩次犯同一個錯：量測函式寫了第二套

- `handoff_between_2d` 是 Step 0 就抽出來的，理由正是「Day 12 要跨 frame source
  串 segment」。第一版重寫了一次。
- `segment_at`（Step 6 的，含邊界容差與半開區間規則）也被手寫了一遍，
  結果在 covered interval 的**最後一刻**只找到兩隻支撐腳。

兩次第二套都在**邊界**上出錯——那正是它們存在的理由。
現在 `segment_at` 升成公開的，Step 6 / Step 8 共用。

取樣也跟著改成半開 `[lo, hi)`：在 `hi` 那一刻一隻腳的 chain 已經用完、
另一隻的 swing 已經開始，那個瞬間根本沒有完整的四腳組態。

## 8.4 Step 8 驗收（規格 §15 的八項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 每個取樣有 body pose 與四腳 θβγ | ✅ 有測試逐點檢查 |
| 2 | 每腳有 mode/contact/rim/alpha | ✅ 且 `in_contact` 與 mode 一致 |
| 3 | swing leg / support legs / margin / segment index+kind | ✅ |
| 4 | 保留 segment 的 sampling 參數 | ✅ `arc_samples` 直接對得上原 segment |
| 5 | 五種 handoff 檢查 | ✅ 時間 / body / joint（兩個讀數）/ contact / rim gap |
| 6 | 量化 1.2 mm，不為了藏它改幾何 | ✅ 用既有的 `rim_point_model_gap_2d`，並註明這趟為何是 0 |
| 7 | 不做 runtime replanning | ✅ 這個模組**不呼叫任何 planner** |
| 8 | 序列化 | ✅ `whole_body_rows()`，四種 row kind，單一表頭 |

**Step 9 要做的**：whole-body validation（規格 §16）。
Step 3 量到的「recovery 的 β 角速度要比 rolling 快 **10.553 倍**」
當時**刻意不判斷**它有沒有超出關節速度極限——**那就是 Step 9 的事**，
而 Step 8 現在把每一幀的 θβ 都排好了，所以第一次可以真的去比。
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
     "tests/test_day12_support_stability_2d.py",
     "tests/test_day12_whole_body_trajectory_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
print("（Step 7 的測試另外跑：swing planner 很慢，見 implementation log）")
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
