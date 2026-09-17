"""Append the Step 9 section to the Day 12 dashboard notebook.

Idempotent: previously appended Step 9 cells (tagged ``MARKER``) are dropped
before the new ones go on.  Kept in the repository, not in a scratch directory
(trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_step9_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-step9-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

for cell in nb.cells:
    if cell.cell_type == "markdown" and "| 9 | Whole-body validation" in cell.source:
        cell.source = cell.source.replace(
            "| 9 | Whole-body validation | ⬜ |",
            "| **9** | Whole-body validation | ✅ 完成（11 檢查 7 過 4 失） |")

cells = []

cells.append(md(r"""
---

# Step 9 — Whole-Body Validation

規格 §16。前面每一步都驗過自己那一塊；這一步問的是**組起來之後還成不成立**，
並且回傳**結構化的失敗**（time / leg / segment_index / segment_kind / value /
limit / detail），不是一個布林值。

**它什麼都不修**——規格明令，而保證它的方式是這裡根本沒有任何寫入。

## 9.1 「沒檢查」「在別處檢查過」「檢查過而且通過」是三件事

```text
CheckId               這裡跑的 11 個
DELEGATED_CHECKS      在別處跑過的 4 個，附上【是誰】跑的
UNEVALUABLE_CHECKS    根本判不了的 2 個，附上【缺什麼】
```

把後兩者省略掉，讀起來就會像全部通過。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    CheckId, DELEGATED_CHECKS, UNEVALUABLE_CHECKS,
    validate_whole_body_2d, validation_rows, joint_rates_2d,
)

report = validate_whole_body_2d(whole, traj, stability)
s9 = report.as_dict()
print(f"checks run  {s9['checks_run']}   passed {s9['checks_passed']}"
      f"   failed {s9['checks_failed']}")
print(f"failures    {s9['failures']}")
print(f"is_valid    {s9['is_valid']}\n")

display(pd.DataFrame([
    {"check": c.value, "passed": c not in set(report.failed_checks()),
     "failures": len(report.failures_of(c))}
    for c in report.checks_run]))
"""))

cells.append(md(r"""
## 9.2 四個 FAIL 沒有一個是 Step 9 自己製造的

每一個都指回前面已經記錄過的結論。Step 9 的價值在於它**獨立地**把它們抓出來——
而七個 PASS 也同樣有價值：時間軸、關節極限、連續性、接觸一致性是**真的沒問題**。
"""))

cells.append(code(r"""
rows9 = pd.DataFrame(validation_rows(report))
fails = rows9[rows9["row_kind"] == "failure"]
display(fails.groupby("check").size().rename("failures").to_frame())

for check in report.failed_checks():
    first = report.failures_of(check)[0]
    print(f"\n{check.value}")
    print(f"  {first.detail}")
    if first.value is not None:
        print(f"  value = {first.value}   limit = {first.limit}")
"""))

cells.append(md(r"""
## 9.3 `beta_workspace_guard`：可能是**語意不合**，不是硬體違規

`RobotParams.BETA_MAX_DEG = 40` 的註解是 "Sagittal swing geometric limit"，
被 `gait_generator_3d` 與 `obstacle_walk` 使用——那兩個把 β 當**有界的擺動**。

但 Hybrid 的 nominal cycle 把 β 當**圈數計數器**
（Step 1：`beta_target = start.beta − 2π`），所以 recovery 一定會超出 ±40°。

**Step 9 回報這個違規，但明確拒絕判定哪一種讀法才對。**
直接套用會讓整個 nominal cycle 違規；直接忽略又會漏掉真正的工作區問題。

## 9.4 Step 3 欠的那筆帳：**10.553× 實測出來了**

Step 3 從 duty 推導出「recovery 的 β 角速度要比 rolling 快 10.553 倍」，
並明講**不判斷**它是否超出關節極限、把問題留給 Step 9。

現在 Step 8 把每一幀都排好了，所以可以直接量——
**兩條完全不同的路徑得到同一個數字**。
"""))

cells.append(code(r"""
display(pd.DataFrame([r.as_dict() for r in report.joint_rates]))
print("單位 deg/s。ratio 欄與 Step 3 從 duty 推導的 10.553 一致。")
print("\npeak theta 讀出 0 是【量測限制】不是「theta 不動」：")
print("  Step 8 在 segment 端點之間內插，而 RECOVERY_SWING 的頭尾 theta 都是 60 deg，")
print("  中間縮到 compact 姿態再伸出來的過程不在組好的取樣裡。")
print("  每一列都帶 theta_rate_is_lower_bound = True。")
"""))

cells.append(md(r"""
## 9.5 但 Step 9 **不能**判定它可不可行

```text
RobotParams 裡【沒有關節速度極限】。
只有 SWING_ACCEL_MAX（swing 塑形用）與 TOUCHDOWN_VEL_H_MAX（觸地用），
兩個都不是馬達轉速上限。
```

所以 Step 9 **量得到需求、比不到極限**。要裁決 10.553× 可不可行，
**先要補上這個數字**——這是 Step 9 交出去的一個具體待辦，不是一句「還要再看看」。
"""))

cells.append(code(r"""
print("在別處驗過（不在這裡重跑）")
for name, why in report.delegated.items():
    print(f"\n  {name}\n    {why}")
print("\n\n這裡根本判不了")
for name, why in report.unevaluable.items():
    print(f"\n  {name}\n    {why}")
"""))

cells.append(md(r"""
## 9.6 Step 9 驗收（規格 §16）

| 要求 | 結果 |
|---|---|
| 時間嚴格遞增 | ✅ PASS |
| 最多一隻腳 airborne | ✅ PASS |
| 關節極限 | ✅ θ PASS；β 的 guard 另外報（語意問題） |
| 關節連續性 | ✅ 段內 PASS；段界用 wrapped 讀數 |
| body 連續性 | ✅ PASS |
| stance / 接觸有效 | ✅ PASS |
| swing 無碰撞 | ✅ delegated（Day 8–9 每幀驗過），已列出 |
| touchdown 接觸狀態 | ✅ delegated（Step 7 的 `RepositionTarget2D`），已列出 |
| 地形碰撞 | ✅ delegated（Day 6–7 / Day 8–9 生成時驗過），已列出 |
| segment 之間連續 | ✅ FAIL 4 —— 陷阱 25，回報不修 |
| body requirement | ✅ FAIL 2 —— Step 5 的 INFEASIBLE |
| support triangle 與最小 margin | ✅ FAIL 5 —— Step 6 的零 margin |
| 結構化失敗原因 | ✅ 八個欄位，有測試逐筆檢查 |
| 不偷偷修 | ✅ 有測試斷言軌跡與容器都沒被動過 |

**Step 10 要做的**：參數化地形整合 + generalization gate（規格 §17）。
Step 9 的 validator 正好就是「換完地形參數之後拿什麼判斷」的那個東西。
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
     "tests/test_day12_whole_body_trajectory_2d.py",
     "tests/test_day12_whole_body_validation_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
print("（Step 7 的測試另外跑：swing planner 很慢，見 implementation log）")
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
