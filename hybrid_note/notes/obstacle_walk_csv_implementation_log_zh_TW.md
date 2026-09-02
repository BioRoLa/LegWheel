# Obstacle Walk CSV 實作紀錄

建立日期：2026-08-31

實作 repo：`/home/chang/corgi_ws/LegWheel`

規劃文件：`/home/chang/corgi_ws/corgi-research/04_Corgi/24_Software/csv_generate/obstacle_walk_csv_plan.md`

## 這份筆記的用途

這是「已知單一矩形障礙物的 offline terrain-aware Walk CSV generator」的持續實作紀錄。

之後每完成一個 Step，都要在此追加：

- 實際做了什麼。
- 新增或修改哪些檔案。
- 現在已經能做什麼。
- 測試命令與結果。
- 尚未完成或不能宣稱的內容。
- 下一步從哪個介面接續。

這份筆記記錄的是 **Pure Walk obstacle baseline**，不要和 `hybrid_note` 內的 Hybrid planner 完成度混為一談。

---

## 目前整體狀態

| Step | 狀態 | 已有能力 |
| --- | --- | --- |
| Step 0 | 完成 | 固定目前 flat Walk 硬體 CSV 輸出作為 regression baseline |
| Step 1 | 完成 | `WalkState`、`TrajectorySegment`、flat Walk segment adapter |
| Step 2 | 完成 | lossless segment slicing、接點 continuity report 與安全拼接 |
| Step 3 | 完成 | 單一矩形障礙物的 1-D touchdown surface/edge-margin query |
| Step 4 | 完成 prototype | 靜態 body、單腳任意 world touchdown、Bezier/rim-point IK 與檢查介面 |
| Step 5 | 完成 prototype | 四腳 world-fixed ground/top 混合高度 stance 與顯式 body trajectory |
| Step 6 | 完成 prototype | 依現有 Walk phase 產生 event-aligned obstacle segment request list |
| Step 7 | 完成 prototype | 完整 obstacle-walk 軌跡組裝、驗證與 12 欄硬體 CSV/phase/metadata/validation 輸出 |
| Step 8 | 進行中 | 完整腿部幾何碰撞、支撐多邊形穩定度、逐 stage 驗證與可視化；已抓到兩個未通過項 |
| Step 9 | 未開始 | 模擬比較與實機前安全檢查 |

目前已可由單一指令產生「接近 → 四腳上障礙物 → 頂面行走 → 四腳下障礙物 → 平地恢復」的完整離線軌跡，並輸出硬體 12 欄 CSV、row-aligned phase CSV、metadata 與 validation report。

但必須注意 Step 7 產生的是 **quasi-static crawl**（世界固定接觸點、body 只在四腳支撐時前進），不是既有 `GaitGenerator3D` 的 periodic rolling Walk。兩者 stance law 不同，無法直接拼接。詳見 Step 7 章節。尚未完成模擬、完整幾何碰撞、穩定度與實機驗證。

---

# Step 0：Flat Walk baseline regression

完成日期：2026-08-31

## 做了什麼

鎖定目前正式硬體 CSV 路徑：

```text
GaitGenerator3D
→ generate_full_gait()
→ generate_hardware_csv()
→ 12-column hardware CSV + 4-column phase CSV
```

固定一組快速回歸參數：

```text
gait_type       = Walk
twist           = [wz=0.0, vx=0.05, vy=0.0]
stand_height    = 0.25 m
step_height     = 0.04 m
period          = 1.0 s
dt              = 0.01 s
n_cycles        = 1
stability_margin= 0.02 m
launch          = false
```

`dt=0.01 s` 只用於縮短 regression test 時間，不是實機預設的 `dt=0.001 s`。

## 新增檔案

- `tests/fixtures/flat_walk_baseline_v1.json`
- `tests/test_flat_walk_baseline_regression.py`

## Fixture 保存的證據

- 完整生成參數與重現指令。
- hardware 與 phase 欄序。
- 主 CSV shape：`(600, 12)`。
- phase CSV shape：`(600, 4)`。
- 500-row prep 與 100-row steady Walk。
- 首列、prep 最後一列、steady 第一列和末列。
- phase transitions 與各腳 stance/swing row count。
- 兩份 CSV 的 SHA-256，用來鎖住每一個六位小數輸出值。

觀察到的 swing event 順序：

```text
FL → RR → FR → RL
```

prep 最後一列和 steady 第一列完全相同。

## 測試

```bash
MPLCONFIGDIR=/tmp/legwheel_step0_flat_walk_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q tests/test_flat_walk_baseline_regression.py
```

結果：`2 passed`。

連同當時相關 gait tests：`7 passed`。

## 尚未完成

- 沒有 segment state。
- 沒有 obstacle 或 terrain query。
- 沒有不同高度 swing/stance。
- 沒有四腳越障排程。

---

# Step 1：WalkState、TrajectorySegment 與 flat adapter

完成日期：2026-08-31

## 做了什麼

新增一個和既有 Hybrid 2-D prototype 分開的 package：

```text
legwheel/planners/obstacle_walk/
├── __init__.py
├── types.py
└── flat_adapter.py
```

Step 1 只建立 segment contract 和 flat adapter。package 名稱代表後續用途，不代表 obstacle 功能已經完成。

## 固定的資料順序與座標語意

segment 內部 commands 固定為：

```text
shape = (N, 4, 3)
legs  = [FL, FR, RR, RL]
joints= [theta, beta, gamma]
```

這是 planner leg-major order。硬體的 12 欄 reorder 仍應只在最終 CSV exporter 做一次。

世界座標使用：

```text
+x forward
+y left
+z upward
body pose = [x, y, z, roll, pitch, yaw]
```

## 新增 `WalkState`

`WalkState` 保存下一段生成時需要的邊界狀態：

- 四腳關節角 `(4, 3)`。
- 上一列關節角，可供之後計算接點速度。
- body world pose `(6,)`。
- 各腳所選最低輪緣點的 world position `(4, 3)`。
- stance/swing phase `(4,)`。
- `contact_active`。
- 各腳 surface ID。
- gait cycle phase。
- 下一隻預計進入 swing 的腳。

所有 NumPy state arrays 都複製後設為 read-only，避免某一段意外改到前一段的 final state。

## 新增 `TrajectorySegment`

每段同步保存：

- `time_s`。
- `commands_rad`，shape `(N, 4, 3)`。
- `phase`，shape `(N, 4)`。
- `body_pose_world`，shape `(N, 6)`。
- `foot_contact_points_world_m`，shape `(N, 4, 3)`。
- `contact_active`，shape `(N, 4)`。
- 每列每腳的 `surface_ids`。
- `start_state` 與 `final_state`。
- `dt_s`、segment type、command order。

Constructor 會檢查：

- 所有 shape 與 finite values。
- phase 只能是 stance `0` 或 swing `1`。
- Step 1 flat Walk 中 `contact_active == (phase == 0)`。
- time 從零開始並以固定 `dt_s` 前進。
- start/final state 必須和 segment 首末 sample 完全一致。

提供：

```python
segment.to_planner_commands()  # 還原 writable (N, 12)
segment.to_phase_array()       # 還原 writable (N, 4)
```

## 新增 flat adapter

主要介面：

```python
generate_flat_walk_segment(generator, n_cycles=1)

flat_walk_segment_from_generator(generator)
```

第一個介面會先呼叫既有 `GaitGenerator3D.generate_full_gait()`，第二個介面包裝已經生成完成的 `CMDS/PHASE`。

flat adapter 做的事情：

1. 不修改既有 planner commands。
2. 將 `(N, 12)` reshape 成 `(N, 4, 3)`。
3. 保存完全相同的 phase。
4. 由 `v_com`、`omega_z`、`dt` 和初始 body pose 重建 constant-twist body world trajectory。
5. 使用現有 `CorgiLegKinematics.foot_rim_contact_fk()` 與 `forward_kinematics()`，計算各列各腳的最低輪緣點世界座標。
6. flat Step 1 的 surface ID 全部設為 `ground`。
7. 建立可供下一段使用的 start/final `WalkState`。

## 世界座標 metadata 的證據界線

這些 body pose 和 foot point 是由已知命令與 constant body twist **離線重建**：

- 它們不是 Webots ground truth。
- 它們不是實機量測。
- swing 時的最低輪緣點不是實際 terrain contact，因此 `contact_active=False`。
- Step 1 尚未執行 terrain query 或碰撞檢查。

後續 Step 5 若建立真正的 world-fixed stance contact，必須再依其物理 contract 更新 stance segment，不能把目前的 kinematic reconstruction 當成最後 contact model。

## 新增測試

- `tests/test_obstacle_walk_segment_contract.py`

測試內容：

- 現有 `CMDS` 經 adapter 後可逐值還原。
- 現有 `PHASE` 經 adapter 後可逐值還原。
- start/final state 對應首末列。
- final state 保存上一列關節角。
- body pose、foot point、phase、contact_active row-aligned。
- 支援指定初始 world pose。
- segment arrays 為 immutable。
- 回傳的 round-trip arrays 可寫入且不會改到 segment。
- 未先生成 gait 時拒絕。
- 非 Walk gait 拒絕。
- boundary state 和 sample 不一致時拒絕。

## 測試指令與結果

```bash
MPLCONFIGDIR=/tmp/legwheel_step1_walk_segment_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_segment_contract.py \
  tests/test_flat_walk_baseline_regression.py
```

結果：`8 passed`。

另外連同既有 gait-duty/swing-velocity 與 Hybrid planning contract tests 一起執行：`24 passed`。

## 現在已有的功能

```python
from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import generate_flat_walk_segment

generator = GaitGenerator3D(
    gait_type="Walk",
    stand_height=0.25,
    twist=[0.0, 0.05, 0.0],
    step_height=0.04,
    period=1.0,
    dt=0.01,
)

segment = generate_flat_walk_segment(generator, n_cycles=1)

planner_commands = segment.to_planner_commands()
phase = segment.to_phase_array()
next_state = segment.final_state
```

## 尚未完成

- 尚未拼接兩個 segment。
- 尚未檢查 segment boundary 的關節速度連續性。
- 尚未有障礙物或不同 surface height。
- 尚未有任意 touchdown target。
- 尚未有不同高度 stance。
- 尚未輸出 obstacle Walk 硬體 CSV。
- 尚未完成 Webots 或實機驗證。

## Step 2 的直接入口

Step 2 應以目前的：

```text
TrajectorySegment.final_state
TrajectorySegment.start_state
TrajectorySegment.to_planner_commands()
TrajectorySegment.to_phase_array()
```

實作 `concatenate_segments()` 與 continuity report，先做 flat + flat 的成功案例，以及位置、速度、phase、body pose、foot contact 不連續的拒絕案例。Step 2 不應開始 terrain 或 obstacle swing。

---

# Step 2：Segment 拼接與接點連續性檢查

完成日期：2026-08-31

## 做了什麼

新增：

```text
legwheel/planners/obstacle_walk/assembly.py
tests/test_obstacle_walk_segment_assembly.py
```

並對 Step 1 的 `TrajectorySegment` 增加逐列 `gait_cycle_phase`，確保切割和重新合併時不會遺失 gait cycle 位置。

## 固定的拼接語意

相鄰兩段必須共享一個端點：

```text
left segment : ... q[k-1], q[k]
right segment:     q[k], q[k+1], ...
```

接點通過後，assembler 只保存一次 `q[k]`：

```text
combined: ... q[k-1], q[k], q[k+1], ...
```

這代表目前的 `concatenate_segments()` 不是把兩份互不相關的 CSV 直接上下貼合；右段必須真的從左段 `final_state` 開始。

## 新增主要介面

```python
slice_segment(segment, start, stop)

validate_segment_boundary(
    left,
    right,
    tolerances=ContinuityTolerances(...),
)

concatenate_segments(
    segments,
    dt=None,
    tolerances=ContinuityTolerances(...),
)
```

`concatenate_segments()` 回傳：

```python
ConcatenationResult(
    segment=combined_segment,
    boundary_reports=(...),
)
```

因此後續除了拿到軌跡，也能保留每一個接點的驗證數值。

## `slice_segment()`

功能：

- 使用 Python-style `[start:stop]` sample 範圍。
- 每個 slice 至少保留兩列，才能計算接點速度。
- time 重新從 `0` 開始。
- 同步切割 commands、phase、body pose、foot points、contact active、surface IDs 和 gait-cycle phase。
- 自動重建 slice 的 start/final `WalkState`。
- 保存 boundary 前一列關節角與下一個 swing event metadata。

此功能目前主要用於證明「同一份 flat Walk 拆開後可以無損還原」，也可供後續 scheduler 在合法 gait event 切段。

## 接點檢查內容

`validate_segment_boundary()` 目前檢查：

1. 兩段 `dt` 相同。
2. command order 相同。
3. 關節位置連續。
4. 關節速度近似連續。
5. stance/swing phase 完全一致。
6. `contact_active` 完全一致。
7. body world position 連續。
8. body roll/pitch/yaw 連續。
9. 兩側都處於 active contact 的支撐腳，其 world foot point 連續。
10. active contact 的 surface ID 相同。

速度比較使用共享端點兩側的有限差分：

```text
left velocity  = (left[-1] - left[-2]) / dt
right velocity = (right[1] - right[0]) / dt
velocity error = abs(left velocity - right velocity)
```

這個數值同時包含離散加速度與真正的速度跳變，因此不應被解讀成純粹的瞬間物理速度不連續。

## 集中管理的預設容許值

```python
ContinuityTolerances(
    joint_position_rad=1e-9,
    joint_velocity_rad_s=0.5,
    body_position_m=1e-9,
    body_orientation_rad=1e-9,
    support_foot_position_m=1e-9,
)
```

`0.5 rad/s` 是目前 Step 2 regression 所使用的離散接點門檻，不是實機安全速度或最終最佳值。後續不同 `dt`、transition curve 或硬體限制需要重新驗證並明確傳入。

## Boundary report

每個 `BoundaryContinuityReport` 保存：

- 左右 segment index。
- 最大 joint-position error，以及 leg/joint 名稱。
- 最大 joint-velocity error，以及 leg/joint 名稱。
- 最大 body-position error 與 axis。
- 最大 body-orientation error 與 axis。
- 最大支撐腳 world-point error與 leg。
- phase/contact/surface 是否一致。
- `passed`。
- 所有 violation 文字。

例如關節跳號會明確報告：

```text
segment boundary 0 -> 1 failed:
joint position FL.theta error 0.01 rad > 1e-09 rad
```

而不是只回傳一個沒有位置資訊的 `False`。

## Flat + flat 無損還原案例

測試先產生兩個完整 flat Walk cycles，共 100 samples，再切成：

```text
left  = original[0:51]
right = original[50:100]
```

sample 50 是共享端點。合併後回到 100 samples，以下資料均和未切割版本逐值相同：

- commands。
- phase。
- body world pose。
- foot contact points。
- contact active。
- gait-cycle phase。
- surface IDs。

該 boundary 的實際最大離散 joint-velocity difference 為約 `0.161858 rad/s`，低於目前 regression tolerance `0.5 rad/s`。

## 失敗測試

測試會故意建立：

- `FL.theta` 關節位置跳 `0.01 rad`。
- `FR.beta` 關節速度跳變。
- FL phase 不一致。
- body `x` 跳 `0.02 m`。
- active support foot 跳 `0.015 m`。
- active support foot 的 surface ID 突然改變。
- 指定的 `dt` 和 segment 不同。
- 將 velocity tolerance 收緊為 `0.1 rad/s`。

以上案例都會在合併前拒絕，並指出 segment index、leg/joint 或 body axis、實際誤差與門檻。

## 測試指令與結果

```bash
MPLCONFIGDIR=/tmp/legwheel_step2_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_segment_assembly.py \
  tests/test_obstacle_walk_segment_contract.py \
  tests/test_flat_walk_baseline_regression.py
```

結果：`18 passed`。

另外連同既有 gait-duty/swing-velocity 與 Hybrid planning contract tests 一起執行：`34 passed`。

## 現在已有的功能

```python
from legwheel.planners.obstacle_walk import (
    concatenate_segments,
    slice_segment,
)

left = slice_segment(original, 0, split_index + 1)
right = slice_segment(original, split_index, original.sample_count)

result = concatenate_segments([left, right], dt=original.dt_s)
combined = result.segment
reports = result.boundary_reports
```

## 尚未完成

- 目前只驗證由既有 flat Walk 切出的 segments。
- 尚未有 obstacle terrain 或 touchdown surface query。
- 尚未生成 step-up、top-walk 或 step-down segment。
- `joint_velocity_rad_s=0.5` 尚未對實機安全限制校正。
- foot-point metadata 仍是 Step 1 的離線 kinematic reconstruction，不是模擬或量測接觸。
- 尚未驗證不同高度 stance contact。
- 尚未輸出完整 obstacle Walk CSV。

## Step 3 的直接入口

下一步只建立單一矩形障礙物的 terrain/touchdown query：

```text
nominal touchdown world x
→ ground / obstacle_top / NO_LEGAL_TOUCHDOWN
```

Step 3 應回傳 surface ID、高度、edge margin 與拒絕原因，不應直接生成 swing、IK、scheduler 或 CSV。

---

# Step 3：單一矩形障礙物與 touchdown query

完成日期：2026-08-31

## 做了什麼

新增：

```text
legwheel/planners/obstacle_walk/terrain.py
tests/test_obstacle_walk_terrain_query.py
```

這是第一版直線前進 `+x` 的一維地形 contract。它只查詢 nominal touchdown point 所屬的 surface，不產生任何關節命令。

## 新增地形資料結構

```python
RectangleObstacle1D(
    x_start_m=0.40,
    length_m=0.30,
    height_m=0.10,
    edge_margin_m=0.04,
    top_surface_id="obstacle_top",
)

WalkTerrain1D(
    obstacle=obstacle,
    ground_height_m=0.0,
    ground_surface_id="ground",
)
```

障礙物高度是相對 ground plane 的高度，所以 obstacle-top 世界高度為：

```text
ground_height_m + obstacle.height_m
```

第一版固定只有 flat ground 加上一個 rectangle。

## 合法頂面範圍

物理障礙物範圍：

```text
[x_start, x_end]
x_end = x_start + length
```

合法頂面 touchdown 是閉區間：

```text
[x_start + edge_margin, x_end - edge_margin]
```

以上範例是：

```text
physical obstacle = [0.40, 0.70] m
legal top         = [0.44, 0.66] m
```

`x=0.44` 和 `x=0.66` 都是合法頂面 touchdown。比較時使用 `1e-12 m` 的浮點邊界容差，避免 `0.40 + 0.04` 的二進位表示誤差把閉區間端點判錯。這個容差只是數值處理，不是額外的物理 clearance。

若：

```text
length <= 2 * edge_margin
```

就沒有正寬度的合法頂面區域，constructor 會直接拒絕。

## 新增 query

```python
query_touchdown_surface(terrain, nominal_x_world_m)
```

回傳 `TouchdownQueryResult`：

- `nominal_x_world_m`。
- `status`。
- `surface_id`。
- `surface_height_world_m`。
- `distance_to_nearest_obstacle_edge_m`。
- `required_edge_margin_m`。
- `rejection_reason`。
- `is_legal`。

## 三種結果

### 1. Ground

```text
status       = GROUND
surface_id   = ground
height       = ground_height_m
reason       = None
is_legal     = True
```

適用於 `x < x_start` 或 `x > x_end`。

### 2. Obstacle top

```text
status       = OBSTACLE_TOP
surface_id   = obstacle_top
height       = ground_height_m + obstacle.height_m
reason       = None
is_legal     = True
```

適用於合法頂面閉區間。

### 3. No legal touchdown

```text
status       = NO_LEGAL_TOUCHDOWN
surface_id   = None
height       = None
reason       = WITHIN_EDGE_MARGIN
is_legal     = False
```

適用於 physical top 內、但離前緣或後緣小於 `edge_margin_m` 的 touchdown。

Step 3 不會自行把這個 touchdown 移到安全位置。後續 scheduler 必須調整前面的步距、相位或回報沒有合法解。

## Point-query 的物理界線

目前查詢的是 nominal touchdown **點**：

- 障礙物前 `x < x_start` 的地面點仍回傳 ground。
- 障礙物後 `x > x_end` 的地面點仍回傳 ground。
- 即使地面點很靠近垂直面，Step 3 也不會用整個輪緣半徑把它拒絕。

因此：

```text
point-wise legal touchdown
!=
full wheel/leg collision-free trajectory
```

完整輪緣、腿部與障礙物垂直面的 overlap/collision 必須在後續 swing/trajectory geometry checker 驗證，不能把 Step 3 結果當作完整可走證明。

## 輸入驗證

以下輸入會拒絕：

- 非有限的 `x_start/length/height/edge_margin`。
- `length <= 0`。
- `height <= 0`。
- `edge_margin < 0`。
- top 太短：`length <= 2 * edge_margin`。
- 空的 surface ID。
- ground 和 obstacle top 使用相同 surface ID。
- query x 是 `NaN` 或正負無限大。

## 測試案例

- 障礙物前 ground。
- 前緣外側極近的 ground。
- 障礙物後 ground。
- 後緣外側極近的 ground。
- 合法頂面中央。
- 合法頂面左右閉區間端點。
- 前 edge margin。
- 後 edge margin。
- physical 前後角點。
- nonzero ground height。
- zero edge margin。
- 非法障礙物尺寸與高度。
- 沒有正寬度合法頂面的障礙物。
- surface ID 驗證。
- nonfinite touchdown query。

## 測試指令與結果

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_terrain_query.py \
  tests/test_obstacle_walk_segment_assembly.py \
  tests/test_obstacle_walk_segment_contract.py \
  tests/test_flat_walk_baseline_regression.py
```

結果：`42 passed`。

另外連同既有 gait-duty/swing-velocity 與 Hybrid planning contract tests 一起執行：`58 passed`。

## 現在已有的功能

```python
from legwheel.planners.obstacle_walk import (
    RectangleObstacle1D,
    WalkTerrain1D,
    query_touchdown_surface,
)

terrain = WalkTerrain1D(
    RectangleObstacle1D(
        x_start_m=0.40,
        length_m=0.30,
        height_m=0.10,
        edge_margin_m=0.04,
    )
)

result = query_touchdown_surface(terrain, nominal_x_world_m=0.50)

assert result.surface_id == "obstacle_top"
assert result.surface_height_world_m == 0.10
```

## 尚未完成

- query 尚未選擇或修正 foothold。
- 尚未支援多障礙物、斜面、凹洞或非矩形地形。
- 尚未考慮 touchdown 的 world `y` 或完整 3-D footprint。
- 尚未做輪緣、腿部或 body collision check。
- 尚未生成 step-up/step-down swing。
- 尚未做不同高度 stance。
- 尚未建立四腳 scheduler。
- 尚未輸出 obstacle Walk CSV。

## Step 4 的直接入口

Step 4 可以使用：

```text
TouchdownQueryResult.surface_height_world_m
TouchdownQueryResult.surface_id
```

建立指定 world-frame `(x, y, z)` 的單腳 swing。必須沿用現有 3-D rim material point、Bezier 與 IK，並區分 contact-point apex 和完整腿/輪緣 collision clearance。Step 4 不應開始四腳 scheduler。

---

# Step 4：任意 world-frame touchdown 的單腳 swing prototype

完成日期：2026-08-31

## 完成狀態

Step 4 已完成的是：

```text
static-body
single-leg
contact-to-contact
3-D Cartesian Bezier + fixed rim material point + existing IK
trajectory prototype
```

它尚未是四腳 obstacle Walk，也尚未通過內建完整腿/輪緣碰撞檢查，因此狀態表標成「完成 prototype」。

## 新增檔案

```text
legwheel/planners/obstacle_walk/swing.py
tests/test_obstacle_walk_swing_segment.py
examples/gait/generate_obstacle_walk_step4_artifacts.py
```

並在 `SegmentType` 新增：

```python
SegmentType.SWING
```

## 主要介面

```python
generate_swing_segment(
    generator,
    start_state,
    swing_leg,
    touchdown_world_m,
    terrain,
    clearance_m,
    tracking_tolerance_m=1e-3,
    touchdown_height_tolerance_m=1e-3,
    maximum_joint_step_rad=0.35,
    contact_path_clearance_tolerance_m=1e-4,
    full_geometry_collision_checker=None,
)
```

回傳 `SwingPlanResult`，其中包含：

- `TrajectorySegment`。
- swing leg。
- touchdown world XYZ 和 target surface ID。
- 固定追蹤的 touchdown rim alpha，單位是 degree。
- 每列 Cartesian Bezier target。
- 每列實際 rim-point FK position。
- 每列 IK/FK tracking error。
- corridor 內最高地形。
- requested/achieved apex。
- 是否真的執行了外部 full-geometry checker。

## 受控假設

Step 4 swing 期間：

- body pose 固定。
- 另外三腳的 joint commands 固定。
- 另外三腳的 contact metadata 固定。
- 只有指定的一隻腳進入 swing。
- 起點與 touchdown sample 都是 stance，中央 samples 是 swing。

這讓 Step 4 可以先隔離 swing/IK 問題，但尚未處理 body 向前走、混合高度 stance 或三支撐腳為了維持 world contact 而需要的 joint motion。那些屬於 Step 5。

## 沿用的現有元件

沒有建立第二套簡化腿模型。實際沿用：

- `GaitGenerator3D` 內每腳的 `TrajectoryPlanner3D`。
- 現有 `SwingLegPlanner.solveSwingTrajectory()`。
- 現有 12-control-point 3-D Bezier profile。
- `CorgiLegKinematics.foot_rim_contact_fk()`。
- `CorgiLegKinematics.forward_kinematics()`。
- `CorgiLegKinematics.inverse_kinematics()`。
- 現有 joint-limit 參數。

## touchdown rim material point

先用 target world position 轉到固定 body frame，再反覆執行：

```text
固定 alpha 做 IK
→ 由 IK 結果查詢最低輪緣 alpha
→ 更新 alpha 再做 IK
→ 直到 alpha 收斂
```

得到 touchdown `q` 和 `alpha_td` 後，整個 swing 都追蹤同一個：

```text
(alpha_td, w=0)
```

這是「將在 touchdown 接觸的 rubber material point」。在 liftoff pose 中，同一個 material point 不一定是當下最低接觸點，因此圖中的 swing 起點可能高於 ground/top。這不是把腳瞬間移高，而是現有 Walk 所使用的 virtual-liftoff material-point 語意。

`TrajectorySegment.foot_contact_points_world_m` 仍保存每個姿態的最低輪緣點；`SwingPlanResult.cartesian_actual_world_m` 才是固定 `alpha_td` material point 的實際 FK path。兩者不可混用。

## terrain-aware apex

先找 liftoff 到 touchdown world-X corridor 內的最高地形：

```text
z_terrain_max = ground 或 obstacle top
```

再設定：

```text
z_apex = max(
    virtual_liftoff_z,
    velocity_continuity_sample_z,
    touchdown_z,
    z_terrain_max,
) + clearance_m
```

`clearance_m=0.03 m` 在目前 artifacts 中仍只是 nominal prototype parameter，不是硬體安全 clearance。

## Step 2 邊界銜接

第一版直接使用零 liftoff velocity 時，雖然 joint position 完全連續，但第一個 joint finite-difference 和前一段相差約：

```text
3.8 rad/s
```

因此修改為：

```text
q_continuity = q_start + (q_start - q_previous)
```

第一個 swing interval 精確沿用 `WalkState.previous_joint_position_rad` 的離散 joint velocity，再從這個 continuity sample 開始 Bezier。

touchdown 端則保存兩個相同的 target samples：

```text
..., q_touchdown, q_touchdown
```

使輸出端 joint command velocity 為零。

實際測試：

- flat segment → step-up segment 的 joint position error 是 `0`。
- selected FL 的進入 joint velocity 逐值相同。
- 完整 Step 2 boundary report 通過。
- boundary 最大離散速度差約 `0.162 rad/s`，來自 Step 4 受控假設中其餘支撐腳由原 flat motion 轉為固定 command，低於目前 Step 2 prototype tolerance `0.5 rad/s`。

這仍不是實機速度安全證明。

## phase 與接觸語意

以 14-sample 測試軌跡為例，指定腳的 phase 是：

```text
[stance,
 swing, swing, ..., swing,
 stance, stance]
```

- 第一列是共享起點。
- 中間是 swing。
- 倒數第二列到達 touchdown 並保持。
- 最後一列提供零 outgoing command velocity。
- 其他三腳全程 stance。

## 已有檢查

### Terrain 與 endpoint

- start contact 必須符合 Step 3 query surface 和高度。
- target x 不可落在 edge margin。
- target z 必須符合 queried surface height。

### IK 與 joint

- 每列 IK 必須收斂。
- maximum Cartesian IK/FK error 預設 `< 1 mm`。
- 單列 joint step 預設 `< 0.35 rad`。
- theta、beta、gamma 必須在目前 RobotParams limits 內。

### Tracked material-point path

- obstacle footprint 內的中間 material-point samples 必須高於 top。
- path segment 穿越前後 vertical face 時，插值交點必須高於 obstacle top。
- touchdown/hold samples允許接觸目標 surface。

這仍只是被追蹤 material point 的 path check，不是完整輪緣或 linkage collision check。

## 完整幾何 collision checker 介面

新增 `FullGeometryCollisionChecker` protocol。每個 sample 會傳入：

- sample index。
- leg ID。
- 該腳 joint position。
- body world pose。
- terrain。

checker 回傳 `None` 表示該 sample 通過，回傳文字則產生：

```text
FULL_GEOMETRY_COLLISION
```

若沒有提供 checker：

```python
plan.full_geometry_collision_checked == False
```

目前 repo 內尚未把一個實際完整 3-D leg/wheel-vs-rectangle checker 接到此 protocol。測試只驗證 checker 可以拒絕指定 sample，以及無碰撞 checker 可以讓流程完成；不能因此宣稱內建完整幾何已驗證。

## 四種起訖高度測試

使用：

```text
H = 0.04 m
clearance = 0.03 m
dt = 0.02 s
FL swing
```

已測試：

- `0 → 0` flat swing。
- `0 → H` step-up。
- `H → H` obstacle-top swing。
- `H → 0` step-down。

四種皆得到有限且連續的 joint trajectory，maximum IK/FK tracking error 均低於 `1 mm`。

這裡選 `H=0.04 m` 是 Step 4 static-body regression case。它不是最終比較用障礙物高度，也不能推廣成 `H=0.10 m` 的下降已可行；固定 body 在較高 step-down 可能超出腿部 workspace，Step 5 必須處理 body trajectory。

## 明確失敗原因

`SwingPlanningError.reason` 目前可包含：

```text
INVALID_START_CONTACT
ILLEGAL_TOUCHDOWN
TARGET_HEIGHT_MISMATCH
IK_UNREACHABLE
JOINT_LIMIT
JOINT_DISCONTINUITY
CARTESIAN_TRACKING_ERROR
CONTACT_PATH_COLLISION
FULL_GEOMETRY_COLLISION
```

測試包含：

- target 落在 edge margin。
- target z 和 surface height 不同。
- touchdown 超出 workspace。
- material-point path clearance 門檻不足。
- external full-geometry checker 強制回報 sample collision。

## 可重現 artifacts

生成指令：

```bash
MPLCONFIGDIR=/tmp/legwheel_step4_artifact_mpl \
.venv/bin/python \
  examples/gait/generate_obstacle_walk_step4_artifacts.py \
  --output-dir outputs/obstacle_walk_step4
```

輸出：

```text
outputs/obstacle_walk_step4/step4_step_up.png
outputs/obstacle_walk_step4/step4_step_down.png
outputs/obstacle_walk_step4/step4_summary.json
```

Step-up artifact：

```text
touchdown z              = 0.04 m
requested apex           = 0.07 m
achieved apex            = 0.07432 m
maximum IK/FK error      = 0.094 mm
full geometry checked    = false
```

Step-down artifact：

```text
touchdown z              = 0.00 m
requested apex           = 0.08123 m
achieved apex            = 0.08386 m
maximum IK/FK error      = 0.132 mm
full geometry checked    = false
```

圖中的藍色虛線是 Cartesian target，橘色線是固定 rim material point 的實際 FK；圖沒有畫完整腿，不能作為 full-leg collision-free 圖證。

## Step 4 測試

```bash
MPLCONFIGDIR=/tmp/legwheel_step4_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_swing_segment.py
```

結果：`12 passed`。

連同 Step 0–3、既有 gait-duty/swing-velocity 與 Hybrid planning contract tests 一起執行：`70 passed`。

## 現在已有的功能

```python
plan = generate_swing_segment(
    generator,
    start_state,
    swing_leg="FL",
    touchdown_world_m=[x_td, y_td, z_td],
    terrain=terrain,
    clearance_m=0.03,
)

swing_segment = plan.segment
tracking_error = plan.maximum_tracking_error_m
```

## 尚未完成

- body 在 swing 中固定，尚未生成 body forward/height/pitch trajectory。
- 其他三腳 joint commands 固定，尚未解 world-fixed mixed-height stance。
- 內建 full 3-D wheel/link/body collision checker 尚未接上。
- `0.03 m` clearance 尚未做 minimum-safe 或硬體校正。
- 目前四高度 regression 使用 `H=0.04 m`，不是最終 0.10 m obstacle 證據。
- 尚未做四腳 sequential step-up/step-down。
- 尚未建立 obstacle scheduler。
- 尚未輸出完整 obstacle Walk CSV。

## Step 5 的直接入口

Step 5 應從 `SwingPlanResult.segment.final_state` 與其他三腳的 world contacts 出發，明確定義 body trajectory，讓 ground/top 混合接觸的 stance legs 維持 world contact。不能繼續把其他三腳 joint commands 固定當成最終 stance model。

---

# Step 5：Ground/top 混合高度 stance segment prototype

完成日期：2026-08-31

## 完成狀態

Step 5 已完成的是一個受控的 quasi-static stance primitive：

- 四腳全部維持 stance。
- 每一腳沿用輸入 `WalkState` 的 world-frame 最低輪緣接點。
- 接點可以同時分布在 ground 與 obstacle top。
- body 從起始 pose 移動到呼叫端指定的終點 pose。
- 每一列用現有 3-D 最低輪緣 FK、rim material point 與 IK 重算四腳 command。
- 輸出逐列實際接點、contact drift、rim alpha/width、surface ID 與 body trajectory metadata。

它不是完整 Walk stance controller、rolling Jacobian、動態穩定規劃或四腳 obstacle scheduler，因此狀態仍標成「完成 prototype」。

## 新增或修改的檔案

新增：

- `legwheel/planners/obstacle_walk/stance.py`
- `tests/test_obstacle_walk_stance_segment.py`
- `examples/gait/generate_obstacle_walk_step5_artifacts.py`

修改：

- `legwheel/planners/obstacle_walk/types.py`
  - `SegmentType` 新增 `STANCE`。
- `legwheel/planners/obstacle_walk/__init__.py`
  - export Step 5 result、error、generator 與 plotting API。
- 本紀錄檔。

## Body trajectory 的明確假設

第一版不自動規劃 body height 或 pitch。呼叫端必須明確提供：

```python
target_body_pose_world = [x, y, z, roll, pitch, yaw]
```

Step 5 對 world position 使用 cubic smoothstep：

```text
s(u) = 3 u^2 - 2 u^3,  u in [0, 1]
```

position 逐軸插值；roll/pitch/yaw 各自沿最短角差插值。這是可檢查的 Euler-angle prototype，不是一般 SO(3) geodesic planner。

metadata 固定保存：

```text
body_trajectory_assumption = world_pose_cubic_smoothstep_with_endpoint_holds
requested_motion_duration_s
requested_body_pose_world
```

起點與終點各加入一列 hold，讓離散 command 的進入與離開速度為零。因此 `motion_duration_s` 只代表 smoothstep motion interval；整個 segment 另外包含兩個 `dt` 的 endpoint holds。例如：

```text
motion_duration = 0.30 s
dt              = 0.02 s
sample_count    = 18
segment end time= 0.34 s
```

## World-fixed stance 計算

每一腳先固定起點接觸：

```text
p_contact_target^W = start_state.foot_contact_points_world_m[leg]
```

每個 body sample 再計算：

```text
p_contact^B(t) = R_WB(t)^T (p_contact_target^W - p_body^W(t))
```

接著使用目前 `CorgiLegKinematics`：

1. 從前一列 command 取得最低 rim 的 `alpha`、`w`。
2. 對固定的 body-frame target 做 existing numerical IK。
3. 用新 command 重新計算最低 rim `alpha`、`w`。
4. 重複直到實際 lowest-rim FK 接近 world target。
5. 逐列保存實際 contact 與 drift。

沒有另寫簡化腿模型。

## 目前 API

```python
plan = generate_stance_segment(
    generator,
    start_state,
    target_body_pose_world,
    terrain,
    motion_duration_s=0.30,
)

stance_segment = plan.segment
max_drift = plan.maximum_contact_drift_m
body_assumption = plan.body_trajectory_assumption
```

`StancePlanResult` 目前包含：

```text
segment
contact_targets_world_m
contact_actual_world_m
contact_drift_m
rim_alpha_deg
rim_width_m
requested_body_pose_world
body_trajectory_assumption
requested_motion_duration_s
full_geometry_collision_checked
```

## Surface 與 contact drift 檢查

開始前，四個接點都必須：

- 通過 Step 3 terrain query。
- `surface_id` 和 `WalkState.surface_ids` 相同。
- world z 和該 surface height 的差小於門檻。

生成後，每一列實際 lowest-rim FK 都要：

- 和固定 world target 的距離預設 `< 1 mm`。
- query 後仍屬於原 surface。
- 沒有進入 obstacle edge margin。
- world z 仍符合 ground 或 top 高度。

為維持 Step 2 的 shared-boundary exact metadata，segment 第一列直接保留輸入 `WalkState` 的接點；獨立重算的第一列實際 FK 仍保存在 `contact_actual_world_m`，而且仍參與 drift 檢查，沒有被當作零誤差隱藏。

這些檢查能證明被追蹤的最低輪緣點沒有被拉離指定 surface；若沒有外接完整幾何 checker，不能證明整個輪緣、linkage 或 body 沒碰到障礙物側面。

## 拒絕原因

`StancePlanningError.reason` 目前可包含：

```text
INVALID_START_CONTACT
IK_UNREACHABLE
JOINT_LIMIT
JOINT_DISCONTINUITY
CONTACT_DRIFT
CONTACT_SURFACE_MISMATCH
FULL_GEOMETRY_COLLISION
```

joint limits 沿用目前 `RobotParams`：

- theta 的 min/max。
- beta 的 `±BETA_MAX_DEG`。
- gamma 的 `±GAMMA_MAX_DEG`。

單列 joint step 預設必須 `< 0.35 rad`。

## 驗收案例

受控 regression 使用：

```text
obstacle height = 0.04 m
dt              = 0.02 s
body delta x    = +0.01 m
body delta z    = +0.003 m
body delta pitch= +0.5 deg
motion duration = 0.30 s
```

已通過：

- 四腳都在 ground。
- FL 在 top，其餘三腳在 ground。
- FL/FR 在 top，RR/RL 在 ground。
- 四腳都在 top。
- mixed-height body target 超出 workspace 時明確拒絕。
- 人為縮小 joint limit 時回報 `JOINT_LIMIT`。
- external full-geometry checker 可拒絕指定 leg/sample，或標記已檢查。

上述 `H=0.04 m` 是 Step 5 regression case，不是 `H=0.10 m` 或硬體安全證據。

## Step 4 → Step 5 拼接

測試直接使用 FL step-up 的：

```text
SwingPlanResult.segment.final_state
```

作為 stance 起點。結果：

- shared joint position error 是 `0`。
- shared support-foot metadata error 是 `0`。
- phase、contact active 與 active surface 全部相同。
- 完整 Step 2 boundary report 通過。

這證明目前兩個 offline primitives 可依 Step 2 契約拼接；不代表已經有自動四腳順序。

## 可重現 artifact

生成指令：

```bash
MPLCONFIGDIR=/tmp/legwheel_step5_artifact_mpl \
.venv/bin/python \
  examples/gait/generate_obstacle_walk_step5_artifacts.py \
  --output-dir outputs/obstacle_walk_step5
```

輸出：

```text
outputs/obstacle_walk_step5/step5_two_top_stance.png
outputs/obstacle_walk_step5/step5_summary.json
```

artifact 情境：

```text
surface IDs             = FL top, FR top, RR ground, RL ground
body delta              = x +0.01 m, z +0.003 m, pitch +0.5 deg
maximum contact drift   = 0.785 mm
full geometry checked   = false
```

圖只呈現 world X-Z 的 body path、矩形地形與四個固定 contact target；沒有畫完整機器人幾何。

## 測試

Step 5 focused test：

```bash
MPLCONFIGDIR=/tmp/legwheel_step5_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_stance_segment.py
```

結果：`9 passed`。

連同 Step 0–4、既有 gait-duty/swing-velocity 與 Hybrid planning type tests：

```text
79 passed in 27.52s
```

另外已通過 Python compile、`git diff --check` 與本次新增檔案的 99-column 檢查。

## 現在已有的功能

目前已有可拼接的 primitive：

```text
flat segment
→ arbitrary world touchdown swing
→ four-leg world-fixed mixed-height stance
```

其中每一段都能把 `final_state` 傳給下一段，並保留 joint、body、phase、surface 與 world contact metadata。

## 尚未完成

- 尚未在一個 stance segment 中允許部分腳 swing；目前四腳都必須 stance。
- stance 是逐列 quasi-static IK，沒有沿用 periodic Walk 的 rolling Jacobian/contact rolling law。
- body height/pitch 是呼叫端指定，尚未自動依 stability 或 workspace 規劃。
- 沒有 CoM/support polygon、torque、摩擦、受力或動態穩定檢查。
- 內建完整 3-D wheel/link/body collision checker 尚未接上。
- 尚未驗證 0.10 m obstacle、Webots 或硬體。
- 尚未自動決定 flat approach 時間、腳序與 touchdown。
- 尚未輸出完整 obstacle Walk CSV。

## Step 6 的直接入口

Step 6 可以使用目前的：

```text
query_touchdown_surface()
generate_swing_segment()
generate_stance_segment()
WalkState / TrajectorySegment
```

建立 deterministic 四腳 obstacle event scheduler。scheduler 應輸出 segment request list，決定合法 liftoff 切點、腳序與 touchdown；這一階段不要直接把完整 CSV 生成邏輯塞進 scheduler。

---

# Step 6：四腳 obstacle event scheduler prototype

完成日期：2026-08-31

## 完成狀態

Step 6 現在可以從：

```text
GaitGenerator3D Walk parameters
initial WalkState / gait_cycle_phase
single RectangleObstacle1D
```

產生 deterministic、對齊 Walk liftoff/touchdown event 的 segment request list。

scheduler 只決定：

- 障礙物前的 flat approach 要走到哪個 liftoff event。
- 每個未來 swing event 是哪隻腳。
- nominal touchdown 的 world `(x, y, z)`。
- touchdown 位於 ground 或 obstacle top。
- edge-margin touchdown 是否能用有限 x bias 修復。
- swing 是 step-up、top swing、step-down、clear-over 或 ground swing。
- 全腳離開障礙物後的 flat recovery 要補幾個完整 gait cycles。

它不生成 joint commands，也不直接輸出 CSV，因此仍標成「完成 prototype」。

## 新增或修改的檔案

新增：

- `legwheel/planners/obstacle_walk/scheduler.py`
- `tests/test_obstacle_walk_scheduler.py`
- `examples/gait/generate_obstacle_walk_step6_artifacts.py`

修改：

- `legwheel/planners/obstacle_walk/__init__.py`
  - export scheduler request/result/error API。
- 本紀錄檔。

## 第一版運動範圍

目前 scheduler 明確限制為：

```text
gait_type = Walk
vx > 0
vy = 0
wz = 0
body roll/pitch/yaw = 0
world y 固定
單一 world-X 矩形障礙物
initial surface = 四腳 ground
initial phase = 四腳 stance 的合法 event boundary
```

若輸入 lateral walk、turning、非零 body orientation 或障礙物已不在全部腳前方，會以明確 reason 拒絕。這個限制符合目前 Step 3 的 1-D terrain contract；不能把它推廣成任意 3-D terrain scheduler。

## 腳序與 event 的來源

scheduler 沒有寫死腳序。它直接使用：

```text
generator.phase_offsets
generator.stance_duty
generator.T
generator.dt
initial_state.gait_cycle_phase
```

建立與 `GaitGenerator3D.generate_full_gait()` 相同的離散 stance/swing phase pattern，然後只在：

```text
phase: stance(0) → swing(1)
```

時建立 liftoff event，並向後找到同一腳：

```text
phase: swing(1) → stance(0)
```

作為 touchdown event。

測試除了驗證目前 Walk 的循環順序，也實際修改 `generator.phase_offsets`；輸出的 request 腳序會跟著改變，證明不是把 `FL → RR → FR → RL` 硬編碼在 scheduler 裡。

## Nominal touchdown 計算

每隻腳的 body-frame touchdown template 沿用現有 Walk planner：

```text
TrajectoryPlanner3D._level_touchdown_q()
→ existing lowest-rim foot_rim_contact_fk()
→ existing forward_kinematics()
```

直線 Walk 的 nominal world-X touchdown 為：

```text
x_td_nominal = x_body_initial + vx * t_touchdown + x_td_body_template
```

world y 使用固定 body y 加上該腳 touchdown template y。z 不自行猜測，而是呼叫 Step 3：

```python
query_touchdown_surface(terrain, x_td_nominal)
```

選出 ground 或 obstacle-top surface height。

這些是 nominal periodic-Walk predictions，尚未使用 Step 4/5 逐段生成後的 actual final state 修正。

## Request 類型

`ScheduleRequestKind` 包含：

```text
FLAT_APPROACH
STEP_UP
TOP_SWING
STEP_DOWN
CLEAR_OVER
GROUND_SWING
FLAT_RECOVERY
```

其中：

- `FLAT_APPROACH` 吸收障礙物前所有普通 flat Walk，終點一定是第一個 obstacle-interacting liftoff。
- `STEP_UP` 表示 ground → obstacle top。
- `TOP_SWING` 表示 obstacle top → obstacle top。
- `STEP_DOWN` 表示 obstacle top → ground。
- `CLEAR_OVER` 表示起訖都在 ground，但 swing corridor 跨過短障礙物。
- `GROUND_SWING` 表示 traversal 已開始後，其他尚未到達或已離開障礙物的普通 ground swing。
- `FLAT_RECOVERY` 從最後一腳離開障礙物後開始，向上取整成完整 gait cycles。

短障礙物允許沒有四腳同時在頂面，甚至可由某腳 `CLEAR_OVER`；scheduler 不會為了湊出四腳 top 而改寫 Walk phase。

## Request contract

`FlatSegmentRequest` 保存：

```text
kind
start_time_s / end_time_s / duration_s
end_event_leg
end_at_liftoff
aligned_cycle_count
```

`SwingSegmentRequest` 保存：

```text
kind / event_index / leg
liftoff_time_s / touchdown_time_s
liftoff_sample_offset / touchdown_sample_offset
gait_cycle_phase_at_liftoff
nominal_touchdown_world_m
touchdown_world_m
touchdown_bias_x_m
from_surface_id / target_surface_id
```

`ObstacleWalkSchedule` 另外保存：

```text
requests
swing_requests
cycle_sample_count
first_transition_event_index
completion_event_index
maximum_top_contact_count
all_four_top_observed
requested_post_distance_m
scheduled_recovery_distance_m
kinematic_feasibility_checked = false
full_geometry_collision_checked = false
```

最後兩個 flag 固定為 `false`，避免把 request planning 說成 trajectory feasibility 或碰撞驗證。

## Edge-margin 修復與失敗

nominal touchdown 若落在 obstacle footprint 內的 edge margin：

1. 找最近的 legal top boundary。
2. 計算所需 `touchdown_bias_x_m`。
3. bias 不超過 `maximum_touchdown_bias_m` 才接受。
4. 再呼叫 Step 3 query 確認調整後 touchdown 合法。

若所需 bias 太大，回報：

```text
NO_LEGAL_TOUCHDOWN
event_index
leg
nominal_touchdown_x_world_m
```

目前只調整 x touchdown request，尚未重新最佳化前一段 body speed、step length 或 stability；真正 IK 可行性要在 Step 7 生成時驗證。

## 其他明確失敗原因

`ObstacleScheduleError.reason` 可以是：

```text
INVALID_INITIAL_STATE
UNSUPPORTED_MOTION
OBSTACLE_NOT_AHEAD
NO_LEGAL_TOUCHDOWN
NO_OBSTACLE_INTERACTION
MAX_EVENTS_EXCEEDED
```

每個 event failure 能保存 event index 與 leg，避免只有「無解」而不知道失敗在哪一步。

## Recovery 對齊

指定 `post_distance_m` 後，recovery 使用：

```text
recovery_cycles = ceil(post_distance / (vx * period))
```

因此 recovery 不會在 gait cycle 中途結束。輸出同時保留 requested distance 與向上取整後的 scheduled distance。

這是 nominal flat recovery 時間；Step 7 仍需確認實際生成 segment 的起訖 phase 與 row 數。

## 可重現 artifact

生成指令：

```bash
MPLCONFIGDIR=/tmp/legwheel_step6_artifact_mpl \
.venv/bin/python \
  examples/gait/generate_obstacle_walk_step6_artifacts.py \
  --output-dir outputs/obstacle_walk_step6
```

輸出：

```text
outputs/obstacle_walk_step6/step6_schedule.json
```

artifact 輸入：

```text
initial body x          = 0.05 m
vx                      = 0.05 m/s
period                  = 1.0 s
dt                      = 0.02 s
obstacle x start        = 0.65 m
obstacle length         = 0.35 m
obstacle height         = 0.04 m
edge margin             = 0.02 m
requested post distance = 0.30 m
```

artifact 結果：

```text
cycle samples                 = 50
first obstacle event index    = 26
first obstacle liftoff leg    = FR
flat approach duration        = 6.52 s
completion event index        = 97
maximum simultaneous top legs = 2
all four top observed          = false
request count                  = 74
scheduled recovery cycles      = 6
scheduled recovery distance    = 0.30 m
kinematic feasibility checked  = false
full geometry checked          = false
```

74 requests 的分類：

```text
flat approach = 1
step up       = 4
top swing     = 24
step down     = 4
ground swing  = 40
flat recovery = 1
```

這個障礙物長度小於前後腳的 longitudinal separation，因此 front legs 已下降後 rear legs 才踏上，最大 top contact 只有 2；這是此固定尺寸與速度下的 schedule 結果，不是 scheduler 失敗，也不能推廣到其他尺寸。

## 測試

Step 6 focused tests：

```bash
MPLCONFIGDIR=/tmp/legwheel_step6_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q \
  tests/test_obstacle_walk_scheduler.py
```

結果：`13 passed`。

涵蓋：

- 相同輸入得到完全相同 request list。
- event index 連續，approach 只在 liftoff 結束。
- 腳序來自現有 phase offsets。
- 修改 phase offsets 後腳序同步改變。
- 所有 touchdown 通過 Step 3 query 並避開 top edge margin。
- 不同 obstacle start 產生不同但 event-aligned approach duration。
- edge bias 成功與 `NO_LEGAL_TOUCHDOWN` 失敗。
- 短障礙物 `CLEAR_OVER` 且沒有四腳同時 top。
- 長障礙物可觀察到四腳同時 top。
- recovery 向上取整成完整 gait cycles。
- phase mismatch、turning 與 event budget failure 有明確 reason。
- JSON summary 保留未驗證 flags。

連同 Step 0–5、既有 gait-duty/swing-velocity 與 Hybrid planning type tests：

```text
92 passed in 31.34s
```

另外已通過 Python compile、`git diff --check` 與本次新增檔案的 99-column 檢查。

## 現在已有的功能

目前可以先做純 request planning：

```python
schedule = schedule_obstacle_walk(
    generator,
    initial_state,
    terrain,
    post_distance_m=0.30,
    maximum_touchdown_bias_m=0.04,
)

for request in schedule.requests:
    print(request.kind)
```

輸出可經 `schedule_to_dict()` 轉成 JSON-safe metadata。

## 尚未完成

- request 尚未逐段呼叫 Step 1/4/5 generators。
- scheduler 的時間與 touchdown 是 nominal periodic-Walk prediction，尚未被 actual final state feedback 修正。
- Step 4 swing 目前 body 固定；直接把 request list 當成已執行軌跡不會讓 body 自動前進。
- Step 5 stance 目前四腳都必須 stance；Step 7 必須定義 swing 之間如何安排 body advance。
- touchdown x bias 尚未通過 IK、joint limit、stability 或完整 geometry feasibility。
- `CLEAR_OVER` 只代表 contact-point corridor 跨越障礙物，仍要由 Step 4 full trajectory/collision checks 驗證。
- 尚未支援 yaw、lateral walk、任意 body orientation、多障礙物或 2-D/3-D terrain map。
- 尚未輸出任何完整 obstacle Walk joint CSV。
- 尚未完成完整 traversal、Webots 或硬體驗證。

## Step 7 的直接入口

Step 7 應逐一消費：

```text
FlatSegmentRequest
SwingSegmentRequest
```

每一段都必須從上一段 actual `final_state` 生成，而不是只相信 Step 6 的 nominal 時間。生成後重新檢查 touchdown query、IK、joint limits、contact drift、完整 geometry hook 與 Step 2 boundary continuity，最後才一次轉換成 12-column hardware order 並輸出 CSV、phase CSV、metadata 與 validation report。

---

# Step 7：完整 obstacle-walk CSV generator

完成日期：2026-08-31

## 完成狀態

Step 7 已經可以用單一指令，從障礙物尺寸產生一份完整、通過接點驗證的離線軌跡與硬體 CSV：

```text
平地接近
→ 四腳依既有 Walk 腳序踏上障礙物
→ 頂面行走
→ 四腳踏回地面
→ 平地恢復
→ 12 欄硬體 CSV + row-aligned phase CSV + metadata JSON + validation JSON
```

它仍標成「完成 prototype」，因為：

- 產生的是 **quasi-static crawl**，不是既有 `GaitGenerator3D` 的 periodic rolling Walk。
- 內建完整 3-D 腿/輪/機身碰撞檢查仍未接上。
- 沒有支撐多邊形、CoM、力、摩擦或動態穩定檢查。
- 沒有 Webots 或實機驗證。

## 新增或修改的檔案

新增：

- `legwheel/planners/obstacle_walk/traversal.py`：Step 7 軌跡組裝器。
- `legwheel/planners/obstacle_walk/export.py`：硬體欄序、prep、CSV/metadata/validation 輸出。
- `examples/gait/generate_obstacle_walk_csv.py`：CLI。
- `tests/test_obstacle_walk_traversal.py`：Step 7 測試。

修改：

- `legwheel/planners/obstacle_walk/swing.py`
  - 新增 `swing_duration_s`：只改變 Bezier 的取樣密度，不改變路徑，用來壓低高低差大的 step-up/step-down 的單列 joint step。
  - 新增 `next_swing_leg`：讓腳序由呼叫端（來自 phase offsets）決定，不再只用寫死的 `_walk_successor`。
  - `_check_contact_path` 的逐點檢查改為從 sample 2 開始。sample 0 是共享 liftoff，sample 1 只是把上一段的離散 joint velocity 外插一步，兩者仍在 liftoff 接觸面上；原本的寫法會把「腳還站在頂面上」誤判成 `CONTACT_PATH_COLLISION`（實測誤差只有 1 µm）。跨越前後垂直面的插值檢查維持原樣，沒有放寬。
- `legwheel/planners/obstacle_walk/stance.py`
  - 新增 `initial_body_velocity_world_m_s`：body 位置改用 quintic Hermite，從指定的入口速度平滑減速到靜止，並取消起點 hold。未指定時行為與 Step 5 完全相同。
  - 新增 `BRAKING_BODY_TRAJECTORY_ASSUMPTION`，metadata 會標明用的是哪一種 body profile。
- `legwheel/planners/obstacle_walk/__init__.py`：export Step 7 API。
- 本紀錄檔。

Step 4/5 的既有測試（21 passed）在這些修改後仍全數通過。

## 關鍵發現一：既有 Walk 的 stance 是「滾動」的

實測既有 flat Walk（`vx=0.05`、`T=1.0`、`dt=0.02`）在一次 stance 內：

```text
body 前進      +0.036 m
FL 世界接觸點  0.2784 → 0.2954 m（前進 +0.017 m）
FL body-frame  0.2644 → 0.2454 m（後退 -0.019 m）
```

也就是說 `stance_rt_solver` 的 rolling Jacobian 讓輪子在 stance 期間**滾動**，世界接觸點會往前移動，body-frame 的後掃距離只有 body 前進量的約一半。

Step 5 的 stance 則是把世界接觸點**完全固定**。兩者是不同的接觸律，因此：

```text
periodic rolling Walk segment
+
Step 5 world-fixed stance segment
= 接點必然有 joint velocity 跳變
```

實測在 `vx=0.24 m/s` 下，即使先用 quintic Hermite 把 body 速度平滑降到零，接點最大離散 joint velocity 差仍達 `1.08 rad/s`，遠高於 Step 2 的 `0.5 rad/s`。

因此 Step 7 **沒有**把 periodic Walk 接在前面，而是整段（含接近與恢復）都用同一套 crawl primitives。這是刻意的取捨，不是疏漏；代價是 Step 7 的輸出不是 periodic Walk，好處是全部接點的 joint position 與 velocity 誤差都是 `0`。

`initial_body_velocity_world_m_s` 仍保留在 stance.py，供之後真的做出 rolling mixed-height stance 時使用。

## 關鍵發現二：輪緣讓「點合法」不等於「可落腳」

Step 3 只回答 nominal touchdown **點**的地形歸屬。但外徑 `R = 0.145 m` 的輪子，最低點著地時在高度 `h` 處的水平半寬是：

\[
\sqrt{2Rh-h^{2}}
\]

所以只要地面接觸點離高度 `H` 的垂直面比

\[
g=\sqrt{2RH-H^{2}}
\]

還近，輪緣就已經插進障礙物裡。實測第一版就是在這裡被既有 `_check_contact_path` 擋下來：某腳落在 `x_end + 0.0024 m`，sample 1 的被追蹤輪緣點在 `z = 0.0062 m`、`x = 0.99 m`，明確位於障礙物體內。

Step 7 因此在 Step 3 之上再加一層 ground exclusion band：

```text
合法 ground touchdown: x <= x_start - g  或  x >= x_end + g
合法 top touchdown   : [x_start + m, x_end - m]（沿用 Step 3 edge margin）
```

由此得到一個硬性下界：一步必須能從最後一個合法地面點跨到第一個合法頂面點，所以

\[
L_{step} \ge g + m
\]

| 障礙高度 H | 排除帶 g | 最小步長 g+m（m=0.02） |
| --- | --- | --- |
| 0.04 m | 0.1000 m | 0.1200 m |
| 0.05 m | 0.1095 m | 0.1295 m |
| 0.06 m | 0.1175 m | 0.1375 m |
| 0.08 m | 0.1296 m | 0.1496 m |
| 0.10 m | 0.1378 m | 0.1578 m |

`wheel_face_exclusion_m()` 是一個 **輪盤（disc）近似**：它只描述輪緣圓盤與垂直面的干涉，沒有涵蓋連桿、機身或輪寬方向。不能當成完整碰撞檢查。

## 關鍵發現三：crawl 的步幅中心不是 Walk 的 touchdown 模板

Step 6 用 `TrajectoryPlanner3D._level_touchdown_q()` 當 nominal touchdown，那是 rolling stance 的**前端極限**姿態。對 world-fixed stance 來說，整段 stance 會從那裡一路往後掃，等於只用到單邊工作空間。

Step 7 改用「`beta = gamma = 0` 的直立姿態」當步幅中心：以 `stand_height` 解出 `theta`，取最低輪緣點作為 neutral 落點，再讓 touchdown 落在 `neutral + L/2`、liftoff 落在 `neutral - L/2`。

實測 `stand_height = 0.25 m` 時，以最低輪緣點為接觸的可達水平掃幅（含 `theta ∈ [17°,160°]`、`|beta| <= 40°`）隨 body-frame 深度變化：

```text
depth 0.17 m -> ±0.065 m
depth 0.20 m -> ±0.090 m
depth 0.22 m -> ±0.110 m
depth 0.25 m -> ±0.135 m
depth 0.27 m -> ±0.150 m   （最佳）
depth 0.30 m -> ±0.115 m
depth 0.32 m -> ±0.070 m
depth 0.33 m -> ±0.015 m
```

這條曲線就是 Step 7 步長上界的來源，也解釋了為什麼障礙物越高越難：body 必須同時服務「站在頂面」與「站在地面」兩個深度。

## Body pose 假設

Step 7 的 body 全程保持水平（`roll = pitch = yaw = 0`），只調整高度：

```text
body_z = min(heights) + stand_height + body_lift_ratio * (max(heights) - min(heights))
```

其中 `heights` 同時包含**目前四個支撐接點高度**與**這一步即將落下的目標面高度**。兩者都要納入，因為：

- 只看目前接點：step-up 時 body 還太低，抬腳搆不到頂面（實測 IK 誤差 65 mm）。
- 只看目標面：step-down 時 body 提前下降，還踩在頂面的那隻腳會被壓成不可解（實測 `IK_UNREACHABLE`）。

`body_lift_ratio` 預設 `0.6`。不使用 pitch 的理由是：Walk 腳序（FL → RR → FR → RL）會出現「左前在頂面、右前在地面」這種左右不對稱的支撐組合，pitch 無法補償，reference 也會因為 body 旋轉而讓 touchdown 模板耦合到 body 姿態。這是第一版的明確簡化，不是最佳解。

## 每個 event 的搜尋策略

每一個 swing event 的決策順序是（全部 deterministic，沒有隨機）：

```text
for 前進比例 in (1.0, 0.75, 0.5, 0.25, 0.0) * (L/4):
  for touchdown 候選 in 依 |bias| 由小到大 (ground-before / top / ground-after):
      產生 stance segment（body 前進 + 高度調整，四腳世界固定）
      for swing 時長倍率 in (1.0, 1.5, 2.0):
          產生 swing segment
      第一個成功的組合即採用
```

- 前進比例回退讓步態在障礙物前自動「縮步」，等腳走到可以一步跨上頂面的位置。這就是計畫書 Step 3/6 說的「調整前面的步距」。
- swing 時長倍率只改變取樣密度，用來壓低高低差大的那幾步的單列 joint step。
- 每個生成成功的 stance 與 swing 還要再通過 `joint_limit_margin_rad`（預設 `0.02 rad`，約 1.15°）的
  餘裕檢查。Step 4/5 只要求姿態落在 `RobotParams` 的硬限制內，所以 world-fixed stance 可以合法地把
  某隻腳掃到剛好 `beta = ±40°`，接下來的 swing 就一點餘裕都沒有（實測有一次只超出 `5×10⁻⁵ rad`
  就被 `JOINT_LIMIT` 擋掉）。Step 7 寧可先縮短 body 前進量，也不把工作空間用滿。
- 全部候選都失敗時回報 `NO_FEASIBLE_TOUCHDOWN`，並附上每一個被拒絕候選的完整理由（前進比例、touchdown x、surface、失敗原因、餘裕違規）。

## Step 6 reference schedule 的實際狀況

Step 7 仍會呼叫 `schedule_obstacle_walk()`，把結果放進 metadata 的 `step6_reference_schedule`，但要如實說明兩件事：

1. 它是 **nominal periodic-Walk 預測**，不是被執行的計畫。Step 7 的每個 touchdown 都由上一段實際 `final_state` 加上 wheel-face foothold 規則重新推導，時間與落點本來就會不同。
2. 在目前參數（`T = 2.0 s`、`dt = 0.02 s`、`stance_duty = 0.75`）下，scheduler 直接拒絕：

```text
INVALID_INITIAL_STATE: gait_cycle_phase does not match the existing Walk phase pattern
(the periodic Walk cycle at T=2 s, dt=0.02 s, stance_duty=0.75 contains 0
all-stance samples, so a four-leg-stance crawl boundary need not exist in it)
```

原因是 `stance_duty = 0.75` 的四腳 Walk，四段 swing 剛好鋪滿整個週期，是否存在「四腳同時 stance」的取樣點完全取決於 `T/dt` 的取整。crawl 的 event 邊界必定是四腳 stance，所以在這種參數下沒有對應的 periodic phase 可以對齊。這個訊息會原樣寫進 metadata，不會被吞掉。

## 輸出格式

`write_obstacle_walk_csv()` 產生四個檔案：

```text
<name>.csv                 12 欄、無 header、hardware 欄序、%.6f
<name>_phase.csv           4 欄、有 header、row-aligned
<name>_metadata.json       輸入、推導值、每段 row 範圍與 touchdown 證據
<name>_validation.json     已檢查/未檢查清單、stage 結果、worst-case、每個接點報告
```

- 硬體欄序只在 `export.py` 轉換一次，並有測試對照既有 `generate_hardware_csv.py::_to_hw_order` 逐值相同。
- prep（cosine ramp，home `theta = 17°` → 第一列）只出現在整份檔案最前面一次。
- metadata 的每段都同時記錄 `trajectory_start_row/end_row` 與 `csv_start_row/end_row`（含 prep 偏移），測試驗證所有 row 都被某一段覆蓋。

## 可重現範例

```bash
MPLCONFIGDIR=/tmp/legwheel_step7_mpl \
.venv/bin/python examples/gait/generate_obstacle_walk_csv.py \
  --obstacle-x 0.65 \
  --obstacle-length 0.35 \
  --obstacle-height 0.06 \
  --edge-margin 0.02 \
  --step-length 0.15 \
  --period 2.0 \
  --dt 0.02 \
  --step-clearance 0.02 \
  --approach-distance 0.45 \
  --post-distance 0.30 \
  --prep-seconds 5.0 \
  -o outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.02.csv
```

輸出：

```text
outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.02.csv
outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.02_phase.csv
outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.02_metadata.json
outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.02_validation.json
```

實際結果：

```text
輪緣排除帶 g                 = 117.5 mm
最小步長 g + m               = 137.5 mm（實際用 150 mm）
合法頂面區間                 = [0.67, 0.98] m
腳序                         = FL → RR → FR → RL
踏上頂面的腳                 = FR, FL, RL, RR（四腳）
踏回地面的腳                 = FR, FL, RL, RR（四腳）
同時在頂面的最大腳數         = 2
traversal completed          = true
recovery 距離                = 0.300 m

segment 數                   = 92（46 個 event，各一段 stance + 一段 swing）
接點數                       = 91，全部通過
最大接點 joint position 誤差 = 0.0 rad
最大接點 joint velocity 誤差 = 0.0 rad/s
最大 world-fixed contact drift = 0.995 mm
最大 swing IK/FK 誤差        = 0.199 mm

prep rows                    = 250
trajectory rows              = 2439
total rows                   = 2689
最大關節速度                 = 10.93 rad/s（RL_theta）
最大關節加速度               = 287.5 rad/s²
full geometry checked        = false
```

`max simultaneous top = 2` 是這個尺寸的必然結果：前後腳的縱向距離約 0.51 m，比 0.35 m 的障礙物長，所以前腳下來之後後腳才上去，不會出現四腳同時在頂面。這不是失敗。

「最大關節速度 10.93 rad/s」與「最大關節加速度 287.5 rad/s²」只是離線命令的有限差分值，**沒有**對照過實機的速度/加速度/力矩限制。

## 硬體 `dt = 0.001 s` 版本

同一組障礙物參數，改用實機預設 `dt = 0.001 s` 另外生成一份：

```bash
MPLCONFIGDIR=/tmp/legwheel_step7_mpl \
.venv/bin/python examples/gait/generate_obstacle_walk_csv.py \
  --obstacle-x 0.65 --obstacle-length 0.35 --obstacle-height 0.06 \
  --edge-margin 0.02 --step-length 0.15 --period 2.0 --dt 0.001 \
  --step-clearance 0.02 --approach-distance 0.45 --post-distance 0.30 \
  --prep-seconds 5.0 \
  -o outputs/obstacle_walk_step7/obstacle_walk_H060_L150_dt0.001.csv
```

結果：

```text
traversal completed          = true（五個 stage 全部達成）
踏上頂面 / 踏回地面的腳       = 四腳 / 四腳
segment 數                   = 84（42 個 event）
prep rows                    = 5000（5.0 s）
trajectory rows              = 43377
total rows                   = 48377（48.4 s）
最大接點 joint position 誤差 = 0.0 rad
最大接點 joint velocity 誤差 = 0.0 rad/s
最大 world-fixed contact drift = 0.200 mm
最大 swing IK/FK 誤差        = 0.200 mm
最大關節速度                 = 12.20 rad/s（FR_theta）
最大關節加速度               = 4186 rad/s²（FR step-up swing 中段）
被拒絕候選有紀錄的 segment    = 8 段
full geometry checked        = false
```

注意 `dt = 0.001 s` 的落點序列與 `dt = 0.02 s` **不同**（84 段 vs 92 段），這正是前面說的 dt 相依性。

作為對照，既有 flat Walk 在同樣 `dt = 0.001 s` 下的命令加速度是：

```text
vx=0.05, T=1.0 : 最大 15933 rad/s²，逐列中位數 6684 rad/s²
vx=0.10, T=2.0 : 最大 13175 rad/s²，逐列中位數 5522 rad/s²
```

Step 7 這份是 `最大 4186 rad/s²、中位數 58 rad/s²`，比既有 baseline Walk 平順很多。但兩者都**沒有**對照過實機的加速度或力矩上限，這個比較只說明「不比現有 baseline 差」，不構成安全結論。

## 已驗證的可行障礙高度

以 `x_start = 0.65 m`、`length = 0.35 m`、`edge_margin = 0.02 m`、`stand_height = 0.25 m`、
`T = 2.0 s`、`dt = 0.02 s`、`step_clearance = 0.02 m`、`body_lift_ratio = 0.6` 掃描：

| H (m) | 步長 L (m) | 結果 | rows | 最大接點 qd | 最大 contact drift | 最大 IK/FK 誤差 |
| --- | --- | --- | --- | --- | --- | --- |
| 0.06 | 0.14 | 完整越障 | 2809 | 0.0 rad/s | 1.00 mm | 0.20 mm |
| 0.06 | 0.15 | 完整越障 | 2491 | 0.0 rad/s | 1.00 mm | 0.20 mm |
| 0.06 | 0.16 | 完整越障 | 2173 | 0.0 rad/s | 1.00 mm | 0.20 mm |
| 0.08 | 0.155 | 完整越障 | 2703 | 0.0 rad/s | 1.00 mm | 0.20 mm |
| 0.08 | 0.165 | 完整越障 | 3127 | 0.0 rad/s | 1.00 mm | 0.20 mm |
| 0.08 | 0.175 | 失敗 | — | — | — | — |
| 0.10 | 0.162 | 失敗 | — | — | — | — |
| 0.10 | 0.170 | 失敗 | — | — | — | — |
| 0.10 | 0.180 | 失敗 | — | — | — | — |

「完整越障」的定義是四腳都踏上頂面、四腳都踏回地面、四個接觸點都越過 `x_end`，並補完指定的 recovery 距離。

以上掃描使用 `joint_velocity_limit_rad_s = 10.0`；之後預設值改為 `16.0`（理由見下），因此重跑數值可能略有差異，但 0.10 m 的失敗原因（`beta` 限制）與該參數無關。

`contact drift = 1.00 mm` 正好等於 `contact_drift_tolerance_m` 的預設值，這是因為 Step 5 的最低輪緣迭代**收斂到容許值就停**。它是「不超過門檻」的保證，不是獨立量到的誤差，不要解讀成精度指標。

## 測試

```bash
MPLCONFIGDIR=/tmp/legwheel_step7_mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -m pytest -q tests/test_obstacle_walk_traversal.py
```

結果：`21 passed`。

涵蓋：

- `wheel_face_exclusion_m()` 的圓盤幾何與輸入驗證（含 H > R 的飽和）。
- 腳序來自 phase offsets；改 offsets 後腳序跟著變。
- touchdown 候選同時通過 Step 3 query 與 wheel-face 排除帶，且依 |bias| 排序。
- request 輸入驗證拒絕案例。
- 步長不足 `g + m` 時回報 `STEP_TOO_SHORT_FOR_OBSTACLE`。
- 步長過大觸發 Walk workspace guard 時回報 `VELOCITY_GUARD_SCALED`。
- 完整 traversal 的五個 stage 全部達成，四腳上、四腳下。
- 所有 segment boundary 通過 Step 2 validator，且誤差在預設容許值內。
- 全程同時只有一隻腳 swing，`contact_active == (phase == 0)`。
- body 全程水平、不倒退。
- 每個 touchdown 都在合法 surface、避開 edge margin 與 wheel-face 排除帶、bias 在預算內。
- contact drift 與 IK/FK 誤差低於門檻。
- theta/beta/gamma 在 `RobotParams` 限制內，單列 joint step 在門檻內。
- 全程命令都保留 `joint_limit_margin_rad` 的餘裕。
- 把餘裕設成不可能的 `0.60 rad` 時，回報 `NO_FEASIBLE_TOUCHDOWN` 且理由中包含 margin 違規。
- segment row 範圍首尾相接、剛好覆蓋整條軌跡。
- 相同輸入兩次產生逐值相同的結果。
- 硬體欄序與既有 `_to_hw_order` 完全一致。
- prep 起點是 home pose、終點是第一列，`prep_duration = 0` 時不產生任何 prep。
- 匯出的 CSV 為 12 欄無 header、phase row-aligned、prep 只出現一次、metadata/validation 內容正確。
- metadata 的 row 範圍可覆蓋 CSV 每一列。

Step 0–6 與既有 gait/hybrid 測試一起執行：`92 passed`，沒有回歸。

另外通過 Python compile、`git diff --check`，新增檔案皆在 99 欄內。

## 尚未完成、不能宣稱

- **不是 periodic Walk**：輸出是 quasi-static crawl，body 在每次 swing 期間靜止。平均前進速度遠低於同參數的 Walk。
- **沒有恢復成 rolling Walk**：traversal 之後的 recovery 仍是 crawl。要真正接回 periodic Walk，必須先做出可處理混合高度的 rolling stance。
- **沒有完整幾何碰撞檢查**：`FullGeometryCollisionChecker` 介面已接上 stance 與 swing 兩端，但 repo 內仍沒有實際的 3-D 腿/輪/機身 vs 矩形 checker。`full_geometry_collision_checked` 固定回報 `false`。
- **沒有穩定度**：沒有支撐多邊形、CoM、quasi-static stability margin、力、摩擦或滑動檢查。
- **沒有 body pitch/roll**：混合高度時只調整 body 高度，這限制了可行障礙高度。
- **0.10 m 障礙尚未產生完整 traversal**：見「0.10 m 障礙為什麼還走不過去」。
- **沒有模擬或實機**：沒有 Webots，也沒有硬體執行。CSV 的關節速度/加速度只在 validation report 中列出，未對照實機限制校正。
- `joint_velocity_limit_rad_s = 16.0` 是規劃期的守衛值，不是實機安全限制。這個數字取自既有 flat Walk 自己在 `dt = 0.001 s` 下量到的關節速度峰值（`vx=0.05, T=1.0` 為 `15.93 rad/s`；`vx=0.10, T=2.0` 為 `13.18 rad/s`），意思只是「Step 7 不會要求比 baseline Walk 更快的關節」，不代表硬體允許這個速度。

## 0.10 m 障礙為什麼還走不過去

計畫書希望用 `H = 0.10 m` 建立範例。目前這個 crawl 做不到，失敗點很明確且可重現：

```text
H=0.10 L=0.162/0.170/0.180
NO_FEASIBLE_TOUCHDOWN at event 8 for FL
  x=0.8499 surface=obstacle_top swing_scale=1  : JOINT_LIMIT FL.beta = -0.7082 rad (-40.6°)
  x=0.8499 surface=obstacle_top swing_scale=1.5: JOINT_LIMIT FL.beta = -0.7253 rad (-41.6°)
  x=0.8499 surface=obstacle_top swing_scale=2  : JOINT_LIMIT FL.beta = -0.7212 rad (-41.3°)
```

失敗的不是踏上去那一步，而是**已經站在頂面之後的 top swing**。原因是三個約束在 `H = 0.10 m` 同時收緊：

1. 輪緣排除帶要求 `L >= g + m = 0.1578 m`，步長不能縮小。
2. 水平的 body 必須同時服務「站在頂面」與「站在地面」兩種深度，兩者差 `H`。用
   `body_lift_ratio = 0.6` 時，頂面腳的 body-frame 深度只有約 `0.21 m`，對應的靜態
   可達掃幅只有 `±0.10 m` 左右，已經逼近 `L/2 = 0.081 m` 再加上偏移量的需求。
3. swing 追蹤的是「touchdown 時會接觸的那個輪緣材料點」（沿用既有 Walk 的
   material-point 慣例）。world-fixed stance 期間輪子不滾，`beta` 從約 `+40°` 掃到
   `-40°`，所以 liftoff 姿態下那個材料點離當下的最低接觸點可以差到 80° 輪緣角，
   swing 中段為了把它擺到 Bezier 路徑上，`beta` 就超出 `±40°`。

因此這不是把某個容許值放寬就能解決的 —— `±40°` 是 foot rim 還能當接觸點的範圍，而 Pure Walk 不做換 rim
（詳見「關鍵發現四」）。實測也證實：即使違規把上限放到 75°，`H = 0.10` 仍然在更後面的 event 30 撞到第二道牆。

在**不改變 Pure Walk 接觸假設**的前提下，剩下的方向只有：

1. **加入 body pitch**：body 傾斜會改變同一個落點所需的 body-frame `beta`，可望把峰值壓回 40° 內。
   目前 Step 7 的 body 全程水平，這是最直接、還沒試過的一條。
2. **提高 `stand_height`**：把所有腿的工作深度往可用區間中央移。
3. **縮短步長**：但步長被輪緣排除帶從下面卡死（`L >= g + m = 0.158 m`），可調空間很小。
4. **改用滾動 stance**：rolling stance 的 body-frame 後掃只有 world 前進量的約一半，`beta` 掃幅會明顯變小。

若以上都不成立，那結論就是「Pure Walk 以 foot rim 越過 0.10 m 障礙在運動學上不可行」——
這本身就是一個對 Hybrid gait 比較有意義的結果，而不是實作缺陷。

在做到上述任何一項之前，**不能宣稱這套 offline generator 可以越過 0.10 m 障礙**。目前可重現的上界是 `H = 0.08 m`。

## 關鍵發現四：`beta ±40°` 是 foot rim 的接觸範圍，Walk 真的上不去

一度誤判：因為 `BETA_MAX_DEG = 40.0` 在 `config/__init__.py` 裡被歸在
`Workspace Guard Constants  only used for trajectory planning and velocity limiting` 底下，
而且 `corgi_leg.py` 的 FK/IK 沒有 beta 上限、hybrid 2-D 測試姿態集又包含 `(theta, beta) = (110°, 95°)`，
所以曾判斷它「只是平地 Walk 的守衛、不是硬體限制」，並把它參數化放寬。

**這個判斷是錯的，已經改回去。**

`±40°` 是 **foot rim 還能當接觸點的角度範圍**。腿再往外轉，接觸就會離開 foot rim、落到上方的
tyre rim（`RimId.LEFT` / `RimId.RIGHT`）上 —— 那正是 hybrid gait 在做的換 rim 動作，也是 hybrid
測試會出現 `beta = 95°` 的原因。**Pure Walk 不做換 rim，全程只用 foot rim**，所以一旦超過 `±40°`，
規劃出來的 foot-rim 接觸根本不存在，那個姿態是不可行的，不是「比較激進」而已。

換句話說：**foot rim 上不去，就是真的上不去。**

程式已還原成硬性使用 `RobotParams.BETA_MAX_DEG`，`generate_swing_segment()` 與
`generate_stance_segment()` 都不再接受可調的 beta 上限，並在
`_check_joint_limits()` 的 docstring 寫明理由，避免以後有人再誤放寬。

validation report 仍保留：

```text
worst_case.maximum_abs_beta_deg   實際用到的最大 |beta|
worst_case.beta_limit_deg         foot rim 上限
derived.beta_limit_meaning        說明這是硬性可行性邊界
```

保留這個數值是為了讓人一眼看出軌跡離 foot-rim 邊界還有多少餘裕。

### 放寬 beta 的實驗結果（僅作為記錄，結論不採用）

放寬過程跑過的掃描仍有參考價值，因為它證明了「就算放寬 beta，`H = 0.10` 也走不過去」：

| H (m) | L (m) | beta 上限 | 結果 |
| --- | --- | --- | --- |
| 0.08 | 0.175 | 40°（正確值） | 失敗，event 35 撞 beta |
| 0.08 | 0.175 | 60°（不合法） | 完整越障，實際用到 45.5° |
| 0.10 | 0.162 | 50° / 60° / 75°（不合法） | 全部失敗，event 30，訊息完全相同 |
| 0.10 | 0.180 | 60°（不合法） | 失敗，event 30 |

`H = 0.10` 在 50°/60°/75° 三組的失敗點、失敗腳、失敗訊息完全一致：

```text
NO_FEASIBLE_TOUCHDOWN at event 30 for FR
  advance=1.00 x=1.1840 surface=ground stance:
    IK_UNREACHABLE for FL: lowest-rim contact iteration did not converge
```

也就是說 `H = 0.10` 除了 beta 之外還有第二道牆（Step 5 最低輪緣迭代在障礙物遠端落地區不收斂）。
既然 beta 不能放寬，這條路本來就走不通；記錄下來只是為了說明「不要再往放寬 beta 的方向試」。

## dt 會影響規劃結果

Step 7 的每個 event 是用「實際生成成功與否」來挑候選的，而 `generate_swing_segment` 的取樣數與
`maximum_joint_step_rad = joint_velocity_limit * dt` 都跟 `dt` 有關，所以**不同 `dt` 可能挑到不同的
落點序列**。實測 `H=0.06, L=0.15` 在 `dt=0.02` 可以完整越障，第一次用 `dt=0.001` 生成時卻在
event 18 失敗（`joint_velocity_limit=10` 時的 `JOINT_DISCONTINUITY`）。

這代表：

- 每一組實際要輸出的參數（含 `dt`）都必須自己跑過一次，不能用另一個 `dt` 的成功結果外推。
- metadata 內記錄的參數必須完整，才能重現同一條軌跡。目前 metadata 已保存全部 request 欄位。

## Step 8 的直接入口

Step 8 可以直接消費：

```python
from legwheel.planners.obstacle_walk import (
    ObstacleWalkRequest, generate_obstacle_walk, write_obstacle_walk_csv,
)

result = generate_obstacle_walk(request)
result.segment.body_pose_world        # 每列 body world pose
result.segment.foot_contact_points_world_m
result.segment.phase
result.records                        # 每段 row 範圍、stage、touchdown、誤差
result.boundary_reports
```

Step 8 應該補上的，依重要性排序：

1. 實際的完整 3-D 腿/輪/機身 vs 矩形 collision checker，接到既有 `FullGeometryCollisionChecker` 介面，並在 metadata 把 `full_geometry_collision_checked` 變成 `true`。
2. 支撐多邊形與 quasi-static stability margin 逐列檢查。
3. 幾何可視化與動畫，逐 stage 報告 approach / step-up / top support / step-down / recovery。
4. 混合高度的 rolling stance（讓 crawl 能接回 periodic Walk），或明確放棄並記錄。

---

## Step 9 調整：日常驗證不需要 AI 逐案操作（2026-08-31）

Step 9 改為兩層可重跑流程，目的不是跳過模擬，而是避免每生成一個 CSV 都要由 AI 重新讀參數、下指令與判讀結果。

1. **9A 離線初篩**：每個 CSV 用固定的 validator 指令產生 JSON、Markdown 與圖。檢查格式/時間、關節位置速度加速度、segment 接點、Step 8 幾何結果和固定初始 `x` 偏移掃描。輸出 `PASS`、`FAIL_GEOMETRY_OR_LIMIT` 或 `REQUIRES_SIMULATION`。
2. **9B 批次 Webots**：只將 9A 通過的有限候選，交給由 YAML case list 驅動的 batch runner。固定環境與 controller 參數，輸出 traversal、碰撞、body pose、tracking、contact/slip、failure stage；人只看 summary。

這是「初步驗證」：它可證明 CSV 在現有離線幾何與 joint-limit 模型中自洽，也可以篩掉接點跳變、速度超限、幾何碰撞等明確問題；它**不能**替代動態模擬，不能宣稱負載、摩擦、接觸穩定或實機安全。

AI 的合理角色只剩兩個：第一次建立 validator/batch runner，或離線通過但 Webots 失敗時協助讀取 log/影片並提出下一個受控實驗。正常產生與重跑案例不需要 AI 參與。

---

## 固定案例：初始 COM 前方 1 m 的 4 cm 障礙（2026-08-31）

已成功生成一組 offline obstacle-walk CSV。座標定義為初始 body COM 位於 world `x = 0`，障礙物前緣位於 `x = 1.0 m`；因此必須使用 `--approach-distance 1.0`，不可沿用預設 `0.45 m`。

| 項目 | 值 |
| --- | --- |
| 障礙物前緣 | `x = 1.0 m`（相對初始 COM 前方 1 m） |
| 前進方向長度 | `0.40 m` |
| 橫向寬度 | `0.80 m`（供 Webots 場景使用） |
| 高度 | `0.04 m` |
| 命令速度 | `0.10 m/s` |
| period / stride | `2.0 s` / `0.15 m` |
| 取樣時間 | `0.02 s` |

生成指令：

```bash
MPLCONFIGDIR=/tmp/legwheel_obstacle_004_mpl \
.venv/bin/python examples/gait/generate_obstacle_walk_csv.py \
  --obstacle-x 1.0 --obstacle-length 0.4 --obstacle-height 0.04 \
  --edge-margin 0.02 --vx 0.1 --period 2.0 --dt 0.02 \
  --step-clearance 0.02 --approach-distance 1.0 --post-distance 0.30 \
  --prep-seconds 5.0 \
  -o outputs/obstacle_walk_step9/obstacle_walk_x100_L400_W800_H040_v100_dt020.csv
```

結果：`traversal_completed = true`，approach / step-up / top support / step-down / recovery 全部通過；最大 `|beta| = 38.8°`（foot-rim 上限 `40°`）、最大 contact drift `1.000 mm`、最大 swing IK/FK 誤差 `0.196 mm`。輸出共 3325 rows（含 250 rows prep）。

注意：目前 Step 7 產生器是 sagittal 2-D，沒有 `obstacle_width` 參數；`0.80 m` 不改變此 CSV 的 joint 軌跡，必須在 Webots 障礙物 `Box`/場景設定中明確設為橫向寬度。這份結果是 offline 運動學與既有 validator 的通過結果；`full_geometry_collision_checked = false`，尚未完成 3-D 全幾何、動力學或 Webots 驗證。

### 發現：此案例尚不符合 panel / `corgi_csv_control` 的 transform-trigger 契約

使用者指出實際操作是先在 panel 按 Run，controller 先播放 CSV 的 transform 區塊；之後按 Start Trigger 才播放正式軌跡。重新對照目前程式後，確認問題成立：

- `corgi_csv_control.cpp` 以固定 `1 ms` period 播放。
- 它會無條件讀取並發布 CSV 前 **5000 rows**，才等待 trigger；目前 live code 不是 6000 rows。
- obstacle exporter 確實有 home pose 到第一個 trajectory frame 的 5 秒 cosine prep，所以不是完全沒有 transform。
- 但此案例用 `dt = 0.02 s`，5 秒 prep 只有 `250 rows`，而整份 CSV 只有 `3325 rows`。
- 即使另外補足 5000 個 prep rows，controller 仍會以 1 ms 播放原本按 20 ms 規劃的 trajectory，造成時間軸壓縮 20 倍；所以不能只補 row 數。

因此 controller 會把 250-row prep 和後面的 trajectory 一起當成 transform 讀掉，甚至在第 3325 row 之後繼續嘗試解析空行。這份 `obstacle_walk_x100_L400_W800_H040_v100_dt020.csv` **不可直接用目前 panel/controller 播放**；它只能保留為 offline 規劃與幾何驗證 artifact。

需要修正的介面契約：

1. hardware export 必須是 `controller_dt = 0.001 s`。
2. 前導 transform 必須恰好是 5000 rows（row 0--4999）；正式軌跡從 row 5000 開始。
3. 若 planner 使用較粗的 `dt`，export 前必須以受驗證的方法重取樣到 1 kHz，且重新檢查 joint position / velocity / acceleration；不能只改 metadata。
4. exporter 應拒絕產生不符合 controller 契約的「hardware-ready」CSV，並在 metadata 明寫 transform row count、trajectory start row 與 controller period。
5. 加入 integration test：模擬 controller 先消耗 5000 rows，確認 row 5000 仍是正式軌跡第一列且檔案總長大於 5000。

### 修正完成：固定 1 kHz exporter/controller 契約與新版 4 cm CSV

已完成以下修改：

- obstacle hardware exporter 固定 `controller_dt = 0.001 s`、`transform_rows = 5000`、`transform_duration = 5.0 s`。
- planner 仍可使用較粗且為 1 ms 整數倍的 `dt`；exporter 以 component-wise PCHIP 重取樣 joint commands 到 1 kHz。PCHIP 會精確通過原 planner knots，且避免一般 cubic spline 的 overshoot。
- phase 是離散 interval label，不做數值插值；每個 planner interval 展開成對應的 1 ms rows。
- metadata 現在分開記錄 `planner_dt_s`、`controller_dt_s`、`resample_ratio`、`trajectory_start_row`，segment 的 CSV row range 也映射到重取樣後索引。
- exporter 拒絕非 5 秒 transform，避免再次輸出 controller 會誤讀的檔案。
- `corgi_csv_control` 將 5000 rows / 1 kHz 定義為明確常數；transform 或 trajectory 遇到 EOF、非 12 欄、空欄或非有限數值時，會報出 row 並停止，不再以 `std::stod` 解析空字串崩潰。若只有 5000 rows 而沒有 post-trigger trajectory 也會拒絕。

新版固定案例：

```text
outputs/obstacle_walk_step9/
  obstacle_walk_x100_L400_W800_H040_v100_controller_dt001.csv
  obstacle_walk_x100_L400_W800_H040_v100_controller_dt001_phase.csv
  obstacle_walk_x100_L400_W800_H040_v100_controller_dt001_metadata.json
  obstacle_walk_x100_L400_W800_H040_v100_controller_dt001_validation.json
```

實際 row 契約與數值檢查：

| 項目 | 結果 |
| --- | --- |
| transform rows | `5000` |
| trajectory rows | `61481` |
| total rows | `66481` |
| controller dt | `0.001 s` |
| planner dt / resample ratio | `0.02 s / 20` |
| 正式軌跡起點 | row `5000`（0-based） |
| row 4999 → 5000 最大 joint jump | `0.0 rad` |
| 最大 joint velocity（1 kHz CSV） | `9.976 rad/s` |
| 最大 discrete joint acceleration | `722.764 rad/s²` |
| finite values | 全部通過 |
| traversal stages | 全部通過 |

驗證：`corgi_csv_control` package 成功編譯；`tests/test_obstacle_walk_traversal.py -k 'not generation_is_deterministic'` 為 `26 passed, 1 deselected`。另三個 prep/resampling 專屬測試為 `3 passed`。完整 deterministic test 因會額外重跑兩次昂貴 traversal，本輪在確認沒有 assertion failure 後中止，改由上述單次端到端案例確認 deterministic 輸出介面。

界線：新版檔案現在符合 panel/controller 的 row/timing 契約，但仍是 offline kinematic trajectory。PCHIP 後的 joint position/rate 已檢查；1 ms 中間列尚未逐列重建 body/contact/full 3-D geometry，且沒有 Webots dynamics、摩擦、tracking 或實機安全驗證。

### 模擬異常診斷：gamma 左右擺動、機身晃動與偏航

新版 4 cm CSV 在模擬中出現腳持續內收/外展、機身左右晃並旋轉。檢查 1 kHz CSV 與 planner 後確認：

1. 這不是 PCHIP 產生的高頻數值抖動。gamma 命令是連續曲線；問題是 planner 原始命令本身有大幅、低頻的側向動作。
2. 單腳 crawl 即使地形對稱，瞬間的三腳支撐三角形仍不對稱，所以適量 lateral body sway 是合理的。目前 request 要求 `20 mm` support-polygon margin，並允許最多 `90 mm` sway。
3. 真正的累積問題位於 traversal touchdown policy：swing target 使用 `touchdown_y = swayed_y + neutral_leg_y`。也就是把原本只應在該次 swing 暫時使用的 body sway，永久寫進新足跡的 world y。
4. 足跡偏掉後，下一次 `choose_lateral_sway()` 又依偏掉的三個支撐點求新的 body y，造成回授式累積。此案例 body y 原本只在約 `±11.25 mm` 間移動，後段最高漂到 `+84.375 mm`。
5. 左腳 touchdown y 約為 `0.2004--0.2117 m`，但 RR 從 `-0.2004 m` 漂到 `-0.1273 m`（向內約 `73 mm`），FR 也漂到 `-0.1610 m`。左右足跡已不再鏡射對稱。
6. gamma 範圍因此很大且不對稱：FL 約 `-16.6°--+3.2°`、FR `-11.5°--+6.6°`、RR `-18.0°--+13.4°`、RL `-16.6°--+3.0°`；RR span 達 `31.4°`。
7. planner 的 body yaw 雖固定寫成 0，但 CSV 只開迴路命令 joints，沒有 body-yaw feedback。左右足跡與接觸力不對稱時，模擬機身仍會實際偏航。

因此不能把修法簡化成四條 gamma 全設 0；那會移除目前唯一的三腳支撐 margin。正確方向是：body sway 可以暫時存在，但 touchdown world y 應固定在以 `body_y_reference` 為中心的左右 nominal track；完成 swing/crawl cycle 後 body y 應回中，不可把 sway 累積到足跡。修正後必須新增左右 touchdown mirror error、每 cycle body-y return error、net lateral drift、gamma mirror/range 與模擬 yaw drift 檢查。

---

# Step 8：完整幾何碰撞、支撐多邊形穩定度與可視化

完成日期：2026-08-31

## 這一步真正的價值：Step 7 自稱全過的軌跡，被抓到兩個問題

Step 7 的驗證是「逐段生成時檢查」，而且碰撞只檢查**被追蹤的那一個接觸材料點**。
Step 8 對**組裝完成的整條軌跡**重跑檢查，並補上兩件 Step 7 明講沒做的事：

1. 完整腿部幾何碰撞（三條輪胎弧 + 六根連桿），不是單點。
2. 支撐多邊形的準靜態穩定度。

第一次跑（`H=0.06`、`L=0.15`、`step_clearance=0.02`）結果：

```text
joint_position_limits        : passed  最近 FR.beta 餘裕 0.0204 rad
joint_velocity               : passed  10.93 rad/s（限 16）
joint_acceleration           : passed  287 rad/s²
gait_phase_legality          : passed  2439 rows 全程同時只有一隻腳 swing
support_contact_drift        : passed  0.62 mm
full_leg_geometry_collision  : FAILED  foot_rim 穿入 obstacle_top 2.268 mm
quasi_static_support_polygon : FAILED  body 中心投影在支撐多邊形外 44.6 mm
offline_complete_traversal   : false
```

兩個都是真的問題，不是檢查器誤報。

## 新增檔案

- `legwheel/planners/obstacle_walk/collision.py`：完整腿部幾何碰撞。
- `legwheel/planners/obstacle_walk/validation.py`：整條軌跡驗證與逐 stage 報告。
- `examples/gait/generate_obstacle_walk_step8_artifacts.py`：圖、動畫與 validation JSON。
- `tests/test_obstacle_walk_traversal_validation.py`：Step 8 測試。

## 碰撞檢查為什麼可以直接用 hybrid 的 2-D 檢查器

hybrid 那條線已經有 `query_contact()` / `detect_geometry_penetrations_2d()` /
`detect_link_collisions_2d()` / `detect_rectangle_vertical_face_collisions_2d()`，
Step 8 直接接上，沒有另寫碰撞演算法。

**而且 2-D 投影對這個地形是精確的，不是近似**：`WalkTerrain1D` 只是 world `x` 的函數
（矩形在 `y` 方向無限延伸），所以世界點在障礙物內 ⟺ 它的 `(x, z)` 投影在內。
換成有限寬度的障礙物就不成立，這一點寫在模組 docstring 裡。

腿部點取自既有 `PlotLeg` 的 2-D solver，再用 `CorgiLegKinematics._transform_to_body()`
（`forward_kinematics()` 用的同一個轉換）升到 body frame，沒有引入第二套運動學。

驗證方式是 `verify_geometry_matches_forward_kinematics()`：量測「取樣輪緣點」與「3-D FK 接觸點」
的距離，並確認它隨取樣數下降：

```text
arc_samples= 12 -> 3.4509 mm
arc_samples= 48 -> 1.8846 mm
arc_samples=192 -> 0.4637 mm
```

若 frame 錯了，這個距離會停在一個常數而不會收斂。這是「有沒有搞錯座標系」的判別測試。

## 發現一：輪緣削到障礙物的頂前角

穿透點的完整定位：

```text
segment 9, FL 的 step_up swing, 目標落點 (0.7175, 0.06) 在頂面
row 250, FL 在半空中 (phase = swing)
穿透點: foot_rim 上 alpha = -33.04 度的取樣點
        world (0.65281, 0.05773)
障礙物前緣 x = 0.65, 頂面 z = 0.06
=> 前緣內側 2.8 mm、頂面下方 2.27 mm
```

被追蹤的材料點確實跨過了邊緣（Step 4 的 `_check_contact_path` 檢查的就是它），
但輪子是半徑 0.145 m 的圓，離接觸點 33 度的另一段輪緣掛到了角上。

**根本原因是 clearance 的定義**：`z_apex = max(...) + clearance` 是套在被追蹤的**材料點**上，
而輪緣在那個點周圍還往外延伸，所以真正需要的離地高度比名目 step height 大。

掃描結果：

| step_clearance | 碰撞檢查 | 最深穿透 |
| --- | --- | --- |
| 0.02 m | failed | +2.268 mm |
| 0.03 m | passed | +0.039 mm |
| 0.04 m | passed | +0.039 mm |
| 0.05 m | passed | +0.039 mm |

因此把 CLI 的 `--step-clearance` 預設從 `0.02` 改回 `0.03`（也就是計畫書原本的值），
並在 help 文字寫明「0.02 實測會讓輪緣削到頂前角」。

`H = 0.05` 那組在 `clearance = 0.02` 下只剩 `0.0196 mm` 餘裕就通過，屬於僥倖，
不能當作 `0.02` 可用的證據。

## 發現二：這個 crawl 不是準靜態穩定的

`quasi_static_support_polygon` 量的是「機身中心地面投影」到「支撐多邊形」的有號距離
（正 = 在內），沿用既有 Walk planner 的 `stability_margin` 慣例，用的是 repo 既有的
`_hull_signed_margin()`。

先確認正負號慣例沒搞錯：

```text
正方形中心      -> +1.0
正方形外        -> -1.0
三角形對角線上  -> ~0
```

然後量到最差 `-44.6 mm`，也就是機身跑到支撐三角形外面 4.5 cm。

原因是幾何上必然的：原本 crawl 的四個接觸點就落在髖關節正下方附近，抬起一隻腳之後
剩下的三角形，其對角邊幾乎正好通過機身中心。手算 FL 抬腳那一刻餘裕只有 `+33 mm`，
機身再往前走就掉出去。這正是所有靜態爬行步態都要先做**機身側向重心轉移**的原因，
而 Step 7 的 body 只有前進與升降、沒有側移。

---

# 2026-09-01：平地 Walk／障礙 crawl／平地 Walk 分段與可執行預設值

## 這次完成的功能

整條 CSV 現在明確分成三段，不再把全程都當成 obstacle crawl：

1. **進場平地**：直接取既有 `GaitGenerator3D(gait_type="Walk")` 的週期步態；第一個週期只重定時，讓 trigger 後從靜止平滑加速，關節路徑仍是 Walk 的路徑。
2. **障礙物附近**：切到準靜態 crawl；每次揮腳仍使用現有 Cartesian Bezier，輸入是當下足端起點與 planner 指定的落足終點。
3. **退場平地**：crawl 最後四次揮腳強制採用 recovery Walk 的四個落足點，之後經一段全站立 settling blend，再接回既有 Walk；最後一個週期重定時到靜止。

這不是任意 row 都可拼接。進場固定切在 Walk 的 liftoff row；目前回歸案例固定使用 3 個 approach cycles。2-cycle 切點在本案例的 event 37 會因支撐／地形接觸條件找不到可行 RR touchdown，因此會明確拒絕，不宣稱所有相位都可用。

## 已修正的中斷程式

- `handoff.py` 中實作已改名為 `blend_final_row_to()`，但 `traversal.py` 還引用不存在的 `snap_final_row_to()`，導致測試 collection 直接 ImportError；已統一名稱與呼叫參數。
- recovery 不再只留在 crawl：新增最後四腳 footprint resync、recovery Walk、起步與收尾 time warp。
- 舊 `dt=0.001 s` planner 預設不是可執行案例。planner 預設改為 `0.02 s`，exporter 再用 PCHIP 轉為 controller 固定的 `0.001 s`；controller CSV 契約不變。
- CLI `step_clearance` 預設改回完整幾何檢查曾通過的名目 `0.03 m`，不再使用已知可能削到障礙物前上角的 `0.02 m`。

## 現在的 CLI 預設案例

```text
障礙物前緣（相對初始 COM）  1.0 m
障礙物長度 x                 0.4 m
障礙物寬度 y                 0.8 m（只寫入 scene/metadata；planner 是 sagittal 2-D）
障礙物高度 z                 0.04 m
Walk 前進速度                0.10 m/s
planner dt                   0.02 s
controller/export dt         0.001 s
flat approach / recovery     3 / 3 cycles
```

不帶 gait 參數即可生成：

```bash
MPLCONFIGDIR=/tmp/legwheel_mpl .venv/bin/python \
  examples/gait/generate_obstacle_walk_csv.py \
  -o outputs/obstacle_walk_step9/obstacle_walk_x100_L400_W800_H040_v100_controller_dt001.csv
```

## 2026-09-01 實際生成結果

```text
traversal completed          True
stage approach/up/top/down/recovery  全部 True
first / last record          flat_walk / flat_walk_recovery
boundary q error             0 rad
maximum boundary qd error    0.3163 rad/s
ordinary world-fixed drift   1.000 mm
swing IK/FK tracking         0.196 mm
recovery IK branch blend     0.0167 rad
blend temporary contact move 4.999 mm（limit 6 mm）
maximum |beta|               37.9 deg / 40 deg
prep / trajectory / total    5000 / 58801 / 63801 rows
```

focused handoff regression：`12 passed in 137.38s`；再與 exporter／metadata
回歸合跑為 `6 passed, 34 deselected in 443.55s`。最終 CSV 另以腳本確認
`63801 x 12`、trigger row `5000`、首末段種類與所有 boundary reports。

## 必須保留的限制

- recovery 的最後狀態和 Walk 第一列雖有相同落足點，lowest-rim IK 仍落在不同解支；目前用整段平滑 blend 閉合，暫態接觸移動約 **5 mm**。它有獨立的 6 mm guard 並寫入 metadata/validation，不能把它說成 world-fixed 或零滑移 handoff。
- `full_geometry_checked=False`：這次生成沒有直接跑 Webots／實機，也沒有在本次 generator call 注入完整腿幾何 checker。結果只代表離線運動學與目前列出的檢查通過。
- 寬度 `0.8 m` 尚未進入有限寬 3-D 碰撞規劃；它目前用來固定 Webots/場景障礙物尺寸並保存在 metadata。

## 2026-09-01：平地 Walk 扭轉的離線檢查

使用最終 4 cm CSV，只量進場／退場 `flat_walk`：

- 四腳 gamma 都在約 `-0.013°--+0.013°`，span `0.026°`；扭轉不是 CSV 主動命令大幅 gamma。
- 四腳 theta 範圍都相同（約 `81.94°--109.97°`），beta 範圍也相同（約 `-15.06°--16.89°`），只是依 `FL→RR→FR→RL` 錯相；未找到左右欄位或關節範圍不對稱。
- controller 的 A/B/C/D 映射仍是每 module 取 `[theta,beta]`，gamma 取第 `8+module` 欄，與 exporter 一致。
- 但 `vx=0.1 m/s, T=2 s, duty=0.75` 的 Walk 在逐列三腳支撐檢查中，最小 signed margin 約 `-0.11 mm`，幾乎在三角形邊界；200 個取樣中有 4 列略為負值。模擬的接觸／摩擦微小誤差足以讓它搖擺或偏航。
- 把 generator 的 `stability_margin` 從 0 改成 0.02，在這組直線參數下目前 `_compute_walk_bias()` 仍回傳全零 x/y bias，gamma 與支撐 margin 完全沒有改變；因此不能只切換這個參數就宣稱已修復。
- repo 內保存的舊 Walk CSV 是 `vx=0.03 m/s, T=4 s`，不是目前 `vx=0.1 m/s, T=2 s`。目前 obstacle CSV 的 flat 段雖沿用相同 generator，並不是相同的已驗證動態條件。

目前結論：離線命令沒有明顯左右不對稱；最可疑的是「近零準靜態支撐裕度 + 未經模擬驗證的較快 Walk 參數」。下一個隔離實驗應先只跑完全相同的 flat segment（不含 obstacle/crawl），確認扭轉可重現，再以固定地形／摩擦分別比較舊 `0.03 m/s` 與目標 `0.1 m/s`，不要先改 obstacle planner。

## 2026-09-01：預設 flat Walk 恢復舊速度與週期

依模擬中 `0.1 m/s, T=2 s` 平地段嚴重扭轉的觀察，CLI 預設改回 repo 內舊 Walk 基準：

```text
flat Walk vx       = 0.03 m/s
period             = 4.0 s
obstacle crawl step= 0.15 m
```

這三個量已正式解耦。不能直接用 `vx*T*duty=0.09 m` 同時取代 crawl step，因為 4 cm 障礙物的 wheel-face exclusion `0.10 m` 加 top edge margin `0.02 m`，最低需要約 `0.12 m`；`0.09 m` 會在規劃前即被拒絕。`--vx` 現在只控制 flat Walk，`--step-length` 只控制 obstacle crawl。

無額外 gait 參數的完整 4 cm 預設已實際生成成功：

```text
traversal/stages       全部 True
Walk vx / period       0.03 m/s / 4.0 s
crawl step             0.15 m
boundary q / qd        0 rad / 0.0968 rad/s
contact drift          1.000 mm
swing tracking         0.192 mm
recovery blend         0.0166 rad / 5.033 mm
maximum |beta|         38.6 deg / 40 deg
prep/trajectory/total  5000 / 123541 / 128541 rows
```

輸出：`outputs/obstacle_walk_step9/obstacle_walk_x100_L400_W800_H040_flatV030_T4_controller_dt001.csv`。這仍是離線運動學結果，尚未證明舊參數在目前 Webots 場景一定不扭；只是先恢復與舊 flat Walk 相同的速度／週期，再做隔離驗證。

## 2026-09-01：修正 flat Walk 揮腳高度被 obstacle clearance 汙染

第二次檢查發現上一版仍不等同舊 Walk：`build_walk_generator()` 把 obstacle Bezier 的 `step_clearance_m=0.03` 同時餵給 periodic flat Walk 的 `step_height`。因此雖然 flat 速度與週期已回到舊值，平地抬腳仍只有約 4 cm，看起來像拖腳。

直接比較 `vx=0.03 m/s, T=4 s`：

| flat `step_height` | swing 最低輪緣最高 world-z | swing body-relative x range |
| --- | ---: | ---: |
| 錯誤共用 `0.03 m` | `39.8 mm` | `53.8 mm` |
| 舊 Walk `0.12 m` | `127.3 mm` | `49.7 mm` |

所以主要錯誤是抬腳高度，不是完全沒有前後步長。現已新增獨立的 `flat_walk_step_height_m`／CLI `--flat-step-height`：

```text
flat Walk step height       0.12 m
obstacle Bezier clearance   0.03 m
obstacle crawl step length  0.15 m
```

修正後完整 4 cm traversal 已重新生成成功，五個 stage 仍全過，輸出為：

`outputs/obstacle_walk_step9/obstacle_walk_x100_L400_W800_H040_flatV030_T4_flatStep120_controller_dt001.csv`

注意：`0.12 m` 是 repo 內舊 CSV `...S0.120_P4.0...` 使用的名目 flat swing height；目前只是恢復相同 generator input，仍需用只含 flat Walk 的模擬確認實際前進與接觸表現。

### 更正：不能把 `S0.120` 單獨搬過來就稱為舊 Walk

實際模擬回報 `flatStep120` 比前版更晃。逐列檢查使用者實際執行的
`0901_3_...flatStep120...csv`：

- CSV row 0 的四腳 theta 都是 home `17°`；這屬於前 5000-row transform。
- row 4999／trigger 第一列約為 `FL 108.26°, FR 107.51°, RR 107.50°, RL 108.33°`。
- trigger 後 flat swing 的 theta 最低約 `37.94°`，不是 17°；但 `0.12 m` step height 造成每腳約 `70.4°` 的巨大 theta excursion，確實可能比 3 cm 版本更晃。
- 舊保存 CSV 不是只有 `vx=.03,T=4,S=.12` 三個參數：它還使用 `LaunchController(n_ramp=3,ramp_floor=.1)`，第一列腿姿也和目前自訂 time-warp 不同。目前 obstacle flat adapter 會重取樣、time-warp、選 liftoff handoff row 並重標 phase，因此**不是原封不動呼叫舊 flat Walk CSV pipeline**。

所以先前「問題已找到，只是 flat height」的結論過度簡化，`flatStep120` 不應視為已修復版本。正確下一步是把原始 `generate_hardware_csv.py + LaunchController` 的 flat command/phase 當成基準，先做 bitwise／逐列比較，再設計只發生在末端的 crawl handoff；不能繼續從舊檔名猜一個參數搬進目前 time-warp。

## 2026-09-01：flat approach 改為直接使用原始 LaunchController

已移除 obstacle traversal 進場使用的自訂 full-speed Walk time-warp，新增
`legacy_launch_flat_approach_segment()`：

1. 直接呼叫 `LaunchController._generate_launch_sequence_with_phase()`。
2. 預設完全沿用原始 launch：3 cycles，速度比例 `10% → 55% → 100%`。
3. launch 後接 `GaitGenerator3D.generate_full_gait()` 的 3 個 steady cycles。
4. 只額外加入一個 cycle-boundary liftoff row，供 crawl 繼承；不修改前面的 Walk command path。
5. flat step height 改用 `generate_hardware_csv.py` 的正常預設 `0.04 m`；撤回從單一實驗檔名誤搬的 `0.12 m`。

在相同 `dt=0.02` 與參數下，新的 flat block 和直接呼叫原始 LaunchController 比較：

```text
maximum command difference = 0
maximum phase difference   = 0
flat theta range           = 81.41°--108.33°
flat start/end body x      = 0.000 / 0.558 m
```

原始 LaunchController 的限制也保留並明講：trigger 後第一個 10% cycle 並非零 joint-rate launch，因為 swing height 不隨 vx 一起縮放；第一 interval 在測試案例約 `1.24 rad/s`。這次目標是先忠實恢復原始 pipeline，不能再宣稱它由靜止平滑起步。

最終 4 cm 輸出：

`outputs/obstacle_walk_step9/obstacle_walk_x100_L400_W800_H040_legacyLaunch_flatV030_T4_flatStep040_controller_dt001.csv`

```text
traversal stages             all True
prep / trajectory / total    5000 / 123301 / 128301
boundary q / qd              0 rad / 0.0968 rad/s
handoff regression           12 passed in 68.98 s
```

前 5000 row 仍依 controller 契約由 theta home `17°` transform 到第一個 gait pose；17° 不屬於 trigger 後 flat Walk swing。若模擬在 trigger 後再次回到 17°，應檢查 panel 選到的檔案與 controller row boundary，而不是再改 gait step height。
