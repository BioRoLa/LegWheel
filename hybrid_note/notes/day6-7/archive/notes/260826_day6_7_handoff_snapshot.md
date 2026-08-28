# Day 6–7 Rolling Feasibility 進度整理（完成至 Step 6.75）

這份筆記整理 Day 6–7 到 Step 6.75 的研究進度，目的有兩個：

1. 之後寫 paper 時，可以回頭確認方法是怎麼一步一步發展出來的。
2. 下一個對話可以直接閱讀這份文件，接著做 Step 7，不必重新翻完整段對話。

目前完成的不是完整四足 Hybrid Gait planner，而是：

> 一套 single-leg、2D、已知 rectangular obstacle 下的 right-rim roll-up 與 recovery feasibility prototype。

最新完成的 Step 6.75 流程是：

```text
right-rim roll-up
→ 在 obstacle top 往前滾指定距離
→ 解除 right-rim contact
→ 抬腳
→ 空中同步 retract theta 與 reset beta
→ foot rim 朝下
→ foot-rim touchdown on obstacle top
```

---

## 一、Day 6–7 一開始想解決什麼問題

Day 1–2 先定義 offline planner 的資料格式。Day 3–5 建立 terrain-aware contact query，可以回答單一姿態下：

- 哪些 rim samples 靠近 terrain
- 哪些是 contact candidates
- 哪些 rim 或 link 穿入 obstacle
- 接觸的是 ground、front face、top 還是 back face

Day 6–7 接著要回答：

> 找到 contact candidate 以後，這個 leg 是否真的可以沿 obstacle leading edge 滾上去？滾上去以後，又要怎麼恢復到 foot rim 可以站立的姿態？

這裡刻意先做最小問題：

```text
single leg
2D x-z plane
gamma = 0
flat ground + one rectangular obstacle
approach direction = +x
```

目前沒有直接做完整 planner、FSM、四足 stability 或硬體控制，而是把每一個物理問題分段驗證。

---

## 二、整個研究發想脈絡

### Step 1：建立單一姿態測試場景

輸入 `theta、beta、hip_x、hip_z`，畫出 hip origin、完整 leg-wheel geometry、foot/left/right rims、ground 與 rectangular obstacle。

這一步沒有讓腳動，只是先回答：

```text
給定一組姿態時，腳和障礙物的相對位置到底是什麼？
```

### Step 2：把 planned contact 與 collision 分開

研究要刻意利用 right rim 接觸 obstacle front face，因此不能把所有 vertical-face interaction 都叫做 collision。

目前 query 至少區分：

```text
VALID_RIGHT_RIM_FACE_CONTACT
VALID_RIGHT_RIM_TOP_CONTACT
VALID_OTHER_RIM_CONTACT
INVALID_LINK_COLLISION
INVALID_GEOMETRY_PENETRATION
NO_CONTACT
```

重要語意：

```text
right rim 正常碰 front face
→ valid contact = true
→ collision = false

leg link、foot structure 或不允許的 geometry 穿進 obstacle
→ collision = true
```

query 會保留所有 candidates 與 collisions。因此一個姿態即使有 candidate，也可能同時 collision；這種姿態不會被後續 trajectory 接受。

### Step 3：固定 theta，只改 beta 測試 roll-up

固定一個 `theta_climb`，從手動指定的 initial beta 開始逐步改 beta，希望看到：

```text
right rim approach
→ right rim contact obstacle front face
→ right rim near leading corner
→ right rim contact obstacle top
```

每一步都重新呼叫 terrain-aware query，只接受合法 right-rim contact、合法 rim transition、無 penetration、無 link collision。

這一步證明的是：

> 固定 hip pose 與固定 theta 時，是否存在一段 beta sequence 可以形成 climbing contact。

它還不等於機器人真的向前滾上去。

### Step 4：Sweep theta_climb

固定 obstacle 與 initial condition，用 brute-force 從 `theta_min` 掃到 `theta_max`，每次增加 `dtheta`。每個 theta 都重跑 Step 3，記錄 success、failure reason、final beta、final rim 與 final surface。

目前 notebook 示範中：

```text
58°：不可行
59°：可行
60°：可行
```

這裡找到的是 theta candidates，不是最佳化出的唯一 theta。

### Step 4.5：加入真正的 forward rolling

固定 hip pose 只改 theta/beta，無法證明 hip 與接觸點真的往前移動。因此 Step 4.5 加入：

```text
hip_x 往 +x 移動
上一幀 theta/beta 作下一幀 initial guess
追蹤同一個 right-rim contact
每一步重新查詢 contact/collision
```

後來又修正 distance semantics，現在分別記錄：

```text
hip forward displacement
contact point forward displacement
```

兩者不能混為一談。hip 往前 5 cm，不代表 rim 在 obstacle top 的接觸點也往前 5 cm。

### Phase B：Contact continuation

front face、corner 與 top 不是互不相關的姿態，因此加入：

- front face 上持續追蹤 right rim
- leading corner 附近保持同一個 rim
- 切換到 top 時保持 theta/beta 與 rim sample 連續
- 每一步仍通過 terrain-aware query

這讓結果從「一堆獨立 candidate poses」變成一條連續 contact path。

### Phase C：真正的 top roll distance

一開始「滾上去」只代表 right rim 剛接觸 obstacle top，可能只跨過 leading edge 一點點。後來加入使用者指定：

```python
top_roll_distance_m
```

成功條件改成：

```text
right rim 已接觸 obstacle top
AND top contact point 已沿 +x 前進指定距離
```

完成前不允許進入 retract/reset。目前 notebook 使用 `top_roll_distance_m = 0.10 m`，progress 根據 contact point 實際前進距離，不是 hip 位移。

### Step 5：保持 top contact，retract 到 theta = 17°

從成功 roll-up 終點開始，theta 逐步下降到 17°，beta 可局部調整；contact 必須合法，每一步不得 collision。

這一步只處理 retract，還不做 foot-rim reset。

### Step 6：theta = 17° 後做 foot-rim reset

固定 theta = 17°，逐步改 beta，最後希望：

```text
active rim = foot rim
surface = obstacle top
alpha 接近 0°
```

後來發現這個版本使用 unsigned contour arc length。beta 逆轉時仍會強迫 contact 往 +x，不符合 signed no-slip rolling。

因此 Step 6 舊版本只保留作：

> sequential pose-chain / geometry baseline，不再視為正式物理 rolling 結果。

### Step 6.5：同時 retract/reset，並修正 no-slip

Step 6.5 同時更新 theta 與 beta，並比較：

```text
forward_continuation
shortest_reverse
```

加入 signed horizontal no-slip：

```text
Hx_next + r_x(q_next, previous material sample)
= Hx_previous + r_x(q_previous, previous material sample)
```

簡單來說，是把上一幀真正接觸的 tyre material point 固定在世界 x，再由新 configuration 反推出 hip 位置。

修正後：

```text
forward_continuation
→ 可以保持 no-slip 並往 +x

shortest_reverse
→ 下一步物理上需要往 -x
→ NO_SLIP_REQUIRES_NEGATIVE_X_MOTION
```

這證明腳還壓在 obstacle top 上時，不能一邊逆轉，一邊又假裝保持 no-slip 往 +x 滾。

Step 6.5 forward branch 最後可到：

```text
theta = 17°
foot rim ready
required rotation ≈ 258°
contact forward distance ≈ 0.583 m
hip forward distance ≈ 0.557 m
```

但它需要很長的 obstacle top。加上 margin 約需 0.685 m；notebook 使用 1.0 m top 做完整示範。

### Step 6.75：AIRBORNE_RETRACT_AND_FOOT_RESET

為降低 obstacle top 長度需求，提出：

```text
right rim 已滾上 obstacle top
→ 解除這隻腳的接觸
→ 空中把 theta 收到 17°
→ 同時把 foot rim 轉到底下
→ 再落回 obstacle top
```

這樣不必沿 obstacle top 滾完整個 right → left → foot rim 距離。

---

## 三、Step 6.75 現在實際做了什麼

Step 6.75 直接使用成功的 Step 4.5 final frame，不手動重建起點。

```text
TAKEOFF_CONTACT
→ LIFTOFF
→ AIRBORNE_RETRACT_RESET
→ FOOT_ALIGN_DESCENT
→ TOUCHDOWN
```

### TAKEOFF_CONTACT

起點必須是 Step 4.5 成功、right rim 接觸 obstacle top、top-roll distance 已完成，而且沒有 invalid collision。

### LIFTOFF

保持起始 theta/beta，把 hip 往上移。`airborne_clearance_m` 採保守定義：sampled rims 與既有 links 都必須高於 obstacle top 指定距離。

### AIRBORNE_RETRACT_RESET

離地後同步執行：

```text
theta → 17°
beta → foot-rim-down configuration
hip_x → touchdown 所需位置
hip_z → 保持 obstacle clearance
```

空中不要求 valid contact，因為這隻腳已卸載；但每個 sampled frame 仍呼叫 terrain-aware query，檢查 rim/foot penetration、link collision、joint limits 與 clearance。

### FOOT_ALIGN_DESCENT / TOUCHDOWN

theta 到 17°、foot rim 朝下後，保持最終 theta/beta，逐步降低 hip_z。最後必須滿足：

```text
theta = 17°
active rim = foot rim
alpha ≈ 0°
surface = obstacle top
collision = false
```

---

## 四、Step 6.75 兩個 rotation branches

空中沒有 no-slip constraint，因此 beta 可以走兩個方向。

### `forward_continuation`

沿 Step 4.5 原方向繼續轉，不需 motor reversal，但路徑較長。

```text
required beta rotation = 259°
maximum hip lift = 0.0150 m
success = true
```

### `shortest_reverse`

離地後反轉 beta，走最短路徑把 foot rim 轉到底下。路徑短，但未來要考慮 motor reversal、backlash 與加減速。

```text
required beta rotation = 101°
maximum hip lift = 0.02435 m
success = true
```

目前純運動學選擇：

```text
selected branch = shortest_reverse
```

依據是兩者都 collision-free 且 touchdown 合法後，選 accumulated `|delta beta|` 較小者。這不是馬達能量最佳化；加入 HT04 torque/speed/acceleration/energy model 前，不能斷言反轉一定更省能。

---

## 五、目前 notebook 的 Step 6.75 預設結果

```text
theta target = 17°
theta step = 2°
beta step = 2°
vertical step = 0.005 m
airborne clearance = 0.015 m
touchdown contact advance = 0.05 m
touchdown edge margin = 0.015 m
obstacle top length = 0.45 m
```

Step 4.5 roll-up end 約為：

```text
right-rim contact = (0.201018, 0.10) m
theta = 60°
beta ≈ -101°
```

Step 6.75 touchdown 約為：

```text
foot-rim contact = (0.251018, 0.10) m
theta = 17°
beta = 0° equivalent configuration
alpha = 0°
```

contact point advance = 0.05 m，但 hip forward displacement 約 0.02380 m。兩者不同，因為 leg configuration 同時改變。

兩條 branch 的 accepted frames 都滿足：

```text
collision = false
joint_limits_ok = true
final foot_rim_ready = true
```

---

## 六、目前整體策略

目前逐漸形成三種 recovery classes。

### A. Full Rolling Recovery

```text
right rim roll-up
→ 保持 contact 沿 top rolling
→ theta retract
→ right/left/foot rim transition
→ foot rim ready
```

對應 Step 6.5 forward branch。優點是 leg contact 維持較久；缺點是 obstacle top 必須夠長。

### B. Rolling-Assisted Swing / Airborne Reset

```text
right rim roll-up
→ 完成安全 top-roll distance
→ unload and liftoff
→ airborne retract/reset
→ foot-rim touchdown
```

對應 Step 6.75。利用 right rim 幫助爬 leading edge，但不要求 top 足夠長到完整滾回 foot rim。

### C. Direct Swing Up

```text
不使用 right-rim leading-edge rolling
→ 直接卸載抬腳
→ 跨過 obstacle front edge
→ foot-rim touchdown on top
```

目前只有策略討論，尚未實作。它可能適合較低、可以直接跨過的障礙物；較高障礙物可能需要 right-rim contact-assisted roll-up。

未來 planner 可先檢查 direct swing clearance，再決定使用 Direct Swing 或 Rolling-Assisted Swing，而不是所有障礙都固定同一策略。

---

## 七、新增或主要修改的 Python 檔案

Markdown 與 notebook 不逐一解釋，以下以 Python 為主。

### 1. `hybrid_note/scripts/experiments/single_leg_rolling_scene_2d.py`

Day 6–7 最主要的實驗檔。Step 1 到 Step 6.75 幾乎都集中在同一支，沒有為每個 step 一直新增 Python 檔。

可以把它理解成：

> 建立 single-leg obstacle scene，呼叫既有 kinematics/contact query，產生 roll-up、retract、reset、airborne recovery trajectory，並輸出圖、動畫與 CSV。

主要功能：

- `build_single_leg_rolling_scene_2d()`
  - 建立 ground/rectangle scene，重用既有 leg sampler
- `plot_single_leg_rolling_scene_2d()`
  - 畫 hip、完整 leg、各 rims、terrain 與 query 結果
- `run_fixed_theta_right_rim_roll_up_2d()`
  - 固定 theta，只更新 beta
- `sweep_right_rim_roll_up_theta_2d()`
  - brute-force theta sweep
- `run_forward_right_rim_roll_up_2d()`
  - Step 4.5 front/corner/top continuation 與真正 top-roll distance
- `evaluate_right_rim_retract_readiness_2d()`
  - 判斷是否完成 top roll 並可進入 retract
- `run_retract_to_wheel_2d()`
  - Step 5 保持 contact 收到 theta = 17°
- `run_wheel_reset_roll_2d()`
  - Step 6 legacy reset；只保留作 pose-chain baseline
- `run_retract_and_reset_branch_2d()`
  - Step 6.5 coupled retract/reset 與 signed no-slip
- `run_retract_and_reset_comparison_2d()`
  - 比較接觸狀態下兩個 rotation branches
- `run_airborne_retract_and_foot_reset_branch_2d()`
  - Step 6.75 liftoff、空中同步 retract/reset、descent、touchdown
- `run_airborne_retract_and_foot_reset_comparison_2d()`
  - 比較順轉與最短反轉 airborne branches
- 各種 `rows` / `write_*_csv` / `plot_*` / `animate_*`
  - 保存表格、CSV、比較圖與完整動畫

### 2. `legwheel/planners/hybrid/contact_detection_2d.py`

terrain-aware contact/collision query 核心。Day 6–7 沿用並擴充，沒有在 experiment 檔重寫另一套 collision model。

- `ContactCandidate2D`：rim、alpha、contact point、surface、gap、sample index
- `GeometryPenetration2D`：rim/foot penetration point 與 depth
- `LinkCollision2D`：link segment 穿入 obstacle 的資訊
- `ContactStatus2D`：區分合法 right-rim face/top contact、其他 rim、collision、penetration、no contact
- `query_contact()`：統一回傳 candidates 與 collision records
- `plot_contact_query_2d()`：畫 valid contacts 與 invalid collision geometry

這個檔只回傳客觀 query，不會替 planner 選 active contact、ROLL/SWING 或下一個 gait phase。

### 3. `legwheel/planners/hybrid/geometry_2d.py`

負責把既有 LegWheel geometry 接到 2D terrain query，不是新的 leg model。

- `HipPose2D`：hip world x-z pose
- `SampledLegGeometry2D`：rim samples、F/L/R/N、alpha、link segments
- `sampled_leg_geometry_from_legacy_records()`：既有 sampler 到 query geometry 的 adapter
- `legacy_leg_link_segments_2d()`：從 `PlotLeg.forward()` 取得六條 linkage centerlines
- hip/world frame transforms

### 4. `legwheel/planners/hybrid/terrain_2d.py`

Day 3–5 建立、Day 6–7 沿用，負責 flat ground、rectangular obstacle、top/front/back surface IDs 與 terrain visualization。

### 5. `legwheel/planners/hybrid/terrain_query_2d.py`

Day 3–5 建立、Day 6–7 沿用，負責 point-to-surface signed gap、projection、finite range、solid penetration 與 depth。

### 6. `hybrid_note/scripts/kinematics/ground_contact_single_pose.py`

既有 rim geometry sampler。Day 6–7 重用 `sample_contact_geometry_points()` 取得 foot/left/right physical rim arcs，因此沒有重寫 leg-wheel model。

### 7. `legwheel/visualization/plot_leg.py`

既有完整 leg visualization 與 forward geometry source。Day 6–7 用它畫 linkage/wheel，也從相同 geometry source 取得 link endpoints。

### 8. `legwheel/planners/hybrid/__init__.py`

集中 export terrain、geometry、contact query 類別與函式，讓 experiment 使用統一介面。

### 9. `tests/test_single_leg_rolling_scene_2d.py`

Day 6–7 regression tests，涵蓋 fixed scene、gamma guard、visualization、fixed-theta roll-up、theta sweep、Step 4.5 distance semantics、Step 5、Step 6、Step 6.5 signed no-slip、Step 6.75 clearance/touchdown/CSV。

目前結果：

```text
14 passed
```

---

## 八、目前輸出資料與 visualization

獨立 dashboard：

```text
hybrid_note/notes/hybrid_gait_day6_7_progress_dashboard.ipynb
```

已完整執行並寫回，包含 fixed scenes、contact/collision cases、theta feasibility、Step 4.5 roll-up animation、top-roll progress、Step 5、Step 6 legacy、Step 6.5 branches，以及 Step 4.5 + Step 6.75 完整動畫。

主要 CSV：

```text
day6_7_step4_theta_sweep.csv
day6_7_step4_5_forward_rolling.csv
day6_7_step5_retract_to_wheel.csv
day6_7_step6_wheel_reset_roll.csv
day6_7_step6_5_retract_reset_branch_summary.csv
day6_7_step6_5_retract_reset_frames.csv
day6_7_step6_75_airborne_branch_summary.csv
day6_7_step6_75_airborne_frames.csv
```

Step 6 CSV 是 legacy pose-chain；正式物理比較優先看 Step 6.5 與 Step 6.75。

---

## 九、座標與角度定義

```text
[x, z]
+x：機器人前進方向
+z：向上
```

```text
ground z = ground_height
obstacle front x = x_start
obstacle top z = ground_height + height
obstacle back x = x_start + top length
```

Python API 內部用 radians；notebook 多數參數用 degrees，再轉成 radians。gamma 固定 0，theta/beta 沿用既有 LegWheel kinematics。

global rim alpha：

```text
left rim:  -180° ～ -40°
foot rim:   -40° ～  40°
right rim:   40° ～ 180°
```

---

## 十、已驗證與尚未驗證

### 已驗證

- single-leg 2D geometry
- ground + one rectangle
- right-rim front-face planned contact
- front/corner/top continuation
- contact-point-based top-roll distance
- sampled rim/link collision query
- fixed-theta roll-up 與 theta sweep
- retract 到 17°
- Step 6.5 signed no-slip branch
- Step 6.75 兩個 airborne beta directions
- foot-rim touchdown on obstacle top
- CSV、plot、animation 與 regression tests

### 尚未驗證

- 四足 whole-body stability 與其他三腳 stance trajectories
- body roll/pitch/yaw 配合
- contact force、friction cone 與 dynamic roll-up
- HT04 torque/speed/acceleration/energy
- motor reversal backlash
- dynamic liftoff/touchdown impact
- 3D terrain、online replanning、hardware execution
- Direct Swing Up implementation
- Step 7 之後的完整 recovery/traversal

Step 6.75 成功只能解讀為：

> 在目前 sampled single-leg 2D kinematics 與 collision model 下，存在 collision-free airborne retract/reset path，最後能形成合法 foot-rim top contact。

不能直接解讀成整台機器人穩定或馬達一定做得到。

---

## 十一、下一個對話如何直接接 Step 7

下一個對話先讀本文件，以及：

```text
/home/chang/corgi_ws/icra hybrid/LegWheel/hybrid_note/notes/hybrid_gait_day6_7_progress_dashboard.ipynb
```

程式入口：

```text
/home/chang/corgi_ws/icra hybrid/LegWheel/hybrid_note/scripts/experiments/single_leg_rolling_scene_2d.py
```

若 Step 7 接目前預設策略，應直接使用：

```python
day6_7_step675_result.selected_result.final_frame
```

不要手動重建近似 theta/beta/hip pose。

目前 selected final state：

```text
phase = TOUCHDOWN
theta = 17°
beta = 0° equivalent configuration
active rim = foot rim
alpha = 0°
surface = obstacle top
contact point ≈ (0.251018, 0.10) m
collision = false
foot_rim_ready = true
```

Step 7 開始前應先決定：

1. 從 Step 6.75 selected airborne branch 接續，還是也保留 Step 6.5 full rolling branch。
2. Step 7 是否開始 re-extension / stance loading。
3. re-extension 時 foot contact 是否固定在 obstacle top。
4. hip/body 如何移動，以及其他三腳如何支撐。
5. 成功條件使用 leg geometry、body stability，或兩者一起。

建議延續：

- 不覆蓋 Step 1–6.75 baseline
- 繼續擴充同一支 `single_leg_rolling_scene_2d.py`
- notebook 往後追加 Step 7
- 每一步保存 failure reason
- 每一幀繼續呼叫 terrain-aware query
- 清楚標記 single-leg kinematics 與 whole-body assumption 的界線

---

## 十二、執行與驗證

```bash
cd "/home/chang/corgi_ws/icra hybrid/LegWheel"

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
MPLCONFIGDIR=/tmp/day67_test_mpl \
PYTHONPATH=. \
./.venv/bin/pytest -q tests/test_single_leg_rolling_scene_2d.py
```

目前結果：

```text
14 passed
```

Notebook 已完整執行並寫回；可從 Jupyter 開啟：

```text
hybrid_note/notes/hybrid_gait_day6_7_progress_dashboard.ipynb
```

---

## 十三、最簡短的研究結論

截至 Step 6.75，目前已經證明：

1. right rim 接觸 obstacle front face 可以是 planned contact，不必一律視為 collision。
2. 某些 theta_climb 可以形成 front → corner → top 的 right-rim contact sequence。
3. candidate pose 不等於真的滾動；必須加入 hip motion、contact continuation 與 contact-point distance。
4. right rim 可以在 obstacle top 繼續滾指定距離，例如 0.10 m。
5. 保持 top contact 做完整 foot-rim reset 需要相當長的 obstacle top。
6. 接觸狀態下，逆轉 beta 卻強迫 contact 往 +x 不符合 signed no-slip physics。
7. right rim 滾上去後，可以解除接觸，在空中同步 retract 到 17° 並把 foot rim 轉到底下。
8. 順轉 259° 與最短反轉 101° 兩條 airborne paths 都幾何可行；純運動學暫時選最短反轉。
9. 這些仍是 single-leg 2D feasibility，不是完整四足 gait、動力學或硬體驗證。

下一步 Step 7 應從已驗證的 foot-rim touchdown final frame 往後接，不要重新從零建立 recovery 起點。
