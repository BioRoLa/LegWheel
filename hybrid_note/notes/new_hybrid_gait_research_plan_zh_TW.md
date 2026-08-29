# 新版 Hybrid Gait 研究方向與兩週開發計畫

> **用途**：整理目前新版 Hybrid Gait 的研究定位、規劃邏輯、程式架構、短期進度與實驗方向。
>
> 這份筆記可以直接放在本地專案中，之後交給 Codex 作為長期專案背景，讓它在修改 gait planning code 時能理解研究目標，而不是只從單一 `.cpp` 檔案猜測邏輯。
>
> 更新日期：2026-08-23

---

## 1. 研究最終目標

最終目標是替加入 **ABAD 自由度**的新型 leg-wheel robot 發展一套
**offline terrain-aware hybrid gait planner**。Planner 在執行前取得完整已知地形，預先產生整段 traversal 的 body、contact、stance / swing 與 joint reference trajectory；robot runtime 只追蹤這份 deterministic trajectory。

目標地形為：

> **左右不均、具有離散高度差的崎嶇地形**，初期以不同高度、不同左右配置的長方體障礙物組成。

希望處理的情況包含：

- 純輪模式無法跨越障礙物。
- 純輪模式勉強可以通過，但機身 roll / pitch 很大、不平穩。
- 純 walking 可以通過，但需要大量抬腿與 swing，因此能耗較高。
- Hybrid gait 可以盡可能利用 rolling，只在必要時使用 swing。

核心 motivation：

> **能滾就盡量滾，滾不過或滾動不穩定時才跨步，藉此保留輪式運動的效率，同時具有 walking 的障礙跨越能力。**

最終主要比較：

$$
\text{Wheel}
\quad vs. \quad
\text{Walk}
\quad vs. \quad
\text{Proposed Hybrid}
$$

希望 Proposed Hybrid 能做到：

- 比 Wheel 更好的障礙通過能力。
- 比 Walk 更低的能耗。
- 在左右不對稱地形上有較小的 body roll / pitch。
- 減少不必要的 swing 次數。
- 充分利用 leg-wheel rim 的 rolling contact。

---

# 2. 研究定位

不要把研究描述成：

> 「規劃一條軌跡，讓一隻腳跨上一個長方體。」

這樣容易變成一般 obstacle stepping / stair climbing。

研究定位為：

> **Offline multi-contact rolling–stepping gait planning for a leg-wheel robot with ABAD on known asymmetric discontinuous terrain.**

核心包含：

1. **Multi-rim contact information**
2. **Stance 階段的 continuous rolling**
3. **必要時才進行 swing repositioning**
4. **利用 ABAD 調整 lateral support geometry**

因此整個研究最重要的 planning question 是：

> **Given a known terrain, what sequence of rolling and stepping contacts should the robot use to traverse the terrain efficiently while satisfying collision-free motion and support-stability constraints?**

也就是：對完整已知地形預先規劃 contact / motion sequence；能維持連續接觸時使用 rolling，只有受到地形幾何、kinematic limit 或 stability constraint 限制時才 stepping。

本研究明確只處理 **offline trajectory planning**：不包含 online perception、depth-camera reconstruction、runtime adaptation 或 receding-horizon replanning，也不要求 planner real-time 執行。

---

# 3. 與舊版 Hybrid / WLW 的關係

舊版 Hybrid Gait 已經提供很好的底層架構：

```text
wlw_open.cpp
    -> runtime loop / motor command

GaitSelector / Simple_fsm
    -> gait 共用狀態、duty、swing phase

Hybrid::Initialize()
    -> 初始化四隻腳 phase

Hybrid::Step()
    -> stance: LegModel::move()
    -> swing: HybridSwing::generate() / Swing_step()

next_eta[i] = {theta, beta}
    -> motor command
```

舊架構最重要的概念是：

```text
STANCE
    -> 保持 rim contact
    -> 使用 LegModel::move()
    -> body 前進時，contact point 沿 rim 滾動

SWING
    -> 離地
    -> 將腳重新配置到下一個 touchdown state
```

因此新版研究 **不是把舊程式全部丟掉重寫**。

應該保留：

- gait timing
- duty
- swing phase
- `Hybrid::Step()`
- `LegModel::move()`
- motor command pipeline

主要新增的是執行前運作的：

> **Offline terrain-aware contact and motion planning layer**

舊版主要輸入仍偏向：

```text
velocity
step length
stand height
gait phase
```

但缺乏一般化的：

```text
world-frame foothold
terrain geometry
target rim
target contact state
per-leg terrain height
```

新版應該補上這一層。

---

# 4. 預計發展的三個主要技術內容

## Contribution A — Terrain-Aware Multi-Rim Contact Representation

舊 contact map：

$$
(\theta,\beta,\text{slope})
\rightarrow
(S,\alpha,p_c)
$$

主要適用於：

```text
flat ground
uniform slope
```

因為本質上是在某個 ground plane 下尋找 rim 的最低有效點。

新版應改成：

$$
(q,\mathcal T)
\rightarrow
\mathcal C
$$

其中：

$$
q=(\theta,\beta,\gamma)
$$

$$
\mathcal T=\text{known terrain geometry}
$$

輸出：

$$
\mathcal C
=
\{(S_j,\alpha_j,p_j,\text{collision},\text{margin})\}
$$

也就是：

> 對規劃中的 leg configuration 與已知 terrain，找出所有可能的接觸候選與碰撞狀態，供完整 traversal 的 offline search / selection 使用。

第一版只需要支援：

```text
flat terrain
rectangular obstacle
```

**不要一開始做 arbitrary point cloud 或完整 3D terrain。**

---

## Contribution B — Rolling / Swing Hybrid Motion Selection

Planner 不應該預設：

```text
看到 obstacle
    -> swing
```

而是至少考慮兩個 motion primitives：

```text
ROLL
SWING
```

可能出現：

```text
Pure rolling

ROLL -> SWING -> ROLL

Full swing
```

第一版可以完全 rule-based：

```text
if 存在 collision-free continuous rolling trajectory:
    ROLL
else:
    SWING
```

之後如果有時間，再考慮 cost function：

$$
J =
w_E E
+
w_s N_{\text{swing}}
+
w_p J_{\text{posture}}
+
w_c J_{\text{collision}}
+
w_m J_{\text{stability}}
$$

但 **第一版不要急著做 optimization**。

---

## Contribution C — ABAD-Based Stability Adjustment

ABAD 的主要用途不是單純讓腳左右擺動，而是：

> **改變 lateral contact position，進而改變 support polygon。**

假設第 $i$ 隻腳正在 swing，其餘三腳形成：

$$
\mathcal P_{\text{sup}}
=
\operatorname{ConvHull}\!\left(\{p_{c,j}\}_{j\ne i}\right)
$$

穩定條件：

$$
p_{\text{CoM},xy}
\in
\mathcal P_{\text{sup}}
$$

更保守可以要求：

$$
d
\left(
p_{\text{CoM}},
\partial\mathcal P_{\text{sup}}
\right)
>
d_{\min}
$$

ABAD：

$$
\gamma
\rightarrow
y_{\text{contact}}
\rightarrow
\mathcal P_{\text{sup}}
$$

也就是：

```text
調整 gamma
    ↓
改變腳的 lateral foothold
    ↓
改變 support polygon
    ↓
提高 stability margin
```

第一版使用 quasi-static heuristic 即可。

**不要一開始做完整 Whole-Body Optimization。**

---

# 5. 新 Planner 的輸入 / 輸出

新版 planner 是 **trajectory-level offline planner**，不再把 `stand_height` 當成唯一的 terrain-related parameter，也不以 runtime 當下的一次 ROLL / SWING 判斷作為完整 planner 定義。

### Planner input

```text
TerrainProfile                 # 規劃前已知的完整 structured terrain
initial robot / body state
goal / traversal path
robot geometry and kinematics
gait constraints              # joint/workspace/collision/stability/timing limits
```

必須明確區分：

```text
stand_height
swing_clearance / step_height
target_foothold_height
target_foothold_position
target rim
target alpha
```

建議建立：

```cpp
struct ContactState {
    RimId rim;
    double alpha;
    Eigen::Vector3d point_world;
    std::string terrain_surface_id;
};

struct SwingTarget {
    Eigen::Vector3d target_position;
    RimId target_rim;
    double target_alpha;
    double clearance;
};

struct ContactCandidate {
    RimId rim;
    double alpha;
    Eigen::Vector3d point_world;
    double terrain_gap;
    double edge_margin;
    bool collision_free;
    std::string terrain_surface_id;
};

struct TerrainProfile {
    // 第一版只支援：
    // flat + rectangular obstacles
};

struct BodyTarget {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};
```

### Planner output

輸出是涵蓋完整 traversal 的時間序列：

```text
body pose trajectory
per-leg theta / beta / gamma trajectory
per-leg contact state
per-leg rim / alpha
per-leg ROLL / SWING mode and swing phase
per-leg foothold position
stability margin
```

統一輸出成 deterministic `trajectory.csv`（或等價、具有明確 schema 的 trajectory file），供 simulation 與 real robot 使用同一份 reference。建議欄位至少包含：

```text
time
body_x, body_y, body_z, body_roll, body_pitch, body_yaw
leg0_theta, leg0_beta, leg0_gamma, ... leg3_theta, leg3_beta, leg3_gamma
leg0_mode, leg0_rim, leg0_alpha, ... leg3_mode, leg3_rim, leg3_alpha
leg0_foothold_x, leg0_foothold_y, leg0_foothold_z, ...
stability_margin
```

整體資訊流：

```text
Complete known TerrainProfile
Initial state + Goal / Traversal path + Constraints
        ↓
Offline contact / motion planner（規劃完整 traversal）
        ↓
Complete contact sequence + ROLL / SWING decisions
        ↓
Body trajectory + per-leg stance / swing trajectories + ABAD adjustment
        ↓
Complete (theta, beta, gamma) reference trajectory
        ↓
Deterministic trajectory file
        ↓
Runtime gait executor / existing joint position controller
```

Runtime 不執行 high-level terrain replanning。Encoder、IMU、motor current 與 Vicon 可用於 low-level tracking 和實驗量測，但不回饋到 online contact / motion replanning。

---

# 6. 一開始就要保留 Terrain Height

即使第一個測試只做平地，planner interface 也應該允許：

$$
z_{\text{TD}}\neq0
$$

平地只是：

$$
z_{\text{TD}}=0
$$

例如 50 mm 高的平台：

$$
z_{\text{TD}}=0.05
$$

因此同一套 planner 應該可以依序擴充：

```text
flat
    ↓
different touchdown height
    ↓
single rectangular obstacle
    ↓
multiple sparse obstacles
    ↓
left-right asymmetric obstacles
```

而不是每遇到一種 terrain 就換一套 gait logic。

---

# 7. Swing Planning 的選擇

目前舊程式有：

```text
LINEAR
CUBIC
FIVETIMES
OPTIMIZE
```

這些主要是在 joint space 規劃：

$$
\theta(t),\beta(t)
$$

但如果要跨 obstacle，比較自然的是直接規劃 contact point：

$$
p_c(t)
=
\begin{bmatrix}
x_c(t) & y_c(t) & z_c(t)
\end{bmatrix}^{\mathsf T}
$$

再透過 inverse kinematics：

$$
p_c(t)
\rightarrow
(\theta,\beta,\gamma)
$$

第一版建議：

> **Cartesian Bezier swing trajectory**

因為可以直接控制：

- touchdown position
- touchdown height
- obstacle clearance
- approach direction
- touchdown velocity

舊的 FIVETIMES 可以留作 baseline。

目前不要投入大量時間改善 `OPTIMIZE`。

Swing trajectory 本身不一定要成為 paper contribution。

---

# 8. Terrain-Aware Contact Query

第一個真正應該新增的核心 module：

```cpp
std::vector<ContactCandidate> queryContact(
    const LegPose& q,
    const TerrainProfile& terrain
);
```

對每一個 rim sample point：

$$
p_r(\alpha,q)
$$

轉換到 world frame：

$$
{}^{W}\!p_r
=
{}^{W}\!T_B
{}^{B}\!T_L(\gamma)
p_r(\alpha,\theta,\beta)
$$

如果 terrain 可以用：

$$
z=f(x,y)
$$

表示，則定義：

$$
d=z_r-f(x_r,y_r)
$$

分類：

```text
|d| < epsilon
    -> contact candidate

d < -epsilon
    -> collision / penetration

d > epsilon
    -> free
```

長方體障礙物可以先表示成：

$$
f(x,y)=
\begin{cases}
h_o, & (x,y)\in\text{obstacle footprint}\\
0, & \text{otherwise}
\end{cases}
$$

但注意：

> 真正的 rectangular obstacle 還包含 vertical face，因此 collision detection 不能只檢查 height field 的 top surface。

第一版可以直接 sampling。

**不用追求 analytical solution。**

---

# 9. Contact Transition Information

新版 contact representation 最好不要只有：

```text
rim = 4
```

而是保留：

```text
rim
alpha
contact point
distance to rim boundary
collision state
terrain surface ID
```

例如定義：

$$
m_{\text{edge}}
=
\text{distance to nearest rim-transition boundary}
$$

這可以用來描述：

> 目前踩在這個 contact state 時，再往前一點是否很容易被動轉移到另一個 rim。

第一版不需要 probability model。

先用 deterministic geometric margin 即可。

---

# 10. Rolling-Assisted Obstacle Traversal

這是一個很重要的研究實驗。

不要預設所有障礙都一定要 swing。

例如：

```text
            _________
           |
___________|
```

固定或指定 hip trajectory，掃描：

$$
h_o
$$

以及：

$$
(\theta_0,\beta_0)
$$

檢查是否存在 continuous contact trajectory：

$$
q_0
\rightarrow
q_1
\rightarrow
\cdots
\rightarrow
q_N
$$

同時滿足：

```text
valid rim contact
no collision
joint limit
leg workspace
continuous forward progress
```

最後可能得到：

```text
LOW obstacle
    -> continuous rolling

MEDIUM obstacle
    -> rolling-assisted hybrid / short swing

HIGH obstacle
    -> full swing required
```

這張 feasibility map 很可能可以成為 paper 的重要結果。

---

# 11. 前兩週的真正目標

前兩週 **不是完成整篇 ICRA**。

真正目標是：

> **做出可以在第三週開始系統性實驗的 research prototype。**

兩週後希望至少有：

- 單腳平地 hybrid trajectory
- rectangle terrain representation
- terrain-aware contact / collision query
- block rolling feasibility result
- 不同 touchdown height 的 swing
- 第一版 offline roll-vs-swing contact / motion sequence
- 四腳通過至少一個簡單 obstacle 的完整 precomputed trajectory 與 simulation
- 基本 support polygon
- 如果進度允許，再加入第一版 ABAD heuristic

---

# 12. 兩週逐日開發計畫

## Day 1–2 — 鎖定研究與程式架構

### 目標

確定：

```text
research question
planner input/output
software module boundary
coordinate convention
data structure
```

### 必須完成

- `TerrainProfile`
- `ContactState`
- `ContactCandidate`
- `SwingTarget`
- body / hip / foothold coordinate convention
- 一張完整 planner flowchart

### 完成標準

```text
Day 2 後原則上不再大改整體架構。
```

除非後續真的發現核心假設錯誤。

### 實作狀態（2026-08-23）

- [x] 四個核心資料結構已建立於 `legwheel/planners/hybrid/types.py`。
- [x] 座標、單位、模組邊界與 planner flowchart 已鎖定於
  `hybrid_note/notes/day1_2_hybrid_gait_architecture/hybrid_gait_architecture_day1_2_zh_TW.md`。
- [x] 資料契約測試已建立於 `tests/test_hybrid_planning_types.py`。

---

## Day 3–5 — 2D Terrain-Aware Contact Model

### Scope

只支援：

```text
flat terrain
single rectangular obstacle
```

### Implement

```cpp
queryContact(theta, beta, terrain)
```

或等價介面。

### 至少輸出

```text
rim
alpha
contact point
collision flag
terrain gap
edge margin
```

### Visualization

畫出：

- leg geometry
- obstacle
- active rim
- contact point
- collision point

### 完成標準

給定一組：

$$
(\theta,\beta)
$$

程式可以判斷：

```text
接觸平地
接觸 obstacle top
撞到 obstacle vertical face
沒有有效 contact
```

---

## Day 6–7 — 單腳 Rolling Feasibility

### 任務

建立單腳 + rectangular obstacle 測試。

掃描：

```text
obstacle height
initial theta
initial beta
approach configuration
```

### 核心問題

> 是否可以保持 continuous rim contact，直接利用 rolling / leg extension 上到 obstacle？

### 完成標準

產生例如：

```text
obstacle height vs rolling feasibility
```

或：

```text
initial configuration vs maximum rollable obstacle height
```

這是第一個重要 research checkpoint。

---

## Day 8–9 — 不同 Terrain Height 的 Swing

### 方法

只做一種：

```text
Cartesian Bezier swing
```

### Input

```text
start contact
target contact
target z
clearance
swing duration
```

### Check

- IK feasibility
- joint limits
- obstacle clearance
- touchdown configuration
- touchdown velocity

### 完成標準

同一套 planner 可以處理：

```text
zTD = 0
zTD = 20 mm
zTD = 40 mm
zTD = ...
```

不用換 gait logic。

---

## Day 10–11 — 第一版 Offline Roll-vs-Swing Trajectory Planning

> **2026-08-29 修訂。** 本節依 Day 6–7（Step 11R / 12R）與 Day 8–9（Step 1–9 + §26–§28）的實測結果重寫。
> 原版規則 `if rolling feasible: ROLL else SWING` 已被自己的數據推翻，原文保留在本節末「原始規則與它為什麼不成立」，供 paper 追溯設計演進。
> 完整發想脈絡與逐步實作項目見 `notes/day10-11/day10_11_roll_swing_selection_zh_TW.md`。

### 整合對象

```text
rolling feasibility     Day 6–7 Step 11R / 12R
+
swing planner           Day 8–9 generate_swing_2d / swing_onto_step_2d / swing_off_step_2d
```

### 修訂後的核心問題

不是「這個障礙該用 roll 還是 swing」，而是：

> **對同一塊 terrain，roll 與 swing 各自向 body trajectory 索取多少讓步？選索取較少的那一個。**

Planner 因此不再需要 body trajectory 當輸入，而是**輸出 body trajectory 的需求**，交給 Day 12 的四腳整合去滿足。

### 為什麼要改：三個實測結果

**1. 二值可行性在上升側沒有鑑別力。**

```text
Day 8–9 §28.1   swing onto 160 mm  ->  OK（hip +60 mm、liftoff +30 mm）
                swing onto 200 mm  ->  FAIL（"needs a different approach geometry"）
Day 6–7 §11R    rolling h = 0.16   ->  feasible cell = 0
                但 roll_up 本身在 160 mm 是 10/10（失敗在 roll_down）
```

上升側兩種都做得到，差別只在代價：rolling 不需要 hip 讓步，swing 需要 60 mm。
把 cell 的值從 `feasible / infeasible` 換成「向 body 索取多少」，這個平手才會變成結果。

**2. swing 的可行性主要不由 touchdown height 決定。**

Day 8–9 §26.5(6)：同一組 case，起點離台階 0.10 m 撞三個、0.20 m 全過。
輪半徑 0.145 m 大於「接觸點到障礙的距離」時，接觸點還沒開始動輪胎就進去了。
而 `swing_onto_step_2d` 目前把 `approach_distance_m` 固定在 0.20 m 且**不搜尋它**——
失敗時只會回報 `start_knob_name` 叫呼叫端去改。**這一維是 Day 10–11 要補的主軸。**

**3. rolling 的限制是三維，不是一維。**

```text
(obstacle_height × theta_climb × top_length)

h = 0.12   feasible theta 只剩 [45°, 55°]（2 cell）——不是單調遞減
h = 0.16   feasible cell = 0                    ——rolling 天花板
top_length 有下界也有上界：h = 0.06、theta = 85° -> 0.215–0.285 m
```

所以 decision rule 的輸入是一組 rectangle 幾何，不是一個高度。

### 修訂後的第一版規則

**上升與下降是兩個獨立決策。** 策略空間不是 Day 6–7 §8 的三類，而是 2×2 + 1：

```text
ascent  in {ROLL_UP, SWING_UP}
descent in {ROLL_DOWN, SWING_DOWN}          -> 4 種（都落腳在 obstacle top）
外加 SWING_OVER（一次越過，不碰 top）        -> 合計 5 種
```

| # | ascent | descent | 舊名 | 什麼時候勝出 |
| --- | --- | --- | --- | --- |
| 1 | ROLL_UP | ROLL_DOWN | Strategy A | 低矮且 top 夠長 |
| 2 | ROLL_UP | SWING_DOWN | Strategy B | h 超過 roll_down 天花板（160 mm） |
| 3 | SWING_UP | ROLL_DOWN | **原三分類沒有** | **top 太短，付不起 `L_transition`** |
| 4 | SWING_UP | SWING_DOWN | Strategy C 的一種 | 高，且 top 夠長可落腳 |
| 5 | — SWING_OVER — | Strategy C 的另一種 | top 很短 |

兩個修正：舊的 Strategy C 把 #4（落 top）與 #5（不落 top）混成一類，它們的 top-length 需求
完全相反，拆開；#3 完全缺席，而它正好攻擊 rolling 最硬的限制——`LEFT_RIM_READY` 是抵達後緣時
檢查的前提條件，`L_transition`（roll-up 出口 → LEFT_RIM_READY 的 top 前進距離）就是 0.20–0.27 m
那個 top-length 下界的來源。改用 swing 上去，原則上可以**直接落在後緣附近的 LEFT_RIM_READY**，
跳過這筆預算。

決策規則：

```text
for each terrain feature:
    for each pair in {ROLL,SWING} x {ROLL,SWING}  +  SWING_OVER:
        feasible(pair) =  ascent 可行 AND descent 可行
                      AND L_top >= 該 pair 的最小 top-length 預算
                      AND top 上的狀態接得起來
        cost(pair)     =  兩段 body concession 的合成
    pick argmin

lexicographic，不調權重：
    feasible  >  body deviation  >  clearance margin  >  偏好 roll 較多者
```

**Strategy B（#2）是預設路徑，不是「有時間再擴充」。** 依 Day 8–9 §26.7，roll 與 swing 的強弱
互補且分在 traversal 的不同階段，切換點在**下降側**。

### top-length 預算：第二條結果線

五種組合各有自己的最小 top-length 預算，而且順序單調：

```text
ROLL_UP  + ROLL_DOWN     L_transition                 0.20–0.27 m（已量測）
ROLL_UP  + SWING_DOWN    roll-up 出口 -> 起跳距離      待量
SWING_UP + ROLL_DOWN     落點 -> 後緣 LEFT_RIM_READY   待量（可能極短）
SWING_UP + SWING_DOWN    落點 -> 起跳距離              待量（落 top 者中最短）
SWING_OVER               0（不碰 top），但需 stride >= L_top
```

> **越靠 swing 的組合，需要的 top length 越短。**

若被數據證實，`L_top` 從約束升格成與 height 平起平坐的策略選擇主軸——而這是 rolling 的二值
可行性 map 完全看不到的維度。

**關鍵未知數**：swing 能不能真的落在 θ=17° 的 left-rim contact，目前**沒有任何數據**（Day 8–9
所有 swing 的落點姿態都是 θ=60°；θ=17° 是最收縮、reach 最差的姿態）。#3 成立與否取決於此，
由 Step 2b 回答。

### 這兩天的主產出是 envelope 與 decision rule，不是完整 sequence generator

理由：ROLL 那側已有完整 feasibility map，SWING 那側只有零星 case。
**decision rule 缺的是後者那張表，不是串接程式碼。** 而且 envelope 本身就是 paper 的 Figure D，
sequence composer 即使只跑通兩條 demo，Day 10–11 的研究產出仍然完整。

### 完成標準（修訂）

```text
[必要]
  Step 0   scene / approach 幣別對齊：rolling 與 swing 用同一組障礙幾何與同一種 approach 度量
  Step 2   ascent  swing sweep   (height × approach clearance)  -> 最小 hip lift map
  Step 2b  SWING_UP 能否落在 LEFT_RIM_READY -> 決定策略 #3 是否存在
  Step 3   descent swing sweep   (height × takeoff distance)     -> 最小 hip hold map
           （起點分 arrival = ROLL_UP / SWING_UP 兩種）
  Step 4   rolling map 換成同一種幣別（RollConcession）
  Step 5   envelope 疊圖 -> 五個候選的 decision rule + Figure D + top-length 切片圖
  Step 6   segment 級 motion sequence schema（含取樣參數與 body requirement）

[目標]
  Step 7   sequence composer：ROLL+ROLL、ROLL+SWING、SWING+ROLL 各生一條完整、交接連續的 sequence
  Step 8   五個 terrain case 各落在一個 region，含負向驗證；至少一個 case 的最佳解是混合的
  Step 9   body requirement timeline，作為 Day 12 的輸入
```

Sequence 的每一段必須記下**取樣參數**（`arc_samples` / `sample_count` / `max_joint_step_rad`）。
Day 8–9 Step 9 已證明連續性限制與取樣密度耦合，不記就不可重現。

這不是 runtime online decision；執行階段只讀取已完成的 trajectory。

### 明確押後

```text
四腳 timing / body trajectory 生成      Day 12（Day 10–11 只輸出對它的需求）
cost function 權重調校                  第一版用 lexicographic，不調
online replanning                       不在範圍
1.2 mm rim 幾何差異                     沿用 Day 8–9 的決定：量化記錄、不修
                                        （交接點會第一次真的咬到，見 Step 0）
```

### 原始規則與它為什麼不成立（保留供追溯）

原始 Day 10–11 規則：

```text
if continuous rolling is feasible:
    ROLL
else:
    SWING
```

不成立的原因，按嚴重性排序：

```text
1. 決策粒度錯   rolling 不是一個 primitive，是五段鏈；決策發生在段邊界，不在障礙層級
                而且上升與下降是【兩個獨立決策】，原規則對整個障礙只選一次
2. 度量錯       上升側二值可行性平手，鑑別力在「向 body 索取多少」
3. 循環         原版要在不知道 body 怎麼走時決定 roll/swing，但可行性由 body 怎麼走決定
                （修訂版把方向反過來，循環自動消失）
4. schema 錯    wheel-mode 段沒有固定 contact point，無法用「每個 transition 的 rim / alpha」表示
```

這四點本身是 paper 中 motion selection 一節的設計論證來源，不要刪。

---

## Day 12 — 四腳 Full-Trajectory Integration

盡量 reuse：

```text
duty
swing_phase
Hybrid::Step()
next_eta
```

不要重新設計整個 timing system。

### 第一個測試

```text
symmetric obstacle
gamma = 0
```

### 完成標準

產生四腳完整、時間同步的 precomputed body/contact/joint trajectory，並讓機器人在 simulation 中至少完成一個簡單 obstacle traversal。

---

## Day 13–14 — 最小 ABAD Stability + Trajectory Generation Freeze

### 第一版 ABAD logic

每次一隻腳 swing：

1. 用另外三個 contact point 建 support polygon。
2. 計算 CoM projection。
3. 算 stability margin。
4. 如果 margin 太小，調整支撐腳 $\gamma$。
5. 重新計算 support polygon。

### 第一版可以很簡單

例如：

```text
swing right leg
    -> 將部分 support legs 往 lateral direction 展開
    -> increase stability margin
```

### 完成標準

至少做到：

- support triangle 計算正確
- gamma 可以改變 lateral foothold
- stability margin 有改善
- ABAD 調整已寫入完整 joint reference trajectory
- simulation 與 real robot 共用的 deterministic trajectory schema 固定

### Day 14 後

> **Freeze main architecture。**

開始做實驗，不要持續無止境重寫 planner。

---

# 13. 第三週開始：實驗 + Paper 同時進行

第三週不要等所有功能都完美才開始實驗。

實驗順序：

```text
flat hybrid
    ↓
single low obstacle
    ↓
single higher obstacle
    ↓
whole robot over one obstacle
    ↓
left-right asymmetric obstacle
    ↓
multiple sparse asymmetric obstacles
```

不要第一天就測最困難的 rough terrain。

---

# 14. 最終建議地形

建議最終實驗使用：

> **Known structured asymmetric rough terrain composed of sparse rectangular obstacles**

例如：

```text
Left side:

________      ____________
        |____|

Right side:

____________        ______
            |______|
```

或：

```text
Left:
_______┌─────┐________________
       │     │

Right:
______________┌───┐__________
              │   │
```

優點：

- 可重複
- 可量化
- obstacle height 可調
- 左右 asymmetry 可調
- 容易比較 Wheel / Walk / Hybrid
- 可以直接驗證 ABAD 的作用
- 比 random rocks 更容易做科學比較

---

# 15. 主要實驗 Baseline

主要比較只做：

```text
Wheel
Walk
Proposed Hybrid
```

不要做太多排列組合。

建議 metrics：

```text
success rate
traversal time
average velocity
energy consumption
Specific Resistance / COT
pitch RMS / peak
roll RMS / peak
minimum stability margin
number of swing events
rolling-distance ratio
```

定義：

$$
R_{\text{roll}}
=
\frac{\text{distance traveled under rolling support}}
{\text{total traversal distance}}
$$

這個 metric 很重要。

因為它可以證明：

> Proposed Hybrid 真的有利用 rolling，而不是換皮的 walking。

---

# 16. 建議 Ablation

第一組：

```text
Hybrid without ABAD
vs.
Hybrid with ABAD
```

使用左右不對稱 terrain。

比較：

```text
roll RMS
minimum stability margin
success rate
body attitude variation
```

如果還有時間，再做：

```text
fixed contact policy
vs.
terrain-aware contact policy
```

不要做：

```text
Cubic × Fifth-order × Bezier × Rim × ABAD × Step Height
```

這種大型排列組合。

---

# 17. 最低可接受的 Paper Contribution

比較弱的版本：

> 「我們設計了一條 trajectory，讓新機器人的腳可以跨上 block。」

應避免。

比較強的版本：

> **提出一套適用於具有 ABAD 自由度 leg-wheel robot 的 offline multi-contact rolling–stepping gait planner。給定已知的左右不對稱、離散高度地形，planner 預先產生完整 contact / motion sequence，顯式評估不同 rim 的接觸與 rolling 可行性，只有在 rolling 受地形幾何、kinematic limit 或 stability constraint 限制時才安排 swing，並利用 ABAD 調整 lateral support geometry，以保留 rolling 的能源效率與穩定通過能力。**

可能的 contribution bullets：

### 1. Terrain-aware multi-rim contact representation

- 處理 discontinuous terrain。
- 判斷 rim contact。
- 判斷 collision。
- 提供 rim transition information。

### 2. Rolling / swing hybrid motion selection

- 能滾就保持 rolling。
- terrain 不允許時才 swing。
- 減少 unnecessary swing。

### 3. ABAD-based lateral stability adaptation

- 改變 lateral support geometry。
- 提高 asymmetric terrain 上的 stability margin。

### 4. Experimental validation

比較：

```text
Wheel
Walk
Hybrid
```

驗證：

```text
energy
stability
speed
traversability
```

---

# 18. 第一版刻意不做的事情

目前先不要做：

```text
arbitrary full 3D terrain map
online depth perception
unknown-terrain planning
runtime terrain adaptation
receding-horizon replanning
real-time high-level planning requirement
full whole-body optimization
MPC
reinforcement learning
很多種 swing trajectory
probabilistic contact estimation
完整 dynamic stability model
```

第一版可以接受：

```text
complete terrain known before planning
structured rectangular obstacles
2D / 2.5D terrain representation
quasi-static stability
sampling-based contact query
rule-based roll/swing selection
offline full-trajectory generation
runtime joint-reference tracking
```

先把研究核心做出來。

---

# 19. 開發優先順序

遇到時間不足時，依照以下順序保留功能：

```text
1. Contact representation
2. Rolling feasibility
3. Swing to arbitrary touchdown height
4. Roll-vs-swing selection
5. Four-leg integration
6. ABAD stability
7. Hardware robustness
8. Optimization refinement
```

如果需要砍 scope：

> **從最下面開始砍，不要先砍最核心的 roll-vs-swing planning。**

---

# 20. 給 Codex 的程式架構原則

修改程式時，盡量保持：

```text
Planner
    決定：
        完整 traversal 的 contact / motion sequence
        contact 在哪裡
        使用哪個 rim
        ROLL or SWING
        body trajectory 與 target body/contact geometry

Kinematics
    負責：
        forward
        inverse
        contact geometry
        rolling motion

Gait Executor
    負責：
        讀取 precomputed trajectory
        duty / swing / stance phase 的既有執行語意
        將 reference 依時間送入 next_eta / command pipeline

Controller
    負責：
        theta / beta / gamma reference tracking
```

**不要把 terrain planning 全部塞進 `Hybrid::Step()`。**

優先建立小而可測試的 function：

```cpp
queryContact(...)
checkCollision(...)
checkRollingFeasibility(...)
generateSwing(...)
computeSupportPolygon(...)
computeStabilityMargin(...)
adjustABAD(...)
```

---

# 21. 舊程式的重要檔案

目前重要 reference：

```text
wlw_open.cpp
Simple_fsm.cpp
hybrid_gen.cpp
hybrid_swing.cpp
leg_model.cpp
```

概念上：

```text
wlw_open.cpp
    runtime / ROS / motor publishing

Simple_fsm.cpp
    shared gait state

hybrid_gen.cpp
    gait timing + stance/swing dispatch

hybrid_swing.cpp
    swing trajectory generation

leg_model.cpp
    forward / inverse / contact_map / move
```

能 reuse 的 low-level execution 儘量 reuse。

---

# 22. Research Checkpoints

## Contact Model Checkpoint

要能回答：

```text
哪個 rim 可以碰到 rectangular obstacle？
是否撞到 vertical face？
是碰 obstacle top 還是 ground？
```

---

## Rolling Checkpoint

要能回答：

```text
哪些低障礙可以 continuous rolling 上去？
最大可滾障礙高度是多少？
哪些 configuration 可以滾，哪些不行？
```

---

## Swing Checkpoint

要能回答：

```text
同一個 swing planner 能否到不同 touchdown height？
是否保證 obstacle clearance？
是否可以用指定 rim touchdown？
```

---

## Hybrid Checkpoint

要能回答：

```text
Offline planner 是否能對完整 terrain 產生可執行的 roll-vs-swing sequence？
Contact、mode、body 與四腳 joint trajectory 是否時間同步？
Roll 與 swing 各自向 body trajectory 索取的讓步，是否被量化且可比較？
Hybrid 是否比 swing-only 對 body trajectory 的要求更低？
    （2026-08-29 修訂：原本問「是否使用更少 swing」。Day 8–9 §28.1 顯示
     swing-only 在 160 mm 仍然可行，只是要 hip +60 mm，所以計數 swing
     次數沒有鑑別力，要比的是 body 讓步。）
```

---

## ABAD Checkpoint

要能回答：

```text
gamma 是否真的改變 support geometry？
是否提高 stability margin？
是否降低 asymmetric terrain 上的 body roll？
```

---

## Paper Checkpoint

最後結果應該能清楚呈現：

```text
Wheel:
    fail / unstable

Walk:
    succeed but higher energy

Hybrid:
    succeed
    preserve rolling
    lower energy than walk
    better stability than wheel
```

---

# 23. 兩週成功標準

兩週後若有以下成果，就視為成功：

```text
[ ] 單腳 flat hybrid planner
[ ] rectangular terrain representation
[ ] terrain-aware contact / collision query
[ ] rolling feasibility analysis
[ ] arbitrary touchdown height swing
[ ] 第一版 offline roll-vs-swing contact / motion sequence planner
[ ] synchronized four-leg full trajectory file
[ ] four-leg obstacle simulation using the precomputed trajectory
[ ] basic support polygon calculation
[ ] optional: ABAD adjustment
```

兩週的目標不是 polished final system。

而是：

> **讓第三週可以正式進入 hardware experiment、data collection 與 paper production。**

---

# 24. Paper Writing 時程

不要等實驗做完才開始寫。

## Week 1

先寫 rough notes：

```text
Introduction
Motivation
Related Work
Problem Statement
Planner Overview
```

## Week 2

開始寫 Method：

```text
Contact Representation
Rolling Feasibility
Swing Planner
Hybrid Decision Strategy
ABAD Stability
```

## Week 3 之後

逐步補：

```text
Experimental Setup
Results
Figures
Tables
Discussion
Limitations
```

Paper 和 code 應該一起成長。

---

# 25. 核心 Research Story

目前預設的 paper story：

> 輪式移動具有高能源效率，但面對具有離散高度差與左右不對稱的崎嶇地形時，可能無法通過或造成明顯的機身姿態變化。純 walking 雖然能提升地形通過能力，卻犧牲 rolling 所帶來的效率。因此，本研究提出一套針對已知 structured terrain 的 offline hybrid gait planner：在機器人執行前，對完整 traversal 顯式評估 rim contact、collision、rolling feasibility 與 support stability，產生完整 rolling–stepping contact sequence、body trajectory 與四腳 joint reference trajectory。存在可行且無碰撞的連續接觸路徑時維持 rolling；只有在 rolling 受到地形幾何、kinematic limit 或 stability constraint 限制時，才安排 swing repositioning。同時利用 ABAD 改變 lateral foothold 與 support geometry，在左右不對稱障礙地形上維持穩定，兼顧 walking 的通過能力與 rolling 的能源效率。

---

# 26. 一句話設計原則

> **給定已知地形，預先產生完整 hybrid locomotion trajectory：能連續滾動就滾，必要時才跨，並利用 ABAD 維持穩定的支撐幾何。**

英文：

> **Given a known terrain, precompute the complete hybrid locomotion trajectory: roll whenever continuous contact is feasible, step only when necessary, and use ABAD to maintain stable support geometry.**

---

# 27. 現在立刻要做的下一件事

下一個 implementation task：

> **建立 2D terrain-aware contact query。**

第一版 terrain：

```text
1. flat ground
2. one rectangular obstacle
```

Required output：

```text
rim
alpha
contact point
terrain gap
collision state
edge margin
```

這個 module 穩定之後，下一步立刻做：

> **Single-leg rolling feasibility experiment**

確認不同 obstacle height 下：

```text
哪些可以直接 rolling
哪些需要 hybrid
哪些一定需要 swing
```

---

# 28. 目前 Project Reference Files

```text
old_hybrid_gait_planning_note.md
hybrid_gen.cpp
hybrid_swing.cpp
Simple_fsm.cpp
wlw_open.cpp
leg_model.cpp

Adaptive_Hybrid_Locomotion_for_a_Leg-Wheel_Transformable_Robot_on_Uneven_Terrain.pdf
電子論文ver2_R12_Thesis_YaTing_Hsu-1.pdf
2026ICRA_Lee,Hsing-Chen_2199.pdf
ICRA_YenLi_Lai_Final.pdf
```

之後請 Codex 修改 gait 時，建議同時提供：

```text
這份研究計畫
+
相關 source code
+
必要的論文 / 舊筆記
```

不要只給 Codex 一個 `.cpp`，讓它自己猜整套研究目的。

---

# 29. Codex 開始工作前應理解的最重要事項

```text
1. 這不是單純的 stair-climbing project。

2. Rectangle obstacle 是研究與開發初期使用的 structured terrain，
   不是最終研究定位本身。

3. 核心問題是：
   Given a known terrain, what complete rolling–stepping contact sequence
   should be planned before execution?

4. 舊 Hybrid gait 的 stance / swing execution 架構應盡量 reuse。

5. 新增的主要層級是 terrain-aware contact / motion planning。

6. Swing trajectory 不是目前最重要的 contribution。

7. ABAD 的主要研究用途是 lateral support adjustment。

8. 第一版先做 known 2D / 2.5D terrain，
   不要擅自把問題擴充成完整 3D perception / MPC / WBC。

9. Planner 是 offline、trajectory-level；runtime 只追蹤預先產生的
   `theta / beta / gamma` reference，不做 terrain-aware replanning。

10. 如果 implementation 遇到困難，
   優先確保 contact representation 與 rolling feasibility 正確，
   不要先犧牲核心研究問題去做次要功能。

11. 所有新 module 應盡量獨立、可測試、可視覺化。
```
