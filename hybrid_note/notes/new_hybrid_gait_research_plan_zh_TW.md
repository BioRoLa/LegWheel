# 新版 Hybrid Gait 研究方向與兩週開發計畫

> **用途**：整理目前新版 Hybrid Gait 的研究定位、規劃邏輯、程式架構、短期進度與實驗方向。
>
> 這份筆記可以直接放在本地專案中，之後交給 Codex 作為長期專案背景，讓它在修改 gait planning code 時能理解研究目標，而不是只從單一 `.cpp` 檔案猜測邏輯。
>
> 更新日期：2026-08-30
>
> **2026-08-30 修訂摘要**：加入 §1.1（最終交付物＝長路 + 多障礙 + 單一 trajectory.csv）、
> §10.5（rolling 的三種形態、平地推進、落地後滾走）、§14.1（地形參數量化）、
> Day 15–16（長路 composer 與 CSV 交付），並據此追加 Day 10–11 Step 6 與 Day 12 的介面要求。
> 原因：評估章節（§14 地形有平地、§15 的 R_roll 分母是整段 traversal）一直假設
> 「障礙之間」存在，但逐日計畫從來沒有一天負責生成它。

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

## 1.1 最終交付物（2026-08-30 明確化）

最終要生成的**不是**「一隻腳跨過一個障礙」的片段，而是：

> **一段夠長的連續 traversal，路上跨過一個以上的障礙，並輸出成單一份可直接執行的 `trajectory.csv`。**

具體規格：

```text
路徑長度      >= 2.0 m（數個 stride，不是單一 traversal 事件）
障礙數量      >= 2，左右配置可不同
障礙間距      障礙之間有平地 run，長度足以讓機器人回到 wheel mode 推進
輸出          一份 trajectory.csv，涵蓋起點到終點的完整時間序列
              simulation 與 real robot 讀同一份
```

這條要求會往回改三件事，必須在 Day 14 架構凍結前處理：

```text
1. 平地推進必須是一個【正式的 motion segment】，不能是隱含的背景
   -> 見 §10.5

2. terrain 必須支援多個障礙
   -> 目前 legwheel/planners/hybrid/terrain_2d.py 有 len(obstacles) > 1 的 guard
   -> §11.1 的盤點：真正要改的只有 contact_detection_2d 與 terrain_query_2d 各兩處，
      其餘 46 處是繪圖邊界與測試

3. 每一段的【出口狀態必須能當下一段的入口狀態】
   -> 目前 obstacle traversal 的入口是「站在下層地面 + 前緣餘裕 c」，
      出口卻是寫死 20 mm 的 GROUND_ROLL，兩者格式不同
```

**為什麼這條要求現在才寫進來**：原計畫的評估章節其實已經假設它存在——§14 的建議地形是
sparse rectangular obstacles（障礙之間全是平地），§15 的招牌 metric `R_roll` 分母是
**整段 traversal**。但 §12 的逐日計畫裡沒有任何一天負責生成那段平地。評估端與開發端
對不起來，這一節把它補上。

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

再加上 **segment 級欄位**，讓 CSV 自我描述（見 §10.5 與 Day 10–11 Step 6）：

```text
segment_index                  這一幀屬於第幾段
segment_kind                   APPROACH | WHEEL_ROLL | ROLL_UP | WHEEL_TRANSITION
                               | ROLL_DOWN | SWING_UP | SWING_DOWN | SWING_OVER
                               | POST_TOUCHDOWN_ROLL
legN_roll_budget_remaining_m   該腿目前接觸 rim 在前進方向的剩餘弧（換算成距離）
```

`segment_kind` 是 `R_roll`（§15）能被計算的**前提**：沒有它就無法把「滾動支撐下前進的
距離」從總距離裡分離出來，那個 metric 就只能用估的。

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

# 10.5 Rolling 的三種形態、平地推進與落地後滾走

> **2026-08-30 新增。** 起因是 §1.1 那條「最終要跑一段長路、跨過一個以上障礙」的要求，
> 它暴露了原計畫把 rolling 只當成「跨越障礙的手段」的侷限：障礙之間怎麼走，從來沒有被規劃。

## 10.5.1 「滾走」其實是三種不同的東西

| 形態 | 定義 | 目前用在哪 | 距離上限 |
| --- | --- | --- | --- |
| **Rim rolling** | 固定 θ，接觸點沿某個 rim 的 α 單調前進（no-slip） | `ROLL_UP`（right rim 貼前緣）、`ROLL_DOWN`（left rim 貼後緣） | rim 弧長 |
| **Wheel-mode rolling** | θ = 17°，rim 弧圓心與 hip 重合，腿成為真圓 | `WHEEL_MODE_TOP_ROLL`，只在障礙頂面 | 無上限 |
| **落地後 rim rolling** | 觸地後沿當下接觸 rim 繼續前進 | 只有 `ROLL_DOWN` 收尾寫死的 20 mm | rim 剩餘弧 |

三者的差別是**幾何的，不是實作的**。實測（`LegModel`，`WHEEL_RADIUS_OUTER` = 145 mm）：

```text
θ       rim 弧圓心離 hip      滾 200 mm 時 hip 起伏
17°          0.0 mm                0.0 mm      <- 真圓，hip 高度恆定
30°          7.7 mm                6.3 mm
45°         17.8 mm               14.4 mm
60°         29.9 mm               24.2 mm
80°         50.6 mm               41.0 mm
100°        77.7 mm               62.9 mm
```

θ = 17° 就是 `theta0`：此時所有 rim 弧的圓心恰好落在 hip 上。θ 越大圓心越偏離，
滾動時 hip 被迫繞著那個偏心圓心公轉，起伏振幅就等於偏移量。

> **θ = 17° 的 wheel mode 是唯一能長距離推進的滾走形態。**
> rim rolling 天生是「有限弧長 + 強迫 body 起伏」的動作，只適合當跨越障礙面的手段。

這也說明 Day 6–7 那條五段鏈裡的 `RETRACT_TO_17_DEG` 不是實作上的權宜，是幾何要求。
而且它不是「停下來變形」：`run_retract_to_wheel_on_top_2d` 是 coupled retract/roll，
θ 一邊收、β 一邊繼續滾，接觸點持續前進（Step 8R 的 docstring：*"This is Step 7R
continued, not a second motion."*）。

## 10.5.2 但 rim rolling 對 walk 仍然是勝的——這是與 walk 的差異來源

10.5.1 的比較基準是完美輪子，所以 rim rolling 看起來永遠劣。但本研究要對比的是 **walk**。
同樣前進 200 mm，rolling stance（接觸點沿 rim 移動）vs walk stance（繞固定接觸點 pivot）：

```text
落地 θ     rolling hip 起伏     walk hip 起伏      誰勝
30°             6.3 mm             33.5 mm        rolling 大勝
45°            14.4 mm             28.0 mm        rolling 勝
60°            24.2 mm             24.1 mm        打平        <- 分水嶺
80°            41.0 mm             20.4 mm        walk 勝
100°           62.9 mm             17.6 mm        walk 大勝
```

**而 Day 8–9 所有 swing 的落地姿態正好是 θ = 60°**（`REGRESSION_THETA_RAD`，
`cartesian_swing_planner_2d.py:149`），剛好卡在打平點上。以目前的落地姿態，
「揮完再滾」跟 walk 在 body 平順度上是**沒有差異的**。

> **設計規則：要讓「揮完再滾」真的有別於 walk，swing 的落地 θ 必須往下壓（越接近 17° 越好）。**
> 落地 θ 是 swing planner 的目標值，不是副產品，所以它是可控的決策變數。

**但書**：這張 walk 對照表是一階近似——真實的 walk baseline 用同一條腿、foot rim 也有
145 mm 半徑、body planner 也會補償。它足以當**設計依據**（決定要不要壓低落地 θ），
但不足以當 paper 的論證，那要等 §15 的 Walk baseline 實際跑出來比。

## 10.5.3 滾走預算是一個可規劃的量

滾動距離 = `r_eff × 掃過的弧`，而 r_eff = 145 mm 是三個 rim 共用的：

```text
upper rim  140° → 354 mm
foot  rim   80° → 202 mm
```

**354 mm 是 stride 尺度，不是零頭。** 落地時已經用掉一半也還有 ~175 mm。所以
「揮完腳沿 rim 滾」在弧長上完全可行，問題只在於它目前沒有被規劃。

決定預算的是兩個落地量：

```text
落地 θ     決定滾起來平不平（§10.5.2）
落地 α     決定前進方向還剩多少弧 —— 也就是離 rim 接縫多遠
```

`remaining_rim_arc_rad_at_touchdown` 這個欄位**已經存在**於 `LeftRimRollDownResult2D`
（`right_up_left_down_traversal_2d.py:1007`），但 `validate_swing_touchdown_2d` 沒有
對應的量。補上它，「這個 swing 落完能滾多遠」就變成 `SwingConcession2D` 的一個
可比較欄位，而不是規劃完才發現的結果。

## 10.5.4 新增兩個 motion segment kind

> **2026-08-30 Step 6 完成：兩個 kind 都已進 `SegmentKind`**，即使 Day 10–11
> 不生成它們——之後再補會要改每一個消費端。
> 引擎那半（`run_retract_and_reset_branch_2d` 加 `stop_at = "forward_distance"`）
> 仍未做，那是 Day 15–16 的工作。

> **2026-08-30 討論找到的第三個缺口：`TOP_REPOSITION`（尚未實作）。**
>
> 腿越過頂面後，真實四腳機器人**不一定要用同一個接觸狀態接續下一段**。
> 當其他腿可以支撐 body 時，這條腿可以再次離地、在空中調整 `theta` / `beta`
> 與落點，再以更適合 `ROLL_DOWN` 或 `SWING_DOWN` 的姿態落地。
>
> ```text
> 頂面安全落地 -> 其他腿承重 -> 本腿再次離地
>              -> 空中調整 theta / beta / rim target
>              -> 落地到對下一段更有利的位置
> ```
>
> **為什麼不在 Day 10–11 做**：單腿 2D 模型回答不了「離地期間是誰支撐 body」。
> 現在生成那條空中軌跡，等於默默假設 body 被其他腿支撐——而單腿 planner
> 沒有支撐多邊形或 gait timing 可以驗證它。
>
> **現在的做法**：只在 schema 裡記一個 `TransitionRequirement2D`
> （`requires_external_support = True`、`resolved = False`），
> 不猜 `theta` / `beta` / `duration`，交給後續四腳 gait / timing 階段。
> 完整定義見 day10-11 規格 §5.6。
>
> **這也是唯一能真正重測 `#2` / `#3` 的途徑。**

```text
WHEEL_ROLL             平地 / 平頂上的 θ=17° 推進段。障礙之間的主要行進手段。
                       body requirement = 零 hip 讓步（hip 高度恆定），
                       代價記距離與時間，不記 hip 讓步。

POST_TOUCHDOWN_ROLL    swing 落地後沿當下接觸 rim 繼續前進的段。
                       受剩餘弧限制；結束時若要繼續前進，接 retract 到 17° 的 WHEEL_ROLL。
```

實作成本低，因為引擎已經存在：

```text
run_retract_and_reset_branch_2d    通用的 coupled retract/roll 引擎（θ 收、β 續滾）
                                   目前 stop_at ∈ {foot_rim_ready, theta_target,
                                                   left_rim_ready, trailing_corner}
    需要                           新增 stop_at = "forward_distance"
                                   並解除對 obstacle is not None 的依賴
```

`run_wheel_mode_transition_to_corner_2d` 就是這個引擎加上「停在後緣」的停止條件；
平地段只是換一個停止條件，不是新演算法。

## 10.5.5 一條完整的長路長什麼樣

```text
WHEEL_ROLL              平地推進，直到離下一個障礙前緣 c
  -> [障礙 1 的 ascent/descent 組合，由 Day 10–11 的 decision rule 選]
  -> POST_TOUCHDOWN_ROLL（若該組合以 swing 結束）
  -> WHEEL_ROLL         平地推進，直到離下一個障礙前緣 c
  -> [障礙 2 ...]
  -> WHEEL_ROLL         滾到終點
```

其中 `c` **不是連續代價軸**：Day 10–11 Step 2 實測 approach clearance 在 **c ≈ 60 mm 飽和**，
超過之後再多的空間買不到任何東西（`c = 60 … 160 mm` 每一列的 min hip lift 完全相同）。

> 所以 `WHEEL_ROLL` 段的任務是一個 **reachability 條件**（把腿送到離前緣 ≥ 60 mm 處），
> 不是一個最佳化問題。

這讓長路的組合大幅簡化：**平地段不需要決策，只需要滿足一個門檻**；所有的決策仍然
集中在 per-obstacle 的 decision rule 上，Day 10–11 的產出不需要重做。

## 10.5.6 對 paper 的影響

這一節產生兩條可寫的東西：

**(1) `R_roll` 從「一個宣稱」變成「一個有機制的結果」。** 不只報告「我們滾得比較多」，
而是說明多滾的距離分成兩種來源——平地 wheel mode（零 body 讓步）與落地後 rim rolling
（用 body 讓步換來的）——並給出後者的交換率。

**(2) 「與 walk 的差異」有了一個可量化的判準。** §10.5.2 的分水嶺（θ ≈ 60°）說明
hybrid 不是「walk 加上輪子」：落地姿態選對，stance 期的 body 起伏就比 walk 小；
選錯（例如沿用 Day 8–9 的 60°）就沒有差異。這正好是 reviewer 會問
「你這跟 walking 有什麼不同」的直接答案。

---

# 10.6 質心變化：故事線與它目前被證實到哪裡

> **2026-08-30 新增，來自與指導老師的討論。** 提出的故事是：
> **hybrid 這種滾走步態相對於一般 walking 的優勢，在於機身質心的變化較小。**
>
> **先寫清楚範圍**：質心是**整機**的量，由四隻腳的 stance 幾何加上 body trajectory
> 共同決定。目前的單腳規劃**不直接產生**這個量——它提供的是**機制**與**需求下界**。
> 真正的 CoM metric 要等 Day 12 的四腳 trajectory 才算得出來。這一節先把機制、
> 已有的證據、以及還缺的部分定義好，讓 Day 12 之後可以直接接上。

## 10.6.1 為什麼這個故事值得當主線

現況：§15 的 `energy / COT` 與 `pitch RMS / roll RMS` 是兩個**並列**的 metric，
彼此沒有關係。只能分別報告「比較省電」與「比較穩」，reviewer 問「為什麼省電」時沒有機制回答。

質心框架把它們接成一條因果鏈：

```text
接觸點沿 rim 移動（而非繞固定接觸點 pivot）
        ↓
CoM 垂直起伏 ↓
        ↓
對抗重力的垂直功 ↓  ──→  COT ↓
        ↘
          機身姿態變化 ↓
```

這比兩個獨立數字強，因為它讓 `COT` 這個數字**有機制解釋**，而不只是被量到。

文獻上有現成的語彙可以接：walking 的 **inverted pendulum** 模型（CoM 拱過支撐點）
vs **rolling / rimless wheel**（CoM 保持水平），以及義肢與生物力學的
**curved foot / roll-over shape**。本機器人在 θ = 17° 時 roll-over shape 退化成完美圓、
CoM 完全水平（§10.5.1 量到偏移為 0.0 mm），正好是那套語彙裡的極端情形。

> **待辦**：這塊文獻要實際查過再寫，本節只記概念框架，不代表已有引文。

## 10.6.2 故事的一半已經被量完了——只是名字不同

**`min_hip_lift_m` 字面上就是「質心至少要被抬高多少」。**

Day 10–11 Step 2 已完成的 88 cells，在 concession 框架裡叫「swing 向 body 索取的讓步」，
換到質心故事裡就是「用 swing 跨這個障礙，body 必須付出的垂直位移」：

```text
h <= 100 mm    hip  0 mm
h =  120 mm    hip 20 mm
h =  140 mm    hip 40 mm
h =  160 mm    hip 60 mm
h =  200 mm    hip 80 mm
```

同一個量、兩個名字。所以「不用 swing 可以省下多少質心位移」這半邊**不需要新實驗**，
資料已經在 `day10_11_step2_swing_onto_sweep.csv` 裡。

另一個機制在 §10.5.2：stance 期間接觸點沿 rim 移動，body 的垂直起伏比繞固定接觸點
pivot 小——但**只在落地 θ ≲ 60° 時成立**（θ = 60° 是打平點）。這個門檻不是弱點，
它把模糊的 claim 變成**有條件的設計結果**，而且條件是可控的（落地 θ 是 swing planner
的目標值，不是副產品）。

## 10.6.3 ~~還沒被證實的一半，以及它可能反過來~~ 【2026-08-30 Step 4 已量測】

roll 那半對應 Day 10–11 **Step 4**。規格 §5.3 預埋的風險是：

```text
swing 對 body 的要求：  hip 在某個時刻必須抬到某個高度   （一個下界）
roll  對 body 的要求：  hip 必須全程跟著一條被幾何決定的軌跡（一條軌跡）
```

rolling 的 hip 高度不是自由變數，是 θ 與接觸幾何的**輸出**。所以 rolling 不是
「零質心變化」，而是**另一種形狀的質心變化**。

> **2026-08-30 Step 4 完成：故事成立，但要加限定詞，而且加了之後更有力。**
> 詳見 `day10-11/day10_11_roll_swing_selection_zh_TW.md` 的 Step 4 完成紀錄。
>
> ```text
> rolling 的 hip 起伏 = h + overhead(theta_climb)
>     theta = 40 deg -> overhead 是【常數 14.2 mm】，在每一個高度都一樣
> swing  的 hip 起伏 = h + min_hip_lift
>     lift 在 h <= 100 mm 是 0，之後跳成 20 / 40 mm
> ```
>
> ```text
> h <= 100 mm   兩者在 1-3% 內打平（swing 略優，因為 roll 多付那 14.2 mm）
> h >= 120 mm   roll 便宜 8.9% (h=120) / 12.2% (h=140)
> 交叉點        h = 100-120 mm，三個獨立指標一致
>               （peak-to-peak、per-forward、§5.3 的跨型別規則）
> ```
>
> **所以 paper 不能寫「hybrid 滾走的質心變化比 walking 小」，要寫：**
>
> ```text
> 「h >= 120 mm 時滾走的 hip 起伏比 swing 少 9-12%；h <= 100 mm 兩者打平。
>   交叉點由【swing 的 hip lift 何時開始長】決定 ——
>   lift 在 h <= 100 是 0，之後跳成 20 / 40 mm，
>   而 rolling 的 overhead 是常數 14.2 mm。」
> ```
>
> **這比原本的 story 更有力，不是更弱**：它有機制、有交叉點、有數字，
> 而且交叉點的位置是可以預測的。
>
> 而 §5.3 猜的「取捨」版本也成立：兩種 body 要求的**形狀**不同——
> **rolling 是梯形（含一段平頂），swing 是三角形**。平頂那段是 rolling 在
> wheel mode 下免費賺前進距離的地方。
>
> **三個引用時必須帶上的限定詞：**
>
> ```text
> 1. 是 whole obstacle 不是 matched stages。
>    只比 ROLL_UP vs SWING_UP 的話 swing 在【每個高度】都便宜 1.1x-2.3x，
>    因為那把 rolling 免費賺距離的平頂拿掉了。walking 也得走過頂面，所以要用整顆。
>
> 2. 是對齊 approach clearance = 40 mm 的版本。
>    讓 swing 自選 clearance 會在零垂直代價下加長前進距離、稀釋自己的分母，
>    實測會在 h = 60 / 80 mm 翻盤。
>
> 3. rolling 的 traversal 不是姿態中性的（起於 theta_climb、終於 wheel mode，
>    淨差 -39.8 mm）。這在 peak-to-peak 裡抵銷，在絕對高度圖裡不會。
> ```

## 10.6.4 Metric 定義（Day 12 之後才算得出來）

「質心變化」要先定義是哪一種，三者的意義與可比性差很多：

| 定義 | 對應什麼 | 用途 |
| --- | --- | --- |
| 垂直位移 peak-to-peak / RMS | 最直觀，最接近口語的「質心變化」 | 主要報告值 |
| **垂直位移 ÷ 水平前進距離** | 無因次，**跨 gait / 跨地形可比** | **建議主推** |
| 垂直功（對抗重力）per unit distance | 因果鏈裡接到 COT 的那一環 | 連接能耗論證 |

注意現有的 `pitch RMS / roll RMS` 量的是**姿態**，不是質心**位置**；兩者不同，
目前 §15 沒有後者。見 §15 的追加條目。

## 10.6.5 對開發順序的影響

**不改變 Day 10–11 的 Step 順序。** Step 4 仍然在 Step 3 之後。

改變的是 Step 4 的**定位**：它原本被寫成「方法論完備性」（讓 roll 與 swing 的 concession
可以互相比較），現在它同時是**這條故事線的關鍵實驗**。因此 Step 4 的輸出要多帶一組
質心代理量——細節寫在 Day 10–11 規格的 Step 4，不在這裡重複。

> **2026-08-30：Step 4 完成，四個代理量兩側都算過。** §10.6.4 建議主推的
> 「垂直位移 ÷ 水平前進距離」就是實際採用的主要值，`hip_z_per_forward_distance`。
> 命名照 Step 4 的要求一律用 `hip_*` 而不是 `com_*`——整機 CoM 仍是 Day 12 之後的量。
>
> **Step 4 沒有重跑任何 traversal**：`day6_7_step11r_sweep_trajectories.csv` 已經記了
> 全部 70 個 cell 的 hip 軌跡。這讓原本估計要 264 s × 70 的一步變成讀檔即可。

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

**§1.1 的長路交付物不在這兩週內。** 它是 Day 15–16 的工作，兩週結束時只要求
「四腳通過**一個**障礙」。但它會往回影響兩週內的兩個決定，這兩個決定不能拖：

```text
Day 10–11 Step 6   schema 要先留下 WHEEL_ROLL / POST_TOUCHDOWN_ROLL 兩個 kind
                   理由：Day 12 的四腳 timing 會以 segment 為單位寫，
                         事後插入新 kind 等於重做 Day 12
Day 12             body trajectory 的介面要允許「連續多個 feature」，
                   不能寫死成「一個障礙的 traversal」
```

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

> **2026-08-30 實測更新**：Step 2b 已完成，結論是 **negative**——策略 #3
> (`SWING_UP + ROLL_DOWN`) 在目前 planner 下不成立（子問題 (b)(c) 成立、(a) 不成立，
> 且退化版本也不成立）。**候選從五個降為四個**（Step 3 之後再降為三個，見下）。以下關於 2×2 + 1 的論證保留，
> 因為它是「上升與下降必須獨立決策」的設計依據；但 Step 5 的實際候選數以
> `day10-11/day10_11_implementation_log_zh_TW.md` 為準。

**上升與下降是兩個獨立決策。** 策略空間不是 Day 6–7 §8 的三類，而是 2×2 + 1：

```text
ascent  in {ROLL_UP, SWING_UP}
descent in {ROLL_DOWN, SWING_DOWN}          -> 4 種（都落腳在 obstacle top）
外加 SWING_OVER（一次越過，不碰 top）        -> 合計 5 種
```

| # | ascent | descent | 舊名 | 什麼時候勝出 |
| --- | --- | --- | --- | --- |
| 1 | ROLL_UP | ROLL_DOWN | Strategy A | 低矮且 top 夠長 |
| 2 | ~~ROLL_UP~~ | ~~SWING_DOWN~~ | Strategy B | ~~h 超過 roll_down 天花板（160 mm）~~ **2026-08-30 Step 3 推翻，見下** |
| 3 | ~~SWING_UP~~ | ~~ROLL_DOWN~~ | **原三分類沒有** | ~~top 太短，付不起 `L_transition`~~ **2026-08-30 Step 2b 推翻，見下** |
| 4 | SWING_UP | SWING_DOWN | Strategy C 的一種 | 高，且 top 夠長可落腳 |
| 5 | — SWING_OVER — | Strategy C 的另一種 | top 很短 |

兩個修正：舊的 Strategy C 把 # 4（落 top）與 # 5（不落 top）混成一類，它們的 top-length 需求
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
ROLL_UP  + ROLL_DOWN     L_transition                 0.20–0.27 m（已量測，【下界】）
ROLL_UP  + SWING_DOWN    roll-up 出口 -> 起跳距離      【不成立，見下 Step 3】
SWING_UP + ROLL_DOWN     落點 -> 後緣 LEFT_RIM_READY   【不成立，見下 Step 2b】
SWING_UP + SWING_DOWN    落點 -> 起跳距離              takeoff <= 0.24 m (h<=100)
                                                      … <= 0.12 m (h=200)【上限】
SWING_OVER               0（不碰 top），但需 stride >= L_top
```

> **越靠 swing 的組合，需要的 top length 越短。**
>
> **2026-08-30 Step 5 的最終判決：照量到的數字【不成立】，而它的真假由一個
> 沒有被最佳化的參數決定。**
>
> ```text
> #1 ROLL  + ROLL    需要 L_top >= 205 - 245 mm
> #4 SWING + SWING   需要 L_top >= 240 mm         <- 比 #1 還長
> #5 SWING_OVER      需要 L_top <= 170 - 280 mm   <- 唯一反向的，無條件成立
> ```
>
> `#4` 只在 6 個高度中的 1 個比 `#1` 短。但它的下界是
> `landing_distance + 最短 takeoff = 160 + 80`，而 `landing_distance = 160 mm`
> 只是 Step 2 格點的固定值 —— close-out 掃過這一軸，**100 mm 在 h = 100/150/200
> 都可行且 lift 完全不變**，改用它就變成 `>= 180 mm`，於是每個高度都比 `#1` 短。
> **兩個版本都寫在 Day 10–11 規格的 Step 5 完成紀錄裡。**
>
> `L_top` 仍然升格成主軸 —— 只是理由是 §2.6 的**方向相反**（Step 3 A 段）
> 與 **`theta` 階梯**（Step 5），不是這條單調性。
>
> **2026-08-30 Step 3 A 段：這句話成立，而且比原本寫的更強。** 不只是「較短」——
> 兩者是**同一個座標的相反方向**：rolling 需要 `L_top` **大於** 196–249 mm，
> swing off 需要 takeoff **小於** 120–240 mm（上限隨高度收縮）。
> 所以 decision rule 的第一個判準是頂面長度的**方向**，不是代價比較：
> 短頂面只有 swing 下得去；長頂面兩者都行，才輪到比 body 讓步。

若被數據證實，`L_top` 從約束升格成與 height 平起平坐的策略選擇主軸——而這是 rolling 的二值
可行性 map 完全看不到的維度。

> **2026-08-30 Step 2b 完成：#3 在目前的 planner 下被推翻。**
> 詳見 `day10-11/day10_11_roll_swing_selection_zh_TW.md` 的 Step 2b 一節。
>
> ```text
> (b) 成立    theta = 17 deg 的 left-rim 窗口寬 132 deg、連續、與高度無關
>             rolling 自己抵達的 beta 就落在窗口內
> (c) 成立    rim budget 與 alpha seam margin 是 1:1 互換；選擇規則無自由參數
> (a) 不成立  35/35 落地姿態通過 LEFT_RIM_READY，但沒有一個 swing 到得了
>             四個候選解釋全被數據排除：不是 top length（0.08–0.35 m 全失敗）、
>             不是 hip travel（0.28 m < Step 2 成功的 0.38 m）、不是落地姿態、
>             也不是 approach 姿態（theta 40–85 × c 60/100/160 全失敗）
> 退化版本    落地 theta 放寬到 25/35/45/60 deg 也一樣，15/15 失敗
> ```
>
> **真正的原因**：站姿接觸永遠在 `foot_rim, alpha = 0`，要落到 left rim 就得穿過
> `alpha = -40°` 的 rim 分段接縫——實測那是一個約 **29°** 的關節不連續，
> 而且加密取樣（61 → 481）不會讓它變小，只會收斂上去。
> 所以 `SWING_UP + ROLL_DOWN` **仍然需要在頂面做一次 rim handover**（foot→left 取代
> right→left），**它並沒有躲掉 `L_transition`，只是把它換了個名字**。
> 上面那句「越靠 swing 的組合 top length 越短」因此只在第 2、4、5 列之間成立。
>
> 這是 negative result，但歸屬明確：擋住它的是 **rim 分段接縫這個模型性質**，
> 不是機器人做不到。rim 之間的接觸連續化之後要重測。Step 5 的候選先降為四個（Step 3 之後再降為三個）。
>
> 另外兩個會直接影響 Step 5 的量測：
>
> ```text
> 1. 策略 #3 對 body 的需求是【端點釘死 + 中段拱高約 40 mm】
>    （兩端 hip 一起 +40 mm 時 IK 從 21/61 變成 61/61 全收斂）。
>    HipTrajectory2D 是直線，表達不出來 -> 這是 Day 12 的輸入。
> 2. LEFT_RIM_READY 是必要條件，但對【空中來的】抵達不是充分條件：
>    35/35 通過 precondition，h >= 160 mm 的 14 個 Step 9R 仍然下不去。
>    下降側自己的天花板落在 140–160 mm 之間，且與 rim budget 無關。
> ```

> **2026-08-30 Step 3 完成：#2 也被推翻，但它是【兩道不同的牆】。**
> 詳見 `day10-11/day10_11_roll_swing_selection_zh_TW.md` 的 Step 3 一節。
> 六段掃描共 204 cells、10,305 次 `generate_swing_2d`。
>
> ```text
> A  99 cells, 74 可行   下降側有 takeoff distance 的【上限】：0.24 m (h<=100) -> 0.12 m (h=200)
> B   6 cells,  0 可行   Day 8-9 的 160 mm claim 重現不了（兩種取樣都是 -1.63 / -0.95 mm）
> C  15 cells, 10 可行   最佳 theta 隨高度上升（h=100 -> 60 deg、h=150 -> 85 deg）
> D  45 cells,  0 可行   arrival = ROLL_UP 直接交接
> E  30 cells,  0 可行   退化路徑：先 retract 到 theta = 17/25/30 再起飛
> F   9 cells,  6 可行   theta 32/35/38 —— 夾出下降側的 theta 下限
> ```
>
> **D 與 E 不是同一件事，不要合併成一句「#2 不可行」。**
>
> ```text
> D  binding = fit    JOINT_DISCONTINUITY 26 + IK_NOT_CONVERGED 19
>    rolling 交過來的起飛姿態一律在 right rim, alpha = +82 … +107.7 deg，
>    而 foot_rim = (-40, +40) —— 每一格都要跨過 +40 deg 的接縫。
>    這是 Step 2b 的【鏡像】（那邊是落地跨 -40 deg），同樣是 rim 分段模型性質。
>    => 不可修，等接觸連續化。
>
> E  binding = reach  IK_NOT_CONVERGED 30，而且【每一個】takeoff 距離都失敗
>    F 段夾出原因：下降側的 theta 下限是 35 deg，且與 takeoff 距離無關
>    （32 失敗、35 可行 hold 1.000、38 可行 hold 0.875，三個距離結果一致）。
>    E 段跑的 17/25/30 deg 全部在下限以下。
>    Day 6-7 Step 6.5 把 retract 的 theta_target 寫死成 17 deg，
>    因為它的目的是【進 wheel mode】給 ROLL_DOWN 用；SWING_DOWN 不需要 wheel mode。
>    => 【可修】，而且很便宜：L_transition = retract_forward (20.5-64.4 mm)
>       + wheel_mode_forward (131.8-228.4 mm)，停在 35 deg 就不用付後半，
>       頂面需求掉 3-10 倍。修好後 #2 要重測。
> ```
>
> **B 段的處理方式是 refutation with attribution，不是「前人量錯」。**
> 兩種取樣（121/31 與 241/51）都跑，加密後穿透從 -1.63 收斂到 -0.95 mm 卻仍未跨 0，
> **所以不是取樣假影**。A 段給出歸屬：`h = 160 mm` 的 takeoff 上限是 **0.14 m**，
> 而 docstring 是在 0.16 m 量的——**高度沒錯，距離錯了**。
> 同一批數據也推翻了 Day 8–9「off 200 mm 不可行」：takeoff <= 0.12 m 時可行。
>
> **Step 5 的候選因此降為三個：#1 / #4 / #5。**
>
> **2026-08-30 用語修正（day10-11 規格 §5.5）**：上面的「推翻」一律讀作
> `DIRECT_HANDOFF_INFEASIBLE` —— **目前的單腿連續接觸 primitive 無法直接交接**。
> 它們**不是** `PHYSICALLY_INFEASIBLE`。若允許四腳支撐下的 `TOP_REPOSITION`
> （規格 §5.6），`#2` / `#3` 仍可能成立，**只是目前尚未驗證**。
> 引用這些 negative result 時務必帶上這個限定詞。

### 這兩天的主產出是 envelope 與 decision rule，不是完整 sequence generator

理由：ROLL 那側已有完整 feasibility map，SWING 那側只有零星 case。
**decision rule 缺的是後者那張表，不是串接程式碼。** 而且 envelope 本身就是 paper 的 Figure D，
sequence composer 即使只跑通兩條 demo，Day 10–11 的研究產出仍然完整。

### 完成標準（修訂）

```text
[必要]
  Step 0   scene / approach 幣別對齊：rolling 與 swing 用同一組障礙幾何與同一種 approach 度量
  Step 2   ascent  swing sweep   (height × approach clearance)  -> 最小 hip lift map
  Step 2b  SWING_UP 能否落在 LEFT_RIM_READY -> 決定策略 #3 是否存在  【2026-08-30 完成：不成立】
  Step 3   descent swing sweep   (height × takeoff distance)     -> 最小 hip hold map
           （起點分 arrival = ROLL_UP / SWING_UP 兩種）  【2026-08-30 完成】
           三個完成標準全達成，其中兩個以「推翻並記錄」的形式達成 -> 策略 #2 也不成立
  Step 4   rolling map 換成同一種幣別（RollConcession）  【2026-08-30 完成】
           五個完成標準全達成；§5.3 缺的跨型別排序規則已實作
  Step 5   envelope 疊圖 -> decision rule + Figure D + top-length 切片圖  【2026-08-30 完成】
           （候選為三個：#1 / #4 / #5；Step 2b 推翻 #3、Step 3 推翻 #2）
           五條完成標準四條達成；第 2 條「每個 region 生一條軌跡」是 Step 7 的工作。
           核心機制：L_top 不只選策略，還在 #1 內部選 theta_climb -> rolling 的代價是階梯。
           新量測：#5 SWING_OVER（先前沒有任何一步量過），h <= 80 mm 才可行。
           **能力圖上有一個洞**：h >= 100 mm 且 L_top < 205-235 mm，172/609 格全部過不去。
  Step 6   segment 級 motion sequence schema（含取樣參數與 body requirement）  【2026-08-30 完成】
           **含 WHEEL_ROLL 與 POST_TOUCHDOWN_ROLL 兩個 kind**（見 §10.5.4）
           三條完成標準全達成。資料另外逼出四件規格沒寫的事，見 Day 10–11 規格 Step 6

[目標]
  Step 7   sequence composer  【2026-08-30 完成，但點名的三對有兩對已不存在】
           `ROLL+SWING` 被 Step 3 推翻、`SWING+ROLL` 被 Step 2b 推翻，改以第一級輸出記錄理由。
           實際生出的是還活著的三對：`ROLL+ROLL` / `SWING+SWING` / `SWING_OVER`。
           交接量化：關節跳躍 <= 1.75 deg、接觸點最大 215.9 mm（換 rim 非不連續）、
           rim 幾何 gap 最大 1.2000 mm（不累積）。
           **已知風險**：`#1` 最短頂面那條有 88 幀貼著 ±180 度接縫 1.17 deg。
  Step 8   五個 terrain case 各落在一個 region，含負向驗證  【2026-08-30 完成】
           五個 case 的正向與負向都成立、19 個拒絕全部具名。
           **「至少一個 case 的最佳解是混合的」無法達成**——兩個混合對都被推翻。
           2×2 在對角線之外被推翻；代價已量化（h = 160 mm 上貴 2.7 倍）。
  Step 9   body requirement timeline，作為 Day 12 的輸入
```

Sequence 的每一段必須記下**取樣參數**（`arc_samples` / `sample_count` / `max_joint_step_rad`）。
Day 8–9 Step 9 已證明連續性限制與取樣密度耦合，不記就不可重現。

**追加（2026-08-30，來自 §10.5）**：swing 的落地狀態要多帶兩個量，它們決定
「落完能不能繼續滾」，也就是與 walk 的差異來源。

```text
[必要，很輕]
  在 validate_swing_touchdown_2d 加 remaining_rim_arc_m
      = 落地 rim 在【前進方向】的剩餘弧，換算成距離
      重用既有的 legacy_rim_edge_margin_rad，不要新寫幾何
  在 SwingConcession2D 加 touchdown_theta_rad 與 post_touchdown_roll_budget_m

[目標，Step 2 / Step 3 多掃一軸]
  落地 θ ∈ {17° … 60°}
  問題：壓低落地 θ 會讓 reach 變差、要付 hip 讓步，
        那麼「多少 body 讓步換多少 rolling distance」？
  這條交換曲線就是 R_roll 的機制解釋，也是 concession 框架能答、
  二值可行性答不出來的問題
```

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

> **2026-08-30：Day 10–11 交過來三件事，其中兩件是「已知但未解」。**
>
> ```text
> 已解    #1 / #4 / #5 的 body-requirement timeline（Step 9 產出）
>         硬約束 = TRACK / PINNED，偏好 = LOWER_BOUND
>
> 未解    #2 / #3 的 TransitionRequirement2D
>         kind = TOP_REPOSITION、requires_external_support = True、resolved = False
>         Day 10-11 【刻意沒有】猜它的 theta / beta / duration
>
> 未解    rolling 段沒有 duration（Day 6-7 是準靜態資料）
>         Step 9 會擇一：以 x 為自變數，或明說一個 timing 假設
> ```
>
> **`TOP_REPOSITION` 只有到了這裡才驗證得了**——因為它需要「其他腿能否承重」
> 這個判斷，而那正是四腳 timing 的工作。定義見 day10-11 規格 §5.6。
> 一旦它能生成，`#2` / `#3` 就可以**真正重測**，
> 而不是停在 `DIRECT_HANDOFF_INFEASIBLE`。

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

### 介面要求（2026-08-30 追加，為 §1.1 的長路鋪路）

Day 12 只需要跑通**一個**障礙，但介面不能寫死成「一個障礙的 traversal」：

```text
輸入        一串 MotionSegment2D，不是「一個 obstacle 的 traversal 物件」
時間軸      以 segment 邊界對齊，segment 數量不限
輸出 CSV    帶 segment_index / segment_kind 欄位（§5）
狀態交接    每段的出口狀態必須能當下一段的入口狀態，並寫一個測試證明
            （目前 obstacle traversal 的入口是「站在地面 + 前緣餘裕 c」，
              出口是 20 mm 的 GROUND_ROLL，格式不同——這裡要統一）
```

**這是整份計畫裡最便宜、但拖不得的一件事。** Day 12 之後四腳 timing 一旦以
「一個障礙」為單位寫死，Day 15–16 要插入平地段就等於重做 Day 12。

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

之後只做「填段」與實驗，不重寫 planner 核心。

---

## Day 15–16 — 長路 Multi-Obstacle Traversal 與 CSV 交付

> **2026-08-30 新增。** 這是 §1.1 交付物的實作日。放在架構凍結之後，
> 是因為它**不需要新演算法**——只需要新增兩個 segment kind、解除單障礙 guard、
> 把段串起來。若 Day 12 的介面要求有做到，這兩天就是組裝。

> **2026-08-30 追加一項必要工作：coverage test，而且它決定能講什麼話。**
>
> Day 10–11 的 `decide_2d(h, L_top)` 是**目前已掃範圍內**的單腿 2D 決策函式，
> **不是對任意地形的成功保證**。它已知的 capability hole 是
> **「較高障礙 + 較短頂面」**，而那個洞的成因不是「太高」或「太小」，
> 是三個可用策略各自的幾何限制（見 day10-11 規格 Step 8 case D）。
>
> ```text
> 在定義實使用環境的 obstacle distribution 之前，
> 【不要】用現有 map 的格數宣稱「多數 obstacle 都可以通過」。
> ```
>
> 所以這兩天要多做一件事：
>
> ```text
> 4. 多障礙長路 coverage test
>    先定義 (h, L_top, gap) 的機率範圍與分布，再產生大量地形，統計：
>        - 在【明確定義的】障礙分布中的實際成功率
>        - 各種策略被選中的比例
>        - 失敗是幾何 reach / joint limit / 接觸交接，還是支撐假設造成
>        - 哪些 capability hole 是機器人的物理極限，
>          哪些只是 planner 少缺 primitive（見 day10-11 規格 §5.5 的用語）
> ```

### 任務

```text
1. 解除單障礙限制
   legwheel/planners/hybrid/terrain_2d.py 的 len(obstacles) > 1 guard
   真正要改的是 contact_detection_2d 與 terrain_query_2d 各兩處（§11.1 已盤點）
   其餘 46 處是繪圖邊界與測試，機械性修改

2. 實作 WHEEL_ROLL 段（§10.5.4）
   run_retract_and_reset_branch_2d 新增 stop_at = "forward_distance"
   解除對 obstacle is not None 的依賴
   驗收：在純平地上滾 1.0 m，hip 高度變化 < 1 mm（θ=17° 是真圓，應該恆定）

3. 實作 POST_TOUCHDOWN_ROLL 段（§10.5.3）
   起點 = swing 的落地 ContactState
   長度 = min(需求距離, remaining_rim_arc_m)
   弧用完就接 retract 到 17° 的 WHEEL_ROLL

4. Long-horizon composer
   輸入：TerrainProfile（多障礙）+ 起點 + 終點
   對每個 obstacle 套用 Day 10–11 的 decision rule
   段與段之間插入 WHEEL_ROLL，長度由「離下一個前緣 >= 60 mm」的門檻決定
   輸出：一條 MotionSequence2D，涵蓋起點到終點

5. 導出 trajectory.csv（§5 的完整 schema，含 segment_index / segment_kind）
```

### 測試地形

```text
Case L1   兩個等高障礙，間距充裕        驗證段能串、CSV 能跑完
Case L2   兩個不等高障礙，決策不同      驗證 decision rule 在同一條路上會切換策略
Case L3   兩個障礙間距【不足】60 mm     負向：驗證平地段的門檻條件真的會擋下來，
                                        並記錄 planner 如何處理（改策略或報失敗）
Case L4   左右不對稱配置                為 Day 13–14 的 ABAD 與 §16 的 ablation 供料
```

`Case L3` 是這一步最重要的一個：它是唯一能證明「平地段是一個真的約束、
不是填充物」的證據。§11.3 也指出這正是 reviewer 咬得最兇的一點——真實崎嶇地的
feature 間距常小於腿的可及範圍。

### 完成標準

```text
- 產生至少一條 >= 2.0 m、跨過 >= 2 個障礙的完整 trajectory.csv
- CSV 每一幀都帶 segment_index / segment_kind，且 segment 邊界的狀態連續量被量化
- R_roll（§15）可以直接從這份 CSV 算出來，並拆成
      平地 wheel-mode 距離 / 落地後 rim rolling 距離 / 跨障礙 rim rolling 距離
- 至少一條路上，兩個障礙被選了【不同的】策略（否則 decision rule 沒有被驗證）
- Case L3 的負向行為有明確記錄
- simulation 讀這份 CSV 能跑完全程
```

### 明確不做

```text
online replanning            不在範圍
斜面 / 非軸對齊障礙          §11.4 已列 future work
障礙重疊或間距為零           先要求障礙彼此分離
3D / gamma != 0 的長路       Day 13–14 的 ABAD 先在單障礙上驗證
```

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

## 14.1 地形參數（2026-08-30 明確化，配合 §1.1）

`sparse` 這個字之前沒有被量化，導致 §12 沒有任何一天負責「障礙之間」。補上：

```text
總長度        >= 2.0 m
障礙數量      >= 2（主實驗用 2–3 個）
障礙高度      取自 Day 10–11 decision map 的不同 region，
              讓同一條路上至少出現兩種不同策略
障礙間距      >= 0.30 m 的平地 run（主 case）
              另備一個 < 0.06 m 的負向 case（見 Day 15–16 的 Case L3）
左右配置      主實驗用不對稱，供 §16 的 ABAD ablation
```

**間距是一個實驗變數，不是佈景。** §10.5.5：平地段的任務是把腿送到離前緣 >= 60 mm 處，
所以間距不足時 decision rule 會被迫改變——那正是「per-feature 決策會被鄰居影響」的證據，
也是 §11.3 列為 reviewer 咬人機率最高的那一項。

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

**追加：CoM 垂直運動（2026-08-30，見 §10.6）**

現有清單量的 `pitch / roll RMS` 是**姿態**，沒有任何一項量質心的**位置**。
而 §10.6 的故事線（hybrid 相對 walking 的優勢在於質心變化較小）需要後者：

```text
com_z_peak_to_peak_m          質心垂直位移的峰對峰值
com_z_rms_m                   質心垂直位移 RMS
com_z_per_distance            垂直位移 / 水平前進距離   <- 無因次，主推這個
com_vertical_work_per_dist    對抗重力的垂直功 / 前進距離
```

`com_z_per_distance` 建議當主要報告值：它無因次，所以 Wheel / Walk / Hybrid
三者可以直接比，也不會因為地形長度不同而失真。

`com_vertical_work_per_dist` 是把這組 metric 接到 `COT` 的那一環——有了它，
「Hybrid 比較省電」才有機制解釋，而不是只有一個數字。

> **前提**：這四項都是**整機**量，要 Day 12 的四腳 trajectory 才算得出來。
> 單腳階段（Day 6–11）只提供機制與需求下界，見 §10.6.1 與 §10.6.2。

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

**計算前提（2026-08-30 補充）**：分母是**整段 traversal**，所以這個 metric 只有在
§1.1 的長路 CSV 上才算得出來——單一障礙的 traversal 沒有分母。而分子必須由
`segment_kind`（§5）分離，不能用估的。

建議把 `R_roll` 拆成三項回報，因為三者的性質完全不同：

```text
R_roll_wheel      平地 / 平頂的 θ=17° wheel-mode 距離
                  零 body 讓步，是效率的主要來源

R_roll_traverse   跨越障礙面的 rim rolling（ROLL_UP / ROLL_DOWN）
                  是「不用 swing 就能過」的證據

R_roll_stance     swing 落地後的 rim rolling（POST_TOUCHDOWN_ROLL）
                  是與 walk 最直接的差異：walk 的 stance 是繞固定點 pivot，
                  這裡的 stance 接觸點會沿 rim 移動（§10.5.2）
```

第三項要搭配落地 θ 一起報告：§10.5.2 的分水嶺是 θ ≈ 60°，落地姿態在那之上時，
stance 期的 body 起伏並不會比 walk 小，`R_roll_stance` 就只是數字好看而沒有實質差異。

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
- 選擇判準是「向 body trajectory 索取多少讓步」，不是二值可行性
  （Day 10–11 §2.2；上升側兩種都可行，二值判準沒有鑑別力）。
- **該讓步的物理意義就是質心垂直位移需求**，因此 motion selection 直接連到
  §10.6 的質心故事線與 §15 的 CoM metric。

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
6. Long-horizon multi-obstacle traversal + trajectory.csv   <- 2026-08-30 新增
7. ABAD stability
8. Hardware robustness
9. Optimization refinement
```

如果需要砍 scope：

> **從最下面開始砍，不要先砍最核心的 roll-vs-swing planning。**

**第 6 項為什麼排在 ABAD 之前**：它是 §15 主要 metric `R_roll` 的**分母**——沒有長路
就沒有分母，那個 metric 算不出來。ABAD 影響的是 roll RMS 與 stability margin，
是 §16 的 ablation，砍掉仍有主結果；長路砍掉則主結果本身缺一塊。

**但它可以退化交付**：若時間真的不足，最低版本是「兩個障礙、一條 2 m 的路、
不做 Case L3 負向」，仍然足以算出 `R_roll`。

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

**§1.1 的長路交付物不在這張清單上**（它是 Day 15–16）。但兩週內必須留下它的接口，
否則第三週要重做 Day 12：

```text
[ ] Day 10–11 Step 6 的 schema 含 WHEEL_ROLL / POST_TOUCHDOWN_ROLL 兩個 kind
[ ] Day 12 的四腳 timing 以 segment 序列為輸入，不是以「一個障礙」為輸入
[ ] trajectory.csv 帶 segment_index / segment_kind 欄位
```

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

> **2026-08-30 更新。** 原本的 story（下方保留）只講「保留 rolling 的效率、兼顧 walking 的
> 通過能力」，兩個好處是**並列**的，沒有機制把它們連起來。與指導老師討論後補上的那條線
> 是：**hybrid 滾走步態相對於一般 walking 的優勢，在於機身質心的變化較小**，而質心變化
> 同時解釋了能耗與姿態穩定兩者。見 §10.6。

## 25.1 修訂版 story（含質心機制）

> 輪式移動具有高能源效率，但面對具有離散高度差與左右不對稱的崎嶇地形時，可能無法通過或造成明顯的機身姿態變化。純 walking 雖然能提升地形通過能力，卻犧牲 rolling 所帶來的效率——**而效率損失的一個主要來源是質心的垂直運動：walking 的 stance 是繞著固定接觸點的 inverted-pendulum 運動，機身必須被拱起再放下；rolling 的接觸點沿 rim 連續移動，機身可以維持接近水平的軌跡。**
>
> 因此，本研究提出一套針對已知 structured terrain 的 offline hybrid gait planner：在機器人執行前，對完整 traversal 顯式評估 rim contact、collision、rolling feasibility 與 support stability，產生完整 rolling–stepping contact sequence、body trajectory 與四腳 joint reference trajectory。**motion selection 的判準不是「哪一種 primitive 可行」，而是「哪一種對 body trajectory 的要求較低」——而該要求的物理意義正是質心必須付出的垂直位移。** 只有在 rolling 受到地形幾何、kinematic limit 或 stability constraint 限制時，才安排 swing repositioning。同時利用 ABAD 改變 lateral foothold 與 support geometry，在左右不對稱障礙地形上維持穩定。

## 25.2 這條 story 目前被證實到哪裡

**寫 paper 前必須照這個分界寫，不可以把未證實的部分寫成結果。**

```text
[已有資料]  swing 側的質心位移需求
            Day 10-11 Step 2 的 88 cells，min_hip_lift 就是這個量
            h <= 100 mm 免費；120/140/160/200 mm 分別要 20/40/60/80 mm

[已有資料]  stance 期的機制與門檻
            接觸點沿 rim 移動 vs 繞固定點 pivot，分水嶺在落地 θ ≈ 60°（§10.5.2）
            但這是單腳一階近似，不是整機量測

[尚未量測]  roll 側的質心軌跡需求
            Day 10-11 Step 4。rolling 的 hip_z 是輸出而非自由變數，
            它的要求是【一條軌跡】而不是一個下界（規格 §5.3）
            >> 若 Step 4 量出 rolling 的起伏更大，這條 story 要改寫成
               「兩種不同形式的 body 要求之間的取捨」

[尚未量測]  整機 CoM
            需要 Day 12 的四腳 trajectory 與 §15 的 CoM metric

[尚未量測]  與 Walk baseline 的實際比較
            §15 的 Walk baseline 尚未實作
```

## 25.3 原始 story（保留供追溯）

目前預設的 paper story：

> 輪式移動具有高能源效率，但面對具有離散高度差與左右不對稱的崎嶇地形時，可能無法通過或造成明顯的機身姿態變化。純 walking 雖然能提升地形通過能力，卻犧牲 rolling 所帶來的效率。因此，本研究提出一套針對已知 structured terrain 的 offline hybrid gait planner：在機器人執行前，對完整 traversal 顯式評估 rim contact、collision、rolling feasibility 與 support stability，產生完整 rolling–stepping contact sequence、body trajectory 與四腳 joint reference trajectory。存在可行且無碰撞的連續接觸路徑時維持 rolling；只有在 rolling 受到地形幾何、kinematic limit 或 stability constraint 限制時，才安排 swing repositioning。同時利用 ABAD 改變 lateral foothold 與 support geometry，在左右不對稱障礙地形上維持穩定，兼顧 walking 的通過能力與 rolling 的能源效率。

---

# 26. 一句話設計原則

> **給定已知地形，預先產生完整 hybrid locomotion trajectory：能連續滾動就滾，必要時才跨，並利用 ABAD 維持穩定的支撐幾何。**

英文：

> **Given a known terrain, precompute the complete hybrid locomotion trajectory: roll whenever continuous contact is feasible, step only when necessary, and use ABAD to maintain stable support geometry.**

---

# 27. 現在立刻要做的下一件事

> **2026-08-30 更新。** 本節原本寫的是「建立 2D terrain-aware contact query」，
> 那是 Day 3–5 的任務，早已完成。原文保留在本節末供追溯。

目前進度見 `day10-11/day10_11_implementation_log_zh_TW.md`：

```text
Day 1–2   完成      trajectory CSV 契約
Day 3–5   完成      2D terrain-aware contact / collision query
Day 6–7   完成      rolling feasibility map、L_transition 分析
Day 8–9   完成      swing planner、兩道天花板
Day 10–11 進行中    Step 0–8 全部完成（約 85-90%）。
                    下一步是【Step 9 之前的兩個小修正】（結論用語 + TransitionRequirement2D），
                    然後才是 Step 9。詳見 day10-11 規格 §5.5 / §5.6 與實作紀錄 §3。
```

下一個 implementation task，依序：

```text
1. ~~Step 2b~~  **2026-08-30 完成：#3 不成立**（見 §「top-length 預算」的更新框）。
2. ~~Step 3~~   **2026-08-30 完成：#2 也不成立，但是【兩道不同的牆】**（同上更新框）。
             D 是 +40 deg 的 rim 接縫（不可修）；E 是下降側自己的 theta >= 35 deg 下限（可修）。
             **Step 5 的候選降為三個：#1 / #4 / #5。**
             另外量到：頂面長度是【雙向】約束 —— rolling 要下界、swing off 要上限。

   Step 5 之前必須先處理【四件事】：
             (a) theta 要取最佳（上升與下降【兩邊】都要；Step 2 close-out C + Step 3 C 段）
             (b) clearance 門檻是 c_threshold(h) 不是常數
             (c) 下降側自己的高度天花板（140–160 mm，與 rim budget 無關）
             (d) 頂面長度的雙向約束要當成 decision rule 的第一個判準

3. 落地滾走的兩個輕量欄位（§10.5.3、Day 10–11 完成標準的「追加」段）
             validate_swing_touchdown_2d 加 remaining_rim_arc_m
             SwingConcession2D 加 touchdown_theta_rad / post_touchdown_roll_budget_m
             做完就能在既有的 88 個 cell 上直接看到每個落點還剩多少滾走預算

4. ~~Step 4~~   **2026-08-30 完成：質心 story 成立但要加限定詞**（見 §10.6.3 的更新框）。
             rolling 的 hip 起伏 = h + 14.2 mm，不是 0 —— §2.2 的直覺被推翻。
             h >= 120 mm roll 優 9-12%；h <= 100 mm 打平（swing 略優 1-3%）。
             §5.3 缺的跨型別排序規則已實作，**roll 與 swing 現在可以比了**。
             另外量到：theta_climb 是取捨（低 theta 省 34-47 mm 起伏但要多 64.4 mm 頂面）。

5. ~~Step 5~~   **2026-08-30 完成：Figure D 有了**（見 §「top-length 預算」的更新框）。
             `decide_2d(h, L_top)` 是純函式。核心機制：**`L_top` 不只選策略，
             還在 `#1` 內部選 `theta_climb`** —— `required_top_length(θ)` 單調下降
             （269.4→205.0 mm）而 hip 起伏 `overhead(θ)` 單調上升（14.2→49.4 mm），
             所以最佳 θ 永遠在約束邊界，rolling 的代價是 `L_top` 的**階梯**。
             同時補上了 `#5 SWING_OVER` 的第一次量測（先前沒有任何一步量過它）。

6. ~~Step 6~~   **2026-08-30 完成：schema 含 `WHEEL_ROLL` / `POST_TOUCHDOWN_ROLL`**。
             三條完成標準全達成（無損表示 Step 10R 的 299 幀與一條 Day 8–9 swing）。
             資料逼出四件 §5.4 沒寫的事，最重要的是 **`alpha_range` 不夠用**——
             corner pivot（α 與接觸點都釘死、只有 β 掃）佔 **72/299 幀**。
             另外：rolling 段的 `duration_s` 全是 `None`，Day 6–7 是準靜態的。

7. ~~Step 7~~   **2026-08-30 完成：三條 sequence 都生出來了，全程 collision-free**。
             `#1` h=140/L=225（10 段 272 幀）、`#4` h=80/L=240（2 段 62 幀）、
             `#5` h=60/L=75（1 段 31 幀，且是 sweep 未掃過的格）。
             **這一步最大的價值是它抓到前面幾步的三個缺陷**——見 Day 10–11 規格 Step 7。
             最重要的一個：`required_top_length` 是成功後回報的**消耗量**，不是可行性前提。
             Step 5 的地圖已用修正後的界線重跑（`#1`: 205–245 → 215–245 mm）。
             `duration_s` 對 rolling 段仍是 `None`——指定它是新的建模決定，留給 Step 9。

8. ~~Step 8~~   **2026-08-30 完成，而且它的第三條完成標準【無法達成】——那就是結果。**
             規格要「至少一個 case 的最佳解是混合的，否則 2×2 沒有被驗證」。
             兩個混合對都被推翻了（`#3` by Step 2b、`#2` by Step 3），
             所以 **2×2 不是未被驗證，是【對角線之外被推翻】**。
             但 Step 8 量出了代價，而那證明分開決策**確實有價值**：
             `h = 160 mm` 時 Day 6–7 的 `roll_up` **10/10** 成功、`roll_down` **0/10**，
             而那個用不了的爬升對 body 只要 **80.1 mm**，
             被迫改用的 `SWING_UP` 要 **220.0 mm**——**貴 2.7 倍**。
             擋住它的 Step 3 E 段那道牆是**可修的**，這一格量出了修它值多少。

9. ~~**Step 9 之前的兩個小修正**~~ **2026-08-30 完成**（都是標籤與型別，沒有動任何數字）
   (i)  `Verdict` 階梯型別（`COMPOSED` / `DIRECT_HANDOFF_INFEASIBLE` /
        `REQUIRES_MULTILEG_REPOSITION` / `PHYSICALLY_INFEASIBLE`），
        並把純字串 `REFUTATIONS` 換成結構化的 `BLOCKED_PAIRS`
        （`verdict` / `evidence` / `single_leg_fix` / `multileg_route`）——
        字串會被單獨引用，紀錄強迫呼叫端連退路一起帶走。
        `Availability.REFUTED` → `HANDOFF_BLOCKED`。
        **`effective_verdict` 永遠不會推導出 `PHYSICALLY_INFEASIBLE`。**
   (ii) `TransitionRequirement2D` + `MotionSequence2D.unresolved` / `.is_complete`。
        它**不帶軌跡也不帶時間**；`resolved=True` 會拋錯（解掉它的方式是換成真正的動作）。

10. Step 9   body requirement timeline（Day 12 交接物）  <- 【下一個】
    `#1/#4/#5` 輸出 timeline；`#2/#3` 輸出未解的 transition requirement。
    **必須明說**：Day 6–7 的 rolling 是準靜態資料、沒有 duration，
    任何 rolling timing 都是新的建模決定。

11. **Day 10–11 之後的六個方向**（day10-11 規格 §7 Step 10 有完整版）
    優先 3（`RETRACT_FOR_SWING_DOWN`）是**最可能快速拿回 `#2` 的單腿介面修正**，
    價碼已量出：`h = 160 mm` 上 2.7 倍的 body 代價。
```

<details>
<summary>原文（Day 3–5 時期，保留供追溯）</summary>

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

</details>

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
