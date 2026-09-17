# Hybrid Gait Day 12：Generalizable Four-Leg Whole-Body Integration 與對稱地形 Traversal

> <span style="color:#188038"><b>FINAL FREEZE UPDATE：</b></span> 本版加入 flat-ground nominal Hybrid 的最終定義：有限 foot-rim rolling stroke + airborne compact recovery；contact-phase rolling direction 保持一致；Day 12 MVP 的 recovery 先使用可參數化的 `theta_compact = 17°`，之後再升級成 minimum-clearance retraction。  
>
> **用途**：這份筆記是 Day 12 的實作規格、研究發想紀錄與可逐步交給 AI /
> Codex 的 task list。\
> **更新背景**：本版本建立在 Day 10--11 已完成的 motion sequence /
> body-requirement 結果上，並納入 2026-08-31 新 freeze 的 Hybrid gait
> 定義。\
> **Day 12 核心目標**：把單腳 terrain-transition planning
> 接成四腳、時間同步、具有基本三腳支撐穩定性檢查的 whole-body
> trajectory，並讓同一套 planner 能接受**參數化的已知地形**，而不是
> 為某一個 obstacle 尺寸寫死規則。\
> <span style="color:#188038"><b>🟩 新的 Generalization 原則：</b></span>
> planner 的輸入是 `TerrainProfile / obstacle geometry / initial state /
> goal / constraints`；`4 cm × 40 cm`、`10 cm × 40 cm`、`19 cm × 40 cm`
> 只是後續實驗 query，不得出現在核心 decision rule 的 hard-coded
> branch 中。\
> <span style="color:#b06000"><b>🟨 Day 12 integration smoke test：</b></span>
> 可以先用 `4 cm × 40 cm` 對稱平台讓 whole-body pipeline 跑通，但
> Step 10 必須再用至少另一個不同高度驗證「同一份 code、只換 terrain
> parameters」仍能正常規劃或回報 infeasible。

------------------------------------------------------------------------

## 修改標記

- <span style="color:#188038"><b>🟩 綠色</b></span>：本次新增或實質修改。
- <span style="color:#b06000"><b>🟨 黃色</b></span>：本次特別澄清，避免 AI 把實驗尺寸寫進 planner。
- <span style="color:#c5221f"><b>🟥 紅色</b></span>：舊版假設被取代，不應再作為 Day 12 的固定條件。

## <span style="color:#188038">🟩 0.1 Generalization Freeze：演算法與實驗地形要分開</span>

本研究的 planner 應該解的是一個**參數化 terrain-planning problem**，不是：

```text
if obstacle_height == 0.04:
    ...
elif obstacle_height == 0.10:
    ...
elif obstacle_height == 0.19:
    ...
```

正確架構是：

```text
Known TerrainProfile
(height, length, edge locations, surface IDs, ...)
        ↓
same contact / feasibility / motion-selection pipeline
        ↓
same four-leg whole-body integration
        ↓
trajectory OR explicit infeasible result
```

實驗地形則只是對同一個 planner 的 evaluation queries：

```text
E0  Flat ground
E1  0.04 m high × 0.40 m long platform
E2  0.10 m high × 0.40 m long platform
E3  0.19 m high × 0.40 m long platform  (challenge)
```

其中 `0.40 m` 是實驗平台長度，不應被 planner 當成特殊常數。未來改成
`0.20 m / 0.55 m / multiple obstacles` 時，核心 planner interface 不應改動。

<span style="color:#b06000"><b>🟨 特別注意：</b></span>
如果某一組地形落在現有 geometry / kinematics / stability capability 外，
正確輸出是 `infeasible + reason`，不是為了讓指定實驗成功而偷偷放寬
constraint 或加入該高度專屬 heuristic。

------------------------------------------------------------------------

# <span style="color:#188038">🟩 0.2 FINAL FREEZE：Flat-Ground Nominal Hybrid Cycle</span>

前一版把 `FOOT_RIM_ROLL` 寫得太像「foot rim 可以永遠保持 contact 而不需要週期性抬腳」。
本版修正為更符合實際 leg-wheel geometry 的 nominal cycle：

```text
ROLLING STANCE
    foot rim in terrain contact
    forward rolling direction is fixed
    contact evolves along usable foot-rim arc
        ↓
reach rolling-stroke endpoint
        ↓
LIFTOFF
        ↓
COMPACT RECOVERY / REPOSITION
    retract theta toward theta_compact
    beta / leg-wheel orientation continues in the same nominal forward rotation sense
    no rolling constraint because the leg is airborne
        ↓
extend theta toward the next touchdown configuration
        ↓
TOUCHDOWN
        ↓
next ROLLING STANCE
```

因此，Day 12 後續用詞應區分：

```text
FOOT_RIM_ROLL
    = 有 terrain contact 的 rolling stance segment

RECOVERY_SWING
    = rolling stroke 之間的 airborne reposition segment

NOMINAL_HYBRID_CYCLE
    = FOOT_RIM_ROLL + RECOVERY_SWING
```

<span style="color:#b06000"><b>🟨 Rotation-direction 原則：</b></span>

- 只要是 terrain-contact `ROLL` phase，forward locomotion 對應的 rolling direction 必須一致。
- airborne recovery 不稱為「逆滾」或「順滾」，因為此時沒有 rolling contact constraint。
- Day 12 nominal design 讓 leg-wheel orientation 在 recovery 中**延續同一個 forward rotation sense**，再進入下一次 touchdown。
- 若未來 optimization 證明其他 airborne path 更佳，可以比較，但不要改變 contact-phase forward rolling 的 semantic。

<span style="color:#b06000"><b>🟨 Compact-recovery 原則：</b></span>

Day 12 MVP 先採：

```text
theta_compact = 17 deg
```

但 `17 deg` 必須只是一個**可設定的 recovery parameter**，不能散落成 obstacle-specific hard code。

Day 12 的 recovery trajectory：

```text
theta_roll_end
    ↓ retract
theta_compact
    ↓ rotate/reposition while airborne
theta_compact
    ↓ extend
theta_touchdown
```

其中：

```text
theta_touchdown
```

由下一個 desired `ContactState` / terrain geometry 決定，**不是固定 17 deg**。

未來可升級成：

```text
theta_compact
    = minimum retraction satisfying
      terrain clearance
      + body clearance
      + joint limits
      + valid next touchdown
```

這個 optimization 不屬於 Day 12 MVP。

<span style="color:#c5221f"><b>🟥 被取代的舊假設：</b></span>

不要再使用：

```text
flat Hybrid = continuous FOOT_RIM_ROLL forever with no swing
```

也不要宣稱：

```text
Hybrid on flat ground has zero swing events
```

新的科學敘事是：

> **The nominal Hybrid gait exploits foot-rim rolling during stance and uses a compact airborne recovery to reposition the leg-wheel for the next rolling stroke.**

因此 Hybrid vs pure Walk 的潛在優勢應由後續實驗驗證，例如：

```text
rolling displacement per stance
swing trajectory / swing work
body / CoM vertical variation
electrical energy / COT
```

Day 12 只需要把這些 trajectory-level quantities 留下來，不負責證明 Hybrid 一定比 Walk 省能。

------------------------------------------------------------------------

# 1. Day 12 前先重新 Freeze：現在 Hybrid Gait 是什麼？

本研究不再把 Hybrid gait 定義成：

``` text
wheel mode
    ↕
walking mode
```

也不要求整條平地都執行 terrain-aware ROLL/SWING decision。

新的定義是：

> **Hybrid gait 以展開腿構型下的 foot-rim rolling 作為 nominal
> locomotion；當機器人接近已知地形中的 discontinuous terrain transition
> 時，terrain-aware planner 根據 rolling / swing 的 body requirement 與
> feasibility，決定維持連續 rim rolling，或插入必要的 swing /
> reposition；再由 whole-body planner 協調四腳與 body motion 完成穩定
> traversal。**

概念上：

``` text
                         HYBRID GAIT
                              |
          +-------------------+-------------------+
          |                                       |
  Nominal locomotion                       Terrain transition
  flat / safe region                       obstacle edge region
          |                                       |
  NOMINAL_HYBRID_CYCLE                  terrain-aware decision
  ROLL + compact recovery                              |
  no repeated swing                    +---------+---------+
                                       |                   |
                                     ROLL                SWING
                                       |                   |
                                       +---------+---------+
                                                 |
                                         return to nominal
                                         FOOT_RIM_ROLL
```

## 1.1 本版本的重要新決策

平地 nominal propagation **不是強制 theta = 17 deg 的 wheel mode**。

Day 12 第一版應保留：

``` text
FOOT_RIM_ROLL
```

代表：

-   腿維持 Hybrid gait 的 nominal expanded configuration；
-   theta 不必等於 17 deg；
-   foot rim 與地面保持接觸；
-   利用 foot-rim contact evolution / rolling 讓 body 前進；
-   不需要週期性抬腳；
-   只有接近 obstacle transition 時才啟動 terrain-aware transition
    planner。

因此舊規格中的 `WHEEL_ROLL` 若實際代表
`theta = 17 deg`，**不要直接拿它當本研究平地 Hybrid 的唯一 nominal
propagation 定義**。

建議 Day 12 新增或重新命名 semantic segment：

``` text
FOOT_RIM_ROLL
```

如果程式相容性暫時需要保留 `WHEEL_ROLL` enum，可以：

``` text
先不破壞舊 enum
但 planner-level semantic 明確區分：

FOOT_RIM_ROLL   = expanded-leg nominal flat propagation
WHEEL_ROLL      = theta ~= 17 deg true wheel-mode rolling（若某 transition 內部需要）
```

不要讓兩者在 paper 或 CSV 裡混為一談。

------------------------------------------------------------------------

# 2. Day 12 到底要解什麼問題？

Day 3--11 大部分仍是 single-leg / 2D planning：

``` text
terrain
   ↓
contact / collision
   ↓
ROLL feasibility
   ↓
SWING feasibility
   ↓
transition motion selection
   ↓
per-leg MotionSegment2D
   ↓
body requirement
```

但單腳 planner 無法回答：

``` text
這隻腳現在可以離地嗎？
其他三腳能不能支撐 body？
四隻腳的 body requirement 能不能同時成立？
哪一隻腳先做 transition？
四腳 trajectory 如何放到同一條時間軸？
```

因此 Day 12 的核心問題是：

> **Given per-leg terrain-transition motion sequences, construct a
> synchronized four-leg whole-body trajectory that satisfies contact,
> collision, body-motion, timing, and basic quasi-static support
> constraints.**

Day 12 是從：

``` text
single-leg motion planning
```

正式跨到：

``` text
whole-robot gait planning
```

的第一天。

------------------------------------------------------------------------

# 3. Day 12 的 Scope

## 3.1 Day 12 必做

``` text
parameterized symmetric rectangular terrain input
gamma = 0
four-leg synchronized timeline
nominal FOOT_RIM_ROLL before / after obstacle
per-leg terrain-transition sequence
body trajectory assembly
one-leg-at-a-time swing scheduling
three-leg support triangle
CoM projection / stability margin
TOP_REPOSITION resolution
collision / continuity validation
complete simulation trajectory
```

## 3.2 Day 12 第一版不要做

``` text
不要做 ABAD gamma optimization
不要做左右不對稱 obstacle
不要做 dynamic ZMP / capture point
不要做 MPC / whole-body optimization
不要做 online replanning
不要做 perception
不要一次做 >= 2 m multi-obstacle long route
不要把 4 cm / 10 cm / 19 cm 寫成 planner 內部的特殊 case
不要把 40 cm top length 寫成固定 gait constant
不要重寫 Day 3–11 已完成的 geometry / swing / rolling engine
不要把 terrain logic 塞進 Hybrid::Step() 或 motor layer
```

ABAD 主動提高 stability margin 留給 Day 13--14。

Day 12 只先回答：

> **在 gamma = 0 的情況下，這個 four-leg schedule
> 是否具有合法的三腳支撐？**

------------------------------------------------------------------------

# 4. <span style="color:#188038">🟩 Day 12 的 Integration Demo 與後續 Evaluation Terrain 分離</span>

<span style="color:#c5221f"><b>🟥 舊版：</b></span>
把 `40 mm × 100 mm` 寫成 Day 12 的主要固定 testcase。

<span style="color:#188038"><b>🟩 新版：</b></span>
Day 12 的程式介面先支援**任意參數化的左右對稱 rectangular platform**：

```text
height = input parameter
top_length = input parameter
x_start = input parameter
flat_before / after = input parameter
```

whole-body integration 不得知道「這是不是 4 cm、10 cm 或 19 cm 實驗」。

## 4.1 Day 12 smoke test

為了快速 debug，可以先跑：

```text
height = 0.04 m
top_length = 0.40 m
left-right symmetric
flat ground before / after
```

流程：

```text
flat ground
    ↓
FOOT_RIM_ROLL
    ↓
leading-edge transition
    ↓
platform top
    ↓
FOOT_RIM_ROLL / top reposition if required
    ↓
trailing-edge transition
    ↓
lower ground
    ↓
FOOT_RIM_ROLL
```

## 4.2 Day 12 generalization gate

在宣告 Day 12 integration 完成前，至少再換一組不同的 terrain parameter
做 regression，例如：

```text
height = 0.10 m
top_length = 0.40 m
```

要求：

```text
same planner code
same API
same decision pipeline
only terrain parameters change
```

結果可以是：

```text
valid trajectory
```

或：

```text
infeasible + structured failure reason
```

兩者都能證明 planner 沒有把 4 cm 寫死；是否能 traverse 則由 capability
與 constraints 決定。

## 4.3 最終實驗 evaluation set（不是 Day 12 hard-coded target）

```text
E0  Flat ground
E1  H = 0.04 m, L = 0.40 m
E2  H = 0.10 m, L = 0.40 m
E3  H = 0.19 m, L = 0.40 m   challenge
```

這四組是 paper / simulation / hardware 的**評估條件**。它們不應改變
planner architecture、segment schema、decision rule 或 support constraint。

特別是 E3：

> `19 cm` 應被視為 challenge query。若 planner 根據真實幾何、joint
> limits、collision 或 support constraints 判定 infeasible，必須保留這個
>結果；不要為了「實驗表上要有 19 cm」而加入特製軌跡。

------------------------------------------------------------------------

# 5. Day 12 的輸入與輸出

## 5.1 輸入

Day 12 不要輸入：

``` text
OneObstacleTraversal
```

而應輸入可組合的：

``` text
list[MotionSegment2D]
```

或等價 segment sequence。

每段至少應能提供：

``` text
segment_kind
start_state
end_state
samples / geometric path
sampling parameters
body requirement
transition requirement
contact / rim information
```

Day 10--11 已經建立的資訊應直接 reuse。

## 5.2 Day 12 whole-body output

至少產生：

``` text
time

body_x
body_y
body_z
body_roll
body_pitch
body_yaw

leg0_theta / beta / gamma
leg1_theta / beta / gamma
leg2_theta / beta / gamma
leg3_theta / beta / gamma

per-leg mode
per-leg rim
per-leg alpha
per-leg contact point

swing_leg
support_legs

stability_margin
segment_index
segment_kind
```

Day 12 可以先輸出 internal trajectory structure；若已有 CSV
writer，最好同步輸出 deterministic CSV。

------------------------------------------------------------------------

# 6. Day 12 實作順序總覽

建議嚴格按照：

``` text
Step 0   Freeze Day 12 semantics / interface
Step 1   Implement nominal FOOT_RIM_ROLL
Step 2   Build four-leg initial state and terrain registration
Step 3   Build four-leg timing skeleton
Step 4   Map per-leg transition sequences onto common timeline
Step 5   Assemble body requirement → body trajectory
Step 6   Add three-leg support triangle + CoM margin
Step 7   Resolve TOP_REPOSITION
Step 8   Assemble complete joint/contact trajectory
Step 9   Whole-body validation
Step 10  parameterized-terrain generalization test + 4/10 cm smoke tests
Step 11  Export paper-oriented metrics / plots
Step 12  Day 12 freeze + handoff to Day 13–14
```

不要一次叫 AI 寫完 Day 12。

每完成一個 Step：

``` text
run tests
→ visualize
→ inspect trajectory
→ commit / freeze
→ 再做下一步
```

------------------------------------------------------------------------

# 7. Step 0 --- Freeze Day 12 Semantic Contract

## 目的

先避免 AI 把新 Hybrid 又做回：

``` text
theta = 17 deg wheel mode
→ obstacle
→ walking
→ theta = 17 deg wheel mode
```

Day 12 開始前先寫清楚：

``` text
Nominal flat locomotion:
    FOOT_RIM_ROLL

Terrain transition:
    ROLL / SWING / SWING_OVER / TOP_REPOSITION ...
```

## 建議新增 semantic

``` text
FOOT_RIM_ROLL
```

定義：

> expanded-leg configuration 下，以 foot rim 為主要 ground contact，利用
> rolling contact propagation 前進的 nominal Hybrid locomotion segment。

## 這一步也要確認

``` text
MotionSegment2D 是否可表示 FOOT_RIM_ROLL
segment start / end state 是否統一
segment chaining 是否不依賴 obstacle object
CSV / log 是否能辨識 FOOT_RIM_ROLL
```

## 給 AI / Codex 的指令

``` text
Day 12 Step 0 — Freeze the updated Hybrid-gait segment semantics.

Project context:
The proposed Hybrid gait no longer uses theta=17 deg wheel mode as the mandatory nominal flat-ground locomotion. The nominal flat-ground motion is FOOT_RIM_ROLL: the leg stays in an expanded Hybrid posture (theta is not necessarily 17 deg), the foot rim remains in ground contact, and forward motion is generated through rim rolling/contact evolution. Terrain-aware ROLL/SWING decisions are only required near discontinuous terrain transitions.

Please inspect the existing Day 10–11 MotionSegment2D / SegmentKind schema and:

1. Determine whether a semantic FOOT_RIM_ROLL segment can be represented without breaking existing segments.
2. If necessary, add FOOT_RIM_ROLL as a segment kind.
3. Keep WHEEL_ROLL distinct if it specifically means theta ~= 17 deg true wheel-mode rolling.
4. Do not rewrite existing rolling/swing geometry.
5. Do not implement four-leg timing yet.
6. Ensure segment start/end states use a chainable common format.
7. Add regression tests proving:
   - FOOT_RIM_ROLL can be constructed;
   - it is distinguishable from WHEEL_ROLL;
   - one segment's end state can be passed to the next segment.

Before changing code, report the relevant existing classes/files and a minimal modification plan.
```

## 驗收

可以明確回答：

``` text
FOOT_RIM_ROLL != WHEEL_ROLL
```

且 segment schema 不再把 Hybrid flat propagation 綁死在 theta=17 deg。

------------------------------------------------------------------------

# 8. <span style="color:#188038">🟩 Step 1 --- 建立 Nominal Flat-Ground Hybrid Cycle</span>

## 目的

Day 12 的 flat-ground nominal locomotion 不再被建模成一條可以無限延伸的單一 `FOOT_RIM_ROLL` segment。

第一版應建立：

```text
NOMINAL_HYBRID_CYCLE
    =
FOOT_RIM_ROLL
    +
RECOVERY_SWING
```

## 8.1 FOOT_RIM_ROLL

有 contact 的 rolling stroke：

```text
start ContactState
    ↓
roll along usable foot-rim arc
    ↓
end ContactState / rolling-range endpoint
```

要求：

```text
valid foot-rim contact
same forward rolling direction
no terrain penetration
joint limits
positive body progress
continuous theta / beta
```

## 8.2 RECOVERY_SWING

當 rolling stroke 用完：

```text
liftoff
    ↓
retract theta
    ↓
theta_compact = configurable parameter
    ↓
continue nominal forward rotation sense while airborne
    ↓
extend theta toward next touchdown state
    ↓
touchdown
```

Day 12 MVP：

```text
theta_compact = 17 deg
gamma = 0
```

但必須寫成：

```text
recovery_config.theta_compact
```

或等價 config，而不是在 obstacle / gait decision code 中到處 hard-code `17 deg`。

## 8.3 Recovery path 應 reuse swing infrastructure

優先 reuse Day 8--9 的 Cartesian swing / collision checking。

不要另外寫：

```text
special_flat_recovery_swing()
```

如果現有 swing generator 需要增加 orientation / theta recovery constraint，應以 general parameter 擴充。

## 8.4 下一個 touchdown

`theta_touchdown` 不等於 `theta_compact`。

它由：

```text
next desired ContactState
terrain height
body / hip pose
rim/contact requirement
```

共同決定。

因此即使 recovery 中縮到 17 deg，touchdown 前仍需伸回下一段 rolling 所需的 leg length。

## 8.5 給 AI / Codex 的指令

```text
Day 12 Step 1 — Implement the final nominal flat-ground Hybrid cycle.

Final gait semantics:
- A nominal flat-ground cycle is FOOT_RIM_ROLL + RECOVERY_SWING.
- FOOT_RIM_ROLL is a finite terrain-contact rolling stroke along the usable foot-rim arc.
- All contact-phase rolling strokes preserve the same forward rolling direction.
- When the rolling stroke reaches its endpoint, the leg lifts off.
- During RECOVERY_SWING, there is no rolling-contact constraint.
- The leg retracts toward a configurable compact recovery posture, continues the nominal forward rotation sense while airborne, then extends toward the configuration required by the next touchdown ContactState.
- For the Day 12 MVP, use theta_compact = 17 deg as a configurable recovery parameter.
- theta_touchdown is determined by the next ContactState and is not fixed to 17 deg.
- gamma = 0.

Implementation requirements:
1. Reuse existing contact/rolling kinematics for FOOT_RIM_ROLL.
2. Reuse the Day 8–9 swing/collision infrastructure for RECOVERY_SWING.
3. Do not create obstacle-height-specific recovery logic.
4. Do not hard-code 17 deg throughout the planner; isolate it in RecoveryConfig or equivalent.
5. Generate at least two consecutive nominal cycles on flat ground.
6. Validate:
   - valid foot-rim contact during rolling;
   - same forward rolling direction for consecutive contact phases;
   - liftoff before recovery rotation;
   - terrain clearance during recovery;
   - joint limits;
   - valid touchdown;
   - continuous body/joint trajectory;
   - positive net forward progress.
7. Return chainable MotionSegment2D-compatible FOOT_RIM_ROLL and RECOVERY_SWING segments.
8. Produce an animation and a phase plot showing ROLL / RECOVERY / ROLL.

Before implementation, inspect whether the existing Day 8–9 swing generator can express the required compact-recovery orientation change. Prefer extending a general interface over adding a flat-ground-only swing generator.
```

## 驗收

至少能在 flat ground 上連續產生：

```text
FOOT_RIM_ROLL
→ RECOVERY_SWING
→ FOOT_RIM_ROLL
→ RECOVERY_SWING
```

並且：

```text
contact-phase rolling direction is consistent
theta_compact is configurable
next touchdown leg length is terrain/state dependent
no terrain collision during recovery
net body displacement > 0
```

------------------------------------------------------------------------

# 9. Step 2 --- 建立 Four-Leg Initial State 與 Terrain Registration

## 目的

把單腳 2D trajectory 放進真實四腳 robot geometry。

需要建立：

``` text
LF
RF
LH
RH
```

各自的 hip / module offset。

## 第一版

``` text
body roll = 0
body pitch = 0
body yaw = 0
gamma = 0
symmetric obstacle
```

左右腳 longitudinal terrain profile 相同，但 hip x offset
不同，所以前後腳到達 edge 的時間不同。

## 要求

對每隻腳能取得：

``` text
hip pose in world
contact point in world
terrain surface under / near leg
current theta / beta / gamma
```

## 給 AI / Codex 的指令

``` text
Day 12 Step 2 — Build the four-leg world-frame initialization and terrain registration.

Please create the minimal whole-body state needed to place the existing single-leg 2D planner outputs on the four-leg robot.

Requirements:
1. Represent LF/RF/LH/RH hip/module offsets relative to the body.
2. Use the project's frozen world-frame convention.
3. gamma = 0.
4. body roll/pitch/yaw = 0 for the first symmetric test.
5. Register one parameterized symmetric rectangular platform in world coordinates; height/top_length/x_start must come from terrain input rather than fixed experiment constants.
6. For each leg, compute:
   - hip pose in world;
   - current contact point in world;
   - terrain surface ID;
   - theta/beta/gamma.
7. Do not implement gait timing yet.
8. Add a visualization showing body, four hips, four contact points, and the obstacle.
9. Add symmetry sanity checks for left/right legs.

Reuse existing robot dimensions/configuration. Do not invent geometry constants if they already exist in the project.
```

## 驗收

一張圖可以清楚看到：

``` text
body
4 hips
4 foot contacts
parameterized rectangular platform (first smoke test may use 40 mm × 400 mm)
```

座標沒有左右/前後 mapping 錯誤。

------------------------------------------------------------------------

# 10. Step 3 --- Four-Leg Timing Skeleton

## 核心原則

Day 12 第一版：

> **同一時間最多一隻腳 airborne。**

先不要追求最快。

這樣每次 swing 都可以用另外三腳形成 support triangle。

## Reuse 舊架構

盡量 reuse：

``` text
duty
swing_phase
existing leg ordering / phase semantics
Hybrid::Step() concepts
```

但注意：

> terrain decision 已經 offline 做完；runtime timing framework
> 只能用來組 trajectory / playback，不得重新做 terrain decision。

## 第一版 scheduler 要回答

``` text
哪隻腳現在可以進 transition？
哪隻腳必須保持 support？
哪隻腳正在 FOOT_RIM_ROLL？
哪隻腳正在 SWING？
```

## 給 AI / Codex 的指令

``` text
Day 12 Step 3 — Build a minimal four-leg timing skeleton.

Goal:
Create a common timeline for LF/RF/LH/RH without redesigning the legacy gait timing system.

Requirements:
1. Reuse existing duty/swing_phase/leg-order semantics where practical.
2. The planner is offline; do not put terrain decisions into runtime Hybrid::Step().
3. Enforce at most one airborne leg at a time in the first version.
4. Each leg must have an explicit state/mode on every timeline interval.
5. Support arbitrary MotionSegment2D sequences rather than a hard-coded one-obstacle object.
6. The scheduler must expose:
   - active/swing leg;
   - three support legs;
   - segment index/kind per leg;
   - start/end time of each segment.
7. Do not solve support-polygon stability yet; only produce the timing skeleton.
8. Add a timeline plot for the four legs.

First show the proposed scheduling data structure and mapping to existing duty/swing_phase before implementation.
```

## 驗收

得到類似：

``` text
time --->

LF  FOOT_RIM | SWING_UP | TOP ... | ...
RF  FOOT_RIM | SUPPORT  | ...
LH  FOOT_RIM | SUPPORT  | ...
RH  FOOT_RIM | SUPPORT  | ...
```

而且任何時間：

``` text
airborne_count <= 1
```

------------------------------------------------------------------------

# 11. Step 4 --- 將 Day 10--11 Per-Leg Motion Sequence 放入共同時間軸

## 目的

Day 10--11 已經知道 obstacle transition 要求什麼 motion。

Day 12 不要重新決策。

Day 12 做的是：

``` text
per-leg sequence
      ↓
schedule / synchronize
      ↓
whole-body timeline
```

## 重要

上升與下降仍是獨立 transition decision。

不要寫成：

``` text
this obstacle = ROLL
```

而應保留：

``` text
ascent primitive
descent primitive
```

各自來自 Day 10--11。

## 給 AI / Codex 的指令

``` text
Day 12 Step 4 — Map the existing per-leg Day 10–11 motion sequences onto the common four-leg timeline.

Requirements:
1. Treat Day 10–11 motion selection as already solved; do not recompute the ROLL/SWING decision here.
2. Preserve ascent and descent as independent transition decisions.
3. Insert nominal FOOT_RIM_ROLL outside terrain-transition regions.
4. Preserve each segment's sampling parameters and body requirement.
5. If a TransitionRequirement2D is unresolved, keep it explicit instead of silently inventing a trajectory.
6. Ensure every segment exit state is a valid next-segment entry state.
7. Detect and report timing/contact conflicts rather than hiding them.
8. Produce a debug table:
   time interval | leg | segment kind | contact/airborne | body requirement.

Do not implement TOP_REPOSITION resolution yet.
```

## 驗收

四隻腳都已經有完整的「想做什麼」timeline。

------------------------------------------------------------------------

# 12. Step 5 --- Body Requirement → Whole-Body Trajectory

## Day 10--11 已經提供

概念上：

``` text
TRACK / PINNED
LOWER_BOUND
```

Day 12 要把四隻腳同時提出的 requirement 合起來。

## 第一版原則

不要做 full optimization。

可以使用 deterministic merge：

``` text
hard requirement
    必須滿足

lower-bound requirement
    body_z 至少滿足最大需求

沒有 requirement
    優先維持 nominal body trajectory
```

概念上：

``` text
body_z(t)
    >= max(active lower-bound requirements)
```

並同時滿足 active hard constraints。

如果 requirement 互相衝突：

``` text
return infeasible
```

不要偷偷平均。

## CoM 與 body

Day 12 第一版如果完整 robot CoM model 已存在，使用真正 CoM。

若目前只有 body-frame CoM approximation，必須在 log / paper note
明確標記：

``` text
quasi-static body-CoM approximation
```

不要把 approximation 寫成精確 whole-robot CoM。

## 給 AI / Codex 的指令

``` text
Day 12 Step 5 — Assemble the whole-body trajectory from the per-leg body requirements.

Requirements:
1. Consume the body-requirement timeline generated by Day 10–11 and synchronized in Step 4.
2. Respect hard TRACK/PINNED requirements.
3. For LOWER_BOUND requirements, choose the minimum body motion that satisfies all active legs.
4. Do not introduce a weighted whole-body optimizer.
5. Detect incompatible simultaneous hard requirements and report infeasible.
6. Maintain continuity of body_x/body_z and, for the symmetric first test, keep body_y/roll/pitch/yaw at their nominal values unless existing geometry requires otherwise.
7. Output body trajectory samples synchronized with all four legs.
8. Record the resulting vertical body/CoM variation for later evaluation.
9. Add tests with:
   - no body concession;
   - one active lower bound;
   - multiple compatible lower bounds;
   - conflicting hard constraints.

Do not add ABAD adjustment.
```

## 驗收

得到第一條：

``` text
body_x(t)
body_z(t)
```

並且可以說明每一次 body_z 變化是被哪隻腳 / 哪個 transition 要求的。

------------------------------------------------------------------------

# 13. Step 6 --- Three-Leg Support Triangle + CoM Stability Margin

這一步升級成 **Day 12 必做**。

## 問題

當 leg `i` swing：

``` text
other three legs = support legs
```

建立：

\[ P\_{sup}(t)=ConvHull(p\_{c,j,xy}(t)),`\quad `{=tex}j`\neq `{=tex}i \]

要求：

\[ p\_{CoM,xy}(t)`\in `{=tex}P\_{sup}(t) \]

並計算 signed stability margin：

``` text
margin > 0   inside support triangle
margin = 0   on boundary
margin < 0   outside
```

## Day 12 只做 feasibility

不要調 gamma。

如果 margin 不足：

``` text
mark schedule / swing infeasible
```

Day 13--14 才：

``` text
adjust gamma
→ change lateral support geometry
→ improve margin
```

## 重要實作細節

support triangle 必須使用：

``` text
actual world contact points
```

而不是 hip positions。

Rolling support leg 的 contact point 可能隨時間改變，因此 stability
margin 應沿 swing trajectory 取樣，不只檢查 liftoff 一幀。

## 給 AI / Codex 的指令

``` text
Day 12 Step 6 — Add quasi-static three-leg support stability checking.

For every interval in which one leg is airborne:

1. Use the actual world-frame contact points of the other three legs.
2. Build the support triangle / convex hull in the horizontal plane.
3. Project the robot CoM onto the horizontal plane.
4. Compute a signed stability margin:
   positive = inside,
   zero = boundary,
   negative = outside.
5. Evaluate the margin over the complete swing interval, not only at liftoff.
6. Store minimum stability margin for each swing segment and for the complete traversal.
7. If margin < required minimum, mark the segment/schedule infeasible.
8. gamma must remain 0 in Day 12; do not attempt ABAD correction.
9. Add visualizations of:
   - support triangle;
   - CoM projection;
   - swing leg;
   - stability margin.
10. Add unit tests for clearly inside / boundary / outside cases.

Use contact points, not hip positions.
If the project does not yet have an exact whole-robot CoM model, clearly isolate and label the approximation rather than silently treating body center as exact CoM.
```

## 驗收

每一個 swing 都能輸出：

``` text
swing_leg
support_legs
minimum_stability_margin
stable = true / false
```

------------------------------------------------------------------------

# 14. Step 7 --- Resolve TOP_REPOSITION

這是 Day 12 最重要的 cross-layer integration 之一。

Day 10--11 留下：

``` text
TransitionRequirement2D
kind = TOP_REPOSITION
requires_external_support = True
resolved = False
```

原因是單腳 planner 不知道：

``` text
其他三腳能不能支撐 body
```

現在 Day 12 可以回答。

## 流程

``` text
leg reaches safe top contact
        ↓
scheduler assigns other three legs as support
        ↓
support triangle / CoM check
        ↓
stable?
  +-----+-----+
  |           |
 NO          YES
  |           |
infeasible    liftoff
              ↓
        airborne reposition
              ↓
     target ContactState suitable
     for next descent primitive
              ↓
           touchdown
              ↓
        resolved = True
```

## Airborne trajectory

reuse Day 8--9：

``` text
generate_swing_2d()
```

或 generalized swing planner。

不要寫第二套 reposition trajectory generator。

## 給 AI / Codex 的指令

``` text
Day 12 Step 7 — Resolve TOP_REPOSITION using the four-leg support context.

Context:
Day 10–11 intentionally left TOP_REPOSITION unresolved because the single-leg planner cannot know whether the other three legs can support the body.

Requirements:
1. Find unresolved TransitionRequirement2D(kind=TOP_REPOSITION).
2. Schedule the target leg as airborne while the other three legs remain support contacts.
3. Run the Step 6 support-triangle / CoM-margin check over the proposed reposition interval.
4. If support is insufficient, return unresolved/infeasible; do not invent ABAD correction.
5. If support is valid, generate the airborne reposition by reusing the Day 8–9 Cartesian swing generator.
6. The touchdown target must be a valid ContactState for the next planned primitive.
7. Re-run terrain collision/contact validation over the complete reposition trajectory.
8. On success, set the transition requirement to resolved and store the generated segment.
9. Re-test the Day 10–11 cases that previously failed only because direct handoff was impossible.
10. Keep all original failure reasons for traceability.

Do not create a new swing algorithm specifically for TOP_REPOSITION.
```

## 驗收

至少一個原本：

``` text
requires_external_support = True
resolved = False
```

的 case 能在四腳 context 下被真正判定：

``` text
resolved = True
```

或有明確的 support-related failure reason。

------------------------------------------------------------------------

# 15. Step 8 --- Assemble Complete Four-Leg Joint / Contact Trajectory

現在才真正組：

``` text
body trajectory
+
4 × leg trajectories
+
contact state
+
mode
+
support state
```

每一幀至少：

``` text
body pose
LF theta beta gamma
RF theta beta gamma
LH theta beta gamma
RH theta beta gamma
per-leg mode
per-leg contact
stability margin
segment metadata
```

## Continuity

每個 segment handoff 檢查：

``` text
joint jump
body jump
contact-point jump
rim geometry gap
time monotonicity
```

之前已知的約 1.2 mm rim geometry gap：

``` text
量化
記錄
不要在 Day 12 偷改 geometry model
```

除非它實際造成 trajectory 無法執行，再另開 issue。

## 給 AI / Codex 的指令

``` text
Day 12 Step 8 — Assemble the complete synchronized four-leg trajectory.

Inputs:
- common timeline;
- whole-body trajectory;
- four per-leg segment sequences;
- resolved transition requirements;
- support/stability information.

Output one synchronized trajectory structure.

Requirements:
1. Every sample contains body pose and all four legs' theta/beta/gamma.
2. Every leg has explicit mode/contact/rim/alpha metadata.
3. Include swing leg, support legs, stability margin, segment_index, segment_kind.
4. Preserve segment sampling parameters.
5. Validate all segment handoffs:
   - time continuity;
   - body continuity;
   - joint continuity;
   - contact-state continuity;
   - known rim-geometry handoff gap.
6. Quantify the existing ~1.2 mm geometry gap; do not modify the geometry model solely to hide it.
7. Do not perform runtime replanning.
8. Add serialization support if the existing trajectory writer can be reused.

Produce a summary report of maximum joint jump, maximum contact gap, and minimum stability margin.
```

------------------------------------------------------------------------

# 16. Step 9 --- Whole-Body Validation

不要只看 animation。

Day 12 final trajectory 至少驗證：

## Timing

``` text
time strictly increasing
airborne legs <= 1
every swing has exactly three intended support legs
```

## Kinematics

``` text
joint limits
IK residual
joint continuity
```

## Contact

``` text
stance / rolling legs have valid contact
swing leg is not accidentally penetrating terrain
expected touchdown surface
expected rim / alpha where specified
```

## Collision

``` text
ground penetration
obstacle top penetration
vertical-face collision
other relevant leg-wheel geometry collision
```

## Whole-body

``` text
body requirement satisfied
body trajectory continuous
```

## Stability

``` text
CoM inside support triangle during every swing
minimum stability margin recorded
```

## Segment chaining

``` text
end(segment k) -> start(segment k+1)
```

不可 teleport。

## 給 AI / Codex 的指令

``` text
Day 12 Step 9 — Implement whole-body trajectory validation.

Create a validator that checks the complete four-leg trajectory rather than individual primitives only.

At minimum validate:
- strictly increasing time;
- at most one airborne leg;
- joint limits and continuity;
- body trajectory continuity;
- stance/contact validity;
- swing collision-free motion;
- expected touchdown contact state;
- terrain collision;
- segment-to-segment state continuity;
- all active body requirements;
- support triangle and minimum CoM stability margin.

Return structured failure reasons with time, leg, segment_index, segment_kind, and relevant values.

Do not silently repair invalid trajectories inside the validator.
```

------------------------------------------------------------------------

# 17. <span style="color:#188038">🟩 Step 10 --- Parameterized Terrain Integration + Generalization Gate</span>

## 目的

這一步不是「讓 planner 通過某一個特定尺寸」，而是驗證：

> **同一套 whole-body Hybrid planner 能在不修改核心 code 的前提下，
> 接受不同已知 rectangular terrain parameters，產生合法 trajectory 或
> 明確 infeasible reason。**

## Test A — Flat ground sanity check

```text
terrain = flat
```

預期：

```text
NOMINAL_HYBRID_CYCLE
= FOOT_RIM_ROLL + RECOVERY_SWING

no terrain-transition-specific SWING
valid four-leg/support state
```

注意：flat ground 現在**允許且預期存在 nominal recovery swing**；真正不應出現的是沒有必要的 obstacle-transition swing。

這一項很重要，因為它能證明 Hybrid nominal locomotion 本身不是 obstacle-specific
script。

## Test B — Day 12 primary smoke test

```text
height = 0.04 m
top_length = 0.40 m
left-right symmetric
flat ground before / after
```

## Test C — Generalization regression

```text
height = 0.10 m
top_length = 0.40 m
left-right symmetric
flat ground before / after
```

Test B 與 Test C 必須：

```text
use exactly the same planner entry point
use the same segment schema
use the same decision rule
use the same whole-body scheduler
change terrain parameters only
```

如果某一 case 無法通過，必須輸出 structured infeasible reason，不得加入：

```text
if height == 0.04 ...
if height == 0.10 ...
```

## Challenge — 0.19 m × 0.40 m

```text
height = 0.19 m
top_length = 0.40 m
```

Day 12 若時間允許可以先 query / simulation，但**不是 Day 12 integration
是否成功的必要條件**。它是後續 paper experiment 的 challenge terrain。

如果成功：

```text
record trajectory + mode sequence + stability
```

如果失敗：

```text
record first limiting constraint
geometry / collision / joint / body requirement / support stability / handoff
```

這個 failure 本身也有研究價值。

## Success criterion

Day 12 的正式完成條件改成：

> **Given a parameterized known symmetric rectangular terrain profile,
> generate a synchronized four-leg Hybrid trajectory—or a structured
> infeasibility result—without terrain-size-specific logic, while
> satisfying contact, collision, body-motion, timing, and quasi-static
> three-leg support constraints.**

至少要求：

```text
Flat                → valid FOOT_RIM_ROLL
4 cm × 40 cm        → run through the same pipeline
10 cm × 40 cm       → run through the same pipeline
19 cm × 40 cm       → challenge query; no hard-coded workaround
```

## 給 AI / Codex 的指令

```text
Day 12 Step 10 — Validate parameterized-terrain generalization.

Important research constraint:
The planner must NOT be fitted to the experimental obstacle sizes.
The experiment set (flat, 4 cm, 10 cm, 19 cm; 40 cm top length) is only an evaluation set.

Please create one common planner entry point that receives a TerrainProfile /
rectangular-platform geometry as input.

Run:
A. flat ground;
B. H=0.04 m, L=0.40 m;
C. H=0.10 m, L=0.40 m;
D. optionally/challenge: H=0.19 m, L=0.40 m.

Requirements:
1. Do not add height-specific or length-specific branches.
2. Use the same Day 10–11 terrain-transition planning interfaces for all obstacle cases.
3. Use the same Day 12 four-leg timing/body/support pipeline for all cases.
4. Flat ground should execute the nominal FOOT_RIM_ROLL + RECOVERY_SWING cycle. Do not create extra terrain-transition swing events beyond the nominal recovery required to reset the finite foot-rim rolling stroke.
5. For each obstacle, independently select/consume the valid ascent and descent primitives.
6. If a terrain is infeasible, return a structured failure reason; do not relax constraints to make the experiment pass.
7. Produce a comparison table:
   terrain | feasible | ascent primitive | descent primitive |
   swing count | max body lift | min stability margin | failure reason.
8. Assert in tests that changing obstacle height/length does not change planner code paths through explicit experiment-ID special cases.

Generate animation/plots for the first successful obstacle integration and concise
diagnostics for the rest.
```

------------------------------------------------------------------------

# 18. Step 11 --- Day 12 就開始輸出 Paper Metrics

Day 12 不需要等實機才開始記 quantitative result。

至少輸出：

``` text
total traversal distance
total traversal duration

number of swing events
nominal recovery swing count
terrain-transition swing count
swing distance / time

distance under FOOT_RIM_ROLL
distance under transition ROLL

body_z peak-to-peak
body_z RMS or standard deviation

CoM_z peak-to-peak（若有完整 CoM model）
CoM_z RMS（若有完整 CoM model）

minimum stability margin
mean stability margin during swing

maximum body requirement / hip lift
maximum joint discontinuity
maximum contact handoff gap
```

## 特別重要

未來 Hybrid vs Walk 的故事可能是：

``` text
Hybrid:
    rolling-contact stance
    + compact nominal recovery
    + terrain-transition swing only when required
            ↓
    potentially different swing work / body excursion from pure Walk
    potentially smaller vertical body / CoM variation
            ↓
    energy advantage must be verified experimentally
```

但 Day 12 只先量：

``` text
trajectory-level mechanism metrics
```

不要在沒有 energy experiment 前直接宣稱：

``` text
therefore COT is lower
```

那要等實驗。

## 給 AI / Codex 的指令

``` text
Day 12 Step 11 — Add paper-oriented trajectory metrics.

From the final synchronized trajectory, compute and export:
- traversal distance/time;
- total swing count;
- nominal RECOVERY_SWING count;
- terrain-transition swing count;
- time/distance in FOOT_RIM_ROLL;
- time/distance in terrain-transition ROLL;
- body-z peak-to-peak and RMS/std;
- CoM-z metrics if an actual whole-robot CoM model is available;
- minimum support stability margin;
- maximum requested body/hip lift;
- maximum joint handoff discontinuity;
- maximum contact-state handoff gap.

Keep body-center metrics and true whole-robot CoM metrics explicitly separated.

Export raw CSV plus a concise summary table.
Do not infer energy/COT from these metrics.
```

------------------------------------------------------------------------

# 19. Step 12 --- Day 12 Freeze 與 Day 13--14 Handoff

Day 12 做完後應該 freeze：

``` text
four-leg state representation
timeline representation
FOOT_RIM_ROLL semantic
per-leg segment integration
body requirement merge
support triangle API
stability margin API
TOP_REPOSITION resolution path
whole-body trajectory schema
validator
```

Day 13--14 不應重寫這些。

Day 13--14 只在：

``` text
stability margin insufficient
        ↓
adjust gamma
        ↓
change lateral support contact
        ↓
recompute support polygon
        ↓
increase margin
```

這一層往上加。

## 給 AI / Codex 的指令

``` text
Day 12 Step 12 — Freeze the four-leg integration architecture.

Please:
1. Summarize all Day 12 public data structures and APIs.
2. Identify which interfaces Day 13–14 ABAD adjustment should consume.
3. Confirm that gamma is currently fixed at 0 but represented in the trajectory.
4. Confirm that support/stability evaluation is separated from ABAD correction.
5. Confirm terrain reasoning remains outside runtime motor-control code.
6. Run the complete Day 12 regression suite.
7. Write a short architecture note and list remaining known limitations.

Do not start implementing ABAD optimization in this step.
```

------------------------------------------------------------------------

# 20. Day 12 完成條件 Checklist

只有以下全部成立才算 Day 12 完成：

-   [ ] `FOOT_RIM_ROLL` 已有明確 semantic，且不等同於 theta=17 deg wheel
    mode。
-   [ ] 平地可以生成至少兩個連續 `FOOT_RIM_ROLL → RECOVERY_SWING` nominal cycles。
-   [ ] 四腳 world-frame initialization 正確。
-   [ ] 四腳有共同時間軸。
-   [ ] 同一時間最多一腳 airborne。
-   [ ] Day 10--11 per-leg motion sequence 可直接接入。
-   [ ] ascent / descent decision 仍然彼此獨立。
-   [ ] body requirement 可以合成 whole-body trajectory。
-   [ ] 每次 swing 都建立三腳 support triangle。
-   [ ] CoM projection / stability margin 可以計算。
-   [ ] stability margin 不足會被標成 infeasible，而不是被忽略。
-   [ ] `TOP_REPOSITION` 可以在 four-leg context 中被 resolve
    或明確拒絕。
-   [ ] complete four-leg joint/contact trajectory 可生成。
-   [ ] segment handoff 無 teleport。
-   [ ] collision / contact / joint / timing validator 全部通過。
-   [ ] terrain height / length / x-position 由 `TerrainProfile` 或等價輸入提供，核心 planner 沒有 4/10/19 cm 專屬 branch。
-   [ ] Flat-ground case 能以 `FOOT_RIM_ROLL + RECOVERY_SWING` 正常 propagation；不產生額外 terrain-transition swing。
-   [ ] 4 cm × 40 cm 與 10 cm × 40 cm 至少都能用**同一個 planner entry point** 執行；結果可為 valid trajectory 或 structured infeasible。
-   [ ] 19 cm × 40 cm 可作 challenge query；不得為它加入 hard-coded workaround。
-   [ ] obstacle 前後都回到 `FOOT_RIM_ROLL` nominal locomotion。
-   [ ] paper-oriented raw metrics 已輸出。

------------------------------------------------------------------------

# 21. Day 12 不成功時，先看哪裡？

不要一失敗就改 planner。

依序分類：

``` text
A. SEGMENT_SCHEMA_FAIL
   segment 接不起來 / state 格式不同

B. FOOT_RIM_ROLL_FAIL
   nominal flat propagation 本身不成立

C. TIMING_CONFLICT
   同時需要兩腳 airborne / schedule 衝突

D. BODY_REQUIREMENT_CONFLICT
   四腳對 body 的 hard requirements 無法同時滿足

E. SUPPORT_STABILITY_FAIL
   CoM 不在三腳 support triangle

F. TOP_REPOSITION_FAIL
   support 不足 / swing path 不可行 / touchdown 不合法

G. CONTACT_OR_COLLISION_FAIL
   whole-body assembly 後出現 terrain interference

H. HANDOFF_DISCONTINUITY
   segment boundary joint/contact jump 過大

I. TERRAIN_CAPABILITY_FAIL
   輸入 terrain 落在 Day 10–11 / whole-body capability 外；
   failure 必須來自 geometry/kinematics/collision/support constraint，
   不能因為「不是指定實驗尺寸」而失敗

J. EXPERIMENT_OVERFIT
   planner 出現 4 cm / 10 cm / 19 cm 或 40 cm length 專屬 branch；
   這屬於 architecture failure，必須移除 hard-coded experiment logic
```

這個 failure taxonomy 之後也很適合保留在 paper development log。

------------------------------------------------------------------------

# <span style="color:#188038">🟩 21.1 Planner Generality 與 Experimental Evaluation 的 Paper 寫法</span>

paper 裡應該把兩件事分開寫：

```text
Method:
    planner accepts known TerrainProfile
    → terrain transition detection
    → ROLL/SWING capability / body-requirement decision
    → whole-body scheduling and stability validation

Experiments:
    query the same planner on:
    flat
    4 cm × 40 cm
    10 cm × 40 cm
    19 cm × 40 cm challenge
```

不要寫成：

```text
we designed a gait for 4 cm / 10 cm / 19 cm obstacles
```

比較好的 framing 是：

> **The proposed planner is terrain-parameterized; the selected platform
> heights are evaluation conditions used to probe nominal, moderate, and
> challenge-level traversal behavior.**

19 cm 是否最後成功，應由實際 capability 結果決定；不要先在方法章承諾它一定可行。

------------------------------------------------------------------------

# 22. Day 12 對 Paper 的研究脈絡

Day 12 不只是「把 code 接成四腳」。

它第一次把前面的三條研究線接起來：

``` text
Terrain-aware contact geometry
          +
ROLL / SWING transition planning
          +
Whole-body support feasibility
          ↓
Executable Hybrid gait
```

## 22.1 Hybrid 的新敘事

可以寫成：

> The proposed Hybrid gait uses foot-rim rolling in an expanded leg
> configuration as its nominal locomotion mechanism. Terrain-aware
> motion selection is activated around discontinuous terrain
> transitions, where continuous rolling is retained when favorable and
> swing/repositioning is introduced only when required. The resulting
> per-leg motions are coordinated at the whole-body level under support
> constraints.

這比：

``` text
switch between wheel mode and walking mode
```

更貼近目前真正做的事情。

## 22.2 Day 12 對 Hybrid vs Walk 的意義

Walk：

``` text
periodic stance
→ swing
→ stance
→ swing
```

Proposed Hybrid：

``` text
nominal FOOT_RIM_ROLL
→ no repeated swing on ordinary flat region

only near terrain transition:
    ROLL or SWING/reposition
```

因此之後可以比較：

``` text
swing count
rolling distance ratio
body / CoM vertical variation
energy / COT
```

其中 Day 12 已經能產生前三類 trajectory-level 指標；energy / COT 留給
simulation / hardware experiment。

## 22.3 Day 12 對 ABAD contribution 的鋪路

Day 12 先得到：

``` text
support triangle
CoM projection
stability margin
```

Day 13--14 才能自然地說：

``` text
without ABAD:
    margin insufficient

with ABAD:
    gamma changes lateral contact
    → support polygon changes
    → margin improves
```

所以 ABAD 不是為了「有第三自由度所以要用」，而是對 Day 12 暴露出的
whole-body stability problem 提供機構上的解法。

------------------------------------------------------------------------

# 23. 建議 Day 12 的實際工作節奏

如果時間非常趕，優先順序：

``` text
P0 必做
Step 0  semantic/interface
Step 1  FOOT_RIM_ROLL
Step 2  four-leg registration
Step 3  timing
Step 4  sequence mapping
Step 5  body trajectory
Step 6  support triangle
Step 8  full trajectory
Step 9  validation
Step 10 parameterized terrain + generalization test

P1 很重要
Step 7  TOP_REPOSITION
Step 11 paper metrics

P2 收尾
Step 12 architecture freeze note
```

但如果 final testcase 實際需要 `TOP_REPOSITION`，Step 7 自動升級成 P0。

------------------------------------------------------------------------

# 24. 給 AI 的 Day 12 共用 Context

之後每一個 Step 都可以把下面這段貼在最前面：

``` text
Project context — Hybrid Gait Day 12

This project develops an offline terrain-aware Hybrid gait planner for a leg-wheel robot with ABAD.

The current Hybrid-gait definition is:

- Nominal locomotion on ordinary flat/safe terrain is a repeated NOMINAL_HYBRID_CYCLE = FOOT_RIM_ROLL + RECOVERY_SWING.
- FOOT_RIM_ROLL is a finite contact-phase rolling stroke, not an infinite no-swing motion.
- When the usable foot-rim stroke ends, the leg lifts off and performs RECOVERY_SWING.
- During RECOVERY_SWING, retract toward configurable theta_compact (Day 12 MVP: 17 deg), continue the nominal forward rotation sense while airborne, then extend to the next touchdown ContactState.
- All terrain-contact rolling phases preserve the same forward rolling direction.
- Airborne recovery has no rolling-contact constraint and should not be described as reverse rolling.
- FOOT_RIM_ROLL uses an expanded leg configuration; theta is not necessarily 17 deg.
- The foot rim remains in terrain contact and forward motion is produced through rolling/contact evolution.
- Terrain-aware motion selection is activated near discontinuous terrain transitions.
- At each terrain transition, the existing Day 10–11 planner provides the selected ROLL/SWING-related motion sequence and body requirements.
- Day 12 must NOT redo the Day 10–11 motion-selection research.
- Day 12 integrates the per-leg sequences into a synchronized four-leg whole-body trajectory.
- At most one leg is airborne at a time in the first version.
- During every airborne phase, the other three contact points form a support triangle and the projected CoM must satisfy the quasi-static support constraint.
- gamma is fixed to 0 in Day 12. ABAD-based correction is Day 13–14.
- TOP_REPOSITION was intentionally left unresolved by the single-leg planner and may only be resolved here using four-leg support information.
- The planner is offline. Runtime performs deterministic trajectory playback only.
- Reuse existing contact, rolling, swing, kinematics, duty/swing-phase, and trajectory infrastructure. Do not duplicate geometry or put terrain decisions into Hybrid::Step() or the motor layer.
- Terrain geometry must be parameterized. The planner must not contain special cases for the paper's evaluation terrains.
- The current evaluation set is: flat ground; H=0.04 m/L=0.40 m; H=0.10 m/L=0.40 m; and H=0.19 m/L=0.40 m as a challenge.
- Day 12 may use H=0.04 m/L=0.40 m as the first integration smoke test, but at least one additional height must be run through the exact same pipeline as a generalization regression.
```

------------------------------------------------------------------------

# 25. 一句話版本

Day 12 的一句話目標：

> **把「平地 foot-rim rolling + obstacle transition 的 ROLL/SWING
> planning」組成一條具有四腳 timing、body trajectory
> 與三腳支撐穩定性檢查的完整 Hybrid gait，並讓機器人在 simulation
> 中處理參數化的對稱 rectangular terrain，並在不針對實驗尺寸特製規則的前提下產生合法 trajectory 或明確 infeasible 結果。**

英文工作版：

> **Integrate nominal foot-rim rolling and terrain-aware rolling/swing
> transitions into a synchronized four-leg Hybrid trajectory with
> whole-body motion and quasi-static three-leg support constraints, and
> demonstrate parameterized symmetric-terrain planning in simulation
> without experiment-size-specific logic.**

------------------------------------------------------------------------

# 26. Day 12 做完後應該看到的畫面

理想上最後不是只有一張 leg animation，而是同時有：

``` text
A. Whole-body animation
   body + 4 legs + obstacle + contact points

B. Four-leg mode timeline
   FOOT_RIM_ROLL / ROLL / SWING / TOP_REPOSITION

C. Body / CoM plot
   x(t), z(t)

D. Stability plot
   minimum support margin over time

E. Joint plot
   theta / beta / gamma for all four legs

F. Contact plot
   rim / alpha / surface over time
```

這些圖本身就會成為後面寫 paper Methods / Results 時的重要素材。

------------------------------------------------------------------------

# 27. Day 12 結束後的下一步

``` text
Day 12
FOOT_RIM_ROLL
+ terrain transition planning
+ four-leg timing
+ body trajectory
+ support feasibility
        ↓
Day 13–14
ABAD gamma adjustment
+ stability-margin improvement
+ asymmetric terrain
        ↓
Day 15–16
multi-obstacle long-route composer
+ final deterministic trajectory.csv
        ↓
simulation / hardware experiments
Wheel vs Walk vs Proposed Hybrid
```

Day 12 完成後，核心問題不應再是：

``` text
四腳怎麼接起來？
```

而應變成：

``` text
已經有可執行 whole-body Hybrid gait，
現在 ABAD 能不能讓它在 asymmetric terrain 上更穩？
```

這就是 Day 12 應該 freeze 的位置。

------------------------------------------------------------------------

# <span style="color:#188038">🟩 FINAL REVISION SUMMARY：這次最後修改了什麼？</span>

1. <span style="color:#c5221f"><b>🟥 移除：</b></span>「flat `FOOT_RIM_ROLL` 可以永遠 continuous contact、完全不需要 swing」的舊假設。

2. <span style="color:#188038"><b>🟩 新增：</b></span>flat-ground nominal locomotion 正式定義為：

```text
NOMINAL_HYBRID_CYCLE
= FOOT_RIM_ROLL + RECOVERY_SWING
```

3. <span style="color:#188038"><b>🟩 新增：</b></span>所有有 terrain contact 的 rolling stroke 保持相同 forward rolling direction。

4. <span style="color:#188038"><b>🟩 新增：</b></span>rolling stroke 結束後先 liftoff，再做 airborne recovery；airborne phase 不稱為 reverse rolling。

5. <span style="color:#188038"><b>🟩 新增：</b></span>recovery 採「縮腿 → 同 nominal rotation sense reposition → 再伸腿 → touchdown」。

6. <span style="color:#b06000"><b>🟨 MVP 決策：</b></span>`theta_compact = 17 deg`，但只能是 configurable recovery parameter，不能成為 obstacle-specific hard code。

7. <span style="color:#188038"><b>🟩 新增：</b></span>`theta_touchdown` 由下一個 `ContactState` / terrain geometry 決定，不固定等於 17 deg。

8. <span style="color:#188038"><b>🟩 Step 1 重寫：</b></span>要求至少生成兩個連續 `ROLL → RECOVERY → ROLL` cycle，並 reuse Day 8–9 swing/collision infrastructure。

9. <span style="color:#188038"><b>🟩 Flat test 修正：</b></span>flat ground 現在預期有 nominal recovery swing；不應出現的是**額外、不必要的 terrain-transition swing**。

10. <span style="color:#188038"><b>🟩 Metrics 修正：</b></span>將 `nominal recovery swing` 與 `terrain-transition swing` 分開統計，避免後續 Hybrid vs Walk 分析混淆。

11. <span style="color:#b06000"><b>🟨 Energy 敘事修正：</b></span>Day 12 不證明 Hybrid 比 Walk 省能；只輸出 trajectory-level metrics，energy / COT 留到後續 simulation / hardware comparison。

12. <span style="color:#188038"><b>🟩 保留上一版 generalization 原則：</b></span>flat、4 cm × 40 cm、10 cm × 40 cm、19 cm × 40 cm 都只是 evaluation queries；planner 不得針對這些尺寸 hard-code。
