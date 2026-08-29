# Hybrid Gait Day 8–9：不同 Terrain Height 的 Cartesian Swing Planning

> **用途**：記錄 Day 8–9 swing planner 的研究發想、與 Day 6–7 rolling feasibility 的關係、設計決策、實作順序與可逐步交給 Codex 的工作項目。之後撰寫 paper 時，可回頭追溯為什麼採用 contact-state-to-contact-state 的 Cartesian swing，而不是只做傳統 foot-tip swing。
>
> **研究範圍**：offline terrain-aware hybrid gait planning。Day 8–9 只處理 SWING motion primitive；何時選擇 ROLL 或 SWING 留到 Day 10–11。

---

## 1. Day 8–9 的核心問題

Day 8–9 不應被定義成「寫一個跨 20 mm / 40 mm 障礙物的特殊 gait」。真正要完成的是一個通用的 swing primitive：

> **給定合法的 swing 起始 contact state 與指定的 touchdown contact state，產生一條 collision-free、IK-feasible、joint-limit-feasible 的 Cartesian swing trajectory。**

因此同一套 planner 應能處理：

```text
flat -> flat
z_start = 0
z_TD    = 0

flat -> 20 mm top
z_start = 0
z_TD    = 0.02

flat -> 40 mm top
z_start = 0
z_TD    = 0.04

40 mm top -> flat
z_start = 0.04
z_TD    = 0
```

平地 swing 只是 `z_TD = 0` 的特殊情況，不需要另外一套 gait logic。

---

## 2. 為什麼使用 Cartesian Bezier，而不是沿用舊 HybridSwing

舊版 Hybrid / WLW 的 swing 主要直接在 joint space 規劃：

```text
theta(t), beta(t)
```

包含 LINEAR、CUBIC、FIVETIMES、OPTIMIZE 等策略。這種方式適合已知起終 joint configuration 的擺腿，但不容易直接指定：

- touchdown 世界座標；
- touchdown terrain height；
- swing clearance；
- obstacle clearance；
- touchdown approach direction / velocity。

新版研究已經把 foothold 與 terrain geometry 提升到 planning layer，因此 Day 8–9 改用：

```text
Cartesian contact trajectory
        ↓
IK
        ↓
theta / beta / gamma trajectory
```

第一版只需要一種 Cartesian Bezier swing，不需要同時維護很多 swing profile。

---

## 3. Walk swing 與新版 Hybrid swing 的真正差別

不要把差別寫成：

> Walk 只能用 Foot Rim 的固定一點，而 Hybrid 可以從 Foot Rim A 點 swing 到 B 點。

更準確的區分是：

### 傳統 Walk swing

比較接近：

```text
foothold A
   ↓
Cartesian swing
   ↓
foothold B
```

high-level target 主要是腳端 / foothold 的 Cartesian position。

### 新版 Hybrid swing

應提升成：

```text
ContactState A
(p_A, rim_A, alpha_A)
        ↓
      SWING
        ↓
ContactState B
(p_B, rim_B, alpha_B)
```

也就是 **contact-state-to-contact-state repositioning**。

因此以下都只是同一個 generalized swing primitive 的不同 case：

```text
FootRim(alpha_A) -> FootRim(alpha_B)
FootRim(alpha_A) -> LowerRim(alpha_B)
LowerRim(alpha_A) -> FootRim(alpha_B)
```

這個設計的重要性在於 touchdown 之後通常還要接下一段 rolling。即使兩個 touchdown 的世界座標相同，不同的 `rim / alpha / joint configuration` 可能具有不同的後續 rolling range。因此 Hybrid planner 不應只記錄 foothold position，也要保留 touchdown contact configuration。

---

## 4. ROLL 在本研究中的定義

本研究中的 `ROLL` **不等於一定要收成完整 wheel mode**。

建議 paper / code 中明確定義：

> **ROLL = continuous rim-contact motion：腳不離開 terrain，接觸點沿 leg-wheel rim 連續演化並產生 rolling displacement。**

因此下列情況都可以算 ROLL：

```text
Foot Rim 上連續 rolling
Lower Rim 上連續 rolling
Upper Rim 上連續 rolling
可行且連續的 rim transition
```

只要仍維持合法 continuous contact，而不是把腳抬起重新配置，就屬於 ROLL primitive。

因此整個 Hybrid locomotion 比較適合描述成：

```text
ROLL (continuous contact evolution)
        ↓
rolling becomes infeasible
        ↓
SWING (contact relocation)
        ↓
new touchdown ContactState
        ↓
ROLL (continuous contact evolution)
```

而不是簡化成：

```text
Wheel mode -> Walk mode -> Wheel mode
```

---

## 5. Day 6–7 與 Day 8–9 的關係

兩者不是完全獨立，而是共用底層 contact / kinematics / collision infrastructure。

### Day 6–7 問題

> 從目前 ContactState 出發，能不能在不離地的情況下繼續 continuous rolling？

```text
ContactState
    ↓
rolling propagation
    ↓
contact / collision / joint / workspace checks
    ↓
ROLL feasible ?
```

### Day 8–9 問題

> 如果決定離地重新配置，能不能從 ContactState A swing 到 ContactState B？

```text
ContactState A
    ↓
Cartesian Bezier
    ↓
rim-point IK
    ↓
collision / joint checks
    ↓
ContactState B
```

### 共同底層能力

兩邊應共用：

- `ContactState`；
- rim / alpha geometry；
- FK / IK；
- joint limits；
- terrain representation；
- full-leg collision checker。

不要讓 Day 8–9 重新實作一套 obstacle checker。

---

## 6. 什麼時候可以從 Day 6–7 同步開始 Day 8–9

不需要等 Day 6–7 完整 feasibility map 做完。

最重要的 dependency 是：

```text
rolling termination state
        ↓
swing start state
```

建議 Day 6–7 做到以下 checkpoint，就可以同步開 Day 8–9：

1. 可以從一個初始 `theta / beta` 做多 timestep rolling propagation。
2. 每個 timestep 可以取得 `rim / alpha / contact point`。
3. 可以判斷 rolling continuation 是否失敗，至少包含：
   - collision；
   - joint / workspace limit；
   - lost valid contact。
4. rolling 停止時可以輸出一個明確的 `ContactState`。
5. terrain collision checker 已經可以被其他 planner reuse。

也就是：

```text
[1] single-state contact query
        ↓
[2] continuous rolling propagation
        ↓
[3] detect infeasible continuation
        ↓
[4] output roll-end ContactState   ← 到這裡開始 Day 8–9
        ↓
[5] sweep obstacle heights / initial configurations
        ↓
[6] feasibility map / max rollable height
```

[5]、[6] 可以和 Day 8–9 平行進行。

---

# 7. Day 8–9 建議資料介面

## 7.1 Start ContactState

至少包含：

```text
position_world
rim
alpha
theta
beta
gamma (第一版可固定 0)
terrain_surface_id
```

Day 8–9 最理想的實際 start state，就是 Day 6–7 輸出的 `roll_end_contact`。

## 7.2 SwingTarget

建議至少包含：

```text
target_position_world
target_rim
target_alpha
clearance
```

`target_position_world.z` 就是 touchdown terrain height，因此不必另外建立與 target position 重複的 `target_z` state。

## 7.3 其他輸入

```text
body / hip trajectory during swing
swing_duration
terrain
robot geometry / kinematics
constraints
```

第一版可以固定：

```text
gamma = 0
```

ABAD stability 留到 Day 13–14。

---

# 8. Cartesian Bezier 第一版

不需要一開始複製非常複雜的多 control-point trajectory。第一版可以使用 quintic Bezier：

\[
P(s)=\sum_{i=0}^{5}{5\choose i}(1-s)^{5-i}s^iP_i,
\qquad s\in[0,1]
\]

其中：

```text
P0 = start position
P1 = lift-off shaping / initial derivative
P2 = upper swing region
P3 = upper swing region
P4 = touchdown approach / final derivative
P5 = target touchdown position
```

示意：

```text
                 P2 -------- P3
               /                \
             P1                  P4
            /                      \
START P0                          P5 TARGET
```

Bezier endpoint derivative：

\[
P'(0)=5(P_1-P_0)
\]

\[
P'(1)=5(P_5-P_4)
\]

因此之後可以直接透過 `P1 / P4` 控制 lift-off 與 touchdown velocity。

---

# 9. Clearance 的定義

不要定義成單純：

```text
z_apex = z_start + clearance
```

因為 target 可能比 start 高。

比較合理的第一版定義：

\[
z_{apex}
=
\max(z_{start},z_{target},z_{obstacle\ along\ path})
+h_{clear}
\]

例如：

```text
obstacle top = 40 mm
clearance    = 30 mm

z_apex = 70 mm
```

但 `z_apex` 只負責建立初始 swing curve，**不能取代完整 collision check**。

原因是 leg-wheel 不是 point foot：即使規劃的 contact point 已高於 obstacle，其他 rim、foot structure 或 linkage 仍可能撞 vertical face。

所以每個 trajectory sample 都必須：

```text
Cartesian target
    ↓
IK
    ↓
full leg configuration
    ↓
terrain collision query
```

---

# 10. Rim-point IK 是 Day 8–9 的核心能力

第一版流程：

\[
(p_i, rim_i, \alpha_i)
\rightarrow
q_i=(\theta_i,\beta_i,\gamma_i)
\]

Day 8–9 可以先限制為：

\[
\gamma=0
\]

因此先解：

\[
(p_i,rim_i,\alpha_i)
\rightarrow
(\theta_i,\beta_i)
\]

注意：不能只把現有 `inverse(pos, "G")` 當作 generalized contact IK，因為新的 target 是指定 rim / alpha 的 contact state。

數值 IK 建議使用 trajectory continuity：

\[
q_i^{init}=q_{i-1}
\]

不要每個 sample 都從固定初始 guess 解，避免：

- convergence 變差；
- solution branch 跳動；
- joint trajectory 不連續。

每個 sample 至少檢查：

\[
\|FK(q_i)-p_i\|<\epsilon_p
\]

\[
q_{min}\le q_i\le q_{max}
\]

以及合理的相鄰 joint continuity：

\[
\|q_i-q_{i-1}\|<\Delta q_{max}
\]

---

# 11. Touchdown configuration check

最後一個 sample 不能只確認 Cartesian position 到了。

應檢查：

```text
target contact position error
target rim
target alpha
joint limits
no terrain penetration
correct terrain surface
```

例如：

\[
\|p_{TD}^{actual}-p_{TD}^{target}\|<\epsilon_p
\]

\[
|\alpha_{TD}^{actual}-\alpha_{TD}^{target}|<\epsilon_\alpha
\]

最後最好重新呼叫 terrain-aware contact query，確認真正接觸的是預期 surface，而不是另一個 rim / linkage 更早碰到 obstacle。

---

# 12. Touchdown velocity check

Touchdown configuration 和 touchdown velocity 要分開處理。

第一版先要求 touchdown 不要高速撞向 terrain：

\[
|v_n^{TD}|<v_{n,max}
\]

其中 `n` 是 touchdown surface normal。

Day 8 最簡單可以先採：

```text
near-zero touchdown Cartesian velocity
```

Day 9 若時間足夠，再考慮：

```text
body-velocity-matched tangential touchdown
```

讓 touchdown 後更容易接 continuous rolling。

不需要在第一版就做複雜 trajectory optimization。

---

# 13. Day 8–9 完整資料流

```text
Start ContactState
Target SwingTarget
Body / hip trajectory
Swing duration
Terrain
        │
        ▼
Construct Cartesian Bezier
        │
        ▼
Sample p(t), v(t)
        │
        ▼
Rim-point IK
(p, rim, alpha) -> theta / beta / gamma
        │
        ▼
For every sample:
    IK residual
    joint limits
    joint continuity
    full-leg terrain collision
        │
        ▼
Final sample:
    target contact position
    target rim / alpha
    target terrain surface
    touchdown velocity
        │
        ▼
JointTrajectory / SwingResult
```

---

# 14. Day 8–9 不做什麼

為避免 scope 膨脹，這兩天先不要做：

```text
ROLL-vs-SWING selection
four-leg gait timing
support polygon
ABAD optimization
energy optimization
Bezier control-point optimization
online replanning
```

這些屬於後面的整合階段。

---

# 15. 建議實作步驟

以下步驟刻意拆細，之後可以逐項交給 Codex。

## Step 1 — Freeze swing input / output contract

### 目標

先建立 swing planner 的資料契約，不寫複雜 trajectory。

### Input

```text
ContactState start
SwingTarget target
TerrainProfile terrain
body / hip pose or trajectory
swing_duration
constraints
```

### Output

建議建立：

```text
SwingTrajectory / SwingResult
```

至少包含：

```text
time samples
Cartesian samples
joint samples
valid flag
failure reason
minimum clearance
final contact error
final alpha error
```

### 完成標準

可以建立一個合法的 flat-to-flat test case 並通過 data validation。

---

## Step 2 — Implement Cartesian quintic Bezier generator

### 任務

輸入：

```text
start position
target position
clearance
duration
sample count
```

輸出：

```text
time
position
velocity
```

### 第一版要求

- `P0 = start`；
- `P5 = target`；
- apex 根據 start / target / terrain height + clearance 決定；
- start / end velocity 可先接近 0；
- 不做 optimization。

### 測試

```text
0 -> 0 mm
0 -> 20 mm
0 -> 40 mm
40 -> 0 mm
```

### 完成標準

所有 case 使用同一個 generator，且 endpoint / duration 正確。

---

## Step 3 — Add terrain-aware apex / clearance construction

### 任務

找出 start-target corridor 內 relevant obstacle top height，建立：

\[
z_{apex}=\max(z_s,z_t,z_{obs})+h_{clear}
\]

### 注意

這只是 trajectory construction heuristic，不代表 collision-free。

### 完成標準

中間存在 obstacle 時，Bezier path 會自動提高，而不是只看 target z。

---

## Step 4 — Implement / expose generalized rim-point IK

### 任務

建立明確介面，例如：

```text
solve_contact_ik(
    desired_contact_position,
    target_rim,
    target_alpha,
    initial_guess
)
```

### 要求

- 第一版 `gamma = 0`；
- previous solution 作為 next initial guess；
- 回傳 convergence / residual；
- 不要默默 clamp 不可行解。

### 完成標準

指定數個已知 FK 產生的 `(p, rim, alpha)`，IK 可以重建對應 configuration，且 residual 在 tolerance 內。

---

## Step 5 — Convert complete Bezier path to joint trajectory

### 任務

對每個 Cartesian sample 做 IK：

```text
p[0] -> q[0]
p[1] -> q[1], initial_guess=q[0]
...
p[N] -> q[N], initial_guess=q[N-1]
```

### Check

- IK convergence；
- IK residual；
- theta / beta limits；
- joint continuity。

### 完成標準

flat-to-flat 與不同 target height case 都可以輸出連續 joint trajectory。

---

## Step 6 — Reuse Day 3–5 collision checker over swing trajectory

### 任務

每個 `q_i` 都建立完整 leg geometry，對 terrain 做 collision query。

不要只檢查 Bezier point。

### 至少檢查

```text
ground penetration
obstacle top penetration
vertical-face collision
other rim / linkage collision
```

### Output

```text
collision_free
first_collision_index
collision_type
minimum_clearance
```

### 完成標準

可以建立一條「contact point 看起來跨得過，但 leg-wheel body 會撞 vertical face」的反例，並正確判 invalid。

---

## Step 7 — Touchdown contact validation

### 任務

最後 sample 重新做 contact query / FK validation。

### 檢查

```text
position error
rim match
alpha error
terrain surface match
no penetration
joint limits
```

### 完成標準

只有真正落到指定 `ContactState` 的 trajectory 才回傳 success。

---

## Step 8 — Touchdown velocity validation

### 第一版

使用 near-zero touchdown velocity，檢查 terrain normal velocity。

### 可選擴充

若時間足夠，再加入 tangential velocity 與 body velocity matching，為 touchdown 後接 ROLL 做準備。

### 完成標準

輸出 touchdown Cartesian velocity 與 normal/tangential components，並可依 threshold 判斷 valid / invalid。

---

## Step 9 — Different-height regression tests

至少建立：

```text
Case A: flat -> flat
Case B: flat -> 20 mm
Case C: flat -> 40 mm
Case D: 40 mm -> flat
Case E: same-height touchdown, obstacle between start and target
```

每個 case 都檢查：

```text
same planner logic
endpoint correctness
IK feasibility
joint limits
collision-free
correct touchdown state
touchdown velocity
```

### Day 8–9 最終完成標準

> **同一個 `generate_swing()` 只修改 `SwingTarget`，即可產生不同 touchdown height 的 collision-free、IK-feasible contact-state-to-contact-state swing trajectory，不需要依 terrain height 切換 gait logic。**

---

# 16. 與 Day 6–7 的同步工作方式

一旦 Day 6–7 可以輸出：

```text
roll_end_contact
```

就把它直接拿來做 Step 2–8 的真實 start state。

之後兩條 branch 平行：

```text
Branch A — Rolling research
roll propagation
    ↓
height / initial-q sweep
    ↓
rolling feasibility map

Branch B — Swing development
roll_end_contact
    ↓
Bezier swing
    ↓
next ContactState
```

最後 Day 10–11 再接：

```text
current ContactState
        ↓
Can continuous rolling continue?
    ┌───┴───┐
   YES      NO
    │        │
  ROLL     SWING
    │        │
    └───┬────┘
        ↓
next ContactState
```

---

# 17. 對 paper 的潛在敘事

Day 8–9 本身不一定要宣稱成獨立 contribution。比較好的論文敘事是：

1. leg-wheel 可以利用 rim 上的 continuous rolling contact；
2. rolling feasibility 受到 terrain geometry、collision 與 kinematic limits 限制；
3. 當 continuous rolling 無法延續時，planner 使用 swing 重新配置 contact state；
4. swing target 不只包含 foothold position，也包含 `rim / alpha`，讓 touchdown configuration 可以服務下一段 rolling；
5. 因此完整 Hybrid gait 是 continuous contact evolution 與 discrete contact relocation 的組合。

可濃縮成：

\[
\boxed{
\text{ROLL: contact evolution}
\quad + \quad
\text{SWING: contact relocation}
}
\]

這比把 Hybrid 描述成單純的 wheel-mode / walking-mode switching 更符合目前研究真正使用 leg-wheel geometry 的方式。

---

# 18. 給 Codex 的共用背景文字

後續每個 Step 都可以先附上這段：

```text
This project develops an offline terrain-aware hybrid gait planner for a leg-wheel robot.
In this project, ROLL means continuous rim-contact motion; it does not require the leg to be in full wheel mode. SWING is a discrete contact relocation between two ContactStates.

A ContactState contains at least the world contact position, rim ID, contact angle alpha, and the corresponding leg configuration. The Day 8–9 task is to implement a generalized Cartesian Bezier swing from a start ContactState to a target ContactState/SwingTarget. The same planner must support flat-to-flat and different touchdown heights without changing gait logic.

Keep terrain/contact reasoning separate from the runtime motor controller. Reuse the existing terrain-aware collision checker and kinematics instead of implementing duplicate geometry logic. The first version may fix gamma=0. Do not implement ROLL-vs-SWING selection, ABAD stability optimization, or online replanning in this task.
```

---

# 19. 最重要的設計決策摘要

```text
1. Swing 是通用 primitive，不是特定障礙物高度的 gait。
2. 平地 swing = z_TD = 0 的普通 case。
3. Hybrid swing 定義成 ContactState -> ContactState。
4. Touchdown target 保留 position + rim + alpha。
5. ROLL = continuous rim-contact motion，不等於 full wheel mode。
6. Day 6–7 的 roll-end ContactState 是 Day 8–9 最自然的輸入。
7. Day 6–7 做到 rolling termination state 後即可同步開始 Day 8–9。
8. 第一版只做 Cartesian quintic Bezier。
9. gamma 第一版可固定 0。
10. 每個 Bezier sample 都做 rim-point IK、joint check 與 full-leg collision check。
11. Clearance 不能只看 foot/contact point；必須檢查完整 leg-wheel geometry。
12. Touchdown configuration 與 touchdown velocity 分開驗證。
13. Day 8–9 不負責決定何時 ROLL / SWING；Day 10–11 再整合。
```
