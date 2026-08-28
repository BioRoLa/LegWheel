# Hybrid Gait Day 6--7：單腳 Rolling-Assisted Obstacle Traversal

> **用途**：記錄 Day 6--7 單腳 rolling feasibility
> 的研究發想、策略演變、核心假設與實作步驟。\
> 一方面作為之後撰寫 paper
> 時回顧研究脈絡的筆記；另一方面可將「實作步驟」逐項交給 Codex
> 產生與修改程式。
>
> **目前研究範圍**：已知 rectangular obstacle、offline planning、單腳 2D
> sagittal-plane 測試為主；先不處理完整四足協調與 online replanning。

------------------------------------------------------------------------

## 1. Day 6--7 原始目標

Day 3--5 已建立 terrain-aware contact / collision query，能在給定 leg
configuration 與 rectangular obstacle 時，判斷：

-   ground contact
-   obstacle top contact
-   vertical-face interaction
-   invalid penetration / collision
-   no valid contact

Day 6--7 的下一步，是從「單一 configuration 的接觸判斷」進一步研究：

> **給定一個 rectangular obstacle，單腳是否可以不進入一般 walking
> swing，而是利用 leg-wheel 的 rim rolling、leg extension / retraction
> 與 rim transition 通過障礙物？**

原本規劃的輸出包括：

-   obstacle height vs. rolling feasibility
-   initial configuration vs. maximum rollable obstacle height

但在進一步討論後，發現「能不能滾上去」本身還不足以描述完整
traversal，因為腳即使成功爬上
obstacle，仍需要考慮之後如何恢復到可繼續站立 / walking 的
configuration。

因此 Day 6--7 的問題逐漸從單純的 **roll-up feasibility**，擴充成完整的
**rolling-assisted obstacle traversal primitive**。

------------------------------------------------------------------------

# 2. 最初的直覺：利用 Right Rim 直接爬上障礙物

最初想法不是讓腳避開 obstacle vertical face，也不是看到障礙物就直接
swing。

相反地，希望**主動利用 right rim 與 obstacle vertical face / corner
的接觸**。

概念如下：

``` text
                 obstacle
                    ┌────────
                    │
────────── leg ─────┘
              →
```

先根據 obstacle height 選擇一個較大的 `theta`。

由於 `theta` 控制 leg length，因此可以利用 leg extension 改變 right rim
相對 hip 的位置與可達範圍，使 right rim 有機會接觸 obstacle，並利用：

``` text
right-rim contact
+
forward rotation
+
leg extension / configuration change
```

讓 right rim 沿 obstacle vertical face / corner 往上移動。

因此這裡必須區分兩件事：

### 合法的 vertical-face contact

``` text
right rim 正常接觸 obstacle vertical face
```

這是希望利用的接觸，不應直接視為 collision。

### 非法 collision / penetration

例如：

``` text
link 穿入 obstacle
其他 rim 不合理穿入 obstacle
foot structure 與 obstacle 發生不允許的幾何干涉
```

這些才應視為 failure。

因此 Day 3--5 的 `vertical face collision` 在 Day 6--7
需要更精確地區分：

> **planned rim contact** 與 **unwanted geometry interference**。

------------------------------------------------------------------------

# 3. 為什麼只「滾上去」還不夠

一開始可以把問題想成：

``` text
right rim 成功爬上 obstacle
    → SUCCESS
```

但進一步考慮後發現，這並不是完整的 locomotion primitive。

原因是：

> right rim 成功上到 obstacle top 後，目前的 leg configuration
> 不一定適合直接重新增加 theta 站起來。

如果此時仍是較大的 theta，而且 contact rim / contact orientation
不合適，直接增加 theta 可能：

-   無法讓 foot rim 成為主要支撐點；
-   造成不理想的 rim transition；
-   讓其他結構與 obstacle 干涉；
-   無法回到正常 walking / stance configuration。

因此真正的問題變成：

> **爬上去之後，如何把 contact state
> 恢復到一個可以重新伸腿站起來的狀態？**

------------------------------------------------------------------------

# 4. `theta = 17°` 作為 Wheel-Reset Configuration

目前機構在：

``` text
theta ≈ 17°
```

時為 wheel-like configuration。

因此產生了一個重要想法：

> **爬上 obstacle 後，先逐漸把 theta 收回 17°，利用 wheel-like
> configuration 繼續 rolling，把 foot rim 轉到下方，再重新增加 theta
> 站起來。**

因此完整 motion 不再只是：

``` text
roll up
```

而是：

``` text
Extend
    ↓
Right-rim roll-up
    ↓
Retract to theta = 17°
    ↓
Wheel-like rolling
    ↓
Rotate until foot rim reaches usable bottom contact
    ↓
Re-extend theta
```

這裡 `theta = 17°` 的功能不是單純「把腿縮短」，而可以理解成一個：

> **contact-state reset configuration**

它讓 leg-wheel 暫時回到接近 wheel 的狀態，之後可以透過 rolling 把 foot
rim 重新帶到底部，再從適合的 contact configuration 重新伸腿。

------------------------------------------------------------------------

# 5. 第一版 Rolling-Assisted Primitive

目前最清楚的第一版 motion primitive 可以拆成五個 state。

## State 1 --- EXTEND / APPROACH PREPARATION

根據 obstacle height 選擇：

``` text
theta_climb
```

目標是讓 right rim 具有足夠的幾何條件接觸並爬上 obstacle。

第一版可以先固定：

-   `gamma = 0`
-   body / hip height
-   approach direction
-   obstacle geometry

只研究 `theta` 與必要的 `beta`。

------------------------------------------------------------------------

## State 2 --- RIGHT_RIM_ROLL_UP

right rim 接觸 obstacle vertical face / corner。

接著透過：

``` text
forward rotation
+
theta / beta change
```

使 right rim 沿 obstacle 往上移動。

此階段允許：

``` text
right rim ↔ vertical face
right rim ↔ obstacle corner
right rim ↔ obstacle top
```

但不允許其他結構 penetration。

成功條件：

> right rim / 合法 rim contact 成功通過 obstacle leading corner 並進入
> obstacle top region。

------------------------------------------------------------------------

## State 3 --- RETRACT_TO_WHEEL

當腳已經有足夠部分上到 obstacle top 後：

``` text
theta → 17°
```

但不是瞬間切換，而應在保持合法 contact 的情況下逐漸 retract。

需要檢查：

-   contact 是否持續有效
-   是否產生 penetration
-   theta 是否可以連續降到 17°
-   retract 過程是否仍能保持 forward / intended motion

------------------------------------------------------------------------

## State 4 --- WHEEL_RESET_ROLL

當：

``` text
theta = 17°
```

後，保持 wheel-like configuration 繼續向前 rolling。

目的不是單純前進，而是：

> **把 foot rim 旋轉到適合重新站立的位置。**

因此此階段的終止條件應該是類似：

``` text
theta == 17°
AND
foot rim reaches desired bottom orientation/contact
AND
contact surface == obstacle top
```

滿足後才允許進入 re-extension。

------------------------------------------------------------------------

## State 5 --- RE_EXTEND

當 foot rim 已位於適合承重的位置：

``` text
theta ↑
```

重新增加 leg length，使腳回到正常 stance / walking 所需要的
configuration。

成功完成 State 5，才代表完整的：

> **rolling-assisted obstacle traversal**

成功，而不只是「right rim 曾經爬上 obstacle」。

------------------------------------------------------------------------

# 6. 新發現：Obstacle Top Length 也會限制 Rolling Recovery

進一步考慮後發現，即使 right rim 可以成功爬上
obstacle，也不代表一定有足夠空間完成：

``` text
theta → 17°
→ rotate
→ foot rim to bottom
→ re-extend
```

假設 obstacle top 很短：

``` text
             ┌─────┐
             │     │
─────────────┘     └──────
```

right rim 爬上去之後，可能還沒來得及把 foot rim 轉到底部，就已經接近
obstacle trailing edge。

因此 feasibility 不只與：

``` text
obstacle height
```

有關，也與：

``` text
obstacle top length
```

有關。

定義：

``` text
L_top
```

為 obstacle top 的可用長度。

再定義：

``` text
L_reset
```

為從 roll-up 完成後，到：

``` text
theta = 17°
+
foot rim reaches usable bottom contact
```

所需要的前進距離。

則可以先做：

``` text
if L_top >= L_reset:
    full rolling recovery is possible
else:
    full rolling recovery is not possible
```

這是 offline known-terrain planner 很適合事前計算的資訊。

------------------------------------------------------------------------

# 7. 不夠長時，不應直接判定整個 motion 失敗

如果：

``` text
L_top < L_reset
```

並不代表 right-rim rolling 沒有價值。

因為 right rim 可能已經成功把腳 / body 帶上 obstacle。

這時候可以改成：

``` text
right-rim roll-up
    ↓
利用 rolling 能力盡可能前進
    ↓
無法完成 wheel reset
    ↓
切換到既有 swing motion
    ↓
重新選 touchdown / contact configuration
```

因此得到第二種 motion：

> **Rolling-Assisted Swing**

也就是先利用 rolling 完成有利的部分，真的需要離地 reposition 時才
swing。

這比一開始就 full swing 更符合整個 Hybrid Gait 的核心想法：

> **能滾的部分盡量滾，只有 rolling 無法完成剩餘 traversal / recovery
> 時才使用 swing。**

------------------------------------------------------------------------

# 8. 目前形成的三種 Obstacle Traversal Strategy

經過上述討論，目前可以把單腳 obstacle traversal 分成三種候選策略。

## Strategy A --- Full Rolling Recovery

``` text
EXTEND
→ RIGHT_RIM_ROLL_UP
→ RETRACT_TO_17
→ WHEEL_RESET_ROLL
→ FOOT_RIM_READY
→ RE_EXTEND
```

適用於：

-   obstacle height 可由 right rim 克服；
-   obstacle top 有足夠長度；
-   retract / reset 過程皆 collision-free；
-   可以重新建立 foot-rim stance。

這是最完整的 no-swing traversal。

------------------------------------------------------------------------

## Strategy B --- Rolling-Assisted Swing

``` text
EXTEND
→ RIGHT_RIM_ROLL_UP
→ continue useful rolling
→ recovery space insufficient / rolling continuation fails
→ SWING
→ target touchdown
```

適用於：

-   right rim 可以有效爬上 obstacle；
-   但 obstacle top 太短，或幾何條件無法完成 wheel reset；
-   rolling 仍然能減少 swing 所需的 displacement / height / duration。

這可能成為 Hybrid Gait 很重要的一種中間模式。

------------------------------------------------------------------------

## Strategy C --- Direct Swing

``` text
SWING
→ obstacle top / next foothold
```

適用於：

-   right rim 本身就無法有效 roll-up；
-   leg extension / rim geometry 不允許；
-   發生 unavoidable interference；
-   rolling-assisted motion沒有實際優勢。

------------------------------------------------------------------------

# 9. Planner 最後真正要做的選擇

因此未來 offline planner 不一定只是：

``` text
ROLL
vs.
SWING
```

而可以變成：

``` text
                  Known obstacle
                       ↓
          Can right rim roll up?
                 /           \
               NO             YES
               ↓               ↓
         DIRECT SWING     Can full reset
                           be completed?
                          /             \
                        YES              NO
                         ↓                ↓
                FULL ROLLING       ROLL-ASSISTED
                  RECOVERY             SWING
```

換句話說，對已知 terrain，可以事前根據：

``` text
obstacle height
obstacle top length
initial configuration
available rim/contact geometry
```

決定最合適的 motion primitive。

------------------------------------------------------------------------

# 10. Day 6--7 現階段不要一次做完整 Planner

雖然最終研究問題已經可以看到上述三種 strategy，但 Day 6--7
現階段的重點仍然應該是：

> **先證明單腳 rolling-assisted primitive
> 在幾何上存在，並建立可以系統性測試它的 simulator / feasibility
> evaluator。**

不要現在就：

-   做四腳 gait；
-   做完整 global search；
-   做 cost optimization；
-   做 ABAD；
-   一次掃所有 `(height, length, theta, beta)`；
-   重寫 swing planner。

Day 6--7 應該先把單腳問題做乾淨。

------------------------------------------------------------------------

# 11. Day 6--7 建議實作順序

以下每一個 Step 都刻意切小，可以之後逐項交給 Codex。

------------------------------------------------------------------------

## Step 1 --- 建立單一 Obstacle Test Scene

先固定一個 rectangular obstacle，例如：

``` text
height = 20 or 30 mm
top length = 足夠長
gamma = 0
```

建立單腳 2D visualization：

-   leg geometry
-   ground
-   obstacle vertical face
-   obstacle top
-   active contact rim
-   contact point
-   penetration / collision points

### 完成標準

給定：

``` text
theta
beta
hip pose
```

可以清楚畫出目前腳與 obstacle 的幾何關係。

------------------------------------------------------------------------

## Step 2 --- 區分 Planned Contact 與 Invalid Collision

修改 / 擴充 Day 3--5 contact query。

不能再把所有 vertical-face interaction 都視為 failure。

至少需要分辨：

``` text
VALID_RIGHT_RIM_FACE_CONTACT
VALID_TOP_CONTACT
INVALID_LINK_COLLISION
INVALID_OTHER_GEOMETRY_PENETRATION
NO_CONTACT
```

### 完成標準

right rim 正常碰 obstacle vertical face 時：

``` text
valid contact = true
collision = false
```

但其他不允許的結構穿入 obstacle 時：

``` text
collision = true
```

------------------------------------------------------------------------

## Step 3 --- 固定 Theta，測試 Right-Rim Roll-Up

先不要自動選 theta。

手動給：

``` text
theta_climb = fixed
```

例如選一個較大的 theta。

讓 motion 從 obstacle 前方開始，逐步改變 `beta` / forward
configuration，檢查 right rim 是否可以：

``` text
ground
→ vertical face
→ corner
→ obstacle top
```

### 完成標準

至少找到一組：

``` text
obstacle height
theta_climb
initial beta / approach configuration
```

可以讓 right rim 成功從 obstacle 前方移動到 obstacle top。

此時先不管 reset。

------------------------------------------------------------------------

## Step 4 --- 加入 Theta Extension / Configuration Search

確認 Step 3 成功後，再讓程式針對 obstacle height 搜尋合適的：

``` text
theta_climb
```

第一版不需要 optimization。

可以直接 sweep：

``` text
theta = theta_min : dtheta : theta_max
```

對每一個 theta 測試 roll-up 是否成功。

記錄：

``` text
minimum feasible theta
feasible theta range
failure reason
```

### 完成標準

對固定 obstacle height 可以回答：

> 哪些 theta 可以讓 right rim roll-up？

------------------------------------------------------------------------

## Step 5 --- 實作 Retract to 17°

當 right rim 已經進入 obstacle top region 後：

``` text
theta_current → 17°
```

逐步降低 theta。

每一步都呼叫 terrain-aware contact / collision query。

### 檢查

-   contact 是否仍合法
-   是否 penetration
-   是否能連續 retract
-   contact rim 如何變化

### 完成標準

至少找到一個 case 可以：

``` text
roll-up success
→ theta continuously retracts to 17°
```

------------------------------------------------------------------------

## Step 6 --- 計算 Foot-Rim Reset 所需 Rotation

在：

``` text
theta = 17°
```

時，繼續 rolling。

追蹤：

``` text
beta / rotation
rim
alpha
contact point
```

直到 foot rim 到達適合重新伸腿的 bottom contact。

記錄：

``` text
required rotation angle
required forward rolling distance
```

定義：

``` text
L_reset
```

### 完成標準

對一個 roll-up end state，可以回答：

> 還需要往前滾多少距離，foot rim 才能回到底部？

------------------------------------------------------------------------

## Step 7 --- 實作 Re-Extension

當：

``` text
theta = 17°
foot rim ready
contact surface = obstacle top
```

逐步增加 theta。

檢查：

-   foot rim 是否保持有效支撐
-   是否有 collision
-   是否能回到指定 stance leg length

### 完成標準

完成一整條：

``` text
EXTEND
→ RIGHT_RIM_ROLL_UP
→ RETRACT
→ WHEEL_RESET
→ RE_EXTEND
```

的 trajectory。

這是 Day 6--7 最重要的第一個 milestone。

------------------------------------------------------------------------

## Step 8 --- 建立 Full Rolling Recovery Feasibility Function

將前面的流程包成類似：

``` python
check_full_rolling_recovery(
    obstacle,
    initial_state,
    theta_climb,
) -> RollingResult
```

`RollingResult` 建議至少記錄：

``` text
feasible
failure_stage
failure_reason

theta_climb
trajectory

roll_up_success
retract_success
reset_success
reextend_success

required_reset_distance
minimum_collision_margin
```

### Failure stage 建議

``` text
ROLL_UP
RETRACT
WHEEL_RESET
RE_EXTEND
```

這對後續分析與 paper 很重要。

------------------------------------------------------------------------

## Step 9 --- Sweep Obstacle Height × Theta

等單一完整 trajectory 成功後才開始 sweep。

例如：

``` text
obstacle height:
5, 10, 15, 20, ... mm

theta_climb:
20°, 25°, 30°, ...
```

產生 feasibility map：

``` text
              obstacle height
theta       10  20  30  40  50
--------------------------------
30°          ✓   ✓   ✗   ✗   ✗
40°          ✓   ✓   ✓   ✗   ✗
50°          ✓   ✓   ✓   ✓   ✗
...
```

並可進一步得到：

``` text
obstacle height
→ minimum required theta
```

或：

``` text
initial / climb theta
→ maximum traversable obstacle height
```

------------------------------------------------------------------------

## Step 10 --- 加入 Obstacle Top Length

當 full rolling recovery 已經可以運作後，再改變：

``` text
L_top
```

比較：

``` text
L_top
vs.
L_reset
```

建立：

``` text
if L_top >= required recovery distance:
    FULL_ROLLING_RECOVERY
else:
    FULL_ROLLING_RECOVERY_NOT_POSSIBLE
```

注意：

> 這裡不能只用簡單長度公式取代 simulation / geometry validation。

因為 retract 過程本身也會消耗 forward distance，且不同 theta / contact
state 可能造成不同的 rolling displacement。

------------------------------------------------------------------------

## Step 11 --- 加入 Rolling-Assisted Swing Fallback

如果：

``` text
roll_up_success == true
```

但是：

``` text
full_recovery_success == false
```

則保留目前 rolling 所到達的最後合法 state。

把這個 state 作為：

``` text
swing start state
```

接到既有 swing planner。

比較：

``` text
ROLL_UP → SWING
```

與：

``` text
DIRECT_SWING
```

的差異。

第一版只要求 trajectory 可以接起來，不需要立刻做 energy optimization。

------------------------------------------------------------------------

## Step 12 --- 最後建立三類 Motion Classification

對每個 obstacle case 輸出：

``` text
FULL_ROLLING_RECOVERY
ROLLING_ASSISTED_SWING
DIRECT_SWING
```

之後 Day 10--11 的 offline planner 就可以直接使用這個分類結果。

------------------------------------------------------------------------

# 12. Day 6--7 最低完成標準

如果時間只有兩天，不需要強迫把 Step 1--12 全部做完。

最低應完成：

### Must Have

1.  right rim vertical-face contact 被正確視為合法 contact。
2.  找到至少一條 right-rim roll-up 成功 trajectory。
3.  可以從 roll-up end state 嘗試 retract 到 `theta = 17°`。
4.  可以量出 / 模擬 wheel-reset 到 foot rim 所需的 rotation / distance。
5.  至少一個 case 完成： `roll-up → retract → reset → re-extend`。
6.  儲存完整 trajectory 與每階段 contact state。

### Strongly Preferred

7.  掃 obstacle height × theta。
8.  得到 minimum required theta 或 maximum rollable height。
9.  測試不同 obstacle top length 對 full recovery 的影響。

### 可以留到後面

10. rolling-assisted swing 與舊 swing planner 的完整整合。
11. energy / time cost comparison。
12. 四腳 coordination。
13. ABAD stability adjustment。

------------------------------------------------------------------------

# 13. 建議儲存的資料

不要只存：

``` text
feasible = true / false
```

每一次 simulation 建議至少保存：

``` text
time / step
phase

hip_x
hip_z

theta
beta
gamma

active_rim
alpha

contact_x
contact_z
terrain_surface

valid_contact
collision

obstacle_height
obstacle_top_length

failure_reason
```

另外對整條 trajectory 保存 summary：

``` text
roll_up_success
retract_success
reset_success
reextend_success

theta_climb
required_reset_rotation
required_reset_distance

final_motion_class
```

這些資料之後畫 figure、分析 failure mode、寫 paper 都會很有用。

------------------------------------------------------------------------

# 14. Paper 發想脈絡備忘

這段特別保留給之後寫 paper 時回顧。

研究一開始的直覺是：

> rectangular obstacle 並不一定需要 walking swing。由於 leg-wheel 的 rim
> 本身可以形成移動接觸點，而且 theta 可以改變 leg
> length，因此可以主動利用 obstacle vertical face / corner 作為 rolling
> contact surface。

接著發現：

> 「right rim 能爬上 obstacle」並不等於「完整 traversal feasible」。

因為爬上去後還需要恢復到適合後續 stance / walking 的 contact
configuration。

因此引入：

> `theta = 17°` wheel-like configuration 作為 contact-state reset
> configuration。

透過：

``` text
right-rim climb
→ retract to wheel state
→ wheel rolling
→ foot-rim recovery
→ re-extension
```

完成 no-swing obstacle traversal。

進一步又發現：

> 完成 recovery 所需的 rolling distance 會受到 obstacle top length
> 限制。

因此 terrain geometry
不只決定「能不能爬上去」，也決定「爬上去後能不能完成 contact
recovery」。

最後形成三種 terrain-aware motion strategy：

``` text
Full Rolling Recovery
Rolling-Assisted Swing
Direct Swing
```

這讓 Hybrid Gait 的 planning 問題從單純：

``` text
ROLL vs. WALK
```

變成更具體的：

> **Given known obstacle geometry and the current leg configuration, how
> much of the traversal can be completed through continuous rolling
> contact before a swing repositioning becomes necessary?**

這個問題與研究的核心 motivation 一致：

> **優先利用 rolling contact 與可變 leg geometry；只有當 rolling
> 無法完成幾何 traversal 或 contact recovery 時，才引入 swing。**

------------------------------------------------------------------------

# 15. Day 6--7 最後希望得到的 Research Figures

後續可以考慮整理成以下結果。

### Figure A --- Rolling-Assisted Primitive Sequence

顯示：

``` text
EXTEND
→ RIGHT_RIM_ROLL_UP
→ RETRACT
→ WHEEL_RESET
→ RE_EXTEND
```

每個階段畫 leg geometry + obstacle + contact point。

### Figure B --- Obstacle Height × Theta Feasibility Map

顯示哪些：

``` text
(height, theta_climb)
```

組合可以完成 full rolling recovery。

### Figure C --- Required Reset Distance

例如：

``` text
obstacle height / climb configuration
→ required reset distance
```

用來說明為什麼 obstacle top length 會影響 motion selection。

### Figure D --- Terrain Geometry → Motion Class

例如在：

``` text
obstacle height × obstacle top length
```

平面上標示：

``` text
FULL ROLLING
ROLLING-ASSISTED SWING
DIRECT SWING
```

如果結果夠清楚，這張圖可能會成為後續 paper 很重要的 planner motivation /
result figure。

------------------------------------------------------------------------

# 16. 下一個最實際的 Coding Task

目前**不要直接叫 Codex 寫完整 sweep 或完整三策略 planner**。

下一個最小 task 應該是：

> 建立一個固定 rectangular obstacle、固定 `theta_climb` 的單腳 2D
> test，允許 right rim 與 obstacle vertical face / corner 成為合法
> contact，逐步改變 leg configuration，驗證 right rim 是否能從
> ground-side approach 連續移動到 obstacle top，同時拒絕其他 leg
> geometry 對 obstacle 的 penetration。每一步輸出 theta、beta、active
> rim、contact point、terrain surface 與 collision state，並提供逐 frame
> visualization。

等這個 **RIGHT_RIM_ROLL_UP** 單一 case 確認正確，再依序進入：

``` text
Retract to 17°
→ Wheel reset
→ Foot-rim detection
→ Re-extension
→ Height/theta sweep
→ Top-length feasibility
→ Rolling-assisted swing fallback
```

這樣每一步都可以獨立 debug，也最適合逐項交給 Codex 實作。

---

# 17. 每一步可以直接給 Codex 的指令

以下指令刻意寫成「一次只做一件事」的形式。建議不要一次把全部丟給 Codex，而是每完成一步、確認結果正確後，再進到下一步。

使用前可以先把這份筆記提供給 Codex，並告訴它：

> 請先閱讀這份 Day 6–7 規劃筆記，理解目前研究目標與既有架構。不要重新設計整個專案，也不要把 terrain reasoning 塞進 runtime gait controller。Day 6–7 目前只處理單腳、2D rectangular obstacle、offline rolling feasibility。優先 reuse 現有 leg kinematics、contact query 與 visualization。每次只修改本步驟需要的最小範圍，並說明修改了哪些檔案。

---

## Codex Step 1 — 建立單一 rectangular obstacle 測試場景

### 可以直接給 Codex 的指令

```text
請先建立 Day 6–7 的單腳 rolling feasibility 測試場景。

目標：
建立一個最小的 2D single-leg + rectangular obstacle test，不做 sweep、不做 planner，只要能固定一組 theta、beta、hip pose，畫出 leg geometry 與 obstacle 的相對位置。

需求：
1. Terrain 先只支援：
   - flat ground
   - single rectangular obstacle
2. 第一版固定 gamma = 0。
3. obstacle 參數至少包含：
   - x_start
   - width / top length
   - height
4. visualization 至少畫出：
   - ground
   - obstacle vertical face
   - obstacle top
   - leg-wheel geometry
   - hip position
   - 各 rim
5. 輸入一組 theta、beta、hip_x、hip_z 後，可以產生一張圖。
6. 請優先 reuse 現有的 leg kinematics / geometry code，不要重新寫一套 leg model。
7. 不要現在做 rolling motion、search、FSM 或 sweep。

完成後請告訴我：
- 新增或修改哪些檔案
- 執行方式
- 哪些參數可以調
- 目前使用的座標定義
```

### 驗收重點

```text
我可以手動改：
theta
beta
hip_x
hip_z
obstacle height
obstacle width

然後看到正確的 leg + obstacle 圖。
```

---

## Codex Step 2 — 區分合法 Right-Rim Contact 與非法 Collision

### 可以直接給 Codex 的指令

```text
請在目前 Day 6–7 single-leg rectangular obstacle test 上，擴充 terrain-aware contact / collision query。

研究需求：
right rim 接觸 obstacle vertical face 是這個研究刻意要利用的 planned contact，不能把所有 vertical-face interaction 都直接視為 collision。

請至少區分以下狀態：
1. VALID_RIGHT_RIM_FACE_CONTACT
2. VALID_RIGHT_RIM_TOP_CONTACT
3. VALID_OTHER_RIM_CONTACT
4. INVALID_LINK_COLLISION
5. INVALID_GEOMETRY_PENETRATION
6. NO_CONTACT

要求：
- right rim 正常碰 obstacle vertical face 時：
  valid_contact = true
  collision = false
- 但如果 leg link、其他不允許的 rim、foot structure 穿進 obstacle：
  collision = true
- 請保留 contact point、rim id、terrain surface id、gap / penetration depth 等資訊。
- obstacle surface 最少要能分：
  ground
  obstacle_front_face
  obstacle_top
  obstacle_back_face（如果目前架構容易支援）
- 請不要更改整個 contact representation，只在現有 queryContact / contact candidate 架構上做最小擴充。

另外請在 visualization 上：
- 標出 active / valid contact point
- 如果有 invalid collision，標出 collision point 或 collision geometry

最後請提供幾組測試：
A. right rim 還沒碰牆
B. right rim 正好碰 vertical face
C. right rim 接觸 obstacle top
D. link 明顯穿入 obstacle
```

### 驗收重點

```text
right rim 碰牆 != collision
其他結構 penetration = collision
```

---

## Codex Step 3 — 固定 Theta，測試 Right-Rim Roll-Up

### 可以直接給 Codex 的指令

```text
現在請只做一個固定 theta_climb 的 right-rim roll-up 測試。

目標：
不自動搜尋 theta，不做完整 recovery。
先確認在一個固定 obstacle height 與固定 theta_climb 下，是否可以透過改變 beta / forward configuration，讓 right rim 從 obstacle 前方移動到 obstacle top。

設定：
- single leg
- 2D
- gamma = 0
- 固定 obstacle height
- 固定 obstacle top length，先設很長，避免 recovery space 影響
- 固定 theta_climb
- hip height 先固定
- approach direction 為 +x

希望的 contact sequence 概念：
ground / right rim approach
→ right rim contact obstacle front vertical face
→ right rim near / across leading corner
→ right rim contact obstacle top

實作要求：
1. 從一個手動指定的 initial beta 開始。
2. 逐步搜尋或更新 beta，使 motion 朝 forward / roll-up 方向發展。
3. 每一步都呼叫現有 terrain-aware contact / collision query。
4. 只接受：
   - 合法 right-rim contact
   - 合法 rim transition
   - 無 penetration
5. 每一步記錄：
   - step
   - theta
   - beta
   - active rim
   - alpha
   - contact point
   - terrain surface
   - collision flag
6. 若失敗，記錄 failure reason。
7. 請提供逐 frame visualization 或可播放的 trajectory plot。

注意：
- 這一步先不要自動改 theta。
- 不要做 retract 到 17 度。
- 不要做 swing。
- 不要做 obstacle height sweep。
```

### 驗收重點

```text
至少找到一組 fixed theta_climb，
讓 right rim 能從 obstacle 前方成功到 obstacle top。
```

---

## Codex Step 4 — 搜尋可行的 Theta_climb

### 可以直接給 Codex 的指令

```text
請在已經能跑通 fixed-theta right-rim roll-up 的基礎上，加入 theta_climb sweep。

目標：
對固定 obstacle geometry 與固定 initial approach condition，測試哪些 theta_climb 可以成功完成 right-rim roll-up。

請：
1. 定義 theta sweep：
   theta_min 到 theta_max
   使用可設定的 dtheta
2. 對每一個 theta_climb：
   - 建立相同 initial condition
   - 執行目前的 right-rim roll-up test
   - 記錄 success / failure
3. 輸出至少：
   - theta_climb
   - roll_up_success
   - failure_reason
   - final beta
   - final rim
   - final contact surface
4. 找出：
   - minimum feasible theta
   - maximum feasible theta（如果有）
   - feasible theta range
5. 將結果輸出成 CSV。
6. 畫一張簡單圖：
   theta_climb vs roll-up feasibility

注意：
- 這一步只判斷 roll-up，不判斷 retract / reset / re-extension。
- 不要改變 obstacle height。
- 不要做 optimization；brute-force sweep 即可。
```

### 驗收重點

```text
固定 obstacle height 時，
我可以知道哪些 theta 能成功 roll-up。
```

---
## Codex step4.5 - 確定可以滾之後往前滾一段
```text
請在 theta candidate search 之後加入真正的 forward rolling simulation。

目前 Step 4 只證明固定 hip pose 下，某些 theta 有可能形成 right-rim climbing contact；
現在要讓 hip 在 +x 方向逐步前進，確認這些 candidate theta 是否真的能完成 continuous roll-up。

每個 simulation step：
1. hip_x += dx
2. 以上一個 theta / beta 為 initial guess
3. 搜尋新的 theta / beta，使 right rim 保持合法 terrain contact
4. 檢查 vertical-face / corner / top contact
5. 排除 link / geometry penetration
6. 要求 trajectory 在 theta / beta 上連續
7. 持續到 contact surface 成為 obstacle_top 且 hip 已跨過 leading edge

只有完整做到 obstacle top，才把該 theta 標成 true roll-up feasible。
```

```text
可以，這段我建議規劃成「從可行姿態進入，經由連續接觸滾上 obstacle top，再交給 retract/reset」的局部 motion primitive。

先確認邊界：

- `corgi_ros2_ws-dev` 只作為概念參考。
- 不會從那裡 import、複製程式、讀取 runtime 資料。
- 需要的資料結構與測試資料都放在 LegWheel 內。
- 這一輪只做規劃，尚未修改程式或舊 ROS2 資料夾。

## 核心架構

目前 Step 4 得到的是：

> 固定 hip pose 下，哪些 `theta_climb` 可以形成合法 right-rim contact。

接下來需要再分兩層：

### 1. Roll-up entry candidate

輸入：

- candidate `theta_climb`
- initial `beta`
- `hip_x`, `hip_z`
- obstacle geometry
- gamma = 0

輸出一個明確的起始姿態：

- theta / beta
- hip pose
- active rim
- contact point
- terrain surface
- contact alpha
- leading-edge clearance

這個姿態代表「可以開始滾」，但還不代表一定能完整滾上去。

### 2. Continuous roll-up continuation

從 entry pose 開始，每一步：

1. hip 往 +x 前進。
    
2. 使用上一幀 theta / beta 當作下一幀初始值。
    
3. 保持 right rim 的合法 contact。
    
4. 允許 theta / beta 做小幅連續調整。
    
5. 檢查：
    
    - vertical face contact
    - leading corner transition
    - obstacle top contact
    - link collision
    - geometry penetration
    - rim 是否非法切換

這裡不是重新做全域 search，而是沿著上一個合法姿態做 local continuation。

## 建議的 contact phase

先建立 LegWheel 自己的簡單 phase，不使用舊 ROS2 的 FSM：

```
APPROACH
  ↓
FRONT_FACE_CONTACT
  ↓
LEADING_CORNER_TRANSITION
  ↓
TOP_CONTACT
  ↓
TOP_ROLL_DISTANCE
  ↓
READY_FOR_RETRACT
```

這只是 single-leg rolling primitive 的 phase，不是整個機器人的 walking FSM。

## Top roll 距離怎麼定

這裡我建議把兩種距離分開記錄：

```
hip_progress_m
top_contact_progress_m
```

每一步仍然由 `hip_x += dx` 推進，但使用者設定的：

```
top_roll_distance_m = 0.10
```

應該主要代表：

> right rim 接觸點沿著 obstacle top 往前移動 0.10 m。

原因是 hip 往前移動，不一定代表輪子真的在 top 上滾了同樣距離。

因此建議：

```
hip_x 是控制變數
top contact point 的 x 位移是滾動進度
```

完成條件：

```
right rim 持續合法接觸 obstacle_top
且沒有 collision / penetration
且 top_contact_progress_m >= top_roll_distance_m
且離 leading edge 已經大於安全距離
```

這樣才能真正表達：

> wheel 已經穩定滾上 top 一段距離，可以開始 retract / reset。

目前 Step 4.5 的 prototype 暫時是用 hip progress 判斷 top roll；下一階段我建議把 user-facing 的 `top_roll_distance_m` 改成 top contact progress，同時保留 hip progress 作為診斷資料。

## 從舊 stair controller 借鑑的概念

舊資料中值得參考的是概念，不是程式碼：

|舊概念|LegWheel 自己的對應|
|---|---|
|contact edge / contact alpha|`ContactAnchor2D`|
|維持 edge contact 的 local solve|theta / beta continuation|
|CoM 或 hip 往前移動|`ForwardRollingStep2D`|
|wheel mode|`TOP_CONTACT` / `TOP_ROLL` phase|
|foothold / edge margin|leading-edge clearance 與 top-roll distance|

不需要把 quadruped 的 foothold planner、完整 gait state machine 或 stair controller 搬過來。

## LegWheel 內部資料建議

先繼續沿用目前的：

```
single_leg_rolling_scene_2d.py
hybrid_gait_day6_7_progress_dashboard.ipynb
```

暫時不新增很多 Python 檔案，只在現有 module 裡增加少量 dataclass，例如：

```
RollUpEntry2D
RollingContinuationFrame2D
RollingContinuationResult2D
```

每一幀記錄：

- step
- phase
- hip pose
- theta / beta
- active rim
- alpha
- contact point
- terrain surface
- leading-edge clearance
- top contact progress
- collision / penetration
- accepted
- failure reason

## 建議實作順序

### Phase A：先修正 distance semantics

- 保留 hip forward motion。
- 新增 top contact point 的實際前進距離。
- 動畫上同時顯示兩者。

### Phase B：加入 contact continuation

- front face 上維持 right-rim contact。
- corner 附近追蹤同一個 rim。
- 切換到 top 時保留連續性。
- 每一步仍通過現有 terrain-aware query。

### Phase C：加入 top roll

- 使用者指定 `top_roll_distance_m`。
- 例如 0.02、0.05、0.10 m。
- 完成前不進入 retract/reset。
- 動畫中顯示 top-roll progress。

### Phase D：再接 retract/reset

只有收到：

```
READY_FOR_RETRACT
```

之後，才開始規劃下一段腿部轉換。

## 需要特別保留的研究界線

這個階段的「穩定」先定義成幾何穩定：

- right rim 合法接觸 top
- 沒有 link collision
- 沒有 geometry penetration
- 已離開 leading edge 危險區域
- 已完成指定 top rolling distance

它還不是完整的力學穩定性。之後若要確認真的能承受機身重量，還需要加入：

- normal force
- friction constraint
- torque feasibility
- CoM / support condition
- wheel-ground no-slip relation

目前先完成幾何連續滾動是合理的第一步。
```

### ==代完成==：滾到哪要開始收腳
對，目前的停止距離是手動輸入：

```
top_roll_distance_m = 0.10
```

程式只是確保實際 no-slip 累積弧長達到這個值。

之後我建議改成「自動找最早可以安全收腳的位置」，而不是固定 0.10 m。

判斷方式可以是：

```
right rim 進入 top
→ 每完成一個 no-slip sample advance
→ 執行 READY_TO_RETRACT guard
→ 第一次連續 K 幀通過時停止 rolling
```

也就是：

```
stop_rolling = (
    right_rim_on_top
    and leading_edge_clearance_sufficient
    and stable_contact_for_k_frames
    and no_slip_contact
    and no_collision
    and retract_preview_success
)
```

我建議分三層判斷。

1. 基本安全距離

先避免剛跨過 corner 就馬上收腳：

```
top_roll_progress_m >= minimum_clearance_m
```

例如先設：

```
minimum_clearance_m = max(
    2 * rim_sample_arc_length,
    2 * contact_tolerance,
    geometry_safety_margin,
)
```

這只是最低門檻，不代表一定可以收腳。

2. Contact 穩定條件

最近 3～5 幀必須：

- 都是 `right_rim + obstacle_top`
- `collision=False`
- `alpha` 沿 rolling 方向單調變化
- contact advance 等於 rim arc advance
- 沒有其他 rim、foot 或 link 接近 penetration
- `theta、beta、hip` 沒有突然跳動

這可以排除剛經過 leading corner、contact 還不穩定的狀態。

3. Retract feasibility

最重要的判斷是：從目前位置開始收腳是否真的可行。

未來可以在每個 top-roll frame 執行：

```
目前姿態
→ theta - 1°
→ theta - 2°
→ ...
→ theta = 17°
```

每個 preview step 都重新解 `beta / hip pose` 並檢查：

- right rim 保持在 obstacle top
- 無 geometry/link penetration
- 姿態連續
- contact 不掉回 front face
- 不超過 obstacle back edge

如果完整 preview 到 17° 都成功，這個位置才是真正的 retract-feasible position。

因此最終可以定義：

```
automatic_stop_distance =
    最早通過完整 retract preview 的 top-roll distance
```

例如結果可能是：

```
0.02 m：preview 失敗，link 會碰 leading edge
0.04 m：preview 失敗，contact transition 不連續
0.06 m：完整 retract preview 成功
```

那麼就自動在約 0.06 m 停止，不必事先猜 0.10 m。

實作上我建議仍保留兩個人工界線：

```
minimum_top_roll_distance_m = 0.02
maximum_top_roll_distance_m = 0.15
```

- 小於 minimum：不執行 preview，避免太靠近 leading edge。
- minimum 到 maximum：每個 rolling frame 自動測試 retract feasibility。
- 第一次連續 3 幀 preview 成功：停止 rolling。
- 到 maximum 仍不成功：回報 `NO_RETRACT_ENTRY_FOUND`。

所以未來最合理的停止標準不是「固定滾 0.10 m」，而是：

> 在安全距離範圍內，找到最早能連續、無碰撞地完成 retract-to-17° 的位置。

目前 Phase D 已經有短期 5° preview。下一步可以先把它改成「rolling 過程中逐幀觸發」，之後再把 preview 從 5°擴充成完整試算到 17°。
## Codex Step 5 — Roll-Up 後 Retract 到 17°

### 可以直接給 Codex 的指令

```text
請在一條已經成功 right-rim roll-up 到 obstacle top 的 trajectory 後面，加入 RETRACT_TO_WHEEL 階段。

目標：
從 roll-up end state 開始，在保持合法 terrain contact 的前提下，將 theta 逐步降低到 17 degrees。

要求：
1. 起點直接使用 roll-up 成功後的：
   theta
   beta
   hip pose
   active rim
   contact state
2. theta 逐步往 17 deg 下降。
3. 每個 theta step 允許對 beta 做局部調整，使 contact 保持可行。
4. 每一步都檢查：
   - valid contact
   - active rim / alpha
   - terrain surface
   - collision / penetration
   - joint limits
5. 若某一步無法繼續 retract，停止並記錄：
   - failure_theta
   - failure_beta
   - failure_reason
6. 成功條件：
   theta reaches 17 deg
   AND no invalid collision
   AND contact remains valid through the whole retract segment
7. 保存 retract trajectory，並接在原本 roll-up trajectory 後面做 visualization。

注意：
- 這一步先不要繼續 wheel rolling。
- 不要做 foot-rim reset。
- 不要做 re-extension。
```

### 驗收重點

```text
roll-up 之後，theta 可以連續降到 17°，
而不是直接跳到 17°。
```

---

## Codex Step 6 — 計算 Foot-Rim Reset 所需 Rotation / Distance

### 可以直接給 Codex 的指令

```text
請從成功 retract 到 theta = 17 degrees 的 state 開始，實作 WHEEL_RESET_ROLL。

研究目的：
theta = 17 deg 時，讓 leg-wheel 保持 wheel-like configuration 繼續向前 rolling，直到 foot rim 轉到適合重新伸腿的 bottom contact。

要求：
1. theta 固定為 17 deg。
2. 逐步改變 beta / rolling configuration。
3. 每一步記錄：
   - beta
   - active rim
   - alpha
   - contact point
   - terrain surface
   - hip / contact forward displacement
4. 定義 foot-rim-ready condition。
   第一版可以使用：
   - active contact rim == foot rim
   - contact surface == obstacle top
   - contact orientation / alpha 落在指定 tolerance
5. 找到 foot-rim-ready state 後停止。
6. 計算並輸出：
   - required_reset_rotation
   - required_reset_forward_distance
7. 將 required_reset_forward_distance 定義為 L_reset。
8. 如果在可設定的最大 rotation / distance 內找不到 foot-rim-ready state，回傳 reset failure。
9. visualization 請標出 reset start 與 foot-rim-ready frame。

注意：
- 不要做 re-extension。
- 先假設 obstacle top 足夠長。
```

### 驗收重點

```text
給定 roll-up + retract 後的 state，
程式可以回答：
「還要往前滾多少，foot rim 才會回到底下？」
```

---

## Codex Step 7 — Foot Rim Ready 後 Re-Extend

### 可以直接給 Codex 的指令

```text
請在 foot-rim-ready state 後加入 RE_EXTEND 階段。

目標：
當 foot rim 已經在 obstacle top 形成適合的 bottom contact 時，逐步增加 theta，重新建立 stance leg length。

要求：
1. 起點必須滿足：
   theta == 17 deg
   foot rim ready == true
   contact surface == obstacle top
2. theta 從 17 deg 逐步增加到指定 target_stance_theta。
3. 每一步允許 beta 做局部修正，以保持合法 foot-rim / terrain support。
4. 每一步檢查：
   - valid contact
   - active rim
   - alpha
   - obstacle collision
   - joint limits
5. 成功條件：
   theta reaches target_stance_theta
   AND contact remains valid
   AND no invalid penetration
6. 失敗時記錄：
   failure_theta
   failure_beta
   failure_reason
7. 把 trajectory 接在：
   roll-up
   → retract
   → wheel reset
   後面。

最後請提供一個完整 visualization，顯示：
EXTEND / APPROACH
→ RIGHT_RIM_ROLL_UP
→ RETRACT_TO_17
→ WHEEL_RESET
→ RE_EXTEND
```

### 驗收重點

```text
完成第一條完整 no-swing obstacle traversal trajectory。
```

---

## Codex Step 8 — 包成 Full Rolling Recovery Feasibility Function

### 可以直接給 Codex 的指令

```text
目前各階段已經可以個別執行，請把它們整理成一個 reusable full rolling recovery feasibility function。

希望有類似介面：

check_full_rolling_recovery(
    obstacle,
    initial_state,
    theta_climb,
    constraints
) -> RollingResult

RollingResult 至少包含：
- feasible
- failure_stage
- failure_reason

- roll_up_success
- retract_success
- reset_success
- reextend_success

- theta_climb
- required_reset_rotation
- required_reset_distance

- trajectory
- minimum_collision_margin

failure_stage 至少分：
- ROLL_UP
- RETRACT
- WHEEL_RESET
- RE_EXTEND

要求：
1. 不要複製四套邏輯，請 reuse 前面已建立的 stage functions。
2. 每個 stage 都保留自己的 debug output。
3. full function 只負責依序執行與整合結果。
4. 成功條件必須是四個 stage 全部成功，不是只要 roll-up 成功。
5. 加入 unit / regression tests，至少包含：
   - 一個完整成功 case
   - 一個 roll-up failure case
   - 一個 retract or reset failure case

請不要做 obstacle sweep，這一步只做 reusable API。
```

### 驗收重點

```text
一個 function 就能回答：
「這個 obstacle + theta_climb 能不能完成完整 rolling recovery？」
```

---

## Codex Step 9 — Sweep Obstacle Height × Theta

### 可以直接給 Codex 的指令

```text
請使用現有 check_full_rolling_recovery() 建立 Day 6–7 第一個正式 feasibility sweep。

掃描：
1. obstacle height
2. theta_climb

其餘條件先固定：
- obstacle top length 設很長
- initial hip height 固定
- initial approach condition 固定
- gamma = 0

要求：
1. 對每組 (obstacle_height, theta_climb) 執行 full rolling recovery。
2. 輸出 CSV，至少包含：
   - obstacle_height
   - theta_climb
   - feasible
   - failure_stage
   - failure_reason
   - required_reset_distance
3. 產生 2D feasibility map：
   x = obstacle height
   y = theta_climb
   成功 / 失敗清楚標示
4. 對每個 obstacle height，額外計算：
   - minimum feasible theta_climb
   - maximum feasible theta_climb（若有）
5. 額外輸出：
   obstacle height vs minimum required theta
6. 請保留所有 raw result，不要只存圖。

注意：
- 目前仍不要改 obstacle top length。
- 不要加入 swing。
```

### 驗收重點

```text
得到：
obstacle height × theta_climb feasibility map
以及 minimum required theta。
```

---

## Codex Step 10 — 加入 Obstacle Top Length Feasibility

### 可以直接給 Codex 的指令

```text
請在 full rolling recovery feasibility 上加入 obstacle top length 的影響。

研究問題：
即使 right rim 可以爬上 obstacle，如果 obstacle top 太短，也可能沒有足夠距離完成：
retract
→ wheel reset
→ foot rim ready
→ re-extension

請：
1. 對固定 obstacle height 與固定 / feasible theta_climb，掃描 obstacle top length L_top。
2. 每次都執行完整 full rolling recovery。
3. 記錄：
   - L_top
   - full_recovery_success
   - required_reset_distance
   - failure_stage
   - failure_reason
4. 比較：
   L_top
   vs
   required recovery distance
5. 注意 required recovery distance 不只包含 theta=17 後的 wheel roll；如果 retract 本身造成 forward displacement，也要納入實際幾何 trajectory 所需的總 usable top distance。
6. 找出 minimum top length required for full rolling recovery。
7. 輸出 CSV 與 plot：
   obstacle top length vs recovery feasibility

如果現有 terrain representation 會讓 trailing edge 造成 collision / contact transition，請正確納入，不要只做 algebraic length comparison。
```

### 驗收重點

```text
程式可以回答：
「這個 obstacle 雖然爬得上去，但 top 至少要多長才能完整 reset？」
```

---

## Codex Step 11 — Rolling-Assisted Swing Fallback

### 可以直接給 Codex 的指令

```text
現在請加入 rolling-assisted swing fallback，但不要重寫新的 swing planner。

情況：
如果：
roll_up_success == true
但：
full rolling recovery == false

則不要直接把整個 traversal 判定失敗。
請保留 rolling 所到達的最後一個合法 state，並將它作為既有 swing planner 的 start state。

希望流程：
RIGHT_RIM_ROLL_UP
→ useful rolling as far as feasible
→ recovery fails / top length insufficient
→ switch to existing swing planner
→ generate touchdown trajectory

要求：
1. 優先 reuse 現有 HybridSwing / swing generation code。
2. 不要現在 redesign swing trajectory。
3. 明確建立 rolling-to-swing transition state：
   - theta
   - beta
   - hip pose
   - active rim
   - contact point
4. 指定一個簡單 target touchdown：
   - obstacle top 上安全位置
   - 或下一個可行 foothold
5. 產生完整：
   roll segment + swing segment
6. 檢查 transition 前後 joint trajectory 是否連續。
7. 輸出：
   - rolling_distance_before_swing
   - swing_start_state
   - swing_end_state
   - success / failure
8. visualization 要可以看出：
   先滾上去，再 swing reposition。

目前不需要做 energy optimization，也不需要比較最佳 swing cost。
```

### 驗收重點

```text
Full rolling recovery 不可行時，
仍然可以：
roll first → swing later
```

---

## Codex Step 12 — 三類 Motion Classification

### 可以直接給 Codex 的指令

```text
請把目前 Day 6–7 的結果整理成三類 obstacle traversal classification。

三類為：

1. FULL_ROLLING_RECOVERY
   right rim roll-up
   → retract to 17
   → wheel reset
   → foot rim ready
   → re-extend

2. ROLLING_ASSISTED_SWING
   right rim roll-up 有幫助
   → 但 full recovery 不可行
   → 從最後合法 rolling state 接既有 swing

3. DIRECT_SWING
   right rim roll-up 本身不可行，或 rolling 沒有有效 progress

請建立一個簡單 deterministic classification function。

建議邏輯：
- 先測試是否存在 feasible theta_climb 可以完成 full rolling recovery。
  有 → FULL_ROLLING_RECOVERY
- 否則測試是否存在 useful roll-up trajectory，且可以接 swing。
  有 → ROLLING_ASSISTED_SWING
- 否則 → DIRECT_SWING

要求：
1. 第一版不要做 cost optimization。
2. 不要根據 runtime sensor 做 decision；這是 offline known-terrain classification。
3. 輸出結果至少包含：
   - motion_class
   - selected_theta_climb
   - failure / selection reason
   - rolling trajectory（若有）
   - swing trajectory（若有）
4. 建立幾個 regression cases：
   - 低且長 obstacle → full rolling
   - 可爬但 top 太短 → rolling-assisted swing
   - 太高 / geometry impossible → direct swing
5. 輸出一個 summary CSV，方便之後做：
   obstacle height × top length → motion class
```

### 驗收重點

```text
對一個已知 rectangular obstacle，
可以自動分類：

FULL_ROLLING_RECOVERY
ROLLING_ASSISTED_SWING
DIRECT_SWING
```

---

# 18. 建議實際丟給 Codex 的順序

不要一次把 Step 1–12 全部做完。

最推薦的實作順序：

```text
Step 1
↓
Step 2
↓
Step 3
↓
【停下來，先人工確認 right-rim roll-up 的物理行為真的正確】
↓
Step 4
↓
Step 5
↓
Step 6
↓
Step 7
↓
【完成第一條完整 rolling recovery trajectory】
↓
Step 8
↓
Step 9
↓
【產生第一張正式 feasibility map】
↓
Step 10
↓
Step 11
↓
Step 12
```

其中最重要的三個 checkpoint：

```text
Checkpoint A:
Step 3 完成
→ right rim 真的可以利用 vertical face / corner roll-up

Checkpoint B:
Step 7 完成
→ 完整 no-swing primitive 成立

Checkpoint C:
Step 9 完成
→ 有系統性的 obstacle height × theta feasibility result
```

如果某個 checkpoint 的物理行為不合理，先停下來改模型，不要急著繼續堆後面的 planner。
