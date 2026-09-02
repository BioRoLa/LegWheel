# Hybrid Gait Day 10–11：Roll-vs-Swing Motion Selection

> **用途**：記錄 Day 10–11 的研究發想、為什麼原始規劃在這兩天開始之前就已經被 Day 6–7 / Day 8–9 的數據推翻、最後選定的方法，以及可以逐步交給 Claude 的實作項目。
> 之後撰寫 paper 時，可回頭追溯為什麼 motion selection 的判準是「向 body trajectory 索取的讓步」，而不是「哪一種 primitive 可行」。
>
> **研究範圍**：offline、單腳、2D sagittal。Day 10–11 決定 **ROLL / SWING 如何選**，並把選擇的結果表示成一條可執行的 segment sequence。四腳 timing、body trajectory 生成、ABAD 都不在範圍。
>
> **2026-08-30 加上的閱讀前提（很重要）**：這份文件裡大量出現「被推翻 / 不可行」，
> 那些全部是**關於目前這個單腿 2D planner 的敘述**，不是關於機器人的。
> 正式用語與四個層級定義在 **§5.5**；`TOP_REPOSITION` 這個新缺口在 **§5.6**。
> **引用本文的任何 negative result 之前請先讀那兩節。**
>
> **成果的實際 scope**：`offline / single-leg / 2D sagittal / 已知矩形障礙 /
> 幾何與運動學與取樣式碰撞檢查`。**不包含**四腳 timing 與支撐、body trajectory
> 的正式生成、ABAD 與 3D lateral、載重/動力/摩擦/能量、真實 joint command 與硬體驗證，
> 也**不包含**對任意隨機障礙都保證有解。
>
> **前置**：Day 6–7 Step 11R / 12R（rolling feasibility map、top-length 分析）、Day 8–9 Step 1–9 + §26–§28（swing planner、showcase、兩道天花板）。

---

# 1. Day 10–11 的核心問題

原始規劃寫的是：

```text
if continuous rolling is feasible:
    ROLL
else:
    SWING
```

這兩天要回答的問題已經改成：

> **對同一塊 terrain，roll 與 swing 各自向 body trajectory 索取多少讓步？選索取較少的那一個。**

改動看起來只是換了判準，實際上改掉了三件事：planner 的輸入輸出方向、feasibility map 的 cell 值、以及 Day 12 收到的東西。以下依序說明。

---

# 2. 原計畫在哪裡失效

## 2.1 落差一：決策粒度錯了

原規則假設 ROLL vs SWING 是 per-obstacle 的二選一。但 Day 6–7 收斂出來的 rolling 不是一個 primitive，是一條五段鏈：

```text
APPROACH
  -> RIGHT_RIM_ROLL_UP
  -> RETRACT_TO_17_DEG
  -> WHEEL_MODE_TO_TRAILING_CORNER
  -> LEFT_RIM_ROLL_DOWN
```

而 Day 8–9 §26.7 的第一個發現是：**roll 與 swing 的強弱互補，而且分在 traversal 的不同階段**。

```text
160 mm 障礙
    roll_up    10 / 10
    roll_down   0 / 10      <- rolling 的整體失敗發生在這裡
    swing down  可行（只要 hip 不跟著腳一起掉）
```

所以真正的決策不是「整個障礙用哪一種」，而是**在段邊界上要不要切出去**。上升側幾乎恆為 ROLL，決策發生在 top 上要不要改成 swing 下去。

Day 6–7 §8 已經定義了三類：

```text
Strategy A   Full Rolling Recovery    right-rim up -> 17° wheel -> left-rim down
Strategy B   Rolling-Assisted Swing   roll up -> wheel transition -> swing down
Strategy C   Direct Swing             swing over
```

原規劃把三段式列為「之後有時間再擴充」。實際上 **Strategy B 是預設路徑**——它是切換點落在下降側的直接後果。

但這三類**本身也不完整**，見 §2.6：上升與下降是兩個獨立決策，策略空間是 2×2 + 1，不是 3。

## 2.2 落差二：度量錯了（這是最關鍵的一點）

原規則用二值可行性當判準。但把 Day 8–9 §28.1 的 showcase 和 Day 6–7 Step 11R 放在一起看：

| h (mm) | swing onto | swing 的代價 | rolling traversal |
| --- | --- | --- | --- |
| 20 | OK | 無 | 可行（θ 40–70°） |
| 60 | OK | 無 | 可行（θ 40–85°） |
| 100 | OK | 無 | 可行（θ 40–85°） |
| 120 | OK | hip +20 mm | 可行但只剩 θ ∈ [45°, 55°]（2 cell） |
| 140 | OK | hip +40 mm | 可行（θ 40–70°） |
| 160 | OK | hip +60 mm、liftoff +30 mm | **不可行（0 cell）** |
| 200 | FAIL | — | 不可行 |

上升側**兩種都做得到**。二值可行性在這裡是平手，沒有鑑別力。

有鑑別力的是右邊那一欄。程式本身就把這件事講得很清楚——`swing_onto_step_2d` 的 docstring 寫：

> Raising the hip in particular is a request to the *body* planner — the hip trajectory is an input here — so it is never silently folded into the answer.

`hip_lift` 被刻意做成會回報的 `adjustments`，就是因為它不是 swing planner 自己能付的代價。

**把 cell 的值從 `feasible / infeasible` 換成「向 body 索取多少」，這個平手才會變成結果。**

以 ICRA 的審查標準看，這個差別是決定性的。二值 map 會遇到一個立即的質疑：「那你把 hip 抬高一點 swing 不就過了嗎？」——而數據顯示**確實會過**。這個問題在二值框架下無法回答；在 concession 框架下，答案就是結果本身：

> 在 h ≤ 140 mm，rolling 以零 body 讓步完成 traversal；swing 完成同一件事需要 hip 抬 40–60 mm。Planner 的判準不是「誰可行」，而是「誰對 body trajectory 的要求較小」。

> **2026-08-30 Step 4 更新：「零 body 讓步」那半是錯的，判準那半成立。**
> rolling 的 hip 起伏是 `h + 14.2 mm`（theta_climb = 40 deg），不是 0。
> 量出來的比較是：**h ≤ 100 mm swing 略優（1–3%），h ≥ 120 mm roll 優 9–12%**。
> 判準本身（比 body 讓步而不是比可行性）**仍然成立**，而且現在有了跨型別的排序規則
> （§5.3 的更新框）。正確的一句話寫法見 Step 4 完成紀錄最後一段。

還有一個 review-proofing 的理由：showcase 那張表的 `min_clearance` 全部落在 **1.0–1.8 mm**。二值判定整個騎在毫米級餘裕上，而 Day 8–9 §26.5(5) 已經證明取樣密度會動這個量。用 concession / margin 當 cell 值，結論就不會建立在一個對取樣參數敏感的門檻上。

## 2.3 落差三：swing 的主變數不在 swing planner 手上

Day 8–9 §26.5(6)：

```text
同一組 case、同一組落點高度
    起點離台階 0.10 m  ->  五個撞三個
    起點離台階 0.20 m  ->  五個全過
```

機制：輪半徑 0.145 m 大於「接觸點到障礙的距離」時，接觸點還沒開始動，輪胎就已經在障礙裡了。

而 `swing_onto_step_2d` 目前把 `approach_distance_m` **固定在 0.20 m 且不搜尋它**——失敗時只回報 `start_knob_name = "approach_distance_m (start further from the step)"`，叫呼叫端自己去改。

```text
LegWheel/hybrid_note/scripts/experiments/cartesian_swing_planner_2d.py:1118
    swing_onto_step_2d(..., approach_distance_m: float = 0.20, ...)
```

**這一維就是 Day 10–11 要補的主軸。** 它不是 planner 的調校旋鈕，它是 swing 這個 primitive 的主要自由度，而且它同時是一個 body-level 的量（腳在哪裡落地，決定於 body 走到哪裡）。

## 2.4 落差四：循環依賴

把 2.3 和原規劃並排就會看到問題：

```text
原規劃：
    Day 8–9   hip trajectory 是 swing planner 的【輸入】
    Day 10–11 決定 roll / swing
    Day 12    生成 body / hip trajectory

但 2.3 說：swing 的可行性主要由 hip trajectory 決定
=> Day 10–11 要在不知道 body 怎麼走的情況下，決定一件由 body 怎麼走決定的事
```

Concession 框架讓這個循環自動消失，因為它把方向反過來：

```text
原本：body trajectory -> swing feasibility -> 選 primitive        （循環）
改成：terrain -> 每個 primitive 的最小 body 讓步 -> 選 primitive
                                                -> 輸出成 Day 12 的 body 約束
```

Planner 不再需要 body trajectory 當輸入，而是**輸出 body trajectory 的需求**。Day 12 收到的不再只是 ROLL/SWING 標籤，而是「在 x ∈ [a, b] 這段，hip 至少要抬到 z ≥ …」這種可以直接餵給四腳 timing 的約束。

**這是 Day 10–11 最重要的架構決定。** 它讓 Day 10–11 的產出即使 Day 12 沒做完也是完整的。

## 2.5 落差五：輸出 schema 兜不起來

原規劃的完成標準要求：

```text
target contact / rim / alpha / foothold for every transition
```

但：

- **wheel-mode 段沒有固定接觸點。** θ=17° 連續滾過去，接觸點在 rim 上連續移動，用「一個 rim + 一個 alpha」表示不了。
- **swing 段有只有 planner 內部知道的欄位**：`liftoff_rise_m`、`touchdown_drop_m`、`swing_duration_s`、`clearance_m`。
- **取樣密度是 case 的一部分，不是自由旋鈕。** Day 8–9 Step 9 發現 `max_joint_step_rad` 是 per-step 限制，同一條軌跡取樣密一倍就少一半。schema 不記 `arc_samples` / `sample_count`，sequence 就不可重現。

所以 schema 必須是 **segment 級**，而且每段要帶自己的取樣參數與 body requirement。

## 2.6 落差六：上升與下降被綁成同一個選擇

原規則 `if rolling feasible: ROLL else SWING` 對整個障礙只做一次選擇。Day 6–7 §8 的
Strategy A / B / C 雖然把 rolling-assisted 拆出來了，但仍然只列出三條路徑。

**這是不完整的。** 上升與下降是兩個獨立的決策，正確的策略空間是：

```text
ascent  in {ROLL_UP, SWING_UP}
descent in {ROLL_DOWN, SWING_DOWN}
=> 2 x 2 = 4，全部都落腳在 obstacle top

外加一種不落 top 的：
        SWING_OVER      一次 swing 越過整個障礙
=> 合計 5 種
```

> **2026-08-30 Step 8 的最終判決：這個分解【概念上】成立，但在
> 【目前的單腿 planner 下】選擇集合塌陷成對角線。**
>
> **用語提醒（§5.5）**：下面的「推翻」一律讀作 `DIRECT_HANDOFF_INFEASIBLE`
> ——目前的兩個接觸 primitive 無法**直接連續交接**。它們**不是**
> `PHYSICALLY_INFEASIBLE`；在四腳支撐下的 `TOP_REPOSITION`（§5.6）之後可能成立，
> 只是**尚未驗證**。
>
> ```text
> #3  SWING_UP + ROLL_DOWN    落地要跨 alpha = -40 deg 接縫   Step 2b  不可修
> #2  ROLL_UP  + SWING_DOWN   起飛要跨 alpha = +40 deg 接縫   Step 3 D 不可修
>                             或退化成 retract，撞 theta >= 35 deg 下限
>                                                            Step 3 E 【可修】
> ```
>
> Step 8 的第三條完成標準是「至少有一個 case 的最佳解是混合的，否則 2×2 沒有被驗證」。
> **那條標準無法達成**——沒有混合對可選。
>
> **但 Step 8 量出了這件事的代價，而且它證明這個分解是對的。**
> 在 `h = 160 mm`（正是本節第 2 列說的「h 超過 roll_down 天花板」），
> Day 6–7 的 sweep 是 `roll_up` **10/10** 成功、`roll_down` **0/10**。
> 那個爬得上去卻用不了的 ascent，對 body 的要求是 **80.1 mm**，
> 而被迫改用的 `SWING_UP` 要 **220.0 mm**——**貴 2.7 倍**。
>
> ```text
> => 分開決策【確實】有價值：那塊地形上最好的爬升方式和最好的下降方式
>    分屬不同 primitive。分解不是錯的，是【現在做不到】。
>    而做不到的那道牆（Step 3 E）是可修的，這一格量出了修它值多少。
> ```

對照舊名稱：

| # | ascent | descent | 舊名 | 什麼時候勝出 |
| --- | --- | --- | --- | --- |
| 1 | ROLL_UP | ROLL_DOWN | Strategy A | 低矮且 top 夠長 |
| 2 | ROLL_UP | SWING_DOWN | Strategy B | h 超過 roll_down 天花板（160 mm） |
| 3 | ~~**SWING_UP**~~ | ~~**ROLL_DOWN**~~ | **缺** | ~~top 太短，付不起 L_transition~~ **Step 2b 已推翻，見該節** |
| 4 | SWING_UP | SWING_DOWN | Strategy C 的一種 | 高，且 top 夠長可落腳 |
| 5 | — SWING_OVER — | Strategy C 的另一種 | top 很短 |

兩個發現：

**(a) 舊的 Strategy C 把 #4 和 #5 混成一類。** 它們的 top-length 需求完全不同——#4 要在 top 上
落腳再起跳，#5 完全不碰 top。混在一起會讓 decision map 上的 region 邊界失去意義。

**(b) #3 是被漏掉的那一種，而且它正好攻擊 rolling 最硬的限制。**

> **2026-08-30 後見之明**：這一段的【推理】成立，【結論】不成立。#3 確實是 2×2 裡
> 唯一攻擊 `L_transition` 的一格，所以值得測——但 Step 2b 測完之後它被推翻了。
> 這段保留原文，因為它是 Step 2b 為什麼要做的論證來源。結論見 Step 2b 一節。

理由要從 `LEFT_RIM_READY` 的定義看。依 Day 6–7 Step 8R 的決定，它是**抵達後緣時檢查的前提條件**，
不是一個停止點：

```text
theta == 17°
AND left rim 已接手為 obstacle-top 的有效接觸 rim
AND no collision / penetration
AND 幾何上可做 trailing-edge approach
```

而 `L_transition` 被定義為「從 right-rim roll-up 出口到 LEFT_RIM_READY 所需的 top 前進距離」——
**它就是 Step 12R 那個 0.20–0.27 m top-length 下界的來源**。

如果改用 swing 上去，Day 8–9 的 swing 本來就是 contact-state-to-contact-state 的，
可以直接指定落點的 rim / alpha / theta。原則上可以**直接落在後緣附近、已經滿足 LEFT_RIM_READY 的
狀態**，完全跳過 `L_transition` 這筆預算。

## 2.7 五種策略的耦合結構：top-length 預算

四種落 top 的組合不是獨立的，它們透過 **top 上的狀態**耦合：

```text
ascent 留下什麼狀態          descent 需要什麼狀態
ROLL_UP   -> roll-up 出口     ROLL_DOWN   -> 後緣的 LEFT_RIM_READY
             （theta_climb 決定，                （theta=17°、left rim、可做 trailing-edge approach）
               位置由幾何決定）
SWING_UP  -> 任意可行落點     SWING_DOWN  -> top 上的合法站姿 + 起跳距離
             （IK 可行範圍內自由）
```

把兩端接起來，每一對都有自己的**最小 top-length 預算**，而且順序是單調的：

```text
ROLL_UP  + ROLL_DOWN     L_transition                     0.20–0.27 m（已量測，下界）
ROLL_UP  + SWING_DOWN    roll-up 出口 -> 起跳距離          【不成立，見 Step 3 D/E】
SWING_UP + ROLL_DOWN     落點 -> 後緣 LEFT_RIM_READY       【不成立，見 Step 2b】
SWING_UP + SWING_DOWN    落點 -> 起跳距離                  takeoff <= 0.24 m (h<=100)
                                                          … <= 0.12 m (h=200)【上限】
SWING_OVER               0（不碰 top），但需 stride >= L_top
```

> **2026-08-30 更新（Step 2b）**：第 3 列不成立。swing 只能交出 foot-rim 落地，
> 要換到 left rim 得穿過 `alpha = -40 deg` 那道 29 deg 的接觸不連續。所以
> `SWING_UP + ROLL_DOWN` 仍需在頂面做一次 rim handover（foot→left 而非 right→left），
> **沒有躲掉 `L_transition`，只是換了個名字**。下面「越靠 swing 的組合，需要的 top length
> 越短」這句話，因此只在第 2、4、5 列之間成立。

> **2026-08-30 更新（Step 3）**：第 2 列也不成立，但**原因和第 3 列不同**——
> D 段（直接交接）撞的是 `alpha = +40 deg` 的 rim 接縫（Step 2b 的鏡像，不可修）；
> E 段（退化成先 retract 再起飛）撞的是**下降側自己的 `theta >= 35 deg` 下限**，
> 而 Day 6–7 的 retract 寫死停在 17 deg。**E 是可修的**（見 Step 3 完成紀錄）。
> 目前站得住的落 top 組合只剩第 1、4 列。

> **2026-08-30 更新（Step 3 A 段）**：下面這句話成立，而且比原本寫的更強。
> 不只是「swing 需要的 top length 較短」——**兩者是同一個座標的相反方向**：
> rolling 需要 `L_top` **大於** 196–249 mm，swing off 需要 takeoff **小於** 120–240 mm。
> 所以 Step 5 的第一個判準是頂面長度的**方向**，不是代價比較。

> **越靠 swing 的組合，需要的 top length 越短。**

> **2026-08-30 Step 5 F 段的最終判決：照量到的數字【不成立】，而且它的真假
> 由一個沒有被最佳化的參數決定。**
>
> ```text
> #1 ROLL  + ROLL    需要 L_top >= 205 - 245 mm
> #4 SWING + SWING   需要 L_top >= 240 mm         <- 比 #1 還長
> #5 SWING_OVER      需要 L_top <= 170 - 280 mm   <- 唯一反向的，無條件成立
> ```
>
> `#4` 只在 6 個高度中的 1 個比 `#1` 短。但 `#4` 的下界是
> `landing_distance + 最短 takeoff = 160 + 80`，而 `landing_distance = 160 mm`
> 只是 Step 2 格點的固定值 —— close-out 掃過這一軸，**100 mm 在
> h = 100/150/200 都可行且 lift 完全不變**。改用它變成 `>= 180 mm`，
> 於是每個高度都比 `#1` 短、這句話成立。
>
> **兩個版本都寫進 Step 5 完成紀錄**，並標明 close-out 只在三個高度驗證過。
> 下面那句「升格成主軸」仍然成立 —— 只是升格的理由是 §2.6 的**方向相反**
> （Step 3 A 段）與 `theta` 階梯（Step 5），不是這條單調性。

這句話如果被數據證實，`L_top` 就從一個約束升格成**與 height 平起平坐的策略選擇主軸**，
而且它是一個 rolling 的二值 map 完全看不到的維度。這是 §3 選 (i) 之外，Day 10–11 第二個
可以撐起 paper 的結果。

## 2.8 #3 成立與否的關鍵未知數

**swing 能不能真的落在 theta = 17° 的 left-rim contact，目前沒有任何數據。**

theta = 17° 是最收縮的姿態，reach 最差；而 Day 8–9 的所有 swing 都用 theta = 60°
（`REGRESSION_THETA_RAD`）當落點姿態，`theta_min_deg` 那一欄量的是軌跡途中的最小值，
不是落點值。

三個必須一起檢查的子問題：

```text
(a) IK 可行嗎    theta=17° 時 hip 要多低才搆得到 top？這本身是一筆 body 讓步
(b) rim 對嗎     swing 由後往前擺，落在 left rim（輪子的後側）是不同的姿態，
                 不是只換一個 alpha 數字
(c) alpha 縫     left rim 在 top 上的接觸 alpha 離 ±180° 那道縫多遠？
                 Day 8–9 §26.5(2)：跨過去會瞬移 162 mm
```

三個都過，#3 才成立。**任何一個不過，#3 就退化成「必須先落在別的姿態、再 retract 到 17°」**，
那筆 retract 又會吃掉 top length，優勢就縮小了。這要在 **Step 2b** 明確測，不能假設。

> **2026-08-30 實測結果**：(b) 與 (c) 都過且有數字，(a) 不過。而且退化版本也不成立——
> 落地 theta 放寬到 60° 也一樣落不上去，所以擋住的不是「17° 太極限」。
> 真正的原因是**第四個、這裡沒列到的**子問題：站姿接觸永遠在 `foot_rim, alpha = 0`，
> 要遷移到 left rim 就得穿過 `alpha = -40°` 的 rim 分段接縫（約 29° 的關節不連續）。
> 上面只把 `±180°` 那道縫列為風險，漏了 `-40°` 這一道——而擋住 #3 的正是後者。
> 完整結論見 Step 2b 一節。

---

# 3. 為什麼是這個 sweep，不是另外兩個

Day 10–11 開始前考慮過三種 swing feasibility sweep：

```text
(i)   height × approach clearance，theta 固定
(ii)  height × theta，approach 固定成幾個離散值
(iii) 只掃下降側（top -> flat）
```

選 **(i)**，並加上兩個修正。以下保留 (ii)/(iii) 被否決的理由，因為它們是 paper 中方法論一節的論證來源。

## 3.1 為什麼不是 (ii)

(ii) 的賣點是「軸和 rolling map 完全對齊、疊圖漂亮」。但 roll 的 `theta_climb` 和 swing 的 `theta_liftoff` 名字一樣，**不是同一個物理量**：

```text
theta_climb    用什麼姿態【滾】上去   —— rolling 的內部自由度
theta_liftoff  用什麼姿態【離地】     —— swing 的內部自由度
```

把它們畫在同一根軸上，圖是齊了，但那個重疊沒有意義——等於在比較兩件不同的事。而且它把 swing 真正的主變數（approach）降級成參數，正好丟掉 2.3 那個發現。

**正確的共用軸只能放 terrain 參數。** 每個 primitive 對自己的內部自由度取最佳：

```text
ROLL  cell(h, L_top)  = 存在 theta_climb 使 traversal 可行？代價多少？
SWING cell(h, c)      = 最小的 (hip_lift, liftoff_rise)
```

這是標準的 feasibility-envelope 做法，也是唯一公平的比法。

## 3.2 為什麼不是 (iii)

(iii)（只掃下降側）最省，而且看起來最貼近實際決策——§26.7 說切換點在下降側。

但它砍掉的正好是唯一能證明「rolling 有存在必要」的那半邊數據。只掃下降側，能 claim 的是 rolling 充分，不能 claim rolling 必要：上升側沒有 swing baseline，計畫 §16 的 ablation 表上那一格是空的。

而且 2.2 那張表顯示，**上升側恰恰是代價差距最大的地方**（0 mm vs 60 mm hip lift）。這是目前全部數據裡最好的一個數字，不該掃不到。

另外，「切換點在下降側」這句話描述的是**二值可行性**在哪裡翻面——而 2.2 已經論證二值是錯的度量。換成 concession 之後，上升側不再是平手區，它是主要證據區。

## 3.3 選 (i) 的兩個修正

```text
修正 1   cell 的值是 concession，不是 feasible / infeasible
修正 2   共用軸只放 terrain 幾何；approach clearance 是 swing 的內部自由度，
         在疊圖時對它取最小，內部 map 留作 supporting figure
```

內部 map（h × c 的完整 concession 熱圖）不是廢棄物：它顯示可行窗口有多寬，也就是這個決策對執行誤差有多敏感。真機實驗要用到。

---

# 4. 共用軸與幣別：Step 0 必須先解掉的事

## 4.1 現況：兩邊用不同的 approach 幣別

```text
rolling sweep   right_up_left_down_sweep_2d.SweepSettings2D
    obstacle_x_start_m         = 0.10
    obstacle_width_m           = 0.35
    approach_start_clearance_m = 0.04     <- 固定的是【前緣餘裕】
    （用 _hip_x_for_start_clearance 做 bisection 反解 hip x）

swing showcase  cartesian_swing_planner_2d.swing_onto_step_2d
    obstacle_x_start_m   = 0.20
    obstacle_width_m     = 0.45
    approach_distance_m  = 0.20           <- 固定的是【hip x 距離】
```

rolling 那邊選擇固定餘裕而不是固定 hip x，理由寫在 `SweepSettings2D` 的註解裡，而且是對的：

> the leg radius grows with theta_climb, so one hip x would leave every theta a different distance to roll

同樣的理由對 swing 也成立，而且 2.3 的機制（輪半徑 vs 接觸點到障礙的距離）**本來就是一個餘裕問題，不是一個 hip x 問題**。

## 4.2 決定

```text
共同 approach 幣別   前緣餘裕 c（standing 姿態下，腿到障礙前緣的最短距離）
共同障礙幾何         沿用 rolling 那組 (x_start = 0.10, width = 0.35)，
                     top length 需要變動時才動 width
轉換工具             重用 right_up_left_down_sweep_2d._hip_x_for_start_clearance
```

理由：rolling 的 map 已經跑完了，換 swing 的幣別成本遠低於重跑 rolling。

## 4.3 順帶要解掉的三件事

**(a) roll-end → swing-start 的交接要一般化。** 目前只有 `day6_7_roll_end_swing_start_2d()` 這一條單一 case，不是任意 (h, θ) 的轉換。Strategy B 要串起來就需要一般版。

**(b) 1.2 mm rim 幾何差異會在交接點第一次真的咬到。** Day 8–9 選了繪圖弧 0.1438 m 建 FK/IK 以對齊 `query_contact`，但 `LegModel.rim_point` 是 0.145 m，差值大於 1 mm 的 contact tolerance。單獨在 roll 或 swing 內部都沒事（各自自洽），但 roll 的終點 contact state 要餵進 swing 的 IK 時，兩套幾何會在同一個交接點上相遇。**Step 0 要量它，不一定要修它**——沿用 Day 8–9 的決定（量化記錄），但要知道交接誤差有多大。

> **2026-08-30 實測修正。** 上面這段的說法不夠精確。Step 0 量到的是：
>
> ```text
> 在接觸管線自己的幾何裡，交接是【精確的】
>     contact drift 0.0 mm、alpha 差 0.0 deg、rim 標籤 52/52 相符
> 對物理腿模型的偏差是 1.2 mm，而且幾乎每個交接點都非零
>     ROLL_UP 出口 -> right_rim（28 個）
>     WHEEL_TRANSITION 出口 -> left_rim（23 個）   兩者都是 upper tyre
>     唯一為 0 的是一個 foot_rim 的 ROLL_DOWN 出口
> ```
>
> 也就是說 1.2 mm **不會**污染 roll→swing 的交接（那是自洽的），它污染的是
> **往物理模型的映射**。所以它不是 Day 10–11 的阻礙，而是 Day 12 產生真實
> joint command 時才必須處理的事——那時才會有人同時使用兩套幾何。

**(c) hip_z 在兩邊的語意不同。** rolling 的 `TraversalInitialState2D` 讓 `hip_z_m = None`，因為「站在下層地面上」是固定條件，hip 高度是 θ 的**輸出**；swing 的 hip 高度是**輸入**。這一點在 Step 4 會變成 rolling concession 的核心——見下。

---

# 5. 資料介面

## 5.1 `SwingConcession2D`

把 `StepSwingShowcase2D` 那串 `adjustments: tuple[str, ...]` 字串正規化成可比較的數值。

```text
feasible                 bool
direction                "onto" | "off"
obstacle_height_m        float
approach_clearance_m     float          # 這一格的 approach 座標
min_hip_lift_m           float | None   # onto：body 必須抬高多少
min_hip_hold_fraction    float | None   # off：body 必須「不要跟著掉」多少（台階高度的比例）
min_liftoff_rise_m       float
min_touchdown_drop_m     float
duration_scale           float          # 1.0 = 沒有被延長
min_clearance_m          float | None
theta_min_deg            float | None
binding_ceiling          "REACH" | "FIT" | "NONE"
failure                  SwingFailure | None
```

`binding_ceiling` 一定要有：Day 8–9 §28 量到的是兩道**依序**生效的天花板（搆得到 / 塞得下），而它們要跟 body planner 要的東西不一樣。REACH 要的是 hip 高度，FIT 要的是端點控制點或 approach 距離。

## 5.2 `RollConcession2D`

欄位要能和 5.1 直接比較。

```text
feasible                 bool
obstacle_height_m        float
top_length_m             float
approach_clearance_m     float
best_theta_climb_deg     float | None   # 內部自由度取最佳後選到哪一個
feasible_theta_span_deg  float          # 窗口寬度 = 對誤差的容忍度
required_hip_z_range_m   (float, float) # rolling 過程中 hip 被迫走的高度範圍
min_clearance_m          float | None
failure_stage            str | None     # 沿用 Step 8 的 failure stage 命名
```

`required_hip_z_range_m` 是這一步的重點，理由見下。

## 5.3 rolling 不是「零 body 讓步」——這是 Step 4 真正要量的東西

2.2 那句「rolling 以零 body 讓步完成 traversal」是**現階段的直覺，不是量測結果**，寫進 paper 前必須被證實或修正。

因為 4.3(c)：rolling 的 hip 高度不是自由變數，它是被 θ 和接觸幾何決定的**輸出**。也就是說 rolling 對 body 的要求不是「零」，而是**另一種形式**：

```text
swing 對 body 的要求：  hip 在某個時刻必須抬到某個高度（一個下界）
roll  對 body 的要求：  hip 必須【全程跟著】一條被幾何決定的軌跡（一條軌跡，不是一個下界）
```

這兩者哪一個對四腳 body trajectory 比較苛刻，**不是先驗的**。有可能 rolling 的 hip 軌跡約束反而更難滿足。Step 4 要把它量出來；如果結果是 rolling 更苛刻，那是一個更有意思的 paper 結果，不是壞消息。

> **2026-08-30 Step 4 完成：規則有了。** 實作在
> `day10_11_roll_concession_2d.compare_body_demand_2d`。
>
> ```text
> 一個 LOWER_BOUND 是【一族】body 軌跡，而那一族裡最便宜的成員，
> 其垂直起伏【恰好等於】那個下界。
>
> => 兩者在【各自的最小值】上是可比的：
>    rolling 的起伏是被規定死的、因此已經是最小；
>    swing 的最小值就是它的下界。兩邊都在回答
>    「這個 primitive 最少能向 body 要求多少」。
> ```
>
> **大小沒有捕捉到、呼叫端不能忘記的事**：rolling 還額外規定了**形狀**（每一個瞬間
> 都被決定），swing 只規定一個極值。那份自由是真的、而且這裡沒有計價。
> **所以大小打平時算 swing 贏**，`compare_body_demand_2d` 的回傳值就是這樣寫的。
>
> **量出來的答案**（`h` 是障礙高度，roll 已對 `theta_climb` 取最佳）：
>
> ```text
> h <= 100 mm  -> SWING 便宜（roll 多付一個常數 14.2 mm 的 roll-up overhead）
> h >= 120 mm  -> ROLL  便宜（swing 的 min_hip_lift 開始長：20 / 40 mm）
> ```
>
> 所以 2.2 那句「rolling 以零 body 讓步完成 traversal」**是錯的**：
> rolling 的 hip 起伏是 `h + 14.2 mm`，不是 0。但它的**形狀**是梯形（含一段平頂），
> 而 swing 是三角形，這才是兩者真正的差別。

## 5.4 `MotionSegment2D` / `MotionSequence2D`

```text
MotionSegment2D
    kind            APPROACH | ROLL_UP | WHEEL_TRANSITION | ROLL_DOWN
                    | SWING_UP | SWING_DOWN | SWING_OVER
                    # swing 分成三種不是為了好看：它們的 target 語意不同
                    #   SWING_UP    落在 obstacle top（可能是 LEFT_RIM_READY）
                    #   SWING_DOWN  落在下層地面
                    #   SWING_OVER  完全不碰 top，越過整個障礙
    start_contact   ContactState | RollingContact
    end_contact     ContactState | RollingContact
    duration_s      float
    sampling        {arc_samples, sample_count, leg_arc_samples, max_joint_step_rad}
    body_requirement BodyRequirement2D
    frames          逐幀資料的參照（不內嵌）

RollingContact                       # WHEEL_TRANSITION / ROLL_* 段用
    rim             RimId
    alpha_range     (float, float)   # 不是單點
    surface_id      str

BodyRequirement2D
    x_range_m           (float, float)
    hip_z_min_m         float | None     # swing 段：下界
    hip_z_profile       ndarray | None   # roll 段：被幾何決定的軌跡
    kind                "LOWER_BOUND" | "TRACK"
```

`start_contact` / `end_contact` 允許是 `RollingContact` 就是為了 2.5 講的 wheel-mode 段。這是原規劃「every transition 一個 rim / alpha」表示不了的那個東西。

> **2026-08-30 Step 6 實作時，資料改了這張表兩個地方。**
>
> ```text
> 1. RollingContact 需要 beta_range，不只是 alpha_range
>    Step 10R 有 72/299 幀是【corner pivot】：alpha 與接觸點都釘死在後緣角，
>    只有 beta 掃了 71 度。只記 alpha_range 會把它記成一個靜止姿態。
>    => 加 beta_range / theta_range，加 RollingMode（SURFACE_ROLL / CORNER_PIVOT）。
>
> 2. sampling 不是一個結構，是【兩個】
>    swing 用 (arc_samples, sample_count, leg_arc_samples, max_joint_step_rad)；
>    rolling 用 (arc_samples, beta_step_rad, theta_step_rad)。
>    一個半數欄位是 None 的結構，會讓「哪些必填」變成無法回答的問題 ——
>    而那正是 Step 6 第三條完成標準在測的。
>    => RollSampling2D 與 SwingSampling2D，兩個都沒有預設值。
> ```
>
> **另外兩個實作決定：**
>
> ```text
> 3. 端點一律是 PointContact2D，即使 rolling 段也是。
>    上面的二選一不夠：Step 7 的交接檢查要比對關節值，兩端都需要確定的姿態；
>    §2.5 抱怨的是兩端【之間】發生了什麼。union 型別保留，但 builder 不產生它。
>
> 4. duration_s 是 float | None，不是 float。
>    Day 6-7 的 traversal 是準靜態的、從未指定時間 —— 記成缺失而不是編造。
>    Step 7 的 composer 必須替 rolling 段指定時間，那是一個新的決定。
> ```

---

## 5.5 結論用語：`COMPOSED` / `DIRECT_HANDOFF_INFEASIBLE` / `REQUIRES_MULTILEG_REPOSITION` / `PHYSICALLY_INFEASIBLE`

> **2026-08-30 新增。這一節是整份文件（以及 notebook、實作紀錄、研究計畫）
> 對「某個策略行不行」的唯一用語來源。**

Day 10–11 的 negative result 大多是**條件性**的。把它們寫成「機器人做不到」是
**過度推廣**，而且會誤導後續的四腳階段——那正是最可能把它們解掉的地方。

```text
COMPOSED
    目前的單腿 2D planner 已經能生成完整、collision-free 的 sequence。

DIRECT_HANDOFF_INFEASIBLE
    目前的兩個接觸 primitive【無法直接連續交接】。
    這是一個關於【現有 primitive 集合】的敘述，不是關於機器人的。

REQUIRES_MULTILEG_REPOSITION
    可能需要其他腿支撐、本腿再次離地重新就位（見 §5.6 的 TOP_REPOSITION）。
    【尚未驗證】——單腿 2D 模型沒有能力驗證它。

PHYSICALLY_INFEASIBLE
    只有在具備 reach / joint limit / collision / 接觸與支撐的證據時才能使用。
    Day 10-11 目前【沒有任何一格】夠格用這個標籤。
```

**這些標籤的關係是層級，不是互斥的同義詞。** 一個 cell 可以同時是
`DIRECT_HANDOFF_INFEASIBLE` 和 `REQUIRES_MULTILEG_REPOSITION`；只有
`PHYSICALLY_INFEASIBLE` 需要額外證據才能升格。

> **2026-08-30 實作時補上的兩個標籤 —— 而且補的理由值得記下來。**
>
> 上面四個是針對 `#2` / `#3` 那個問題寫的。第一版實作把它們套到**整張地圖**，
> 結果 19 個拒絕**全部**被標成 `DIRECT_HANDOFF_INFEASIBLE`——
> 但 `#1` 在短頂面上不行**不是交接失敗**，是跑完了地形。
>
> **那是同一種過度推廣，只是方向相反。** 所以補兩個：
>
> ```text
> OUT_OF_ENVELOPE
>     策略本身沒問題，是【這塊地形】超出它的可用範圍或超出已掃範圍。
>     #1 頂面太短、#5 stride 太長都屬於這一類。
>     不是交接失敗，也不是機器人的極限。
>
> NOT_MEASURED
>     沒有人量過這一格。「沒人看過」不是一種「不行」。
> ```
>
> `StrategyCell2D.effective_verdict` 因此改成**依 `Availability` 對照**而不是
> 二分猜測，而且**永遠不會推導出 `PHYSICALLY_INFEASIBLE`**。

### 用這套用語重述 `#2` 與 `#3`

前面幾節（§2.6、Step 2b、Step 3、Step 5、Step 7、Step 8）都寫了「被推翻」。
**那個措辭要照下面這樣讀：**

```text
#3 SWING_UP + ROLL_DOWN
    DIRECT_HANDOFF_INFEASIBLE
        swing 落在 obstacle top 時通常是 foot_rim, alpha ~ 0 deg，
        而 roll down 的入口要 left_rim、靠近後緣、滿足 LEFT_RIM_READY。
        現有 planner 嘗試直接把兩個接觸狀態連起來 -> 必須穿越 alpha = -40 deg
        的 rim 接縫，出現約 29 deg 的關節跳躍。
    REQUIRES_MULTILEG_REPOSITION
        未來可能的路徑：SWING_UP -> foot_rim 在頂面安全落地
                     -> TOP_REPOSITION（其他腿支撐）-> LEFT_RIM_READY -> ROLL_DOWN
    【不是】PHYSICALLY_INFEASIBLE。

#2 ROLL_UP + SWING_DOWN
    DIRECT_HANDOFF_INFEASIBLE
        (a) 直接從 right_rim 交接 swing -> 撞 alpha = +40 deg 的 rim 接縫
        (b) 改用 Day 6-7 的 retract   -> 它固定停在 theta = 17 deg，
                                        而 swing down 需要 theta >= 35 deg
    兩條可能的修法（(A) 純單腿、(B) 需要四腳）：
        A. 新增 RETRACT_FOR_SWING_DOWN —— 只收到 theta >= 35 deg 的合法起飛姿態
        B. 在四腳支撐下使用 TOP_REPOSITION —— 解除 right_rim contact、空中重新展延
    【不是】PHYSICALLY_INFEASIBLE。而且 (a) 那條牆是 rim 分段的模型性質，
    (b) 那條是 Day 6-7 為 roll down 設計的介面被借用到 swing down 上。
```

> **Step 8 量出的 2.7 倍代價要跟著這個用語一起讀。**
> `h = 160 mm` 上 roll-up 對 body 只要 80.1 mm、被迫改用的 swing-up 要 220.0 mm。
> 那個數字說的是**修好 `#2` 值多少**，不是「機器人在那裡只能付 220 mm」。

## 5.6 `TOP_REPOSITION`：這次討論找到的新缺口

> **2026-08-30 新增。目前【尚未實作】，也不打算在 Day 10–11 實作。**

腿越過 obstacle 頂面後，**真實四腳機器人不一定要用同一個接觸狀態接續下一段**。
當其他腿可以支撐 body 時，這條腿可以再次離地、在空中調整 `theta` / `beta` 與落點，
再以更適合 `ROLL_DOWN` 或 `SWING_DOWN` 的姿態落地：

```text
頂面安全落地
    -> 由其他腿承重
    -> 本腿再次離地
    -> 空中調整 theta / beta / rim target
    -> 落地到對下一段更有利的位置
```

命名：`TOP_REPOSITION`（或 `CONTACT_RECONFIGURATION_SWING`）。

### 為什麼**不**在 Day 10–11 實作

目前是 single-leg 2D model。如果唯一的腿已經在 obstacle top 承重，然後它再次離地，
單腿模型**回答不了**這個問題：

```text
離地期間是誰支撐 body？
```

若現在直接生成那條空中軌跡，就等於**默默假設 body 被其他腿支撐**——而單腿 planner
並沒有支撐多邊形或 gait timing 可以驗證這件事。**那會是一個未被驗證的假設被寫成結果。**

### 現在的正確做法

```text
1. 不在 Day 10-11 強行產生 top reposition 軌跡。
2. 不猜測它的 theta / beta / duration。
3. 保留一個【未解的 transition requirement】（見下）。
4. 交給後續四腳 gait / timing planner 確定何時有其他腿可以支撐。
```

### `TransitionRequirement2D`（Step 9 之前要加進 schema 的小型資料）

```text
TransitionRequirement2D
    kind                       TOP_REPOSITION | ...
    source_contact             離開時的接觸狀態
    target_condition           下一段需要的入口條件（例如 LEFT_RIM_READY）
    requires_external_support  True
    resolved                   False
    evidence                   為什麼直接交接不成立（指向 Step 2b / Step 3 的量測）
```

**現階段只記錄「需要這個 transition」，不產生未驗證的軌跡或時間。**
它和 §5.4 的 `MotionSegment2D` 是兩種東西：一個是**已生成的動作**，
另一個是**已知但未解的需求**。`MotionSequence2D` 要能同時裝下兩者，
否則 Step 9 的交接物會把「還沒解」和「不存在」混為一談。

---

# 6. 已知會咬人的三件事

## 6.1 two-ceiling 搜尋是貪婪的，而且對高度不單調（已實測）

2026-08-29 實測，`sample_count=31, arc_samples=61`：

```text
140 mm  onto:  OK    hip +40 mm                        clearance 1.05 mm
150 mm  onto:  FAIL  theta pinned at limit             （hip 只升到 +40 mm 就停了）
160 mm  onto:  OK    hip +60 mm, liftoff +30 mm        clearance 1.37 mm
```

機制在 `_run_step_swing_showcase`：

```text
ceiling 1   沿 hip_offset_ladder 升階，用 _reachable() 這個便宜的 Steps 3–5 探針判定，
            一旦探針說「搆得到」就【停止升階】
ceiling 2   repair_swing_2d 為了閃過障礙把 liftoff 抬高
            -> 反而把 reach 打壞
            -> 但流程【不會回頭】再升 hip
```

150 mm 就落在這個縫裡：探針在 hip +40 mm 說可以，repair 加了 20 mm liftoff 之後 IK 的 θ 撞到下限。

**後果**：sweep 如果直接沿用這個 ladder，map 上會出現純屬搜尋順序的假洞，而假洞會直接汙染 decision rule。Step 2 必須改成在 `(hip_lift × liftoff_rise)` 的完整格點上取最小可行組合，而不是沿用貪婪 ladder。

這個 bug 本身也值得留在 paper 的方法一節：它說明了為什麼 concession 必須被定義成「最小可行讓步」，而不是「搜尋程序碰巧停在哪」。

## 6.2 毫米級餘裕

showcase 的 `min_clearance` 全落在 1.0–1.8 mm，而 contact tolerance 是 1 mm、rim 幾何差異是 1.2 mm。任何「剛好可行」的 cell 都不該被當成可行——Step 5 的 decision rule 要有 margin 門檻，不能只看 `feasible == True`。

## 6.3 α 在 rim 接縫上會瞬移

Day 8–9 §26.5(2)：±40° 跳 45 mm、±180° 跳 162 mm。Strategy B 的交接點（roll 結束 → swing 起始）正好落在 upper rim 上，很可能靠近接縫。Step 7 串接時要明確檢查交接點離接縫多遠。

> **2026-08-30 Step 6 量到的修正**：那個 162 mm 是**同一個 rim 參數化下**的值。
> Step 10R 實際跨 ±180° 接縫時（`WHEEL_MODE_TOP_ROLL -> LEFT_RIM_READY`），
> `alpha` 從 `179.4` 變成 `-179.4`（未折疊看起來跳 358.8 度），
> 但**接觸點只移動 2.9 mm** —— 換到另一個 rim 之後兩者在物理上幾乎重合。
> 那是**換座標卡**，不是運動。`handoff_report_2d` 因此把 alpha 差折進 `(-180, 180]`。
>
> 同一份報告量到：整條 traversal 的最大**接觸點**跳躍是 180.5 mm（foot → right rim），
> 但最大**關節**跳躍只有 `theta 1.00 deg` / `beta 1.75 deg`，都在一個取樣步長以內。
> **接觸點大跳不是不連續；關節大跳才是。**

> **2026-08-30 Step 7 把這一節的擔心量成了數字，而且它是真的。**
> 在地圖允許的**最短**頂面上（`#1`, h = 140 mm, L_top = 225 mm），
> `LEFT_RIM_READY` 只有 **1 幀**，接著 88 幀的下降全部把 `alpha` 釘在 `-178.83 deg`
> ——**距離 ±180 度接縫 1.17 度**。對照 `L_top = 350 mm` 的同類 traversal，
> 交出去之後還有 **45 度**的 left-rim 弧可用。
>
> ```text
> => #1 的頂面下界，實質上就是【left-rim 弧預算歸零】的那一點。
>    跨過去時接觸點只移動 5.9 mm（交接本身乾淨），但沒有任何餘裕：
>    取樣改變只要讓接縫位置移動超過 1.17 度，這個交接點就會跑掉。
> ```
>
> **Day 12 之後若要動 `arc_samples`，這一格要重測。**

---

# 7. 實作步驟

每一步都可以獨立交給 Claude。順序有依賴，不要跳。

## Step 0 — Scene / approach 幣別對齊

### 目的

讓 rolling 與 swing 的 cell 指的是同一件事。這一步不做，後面所有疊圖都是假的。

### 任務

1. 建 `day10_11_shared_scene_2d.py`，提供單一入口：給 `(h, L_top, c)` 回傳 rolling 與 swing 兩邊都能用的 scene 參數。
2. approach 一律以**前緣餘裕 c** 表示，重用 `_hip_x_for_start_clearance` 做 c → hip_x 的轉換（把它從 `right_up_left_down_sweep_2d` 提升成共用函式，不要複製）。
3. 障礙幾何統一到 `(x_start = 0.10, width = L_top)`。
4. 把 `day6_7_roll_end_swing_start_2d()` 一般化成任意 `(h, θ_climb)` 的 roll-end → swing-start 轉換。
5. 量交接誤差：roll 終點的 contact state 餵進 swing 的 IK，記錄位置殘差、α 誤差、以及 1.2 mm rim 幾何差異在這個點上實際貢獻多少。

### 輸出

```text
day10_11_step0_scene_alignment.csv
    h, theta, c, hip_x_roll, hip_x_swing, hip_z_roll, hip_z_swing,
    handoff_position_error_m, handoff_alpha_error_deg, rim_geometry_delta_m
```

### 完成標準

- 同一個 `(h, θ, c)` 在兩個模組建出來的 standing scene，hip pose 差 < 1e-9 m。
- roll-end → swing-start 的交接誤差被量化並寫進 CSV（**不要求為零**，要求被記錄）。
- 一般化的轉換函式在至少 5 組 `(h, θ)` 上不拋例外。

> **2026-08-30 完成。** 80/80 standing cells 的 hip_z 差恰為 1 nm 的 `surface_offset`；
> 60 個 stage 出口中 52 個抵達，全部重建成功。
>
> **並且多了一個沒有規劃、但比對齊檢查更有力的證據**：交接量測必須跑完整 traversal，
> 所以順帶對每個 cell 產生了可行性判定，拿去和 Day 6–7 已完成的 map 對照——
> **20/20 逐格相同**，包含兩處非單調（`h = 120 mm` 在 40/50/60/70/85 全部不可行；
> `h = 140 mm` 只有 40 與 70 可行）。對齊檢查證明兩邊「建出同一個姿態」，
> 這張表證明兩邊「跑出同一個結論」，連地圖上的洞都對得起來。
>
> 另外量到一個結構性事實：**`WHEEL_TRANSITION` 的出口停在哪個 rim，與整條 traversal
> 可不可行完全相關**（left_rim → 可行 12/12，right_rim → 不可行 8/8）。這正是策略 #3
> 要繞過的那一格，也是把 Step 2b 提前的理由。

---

## Step 1 — Concession 契約

### 目的

把「代價」從 `adjustments` 字串變成可比較的數值。

### 任務

1. 定義 §5.1 的 `SwingConcession2D` 與 §5.2 的 `RollConcession2D`。
2. 寫 `from_showcase(StepSwingShowcase2D) -> SwingConcession2D`，**無損**——現有 showcase 的每一個 `adjustments` 字串都要能被解析成欄位，不能有資訊只存在字串裡。
3. `binding_ceiling` 由 showcase 的失敗路徑推出（reach 分支 vs repair 分支）。

### 完成標準

- Day 8–9 §28.1 那 7 個高度的 showcase 全部能轉成 `SwingConcession2D`，且回轉的 summary 字串與原本一致。
- 沒算過的欄位是 `None`，不是 0（沿用 Day 8–9 Step 1 的契約原則）。

---

## Step 2 — 上升側 swing sweep：`height × approach clearance`

### 目的

補上 2.3 那一維。這是 Day 10–11 的主產出之一。

### 任務

1. **不要沿用貪婪 ladder**（見 §6.1）。每個 cell 在 `(hip_lift × liftoff_rise)` 的完整格點上求最小可行組合。
2. 網格建議：

```text
h  ∈ {20, 40, 60, 80, 100, 120, 140, 150, 160, 180, 200} mm    <- 150 一定要在裡面
c  ∈ {0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16} m
hip_lift    ∈ {0, 20, 40, 60, 80, 100, 120} mm
liftoff_rise∈ {0, 10, 20, 30, 50} mm
```

3. 平行化沿用 `right_up_left_down_sweep_2d` 的 `_run_one_cell` + `ProcessPoolExecutor` 模式與 picklable settings bundle。
4. 每個 cell 輸出一列 `SwingConcession2D`。

### 輸出

```text
day10_11_step2_swing_onto_sweep.csv
day10_11_step2_min_hip_lift_map.png        # heatmap，軸 = (h, c)，值 = min_hip_lift
day10_11_step2_binding_ceiling_map.png     # 哪一道天花板在管
```

### 完成標準

- 能重現 §26.5(6)：固定 h，小 c 不可行 / 大 c 可行，且轉折點附近 c 與輪半徑 0.145 m 的關係被明確寫出來。
- **150 mm 那一格不再是洞**（若仍是洞，必須證明它是幾何造成的，不是搜尋造成的）。
- 200 mm 在**所有** c 下是否可行，有明確答案。
- 每個 cell 的 `binding_ceiling` 有值。

> **2026-08-30 完成，並且第一條的後半被數據推翻。** 88 cells、872 次 `generate_swing_2d`、708 s。
>
> ```text
> [成立] clearance 軸確實會翻轉 cell
>        h = 150 / 160 / 180 mm 在 c = 20 mm 失敗、c = 40 mm 成功
>        h = 200 mm 在 c = 20 / 40 mm 失敗、c = 60 mm 成功
>
> [成立] 對高度單調，沒有洞。150 mm 的貪婪假洞【沒有】在格點搜尋下重現
>        每個 clearance 的高度天花板：
>            c = 20 mm  -> 140 mm
>            c = 40 mm  -> 180 mm
>            c >= 60 mm -> 200 mm（掃描範圍上限）
>
> [成立] 200 mm：c >= 60 mm 可行；c = 20 / 40 mm 為 TERRAIN_COLLISION
>
> [成立] 88/88 cells 的 binding_ceiling 有值；0 個帶貪婪簽名；全部 grid_minimum
>
> [推翻] 轉折點【不】在輪半徑附近。實測 h = 150 mm：
>            c =  20 mm -> 接觸點到前緣 176 mm
>            c = 160 mm -> 接觸點到前緣 319 mm
>        整條軸上這個距離從未低於 145 mm，但 c = 20 mm 仍然失敗。
>        所以 §26.5(6) 提出的「輪半徑 > 接觸點到障礙距離」在這個幾何下不是操作中的機制。
> ```
>
> **真正的機制由「哪個旋鈕修得好它」指出來**：修好小 c 的旋鈕是 `liftoff_rise`——
> `c = 20 mm` 時 `h >= 60 mm` 每一格都需要 50 mm 的 liftoff rise，而 `c >= 60 mm` 全部降為 0。
> liftoff rise 是在 swing **起始**把腿垂直抬起的旋鈕，所以問題是**腿體在還貼著前緣時的掃掠體積**，
> 不是靜態的接觸點幾何。
>
> **另一個沒預期到的結構**：clearance 軸在 `c ≈ 60 mm` **飽和**——超過之後再多的空間買不到任何東西
> （`c = 60 … 160 mm` 每一列的 min hip lift 完全相同）。這對 Step 5 有直接影響：
> approach clearance 不是一個連續的代價軸，而是一個**有門檻的二元條件**
> （「夠不夠 60 mm」），門檻之上 body 讓步就只由高度決定。

### Step 2 close-out：主 sweep 固定住、但沒有論證過的三件事

主 sweep 回答了「一個 swing 上台階要付多少」，但它同時**固定**了三件事而沒有說明為什麼可以固定。
每一件都會讓那張地圖變成有條件的，而 Step 5 會默默繼承那個條件。
`day10_11_step2_closeout_driver.py`（48 cells、961 次 `generate_swing_2d`、987 s，clearance 一律
取飽和後的 80 mm，所以變動的只有被測的那一項）。

> **2026-08-30 完成。三段都有明確答案，其中兩段修正了主 sweep 的結論。**

**A. 真正的高度天花板是 240 mm，而且 clearance 的飽和點會隨高度上移**

```text
c =  60 mm   最高可行 200 mm      （220 mm 起失敗）
c = 100 mm   最高可行 240 mm
c = 160 mm   最高可行 240 mm      （與 100 mm 相同 -> 飽和點在 100 mm）
260 / 280 / 300 mm  三個 clearance 全部失敗，binding_ceiling = reach
```

兩個結論：

```text
1. 天花板 240 mm，而且是 reach 型的 —— 腿【搆不到】，不是【放不下】。
   這是幾何硬上限，加 clearance 無效。

2. 「c ≈ 60 mm 飽和」只在 h <= 200 mm 成立。
   h = 220 / 240 mm 時 c = 60 失敗、c = 100 成功 -> 門檻升到 100 mm。
   Step 5 的門檻條件必須寫成 c_threshold(h)，不能寫成常數 60 mm。
```

**B. landing distance 在天花板以下不影響答案，在天花板附近才變成綁束**

```text
min hip lift [mm]，c = 80 mm
    h \ landing   0.10   0.13   0.16   0.19   0.22
    100 mm           0      0      0      0      0
    150 mm          40     40     40     40     40
    200 mm          80     80     80    100      x
```

所以主 sweep 固定 `landing = 0.16 m` 是**可以的**，但理由不是「landing 無所謂」，而是
「landing 在天花板以下無所謂」。h = 200 mm 已經接近 240 mm 的天花板，那裡它就開始咬了。

**C. theta 不是中性的參數 —— 它是 swing 最強的內部自由度，而 60° 只在矮台階是最佳**

```text
min hip lift [mm]，c = 80 mm、landing = 0.16 m
    h \ theta      40     50     60     70     85
    100 mm         80     40      0      0      0
    150 mm        120     80     40     20      0
    200 mm          x    120     80     40      0
```

`min_hip_lift` 是相對於「該 theta 的標稱站姿」量的，所以直接比會誤導。換成 Step 5 真正要的
**絕對幣別**（落地瞬間 hip 在台面上方多高）之後：

```text
落地時 hip 距台面高度 [mm]
    h \ theta      40     50     60     70     85
    100 mm      263.6  241.5  219.4  237.5  264.9      <- 最小在 theta = 60
    150 mm      303.6  281.5  259.4  257.5  264.9      <- 最小在 theta = 70
    200 mm          x  321.5  299.4  277.5  264.9      <- 最小在 theta = 85
（標稱站姿本身：17° = 145.0、40° = 183.6、50° = 201.5、60° = 219.4、70° = 237.5、85° = 264.9 mm）
```

**最佳 theta 隨高度上移，而且是內部極小值（不是端點）。** 這對 Day 10–11 有兩個直接後果：

```text
1. 主 sweep 的 map 是【上界】，不是 swing 的代價。
   h = 200 mm 它報 299.4 mm，真正的最小是 264.9 mm —— 高估了 34.6 mm。

2. 這正是 §3.1 自己寫的規則：「每個 primitive 對自己的內部自由度取最佳」。
   Step 2 沒有對 theta 取最佳，Step 5 疊圖之前必須補。
   注意這【不】推翻 §3.1 否決 sweep (ii) 的理由：theta_climb 與 theta_liftoff 仍然不是
   同一個物理量，不該畫在同一根軸上。它們是各自 primitive 的內部旋鈕，該被【取最佳掉】，
   不是被【當共用軸】。close-out 反而讓這個區分更站得住腳。
```

---

## Step 2b — SWING_UP 能不能落在 `LEFT_RIM_READY`

### 目的

回答 §2.8 那三個子問題。這一步決定策略 #3（swing up + roll down）到底存不存在。
**它是整個 2×2 裡唯一一格目前完全沒有數據的。**

### 任務

1. 對每個 `h`，把 swing 的 target ContactState 設成 Step 9R 下降所需的 `LEFT_RIM_READY`：

```text
theta_touchdown = 17°
rim             = left rim
alpha           = Step 8R 抵達後緣時實測的接觸 alpha
position        = obstacle top，距後緣 d_corner
surface         = obstacle top
```

2. 掃 `d_corner ∈ {0.02 … 0.16} m` 與 approach clearance `c`，記錄：
   - IK 是否可行，以及需要的 `hip_z`（這是一筆 body 讓步，要進 `SwingConcession2D`）
   - 落點 alpha 離 ±180° 接縫多遠
   - 落地後直接接 Step 9R 的 trailing-edge pivot 是否 collision-free
3. **同時跑退化版本**：若無法直接落在 17°，改成「落在較伸展的姿態 → retract 到 17° → 再 roll down」，
   量這個 retract 需要吃掉多少 top length。

### 輸出

```text
day10_11_step2b_swing_to_left_rim_ready.csv
day10_11_step2b_alpha_seam_distance.png
```

### 完成標準

- 明確回答 §2.8 的 (a)(b)(c) 三題，每題有數值。
- 若 #3 成立：給出 `SWING_UP + ROLL_DOWN` 的最小 top-length 預算，並與 `L_transition` 的
  0.20–0.27 m 比較。
- 若 #3 不成立：給出退化版本的預算，並明確記錄是哪一個子問題擋住的
  （這是 negative result，要留在 note 裡）。

> **2026-08-30 完成。(b) 與 (c) 成立且有數字；(a) 不成立。**
> `day10_11_step2b_driver.py`：A 窗口 + B 落地 (25 cells) + C top-length (35 cells)
> + D 診斷 (15 cells)，1283 s。approach clearance 一律 100 mm（Step 2 close-out A 的門檻之上）。

### (b) 成立：left rim 可達，而且窗口寬得不像勉強

```text
theta = 17 deg 時 beta in [-312, -180] deg 使 left rim 承接接觸
    寬 132 deg、連續、【四個高度完全相同】-> 這是【腿】的性質，不是台階的性質
    rolling ascent 自己抵達的 beta in [-238.2, -212.7] 完全落在窗口內
    -> swing 被要求去的，正是 roll 自己會經過的狀態
    hip 被釘在台面上方 143.80 mm，而且每個 beta 都一樣（17 度的腿確實是個圓）
```

### (c) 成立：接縫不是硬阻擋，它是一個 1:1 的兌換率

rim budget（接觸點往 `alpha = -40 deg` 方向剩下的弧，也就是 corner pivot 能花的）
與 seam margin 是 1:1 互換的。落地選擇規則沒有自由參數：**先買足 pivot 需要的 budget，
再最大化 seam margin**。

```text
height | pivot 需要 | 選到的 beta | 落地 alpha | rim budget | seam margin | Step 9R 下得去嗎
    60 |   54.1 deg |   -250 deg |   -110.0 deg |   70.0 deg |    70.0 deg | 可以
   100 |   71.9 deg |   -248 deg |   -112.3 deg |   72.3 deg |    67.7 deg | 可以
   140 |   88.0 deg |   -231 deg |   -128.7 deg |   88.7 deg |    51.3 deg | 可以
   160 |   95.9 deg |   -223 deg |   -136.8 deg |   96.8 deg |    43.2 deg | 【不行】
   200 |  112.3 deg |   -207 deg |   -153.2 deg |  113.2 deg |    26.8 deg | 【不行】
```

**一個沒預期到的結果：`LEFT_RIM_READY` 是必要條件，但對【空中來的】抵達不是充分條件。**
35 個 top-length cell 的落地姿態 **35/35 全部通過 `LEFT_RIM_READY`**，但其中 14 個
（h = 160 / 200 mm 的全部）Step 9R 下不去，失敗於
`LEFT_RIM_ROLL_DOWN / NO_LEGAL_CORNER_PIVOT_CONTINUATION`。
而且這【不是】rim budget 不夠：h = 140 mm 只多 0.6 deg 就成功，h = 160 mm 多 0.9 deg 卻失敗。
所以一階的 `arccos(1 - h/R)` 預測不是真正的綁束條件。
**下降側對「swing 放上去的抵達」有自己的高度天花板，落在 140–160 mm 之間。**

### (a) 不成立，而且三個先驗上合理的解釋都被排除了

```text
[排除] 不是 top length      L_top ∈ {0.08 … 0.35} m 每一個都失敗（35/35）
[排除] 不是 hip travel      最短只要 0.28 m；Step 2 成功走過 0.38 m
[排除] 不是落地姿態         35/35 全部通過 LEFT_RIM_READY
[排除] 不是 approach 姿態   section D：theta ∈ {40…85} × c ∈ {60,100,160} 全部 15 格失敗
```

**真正的機制：`LEFT_RIM_READY` 要求腿處在自己的機械極限。**
`legwheel/config`：`MIN_THETA_DEG = 17.0`。也就是說 `theta = 17 deg` 不是一個普通姿態，
而是**這隻腿能收到的最短狀態**。

```text
IK 的原話：「theta is pinned at its limit, so the target is out of reach」
飛行中段要把接觸點抬過障礙，IK 想把腿再縮短一點 —— 但 17 度已經是最短。
31 個取樣只有 11 個收斂，最大殘差 40 mm。
```

**滾上去的腿可以到 17 度，因為它是【已經被頂面支撐著】慢慢收縮過去的；
swing 卻必須在【還懸在角落上空】時就已經在 17 度。**

### 把兩個候選機制分開：關節步長是取樣假影，reach 才是真的

```text
samples=31,  0.6s -> max_step 11.7/10 deg, speed 583   IK_NOT_CONVERGED  ik_ok 11/31
samples=61,  0.6s -> max_step  5.8/10 deg, speed 580   IK_NOT_CONVERGED  ik_ok 21/61
samples=61,  1.2s -> max_step  5.8/10 deg, speed 290   IK_NOT_CONVERGED  ik_ok 21/61
samples=121, 1.8s -> max_step  2.8/10 deg, speed 189   IK_NOT_CONVERGED  ik_ok 41/121
```

加密取樣、放慢擺動，**關節步長與速度的違規完全消失，失敗卻一點都沒變**，
而且 IK 收斂比例固定在 1/3 —— 這是【路徑】的幾何性質，不是離散化造成的。

### 缺的是什麼：一個【中段拱起、端點釘死】的 hip 軌跡

把起訖兩端的 hip 同時抬高（`HipTrajectory2D` 是直線，這是它唯一允許的「拱起」）：

```text
hip +  0 mm   ik_ok 21/61   IK_NOT_CONVERGED
hip + 20 mm   ik_ok 39/61   JOINT_DISCONTINUITY
hip + 40 mm   ik_ok 61/61   JOINT_DISCONTINUITY   <- reach 問題【完全消失】
hip + 60 mm   ik_ok 61/61   JOINT_DISCONTINUITY
hip + 80 mm   ik_ok 57/61   JOINT_DISCONTINUITY
```

**抬高 40 mm，整條接觸路徑就全部搆得到了。** 所以擋住 (a) 的第一件事，
精確地說是「飛行途中 hip 比那條直線低了約 40 mm」。

但端點被釘死在 143.8 mm（`PINNED`），**一條直線沒辦法「中間高 40 mm、端點又剛好」**。

```text
=> 策略 #3 對 body 的需求不是 PINNED，也不是 LOWER_BOUND，
   而是【端點釘死 + 中段拱高 ~40 mm】—— Day 8-9 的 HipTrajectory2D 表達不出來。
   這是 concession 框架本來就該長出來的東西：primitive 提出一個目前 body model
   無法表示的需求，那就是 Day 12 的輸入，不是 Day 10-11 的失敗。
```

**誠實的界線**：抬高 hip 讓 reach 問題消失，但 `JOINT_DISCONTINUITY` 仍在。
所以「中段拱起」被證明是**必要**的，**沒有**被證明是充分的——因為還有第二道牆。

### 第二道牆：alpha = -40° 的接縫是真的不連續，而且【任何】left-rim 落地都要穿過它

reach 問題修好之後（hip +40 mm，IK 61/61 全收斂），`JOINT_DISCONTINUITY` 沒有消失。
把取樣一路加密：

```text
samples= 61   max joint step 24.3 deg
samples=121   max joint step 27.5 deg
samples=241   max joint step 28.5 deg
samples=481   max joint step 29.3 deg
```

**加密不會讓它變小，反而收斂到約 29 度。** 這是真的不連續，不是離散化假影。
而且位置就在接縫上，一步跨過去：

```text
sample 42:  foot_rim  alpha = -39.3 deg   ->   left_rim  alpha = -40.3 deg
            關節跳 23.6 deg（第二大的一步只有 2.7 deg）
```

`alpha = -40 deg` 正是 foot rim 與 upper_tyre_l 的邊界，也就是 Day 8–9 §26.5(2)
量到接觸點會瞬移 45 mm 的那道縫（另一道是 ±180 deg 的 162 mm）。

**這件事的射程比策略 #3 大得多。** 站姿的接觸永遠在 `foot_rim, alpha = 0`
（Step 0 的 80/80 都是），所以**任何**要落在 left rim 的 swing 都必須把接觸
從 foot rim 遷移到 left rim，也就必須穿過這道縫。這不是「17 度太極限」的問題，
是 rim 分段本身的問題。

### 退化版本也不成立：問題不在 17 度，在 left rim

規格要求「若 #3 不成立，給出退化版本的預算」。退化版本是「落在較伸展的姿態 →
retract 到 17° → 再 roll down」，所以把落地 theta 當成軸掃一次
（`--only degenerate`，15 cells，`L_top = 0.20 m`、`d_corner = 20 mm`）：

```text
每個落地 theta 的 left-rim 窗口都存在，而且 hip 一樣是被【釘死】的：
    theta_land = 17 deg   窗口 132 deg   hip 釘在台面上方 143.8 mm
    theta_land = 25 deg   窗口 126 deg   hip 釘在           147.5 mm
    theta_land = 35 deg   窗口 120 deg   hip 釘在           152.2 mm
    theta_land = 45 deg   窗口 120 deg   hip 釘在           157.1 mm
    theta_land = 60 deg   窗口 110 deg   hip 釘在           164.8 mm

swing 可行嗎：
    height |   17    25    35    45    60   deg
        60 |    -     -     -     -     -
       100 |    -     -     -     -     -
       140 |    -     -     -     -     -
```

**放寬到 60 度也一樣落不上去。** 失敗模式是 `IK_NOT_CONVERGED` 與
`JOINT_DISCONTINUITY` 混合（60 度時以後者為主），另有 obstacle front / top 的穿透。

> 註：E 段的失敗模式是混合的，不能全部歸給接縫。接縫的**乾淨證據**是上一節那個
> 專門的探針——先用 +40 mm 把 reach 問題移除，再把跳躍定位在 `alpha = -40 deg`
> 這一步上，並證明它在加密下收斂到 29 deg。E 段證明的是**退化版本沒有救**。

### 結論：#3 在目前的 planner 下不成立，而且原因可以指名道姓

```text
swing planner 目前只能交出【foot rim】的落地（站姿接觸永遠是 foot_rim, alpha = 0，
而遷移到 left rim 要穿過 alpha = -40 deg 那道 29 deg 的不連續）。
=> SWING_UP + ROLL_DOWN 仍然需要在頂面上做一次 rim handover，
   只是從 right->left 換成 foot->left。
=> 【#3 並沒有躲掉 L_transition，它只是把它換了個名字。】
   §2.6 那個「swing 可以直接落進 LEFT_RIM_READY，跳過 L_transition」的論證不成立。
```

這是一個 **negative result，但它有明確歸屬**：擋住 #3 的是 rim 分段的接縫這個
**模型性質**，不是機器人做不到。等 rim 之間的接觸連續化（或 IK 允許跨段延續）之後，
這一格要重測。Step 5 的五個候選在此之前先降為四個（Step 3 推翻 #2 之後再降為三個）。

---

## Step 3 — 下降側 swing sweep：`height × takeoff distance`

### 目的

下降側是 §26.7 說的切換點所在，而且 takeoff distance 與 rolling 的 `top_length` 是同一個物理量，是兩張 map 的天然橋樑。

### 任務

1. 用 `swing_off_step_2d`，body knob 是 `hip_hold_ladder`（台階高度的比例）。同樣改成完整格點，不用貪婪 ladder。
2. 網格：

```text
h        ∈ 同 Step 2
takeoff  ∈ {0.08 … 0.24} m，step 0.02      <- 對應 rolling 的 top_length 座標
hip_hold ∈ {0, 0.25, 0.5, 0.75, 1.0}
```

3. **起始狀態要分兩種**（§2.7 的耦合）：SWING_DOWN 的起點取決於怎麼上來的。

```text
arrival = ROLL_UP    起點 = roll-up 出口狀態（theta_climb 決定，位置由幾何決定）
arrival = SWING_UP   起點 = 任意可行落點（IK 範圍內自由）
```

兩種都要掃，因為它們對應 §2.7 表裡的不同列，最小 top-length 預算也不同。

> **2026-08-30 實作時發現的兩件事，寫在跑之前**
>
> **(i) 兩種 arrival 的掃描【軸】不同，不是同一張表換個起點。**
> Step 0 的 20 個 ROLL_UP 出口**全部在 right rim**，θ = θ_climb、β ≈ -66…-72°，
> 而且 hip 只在前緣往前約 30 mm。所以 `arrival = ROLL_UP` 的 takeoff distance
> **不是自由度**——它等於 `L_top - (出口位置 - 前緣)`。那一列要掃的是 `top_length`，
> takeoff distance 是導出量。`arrival = SWING_UP` 才能把 takeoff distance 當軸。
>
> 順帶：這讓 Step 2b 的問題有了鏡像版本。Step 2b 發現要**落在** left rim 會被
> `alpha = -40°` 的接縫擋住；`ROLL_UP` 的下降是從 **right rim 出發**去搆一個
> foot-rim 落地，跨的是 `+40°` 那道縫。同一道縫的另一邊，值得一起量。
>
> **(ii) 下降需要一個上升不需要的旋鈕：`swing_duration`。**
> 實測 `h = 160 mm`、takeoff 0.16 m：在 (hold × drop) 的 45 格上**全部失敗**，
> 而且失敗原因一律是 `TOUCHDOWN_VELOCITY_TOO_HIGH`。
> 對照 Step 2 / close-out / 2b 三份 CSV，上升側 **18 個不可行 cell 沒有一個**是
> 這個原因（全是 `TERRAIN_COLLISION` 或 reach 類）。
>
> ```text
> 原因：下降的腳帶著整個台階高度的落差抵達下層地面，觸地速度才是綁束條件，
>       而它的旋鈕是【時間】不是幾何。
>       swing_off_step_2d 的 repair 本來就會延長 duration，正是為了這件事。
> => 沒有 duration ladder 的格點會把「只是太趕」的 cell 報成不可行。
>    這會【假性推翻】Day 8-9 docstring 那個 160 mm 的量測 —— 用錯的理由。
> ```
>
> 所以三個 ladder 的巢狀順序是有意義的：
>
> ```text
> for hip_hold:            <- 唯一的【body 讓步】，Step 5 要拿去比的就是它，所以放外層
>     for touchdown_drop:  <- 軌跡整形
>         for duration:    <- 軌跡整形
> ```
>
> hold 在最外層才能保證回報的 `min_hip_hold_fraction` 是真正的最小 body 讓步；
> 另外兩個是「為了讓這個 hold 成立付出了什麼」，不與 hold 互相交易。

4. 特別確認 `swing_off_step_2d` docstring 記的那個量測：160 mm 時 hip 跟著掉會撞、hold 一半有約 1.8 mm 餘裕。

### 輸出

```text
day10_11_step3_swing_off_sweep.csv
day10_11_step3_min_hip_hold_map.png
```

### 完成標準

- rolling 在 `h = 0.16` roll_down 0/10 的那一格，swing 下降要付的 `hip_hold` 有明確數值。
- docstring 的 160 mm 量測被獨立重現（或被推翻並記錄）。
- `arrival = ROLL_UP` 與 `arrival = SWING_UP` 兩種起點各自的最小 top-length 預算都有數值，
  可以直接填進 §2.7 那張表的第 2、4 列。

### Step 3 結果（2026-08-30，六段掃描共 10,305 次 `generate_swing_2d`）

三個完成標準**全部達成**，但第 2、3 條都是以「推翻並記錄」的形式達成的。

```text
段  內容                                    cells   可行
A   map: height x takeoff distance          99      74
B   160 mm claim（兩種取樣各 3 個 hold）      6       0
C   theta 40-85（h = 100/150/200）           15      10
D   arrival = ROLL_UP 直接交接                45      0
E   退化路徑：retract 到 theta = 17/25/30     30      0
F   theta 32/35/38 x takeoff 0.08/0.12/0.16   9       6
```

#### A — 下降側有一個 takeoff distance 的【上限】，形狀與 rolling 相反

```text
h <= 100 mm   takeoff <= 0.24 m       min hold 0.000
h  = 120 mm   takeoff <= 0.20 m       min hold 0.125
h  = 140 mm   takeoff <= 0.18 m       min hold 0.250
h  = 150 mm   takeoff <= 0.16 m       min hold 0.375
h  = 160/180  takeoff <= 0.14 m       min hold 0.375
h  = 200 mm   takeoff <= 0.12 m       min hold 0.500
```

**這是 Step 5 decision rule 真正的第一個鑑別項，而且它不是代價比較。**

```text
rolling    需要 L_top  >= 196-249 mm   （下界，Day 6-7 Step 12R 的 L_transition）
swing off  需要 takeoff <= 120-240 mm  （上限，且隨 h 收縮）
```

兩者是**同一個座標的兩側**：短頂面只有 swing 下得去；長頂面兩者都行，才輪到比 body 讓步。
§2.7 那句「越靠 swing 的組合，需要的 top length 越短」因此**成立，但理由比原本寫的更強**——
不只是「較短」，是**上下界方向相反**。

失敗模式 25 格：`TOUCHDOWN_VELOCITY_TOO_HIGH` 13、`TERRAIN_COLLISION` 12。
前者證實了跑之前寫的 (ii)：duration ladder 是必要的，上升側 18 個不可行 cell 沒有一個是這個原因。
另有 **153 次 planner refusal 分布在 11 格**——Day 8–9 的 planner 在「腳最後停在空中」
（`final_rim=None`）時丟 `ValueError`。這裡把它**記成 refusal 而不去改 planner**，
因為那支 planner 產出了 Step 2 / 2b 的全部結果，不能在 Step 3 中途換掉。

#### B — 160 mm 的 claim 重現不了，但原因指得出來

```text
取樣 (121/31)    hold 0.00 -> -6.76 mm   0.25 -> -3.51 mm   0.50 -> -1.63 mm
取樣 (241/51)    hold 0.00 -> -6.64 mm   0.25 -> -2.28 mm   0.50 -> -0.95 mm
```

**機制成立，操作點不成立。** hold 單調把穿透從 6.8 mm 壓到 1.0 mm；
加密取樣讓數字更接近 0 卻仍未跨過，**所以這不是取樣假影**。
A 段給出原因：`h = 160 mm` 的 takeoff 上限是 **0.14 m**，而 docstring 是在 0.16 m 量的。
**高度沒錯，距離錯了。** 這也順帶推翻了 Day 8–9「off 200 mm 不可行」那句話——
h = 200 mm 在 takeoff <= 0.12 m 是可行的。

#### C — theta 的最佳值隨高度上升（下降側也要取最佳）

換成絕對幣別（touchdown 時 hip 離低地的高度 = 站姿高度 + hold x 障礙高度）：

```text
h = 100 mm   theta 40 -> 271.1   50 -> 239.0   60 -> 219.4*  70 -> 237.5   85 -> 264.9
h = 150 mm   theta 40 -> 333.6   50 -> 314.0   60 -> 313.2   70 -> 293.7   85 -> 283.6*
h = 200 mm   theta 40-85 全部 TERRAIN_COLLISION（在 takeoff = 0.16 m，與 A 段一致）
```

U 形曲線的底隨高度右移：h = 100 在 60°，h = 150 在 85°。
**與上升側同一個模式**，所以 Step 2 close-out C 那條「theta 要取最佳」的要求，
下降側同樣成立，Step 5 疊圖前兩邊都要先各自取最佳。

#### D / E / F — `ROLL_UP + SWING_DOWN` 不成立，而且是【兩道不同的牆】

```text
D  0/45   binding = fit     JOINT_DISCONTINUITY 26 + IK_NOT_CONVERGED 19
E  0/30   binding = reach   IK_NOT_CONVERGED 30（每一個 takeoff 距離都失敗）
F  6/9    theta = 32 失敗；35 可行（hold 1.000）；38 可行（hold 0.875）
```

**D 是 Step 2b 的鏡像。** rolling 交過來的起飛姿態一律在 **right rim**、
`alpha = +82 … +107.7°`，而 `foot_rim = (-40, +40)`——每一格都要跨過 `+40°` 那道接縫。
Step 2b 是落地跨 `-40°`，D 是起飛跨 `+40°`。同一個 rim 分段模型性質，換一邊而已。
這正是跑之前寫的 (i) 所預期的「同一道縫的另一邊」。

**E 完全不是同一回事。** binding ceiling 是 `reach` 不是 `fit`，失敗清一色 `IK_NOT_CONVERGED`。
F 段把原因夾出來：

> **下降側的 theta 下限是 35°，而且與 takeoff 距離無關**（0.08 / 0.12 / 0.16 m 三個距離結果一致）。
> E 段跑的 theta = 17 / 25 / 30° **全部在下限以下**。

**所以 E 是可修的，D 不是。**

```text
Day 6-7 的 Step 6.5 把 retract 的 theta_target 寫死成 17 度，
因為它的目的是【進 wheel mode】給 ROLL_DOWN 用。
但 SWING_DOWN 不需要 wheel mode，它只需要
   (a) 接觸點回到 foot rim，且  (b) theta 停在 35 度以上。

而且這個修法很便宜：
   L_transition = retract_forward (20.5-64.4 mm) + wheel_mode_forward (131.8-228.4 mm)
   停在 35 度就不用付 wheel_mode_forward -> 頂面需求掉 3-10 倍。
```

這是一個具體的**介面需求**，和 Step 2b 的「中段拱高 40 mm」同一類：
primitive 提出一個目前 retract / body 模型表達不出來的需求。**修好之後這一格要重測。**

#### 對 §2.7 那張表的回填

第 2 列（`ROLL_UP + SWING_DOWN`）填不出數值——它在目前的 planner 下**不可行**，
不是「預算很大」。第 4 列（`SWING_UP + SWING_DOWN`）的預算是
`landing_distance + takeoff 上限`，且上限隨高度收縮。

---

## Step 4 — 把 rolling map 換成同一種幣別

### 目的

§5.3 那個問題：rolling 對 body 的要求到底是什麼？這一步可能是整份 note 研究價值最高的一步。

> **2026-08-30 追加定位。** 與指導老師討論後確立的 paper story 是：
> **hybrid 滾走步態相對於一般 walking 的優勢，在於機身質心的變化較小**
> （研究計畫 §10.6）。這讓 Step 4 從「方法論完備性」升格成**這條 story 的關鍵實驗**。
>
> **Step 順序不變**——Step 4 仍在 Step 3 之後，Step 3 正在進行中。變的只是 Step 4
> 要多帶一組質心代理量，見下方任務 4 與完成標準的追加條目。
>
> 為什麼是關鍵：story 的 swing 那半已經有資料（Step 2 的 `min_hip_lift` 字面上就是
> 「質心至少要被抬高多少」，h = 120/140/160/200 mm 分別要 20/40/60/80 mm）。
> **roll 那半就是這一步。** 而 §5.3 已經預埋了風險：rolling 的 hip_z 是輸出而非自由
> 變數，所以 rolling 不是「零質心變化」，是**另一種形狀的質心變化**。若量出來 rolling
> 的起伏更大，story 要改寫成「兩種不同形式的 body 要求之間的取捨」——那仍然是結果，
> 但**在這一步完成之前，這條 story 不能寫成 paper 的結論**。

### 任務

1. 重跑（或從既有 `day6_7_step11r_sweep_trajectories.csv` 重新萃取）每個可行 cell 的 **hip_z 軌跡**。
2. 對每個 `(h, L_top)`，在 θ_climb 上取最佳，填出 `RollConcession2D`：

```text
best_theta_climb_deg
feasible_theta_span_deg      <- 直接來自 day6_7_step11r_feasible_theta_ranges.csv
required_hip_z_range_m       <- 新量的
```

3. 把 `day6_7_step12r_minimum_top_length.csv` 的上下界併進來，讓 `L_top` 成為正式座標軸。

4. **（2026-08-30 追加）質心代理量。** hip 不是質心——質心是整機量，要 Day 12 的四腳
   trajectory 才算得出來。但單腳的 hip_z 軌跡是它的**機制與需求下界**，這一步要把它
   整理成 Day 12 可以直接接的形式：

```text
從每個可行 cell 的 hip_z 軌跡萃取
    hip_z_peak_to_peak_m           峰對峰值
    hip_z_rms_m                    對該段均值的 RMS
    hip_z_per_forward_distance     垂直位移 / 水平前進距離     <- 無因次，主要比較值
    hip_vertical_work_per_dist     單位質量的垂直功 / 前進距離  <- 接到 COT 的那一環

同一組量也要對 swing 側算一次（從 Step 2 / Step 3 的軌跡），否則沒有可比對象
```

`hip_z_per_forward_distance` 是主要值：它無因次，所以 roll 與 swing、
以及不同 (h, L_top) 的 cell 之間可以直接比，不會因為某一段比較長而失真。

> **命名要誠實**：欄位一律用 `hip_*` 而不是 `com_*`。整機 CoM 是 Day 12 之後的量，
> 現在用 `com_` 開頭會讓後續讀者以為這一步已經算出質心。

### 完成標準

- 對 Day 6–7 所有可行 cell 都有 `RollConcession2D`。
- **明確回答**：rolling 的 hip 軌跡約束，和 swing 的 hip 下界，哪一個對 body trajectory 較苛刻？答案是什麼都可以，但必須有數據，不能沿用直覺。

**追加（2026-08-30，質心 story）**

- 每個可行 cell 的四個質心代理量都有值，且 roll 與 swing 兩側都算過。
- **明確回答**：以 `hip_z_per_forward_distance` 比較，roll 是否真的低於 swing？
  在哪些 `(h, L_top)` 成立、哪些不成立？
- 若 roll **更高**，明確寫下來並記錄在哪個區域——那會改寫研究計畫 §25.1 的 story，
  是 negative result，不是失誤。
- 產出一張圖，橫軸 `h`，同時畫 roll 與 swing 的 `hip_z_per_forward_distance`，
  標出交叉點（若有）。這張圖是計畫 §10.6 那條因果鏈在單腳層級的唯一直接證據。

```text
day10_11_step4_hip_excursion.csv
day10_11_step4_hip_excursion_roll_vs_swing.png
```

### Step 4 結果（2026-08-30，七段 A–G，**沒有重跑任何 traversal**）

五個完成標準（原三個 + 質心追加兩個）**全部達成**。

`day6_7_step11r_sweep_trajectories.csv` 已經記了全部 70 個 cell 的 `hip_x_m` / `hip_z_m`，
含 stage / phase / `accepted`。重跑要花約 264 s／cell（交接檔陷阱 5）去重算硬碟上已有的數字。
**唯一是推導而非量測的是 `L_top` 依賴**，而它由資料本身授權：`WHEEL_MODE_TOP_ROLL` 與
`LEFT_RIM_READY` 兩段把 hip 高度保持在 **0.027 mm 以內**，所以加長頂面只加平的前進距離。

#### 前提檢查：三個 sweep 是同一根柱子

```text
Day 6-7 step11r      obstacle_width_m = 0.35,  x_start = 0.10
Step 2 (swing onto)  top_length_m     = 0.35,  x_start = 0.10
Step 3 (swing off)   top_length_m     = 0.35,  x_start = 0.10
```

沒有任何程式強制這件事，所以 `tests/test_day10_11_roll_concession_2d.py` 有一個測試盯著它。
不成立的話，Step 4 產出的每一個數字都會悄悄變成兩塊不同地形之間的比較。

#### B — rolling 的 hip 起伏有一個閉式

```text
hip_z_travel = h + overhead(theta_climb)

theta = 40 deg   overhead = 14.2 - 14.9 mm  在【每一個】高度都一樣
theta = 85 deg   overhead = 49.4 mm
```

**障礙高度是加性的**，`theta_climb` 只透過 overhead 進來；overhead 是 right rim 爬升時
hip 衝過頂面 wheel-mode 高度的那一段。所以取最佳 theta 幾乎總是取最小的可行 theta——
h = 120 mm 是唯一例外（40 deg 在那裡不可行，梳狀窗口，§6.1）。

#### C / D — 誰對 body 比較苛刻？答案取決於量什麼，而兩種問法都對

```text
C  matched stages   ROLL_UP vs SWING_UP、ROLL_DOWN vs SWING_DOWN（兩邊都不含頂面）
                    -> swing 在【每一個】高度都便宜，1.1x 到 2.3x
D  whole obstacle   rolling 完整 traversal vs SWING_UP + 走過頂面 + SWING_DOWN
                    -> swing 贏 h <= 100（只贏 0.4-3.2%）；roll 贏 h >= 120（贏 8.9 / 12.2%）
```

**C 與 D 相反，而且原因很具體**：matched stages 把頂面那段拿掉了，
而**那正是 rolling 免費賺前進距離的地方**（wheel mode 平走，垂直位移 0）。
拿掉它等於只留下 rolling 最貴的兩段。
**回答 paper 的 story 要用 D**，因為 walking 也必須走過頂面。

> **一個方法學上的坑，已經修掉並記錄。** swing 可以自選 approach clearance，
> 而較大的 clearance 會在**零垂直代價下加長前進距離**，稀釋它自己的無因次指標；
> rolling 的 clearance 卻被 Day 6–7 凍結在 40 mm。實測：讓 swing 自選會在
> h = 60 / 80 mm **翻盤**。所以主要結果用對齊的 `c = 40 mm`，swing 自選當敏感度檢查，
> 兩者都寫進 CSV 的 `swing_variant` 欄。

**機制一句話：**

```text
roll  的起伏 = h + 14.2 mm       <- 常數 overhead
swing 的起伏 = h + min_hip_lift  <- lift 在 h <= 100 是 0，之後跳成 20 / 40 mm
```

**三個獨立指標一致地把交叉點放在 h = 100–120 mm**：peak-to-peak、per-forward、
以及下面 F 段那條規則。

#### 姿態不中性：rolling 的起伏裡有 39.8 mm 不是障礙造成的

rolling 站著（θ = 40 deg，hip 183.6 mm）出發，最後停在 wheel mode（θ = 17 deg，hip 143.8 mm）。
那是 **−39.8 mm 的姿態改變**，不是跨越障礙的代價。swing 兩端都是 θ = 60 deg 的站姿，
**是姿態中性的**。這個偏移在 peak-to-peak 裡會抵銷，所以比較仍然成立；
但它在絕對高度的圖裡看得見，`HipExcursion2D.is_posture_neutral` 把它標出來。

#### E — `L_top` 決定的不只是可行性，還有 body 代價的分界線

兩邊對 `L_top` 的反應形式相同（`V / (D_ref + (L - 0.35))`），兩式相等有閉式解：

```text
L* = 0.35 + (V_swing * D_roll - V_roll * D_swing) / (V_roll - V_swing)

h =  40 mm   V_roll  73.9  V_swing  80.0   L* = 0.610 m
h =  60 mm   V_roll 112.2  V_swing 120.0   L* = 0.386 m
h =  80 mm   V_roll 151.6  V_swing 160.0   L* = 0.415 m
h = 100 mm   V_roll 192.5  V_swing 200.0   L* = 0.666 m
h = 120 mm   V_roll 229.2  V_swing 265.0   L* < 0  -> 在 0.35 m 就已經贏了
h = 140 mm   V_roll 270.9  V_swing 325.0   L* < 0
```

**`V_roll < V_swing` 在每一個高度都成立**，所以頂面夠長時 roll 一律贏。
這一段直接餵給 Step 5：decision map 的軸就是 `(h, L_top)`。

> **界線**：Day 6–7 Step 12R 只量了 5 個 `(h, theta)` 組合的頂面下限，
> 所以 6 個高度裡有 4 個沒有實測下限，它們的短頂面欄位是**未驗證的外推**。
> `per_forward_at_top_length` 在有下限時會拒絕低於它的外推。

#### G — `theta_climb` 是一個【取捨】，不是可以自由最佳化的旋鈕

```text
h =  60 mm   theta 40 -> 85 deg   hip 起伏 +46.8 mm   需要的頂面 -64.4 mm
h = 100 mm   theta 40 -> 85 deg   hip 起伏 +34.5 mm   需要的頂面 -64.4 mm
```

B 段說「取最小的可行 theta」只在 body 起伏這一個座標上成立。Day 6–7 Step 12R 量的是
另一半：**姿態越伸展，需要的頂面越短。** Step 5 只按 body 代價選 theta，
會選到一個放不進它自己挑的那個障礙的姿態。

#### F — §5.3 缺的那條規則（見下方 §5.3 的更新框）

```text
h <=  100 mm  -> SWING（roll 起伏 54.3 / 74.3 / 94.2 / 114.9 mm，swing 下界 40 / 60 / 80 / 100 mm）
h >=  120 mm  -> ROLL （roll 137.6 / 154.2 mm，swing 下界 140 / 180 mm）
```

這條規則沒有分母，所以上面那個 clearance 稀釋問題影響不到它。

#### 對 paper §25.1 story 的結論

```text
不能寫：「hybrid 滾走的質心變化比 walking 小」
可以寫：「在 h >= 120 mm 時，滾走的 hip 起伏比 swing 少 9-12%；
        h <= 100 mm 兩者在 1-3% 內打平（swing 略優）。
        交叉點由【swing 的 hip lift 何時開始長】決定 ——
        lift 在 h <= 100 mm 是 0，之後跳成 20 / 40 mm，
        而 rolling 的 overhead 是常數 14.2 mm。」
```

**這比原本的 story 更有力，不是更弱**：它有機制、有交叉點、有數字，
而且交叉點的位置可以預測（lift 開始長的地方）。

---

## Step 5 — 疊圖與 decision rule

### 目的

Day 10–11 的核心結果，也是 paper 的 Figure D。

### 任務

1. 共用軸 `(h, L_top)`。每格放**五個**候選策略（§2.6），不是兩個 primitive：

```text
#1 ROLL_UP  + ROLL_DOWN
#2 ROLL_UP  + SWING_DOWN       <- 【Step 3 D/E 已推翻；E 可修，修好要重測】
#3 SWING_UP + ROLL_DOWN        <- 【Step 2b 已推翻】
#4 SWING_UP + SWING_DOWN
#5 SWING_OVER
```

> **2026-08-30：候選降為三個（#1 / #4 / #5）。** 兩個被推翻的組合都是「落 top 但
> 上下兩段用不同 primitive」的那種——目前的 planner 交不出跨 rim 的 handover。
> Step 5 先用三個候選跑；#2 在 retract 改成「停在 theta >= 35 deg」之後要重測。

2. 每個候選的可行性是**兩段的合取加上一個耦合條件**：

```text
feasible(pair) =  ascent 可行
              AND descent 可行
              AND L_top >= 該 pair 的最小 top-length 預算（§2.7）
              AND top 上的狀態接得起來（ascent 的出口滿足 descent 的入口前提）
```

`SWING_OVER` 不受 top-length 下界約束，但受 `stride >= L_top` 的上界約束——它是唯一
一個 `L_top` 越大越不利的策略。

3. 每個候選的 concession = 兩段 concession 的合成（第一版取兩者的**逐項最大**：
   hip 下界取較高者、hip 軌跡約束取聯集）。合成方式要寫明，因為它會影響 region 邊界。

4. Decision rule 第一版用 **lexicographic，不調權重**：

```text
1. feasible（且 min_clearance > margin 門檻，見 §6.2）
2. body deviation 較小者
3. clearance margin 較大者
4. 平手時偏好 roll 較多的那個（tie-break，要在圖上標出哪些格是靠 tie-break 決定的）
```

5. 標出五個 region，並**必須標出多個策略皆可行的區域**，在該區域內顯示代價差——
   這是全篇最有說服力的一塊。

6. 額外畫一張 `L_top` 切片：固定 `h`，橫軸 `L_top`，顯示策略如何隨 top 變短而
   從 #1 往 #3 / #5 移動。§2.7 若成立，這張圖會是單調的。

### 輸出

```text
day10_11_step5_decision_map.csv
day10_11_step5_figure_d.png          # terrain geometry -> motion class
day10_11_step5_cost_gap.png          # 兩者皆可行區域內的代價差
```

### 完成標準

- 五個候選都有 concession 數值；**不可行的要有理由**（哪一段失敗、或是 top-length 預算不足）。
- 每個實際出現在 map 上的 region 至少有一個 cell 能生出軌跡（Step 7 驗證）。
- decision rule 是一個純函式：`(h, L_top) -> (ascent, descent) + 內部參數 + BodyRequirement`。
- §2.7 的單調性（越靠 swing、需要的 top 越短）**被證實或被推翻**，兩者都要明確寫下來。
- 對 lexicographic 順序做敏感度檢查：換順序會不會改變 region 邊界？會的話記錄下來。

### Step 5 結果（2026-08-30，六段 A–F；只有 A 段跑 planner）

五條完成標準四條達成，第二條（每個 region 生一條軌跡）是 Step 7 的工作。

#### A — `SWING_OVER` 的第一次量測（先前沒有任何一步量過它）

Step 1 的 showcase 只有 `onto` / `off`，Step 2/3 掃的是落腳的兩半，
所以「五個候選都有 concession 數值」這條標準必須先補一次掃描：
80 格（8 個高度 × 10 個頂面長度），38 分鐘，**33/80 可行**。

**它的內部自由度是 θ，而且是被逼出來的。** apex 在頂面上方 30 mm，
所以飛行中段腳約在地面上方 `h + 30`，而髖在站姿高度；腿要跨過這個落差，
而它最短的伸展是 `θ = 17°` 的一個輪半徑 143.8 mm。
`HipTrajectory2D` 是直線，沒有旋鈕能讓髖在**中段**拱起（Step 2b 撞的同一道牆），
唯一能抬高髖的方法是兩端同時抬 —— 也就是更伸展的站姿。

```text
h <=  80 mm  可行
h >= 100 mm  全滅，失敗模式【100% IK_NOT_CONVERGED】

閉式預測：h_max = standing_hip(85 deg) - apex_clearance - min_leg_length
               = 264.9 - 30 - 143.8 = 91.1 mm     <- 實測天花板落在 80 與 100 之間

寬度天花板隨高度收縮   350 -> 280 -> 240 -> 170 mm  (h = 20/40/60/80)
需要的 theta 隨高度上升  50 -> 60 -> 70 -> 80 deg
stride 需求            281 - 734 mm
```

**可行集在每個高度都是嚴格前綴**（h=20 全過、40 前九、60 前八、80 前六）。
同一高度下較短的頂面就是較短的 stride，嚴格更容易，所以決策函式在未掃到的
`L_top` 上做**單調閉包**；沒有它，Figure D 上的 `#5` 會變成十個孤立的點。
閉包內取「不比任何更寬的已量測頂面更收縮」的 θ，並在 CSV 標 `measured_exactly`。

#### 機制：`L_top` 決定策略，也在 `#1` 內部決定 `theta_climb`

這是 Step 5 真正的結果，而它是把 Day 6–7 與 Step 4 兩邊的量測**接起來**才浮現的。

```text
theta 40 -> 85 deg
    required_top_length   269.4 -> 205.0 mm   （Day 6-7 Step 12R，單調【下降】）
    hip 起伏 overhead       14.2 ->  49.4 mm   （Step 4 B 段，單調【上升】）
```

`required_top_length` **只由 θ 決定**：70 個 cell 跨高度的離散，在 10 個 θ 裡有 8 個
小於 3 μm。兩個代價方向相反，`L_top` 是解開它們的變數：

```text
最便宜的 rolling 計畫 = 「required_top_length 還放得下」的【最小】 theta
```

不需要搜尋 —— 最佳解永遠在約束邊界上。**所以 rolling 的代價是 `L_top` 的階梯。**

#### B / C — Figure D：三個 region，加上一個【洞】

609 格（7 個高度 × 87 個頂面長度）：

```text
#4 SWING + SWING   210 格
#5 SWING_OVER      129 格
#1 ROLL  + ROLL     98 格
無解               172 格
```

**那個洞是能力缺口，不是資料缺口**：`h >= 100 mm` 且 `L_top < 205-235 mm`，
172/609 格**完全過不去**，而且三個策略各自被不同的理由擋住 ——
`#5` 搆不到（h > 91 mm）、`#1` 付不起 `L_transition`（>= 205 mm）、
`#4` 塞不下落點加起跳點（>= 240 mm）。這是 negative result，
但它指出了 Day 12 之後最值得攻的方向。

**257 格有兩個選項、10 格有三個** —— 這是規格說「最有說服力的一塊」的區域。

#### D — 切片：策略切換是被 `theta` 階梯推動的

```text
h =  40 mm   #5@20 -> #4@285
h =  60 mm   #5@20 -> #4@245
h =  80 mm   #5@20 -> 無解@175 -> #1@205 -> #4@240
h = 100 mm   #1@205 -> #4@240
h = 120 mm   #4@240 -> #1@265
h = 140 mm   #1@225
h = 160 mm   #4@240
```

`h = 120 mm` 那一列最能說明機制：`L_top = 240-260` 時 `#4` 贏，`>= 265` 時 `#1` 贏。
**不是因為 `#1` 在 240 不可行**，而是那裡只有 `θ = 55°` 放得下（144.2 mm），
輸給 `#4` 的 140 mm；到 265 時 `θ = 45°` 放得下（137.6 mm）就反超了。

#### E — lexicographic 順序：**敏感，而且要寫下來**

```text
default（body -> margin -> roll）    基準
margin 放在 body 前面                208 / 609 格改變
roll 偏好放在 body 前面              186 / 609 格改變
靠 tie-break 決定的格數                0
```

**約三分之一的格子會因為判準順序而換策略。** 第一版選 lexicographic 不調權重是對的
（邊界由數據而非旋鈕決定），但**順序本身就是一個未被論證的選擇**，必須跟結果一起講。

「平手時偏好 roll 較多的那個」這條 tie-break 在目前資料上**從未被觸發** ——
預設順序下 body 代價永遠分得開前兩名。

> **§6.2 的 margin 門檻預設是【關的】，而且這是踩到才改的。**
> 原本預設 `0.0`，結果在 h = 60 / 80 mm 開出一條假的「無解」帶。
> 原因：三個 sweep 都有少數可行計畫的最小餘裕在 **1e-4 ~ 1e-3 mm** 量級 ——
> swing 最緊的一點通常就是**觸地**，那裡餘裕依定義為零。這比 planner 自己的
> `collision_tolerance_m = 1 mm` 細一千倍，用 `0.0` 當門檻等於**用比模型自身容差
> 銳利 1000 倍的標準去推翻模型的判定**。現在預設是「信任 planner 已經判過」，
> 門檻做成敏感度：任何 `>= 0` 的門檻都會改變 58 格。

#### F — §2.7 的單調性：**照量到的數字不成立**

```text
#1 ROLL  + ROLL     需要 L_top >= 205 - 245 mm
#4 SWING + SWING    需要 L_top >= 240 mm          <- 【比 #1 還長】
#5 SWING_OVER       需要 L_top <= 170 - 280 mm    <- 唯一反向的
```

**`#4` 只在 6 個高度中的 1 個比 `#1` 需要更短的頂面。**

**但它的真假取決於一個 Step 2 主 sweep 凍結掉的軸。** `#4` 的下界是
`landing_distance + 最短 takeoff = 160 + 80 = 240 mm`，而 `landing_distance = 160 mm`
只是 Step 2 格點的固定值。Step 2 的 **close-out 掃過這一軸**（100–220 mm），
100 mm 在 h = 100 / 150 / 200 都可行且 **lift 完全不變**。改用 100 mm：

```text
#4 需要 L_top >= 180 mm   -> 現在【每個高度】都比 #1 短，claim 成立
```

**所以誠實的結論是：這句話的真假由一個沒有被最佳化的參數決定。**
照主格點是錯的；把 landing 移到 close-out 驗證過的最近位置，是對的。兩者都記錄，
並標明 close-out 只在三個高度驗證過。

`#5` 那一列則**無條件成立**：它是唯一一個 `L_top` 越大越不利的策略，
而且它在 20 mm 的頂面上仍然可行。

#### `SWING_OVER` 的 body 代價為什麼是零

兩端都站在低地、同一個 θ，所以直線 hip 軌跡是**水平的** —— 跨越過程對 body
的垂直要求是零。這是策略的真實性質，也是它在可行處一律勝出的原因。

它不是免費的：那個站姿把髖held 在最收縮姿態上方 `stance_hip_above_min` 處、
持續整個跨越。這個代價**刻意沒有折進比較** —— 它是否已經付掉，
取決於障礙**之間**的步態，那是 Day 15–16 的問題，不是這一步的。
它以獨立欄位回報。

---

## Step 6 — Segment 級 sequence schema

### 目的

原規劃 schema 兜不起來（§2.5），這一步定義能兜起來的版本。

### 任務

實作 §5.4 的 `MotionSegment2D` / `RollingContact` / `BodyRequirement2D` / `MotionSequence2D`。

### 完成標準

- 能**無損**表示 Day 6–7 Step 10R 的完整 traversal（含 wheel-mode 段）。
- 能無損表示 Day 8–9 的一條 swing（含 `liftoff_rise` / `touchdown_drop` / duration）。
- 每段都帶自己的取樣參數；缺任何一個就無法重建軌跡（寫一個測試證明這件事）。

### Step 6 結果（2026-08-30）

三條完成標準**全部達成**。

```text
(1) Day 6-7 Step 10R -> 10 個 segment、299/299 幀、無缺漏無重複，wheel-mode 段完整保留
(2) Day 8-9 的一條 swing -> liftoff_rise / touchdown_drop / duration / clearance 全部保留
(3) 四個取樣參數各自被【演示】成會改變它描述的軌跡，而不是只被斷言存在
```

#### 四件被資料逼出來、§5.4 沒有預期的事

**1. `alpha_range` 不夠用。**

Step 10R 的 `LEFT_RIM_TRAILING_TRANSITION` 與 `LEFT_RIM_ROLL_DOWN` 把 `alpha` 釘在
`-134.5 deg`、接觸點釘在後緣角 `(450.0, 100.0) mm`，持續 **72 幀**，而 `beta` 掃了 71 度。
腿不是在滾，是**繞著 rim 上的一個點在轉**。

```text
只記 alpha_range 的 schema，會把這 72 / 299 幀記成【一個靜止姿態】。
=> RollingContact2D 另記 beta_range 與 theta_range，
   並用 RollingMode（SURFACE_ROLL / CORNER_PIVOT）指名是哪一種。
   is_static 再把「兩者都沒動」分出來 —— 那只會發生在單幀 segment 上。
```

**2. rolling 與 swing 不共用取樣詞彙。**

swing 以 `sample_count` 切時間、對 per-sample 關節步長檢查；rolling 以 `beta_step`
（retract 段另加 `theta_step`）逐步推進到停止條件。一個半數欄位是 `None` 的結構，
會讓「哪些欄位是必填」變成無法回答的問題——而那正是第 3 條完成標準在測的。

```text
=> RollSampling2D（arc_samples, beta_step_rad, theta_step_rad）
   SwingSampling2D（arc_samples, sample_count, leg_arc_samples, max_joint_step_rad）
   兩個都【沒有預設值】：漏掉一個是建構錯誤，不是靜默 fallback。
```

**3. 端點一律是 `PointContact2D`，即使 rolling 段也是。**

§5.4 寫的是 `start_contact: ContactState | RollingContact`，二選一。資料說兩個都要：
Step 7 的交接檢查要比對關節值，兩端各需要一個確定的姿態；而 §2.5 抱怨的是
**兩端之間**發生了什麼。所以端點是點接觸，`RollingContact2D` 描述中間那段。
union 型別留在 schema 裡（規格允許另一種讀法），但本步的 builder 不產生它。

**4. `duration_s` 對整條 rolling traversal 是 `None`。**

Day 6–7 的 traversal 是**準靜態**的，從來沒有被指定過時間。這裡記成**缺失而不是編造**——
編一個數字等於在真數字該去的位置放假資料。`MotionSequence2D.total_duration_s` 因此也是
`None`（部分求和會被讀成整條的真實時間）。**Step 7 的 composer 必須替 rolling 段指定
時間，那是一個新的決定，不是從既有資料讀得出來的東西。**

#### 交接：接觸點可以跳，關節不行

```text
最大接觸點跳躍   180.5 mm   （foot rim -> right rim：站在地上換成頂到前緣）
最大 theta 跳躍    1.00 deg
最大 beta  跳躍    1.75 deg
```

接觸點大跳**不是** bug：換 rim 就是把接觸換到輪子的另一個部位。真正會是不連續的是
**關節**，而它們全都在一個取樣步長以內。

> **順帶修正 §6.3 容易誤讀的地方。** `WHEEL_MODE_TOP_ROLL -> LEFT_RIM_READY` 這一格，
> `alpha` 從 `179.4` 變成 `-179.4`，未折疊時看起來跳了 **358.8 度**，
> 但**接觸點只移動 2.9 mm**。那是**換座標卡**，不是運動。
> §6.3 的「±180° 跳 162 mm」是**同一個 rim 參數化下**的值；實際換到另一個 rim 之後，
> 兩者在物理上幾乎重合。hand-off 報告因此把 alpha 差折進 `(-180, 180]`。

#### 第三條完成標準：四個參數各自的演示

```text
arc_samples          seam bridge 34.73 mm (61) -> 5.00 mm (481)，差 6.9 倍
                     沒有它，「這次 rim 交接合不合法」答不出來（§6.1 的取樣假影）
sample_count         最大關節步長 3.41 deg (16) -> 1.71 deg (31)
leg_arc_samples      最小餘裕 1.67085 mm (31) -> 1.66949 mm (241)
                     位移很小，但 §6.2 的餘裕本來就是毫米級，
                     而 Step 5 量到 margin 門檻會動 58 格
max_joint_step_rad   是【per-sample】限制，所以 (0.20 rad, 16) 與 (0.10 rad, 31)
                     是【同一個約束】—— 兩個欄位都記才定得下來
```

#### 兩個小但會咬人的細節

```text
- FrameRef2D 用【明確的 index 列表】而不是 (start, stop) 區間：
  Step 10R 自己的輸出跳過 index 11（一幀被提出又否決），區間會默默把它算進去。
- BodyRequirement2D 允許單點 hip_z profile：
  RIGHT_RIM_FRONT_CONTACT 只有 1 幀（輪子剛碰到前緣的那一瞬間），
  而它是 traversal 自己命名的 phase。逼它併進鄰段就是這一步要避免的那種損失。
```

#### schema 已含研究計畫 §10.5.4 要求的兩個 kind

`WHEEL_ROLL` 與 `POST_TOUCHDOWN_ROLL` 都在 `SegmentKind` 裡，即使 Day 10–11 不生成
它們——之後再補會要改每一個消費端。

---

## Step 7 — Sequence composer 與交接連續性

### 任務

1. 給 `(h, L_top)`，用 Step 5 的 rule 選 `(ascent, descent)`，串出完整 `MotionSequence2D`。
   至少要串通三對：`ROLL+ROLL`、`ROLL+SWING`、`SWING+ROLL`——**第三對是驗證 2×2 的那一條**。
2. 交接檢查：
   - contact state 連續（重用 `_handoff_discontinuity_m`）
   - joint 連續（θ, β 的跳變 < 門檻）
   - 交接點離 rim 接縫多遠（§6.3）
   - 1.2 mm rim 幾何在交接點的實際貢獻（Step 0 已量，這裡驗證沒有累積）

### 輸出

```text
day10_11_step7_strategy_a_frames.csv / _summary.csv
day10_11_step7_strategy_b_frames.csv / _summary.csv
day10_11_step7_handoff_report.csv
```

### 完成標準

- `ROLL+ROLL`、`ROLL+SWING`、`SWING+ROLL` 各生出一條完整 sequence，全程 collision-free。
- 所有交接點的不連續量被量化並低於門檻，或被明確記錄為已知誤差。
- `SWING+ROLL` 那條的 top-length 用量被量出來，並與 `L_transition` 比較。

### Step 7 結果（2026-08-30）

三條完成標準達成，其中兩條需要改述——**這一節點名的三對裡有兩對已經不存在了**。

```text
#2 ROLL_UP  + SWING_DOWN   被 Step 3 D/E 推翻
#3 SWING_UP + ROLL_DOWN    被 Step 2b 推翻
```

所以「串通三對」誠實的讀法是：串通還活著的三對（`#1` / `#4` / `#5`），
並把兩個推翻當成**第一級輸出**而不是略過。`compose_2d` 對五個候選都會回答。
每一條都在 Step 5 的規則**實際會選它**的那一格組出來——一條規則永遠不會選的策略，
生得出軌跡也證明不了那條規則。

```text
#1 ROLL + ROLL     h=140 L=225 mm   10 段 272 幀   餘裕 1.834 mm   用掉 202.0 mm 頂面
#4 SWING + SWING   h= 80 L=240 mm    2 段  62 幀   餘裕 1.393 mm   用掉 240.0 mm
#5 SWING_OVER      h= 60 L= 75 mm    1 段  31 幀   餘裕 1.298 mm   用掉   0.0 mm
```

#### 這一步真正的價值：它抓到前面幾步的三個缺陷

Step 7 是第一次拿決策去**實際生軌跡**，而那揭露了三件事。

**1. 決策回報的參數不足以重建它選的動作。**
Step 2 / Step 3 是靠 repair ladder 走到可行的（`liftoff_rise` / `touchdown_drop` /
`duration_scale`），而 `decide_2d` 只回報 body 旋鈕。第一版 composer 照著重建，
**重現了 sweep 早就修掉的 `TERRAIN_COLLISION`**——`#4` 在 h = 80 mm 實際需要
`ascent_liftoff_rise = 50 mm`。

> Step 5 的第三條完成標準寫的是「純函式 `(h, L_top) -> (ascent, descent) +
> **內部參數** + BodyRequirement`」。這一步證明了那些 repair 旋鈕**就是**內部參數，
> 不是裝飾。已補進決策輸出，而且**沒有**折進 `body_deviation_m`——
> 它們是軌跡整形不是 body 讓步，這正是 Step 3 把 hold 放在 ladder 最外層的理由。

**2. `#1` 的頂面下界拿「輸出」當「前提」。**
`required_top_length_m` 是一次成功的 traversal 回報的消耗量，不是可行性前提。
Day 6–7 Step 12R 自己的 top-length sweep（用它自己的設定）就試過：

```text
h = 60 mm, theta = 40 deg
    L = 269.4 mm   失敗   LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER   <- 正好等於 required
    L = 279.4 mm   成功
```

**3. composer 必須沿用 sweep 自己的設定 bundle——而這是 `#1` 前兩次失敗的真正原因。**

```text
ObstacleSpec2D.arc_samples          預設 241，但 Day 6-7 的 sweep 跑的是 121
TraversalConstraints2D
    .max_seam_bridge_m              預設 5 mm，但必須跟 arc_samples 【配對】：
                                    seam_bridge_for_sampling_m(121) = 17.5 mm
```

這正是 §6.1 那類取樣假影，也是交接檔陷阱 2。修好之後 `#1` 在 L=350 **和** L=235
都組得出來。**所以先前那兩次失敗不是界線的證據，是設定 bug**——註解與測試裡
引用它們當證據的地方已全部改掉。修法不是把數字抄對，是**沿用 `SweepSettings2D`**，
讓這種配對不可能再錯一次。

#### Step 7 自己的新量測，而且它否決了我對第 2 點的第一版修法

我一度把界線改成「一律 `required + 10 mm`」。但 10 mm 是 θ=40 那次 sweep 的
**格點間距**，不是量到的差額。Step 7 在另一個 θ 上直接夾了一次：

```text
h = 140 mm, theta = 70 deg      required_top_length = 222.5 mm
    L = 220 mm   失敗   LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER
    L = 225 mm   成功
=> 真實下界在 (220, 225]，而 222.5 mm 【落在區間內】。
```

兩個直接量測對「差多少」不一致，而第二個說一律加 10 mm 會超調。最終規則：

```text
Step 12R 實測過的 theta（40 / 60 / 85）  -> 用實測最小值 279.4 / 247.2 / 215.0 mm
其餘 7 個 theta                          -> 用 required_top_length，並標記為【未驗證】
```

> **副作用要說清楚**：界線因此在 theta 上**不再單調**——θ=60 的實測 247.2 mm 高於
> θ=55 的需求 243.0 mm，儘管更伸展的姿態該要更短的頂面。那是兩種來源混用的
> 可見假影（實測含差額、需求不含），不是物理反轉。**留著讓它可見**，
> 因為抹平它就等於發明那個已被第二個量測否證的邊際值。

**Step 5 的地圖已用修正後的界線重跑**：`#1` 的下界 205–245 → **215–245 mm**，
§2.7 的判決不變（1/6），順序敏感度 200 / 184（原 208 / 186）。

#### 交接檢查

```text
最大接觸點跳躍   215.9 mm   （foot rim -> right rim：站在地上換成頂到前緣）
最大 theta 跳躍    1.00 deg  = 一個取樣步長
最大 beta  跳躍    1.75 deg  = 不到兩個取樣步長
最大 rim 幾何 gap  1.2000 mm = 正好是 Step 0 的標稱值，【不累積】
```

接觸點大跳**不是**不連續：換 rim 就是把接觸換到輪子的另一個部位。
1.2 mm 那項的檢查是**最大值**而不是變化量——foot rim 上是 0、upper tyre 上是 1.2 mm，
交接只是在兩者之間切換而不是相加。

**但 §6.3 擔心的事被量出來了，而且要記錄成已知風險。** 在地圖允許的**最短**頂面上
（`#1`, h=140, L=225 mm）：

```text
LEFT_RIM_READY                  1 幀   alpha = -178.83
LEFT_RIM_TRAILING_TRANSITION    7 幀   alpha = -178.83
LEFT_RIM_ROLL_DOWN             81 幀   alpha = -178.83
=> 88 幀全部在距離 ±180 度接縫 【1.17 度】的地方進行。
```

對照 Step 6 那條 `L_top = 350 mm` 的 traversal：`LEFT_RIM_READY` 有 46 幀、
alpha 從 −179.4 滾到 −134.5，交出去之後還有 **45 度**的 left-rim 弧可用。

```text
=> #1 的頂面下界，實質上就是【left-rim 弧預算歸零】的那一點。
   跨過去時接觸點只移動 5.9 mm（乾淨），但那個交接【沒有任何餘裕】——
   只要取樣改變讓接縫位置移動超過 1.17 度，這個交接點就會跑掉。
   Day 12 之後若要動 arc_samples，這一格要重測。
```

#### 頂面用量 vs `L_transition`（第三條完成標準，改述）

規格要 `SWING+ROLL` 的用量。那一對不存在，所以量的是 `#4`：

```text
#4 用掉 240.0 mm   落在 L_transition 的 196-249 mm 帶內
#1 用掉 202.0 mm   也在帶內，但它【需要】L_top >= 225 mm
#5 用掉   0.0 mm   完全不碰頂面
```

`#1` 那 23 mm 的差是**前後緣的進出餘裕**：`L_transition` 只是 retract 加
wheel-mode 滾動的部分，**不是整個頂面需求**。引用它當「rolling 需要多長的頂面」
會低估 23 mm。

#### Step 5 留下的三件，全部驗證

```text
1. #5 的單調閉包        成立 —— L = 75 mm（sweep 未掃過）組得出來，餘裕 1.298 mm
2. #4 的 close-out landing  成立 —— h = 100 mm、L = 200 mm、landing 100 mm 組得出來
                            這正是決定 §2.7 單調性成不成立的那一格
3. 那個【洞】             確認不是單一天花板：#1 / #4 被 top length 擋、#5 被 stride 擋
```

---

## Step 8 — 五個 terrain case 的 regression（含負向）

### 任務

從 Step 5 的每個 region 各挑一個 terrain，跑完整 composer。**每個 case 同時要跑被否決的策略**，
證明它們確實不可行或代價確實較高。

```text
#1 低矮、top 夠長           -> ROLL_UP + ROLL_DOWN，且全 swing 代價較高
#2 h ≈ 0.16、top 夠長       -> ROLL_UP + SWING_DOWN，且 ROLL_DOWN 確實不可行（0/10）
#3 top 短於 L_transition    -> SWING_UP + ROLL_DOWN，且 ROLL_UP + ROLL_DOWN 確實付不起 top 預算
                               （若 Step 2b 判定 #3 不成立，這個 case 改成驗證退化版本）
#4 高、top 夠長可落腳       -> SWING_UP + SWING_DOWN
#5 top 很短                 -> SWING_OVER，且所有落 top 的策略都付不起
```

`#3` 這個 case 是這一步最重要的一個：它是唯一能證明「上升與下降必須獨立決策」的證據。
如果只跑 #1/#2/#5，2×2 的必要性就沒有實驗支撐。

### 完成標準

- 五個 case 的正向與負向都成立（#3 若退化，退化版本也要跑完並記錄）。
- 負向失敗的 `failure` 與 `binding_ceiling` 有具體理由，不是「就是不行」。
- 至少有一個 case 的最佳策略是**混合的**（ascent 與 descent 不同），否則 2×2 沒有被驗證。

### Step 8 結果（2026-08-30）

**第三條完成標準無法達成，而那本身就是這一步最重要的結果。**

```text
#2 ROLL_UP  + SWING_DOWN   被 Step 3 D/E 推翻
#3 SWING_UP + ROLL_DOWN    被 Step 2b 推翻
=> 兩個混合對都不在了，所以沒有任何 case 的最佳解會是混合的。
   2x2 不是「未被驗證」，是【對角線之外被推翻】。
```

五個 case 因此重新切到 Step 5 的**實際 region** 上，但保留每個原 case 的目的，
而且每個 case 都把**所有**策略（贏家和輸家）都組一次——負向結論要建立在
實際執行上，不是查表。

#### 五個 case

```text
                              贏家              輸家的處境
A  h= 40 L= 50   #5 SWING_OVER   0.0 mm    #1 / #4 都在 top length 上失敗
B  h=140 L=350   #1 ROLL+ROLL  154.2 mm    #4 【可行但要 180.0 mm】—— 貴 25.8 mm
C  h= 80 L=350   #4 SWING+SWING 80.0 mm    #1 【可行但要  94.2 mm】—— 貴 14.2 mm
D  h=120 L=150   都不行                     三個拒絕，三個【不同】理由
E  h=160 L=350   #4 SWING+SWING 220.0 mm   #1 不可行（沒有任何 theta 走得完）
```

**B 和 C 是決策規則真正被驗證的地方**：輸家不是「不可行」，而是**確實可行、
確實比較貴**。那正是 Step 5 的 lexicographic 規則在做的事，而且兩個方向都驗到了。

#### 每個負向都說得出理由

```text
19 個拒絕，0 個沒有理由。
```

那個洞（case D）的三個拒絕有 **3 個不同理由**，但只有 **2 個 limiter 名稱**：

```text
#1  top length : 每個可行 theta 都需要更長的頂面，最省的要 243.0 mm
#4  top length : 落點吃掉 160 mm、最短起跳 80 mm，所以頂面至少要 240 mm
#5  stride     : 沒有任何 theta 跨得過去（IK_NOT_CONVERGED）
```

`#1` 和 `#4` **共用一個 limiter 名稱卻是不同的牆**——一個付不起 `L_transition`，
另一個塞不下「落點加起跳點」。所以那個洞不是單一天花板，
而且比較理由時要比**完整理由**而不是它的第一個子句。

#### Case E：量化推翻 `#2` 的代價

這是規格原本為 `#2` 設計的那塊地形（「`h ≈ 0.16`、top 夠長 → `ROLL_UP + SWING_DOWN`，
且 `ROLL_DOWN` 確實不可行（0/10）」），而**它的前提完全正確**：

```text
Day 6-7 在 h = 160 mm：
    roll_up   成功  10/10 個 theta
    roll_down 成功   0/10 個 theta
```

用 `keep_partial` 保留失敗 traversal 中成功的階段，把那個「爬得上去卻用不了」的
ascent 實際計價：

```text
ROLL_UP  hip peak-to-peak    80.1 mm   （2 段 60 幀，theta = 85 deg）
SWING_UP hip peak-to-peak   220.0 mm   （= h 160 + min_hip_lift 60）
=> 滾上去對 body 的要求少【2.7 倍】，而且它在【每一個】theta 都成立。
```

**它用不了的原因和這個 ascent 完全無關**——`#2` 被推翻是**下降側**的理由
（Step 3 D 段的 `+40 deg` 接縫、E 段的 `theta >= 35 deg` 下限）。
那條 partial traversal 停在 `ROLL_DOWN: NO_LEGAL_CORNER_PIVOT_CONTINUATION`，
和 Step 2b 在 `h >= 160 mm` 對「空中來的抵達」量到的**同一個**失敗——
再次確認那道牆是下降側的，與上升怎麼來的無關。

> **這就是 2x2 對角線之外被推翻的代價，而且它是一個具體的數字。**
> `h = 160 mm` 的地形上，機器人被迫用一個對 body 要求 2.7 倍的爬升方式，
> 只因為它想用的那個下降方式配不上任何一種爬升。
>
> Step 3 說 E 段那道牆**是可修的**（retract 停在 `theta >= 35 deg` 而不是 17 deg）。
> 這一格量出了修好它值多少，把它從一個技術待辦升格成一個**有價碼的目標**。

#### 對 §2.6 的結論

```text
§2.6 主張「上升與下降必須獨立決策」，而那個主張的實驗支撐就是
「至少有一個地形，最佳解是混合的」。

實測：兩個混合對各自撞到一道 rim 分段接縫。
    #3  落地要跨 alpha = -40 deg                  （Step 2b，不可修）
    #2  起飛要跨 alpha = +40 deg                  （Step 3 D 段，不可修）
        或退化成 retract，撞 theta >= 35 deg 下限  （Step 3 E 段，【可修】）

=> 分解【概念上】仍然成立（你確實是分開選的），
   但選擇集合在目前的 planner 下【塌陷成對角線】。
```

---

## Step 9 — Day 12 交接物

> **2026-08-30 改寫。** 原任務只寫「導出 body requirement timeline」。
> 這次討論加了三件必須先做或必須明說的事，見下方「Step 9 之前的小修正」。

### Step 9 之前的小修正 **【2026-08-30 已完成】**

兩件都做完了，而且**沒有動任何量測數字**——只換了標籤與型別。

#### (i) 結論用語：`Verdict` 這個階梯型別

`day10_11_decision_map_2d.py` 新增：

```text
Verdict                       COMPOSED / DIRECT_HANDOFF_INFEASIBLE /
                              REQUIRES_MULTILEG_REPOSITION / OUT_OF_ENVELOPE /
                              NOT_MEASURED / PHYSICALLY_INFEASIBLE
StrategyCell2D.verdict        每一格帶自己的標籤
    .effective_verdict        依 Availability 對照，不是二分猜測；
                              【永遠不會】推導出 PHYSICALLY_INFEASIBLE
Availability.REFUTED
    -> Availability.HANDOFF_BLOCKED     「refuted」讀起來像在講機器人；
                                        量到的是兩個 primitive 不接得起來
Limiter.REFUTED -> Limiter.HANDOFF_BLOCKED
refuted_cell_2d -> blocked_pair_cell_2d （舊名保留為別名，不破壞既有呼叫端）
```

`REFUTATIONS`（純字串）換成 **`BLOCKED_PAIRS`（結構化紀錄）**：

```text
BlockedPair2D
    verdict           DIRECT_HANDOFF_INFEASIBLE
    evidence          量到什麼、在哪裡量的
    single_leg_fix    純單腿能不能修（#2 有：RETRACT_FOR_SWING_DOWN；#3 沒有）
    multileg_route    四腳路徑（兩者都有，且都標明【尚未驗證】）
```

> **為什麼從字串改成紀錄**：字串會被單獨引用，於是「被推翻」就旅行到別處去了。
> 紀錄強迫呼叫端**連同結論等級與退路一起帶走**。
> `REFUTATIONS` 仍以 `{strategy: blocked.summary}` 保留，舊讀者不會壞。

#### (ii) `TransitionRequirement2D`

`day10_11_motion_schema_2d.py` 新增 `TransitionKind.TOP_REPOSITION` 與
`TransitionRequirement2D`，並讓 `MotionSequence2D` 多帶一個 `unresolved` 欄：

```text
MotionSequence2D.unresolved     未解的 transition requirement
                .is_complete    有任何一個未解 -> 【不是】完整計畫
                .rows()         segment 與 unresolved 同表，用 row_kind 區分
```

三個刻意的設計決定：

```text
1. 它【不帶】軌跡也不帶時間。
   單腿模型答不出「離地期間誰支撐 body」，寫下 theta/beta/duration
   就是把未驗證的假設穿上量測的外衣。

2. resolved=True 會【拋錯】。
   一個已解的 transition 是 segment，不是 requirement ——
   解掉它的方式是【換成真正的動作】，不是把旗標翻過來。

3. target_condition 是文字（例如 LEFT_RIM_READY），不是姿態。
   把姿態釘死正是 §5.6 要延後的那件工作。
```

`compose_2d` 現在對 `#2` / `#3` 回傳 `verdict` 與一筆 `TOP_REPOSITION` 需求，
CSV 也多了 `verdict` / `unresolved_transitions` / `unresolved_kinds` 三欄——
**只讀 CSV 的人也能分辨「還沒解」和「不存在」。**

#### 已重新產生的輸出

```text
day10_11_step5_decision_map.csv     多了 verdict 欄（數字未變）
day10_11_step8_cases.csv            多了 verdict / unresolved_* 欄
day10_11_step7_refusals.csv         改為 BlockedPair2D 的四欄（verdict / evidence /
                                    single_leg_fix / multileg_route）
```

> **Step 7 的 CSV 需要重跑 driver 才會更新**（它含一次 rolling traversal，約 2 分鐘）。
> 數字不會變，只有欄位會變。

#### 實作過程中自己踩到、值得記下來的一件事

第一版把 §5.5 原本的四個標籤直接套到整張地圖，於是 19 個拒絕**全部**變成
`DIRECT_HANDOFF_INFEASIBLE`——包括 `#1` 只是因為頂面太短的那些。
**那是同一種過度推廣，只是方向相反**：原本擔心的是把條件性結果寫成物理結論，
結果變成把幾何界線寫成交接失敗。

修法是補 `OUT_OF_ENVELOPE` 與 `NOT_MEASURED`，並讓 `effective_verdict`
**依 `Availability` 對照而不是二分猜測**。教訓：

```text
一套為某個特定問題設計的標籤，套到更大的範圍之前要先檢查每一類拒絕落在哪裡。
```

### 任務

把 Step 7 的 sequence 導出成 body requirement timeline：

```text
t 或 x -> hip_z 下界 / hip_z 軌跡 / 該段屬於哪個 primitive
                                 / 已解或未解的 transition requirement
```

**對三種策略的處理不同：**

```text
#1 / #4 / #5   輸出已 composed sequence 的 body-requirement timeline
#2 / #3        輸出 REQUIRES_MULTILEG_REPOSITION 或更具體的 unresolved transition
               【不要】猜測 top reposition 的 theta / beta / duration
```

### Step 9 必須明說的一個問題：rolling 沒有時間

```text
Day 6-7 的 rolling trajectory 是【準靜態】資料，沒有 duration。
=> 任何 rolling timing 都是一個【新的 modeling decision】，
   不是從既有資料直接讀出來的量測結果。
```

兩種都可以接受，但**必須擇一並寫明理由**：

```text
(a) 以 x 為自變數     不需要新決定，但 Day 12 的 timing 要自己補時間
(b) 以 t 為自變數     要指定 rolling 段的時間；那是新的建模決定，
                     必須寫出用了什麼假設（等速？等 beta 速率？）與為什麼
```

### 完成標準

- Day 12 的四腳 timing 可以只讀這個檔案，不需要回頭讀 Day 6–7 / 8–9 的任何內部結構。
- 檔案裡明確標示哪些是**硬約束**（違反則 sequence 失效），哪些是**偏好**：

```text
硬約束   BodyRequirementKind.TRACK        rolling 段 —— 違反則接觸幾何不成立
         BodyRequirementKind.PINNED       Step 2b 那種落地 —— hip 沒有自由度
偏好     BodyRequirementKind.LOWER_BOUND  swing 段 —— body 可以更高，只是沒必要
```

- **未解的 transition requirement 有自己的列**，而且看得出它是「還沒解」不是「不存在」。
- rolling 的時間來源（`x` 或某個明說的 timing 假設）寫在檔案的表頭或旁註裡。

### Step 9 結果（2026-08-31，六段 A–F；只有 A / B 段跑 planner）

**四條完成標準全部成立。** 交付物是規格 §9 點名的那個檔案：

```text
day10_11_step9_body_requirements.csv    392 列 x 63 欄
    provenance 7 / sequence 5 / segment 13 / knot 365 / unresolved_transition 2
```

#### 時間基準：選 (a) 以 `x` 為自變數，而且它是【被檢查的】

```text
Day 6-7 的 rolling traversal 是準靜態的，duration_s 全部是 None。
=> 指定 rolling 的時間是【新的建模決定】，不是量測結果。

選 (a)。已存在的時間【照原樣帶過去】而不是丟掉：
    swing 段     segment_duration_s = 0.6 s、逐取樣的 knot_time_s
    rolling 段   兩欄留白 —— 留白的意思是「從未被指定」，不是 0

而「x 能不能當自變數」不是假設：三條 sequence 的 knot 序列全部單調
（tolerance 1e-9 m），寫在每一條 sequence 列的 x_is_monotonic 上。
```

#### 三條 timeline

```text
                          knots  硬   偏好  最大 knot 間距  clearance budget
#1 ROLL_ROLL   h=140 L=225  272  272    0      40.6 mm        1.834 mm
#4 SWING_SWING h= 80 L=240   62    4   58       9.8 mm        1.393 mm
#5 SWING_OVER  h= 60 L= 75   31    2   29      13.5 mm        1.298 mm
#2 / #3                       0    0    0        --             --
```

`clearance_budget_mm` 是整條 sequence 上量到的最小地形餘裕。**Day 10–11 沒有量過
`TRACK` 的追蹤容差**，這是唯一有的替代品：body 偏離要求時吃掉的就是它。

#### A — `PINNED` 不是 Step 2b 的特例，是【每一條 swing 的兩個端點】

上面「硬約束 vs 偏好」那張對照表，照字面套會讓 `PINNED` 在三條活著的 sequence 裡
**一次都不出現**——因為 Step 2b 那種落地屬於已被推翻的 `#3`。

**那個結論是錯的。** 一條 swing 的兩個端點都是接觸瞬間：

```text
量測：站在一個面上時 hip_z 是 theta 的嚴格單調函數
    theta = 60 deg -> hip_z = 219.4 mm
    body 低 10 mm  -> theta = 54.4 deg
    body 高 10 mm  -> theta = 65.6 deg
=> 端點的 hip 高度與 theta【互相決定】，body 在那裡沒有自由度。
```

這是實作紀錄陷阱 16 的另一面：`plan.valid` 只說 swing 成功了，不說落在要求的
contact state。**把整條 swing 標成偏好，下游會在觸地那一刻換掉落地姿態而不自知。**

```text
所以規則是：
    rolling 每一幀      TRACK        HARD
    swing 的頭尾兩幀    PINNED       HARD        <- 本節新增，§5.3 沒有寫
    swing 的中段        LOWER_BOUND  PREFERENCE
    => 278 硬 / 87 偏好
```

`contact_phase` 同時把 Day 12 要的 duty 交出去了：`STANCE`（rolling 全段 + swing
端點）vs `FLIGHT`（swing 中段）。

#### B — segment 的純量是 **envelope**，不是 **timeline**

`MotionSegment2D.body_requirement` 對一整條 swing 只留一個 `hip_z_min`，
它是那條 hip 軌跡的**最大值**（Step 6 的 builder）。

```text
#4 SWING_UP    envelope 299.4 mm，起跳端的真實要求 219.4 mm  -> 多 80.0 mm
#4 SWING_DOWN  envelope 299.4 mm，觸地端的真實要求 219.4 mm  -> 多 80.0 mm
#5 SWING_OVER  envelope 237.5 mm = 全段要求                  -> 多  0.0 mm
```

那 80 mm 剛好是障礙高度：拿 envelope 當 timeline，等於在 swing 還沒起跳就叫 body
先站到頂面高度。所以**兩個都寫進檔案**——`segment_envelope_hip_z_mm` 在 segment 列
（比較策略時用它，Step 5 就是拿它決策的），`hip_z_required_mm` 在 knot 列。

**而 `TRACK` 段根本沒有那個純量**，欄位留白：`TRACK` 的要求**就是**那條 profile。

#### 第一條完成標準怎麼檢查的

規格說「Day 12 只讀這個檔案就夠」。誠實的檢查方式不是在文件裡宣稱它，而是
**用一個不 import 本專案任何東西的 reader 把它讀回來**：

```text
reader_check_2d()  只 import csv。它稽核：
    每個 segment 宣告的 frame_count 與實際 knot 列數相符
    每條 sequence 的 x 單調
    每個 knot 的 constraint_class 落在 HARD / PREFERENCE 之內
    每個 knot 的 hip_z_upper_bound_mm 是 NOT_MEASURED（留白讀起來像「無上界」）
    每列未解 transition 的 resolved=False 且姿態欄是 NOT_GENERATED
    沒有任何一列宣稱 PHYSICALLY_INFEASIBLE
結果：PASS。測試裡有三個【故意弄壞檔案】的案例確認它抓得到。
```

檔案自己帶 7 列 `provenance`：時間基準、硬/偏好的定義與違反後果、envelope 與
timeline 的差別、`NOT_MEASURED` 的意思、§5.5 的用語階梯、scope。
**這些不能只留在 notebook 裡**——完成標準要的是 Day 12 打開檔案就看得到。

圖也是**從交付的 CSV 畫的**，不是從記憶體裡的物件，所以它同時是第一條完成標準的示範。

#### `#2` / `#3`：2 列，姿態欄寫 `NOT_GENERATED`

```text
#2 ROLL_UP + SWING_DOWN   h=160 L=350（Step 8 case E 的地形）
    需要：頂面上一個 theta >= 35 deg 的 foot-rim 起跳
    單腿修法：RETRACT_FOR_SWING_DOWN（Step 8 已標價 2.7 倍）
#3 SWING_UP + ROLL_DOWN   h=140 L=225
    需要：LEFT_RIM_READY
    單腿修法：沒有；只有四腳路徑

theta_deg / beta_deg / segment_duration_s = NOT_GENERATED，不是留白。
留白會被讀成 0，而它們不是 0，是【還沒有人算過】。
```

#### 實作時自己踩到、值得記下來的一件事

```text
第一版對 TRACK 段也算了一個 envelope = max(profile)，於是 #1 被報成
「被多約束了 136.9 mm」。但 TRACK 的要求【就是】那條 profile，
它從來沒有一個純量可以被誤讀 —— 那個數字是憑空製造出來的。
同一次還有第二個：blocked pair 的 x_is_monotonic 原本寫 True（空集合上為真），
那什麼也沒說，卻會被讀成「檢查過了，沒問題」。兩個都改成留白。

【和 Step 9 前置修正踩到的是同一類錯】：
一個為某個情況設計的量或標籤，套到不適用的情況上，
就會生出一個沒有人主張過的結論。
```

---

## Step 10 —（Day 10–11 之後）六個發展方向

> **2026-08-30 新增。這些【不是 Day 10–11 的範圍】**，寫在這裡是為了讓 Step 9
> 收尾之後知道往哪走，以及讓每一項都有它自己的價碼。

```text
優先 1  Swing endpoint theta 自動選擇
        Step 2 的主 map 仍固定 theta = 60 deg，所以 #4 的 body 代價是【偏保守的上界】。
        對每個 terrain cell 自動選 endpoint theta，可以直接壓低它。
        （Step 2 close-out C 已量出最佳 theta 隨高度上移，只差把主 map 重掃。）

優先 2  最佳化 landing / takeoff distance
        #4 的頂面需求受固定 landing distance 影響。close-out 已顯示
        landing 從 160 mm 縮到 100 mm 在部分高度仍可行【而且不增加 lift】。
        把 landing / takeoff / endpoint theta / repair knobs 一起展開，
        可能縮小「高障礙 + 短頂面」的 capability hole。

優先 3  新增 RETRACT_FOR_SWING_DOWN
        不要再把 Day 6-7 為 roll-down 設計的 theta = 17 deg retract 用在 swing-down。
        新變體只收到 theta >= 35 deg 且可以安全起飛的姿態。
        【這是目前最可能快速拿回 #2 的單腿介面修正】，不必等四腳 TOP_REPOSITION。
        價碼已知：Step 8 量到 h = 160 mm 上 roll-up 只要 80.1 mm、
        被迫改用的 swing-up 要 220.0 mm —— 2.7 倍。

優先 4  分段 hip trajectory
        目前 swing 受限於直線式 HipTrajectory2D。允許
        「起點維持指定高度 -> 中間局部拱高 -> 落點回到釘死的接觸高度」之後：
            - SWING_OVER 的高度天花板（目前 91 mm）會直接上移
            - Step 2b 那種「端點釘死但中段需要 clearance」的落地才有可能
        Step 2b 與 Step 5 A 段撞到的是【同一個】限制。

優先 5  四腳 TOP_REPOSITION（見 §5.6）
        等四腳 timing 與支撐評估到位之後再實作：
            SAFE_TOP_CONTACT -> 判斷其他腿能否承重 -> 抬起目標腿
            -> 找到新落點 / theta / beta / rim target
            -> ROLL_DOWN_READY 或 SWING_DOWN_READY
        此時才可以【真正重測 #2 / #3】，判斷它們在四腳系統中是否成立。

優先 6  多障礙長路 coverage test
        先定義 (h, L_top, gap) 的機率範圍與分布，再產生大量地形，統計：
            - 在明確障礙分布中的實際成功率
            - 各種策略被選中的比例
            - 失敗是幾何 reach / joint limit / 接觸交接，還是支撐假設造成
            - 哪些 capability hole 是機器人的物理極限，哪些只是 planner 少缺 primitive
        【在做完這件事之前，不要用現有 map 的格數宣稱「多數 obstacle 都能通過」。】
```

---

# 8. Day 10–11 不做什麼

```text
四腳 timing / body trajectory 生成      Day 12
support polygon / ABAD                  Day 13–14
cost function 權重調校                  第一版 lexicographic，不調
energy / time optimality                不在範圍
online replanning                       不在範圍
3D / gamma != 0                         不在範圍
1.2 mm rim 幾何差異的【修復】           只量、不修（沿用 Day 8–9 決定）
alpha 接縫的通用處理                    只在交接點檢查距離，不做通用 re-parameterization
```

---

# 9. 預期輸出檔案

```text
day10-11/
    day10_11_roll_swing_selection_zh_TW.md        本檔
    day10_11_step0_scene_alignment.csv            80 cells         [完成]
    day10_11_step0_roll_exit_handoff.csv          60 stage exits   [完成]
    day10_11_step1_swing_concessions.csv          16 showcases     [完成]
    day10_11_step2_swing_onto_sweep.csv           88 cells         [完成]
    day10_11_step2_min_hip_lift_map.png                            [完成]
    day10_11_step2_binding_ceiling_map.png                         [完成]
    day10_11_step2_min_liftoff_map.png                             [完成]
    day10_11_step2_closeout.csv                   48 cells         [完成]
    day10_11_step2_closeout.png                                    [完成]
    day10_11_step2b_beta_window.csv               4 heights        [完成]
    day10_11_step2b_swing_to_left_rim_ready.csv   90 cells         [完成]
    day10_11_step2b_alpha_seam_distance.png                        [完成]
    day10_11_step2b_landing_map.png                                [完成]
    day10_11_step3_swing_off_sweep.csv            204 cells / 6 段 [完成]
    day10_11_step3_min_hip_hold_map.png                            [完成]
    day10_11_step4_roll_concession.csv            77 rows          [完成]
    day10_11_step4_roll_hip_profile.png                            [完成]
    day10_11_step4_hip_excursion.csv              78 rows          [完成]
    day10_11_step4_hip_excursion_roll_vs_swing.png                 [完成]
    day10_11_step5_swing_over.csv                 80 cells         [完成]
    day10_11_step5_decision_map.csv               4675 rows        [完成]
    day10_11_step5_figure_d.png                                    [完成]
    day10_11_step5_cost_gap.png                                    [完成]
    day10_11_step5_top_length_slice.png                            [完成]
    # §2.7 那張表的實測版併進 day10_11_step5_decision_map.csv 的
    # monotonicity / landing_sensitivity 兩段，沒有另開檔案。
    day10_11_step6_sequence_segments.csv          11 segments      [完成]
    day10_11_step6_handoff.csv                    9 hand-overs     [完成]
    day10_11_step6_sampling_evidence.csv          4 parameters     [完成]
    day10_11_step6_sequence_segments.png                           [完成]
    day10_11_step7_pair1_roll_roll_frames.csv / _summary.csv       [完成]
    day10_11_step7_pair4_swing_swing_frames.csv / _summary.csv     [完成]
    day10_11_step7_pair5_swing_over_frames.csv / _summary.csv      [完成]
    # pair2 / pair3 不存在：那兩對被 Step 3 / Step 2b 推翻，
    # 改以 day10_11_step7_refusals.csv 記錄理由。
    day10_11_step7_handoff_report.csv             10 hand-overs    [完成]
    day10_11_step7_refusals.csv                   2 pairs          [完成]
    day10_11_step7_top_length_budget.csv          3 strategies     [完成]
    day10_11_step7_open_items.csv                 Step 5 的三件     [完成]
    day10_11_step7_compose_attempts.csv                            [完成]
    day10_11_step7_sequences.png                                   [完成]
    day10_11_step8_cases.csv                      25 rows          [完成]
    day10_11_step8_two_by_two_verdict.csv                          [完成]
    day10_11_step8_mixed_pair_cost.csv                             [完成]
    day10_11_step8_cases.png                                       [完成]
    # 原本寫 day10_11_step8_regression.csv，實際檔名是上面四個。
    day10_11_step9_body_requirements.csv          392 rows         [完成]
                                                  <- Day 12 的交接物，只讀這一個
    day10_11_step9_pinned_endpoint_evidence.csv                    [完成]
    day10_11_step9_completion_criteria.csv                         [完成]
    day10_11_step9_body_requirements.png                           [完成]
```

Notebook 入口比照 Day 8–9：`hybrid_gait_day10_11_motion_selection_dashboard.ipynb`，
notebook 只顯示摘要與首尾預覽，完整逐幀資料留在 CSV。

---

# 10. 對 paper 的敘事

這兩天要撐起 Contribution B。目前可以寫的故事線：

**（1）問題的重新表述。** 現有 hybrid legged-wheeled 工作多半把 rolling 與 walking 當成兩個模式，用地形分類或啟發式切換。本工作指出，在單腳層級這個選擇的正確判準不是「哪一個 primitive 可行」，而是「哪一個對 body trajectory 的要求較低」——因為在相當大的地形範圍內兩者都可行，而可行性本身沒有鑑別力。

**（2）支持這個表述的量測。** 160 mm 台階：rolling 的完整 traversal 不可行（roll_down 0/10），但 roll_up 本身 10/10；swing 可行但要求 hip 抬 60 mm。這一個 cell 同時展示了「二值判準會給出錯誤答案」與「代價判準給出正確答案」。

**（3）方法。** 對每個 primitive 在自己的內部自由度上取最小 body 讓步，得到兩張同座標軸的 concession envelope，疊圖直接產生 decision rule。這個做法的副產品是：planner 不再需要 body trajectory 當輸入，而是輸出對它的約束——解掉了 planning 順序上的循環依賴。

**（3b）第二條結果線：top length 才是選擇策略的主軸。** 上升與下降是兩個獨立決策，
策略空間是 2×2 + 1；而五種組合各有自己的最小 top-length 預算，且越靠 swing 越短
（§2.7）。這條線的價值在於它是一個 rolling 的二值可行性 map 完全看不到的維度——
障礙不高但頂面很短時，full rolling 失敗的原因不是爬不上去，是付不起 `L_transition`。
`SWING_UP + ROLL_DOWN` 這一格就是專門攻擊這個限制的，而它在原本的三分類裡不存在。

**（4）negative results 值得寫。** §3.1 的「θ 軸對齊是假對齊」、§6.1 的「貪婪搜尋造成的非單調假洞」，都是方法論上的實質內容，而不是失誤紀錄。§6.1 尤其重要：它說明了為什麼 concession 必須定義成「最小可行讓步」。

**（5）~~尚未成立、不能先寫的。~~ 【2026-08-30 Step 4 已回答】** 「rolling 是零 body 讓步」是**錯的**——rolling 的 hip 起伏是 `h + 14.2 mm`。而且答案正如這裡預期的那樣更有意思：故事確實要改寫成**兩種不同形式的 body 要求之間的取捨**（梯形 vs 三角形），而且有交叉點（h = 100–120 mm）與機制（swing 的 `min_hip_lift` 何時開始長）。

---

# 11. Scope 邊界與推廣路徑：矩形是不是太人工？

這一節回答一個在投稿前一定會被問、而且應該現在就想清楚的問題：整套研究都建立在
單一矩形障礙上，推廣到隨機崎嶇地時會不會是致命侷限？

## 11.1 侷限實際寫在程式的哪裡

```python
# legwheel/planners/hybrid/terrain_2d.py
if len(obstacles) > 1:
    raise ValueError("Day 3--5 TerrainProfile supports at most one rectangle.")

class SurfaceOrientation(str, Enum):
    HORIZONTAL = "horizontal"
    VERTICAL   = "vertical"        # 軸對齊寫在型別裡，沒有斜面
```

兩個假設的編碼廣度是反直覺的：

| 假設 | 消費點 | 分布 |
| --- | --- | --- |
| 單一障礙 | 50 處 | core 13（多為繪圖邊界；真正要改的是 `contact_detection_2d` 與 `terrain_query_2d` 各兩處）、experiments 26、tests 11 |
| 軸對齊 | 8 處 | 集中在 `terrain_query_2d` |

看起來最根本的（軸對齊）編碼最窄，看起來最偶然的（單障礙）散得最開。但**語意上完全相反**：
解除單障礙是機械性的；解除軸對齊會打壞三個語意——right rim 貼垂直面的 roll_up、
θ=17° 的平頂 wheel traverse、以及 `LEFT_RIM_READY` 的「後緣角」。

## 11.2 為什麼矩形是對的選擇（而不只是方便）

**(a) 矩形是輪子的最壞情況。** 輪子天生處理得了有界坡度的地形；連續但崎嶇的地面，
這台機器人直接滾過去，不需要 mode selection。真正打破 rolling 的是**高度不連續 +
接近垂直的面**。矩形恰好隔離出「選擇才有意義」的 regime。用平滑崎嶇地測，多半只會
得到「全部都 roll」這種沒有內容的結果。

**(b) 90° 是最難的面，不是最容易的。** 軸對齊看起來像簡化，但對 roll_up 而言垂直面是
最壞情況，面角 < 90° 只會讓 ROLL region 變大。**現在的結果是保守的，不是被美化的**——
這句話要寫進 paper，它把一個看似的弱點轉成方法的下界保證。

**(c) pipeline 真正依賴的抽象不是「矩形」，是 edge feature。**

```text
roll_up            需要  上升凸邊 + Δh + 面角
wheel transition   需要  一段可滾的 run
roll_down          需要  下降凸邊
swing              需要  起始面、目標面、中間的遮擋

矩形 = 上升邊 + 平 run + 下降邊，面角 90°
```

所以 `(h, L_top)` 這組軸，本質是 `(Δh, L_run)`——一個 **per-feature descriptor**，
推廣時活得下來。

## 11.3 真正的侷限，按 reviewer 咬人機率排序

**1. 單一孤立障礙（最高，且最便宜修）。** 真實崎嶇地的 feature 間距常小於腿的可及範圍，
而 **approach clearance 與 top length 這兩個軸，正好就是被鄰近 feature 吃掉的量**。
這直接打在 contribution 上，因為 contribution 是 per-feature 的決策規則。

有利的一面：§2.7 的 top-length 預算表，正是「第二個 feature 會吃掉的東西」。所以
**兩階台階的實驗有很大機會產出真結果**——決策會因為鄰居而改變，那比孤立 case 強。

**2. 平坦水平的頂面。** θ=17° wheel-mode traverse 與 `LEFT_RIM_READY` 都假設頂面可滾
且水平。斜頂會破壞後緣角的幾何前提。中等成本，這一輪不碰。

**3. 只有 90° 面。** 見 (b)，是保守方向，列 future work。

**4. 2D sagittal + 已知地形 offline。** 已 scope 掉，但 paper 要明講，不能靠讀者猜。

## 11.4 這一輪的處理方式

```text
免費（純寫作）   軸改用 feature 語彙 (Δh, L_run, face angle)；
                 scope 寫成 "structured terrain with discrete height discontinuities"
                 （台階、路緣、樓梯邊、瓦礫面），不要寫 "rough terrain"
半天（可選實驗） 解除 len(obstacles) > 1 guard，跑一個兩階台階 case
                 -> 見 Step 10（optional）
不做             斜面：型別要改，且三個語意要重做
```

## 11.5 一個順帶的好消息

**concession 框架比 binary feasibility 更容易推廣到多 feature。** 「最小 body 讓步」
是可合成的——下界取 max、軌跡約束取聯集；而「feasible / infeasible」沒有告訴你怎麼
合成兩個重疊 feature 的結果。§2.2 那個改動，順帶把推廣路徑也變好了。這值得在
paper 的 discussion 講一句。

---

# 12. 最重要的設計決策摘要

```text
1. 判準從 feasibility 改成 concession（向 body trajectory 索取多少讓步）
   理由：上升側二值可行性平手，沒有鑑別力；而且毫米級餘裕撐不起二值門檻

2. 共用軸只放 terrain 幾何 (h, L_top)；每個 primitive 對自己的內部自由度取最佳
   理由：theta_climb 與 theta_liftoff 不是同一個物理量，對齊它們是假對齊

3. Planner 輸出 body requirement，而不是消費 body trajectory
   理由：解掉 Day 10–11 與 Day 12 之間的循環依賴

4. approach 一律以前緣餘裕 c 表示，不用 hip x
   理由：腿的等效半徑隨 theta 變；而且 swing 失敗的機制本來就是餘裕問題

5. 上升與下降是兩個【獨立】決策：策略空間 = {ROLL_UP, SWING_UP} x {ROLL_DOWN, SWING_DOWN}
   + SWING_OVER = 5 種，不是 Day 6-7 §8 的 3 種
   理由：切換點在下降側，所以 ROLL+SWING 是預設；而 SWING+ROLL 專門攻擊
         top 太短、付不起 L_transition 的 case，在三分類裡不存在
   附帶：舊的 Strategy C 把「落 top 的全 swing」與「不落 top 的越過」混成一類，拆開

6. Sequence schema 是 segment 級，允許 RollingContact，且每段自帶取樣參數
   理由：wheel-mode 段沒有固定接觸點；連續性限制與取樣密度耦合

7. concession 定義成「最小可行讓步」，不是「搜尋停在哪」
   理由：現有貪婪 ladder 在 150 mm 產生假洞（已實測）

8. 第一版 decision rule 用 lexicographic，不調權重
   理由：權重會讓結論不可辯護；先讓 region 邊界由數據決定

9. 每一對 (ascent, descent) 有自己的最小 top-length 預算，L_top 是與 height 平起平坐的軸
   理由：full rolling 在矮但短的障礙上失敗，原因是 L_transition 付不起，不是爬不上去
```

**2026-08-30 由數據補上的三條（Step 2 close-out 與 Step 2b）**

```text
10. body requirement 多一種形狀：PINNED（等式），不是只有 LOWER_BOUND 與 TRACK
    理由：落地 contact state 被完全指定（point + rim + alpha + theta）之後，
          hip 高度就是輸出而不是旋鈕。抬高它不會讓落地變容易，只會變成另一個落地。
          compare_concessions 因此也拒絕 PINNED vs LOWER_BOUND 的排序。

11. 第 5 條的策略 #3（SWING_UP + ROLL_DOWN）在目前 planner 下【不成立】
    理由：站姿接觸永遠在 foot_rim, alpha = 0，要落到 left rim 就得穿過 alpha = -40 deg
          的 rim 分段接縫 —— 實測是約 29 deg 的關節不連續，加密取樣只會收斂上去。
          所以 #3 仍需在頂面做 rim handover，並沒有躲掉 L_transition。
    界線：擋住它的是【模型性質】（rim 分段），不是機器人做不到。
          rim 之間的接觸連續化之後要重測。Step 5 先用四個候選
          （Step 3 推翻 #2 之後再降為三個：#1 / #4 / #5）。

12. 第 2 條的「對自己的內部自由度取最佳」是【還沒做】的，不是已經做了
    理由：Step 2 固定 theta_liftoff = 60 deg，而 close-out 顯示最佳 theta 隨高度上移
          （h=100 -> 60 deg、h=150 -> 70 deg、h=200 -> 85 deg）。
          所以 Step 2 的 map 是上界；h = 200 mm 高估 34.6 mm。Step 5 之前必須補。
          Step 3 C 段確認下降側同一個模式（h=100 -> 60 deg、h=150 -> 85 deg），
          所以這是【兩邊】都要補，不是只有上升側。
```

**2026-08-30 由 Step 3 再補上的三條**

```text
13. 頂面長度是【雙向】約束，不是單向下界
    理由：rolling 要 L_top >= 196-249 mm（L_transition，下界）；
          swing off 要 takeoff <= 240 mm (h<=100) … <= 120 mm (h=200)（上限，隨 h 收縮）。
          所以 Step 5 的第一個判準是頂面長度的【方向】而不是代價比較：
          短頂面只有 swing 下得去，長頂面兩者都行、才輪到比 body 讓步。

14. 策略 #2（ROLL_UP + SWING_DOWN）不成立，而且是【兩道不同的牆】
    D 段：rolling 交過來的起飛姿態在 right rim, alpha = +82 … +107.7 deg，
          要跨 +40 deg 的接縫 —— Step 2b 的鏡像，同樣是模型性質，不可修。
    E 段：退化成先 retract 再起飛，撞的是下降側自己的 theta >= 35 deg 下限
          （F 段夾出來：32 失敗、35 可行、38 可行；與 takeoff 距離無關），
          而 Day 6-7 Step 6.5 把 retract 的 theta_target 寫死成 17 deg。
    界線：D 不可修，E 可修。不要把兩者合併成一句「#2 不可行」。

15. 下降側的 swing planner 會【拒答】而不是回報不可行
    理由：generate_swing_2d 在腳最後停在空中（final_rim=None）時丟 ValueError，
          因為它無法判定一個沒有觸地的 touchdown contact state。
          A 段有 153 次落在 11 格。這要記成 planner_refusals 而不是 infeasible，
          否則 map 會把「planner 表達不了」和「機器人做不到」混在一起。
```

**2026-08-30 由 Step 4 再補上的三條**

```text
16. 「rolling 是零 body 讓步」是錯的，而且錯得可以量化
    理由：rolling 的 hip 起伏 = h + overhead(theta_climb)，theta = 40 deg 時
          overhead 是常數 14.2 mm。第 2.2 節那句直覺必須從 paper 拿掉。
          真正的差別是【形狀】：rolling 是梯形（含平頂），swing 是三角形。

17. 無因次指標（任何有分母的比較）對「誰能自選什麼」極度敏感
    理由：swing 可自選 approach clearance，而較大的 clearance 在【零垂直代價】下
          加長前進距離，稀釋它自己的分母；rolling 的 clearance 被凍結在 40 mm。
          實測讓 swing 自選會在 h = 60 / 80 mm 翻盤。
    做法：主要結果一律用【對齊的自由度】，自選版本當敏感度檢查，兩者都寫進 CSV。
          沒有分母的比較（例如 §5.3 那條規則）不受影響。

18. rolling 的 traversal 不是姿態中性的
    理由：它站著（theta_climb）出發，停在 wheel mode（theta = 17 deg），
          淨差 -39.8 mm。那不是跨越障礙的代價。
          在 peak-to-peak 裡會抵銷，在絕對高度圖裡不會 —— 不先講會誤導。
          HipExcursion2D.is_posture_neutral 把它標出來。
```

**2026-08-30 由 Step 5 再補上的三條**

```text
19. L_top 不只選策略，還在 #1 內部選 theta_climb
    理由：required_top_length(theta) 單調【下降】（269.4 -> 205.0 mm），
          而 hip 起伏 overhead(theta) 單調【上升】（14.2 -> 49.4 mm）。
          兩個代價方向相反，最佳解永遠在約束邊界：
          「required_top_length 還放得下的最小 theta」，不需要搜尋。
    => rolling 的代價是 L_top 的【階梯】，不是常數。
       h = 120 mm 的 #1/#4 策略切換就是被這個階梯推動的，不是可行性造成的。

20. decision rule 對 lexicographic 順序【敏感】，約 1/3 的格子會變
    margin 放 body 前面 -> 208/609 格改變；roll 偏好放前面 -> 186/609。
    第一版不調權重是對的（邊界由數據決定），但【順序本身是一個未被論證的選擇】，
    引用 region 邊界時必須一起講。
    另外：預設順序下 tie-break 從未被觸發（body 代價永遠分得開前兩名）。

21. margin 門檻不能預設在 0.0 —— 那比模型自身容差銳利 1000 倍
    理由：三個 sweep 都有少數可行計畫的最小餘裕在 1e-4 ~ 1e-3 mm 量級，
          因為 swing 最緊的一點通常就是【觸地】，那裡餘裕依定義為零。
          planner 自己的 collision_tolerance_m 是 1 mm。
          預設 0.0 曾在 h = 60/80 mm 開出一條【假的無解帶】。
    做法：預設 None（信任 planner 已經判過），門檻做成敏感度回報。
```

**2026-08-30 討論後固定下來的八條原則（改任何文件之前先讀）**

```text
1. 不得把 prototype、提案或 unresolved transition 寫成【已完成能力】。
2. 不得把「現有單腿 primitive 直接交接失敗」寫成「機器人物理上不可行」。
   用 §5.5 的四個標籤。
3. TOP_REPOSITION 是這次討論後的後續方向，【目前尚未實作】（§5.6）。
4. Day 10-11 只完成單腿 motion selection 與 Step 9 交接；
   不在這一階段強行解四腳支撐問題。
5. h_clear = 30 mm 只是 nominal regression parameter，不是硬體安全值。
6. decide_2d(h, L_top) 是【目前已掃範圍內】的單腿 2D 決策函式，
   不是對任意地形的成功保證。
7. rolling theta 有選擇邏輯；swing theta 只有部分 sweep；beta 主要是 IK 輸出。
   【尚未】進行 theta / beta 的全軌跡聯合最佳化 —— 不要寫成已最佳化。
8. 後續宣稱「多數 obstacle 都可以通過」前，必須先定義 obstacle distribution
   並做 coverage test（Step 10 優先 6）。
```

**2026-08-30 由 Step 7 再補上的三條**

```text
22. 一個決策若不含 repair 旋鈕，就【不足以重建它選的動作】
    理由：Step 2 / Step 3 是靠 liftoff_rise / touchdown_drop / duration_scale
          走到可行的。只回報 body 旋鈕的決策，重建時會重現 sweep 早就修掉的碰撞。
          Step 5 的完成標準本來就寫了「內部參數」—— 這一步證明那包含 repair。
    界線：repair 旋鈕【不能】折進 body_deviation_m。它們是軌跡整形不是 body 讓步，
          折進去會改變 Step 5 比較的意義。

23. 不要把「成功後回報的消耗量」當成「可行性前提」
    理由：required_top_length_m 是一次成功 traversal 消耗了多少頂面。
          Day 6-7 Step 12R 自己的 sweep 在正好等於它的長度上【失敗】。
    但也不要一律加一個邊際值：Step 7 在另一個 theta 上夾出 (220, 225]，
          而 required 的 222.5 mm 落在區間內 —— 一律 +10 mm 會超調。
    做法：實測過的 theta 用實測值，其餘用 required 並【標記為未驗證】。
          代價是界線在 theta 上不再單調（實測含差額、需求不含）；留著讓它可見。

24. 重跑別人的實驗時，要沿用它自己的 settings bundle，不要自己重建
    理由：ObstacleSpec2D.arc_samples 預設 241 但 Day 6-7 的 sweep 跑 121，
          而 max_seam_bridge_m 必須跟 arc_samples 【配對】
          （seam_bridge_for_sampling_m(121) = 17.5 mm vs 預設 5 mm）。
          兩個預設都不對，結果 traversal 在 sweep 說可行的地方失敗。
    做法：用 SweepSettings2D 建 obstacle / initial_state / constraints。
    教訓：那兩次失敗一度被我當成第 23 條的證據 —— 錯誤歸因。
          先確認自己重現得了原實驗，再拿失敗當數據。
```

**2026-08-31 由 Step 9 再補上的三條**

```text
25. PINNED 不是落地的特例，是【每一個接觸瞬間】—— 包含每一條 swing 的頭尾兩幀
    理由：腳踩在面上時 hip_z 與 theta 互相決定（實測 theta=60 deg -> hip_z=219.4 mm，
          body ±10 mm -> theta 54.4 / 65.6 deg）。第 10 條把 PINNED 定義成
          「落地 contact state 被完全指定」的結果，但更基本的條件是【有接觸】。
    後果：把整條 swing 標成 LOWER_BOUND，下游會在觸地那一刻抬高 body，
          plan.valid 仍然成立而落地姿態已經換掉。
    界線：swing 的【中段】仍然是 LOWER_BOUND —— 那裡腳在空中，body 確實可以更高。

26. segment 級的純量 hip_z_min 是 envelope，不是 timeline
    理由：Step 6 的 builder 對一條 swing 只留 max(hip 軌跡)。那是正確的 envelope
          （Step 5 就是拿它決策的），但沿路用它會在 #4 的兩段各多抬 80 mm
          —— 剛好是障礙高度，等於在 swing 還沒起跳就叫 body 站到頂面高度。
    做法：交付檔兩個都寫。比較策略用 envelope，沿路走用 per-knot 的要求。

27. 一個為某個情況設計的量或標籤，套到不適用的情況上，會生出沒有人主張過的結論
    理由：Step 9 第一版對 TRACK 段也算了 envelope = max(profile)，於是 #1 被報成
          「被多約束 136.9 mm」—— 但 TRACK 的要求【就是】那條 profile。
          同一次：blocked pair 的 x_is_monotonic 寫 True（空集合上為真），
          那什麼也沒說卻會被讀成「檢查過了，沒問題」。兩個都改成留白。
    這與 Step 9 前置修正踩到的是【同一類錯】（那次是把幾何界線寫成交接失敗）。
    做法：一個量或標籤要推廣範圍之前，先檢查【每一類對象】落在哪裡。
```

**2026-08-31 — Day 10–11 結束**

```text
Step 0-9 全部完成。交付物：day10_11_step9_body_requirements.csv
（392 列 / 365 個 knot，278 硬 / 87 偏好，以 x 為自變數，
  用一個只 import csv 的 reader 稽核過）。
下一步是 §7 Step 10 的六個方向，或直接進 Day 12。
```
