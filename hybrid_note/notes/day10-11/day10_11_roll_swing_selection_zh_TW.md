# Hybrid Gait Day 10–11：Roll-vs-Swing Motion Selection

> **用途**：記錄 Day 10–11 的研究發想、為什麼原始規劃在這兩天開始之前就已經被 Day 6–7 / Day 8–9 的數據推翻、最後選定的方法，以及可以逐步交給 Codex 的實作項目。
> 之後撰寫 paper 時，可回頭追溯為什麼 motion selection 的判準是「向 body trajectory 索取的讓步」，而不是「哪一種 primitive 可行」。
>
> **研究範圍**：offline、單腳、2D sagittal。Day 10–11 決定 **ROLL / SWING 如何選**，並把選擇的結果表示成一條可執行的 segment sequence。四腳 timing、body trajectory 生成、ABAD 都不在範圍。
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

對照舊名稱：

| # | ascent | descent | 舊名 | 什麼時候勝出 |
| --- | --- | --- | --- | --- |
| 1 | ROLL_UP | ROLL_DOWN | Strategy A | 低矮且 top 夠長 |
| 2 | ROLL_UP | SWING_DOWN | Strategy B | h 超過 roll_down 天花板（160 mm） |
| 3 | **SWING_UP** | **ROLL_DOWN** | **缺** | **top 太短，付不起 L_transition** |
| 4 | SWING_UP | SWING_DOWN | Strategy C 的一種 | 高，且 top 夠長可落腳 |
| 5 | — SWING_OVER — | Strategy C 的另一種 | top 很短 |

兩個發現：

**(a) 舊的 Strategy C 把 #4 和 #5 混成一類。** 它們的 top-length 需求完全不同——#4 要在 top 上
落腳再起跳，#5 完全不碰 top。混在一起會讓 decision map 上的 region 邊界失去意義。

**(b) #3 是被漏掉的那一種，而且它正好攻擊 rolling 最硬的限制。**

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
ROLL_UP  + ROLL_DOWN     L_transition                     0.20–0.27 m（已量測）
ROLL_UP  + SWING_DOWN    roll-up 出口 -> 起跳距離          待量
SWING_UP + ROLL_DOWN     落點 -> 後緣 LEFT_RIM_READY       待量（若能直接落在 READY，可極短）
SWING_UP + SWING_DOWN    落點 -> 起跳距離                  待量（落 top 者中最短）
SWING_OVER               0（不碰 top），但需 stride >= L_top
```

> **越靠 swing 的組合，需要的 top length 越短。**

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
那筆 retract 又會吃掉 top length，優勢就縮小了。這要在 Step 3 明確測，不能假設。

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

---

# 7. 實作步驟

每一步都可以獨立交給 Codex。順序有依賴，不要跳。

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

---

## Step 4 — 把 rolling map 換成同一種幣別

### 目的

§5.3 那個問題：rolling 對 body 的要求到底是什麼？這一步可能是整份 note 研究價值最高的一步。

### 任務

1. 重跑（或從既有 `day6_7_step11r_sweep_trajectories.csv` 重新萃取）每個可行 cell 的 **hip_z 軌跡**。
2. 對每個 `(h, L_top)`，在 θ_climb 上取最佳，填出 `RollConcession2D`：

```text
best_theta_climb_deg
feasible_theta_span_deg      <- 直接來自 day6_7_step11r_feasible_theta_ranges.csv
required_hip_z_range_m       <- 新量的
```

3. 把 `day6_7_step12r_minimum_top_length.csv` 的上下界併進來，讓 `L_top` 成為正式座標軸。

### 輸出

```text
day10_11_step4_roll_concession.csv
day10_11_step4_roll_hip_profile.png     # rolling 逼著 hip 走的軌跡，疊在 swing 的 hip 下界上
```

### 完成標準

- 對 Day 6–7 所有可行 cell 都有 `RollConcession2D`。
- **明確回答**：rolling 的 hip 軌跡約束，和 swing 的 hip 下界，哪一個對 body trajectory 較苛刻？答案是什麼都可以，但必須有數據，不能沿用直覺。

---

## Step 5 — 疊圖與 decision rule

### 目的

Day 10–11 的核心結果，也是 paper 的 Figure D。

### 任務

1. 共用軸 `(h, L_top)`。每格放**五個**候選策略（§2.6），不是兩個 primitive：

```text
#1 ROLL_UP  + ROLL_DOWN
#2 ROLL_UP  + SWING_DOWN
#3 SWING_UP + ROLL_DOWN        <- 若 Step 2b 證明成立
#4 SWING_UP + SWING_DOWN
#5 SWING_OVER
```

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

---

## Step 9 — Day 12 交接物

### 任務

把 Step 7 的 sequence 導出成 body requirement timeline：

```text
t 或 x -> hip_z 下界 / hip_z 軌跡 / 該段屬於哪個 primitive
```

### 完成標準

- Day 12 的四腳 timing 可以只讀這個檔案，不需要回頭讀 Day 6–7 / 8–9 的任何內部結構。
- 檔案裡明確標示哪些是**硬約束**（違反則 sequence 失效），哪些是**偏好**。

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
    day10_11_step0_scene_alignment.csv
    day10_11_step2_swing_onto_sweep.csv
    day10_11_step2_min_hip_lift_map.png
    day10_11_step2_binding_ceiling_map.png
    day10_11_step2b_swing_to_left_rim_ready.csv
    day10_11_step2b_alpha_seam_distance.png
    day10_11_step3_swing_off_sweep.csv
    day10_11_step3_min_hip_hold_map.png
    day10_11_step4_roll_concession.csv
    day10_11_step4_roll_hip_profile.png
    day10_11_step5_decision_map.csv
    day10_11_step5_figure_d.png
    day10_11_step5_cost_gap.png
    day10_11_step5_top_length_slice.png
    day10_11_step5_top_length_budget.csv     # §2.7 那張表的實測版
    day10_11_step7_pair1_roll_roll_frames.csv / _summary.csv
    day10_11_step7_pair2_roll_swing_frames.csv / _summary.csv
    day10_11_step7_pair3_swing_roll_frames.csv / _summary.csv
    day10_11_step7_handoff_report.csv
    day10_11_step8_regression.csv
    day10_11_step9_body_requirements.csv
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

**（5）尚未成立、不能先寫的。** 「rolling 是零 body 讓步」目前是直覺，Step 4 才會給答案；如果 rolling 的 hip 軌跡約束比 swing 的 hip 下界更苛刻，故事要改寫成「兩種不同形式的 body 要求之間的取捨」，那反而更有意思。

---

# 11. 最重要的設計決策摘要

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
