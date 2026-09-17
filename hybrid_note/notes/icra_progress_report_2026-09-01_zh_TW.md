# ICRA 進度彙報與初稿前風險盤點（2026-09-01）

> 用途：跟老師報告目前已完成什麼、Hybrid 整體規劃、整機為何還沒跑起來，以及初稿前最短的收斂路徑。  
> 結論邊界：本文把「已驗證」、「離線 prototype」、「初步探索」與「尚未完成」分開；單腳幾何成功不等於整機、控制器或實機成功。

## 1. 一句話現況

目前不是「什麼都沒有」，而是已完成一條相當完整的**離線規劃研究鏈**：多 rim 接觸模型 → 單腳滾動／擺動 primitive → 地形條件下的 motion selection → 四腳時間軸、body requirement、support 與 validation。

真正的缺口是：目前整機管線會產生**結構化 infeasible 結果**，還沒有得到一條可交給 controller 的 feasible 四腳軌跡，更沒有 Webots／實機比較。初稿已有單腳幾何結果可寫，但 whole-robot results、baseline comparison、reference 與實驗章仍是空的，因此本週直接以「完整實機 Hybrid 論文」為目標風險很高。

## 2. 目前已完成的工作

### 2.1 接觸模型與規劃架構（Day 1–5）

- 建立 foot / right / left 三種實體 rim 的 2-D contact query 與 contact state 表示。
- contact state 保留 rim、rim parameter、world contact、collision／terrain gap 等資訊，供 rolling 與 swing 共用。
- 已把 planner、trajectory schema 與 runtime playback 的責任分開；目前 scope 是已知地形上的 offline planning，不包含 perception、MPC、RL 或 online replanning。

這一段的價值是：後面的 rolling、swing 與 motion selection 都建立在同一套接觸語意上，不是各自畫軌跡。

### 2.2 單腳連續滾動（Day 6–7，已驗證的離線 2-D prototype）

- 完成 right-rim climb → top transition → left-rim descent → 回到下層地面的完整十階段 traversal。
- nominal case：障礙高 0.10 m、頂面長 0.35 m、`theta_climb = 60 deg`，共 299 個 accepted frames。
- sampled minimum collision margin 為 2.40 mm；最大 stage handoff gap 為 3.04 mm，已如實保留，沒有稱為完全連續解。
- 70 個 `(height, theta)` case 中有 42 個完整 traversal；0.16 m 那一列在測試 grid 上沒有完整成功。
- 已量出 top transition 對平台長度的需求；這說明「能爬上邊緣」不等於「頂面長度足夠完成 rim transition」。

此結果只能支持 sampled single-leg geometric/contact feasibility，尚未包含載重、摩擦、力矩、動態、支撐或實機。

### 2.3 單腳 Cartesian swing（Day 8–9）

- 建立 Cartesian Bezier swing request／generator，包含 rim-point IK、collision／clearance、touchdown configuration 與 touchdown velocity checks。
- 可從 rolling／standing state 產生 swing segment，並使用同一份 contact／segment schema。
- `clearance = 30 mm` 是目前 planner 的 nominal 設定，不是硬體安全性證明。

### 2.4 ROLL／SWING 選擇與單腳 sequence（Day 10–11）

- 建立參數化 `decide_2d(h, L_top)`，先做 feasibility，再比較 body concession；不是寫死 4／10／19 cm 的分支。
- 已支援並比較 `ROLL_UP`、`ROLL_DOWN`、`SWING_UP`、`SWING_DOWN`、`SWING_OVER`。
- 三條仍可用的策略可各自組成 collision-free single-leg sequence。
- 已輸出 `day10_11_step9_body_requirements.csv`（392 列），作為四腳整合的正式 handoff。
- 已清楚定位兩種 direct handoff 失敗：它們是現有 primitive 無法直接交接，可能需要其他腿支撐下的 `TOP_REPOSITION`；不能寫成機器物理上不可能。
- Day 10–11 regression：181 passed。

#### 2.4.1 上障礙時，如何決定要 ROLL 還是 SWING？

核心不是簡單的：

```text
if roll 可行：選 roll
else：選 swing
```

因為在許多地形上 roll 和 swing 都做得到；只看二元的 feasible／infeasible 無法說明為什麼選其中一個。目前採用的判斷是：

> **先排除不符合幾何、運動學、碰撞、clearance 與交接條件的候選，再比較每個候選向整機 body trajectory 索取多少讓步。**

完整判斷分成三層。

**第一層：這個 motion pair 能不能完成整個障礙，而不只是一個邊緣？**

Planner 原先把上升和下降拆開考慮：

```text
ascent  = ROLL_UP  or SWING_UP
descent = ROLL_DOWN or SWING_DOWN

另外保留 SWING_OVER：不落在頂面，直接跨過整個障礙
```

每個候選必須同時通過：

- IK／joint range；
- sampled collision 與 clearance；
- touchdown contact／rim 合法性；
- 上升出口和下降入口能連續交接；
- 平台頂面長度足以容納 transition、落腳與起跳；
- `SWING_OVER` 的 stride 足以跨過平台。

這裡特別重要的是 `L_top`：

- `ROLL_UP + ROLL_DOWN` 需要約 0.20–0.27 m 的頂面完成 inter-rim transition，所以頂面太短會失敗；
- `SWING_UP + SWING_DOWN` 需要在頂面同時放得下合法落點與起跳距離；
- `SWING_OVER` 不需要頂面落腳，因此不受最小頂面長度約束，但平台越長、所需 stride 越大，反而更難。

因此 obstacle height 不是唯一判據；**height 決定能否搆到與需要多少抬升，top length 決定哪一種 transition 放得下。**

**第二層：如果多個候選都可行，比較它們要求 body 付出的 concession。**

目前比較的不是能量，而是幾何／運動學上的 body requirement：

- Swing 通常給 body 一個 `LOWER_BOUND`：例如 hip 至少必須抬到某個高度，但更高仍可接受。
- Roll 通常給 body 一條 `TRACK` trajectory：hip 必須隨接觸幾何追蹤指定高度變化。
- 一個完整策略若有兩段，hip lower bound 取較高者，trajectory constraints 取聯集。
- 每個 primitive 都先在自己的內部自由度中找「最小可行讓步」，再拿不同 primitive 比較，避免把搜尋順序碰巧找到的結果當成代價。

所以這裡的「代價較小」不是已證明比較省電，而是：**要求整機 body 改變得較少、或施加的 body trajectory 約束較弱。** 真正 energy／COT 仍要靠實機電流、電壓或 torque 資料驗證。

Roll 也不是零代價。固定 `theta` 滾動時，hip 會沿幾何軌跡起伏；量測顯示 rolling 的 hip excursion 約為 `obstacle height + 14.2 mm`。因此實際比較的是「roll 的整段 TRACK」和「swing 的 hip lower bound」，不是預設 roll 一定比較好。

**第三層：使用固定、可解釋的 lexicographic ranking，不先調任意權重。**

```text
1. 先選 feasible 且 clearance 超過門檻者
2. body deviation 較小者優先
3. clearance margin 較大者優先
4. 前三項平手時，才偏好 roll 較多者
```

這代表 planner 不會為了保留 rolling 而接受碰撞，也不會把 1 mm clearance 和 20 mm clearance 當成一樣。最後的 roll preference 只是 tie-break，不是最高優先規則。

目前單腳 planner 的實際結果並沒有保留完整 2×2：

| 候選 | 目前狀態 | 原因／適用區域 |
|---|---|---|
| `ROLL_UP + ROLL_DOWN` | 可用 | 較低障礙且頂面足以完成 rim transition |
| `ROLL_UP + SWING_DOWN` | direct handoff 未成立 | roll-up 出口在 right rim，接 swing-down 時遇到 `alpha=+40 deg` seam／theta 條件 |
| `SWING_UP + ROLL_DOWN` | direct handoff 未成立 | foot-rim touchdown 無法直接交給 `LEFT_RIM_READY`，遇到 `alpha=-40 deg` seam |
| `SWING_UP + SWING_DOWN` | 可用 | 頂面能容納落腳與起跳；常用於較高障礙 |
| `SWING_OVER` | 可用 | 低且短、可以一次跨過，不接觸頂面 |

因此目前 decision map 實際出現三種策略：`ROLL+ROLL`、`SWING+SWING`、`SWING_OVER`。另外有一塊 `h >= 100 mm` 且 `L_top` 約小於 205–235 mm 的能力缺口：平台太高而無法 swing over，同時又太短而放不下 rolling transition 或 top landing。這是目前 primitive 集合的缺口，不應寫成機器物理上不可能。

在 609 個 `(h, L_top)` sampled cells 中，目前分布為：

| 決策結果 | Cells |
|---|---:|
| `SWING_UP + SWING_DOWN` | 210 |
| `SWING_OVER` | 129 |
| `ROLL_UP + ROLL_DOWN` | 98 |
| 目前 primitive 集合無解 | 172 |

有 257 格同時存在兩個候選、10 格同時存在三個候選；這些重疊區才真正展示 concession ranking 的作用，而不是只在「roll 做不到」時被迫 swing。

一個很適合口頭報告的案例是 160 mm 障礙：roll-up 本身 10/10 成功，要求的 hip peak-to-peak 為 80.1 mm；但 roll-down 0/10，使完整 roll traversal 無法成立。目前只能改用 swing-up，而它要求 220.0 mm，約為 2.7 倍。這說明：

1. 上升的 roll primitive 本身可能很有價值；
2. 真正失敗的是後續 direct handoff／descent capability；
3. 四腳 `TOP_REPOSITION` 若能補上，未來可能重新啟用現在被拒絕的混合策略。

報告時可直接搭配：

- `hybrid_note/notes/day10-11/day10_11_step5_figure_d.png`：terrain geometry → motion class；
- `hybrid_note/notes/day10-11/day10_11_step5_cost_gap.png`：多個候選皆可行時的 concession 差；
- `hybrid_note/notes/day10-11/day10_11_step5_top_length_slice.png`：固定高度後，策略如何隨頂面長度切換。

#### 2.4.2 目前踏點如何設定，以及 decision map 的證據邊界

目前還不是在整個 terrain surface 上連續搜尋最佳 foothold，而是**先定義有限的 touchdown／takeoff 候選，再對每個候選做 IK、碰撞、接觸、交接與 body-concession 檢查**：

| Motion | 目前的位置設定 |
|---|---|
| `SWING_UP` | target hip 固定在障礙前緣後方 160 mm，使用 `theta=60 deg`、foot-rim standing contact；實際接觸點由幾何驗證 |
| `SWING_DOWN` | 平台上的 takeoff distance 掃 80–240 mm；低地 landing hip 固定在後緣後方 200 mm |
| `SWING_UP + ROLL_DOWN` | touchdown 不能只是站上頂面，必須直接滿足 `LEFT_RIM_READY` 的 rim、alpha、theta／beta 與 hip-height 條件 |
| `ROLL_UP + ROLL_DOWN` | 沒有單一離散踏點；contact 沿 rim 與頂面連續演化，受約 0.20–0.27 m transition-distance 約束 |
| `SWING_OVER` | 前後低地採鏡像起落配置，approach clearance 掃 40／60／100 mm，平台越長所需 stride 越大 |

因此目前 Figure D 應解讀為：

> **在目前指定的 touchdown／takeoff 候選規則與 sampled parameter grid 下，哪個 motion strategy 可行且 body concession 較小。**

不能解讀為：

> 已經遍歷障礙物表面所有踏點並找到全域最佳策略。

這個限制可能會影響目前的 region boundary，尤其 `SWING_UP` 固定前緣後 160 mm，可能讓部分「目前無解」cell 只是候選踏點不足，而不是機器幾何上沒有任何解。後續應把 foothold position 升級成 planner variable，並保留 `NOT_MEASURED`／`OUT_OF_ENVELOPE`，避免把尚未搜尋的位置寫成 infeasible。

### 2.5 四腳整合與失敗診斷（Day 12 Step 0–11）

已實作並驗證：

1. segment semantic 與 `CUT`／`HANDOVER` 邊界契約；
2. 平地 `FOOT_RIM_ROLL + RECOVERY_SWING` nominal cycle；
3. 四腳 world-frame initialization 與參數化 terrain registration；
4. 使用既有 Walk phase 的四腳共同時間軸；
5. per-leg sequence 映射到共同時間；
6. body requirement merge 與 conflict reporting；
7. 以實際 contact points 建立三腳 support triangle 與 signed margin；
8. `TOP_REPOSITION` support gate；
9. 完整 body／四腳 joint／contact trajectory container；
10. whole-body validator 與結構化 failure records；
11. flat、4 cm、10 cm、19 cm 共用同一個 terrain-parameterized entry point；
12. paper metrics exporter，且明確禁止在沒有資料時聲稱 energy／CoM 結果。

Day 12 已記錄 324 passed；連同 Day 10–11 共 505 passed。這證明軟體契約與失敗檢查可重現，不代表 locomotion 成功。

## 3. Hybrid 的整體規劃

目前的 Hybrid 不是單純的「遇到障礙就把 Walk 和 Wheel 拼起來」，而是：

```text
Known TerrainProfile + robot initial state
    -> multi-rim contact / collision query
    -> per-leg ROLL or SWING feasibility
    -> motion selection + body requirement
    -> four-leg gait timing and support allocation
    -> merge body requirements
    -> support / contact / joint / timing validation
    -> feasible synchronized trajectory
       OR structured infeasible result with reason
    -> controller conversion
    -> Webots / hardware playback and measurement
```

平地 nominal cycle 目前定義為有限的 `FOOT_RIM_ROLL`，到 stroke endpoint 後做 `RECOVERY_SWING`，再落到下一個 rolling stroke。障礙附近則由 terrain-aware selector 決定該腳 roll up/down、swing up/down 或 swing over。四腳層負責錯相、支撐、body motion 與單腳無法自行完成的 reposition。

論文可主張的核心目前應放在：**terrain-parameterized multi-rim contact representation，以及 rolling／swing primitive 在整機約束下可被組合、驗證或明確拒絕的 offline planning framework**。不要先承諾節能，因為尚無電流／力矩／COT 證據。

### 3.1 ICRA 完整規劃路線

為了把現有 prototype 收斂成一篇完整 ICRA 工作，建議把研究主線固定成四層：

```text
Layer 1  Terrain-aware contact and foothold candidates
         已知地形 -> 多 rim 接觸候選 + touchdown/takeoff 候選

Layer 2  Per-leg motion planning and selection
         每個候選生成 ROLL / SWING primitive
         -> feasibility -> body concession -> motion selection

Layer 3  Four-leg whole-body integration
         gait timing + body requirement merge + ABAD/gamma support adjustment
         -> synchronized body/joint/contact trajectory or structured infeasible

Layer 4  Evaluation
         offline regression -> whole-body animation -> Webots/controller
         -> hardware Wheel/Walk/Hybrid matched comparison
```

各層目前與最終目標如下：

| 層級 | 現況 | ICRA 最終目標 |
|---|---|---|
| Terrain／contact | 已有參數化矩形地形與 foot/right/left rim query | 保留統一 `TerrainProfile`；加入可行 surface interval 與 foothold candidates，不寫尺寸特例 |
| Foothold selection | `SWING_UP` 等使用固定位置或離散 sweep | 把 touchdown／takeoff x、rim、alpha 與必要 posture 變成候選變數；先求可行區間，再以 concession／margin 排序 |
| Single-leg planning | Rolling、Cartesian swing、三種可用策略已有 sampled evidence | 同一介面輸出 segment、body requirement、clearance、failure reason 與 provenance |
| Whole-body planning | 四腳 pipeline 已接通；levelled rolling 初步解掉 body conflict，support 仍失敗 | 用 theta compensation、gamma／ABAD 或 phase adjustment 取得至少一條 validator 全通過的同步軌跡 |
| Visualization | 已有單腳影片、四腳 timeline 與 diagnostic plots | 先完成 constraint-aware whole-body kinematic animation，明示 infeasible frame；不可稱 dynamics simulation |
| Controller／simulation | 尚未完成 | motor-command conversion、Webots tracking、碰撞／滑動與 safety gate |
| Hardware | 尚未開始 | Flat → 4 cm → 10 cm；19 cm 只作 challenge，依 capability 結果決定是否執行 |
| Paper evaluation | 單腳 sweep 已有；whole-robot table 為空 | 同條件 Wheel／Walk／Hybrid comparison，報 success、time、tracking、body attitude、margin；有可靠電氣資料才報 energy |

Foothold planner 的建議實作順序是：

1. 將 obstacle top 表示成扣除前後緣安全距離後的合法 touchdown interval。
2. 對 interval 離散取樣 touchdown x；每個 x 再產生合法的 `(rim, alpha, theta, beta)` contact candidates。
3. 對 `SWING_UP`、`SWING_DOWN` 與需要的 `TOP_REPOSITION` 分別生成與驗證 trajectory。
4. 將 touchdown 和後續 takeoff／roll-down 一起評估，避免只選「容易落下、卻接不了下一段」的局部最佳點。
5. 先以 lexicographic rule 排序：完整 sequence feasible → body concession → clearance／edge margin → roll continuity；暫不加入沒有量測依據的 energy weight。
6. 將固定 160 mm 與最佳候選的 decision map 並排比較，量化原本 172 個無解 cells 中有多少是 foothold restriction、多少仍是 primitive capability 缺口。

這項擴充的 paper 價值不只是提高成功率，也能把目前的 motion-selection claim 從「固定踏點下選 ROLL／SWING」提升為：

> **joint contact-state, foothold, and motion-primitive selection under whole-body concession and continuity constraints.**

但若本週只交初稿，應把這項列為明確的 next method step，不要因為它尚未完成而延後整理現有 single-leg evidence 與 Day 12 negative results。

## 4. 為什麼「單腳規劃了，但合不進整機」

這不是一個單一 bug，而是四個不同層級的約束在整機後才出現：

| 層級 | 已量到的問題 | 意義 |
|---|---:|---|
| Timing | 0.6 s swing window 要塞入 1.2 s 已規劃動作，overrun = 2.000x | 單腳 path 存在，不代表四腳 gait window 放得下 |
| Body height | 固定 theta rolling 時，三隻 stance legs 的 hard body-height 要求最多差 15.329 mm | level rigid body 無法同時滿足三腳；不能用平均值掩蓋 |
| Support | 五個 swing 全部低於 planning floor；minimum margin 約 0 mm | Walk phase + gamma=0 幾乎沒有準靜態餘裕 |
| TOP_REPOSITION | 兩個 case 在 support gate 就被拒絕 | 單腳所缺的 reconfiguration 尚未取得外部支撐條件 |

此外還有兩個模型／資料缺口：

- recovery beta 速率需求是 rolling 的 10.553 倍，但原模型沒有 joint speed limit，尚不能判斷 actuator 是否可行；
- beta 在 Hybrid 被當作可累積的 rotation coordinate，但舊 swing planner 用 `±40 deg` workspace guard，兩邊語意尚未統一。

因此目前最精確的說法是：**四腳架構已接上，validator 也成功指出它為何不可執行；尚未完成的是 constraint resolution，而不是單腳程式根本沒有接進去。**

## 5. 最新但尚未 freeze 的初步突破

`hybrid_note/scripts/experiments/day13_levelled_rolling_2d.py` 已做一個 opt-in 的 theta-compensated rolling probe：讓各 stance leg 調 theta 以維持固定 hip height，而不是固定 theta 讓 hip 畫弧。

目前輸出 `hybrid_note/notes/day12/day13_levelled_rolling.csv` 顯示：

- body conflicts：227 → 0；
- usable body samples：1/121 → 121/121；
- 加上 continuous chaining 後，Step 9 failed checks 從 3 類降為 1 類；
- 最後仍剩 `support_margin`，5/5 swings unstable。

這是很重要的方向性結果：Step 5 高度衝突可由 per-leg theta compensation 消除，顯示先前的 `INFEASIBLE` 是「fixed theta + Walk phase + level body + hard TRACK」這組假設不相容，不是整台機器做不到。

但這仍是 preliminary offline probe：尚未寫入 Day 12 freeze／正式測試，且速度、力矩、碰撞、載重與實機皆未驗證，報告時不應稱為整機成功。

## 6. 可直接展示的 notebook／影片／結果

建議跟老師展示時控制在三組，不要從頭播放全部 notebook。

### A. 單腳完整 rolling 證據（最成熟）

- Notebook：`hybrid_note/notes/hybrid_gait_day6_7_right_up_left_down_dashboard.ipynb`
- 互動 traversal：`hybrid_note/notes/day6-7/day6_7_showcase_traversal.html`
- 關鍵畫面：`hybrid_note/notes/day6-7/day6_7_showcase_key_frames.png`
- feasibility map：`hybrid_note/notes/day6-7/day6_7_step11r_feasibility_map.png`
- top-length 結果：`hybrid_note/notes/day6-7/day6_7_step12r_transition_budget.png`

### B. ROLL／SWING 選擇與 body requirement

- Notebook：`hybrid_note/notes/hybrid_gait_day10_11_motion_selection_dashboard.ipynb`
- 決策圖：`hybrid_note/notes/day10-11/day10_11_step5_figure_d.png`
- sequence：`hybrid_note/notes/day10-11/day10_11_step7_sequences.png`
- body requirement：`hybrid_note/notes/day10-11/day10_11_step9_body_requirements.png`

### C. 四腳整合與目前卡點（老師最需要看）

- Notebook：`hybrid_note/notes/hybrid_gait_day12_whole_body_dashboard.ipynb`
- nominal cycle 動畫：`hybrid_note/notes/day12/day12_step1_cycle_animation.gif`
- 四腳 timing：`hybrid_note/notes/day12/day12_step3_timeline.png`
- body-height conflict：`hybrid_note/notes/day12/day12_step5_body_trajectory.png`
- support margin：`hybrid_note/notes/day12/day12_step6_stability.png`

建議口頭順序：先播 A 證明 primitive 確實存在，再看 B 說明 planner 不是 hard-coded，最後用 C 說明整機 gate 抓到的真問題與 Day 13 修正方向。

## 7. 論文目前能寫到哪裡

`ICRA/main.tex` 已有約 4 頁 working draft，包含：problem framing、contact representation、single-leg rolling method、70-case sweep 與 limitations。現在可以保留的 quantitative evidence 主要是 Day 6–7 single-leg 2-D 結果。

尚未完成且不能假裝已有結果的部分：

- verified related-work references 與正式 bibliography；
- author／affiliation；
- 完整四腳 feasible trajectory 與 whole-robot quantitative results；
- Wheel／Walk／Hybrid 公平 baseline；
- Webots 與實機 tracking；
- success rate、tracking error、body attitude、support margin、energy／COT；
- 系統圖、整機結果圖與最終 discussion／conclusion。

目前 Results table 仍是空的 `--`。若本週必須交「初稿」，應將它定位成**方法與單腳離線結果完整、整機與實驗結果明確標 TODO 的 internal draft**，而不是 submission-ready manuscript。

## 8. 後續未完成工作與優先順序

### P0：今天先和老師確認 paper scope（不確認會一直擴張）

- 初稿是否接受以 offline planning + single-leg evidence 為主，whole-body／hardware 先列 ongoing？
- 本次 paper 的必要主張究竟是「planner representation」，還是一定要有完整 Hybrid 實機成功？
- 若一定要實機，本週是否接受先做 Walk／Wheel baseline 與最小 Hybrid pilot，而不是一次完成 19 cm challenge？

### P1：先讓整機產生一條最小 feasible offline trajectory

1. 正式化並測試 theta-compensated／levelled rolling，先關閉 Step 5 body conflict。
2. 用 gamma／ABAD 或重新選 gait phase 改善 support margin；這是目前最後仍擋住全部 swing 的 hard gate。
3. 重排 swing duration 或延長 cycle，解 2.000x overrun；duration 是 scheduling choice，不是物理定律。
4. 統一 beta 語意，加入實際 joint speed／position limits。
5. 重新跑 contact、collision、joint、timing、support validation；只有全過才輸出 controller candidate。

### P2：控制器／模擬 gate

1. 把 synchronized body／joint trajectory 轉成實際 controller 所需的 motor command CSV。
2. 先做 flat-ground nominal cycle，再做 4 cm smoke test，再做 10 cm main Hybrid case。
3. Webots 檢查 tracking、碰撞、滑動、body attitude 與安全停止條件。
4. 19 cm 保留為 challenge；目前 traversal sweep 資料不足，不能為了成功寫高度專屬 workaround。

### P3：實機與 paper comparison

- 固定相同 terrain、起終點、初始 body height、command speed、control rate、friction／表面、成功標準與安全限制。
- Wheel、Walk、Hybrid 使用相同條件；至少記錄 success、time、tracking error、peak body roll／pitch、minimum support margin。
- energy／COT 只有在能同步取得可靠電流、電壓或 torque estimate 時才報；否則刪除節能主張。
- 先低速 flat／4 cm，每次只提高一個風險維度；確認 emergency stop 與機械限位後才做 10 cm。

## 9. 本週初稿的最小救火版本

如果期限確實是本週，建議將工作切成以下 stop-rule：

1. **先完成可交的 manuscript skeleton**：補 references、系統圖、方法、現有單腳 results、limitations；所有沒有數據的欄位保持 TODO，不填推測值。
2. **整機只追一條最小成功路徑**：levelled rolling + support repair + flat-ground validation。沒有先過 flat，就停止往 10／19 cm 擴張。
3. **實驗先取可重現 baseline**：若 Hybrid 尚未通過 controller gate，先取得 Wheel／Walk 的 matched-condition pilot data，並清楚標為 preliminary。
4. **截稿前的宣稱界線**：沒有 controller playback 就不稱 executable；沒有 Webots／實機就不稱 whole-robot validation；沒有量測就不稱節能。

## 10. 跟老師可以直接這樣報告

> 我目前已經把 Hybrid 從多 rim 接觸、單腳 roll／swing、策略選擇一路接到四腳 timing、body requirement 和 support validation。單腳 2-D 有完整越障與 parameter sweep，四腳架構也能對 flat、4、10、19 cm 用同一入口跑，但現在輸出的是結構化 infeasible，不是可下機的軌跡。主要不是單腳接不上，而是固定 theta 造成三腳高度要求衝突，加上 Walk phase 在 gamma=0 時支撐 margin 幾乎為零。最新 probe 已用每腳 theta compensation 把高度衝突清掉，現在最主要剩 support／gamma、timing 與 joint-limit validation。論文方法和單腳結果可以先成稿，但 whole-robot comparison 和實機仍未完成，所以我需要先跟老師確認本週初稿是否接受把它們明確列為 ongoing，並把本週資源集中在一條最小 feasible whole-body trajectory 與基本 baseline。

## 11. 最誠實的完成度判定

| 項目 | 狀態 |
|---|---|
| Contact representation | 已完成離線 prototype 與測試 |
| Single-leg rolling traversal | 已完成 sampled 2-D evidence |
| Cartesian swing | 已完成離線 path／IK／touchdown checks |
| ROLL／SWING selection | 已完成參數化 decision prototype |
| Four-leg architecture | 已接通並能輸出結構化失敗 |
| Feasible synchronized four-leg trajectory | 尚未完成；preliminary probe 已解 body conflict，support 仍失敗 |
| Controller-ready motor CSV | 尚未完成 |
| Webots whole-robot validation | 尚未完成 |
| Hardware experiment | 尚未開始 |
| Fair Wheel／Walk／Hybrid comparison | 尚未完成 |
| Submission-ready paper | 尚未完成；現有 working draft 可作本週 internal draft 基礎 |
