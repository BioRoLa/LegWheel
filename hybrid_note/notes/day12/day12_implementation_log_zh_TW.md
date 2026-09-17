# Day 12 實作紀錄與交接檔

> **Day 12 已於 2026-09-05 凍結。**
>
> * 資料索引與收尾 -> `day12_README_zh_TW.md`【先讀這份】
> * 凍結清單與交接 -> `day12_step12_freeze_and_handoff_zh_TW.md`
> * 之後的工作 -> `../day13/day13_plan_zh_TW.md`
>
> 本檔是按時間順序的實作紀錄，保留原樣不再追加。
> 2026-09-06 之後關於 B5（頂面策略）的量測，記在 freeze 文件的 B5 各節。

> **這份檔案的用途**：記錄 Day 12 每一次工作階段實際做了什麼、目前停在哪裡、下一步該做什麼。
> **如果你是在新的對話中讀到這份檔案**，請先讀 §0，它會告訴你需要先讀哪幾份文件、
> 目前的狀態、以及可以直接開始的下一個任務。
>
> 規格與 task list 在 `hybrid_gait_day12_whole_body_integration_FINAL_zh_TW.md`（以下簡稱「規格」）。
> 這份只記錄**動作與狀態**，不重複規格的論證。

---

# 0. 新對話快速上手

## 0.1 必讀順序

```text
1. 本檔 §0.4「一句話現況」與 §2「目前狀態」
2. hybrid_gait_day12_whole_body_integration_FINAL_zh_TW.md   Day 12 規格
   §0.1 generalization freeze（地形是參數，不是 branch）
   §0.2 FINAL FREEZE：NOMINAL_HYBRID_CYCLE = FOOT_RIM_ROLL + RECOVERY_SWING
   §6 Step 0-12 順序、§20 完成 checklist、§21 失敗分類、
   §24 給 AI 的共用 context（每個 Step 開頭都可以貼）
   §最後的 FINAL REVISION SUMMARY 有 12 條「這次改了什麼」
3. ../day10-11/day10_11_implementation_log_zh_TW.md
   §0.0 結論用語（【非常重要】不讀會把條件性結果寫成物理結論）
   §2 可以直接使用的東西、§4 已知陷阱 45 條
4. 需要細節時才回頭讀 day6-7 / day8-9 的筆記
5. 本檔 §4 是 Day 12 自己新增的陷阱
```

## 0.2 工作規則（使用者指定）

```text
可以引用 / import：  LegWheel/ 這包裡的東西（legwheel.* 與 hybrid_note.scripts.experiments.*）
只能參考、不可引用：  /home/chang/corgi_ws/icra hybrid/corgi_ros2_ws-dev/（以前機器人的資料）
                     可以看、可以參考做法，但【不可以 import，也不可以複製程式碼進來】
每個進度要展示在：    notes/hybrid_gait_day12_whole_body_dashboard.ipynb
                     （Day 12 專用的新 notebook，固定這一份，不要另開）
每個工作階段要：      更新本檔（day12_implementation_log_zh_TW.md）
                     目的：對話 memory 用完時，可以直接開新對話照著這份往下做
```

**驗證沒有引用舊機器人**（隨時可重跑）：

```bash
cd LegWheel
grep -rn "corgi_ros2_ws" hybrid_note/scripts/experiments/day12_*.py tests/test_day12_*.py
# 沒有輸出 = 沒有引用到舊機器人的東西
```

2026-09-01 實測：無輸出。

## 0.3 環境

```text
執行位置     /home/chang/corgi_ws/icra hybrid
import 路徑  sys.path.insert(0, os.path.abspath('LegWheel'))
模組         hybrid_note.scripts.experiments.<module>
跑測試       cd LegWheel && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_day12_*.py -q
             （不加那個環境變數，ROS 的 launch_testing plugin 會被 pytest 自動載入
               並因為缺 yaml 而整個崩掉 —— Day 10-11 陷阱 8）
```

## 0.4 一句話現況

```text
Step 0 - Step 11 完成並驗證。下一個是 Step 12
（Day 12 freeze 與 Day 13-14 handoff，規格 §19）。

另有「附錄 A — 全機動畫」（見本檔 2026-09-01 那一節）。
它【不是一個 Step】：不呼叫 planner、不產生運動，只把 Step 8 的軌跡畫出來。
唯一被發明的數字是一個【畫圖用】的 body 高度——因為 Step 5 拒絕給一個，
而那個選擇的代價（每一個瞬間都有一隻腳浮在地面上方）就畫在圖上。

另有「附錄 B — 越障段的世界座標註冊」。它從「把越障也畫出來」開始，
結論是【畫不出地形】，並且量出兩個新的未解結論：
越障段隱含的障礙物位置散佈 1045 mm（body-advance 需求 17.3x，Step 4 的距離版），
以及在頂面站立的總時間是 0.000 s（Step 7 未解的 TOP_REPOSITION 就是那一段）。

【要先知道】現在有四個**互相獨立**、都還沒解的結論：

```text
Step 4  時間放不下   0.6 s 的 swing 窗口要裝 1.2 s 的已排動作（2.000 x）
Step 5  高度對不起來 三隻站立腳要求的 body 高度差到 15.329 mm -> INFEASIBLE
Step 6  沒有餘裕     margin 最小 0.000 mm，五個 swing 全部 unstable
Step 7  撐不住       兩個 TOP_REPOSITION 都在 support gate 就停住
```

四個都不是 bug，都是規格要求「要問出來」的東西。不要合成一個「還沒好」。

【2026-09-01 規劃更新】規格換成 FINAL 版，兩個改動影響已完成的工作：
  §0.1 generalization freeze：地形必須是參數（height / top_length / x_start），
       4 cm / 10 cm / 19 cm 只是 evaluation query，不得寫進 decision rule。
  §0.2 FINAL FREEZE：平地 nominal 不再是「永遠連續接觸、不需要 swing」，
       而是一個循環 NOMINAL_HYBRID_CYCLE = FOOT_RIM_ROLL + RECOVERY_SWING。
       -> Step 0 因此要補做（見下），Step 1 整個以新定義實作。

Step 0（含 2026-09-01 的補做）
  FOOT_RIM_ROLL 與 WHEEL_ROLL 分開，且是【可機器檢查的 property】不是註解。
  補做：新增 RECOVERY_SWING；is_nominal_locomotion 從「只有 FOOT_RIM_ROLL」
        改成「FOOT_RIM_ROLL + RECOVERY_SWING」——舊版那個單一成員的寫法，
        等於把【已被取消的主張】（平地不需要 swing）寫死進程式。
  補做：新增 is_terrain_transition，因為規格 §18 要求 metrics 把
        nominal recovery swing 與 terrain-transition swing 分開統計。
  三個發現：(1) 運動已存在（Day 6-7 APPROACH）；
            (2) MotionSequence2D 串不起跨來源的 chain -> SegmentChain2D；
            (3) boundary 有兩種（CUT / HANDOVER），不能用同一組門檻。

Step 1
  兩個連續 cycle，完全相同（週期性成立）。核心量測：
      alpha  -40 -> +40 deg（整條可用 foot 弧）
      contact 前進 202.5 mm，hip 前進 297.1 mm / cycle
      stance 期間 hip_z 起伏 17.29 mm  <- nominal cycle 自己的 body requirement
      rolling 79.69 deg + airborne 280.31 deg = 360.00 deg（剛好一整圈）
      rotate 期間最小 clearance 57.2 mm
  回答了規格要求先確認的問題：Day 8-9 的 Cartesian swing generator
      【不能】表達 compact recovery（兩個結構性理由，見 §1）。
  兩個量錯又修正的地方（都在 §4 陷阱）：弧起點、clearance 門檻的適用範圍。

Step 2
  四腳 mounting 全部從既有 CorgiLegKinematics 轉換得到，沒有自訂任何機器人尺寸。
      sagittal 平面在【輪子中平面】：BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET = 211.675 mm，
      【不是】BODY_WIDTH/2 = 120 mm。垂直是 ABAD_AXIS_OFFSET = 57.166 mm。
  量到 2D 與 3D 對「腿伸多長」完全一致（219.4486 mm，差 1e-6 mm，
      而那個殘差正好是 2D scene 自己的 surface_offset_m = 1e-9 m）——
      這是把 Step 1 的 2D 軌跡掛上四腳 frame 的【前提】，所以量了而不是假設。
  地形用 Day 10-11 既有的參數化 SharedTerrainSpec2D，模組內無任何實驗尺寸。
  對稱檢查 14 項；y 比【鏡像】不比相等（比相等的話「兩腳同側」會通過）。
  跨過平台的站姿被【如實報成不合法】（gap = -40 mm、collision=True），沒有偷偷修好。

Step 3
  重用既有 GAIT_LIBRARY["Walk"]（duty 0.75、phase_offsets [0.75,0.25,0.5,0]）——
      驗證過四個 swing 窗口【剛好鋪滿一圈】，順序 LF->RH->RF->LH 與檔案註解一致。
  【最重要的量測】duty 是被強迫的不是選的：
      Step 1 的 cycle 若讓時間正比於轉角，duty 只有 0.221 -> 平均 3.1 隻腳在空中，
      「同時一隻腳 airborne」根本不可能。四腳要求 duty >= 0.75，代進去得到
      recovery 的 beta 角速度必須是 rolling 的 【10.553 倍】。
      Step 3 只記錄這個比值，【不判斷】它是否超出關節極限（那是 Step 9）。
  實跑四腳兩個 cycle：max airborne = 1、每次 swing 三腳支撐、零衝突。
  發現 ragged ends 不是 gait 錯誤（phase offset 就是時間位移，四腳 chain 不同時開始）
      -> 四腳約束只在 covered_interval_s 上評估，頭尾分開回報。

Step 4
  呼叫 compose_2d 不等於重做決策：decide_2d 是純查表、compose_2d 只是把已定的
      策略產生成動作。連示範用的 cell 都引用 Day 10-11 Step 9 自己的挑法。
  phase 從 SegmentKind 讀（它本來就叫 ROLL_UP / SWING_DOWN），不從幾何猜——
      猜會多出一個會和它打架的真相來源。
  【最重要的發現】Step 3 的 conflicts 是 0，但計畫【不可執行】：
      Step 3 的窗口規則是除法不是檢查，所以它永遠塞得下。
      用 Day 8-9 排好的 duration 當尺才量得到：兩段 0.6 s 的越障 swing
      被壓進一個 0.6 s 的 swing 窗口 = 【2.000 倍】，還沒算同窗口裡的 recovery。
      -> 新增 AirborneOverrun2D。Step 4 只報不修。

Step 5
  合併規則全部在 merge_demands() 一個函式裡：hard 壓過 lower bound、
      lower bound 取最大且不把 body 往下拉、都沒有就維持 nominal。
  【最重要的結果】平地四腳 nominal cycle 是 INFEASIBLE：
      foot-rim 滾動的 hip 高度是一段弧（兩端 202.161 / 中間 219.448 mm，
      起伏 17.287 mm），Walk 的相位把三隻站立腳放在弧的不同位置，
      三個 TRACK 要求三個不同的 body 高度，最多差 15.329 mm。
      241 個取樣裡 240 個 infeasible。規格 §12 要求 5 就是要問出這個。
  CoM 的說法：全程標成 quasi-static body-frame approximation，標籤掛在資料上。

Step 6
  support triangle 用【實際接觸點】而不是髖位置（規格特別點名），
      而且用「接觸點相對髖」的偏移，與 chain 的 x 原點無關。
      橫向座標是 Step 2 量到的 mounting offset —— gamma = 0，所以矢狀面就固定在那裡。
  【最重要的結果】margin 最小 0.000 mm，五個 swing 全部 unstable：
      LH / RH 起跳的瞬間，對角兩隻支撐腳在 (+239.2,-211.7) 與 (-239.2,+211.7) mm，
      對 body 中心完全對稱 -> 那條邊正好通過中心 -> margin 正好 0（對稱，不是捨入）。
      其他三個 swing 最好也只有 0.995 mm。整趟從來沒有超過 1 mm 餘裕。
  順便證明規格要求 5 是對的：只看 liftoff 會看到 20.897 mm 並判定通過，
      但它在 swing 期間線性衰減到 0.995 mm。

Step 7
  順序是【支撐先、動作後】：gate 沒過就完全不生成軌跡，也不發明 ABAD 補償。
  【不寫第二套 swing generator】（規格明令）：
      standing_scene_2d / left_rim_landing_scene_2d -> build_swing_request_2d
      -> generate_swing_2d（內建碰撞與 touchdown 驗證）-> segment_from_swing_plan_2d
  結果：planning floor（10 mm）下兩個 case 都是 support_insufficient（margin 0.995 mm）。
      放寬 floor 之後 #2 真的 resolved（落在 foot rim、theta 37.00 deg，滿足 >= 35 deg），
      #3 則是 JOINT_DISCONTINUITY —— LEFT_RIM_READY 需要一個離起始很遠的 beta，
      直接一個 swing 過不去。那是真發現：#3 還缺一段把 beta 轉過去的動作。
  新增 SegmentKind.TOP_REPOSITION_SWING（Step 0 以來第一次動既有檔案，純新增）。

Step 8
  把 Step 3-7 的結果對齊到同一組取樣上。【不呼叫任何 planner】——
      規格要求 7（不做 runtime replanning）在這裡等於「沒有東西可以 replan」。
  360 deg 的 joint jump 是【一整圈】不是斷點：beta 在這棵樹裡是圈數計數器，
      從來不 wrap。raw 與 wrapped 兩個讀數都報，wrapped 是 0.000 deg。
  rim geometry gap 這一趟是 0，【但那是因為全程在 foot rim 上】——
      1.2 mm 是 upper tyre 上的值。摘要有一欄專門講這件事。
  finite body_z samples = 0 of 241：Step 5 的結論在這裡變成「一個可用高度都沒有」。
  assumptions 欄位是【算出來的】：平地沒有越障，所以 Step 4 的 overrun 不在列。

Step 9
  11 個檢查 7 過 4 失，251 筆結構化失敗記錄。四個 FAIL 沒有一個是 Step 9 自己
      製造的——都指回 Step 5 / 6 / 8 記錄過的結論；七個 PASS 證明時間軸、
      關節極限、連續性是真的沒問題。
  beta_workspace_guard 失敗 240 筆：BETA_MAX_DEG = 40 是舊 planner 的「有界擺動」，
      Hybrid 把 beta 當【圈數計數器】。Step 9 回報違規但【拒絕判定哪種讀法才對】。
  【Step 3 欠的帳結清】實測 airborne/stance beta 速率比 = 10.553，
      與 Step 3 從 duty 推導的完全一致——兩條不同路徑同一個數字。
  但【判不了可不可行】：RobotParams 沒有關節速度極限。這寫進 UNEVALUABLE_CHECKS，
      與 DELEGATED_CHECKS 分開——「沒檢查」和「檢查過且通過」不是同一件事。
  peak theta 讀出 0 是【量測限制】：Step 8 在端點之間內插，而 recovery 頭尾
      theta 都是 60 deg，中間的縮腿不在取樣裡。每列都帶 theta_rate_is_lower_bound。

Step 10
  唯一入口 plan_terrain_2d(terrain, tables)；terrain=None 就是平地
      （平地是「沒有障礙」，不是「尺寸為 0」）。
  平地 / 4 cm / 10 cm 走同一條 pipeline，差別只有 terrain-transition swing 0 -> 8。
      兩個障礙的 primitive 都是 decide_2d 選的，Step 10 沒選任何東西。
  19 cm 在 decision 階段停住：「the rolling traversal was never swept at this
      height」——是【資料沒有】不是【物理不可能】，這個區別要保留。
  【gate 抓到我自己的違規】：為了拿 nominal body height 而蓋了一個 4 cm 平台。
      改成從腿的取樣幾何解：162.2826 mm，與 Step 2 一致。
  「max body lift = 0」是假的：body_z 幾乎全是 NaN，只剩 1 個有限值。
      改成有限取樣 < 2 就回 None，並多報 usable_body_samples。

Step 11
  §18 的兩條紅線都做成【機器可檢查】：
      energy_vocabulary() 掃程式碼有沒有能量詞彙（回空），且 guard 本身有測試證明會叫；
      CoM 指標永遠 None 並附完整理由，不填 0 也不填 body centre 的數字。
  第一版的 roll/swing 時間都等於整段時長（判斷式是「任何一隻腳」）——廢指標。
      改成 leg-seconds：8.926 + 2.975 = 11.901 = 4 x 2.975，有測試釘住。
  transition ROLL 距離 = 0 是【不存在】不是【沒量到】：#4 的 primitive 是 swing。
  bodyz p2p 回 None 不回 0：121 個取樣裡【一個】可用的 body 高度都沒有。

測試   Day 12  27+31+31+27+40+24+28+21+24+29+23 + 19（Step 11）= 324 passed
       Day 10-11 的 181 個【全部照舊通過】
       合計 505 passed
       注意：Step 7 的測試要跑約 6.5 分鐘（Day 8-9 的 swing planner 每次約 110 秒）
```

---

# 1. 工作階段紀錄

## 2026-09-01 — Step 0 實作（Freeze Day 12 Semantic Contract）

### 動作

**1. 先報告，再改程式**（規格 Step 0 指令的最後一句要求的）。
報告內容 = 本檔 §0.4 的 (1)–(3)，以及 notebook 的 0.1–0.4 節。

**2. 修改 `day10_11_motion_schema_2d.py`（3 處，都是加法）**

```text
+ WHEEL_MODE_THETA_RAD            取自 RobotParams.THETA0_DEG，不是自己寫 17
+ WHEEL_MODE_THETA_TOLERANCE_RAD  1 deg
+ SegmentKind.FOOT_RIM_ROLL       附完整 docstring（為什麼不是 APPROACH、不是 WHEEL_ROLL）
+ SegmentKind.pins_theta          只有 WHEEL_ROLL 為 True
+ SegmentKind.is_nominal_locomotion  只有 FOOT_RIM_ROLL 為 True
+ MotionSegment2D._check_day12_rolling_semantics()
+ MotionSegment2D.as_dict() 多兩欄：pins_theta / is_nominal_locomotion
~ SegmentKind.WHEEL_ROLL 的 docstring 補一句「不是 nominal」
```

**為什麼要在 `MotionSegment2D` 加驗證**：一個沒有被強制意義的 enum 成員會漂移。
Day 10-11 log §4 反覆踩到同一類錯（第 44、45 則的總結：
「一個為某個情況設計的量或標籤，套到不適用的情況上，就會生出一個沒有人主張過的結論」）。
所以兩個名字**各自只主張一件事**，而且那一件事被檢查：

```text
FOOT_RIM_ROLL  主張接觸在 foot rim。對 theta 【不做任何主張】——
               擺脫 wheel mode 正是它存在的理由，在這裡加 theta 檢查
               等於把它要移除的東西加回去。
WHEEL_ROLL     主張 theta 是 wheel mode（整段，容差 1 deg）。
               對哪一個 rim 【不做主張】——在 17 deg 那是幾何的事。
```

`FOOT_RIM_ROLL` 不准滾 left rim 的拒絕**不可以被讀成**「Day 12 不能滾 left rim」：
那是 `POST_TOUCHDOWN_ROLL` 的工作，測試裡有一條專門擋這個誤讀。

**3. 重構 `day10_11_sequence_builders_2d.py`（1 處，純抽取）**

```text
+ handoff_between_2d(before, after)   單一 boundary 的量測
~ handoff_report_2d()                 改成呼叫它
```

Day 12 需要對**跨來源**的 segment 做同樣的量測，而 `handoff_report_2d` 吃的是
`MotionSequence2D`（它不接受跨來源）。抽出來讓 Day 12 重用，**不複製**。

**4. 新增 `day12_segment_contract_2d.py`**

```text
entry_state_2d / exit_state_2d   可串接的共同端點格式，契約化 PointContact2D
BoundaryKind.CUT / HANDOVER      兩種 boundary
boundary_kind_2d()               同來源且 index 前進 = CUT，其餘 = HANDOVER
ChainTolerance2D                 每個預設值都有量測依據（見下）
ChainBreak2D                     一個沒過的檢查，帶 value / limit / boundary kind
chain_boundaries_2d()            量測 + 判定
boundary_rows_2d()               每個 boundary 一列
SegmentChain2D                   多來源 chain
  .sources / .entry_state / .exit_state
  .handoffs / .breaks / .is_chained / .is_complete
  .total_duration_s / .untimed_segments
  .rows()
SEGMENT_SEMANTICS                十個 kind 各自主張什麼，當作資料
segment_semantics_rows()
```

**5. 新增 `day12_step0_driver.py`** — 產出兩個 CSV 與一張圖。

**6. 新增 `tests/test_day12_segment_contract_2d.py`** — 25 個測試。

### 2026-09-01 稍後補做：規格換 FINAL 版之後 Step 0 少了什麼

規格 §7（Step 0 本體）**一字未改**，但 §0.2 的 FINAL FREEZE 改掉了 Step 0
**所凍結的東西**，所以要補：

```text
+ SegmentKind.RECOVERY_SWING          nominal cycle 的空中那一半
~ SegmentKind.is_swing                多收 RECOVERY_SWING
~ SegmentKind.is_nominal_locomotion   {FOOT_RIM_ROLL} -> {FOOT_RIM_ROLL, RECOVERY_SWING}
+ SegmentKind.is_terrain_transition   規格 §18 要 metrics 分開統計
+ RecoveryShaping2D                   第三個 shaping family（見下）
~ MotionSegment2D.__post_init__       RECOVERY_SWING 走自己的分支
~ MotionSegment2D.as_dict()           多一欄 is_terrain_transition
~ SEGMENT_SEMANTICS                   兩個新條目
~ tests/test_day10_11_motion_schema_2d.py::test_the_three_swing_kinds_are_kept_apart
      改成 test_the_three_terrain_transition_swing_kinds_are_kept_apart
```

**為什麼 `is_nominal_locomotion` 一定要改**：舊版只回傳 `FOOT_RIM_ROLL`，
那等於把「平地不需要 swing」——**FINAL FREEZE 已經取消的主張**——寫死在程式裡。
`test_the_semantics_table_covers_every_kind` 抓到了這件事（它斷言
nominal 只有一個），這正是那個測試存在的理由。

**為什麼要有 `RecoveryShaping2D` 而不是塞 `SwingShaping2D`**：
Day 8-9 的 `SwingShaping2D` 記的是 **Cartesian** swing 的旋鈕
（apex clearance / liftoff rise / touchdown drop / duration scale）。
recovery **一個都沒有**——它不是接觸點路徑，是關節空間的動作。
用零去填那些欄位，等於在真資料該去的位置放假資料，
而那正是這個 schema 有兩個 sampling family 的原因。
所以 recovery 記**自己的**旋鈕，`MotionSegment2D` 在 kind 是 `RECOVERY_SWING` 時
要求 `recovery_shaping` 而不是 `swing_shaping`，
且 sampling 用 `RollSampling2D`（它是以 theta / beta 步進產生的，不是以時間取樣）。

### 第一版寫錯的地方（這是 Step 0 最有價值的部分）

`ChainTolerance2D` 第一版對**每個 boundary** 用同一組門檻：joint 2 deg、hip 2 mm。
拿 Day 6-7 Step 10R 的真實 traversal 一跑，**5 個 boundary 沒過**。

第一反應可能是「調鬆 hip 門檻」。那是錯的。真正的問題是**門檻問對了問題、用錯了地方**：

`day10_11_sequence_builders_2d.sequence_from_traversal_frames_2d` 是在
`(stage, phase)` 改變的地方**切**一條已經連續驗證過的 run，所以相鄰兩段拿的是
**相鄰的 frame**——中間的「跳躍」是**一步真實的滾動**，不是不連續。實測：

```text
Step 10R 的 9 個 boundary，全部是 CUT，frame gap 都是 1（有一個是 2，因為
第 11 幀被 traversal 自己否決了 —— Day 10-11 陷阱 34）

                                                     dtheta   dbeta   dhip    dcontact
APPROACH -> RIGHT_RIM_FRONT_CONTACT                   0.00   -1.75    6.623      4.22
RIGHT_RIM_FRONT_CONTACT -> RIGHT_RIM_ROLL_UP          0.00   -1.00    3.547    180.47
RIGHT_RIM_ROLL_UP -> RIGHT_RIM_TOP                    0.00   -0.50    1.498      1.46
RIGHT_RIM_TOP -> RETRACT_TO_WHEEL                    -1.00   -1.00    1.255      1.46
RETRACT_TO_WHEEL -> WHEEL_MODE_TOP_ROLL              -1.00   -1.00    0.639      0.00
WHEEL_MODE_TOP_ROLL -> LEFT_RIM_READY                 0.00   -1.00    2.510      2.93
LEFT_RIM_READY -> LEFT_RIM_TRAILING_TRANSITION        0.00   -1.00    5.549      3.04
LEFT_RIM_TRAILING_TRANSITION -> LEFT_RIM_ROLL_DOWN    0.00   -1.00    2.510      0.00
LEFT_RIM_ROLL_DOWN -> LOWER_GROUND_CONTACT            0.00   -0.17    0.434    169.72
                                                    [deg]   [deg]    [mm]      [mm]

關節最大跳躍 theta 1.00 / beta 1.75 deg —— 與 Day 10-11 Step 7 報的數字【完全一致】
接觸點最大跳躍 180.5 mm —— 也與 Step 7 一致（交叉驗證通過）
```

而 Day 12 才出現的第二種 boundary 是：**兩個各自獨立產生的運動接在一起**。
那裡腿真的可能瞬移，而且**從來沒有人檢查過**。

用同一個門檻同時服務兩者，一定有一邊是錯的：

```text
鬆到容得下 CUT（hip 6.6 mm）  -> 會放過 HANDOVER 的瞬移
緊到擋得住 HANDOVER 的瞬移     -> 會否決 traversal 自己的切點
```

所以拆成兩種，各自檢查各自該檢查的事：

```text
CUT       問「這個切點有沒有漏掉運動」
          -> 關節跳躍 vs 該段【自己的 sampling step】x frame gap x 1.5 slack
          -> hip 【不檢查】。腳踩在面上時 hip_z 與 theta 互相決定
             （Day 10-11 陷阱 43），關節只動一步，hip 就只動一步的量。
             另外加一個 hip 門檻不會多出任何資訊，只會多出一個沒人量過的常數。

HANDOVER  問「腿有沒有瞬移」
          -> 關節 2 deg（Day 6-7 一步最大 1.75 deg；且擋得住 Step 2b 量到的
             alpha = -40 deg 接縫 29 deg 跳躍，也就是推翻策略 #3 的那一個）
          -> hip 10 mm（Day 6-7 一步最大 6.62 mm）
```

**同樣是 29 deg 的 beta 跳躍，在兩種 boundary 上是兩個不同的失敗**
（`beta_jump` vs `beta_step_overrun`），要去修的地方完全不同。
報成同一個名字會讓 Day 12 找錯方向。測試裡兩個都有。

**接觸點跳躍預設不判**：`max_contact_jump_m = None`。
Day 10-11 陷阱 32——跨 `±180 deg` rim 接縫時接觸點移動 180 mm 但關節只動一度，
那是**換座標卡不是運動**。會失敗於接觸點跳躍的檢查器，第一個否決的就是
Day 6-7 traversal 賴以成立的那個交接。永遠量、永遠報，但要判必須由呼叫端明說。

### 產出

```text
程式
  hybrid_note/scripts/experiments/day12_segment_contract_2d.py   新增
  hybrid_note/scripts/experiments/day12_step0_driver.py          新增
  hybrid_note/scripts/experiments/day10_11_motion_schema_2d.py   加法修改
  hybrid_note/scripts/experiments/day10_11_sequence_builders_2d.py  抽取重構
  tests/test_day12_segment_contract_2d.py                        新增，25 passed

資料
  notes/day12/day12_step0_segment_semantics.csv      10 個 kind x 5 欄
  notes/day12/day12_step0_boundary_evidence.csv      10 個 boundary
  notes/day12/day12_step0_boundary_evidence.png      三面板證據圖

展示
  notes/hybrid_gait_day12_whole_body_dashboard.ipynb   Step 0 節（0.1-0.8），已執行含輸出
```

### 驗收（規格 §7 的七項要求）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 判斷 `FOOT_RIM_ROLL` 能否表示而不破壞既有 segment | ✅ segment 層可以；**sequence 層不行**，這是本步最重要的發現 |
| 2 | 必要時新增 `FOOT_RIM_ROLL` | ✅ 加了，並附兩個可機器檢查的 property |
| 3 | `WHEEL_ROLL` 保持獨立（theta ≈ 17 deg） | ✅ 17 deg 取自 `RobotParams.THETA0_DEG` |
| 4 | 不重寫既有 rolling / swing 幾何 | ✅ 一行幾何都沒動 |
| 5 | 先不做四腳 timing | ✅ |
| 6 | segment start/end 用可串接的共同格式 | ✅ `entry_state_2d` / `exit_state_2d` |
| 7 | regression tests | ✅ 25 個 |

規格的驗收條件「可以明確回答 `FOOT_RIM_ROLL != WHEEL_ROLL`，且 schema 不再把
Hybrid flat propagation 綁死在 theta = 17 deg」：**成立**，而且是可以 `assert` 的。

---

## 2026-09-01 — Step 1 實作（Nominal Flat-Ground Hybrid Cycle）

### 先回答規格要求先回答的問題

規格 §8.5 最後一句：「實作前先確認 Day 8–9 的 swing generator 能不能表達
compact-recovery 的姿態變化。優先擴充通用介面，不要另寫只給平地用的 swing generator。」

**答案：不能。兩個理由都是結構性的，不是缺一個參數。**

```text
1. 在那裡 theta 是 IK 的【輸出】不是輸入。
   generate_swing_2d 先造 Cartesian 接觸點路徑，再逐 sample 解 IK；
   SwingConstraints2D 只有 theta_min / theta_max 【邊界】，
   沒有 theta waypoint、沒有 theta profile。
   「縮到 theta_compact 再伸到 theta_touchdown」沒有欄位可以放。
   （Day 10-11 陷阱 16 是同一件事的另一面。）

2. recovery 的定義性動作是 beta 轉將近一整圈（實測 280.31 deg）。
   Cartesian 路徑從 A 點到 B 點【決定不了圈數】，IK 會走近路。
   SwingRequest2D 連「轉向」都沒有欄位。
```

**Day 6-7 已經有正確的動作**：`run_airborne_retract_and_foot_reset_branch_2d`，
`branch="forward_continuation"` 就是這個轉向，`theta_target_rad` 預設就是 17 deg，
`_airborne_beta_target_2d` 算的就是「下一個等價的 foot-down beta」。語意完全對。

**但它被綁在障礙物上**：`terrain.obstacle is None` 直接丟例外；要求起始幀是完成的
right-rim 頂面滾動；落點過濾器 `_select_foot_top_candidate_2d` 只收 `obstacle_top`。

所以 Step 1 重用兩個 driver 共同的**基元**，**沒有新增任何幾何**：

```text
build_single_leg_rolling_scene_2d          （obstacle_x_start_m=None -> 純平地）
query_single_leg_rolling_scene_2d
_translated_scene_with_sample_on_target_2d
_lowest_contact_sample / _candidate_for_sample
_solve_flat_roll_rotation / _flat_roll_template   <- 滾動就是 Day 6-7 approach 那一步
_airborne_beta_target_2d                          <- 用來驗證 Day 12 的推廣正確
rim_alpha_limits_rad(RimId.FOOT)                  <- foot 弧的界線，不是自己寫的
```

### 動作

```text
+ hybrid_note/scripts/experiments/day12_nominal_cycle_2d.py
    NominalPosture2D      theta 是參數（預設 60 deg）、arc_samples、roll_step
      .hip_z_for_flat_stance(beta)   從幾何解，不是從 STAND_HEIGHT 常數
      .scene_kwargs                  obstacle_x_start_m=None
    RecoveryConfig2D      theta_compact（預設 RobotParams.THETA0_DEG）、步長、
                          hip_advance_m（預設 0）、min_clearance_m
    CycleFrame2D          一幀；airborne 時 rim/alpha/contact 為 None
    RollStroke2D / RecoverySwing2D / NominalCycle2D
    run_foot_rim_roll_2d(max_distance_m=None)
    run_recovery_swing_2d()   四相位 RETRACT / ROTATE / EXTEND / TOUCHDOWN
    run_nominal_cycles_2d(n)
    recovery_beta_target_2d(stroke) = stroke.start.beta - 2*pi
    cycle_segments_2d()   -> (FOOT_RIM_ROLL, RECOVERY_SWING)
+ hybrid_note/scripts/experiments/day12_step1_driver.py
+ tests/test_day12_nominal_cycle_2d.py    31 passed
```

### 量到的東西（theta = 60 deg，平地）

```text
alpha 可用範圍        -40 deg -> +40 deg   （整條 foot 弧，rim_alpha_limits_rad）
stroke 接觸前進        202.46 mm
cycle  hip 前進        297.06 mm
stance 期間 hip_z 起伏  17.29 mm   <- nominal cycle 自己的 body requirement
                                     （202.161 -> 219.45 -> 202.161 mm 的拱形）
rolling 轉角            79.69 deg
airborne 轉角          280.31 deg
cycle  轉角            360.00 deg  <- 剛好一整圈，不是巧合
rotate 期間最小 clearance  57.16 mm
theta_compact 實際到達   17 deg
兩個 cycle 完全相同（週期性成立，10^-9 等級）
```

**「一個 cycle 剛好是 leg-wheel 轉一整圈」不是觀察，是定義**：
要讓接觸回到弧上同一個位置，就必須轉滿一圈。
所以 `recovery_beta_target_2d` 是一個**減法**，不是一次搜尋。

### 兩個量錯又修正的地方

**(a) stroke 的起點，錯了兩次。**

```text
錯法一  往回滾找起點。
        _flat_roll_template 要求新 sample 【大於】舊 sample，只會往前走，
        往回呼叫第一步就回傳 None -> 起點被無聲報成 beta = 0（弧的【中點】），
        每個 stroke 少了一半（101 mm 而不是 202 mm）。

錯法二  找 foot 區域的盡頭（contact_regions 還是 foot_rim 的最大 beta）。
        過了 beta ~= 40 deg 之後支撐 sample 【不動了】，釘在 index 0，
        也就是 foot/left 接縫的角，腿繞著它轉 —— 那是 CORNER_PIVOT
        （Day 10-11 陷阱 31），而且還會再持續 20 deg 到 beta ~= 60 deg。
        從那裡起跑滾動距離【一點都沒有多】，卻多付 28.6 mm 的 hip 下沉。

正解    起點 = 接觸第一次抵達 foot rim 自己 alpha 下界的【最小】beta。
        下界從 rim_alpha_limits_rad(RimId.FOOT) 讀，不是寫在 Day 12 裡。
        過了它是 pivot，在它之前是 roll。
        證據表：day12_step1_arc_start_evidence.csv
```

**(b) clearance 門檻的適用範圍。**

第一版對**每一個** airborne frame 都要求 `min_clearance_m`，
然後在自己的第一個 cycle 就失敗（`EXTEND_HITS_TERRAIN_BEFORE_TOUCHDOWN`）。
原因：liftoff 與 touchdown 兩側的 frame，clearance **依定義為 0**——它們就是離地與觸地那一刻。

```text
RETRACT  不設 clearance 下限，只要求不穿透（>= -contact_tolerance）
ROTATE   clearance >= min_clearance_m   <- 腿橫掃過地面的那一段，門檻屬於這裡
EXTEND   不設下限，要求不穿透【且 clearance 單調下降】（是在接近地面，不是撞上去）
回報      min_clearance_m 只取 ROTATE；ramp 的另外用 ramp_min_clearance_m 分開報
```

**和 Step 0 踩到的是同一類錯**：為某種情況設計的量，套到不適用的情況上。

**(c) 一個原本以為需要、實際上不需要的東西。**

我在 docstring 裡寫過「recovery 期間 hip 必須抬 17.3 mm，否則伸腿會插進地面」。
**那是錯的，而且方向也錯。** foot 弧對 `alpha = 0` 對稱，所以弧的兩端 hip 高度
**完全相同**（202.161 mm 對 202.161 mm），整圈 cycle 的 hip ramp 剛好是 **0.0000 mm**。
但那段 ramp 不是廢碼：被 `max_distance_m` 截短的 stroke（接近 transition 的腿）
會停在拱形半路，得**降**回落地高度——實測 50 mm stroke 是 **−12.73 mm**。
兩個事實都有測試釘住。

### 驗收（規格 §8）

| 條件 | 結果 |
|---|---|
| flat ground 連續產生 `ROLL → RECOVERY → ROLL → RECOVERY` | ✅ 兩個 cycle 且完全相同 |
| contact-phase rolling direction is consistent | ✅ `beta` 全程單調下降 |
| `theta_compact` is configurable | ✅ `RecoveryConfig2D.theta_compact_rad` |
| next touchdown leg length is terrain/state dependent | ✅ 60 deg ≠ `theta_compact` 17 deg |
| no terrain collision during recovery | ✅ rotate 最小 57.2 mm |
| net body displacement > 0 | ✅ 297.1 mm / cycle |

§8.5 九項實作要求逐條成立。

### 產出

```text
程式  day12_nominal_cycle_2d.py / day12_step1_driver.py    新增
      tests/test_day12_nominal_cycle_2d.py                 新增，31 passed
      day10_11_motion_schema_2d.py                         加 RecoveryShaping2D（見 Step 0 補做）
資料  day12_step1_cycle_frames.csv       334 幀 x 2 cycle
      day12_step1_cycle_summary.csv      每個 cycle 一列
      day12_step1_arc_start_evidence.csv roll / pivot 邊界的量測
      day12_step1_phase_plot.png         五面板相位圖
      day12_step1_cycle_animation.gif    兩個 cycle
展示  notebook 的 Step 1 節（1.1-1.9），已執行含輸出
```

---

## 2026-09-01 — Step 2 實作（Four-Leg Initial State 與 Terrain Registration）

### 動作

```text
+ hybrid_note/scripts/experiments/day12_four_leg_state_2d.py
    LegId / LEG_ORDER          LF RF LH RH，【索引沿用專案的 0 1 2 3】
    LegMount2D / leg_mounts_2d()
    FlatRunExtent2D            flat_before / flat_after
    LegState2D / FourLegState2D / SymmetryCheck2D
    initialize_four_leg_state_2d()
    sagittal_reach_agreement_2d()
    four_leg_rows() / plot_four_leg_state_2d()
+ hybrid_note/scripts/experiments/day12_step2_driver.py
+ tests/test_day12_four_leg_state_2d.py    31 passed
```

**沒有修改任何既有檔案。** Step 2 完全是新增。

### mounting 從哪裡來

把 sagittal leg frame 的原點（`p_L = 0`）用專案自己的
`{Li} -> {Mi} -> {B}` 矩陣（`CorgiLegKinematics._get_transformation_matrices`）
推到 body frame，**不是**自己從 `WHEEL_BASE` / `BODY_WIDTH` 重推：

```text
LF (index 0)   (+0.255, +0.211675, +0.057166) m
RF (index 1)   (+0.255, -0.211675, +0.057166) m
RH (index 2)   (-0.255, -0.211675, +0.057166) m
LH (index 3)   (-0.255, +0.211675, +0.057166) m
```

**橫向那一項是最容易寫錯的地方**：sagittal 平面在**輪子中平面**，不在 ABAD 軸上，
所以是 `BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET = 120 + 91.675 = 211.675 mm`，
**不是** `BODY_WIDTH/2 = 120 mm`。有一條測試專門釘住這個差別。

垂直那一項是 `ABAD_AXIS_OFFSET = 57.166 mm`——腿平面吊在 body 原點**上方**，
所以 `body_z = hip_stance_z - 57.166 mm`。實測 body 在 162.283 mm、hip 在 219.449 mm。

### 掛載的前提：先量 2D 與 3D 有沒有在講同一隻腳

整條 Day 6-11 pipeline 是矢狀面 2D，它的「hip」被宣稱等於 3D 的 leg-plane 原點。
**那是掛載能不能成立的前提，所以量而不是假設**：

```text
CorgiLegKinematics.forward_kinematics(60 deg, 0, 0)
    腳在 leg-plane 原點【下方】      219.448615 mm
NominalPosture2D.hip_z_for_flat_stance(0)
    hip 在平地【上方】               219.448616 mm
    差                              -1.0e-6 mm
```

兩條完全不同的程式路徑，而那個殘差**正好就是** 2D scene 自己刻意加的
`surface_offset_m = 1e-9 m`，不是模型分歧。
`sagittal_reach_agreement_2d()` 把它做成可重跑的函式，測試在 theta = 90 deg 也驗一次。

### 地形

直接用 Day 10-11 既有的參數化矩形 `SharedTerrainSpec2D`
（`height_m` / `top_length_m` / `x_start_m` / `ground_height_m`），
**沒有新增地形型別**，模組裡也沒有出現任何實驗尺寸（規格 §0.1）。

`flat_before` / `flat_after` 另外用 `FlatRunExtent2D` 帶。
理由：`TerrainProfile2D` 的地面是**無界**的，
所以它們不是幾何邊界，而是「nominal cycle 打算在哪裡跑」這個**計畫輸入**。
放在同一個 spec 裡會被讀成地形邊緣。

**generalization 的證據**：同一份 code 跑五組平台參數
（1 / 40 / 100 / 190 mm 高，400 與 220 mm 長），`body_z` **五組完全相同**
（162.283 mm）——初始站姿在下層地面，本來就不應該隨平台高度改變。
如果哪一天它變了，就表示有東西偷偷 branch 在高度上。
`day12_step2_terrain_sweep.csv`。

### 對稱檢查：`y` 要比「相反」不是比「相等」

這是這一步最該抓到的錯。左腳與其鏡像面對相同的縱向地形，所以每個量都要相等——
**除了 `y`，它必須大小相等、正負相反**。
拿 `y` 去比相等的話，「兩隻腳都在同一側」的機器人會**通過**檢查。

`SymmetryCheck2D` 因此把比較的量明寫出來，`hip_y_m (mirrored)` 比的是**和為零**。
共 14 項（每個左右對 7 項 x 2 對）。有一條測試故意把 RF 的 hip 換成 LF 的，
確認檢查**會失敗**——不會失敗的檢查通過了也沒有意義。

### 一個不合法的站姿要被【報出來】，不是被偷偷修好

body 水平、站在下層地面高度時，如果某隻腳的 hip 已越過平台前緣，
那隻腳構不到平台面——實際上是**插進去**：

```text
body_x = 0.90 m，平台 40 mm 高 / x 從 1.00 m 起
  LF / RF   in_contact=False   collision=True   gap = -40.000 mm   <- 穿透，不是懸空
  LH / RH   in_contact=True    collision=False  gap =  +0.000 mm
  is_symmetric = True                                 <- 仍然左右對稱
```

Step 2 的正確輸出是「這不是合法的四腳站姿，這是原因」。
把 body 抬起來修它是 **Step 5** 的工作；在這裡偷偷放低一隻腳，
等於生出一個沒有人規劃過的站姿。

`is_symmetric` 與 `all_in_contact` 是**兩個分開的問題**，所以是兩個分開的旗標。

### 視覺化

兩個視角，因為一個抓不到 mapping 錯誤：
矢狀視圖給平台與站高，俯視圖給左右與前後——索引排列錯了會直接看到一台鏡像機器人。

（第一版的 x 範圍只取地形 extent，把後腳畫到圖外了；現在取地形與機器人的聯集。）

### 踩到的既有陷阱

Day 10-11 陷阱 21：`write_rows_csv` 用**第一列**的 keys 當表頭，
而 `four_leg_rows` 有 body / terrain / leg / symmetry 四種不同欄位的列，直接丟會 raise。
修在 `four_leg_rows` 裡（取欄位聯集補空值），不是在每個呼叫端。

### 驗收（規格 §9 的九項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | LF/RF/LH/RH hip/module offset | ✅ 由 `CorgiLegKinematics` 轉換，未自訂常數 |
| 2 | 專案既有 world-frame 慣例 | ✅ `+x` 前 `+y` 左 `+z` 上；輸出 `InitialRobotState` |
| 3 | gamma = 0 | ✅ `leg_mounts_2d(gamma!=0)` 直接拒絕 |
| 4 | body roll/pitch/yaw = 0 | ✅ 非水平 body 會被 `FourLegState2D` 擋下 |
| 5 | 參數化對稱平台 | ✅ `SharedTerrainSpec2D`，模組內無實驗尺寸 |
| 6 | 每腳 hip pose / contact point / surface id / θβγ | ✅ `LegState2D` |
| 7 | 不做 gait timing | ✅ |
| 8 | 一張圖 | ✅ 兩個視角 |
| 9 | 左右對稱 sanity check | ✅ 14 項 |

### 產出

```text
程式  day12_four_leg_state_2d.py / day12_step2_driver.py   新增
      tests/test_day12_four_leg_state_2d.py                新增，31 passed
資料  day12_step2_leg_mounts.csv        四個 mounting offset
      day12_step2_four_leg_state.csv    body + platform + 四腳 + 對稱檢查
      day12_step2_terrain_sweep.csv     同一份 code、五組地形參數
      day12_step2_four_leg_state.png    矢狀 + 俯視
展示  notebook 的 Step 2 節（2.1-2.7），已執行含輸出
```

---

## 2026-09-01 — Step 3 實作（Four-Leg Timing Skeleton）【已完成】

### 先報告資料結構與對既有 duty/swing_phase 的對應（規格 §10 要求先做這件事）

**既有的 walk gait 定義【本身就是】「同時最多一隻腳 airborne」的結構**，
所以不需要自己發明相位：

```text
legwheel/planners/gait_generator_3d.py 的 GAIT_DEFINITIONS["walk"]
    phase_offsets = [0.75, 0.25, 0.5, 0.0]   索引 [FL, FR, RR, RL] = [0, 1, 2, 3]
    stance_duty   = 0.75
    註解寫的 swing order: FL -> RR -> FR -> RL
```

相位語意（同檔 §49-51）：`phase_offset` 是「從 traj 曲線的哪個比例開始」，
而曲線本身是 `[stance duty | swing duty]`。所以腿 i 在時間 t 的相位是
`(t/T + phi_i) mod 1`，`phase < D` 是 stance、`>= D` 是 swing。代進去：

```text
FL  swing 於 t/T in [0.00, 0.25)
RR  swing 於 t/T in [0.25, 0.50)
FR  swing 於 t/T in [0.50, 0.75)
RL  swing 於 t/T in [0.75, 1.00)
```

四個互不重疊、剛好鋪滿一圈 —— **同時恰好一隻腳 airborne**，
而且與註解宣稱的 swing order 一致。這是驗證過的重用，不是照抄。

### Step 3 最重要的量測：duty 是被【強迫】的，不是選的

Step 1 的 nominal cycle：rolling 轉 79.69 deg、airborne 轉 280.31 deg。
如果時間正比於轉角，stance 佔比只有 `79.69/360 = 0.221`，
四隻腳平均會有 `4 x 0.779 = 3.1` 隻同時在空中 —— **一隻腳 airborne 的約束根本不可能成立**。

但時間**不需要**正比於轉角：recovery 是空中動作、沒有接觸約束，可以走得快；
rolling stroke 是準靜態滾動，可以走得慢。所以 duty 是一個**建模自由度**。

四腳、同時最多一隻 airborne 的條件是 `airborne 佔比 <= 1/4`，即 `duty >= 0.75`。
取既有 walk 的 `D = 0.75`，代進 Step 1 的轉角：

```text
rolling  角速度  79.69 deg / (0.75 T)  = 106.25 deg / T
recovery 角速度 280.31 deg / (0.25 T)  = 1121.24 deg / T
比值                                    = 10.55 x
```

> **所以「同時最多一隻腳 airborne」這個約束，等價於要求 recovery 的 beta
> 角速度是 rolling stroke 的 10.55 倍。**

這是 Step 3 的第一個實質結果。Step 3 **只記錄**這個比值，
**不判斷**它是否超出關節速度極限 —— 那是 Step 9 validation 的事。

### 時間是 Step 3 自己指定的（要一直記得說）

Day 6-7 準靜態、Step 1 以 beta 前進，`duration_s` 全是 `None`
（Day 10-11 陷阱 33）。Step 3 給的每一個 duration 都是**它自己的建模決定**，
不是從既有資料讀出來的。

### 資料結構

```text
GaitTiming2D          cycle_period_s / stance_duty / phase_offsets
                      預設值全部從 GaitGenerator3D 的 walk 定義讀，不自訂
LegPhaseWindow2D      一個 stance 或 swing 窗口：leg / start_s / end_s / mode
LegInterval2D         一個 segment 佔的時間區間：
                      leg / start_s / end_s / mode / segment_index /
                      segment_kind / phase_label
FourLegSchedule2D     .timing / .intervals（每腳一串）
                      .airborne_legs_at(t) / .support_legs_at(t) / .swing_leg_at(t)
                      .max_airborne_count / .conflicts
                      -> 【只做 timing】，不解 support polygon（那是 Step 6）
```

**輸入是 per-leg 的 `SegmentChain2D`（Step 0 的容器），不是 one-obstacle 物件**
（規格 §10 要求 5）。

### segment -> 時間窗口的對應規則

一條 chain 先切成**接觸段落**與**空中段落**的極大連續 run
（`SegmentKind.is_swing` 決定），然後：

```text
接觸 run  -> 一個 stance 窗口   長度 D * T
空中 run  -> 一個 swing  窗口   長度 (1-D) * T
run 內部的多個 segment -> 依【frame 數】按比例瓜分該窗口
```

按 frame 數瓜分是一個**建模選擇**（frame 是唯一可用的外延量），要明說。
這條規則不看地形，所以 terrain-transition 的 chain
（APPROACH / ROLL_UP / WHEEL_TRANSITION / ROLL_DOWN ...）也能吃。

### 衝突要【報】不要修

一隻腳在做 terrain transition 時，它的空中 run 可能比一個 swing 窗口長，
於是會和別隻腳的 swing 重疊。Step 3 必須**偵測並回報**，不是自動調整。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_timing_skeleton_2d.py
    LegMode.STANCE / AIRBORNE      LegMode.of(kind) 由 SegmentKind.is_swing 決定
    GaitTiming2D                   .phase_at / .mode_at / .swing_window
                                   .stance_duration_s / .swing_duration_s
                                   .max_simultaneous_airborne
    walk_timing_2d(cycle_period_s=2.4)     從 GAIT_LIBRARY["Walk"] 讀
    rotation_rate_demand_2d()      duty 對 recovery 角速度的要求
    ScheduledSegment2D             .duration_is_assigned 永遠 True
    TimingConflict2D
    FourLegSchedule2D              .airborne_legs_at / .support_legs_at
                                   .swing_leg_at（多腳在空中時【丟例外】）
                                   .covered_interval_s / .ragged_intervals_s
                                   .max_airborne_count / .conflicts
                                   .one_leg_airborne_at_a_time
                                   .every_swing_has_three_supports
    schedule_chains_2d(chains, timing)
    schedule_rows() / plot_timeline_2d()
+ hybrid_note/scripts/experiments/day12_step3_driver.py
+ tests/test_day12_timing_skeleton_2d.py    27 passed
```

**沒有修改任何既有檔案。** Step 3 全部是新增。

實跑（四腳都用 Step 1 的兩個 nominal cycle）：

```text
16 個 segment，span [-1.800, 4.800] s
covered interval          [0.000, 3.000] s
max airborne legs         1
one airborne at a time    True
every swing has 3 supports True
conflicts                 0
swing 順序（時間軸上）      LF -> RH -> RF -> LH   （與 gait 檔宣稱的一致）
```

### 做的時候發現的一件事：ragged ends 不是 gait 錯誤

phase offset **就是時間上的位移**，所以四隻腳的 chain 不是同時開始的：
LF 從 −1.8 s 開始、LH 從 0.0 s 開始。於是整段 span 的頭尾有一段
**不是四隻腳都有 segment**。

第一版在整段 span 上檢查，`every_swing_has_three_supports` 報 `False`——
但那不是步態錯誤，是在「還沒進計畫的腿」上數支撐腳。
**「沒被排程」不等於「在空中」。**

修法：加 `covered_interval_s`（四腳都有 segment 的區間），
四腳約束（`max_airborne_count` / `conflicts` / `every_swing_has_three_supports`）
**只在那個區間上評估**；頭尾用 `ragged_intervals_s` 分開回報，不隱藏。
圖上兩個 panel 都把 ragged 區段打灰底。

### 驗收（規格 §10 的八項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | reuse 既有 duty / swing_phase / leg-order | ✅ 直接讀 `GAIT_LIBRARY["Walk"]`，並驗證四個窗口鋪滿一圈 |
| 2 | planner offline，不進 runtime | ✅ 純資料結構，無 runtime 呼叫 |
| 3 | 同時最多一隻腳 airborne | ✅ 實跑 `max_airborne_count = 1` |
| 4 | 每腳每個區間都有明確 state | ✅ covered interval 上取樣驗過 |
| 5 | 支援任意 MotionSegment2D 序列 | ✅ 輸入是 `SegmentChain2D`；terrain-transition chain 有測試 |
| 6 | 暴露 swing leg / 三支撐腳 / segment index+kind / 起訖時間 | ✅ |
| 7 | 先不解 support polygon | ✅ 只回傳腿，沒有多邊形或 margin（有測試擋住） |
| 8 | 四腳 timeline 圖 | ✅ ＋ 下方 airborne 計數 panel |

### 產出

```text
程式  day12_timing_skeleton_2d.py / day12_step3_driver.py   新增
      tests/test_day12_timing_skeleton_2d.py                新增，27 passed
資料  day12_step3_schedule.csv      timing + 16 segment + conflicts + summary
      day12_step3_rate_demand.csv   三個週期下的 rate demand（比值不變）
      day12_step3_timeline.png      四腳 timeline + airborne 計數
展示  notebook 的 Step 3 節
```

---

## 2026-09-01 — Step 4 實作（Per-Leg Sequence → 共同時間軸）【已完成】

### 規格 §11 的八項要求（原文摘要）

```text
1. Day 10-11 的 ROLL/SWING 決策【已解】，Step 4 不得重算
2. ascent 與 descent 保持【各自獨立】的 transition decision
3. terrain-transition 區段【之外】要插入 nominal FOOT_RIM_ROLL
4. 保留每段的 sampling 參數與 body requirement
5. 未解的 TransitionRequirement2D 要【明擺著】，不得偷偷編一條軌跡
6. 每一段的 exit state 必須是下一段合法的 entry state
7. timing / contact 衝突要【偵測並回報】，不是藏起來
8. debug 表：time interval | leg | segment kind | contact/airborne | body requirement
```

### 為什麼「不重算決策」不等於「不能跑 composer」

`decide_2d()` 是**純表格查詢**（讀 Day 10-11 各步驟凍結下來的 CSV），
`compose_2d()` 只是把已經決定好的策略**產生成動作**。
所以 Step 4 呼叫 `compose_2d` **沒有**重做決策——重做決策會長成
自己寫一個 if/else 去挑 ROLL 或 SWING，那才是規格禁止的事。

代價：`ROLL_ROLL` 的 traversal 要跑約 3 分鐘（Day 10-11 陷阱裡記過）。
driver 因此要能背景跑。

### ascent / descent 不可以被壓成一個決定

規格特別點名不要寫成 `this obstacle = ROLL`。
所以每一段要掛一個 `TransitionPhase`（`ASCENT` / `ON_TOP` / `DESCENT`），
而策略是 **per-phase** 記錄的（`StrategyId` 的組合名稱如 `SWING_SWING`
其實就是 `ascent=SWING, descent=SWING`），
`LegPlan2D` 要能分別回答「上去用什麼」「下來用什麼」。

### 資料結構（實作前先講清楚，規格的慣例）

```text
TransitionPhase        NOMINAL_BEFORE / ASCENT / ON_TOP / DESCENT / NOMINAL_AFTER
PhasedSegment2D        MotionSegment2D + phase + source_label + body requirement 摘要
                       -> 不複製 MotionSegment2D 的內容，只掛標籤（要求 4）
LegPlan2D              一隻腳的完整計畫
                       .chain          SegmentChain2D（多 source：nominal + day10_11）
                       .ascent_strategy / .descent_strategy   分開存（要求 2）
                       .unresolved     從 ComposedSequence2D 原封帶過來（要求 5）
                       .is_executable  有 unresolved 就是 False
                       .breaks         chain_boundaries_2d 的結果（要求 6）
build_leg_plan_2d()    nominal(before) + transition + nominal(after) 組成 chain
FourLegPlan2D          四隻腳 + Step 3 的 FourLegSchedule2D + conflicts（要求 7）
debug_rows()           要求 8 的那張表
```

### 預期會發生、而且【不可以修掉】的事

平地 nominal cycle 的空中段剛好塞得進一個 swing 窗口，Step 3 實跑 conflicts = 0。
terrain transition 的空中段**比一個 swing 窗口長**，所以 Step 4 應該要報出來。

### 這裡量錯又修正的一次（Step 4 最有價值的部分）

第一版預期 Step 3 的 `conflicts` 會自己變成非空。**實跑是 0。**

原因是 Step 3 的規則「一個極大空中段配一個 swing 窗口，段內按 frame 數分」
**永遠塞得下**——它不是在檢查合不合適，它是在做除法。
越障把 `RECOVERY_SWING + SWING_UP + SWING_DOWN` 放進同一個 run，
規則就把三段一起壓進那 0.6 s，`conflicts` 依然是 0。
**那正是規格要求 7 在防的「藏起來」，只是藏在 Step 3 的除法裡。**

第二版改用 beta 掃掠角當尺——也沒抓到，
因為 recovery 自己就掃 280 deg，兩段越障 swing 的 beta 相比之下是小數。

第三版才對：**用已經排好的時間當尺**。
Day 8-9 幫每一段越障 swing 排過 `duration_s = 0.6 s`，Day 10-11 原封帶過來
（`day10_11_step9_body_requirements.csv` 裡看得到），
那是**整條鏈上唯一真實的時間**。兩段 0.6 s 塞進一個 0.6 s 窗口 = **正好 2.000 倍**，
而且還沒算同一個窗口裡那段沒被指定時間的 recovery。

-> 新增 `AirborneOverrun2D` / `airborne_overruns_2d()`，
   讓 `FourLegPlan2D.is_executable` 同時看 `conflicts` 與 overrun。
   **Step 3 的 conflicts 為 0，計畫依然不可執行**——兩個旗標分開存在，
   正是因為其中一個為零不代表另一個沒事。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_transition_mapping_2d.py
    TransitionPhase              NOMINAL_BEFORE / ASCENT / ON_TOP / DESCENT
                                 / OVER / NOMINAL_AFTER
    TRANSITION_PHASE_OF_KIND     phase 從 SegmentKind 讀，不從幾何猜
    STRATEGY_HALVES              每個 StrategyId 的 ascent / descent 兩半
    PhasedSegment2D              MotionSegment2D + phase + source（不複製內容）
    LegPlan2D                    .phased / .chain / .breaks / .unresolved
                                 .ascent_strategy / .descent_strategy
                                 .phases_are_separable / .is_executable
    build_leg_plan_2d()          nominal + 越障 + nominal
    AirborneOverrun2D            窗口壓縮量，用 planned duration 量
    airborne_overruns_2d()
    FourLegPlan2D / plan_four_legs_2d()
    debug_rows()                 規格要求 8 的表
+ hybrid_note/scripts/experiments/day12_step4_driver.py
+ tests/test_day12_transition_mapping_2d.py    40 passed
```

**沒有修改任何既有檔案。** Step 4 全部是新增。

實跑三格（都是 Day 10-11 Step 9 自己組過的格子，Step 4 沒有挑策略）：

```text
#5 SWING_OVER   h=60  L=75   未組成（該高度沒有掃過）
                separable = False   <- 這是誠實回答，不是缺陷
#4 SWING_SWING  h=80  L=240  組成
                ASCENT 1 段 / DESCENT 1 段，前後各一個 nominal cycle
                timing conflicts   0
                airborne overruns  4（每腳一個）
                  RECOVERY_SWING,SWING_UP,SWING_DOWN 共用一個 0.600 s 窗口
                  1.200 s 的已排時間 + 1 段未排時間  ->  2.000 x
                whole plan executable  False
#2 ROLL_SWING   h=160 L=350  blocked pair
                unresolved transitions 1，洞沒有被前後的 nominal run 蓋掉
```

### 驗收（規格 §11 的八項）

| #   | 要求                             | 結果                                                                           |
| --- | ------------------------------ | ---------------------------------------------------------------------------- |
| 1   | 不重算 ROLL/SWING 決策              | ✅ 只呼叫 `decide_2d`（純查表）＋`compose_2d`；連 cell 都引用 Step 9 的 `_first_winning_top` |
| 2   | ascent / descent 保持獨立          | ✅ 每段掛 `TransitionPhase`；`#5` 回報 `separable = False`                          |
| 3   | 越障區段外插入 nominal                | ✅ 前後各一個完整 cycle（roll + recovery）                                             |
| 4   | 保留 sampling 與 body requirement | ✅ 段是**同一個物件**（用 `is` 驗過），沒有複製                                                |
| 5   | 未解的 requirement 明擺著            | ✅ `#2` 帶著 `unresolved`，`chain.is_complete = False`                           |
| 6   | exit state 要是合法 entry state    | ✅ 用 Step 0 的 `chain_boundaries_2d`，斷點回報不修                                    |
| 7   | 衝突偵測並回報                        | ✅ Step 3 的 `conflicts` ＋ 新增的 `AirborneOverrun2D`                             |
| 8   | debug 表                        | ✅ `debug_rows()`                                                             |

### 產出

```text
程式  day12_transition_mapping_2d.py / day12_step4_driver.py   新增
      tests/test_day12_transition_mapping_2d.py                新增，40 passed
資料  day12_step4_leg_plans.csv     每格一列（含 overrun 計數）
      day12_step4_debug_table.csv   規格要求 8 的表
      day12_step4_timeline.png      四腳 timeline（含越障）
展示  notebook 的 Step 4 節
```


## 2026-09-01 — Step 5 實作（Body Requirement → Whole-Body Trajectory）【已完成】

### 規格 §12 的九項要求（原文摘要）

```text
1. 吃 Day 10-11 產生、Step 4 同步過的 body requirement timeline
2. HARD（TRACK / PINNED）必須滿足
3. LOWER_BOUND 取【滿足所有 active 腳的最小 body 運動】
4. 【不要】加權重式 whole-body optimizer
5. 同時出現互斥的 hard requirement -> 回報 infeasible，不要偷偷平均
6. body_x / body_z 要連續；對稱測試裡 body_y / roll / pitch / yaw 維持 nominal
7. 輸出與四隻腳同步的 body trajectory 取樣
8. 記錄 body / CoM 的垂直變化量，供後續 evaluation
9. 測試要涵蓋：無讓步 / 一個 lower bound / 多個相容 lower bound / 互斥 hard
```

### 合併規則（deterministic，沒有 optimizer）

```text
每個時間取樣 t：
  for each leg:
      找出它在 t 的 scheduled segment
      TRACK       -> 由 hip_z_profile_m 內插出【硬性】的 hip_z
      PINNED      -> 端點的 hip_z 是硬性的
      LOWER_BOUND -> hip_z_min_m 是【下界】
      airborne 且無 requirement -> 不出聲

  hip_z -> body_z 用 Step 2 定好的關係： body_z = hip_z - ABAD_AXIS_OFFSET

  hard 有兩個以上而且彼此不合 -> INFEASIBLE（記下是哪兩隻腳、差多少）
  hard 只有一個（或彼此相容）   -> body_z = 那個值
  沒有 hard                    -> body_z = max(active lower bounds, nominal)
```

**provenance 是驗收條件**：規格要求「可以說明每一次 body_z 變化是被哪隻腳 /
哪個 transition 要求的」，所以每個取樣都要記下 `driver_leg` 與 `driver_segment`。

### body_x 為什麼要用【增量】而不是絕對值

Step 4 給四隻腳的是**同一條 chain**（平地上動作一樣，只差相位），
每條 chain 的 `hip_x` 都從 0 起算，所以四隻腳的**絕對 x 互相對不上**——
那是 Step 4 沒有做腳的 world-x 佈置留下的缺口，不是 Step 5 該偷偷平均掉的東西。

Step 5 因此用**站立腳的 hip_x 增量**積分出 `body_x`，起點取 Step 2 的 body 位置。
增量是良定義的（與絕對偏移無關）。若站立腳之間的**增量**也對不上，那是真衝突，要報。

### CoM 的說法要誠實（規格特別點名）

這個 2D pipeline 沒有 whole-robot CoM 模型，只有 body frame。
所以輸出一律標成 `quasi-static body-frame approximation`，
**不得**寫成 whole-robot CoM。標籤掛在**資料上**（`BODY_BASIS` 進每一列），
這樣它不會在往 paper 的路上被弄丟。

---

## 【Step 5 最重要的結果】平地四腳 nominal cycle 是 INFEASIBLE

這不是 merge 寫壞，是 merge 照規格做事之後**問出來的真相**。

```text
foot-rim 滾動的 hip 高度是一段【弧】（Step 1 量的）：
    兩端 202.161 mm，中間（alpha = 0）219.448 mm  ->  一個 stroke 起伏 17.287 mm

Walk 的 phase offset 把三隻站立腳放在這條弧的【三個不同位置】，
所以在同一個瞬間，三隻腳各自要求一個不同的 body 高度。
而它們全都是 TRACK —— TRACK 是 hard。

實跑：241 個取樣裡 240 個 infeasible，
      最嚴重的兩隻腳差 15.329 mm（上界就是那 17.287 mm，測試釘住了）
```

規格 §12 要求 5 白紙黑字寫「conflict -> return infeasible，不要偷偷平均」。
所以 Step 5 **不會**給你一條平滑的 body_z 曲線——那條曲線要存在，
就得先承認下面三條路之一：

```text
(a) body 允許 heave / pitch      -> 但規格 §12 要求 6 在對稱測試裡把 rpy 釘在 0
(b) 每隻腳用 theta 補償弧的起伏   -> 那是 joint trajectory（Step 8）的事
(c) 換一個 hip 高度不隨滾動起伏的 nominal cycle
```

**這三條都不是 Step 5 可以自己決定的**，所以 Step 5 的正確輸出就是
「不可行，而且這是誰跟誰差多少」。

> 注意這條和 Step 4 的 airborne overrun 是**兩件獨立的事**：
> 一個是時間放不下，一個是高度對不起來。兩個都在，而且互不掩蓋。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_body_trajectory_2d.py
    BODY_BASIS            "quasi-static body-frame approximation
                           (no whole-robot CoM model)" —— 進每一列
    HIP_TO_BODY_Z_M       = RobotParams.ABAD_AXIS_OFFSET（Step 2 的關係，不是新數字）
    HARD_AGREEMENT_M      1 mm：比 Day 6-7 一個滾動步的 6.62 mm 小得多
    BodyDriver            HARD / LOWER_BOUND / NOMINAL / INFEASIBLE
    LegDemand2D           一隻腳在一個瞬間要求什麼，以及【是哪一段要求的】
    leg_demand_at()       TRACK 依窗口位置內插 profile
                          PINNED 只在【端點】出聲（Day 10-11 Step 9 C 節）
                          高度取 hip_z_min_m —— PINNED 的那個欄位是值不是下界
    merge_demands()       整條合併規則就這一個函式（好測、好讀）
    BodyConflict2D / BodySample2D / BodyTrajectory2D
                          .body_z_travel_m / .max_body_z_step_m（都跳過 NaN）
                          .concession_intervals()  <- 規格的驗收條件
    body_rows() / plot_body_trajectory_2d()
+ hybrid_note/scripts/experiments/day12_step5_driver.py
+ tests/test_day12_body_trajectory_2d.py    24 passed（約 19 秒）
```

**沒有修改任何既有檔案。** Step 5 全部是新增。

### 兩個實作上的判斷

1. **PINNED 的高度在 `hip_z_min_m`，不在端點 contact 裡。**
   第一版去讀 `start_contact.hip_xz_m[1]`，schema 直接擋下來
   （`a pinned requirement is a height; it needs hip_z_min_m`）。

2. **`body_x` 用站立腳的【增量】積分，不用絕對 `hip_x`。**
   Step 4 給四隻腳同一條 chain，每條的 `hip_x` 都從 0 起算，
   所以絕對值互相對不上——那是 Step 4 沒做 world-x 佈置留下的缺口。
   增量與絕對偏移無關，所以是良定義的；四隻站立腳的增量若也對不上才是真衝突。

### 驗收（規格 §12 的九項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 吃 Step 4 同步過的 requirement timeline | ✅ 直接讀 `FourLegPlan2D` |
| 2 | HARD（TRACK/PINNED）必須滿足 | ✅ hard 一律壓過 lower bound |
| 3 | LOWER_BOUND 取滿足所有 active 腳的最小 | ✅ 取最大下界，且**不會**把 body 往下拉 |
| 4 | 不加 optimizer | ✅ `merge_demands()` 是純規則 |
| 5 | 互斥 hard -> infeasible，不平均 | ✅ 回 NaN ＋ `BodyConflict2D`；有測試擋住中點 |
| 6 | body_x/z 連續、y 與 rpy 維持 nominal | ✅ body_x 單調連續；y/rpy 掛在資料上可驗 |
| 7 | 與四腳同步的取樣 | ✅ 在 `covered_interval_s` 上取樣 |
| 8 | 記錄垂直變化 | ✅ `body_z_travel_m`（跳過 NaN，不被 infeasible 抹掉） |
| 9 | 四個測試情境 | ✅ 無讓步 / 一個下界 / 多個相容下界 / 互斥 hard，全部直接測 `merge_demands` |

### 產出

```text
程式  day12_body_trajectory_2d.py / day12_step5_driver.py   新增
      tests/test_day12_body_trajectory_2d.py                新增，24 passed
資料  day12_step5_body_trajectory.csv   取樣 + concession + conflict + summary
      day12_step5_body_trajectory.png   body_z(t)（依 driver 上色）+ body_x(t)
展示  notebook 的 Step 5 節
```


## 2026-09-01 — Step 6 實作（三腳 Support Triangle + Stability Margin）【已完成】

### 規格 §13 的十項要求（原文摘要）

```text
1. 用另外三隻腳的【實際 world contact point】
2. 在水平面上建 support triangle / convex hull
3. 把 CoM 投影到水平面
4. signed margin：正 = 內部、零 = 邊界、負 = 外部
5. 【整個 swing 區間】都要算，不是只看 liftoff 那一幀
6. 每個 swing 段、以及整趟，都要存最小 margin
7. margin < 要求下限 -> 標記該段 / 整個 schedule infeasible
8. gamma 在 Day 12 維持 0，不做 ABAD 補償
9. 視覺化：support triangle / CoM 投影 / swing leg / margin
10. 單元測試涵蓋明確的 inside / boundary / outside
```

### 第一個問題：2D pipeline 沒有 y，contact point 從哪來

矢狀面 2D 只給 `(x, z)`。水平面需要 `(x, y)`。

`gamma = 0`，所以每隻腳的矢狀面**就固定在它自己的 y**——
那個 y 就是 Step 2 量到的 mounting offset（±211.675 mm），
不是新數字，也不需要新模型。

```text
contact_world_xy(leg, t) = ( body_x(t) + mount_x + (contact_x - hip_x)_leg-plane ,
                             mount_y )
```

**用 `contact_x - hip_x`（腳相對髖的偏移）而不是絕對 `contact_x`**，
理由和 Step 5 的 `body_x` 一樣：Step 4 給四隻腳同一條 chain，
每條的絕對 x 都從 0 起算、互相對不上，但**相對偏移是真幾何**。

規格特別強調「用 contact point 不要用 hip position」——
上面這條式子的第三項就是兩者的差，所以它確實是 contact，不是 hip。

### 第二個問題：CoM 是什麼

沒有 whole-robot 質量模型。**沿用 Step 5 的 `BODY_BASIS` 標籤**，
CoM 投影 = body 中心 `(body_x(t), 0)`，而且每一列都標明它是近似。
規格第 10 點就是在講這件事：要把近似**隔離並標記**，不要當成精確 CoM。

### 第三個問題：body 高度不可行，還能算穩定性嗎

可以，但**必須講清楚**。margin 是**水平**問題，
Step 5 不可行的是 `body_z`（垂直）。`body_x(t)` 在那些取樣上依然良定義。

所以 Step 6 的每個結果都要掛一個 body 假設欄位：
`body_x from Step 5; body_z INFEASIBLE (see Step 5)`。
不掛的話，一個「margin 很夠」的結論會建立在一個根本不存在的 body 上。

### signed margin 的定義

三個接觸點在水平面上構成三角形。margin = CoM 投影到**每一條邊**的
signed distance 取最小值（以三角形內部為正）。
退化情形（三點共線）要**明講**，不要回一個看起來很好的數字。

---

## 【Step 6 最重要的結果】margin 最小是 0.000 mm，五個 swing 全部不穩

```text
margin floor 10 mm     ->  5 個 swing 全部 unstable
最小 margin            ->  0.000 mm（LH 與 RH 兩個 swing 的【起點】）
其他三個 swing 最好也只有 0.995 mm
```

**為什麼剛好是 0：對稱，不是捨入誤差。**
在 LH / RH 這兩個 swing 起跳的瞬間，對角的兩隻支撐腳
量到在 `(+239.2, -211.7)` 與 `(-239.2, +211.7)` mm——
**對 body 中心完全對稱**，所以連接它們的那條邊**正好通過 body 中心**，
margin 因此正好是 0。

另外三個 swing 沒有那個對稱，但也只有 0.995 mm。
所以**這個步態在整趟裡從來沒有超過 1 mm 的餘裕**（起跳瞬間最多 20.9 mm，
然後線性衰減到邊界）。

**規格自己指出的路是 Day 13-14 調 gamma**（改變橫向支撐幾何）。
Day 12 只做 feasibility，所以 Step 6 的正確輸出就是「不穩，而且這是哪個 swing、
哪一刻、差多少」。

### 順便證明了規格要求 5 是對的

「只檢查 liftoff 那一幀」會怎樣？RF 的 swing 在起跳時 margin 是 **20.897 mm**，
輕鬆通過 10 mm 的門檻——但它在 swing 期間**線性衰減**到 0.995 mm。
只看第一幀會把一個不穩的 swing 判成穩的。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_support_stability_2d.py
    COM_BASIS              「body 中心投影，不是 whole-robot CoM」
    GAMMA_RAD = 0.0        Day 12 不調 ABAD（要求 8）
    BOUNDARY_TOLERANCE_S   1e-6 s，見下面的陷阱
    DEGENERATE_AREA_M2     1e-6 m^2（真三角形約 0.05 m^2）
    contact_offset_from_hip_m()   接觸點【相對髖】的 x，與 chain 原點無關
    SupportTriangle2D      .area_m2 / .is_degenerate / .signed_margin_m()
    StabilitySample2D      一個瞬間；margin 為 None 時【永遠不算 stable】
    SwingStability2D       一個 swing：min margin / 何時最小 / stable
    TraversalStability2D   整趟 + body 假設欄位
    swing_stability_2d() / stability_rows() / plot_stability_2d()
+ hybrid_note/scripts/experiments/day12_step6_driver.py
+ tests/test_day12_support_stability_2d.py    28 passed（約 19 秒）
```

**沒有修改任何既有檔案。** Step 6 全部是新增。

### 這一步修掉的四個 bug（都是 driver 先看出來的）

1. **support 出現四隻腳。** 邊界瞬間兩個 segment 都宣稱擁有它。
   第一版讓 airborne 優先——但那在 swing 的**結尾**是錯的（那是 touchdown，
   腳其實已經著地）。改成 segment 擁有 `[start, end)`，最後一段擁有自己的結尾。
2. **浮點邊界。** 窗口邊界是 `phase_offset * period` 的和，
   同一個瞬間一隻腳算出 `0.6`、另一隻算出 `0.5999999999999999`，
   於是有一隻腳落後一個 segment，變成「兩隻腳支撐」。
   加 `BOUNDARY_TOLERANCE_S = 1e-6`（遠低於任何真實時間，遠高於誤差）。
3. **三角形被切片。** `points[:3]` 會把四個接觸點默默算成三角形。
   改成直接 `raise`——四個點是排程結果要回報，不是幾何問題。
4. **support triangle 要以 swing leg 為準，不是每個瞬間重新問誰在空中。**
   規格寫的是「另外三隻腳」。每個瞬間重問的話，swing 的最後一個取樣
   會換成【下一個】swing 的支撐組，等於回報一個從來沒被測試的三角形。
   同理取樣要用**半開區間**：在 `end` 那一刻下一隻腳已經離地了。

### 驗收（規格 §13 的十項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 用實際 world contact point | ✅ `contact_offset_from_hip_m`，與 chain 原點無關；有測試證明平移不變 |
| 2 | 水平面 support triangle | ✅ 橫向座標是 Step 2 量到的 mounting offset |
| 3 | CoM 投影 | ✅ body 中心，且每一列都標明它不是 whole-robot CoM |
| 4 | signed margin | ✅ inside / boundary / outside 三種都有測試 |
| 5 | 整個 swing 都要算 | ✅ 而且證明了只看 liftoff 會誤判（20.897 -> 0.995 mm） |
| 6 | 每個 swing 與整趟都存最小 margin | ✅ 含「最小發生在哪一刻」 |
| 7 | margin 不足 -> 標記 infeasible | ✅ 5 個 swing 全部 unstable |
| 8 | gamma 維持 0 | ✅ `GAMMA_RAD = 0.0`，有測試 |
| 9 | 視覺化 | ✅ 最差瞬間的三角形 + CoM 投影 + swing leg，右側 margin 對時間 |
| 10 | inside / boundary / outside 單元測試 | ✅ 用手算的直角三角形，不用 planner 的輸出 |

### 產出

```text
程式  day12_support_stability_2d.py / day12_step6_driver.py   新增
      tests/test_day12_support_stability_2d.py                新增，28 passed
資料  day12_step6_stability.csv     traversal / swing / sample 三種列
      day12_step6_stability.png     最差瞬間的三角形 + margin 對時間
展示  notebook 的 Step 6 節
```

### 資料結構

```text
SupportTriangle2D    三個 contact point + swing leg + 面積 + 是否退化
StabilitySample2D    一個時刻：t / swing leg / support legs / CoM xy / margin
SwingStability2D     一個 swing 段：min margin / 何時最小 / stable
TraversalStability2D 整趟：min margin / 哪個 swing 最差 / is_stable
                     + body 假設欄位
```


## 2026-09-01 — Step 7 實作（Resolve `TOP_REPOSITION`）【已完成】

### 規格 §14 的十項要求（原文摘要）

```text
1. 找出未解的 TransitionRequirement2D(kind=TOP_REPOSITION)
2. 把目標腳排成 airborne，另外三隻維持支撐
3. 對提出的 reposition 區間跑 Step 6 的 support triangle / CoM margin 檢查
4. 支撐不足 -> 回 unresolved/infeasible，【不要】發明 ABAD 補償
5. 支撐足夠 -> 用 Day 8-9 的 Cartesian swing generator 生成空中段
6. touchdown 必須是下一個 primitive 合法的 ContactState
7. 對整條 reposition 軌跡重跑地形碰撞 / 接觸驗證
8. 成功則標記 resolved 並存下產生的 segment
9. 重測那些【只因為 direct handoff 不可行】而失敗的 Day 10-11 case
10. 保留所有原始失敗原因以利追溯
```

### 不要寫第二套 swing generator（規格明令）

重用鏈是現成的：

```text
standing_scene_2d(spec, theta, hip_x_m=..., support_height_m=spec.top_z_m)
        ↓  兩個 scene（起點在平台上、終點也在平台上，只是換了姿態/位置）
build_swing_request_2d(start_scene, target_scene, ...)
        ↓
generate_swing_2d(request)      <- Day 8-9，已含碰撞與 touchdown 驗證（要求 7）
        ↓
segment_from_swing_plan_2d(...) <- Day 10-11 已有的寫入 schema 的函式
```

### `resolved = True` 不能寫回 requirement —— schema 自己擋

`TransitionRequirement2D.__post_init__` 明講：

```text
a resolved transition is a segment, not a requirement:
replace the record with the motion that solves it.
```

所以 Step 7 的產出是一個**解決紀錄**（`TopRepositionAttempt2D`），
裡面同時放**原始的 requirement**（要求 10 的追溯）與**生成出來的 segment**。
不是把旗標翻掉。

### `target_condition` 要變成機器可檢查的東西（要求 6）

Day 10-11 把它寫成人話。Step 7 要能真的判斷，所以每個 blocked pair
對應一個可檢查的條件，直接讀 swing 最後一個 sample：

```text
#2 ROLL_SWING   touchdown 在 foot rim 且 theta >= 35 deg（Step 3 F 的下限）
#3 SWING_ROLL   LEFT_RIM_READY：left rim 承載、theta 約 17 deg
```

### 【先講清楚】這一步的答案很可能是「支撐不足」

Step 6 已經量到**平地走路本身** margin 最小就是 0.000 mm。
在那個基礎上問「reposition 期間三腳撐不撐得住」，答案幾乎一定是不撐得住。

**規格的驗收本來就允許兩種結果之一**：

```text
resolved = True   或   有明確的 support-related failure reason
```

所以 Step 7 的工作不是「想辦法讓它 resolved」，
而是**讓那個判定真的被跑出來、而且理由是真的**。
要防的是把「支撐不足」偷偷換成「再放寬一點門檻就好」。

為了證明 gate 兩個方向都會動，會**額外跑一次放寬 margin floor 的查詢**，
並且在報告裡標明那是 `RELAXED_FLOOR` 的探索，不是結論。

---

## Step 7 的結果

```text
planning floor（10 mm）
  #2 ROLL_SWING   support_insufficient   margin 0.995 mm < 10 mm
  #3 SWING_ROLL   support_insufficient   margin 0.995 mm < 10 mm
  -> 兩個都【沒有】生成任何軌跡。gate 沒過就停，不生成（要求 4）。

RELAXED floor（探索用，【不是答案】）
  #2 ROLL_SWING   resolved = True
                  Day 8-9 swing 通過碰撞與 touchdown 驗證，
                  落在 foot rim、theta 37.00 deg，滿足 >= 35 deg 的條件
  #3 SWING_ROLL   swing_failed / JOINT_DISCONTINUITY
```

**規格 §14 的驗收兩種都接受**（`resolved = True` 或明確的 support-related
failure reason），所以 planning floor 下的兩個 `support_insufficient`
就是通過驗收的答案；`#2` 在放寬支撐後真的走完整條鏈，證明機制本身是通的。

### `#3` 的失敗是一個真發現，不是「跑不出來」

`LEFT_RIM_READY` 要的是**左輪緣承載**，而那需要一個離起始姿態很遠的 beta。
直接一個 swing 過去會被 Day 8-9 判成 `JOINT_DISCONTINUITY`。
換句話說：**`#3` 的 reposition 不是「支撐夠了就能做」，它還缺一段把 beta 轉過去的動作。**

### 這一步改到既有檔案（Step 0 以來第一次）

`TOP_REPOSITION_SWING` 是**新的 `SegmentKind`**。理由是不加就得說謊：

```text
SWING_OVER      是「一個 primitive 跨過整個障礙」——不是這個
RECOVERY_SWING  是「步態本來就會做的」——這個是地形逼出來的
```

規格 §18 要求 terrain-transition swing 要單獨計數，所以借用別的 kind
會直接污染那個統計。改動是**純新增**：

```text
day10_11_motion_schema_2d.py   加 SegmentKind.TOP_REPOSITION_SWING，
                               並列進 is_swing
day12_segment_contract_2d.py   加進 SEGMENT_SEMANTICS
day12_transition_mapping_2d.py 加進 TRANSITION_PHASE_OF_KIND（phase = ON_TOP）
tests/（兩個）                 「三個 terrain-transition swing」的斷言改成四個
```

那兩個測試失敗**是它們在做事**：Step 0 就是設計成新增 kind 一定會撞到。

### 兩個量錯又修正的地方

1. **對著 floor 瞄準會失敗。** 第一版把 target theta 設成 35 deg（= 下限），
   IK 解出來是 **34.99955 deg**，差 0.00045 deg 被判不合格。
   touchdown theta 是 IK 的**輸出**不是請求的輸入（Day 10-11 陷阱 16）。
   -> 加 `TARGET_THETA_HEADROOM_RAD = 2 deg`：**請求**瞄在下限之上，
      **檢查**仍然用下限本身。

2. **`#3` 的 target 不能用 standing scene。** 第一版兩邊都用 `standing_scene_2d`，
   結果 `#3` 落在 foot rim（theta 17 deg 的站姿仍是腳輪緣接觸），
   而它要的是 left rim。
   -> 改用 Day 10-11 已經有的 `left_rim_beta_window_2d` + `choose_landing_beta_2d`
      + `left_rim_landing_scene_2d`。**不是新寫一套**，是接既有的。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_top_reposition_2d.py
    RepositionOutcome        SUPPORT_INSUFFICIENT / SWING_FAILED
                             / TOUCHDOWN_UNSUITABLE / RESOLVED
    RepositionTarget2D       把 target_condition 變成可檢查的述詞
    REPOSITION_TARGETS       #2 -> foot rim, theta >= 35 deg
                             #3 -> left rim, LEFT_RIM_READY 的 theta 區間
    TARGET_THETA_HEADROOM_RAD
    SupportGate2D            Step 6 的檢查，套在 Step 3 已經排好的空中區間上
    TopRepositionAttempt2D   原始 requirement + gate + swing + segment + 理由
    resolve_top_reposition_2d()   順序是【支撐先、動作後】
    reposition_rows()
~ day10_11_motion_schema_2d.py     加一個 SegmentKind（純新增）
~ day12_segment_contract_2d.py     語意表加一列
~ day12_transition_mapping_2d.py   phase 表加一列
~ tests/test_day10_11_motion_schema_2d.py     斷言 3 -> 4
~ tests/test_day12_segment_contract_2d.py     斷言 3 -> 4
+ hybrid_note/scripts/experiments/day12_step7_driver.py
+ tests/test_day12_top_reposition_2d.py
```

### 驗收（規格 §14 的十項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 找出未解的 `TOP_REPOSITION` | ✅ 用 Day 10-11 自己的 `top_reposition_requirement_2d` |
| 2 | 目標腳 airborne、其餘三隻支撐 | ✅ 直接讀 Step 3 已經排好的空中區間，不另外發明 |
| 3 | 跑 Step 6 的 margin 檢查 | ✅ `SupportGate2D` |
| 4 | 支撐不足 -> unresolved，不發明 ABAD | ✅ gate 沒過就**完全不生成軌跡**，有測試擋住 |
| 5 | 重用 Day 8-9 swing generator | ✅ `build_swing_request_2d` + `generate_swing_2d`，沒有第二套 |
| 6 | touchdown 要是下一個 primitive 合法的狀態 | ✅ `RepositionTarget2D.accepts()`，讀 swing 最後一個 sample |
| 7 | 對整條軌跡重跑碰撞 / 接觸驗證 | ✅ `generate_swing_2d` 內建，靠重用滿足而不是重寫 |
| 8 | 成功則存下 segment | ✅ 存在 attempt 記錄裡；**不是**把 requirement 的旗標翻掉（schema 擋） |
| 9 | 重測原本只因 direct handoff 失敗的 case | ✅ `#2` 與 `#3` 兩個 blocked pair 都跑了 |
| 10 | 保留原始失敗原因 | ✅ `original_evidence`，有測試確認沒被覆蓋 |

### 產出

```text
程式  day12_top_reposition_2d.py / day12_step7_driver.py   新增
      tests/test_day12_top_reposition_2d.py                新增
資料  day12_step7_reposition.csv    attempt / support_gate 兩種列
展示  notebook 的 Step 7 節
```


## 2026-09-01 — Step 8 實作（完整四腳 Joint / Contact Trajectory）【已完成】

### 規格 §15 的八項要求（原文摘要）

```text
1. 每個取樣都要有 body pose 與四隻腳的 theta/beta/gamma
2. 每隻腳要有明確的 mode / contact / rim / alpha
3. 要含 swing leg、support legs、stability margin、segment_index、segment_kind
4. 保留 segment 的 sampling 參數
5. 驗證所有 segment handoff：時間 / body / joint / contact-state 連續性
   ＋ 已知的 rim-geometry handoff gap
6. 量化那個約 1.2 mm 的幾何差異；【不要】為了藏它而改幾何模型
7. 不做 runtime replanning
8. 若既有 trajectory writer 可重用就加上序列化
輸出摘要：最大 joint jump、最大 contact gap、最小 stability margin
```

### 輸入全部是前面步驟已經算好的東西

```text
Step 3  FourLegSchedule2D     共同時間軸、每個瞬間誰在空中
Step 4  FourLegPlan2D         四隻腳的 segment 序列（含 phase / source）
Step 5  BodyTrajectory2D      body_x(t) / body_z(t) ＋ driver 出處
Step 6  TraversalStability2D  每個 swing 的 margin
Step 7  TopRepositionAttempt2D  已解 / 未解的 TOP_REPOSITION
```

**Step 8 不重算任何一個**，它只是把它們對齊到同一組取樣上。
規格要求 7（不做 runtime replanning）在這裡等於：**這一步不呼叫任何 planner**。

### 1.2 mm 是什麼、以及為什麼不修

`LegModel.rim_point` 用 0.145 m，接觸 pipeline 用的繪製弧是 0.1438 m。
差 1.2 mm，而 contact tolerance 是 1 mm。
`rim_point_model_gap_2d(theta, beta, rim, alpha)` 是既有的量測函式——
Step 8 **重用它**，在每個 handoff 上量一次，不自己重推。

Day 8-9 與 Day 10-11 都決定「只量不修」，Step 8 沿用。
規格特別加一句「不要為了藏它而改幾何模型」，所以這個值要**出現在輸出裡**。

註：Day 10-11 已經量過它在 foot rim 上是 0、在 upper tyre 上才是 1.2 mm，
所以摘要要報**最大值**，不是變化量。

### 四個未解結論要掛在輸出上

Step 4 的 overrun、Step 5 的 body infeasible、Step 6 的零 margin、
Step 7 的 support 不足——**Step 8 組出來的軌跡是「在這四件事都還在」的前提下組的**。
所以 trajectory 物件要有一個 assumptions 欄位把它們列出來，
不要讓「Step 8 完成」被讀成「這條路通了」。

### 資料結構

```text
LegSample2D          一隻腳在一個瞬間：theta/beta/gamma、mode、rim、alpha、
                     contact 世界座標、segment_index、segment_kind、phase
WholeBodySample2D    一個瞬間：時間、body pose、四隻腳、swing leg、support legs、
                     stability margin
HandoffCheck2D       一個 segment 交接：時間 / body / joint / contact 跳變
                     ＋ rim geometry gap
WholeBodyTrajectory2D  取樣序列 + handoff 檢查 + 摘要 + assumptions
```

---

### 結果

```text
規格 §15 要的三個摘要數字
  max joint jump          360.000 deg   （raw，含整圈）
  max joint discontinuity   0.000 deg   <- wrapped；4 個交接是【整圈】不是斷點
  max contact gap         297.065 mm    <- 陷阱 25 的 chain break，不是新東西
  min stability margin      0.000 mm    <- Step 6 的結論

rim geometry gap          0.0000 mm
  【但這不代表 1.2 mm 不存在】——這一趟全程都在 foot rim 上，
  而 foot rim 上的 gap 依定義是 0；1.2 mm 是在 upper tyre 上。
  摘要裡有 rim_gap_note 這一欄講清楚這件事。

finite body_z samples     0 of 241
  Step 5 說 body 高度不可行，所以【一個取樣都沒有】可用的高度。

shape
  samples 241 / handoffs 12 / time monotonic True / 每個取樣都有四隻腳
```

### 三個「這條軌跡是【疊在什麼之上】」的欄位

`WholeBodyTrajectory2D.assumptions` 是**算出來的**，不是寫死的：

```text
- Step 5: body_z is INFEASIBLE -- 451 conflicts, worst 15.329 mm apart
- Step 6: 5 of 5 swings are unstable -- minimum margin 0.000 mm
- Step 7: 2 TOP_REPOSITION requirement(s) remain unresolved
```

（平地這一趟沒有越障，所以 Step 4 的 overrun 不在列——那是因為它真的不存在，
不是被漏掉。有測試擋住「無中生有一條 Step 4 假設」。）

### 三個修正

1. **360 deg 不是斷點，是一整圈。**
   第一版自己算 joint jump，報出 360 deg 看起來像大災難。
   實際上 Step 1 建 recovery 時就是 `beta_target = start.beta - 2*pi`——
   **beta 在這整棵樹裡是圈數計數器，從來不 wrap**。
   -> 改成**兩個都報**：raw（含整圈）與 wrapped（是不是真的不連續）。
      wrapped 是 0.000 deg，而且 `is_whole_turn` 直接標出那 4 個交接。

2. **handoff 的量測不要自己寫第二套。**
   Step 0 早就把 `handoff_between_2d` 抽出來，理由正是
   「Day 12 要跨 frame source 串 segment」。第一版重寫了一次。
   -> 改成 delegate 給它。

3. **segment 查詢也不要自己寫第二套。** 同一個錯誤犯第二次：
   Step 6 的 `segment_at` 有邊界容差與半開區間規則，第一版又手寫了一遍，
   結果在 covered interval 的**最後一刻**只找到兩隻支撐腳。
   -> `_segment_at` 升成公開的 `segment_at`，Step 8 直接用。
      取樣也改成半開 `[lo, hi)`：在 `hi` 那一刻一隻腳的 chain 已經用完、
      另一隻的 swing 已經開始，那個瞬間根本沒有完整的四腳組態。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_whole_body_trajectory_2d.py
    LegSample2D            theta/beta/gamma + mode/rim/alpha/contact
                           + segment_index/kind/phase + arc_samples
                           + rim_geometry_gap_m
    WholeBodySample2D      body pose + 四隻腳 + swing/support + margin
    HandoffCheck2D         五種檢查，含 raw 與 wrapped 兩個 joint 讀數
    WholeBodyTrajectory2D  .max_joint_jump_rad / .max_joint_discontinuity_rad
                           .max_contact_gap_m / .max_rim_geometry_gap_m
                           .minimum_stability_margin_m / .assumptions
    assumptions_of()       從 Step 4-7 的結果算出來
    assemble_whole_body_2d() / whole_body_rows()
~ day12_support_stability_2d.py   `_segment_at` -> 公開的 `segment_at`
+ hybrid_note/scripts/experiments/day12_step8_driver.py
+ tests/test_day12_whole_body_trajectory_2d.py    24 passed（約 22 秒）
```

### 驗收（規格 §15 的八項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 每個取樣有 body pose 與四腳 θβγ | ✅ 有測試逐點檢查 |
| 2 | 每腳有 mode/contact/rim/alpha | ✅ 且 `in_contact` 與 mode 一致 |
| 3 | swing leg / support legs / margin / segment index+kind | ✅ |
| 4 | 保留 segment 的 sampling 參數 | ✅ `arc_samples` 直接對得上原 segment |
| 5 | 五種 handoff 檢查 | ✅ 時間 / body / joint（兩個讀數）/ contact / rim gap |
| 6 | 量化 1.2 mm，不為了藏它改幾何 | ✅ 用既有的 `rim_point_model_gap_2d`；並註明這趟為何是 0 |
| 7 | 不做 runtime replanning | ✅ 這個模組**不呼叫任何 planner**，只在既有結果之間內插 |
| 8 | 序列化 | ✅ `whole_body_rows()`，四種 row kind，單一表頭 |

### 產出

```text
程式  day12_whole_body_trajectory_2d.py / day12_step8_driver.py   新增
      tests/test_day12_whole_body_trajectory_2d.py                新增，24 passed
資料  day12_step8_whole_body.csv   summary / assumption / handoff / sample
展示  notebook 的 Step 8 節
```


## 2026-09-01 — Step 9 實作（Whole-Body Validation）【已完成】

### 規格 §16 的要求（原文摘要）

```text
驗證【組好的四腳軌跡】，不是只驗個別 primitive。至少要檢查：
時間嚴格遞增 / 最多一隻腳 airborne / 關節極限與連續性 / body 連續性 /
stance 接觸有效 / swing 無碰撞 / touchdown 接觸狀態符合預期 / 地形碰撞 /
segment 之間狀態連續 / 所有 active body requirement / support triangle 與最小 margin
回傳【結構化的失敗原因】：time、leg、segment_index、segment_kind、相關數值
【不要在 validator 裡偷偷修好無效的軌跡】
```

### 結果：11 個檢查，7 過 4 失，251 個失敗記錄

```text
PASS  time_strictly_increasing      時間嚴格遞增
PASS  at_most_one_airborne          同時最多一隻腳在空中
PASS  three_support_legs            每次 swing 剛好三隻支撐腳
PASS  theta_within_limits           theta 全程在 [17, 160] deg 內
PASS  joint_continuity              段【內】沒有關節跳變
PASS  body_continuity               body 沒有瞬移
PASS  stance_contact_valid          stance/airborne 與 in_contact 一致

FAIL  beta_workspace_guard      240  見下
FAIL  body_requirement_satisfied  2  Step 5 的結論
FAIL  segment_chaining            4  陷阱 25 的 chain break（297.065 mm）
FAIL  support_margin              5  Step 6 的結論
```

**四個 FAIL 沒有一個是 Step 9 自己製造的**——每一個都指回 Step 5 / 6 / 8 已經
記錄過的結論。Step 9 的價值在於它**獨立地**把它們抓出來，而且七個通過的檢查
證明時間軸、關節極限、連續性這些是真的沒問題。

### `beta_workspace_guard`：可能是【語意不合】而不是硬體違規

`RobotParams.BETA_MAX_DEG = 40` 註解寫的是 "Sagittal swing geometric limit"，
被 `gait_generator_3d` 與 `obstacle_walk` 使用——那兩個把 beta 當**有界的擺動**。

但 Hybrid 的 nominal cycle 把 beta 當**圈數計數器**（Step 1：`beta_target = start.beta - 2*pi`）。
所以 recovery 一定會超出 ±40 deg，實測 240 個取樣違規。

**Step 9 回報這個違規，但明確拒絕判定哪一種讀法才對**——
每一筆失敗的 detail 都寫著「this may be a difference in meaning rather than a
hardware violation」。這是 Day 13-14 或硬體那邊要回答的。

### 【Step 3 欠的那筆帳】10.553 倍，實測出來了

Step 3 從 duty 推導出「recovery 的 beta 角速度要比 rolling 快 10.553 倍」，
並明講不判斷它是否超出關節極限、把問題留給 Step 9。

現在 Step 8 把每一幀都排好了，所以可以**直接量**：

```text
leg   peak theta   peak beta   stance beta   airborne beta    ratio
LF        0.00       467.19        44.27          467.19     10.553
（四隻腳都一樣；單位 deg/s）
```

**實測比值 = 10.553，與 Step 3 從 duty 推導的完全一致**——
兩條完全不同的路徑得到同一個數字。

### 但 Step 9 【不能】判定它可不可行——而且原因要講清楚

```text
RobotParams 裡【沒有關節速度極限】。
只有 SWING_ACCEL_MAX（swing 塑形用）與 TOUCHDOWN_VEL_H_MAX（觸地用），
兩個都不是馬達轉速上限。
```

所以 Step 9 **量得到需求、比不到極限**。這寫進 `UNEVALUABLE_CHECKS`——
和 `DELEGATED_CHECKS`（在別處驗過的）分開列，因為
**「沒檢查」和「檢查過而且通過」不是同一件事**，兩者都不能被讀成後者。

### 另一個必須講的量測限制：peak theta 讀出來是 0

Step 8 是在 segment 的**兩個端點之間內插**，而一個 RECOVERY_SWING 的頭尾
theta 都是 60 deg——所以它中間「縮到 compact 姿態再伸出來」的過程
**根本不在組好的取樣裡**。

如果不講，`peak theta = 0.00 deg/s` 會被讀成「theta 全程不動」。
所以每一列都帶 `theta_rate_is_lower_bound = True`，
並在 `UNEVALUABLE_CHECKS` 裡說明真正的幀在 `FrameRef2D` 後面、
要拿到得呼叫 Step 8 刻意不呼叫的 generator。
（beta 不受影響：它在端點之間是單調前進的。）

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_whole_body_validation_2d.py
    CheckId                11 個檢查
    DELEGATED_CHECKS       4 個「在別處驗過」，附上是誰驗的
    UNEVALUABLE_CHECKS     2 個「這裡根本判不了」，附上缺什麼
    ValidationFailure2D    check/time/leg/segment_index/segment_kind/value/limit/detail
    ValidationReport2D     .is_valid / .failed_checks() / .passed_checks()
                           .failures_of() / .joint_rates / .assumptions
    JointRateMeasurement2D 每腳的 peak theta/beta 速率，stance 與 airborne 分開
    joint_rates_2d()       只取【段內】的步進：段界是一整圈，除以取樣間隔會編出
                           一個沒人在要求的速率
    validate_whole_body_2d() / validation_rows()
+ hybrid_note/scripts/experiments/day12_step9_driver.py
+ tests/test_day12_whole_body_validation_2d.py    29 passed（約 22 秒）
```

**沒有修改任何既有檔案。** Step 9 全部是新增。

### 測試怎麼寫的

每個檢查都用**手工組的取樣**單獨觸發一次（兩隻腳在空中、swing 只有兩隻支撐、
時間倒退、theta 超限、stance 沒接觸、段內關節瞬移、body 瞬移），
這樣才能證明它是**為自己的理由**而失敗，
而不是被別的檢查找到的問題連帶記在頭上。

### 一個測試自己的坑

「validator 不修東西」那個測試，第一版直接比較 `as_dict()` 的 list——
但裡面有 `nan`，而 `nan != nan`，所以它報的是**自己的比較方式**壞掉，
不是待測程式壞掉。改成 nan-aware 比較，並額外斷言連容器物件都沒被換掉。

### 驗收（規格 §16）

| 要求 | 結果 |
|---|---|
| 時間嚴格遞增 | ✅ PASS |
| 最多一隻腳 airborne | ✅ PASS |
| 關節極限 | ✅ theta PASS；beta 的 guard 另外報（語意問題） |
| 關節連續性 | ✅ 段內 PASS；段界用 wrapped 讀數 |
| body 連續性 | ✅ PASS |
| stance / 接觸有效 | ✅ PASS |
| swing 無碰撞 | ✅ delegated（Day 8-9 每幀驗過），已列出 |
| touchdown 接觸狀態 | ✅ delegated（Step 7 的 `RepositionTarget2D`），已列出 |
| 地形碰撞 | ✅ delegated（Day 6-7 / Day 8-9 生成時每幀驗過），已列出 |
| segment 之間連續 | ✅ FAIL 4 —— 陷阱 25，回報不修 |
| body requirement | ✅ FAIL 2 —— Step 5 的 INFEASIBLE |
| support triangle 與最小 margin | ✅ FAIL 5 —— Step 6 的零 margin |
| 結構化失敗原因 | ✅ 八個欄位，有測試逐筆檢查 |
| 不偷偷修 | ✅ 有測試斷言軌跡與容器都沒被動過 |

### 產出

```text
程式  day12_whole_body_validation_2d.py / day12_step9_driver.py   新增
      tests/test_day12_whole_body_validation_2d.py                新增，29 passed
資料  day12_step9_validation.csv   summary / check / failure / delegated
                                   / unevaluable / joint_rate / assumption
展示  notebook 的 Step 9 節
```


## 2026-09-01 — Step 10 實作（參數化地形整合 + Generalization Gate）【已完成】

### 規格 §17 要驗的到底是什麼

> **不是**「planner 能不能過某一個特定尺寸」，
> 而是「**同一套** whole-body planner 能不能在**不改核心 code** 的前提下
> 吃不同的矩形地形參數，產生合法軌跡**或**明確的 infeasible reason」。

### 結果：四個地形，同一個入口，gate 乾淨

```text
   terrain  feasible   ascent     descent    nom sw  tt sw   lift    margin
      flat     False     None        None         8      0   n/a      0.000
 40 x 400      False   SWING_UP  SWING_DOWN       8      8   n/a      0.000
100 x 400      False   SWING_UP  SWING_DOWN       8      8   n/a      0.000
190 x 400      False     None        None         0      0   n/a      n/a
```

**平地 / 4 cm / 10 cm 走的是同一條 pipeline**，差別只有
terrain-transition swing 從 0 變成 8——那正是規格要求 4 說的
「平地要跑 nominal cycle，不要多出不必要的越障 swing」。
兩個障礙的 primitive 都是 Day 10-11 的 `decide_2d` 選的，Step 10 沒有選任何東西。

**19 cm 在 decision 階段就停住**，理由是：

```text
not measured: no data: the rolling traversal was never swept at this height.
```

這是**資料沒有**，不是**物理上不可能**——這個區別很重要，
規格說這個 failure 本身有研究價值，而它指出的待辦是「去掃那個高度」。

### Generalization gate 抓到我自己的違規

`planner_size_literals()` 掃 9 個 planner 模組裡有沒有出現評估尺寸的字面值。
第一版跑出來抓到**我自己剛寫的那一行**：

```python
# 為了拿 nominal body height 而蓋了一個 4 cm 的平台
reference = terrain if terrain is not None else SharedTerrainSpec2D(
    height_m=0.04, top_length_m=0.40, ...)
```

一個 planner 根本不需要的尺寸，變成了 planner 裡的常數。

修法是回到 Step 2 早就講過的事實：**nominal 站姿在下層地面上，與平台無關**——
所以它也**不該用蓋平台的方式取得**。改成從腿的取樣幾何直接解：

```text
nominal_body_height_m() = posture.hip_z_for_flat_stance(0) - ABAD_AXIS_OFFSET
                        = 162.2826 mm     （與 Step 2 量到的 162.283 一致）
```

現在 gate 只剩一個 hit，而且是誤判：
`0.10 * schedule.timing.cycle_period_s` 是**週期比例不是長度**。
測試用一個**寫明理由的 allowlist** 接受它，並且**另外斷言那一行還在**——
allowlist 活得比它的理由久，就變成一個被消音的檢查。

### 「max body lift = 0.000 mm」是假的，已修

`body_z_travel_m` 會跳過 NaN，而 Step 5 之後幾乎全是 NaN，
只剩 1 個有限值 -> travel = 0。那個 0 會被讀成「body 完全沒有起伏」，
意思**正好相反**。改成有限取樣少於 2 個時回 `None`，
並且多報一欄 `usable_body_samples`（實測：241 個裡只有 1 個）。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_terrain_generalization_2d.py
    plan_terrain_2d(terrain, tables)   <- 【唯一】入口；terrain=None 就是平地
    nominal_body_height_m()            從腿解，不蓋平台
    TerrainRun2D                       .feasible / .ascent_primitive
                                       .descent_primitive
                                       .nominal_recovery_swings
                                       .terrain_transition_swings
                                       .max_body_lift_m / .usable_body_samples
                                       .min_stability_margin_m
                                       .first_limiting_constraint
    TerrainFailure2D / Stage           decision / composition / assembly / validation
    planner_size_literals()            generalization gate 本體
    comparison_rows()                  規格要求 7 的表
+ hybrid_note/scripts/experiments/day12_step10_driver.py
+ tests/test_day12_terrain_generalization_2d.py    23 passed（約 40 秒）
```

**沒有修改任何既有檔案。** Step 10 全部是新增。

### 又一個測試自己寫錯的例子

「原始碼裡不可以有對高度的分支」那個測試，第一版的 regex 是
`height_m\s*[<>=]=?\s*0\.\d`——裡面那個裸的 `=` 讓它去比對到
`height_m = 0.0 if terrain is None else ...` 這個**賦值**。
那一行是依「有沒有障礙」分支，不是依「障礙多大」分支。
**測試錯了不是程式錯了**，regex 已改成只認比較運算子。

### 驗收（規格 §17）

| 要求 | 結果 |
|---|---|
| 1 不得有 height/length 專用分支 | ✅ `planner_size_literals()` 機器檢查，唯一 hit 有書面理由 |
| 2 障礙一律走 Day 10-11 的介面 | ✅ `decide_2d` + `compose_2d`，Step 10 不選策略 |
| 3 一律走 Day 12 的四腳 pipeline | ✅ 同一段程式，terrain 只以資料傳遞 |
| 4 平地跑 nominal cycle，不多加越障 swing | ✅ 平地 kinds 只有 `{FOOT_RIM_ROLL, RECOVERY_SWING}`，tt swing = 0 |
| 5 ascent / descent 各自獨立 | ✅ 分開回報 |
| 6 不可行要回結構化原因，不得放寬約束 | ✅ 19 cm 在 decision 停住，四個策略各自有理由 |
| 7 比較表 | ✅ `comparison_rows()`，九個欄位 |
| 8 測試斷言沒有 experiment-ID 特例 | ✅ 字面值掃描 ＋ 分支 regex ＋ 入口簽章檢查 |

**Test A（平地）** 證明了規格說的那件事：
Hybrid 的 nominal locomotion **本身不是 obstacle-specific script**。

### 產出

```text
程式  day12_terrain_generalization_2d.py / day12_step10_driver.py   新增
      tests/test_day12_terrain_generalization_2d.py                 新增，23 passed
資料  day12_step10_generalization.csv   規格要求 7 的比較表 + 結構化失敗
      day12_step10_size_literals.csv    gate 掃到的每一行
展示  notebook 的 Step 10 節
```


## 2026-09-01 — Step 11 實作（Paper Metrics）【已完成】

### 規格 §18 的兩條紅線

```text
1. body centre 的指標與 whole-robot CoM 的指標【必須分開】
2. 在還沒做 energy experiment 之前，【不得】從這些指標推出 COT
```

第 2 條做成**機器可檢查**：`energy_vocabulary()` 掃這個模組的**程式碼**
（跳過 docstring 與註解，也跳過禁用字清單本身），
出現 `energy` / `power` / `cot` / `joule` 等字就回報。實測回空。
另外有一個測試用一個假檔案證明這個 guard **真的會叫**——不會叫的 guard 不是 guard。

### 結果

```text
    terrain   dist mm  dur s  swings  nom   tt   roll ls  swing ls
       flat    491.02  2.975       8    8    0     8.926     2.975
 40 x 400      491.02  2.975      16    8    8     8.926     2.975
100 x 400      491.02  2.975      16    8    8     8.926     2.975
190 x 400      沒有軌跡可量（decision 階段就停住）

    terrain  bodyz p2p  bodyz std  usable   min margin  mean margin  hip lift
       flat        n/a        n/a   0/121       0.0000      10.5482     0.000
 40 x 400          n/a        n/a   0/121       0.0000      10.4869    40.000
100 x 400          n/a        n/a   0/121       0.0000      10.4869   100.000

    terrain  max joint disc  max contact gap  roll dist  transition dist
       flat          0.0000          297.065     491.02             0.00
 40 x 400           39.8438          652.696     491.02             0.00
100 x 400           39.8438          652.696     491.02             0.00
```

### 第一版的時間指標是廢的，已修

原本 `roll s` 與 `swing s` 都等於整段時長 2.975 s——因為判斷式是
「**任何**一隻腳在這個 kind」，而任何時刻都同時有腳在滾、有腳在空中。
那個數字永遠等於整段長度，量不到任何東西。

改成 **leg-seconds**（四隻腳分別計時再加總），意義明確而且**會加總**：

```text
roll 8.926 + swing 2.975 = 11.901 = 4 x 2.975   （有測試釘住）
```

欄位名也一起改成 `*_leg_seconds`，因為叫 `time_s` 會被讀成牆上時鐘。

**距離**則另外處理：body 前進是共用的，要歸給哪個 kind 需要一條規則。
現在的規則是「至少一隻**站立**腳在該 kind 的區間內的 body 前進」，
而且**明講這些類別會重疊、不可以相加**（不同腳可以同時處在不同 kind）。

### `transition dist = 0` 不是漏量

`#4 SWING_SWING` 的越障 primitive **是 swing 不是 roll**，
所以「transition ROLL 距離」本來就是 0。那是**不存在**，不是**沒量到**。

### `bodyz p2p = n/a` 而不是 0

Step 5 之後 121 個取樣裡**一個**都沒有可用的 body 高度。
如果照算會得到 0，而 0 會被讀成「body 完全不起伏」，意思正好相反。
所以有限取樣少於 2 個就回 `None`，並且把 `usable_body_samples` 一起報出去。

### 實作結果

```text
+ hybrid_note/scripts/experiments/day12_paper_metrics_2d.py
    COM_METRICS_ABSENT     為什麼沒有 CoM 指標（而不是填 0 或填 body 的）
    ENERGY_WORDS / energy_vocabulary()   §18 第 2 條紅線的機器檢查
    TrajectoryMetrics2D    §18 的完整清單；量不到的一律 None 附理由
                           .com_z_peak_to_peak_m / .com_z_std_m 永遠 None
    trajectory_metrics_2d() / metrics_rows()
+ hybrid_note/scripts/experiments/day12_step11_driver.py
+ tests/test_day12_paper_metrics_2d.py    19 passed（約 27 秒）
```

**沒有修改任何既有檔案。** Step 11 全部是新增。

### 驗收（規格 §18）

| 要求 | 結果 |
|---|---|
| traversal distance / duration | ✅ 491.02 mm / 2.975 s |
| total swing count | ✅ 平地 8、越障 16 |
| nominal RECOVERY_SWING count | ✅ 一律 8（不受地形影響） |
| terrain-transition swing count | ✅ 平地 0、越障 8 |
| FOOT_RIM_ROLL 的時間 / 距離 | ✅ leg-seconds ＋ 距離（並標明重疊） |
| transition ROLL 的時間 / 距離 | ✅ 0，因為 #4 的 primitive 是 swing |
| body_z peak-to-peak / std | ✅ `None` ＋ `usable_body_samples`，不填假的 0 |
| CoM_z 指標（若有 model） | ✅ **沒有 model，所以不報**，附完整理由 |
| 最小 support margin | ✅ 0.0000 mm |
| swing 期間平均 margin | ✅ 10.49-10.55 mm |
| 最大 body / hip lift | ✅ 平地 0、40 mm、100 mm（隨障礙高度） |
| 最大 joint handoff 不連續 | ✅ 平地 0、越障 39.84 deg |
| 最大 contact handoff gap | ✅ 297.065 / 652.696 mm |
| body centre 與 CoM 分開 | ✅ 兩個 basis 欄位，且有測試斷言它們不相等 |
| 不得推導 energy / COT | ✅ `energy_vocabulary()` 回空，且 guard 本身有測試 |

### 產出

```text
程式  day12_paper_metrics_2d.py / day12_step11_driver.py   新增
      tests/test_day12_paper_metrics_2d.py                 新增，19 passed
資料  day12_step11_metrics.csv   每個地形一列 ＋ 兩列 provenance note
展示  notebook 的 Step 11 節
```


# 1.5 問題總排查（2026-09-01，Day 13 之前的全面盤點）

> **為什麼有這一節。** Day 12 原本預期產出 feasible 的軌跡，實際上**四個評估尺寸
> 沒有一個做到**。這一節不只列「擋住 feasible 的東西」，而是把 Day 12 目前
> **所有**已知問題排在一起，包括不影響 feasibility 但會影響研究主張、
> 上機安全、或判讀正確性的。
>
> **不預設任何一項是「因為沒有 ABAD」。** 不對稱地形都還沒做，
> 現在把問題歸因給缺少 ABAD 是沒有根據的。下面每一項都寫它**實際**的機制。

## A. 直接擋住「產生 feasible 軌跡」的

| #   | 問題                            | 量到的數字                                          | 狀態                                                             |
| --- | ----------------------------- | ---------------------------------------------- | -------------------------------------------------------------- |
| A1  | Step 5：三隻站立腳要求的 body 高度不一致    | 差 15.329 mm，227 個衝突                            | **已解**（Day 13 θ 補償）：0 衝突                                       |
| A2  | Step 8：前後兩段 nominal run 接不起來  | 297.065 mm                                     | **已解**（連續生成 ＋ recovery 落點）：0.000 mm                            |
| A3  | Step 9：`beta_workspace_guard` | 240 筆                                          | **已解**（硬體事實：β 可連續轉）                                            |
| A4  | Step 6：support margin         | 最小 **0.000 mm**，5/5 swing unstable             | **已解**（§1.6）：duty 0.85 → 4.839 mm，floor 3 mm 推導而得，Step 9 12/12 |
| A5  | 越障：組出來的軌跡**裡面沒有障礙物**          | 隱含位置散佈 1045.3 mm；接觸點最遠 789 mm，平台在 1000-1400 mm | **未解**（附錄 B 量化）                                                |
| A6  | 越障：`TOP_REPOSITION` 沒有生成      | 頂面站立時間 **0.000 s**，同一瞬間觸地又離地；缺 160 mm          | **未解**                                                         |
| A7  | 19 cm：從來沒有掃過                  | decision 階段就停                                  | **未解**（資料缺口，非物理限制）                                             |

| A8  | **【新發現，§1.6】** 馬達速率量測方式錯誤     | 同一條軌跡 241 取樣讀 48.1%、1921 取樣讀 127.7%           | **已解**（改成 frame-to-frame）：43.2%    |
| A9  | **【新發現，§1.9】** 段落被排到 body 軌跡涵蓋區間之外 | RF/LH 的 `ROLL_DOWN` 三段 `delivered = 0.000 mm`，靜靜地當成沒位移 | **未解**                              |

**平地【12/12 全過】（2026-09-02，見 §1.6）。越障仍是 A5 + A6 疊在一起。**

## B. 決策層：會動搖研究主張，不只是 feasibility

### B1【新發現】所有評估尺寸都選 SWING，滾動從來沒被用到

```text
h = 40 mm    #1 ROLL_ROLL   feasible   body deviation  54.29 mm
             #4 SWING_SWING feasible   body deviation  40.00 mm  <- 勝出
h = 100 mm   #1 ROLL_ROLL   feasible   body deviation 114.86 mm
             #4 SWING_SWING feasible   body deviation 100.00 mm  <- 勝出
```

**ROLL 在兩個尺寸都是可行的**，只是 body deviation 比 SWING 大而輸掉。
於是 Hybrid 的「滾動接觸越障」在**每一個評估尺寸上都沒有被使用**——
越障全部靠 swing，滾動只出現在平地。

這對論文的主張是致命的：如果決策準則永遠選 swing，
那麼在越障這件事上 **Hybrid 和 Walk 沒有差別**。

### B2 決策準則本身對滾動系統性不利

準則是「body deviation 最小」。SWING 的 body deviation **恰好等於障礙高度**
（40.00 / 100.00），ROLL 則多出 14-15 mm。
但 Hybrid 的賣點是**滾動接觸比擺動省能量**，而 body deviation **不是能量**。
用 body deviation 當唯一成本，等於預先判定滾動輸。

**這是一個需要你決定的研究問題**，不是一個 bug。

### B3 ~~`#5 SWING_OVER` 在所有評估高度都是 "not measured"~~【2026-09-02 更正：這條寫錯了】

**原文是錯的，見 §1.7 B3。** 實際上全圖 `feasible 16 / infeasible 152 /
not measured 231`：在 h=40 與 h=100 它是**量過而且不可行**（理由具體），
在 h 20–60 mm × L_top 200–340 mm 的 16 格是**可行的**，
body deviation 0.00 mm，那 16 格**每一格都贏**。
只有 h ≥ 180 mm 真的沒掃過，那併入 A7。**B3 降級為 A7 的一部分。**

## C. 結構性：四隻腳其實從來沒有被組裝成一台機器

### C1 四隻腳用的是**同一條 chain**

Step 3/4 給四隻腳同一條 nominal chain，只用相位區分
（`day12_step3_driver.py` 自己寫著 "The same chain is given to all four legs
on purpose"）。**沒有任何一隻腳被放在它自己的 world x 上生成。**

### C2 Step 2 的 `FourLegState2D` 沒有任何下游步驟消費

Step 2 產出初始四腳狀態、接觸點、對稱檢查、`as_initial_robot_state()`，
但 grep 過整條 pipeline：**只有 driver 拿去產表，Step 3-11 一個都沒用**。
真正被消費的只有 `leg_mounts_2d()`（掛載偏移）。

### C3 那個初始狀態**不是步態能起步的狀態**

```text
Step 2 初始狀態   四隻腳 theta 60、beta 0、alpha 0 —— 完全相同
Step 3 步態需要   LF/RF/LH/RH 相位 0.75 / 0.25 / 0.00 / 0.50 —— 四個不同
```

四隻腳同時處在滾動行程的同一點，這在 Walk 的相位下不可能發生。
**Step 2 與 Step 3 從來沒有被對過帳。**

### C4 body_x 用增量積分，把絕對座標丟掉

Step 5 用站立腳的**增量**積出 `body_x`，理由寫在 docstring 裡：
「絕對值不共用原點」。那是當下唯一安全的作法，
但它也是 **A5（軌跡裡沒有障礙物）的根源**。

### C5【新發現】擺動腳的 hip 在生成時被假設不動，實際上會前進 108.6 mm

```text
RecoveryConfig2D.hip_advance_m 預設 = 0.0
LF swing [0.000, 0.600] s   body 實際前進 108.639 mm
```

recovery 是在「body 不動」的前提下生成並檢查 clearance 與 touchdown 的，
但在四腳排程裡，那隻腳擺動的同時**另外三隻腳正把 body 往前推 108.6 mm**。

Step 8 把接觸點放成相對 hip，所以組出來的軌跡**默默地**讓那條 swing 整個
往前拖了 108.6 mm —— 而它的 clearance 是在原地驗的。
**這條 swing 的地面關係不是生成時檢查過的那一個。**

## D. 從來沒有檢查過的東西

| # | 沒檢查的 | 為什麼要緊 |
|---|---|---|
| D1 | **腿對腿碰撞** | 四隻腳在不同相位，擺動腳會不會打到站立腳？grep 過：Day 12/13 完全沒有這個檢查 |
| D2 | **腿對車體碰撞** | 同上，沒有 |
| D3 | **車體對地形碰撞** | body 在整條 pipeline 裡是一個**點**，沒有幾何 |
| D4 | **動力學 / 接觸力 / 摩擦** | no-slip 是輪緣幾何的性質，不是驗證過的物理性質；規格說 Day 12 只做準靜態，但這一點要一路帶到上機 |
| D5 | **不對稱地形** | 完全沒做過。所以任何「需要 ABAD」的推論目前都沒有證據 |

規格 §16 列的 "other relevant leg-wheel geometry collision" **沒有被實作**，
而 Step 9 也沒有把它列進 `DELEGATED_CHECKS` 或 `UNEVALUABLE_CHECKS`——
它是一個**單純的遺漏**，這是這次排查才發現的。

## E. 沒被質疑過的建模選擇

| # | 選擇 | 現況 |
|---|---|---|
| E1 | `hip_advance_m = 0`（recovery 期間 body 不前進） | 見 C5 —— 與四腳排程矛盾 |
| E2 | duty 0.75 / T = 2.4 s | 是我挑的。rotation-proportional duty 只有 0.221 |
| E3 | margin floor 10 mm | 是我挑的佔位值，不是量出來的 |
| E4 | 滾動時 θ 固定 60° | **已解**（Day 13）——但它示範了「凍結一個自由度」會造成什麼 |
| E5 | 前後兩段 nominal run 各自生成 | **已解**（Day 13） |
| E6 | arc_samples = 241 / roll_step = 4 mm | 沿用 Day 6-7，沒有為四腳重新檢查過 |

**E4 值得單獨記住**：Step 5 的「不可行」看起來像機器人的物理限制，
實際上是 Step 1 的一個建模選擇造成的。**其他被凍結的自由度可能也一樣。**

## F. 量測與報告上的缺陷（會讓人讀錯結論）

| # | 缺陷 | 狀態 |
|---|---|---|
| F1 | Step 8 端點內插 → θ 速率是下界（讀成 0） | **已解**（Day 13 真實幀）：真值 477.57 deg/s |
| F2 | 驗證器把 θ、β 兩個峰值相加估馬達速率 | **已解**：951 被高估成 1429 |
| F3 | `max body lift` 回假的 0 | **已解**：改回 `None` ＋ `usable_body_samples` |
| F4 | Step 11 的 paper metrics 是在**內插軌跡**上算的 | **未更新**：θ 補償之後 body_z 相關的欄位全部過時 |
| F5 | notebook 是在 β / 馬達 / 真實幀三項改動**之前**執行的 | **未更新**：顯示的 Step 9 是 7 過 4 失，現在是 11 過 1 失 |
| F6 | Step 10 的 generalization gate 通過，但四個地形都不可行 | 「通過」講的是**程式的形狀**，不是能跑。表格容易被讀成成功報告 |

## G. 流程與資產狀態

```text
G1  Step 12（freeze + Day 13-14 handoff）從來沒做
G2  notebook 過時（同 F5），136 cells 全部是舊數字
G3  Day 13 的三個改動都是【預設關閉】的 opt-in，
    所以 Day 12 的所有 driver 與 notebook 仍然跑舊行為 —— 這是刻意的
    （為了讓凍結的數字可比），但要記得它們現在【不是】最好的結果
```

## H. 建議的排查順序

```text
第一優先（擋住平地上機）
  A4  support margin 0.000 mm   -- 【結案】見 1.6 A4-1..A4-8
      duty 0.85 -> margin 4.839 mm、馬達 72.0%、全速。
      floor 3 mm（由負責人的重心量測推導）-> Step 9 【12/12 全過】。
  A8  馬達速率量測 ＋ 匯出指令都是階梯 -- 已解，見 1.6 / A4-6
      frame 之間改成內插；兩個獨立量測現在對上（1425.69 deg/s，1.000x）。

第二優先（決定研究主張站不站得住）
  B1/B2  決策準則永遠選 SWING -- 【結案，見 1.7】
         負責人選 B：HYBRID_DECISION_ORDER + HYBRID_BODY_TOLERANCE_M = 15 mm。
         Step 10 重跑：40/100 mm 都改成 ROLL_UP/ROLL_DOWN，tt swings 8 -> 0。
  B3     【更正】原本寫錯，實際上是 A7 的一部分，見 1.7。
  B4     【新發現】滾動越障 margin 是負的（-12 ~ -14.5 mm），
         而且提高 duty 會【更糟】（平地相反）。這是選 B 的新代價，
         要跟 A5/A6 一起解。

第三優先（越障根本還沒真的成立）
  B4     滾動越障 margin 為負 -- 選 B 之後新增，與下列同一批
  【接線缺口】-- 已解，見 1.8。Step 10 現在跑選定組態。
  A5/C5  -- 已排查完，見 1.9。
         C5：可修但在選定組態下【沒有效果】（平地平移不變），維持預設 0.0。
         A5：不是校正問題。四隻腳拿到【完全相同的鏈】，各自在自己的
             時間窗越過「自己的障礙物」-> implied 位置散佈 753.8 mm。
             要修的是【共同世界座標 + 依世界位置排程】，設計層級改動。
  A9     段落被排到 body 涵蓋區間外，delivered 靜靜變成 0（新發現）。
  C4/A5  world-x 註冊；A6 TOP_REPOSITION 的 160 mm；C5 擺動腳 hip 前進 108.6 mm
         —— 這三個是同一件事的三個面，要一起解。
         Step 4 的 2.000x（時間）與附錄 B 的 17.3x（距離）也是同一件事。

第四優先（安全與正確性，不擋 feasible 但擋上機）
  D1/D2/D3  碰撞檢查完全缺席
  C2/C3     Step 2 與 Step 3 對帳
  E1        hip_advance_m 與四腳排程的矛盾

隨時可做（不影響結論，只影響可讀性）
  F4/F5/G2  重跑 metrics 與 notebook
  G1        Step 12

已完成
  A4 / A8   2026-09-02，見 1.6 —— 平地步態完整可行，12/12
```

---

# 1.6 A4 排查結果（2026-09-02）：support margin 0.000 mm

> 依 §1.H 的第一優先處理。結論先講：**A4 不是 bug，也不是缺 ABAD。**
> 它是 `stance_duty = 0.75` 這個值的**定義值**。真正要回答的問題
> 換了一個：`DEFAULT_MARGIN_FLOOR_M = 10 mm` 這個 floor 從哪來，
> 因為這台機器的幾何**在任何 duty、任何速度下都到不了 10 mm**。

新增檔案：

```text
scripts/experiments/day12_support_margin_scan_2d.py     掃描與量測
scripts/experiments/day12_a4_margin_scan_driver.py      driver
tests/test_day12_support_margin_scan_2d.py              26 tests
notes/day12/day12_a4_margin_scan.csv / .png             結果
```

**沒有改動任何既有模組。** Day 12 的所有凍結數字不受影響。

## A4-1 零 margin 的機制

交接瞬間（t = 0.600 s，swing = RH）三個支撐點相對 body 中心：

```text
   LF  (+302.304, +211.675) mm
   RF  (+239.232, -211.675) mm
   LH  (-239.232, +211.675) mm    <- 與 RF 互為相反數
```

RF 與 LH 是**對角**，且座標互為相反數，所以 RF–LH 這條邊**通過 body 中心**，
margin 恰好 = 0。四腳接觸點在該瞬間構成一個以 body 中心為中心的平行四邊形。

這是四足 wave gait 的教科書結果：縱向穩定裕度正比於 `stance_duty − 3/4`，
在 3/4 **恆等於零**。而 `GAIT_LIBRARY["Walk"]` 的 `stance_duty` 正好就是 **0.75**。
Step 6 量到的 0.000 mm 是對的，它量的是臨界 duty。

## A4-2 三個槓桿，掃過之後只剩一個

### 抬腳順序：已經是最好的，這條路走完了

六種相異的四足抬腳順序全掃（固定 LF first，3! = 6 種）：

```text
              sequence          order    d=0.750    d=0.800
          project_walk    LF RH RF LH    -0.0000     2.5778   <- 專案自己的
      front_pair_first    LF RF RH LH   -14.4752   -10.7906
 front_pair_then_cross    LF RF LH RH   -27.5796   -23.2001
diagonal_then_ipsilateral LF RH LH RF   -27.5796   -23.2001
     ipsilateral_first    LF LH RF RH   -25.0684   -21.2136
ipsilateral_then_cross    LF LH RH RF   -14.4752   -10.7906
```

**專案的順序是唯一非負的**，其他五種都是 −5 到 −28 mm（重心在多邊形外）。
這個軸沒有東西可以換。

### 滾動行程：這條路**不存在**，而且這是本次的發現

支撐多邊形只對**接觸點相對自己髖部的位移**有反應。而滾動站立
把髖部前進的大部分「花在把接觸點也往前推」：

```text
                    固定 theta      水平化（實際採用）
   hip advance        297.065 mm      325.916 mm
   contact advance    202.458 mm      202.458 mm
   relative stride     94.607 mm      123.458 mm   <- 支撐多邊形唯一看得到的
   插地腳會多給          3.140x          2.640x
```

> **【2026-09-03 更正】** 上表右欄才是對的。
> `rolling_stride_2d()` 原本預設用**未補償**的姿態，
> 於是這一節引用了「這個步態沒有的行程」。
> **margin 的數字一直都是用水平化姿態量的**（掃描建 plan 時就傳了 levelled），
> 所以只有這個診斷指標描述錯了對象，結論不變：
> 滾動仍然把支撐多邊形的前後行程壓掉 **2.640 倍**。
> 預設已改成 `hybrid_posture_2d()`，並加測試守著兩者不同。

**「那就少滾一點」不成立**：relative stride 與滾動比例是**正比**的
（量到 0.20/0.40/0.60/0.80 → 0.182/0.380/0.608/0.805），
少滾只會讓 margin 更小。

> **這是 Hybrid 步態的結構性代價，而且是可以寫進論文的一句話：**
> 滾動接觸把支撐多邊形的前後行程壓掉 **3.14 倍**，
> 所以 Hybrid 要達到與 Walk 相同的裕度，必須用更高的 duty。
> 這跟 ABAD 無關。

### stance duty：唯一能改變正負號的槓桿

```text
  duty  swing_s   margin mm  motor deg/s    util  min period s   speed  budget
 0.750    0.600     -0.0000       855.41   0.432         2.400   1.000  ok
 0.775    0.540      1.3324       950.46   0.480         2.400   1.000  ok
 0.800    0.480      2.5778      1069.27   0.540         2.400   1.000  ok
 0.825    0.420      3.7444      1222.02   0.617         2.400   1.000  ok
 0.850    0.360      4.8394      1425.69   0.720         2.400   1.000  ok
 0.875    0.300      5.8692      1710.83   0.864         2.400   1.000  ok
 0.900    0.240      6.8396      2138.53   1.080         2.592   0.926  slower
 0.925    0.180      7.7554      2851.38   1.440         3.456   0.694  slower
 0.950    0.120      8.6212      4277.07   2.160         5.184   0.463  slower
 0.970    0.072      9.2805      7128.45   3.600         8.641   0.278  slower
```

「over budget」不是被拒絕，是**要跑慢一點**：frame 間隔隨 cycle period 縮放，
所以 period 是馬達預算上的真槓桿（`min period s` = 該 duty 下馬達允許的最快週期）。

**關鍵結果：到 duty 0.970、擺動窗只剩 72 ms、速度只剩 27.8% 時，margin 只有 9.28 mm。**
10 mm floor 在**任何 duty、任何速度**下都達不到。

## A4-3 「那就把重心往前/後移」——量過了，不可行

在每個取樣瞬間搜尋最佳的前後 body 位移：

```text
   nominal margin                 -0.0000 mm
   best margin any shift gives   109.3682 mm
   the shift it needs           -162.0 .. +164.0 mm
   sign reverses within cycle:   True
```

前後位移確實能買到 **109 mm** 的裕度 —— 但它需要的**位移方向在一個週期內會反號**
（擺前腳時要 −150 mm，擺後腳時要 +150 mm）。所以：

* 任何**固定的**重心修正（改電池位置、配重）**一點都拿不到**；
* 要拿到就得讓車體每個週期前後湧動兩次 ±160 mm，
  而滾動站立中髖部的 x 是 β 的**結果**，不是自由變數。

這一項寫進紀錄，是因為「把重心移一下不就好了」是任何人都會先提的第一個建議。

## A4-4【新發現 A8】馬達速率的量測方式是錯的

排查 A4 時必須用馬達預算當 duty 的上界，一量才發現**原本的量法會隨取樣數變**：

```text
   同一條軌跡（duty 0.750），差分不同密度的網格：
     samples   resampled deg/s    util    frame-to-frame deg/s    util
         241            951.48   0.481                  855.41   0.432
         481           1266.01   0.639                  855.41   0.432
         961           1264.70   0.639                  855.41   0.432
        1921           2528.08   1.277                  855.41   0.432
```

**原因**：`leg_sample_at`（day12_whole_body_trajectory_2d.py:156）用
`int(round(fraction * (len(indices)-1)))` 取**最近的 frame**，所以重取樣出來的
θ/β 是一條**階梯**。對階梯做差分，得到的是「一個 frame step ÷ 一個取樣間隔」——
量的是離散化，不是運動。峰值就是 `RecoveryConfig2D.beta_step_rad = 4°` 那一步。

**正解**（兩件事，見 A4-6）：

1. 量**相鄰生成 frame 之間、在它們被排定的時刻**的速率
   （`frame_motor_rate_2d`）。這個值與取樣網格無關。
2. **而且要把階梯本身修掉**——`leg_sample_at` 改成 frame 之間內插。
   因為同一個根因不只讓量測錯，它讓**匯出的指令本身不可執行**（見 A4-6）。

修好之後 driver 印出的兩欄在每個密度都一致：

```text
     samples   resampled deg/s    util    frame-to-frame deg/s    util
         241            855.41   0.432                  855.41   0.432
         481            855.41   0.432                  855.41   0.432
         961            855.41   0.432                  855.41   0.432
        1921            855.41   0.432                  855.41   0.432
```

**這兩欄如果再分開，就是階梯回來了**（回歸測試守著這件事）。

**這推翻了 Day 13 Step 2 的一個數字。** 匯出的 `day13_motor_command.csv` 標的
「951.48 deg/s = 48.1%」是 241 取樣的產物；真值是 **855.41 deg/s = 43.2%**。
方向上是好消息（比原本以為的更寬鬆），但**理由是錯的**，而且同一個量法在
1921 取樣下會讀出 127.7%（超標）——上機前必須改用 frame-to-frame。

順帶：`walk_timing_2d` 的 docstring 說 cycle period「Nothing below depends on
its value」——**這句話現在是錯的**，馬達速率正比於 1/period。

## A4-5 所以 A4 要你做的決定

不是「怎麼修」，是「floor 定多少、duty 定多少」。三個選項，數字都在上面：

```text
選項            duty   margin      馬達    速度    說明
(a) 現況        0.750   0.000 mm   43.2%   100%   準靜態上正好在翻覆邊界。不建議上機。
(b) 推薦        0.850   4.839 mm   72.0%   100%   全速、留 28% 馬達餘裕，margin 由 0 變正。
(c) 全速極限    0.875   5.869 mm   86.4%   100%   全速能買到的最多，馬達餘裕剩 13.6%。
(d) 犧牲速度    0.970   9.281 mm  100.0%  27.8%   仍然到不了 10 mm。
```

**建議 (b) duty = 0.85。** 理由：margin 從 0 變成 4.84 mm，馬達仍有 28% 餘裕，
速度完全不損失。而 10 mm 這個 floor 的來源，模組自己的 docstring 已經寫明
是「a planning floor, not a measured physical limit」——它從來沒有被量過。
要嘛量出真的 CoM 不確定度來訂 floor，要嘛承認這台機器的上限是 ~5.9 mm。

## A4-6 決定與落實（2026-09-02，專案負責人選 (b)）

**選定 `stance_duty = 0.85`。**

```python
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    HYBRID_STANCE_DUTY,   # = 0.85，常數旁邊就是證據
    hybrid_timing_2d,     # 專案自己的抬腳順序，換成這個 duty
)
```

**`walk_timing_2d` 沒有動，`GAIT_LIBRARY` 也沒有動。** 前者仍然逐字讀專案的
Walk（duty 0.75），Day 12 所有凍結數字仍然可重現；`hybrid_timing_2d` 是
「Hybrid 跑的步態」，兩者**只差 duty**（抬腳順序軸已經掃完，沒有更好的可換）。

### 落實時發現：匯出的 CSV 本身也是階梯

A8 原本被當成「量測方式錯」，但同一個根因也在**輸出**上：
`leg_sample_at` 取最近的 frame，所以 `day13_motor_command.csv` 裡有整段
一模一樣的姿態、中間夾一個 4° 的跳階。以 80.33 Hz 播放，等於**要求機器人
在一個播放週期內走完那 4°**——這不是量測誤差，是**指令本身不可執行**。

修法：frame 之間**內插**（`day12_whole_body_trajectory_2d.leg_sample_at`）。
連續量（θ、β、α、接觸點）內插；`rim` 與 airborne 是**類別量**，取較近的那一幀
（半個 rim 不是 rim）。只動 Day 13 的 `use_generator_frames` 路徑，
預設的端點內插路徑沒動，所以 Day 12 凍結數字不受影響。

> **代價要講清楚**：內插會讓腿經過一些**沒有被逐一碰撞檢查過**的姿態。
> 生成器自己的步長（θ 2°、β 4°）界定了這些姿態離已檢查姿態最遠有多遠。

### 修好之後兩個量測互相對上了

```text
the two rate measurements, which must now agree
  between generator frames    1425.69 deg/s   ( 72.0%)
  across the command's dt     1425.69 deg/s   ( 72.0%)
  the command asks for      1.000x the planned rate
```

修之前這兩個數字**不會**一致（指令端 951.48、計畫端 855.41，比值隨取樣數變）。
現在一致，而且 72.0% 與掃描表 duty 0.85 那一列完全吻合。

### duty 0.85 的實際產出

```text
stance duty          0.850       swing window 360 ms of 2.4 s
liftoff order        LF RH RF LH （與 GAIT_LIBRARY 同序，只換 duty）
support margin       4.8394 mm   （duty 0.75 時是 0.000 mm）
peak motor           1425.69 deg/s = 72.0% of 1980
Step 9               11/12 通過，只剩 support_margin
```

`support_margin` **還是 fail**，但意義變了：不再是「零裕度、正好在翻覆邊界」，
而是「4.84 mm 的正裕度，低於一個構不到的 floor」。
`day13_step2_motor_driver.ACCEPTED` 的註解已改寫成這件事。

## A4-7 重心敏感度（2026-09-02）：floor 該多少，其實是機構問題

A4-6 之後被問到「LegWheel 裡到底有沒有質量參數」。查證結果與量測如下。

### 查證：整個專案沒有質量，只有一個「假設」

```text
legwheel/ 全套件搜 mass / inertia / weight / kg  -> 一個質量數字都沒有
唯一與重心相關者：legwheel/config/__init__.py
    COM_BIAS_X = 0.0                # 假設，不是量測
    COM_BIAS_Y = 0.0                # 假設，不是量測
    COM_BIAS_Z = ABAD_AXIS_OFFSET
參考用的舊 repo corgi.urdf          -> 沒有任何 <mass> 標籤（純視覺模型）
```

Day 12 拿來投影的點是 `com = (body_x_m, 0.0)`
（`day12_support_stability_2d.py:275`）——掛載矩形 (±255, ±211.675) 的
**幾何中心**。這與專案自己的 `COM_BIAS = 0` 一致，Day 12 沒有偏離專案，
只是把同一個假設繼承下來。`COM_BASIS` 字串一直誠實標注「NOT a whole-robot CoM」。

### 量測一：腿的質量幾乎不影響（好消息）

```text
四隻腿的平均重心相對自己髖部的偏移，整個 run 只擺動 7.594 mm
（proxy：腿重心取髖-足連線中點）

  f = 4*m_leg/m_total     worst margin
    0.00（現況假設）        4.8394 mm
    0.20                    4.3721 mm   (-0.467)
    0.50（極端）            3.6711 mm   (-1.168)
```

而且 γ=0 時每隻腳都在自己的矢狀面裡，四個掛載 y **精確抵銷**
→ 這個 2D 模型裡**橫向完全沒有重心偏移**。腿再重也只吃掉 1 mm 出頭。

### 量測二：車體重心偏移才是真正的問題

```text
固定的車體重心偏移 vs worst margin（duty 0.85，標稱 4.8394 mm）
  偏移        x 方向       y 方向     x,y 各偏
   5 mm      1.5075      1.1113      -2.2206
  10 mm     -1.8244     -2.6168      -9.2805
  20 mm     -8.4882    -10.0729     -23.4005

會讓 margin 歸零的前後偏移：7.26 mm
敏感度：每 1 mm 前後偏移吃掉約 0.666 mm 裕度
```

### 所以 A4-5 對 floor 的說法要修正

A4-5 寫「10 mm floor 沒依據、構不到，所以是 floor 要被交代」——
**這個框架不完整。**

10 mm floor 換算過來等於「**容忍 10 mm 的重心誤差**」，對一台真實機器人
是**合理的工程要求**，不是隨便訂的數字。而這台機器在這個步態下只能容忍 **7.26 mm**。

**真正的結論**：floor 不是太保守，是**這個步態的安全餘裕本來就非常薄**，
而且它是**機構問題不是規劃問題**——電池／馬達／電路板配置偏個 1 公分，
duty 0.85 就站不住。duty 0.875 只多給 1 mm，幫不上忙。

### 質量數字：專案沒有，但同一台機器的舊 repo 有（僅參考）

```text
corgi_ros2_ws/.../corgi_hybrid/src/Four_leg.cpp:312-314   ← 僅參考，未引用
    m_body = 19.5 kg
    m_leg  = 0.681 kg
    total  = 19.5 + 4*0.681 = 22.224 kg
    -> 腿佔總質量 f = 12.26%
```

代入 A4-7 的量測一：f = 0.1226 只吃掉約 0.29 mm 裕度，**腿確實可以忽略**。
車體佔 **87.7%**，所以「全機重心」基本上就是「車體重心」。

**但這組數字沒有給出重心的位置**——舊 repo 一樣把 19.5 kg 當成一個
放在車體原點的集中質量。所以缺的仍然是那一個偏移量。

### 每個 duty 能容忍多少重心偏移

```text
  duty  margin mm   motor   speed  CoM tol x mm  CoM tol y mm  tol both mm
 0.750    -0.0000   0.432   1.000          0.00          0.00         0.00
 0.800     2.5778   0.540   1.000          3.86          3.46         1.83
 0.850     4.8394   0.720   1.000          7.26          6.49         3.43
 0.875     5.8692   0.864   1.000          8.82          7.86         4.16
 0.900     6.8396   1.080   0.926         10.29          9.16         4.84
 0.925     7.7554   1.440   0.694         11.68         10.37         5.49
 0.950     8.6212   2.160   0.463         13.00         11.52         6.11
```

「CoM tol」＝車體重心離掛載矩形中心多遠時，最差裕度歸零（取最不利方向）。
**這張表把 floor 的問題整個換掉了**：不必先訂 floor，只要回答
「重心偏多少」，就能直接讀出需要哪一個 duty。

```text
決策樹
  重心偏移 <=  7 mm  ->  duty 0.85 可用（全速）
  重心偏移 7-10 mm   ->  要 duty 0.90，速度降到 92.6%
  重心偏移 10-13 mm  ->  要 duty 0.95，速度只剩 46.3%
  重心偏移 > 13 mm   ->  【任何 duty 都不行】，必須改配重或改步態
```

### 這個預算有多緊

7.26 mm 只佔軸距 510 mm 的 **1.42%**。換成零件的話：

```text
重心位移 = m_part * d / 22.224 kg
   1.0 kg 偏心 100 mm  ->  4.50 mm   （吃掉 62% 的預算）
   2.0 kg 偏心 100 mm  ->  9.00 mm   （已超過）
   4.0 kg 偏心  40 mm  ->  7.20 mm   （幾乎剛好用完）
```

**單一個中等零件稍微偏心就把預算吃光。** 一台沒有特別為此設計對稱性的
機器人，偏移落在 10-30 mm 是常見的——若真是如此，表上**沒有一個 duty 夠**。
（這句是工程預期，不是量測，要以實測為準。）

### 怎麼量：兩台體重計，十分鐘

```text
前後：前兩腳踩一台、後兩腳踩另一台
      x_com = 255 mm * (W_front - W_rear) / W_total
左右：左兩腳、右兩腳
      y_com = 211.675 mm * (W_left - W_right) / W_total

解析度：前後 11.47 mm/kg、左右 9.52 mm/kg
        一般體重計 100 g 解析度 -> 約 1 mm，足夠
```

比找 CAD 快，而且量到的是**含線材、電池、實際裝配**的真實重心。

### 結案只需要一個數字

**車體重心相對四個掛載點矩形中心，前後偏多少？**
（CAD、實機平衡測試、或配重估算皆可）

拿到之後 `floor = 該偏移 × 0.666`，floor 就有依據，
同時解掉 E 類「沒有質量模型」的一半。
**若偏移超過 7 mm，duty 0.85 不夠，要重新談。**

## A4-8 floor 定案（2026-09-02）：平地步態 12/12 通過

**專案負責人提供的事實**：「之前量過，重心就真的是在機器人的中心。」

這是**事實輸入**，跟 330 rpm、β 可連續轉同一類——repo 裡推導不出來
（`legwheel/` 沒有質量、沒有慣量；`COM_BIAS_X/Y = 0.0` 是**假設**不是量測）。
但這句話**沒有附精度**，所以精度是這裡**假設**的，不是裝作沒有。

### 推導鏈（寫成可執行的，不是註解）

```python
COM_UNCERTAINTY_PER_AXIS_M   = 0.002   # 假設：量測殘餘 ±2 mm/軸
MARGIN_LOST_PER_COM_OFFSET   = 1.412   # 實測：兩軸同時偏，每 1 mm 吃 1.412 mm
derived_margin_floor_m()     = 2.824 mm
HYBRID_MARGIN_FLOOR_M        = 0.003   # 向上取整
```

實測的敏感度（0.5–5 mm 範圍內線性到四位數）：

```text
  x 前後單獨    每 1 mm 吃掉 0.6664 mm
  y 左右單獨    每 1 mm 吃掉 0.7456 mm
  兩軸同時      每 1 mm 吃掉 1.4120 mm   <- floor 要用的是最壞的這個
```

**為什麼 ±2 mm**：體重計法在這台 22.2 kg 機器上解析度約 1 mm，
±2 mm 是對「量過，在中心」這句話公允偏保守的讀法。
**若實際量測比這粗，floor 就太小**——重算只是一個乘法，
`derived_margin_floor_m(真實精度)`。

### 結果：Step 9 從 11/12 變成 12/12

```text
Step 9 on the exported trajectory
  checks failed  0   []
  failures       0
accepted_failures                （空的）
```

`day13_step2_motor_driver.ACCEPTED` 從 `(SUPPORT_MARGIN,)` 變成 `()`。
**平地 Hybrid 步態現在是完整可行的**，不帶任何 accepted failure。

### floor 涵蓋什麼、不涵蓋什麼

```text
涵蓋：重心不在量測位置
不涵蓋：動力學、接觸力、摩擦、地形不平、關節誤差
        （Day 12 一個都沒有模型，硬編一個數字會讓這個常數
          看起來比實際上更有根據）

duty 0.85 給 4.8394 mm，扣掉 3 mm floor
-> 留給上面所有「不涵蓋」的東西合計 1.839 mm
```

**這 1.839 mm 很薄，要知道。** 另外：重心兩軸各偏 **3.43 mm** 裕度就歸零。

`DEFAULT_MARGIN_FLOOR_M = 10 mm` **原封不動**——Day 12 所有凍結數字都是對它量的。
回歸測試同時守著兩件事：這個步態**過** `HYBRID_MARGIN_FLOOR_M`、
**不過** `DEFAULT_MARGIN_FLOOR_M`，所以「它是靠 floor 過的」這件事在紀錄上。

### 還開著的（不擋平地上機，但要記得）

* **±2 mm 是假設的量測精度**，不是負責人給的。真值若更粗要重算。
* 投影點仍是 body 中心不是 CoM（`COM_BASIS` 一直這樣標）。
  腿的質量已量過影響很小（f = 12.26% 只吃 0.29 mm），
  但**車體幾何仍然是一個點**，所以 D1/D2/D3 碰撞檢查一項都做不了。
* 那 1.839 mm 的剩餘裕度**沒有任何模型支撐**。

**還沒做、但這裡指出來的**：body 中心 ≠ CoM（`COM_BASIS` 一直這樣標注），
而**整個 Day 12 沒有任何質量模型**。沒有質量模型，10 mm 和 4.84 mm
哪個「夠」其實都無法回答。這件事屬於 E 類（未被質疑的建模選擇）。

---

# 1.7 B1/B2/B3 排查結果（2026-09-02）：決策準則永遠選 SWING

> 依 §1.H 第二優先處理。結論：**B1/B2 成立，而且比原本寫的更嚴重；
> 但 B3 我寫錯了，這裡更正。**

新增／改動：

```text
scripts/experiments/day10_11_decision_map_2d.py   +body_tolerance_m（預設 0.0）
scripts/experiments/day12_b1_decision_criterion_driver.py   證據
tests/test_day10_11_decision_map_2d.py            30 -> 55 tests
notes/day12/day12_b1_decision_criterion.csv / .png
```

**預設值 0.0 完全重現原本的行為**，不是近似——body 項變成
`max(0, body − best)`，是 `body` 的單調平移，排序完全相同。有測試守著。

## B1-1 機制：`roll_preference` 是死碼

`DEFAULT_ORDER = ("feasible", "body", "margin", "roll_preference")`。
`ROLL_PREFERENCE` 這張表**已經存在**且把 ROLL_ROLL 排第一，但它在字典序的
**最後一位**，只有在 `body` 與 `margin` **完全相等**時才會觸發。
而 body deviation 差 14–15 mm，所以它**從來沒有被執行過**。

實際可選的其實只有兩個：`#2` 與 `#3` 是**永久 handoff blocked**
（DIRECT_HANDOFF_INFEASIBLE），所以每一格都只是 `#1` 對 `#4`（偶爾加 `#5`）。

## B2-1 規格自己就有矛盾，而程式默默選了一邊

規格（FINAL 版 1710 行附近）寫 Hybrid 的機制是：

```text
rolling-contact stance
+ compact nominal recovery
+ terrain-transition swing ONLY WHEN REQUIRED     <- 這句 = roll_preference 優先
        ↓
potentially smaller vertical body / CoM variation  <- 這句 = body 優先
```

**這兩句給出相反的準則**，而程式選了 body 優先，沒有標注這是一個選擇。

而且規格同時明確禁止從這些指標推能量／COT
（「Do not infer energy/COT from these metrics.」）。
所以：**Hybrid 的賣點（滾動接觸）用現有的成本函數無法表達**，
而唯一在用的成本 body deviation，剛好是滾動系統性輸的那一個。

## B1-2 量化：換準則會換掉誰贏，但**完全不動哪裡有解**

全圖 399 格（h 20–200 mm × L_top 200–600 mm），可解 217 格：

```text
                  ordering  tol mm   ROLL   OVER  SWING  roll share  worst body cost
body,margin,roll (current)       0     38     16    163       17.5%            0.00 mm
body,margin,roll (current)      15     61     16    140       28.1%           14.86 mm
          body,roll,margin       0     38     16    163       17.5%            0.00 mm
          body,roll,margin      10     39     16    162       18.0%            4.32 mm
          body,roll,margin      15    106     16     95       48.8%           14.86 mm
          body,roll,margin      20    106     10    101       48.8%           20.00 mm
          body,roll,margin      30    109     10     98       50.2%           21.56 mm
```

**所有規則下，可解性改變的格數都是 0。**
這正好回答了 Day 10–11 規格最後一條完成條件（「重排順序會不會移動區域邊界」）：
**不會**，順序只決定誰贏。

### 光加容忍帶不夠——`margin` 會攔截

`DEFAULT_ORDER` 把 `margin` 排在 `roll_preference` 前面，
所以 body 打平之後**是最小間隙在決定**，preference 仍然輪不到。
容忍帶必須搭 `("feasible", "body", "roll_preference", "margin")` 才有意義
（看上表 current 那三列 15/20/30 mm 就是被 margin 攔掉的結果）。

### 15 mm 是個明確的拐點

```text
tol  10 mm -> ROLL 18.0%
tol  15 mm -> ROLL 48.8%   最壞 body 代價 14.86 mm，SWING_OVER 保住 16 格
tol  20 mm -> ROLL 48.8%   沒多拿，但 SWING_OVER 掉到 10 格
```

**15 mm 拿到幾乎全部的滾動採用率（48.8% vs 上限 51.2%），
最壞代價 14.86 mm，而且不吃掉 SWING_OVER。**
20 mm 以上只是開始傷害別的策略。

15 mm 也有物理意義：翻轉格子的 body 代價**中位數就是 14.29 mm**
（純滾動偏好下 84 格翻轉：最小 4.32、中位 14.29、平均 19.26、最大 104.46 mm）。
也就是說 15 mm ≈「接受滾動固有的額外抬升，但不接受 100 mm 的離群值」。

### 評估尺寸上的結果

```text
                  ordering  tol mm         h=40         h=100        h=190
body,margin,roll (current)       0   #4 SWING     #4 SWING          NONE
          body,roll,margin      15   #1 ROLL      #1 ROLL           NONE
```

**換成 body,roll,margin + 15 mm，兩個評估尺寸都改用滾動越障。**
h=190 仍然 NONE，那是 A7 的資料缺口（五個策略在該高度**全部**是 not measured），
不是準則問題。

## B3【更正】我在 §1.5 寫錯了

原文寫「`#5 SWING_OVER` 在所有評估高度都是 not measured」。**這是錯的。**

```text
全圖狀態: {'feasible': 16, 'infeasible': 152, 'not measured': 231}
FEASIBLE 的 16 格: h 20-60 mm, L_top 200-340 mm
```

* 在評估高度 h=40 / h=100 它是 **measured 而且 infeasible**
  （理由具體：「the widest top an over-swing clears at this height is 280 mm」），
  不是沒資料；
* 它在 16 格是**可行的**，body deviation **0.00 mm**，
  所以在那 16 格它**每一格都贏**。

真正成立的只有「h ≥ 180 mm 沒掃過」，那併入 A7。
**B3 降級為 A7 的一部分，不再是獨立問題。**

> 教訓：`not measured` 是三態之一，我把「在我看的那兩個尺寸剛好不可行」
> 讀成了「從來沒被量過」。三態的重點就是不要把這兩件事混在一起，
> 結果我自己在排查裡混了。

## B1-3 這件事對論文的意義

現行準則下，越障**全部**靠 swing，滾動只出現在平地。
那麼「Hybrid vs Walk」在越障這件事上**沒有差別**——
Hybrid 的優勢（若有）就只剩平地的滾動站立。

這不是 bug，是**研究主張要怎麼立**的問題：

```text
選項 A  維持現狀（body 優先，tol 0）
        誠實，但要在論文裡明說「越障時本方法選擇 swing」，
        Hybrid 的貢獻限縮在平地站立方式。
選項 B  body,roll,margin + tol 15 mm  【建議】
        體現規格自己寫的「swing only when required」。
        滾動在 48.8% 的可解格子被採用，兩個評估尺寸都改用滾動。
        代價：最壞多 14.86 mm body deviation，且必須在論文裡
        說明這個 15 mm 容忍帶是一個【設計選擇】。
選項 C  等能量模型再決定
        規格禁止現在推 COT。真正能判定滾動划不划算的成本函數
        還不存在——這才是根本問題，但它擋不住 A/B 二選一。
```

**建議 B**，理由：它是規格自己寫的設計意圖，不動任何可解區域，
代價有界且可量化，而且 tol 是一個具名、可掃描、有證據的參數，
不是藏在排序裡的隱含偏好。

## B-決定與落實（2026-09-02，專案負責人選 B）

```python
# day12_terrain_generalization_2d.py —— Day 12 的單一入口
HYBRID_DECISION_ORDER  = ("feasible", "body", "roll_preference", "margin")
HYBRID_BODY_TOLERANCE_M = 0.015
```

`plan_terrain_2d` 現在**預設**使用這個規則。
**Day 10–11 的預設完全沒動**（`DEFAULT_ORDER`、`DEFAULT_BODY_TOLERANCE_M = 0.0`），
所以 Day 10–11 的凍結證據仍然可重現。

### 落實時修掉一個潛在 bug：兩處會各自重新決策

`plan_terrain_2d` 先呼叫 `decide_2d`，然後呼叫 `compose_2d(h, L, tables)`
——**而 `compose_2d` 內部會自己再決策一次**。兩邊如果用不同規則，
就會「報告一個策略、組出另一個策略」，而且不會有任何東西講出來。
已把 `order` / `body_tolerance_m` 貫穿到 `compose_2d`，兩處保證同一個規則。

### Step 10 重跑的實際差異

```text
                40mm x 400mm              100mm x 400mm
ascent          SWING_UP -> ROLL_UP       SWING_UP -> ROLL_UP
descent         SWING_DOWN -> ROLL_DOWN   SWING_DOWN -> ROLL_DOWN
tt swings       8 -> 0                    8 -> 0
```

**越障不再用 swing 了**，terrain-transition swing 歸零，符合規格
「swing only when required」。

## B4【新發現】滾動越障的支撐裕度是**負的**，而且提高 duty 會更糟

選了 B 之後立刻量到的：

```text
h = 40 mm                     ascent   tt swings   min margin
  SWING（舊準則）＋ duty 0.75   SWING_UP        2       0.000 mm
  ROLL （準則 B）＋ duty 0.75   APPROACH        0     -12.058 mm
  ROLL （準則 B）＋ duty 0.85   APPROACH        0     -14.532 mm

h = 100 mm
  SWING（舊準則）＋ duty 0.75   SWING_UP        2       0.000 mm
  ROLL （準則 B）＋ duty 0.75   APPROACH        0     -12.742 mm
  ROLL （準則 B）＋ duty 0.85   APPROACH        0     -14.110 mm
```

**兩件事要講清楚：**

1. **這不是舊組態的假象。** 我先懷疑是 Step 10 還在用 duty 0.75、
   還沒接上 A4 的水平化姿態，所以用 duty 0.85 再跑一次——
   **margin 反而更差**（−12.058 → −14.532）。
   平地上提高 duty 會改善裕度，**越障時方向相反**。
   所以這是滾動越障本身的性質。

2. **SWING 也沒有比較好，只是剛好在邊界上**（0.000 mm）。
   兩個都 ≤ 0，都不能上機。B 換掉的是「在邊界上」變成「明確在外面 12–14 mm」。

**這是選 B 帶來的、具體的新代價**，必須跟 A5/A6 一起解，
不能寫成「換了準則就好了」。

### 另外注意：A4 的修正沒有接到 Step 10

`plan_terrain_2d` 用的是 `walk_timing_2d()`（duty 0.75）、
預設姿態（**非**水平化滾動）、以及 `DEFAULT_MARGIN_FLOOR_M`（10 mm）。
所以四個地形的第一個失敗都還是 `body_requirement_satisfied`
（A1，Day 13 的 θ 補償已經解掉，但**沒有接進 Step 10**）。
這是**接線缺口**，不是新問題——但它讓 Step 10 的輸出看起來比實際狀況差。

> Step 10 的 `flat` 那一列 failure_reason 也變了
> （`beta_workspace_guard` → `body_requirement_satisfied`），
> **那與準則 B 無關**——舊 CSV 是 Day 13 之前產生的，
> β 連續性修好之後才輪到下一個失敗顯示出來。

**但預設值我只改了 Day 12 入口。** `DEFAULT_BODY_TOLERANCE_M = 0.0`、
`DEFAULT_ORDER` 原樣，所以 Day 10–11 的所有凍結數字都還原封不動。

---

# 1.8 接線 ＋ 兩個「取樣相依」的檢查（2026-09-02/03）

> 準則 B 落實後接著做的：把 A1/A4 的修正接進 Step 10，讓後續量測可信。
> 接線過程量出**一個更嚴重的問題**，並更正我先前的一個結論。

## 1.8-1【更正】「平地 12/12 feasible」只在 samples=241 成立

我在 §1.6 A4-8 報告平地 Step 9 全過。**那個結論只在 241 取樣成立。**

```text
samples  失敗  原因
     61     1  joint_continuity      (70.12 deg > 30 deg)
    121     1  joint_continuity      (35.35 deg > 30 deg)
    181     0
    241     0     <- A4-8 報告的就是這一格
    481     1  stance_contact_valid
    961     2  stance_contact_valid
```

**判定隨網格變，而且不單調。** 而軌跡本身的關節速率在**每一個**密度都是
**1425.69 deg/s**，完全收斂。所以問題不在軌跡，在**檢查**。

## 1.8-2 機制一：`joint_continuity` 拿「每步角度」比固定門檻

```python
step = max(|Δtheta|, |Δbeta|)          # 隨 dt 縮放
if step > MAX_JOINT_STEP_RAD:          # 固定 30 deg
```

步長 61 取樣是 70.12 deg、961 取樣是 4.45 deg——**同一條軌跡**。
模組的 docstring 其實**自己就寫了**這是取樣產物
（「the per-sample step is a modelling artefact of the sample count」），
卻仍然用固定角度當門檻。

**修法**：改成**速率**門檻。

```python
TELEPORT_RATE_RAD_S = 2.0 * MOTOR_MAX_RATE_RAD_S
if step / dt > TELEPORT_RATE_RAD_S: ...
```

低於馬達上限的本來就歸 `motor_rate_limit` 管；這個檢查要問的是
「有沒有任何速率解釋不了的跳躍」，所以取 2 倍上限。
`MAX_JOINT_STEP_RAD` **保留**給**跨段落**的 handoff 檢查——
那是兩個段落之間的真實不連續，與取樣無關，固定角度才是對的形狀。

## 1.8-3 機制二：`stance_contact_valid` —— **這個是我自己弄壞的**

做 frame 內插（§1.6 A4-6）時，我把 `is_segment_boundary_frame` 從
「最近的幀是首幀或末幀」改成了：

```python
is_segment_boundary_frame=(low == 0 and blend == 0.0) or high == len(indices)-1
```

`blend == 0.0` **窄太多**——只有恰好落在第一幀的取樣才算。
於是離地瞬間那一幀（本來就還在接觸，Day 13 §5.4 已經記過）不再被豁免，
**細取樣才會冒出來**。

修法：改成「落在首或末的**幀區間**內」：

```python
is_segment_boundary_frame=(low == 0 or high == len(indices) - 1)
```

兩個邊界都是段落的**固定比例**，與取樣數無關。

修好之後：

```text
samples   61  121  181  241  481  961
失敗       0    0    0    0    0    0
```

**判定現在是軌跡的性質，不是網格的性質。**
`test_day12_whole_body_validation_2d.py` 新增 4 個測試守著這件事。

## 1.8-4 接線：Step 10 現在跑的是選定的組態

`plan_terrain_2d` 原本用 `walk_timing_2d()`（duty 0.75）、預設姿態（非水平化）、
`DEFAULT_MARGIN_FLOOR_M`（10 mm）、端點內插——**A1/A4 的修正一項都沒接進來**。

現在預設是選定組態；`legacy_configuration=True` 可以跑回 Day 12 原版
（凍結數字的重現方式）。

```text
       terrain  feas     ascent    descent  lift mm  margin mm
          flat  True       None       None    0.000      4.839
  40mm x 400mm False    ROLL_UP  ROLL_DOWN    0.000    -18.571
 100mm x 400mm False    ROLL_UP  ROLL_DOWN    0.139    -17.985
 190mm x 400mm False       None       None      n/a        n/a
```

* **平地在 Step 10 裡終於是 `feasible = True`**（之前是 False）。
* 越障 margin **−18.571 mm**，比接線前量到的 −12.058 更差
  （與 B4 一致：duty 0.85 對越障不利）。**這是乾淨組態下的真實起點。**

## 1.8-5【A6 化解】滾動越障真的站在障礙物上了

```text
                       A6 頂面站立   within-leg gap   drift
SWING（舊準則+legacy）    0.0000 s      436.245 mm   276.245 mm
ROLL （準則 B+接線後）    6.4168 s      420.203 mm   186.293 mm
```

**A6 不是被「修好」，是被「化解」**：滾動把接觸點連續帶過頂面，
根本不需要 `TOP_REPOSITION` 那 160 mm。swing 越障才需要，而它從來沒生成。

**但 A5 仍在**：同一隻腳的自我矛盾 436 → 420 mm、drift 276 → 186 mm，
改善了但仍是數百 mm。world-x 註冊還是壞的，那是下一項。

## 1.8-6 八個測試更新（全部是「編碼了舊決策行為」）

準則 B 之後 8 個測試失敗，逐一判斷後全部是斷言寫死了舊行為，
改成斷言**真正的不變量**而不是當時的數值：

```text
test_both_obstacles_use_the_same_primitives_chosen_by_day_10_11
  寫死 SWING_SWING -> 改成對照 decide_2d 自己的答案。
  這個測試的用意是「Step 10 不自己決定」，寫死答案反而讓它在規則改變時
  因為錯誤的理由失敗。
test_the_obstacle_runs_add_transition_swings_over_the_flat_one
  -> 改名 test_the_terrain_does_not_disturb_the_nominal_recovery_count
     滾動越障的 transition swing 本來就是 0，那正是改規則的目的。
test_the_swing_counts_are_reported_separately
  -> 只斷言兩個欄位分開存在，數值歸決策規則管。
test_an_unmeasurable_body_excursion_is_none_not_zero
  平地現在可行、有 61 個可用樣本，原本借用平地當「不可量」案例。
  -> 改成用 dataclasses.replace 建構該案例，並另加一個測試斷言
     可行的平地【確實】會報出 body excursion。
test_an_obstacle_adds_transition_swings_without_changing_the_nominal_ones
  -> 去掉 transition swings > 0，保留真正的不變量。
test_a_swing_swing_crossing_has_no_transition_roll
  -> 改名 test_the_two_crossing_kinds_are_counted_in_different_fields
     「越障由某種東西組成，而且一定被算在某個欄位」。
test_the_hip_lift_grows_with_the_obstacle
  40 mm 是 swing 的抬升（恰等於障礙高）；滾動是 37.80 mm。
  -> 改成「與障礙同量級」而不是釘死等於障礙高。
test_nothing_ever_stands_on_the_obstacle
  -> 拆成兩個：滾動越障【會】站上去；swing 越障【仍然不會】。
     A6 是真的，而且它是 swing 越障的性質，不是被底下偷偷修掉了。
```

結果：terrain_generalization 23、paper_metrics 20、
obstacle_registration 11，**全部通過**。

---

# 1.9 A5/C5 排查結果（2026-09-03）：越障根本沒有共同世界座標

> 第三優先。結論：**C5 可以修但沒有效果；A5 不是小修，是四腳越障模型缺了一塊。**

## C5：量到了、可以修、但在選定組態下沒有效果

`RecoveryConfig2D.hip_advance_m` 預設 0.0，實際擺動窗內 body 前進：

```text
duty 0.75   每個 swing 都是 108.639 mm
duty 0.85   每個 swing 都是  57.515 mm   <- 五個 swing 完全相同
```

**不動點一步收斂**（0 → 57.5145 → 57.5145）：body_x 是由**站立腳**積出來的，
不受擺動腳自己的 hip_advance 影響，所以這其實是一次性代入而不是迭代。

把它設進去之後，recovery **每個值都成功**，clearance 也沒有破：

```text
hip_advance mm   success   min clearance mm   touchdown x mm
        0.0000      True             0.0000          387.645
       57.5145      True             0.0000          445.159
      120.0000      True             0.0000          507.645
```

（min clearance 0.0000 是觸地那一幀，本來就是零。）

**但是**：我原本假設 C5 是 A5 的根源，量下去**假設是錯的**——
把 hip_advance 設成 57.5145 mm，註冊指標**一個小數位都沒變**：

```text
hip_advance mm   within-leg gap mm   drift mm   stance on top s
        0.0000             420.203    186.293           6.4168
       57.5145             420.203    186.293           6.4168
```

原因：平地上 clearance 對 x **平移不變**，而 Step 8 把接觸點放成相對 **body**，
所以整條 swing 平移不改變任何被檢查的量。
**C5 在平地上是無害的**；它會有影響的地方是「擺動腳經過障礙物旁邊」，
而那件事現在根本沒有被檢查（nominal recovery 是對著**平地**生成的）。

## A5：程式裡寫得很清楚，是我之前沒讀到底

```python
implied_x_start_entry = world_hip_start - local_hip_start + frame_x_start
drift = delivered_advance - demanded_advance
```

* `demanded` = 段落**自己的生成座標**要求 hip 走多遠
* `delivered` = 排程在同一個時間窗內**實際**把 hip 移了多遠

40 個越障段落實測（h=40 mm，選定組態）：

```text
 leg             kind    demanded mm  delivered mm    ratio   implied x mm
  LF         APPROACH         62.790        11.099    5.657        486.876
  LF          ROLL_UP         67.590        38.343    1.763        430.301
  LF WHEEL_TRANSITION        225.865       130.508    1.731        390.820
  RF          ROLL_UP         67.590        81.938    0.825        701.638
  RF        ROLL_DOWN         12.519         0.000      n/a        663.186
  LH WHEEL_TRANSITION        225.865        39.670    5.694        476.707
  RH         APPROACH         62.790        17.743    3.539         72.733
  ...
平台實際位置          1000.0 mm
implied 位置散佈       753.801 mm
worst drift            186.195 mm
最近的 implied 位置離規劃地形  251.456 mm
worst advance ratio      5.694
```

### 三個一眼可見的事實

1. **四隻腳的 `demanded` 完全一樣**（62.790 / 67.590 / 19.290 / 225.865 / …）。
   驗證過了：`build_leg_plan_2d` 給四隻腳**完全相同的鏈**，14 段、起點都是 0.000 mm。
   但四隻腳的**時間窗不同**（相位差）。
   → **等於讓四隻腳各自越過「自己的一座障礙物」**，而不是四隻腳輪流越過同一座。
   這就是 implied 位置散佈 753.8 mm 的直接來源。

2. **ratio 從 0.324 到 5.694，不是常數。**
   所以這不是少了一個校正係數，不能靠乘一個數字修好。

3. **RF 與 LH 的 `ROLL_DOWN` 段 `delivered = 0.000`**——
   那些時間窗（3.006–3.291 s、3.568–3.891 s）**排在 body 軌跡的涵蓋區間之外**，
   body 根本沒在動。段落被排到資料範圍外，卻**靜靜地**拿到 0 位移。
   這是一個獨立的具體 bug（**新增 A9**）。

## A5 要修的是什麼

不是校正，是**四腳越障模型缺的那一塊**：

```text
現在：  compose_2d(h, L) -> 一條鏈 -> 四隻腳each拿同一條
        每隻腳在自己的局部座標裡，於自己的時間窗越障

要的：  一座障礙物在世界座標的一個位置
        四隻腳在【不同時間】抵達【同一個】位置
        每隻腳的越障段落必須依它【自己何時走到那裡】來排程
```

這需要：**把腳的世界 x 位置當成排程的輸入**，而不是只用相位。
也就是 C4 說的「`body_x` 用增量積分、把絕對座標丟掉」要先解決——
必須有一個共同的世界原點，讓「腳在哪裡」和「障礙物在哪裡」可以比較。

**這是設計層級的改動，不是 bug fix。** 我沒有動手，因為它會改變
Day 10–11 交出來的 `ComposedSequence2D` 怎麼被使用，
應該先確認方向再做。

## 對 C5 的處置

`hip_advance_m` **維持 0.0 預設沒有改**。理由：
在選定組態下它不改變任何被檢查的量（上面量過），
而把它設成非零會讓 recovery 的觸地點前移 57.5 mm、
改變所有既有數字，卻**換不到任何正確性**。
等 A5 的世界座標解決之後，它才會變成一個有意義的量。

已記錄量測方法與數值，之後要用時直接取。

---

# 1.10 A5 動工前的量測（2026-09-03）：越障的鏈**兩端都沒有接上**

> 專案負責人選了做法 1（共同世界座標）。動工前先把邊界量清楚，
> 結果**做法 1 是必要但不充分的**——這裡是量到的東西與範圍變化。

## 1.10-1 已經確認可行的前提

```text
世界原點【存在】     Step 2 的 body 起點 x = 145.0 mm，平台前緣 x = 1000.0 mm
                     （只是從來沒有被下游使用 —— 這就是 C2/C4）
部分行程【可用】     run_foot_rim_roll_2d(max_distance_m=...) 已經實作
                     要 50.0 得 50.614、要 101.2 得 101.229（顆粒 = roll_step 4 mm）
每腳所需【算得出】   用步態相位定起始接觸點：
   leg   contact x mm   到平台 mm   完整 stroke   餘量 mm
    LF        338.27       661.73           3      54.36
    RF        410.89       589.11           2     184.19
    LH        -62.80      1062.80           5      50.51
    RH       -135.42      1135.42           5     123.13
```

四隻腳需要**不同的** `cycles_before`（3/2/5/5）加上**不同的**部分接近距離。
`cycles_before` 是現成參數，部分行程也是現成能力——**這一半沒有阻礙**。

## 1.10-2 但是：鏈在越障的**兩端**都是斷的

`build_leg_plan_2d` 產生的 LF 計畫（`cycles_before=3`）逐段端點：

```text
 idx               kind             theta0   theta1      beta0      beta1   hip0 mm   hip1 mm
   5     RECOVERY_SWING              72.49    72.49     -759.8    -1040.2     977.7     977.7
   6           APPROACH              40.00    40.00        0.0      -19.7     -36.0      26.8
  ...
  15          ROLL_DOWN              17.00    17.00     -276.3     -285.3     599.5     622.1
  16      FOOT_RIM_ROLL              72.49    72.49       39.8      -39.8       0.0     325.9
```

兩個邊界，各三個理由，共 6 個 break：

```text
進入越障 RECOVERY_SWING -> APPROACH
   hip_jump      1014.4 mm   (limit 10 mm)
   contact_jump  1075.5 mm
   beta_jump     1040.2 deg
   theta_jump     -32.5 deg   72.49 -> 40.00

離開越障 ROLL_DOWN -> FOOT_RIM_ROLL
   hip_jump       626.7 mm
   contact_jump   559.6 mm
   beta_jump      325.1 deg
   theta_jump     +55.5 deg   17.00 -> 72.49
```

**`cycles_before` 改成 1/2/3/5 都一樣是 6 個 break**——
這不是累積誤差，是**座標系與姿態各自獨立**。

## 1.10-3 拆開來看，三件事只有一件是記帳問題

| | 內容 | 性質 |
| --- | --- | --- |
| **hip x** | 越障段落用自己的原點（`COMPOSER_FRAME_X_START_M`），與 nominal 鏈無關 | **記帳**，rebase 就好 |
| **beta** | −1040.2 -> 0。取模 360 之後是 **+39.8 deg 的真實方位差**（nominal 週期回到 +39.8，APPROACH 要 0） | **一部分記帳、一部分真動作** |
| **theta** | 72.49 -> 40.00（進入）、17.00 -> 72.49（離開） | **真實姿態變化，沒有任何段落做這件事** |

## 1.10-4 所以做法 1 的範圍比原本說的大

原本描述：「給 body_x 一個世界原點，依腳的世界位置排程越障」。
量完之後實際需要：

```text
(a) 世界重定基準   把越障段落的座標搬到「這隻腳實際在哪」  <- 原本說的那一半
(b) 兩段【新的過渡動作】
      nominal 姿態 (theta 72.49, beta +39.8) -> 越障入口 (theta 40.00, beta 0)
      越障出口 (theta 17.00, beta -285.3)     -> nominal 姿態 (theta 72.49, beta +39.8)
```

**(b) 是新的 primitive，不是修 bug**——性質上跟 `RECOVERY_SWING` 同級：
要生成、要檢查 clearance、要確認關節速率、要進 SegmentKind。

## 1.10-5 這同時解釋了為什麼越障從來沒 feasible 過

Day 13 把 `segment_chaining` 修到 0.000 mm，那是**平地**
（`continuous_nominal`，只在沒有越障時啟用，這一點當時就寫在程式註解裡）。
**越障的鏈從來沒有被接起來過**，而 §1.5 的 A2 只記了平地那一半。
A5「軌跡裡沒有障礙物」和「鏈沒接上」是同一件事的兩個症狀。

## 1.10-6 沒有動手，等確認

(a) 我可以直接做。(b) 是新 primitive，會動到 Day 10-11 交出的
`ComposedSequence2D` 怎麼被銜接，而且要決定那兩段過渡是
「Day 12 自己生成」還是「回頭要求 Day 8-9/10-11 補上」。
**方向要先確認**，所以停在這裡。

---

# 1.11 A5 實作進度（2026-09-03）：過渡 primitive 做好了，出口卡在 0.25 mm

> 做法 1（(a) 世界重定基準 ＋ (b) 兩段過渡，由 Day 12 生成）。
> 這一節是**進行中**的紀錄：primitive 已驗證可用，鏈的拼接還沒做。

## 已完成並驗證

### (1) 把 recovery 生成器參數化，不是複製

`run_recovery_swing_2d` 新增三個**可選**覆寫，預設 = 原行為：

```python
beta_target_rad=None       # 預設 recovery_beta_target_2d(stroke)
theta_touchdown_rad=None   # 預設由水平化約束解出
hip_z_touchdown_m=None     # 預設 posture 的站立高度
```

**刻意不寫第二個 swing 生成器**：那三個 ramp 的 clearance 規則
（哪一段該檢查、哪一段不該）已經錯過一次（見 §1 Step 1 紀錄），
複製一份只會用新的方式再錯一次。

### (2) `run_posture_transition_2d` —— 缺的那個 primitive

`day12_world_registration_2d.py`。就是上面那個函式，把兩個隱含目標講明。

**入口方向已驗證可用**：

```text
從 nominal stroke 末端  theta 72.49  beta  -39.84  hip_z 219.45 mm
到越障入口              theta 40.00  beta    0.00  hip_z 183.63 mm

hip_z 目標 mm   success  frames
       預設      False     124   TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT
     219.45      False     124   TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT
     183.63       True     125   -> 落點 theta 40.00, beta -360.00 (mod360 = 0.00),
                                     hip_z 183.63, contact z 0.0000, rim=foot_rim
```

**恰好落在越障入口姿態上。** 注意前兩列的失敗是**對的**：
強迫 theta 卻不給對應的髖高，腿根本沒踩到地——生成器拒絕是正確行為。

### (3) `forward_beta_for_orientation` —— beta 是圈數計數器

```text
current -1040.2 -> 想要方位   0.0  =>  target -1080.0  (轉 -39.8 deg)
current  -285.3 -> 想要方位  39.8  =>  target  -320.2  (轉 -34.9 deg)
```

與手算一致。**只用整圈數搬移，剩下的角度差是真動作**，
放進 offset 等於宣稱輪子在它其實不在的地方。

### (4) 每隻腳的世界接近距離

```text
 leg  phase  start contact x   到平台      完整 stroke   餘量
  LF  0.850       338.27 mm   661.73 mm        3      54.36 mm
  RF  0.350       410.89 mm   589.11 mm        2     184.19 mm
  LH  0.100       -62.80 mm  1062.80 mm        5      50.51 mm
  RH  0.600      -135.42 mm  1135.42 mm        5     123.13 mm
```

用**步態相位**定起始接觸點，不是用 Step 2 的初始狀態
（後者四腳接觸點都與髖同 x，那不是 walk 步態起得了步的狀態 —— C3）。

## 出口方向的兩個問題，都解了

### 問題一【更正】不是 0.25 mm 的容差落差，是 0.144 **微米**

我第一次報告「Day 10-11 與 Day 12 之間有 0.25 mm 的容差落差」。
**那是我測試網格的粗細，不是量到的值。**（我只試了 0、0.25、0.5…）

直接從穿透深度算：

```text
越障出口姿態的 clearance = -0.0001 mm   (負 = 穿透)
-> 需要抬升 0.144 um，抬升後 contact z = 0.000000 mm，rim = left_rim
```

**0.144 微米**是半公尺幾何上的浮點邊緣，不是建模歧異。
原本「在 Day 12 這側吸收上游誤差」的說法**過重了**。

實作：`standing_pose_penetration_m` 量穿透，`standing_stroke_at_2d` 抬升並
**回傳抬升量**，而且預設上限 = `collision_tolerance_m`，超過就 **raise**：
把上限設成 1 nm 時 guard 確實會叫（驗過）。
所以它是「已知且有界的容差吸收」，不是「悄悄把問題蓋掉」。

### 問題二 出口的腿【抬不起來】——這是真的物理，不是容差

抬升修好之後，出口過渡仍然在第 2 幀就 `ROTATION_CLEARANCE_LOST`。
原因是對的：

```text
越障出口   theta 17.00 deg（已經是 compact），hip_z 143.80 mm
輪外徑 145 mm -> 輪子本來就貼在地上
```

**nominal recovery 的離地間隙來自「縮腿」**——compact 姿態下腿是一個
145 mm 的輪子、髖在 219 mm 高。**離開越障的腿已經是縮的、而且髖很低**，
縮無可縮，旋轉的第一幀就失去間隙。

所以間隙必須改由**車體**提供：加第四段 ramp `RECOVERY_LIFT`，
先把髖從 143.80 抬到 219.45 mm，再在抬起的高度旋轉。

`lift_hip_before_rotation=False` **預設關閉**（nominal 不需要，
而且所有凍結數字都是沒有它量的）。

### 兩段過渡現在都通過

```text
入口  125 frames  -> theta 40.00, beta 方位 0.00, hip_z 183.63, rim foot_rim
出口   60 frames  -> theta 72.49, beta 方位 39.84, hip_z 219.45, rim foot_rim
```

**出口恰好落在 nominal cycle 的起始姿態上**，接得回去。

## 拼裝：接近段（`ApproachRun2D` / `run_approach_2d`）

把每隻腳從起點滾到障礙物前緣。整圈用 `run_nominal_cycles_2d`
（**不另寫一份** beta/hip_x 的接續規則），餘量用 `max_distance_m` 的部分行程。

### 錯了兩次，兩次都是量出來才發現

**第一次：拿 stroke 的接觸前進去數圈數。**

```text
 leg  cycles   最終接觸 x    誤差
  LF       3     1370.85   +370.85 mm
  RF       2     1248.31   +248.31 mm
  LH       5     1617.40   +617.40 mm
  RH       5     1620.70   +620.70 mm
```

每圈超前 370.85/3 = **123.6 mm**，正好是 relative stride 123.458 mm。
原因：一個**完整 cycle** 的接觸前進 = **325.916 mm**，
不是 stroke 的 202.458 mm ——**recovery 的觸地會把腳再往前放**。
（325.916 也正好等於 stroke 的髖部前進，因為一圈之後相對姿態回到原點。）

已加 `cycle_contact_advance_m()`，直接量一圈的接觸前進，不再用 stroke 的。

**第二次：餘量可能大到一個 stroke 關不掉。**

餘量最大接近一個 cycle（325.916 mm），但關餘量只能用一個 rolling stroke
（最多 202.458 mm）。中間 **123.458 mm 是構不到的帶**，RF 就落在裡面
（餘量 263.19，只關掉 202.46，差 60.73 mm）。

而且部分行程落在 roll step 上，要求剛好餘量會**超過**最多一個 step，
**超過就收不回來**（`hip_advance_m` 不許為負，車體不倒退）。

**解法**：部分行程**故意少要一個 step**，剩下的交給入口過渡的
`hip_advance_m`（它會 1:1 移動觸地點）。四隻腳的殘量因此全部為正：

```text
 leg  cycles   部分要求   部分實得   最終接觸 x   殘量 mm
  LF       2      5.90      8.44      998.54      1.46
  RF       1    198.46    202.46      939.27     60.73
  LH       3     81.05     84.36      999.31      0.69
  RH       3    153.67    156.06      998.39      1.61
```

**殘量必為正**是設計出來的，不是碰巧：過渡能把髖往前帶，沒有東西能往回帶。

### 第三次錯：殘量量錯了對象（量接觸，該量髖部）

把殘量交給入口過渡的 `hip_advance_m` 之後，四隻腳仍然落在
−56 到 +62 mm：

```text
 leg   殘量 mm  落點接觸 x    誤差
  LF     1.46      943.84   -56.16
  RF    60.73     1061.73   +61.73
  LH     0.69      990.68    -9.32
  RH     1.61     1028.97   +28.97
```

原因：**過渡途中 `theta` 從 72.49 變成 40.00**，
所以落地時接觸點在髖部底下的位置，和離地時**不一樣**（差到 58 mm，而且每腳不同）。
而 `hip_advance_m` 控制的是**髖部**，所以帳要算在髖部上：

```text
落地接觸 = 離地髖部 + hip_advance + 落地時的接觸偏移
```

改成用髖部之後有兩隻腳精準命中，但另外兩隻需要**負的** hip advance
（−1.00、−27.36 mm）——車體不能倒退，被 clamp 成 0 就偏掉了。

**根因**：`leg_approaches_2d` 一直在瞄準**接觸點**。改成瞄準**髖部**
（起點就是 `body_x + mount_x`，乾淨且無相位偏移；
部分行程用實測的 hip:contact = 1.610 換算）。

### 端對端結果：四隻腳全部精準落在平台前緣

```text
 leg   hip advance   success  frames   落點接觸 x    誤差 mm    theta
  LF       130.34      True      124     1000.00      0.00     40.00
  RF        57.71      True      124     1000.00      0.00     40.00
  LH        16.40      True       41     1000.00      0.00     40.00
  RH        89.02      True       41     1000.00      0.00     40.00
```

**誤差 0.00 mm，而且落在越障的入口姿態（theta 40.00）上。**
接近段這一半完成了。

## 重定基準 ＋ 三個接縫：全部歸零

`rebase_sequence_2d` 把整條越障搬到腿自己的世界座標與 beta 計數器上。
搬的只有 **x 與 beta**：`theta` 是關節角不是座標，接觸點的 **z** 也不動
（障礙物在它該在的地方，把越障往旁邊搬不該連著往上搬）。

### 第四次錯：重定基準時又轉了一圈

第一版的 `crossing_rebase_2d` 用 `forward_beta_for_orientation` 算 offset，
接縫因此**正好差 360.000000 度**。

原因：`forward_beta_for_orientation` **刻意不回傳自己的輸入**
（「轉到這個方位」必須是一次旋轉）。但重定基準時，
**過渡已經把腿轉到那個方位了**，這裡要的只是「把兩個計數器對齊」，
是純記帳，直接相減就好。已改，並把這個區別寫在 docstring 裡。

### 結果（LF，h = 40 mm）

```text
入口過渡落點   hip_x 1000.000  beta -720.00  theta 40.00  contact_x 1000.000
重定基準       dx 1036.018 mm  dbeta -720.00 deg (-2.000 圈)
重定後入口     hip_x 1000.000  beta -720.00  contact_x 1000.000
重定後出口     hip_x 1658.131  beta -1005.30  contact_x 1657.385  theta 17.00

接縫 1（入口過渡 -> 越障）
   hip_x 0.000000 mm   beta 0.000000 deg   theta 0.000000 deg   contact 0.000000 mm
接縫 2（越障 -> 出口過渡）
   hip_x 0.000000 mm   beta 0.000000 deg   hip_z 0.000144 mm（容差吸收，見上）
接縫 3（出口過渡 -> nominal after）
   theta 0.000000 deg  方位 0.000000 deg   hip_z 0.000000 mm
```

**三個接縫全部歸零。** 對照 §1.10 動工前量到的：

```text
              動工前          現在
進入越障   hip 1014.4 mm     0.000000 mm
           beta 1040.2 deg   0.000000 deg
           theta -32.5 deg   0.000000 deg
離開越障   hip  626.7 mm     0.000000 mm
           beta  325.1 deg   0.000000 deg
           theta +55.5 deg   0.000000 deg
```

**越障的鏈第一次接起來了。**

## 四隻腳的完整鏈：`build_world_leg_chain_2d`

先把 `cycle_segments_2d` 的兩半**抽出來**成公開函式
（`roll_segment_2d` / `swing_segment_2d`），讓部分行程與兩段過渡
共用同一份 `BodyRequirement2D` / `RollingContact2D` 建構，
而不是出現第三份副本。抽出後既有測試 **71 passed**，行為未變。

`swing_segment_2d` 的 `kind` / `phase_label` 是參數：
**同一個動作**服務 nominal recovery 與兩段越障過渡，
差別只在鏈裡叫它什麼，不在它怎麼生成。

### 結果：四隻腳全部成立

```text
 leg   seg  frame  cycles  advance     落點接觸 x    誤差 mm   breaks
  LF    17    585       1   130.34      1000.000     0.0000        1
  RF    17    585       1    57.71      1000.000     0.0000        1
  LH    21    841       3    16.40      1000.000     0.0000        1
  RH    21    841       3    89.02      1000.000     0.0000        1
```

* **四隻腳都精準落在平台前緣 1000.000 mm，誤差 0.0000 mm**
* **break 從 6 個降到 1 個**（再修一個之後歸零，見下）
* 四隻腳的 `cycles` 與 `advance` **各不相同**（1/1/3/3、130/58/16/89 mm）
  ——這正是重點：它們在不同時間抵達**同一座**障礙物，
  而不再是各自越過自己的一座

### 第五次錯：越障之後的 nominal run 沒有接續 beta 計數器

剩下那 1 個 break 是 `beta_step_overrun`，值 6*pi（LF/RF）與 8*pi（LH/RH）。

**先走錯一次**：我以為是 `posture.scene()` 把 beta 折疊了，量了之後
**不是**——要求 −1005.30 就回 −1005.30，一度不差。

真正原因：我的診斷輸出**差一格**（印 `kinds[i-1] -> kinds[i]`，
應該是 `kinds[i] -> kinds[i+1]`）。真正的邊界是
**出口過渡 -> 越障後的 nominal run**。

`run_nominal_cycles_2d` 沒有 `start_beta_rad`，beta 一律從弧起點重來，
所以計數器**歸零重數**，接縫正好是整數圈：
越障前滾 1 圈的腿差 6*pi，滾 3 圈的差 8*pi。

已加 `start_beta_rad` 參數（`None` = 原行為，從弧起點開始）並在組裝時傳入。

> 教訓：**diagnostic 印錯索引，會把你送去查一個不存在的 bug。**
> 我因此花時間去驗證「scene 是不是折疊 beta」——那個假設本身是憑空的，
> 而且驗證它的成本比檢查自己的索引高得多。

### 最終結果：四隻腳，越障，0 個 break

```text
 leg   seg  frame  cycles  advance     落點接觸 x    誤差 mm   breaks
  LF    17    585       1   130.34      1000.000     0.0000        0
  RF    17    585       1    57.71      1000.000     0.0000        0
  LH    21    841       3    16.40      1000.000     0.0000        0
  RH    21    841       3    89.02      1000.000     0.0000        0
```

對照動工前（§1.10）：**6 個 break、hip 差 1014 mm、beta 差 1040 度、
theta 差 32.5 度**。

## 還沒做

```text
接進 build_leg_plan_2d / plan_terrain_2d 正式 API
四隻腳一起排程，重量 registration（目前 spread 753.8 mm）
補測試
效能：目前四隻腳一次 10-15 分鐘（見下）
```

## 效能注意

每隻腳要生成數個完整 cycle，四隻腳一次約 **10-15 分鐘**。
滾動行程是**平移不變**的，原則上可以生成一次再平移，
而不是每腳每圈重新生成——還沒做，但接進正式 API 之前應該做，
否則 `plan_terrain_2d` 會慢到不能用。

---

# 1.12 效能排查（2026-09-03）：接觸查詢慢了兩個數量級

> A5 的四腳建構要 15m50s，擋住接進正式 API。這一節是排查與修法。
> **前兩次優化都猜錯了地方**，第三次才打中。

## 1.12-1 前兩次：正確、有測試、但沒打中

### (a) nominal cycle 複製

量到每個 cycle 是 cycle 0 的**精確平移**：

```text
cycle 1 vs 0:  dx=325.9158 mm  dbeta=-360.0000 deg
   殘差  hip_x 0.0000 um  hip_z 0.0000 um  beta 0.0000 udeg  theta 0.0000 udeg
cycle 2 vs 0:  dx=651.8315 mm  dbeta=-720.0000 deg   （同上，全零）
```

於是 `run_nominal_cycles_2d` 改成生成第一圈、其餘平移
（`translate_cycle_2d`，`final_scene` 重建而非沿用，免得有欄位描述未平移的位置）。
3 cycles **73.0 s -> 30.6 s**。加 4 個等價測試（含 `hold_hip_z` 兩種姿態），
拿**實際生成**的第 2、3 圈逐幀比對。

### (b) 快取重複的探測行程

profile 顯示一隻腳 181 s 裡 **170 s 在 `run_foot_rim_roll_2d`，被叫 5 次**，
其中 2 次只是為了讀一個數字（contact lead、nominal 起始姿態）。
加了 `nominal_stroke_2d`（`lru_cache`，只快取無參數那一條）。

**結果：15m50s -> 13m04s，只有 17%。** 兩個優化都是對的，但都不是瓶頸。

## 1.12-2 profile 說瓶頸在哪

```text
query_point_to_terrain_surfaces_2d   1,334,571 calls   99 s
query_contact                              617 calls  103 s
build_single_leg_rolling_scene_2d        4,209 calls   76 s
plot_leg.get_shape / _update_geometry   12,627 calls   50 s
```

**地形只有 1 個表面**（平地），而一次 `query_contact` 對它做 **2163 次點查詢**，
每次 **71 µs**。模型算出 151 ms，實測 167 ms —— **92% 的成本就這一項**。

逐項拆解：

```text
完整 query                    71.02 us
  _query_surface 單獨          40.74 us
    dataclass 建構+驗證        27.06 us   <- 最大單項
    _point_xz 一次              3.98 us   (post_init 呼叫 3 次)
    np.isclose(norm(v),1)       7.29 us
    np.clip 純量                 3.80 us
  math.hypot 對照              0.13 us
```

**真正的數學是 0.13 µs，其餘全是驗證與小陣列開銷**——
用 numpy 對兩個純量做運算，每個 2 元素陣列配置約 1 µs。

## 1.12-3 修法：介面不動，只把運算換成 `math`

改 `legwheel/planners/hybrid/terrain_query_2d.py`。**回傳型別完全不變**：

```text
np.all(np.isfinite(point))         -> math.isfinite(x) and math.isfinite(z)
np.isclose(np.linalg.norm(n),1.0)  -> abs(hypot(nx,nz)-1) > atol+rtol（同判準，常數具名）
np.isfinite(scalar) x3             -> math.isfinite
np.clip(純量, lo, hi)               -> 條件式
np.linalg.norm(a-b)                -> math.sqrt(dx*dx+dz*dz)
每次重建 np.array([0,1]) 等法向量    -> 模組層常數，建一次
```

**每一個驗證都保留**，只是算得便宜。

### 一個刻意的取捨：不要更準，要逐位元相同

原本用 `math.hypot`，golden 測試抓到 `euclidean_distance_m` **差一個 ULP**
（hypot 的演算法比較準）。改回 `sqrt(dx*dx+dz*dz)` 去**精確符合 numpy 原本的捨入**。

理由：**「數學沒有變」如果能是逐位元相同的斷言，就比「更準但不一樣」有價值**——
至今所有從這個函式凍結出來的數字都是用原本的捨入算的。
取捨寫在原始碼註解裡。

## 1.12-4 怎麼證明沒改壞：先產生 golden，再動手

**動手之前**先跑舊實作產生對照資料：

```text
3 個地形（平地 / 40 mm 平台 / 沉降地面上的 100 mm 平台）
x 516 點 x 2 個 span 容差  =  9288 筆
涵蓋 ground / obstacle_front / obstacle_back / obstacle_top 四種表面
以及 inside(470) / occluded(312) / out-of-span(5390) 三個分支
```

`tests/test_terrain_query_2d_equivalence.py`（7 passed）：

* 每一筆的**每一個欄位逐位元相同**（`==`，不是 `approx`）
* 一個測試先斷言 **golden 檔本身涵蓋了所有分支**——
  漏掉分支的回歸檔是假的綠燈
* 5 個測試守住驗證仍然會叫：非有限值、形狀錯、負容差、非單位法向量、list 輸入

`-k "terrain or contact"`：**183 passed, 2 failed**，
那 2 個是先前就在失敗的措辭不符（見 §2 的 8 個既有失敗），與本次無關。

## 1.12-5 結果

```text
                          四腳建構      query/point（平地）
起點                      15m50s        71.02 us
+ cycle 複製 & 探測快取    13m04s        71.02 us
+ terrain query 去 numpy    7m43s        25.32 us   (2.8x)
```

**四腳建構 2.05x**，而且結果完全沒變（1000.000 mm、誤差 0.0000、0 break）。

剩下的 25.32 µs 幾乎都是**介面要求的 ndarray 配置**
（`_point_xz` 每個結果建 3 個、`nearest` 再 1 個）。
要再往下必須改回傳型別，會影響所有呼叫端——**停在這裡**。

> **教訓**：兩次猜測、兩次都對但都沒用，直到跑 profile 才找到真的那一個。
> **先量再改**，即使「這裡看起來很慢」的直覺很強。

---

# 1.13 A5 接進正式 API（2026-09-03）：鏈接好了，但**只做了一半**

> `world_leg_plan_2d` / `world_leg_plans_2d` 產出標準的 `LegPlan2D`，
> `plan_terrain_2d` 新增 `world_registered=True`（`False` 重現舊行為）。

## 做到的

```text
                    world_registered=False    =True
四腳 chain breaks             24                 0     <- 鏈接起來了
頂面站立時間               6.4168 s          7.5644 s
越障段落數                     40                48     <- 多了 8 段過渡（每腳 2 段）
```

**24 個 break 降到 0**：四隻腳的越障鏈各自接上了自己的 nominal run，
而且四隻腳的 `cycles`/`advance` 各不相同（1/1/3/3、130/58/16/89 mm）——
它們在不同時間抵達同一座障礙物。

## 沒做到的：註冊指標幾乎沒動，而且有一項變差

```text
                    world_registered=False    =True
implied 位置散佈        748.684 mm        747.150 mm   <- 幾乎沒變
worst within-leg        420.203 mm        747.150 mm   <- 變差
worst drift             186.293 mm        225.865 mm   <- 變差
離規劃地形最近          253.017 mm        845.243 mm   <- 明顯變差
is_registrable             False             False
Step 9 失敗檢查   body_requirement / segment_chaining / support_margin（兩者相同）
```

## 為什麼：我把段落註冊到世界，但**沒有把 body 軌跡註冊到世界**

```text
body_x 範圍        0.00 ..   1220.74 mm      <- 從 0 開始積分
Step 2 世界原點    body x = 145.00 mm，平台 x = 1000.00 mm
LF 段落 hip_x      276.54 / 1159.21 / 1984.05 mm   <- 已在世界座標
```

**兩個座標系。** 段落現在說「髖在 1159 mm」，body 軌跡說「車體在 0..1221 mm」，
而 Step 8 把接觸點放成 `body_x + mount_x + offset`——用的是後者。

這正是 **C4**（`body_x` 用增量積分、把絕對原點丟掉），
我在 §1.9 就寫過它是前提，但實際動手時只做了段落那一半。

`registration_report_2d` 的公式
`implied = world_hip − local_hip + frame_x_start` 在重定基準後也失去原意：
`local_hip` 已經是世界值，卻仍加上 `COMPOSER_FRAME_X_START_M`。
**這個報表需要跟著更新，否則它量的不是它宣稱的東西。**

## 誠實的結論

**A5 的「鏈接不起來」那一半解決了（24 → 0 break），
「軌跡裡沒有障礙物」那一半沒有。**

剩下要做的是 C4：

```text
1. body_trajectory_2d 接受一個世界起點（Step 2 已經有：145.0 mm），
   不再從 0 積分
2. registration_report_2d 的公式跟著更新（重定基準後 frame_x_start 不該再加）
3. 重量
```

這是**下一步**，不是可以跳過的收尾。我沒有把它算成完成。

---

# 1.14 C4（2026-09-03）：body 軌跡世界註冊，量出真正的阻礙

## 做了什麼

`body_trajectory_2d` 新增 `world_registered`。原本 `body_x` 從站立腳的
**增量**積分，理由寫在 docstring 裡：

> Step 4 gives every leg the same chain, whose ``hip_x`` starts at zero,
> so the absolute values do not share an origin.

**世界註冊之後這個前提不成立**，所以改成直接讀站立腳的絕對位置
`hip_x − mount_x`，取中位數；**並把四隻腳彼此的分歧量出來**
（`BodyTrajectory2D.world_x_spread_m`），而不是靠中位數把它平均掉。

## 量出來的：四隻腳對 body_x 分歧 **602.610 mm**

```text
                    world_registered=False    =True
四腳 chain breaks             24                 0
四腳對 body_x 的分歧          n/a          602.610 mm   <- 新量測
body_x 範圍              0..805.55       219.80..1491.00 mm
implied 位置散佈        748.684 mm        822.096 mm
Step 9 失敗檢查     body_req/chaining/margin   ＋ body_continuity（新增）
```

**這個量測是刻意加的，而它立刻證明了註冊仍然不成立。**
以前這個分歧被增量積分藏起來（增量都對，絕對位置各自為政）；
現在它是一個數字。

## 為什麼：排程假設四隻腳的鏈**結構相同**

```text
 leg   段數        排程窗        t=0 時 hip_x
  LF    17    -2.04 - 7.56 s      276.54 mm
  RF    17    -0.84 - 8.76 s      349.16 mm
  LH    21    -0.24 -14.16 s     -124.52 mm
  RH    21    -1.44 -12.96 s     -197.15 mm

涵蓋區間 -0.240 .. 7.560 s
```

世界註冊讓每隻腳有**不同的接近段**（LF/RF 各 1 圈、LH/RH 各 3 圈），
所以段數是 **17/17/21/21**。而 `plan_four_legs_2d` 把每條鏈
**按段落索引**攤在同一個週期時間軸上——段數不同，時間就對不齊：
LH 的鏈被拉到 14.16 s，LF 只到 7.56 s。

於是在任何一個瞬間，四隻腳處在自己鏈上完全不同的位置，
世界座標當然對不上。

## 誠實的結論：這比我原本估的深

**問題不在 C4 本身**（C4 兩行就改完了，而且它做對了一件事：
把隱藏的不一致變成可量的數字）。

**問題在 `day12_timing_skeleton_2d` 假設四隻腳的鏈結構相同**——
同樣的段數、同樣的節奏，只差一個相位。世界註冊打破了這個假設，
因為不同的腳到障礙物的距離不同。

要修的是**排程本身**：時間要由「這隻腳走到哪裡」決定，
而不是由「這是第幾段」決定。這是 Step 3 的重寫，不是接線。

```text
現在：段落索引 -> 時間窗（相位平移），假設四腳結構相同
要的：世界位置 -> 時間（四腳共用一條 body_x(t)），
      每隻腳的段落依它自己在那條軌跡上何時到達來排
```

## 目前的狀態（不誇大）

```text
A5「鏈接不起來」    已解    24 -> 0 break
A5「軌跡裡沒障礙物」 未解    四腳對 body_x 分歧 602.610 mm
C4「body_x 沒原點」  已解    但解開之後露出上面那個
【新】排程假設四腳結構相同                     <- 真正的阻礙
```

`world_registered` 預設為 `True`，但**越障目前仍然 `feasible=False`**，
而且多了一個 `body_continuity` 失敗（body_x 在中位數換腳時跳動）。
平地不受影響（`composed.sequence is None` 時走舊路徑）。

**要不要把它預設回 `False`，是下一個要決定的事**：
現在的 `True` 給出更誠實的量測，但也給出更差的 Step 9 結果。

---

# 1.15 位置排程（2026-09-03）：四腳終於對同一個車體讀同一個時鐘

> §1.14 的結論是「排程假設四腳鏈結構相同」。這一節換掉那個假設：
> **時間由位置決定**，不由段落索引決定。

## 先驗證算術自洽

`world_schedule_2d` 的規則只有一條：
**一個段落持續的時間 = 車體把它的髖從起點帶到終點所需的時間。**

車速由步態自己定義：一個 cycle 的髖部前進 ÷ 週期。

```text
一個 cycle 的髖部前進  383.431 mm = stroke 325.916 + recovery 57.515
週期 2.4 s  ->  車速 159.763 mm/s

段落時間 = 髖部前進 / 車速：
   stroke   2.040 s   對照 stance_duration 2.040 s   相符
   recovery 0.360 s   對照 swing_duration  0.360 s   相符
```

**位置排程精確重現 duty 0.85 的步態**，一毫秒不差。

### 這同時把 C5 從「可修但沒效果」變成「必修」

`RecoveryConfig2D.hip_advance_m` 預設 0（§1.9 量過，在舊排程下改了沒有效果）。
**在位置排程下，hip_advance = 0 的 recovery 佔零時間**——髖不動，車體就不用時間帶它。

正確值不是自由參數，是讓 cycle 加得起來的那一個：

```text
stroke_advance + swing_advance = cycle_period * body_speed
=> swing_advance = stroke_advance * (1 - duty) / duty
   duty 0.85 -> 57.5145 mm   （§1.9 實測 57.515，相符）
```

寫成 `swing_hip_advance_m(timing, posture)`。

## 結果：分歧 602.610 mm -> **0.0000 mm**

```text
                          舊排程        位置排程
四腳對 body_x 的分歧    602.610 mm    0.0000 mm
body_x 範圍           219.8..1491.0  130.5..1844.1 mm
各腳鏈長度 (s)        7.6/8.8/14.2/13.0   11.4/11.0/13.9/14.4
```

**四隻腳現在對車體在哪裡完全一致。** 這是 A5 一路追下來要的那個性質。

## 但步態不變量壞了，原因也已經量出來

```text
一次只有一隻腳離地: False
每個 swing 都有三腳支撐: False

每隻腳第一段的起始：
   LF  hip_x  276.54  beta 39.84 deg
   RF  hip_x  349.16  beta 39.84 deg
   LH  hip_x -124.52  beta 39.84 deg
   RH  hip_x -197.15  beta 39.84 deg
```

**四隻腳的 beta 起始值完全相同（39.84 deg = 弧的起點）。**

`run_approach_2d` 讓每隻腳都從一個**完整 stroke 的開頭**起步
（`hip_x0 = start_x - contact_lead`），所以**腳在 cycle 內的相位被丟掉了**——
四隻腳只差整數個 stroke，不差相位。於是它們的 swing 同時發生。

`leg_start_contact_x_2d` 明明用相位算了起始接觸點，
但 `run_approach_2d` 隨後把它對齊到 stroke 邊界，等於把那個資訊丟了。

## 下一步（明確）

**每隻腳的第一個 stroke 必須是部分的**，從它自己的相位開始，
而不是從弧的起點開始。`run_foot_rim_roll_2d(start_beta_rad=...)` 已經支援，
只是 `run_approach_2d` 沒有用它。

```text
現在：leg i 的鏈 = [完整 stroke] x N + [部分 stroke] + 過渡 + 越障 + ...
要的：leg i 的鏈 = [從相位起的部分 stroke] + [完整] x N + [部分] + 過渡 + ...
```

## 狀態

```text
A5「鏈接不起來」            已解   24 -> 0 break
C4「body_x 沒原點」          已解
【1.14】排程假設四腳結構相同  已解   位置排程，分歧 0.0000 mm
【新】四腳鏈丟了 cycle 內相位            <- 下一個，且已定位
A5「軌跡裡沒障礙物」          仍未解  等相位修好再重量
```

---

# 1.16 補相位（2026-09-03）：步態回來了，但越障會把相位吃掉

> §1.15 的結論是「四腳鏈丟了 cycle 內相位」。這一節補上，
> 並量出補上之後露出的下一件事。

## 改了什麼

### (1) `run_approach_2d` 改成「先量再走」，不再預測

前兩版都先算 `whole_strokes` 再照著建，兩次都算錯（§1.11）。
改成貪婪走：**還放得下一個完整 cycle 就再走一個**，剩下的用部分行程關。
原本必須事先算對的算術，變成一個停止條件。

### (2) `phase_start_beta_rad`：每隻腳從自己的相位起步

```text
                 舊（全部從弧起點）    新（依相位）
   LF beta            39.84 deg         -38.18 deg
   RF beta            39.84 deg           7.03 deg
   LH beta            39.84 deg          30.47 deg
   RH beta            39.84 deg         -16.41 deg
```

**邊界情況**：LF 的 phase 0.850 ÷ duty 0.85 = **恰好 1.0**，
它正好在 stroke 終點，第一段 stroke 長度為零，schema 拒絕
（`beta_step_rad must be finite and non-zero`，拒得對）。
夾到「至少留一個 roll step」——它是「即將離地」而不是「已經離地」。

### (3) 部分起步的 recovery 要瞄準**弧的起點**

`recovery_beta_target_2d` 瞄準 `stroke.start.beta − 2π`。
對中途起步的 stroke，那是**弧的中間**——而 recovery 之後要開始的是
一個**完整** stroke，起點在弧的開頭。

在水平化姿態下，落在弧中間根本不是有效的地面接觸：
四隻腳全部 `TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`。
改成明確傳入 `beta_target_rad = 弧起點 − 2π`。

### (4) 沒有部分行程時，過渡要從 **recovery 的落點**起步

我原本用「最後一個 cycle 的 **stroke**」當過渡起點，
於是過渡從 942.24 開始，而該腳的 recovery 已經走到 999.75——
時間窗重疊，排程直接拒絕（`LH has overlapping segments`，拒得對）。
沒有部分行程時改用 `standing_stroke_at_2d` 從 recovery 落點起。

## 結果：步態回來了

```text
各腳 swing 區間
   LF: [0.05,0.41]  [2.45,2.81]  [3.74,3.78]  [7.90,8.26]  [10.30,10.66]
   RF: [1.21,1.57]  [3.61,3.78]  [7.90,8.26]  [10.30,10.66]
   LH: [1.81,2.17]  [4.21,4.57]  [6.61,6.97]  [6.97,6.97]  [11.09,11.45]
   RH: [0.61,0.97]  [3.01,3.37]  [5.41,5.77]  [6.95,6.97]  [11.09,11.45]
```

**nominal 的 swing 完美交錯**：0.05 / 0.61 / 1.21 / 1.81 / 2.45 / 3.01 …
**恰好每 0.6 s 一隻，就是四分之一週期。** 這正是 walk 步態。

而且**四腳對 body_x 的分歧仍是 0.0000 mm**。

## 但越障會把相位吃掉

```text
取樣 400 點中，同時多於一隻腳離地: 7.0%
最多同時離地: 2
```

重疊全部出現在**越障及其之後**：

* LF 與 RF 越障後的 swing **完全重合**（7.90-8.26、10.30-10.66）
* LH 與 RH 同樣（11.09-11.45）

**原因是結構性的**：越障是一段**固定的髖部距離**（942 → 1658 mm）。
位置排程下，走完固定距離就是固定時間，所以**每隻腳的越障耗時相同**；
而同一側前/後腳的 `mount_x` 相同，它們越過同一座障礙物、
走同一段距離，於是**出口對齊在同一個位置，也就是同一個時刻**。

相位差在越障前是靠「腳在 cycle 內的位置不同」維持的，
越障把每隻腳都帶到同一個姿態（入口 theta 40.00、出口 theta 17.00），
**相位資訊在越障中被歸零了**。

## 這是真問題，不是 bug

要維持步態，越障之後必須**重新建立相位差**——
例如讓不同的腳在越障後走不同長度的第一個 stroke。
這不是排程能解的，是**越障本身要吐出不同的出口相位**。

**沒有動手**：這是繼「排程依位置」之後的下一個設計決定，
而且它會影響 Day 10-11 交出來的越障序列要不要參數化。

## 狀態

```text
A5「鏈接不起來」              已解
C4「body_x 沒原點」            已解
排程假設四腳結構相同           已解（位置排程，分歧 0.0000 mm）
四腳鏈丟了 cycle 內相位        已解（nominal swing 每 0.6 s 交錯）
【新】越障把相位吃掉，7% 的時間兩腳同時離地   <- 下一個
A5「軌跡裡沒障礙物」            仍未重量（等相位問題解完才有意義）
```

---

# 1.17 與 Walk 基準的交界（2026-09-03）

> 專案負責人同時在修 **Walk 步態**（平地＋越障），用來跟 Hybrid 比較。
> 這一節記錄兩邊的交界在哪、以及一個 Walk 那邊會撞到的結論。

## 交界只有一個：`GAIT_LIBRARY["Walk"]`

查證過的（跑 import 圖，不是憑印象）：

```text
legwheel/planners/obstacle_walk/handoff.py      Day 12 完全沒有用到
legwheel/planners/obstacle_walk/traversal.py    Day 12 完全沒有用到
legwheel/planners/obstacle_walk/export.py       Day 12 完全沒有用到
legwheel/planners/trajectory_planning_3d.py     只被【間接】載入，沒有直接 import
```

`trajectory_planning_3d` 是這樣被拉進來的：

```text
day12_timing_skeleton_2d
  -> from legwheel.planners.gait_generator_3d import GAIT_LIBRARY
       -> gait_generator_3d 內部 import trajectory_planning_3d
```

**Day 12 從那整個子樹拿的只有兩個數字**：

```python
GAIT_LIBRARY["Walk"]["stance_duty"]     # 0.75
GAIT_LIBRARY["Walk"]["phase_offsets"]   # [0.75, 0.25, 0.5, 0.0]
```

已加測試 `test_the_only_shared_input_with_the_walk_planner_is_pinned`
把這個相依講明確：**它是與 Walk planner 的契約，不是 sanity check**。
如果 Walk 的 duty 被調動，測試會失敗，並在訊息裡說明要重讀 §1.6 的哪一部分。

## 反向也成立

`test_flat_walk_baseline_regression` 與 `test_lateral_stance_symmetry` 的失敗
**不是 Day 12 造成的**：把我唯一改過的 `legwheel` 檔案
（`terrain_query_2d.py`，§1.12）還原成 git 版本之後，兩者**仍然失敗**，
而且 flat walk 的產生器不 import 那個檔案。
它們指向 `trajectory_planning_3d.py`（09-02 23:00 改），
而 fixture 是 08-31 的。

> 順帶量到：`test_flat_walk_baseline_regression` 的數值
> **在全套裡是 5.662700e-02、單獨跑是 5.650300e-02**——
> 隨執行順序改變。某處有順序相依的全域狀態
> （最可能是 `plot_leg` 的共用 leg model，見 §1.12 profile）。
> 既有問題，但值得知道。

## 給 Walk 基準的兩個結論（Day 12 已經量到的）

### 1. 臨界 duty 是 wave gait 的性質，Walk 也一樣

§1.6 A4-1 的結論——**四足 wave gait 的縱向裕度正比於 `duty − 3/4`，
在 3/4 恆等於零**——**不是滾動造成的**，是步態幾何造成的。

所以 **Walk 在 duty 0.75 的支撐裕度也是 0.000 mm**。
Walk 基準若要 feasible，同樣得把 duty 提高。

### 2. Walk 的行程優勢是 2.640 倍（已量）

支撐多邊形只對「接觸點相對自己髖部的位移」有反應：

```text
              相對行程        說明
Walk         325.916 mm     腳插在地上，髖部走完全程
Hybrid       123.458 mm     腳往前滾 202.458 mm，只剩差額
比值           2.640 x
```

**在同一個 duty 下，Walk 的裕度是 Hybrid 的 2.640 倍。**
這是 Hybrid 一個真實、已量化的劣勢，該寫進論文。

### 3. 比較必須在同一個 duty 與同一個週期下做

Day 12 選了 `HYBRID_STANCE_DUTY = 0.85`（§1.6 A4-6），
因為 0.75 給 0.000 mm。**如果 Walk 跑 0.75 而 Hybrid 跑 0.85，
比較就被 duty 混淆了**——Walk 會同時吃到「裕度較差」與「可能較快」。

建議：兩邊用**同一個 duty**。若用 0.85，依上面的 2.640 倍，
Walk 的裕度會是 Hybrid 的 2.640 倍，那個差距本身就是結果。

> **【2026-09-03 專案負責人確認】Walk 基準也跑 duty 0.85。**
> 所以比較是**受控的**：兩邊同 duty、同週期，
> 差異只剩「滾動 vs 插地」這一個變因。
> 那 2.640 倍的裕度差因此是**可歸因的結果**，不是混淆。

### 4. 由此得到一個可檢驗的預測

Hybrid 在 duty 0.85 量到 **4.839 mm**。若 2.640 倍的行程模型成立，
**Walk 在同一個 duty 應該落在 12.8 mm 附近**——而那會**超過**
`DEFAULT_MARGIN_FLOOR_M = 10 mm`：

> **同一個 floor 下，Walk 過得了，Hybrid 過不了。**

這是 §1.6 A4-5「10 mm floor 這台機器構不到」的另一面：
**構不到是滾動造成的，不是機器造成的。**

這個預測**可以直接用負責人的 Walk 實作驗證**，不必只靠推導。
驗到了，就是論文裡一個很強的機制說明；
驗不到，就表示 §1.6 的 stride 模型漏了東西——兩種結果都有價值。

### 5. 但兩邊的 0.85 目前不是共用常數

`GAIT_LIBRARY["Walk"]["stance_duty"]` 仍然是 **0.75**。
Hybrid 用 `HYBRID_STANCE_DUTY = 0.85`（day12_support_margin_scan_2d），
Walk 那邊的 0.85 是負責人在自己的實作裡設的。

**「兩邊 duty 一致」目前靠約定維持，不是靠共用來源。**
要拿它當受控比較，就該讓兩邊讀同一個常數，
否則哪天一邊改了，另一邊不會知道，而上面第 4 點的預測會靜靜地失效。

---

# 1.18 全套回歸與兩個測試更新（2026-09-03）

## 回歸結果

```text
1129 passed, 11 failed  (58:30，前次 1:27:38 —— §1.12 的 terrain query 效果)
```

11 個失敗拆開：

```text
 8  既有（TUI 5、lateral_stance 1、toroidal 2）
 1  flat_walk_baseline  -> 非 Day 12 造成，見 §1.17
 2  Day 12 的【真實行為改變】，測試待更新   <- 這一節
```

## 那 2 個測試斷言了一個世界註冊之後不再成立的「不變量」

```text
                nominal recovery   tt swings   transition roll
   flat                8              0           0.0000 s
   40mm               18              0           2.3410 s
```

兩個測試都斷言 `obstacle.nominal_recovery == flat.nominal_recovery`。
**18 != 8，而且 18 是對的**：世界註冊讓每隻腳先滾到障礙物才越障
（LF/RF 各 1 圈、LH/RH 各 3 圈，§1.11），跑得更遠自然更多 cycle。

### 同一個測試已經因為同一種原因錯第二次

```text
第一次（§1.7）  斷言「越障會增加 terrain-transition swing」
               -> 滾動偏好讓它變成 0
第二次（本節）  斷言「越障不改變 nominal recovery 數」
               -> 世界註冊讓它從 8 變成 18
```

**兩次都是斷言了一個「設計改動本來就要移動」的數字。**
所以這次改成斷言**方向與理由**，不是數目：

```python
assert run.nominal_recovery_swings > flat.nominal_recovery_swings, (
    "a leg that has to reach the obstacle rolls more cycles to get "
    "there than one walking on the flat")
```

以及真正的不變量——兩個計數**分割**所有 swing：

```python
assert total_swing == nominal_recovery + terrain_transition   # 越障與平地都成立
assert flat.terrain_transition_swings == 0
assert flat.transition_roll_time_s == approx(0.0)             # 平地兩種都沒有
```

> 這是陷阱 70 的第二個實例。第一次記的時候我寫「斷言不變量，不要斷言當時的數值」——
> 然後在**同一個測試**上又犯了一次。
> **判斷一個斷言是不是不變量的方法：問「哪一種設計改動會讓它變？」
> 如果答得出來，它就不是不變量。**

---

---

# 1.19 把 §1.15 真的接進管線，然後才量得到相位的代價（2026-09-04）

> §1.16 說「越障把相位吃掉」，我本來要直接去修相位。
> 先去看管線現在到底跑什麼——結果 §1.15 量到的東西，**一行也沒有接進去**。

## 1.19-1 先發現的：位置排程從來沒有被呼叫過

```bash
grep -rn "swing_hip_advance_m\|world_schedule_2d\|body_speed_m_s" --include=*.py .
# 除了 day12_world_registration_2d.py 自己，沒有任何一個地方
grep -rn "..." tests/     # 也沒有測試
```

`plan_terrain_2d` 走的仍然是 `schedule_chains_2d`（**依段落索引**），
而且 `world_leg_plans_2d` 拿到的 `config=None`，
也就是 `RecoveryConfig2D.hip_advance_m = 0`。

**§1.15 是在一個 scratch 腳本裡量的，量完沒有接回去。**
§1.13 標題寫的「API wiring, half-done」指的就是這件事，只是當時沒寫清楚是哪一半。

### 沒接的後果，用管線自己量（40 mm，`plan_terrain_2d`）

```text
每個 recovery swing 的長度          0.0000 s     <- 髖不動，車體就不用時間帶它
                                                    位置排程下 duty 實際上是 1.0
車速                             135.798 mm/s   <- 少了 recovery 的 57.515 mm
                                                    §1.15 量的是 159.763 mm/s
RF                                 整隻腳沒有段落
   refusal: world registration failed -- the approach overshot:
            it would need a hip advance of -0.553 mm,
            and the body does not reverse
```

**四隻腳只剩三隻，而且它安靜地放在 `LegPlan2D.refusal` 裡。**
（不是例外、不是縮短的鏈——這一點是設計對的；但沒有人去讀它。）

## 1.19-2 修法一：`plan_four_legs_2d` 收一個現成的 schedule

不能讓 `day12_transition_mapping_2d` 去 import `day12_world_registration_2d`
（後者已經 import 前者，會循環）。所以由**知道答案的那一端**傳進來：

```python
def plan_four_legs_2d(plans, timing=None, *, schedule=None):
    ...
    if schedule is None:
        schedule = schedule_chains_2d(...)      # 原本的行為，一字不動
```

`plan_terrain_2d` 裡：

```python
if world_registered and composed.sequence is not None and terrain is not None:
    config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    plans = world_leg_plans_2d(composed, terrain, timing, posture, config, cycles_after=1)
    schedule = world_schedule_2d(plans, timing, posture, config)
else:
    plans = {...}          # 平地
    schedule = None        # 平地維持索引排程
```

**平地為什麼可以維持索引排程**：索引排程的假設是「四腳鏈結構相同」，
平地上這個假設是**真的**。§1.14 說它錯，說的是世界註冊之後的越障。
而且所有凍結過的平地數字都是用它量的。

## 1.19-3 修法二：approach 的部分行程改成「量」，不是「換算」

RF 那 0.553 mm 有三個來源疊在一起：

```text
1  max_distance_m 限制的是**接觸**距離，要停短的卻是**髖**
2  1.610 是整條 stroke 的平均比值，弧上每一段的即時比值會漂
3  stroke 停在「第一個到達或超過請求」的那一步，本來就會多走最多一步
```

原本的 `- posture.roll_step_m` 是想一次蓋掉這三件事。**它差 0.553 mm 沒蓋住。**

改成量：做出來、看髖有沒有超過目標，超過就把請求縮一個 roll step 再做一次。

```python
while request > 0.0:
    attempt = run_foot_rim_roll_2d(..., max_distance_m=request)
    partial = attempt
    if not attempt.success or not attempt.frames:
        break
    if float(attempt.end.hip_xz_m[0]) <= target_hip:
        break
    partial = None
    request -= float(posture.roll_step_m)
```

順手補上一個一直存在、但這個越障剛好看不出來的洞：
`LegApproach2D` 現在帶 `landing_contact_offset_m`，
`run_approach_2d` 瞄的是 `obstacle_x - offset` 而不是 `obstacle_x`。
**這個越障的 offset 剛好是 0.00 mm，所以錯了也不會有人發現。**

## 1.19-4 接上之後，管線自己量到的（40 mm，`plan_terrain_2d`）

```text
                              接上前          接上後
四腳都成立                    3 / 4          4 / 4
body 對 body_x 的分歧         602.610 mm     3.331e-13 mm   (3.33e-16 m)
車速                          135.798 mm/s   159.763 mm/s
recovery swing 長度           0.0000 s       0.3600 s
```

**`world_x_spread = 3.33e-16 m`**：四隻腳對車體在哪裡完全一致，
這是 §1.15 承諾、但一直沒有真的出現在管線裡的那個性質。

Step 9 的檢查同時也誠實了起來（6 項失敗，全部是真的）：

```text
at_most_one_airborne        airborne: LF, RF
three_support_legs          got LH, RH
body_requirement_satisfied  110 instants 有兩個互相矛盾的硬需求
segment_chaining            contact teleports between segment 7 and 8
motor_rate_limit            phi_r exceeds 330 rpm
support_margin              CoM 投影沒有夠深地留在支撐三角形內
```

## 1.19-5 接上之後，相位被吃掉的代價才量得出來

21 個離地窗口，7 個重疊，總重疊 **1.4918 s / 13.8495 s**：

```text
     3.7450 ..    3.7778 (0.0329 s)  RF 入口過渡 x LF 入口過渡
     6.9512 ..    6.9685 (0.0174 s)  LH nominal recovery x RH 入口過渡
     6.9685 ..    6.9701 (0.0015 s)  RH 入口過渡 x LH 入口過渡
     7.8973 ..    8.2573 (0.3600 s)  LF 出口過渡 x RF 出口過渡      <- 完全重合
    10.2973 ..   10.6573 (0.3600 s)  LF nominal recovery x RF        <- 完全重合
    11.0895 ..   11.4495 (0.3600 s)  LH 出口過渡 x RH 出口過渡      <- 完全重合
    13.4895 ..   13.8495 (0.3600 s)  LH nominal recovery x RH        <- 完全重合
```

**三種重疊，成因不同：**

* **入口過渡**只重疊 0.0329 / 0.0174 / 0.0015 s。
  因為入口過渡的髖前進是**每隻腳自己的餘量**（RF 0.1662 s、LF 0.0329 s），長度不同——
  但它們**同時結束**，所以短的那個整段被長的蓋住。
* **出口過渡**完全重合 0.3600 s：長度相同、起點相同。
* **越障後的 nominal recovery** 完全重合 0.3600 s：出口對齊之後，後面每一步都對齊。

交錯間隔本身也看得見被吃掉：

```text
越障前   d = 0.5662 / 0.5969 / 0.5969 / 0.6400 s     （目標 0.6000，誤差 ±34 ms）
越障後   d = 2.4000 / 0.0000 / 0.7922 / 0.0000 s     （相位沒了）
```

## 1.19-6 為什麼同一對腳一定同時越障：這是幾何，不是排程

```text
hip_x = mount_x + body_x            四隻腳的髖都掛在同一個車體上
同一對腳 mount_x 相同               -> 它們的髖永遠在同一個 x
障礙物在固定的世界 x，
入口/出口姿態是固定的               -> 入口觸地與出口離地都發生在固定的 body_x
位置排程：時間 = 位置 / 定速        -> 固定的 body_x 就是固定的時刻
```

**定速車體 ＋ 剛性越障序列 ⇒ 同 `mount_x` 的兩隻腳必然同時越障。**
沒有任何排程選擇能拆開它們。能拆開的只有兩件事：

1. **車體在越障期間停下來**（或慢到同一個 body_x 對應一段時間）——
   時間不再由位置決定，位置排程要在越障視窗裡讓位。
   這是真實四足爬台階時做的事。
2. **越障序列在平地上可以前後伸縮**——每隻腳踩上障礙物的位置不同，
   於是入口觸地落在不同的 body_x。
   這要 Day 10-11 的越障序列吐出一個「落點」參數，
   或在越障前後補一段 theta=40 / theta=17 的平地滾動。

兩件都不是這一層能做的決定，所以**沒有動手**，只留下量到的數字。

## 1.19-7 能解的那一半：越障之後的相位

出口過渡本來一律瞄準**弧的起點**，所以每隻腳都以同一個姿態離開越障。
改成瞄準**它自己的相位**：

```python
resume_theta, resume_beta, resume_hip_z = phase_start_pose_2d(resume_phase, timing, posture)
```

`phase_start_beta_rad` 只有 beta 是不夠的：水平姿態在每個 beta 都用 hold_hip_z 反解 theta，
所以弧中間的 theta 和弧起點的 theta 不一樣（實測跨相位差 > 1 度）。
拿弧起點的 theta 配相位的 beta，要的是一個**不在 stroke 上**的姿勢。

接著第一個 stroke 必須是部分的。`run_nominal_cycles_2d` 做不到——
它做一個 cycle 再平移，第一個部分了後面全部都會部分。
所以有 `run_resumed_cycles_2d`：第一個 stroke 從相位起，
它後面那個 recovery 要被明講落在哪裡（陷阱：`recovery_beta_target_2d`
瞄的是自己 stroke 的起點，對部分 stroke 而言是弧中間，根本不是地面接觸）。

### 相位怎麼選：只有一個自由常數

```text
腳 i 離開越障的時刻    t_exit,i = K - mount_i / speed     （K 四腳共用：髖位置相同）
它的第一個 swing        t_exit,i + (duty - p_i) * T
要求                    t_i 依步態相位排成四分之一週期一組
=>  duty - p_i  ≡  c' + mount_i / cycle_hip - phase_i     (mod 1)
```

`c'` 是唯一的自由度。`p_i` 必須落在 `[0, duty]`——
**stroke 只能縮短、不能加長**，所以 `(duty, 1)` 這條 0.15 寬的帶子是不可達的。
`resume_phases_2d` 把這條帶子放進四個需求之間**最寬的那個間隙**（實測 0.4199 > 0.15）。

### phase 前面的符號不是慣例，是可以量錯的東西

平地排程實測：

```text
LF phase 0.75 -> swing 0.0486 s
RH phase 0.50 -> swing 0.6147 s
RF phase 0.25 -> swing 1.2116 s
LH phase 0.00 -> swing 1.8085 s
```

**相位越大，swing 越早。** 我第一版寫 `+ phase_at`，
四隻腳照樣差四分之一週期、照樣「看起來對」，但**走的是相反的順序**。
測試 `test_the_gait_order_survives_the_crossing` 就是為了這個：
它比的是**環狀順序**（誰接在誰後面），不是誰第一個。

## 1.19-8 結果（`plan_terrain_2d`，40 mm，管線自己量的）

```text
                                  只接排程      ＋相位重建
四腳都成立                        4 / 4          4 / 4
body world_x_spread               3.33e-16 m     2.22e-16 m
離地窗口                          21             21
重疊窗口                          7              5
重疊總時間                        1.4918 s       0.7718 s
schedule conflicts                4              3
```

**越障後的 nominal recovery 不再重合：**

```text
只接排程     LF 10.2973   RF 10.2973    <- 同一個瞬間
             LH 13.4895   RH 13.4895
＋相位重建   LF  8.7890   RF  9.9275
             RH 11.7701   LH 12.9086
```

**而且順序就是平地那個順序。** 越障後四個 swing 對 2.4 s 取模：

```text
RF 0.3275 -> LH 0.9086 -> LF 1.5890 -> RH 2.1701 -> (RF)
平地       LF -> RH -> RF -> LH -> (LF)
```

兩個是同一個環：`LF->RH`、`RH->RF`、`RF->LH`、`LH->LF`。

間隔（目標 0.6000 s）：

```text
0.5811 / 0.6804 / 0.5811 / 0.5574     最大偏差 +80 ms
```

偏差的來源已經知道：`(duty - p) * T` 這個線性模型假設
「髖前進與 beta 成正比」，而水平姿態的 theta 隨 beta 變，滾動半徑就跟著變。
單腳實測第一個 stroke：

```text
要求 82.46 / 274.19 / 51.74 / 243.44 mm
實得 84.94 / 266.83 / 51.21 / 233.12 mm     最大差 10.3 mm = 65 ms
```

**不影響結論**：swing 長 0.360 s，最小間隔 0.5574 s，還差 0.197 s 才會碰到。
要收掉它就不能用換算，要用搜尋（陷阱 85 的同一句話）。

## 1.19-9 平地一個位元都沒動

`plan_terrain_2d(None, tables)` 的輸出在這一整輪之前之後**逐位元相同**
（`diff` 無差異）：8 個 swing、每 0.6000 s 一個、零重疊。
接排程、修溢出、重建相位，三件事都只走 `world_registered` 那一支。

## 1.19-10 還沒解的：出口過渡

```text
     7.8973 ..    8.2573 (0.3600 s)  LF 出口過渡 x RF 出口過渡
    11.0895 ..   11.4495 (0.3600 s)  LH 出口過渡 x RH 出口過渡
     3.7450 ..    3.7778 (0.0329 s)  RF 入口過渡 x LF 入口過渡
     6.9512 ..    6.9685 (0.0174 s)  LH nominal x RH 入口過渡
     6.9685 ..    6.9701 (0.0015 s)  RH 入口過渡 x LH 入口過渡
```

由 §1.19-6 的幾何：**這不是排程能解的**。要解它只有兩條路，
兩條都需要一個目前不存在的運動基元：

* **在出口過渡之前，讓每隻腳在地面上多滾一段（theta=17、輪模式）**——
  出口離地就錯開了。`LOWER_GROUND_CONTACT` 已經是這個姿態滾在地上，
  但 Day 12 這邊沒有「輪模式地面滾動」的產生器
  （`run_foot_rim_roll_2d` 滾的是腳掌弧，不是輪緣）。
* **讓越障序列吐出「踩上障礙物的位置」參數**——
  入口觸地就錯開了。這要 Day 10-11 重新產生序列。

**沒有動手。** 這是一個要選的設計，不是一個要修的 bug。

## 1.19-11 這一輪的測試

新增 `tests/test_day12_world_registration_2d.py`（**這個模組原本一個測試也沒有**）：
11 passed（6:33）。鎖住的是三件事——
位置排程重現步態自己的視窗、recovery 的髖前進不是自由參數、
以及越障交回來的是一個**相位**。

其中 `test_the_gait_order_survives_the_crossing` 在寫完的第一次就**抓到了**
`+ phase` 的符號錯誤（陷阱 86）。

受這次改動影響的四個檔（`plan_terrain_2d` 那條路上的）：

```text
tests/test_day12_transition_mapping_2d.py
tests/test_day12_terrain_generalization_2d.py
tests/test_day12_paper_metrics_2d.py
tests/test_day12_obstacle_registration_2d.py
                                    94 passed（34:16）
```

其他呼叫 `plan_four_legs_2d` 的地方（14 個 driver、11 個測試檔）
全部是位置參數，`schedule` 是 keyword-only 且有預設值，**一個都不受影響**。


---

# 1.20 stance/body formulation 換向：動手前的可行性探測（2026-09-04）

> 來源：`hybrid_gait_day12_discussion_summary_zh_TW.md`。
> 那份文件要把因果從「各腿要求一個 body 高度 -> merge」換成
> 「共用 body 軌跡 -> 各 stance 腿自己解 theta 維持滾動接觸」。
> **動機器之前先做便宜的探測**（Day 13 §3.2 的同一句話）。

## 1.20-1 先量「要求」有多大

越障序列每一段的 `body_requirement` **全部是 `TRACK`（硬的）**，
hip_z profile：

```text
   0 APPROACH                       183.635 -> 181.378
   2 RIGHT_RIM_ROLL_UP              182.890 -> 198.035   <- 最高
   4 RETRACT_TO_WHEEL               197.062 -> 184.347
   8 LEFT_RIM_ROLL_DOWN             182.675 -> 144.874
   9 LOWER_GROUND_CONTACT           143.794 -> 143.798   <- 最低
   整段 hip_z  143.794 .. 198.035 mm（跨度 54.242 mm）
```

而平地 nominal hybrid 站在 **219.4478 mm**。

**所以衝突不是「約 15 mm」，是 21.4 ~ 75.7 mm。**
根源也不是 formulation 的細節，是**兩個生成器的姿態從來沒有對齊過**：
越障序列是用 compact 姿態（theta 40 -> 17 度）產生的，
平地 nominal 是伸展姿態（theta 60 -> 72.5 度）。
平均或放寬容差永遠補不起這個差距——這也解釋了為什麼 Step 5 只能回 INFEASIBLE。

## 1.20-2 探測：平地 stance 腿跟得下去嗎

問題精確化成：**固定 beta，把 hip 降到目標高度，
`theta_for_hip_z_2d` 解得出 theta 嗎？解出來的姿勢還是腳掌輪緣踩在地上嗎？**

```text
 hip_z mm | beta+40        beta+20        beta-0         beta-20        beta-40
   219.45 | 72.5 ok        62.6 ok        60.0 ok        62.6 ok        72.5 ok
   200.00 | 58.4 ok        51.1 ok        49.2 ok        51.1 ok        58.4 ok
   180.00 | 43.9 ok        39.2 ok        37.9 ok        39.2 ok        43.9 ok
   160.00 | 29.1 ok        26.9 ok        26.4 ok        26.9 ok        29.1 ok
   150.00 | theta-limit    20.4 ok        20.2 ok        20.4 ok        theta-limit
   145.00 | theta-limit    17.0 ok        17.0 ok        17.0 ok        theta-limit
   143.80 | theta-limit    theta-limit    theta-limit    theta-limit    theta-limit
```

**跟得下去。** 弧中段可以從 219.45 一路蹲到 **約 145 mm**（theta 60 -> 17 度），
接觸點**全程留在 foot_rim**，抬升修正都在 0.1 um 以下。
弧兩端（beta ±40 度）theta 行程先用完，大約到 150~160 mm。

越障要求的 143.794 ~ 198.035 mm，**只有最後一段
`LOWER_GROUND_CONTACT`（143.8 mm）差約 1~6 mm 超出**，其餘全部在範圍內。

## 1.20-3 一個中途量錯的結果，記下來當教訓

第一次探測我用 `standing_stroke_2d` 判定「站不站得住」，得到
「只能蹲 0.718 ~ 17.287 mm」——**小到會直接否決整個計畫**。

那是假的。`theta_for_hip_z_2d` 解出來的姿勢**剛好踩在 z = 0 上**，
量到的最低點在 ±0.0001 mm 之間跳；落在負的那一半時
`posture.query()` 就判成 collision，於是 `standing_stroke_2d` 回
`STANDING_POSE_IS_NOT_A_GROUND_CONTACT`。

**這正是 `standing_pose_penetration_m` 的 docstring 早就寫過的那件事**
（「越障自己的出口姿勢差 0.0001 mm 就被判成碰撞」），
而 `standing_stroke_at_2d` 就是為了這個而存在——它會先量再抬。
換成它之後，145 mm 以上全部通過。

> 判斷方法：**一個「不可行」的結論，要先確認它來自幾何，不是來自某個
> helper 的驗收政策。** 直接去看最低點落在哪個 region、離地多少，
> 比相信一個回傳 `success=False` 的包裝快得多。

## 1.20-4 對計畫的三個結論

1. **換向是可行的，而且平地不會動到。** 平地的 body 高度是常數，
   換向之後 stance 腿收到的就是同一個常數。
2. **真正第一次考到新 formulation 的，是「一腳在障礙物上、三腳在地上」。**
   平地已經 12/12 全過，只能當回歸防線。
3. **已知的邊界有兩個**，先寫下來免得之後當成 bug：
   * 越障最後一段 143.8 mm 超出 theta 行程約 1~6 mm；
   * stance 腿在自己弧的兩端時可用行程較少（150~160 mm）。
   兩個都很窄，而且都在「出口過渡」附近——和 §1.19-10 是同一個區域。


---

# 1.21 stance 腿改成「跟隨」而不是「要求」：第一塊（2026-09-04）

> §1.20 說換向可行。這一節是它的第一塊實作：
> **讓滾動生成器跟隨一條 body 高度軌跡，而不是釘在一個常數上。**

## 1.21-1 掛鉤選在哪，以及為什麼是髖的 x

`NominalPosture2D.hold_hip_z_m` 是一個常數高度。
新的 `hold_hip_z_profile: HipZProfile2D` 是**依髖部 x 索引**的高度。

選 x 不選時間，是為了避開一個循環：
位置排程已經把時間定義成「髖部行程 ÷ 一個車速」，而 `hip_x = mount_x + body_x`，
所以**一條依髖部 x 的曲線本身就是一條 body 軌跡**——
不需要時鐘，也不需要在 stroke 生成出來之前先問它要跑多久。

```python
HipZProfile2D.constant(z)      # 就是 hold_hip_z_m，一個位元都不差
profile.at(x)                  # 範圍外**夾住兩端**，不外插
posture.held_hip_z_at(hip_x)   # 唯一回答「這裡要多高」的地方
posture.holds_hip_z            # 有沒有在水平化
```

範圍外夾住而不是外插，是刻意的：一隻多滾了一點、超出 body 取樣範圍的
stance 腿，應該**繼續站在最後被告知的高度**，而不是自己編一段 body 運動。

## 1.21-2 先證明它沒有改變任何既有的東西

```text
                     hold_hip_z_m   constant profile
frames                     49              49
contact advance    202.457908 mm   202.457908 mm
hip advance        325.915766 mm   325.915766 mm
hip ripple           0.000176 mm     0.000176 mm
每一幀 theta/beta/hip/contact              逐位元相同
```

**常數 profile 與 `hold_hip_z_m` 逐位元相同。** 平地不會動。

## 1.21-3 斜的 profile：跟得上，而且不損失行程

```text
     slope  frames  stop                 contact mm   track err mm
     0.000      49  RIM_ARC_EXHAUSTED       202.458         0.0001
     0.020      49  RIM_ARC_EXHAUSTED       202.458         0.0256
     0.050      49  RIM_ARC_EXHAUSTED       202.458         0.0682
     0.100      49  RIM_ARC_EXHAUSTED       202.458         0.1571
     0.250      49  RIM_ARC_EXHAUSTED       202.458         0.4270
```

`slope` 是 d(body 高度)/d(髖部 x)。越障下降段最陡是 **0.47 mm/mm**，
但那段只佔 80 mm 的髖部行程，攤到一個 326 mm 的 stroke 上約 **0.12**。

**接觸前進一毫米都沒有少**（202.458 mm，與平地相同）。
跟隨 body 不需要用步幅去換。

## 1.21-4 追隨誤差：三個版本，兩次量錯的方向

誤差正比於 slope，比例常數就是「幾毫米的髖部行程」：

```text
版本 1  在**現在**的髖 x 讀高度          誤差 ~ 8.0 mm x slope
版本 2  在「現在 + 上一步」讀            誤差 ~ 6.4 mm x slope   （只好一點）
版本 3  版本 2 ＋ 補上第一步的種子        誤差 ~ 1.6 mm x slope   （好 5 倍）
```

版本 2 為什麼幾乎沒有改善，是**逐幀印出來才看到的**：

```text
  i     hip_x mm     hip_z mm   want(x) mm       err mm
  1       6.3788     219.4478     218.8099       0.6379   <- 全部在這裡
  2      14.9599     218.1721     217.9518       0.2203
  3      23.2126     217.0938     217.1265      -0.0328
 48     300.8221     189.3761     189.3656       0.0105
```

**0.6379 mm 的「最大誤差」整個就是第 1 幀。** 第一步沒有「上一步」可以學，
`step = 0`，於是它退化回版本 1。第 3 幀之後誤差已經 ≤ 0.04 mm。

補法不是再解一次，而是**把已經算好的固定 theta 樣板放到它的目標上，
問它髖落在哪**——一個平移，不是一次求解。

> 教訓：**一個「最大值」型的指標，要先看它是不是全部集中在一個點上。**
> 我差一點就因為 1.85 mm 去做整個熱迴圈的兩次疊代（成本加倍），
> 而真正要修的只有第一步。

## 1.21-5 現在的誤差在什麼量級上才算可接受

```text
接觸查詢的容差 contact_tolerance_m      1.000 mm
平地 support margin                     4.839 mm
越障實際 slope 下的追隨誤差             ~0.16 mm
```

而且誤差**不隨 stroke 累積**（第 48 幀 0.0105 mm），它是每一步各自的殘差。

## 1.21-6 測試

`tests/test_day12_nominal_cycle_2d.py` **44 passed**（5:19），新增 7 個：
常數 profile 逐位元相同、profile 優先於純量、兩端夾住不外插、
拒絕倒退／長度不符的 profile、斜 profile 不損失行程、
追隨誤差在接觸容差內且正比於 slope、沒有要求時回報 0。

## 1.21-7 第二塊：被告知高度的 stroke，不再要求高度

`roll_segment_2d` 本來一律發 `BodyRequirementKind.TRACK`，
註解寫的理由是：「滾動時 hip 高度是接觸幾何的**輸出**，所以 body 必須追蹤它」。

**那句話對固定 theta 的滾動是對的，對被水平化的滾動是錯的。**
固定 theta 時髖騎在輪緣弧上，腿沒有選擇；
被交付一個高度、用 theta 去守住它的腿，是**輸入**，它已經在跟隨了。

```python
kind=(BodyRequirementKind.NONE if stroke.posture.holds_hip_z
      else BodyRequirementKind.TRACK)
```

固定 theta 那條路一個字都沒動，所以 Day 10-11 對滾動段的讀法不變。

### 先確認平地不會被這個改動推走

拿掉 TRACK 之後平地就沒有硬需求了，body 會落到 nominal 高度；
而且**擺動腿的 LOWER_BOUND 有可能反而把 body 頂上去**。量過才動：

```text
stroke   hip_z  219.4477 .. 219.4479 mm
recovery hip_z  219.4477 .. 219.4478 mm   <- LOWER_BOUND 取的最大值
nominal body_z            162.2818 mm
recovery 最大值 - offset  162.2818 mm     <- 不高於 nominal，不會觸發
```

水平化的 recovery 把髖也守平了，所以那個下界剛好等於 nominal（差 0.1 um）。
**平地安全。**

## 1.21-8 結果：body conflict 109 -> 16

```text
                       改動前   改動後
平地 validator          0 失敗   0 失敗（逐位元相同）
平地 swing              8 個、每 0.6 s、零重疊   同左
40 mm body conflicts    109      16
40 mm 其他五項失敗      不變     不變
```

**85% 的衝突是假的**——它們來自三隻「其實沒有在要求任何東西」的腿。

### 剩下的 16 個還沒歸因

不要猜。已知的線索是：前腳對的越障區間是 3.7778-7.8973 s，
後腳對是 6.9701-11.0895 s，**重疊 0.927 s**，
那段時間四隻腳同時在障礙物上、而且在不同的階段，
body 不可能同時在兩個高度——真要解只能讓車體**俯仰**，而 2D 模型沒有這個自由度。

但 121 個取樣裡 0.927 s 只有約 8.4 個，**不是 16 個**。
所以這個解釋不完整，**還缺一次量測**（要把衝突的時間點印出來）。
寫在這裡是為了下次不要把它當成已經解釋過的事。

## 1.21-9 重跑產物時發現：Step 9 的檔案過期的是**欄位**，不是數字

重跑 `day12_step9_driver.py` 之後 `diff` 說檔案變了，逐格比對才看出原因：

```text
row 0 col 27  motor_limit_deg_s  ->  peak_motor_rate_upper_bound_deg_s
row 0 col 28  motor_utilisation  ->  motor_limit_deg_s
...            所有數值往後位移一欄，值本身完全相同
```

**多了一個欄位**（`peak_motor_rate_upper_bound_deg_s`，A8 那一輪加的），
而磁碟上的檔案還是加欄位之前寫的。12 個「數值差異」全部是位移造成的假象。

> 這也順帶證實了原本的假設：**legacy 那條路（duty 0.75、固定 theta）
> 完全沒有被這一輪的改動碰到**——`roll_segment_2d` 的 `TRACK -> NONE`
> 只在 `posture.holds_hip_z` 時觸發，而 legacy 用的是沒有水平化的姿態。
> Step 3-9 的數字重跑後一模一樣。

> 教訓：**`diff` 說「變了」不等於「數字變了」。** 對逐欄輸出的 CSV，
> 先比欄位名，再比值；不然會把一次 schema 變更讀成一次回歸。

## 1.21-10 那 16 個衝突歸因了：前腳在下坡，後腳同時在上坡

不猜了，印出來：

```text
  t=  7.3274  LF   122.920 mm  vs LH/RH   124.603 mm   gap    1.683 mm
  t=  7.4033  LF   119.220 mm  vs LH/RH   126.063 mm   gap    6.843 mm
  t=  7.4792  LF   114.300 mm  vs LH/RH   132.804 mm   gap   18.504 mm
  t=  7.5551  LF   108.202 mm  vs LH/RH   137.527 mm   gap   29.325 mm
  t=  7.6310  LF   100.977 mm  vs LH/RH   140.183 mm   gap   39.206 mm
  t=  7.7069  LF    92.699 mm  vs LH/RH   140.849 mm   gap   48.150 mm
  t=  7.7828  LF    86.633 mm  vs LH/RH   140.913 mm   gap   54.281 mm
  t=  7.8587  LF    86.631 mm  vs LH/RH   140.811 mm   gap   54.180 mm
```

16 = **8 個瞬間 x 2**（LF 同時和 LH、RH 衝突）。全部落在一個窗口
**7.3274 .. 7.8587 s**，而那個窗口裡四隻腳在做的事是：

```text
  LF / RF : LEFT_RIM_ROLL_DOWN | LOWER_GROUND_CONTACT     <- 從障礙物**下來**
  LH / RH : RIGHT_RIM_ROLL_UP  | RIGHT_RIM_TOP            <- 往障礙物**上去**
```

**前腳在下坡、後腳同時在上坡。** 一個要車體降下去，一個要車體升上來，
差距從 1.683 mm 長到 **54.281 mm**。

**沒有任何單一高度的剛體可以同時滿足兩邊。這需要俯仰（pitch）。**

```text
需要的俯仰角 = atan(54.281 / 510) = 6.08 度      （510 mm 是軸距）
```

### 為什麼會重疊，以及什麼時候不會

```text
越障佔用的髖部行程   658.13 mm   （序列 hip -36.02 -> 622.11）
軸距                 510.00 mm
重疊                 658.13 - 510.00 = 148.13 mm  ->  0.927 s
```

**越障比軸距長，所以前腳還沒下完，後腳就開始上。**
其中最大的一段是 `WHEEL_MODE_TOP_ROLL`（225.9 mm），
而它的長度是由**障礙物頂面長度 400 mm** 決定的。

由此得到一個可檢驗的預測：
**頂面短到讓越障行程 < 510 mm（約 250 mm 以下）時，前後腳不會重疊，
這 16 個衝突應該完全消失。** 一次 `plan_terrain_2d(40mm x 250mm)` 就能驗證。

### 對計畫的影響（誠實版）

因果換向做完了它能做的：**109 -> 16，93 個是假需求**。
剩下的 16 個**不是換向能解的**，也不是排程能解的——
越障腿是在重播 Day 6-7 錄下來的幀，它不會適應；
兩隻重播中的腿要求兩個不同的高度，就是要求兩個不同的高度。

要解只有三條路：

```text
(a) 車體允許俯仰            2D 模型目前沒有這個自由度
                            （討論文件 §6-4 還明確把它固定成 0）
(b) 越障序列可以在指定的 body 高度重新生成，而不是重播
                            = 我先前說的「兩個生成器的姿態對齊」
(c) 只做越障行程短於軸距的地形    迴避，不是解
```

## 1.21-11 用另一個頂面長度驗證，順便把根因講得更準

`40mm x 300mm`（越障行程 558.13 mm，超出軸距只有 48 mm）：

```text
                        400 mm 頂面      300 mm 頂面
越障髖部行程             658.13 mm        558.13 mm
超出軸距                 148.13 mm         48.13 mm
衝突數                        16                8
衝突窗口                 0.531 s          0.212 s
最大歧異                54.281 mm        39.198 mm
窗口內後腳在做的事    RIGHT_RIM_ROLL_UP    APPROACH
```

**方向與量級都對上了**（重疊少了 3 倍，衝突少了一半）。
沒有對到「正好三分之一」的原因也清楚：一個衝突要被記錄，
兩邊的歧異必須大於 `HARD_AGREEMENT_M = 1 mm`，
所以重疊的頭尾（歧異還很小的時候）不算。400 mm 那組第一個衝突的
歧異就是 **1.683 mm**，剛好卡在門檻上。

### 更準的根因：不是「上坡 vs 下坡」，是**越障序列自己的高度跨度**

```text
越障序列自己的 hip_z    143.794 .. 198.035 mm     跨度 54.241 mm
400 mm 量到的最大歧異                             54.281 mm   <- 就是那個跨度
300 mm 量到的最大歧異                             39.198 mm
   （後腳在 APPROACH 的 183.6 mm，前腳在 LOWER_GROUND_CONTACT 的 143.8 mm，
     差 39.837 mm）
```

**只要有兩隻腳同時停在越障的不同位置，它們要求的 body 高度就會差到
越障序列自己的高度跨度為止。** 「前腳下坡、後腳上坡」只是這個現象
最極端的一種排列，不是原因。

因此條件是：

```text
兩隻腳同時在越障中  <=>  越障髖部行程 > 軸距 (510 mm)
歧異上限            =   越障序列自己的 hip_z 跨度
```

這也解釋了為什麼 (c)「只做行程短於軸距的地形」是迴避而不是解——
它只是讓兩隻腳不要同時在裡面，沒有讓序列的高度變得可協商。

## 1.21-12 重跑產物（2026-09-04）

```text
Step 9    重跑     數值完全相同，只是磁碟上的檔案少一個欄位（見 1.21-9）
Step 10   重跑     obstacle 那幾列**大幅改變**，flat 不變
Step 11   重跑     見下
Day 13    重跑     馬達 CSV 與 09-02 版差 3.55e-15 rad（浮點噪音）
                   Step 9 在匯出軌跡上 0 項失敗，馬達 72.0%
```

Step 10 的比較表，改動前後：

```text
                 feasible   nom sw   margin mm   failed
flat      前后      True       8       4.8394       0     <- 不變
40mm      前       False       8     -18.5711       3
40mm      後       False      21     -23.5046       6
100mm     前       False       8     -17.9847       3
100mm     後       False      21     -34.7296       7
190mm     前后     False       0        n/a        n/a    <- 資料缺口，不變
```

`nominal_recovery_swings` 8 -> 21 是**世界註冊接上了**：
每隻腳現在真的滾到障礙物所在的位置，而不是四隻腳各自從 x=0 開始。
`failed_checks` 3 -> 6/7 不是變壞，是**變誠實**：
舊的索引排程把時間平均分給段落，掩蓋掉的失敗現在會報出來。

**平地在這一整輪之後仍然是唯一 feasible 的地形，而且數字沒有動。**

### Step 11 重跑，順便交叉印證了 `segment_chaining`

```text
         terrain  bodyz p2p  bodyz std   usable  min margin  mean margin  hip lift
            flat      0.000      0.000 121/121       4.8394      12.1676     0.000
    40mm x 400mm     37.406     17.083 112/121     -23.5046       6.4061    75.650
   100mm x 400mm     99.549     17.215 103/121     -34.7296       5.2430    97.294

         terrain  max joint disc  max contact gap
            flat          0.0000            0.000
    40mm x 400mm          1.0000          108.512
   100mm x 400mm          1.0000          175.170
```

**`max contact gap` = 108.512 mm，和 §1.19 在越障序列自己的接縫上量到的
`LEFT_RIM_ROLL_DOWN -> LOWER_GROUND_CONTACT` 一模一樣。**
兩個獨立的量測撞在同一個數字上，所以可以確定：
`segment_chaining` 的失敗是**從 Day 10-11 的序列繼承來的接觸換特徵**，
不是 Day 12 組裝造成的，也不是腿真的瞬移（同一個接縫髖只跳 1.14 mm）。

平地：`bodyz p2p 0.000`、`usable 121/121`、margin 4.8394 mm —— 乾淨。

## 1.21-13 爬障礙物用 roll 還是 swing：掃過高度之後的答案

`top = 400 mm`，掃高度（`decide_2d` + `compose_2d`，只讀表，很快）：

```text
  高度      勝出策略        ascent/descent        為什麼
   20 mm   SWING_SWING    SWING_UP/DOWN     ROLL 在這個高度【沒被掃過】
   40 mm   ROLL_ROLL      ROLL_UP/DOWN      roll feasible, theta_climb = 40 deg
   60 mm   ROLL_ROLL      ROLL_UP/DOWN
   80 mm   ROLL_ROLL      ROLL_UP/DOWN
  100 mm   ROLL_ROLL      ROLL_UP/DOWN
  120 mm   ROLL_ROLL      ROLL_UP/DOWN
  140 mm   ROLL_ROLL      ROLL_UP/DOWN      roll 仍然 feasible
  160 mm   SWING_SWING    SWING_UP/DOWN     ROLL【infeasible】：
                                            "no theta_climb completes the
                                             traversal at this height"
  180 mm   SWING_SWING    SWING_UP/DOWN     ROLL 沒被掃過
  190 mm   （沒有勝出）                      五個策略全部沒被掃過
```

（30 / 50 / 70 / 90 / 110 / 130 mm 都是 `no strategy is available at this cell`
——表是以 **20 mm 為間隔**掃的，奇數格沒有資料。）

### 三個結論

1. **主力是 roll：40 到 140 mm 全部用 `ROLL_UP` / `ROLL_DOWN`。**
   這是 B1/B2 的規則 B 在起作用——決策順序把 `roll_preference` 排在 `margin`
   前面，所以只要 roll 可行就選 roll。

2. **滾動的天花板落在 (140, 160] mm，但【還沒被夾緊】。**
   140-160 之間以 2 mm 加密掃過：

```text
   height  winner        ROLL_ROLL     ROLL reason
      140  ROLL_ROLL     feasible      theta_climb = 40 deg, the smallest that fits
      142  None          not measured  no data
      144  None          not measured  no data
      145  None          not measured  no data
      146  None          not measured  no data
      148  None          not measured  no data
      150  SWING_SWING   not measured  no data
      152  None          not measured  no data
      155  None          not measured  no data
      160  SWING_SWING   infeasible    no theta_climb completes the traversal
```

   **只有兩個真實量測**：140 mm 可行、160 mm 不可行（`limiter=ascent`，
   是真的幾何極限不是缺資料）。中間 142-155 mm 全部是 `not measured`。
   所以「Hybrid 能滾多高」目前只能講成 **140 mm 以上、160 mm 以下**，
   要一個數字就得補掃那一段。

   注意 150 mm 的 winner 是 SWING_SWING 而 142-148 / 152-155 都是 None
   ——**swing 表和 roll 表不是掃在同一組高度上**，
   所以「某個高度有沒有答案」取決於哪張表剛好有那一格。

3. **20 mm 用 swing 是【資料缺口造成的】，不是滾不上去**：
   那一格的 ROLL_ROLL 是 `not measured`。
   換句話說**低端的 swing 是掃描沒掃到，不是物理**。

### 順帶兩件一直都成立的事

```text
ROLL_SWING / SWING_ROLL   每一個高度都是 handoff blocked
                          （DIRECT_HANDOFF_INFEASIBLE）
                          -> 混合策略從來沒有出現過，上下半場一定同一種
SWING_OVER                每一個高度都是 not measured
```

## 1.21-14 滾上去的 theta 不是固定的，而且「越大越好」是反的

`theta_climb` 掃過 **40-85 度，每 5 度一格**（`day10_11_step4_roll_concession.csv`）。
選法寫在 `roll_roll_cell_2d` 的 docstring 裡：

> ``#1``: the smallest theta whose required top length still fits.
> Both halves of ``theta_climb`` are monotone and they oppose each other --
> excursion rises with theta, required top length falls with it --
> so the optimum is always at the constraint boundary, and no search is needed.

### 兩個彼此對抗的量，都量出來了

```text
theta_climb   需要的頂面長度    髖部起伏(100mm 障礙)   前進距離
   40 deg       279.4 mm  量測        114.86 mm         686.30 mm
   45 deg       260.6 mm  推估        117.59 mm         685.64 mm
   50 deg       251.8 mm  推估        120.92 mm         684.75 mm
   55 deg       243.0 mm  推估        124.21 mm         683.67 mm
   60 deg       247.2 mm  量測        127.64 mm         682.41 mm
   65 deg       228.4 mm  推估        131.47 mm         680.95 mm
   70 deg       222.5 mm  推估        135.72 mm         679.36 mm
   75 deg       216.7 mm  推估        140.21 mm         670.09 mm
   80 deg       210.8 mm  推估        144.51 mm         663.13 mm
   85 deg       215.0 mm  量測        149.37 mm         658.50 mm
```

**theta 越大 -> 需要的頂面越短（279.4 -> 215.0 mm），但髖部起伏越大
（114.9 -> 149.4 mm，多 30%），前進距離還略減。**

所以「theta 大一點能滾的部份比較多」**只在「頂面可以更短」這個意義上成立**；
代價是車體上下擺得更兇，而那正是 Step 5 body 衝突的來源。
現在的規則挑**最小的可行 theta**，等於在「頂面塞得下」的前提下**把起伏最小化**。

> 10 個 theta 裡只有 **40 / 60 / 85 度是量測**，其餘 7 個是推估。

### 可行性對 theta 完全不是單調的

```text
 obstacle  可行的 theta_climb (deg)                     可行數
     40 mm  40 45 50 55 65 70                            6 / 10
     60 mm  40 45 50 55 60 65 70 75 80 85               10 / 10
     80 mm  40 45 50 55 60 65 70 75 80 85               10 / 10
    100 mm  40 45 50 55 60 65 70 75 80 85               10 / 10
    120 mm  45 55                                        2 / 10
    140 mm  40 45 55 70                                  4 / 10
    160 mm  (none)                                       0 / 10
```

**中間有洞，而且大 theta 失敗得更多。**

```text
 40 mm  的 60/75/80/85 度   APPROACH_DID_NOT_REACH_THE_FRONT_FACE
                            （障礙物太矮，伸得太開的腿反而搆不到前面）
120-140 mm 的多數           COUPLED_RESET_COLLISION_BLOCKED
160 mm 的十個               COUPLED_RESET_COLLISION_BLOCKED /
                            NO_SLIP_REQUIRES_NEGATIVE_X_MOTION /
                            NO_LEGAL_CORNER_PIVOT_CONTINUATION
```

60-100 mm 是甜區（十個 theta 全可行）；難的是**太矮**（40 mm）和**太高**（120 mm 以上）。

# 1.22 接觸換「面」不是腿瞬移（2026-09-04）

> §1.19-10 說 `segment_chaining` 的 108.512 mm 大概是接觸點換特徵。
> 「大概」不夠，先量。

## 1.22-1 量：判別的是「面」，不是輪緣，也不是大小

越障序列自己的九個接縫，逐個量接觸跳、髖跳、輪緣、面：

```text
接縫    接觸跳     髖跳    輪緣            面
0->1     1.687    3.172   foot -> foot    ground -> ground
1->2    92.782    3.238   foot -> right   ground -> obstacle_top     <-
2->3     2.928    2.758   right-> right   同
3->4     0.000    1.111   right-> right   同
4->5     2.928    0.861   right-> right   同
5->6     5.855    2.509   right-> left    同          <- 換了輪緣，只跳 5.9 mm
6->7     1.807    4.317   left -> left    同
7->8     0.000    2.510   left -> left    同
8->9   108.512    1.570   left -> left    obstacle_top -> ground     <-
```

**兩個 >90 mm 的跳，剛好就是兩個換面的接縫；每一個沒換面的接縫都 <= 5.855 mm。**
而且換面的那兩個接縫**髖只動了 3.238 mm 與 1.570 mm** —— 腿還在原地。

`5->6` 換了**輪緣**卻只跳 5.855 mm，所以**輪緣不是判別式**；
大小也不是（那只是結果）。**面才是。**

## 1.22-2 這是陷阱 1 再深一層

Step 0 說「一個 boundary 有兩種（CUT / HANDOVER），不要用同一組門檻」。
這裡是第三種：**接觸轉移到另一個面**。

* 接觸留在同一個面時，該量的是**接觸點** —— 它動了就是腳滑了或腿跳了。
* 接觸換到另一個面時，接觸點**必然**跳，因為現在是腿上**另一個點**碰到
  **另一個東西**。這時該問的是「**腿**有沒有跳」，而髖才回答得了。

```python
if handoff.surface_changed:
    if handoff.body_jump_m > max_contact_gap_m:   # 同一個容差，量另一個東西
        ...
    continue
if handoff.contact_jump_m > max_contact_gap_m:
    ...
```

**容差沒有放寬**（都是 10 mm），換掉的是被量的量。

## 1.22-3 一個刻意不放過的情形

`_surface_changed_2d` 在**任一邊離地時回傳 False**。
擺動段沒有面；若把「沒有面」當成「不同的面」，
**每一次離地與觸地都會被豁免** —— 而那正是最需要這個檢查的地方。
測試 `test_an_airborne_boundary_is_never_called_a_surface_transfer` 就是釘這個。

## 1.22-4 結果

```text
                      修改前   修改後
平地                  0 失敗   0 失敗（body conflicts 0，不變）
40 mm 失敗項          6        5
  segment_chaining    失敗     -> 消失
  其餘四項            不變     不變（16 個 body 衝突也一樣）
```

**只掉那一項，其他一個數字都沒動。** 這正是一個「修對了檢查」該有的樣子：
如果它順手讓別的失敗也消失，那就是豁免開太大。

Step 11 量到的 `max contact gap = 108.512 mm` 仍然會被**報出來**——
它是量測欄位，不是判定；判定改了，量測沒有被藏起來。

## 1.22-5 回歸：107 passed，而且**已知的斷裂還在被抓**

```text
tests/test_day12_whole_body_validation_2d.py
tests/test_day12_whole_body_trajectory_2d.py
tests/test_day12_terrain_generalization_2d.py
tests/test_day12_paper_metrics_2d.py
                              107 passed（18:53）
```

其中 `test_the_chaining_failure_is_the_known_break_not_a_new_one`
斷言的是**平地** legacy 路徑上 **297.065 mm** 的已知斷裂（陷阱 25：
兩段 nominal run 各自獨立生成）。那個接縫**兩邊都在 `ground`、沒有換面**，
所以照舊被抓出來。

**這是豁免沒有開太大的證據**：一個真正的斷裂，和一個換面的轉移，
現在被分開對待，而且分得出來。

新增兩個測試：

```text
test_the_surface_is_what_says_a_contact_jump_is_a_transfer
    釘住「判別式是面，不是輪緣、不是大小」
test_an_airborne_boundary_is_never_called_a_surface_transfer
    釘住「離地／觸地不得被豁免」
```

# 1.23 讓 stance 腿跟隨車體：**沒有成功**，但失敗的原因量出來了（2026-09-04）

## 1.23-1 想做的事

越障腿把車體拖到 183 mm，而三隻還在地上的 nominal 腿仍然生成在
219.4478 mm —— 它們站的高度和車體說的差 36 mm。
所以：從越障序列導出 body 高度 profile，再讓每隻腳依自己的 mount 讀它。

導出**不需要第一趟建構**：`target_hip_x = obstacle_x - landing_offset`
是障礙物相對的，四隻腳共用，所以越障佔用的是**同一個髖部 x 視窗**，
它的高度曲線可以直接從序列算出來。驗證：
重定位後越障第一個接觸落在 **100.0000 mm**，正是障礙物起點。

## 1.23-2 差點出貨的一個 bug

第一版對每個越障需求取 `np.maximum(..., nominal)`。
**但整個越障都在平地站姿【底下】**（body 86.6-140.9 mm vs 162.3 mm），
所以那個 max 把每一個需求都抹掉了，profile 從頭到尾是 162.2818 mm。

看起來會像「這個改動沒有效果」，而不是「這段程式寫錯了」。
**是動手建構之前那個幾秒鐘的數值檢查抓到的。**
改成只在**兩個越障需求之間**取大者之後：

```text
profile body_z   86.628 .. 162.282 mm    低於 nominal 的取樣點 553 / 601
```

## 1.23-3 結果：更糟，而且要說清楚為什麼「衝突變 0」不是好事

```text
                    follow_body=False   follow_body=True
四腳成立              4 / 4               2 / 4     <- LH、RH 整隻不見
body conflicts        16                  0
失敗項                5                   6         <- 多了 body_continuity
```

**`body conflicts` 從 16 掉到 0 不是解決了，是那兩隻互相衝突的腳不存在了。**
一個指標在別的東西壞掉時變好，那個指標當下不能讀。

LH / RH 的 refusal：`the approach overshot: it would need a hip advance of
0.000 mm`。

## 1.23-4 真正的原因：profile 破壞了**平移不變性**

`run_nominal_cycles_2d` 造**一個** cycle，其餘用 `translate_cycle_2d` 平移
（Day 12 早期量過：每個 nominal cycle 都是 cycle 0 的平移，殘差 0.0000 um）。
**位置相依的姿態讓那個前提不成立。**

用一條「一公尺掉 20 mm」的 profile 直接量：

```text
 cycle      起點hip_x      起點hip_z    profile要求     誤差
     0        0.000        219.4478     219.4478     0.0000 mm
     1      378.084        219.4478     211.8861     7.5617 mm
     2      756.167        219.4478     204.3245    15.1233 mm
```

**每個 cycle 都停在第一個 cycle 的高度，誤差線性累積。**

同一個前提還有兩個地方在用：

```text
run_nominal_cycles_2d   造一個 cycle 再平移            <- 上面量到的
run_approach_2d         在 hip_x=0 生一個 probe stroke  再推起點
                        位置相依時，原點的 stroke 不等於實際位置的 stroke
                        -> approach 的落點算錯 -> 就是 LH/RH 的 overshoot
translate_cycle_2d      整個函式的意義就是平移不變
```

## 1.23-5 決定：`follow_body` 預設關閉，機制留著

改動全部保留但**預設 `False`**，所有既有數字不動
（`follow_body=False` 下 40 mm 仍是 4/4、16 個衝突、5 項失敗）。

要真的做成，需要的不是接線而是三件事：

```text
1. run_nominal_cycles_2d 在姿態位置相依時【逐個生成】而不是平移
   （成本：一個 cycle 變 N 個，四腳越障建構會從 8 分鐘往上跳）
2. run_approach_2d 的 probe 要在【實際起點】生，不是在原點
3. translate_cycle_2d 要能拒絕位置相依的姿態，而不是默默給錯答案
```

> 教訓：**加一個「隨位置變化」的參數，會打破所有「先算一次再平移」的最佳化。**
> 那些最佳化當初是對的，而且被量過（殘差 0.0000 um）——
> 正因為它們是對的，才沒有人會想到去檢查它們還成不成立。
> 加這種參數時，要先 grep 一次 `translate`。

## 1.23-6 下一塊（還沒做）

```text
1. 把 16 個衝突的時間點印出來，歸因
2. 兩段式建構：先建越障 -> 從它取出 body 高度 profile
   -> 再用那條 profile 生成三隻 nominal 腿
   （現在 nominal 腿仍然生成在 219.4478 mm，而 body 已經跟著越障腿下到 183 mm）
3. 然後才重量 margin / motor / timing
```

---

# 2. 目前狀態

```text
Step 0   完成並驗證（含 2026-09-01 規格換 FINAL 版之後的補做）
Step 1   完成並驗證
Step 2   完成並驗證
Step 3   完成並驗證
Step 4   完成並驗證
Step 5   完成並驗證（結論是 INFEASIBLE，見 §1）
Step 6   完成並驗證（duty 0.75 下結論是 UNSTABLE，見 §1）
         2026-09-02（§1.6）：0.000 mm 是 duty=0.75 的定義值，不是 bug。
         duty 0.85 + 推導的 3 mm floor -> STABLE，4.839 mm。
Step 7   完成並驗證（planning floor 下 support 不足，見 §1）
Step 8   完成並驗證（軌跡組好了，但疊在三個未解結論上）
Step 9   完成並驗證（11 檢查 7 過 4 失，見 §1）
Step 10  完成並驗證（四地形一入口，gate 乾淨，見 §1）
Step 11  完成並驗證（paper metrics，見 §1）
Step 12  【完成】（2026-09-05）：七項要求全部達成
         day12_step12_freeze_and_handoff_zh_TW.md
         全套回歸 1153 passed / 9 failed，day12 與 day13 【零失敗】
         那 9 個逐一歸因，沒有一個 import hybrid_note
         （2026-09-04）freeze 文件已寫
         day12_step12_freeze_and_handoff_zh_TW.md
         要求 1-5、7 已完成並【查證】過，要求 6 全套回歸進行中

A5/C4 進行中（§1.9-1.23）
  已解  鏈接（24->0 break）、body_x 世界原點、
        位置排程**已接進管線**（`world_x_spread = 3.33e-16 m`，§1.19）、
        cycle 內相位（swing 每 0.6 s 交錯）、
        approach 的 0.553 mm 溢出（RF 從「整隻腳沒有」變成成立，§1.19）、
        越障後的相位重建（越障後的 nominal recovery 不再重合，§1.19）
        接觸換面 != 腿瞬移（§1.22，segment_chaining 失敗消失，6 -> 5）
  未解  同 `mount_x` 的兩隻腳**必然同時**越障（幾何，不是排程；§1.19-6）
        -> 出口過渡仍然 2 x 0.360 s 完全重合
        stance 腿跟隨車體（§1.23）：機制有了，但破壞平移不變性，預設關閉
        A5 的註冊指標尚未重量
...
Step 12  未開始

測試     Day 12   324 passed（27+31+31+27+40+24+28+21+24+29+23+19）
         A4 排查   34 passed（test_day12_support_margin_scan_2d.py）
         Day 10-11  181 passed（未受影響）

         2026-09-02 全套回歸（1:27:38）：
           1079 passed, 8 failed
         那 8 個【與本次工作無關】，已逐一查證：
           test_generate_csv_tui.py         5 個
             AttributeError: 沒有 GENERATE_ACTION_KEY / _start_generation /
             _render_progress_bar ...  測試領先實作（TUI 沒有這些成員）
           test_lateral_stance_symmetry.py  1 個
             3D 站姿高度 -254.98 mm vs 期望 -250 mm（差 4.98 > 容差 2 mm）
           test_toroidal_contact_and_checker.py  2 個
             CLI 印的字串是 "Swing Velocity Guard: Scaling..."，
             測試找的是 "Velocity Guard (Y)" —— 措辭不符
         獨立性證據：這三個檔只 import legwheel.*，
         而本次【完全沒有動 legwheel/ 底下任何檔案】。
```

**可以直接使用的東西**

```python
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    WHEEL_MODE_THETA_RAD,          # = RobotParams.THETA0_DEG，17 deg
    SegmentKind,                   # .FOOT_RIM_ROLL / .pins_theta / .is_nominal_locomotion
    MotionSegment2D, MotionSequence2D, PointContact2D, RollingContact2D,
    RollSampling2D, SwingSampling2D, BodyRequirement2D, FrameRef2D,
    TransitionKind, TransitionRequirement2D,
)

from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    entry_state_2d, exit_state_2d,     # 可串接的共同端點格式
    BoundaryKind, boundary_kind_2d,    # CUT / HANDOVER
    ChainTolerance2D, ChainBreak2D,
    chain_boundaries_2d, boundary_rows_2d,
    SegmentChain2D,                    # 多來源 chain
    SEGMENT_SEMANTICS, segment_semantics_rows,
)

from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    handoff_between_2d,                # 單一 boundary 的量測（Step 0 抽出來的）
    handoff_report_2d, sequence_from_traversal_frames_2d, segment_from_swing_plan_2d,
)

from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,        # theta 是參數，預設 60 deg（引用 Day 6-7 / 8-9）
    RecoveryConfig2D,        # theta_compact 只住這裡；預設 = RobotParams.THETA0_DEG
    RollStroke2D, RecoverySwing2D, NominalCycle2D, CycleFrame2D,
    run_foot_rim_roll_2d,    # 一條有限 stroke；max_distance_m 可截短
    run_recovery_swing_2d,   # retract / rotate / extend / touchdown
    run_nominal_cycles_2d,   # 串 N 個 cycle
    recovery_beta_target_2d, # = stroke 起點 beta - 2*pi
    cycle_segments_2d,       # -> (FOOT_RIM_ROLL, RECOVERY_SWING) 兩個 segment
    cycle_frame_rows, cycle_summary_rows,
)

from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LegId, LEG_ORDER,            # LF RF LH RH；索引仍是專案的 0 1 3 2
    LegMount2D, leg_mounts_2d,   # 由 CorgiLegKinematics 轉換得到
    FlatRunExtent2D,             # flat_before / flat_after（計畫輸入，不是地形邊界）
    LegState2D, FourLegState2D, SymmetryCheck2D,
    initialize_four_leg_state_2d,
    sagittal_reach_agreement_2d, # 2D/3D 掛載前提的量測
    four_leg_rows, plot_four_leg_state_2d,
)

from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode, GaitTiming2D, walk_timing_2d,   # 從 GAIT_LIBRARY["Walk"] 讀
    rotation_rate_demand_2d,                 # duty -> recovery 角速度要求
    ScheduledSegment2D, TimingConflict2D, FourLegSchedule2D,
    schedule_chains_2d,                      # {LegId: SegmentChain2D} -> schedule
    schedule_rows, plot_timeline_2d,
)
```

Day 10-11 那一整包仍然可用，清單見
`../day10-11/day10_11_implementation_log_zh_TW.md` §2。

**重跑 Step 0 的輸出**

```bash
cd "icra hybrid"
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step0_driver.py   # 約 2 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step1_driver.py   # 約 75 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step1_driver.py --no-animation  # 約 45 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step2_driver.py   # 約 20 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step3_driver.py   # 約 45 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step4_driver.py   # 約 5 分鐘
#   加 --with-roll-roll 會多跑 #1 的 traversal，再多約 3 分鐘
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step5_driver.py   # 約 5 分鐘
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step6_driver.py   # 約 40 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step7_driver.py   # 約 20 分鐘
#   慢的是 LEFT_RIM_READY 的 beta 掃描（0.5 deg 一步，每步建一整條腿）約 10 分鐘
#   ＋ Day 8-9 的 swing planner 每次約 110 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step8_driver.py   # 約 40 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step9_driver.py   # 約 40 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step10_driver.py  # 約 45 秒
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step11_driver.py  # 約 45 秒
```

**重建並執行 notebook**

```bash
cd LegWheel/hybrid_note/notes
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -c "
import nbformat
from nbclient import NotebookClient
p='hybrid_gait_day12_whole_body_dashboard.ipynb'
nb=nbformat.read(p, as_version=4)
NotebookClient(nb, timeout=900, kernel_name='python3',
               resources={'metadata':{'path':'.'}}).execute()
nbformat.write(nb,p)
"
```

（notebook 的 generator 腳本是 scratchpad 的一次性工具，不在 repo 裡。
notebook 本身已含全部內容與輸出，直接開啟即可。）

---

## 2026-09-01 — 附錄 A 實作（全機動畫）【已完成，不是一個 Step】

規格沒有要求這一節。加它的理由是：到 Step 11 為止，全機規劃只以**表格與指標**
存在，「四隻腳合起來到底長什麼樣」沒有任何一張圖看得到。

**它不是新的一步**：不呼叫任何 planner、不產生任何運動，畫面上每一個姿態都是
Step 8 的 `WholeBodyTrajectory2D` 讀出來的。

### 唯一被「發明」出來的數字，以及它的規則

Step 8 的 241 個取樣裡 `body_z` **全部是 `nan`**（Step 5 的 451 個 conflict）。
畫圖非有一個高度不可，所以規則寫死成一句可以被讀者反駁的話：

```text
取那個瞬間【所有 hard demand 的最大值】
= 沒有任何一隻站立腳被壓進地面的【最低】body 高度
```

其他站立腳於是浮在地面上方，浮的量 = 那隻腳自己的需求與被選中高度之差。
**那些紅色豎線就是 Step 5 的矛盾本身。**

取**最小值**畫面會更好看——每隻腳都貼著地——但那是把同一個矛盾埋到地面**以下**，
看起來像接觸。所以取最大值：**寧可看得到，不要看起來對**。

量出來的東西與 Step 5 完全對得上：

```text
畫圖用的高度         160.288 .. 162.281 mm
最差 hard demand 分歧  15.329 mm   <- 與 Step 5 的 worst conflict 同一個數
最差被留在空中的腳     15.329 mm （t = 0.000 s，LH）
有腳浮在空中的取樣     241 / 241   <- 每一個瞬間都有
airborne lower bound   min slack 15.294 mm（正值＝畫的 body 沒有低於淨空要求）
```

`viewing_heights_2d` **不回寫**：跑完之後 Step 5 的 `body_z` 仍然是 `nan`，
有一條測試專門守這件事（它失敗＝有一個 viewer 寫進了規劃資料）。

### 姿態必須用 generator frames 讀

`assemble_whole_body_2d(..., use_generator_frames=True)`。
預設的 `False` 在 segment 端點之間內插，而 `RECOVERY_SWING` 的頭尾 `theta` 都是
60 度 —— 於是**整條 swing 的 theta 都是 60 度**，這條 swing 賴以成立的縮腿根本
不會出現。Step 9 的 `peak theta rate` 讀到 0，講的是同一件事。

```text
LF theta，內插版            60.00 .. 60.00 deg
LF theta，generator frames  17.00 .. 60.00 deg
```

**但 Day 12 凍結的每一個數字是用內插版量的**，兩個讀法不要混著引用。

### 畫出來之後多知道的三件事

1. **腳的位置與宣稱的 gap 對得上。** 拿被畫出來的 hip 重建每隻腳的
   `SingleLegRollingScene2D`，量它的**最低點**，和 `float_gap_m` 比：
   誤差在 1.5 mm 以內，也就是那個既有的 rim-model 差（0.145 vs 0.1438 m）。
   這是一個**幾何**的交叉檢查，不是把同一段算術再算一次。
2. **297 mm 的 chain break 看不到，而且那是對的。** 取樣之間最大的接觸點移動只有
   **4.4 mm**：Step 8 的接觸點是**相對 hip** 放的，所以那個 297 mm 活在 handoff
   檢查表裡（Step 8 §8.1），不在取樣裡。動畫看不到它**不代表它不存在**。
3. **「四隻腳同時最多一隻在空中」在時間軸上直接看得到**，margin 掉到 0 也是。

### 三個踩到的繪圖陷阱

```text
19. 視窗的 padding 要繞開【輪子】不是【hip】。
    腿輪的 rim 在 leg-plane 原點【四周】半徑 0.145 m，
    x 與 z 兩邊都要留；只留 hip 的量會把輪子從畫面邊緣切掉一半，
    而且先被切的是側邊——那是最不容易發現的地方。
20. 一隻腳一個 PlotLeg。 PlotLeg 的 patch 是【持久物件】，
    兩隻腳共用一個 instance 會把同一個姿態畫兩次
    （同一個 figure 內換 axes 也一樣）。
21. twinx 的 ticks 在 clear() 之後會【跑回左邊】，
    蓋在腿的標籤上。每一幀都要重新 tick_right()。
```

### 產出

```text
程式  hybrid_note/scripts/experiments/day12_whole_body_animation_2d.py   新增
      hybrid_note/scripts/experiments/day12_animation_driver.py          新增
      hybrid_note/scripts/experiments/day12_animation_notebook_section.py 新增
      tests/test_day12_whole_body_animation_2d.py    新增，11 passed（約 25 秒）
資料  day12/day12_whole_body_viewing_height.csv   每個取樣畫在哪個高度、代價多少
圖    day12/day12_whole_body_frames.png           四次 swing 各一張
      day12/day12_whole_body_animation.gif        整趟 3 秒，61 幀（約 2.9 MB）
展示  notebook 的「附錄 A」節（A.1 - A.5）

重畫：python3 -u LegWheel/hybrid_note/scripts/experiments/day12_animation_driver.py
      （約 50 秒，含 GIF）
```

### 這張圖不是什麼

```text
不是可行性的證據   四個未解結論原封不動，就印在動畫左下角
不是實機時間       時間軸是 Step 3 的建模決定（duty 0.75）
不是 CoM           粗黑線是 body；這條 pipeline 沒有質量模型
不是新的量測       margin 來自 Step 6、姿態來自 Step 8、高度是畫圖挑的
```

---

## 2026-09-01 — 附錄 B 實作（越障段的世界座標註冊）【已完成，不是一個 Step】

起點是一個很小的要求：「附錄 A 只畫了平地，把越障也畫出來。」
做下去第一件事不是畫地形，而是發現**沒有地形可畫**——
而查清楚為什麼，就是這一節的內容。

### 組出來的軌跡有越障運動，卻沒有障礙物

4 cm 的 run（`plan_terrain_2d`）確實含 `SWING_UP` 17 個取樣、`SWING_DOWN` 16 個，
但**沒有任何一個接觸點碰得到平台**（接觸 x 走到 789 mm，平台在 1000-1400 mm）。

三個地方各自都是對的，合起來就把地形弄丟了：

```text
Step 5  body_x 是從支撐腳的【增量】積出來的，
        因為「每隻腳的 chain hip_x 都從 0 開始，絕對值不共用原點」
        —— 這句話就寫在 body_trajectory_2d 的 docstring 裡。
Step 10 terrain 只餵給 decide_2d / compose_2d（選策略），之後不再往下傳。
Step 8  接觸點放成【相對 hip】：contact_x = body_x + mount_x + offset_from_hip。
```

附錄 A 那個「297 mm chain break 看不到」是**同一個機制的另一面**。

### 註冊 = 一個減法

越障 segment 知道自己座標系裡的 hip 起點與終點；軌跡知道那兩個瞬間 hip 實際在哪。

```text
implied_x_start = world_hip_x - local_hip_x + COMPOSER_FRAME_X_START_M
```

`COMPOSER_FRAME_X_START_M` 是這次**唯一動到既有程式**的地方：
`day10_11_composer_2d` 裡三個函式各寫一次 `x_start_m=0.10`，
把它升成具名常數（值不變，16 個 composer 測試照過）。
要把越障放進世界的人，需要的就是那個數字本身，不是它的第四份拷貝。

在 segment 的**進入**與**離開**各算一次。放得進排程的話，兩次會一樣、四隻腳也會一樣。

### 結果：差了一公尺以上，而且每隻腳自己就前後矛盾

```text
越障 segment                8
一個障礙物同時符合它們嗎      False（容忍值 1.0 mm＝pipeline 自己的接觸容忍值）
隱含位置的散佈             1045.3 mm
同一隻腳自己前後矛盾        436.2 mm （它的 ascent 對它自己的 descent）
單一 segment 內就漂移       276.2 mm
最差 body-advance 比值      17.3x
規劃時用的平台在 1000 mm，最近的隱含障礙物差 349 mm
```

漂移的來源是一句可以直接檢查的話：
**這條 swing 的座標系假設 body 在它進行中前進 293 mm，排程只給了 17 mm。**

這正是 **Step 4 那個 `2.000x` 的距離版本**——Step 4 量的是時間放不下，
這裡量的是同一件事在空間上的樣子，而且距離版嚴重一個數量級。
兩個要一起解，不能各自放寬。

### 註冊順手抓到第二件事：沒有任何一隻腳站上去過

```text
在障礙物頂面【站立】的總時間 = 0.000 s
LF  SWING_UP    0.394 - 0.497 s   接觸高度  0.0 -> 40.0 mm
LF  SWING_DOWN  0.497 - 0.600 s   接觸高度 40.0 ->  0.0 mm
```

觸地與離地是**同一個瞬間**。composition frame 裡 260 mm 落地、420 mm 起跳，
中間那 **160 mm 的頂面移動**就是 **Step 7 沒解出來的 `TOP_REPOSITION`**。

那 160 mm **本來就有人記錄過**——`LegPlan2D.notes` 上寫著
「`160 mm of top is crossed by a segment no planner in this project can yet
generate`」。附錄 B 加的不是那個缺口，而是它的**後果被量成一個數**：
頂面站立時間 0.000 s，而且是**同一個瞬間**觸地又離地。
一個「已知的缺口」與「這條軌跡因此從來沒有站上去過」不是同一句話。

所以這條軌跡「越過」障礙物的方式是：**跳上去、在同一瞬間跳下來**。
這不是取樣不夠密，是排程裡真的沒有那一段。
`stance_on_top_seconds_2d` 把它做成一個**會回報 0.000 s 的量**，
而不是一個沒有人去找的缺席。

### 所以圖上沒有平台

```text
畫在 1000 mm（規劃時用的平台）  腳根本不去那裡，差 349 mm
畫在任何一個隱含位置            另外七個 segment 不同意，差最多 1045 mm
```

畫出來的是**每一條 swing 自己宣稱的那個面**：紫色虛線的 `+40 mm`，下面什麼都沒有。
那條線是整張圖上唯一還記得障礙物存在的東西。

### 兩個實作決定

1. **`float_gap_m` 改成相對【那隻腳自己的面】**，不是相對 z = 0。
   demand 的定義是「這隻腳的腳掌剛好碰到它自己那個面時的 body 高度」，
   所以 `chosen - demand` 本來就是「腳離它自己的面多高」。平地上兩者相同，
   所以附錄 A 的圖一格都沒有變。
2. **airborne 的越障腳也要畫面**。原本只有站立腳畫，結果整張圖上看不到 40 mm——
   因為**沒有一個站立取樣在頂面上**。改成 swing 也畫它「要落上去 / 剛離開」的那個面。

### 產出

```text
程式  hybrid_note/scripts/experiments/day12_obstacle_registration_2d.py      新增
      hybrid_note/scripts/experiments/day12_obstacle_animation_driver.py     新增
      hybrid_note/scripts/experiments/day12_obstacle_notebook_section.py     新增
      day12_whole_body_animation_2d.py    加 planned_surfaces_2d / PlannedSurface2D
      day10_11_composer_2d.py             0.10 -> COMPOSER_FRAME_X_START_M（值不變）
      tests/test_day12_obstacle_registration_2d.py   新增，10 passed
資料  day12/day12_obstacle_registration.csv     每個越障 segment 的隱含障礙物
      day12/day12_obstacle_viewing_height.csv
圖    day12/day12_obstacle_registration.png     全部隱含位置畫在同一條世界 x 軸上
      day12/day12_obstacle_frames.png           越障前後四格
      day12/day12_obstacle_animation.gif        整趟 4 cm run，61 幀
展示  notebook 的「附錄 B」節（B.1 - B.6）

重畫（約 70 秒；地形是參數）：
  python3 -u LegWheel/hybrid_note/scripts/experiments/day12_obstacle_animation_driver.py
  python3 -u .../day12_obstacle_animation_driver.py --height-mm 100
```

### 交給 Day 13-14 的三件事

```text
1. 越障 swing 的 body-advance 需求要變成一個【排程約束】（現在 17.3x）
   與 Step 4 的 2.000x 是同一個問題的兩側。
2. 需要頂面那一段（Step 7 的 TOP_REPOSITION）
   沒有它，「越障」是兩個瞬間相接的 swing，中間 160 mm 沒有人規劃。
3. 世界 x 註冊要變成 pipeline 的一部分
   terrain 現在只走到 decide / compose。要讓四隻腳越過【同一個】障礙物，
   body_x 與越障段的座標系必須共用原點——這是一個建模決定，
   與 Step 3 決定時間軸同一種性質。
```

---

# 3. 下一步：Step 12 — Day 12 Freeze 與 Day 13-14 Handoff

規格 §19。這是 Day 12 的最後一步：把做完的東西凍結，並寫出交給 Day 13-14 的清單。

## 3.1 Step 12 要凍結的東西已經全部就位

```text
Step 0   segment 語意契約（SegmentKind 現在有 12 個成員）
Step 1   nominal cycle 生成器
Step 2   四腳世界座標 + 參數化地形註冊
Step 3   時間軸（duty 0.75，10.553 倍的角速度需求）
Step 4   per-leg sequence -> 共同時間軸（含 airborne overrun 偵測）
Step 5   body requirement 合併（結論：INFEASIBLE）
Step 6   support triangle + margin（結論：五個 swing 全 unstable）
Step 7   TOP_REPOSITION（結論：planning floor 下 support 不足）
Step 8   完整四腳軌跡
Step 9   whole-body validation（11 檢查 7 過 4 失）
Step 10  唯一入口 + generalization gate（四地形都跑得出結果）
Step 11  paper metrics（兩條紅線都機器化）
```

## 3.2 交給 Day 13-14 的，是一份【待辦清單】不是一份成功報告

Day 12 的正式完成條件（規格 §17）是
「產生同步四腳軌跡**或**結構化的不可行結果，且不含地形尺寸專用邏輯」——
**這一條達成了**。但**沒有任何一個地形是 feasible**，
所以 handoff 文件必須把這兩件事分開寫清楚。

七個未解結論（每一個都有量化數字，見 §2）：

```text
Step 4  時間放不下      0.6 s 的窗口要裝 1.2 s（2.000 x）
Step 5  高度對不起來    三隻站立腳差 15.329 mm -> INFEASIBLE
Step 6  沒有餘裕        margin 最小 0.000 mm（對稱造成，不是捨入）
Step 7  撐不住          兩個 TOP_REPOSITION 都在 support gate 停住
Step 9  beta guard      超出舊 planner 的 ±40 deg（可能是語意不合）
Step 9  判不了          沒有關節速度極限，10.553 倍無法裁決
Step 10 資料缺口        19 cm 從來沒有被掃過
```

## 3.3 Day 13-14 拿得到的具體待辦

```text
(a) 補一個關節速度極限常數  -> 才能裁決 10.553 倍
(b) 掃 19 cm 的 rolling traversal -> 才能回答 challenge terrain
(c) 決定 beta 的語意        -> 圈數計數器 vs 有界擺動，兩個 planner 要一致
(d) gamma 調整（規格自己指的路）-> 改善 support margin
(e) body heave / pitch 或 per-leg theta 補償 -> 解 Step 5 的高度衝突
(f) 把獨立生成的 nominal run 接起來 -> 解陷阱 25 的 297 mm chain break
```

## 3.4 規格 §19 的要求（實作前先讀原文）

# 4. Day 12 已知陷阱（做下去之前一定要知道）

> Day 10-11 的 45 條陷阱**全部仍然適用**，見
> `../day10-11/day10_11_implementation_log_zh_TW.md` §4。
> 下面只列 Day 12 新增的。

1. **一個 boundary 有兩種，不要用同一組門檻。**
   `CUT`（同一條 run 的相鄰 frame）的「跳躍」是一步真實滾動，Day 6-7 實測
   hip 最大 6.62 mm、beta 最大 1.75 deg。`HANDOVER`（兩個獨立產生的運動相接）
   才是「腿有沒有瞬移」的問題。第一版用固定 2 mm hip 門檻，
   Day 6-7 自己的 traversal 5 個 boundary 過不了——**是門檻錯，不是 traversal 錯**。

2. **`CUT` 的 hip 跳躍【不要】另外設門檻。**
   腳踩在面上時 `hip_z` 與 `theta` 互相決定（Day 10-11 陷阱 43），
   關節只動一步、hip 就只動一步的量。再加一個 hip 門檻不會多出資訊，
   只會多出一個沒人量過的常數。

3. **`MotionSequence2D` 不能裝 Day 12 的 chain。**
   它強制所有 segment 共用一個 `frames.source_id`。這對「從一條 traversal
   切出來的 sequence」是**真正的保證**，Step 7 靠它。不要為了 Day 12 放寬它，
   用 `SegmentChain2D`。同一個來源之內 index 唯一那條規則**仍然成立**，別一起拿掉。

4. **`FOOT_RIM_ROLL` 不准滾 left rim，這【不是】「Day 12 不能滾 left rim」。**
   那是 `POST_TOUCHDOWN_ROLL` 的工作。把拒絕訊息單獨引用出去會變成一個
   沒有人主張過的結論（Day 10-11 §0.0 講的就是這件事）。

5. **`day6_7` 的 `theta = 60 deg` 是 `theta_climb`，不是平地 nominal。**
   見 §3.1。它是為了爬 0.15 m 障礙挑的爬升參數。

6. **改 `SegmentKind` 必須同時更新 `SEGMENT_SEMANTICS`。**
   有一個測試 (`test_the_semantics_table_covers_every_kind`) 會擋下漏掉的情況，
   而且會檢查「nominal 只有一個、pins_theta 只有一個」。

7. **`write_rows_csv(path, rows)` 的參數順序是 path 在前。**
   （Day 10-11 陷阱 21 是另一件事：它用第一列的 keys 當表頭，
   分段輸出時要先取欄位聯集。）

8. **driver 需要自己插 `sys.path`。**
   `_ROOT = Path(__file__).resolve().parents[3]`，照 day10-11 的 driver 寫法。
   不然從 repo root 執行會 `ModuleNotFoundError: No module named 'hybrid_note'`。

9. **`_flat_roll_template` / `_solve_flat_roll_rotation` 只會【往前】走。**
   它要求新的支撐 sample 大於舊的，往回呼叫第一步就回傳 `None`。
   任何「往回滾找起點」的寫法都會無聲地失敗成「起點就是現在」。

10. **foot 弧在 `beta ~= 40 deg` 之後還有 20 度，但那是 pivot 不是 roll。**
    支撐 sample 釘在 index 0（foot/left 接縫的角），`alpha` 釘在 −40 deg，
    接觸點不動，hip 卻下沉 28.6 mm。用「`contact_regions` 還是 foot_rim」
    找弧的起點會找到 pivot 的盡頭，白付那 28.6 mm 而**滾動距離一點都沒有多**。
    正確判準是 `rim_alpha_limits_rad(RimId.FOOT)` 的下界第一次被碰到。

11. **airborne 的 clearance 門檻只能套在 rotate 相位。**
    liftoff 與 touchdown 兩側的 frame clearance **依定義為 0**。
    對整段 airborne 套同一個下限，第一個 cycle 就會失敗，
    而且那是門檻錯不是軌跡錯。ramp 要的是「不穿透」與「單調接近」，
    不是「保持距離」。

12. **不要假設 recovery 期間 hip 一定要抬。**
    foot 弧對 `alpha = 0` 對稱，整圈 cycle 的兩端 hip 高度相同（202.161 mm），
    ramp 剛好是 0。被截短的 stroke 反而要**降** −12.7 mm。
    我第一版在 docstring 裡寫了「必須抬 17.3 mm」——大小和方向都錯。

13. **`theta_compact` 與 `WHEEL_MODE_THETA_RAD` 是【兩個不同的量】。**
    MVP 裡都是 17 deg，但一個是「`WHEEL_ROLL` 的定義」，
    另一個是「腿縮多少才閃得開」。共用一個名字會讓這個巧合變成永久的。
    規格 §0.2 明說 17 deg 只能是 configurable recovery parameter。

14. **`STAND_HEIGHT` 不是「the」nominal posture。**
    `trajectory_planning` 用 0.30、`gait_generator` 用 0.25、`gait_generator_3d` 用 0.31，
    而且換算慣例還不同（一個減 `foot_radius`、一個加 `ABAD_AXIS_OFFSET`），
    這個 2D 模型兩個都沒有。挑一個叫作 nominal 等於虛構一個不存在的共識。
    這條 2D pipeline 實際一致使用的是 `theta = 60 deg`
    （Day 6-7 approach 與 Day 8-9 `REGRESSION_THETA_RAD` 都是它）。

15. **notebook 現在要跑約 5 分鐘**（Step 1 現場產生 cycle ＋ 跑 pytest）。
    用 `nbclient` 執行時 timeout 要設大（2400），不要沿用預設。

16. **sagittal 平面的橫向位置是 `BODY_WIDTH/2 + WHEEL_AXIAL_OFFSET`，不是 `BODY_WIDTH/2`。**
    211.675 mm 對 120 mm。腿平面在**輪子中平面**，不在 ABAD 軸上。
    不要自己從 `RobotParams` 重推 mounting，用
    `CorgiLegKinematics._get_transformation_matrices` 把 `p_L = 0` 推過去。

17. **`ABAD_AXIS_OFFSET` 是腿平面在 body 原點【上方】的高度。**
    所以 `body_z = hip_stance_z - 57.166 mm`，不是等於。

18. **對稱檢查裡 `y` 要比「和為零」，不是比「相等」。**
    比相等的話，「兩隻腳都在同一側」的機器人會**通過**檢查——
    那正是這個檢查要抓的錯。

19. **`LegId` 改名不改號。** 規格叫 LF/RF/LH/RH，專案叫 `0:FL 1:FR 2:RR 3:RL`。
    `LEG_ORDER`（LF RF LH RH，用於閱讀）與 `joint_position_rad` 的列索引
    （0..3，專案的編號）**不是同一個順序**——直接用閱讀順序填矩陣會把兩隻後腳對調。

20. **`is_symmetric` 與 `all_in_contact` 是兩個分開的問題。**
    跨在平台前緣的站姿是**對稱的**，但**不合法**（前腳穿透 40 mm、`collision=True`）。
    合成一個旗標會讓其中一個問題消失。

21. **「沒被排程」不等於「在空中」。**
    phase offset 就是時間位移，所以四隻腳的 chain 不同時開始，整段 span 頭尾
    會有腿還沒被排進去。在那裡數支撐腳會得到假的失敗。
    四腳約束只能在 `covered_interval_s` 上評估，頭尾用 `ragged_intervals_s` 分開報。

22. **`swing_leg_at()` 在多腳同時空中時【丟例外】，不回傳第一隻。**
    這是故意的：Step 4 放 terrain transition 進來時就會踩到，
    默默回一隻腳會讓衝突變成看不見。要問「有幾隻」請用 `airborne_legs_at()`。

23. **Step 3 的每一個 `duration_s` 都是【指定】的，不是量的。**
    上游整條 pipeline 是準靜態、`duration_s` 全 `None`。
    `ScheduledSegment2D.duration_is_assigned` 永遠是 `True`，就是為了擋住
    下游把它讀成觀測值。10.553x 這個比值同理——它是**要求**，不是實測的能力。
    它有沒有超過關節速度極限是 **Step 9** 才回答的問題。

24. **Step 3 的窗口規則是【除法】，不是檢查。**
    「一個空中 run 配一個 swing 窗口，段內按 frame 數分」永遠塞得下，
    所以 `conflicts` 為 0 **不代表**動作放得進去。要另外用
    `airborne_overruns_2d()` 量，而且要用 planned duration 當尺——
    frame 數是取樣選擇，beta 掃掠角會被 recovery 的 280 deg 淹掉。

25. **前後兩段 nominal run 天生接不起來。**
    `build_leg_plan_2d` 的 before / after 是**各自獨立生成**的，
    都從同一個初始姿態出發，所以就算中間完全沒有越障，
    `breaks` 也會是非空。那是「這兩段還沒被接起來」的正確回報，
    不是 bug；要無縫就得從前一段的結束狀態接著生成。

26. **`compose_2d` 會跑 planner，但那【不是】重做決策。**
    `decide_2d` 是純查表。真正被禁止的是「自己寫一個規則挑 ROLL 或 SWING」。
    代價是時間：`#1 ROLL_ROLL` 要跑約 3 分鐘（driver 用 `--with-roll-roll` 才跑）。

27. **`pkill -f day12_step4_driver` 會把自己殺掉。**
    pattern 會match到執行這個指令的 shell 自己的 command line，
    結果是 exit 144 而且看起來像「程式莫名其妙掛掉」。
    要停背景工作請用工作 ID，不要用 `pkill -f` 配這種字串。

28. **`PINNED` 的高度在 `hip_z_min_m`，不在端點 contact 裡。**
    欄位名字看起來像下界，但在 PINNED 上它是**值**。schema 會擋
    （`a pinned requirement is a height; it needs hip_z_min_m`）。
    而且 PINNED 只有**端點**是 hard，內部不是（Day 10-11 Step 9 C 節）。

29. **infeasible 的瞬間不可以有高度。**
    不是中點、不是 nominal、也不是上一個好的值——就是 NaN。
    因此所有摘要（`body_z_travel_m` / `max_body_z_step_m` / min / max）
    都要跳過 NaN，否則一個 infeasible 會把其他取樣的紀錄整個抹掉。

30. **平地四腳 nominal cycle 本來就不可行，這是結論不是待修的 bug。**
    foot-rim 滾動的 hip 高度是弧（起伏 17.287 mm），
    Walk 的相位把三隻站立腳放在弧的不同位置。
    任何「把它調到可行」的動作都等於偷偷平均，規格 §12 要求 5 明令禁止。

31. **scratchpad 會被清掉。** 產生 notebook 的 `build_day12_nb.py` 放在
    `/tmp/.../scratchpad/` 而且真的消失過一次。
    現在改成「直接往 .ipynb 追加、以 cell metadata 的 tag 做 idempotent」，
    腳本放在 repo 裡：`day12_step5_notebook_section.py`。不要放 scratchpad。

32. **邊界瞬間屬於誰：segment 擁有 `[start, end)`，最後一段擁有自己的結尾。**
    第一版讓 airborne 優先，結果 swing 的**結尾**（= touchdown）被判成還在空中。
    另一個方向也錯：讓 stance 優先的話，起跳瞬間會多一隻支撐腳。

33. **窗口邊界有浮點誤差。** 邊界是 `phase_offset * period` 的和，
    同一個瞬間一隻腳算 `0.6`、另一隻算 `0.5999999999999999`。
    沒有容差的話會少一隻支撐腳，看起來像步態壞掉。
    `BOUNDARY_TOLERANCE_S = 1e-6` —— 遠低於任何真實時間，遠高於誤差。

34. **support triangle 要以 swing leg 為準，不要每個瞬間重問誰在空中。**
    規格寫的是「另外三隻腳」，那是**整個 swing 固定**的一組。
    每個瞬間重問的話，最後一個取樣會換成下一個 swing 的支撐組，
    等於回報一個從來沒被測試的三角形。取樣也因此要用**半開區間**。

35. **`points[:3]` 是會說謊的寫法。** 四個接觸點被默默算成三角形。
    四個點是排程結果（有腳沒抬起來），要 `raise` 出來，不是幾何問題。

36. **margin 為 `None` 不等於 margin 沒問題。** 退化三角形、支撐腳不足三隻，
    都回 `None`，而 `is_stable` 對 `None` 一律是 `False`。
    「不知道」和「沒事」不是同一件事。

37. **margin 的 0 是對稱造成的，不是捨入。** 但實際算出來是 `2e-14` 這種數字，
    所以**不要**用 `margin > 0.0` 當門檻——那會被算術雜訊放行。
    測試用 1e-6 m 當底線，說的是「要有餘裕，不是只要非負」。

38. **對著門檻瞄準會失敗。** touchdown 的 theta 是 IK 的**輸出**不是請求的輸入
    （Day 10-11 陷阱 16）。請求 35 deg 會解出 34.99955 deg，差 0.00045 deg 被判不合格。
    **請求**要瞄在門檻之上（`TARGET_THETA_HEADROOM_RAD`），**檢查**才用門檻本身。

39. **`theta = 17 deg` 的站姿仍然是【腳輪緣】接觸。**
    `LEFT_RIM_READY` 要的是**左輪緣承載**，那不是把 theta 調小就會發生的事，
    需要特定的 beta。用 `left_rim_beta_window_2d` + `choose_landing_beta_2d`
    + `left_rim_landing_scene_2d`（Day 10-11 已經有）。

40. **`resolved = True` 不能寫回 `TransitionRequirement2D`。**
    schema 自己擋：「a resolved transition is a segment, not a requirement」。
    產出要是一個**解決紀錄**，同時放原始 requirement 與生成出來的 segment。

41. **新增 `SegmentKind` 一定會撞到兩個測試**，那是設計出來的。
    `tests/test_day10_11_motion_schema_2d.py` 與
    `tests/test_day12_segment_contract_2d.py` 都在斷言
    terrain-transition swing 的完整集合。撞到就更新斷言，
    但**先想清楚新 kind 是不是真的需要**——借用別的 kind 會污染規格 §18 的計數。

42. **`left_rim_beta_window_2d` 很慢：0.5 deg 一步要跑約 10 分鐘**
    （每一步建一整條 721 點的腿）。
    `resolve_top_reposition_2d(landing_beta_step_deg=...)` 可以調粗，
    測試用 2 deg；**driver 保持 0.5 deg**。調粗要在測試裡寫明用了哪個網格。

43. **`beta` 是圈數計數器，全樹都不 wrap。**
    recovery 依 `beta_target = start.beta - 2*pi` 建出來，
    所以交接處 raw 差 360 deg 是**一整圈**，不是不連續。
    只報 raw 會把整圈說成災難；只報 wrapped 會把整圈藏起來。**兩個都要報。**

44. **量測函式不要寫第二套（這一天犯了兩次）。**
    `handoff_between_2d`（Step 0 抽出來的）與 `segment_at`（Step 6 的，
    含邊界容差與半開區間規則）都被重寫過一次，而且第二套都在邊界上出錯。
    要用就 import，不要重打。

45. **`covered_interval_s` 的最後一刻沒有完整的四腳組態。**
    一隻腳的 chain 剛好用完、另一隻的 swing 已經開始。
    取樣要用半開 `[lo, hi)`——Step 6 對 swing 已經是這樣做的。

46. **rim geometry gap 報 0 不等於 1.2 mm 不存在。**
    foot rim 上依定義是 0，1.2 mm 是 upper tyre 上的值。
    任何報 0 的地方都要附上「這一趟沒離開 foot rim」這句話，
    否則下一個人會以為問題消失了。

47. **「沒檢查」「在別處檢查過」「檢查過而且通過」是三件事。**
    Step 9 用 `CheckId`（這裡跑的）、`DELEGATED_CHECKS`（別處跑的，附上是誰）、
    `UNEVALUABLE_CHECKS`（根本判不了，附上缺什麼）三個分開的容器。
    把後兩者省略掉，讀起來就會像全部通過。

48. **`RobotParams` 沒有關節速度極限。**
    `SWING_ACCEL_MAX` 是 swing 塑形、`TOUCHDOWN_VEL_H_MAX` 是觸地，
    兩個都不是馬達轉速上限。所以 Step 3 的 10.553 倍需求
    **量得到、比不到**。要裁決就得先補這個數字。

49. **`BETA_MAX_DEG = 40` 與 Hybrid 的 beta 語意不相容。**
    舊 planner 把 beta 當有界擺動，Hybrid 當圈數計數器。
    直接套用會讓整個 nominal cycle 違規；直接忽略又會漏掉真正的工作區問題。
    正確做法是**單獨列一個檢查並在 detail 裡寫明這個歧義**。

50. **Step 8 的端點內插會讓 recovery 的 theta 縮腿【消失】。**
    量出來的 peak theta rate 是 0，那是**下界不是需求**。
    任何從組好的軌跡讀 theta 動態的分析都要先知道這件事。

51. **測試裡比較含 `nan` 的資料要用 nan-aware 比較。**
    Step 5 之後大部分 body_z 都是 `nan`，直接 `==` 會讓「東西沒被改動」
    這種測試報自己的假失敗。

52. **planner 裡不該出現的尺寸，最容易從「取一個參考值」溜進來。**
    這次是為了拿 nominal body height 而蓋了一個 4 cm 平台。
    `planner_size_literals()` 就是為了機器化地抓這件事而存在的——
    寫完新模組要跑它一次。

53. **allowlist 一定要附書面理由，而且要斷言它還在。**
    `0.10 * cycle_period_s` 是週期比例不是長度，可以放行；
    但如果那一行哪天消失了，allowlist 就變成一個永久豁免、
    也就是一個被消音的檢查。測試要同時擋這兩邊。

54. **`body_z_travel_m` 跳過 NaN，所以它可能回一個【假的 0】。**
    只剩一個有限取樣時 travel = 0，會被讀成「body 完全沒起伏」，意思正好相反。
    任何拿它做報告的地方都要先看 `usable_body_samples`。

55. **「資料沒有」不等於「物理不可能」。**
    19 cm 的拒絕理由是 `the rolling traversal was never swept at this height`。
    把它寫成「19 cm 做不到」會是造假——它指出的待辦是「去掃那個高度」。

56. **regex 檢查程式碼時，`[<>=]=?` 會把賦值也算進去。**
    `height_m = 0.0` 被當成對高度的分支。要比較就只寫比較運算子。

57. **「任何一隻腳在這個 kind」的時間指標永遠等於整段時長。**
    任何時刻都同時有腳在滾、有腳在空中，所以那種 wall-clock 總和量不到東西。
    要用 **leg-seconds**（四隻腳分別計時再加總），而且欄位名要叫 `*_leg_seconds`——
    叫 `time_s` 會被讀成牆上時鐘。

58. **距離歸屬會重疊，不可以相加。** body 前進是共用的，
    不同腳可以同時處在不同 kind。要嘛明講規則並標明重疊，
    要嘛就不要報。偷偷當成 partition 會得到大於總距離的和。

59. **禁止某件事，就要寫一個會叫的 guard。**
    §18 禁止推導 COT，所以有 `energy_vocabulary()` 掃程式碼；
    而且**另外有一個測試用假檔案證明它真的會叫**——
    不會叫的 guard 只是一句寫在註解裡的期望。

60. **測試不要比對會換行的 docstring 片語。**
    `"must not be summed"` 在原始碼裡被折成兩行，直接 substring 比對
    測到的是行寬不是措辭。先 `" ".join(doc.split())` 正規化。

61. **不要對「階梯訊號」做差分——而且先問階梯本身該不該存在。**（A8，見 §1.6）
    `leg_sample_at` 原本用最近的 frame 取值，所以重取樣出的 θ/β 是階梯。
    對它差分得到的是「frame step ÷ 取樣間隔」——refine 網格數字就變大，
    同一條軌跡 241 取樣讀 48.1%、1921 取樣讀 127.7%。
    **自我檢查法：把取樣數加倍，如果答案變了，那就不是物理量。**

    但真正的教訓在後面：這個階梯**不只讓量測錯，它讓輸出不可執行**。
    匯出的 CSV 裡有整段相同姿態夾一個 4° 跳階，以 80.33 Hz 播放
    等於要求機器人在一個播放週期內走完 4°。
    **看到假的量測值時，先問「被量的那個東西本身對不對」**，
    不要只換一個量法就算了——換量法會讓數字變好看，缺陷還在輸出裡。

62. **「這個參數不影響任何結果」這種註解要定期重驗。**
    `walk_timing_2d` 的 docstring 說 cycle period「Nothing below depends
    on its value」——當時（Step 3）是真的，加入馬達預算之後就錯了，
    馬達速率正比於 1/period。註解會過期，而且過期得很安靜。

63. **margin = 0 要先問「是不是臨界值」，再問「是不是 bug」。**
    四足 wave gait 的縱向裕度正比於 `duty − 3/4`，在 3/4 恆等於零。
    專案的 `stance_duty` 正好是 0.75。花時間找幾何 bug 之前，
    先確認量到的不是某個已知公式的定義值。

64. **零裕度的浮點門檻要配合幾何尺度，不是配合 float 精度。**
    支撐點由半公尺尺度上的求根解出，殘差落在**奈米**量級
    （−6.86e-9 m）。用 `abs=1e-9` 斷言會失敗；1e-6 m 才對
    ——它比最小的非零 margin（1.33 mm）低四個數量級。

65. **說一個安全門檻「沒依據」之前，先把它換算成物理量。**（見 §1.6 A4-7）
    我一度說 10 mm 的 margin floor「沒量過、構不到，所以該被交代」。
    換算之後才發現它等於「容忍 10 mm 的重心誤差」——這對真實機器人
    是**合理的工程要求**。真正的結論因此反過來：不是門檻太嚴，
    是**這個步態的餘裕本來就薄**（7.26 mm 的重心偏移就歸零）。
    **把門檻換算成它在管的物理量，再判斷它合不合理。**

66. **字典序的最後一位很可能是死碼。**（B1，見 §1.7）
    `ROLL_PREFERENCE` 這張表存在、正確、而且**從來沒被執行過**——
    它排在 `body` 與 `margin` 後面，而前面兩項幾乎不可能剛好相等。
    **寫了一個 tie-break，就要驗證它真的會觸發**，
    否則它只是一句寫在程式裡的意圖。
    加容忍帶讓它會觸發時，還發現 `margin` 會先攔截——
    順序的每一位都要檢查，不是只看第一位。

67. **三態（feasible / infeasible / not measured）的重點是不要混，
    而我自己在排查裡混了。**（B3）
    我把「在我看的那兩個尺寸剛好不可行」寫成了「從來沒被量過」。
    這兩件事在規格裡被刻意分開，就是因為混了會把
    **實驗缺口**說成**物理限制**，或反過來。
    寫「從來沒有 X」之前，先跑全圖統計，不要只看手邊那兩格。

68. **一個「每步」的門檻不可能是重取樣訊號的極限。**（§1.8）
    `joint_continuity` 拿 `max(|Δθ|,|Δβ|)` 比固定 30 deg——
    網格加密一倍步長就減半，所以同一條軌跡在 61/121 取樣失敗、
    181/241 通過。**極限要寫成速率**，速率才是軌跡的性質。
    自我檢查法（同陷阱 61）：把取樣數加倍，答案變了就不是物理量。

69. **收窄一個「豁免條件」跟放寬一個「限制」一樣危險。**（§1.8）
    我把 `is_segment_boundary_frame` 從「最近的幀是首/末幀」
    收成 `blend == 0.0`，離地瞬間那一幀就不再被豁免，
    於是 `stance_contact_valid` **只在細取樣**失敗。
    改條件時要問的不只是「它現在會不會誤放」，
    還有「它原本在豁免誰、那個人現在怎麼辦」。

70. **測試寫死當下的答案，會在規則改變時因為【錯誤的理由】失敗。**（§1.8）
    `test_both_obstacles_use_the_same_primitives_chosen_by_day_10_11`
    的用意是「Step 10 不自己決定」，卻寫死 `is StrategyId.SWING_SWING`。
    規則一改它就紅，但紅的不是它要守的東西。
    **斷言不變量（對照 `decide_2d` 自己的答案），不要斷言當時的數值。**
    八個失敗測試裡有七個是這一類。

    **【2026-09-03 追記】同一個測試又犯了一次**（§1.18）：
    改成斷言「越障不改變 nominal recovery 數」，
    世界註冊讓它從 8 變成 18。
    **判斷方法：問「哪一種設計改動會讓這個斷言變？」
    答得出來，它就不是不變量。**

71. **「同一個東西」不能靠「同一份資料」來表達。**（A5，見 §1.9）
    `build_leg_plan_2d` 給四隻腳**完全相同的鏈**（14 段，起點都是 0.000 mm），
    再讓它們在不同的時間窗執行。那不是「四隻腳越過同一座障礙物」，
    是「四隻腳各自越過自己的一座」——implied 位置因此散佈 753.8 mm。
    **共用的實體要用共用的座標表達，不是用共用的複本。**

72. **先驗證假設再開始修。**（§1.9）
    我認為 C5（hip_advance = 0）是 A5 的根源，量下去發現
    設成正確值之後註冊指標**一個小數位都沒變**。
    如果先動手改，會得到一個「修好了但沒有效果」的改動，
    而且會改掉一堆既有數字。**一個廉價的量測擋掉了一個昂貴的錯誤。**

73. **`delivered = 0` 要問「是沒動，還是沒資料」。**（A9）
    RF/LH 的 `ROLL_DOWN` 排在 body 軌跡涵蓋區間之外，
    位移於是靜靜地變成 0.000 mm，看起來像「這段不前進」。
    **超出資料範圍要報錯，不要回傳零。**

74. **先 profile，再優化——即使直覺很強。**（§1.12）
    我猜了兩次「這裡看起來很慢」，兩次都改對了、都有測試，
    **兩次加起來只拿到 17%**。跑 profile 之後才發現 92% 的成本
    在一個完全沒想到的地方（地形查詢的驗證）。
    改完 2.05x。**兩次正確但無用的優化，成本高於一次 profile。**

75. **用 numpy 對兩個純量做運算，比純 Python 慢兩個數量級。**（§1.12）
    `np.array([a,b])` 每次約 1 µs、`np.clip(純量)` 3.8 µs、
    `np.isclose(np.linalg.norm(v),1)` 7.3 µs，而同樣的數學用
    `math` 是 0.13 µs。在 133 萬次呼叫上，這是 13 分鐘與 2 分鐘的差別。
    **numpy 是給陣列的；2 元素不是陣列，是兩個數字。**

76. **重構效能時，先產生 golden 再動手。**（§1.12）
    改完之後才想產生對照資料就來不及了——舊行為已經沒了。
    而且 golden 檔**本身要有一個測試斷言它涵蓋了所有分支**，
    否則漏掉分支的回歸檔是假的綠燈。

77. **「更準」有時候比「一樣」差。**（§1.12）
    `math.hypot` 比 `sqrt(dx*dx+dz*dz)` 準，差一個 ULP。
    但至今所有凍結的數字都是用後者算的，
    **能斷言「逐位元相同」比能宣稱「更準」有價值**。
    選了相同，並把理由寫在原始碼裡。

78. **「先量再走」比「先算再建」可靠。**（§1.11、§1.16）
    `run_approach_2d` 前兩版都先算 `whole_strokes` 再照著建，
    **兩次都算錯**（一次拿 stroke 的前進當 cycle 的，一次撞上
    「單一 stroke 關不掉的餘量」）。改成貪婪走——
    還放得下就再走一個，剩下的關掉——之後就不會錯了。
    **必須事先算對的算術，能變成停止條件就變成停止條件。**

79. **一個函式的「預設目標」可能對它的新用途是錯的。**（§1.16）
    `recovery_beta_target_2d` 瞄準 `stroke.start.beta − 2π`，
    對**完整** stroke 那是弧的起點，對**中途起步**的 stroke 那是弧的中間。
    水平化姿態下落在弧中間根本不是有效接觸，四隻腳全部失敗。
    **把隱含的預設改成明確的參數，然後在新用途上明確傳值。**

80. **接續一段動作時，要問「上一段把腿留在哪」，不是「上一段是什麼」。**（§1.16）
    我用「最後一個 cycle 的 stroke」當過渡起點，
    但那個 cycle 的 **recovery** 已經把腿又往前帶了 57.5 mm。
    時間窗因此重疊，排程直接拒絕（拒得對）。
    **「上一段」指的是最後執行的那一段，不是最後一個同類的段。**

81. **被既有檢查擋下來的錯誤，是檢查在做它的工作。**（§1.16）
    這一輪三個錯誤分別被 `beta_step_rad must be finite and non-zero`、
    `TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`、
    `LH has overlapping segments` 擋下。**每一個都拒得對**，
    而且每一個都直接指向真正的成因。
    寫這些檢查的當下看起來像額外工作，回收是在這種時候。

82. **「沒有模型」不等於「無法量」。**
    專案沒有質量模型，但敏感度是可以先量的：
    把未知參數當成掃描軸（腿質量佔比 f、車體重心偏移），
    量出「結果隨它怎麼變」。結果是腿的質量幾乎不影響（<1.2 mm），
    車體重心偏移才致命（7.26 mm 歸零）。
    **這把一個開放式的「缺模型」變成一個具體的、只差一個數字的問題。**

83. **「量到了」不等於「接上了」。**（§1.19）
    §1.15 在 scratch 腳本裡量出位置排程、寫成三個函式、記了一整節，
    然後**沒有任何呼叫端**。`grep` 一次就看得出來：
    `world_schedule_2d` / `body_speed_m_s` / `swing_hip_advance_m`
    在自己的模組以外零次出現，測試也零次。
    **一個修法做完，要 grep 它自己的名字。**

84. **同一個預設值，換了排程規則就換了語意。**（§1.19）
    `RecoveryConfig2D.hip_advance_m = 0` 在「時間由段落索引決定」時，
    是一句無害的宣告（Step 1 不模擬車體）。
    在「時間由位置決定」時，它變成一個**主張**：recovery 不佔時間。
    duty 於是實際上是 1.0，不管 `timing` 寫什麼。
    **改排程規則的時候，要重新讀一次所有預設值的意思。**

85. **要停在某個位置，就做出來量，不要用平均比值換算。**（§1.19）
    `max_distance_m` 限制的是接觸距離，要停短的是髖；
    1.610 是整條 stroke 的平均比值，弧上會漂；
    而且 stroke 停在「第一個到達或超過請求」的那一步，本來就會多走最多一步。
    三件事疊起來是 0.553 mm，剛好夠讓一整隻腳被拒絕。
    （這是陷阱 78「先量再走」的第二個實例，換了一個位置又發生一次。）

86. **相位前面的符號是可以量的，不要用猜的。**（§1.19）
    平地排程自己就寫著：`LF phase 0.75 -> swing 0.0486 s`、
    `LH phase 0.00 -> swing 1.8085 s`——**相位越大，swing 越早**。
    我第一版寫成 `+ phase`，四隻腳照樣差四分之一週期、
    每一個數字看起來都對，但**步態順序是反的**。
    要抓到它，測試必須比**環狀順序**（誰接在誰後面），不是比誰第一個、
    也不是比間隔是不是 0.6 s。

87. **「相位」不只是 beta。**（§1.19）
    水平姿態在每個 beta 都用 `hold_hip_z` 反解 theta，
    所以弧中間的 theta 和弧起點的不一樣（實測跨相位差 > 1 度）。
    拿弧起點的 theta 配相位的 beta 去瞄一個過渡，
    瞄的是一個**不在 stroke 上**的姿勢。
    相位要當成**姿勢**（theta, beta, hip_z）交出去，不是當成一個角度。

---

# 5. 檔案索引

```text
規格        hybrid_gait_day12_whole_body_integration_FINAL_zh_TW.md
            （舊版 hybrid_gait_day12_whole_body_integration_zh_TW.md 已被取代）
本檔        day12_implementation_log_zh_TW.md
展示        ../hybrid_gait_day12_whole_body_dashboard.ipynb

Day 12 程式
  ../../scripts/experiments/day12_segment_contract_2d.py    Step 0：契約
  ../../scripts/experiments/day12_step0_driver.py           Step 0：產出
  ../../../tests/test_day12_segment_contract_2d.py          Step 0：27 tests
  ../../scripts/experiments/day12_nominal_cycle_2d.py       Step 1：nominal cycle
  ../../scripts/experiments/day12_step1_driver.py           Step 1：產出
  ../../../tests/test_day12_nominal_cycle_2d.py             Step 1：31 tests
  ../../scripts/experiments/day12_four_leg_state_2d.py      Step 2：四腳 + 地形註冊
  ../../scripts/experiments/day12_step2_driver.py           Step 2：產出
  ../../../tests/test_day12_four_leg_state_2d.py            Step 2：31 tests
  ../../scripts/experiments/day12_timing_skeleton_2d.py     Step 3：timing skeleton
  ../../scripts/experiments/day12_step3_driver.py           Step 3：產出
  ../../../tests/test_day12_timing_skeleton_2d.py           Step 3：27 tests
  ../../scripts/experiments/day12_transition_mapping_2d.py  Step 4：映射到時間軸
  ../../scripts/experiments/day12_step4_driver.py           Step 4：產出
  ../../../tests/test_day12_transition_mapping_2d.py        Step 4：40 tests
  ../../scripts/experiments/day12_body_trajectory_2d.py     Step 5：body trajectory
  ../../scripts/experiments/day12_step5_driver.py           Step 5：產出
  ../../../tests/test_day12_body_trajectory_2d.py           Step 5：24 tests
  ../../scripts/experiments/day12_support_stability_2d.py   Step 6：support + margin
  ../../scripts/experiments/day12_step6_driver.py           Step 6：產出
  ../../../tests/test_day12_support_stability_2d.py         Step 6：28 tests
  ../../scripts/experiments/day12_support_margin_scan_2d.py A4：掃描 + frame 速率
  ../../scripts/experiments/day12_a4_margin_scan_driver.py  A4：產出
  ../../../tests/test_day12_support_margin_scan_2d.py       A4：26 tests
  ../../scripts/experiments/day12_top_reposition_2d.py      Step 7：TOP_REPOSITION
  ../../scripts/experiments/day12_step7_driver.py           Step 7：產出
  ../../../tests/test_day12_top_reposition_2d.py            Step 7：21 tests（約 6.5 分）
  ../../scripts/experiments/day12_whole_body_trajectory_2d.py Step 8：完整軌跡
  ../../scripts/experiments/day12_step8_driver.py           Step 8：產出
  ../../../tests/test_day12_whole_body_trajectory_2d.py     Step 8：24 tests
  ../../scripts/experiments/day12_whole_body_validation_2d.py Step 9：驗證
  ../../scripts/experiments/day12_step9_driver.py           Step 9：產出
  ../../../tests/test_day12_whole_body_validation_2d.py     Step 9：29 tests
  ../../scripts/experiments/day12_terrain_generalization_2d.py Step 10：唯一入口 + gate
  ../../scripts/experiments/day12_step10_driver.py          Step 10：產出
  ../../../tests/test_day12_terrain_generalization_2d.py    Step 10：23 tests
  ../../scripts/experiments/day12_world_registration_2d.py  A5/C4/C5：把越障註冊到
                                                            世界、位置排程、越障後
                                                            的相位重建
  ../../../tests/test_day12_world_registration_2d.py        A5/C4/C5：11 tests
                                                            （約 6.5 分）
  ../../scripts/experiments/day12_paper_metrics_2d.py       Step 11：paper metrics
  ../../scripts/experiments/day12_step11_driver.py          Step 11：產出
  ../../../tests/test_day12_paper_metrics_2d.py             Step 11：19 tests
  ../../scripts/experiments/day12_step6_notebook_section.py Step 6 節（idempotent）
  ../../scripts/experiments/day12_step7_notebook_section.py Step 7 節（idempotent）
  ../../scripts/experiments/day12_step5_notebook_section.py Step 5：把該節寫進 notebook
                                                           （idempotent，可重跑）

Day 12 資料
  day12_step0_segment_semantics.csv        11 個 kind，三個 Day 12 判別欄
  day12_step0_boundary_evidence.csv        10 個 boundary，cut/handover 分開
  day12_step0_boundary_evidence.png        三面板證據圖
  day12_step1_cycle_summary.csv            每個 cycle 一列
  day12_step1_cycle_frames.csv             334 幀 x 2 cycle
  day12_step1_arc_start_evidence.csv       roll / pivot 邊界的量測
  day12_step1_phase_plot.png               五面板相位圖
  day12_step1_cycle_animation.gif          兩個 cycle 的動畫
  day12_step2_leg_mounts.csv               四個 mounting offset
  day12_step2_four_leg_state.csv           body + platform + 四腳 + 對稱檢查
  day12_step2_terrain_sweep.csv            同一份 code、五組地形參數
  day12_step2_four_leg_state.png           矢狀 + 俯視
  day12_step3_schedule.csv                 timing + 16 segment + conflicts + summary
  day12_step3_rate_demand.csv              三個週期下的 rate demand（比值不變）
  day12_step3_timeline.png                 四腳 timeline + airborne 計數
  day12_step4_leg_plans.csv                每個 cell 一列（含 overrun 計數）
  day12_step4_debug_table.csv              規格 §11 要求 8 的表
  day12_step4_timeline.png                 四腳 timeline（含越障）
  day12_step5_body_trajectory.csv          取樣 + concession + conflict + summary
  day12_step5_body_trajectory.png          body_z(t) + body_x(t)
  day12_step6_stability.csv                traversal / swing / sample 三種列
  day12_step6_stability.png                最差瞬間的三角形 + margin 對時間
  day12_step7_reposition.csv               attempt / support_gate 兩種列
  day12_step8_whole_body.csv               summary / assumption / handoff / sample
  day12_step9_validation.csv               summary / check / failure / delegated
                                           / unevaluable / joint_rate / assumption
  day12_step10_generalization.csv          規格 §17 要求 7 的比較表 + 結構化失敗
  day12_step10_size_literals.csv           generalization gate 掃到的每一行
  day12_step11_metrics.csv                 每個地形一列 ＋ 兩列 provenance note

被 Day 12 修改的 Day 10-11 檔案（181 個測試未受影響）
  ../../scripts/experiments/day10_11_motion_schema_2d.py
      加法：WHEEL_MODE_THETA_RAD、FOOT_RIM_ROLL、RECOVERY_SWING、
            pins_theta / is_nominal_locomotion / is_terrain_transition、
            RecoveryShaping2D、MotionSegment2D 的兩個驗證分支
  ../../scripts/experiments/day10_11_sequence_builders_2d.py
      純抽取：handoff_between_2d()
  ../../../tests/test_day10_11_motion_schema_2d.py
      一個測試改名並改寫斷言（三個 swing -> 三個 terrain-transition swing），
      理由見 §1 的 2026-09-01 補做

Day 12 的輸入（Day 10-11 交付物）
  ../day10-11/day10_11_step9_body_requirements.csv    392 列，365 個 knot
  ../day10-11/day10_11_implementation_log_zh_TW.md    §2 可用清單、§4 陷阱 45 條
  ../day10-11/day10_11_roll_swing_selection_zh_TW.md  規格

Step 2 讀到的既有專案幾何（都不是 Day 12 自訂的）
  legwheel/config/__init__.py       WHEEL_BASE / BODY_WIDTH / ABAD_AXIS_OFFSET
                                    / WHEEL_AXIAL_OFFSET / THETA0_DEG
  legwheel/models/corgi_leg.py      CorgiLegKinematics：腿索引與 {Li}->{Mi}->{B}
  legwheel/planners/hybrid/types.py InitialRobotState / RimId / world-frame 慣例

Day 12 引用到的既有資料
  ../day6-7/day6_7_step10r_full_traversal_frames.csv  Step 10R 的 299 幀
```
