# Day 13 實作紀錄

**2026-09-07 起，這份改為只記【新的】工作。**

先前 2683 行的完整紀錄已移到
`archive/day13_implementation_log_zh_TW.md`（另有一份備份
`archive/day13_implementation_log_FULL_zh_TW.md`）。

原因：那份混著計畫、量測、失敗的嘗試與中途的錯誤修正，
到後來自己都難找。現在拆成三份：

```text
day13_plan_zh_TW.md       要做什麼、待辦排序
day13_findings_zh_TW.md   結論與失敗的歸納（那 2683 行的濃縮）
day13_README_zh_TW.md     檔案索引
```

**新的工作記在這裡，一件事一節，寫完就好。**
如果某件事變成一個「結論」（成功或確定失敗），把它搬進 findings。

---

# 28. 【定位修正】現在的不是「越障策略」，是「查表」（2026-09-07）

## 28.1 專案擁有者的要求

> 「我不希望規劃僅限現在這幾個地形，它應該會是一個跨越普遍障礙物
>   （倒是可以限個高度）的策略，而不是跨越特定幾個障礙物的策略。」

## 28.2 量出來的現況：中間全是洞

`decide_2d` 是 `(h, L_top)` 的純函式，連續掃 20-200 mm：

```text
 h(mm)   L=300        L=400        L=600
    20   #4 feasible  #4 feasible  #4 feasible
    30   -- refused   -- refused   -- refused     <- 沒掃過
    40   #4 feasible  #4 feasible  #4 feasible
    50   -- refused   -- refused   -- refused     <- 沒掃過
    60   #4 feasible  ...
    70   -- refused                                <- 沒掃過
    80   #4 feasible
    90   -- refused                                <- 沒掃過
   100   #4 feasible
   110   -- refused                                <- 沒掃過
   120   #1 feasible
   130   -- refused                                <- 沒掃過
   140   #1 feasible
   150   #4 feasible
   160   #4 feasible
   170   -- refused                                <- 沒掃過
   180   #4 feasible
   190   -- refused                                <- 沒掃過
   200   #4 feasible
```

**每一個沒被掃過的高度都失敗，而它兩側的鄰居都可行。**

## 28.3 根因：以【浮點精確相等】查表

`day10_11_decision_map_2d.py:702`：

```python
key = round(height_m, 6)
if key not in tables.heights_for(StrategyId.SWING_SWING):
    return _not_measured(...)
```

各側實際掃過的高度：

```text
#1 ROLL_ROLL     40 60 80 100 120 140 160
#4 SWING_SWING   20 40 60 80 100 120 140 150 160 180 200
#5 SWING_OVER    （空的，從未掃過）
```

**130 mm 不在表裡就被拒絕**，即使 120 與 140 都可行。
這不是幾何結論，是**查表查不到**。

## 28.4 這正是擁有者說的問題

```text
現在是   一張在特定高度上查得到答案的表
應該是   一個在某個高度【範圍】內都有效的策略
```

而且 Day 12 的 generalization gate（`planner_size_literals`）**通過了**，
因為 planner 裡確實沒有尺寸字面值 —— **尺寸專用性不在 planner，在決策表**。
gate 檢查的是對的東西，但它涵蓋不到這一層。

## 28.5 修法方向

```text
不對   去把每個 10 mm 都掃一遍（掃描成本高，而且判準本身對離散化敏感，
       見 findings §3 —— 掃出來的表不可信）
對     讓決策在【量過的點之間】插值，並誠實標記那是內插而非量測
       + 明確的外插邊界（超出掃過範圍才是真的 NOT_MEASURED）
```

**關鍵區分**：

```text
內插（120 與 140 都可行 -> 130 大概也可行）      可以做，但要標記
外插（200 以上從沒量過 -> 不知道）                不能猜，維持 NOT_MEASURED
```

# 29. 高於 160 mm 的「可行」是假的：頂面沒有段落（2026-09-07）

## 29.1 內插是對的，來源是壞的

```text
120 mm   24 失敗  chaining=0   直接量過
130 mm   24 失敗  chaining=0   內插 <- 與鄰居【完全一致】
140 mm   22 失敗  chaining=0   直接量過

180 mm  182 失敗  chaining=8   【直接量過】，本身就壞
190 mm  182 失敗  chaining=8   內插
200 mm  182 失敗  chaining=8   【直接量過】，本身就壞
```

**先前判斷「190 是內插放寬造成的」是錯的，已收回。**
內插忠實傳遞了來源的品質；問題在 180/200 mm 這兩個【量過】的高度本身。

（順帶：先前加的 `MAX_BRACKET_GAP_M` 是基於那個誤判，
而且量出來根本區分不出好壞——180→200 與 120→140 都是 20 mm。已移除。）

## 29.2 根因：SWING_SWING 在高處只有兩段，中間 160 mm 沒有段落

```text
200 mm 的 composed sequence（只有 2 段）：
   0  SWING_UP     contact (-142.5, 0.0) -> (260.0, 200.0)
   1  SWING_DOWN   contact ( 420.0, 200.0) -> (700.0,   0.0)
   JUMP 0->1: 160.00 mm            <- 頂面上【沒有任何段落】

140 mm 的 ROLL 路徑（10 段）：
   2-6  ROLL_UP / WHEEL_TRANSITION  100 -> 498 mm，確實走過頂面
```

**腿在 x=260 落到頂面，然後下一段從 x=420 開始 ——
中間那 160 mm 沒有任何東西描述它怎麼過去。**

這正是 Day 12 freeze §8 記過的 `TOP_REPOSITION` 問題：
落地點與離開點之間需要一個重新定位，而那一段從來沒有被生成。

## 29.3 所以決策表把它標成 feasible 是錯的

`swing_swing_cell_2d` 只檢查：

```text
上：有沒有可行的 SWING_UP
下：有沒有可行的 SWING_DOWN
幾何：takeoff <= L_top - landing_distance
```

**它沒有檢查「落地點到起飛點之間是不是連得起來」。**
在低高度那個間隙小到可以忽略，在高處就變成 160 mm 的斷裂。

## 29.4 修法

在 `swing_swing_cell_2d` 加一個明確的檢查：
落地點與離開點之間若有間隙，且沒有段落能填它，
就回 `INFEASIBLE`（而不是 feasible），並說明原因。

**這不是放寬也不是收緊，是補上一個本來就該有的檢查。**
它會讓 150-200 mm 誠實地變成不可行，而那與量測相符
（那些高度的軌跡確實接不起來）。

## 29.5 找到真正的判準：是【策略】不是高度，而且 takeoff 選錯了

三次修法，前兩次都錯，記下來因為它們是同一族錯誤。

```text
40 mm  -> #1 ROLL   10 段  26 失敗   確實走過頂面
140 mm -> #1 ROLL   10 段  22 失敗
200 mm -> #4 SWING   2 段 182 失敗   <- 唯一走 SWING 的
```

**壞的不是高度，是 `#4` 這條路徑**：兩段之間 160 mm 沒有任何段落。
150 mm 以上 `#1` 沒資料，`#4` 就默認勝出，然後產生撕裂的軌跡。

### 修法一（錯）：用「未走過的頂面長度」直接拒絕 `#4`

**在每個高度都拒絕**，包括本來好好的 40 mm ——
因為那 160 mm 間隙是 `L_top` 的性質，**每個高度都一樣**。
拿一個在好壞案例上取值相同的量當判準，當然分不出好壞。

### 修法二（也錯）：加 `MAX_BRACKET_GAP_M` 限制內插距離

基於「190 是內插造成的」這個**誤判**。實測 180/200 mm 自己就有 182 個失敗，
內插只是忠實傳遞來源品質。而且 180→200 與 120→140 都是 20 mm，
**間隔寬度同樣分不出好壞**。已移除。

### 修法三（對）：讓 `#4` 選一個【搆得到】的 takeoff

查表發現關鍵：

```text
swing_down 可行的 takeoff（mm）
   20-100 mm 高:  80 100 120 140 160 180 200 220 240
      200 mm 高:  80 100 120                          <- 最長只有 120
```

而原本的挑法是 `min(hip_hold_fraction)` —— **永遠挑最短的 80 mm**，
於是 400 mm 頂面留下 160 mm 沒人走過。

**但 240 mm 的 takeoff 在 100 mm 以下都可行**，選它就剛好接上
（400 − 160 − 240 = 0）。所以改成：

```text
優先   選能接上的 takeoff（未走過 <= 20 mm）
否則   回報 HANDOFF_BLOCKED，並說明最長的 takeoff 仍差多少
```

`HANDOFF_BLOCKED` 而非 `INFEASIBLE`，因為那正是 Day 12 的語意：
**兩個 primitive 接不起來，是對現行 primitive 集合的陳述，不是對機器人的。**

### 結果：包絡線變成與頂面長度相關（這才合理）

```text
        L=300      L=400      L=600
 20-30   #4         #4        拒絕
 40-100  #4         #4         #1
110-140  #4/#1      #1         #1
150-200  #4        拒絕       拒絕
```

需要的 takeoff = `L − 160 mm`，而掃描只量到 240 mm：

```text
L=300 需要 >= 140 mm   -> 多數高度都有
L=400 需要 >= 240 mm   -> 只有 100 mm 以下有
L=600 需要 >= 440 mm   -> 【沒有量過這麼長的】-> 全部 blocked
```

**L=600 全部被擋不是幾何結論，是掃描沒量過那麼長的 takeoff。**
這是誠實的 —— 先前它會假裝可行然後產生撕裂的軌跡。

## 29.6 修正後的結果（2026-09-07 收尾）

### Step 9 失敗數：撕裂消失，剩下的是已知四類

```text
 40 mm  #1 ROLL  26 失敗   {airborne 6, support_legs 6, body_req 2, motor 2, margin 10}
100 mm  #1 ROLL  24 失敗   {airborne 5, support_legs 5, body_req 2, motor 2, margin 10}
140 mm  #1 ROLL  22 失敗   {airborne 4, support_legs 4, body_req 2, motor 2, margin 10}
```

**三個高度都走 `#1`（真的走過頂面），沒有任何一格帶著 182 失敗的撕裂。**
剩下的四類在每個高度都一樣，就是 findings §5 記過的那四項。

### 三份可用的 CSV

```text
hybrid_spliced_40mm     132114 列  132.11 s
hybrid_spliced_100mm    139632 列  139.63 s
hybrid_spliced_140mm    141910 列  141.91 s

三份都：超標列 0、theta 17.00-72.49 度、無折腿、
        row 0 是 home 姿態、gamma 全 0、方向已修正
        峰值 1881.65 deg/s (95.0%) —— 瓶頸在【平地段】，越障段已非限制
```

**140 mm 是新增的** —— 它落在 `#1` 的可行區間，
先前的決策 bug 讓那個區間的判定不可靠。

### 回歸

```text
165 passed（決策圖／地形泛化／論文指標／段落契約／轉換映射）
四個 190 mm 測試現在【正確地】拒絕
```

（修 fixture 而非放寬檢查：測試用的 `_tables()` 只有 0.08/0.24 m 兩個
takeoff，在 L=0.35 下需要 >=0.19 m 才接得上。補了一列 0.19 m
（接得上且 hold=0.0），**0.08 m 保留著、在該處不可用**，
所以「越短越便宜」的錯誤規則仍然會被抓到。）

# 30. 車體俯仰:專案擁有者要求刪掉 `body_rpy=(0,0,0)`(2026-09-07)

> 「模型鎖死 body_rpy = (0,0,0) 這個東西你可以直接幫我刪掉嗎。
>   我沒有要這個限制,它不太合理吧,為什麼要強制機身是平的」

## 30.1 改法:不是刪檢查,是換成正確的模型

`merge_demands` 原本「兩腳要求不同車體高度 -> 拒絕」。
改成**擬合平面** `body_z(x) = z0 + x*tan(pitch)`,
用四腳的 `mount_x` 與各自要求的高度做線性擬合。

**兩腳要求不同高度不是衝突,是斜率。**

仍然拒絕的是**平面真的解不了**的三種:

```text
1  同一個 mount_x 的兩腳要求不同高度   任何俯仰都調不了(那需要 roll)
2  殘差 > 2 mm                        四點根本不共平面(擁有者說的「扭轉」)
3  俯仰 > 25 度                        超出準靜態近似
```

## 30.2 上限一開始訂錯,是我隨手訂的

第一版設 15 度,結果 140 mm 需要 15.35 度,**被我的上限擋掉 0.35 度**。

查了才發現俯仰有乾淨的幾何式:`pitch = arctan(高度 / 軸距)`

```text
 40 mm ->  4.49 度      140 mm -> 15.35 度
100 mm -> 11.09 度      200 mm -> 21.41 度
```

**我用一個隨手訂的上限去否定一個幾何上必然的需求** ——
正是「用模型假設冒充機器人限制」那一族錯誤。
已改成 25 度(涵蓋掃描範圍最高的 200 mm),並註明它是
**對擬合的防呆,不是對機器人的宣稱**。

## 30.3 結果:衝突全部歸零

```text
              之前                     之後
flat          0 失敗                   0 失敗(逐位元不變)
 40mm  衝突 16、NaN 9/121、26 失敗  ->  衝突 0、NaN 0、24 失敗
140mm  衝突  4、NaN 3/121、22 失敗  ->  衝突 0、NaN 0、21 失敗
```

**`body_requirement_satisfied` 整項消失。**

## 30.4 兩個測試要改,但改的是【樣本】不是原則

```text
test_conflicting_hard_requirements_are_refused_not_averaged
test_two_hard_requirements_that_agree_are_one_requirement
```

兩者都用 LF(x=+255) 與 RH(x=-255) —— **前後腳對,正是俯仰能解的**。

改法:
- 前者拆成兩個測試。**前後腳對 -> 是斜率**(並驗證擬合的直線
  確實通過兩個要求,而非取中點);**同 mount_x -> 仍然拒絕**。
- 後者改用 LF/RF(同 mount_x),因為它測的是「同一個要求看兩次」。

**「絕不取中點」這個原則保住了** —— 只是現在它由「平面通過兩點」保證,
而不是由「拒絕」保證。

---

# 31. 【重大】CSV 一直是倒退走的,而且我錯了三次(2026-09-07)

> 「你可以再檢查一次,現在步態是不是讓機器人倒退走阿,
>   我如果是左前腳先揮,好像會往機器人的後面那個方向走欸」

## 31.1 用專案自己的已知前進 CSV 當基準,一次就定案

```text
Walk_Vx0.10_...csv(為 +0.1 m/s 前進產生)  stance 腳 dx = -97.69 um/ms
hybrid_flat_v1                             stance 腳 dx = +150.58 um/ms
hybrid_spliced_40mm                        stance 腳 dx = +150.58 um/ms
```

**符號相反 —— 兩份都是倒退走的。**

判準是幾何的:腳踩在地上不動,所以**腳在髖座標系的漂移方向 = 髖的漂移反向**。
腳往 -x 漂 = 髖往前 = 前進。

## 31.2 我錯三次的共同原因:用約定檢查約定

```text
第一次  越障 driver 沒傳 reverse         擁有者實機跑出來才發現
第二次  splice driver 寫死 reverse=True   造成兩半方向相反
第三次  改成「繼承平地段」                 平地段本身就是反的
                                          -> 「一致」只保證兩份一起錯
```

**我甚至加了一道「兩半方向一致」的檢查,它抓不到兩半一起錯。**

而判準一直都在:`outputs/csv/Walk_Vx0.10_*.csv` 是為前進產生的。
**應該一開始就拿它比對,而不是推理 beta 的符號約定。**

## 31.3 修法

```python
reverse = not _walks_forward(flat)   # 幾何判斷,不是符號約定
...
if not _walks_forward(out_rows):
    raise SystemExit("the spliced file walks backwards ...")
```

檢查改成**驗證前進**而非「兩半一致」,並在註解寫明為什麼必須這樣:

> **其他所有檢查在倒退的軌跡上全都會通過** ——
> theta 範圍、馬達速率、折腿、chaining 都正常,只有機器人在往後走。

---

# 32. 【已解決】方向問題,錯了四次(2026-09-07)

**本節原為暫停點的接手筆記,問題已於同日解決,結論見 32.6。**

## 32.1 已完成且可信的

```text
hybrid_flat_v1.csv        方向【已修正】,驗證前進
                          stance 腳 dx = -150.58 um/ms
                          基準 Walk_Vx0.10 是 -97.69 um/ms(同號 = 同向)
                          舊的倒退版備份在 scratchpad/hybrid_flat_v1_BACKWARDS_backup.csv
車體俯仰                   衝突全部歸零(見 §30),body_requirement 整項消失
                          test_day12_body_trajectory_2d.py 25 passed
決策表兩個 bug             內插 + takeoff 選擇(見 §29)
折腿 bug                   匯出層拒絕 + 時間軸補洞(見 §27)
```

## 32.2 【不要用】磁碟上現有的 spliced CSV

```text
hybrid_spliced_40mm.csv    這三份是【舊的、倒退的】版本
hybrid_spliced_100mm.csv   新的檢查會擋下它們,但檔案還在磁碟上
hybrid_spliced_140mm.csv
```

## 32.3 卡在哪:兩半可能需要【相反】的 reverse

最後一次執行的輸出:

```text
40 mm:  flat half walks forwards -> reverse=False for the crossing
        the crossing half walks backwards.        <- 拒絕出檔
```

平地段修好之後,**越障段在同一個 `reverse` 下卻是倒退的**。

`day13_step5_splice_driver.py` 現在寫的是:

```python
reverse = not _walks_forward(flat)     # 假設兩半用同一個值
```

**那個假設可能是錯的。** 若兩個生成器(平地 nominal vs 越障 traversal)
對 beta 的符號約定本來就不同,就需要各自判斷。

## 32.4 下一步(未完成的診斷)

`scratchpad/n1_cross.py` 已寫好但未跑完。它會回答:

```text
1  規劃器自己的 body_position_world_m 說越障往哪走(唯一可信來源)
2  越障段用 reverse=False / True 匯出,哪一個的正運動學與規劃器一致
```

依結果修 `_walks_forward` 的用法:**分別判斷兩半**,而不是假設一致。

然後重產三份 CSV(各約 8 分鐘),驗證方向,更新 README/findings。

## 32.5 判斷方向的正確方法(別再用 beta 符號)

```text
基準   outputs/csv/Walk_Vx0.10_...csv  是專案為 +0.1 m/s【前進】產生的
判準   跑正運動學,看 stance 腳在髖座標系的漂移
       腳踩地不動 -> 腳往 -x 漂 = 髖往前 = 前進
```

**本輪在方向上錯了三次,全部因為用約定檢查約定(見 §31.2)。**

## 32.6 【解決】第四次的錯:判準只在平地成立

診斷結果(用 phase 欄位判定 stance,不用高度門檻):

```text
FLAT  planner body x     0 -> 475.3 mm  (FORWARD)   需要 reverse=True
越障  planner body x   148 -> 1600.0 mm (FORWARD)   需要 reverse=True
```

**兩者都需要 True**,而且兩個生成器的 `beta_direction` 本來就都是 −1.0。
所以「兩半需要相反的值」那個假設是錯的。

### 錯誤的公式

```python
reverse = not _walks_forward(flat)      # 錯
```

平地修好後 `_walks_forward(flat)` 回 True -> `reverse=False` -> 把越障設成錯的。
**錯在兩層**:平地 CSV 是已完成的成品,方向早已定案;
而越障需要什麼是【這個 plan 自己的性質】,與平地檔案無關。

改成直接問規劃器:

```python
body_xs = [s.body_position_world_m[0] for s in run.trajectory.samples]
reverse = body_xs[-1] > body_xs[0]
```

### 第四次的錯:偵測器沒壞,是用錯地方

```text
hybrid_flat_v1(平地)        -151.07 um/ms  -> 答對
完整越障軌跡                 -1128   um/step -> 答對
【裁切後的越障段】            +85.68  um/step -> 「答錯」
```

而規劃器說那一段 **body x 550.3 -> 1600.0 mm,單調遞增**。

偵測器的前提是「腳踩地不動,所以腳的漂移 = 髖的漂移反向」。
那在 `FOOT_RIM_ROLL` 成立,但裁切後保留的 stance 主要是:

```text
WHEEL_TRANSITION 3、ROLL_DOWN 3、ROLL_UP 2、APPROACH 2、FOOT_RIM_ROLL 2
```

`WHEEL_TRANSITION` 是**縮到 17 度貼著頂面滾** —— 姿態在變、接觸點在輪緣上遷移、
還在一個抬高的平面上。**腳的位移不再只反映髖的位移。**

**修法**:函式改名為 `_flat_walks_forward`,docstring 寫明界線與實測數字;
平地半用它,越障半改用規劃器的 body x(在所有段落都成立的量)。
**名字本身就是防呆。**

## 32.7 最終產出(三份都通過)

```text
              列數      超標  峰值        theta          平地段方向
 40 mm      132114       0   1881.7   17.00-72.49   -151.1 um/ms FWD
100 mm      139632       0   1881.7   17.00-72.49   -151.1 um/ms FWD
140 mm      141910       0   1881.7   17.00-72.49   -151.1 um/ms FWD

三份皆:低於 17 度的列 0、row0 是 home 姿態、gamma 全 0
越障段車體:550.3->1600.0 / 550.8->1675.2 / 550.2->1697.3 mm(皆前進)
```

`hybrid_flat_v1.csv` 亦已用 `--reverse` 重產,驗證前進
(舊的倒退版備份在 `scratchpad/hybrid_flat_v1_BACKWARDS_backup.csv`)。

---

# 33. 【架構層】全機協同排程(2026-09-07)

## 33.1 專案擁有者的定位

> 「雖然當初是單腳先規劃,但現在來到了全機了,所以應該要以全機作為考量,
>   去改善各腳的軌跡,很顯然在只用位置判斷腳的什麼時候要揮非常的不可行」

**這句話把我今天四次失敗的原因講清楚了。**

`world_schedule_2d` 是「單腳規劃 + 事後排時間」的架構:

```python
start_s = (hip_x - mount - origin) / speed
```

每隻腳的軌跡各自生成好,再用位置推它該在什麼時候發生。
**四隻腳之間沒有任何協調機制。**

我今天試的四種方法(延後一隻腳、拉長一段、暫停整台車、錯開落點)
**全部在這個架構之內動手腳,全部失敗,而且失敗原因相同** ——
在這個架構下,腿的時間就是它對車體位置的說法,動一隻就必然分歧。

**我一直在修症狀,擁有者指出的是病因。**

### 對 §25 結論的更正

§25 寫「在位置驅動架構下,兩個需求不可兼得」。
那句話**在架構之內是對的,但我把它寫成了「無解」**。
正確版本:**這個架構解不了,需要換架構。**

## 33.2 做法:延後【車體的時鐘】,不是延後一隻腳

新增 `whole_body_schedule_2d`(`day12_world_registration_2d.py`)。

```text
先前四次   延後【一隻腳】     -> 它與其他三隻分歧 84.068 mm
現在       延後【整條時間軸】 -> 四腳看到同一個延遲
                              -> 分歧【依構造】不可能改變
```

車體沿路徑的進度**仍然是唯一的時間來源**(所以 §1.14 那個 602.6 mm
的問題不會回來),改變的只是**進度被消耗的速率**。

演算法就是擁有者更早提的規則:

```text
沿時間軸走過每一次揮腳:
  若放行它會讓離地腿數 > max_airborne
     -> 延後【整台機器】直到前一隻落地
     -> 記錄這次 hold(誰、多久、等誰)
```

介面:`plan_terrain_2d(..., whole_body=True)`、
CSV driver 的 `--whole-body`,並記進 summary。

## 33.3 過程中撞到的 bug:重複套用延遲

第一次執行崩潰:

```text
ValueError: leg LH has overlapping segments at 6.700528 s;
            one leg does one thing at a time.
```

**這個錯誤是好事** —— 它是 `FourLegSchedule2D` 的不變式,
抓到了我的 bug 而不是讓壞軌跡溜過去。

原因:全機排程器已經把 hold 寫進時間軸,
但下游 `insert_holds_2d` 看到 `swing_waits` 非空又插一次,
**同一個延遲加了兩遍**。已用 `waited_for` 欄位標記全機的 hold 讓它跳過。

## 33.4 基準(位置排程,100 mm swing)

```text
legs/sample   {4: 121}
world spread  0.0000 mm        <- 四腳完全一致,這是要保住的
holds         0
duration      9.801 s
step9         171  {airborne 75, support_legs 75, margin 18, ...}
```

---

# 34. B5:把 SWING_UP 換成 nominal 的揮法(2026-09-07)

## 34.1 專案擁有者的要求

> 「我想要換成這種 nominal locomotion 也就是走平地的那種揮法,
>   不過 day8-9 不會全棄,因為你還是會需要知道初始位置跟末位置」

現在的 `SWING_UP` 是 **Cartesian swing**(Day 8-9 規劃器):
抬腳、越過、放下。擁有者的圖顯示 theta 60->54.2->41.0->36.6->48.1->58.7,
**先縮再伸**,接觸點從地面直接跳到頂面。

要換成 nominal 的 `RECOVERY_SWING`:**縮到 17 度、順著同一個前向旋轉、
再伸開落地** —— 比較不會撞到障礙物前緣。

## 34.2 量到的關鍵事實:起訖姿態【完全相同】

```text
Day 8-9 解出的 SWING_UP(40 mm):
  起飛   theta 60.00  beta 0.00  foot_rim  hip (-33.2, 219.4)
  落地   theta 60.00  beta 0.00  foot_rim  hip (260.0, 259.4)
  接觸 z 0.0 -> 40.0 mm
```

**兩端都是 theta 60、beta 0、foot_rim 正中央 —— 那正是平地 nominal
的接觸狀態。** 唯一差別是落地點高 40 mm(髖也跟著高 40 mm)。

## 34.3 所以 B5 不需要新的生成器

`run_recovery_swing_2d` 的簽章已經收下擁有者說的那三個端點量:

```python
run_recovery_swing_2d(
    stroke,
    beta_target_rad=...,        # 落地的 beta
    theta_touchdown_rad=...,    # 落地的腿長
    hip_z_touchdown_m=...,      # 落地的髖高  <- 只要把這個調高
)
```

**擁有者說「Day 8-9 不會全棄,還是需要知道初始跟末位置」是對的**:
Day 8-9 提供端點,nominal 生成器負責用滾的方式飛過去。
**兩個既有的東西接起來,不是寫第三個。**

## 34.4 支持它可行的既有量測

```text
頂面旋轉淨空  74.448 mm 對 10 mm 要求
              而且平地 / 40 mm 頂 / 100 mm 頂【完全相同】
              —— 淨空只取決於「髖相對於它站的那個面」,
                 整個場景抬高不改變那個關係
```

所以在頂面上做 nominal 旋轉是有餘裕的。

## 34.5 【成功】nominal 揮法確實能落在障礙物頂面

```text
x8  站姿 hip (325.9, 219.4) theta 72.49 beta -39.84
    障礙 h=40 face=395.2
    -> OK  落地 (430.7, 40.0)  125 frames
```

**這就是 B5 要的東西**:沒有用到 Day 8-9 的 Cartesian `SWING_UP`,
是平地那套「縮到 17 度 -> 順著同一個前向旋轉 -> 伸開落地」直接落在頂面。

### 為什麼前面 x5 / x6 / x7 全都失敗 —— 三個【問錯】疊在一起

**(1) x5 / x6:障礙物根本不在揮動的場景裡。**
`run_recovery_swing_2d` 第 996 行是 `posture = stroke.posture` ——
**姿態是從 stroke 拿的,不是從參數拿的。**
我當時「把 stroke 建在平地、把帶障礙的 posture 給 swing」,
那個 posture 從頭到尾沒有進到函式裡。
所以 6/6 的 `TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`
不是「落不上去」,是**在一個沒有障礙物的世界裡要求落在 40 mm 高的空氣中**。

**(2) x7 的「(b) 低指定成功了」是假的成功。**
同一個 bug:`run_recovery_swing_2d(stroke, ...)` 的 stroke 仍然是平地的,
所以它「成功」落在 `(445.2, 0.0)` —— **z = 0,就是平地**。
它只是把平地 nominal 揮法再跑了一次。
**一個成功的結果,如果它成功的對象不是我問的那件事,它就是失敗。**

**(3) 我以為的「過度指定陷阱」不是這裡的問題。**
同時給 `theta_touchdown_rad` 和 `hip_z_touchdown_m` 在這裡**是對的**:
把地面和髖【同時】抬高同一個 h,腿的幾何完全不變,
所以這兩個量是「由構造保證一致」,不是互相矛盾。

正確寫法:

```python
post   = replace(flat, obstacle_xwh_m=(face, 0.40, h))
stroke = standing_stroke_2d(post, th0, b0, hx0, hz0)   # <- 障礙從這裡進去
out    = run_recovery_swing_2d(
    stroke, cfg,
    beta_target_rad     = beta_t,
    theta_touchdown_rad = flat.theta_rad,               # 腿長不變
    hip_z_touchdown_m   = flat.hip_z_for_flat_stance(beta_t) + h,  # 髖抬 h
)
```

## 34.6 第二個「只准站在地面」的假設,藏在另一個地方

打通落地檢查之後,下一格馬上炸:

```text
ValueError: frame 124 (RECOVERY_TOUCHDOWN) has no ground contact on its
support sample; the stroke driver must not have got here.
```

`_stance_frame` 裡又寫死了一次 `surface_ids=(ground_surface_id,)`。
**同一個假設在三個地方各寫一次**,我上次只改了落地檢查那一處。
所以會出現「落地檢查說可以,下一行說沒有接觸」這種自相矛盾。

處置:抽成單一 `_standable_surface_ids(scene)`(地面 + 任何 `_top` 結尾),
三處共用 —— `_stance_frame`、落地檢查、`standing_stroke_2d`。

**刻意沒有改的兩處**:滾動中追蹤接觸的 736 / 831 行仍是只准地面。
滾上頂面是另一種主張,而且 Day 12 的凍結數字建立在那個限制上。

## 34.7 h>=100 的「失敗」是我的掃描範圍問題,不是高度極限

第一次掃描的結論長得像個高度上限:

```text
h= 40:  8/17 faces OK, 364.2..434.2 mm
h= 70:  3/17 faces OK, 414.2..434.2 mm
h=100:  none  {'stroke:STANDING_POSE_IS_NOT_A_GROUND_CONTACT': 17}
h=140:  none  (同上)
h=190:  none  (同上)
```

**但 17/17 全部失敗在 stroke,也就是在揮動開始【之前】,
而且與障礙物放在哪裡完全無關。**
幾何遮擋一定跟位置有關;跟位置無關的失敗是問錯,不是極限。

直接量站姿:

```text
站姿腿的 x 範圍 121.1 .. 462.3 mm   (腳在 264.2,腿body 往前伸到 462.3)
h=100 face=414 -> collision=True
h=100 face=444 -> collision=False   <-- 可行,但我沒掃到這裡
```

**站姿的腿往前伸出腳前方 198 mm。**
我的掃描上限是 `land_x - 4mm = 441.2`,
而 h=100 能站的 face 從 444 才開始 —— **可行區間就在我掃描範圍外面 3 mm。**
「h>=100 不可行」是我自己把窗戶關上再說裡面沒有光。

註:這裡失敗的是我為了做實驗而選的【站姿】與障礙物重疊,
不是揮動本身。真正的規劃裡站姿由前一段決定,不會這樣擺。

## 34.8 B5 的實作:`day13_b5_nominal_ascent_2d.py`

新模組,只做一件事 —— **把 Day 8-9 的「端點」和 nominal 的「飛法」接起來**。

```python
run_nominal_ascent_2d(
    spec, posture, config,
    approach_hip_x_m = ...,   # Day 10-11 composer 給的起點
    landing_hip_x_m  = ...,   # Day 10-11 composer 給的終點
    beta_takeoff_rad = ..., beta_landing_rad = ...,
) -> NominalAscent2D
```

**分工完全照擁有者說的**:
Day 8-9 / Day 10-11 決定【在哪裡起、在哪裡落】,
`run_recovery_swing_2d` 決定【怎麼飛過去】。

### 它取代的是 composer 裡哪一行

`day10_11_composer_2d.py:569` 的 `up_plan = generate_swing_2d(up_request, ...)`。
它的兩端 `start_scene` / `top_scene` 由 `standing_scene_2d` 建 —— **那兩個保留**,
只換中間的生成器。

### 為什麼擁有者要換(用數字說)

```text
Cartesian SWING_UP  theta 60 -> 54.2 -> 41.0 -> 36.6 -> 48.1 -> 58.7
                    先縮再【伸】,通過障礙物前緣時腿接近全長
nominal RECOVERY    theta 縮到 17 度並【保持】,整個旋轉都在最短姿態
```

這就是擁有者說的「不用抬腳揮這樣也比較不會撞到前面的障礙物」,
而且它是可量的:測試 `test_the_swing_retracts_rather_than_reaching_over_the_edge`
斷言旋轉階段每一格的 theta 都等於 `theta_compact_rad`。

### 刻意保留的拒絕資訊

`NominalAscent2D` 即使失敗也記下 `obstacle_xwh_m` 與 `start_contact_xz_m`。
理由就是 34.7:**「它拒絕了」在沒有「它拒絕的是哪個問題」時不是一個量測。**
34.7 那次差 3 mm 的掃描窗誤判成高度上限,就是因為只留了 verdict。

### 測試

`tests/test_day13_b5_nominal_ascent_2d.py`,五項,分別釘住:

1. 可站立面 = 地面 + 任何 `_top`,且【不含】垂直面
2. 障礙物必須經由 stroke 進入(34.5 那個 bug 的回歸測試)
3. **落地 z == 頂面 z**,而且落在頂面範圍【之內】 <- B5 的主張本身
4. 旋轉全程維持 compact(換揮法的理由)
5. 失敗時仍保留它被問的問題(34.7 的教訓)

## 33.5 【失敗】全機排程量出來更糟,而且錯在它最該保住的地方

```text
                 位置排程       全機排程
world spread     0.0000 mm  ->  122.5322 mm   <- 應該維持 0
airborne 失敗    75         ->  93            <- 反而更糟
duration         9.801 s    ->  16.302 s      <- 多 6.5 秒
holds            0          ->  13 次,共 16.908 s
```

**我的設計主張是「延後整條時間軸 -> 四腳看到同一個延遲 -> 分歧依構造不可能
改變」。量出來 122.5 mm,比先前四次的 84 mm 還糟。**

### 錯在哪:兩層

**(a) 實作層** —— `shifted()` 依段落的【起點】決定要不要套用某個 cut:

```python
def shifted(t):
    for cut, delay in cuts:
        if t >= cut - 1e-9:
            out += delay
```

一個【跨越】cut 的段落,起點與終點會被套上不同的延遲量,
等於把那個段落**拉長或壓縮** —— 腿自己的動作就被改掉了。

**(b) 概念層(這個更根本)** —— 我誤解了 `world_x_spread` 的意義。

它量的是**四隻 stance 腿對「車體在哪」的分歧**,而每隻腿的說法來自
它在【那個排定時刻】的 `hip_x`。所以:

```text
延後一隻腿的時間 -> 同一瞬間它呈現的是【鏈上不同的姿態】-> 說法改變
延後整條時間軸   -> 每隻腿在鏈上的【進度不同】,
                    所以同一個延遲【不會】讓它們維持一致
```

**「大家延一樣多所以不會分歧」這個推理是錯的。**
四隻腿在自己的鏈上處於不同位置,平移時間軸不等於平移它們的相對狀態。

### 這一次的教訓與 §25 相同,但更明確

§25 已經寫過「腿的時間就是它對車體位置的說法」。
**我當時理解了那句話,卻沒有推導出它的推論**:
既然時間就是說法,那麼【任何】對時間的改動都會改變說法 ——
包括「對所有腿一視同仁」的改動。

所以擁有者說的「以全機為考量」是對的方向,
**但實現它不能只是在既有排程之後平移時間**,
必須讓四腳的軌跡在【生成階段】就互相知道 —— 那是重寫,不是後處理。

### 處置

`whole_body_schedule_2d` 保留,但改為明確標記「已量測、不可用」,
連同上面的數字,以免下一個人重做一次。
`plan_terrain_2d(whole_body=...)` 預設 False,不影響任何現有結果。

## 34.9 兩個站立高度不是同一個數,而且【本來就不該】是同一個數

模組寫好之後,已經量過可行的 40 mm 案例反而失敗了兩次,兩次都是高度取錯:

```text
第一次  APPROACH_POSE_REFUSED
        起始站姿用了 hip_z_for_flat_stance = 202.2 mm
        但 Hybrid posture 實際站在 hold_hip_z_m = 219.4 mm
        -> 差 17.2 mm,腿被塞進地面裡

第二次  TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT
        我「修正」成落地也用 219.4 + h
        -> 但落地的 theta 是 60 度(下一段要接的姿態),
           60 度伸不到 219.4,反而構不到頂面
```

**正確答案是兩端用不同的高度,而且這不是不一致:**

```text
起飛端  theta 72.49(hold_hip_z_m 調變出來的)-> 髖 219.4
落地端  theta 60.00(下一段的標稱姿態)      -> 髖 202.2 + h
```

起點和終點是【不同的姿態】—— 從一個姿態飛到另一個姿態正是揮動的工作。
`hip_z_for_flat_stance` 回答的是「固定 theta、髖隨輪弧上下」的姿態;
Hybrid 用 theta 調變把髖壓平,所以站得比它高 17.2 mm。
**問錯哪一個,腿就不是插進地裡就是構不到頂面。**

處置:`_stance_hip_z_m(posture, beta)` 只用在起飛端,
落地端維持 `hip_z_for_flat_stance + height`,兩處都寫上為什麼。

## 34.10 B5 完成:端到端量到的結果

```text
起飛  foot_rim  ground              x 264.2   theta 72.49
落地  foot_rim  day6_7_obstacle_top (430.7, 40.000)  theta 60.00
      125 frames
      apex_clearance 0.0 / liftoff_rise 0.0  <- 標稱揮法沒有這些旋鈕
```

`day6_7_obstacle_top` 這個 surface_id 是關鍵 ——
**落地面是障礙物頂面本身,不是「高度剛好對」的地面。**

### 回歸與新測試

```text
tests/test_day12_nominal_cycle_2d.py        44 passed in 857.02s
tests/test_day13_b5_nominal_ascent_2d.py     5 passed in 113.06s
```

`_standable_surface_ids` 的重構沒有改動任何凍結數字。

## 34.11 障礙物位置的可行帶(修正掃描窗之後)

```text
h=  40: 18/47 faces OK, 359.2..444.2 mm
h=  70:  7/47 faces OK, 414.2..444.2 mm
h= 100:  1/47 faces OK, 444.2 mm 一個點,落在 (443.4, 100.5)
h= 140 / 190: 這一輪仍未找到
```

**h=100 只有【最後一個】掃描點可行 —— 窗戶又貼邊了。**
34.7 才剛因為同樣的事誤判過一次,所以這裡不能寫「100 mm 勉強可行、140 不可行」。
正在用更寬的窗(到 land_x + 260 mm)重掃,並且明確報告
「可行帶是否碰到掃描邊界」,讓貼邊這件事自己說出來,不必靠我記得檢查。

**這一欄現在還不能寫結論。**

## 34.12 接進 composer(部分完成)與交接

`compose_swing_swing_2d()` 加了 `nominal_ascent: bool = False`。
**預設 False,每個既有結果原封不動。** 兩條路都驗證過能組出來:

```text
nominal_ascent=False: up_segment end (260.0,40.0) on day10_11_obstacle_top  samples  31
nominal_ascent=True : up_segment end (307.3,40.0) on day6_7_obstacle_top    samples 125
```

### 踩到循環 import

`day12_transition_mapping_2d` 會 import 本模組的 `ComposedSequence2D`,
所以在 `day10_11_composer_2d.py` 模組層 import 任何 day12 的東西都會造成循環。
改成函式內 lazy import,並在註解寫明原因 —— **不要「順手整理」搬回開頭。**

### 【未完成】`frame_rows`

`swing_frame_rows()` 讀 `SwingPlan2D`,標稱 ascent 沒有那個東西
(它的資料在 `up_ascent.swing.frames`)。
目前 `nominal_ascent=True` 時**只帶下降段的 rows**,程式碼裡標了
`UNFINISHED (Day 13 B5)`。**匯出器需要這些 rows,不接完產不出 CSV。**

沒有用「看起來合理的數字」把它填滿 —— 那會變成安靜地匯出一個沒有上升段的越障。

### 交接文件

`day13_handoff_zh_TW.md`(291 行),給新對話用,已從 README 連出去。
