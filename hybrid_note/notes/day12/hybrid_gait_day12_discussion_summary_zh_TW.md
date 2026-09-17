# Hybrid Gait Day 12 討論整理：Whole-Body Stance / Swing Formulation

## 1. 今天釐清的核心問題

目前 Day 12 已經可以組出四腳同步的 hybrid trajectory，甚至可輸出成
position-control 用的 CSV；但這不代表軌跡已經通過完整的物理可行性驗證。

目前主要問題可分開看：

-   **Body / stance formulation**：目前 Step 5 會把各 stance leg
    原本單腳 trajectory 隱含的 `hip_z` 當成 hard
    requirement（`BodyRequirementKind.TRACK`），再要求它們對應到同一個 body
    height。這很可能是 formulation 太嚴格，而不是機器人物理上不能走。
    〔2026-09-04 校正：**平地上這個 conflict 已經是 0 了**。Day 13 §3
    的 theta 補償滾動（`NominalPosture2D.hold_hip_z_m`）把 227 個
    conflict 打到 0，最壞歧異 15.329 -> 0.000 mm，`body feasible` True。
    那個「約 15 mm」是修正前的數字。**還活著的是越障**：40 mm 障礙物下
    109 個瞬間有兩個互相矛盾的 hard requirement。所以這一節要解的問題是
    **不平的接觸高度**，不是平地。〕
-   **Stability**：部分 swing phase 的 quasi-static support margin
    太小／為 0，這是之後 ABAD（gamma）真正該處理的問題。
    〔2026-09-04 校正：**平地上的 0 mm 不是穩定度問題，是 duty 的定義值**
    —— 四足 wave gait 的縱向 margin 正比於 `stance_duty - 3/4`，
    而 `GAIT_LIBRARY["Walk"]` 剛好就是 0.75。改成 duty 0.85 之後平地是
    **4.839 mm**（floor 3 mm），Step 9 **12/12 全過，沒有動到 ABAD**。
    還需要 ABAD 的是越障：Step 10 量到 **-18.571 mm**。〕
-   **Timing**：某些 swing primitive 比目前配置的 swing window
    長，需另外調整 timing / velocity。
    〔2026-09-04 校正：那個 2.000 倍是「依段落索引排程」時代的數字。
    位置排程接進管線之後，nominal swing 就是 **0.3600 s**、
    四腳每 0.6 s 交錯，一毫秒不差。**現在超出 swing window 的只剩越障的
    入口／出口過渡**。〕
-   **TOP_REPOSITION**：仍可能被 support gate 卡住，應在 stance/body
    formulation 與 stability 比較合理後再處理。

重要觀念：**四隻腿不需要相同腿長。**\
Body 是 shared rigid body，但每隻腳可以有不同的
`theta/beta/gamma`，以適應不同 contact height。

〔補一句精確的：body 是剛體，所以在目前的 2D 矢狀面模型裡**四個 hip 的
`hip_z` 本來就恆等**（掛點是剛性的，也還沒有 pitch 自由度）。
會不一樣的是**腿的伸長量 theta**。所以要移除的不是「共用 body height」，
而是「每隻腳都可以指定那個 body height」。〕

------------------------------------------------------------------------

## 2. 今天決定納入的新解法

不要再把 STANCE 和 SWING 都視為「事先固定好的 joint trajectory」。

### SWING：trajectory-driven

揮腳開始時一次規劃完整 swing trajectory：

`start state → swing path → touchdown state`

之後只依照時間取出對應的 `theta/beta/gamma`。

### STANCE / FOOT_RIM_ROLL：contact-constrained propagation

先決定 shared body motion，再由每隻 stance leg 根據自己的 contact state
計算下一小步需要的 joint configuration：

`body Δpose → hip Δpose → rolling/contact solver → next theta/beta/gamma`

因此因果關係應改成：

**Shared body trajectory → 各 stance leg 自己調整 joint configuration**

而不是：

**各 leg nominal trajectory → 各自要求一個 body height → merge body
demands**

這個做法也比較接近舊 Hybrid runtime 中 stance 使用
`LegModel::move()`、swing 使用預先生成 swing trajectory 的概念。

------------------------------------------------------------------------

## 3. 平地 Hybrid 的重新理解

Nominal flat hybrid 仍維持：

`FOOT_RIM_ROLL → RECOVERY_SWING → FOOT_RIM_ROLL → ...`

其中：

-   `FOOT_RIM_ROLL`：有限的 expanded-leg rolling stroke。
-   `RECOVERY_SWING`：離地、縮向約 17°、保持 forward rotation
    sense，再伸到下一個 touchdown configuration。
-   四腳 phase stagger，第一版維持最多一腳 airborne。

但 `FOOT_RIM_ROLL` 的 nominal profile 應主要定義 **rolling
direction、contact phase、stroke boundary**，不必把原本單腳算出的每個
`theta/beta/hip_z` 都當成 whole-body 執行時不可修改的 hard reference。

------------------------------------------------------------------------

## 4. Offline planner 不需要真的用 1000 Hz 解

1000 Hz 是最後 position-control CSV 的 playback rate，不必等於 offline
planning rate。

建議先：

-   planner：100--200 Hz 做 stance propagation；
-   obstacle edge / rim transition 附近必要時提高解析度；
-   完整 trajectory 算完後，再 interpolation / resample 成 1000 Hz CSV。

〔2026-09-04 量到的成本，供決定用：目前 `run_foot_rim_roll_2d` 的步長是
**幾何量**不是時間量 —— 每步 4 mm 接觸前進，換算成髖是 6.44 mm，
在 159.763 mm/s 的車速下等於 **40.3 ms，約 24.8 Hz**。
拉到 200 Hz 就是 8 倍的步數，而四腳越障建構現在要 8 分鐘，8 倍就是
**約 1 小時一次**。建議：**保留幾何步長**（它對滾動接觸才是自然的量），
只在障礙物邊角加密，最後再 resample。要不要真的提到 100--200 Hz，
等新 formulation 跑得出來、看平滑度不夠再說。〕

這樣仍維持目前研究 scope：

**known terrain → offline complete trajectory → deterministic CSV →
open-loop position control**

而不需要 runtime terrain replanning。

------------------------------------------------------------------------

## 5. 接下來優先處理順序

1.  **先改 stance/body formulation**\
    移除「每隻 stance leg 的 nominal hip_z 都是 hard TRACK」的假設。

2.  **先用 flat terrain 驗證**\
    固定一條 shared body trajectory，stance legs 每 timestep 用 rolling
    propagation 求新的 joint state；確認 contact continuity、joint
    smoothness、collision 與 periodic gait。

    〔2026-09-04 校正：**平地現在已經全過**（Step 9 12/12、body conflict 0、
    validator 0 失敗）。所以這一步的意義是**回歸防線**——
    「換了 formulation 之後平地不能變差」——而**不是**新 formulation 的驗證。
    真正第一次會考到新 formulation 的，是**一隻腳站在障礙物上、
    另外三隻站在地上**的那個瞬間，因為那才有不同的接觸高度。
    建議把它當成 step 2.5，緊接在平地回歸之後。〕

3.  **再放入一個已知 Day 6--7 可行的 obstacle case**\
    確認 single-leg rolling primitive 放進 whole-body
    後仍能成立，不要先加入太多新 obstacle。

4.  **再處理 timing**\
    若 swing primitive 塞不進 swing window，調整 phase duration / body
    speed，而不是混進 geometry 問題一起修。

    〔2026-09-04 提醒：**body speed 現在不是自由參數**。位置排程把它定義成
    `一個 cycle 的髖部前進 / 週期` = 159.763 mm/s，動它就等於動 duty。
    新 formulation 要把 body motion 當輸入，就必須明講這個因果從哪裡切：
    建議**沿用平地 nominal cycle 推出來的車速**當固定輸入，
    這樣平地步態一個位元都不會動。〕

5.  **最後處理 stability / ABAD**\
    gamma 應用來改善 support polygon / stability margin，不要拿來補 Step
    5 的 longitudinal hip-height conflict。

6.  **TOP_REPOSITION 最後再重新驗證**\
    等 body/stance formulation 和 support stability
    比較合理後再判斷是否真的 infeasible。

------------------------------------------------------------------------

## 6. 可以直接丟給本地 AI 的指令

> 我想修改目前 Day 12 whole-body integration 的 stance/body
> formulation。請先閱讀目前 Day 12 implementation、舊 Hybrid stance
> `LegModel::move()` 的使用方式，以及目前 Step 5 body merge / required
> hip_z 的邏輯，再提出最小修改方案，不要直接大改整個架構。
>
> 核心修改方向：
>
> 1.  SWING 保持 trajectory-driven：swing 開始時生成完整
>     trajectory，之後依時間取樣。
> 2.  STANCE / FOOT_RIM_ROLL 改成 contact-constrained propagation：先由
>     shared body trajectory 決定每個 timestep 的 body/hip
>     displacement，再讓每隻 stance leg 從上一個 joint/contact state
>     求下一個 `theta/beta/gamma`，維持合法 rolling contact。
> 3.  不要再把每隻 stance leg 原本 nominal single-leg trajectory 隱含的
>     `hip_z` 當成 hard body-height demand，也不要用 averaging/tolerance
>     去硬 merge 多個 required body_z。
> 4.  第一版先固定 `body roll=pitch=yaw=0`、gamma=0，只驗證這個新
>     formulation 能否讓 flat nominal hybrid 成立。
>     （目前是 2D 矢狀面模型，roll / yaw / pitch **根本沒有自由度**，
>     所以這一條現在是免費的；真正要固定的只有 gamma=0。）
> 5.  Offline planning frequency 先用 100--200 Hz；最後再
>     resample/interpolate 成 1000 Hz CSV。
> 6.  保留目前 `FOOT_RIM_ROLL + RECOVERY_SWING` nominal cycle、最多一腳
>     airborne，以及既有 terrain/contact/collision validator。
>     （注意：「最多一腳 airborne」在**平地成立、越障不成立**，而且
>     原因與這次的 formulation 無關 —— 同一個 `mount_x` 的兩隻腳髖永遠在
>     同一個 x，障礙物在固定的世界 x，所以它們必然同時越障。
>     詳見實作紀錄 §1.19-6。**不要把這一項失敗算到新 formulation 頭上。**）
>
> 請先完成「現有程式哪幾個模組需要改、哪些可以保留、Step 5 哪些 hard
> constraint 應移除或改寫」的分析，再給 implementation
> plan。先不要直接寫大量 code。第一個驗證 testcase 只做 flat
> terrain；flat 通過後才接一個已知 Day 6--7 rolling-feasible obstacle
> case。

------------------------------------------------------------------------

## 一句話總結

**目前缺的不是重新發明 Hybrid gait，而是把 whole-body integration
的因果方向改正：SWING 預先規劃，STANCE 由 shared body motion 逐步維持
rolling contact；先讓 flat whole-body 真正通過，再接
obstacle，最後才處理 ABAD stability。**
