# Hybrid Gait Day 3--5：2D Terrain-Aware Contact Model 發想與實作筆記

> **用途**：保存 Day 3--5 contact model
> 的研究發想過程、設計理由與實作順序。\
> 一方面作為之後撰寫 ICRA paper
> 時整理研究脈絡的依據；另一方面把開發工作拆成可以逐步交給 Codex
> 實作與驗證的小任務。
>
> **目前研究定位**：offline terrain-aware hybrid gait planning for a
> leg-wheel robot。已知完整地形後，預先規劃 rolling / stepping contact
> sequence；runtime 只追蹤預先生成的 trajectory。
>
> **Day 3--5 scope**：先做單腳、2D、flat ground + single rectangular
> obstacle 的 terrain-aware contact / collision query。3D 與 ABAD
> 已有既有運動學幾何能力，但此階段先用 2D 問題把 contact reasoning
> 建立正確。

------------------------------------------------------------------------

## 1. 為什麼需要新的 terrain-aware contact model？

舊版 leg-wheel
已經有成熟的運動學模型。給定腿部狀態，可以得到各機構點、rim
geometry，舊版 `contact_map()` 也可以在 flat ground / uniform slope
下，從多個 rim 中找出最低的有效接觸區域、rim、contact angle `alpha` 與
contact point。

舊方法的核心問題可以理解成：

$$
(\theta,\beta,\text{slope})
\rightarrow (\text{lowest valid rim},\alpha,p_c)
$$

這在平地很合理，因為地形基本上是一個無限延伸的平面。只要知道哪個 rim
的有效位置最低，通常就能推測 ground contact。

但研究現在要處理的是 **discontinuous terrain**。例如：

``` text
                         obstacle top
                       ┌───────────────
                       │
───────────────────────┘
        ground          ↑
                  vertical face
```

此時「整隻腳的最低點在哪裡」不再足以描述實際幾何狀態。

可能發生：

1.  最低 rim 仍正常接觸 ground，但前方另一段 rim 已撞到 obstacle
    vertical face。
2.  某個 rim 已經成功接觸 obstacle top。
3.  腳完全離開 terrain。
4.  同一 configuration 同時存在兩個以上接觸候選。
5.  接觸點位於某個 rim 的邊界附近，即將發生 rim transition。

因此新的問題應改寫成：

$$
(q,\mathcal T) \rightarrow (\mathcal C,\mathcal I)
$$

其中：

-   $q=(\theta,\beta,\gamma)$：leg
    configuration。
-   $\mathcal T$：已知 terrain geometry。
-   $\mathcal C$：contact candidates。
-   $\mathcal I$：collision / interference information。

也就是不再只問：

> 哪個點最低？

而是問：

> 給定這個 leg configuration 和 terrain，哪些地方可能形成有效接觸？整個
> leg-wheel 是否還存在其他 terrain interference？

------------------------------------------------------------------------

## 2. 目前已經完成什麼？

目前已有學長建立的 leg-wheel kinematic model，而且 2D / 3D geometry
都已經能取得。

目前已能做到：

$$
(\theta,\beta,\gamma) \rightarrow
\text{leg geometry}
$$

並可進一步取得：

``` text
lowest point
或
lowest contact region / patch
```

因此 **Day 3--5 不需要重新建立 leg geometry，也不需要重新發明
lowest-point algorithm**。

接下來真正要增加的是：

``` text
existing leg geometry
        +
terrain geometry
        ↓
terrain-aware contact / collision query
```

------------------------------------------------------------------------

## 3. 三層架構

為了讓程式與研究問題保持清楚，可以把問題拆成三層。

### Layer 1 --- Leg Geometry

回答：

> 給定 $(\theta,\beta,\gamma)$，這隻 leg-wheel
> 現在長什麼樣？

Input：

``` text
theta
beta
gamma
```

Output：

``` text
rim geometry
rim endpoints
rim centers
rim samples / arcs
foot geometry
other mechanism geometry needed for collision checking
lowest point / lowest region
```

這一層目前大致已有既有運動學模型支援。

------------------------------------------------------------------------

### Layer 2 --- Terrain Geometry

回答：

> 世界裡的地形長什麼樣？

Day 3--5 第一版只支援：

``` text
flat ground
+
single rectangular obstacle
```

2D rectangle 可以表示成：

$$
x\in[x_{\min},x_{\max}], \qquad z\in[0,h]
$$

但不要只把 terrain 當成 `terrain_height(x)`，因為 rectangle 除了 top
surface，還有 vertical faces。

至少保留：

``` text
ground
obstacle_top
obstacle_front_face
obstacle_back_face   # 可先保留資料結構，第一版不一定全部使用
```

Layer 2 **不判斷 rim、不判斷 contact，也不決定 ROLL / SWING**。它只描述
environment geometry。

------------------------------------------------------------------------

### Layer 3 --- Terrain-Aware Contact Query

回答：

> 把 Layer 1 的 leg geometry 放進 Layer 2 的 terrain
> 後，幾何上發生什麼？

概念介面：

``` python
query_contact(
    leg_pose,
    body_or_hip_pose,
    terrain,
) -> ContactQueryResult
```

它比較：

``` text
leg/rim geometry
vs.
terrain surfaces
```

最後輸出：

``` text
contact candidates
collision / interference information
```

所以整體關係是：

``` text
(theta, beta, gamma)
        ↓
   Leg Geometry ─────────┐
                         │
Terrain Geometry ────────┤
                         ↓
                  query_contact()
                         ↓
       Contact Candidates + Collision Info
```

------------------------------------------------------------------------

## 4. 為什麼一開始討論了六種現象？

一開始為了確認 Day 3--5 的模型能力，列出了：

``` text
ground contact
obstacle top contact
vertical-face collision
no contact
rim transition
multiple contact
```

但後來釐清後，**這六個不應該被做成六個互斥的 `ContactType`**。

它們其實是在回答不同問題。

### 4.1 Ground contact

表示某個允許接觸的 rim / foot geometry 與：

``` text
surface_id = ground
```

形成有效 contact。

這是正常 stance / rolling 最基本的狀態。

------------------------------------------------------------------------

### 4.2 Obstacle top contact

本質上和 ground contact 一樣，都是 valid terrain-surface contact。

差別只在：

``` text
surface_id = obstacle_top
```

以及：

$$
z_c=h_{\text{obs}}
$$

因此 Ground Contact 和 Obstacle Top Contact 不應該寫成兩套完全不同的
contact algorithm，而應該 generalize 成：

``` text
ContactCandidate
    rim
    alpha
    point
    terrain_surface_id
```

未來增加多個 rectangle 時，只需要增加不同
`surface_id`，不需要增加大量新的 contact type。

------------------------------------------------------------------------

### 4.3 Vertical-face collision

這是從 flat-ground model 進入 discontinuous terrain
後最重要的新資訊之一。

例如：

``` text
            leg-wheel
               ◯×
               ●│
────────────────┤
                │ obstacle
```

下面的 rim 可能仍正常：

``` text
contact → ground
```

但前面的 rim / mechanism 已經：

``` text
collision → obstacle_front_face
```

因此：

$$
\text{valid contact} \nRightarrow
\text{valid configuration}
$$

新的 contact query 必須同時回答：

1.  哪些 contact 存在？
2.  整個 leg geometry 是否 collision-free？

這與既有 stair-climbing work 的 reasoning 相似：在 stair terrain
中，除了 foothold / rim contact，也需要額外檢查各 rim 與 tread / riser
的 interference。

------------------------------------------------------------------------

### 4.4 No contact

如果所有允許 contact geometry 都和 terrain 有正 gap：

``` text
contacts = []
collision = false
```

則是 no contact。

這個結果本身沒有「好或壞」。

對 SWING：

``` text
no contact → normal
```

對 ROLL / STANCE：

``` text
no contact → continuous contact broken
```

因此 `query_contact()` 只負責客觀回傳沒有 contact；rolling feasibility
由下一階段判斷。

------------------------------------------------------------------------

### 4.5 Rim transition

Rim transition 不應該是一種獨立 contact state，而應該是 **contact
metadata**。

對每個 rim contact，可以記錄：

$$
m_{\text{edge}} =
\text{distance to nearest rim boundary}
$$

例如：

``` text
rim = right_rim
alpha = ...
edge_margin = small
```

代表：

> 目前仍是 right rim contact，但已經靠近 rim transition boundary，再繼續
> rolling 很可能切換到相鄰 rim。

這與本研究想利用 multi-rim continuous rolling 的問題直接相關。

第一版不需要建立 transition probability；使用 deterministic geometric
margin 即可。

------------------------------------------------------------------------

### 4.6 Multiple contact

新 foot geometry 或 obstacle edge 附近可能出現：

``` text
candidate A → foot rim / ground
candidate B → right rim / ground
```

或不同 terrain surfaces 上同時出現幾何接觸。

因此 contact query 不應強迫自己只回傳一個 active rim：

``` python
query_contact(...) -> ContactCandidate
```

而應該允許：

``` python
query_contact(...) -> list[ContactCandidate]
```

Layer 3 先保存所有候選；更高層 planner 之後再決定 active support
contact。

------------------------------------------------------------------------

## 5. 因此最終不採用「六類 enum」

不建議：

``` cpp
enum ContactType {
    GROUND,
    TOP,
    VERTICAL_COLLISION,
    NO_CONTACT,
    RIM_TRANSITION,
    MULTIPLE_CONTACT
};
```

因為它們不是 mutually exclusive。

例如一個 configuration 可以同時：

``` text
ground contact = true
vertical-face collision = true
near rim transition = true
```

比較合理的 representation 是三組資訊：

### A. Contact Candidates

``` text
ContactCandidate[]
    rim
    alpha
    contact_point_world
    terrain_surface_id
    terrain_gap
    edge_margin
```

### B. Collision / Interference Info

``` text
CollisionInfo
    collision
    colliding_part
    terrain_surface_id
    penetration_depth / minimum clearance
```

### C. Contact-set properties

可直接由 candidates 推得：

``` text
candidates.empty()       → no contact
candidates.size() == 1   → single contact
candidates.size() > 1    → multiple contact
```

因此真正需要保存的是：

$$
\boxed{
\text{接在哪個 surface}
+
\text{接哪個 rim / alpha}
+
\text{有幾個 contact}
+
\text{離 rim transition 多近}
+
\text{整隻腳有沒有 terrain interference}
}
$$

------------------------------------------------------------------------

## 6. Rectangle terrain 會不會讓研究太人工？

**Rectangle 作為開發與實驗 terrain 本身不是問題。**

真正需要避免的是：

``` cpp
if obstacle_height < 0.03:
    roll
else:
    swing
```

這種把特定 rectangle 尺寸直接寫進 gait logic 的規則。

本研究應該是：

``` text
terrain geometry
        ↓
contact / collision query
        ↓
continuous rolling feasibility
        ↓
ROLL or SWING
```

也就是 planner 不應該因為：

> 「這是一顆 50 mm rectangle」

就直接決定 swing。

而應該因為：

> 「依照這個 terrain geometry 與 leg configuration，無法找到
> collision-free continuous rolling contact path」

才決定 rolling infeasible。

因此 rectangle 應被視為：

> **controlled geometric terrain primitive**

它適合用來做 parameter sweep、驗證模型與建立 rolling feasibility
boundary。

### 建議實驗層次

``` text
Day 3–5
single rectangle
→ 驗證 contact / collision model

Day 6–7
single rectangle + obstacle-height sweep
→ rolling feasibility map

early integration
single rectangle
→ debug ROLL → SWING → ROLL

paper-level experiment
multiple sparse rectangles
→ 驗證完整 hybrid planner

main scenario
left-right asymmetric rectangles
→ 展現 Hybrid + ABAD 的價值
```

所以研究不應被描述成：

> A gait for traversing rectangular obstacles.

而比較接近：

> Terrain-aware hybrid locomotion planning is developed from geometric
> contact and rolling feasibility; controlled rectangular terrain
> primitives are used for systematic development and evaluation.

------------------------------------------------------------------------

# 7. Day 3--5 的真正完成標準

Day 3--5 **不是要回答「這顆 obstacle 能不能滾過」**。

那是 Day 6--7 rolling feasibility 的工作。

Day 3--5 只需要可靠回答：

> 對單一瞬間的 $(\theta,\beta,\gamma)$、hip/body
> pose 和 terrain，現在的 contact / collision geometry 是什麼？

形式上：

$$
(q,T_{W,H},\mathcal T) \rightarrow (\mathcal C,\mathcal I)
$$

至少能辨識：

``` text
1. flat-ground normal contact
2. rectangle 前方但尚未碰撞
3. valid ground contact + obstacle vertical-face collision
4. valid obstacle-top contact
5. no contact
```

進階再加入：

``` text
6. edge / rim-transition margin
7. multiple contact candidates
```

------------------------------------------------------------------------

# 8. Day 3--5 接下來的實作步驟

以下順序刻意拆得很小，之後可以逐項交給 Codex。

------------------------------------------------------------------------

## Step 0 --- 先盤點既有 Leg Geometry API

### 目的

不要重寫學長已經完成的運動學。

先確認目前程式能從：

``` text
theta
beta
gamma
```

取得哪些資訊。

### 要盤點

``` text
各 rim 的 center
各 rim 的 endpoints
rim arc parameterization
foot rim geometry
rim / foot surface sample points
alpha 定義
lowest point / lowest region
world/module frame conversion
其他 linkage geometry 是否能取得
```

### 特別確認

> 現有 lowest-region code 是只回傳最低結果，還是能取得「所有 rim
> geometry」？

後續 obstacle collision checking 需要的不只是最低點。

### 完成標準

寫出一份簡短 API / geometry inventory，明確知道哪些既有函式可以
reuse、哪些資料需要額外 expose。

------------------------------------------------------------------------

## Step 1 --- 建立 2D Terrain Representation

### 第一版只做

``` text
ground
single rectangle
```

建議資料：

``` python
RectangleObstacle:
    x_min
    x_max
    height
    id
```

Terrain：

``` python
TerrainProfile:
    ground_height
    obstacles
```

### 同時建立 surface identity

例如：

``` text
ground
obstacle_0_top
obstacle_0_front
obstacle_0_back
```

### 完成標準

可以畫出：

``` text
ground + rectangle
```

並能查詢 rectangle 的各 surface 幾何位置。

------------------------------------------------------------------------

## Step 2 --- 統一 Leg Geometry 與 Terrain 的座標系

Terrain 用 world frame。

現有 leg geometry 如果在 module / hip frame，需要做：

$$
{}^{W}\!p = {}^{W}\!T_H {}^{H}\!p
$$

2D prototype 至少明確定義：

``` text
+x = forward
+z = upward
```

不要讓 legacy 2D `(x,y)` 和 world `(x,y,z)` 混用。

### 完成標準

給定 hip pose +
$(\theta,\beta,\gamma)$，可以把所有需要的 rim /
mechanism points 正確畫在 terrain 上。

------------------------------------------------------------------------

## Step 3 --- 先做 Terrain Gap / Distance Query

暫時不要急著決定 active contact。

先建立最基本能力：

> 給定一個 leg sample point $p=(x,z)$，它和每個 terrain surface
> 的幾何關係是什麼？

例如 top surface：

``` text
如果 x 位於 rectangle footprint：
    gap_z = z - h
```

front face：

``` text
如果 z 位於 [0,h]：
    horizontal_gap = x - x_front
```

ground：

``` text
gap_z = z - ground_height
```

### 完成標準

任意一個 point 可以取得：

``` text
nearest / relevant surface
gap
penetration
```

------------------------------------------------------------------------

## Step 4 --- 對 Rim / Foot Geometry 做 Contact Candidate Detection

開始把既有 leg geometry samples 丟進 terrain query。

使用 tolerance：

$$
\|d\|<\epsilon_c \Rightarrow
\text{contact candidate}
$$

第一版可以 sampling，不需要 analytical solution。

每個 candidate 至少保存：

``` text
rim
alpha
point
surface_id
gap
```

### 完成標準

至少能正確得到：

``` text
flat ground → ground candidate
obstacle top → obstacle_top candidate
懸空 → no candidate
```

------------------------------------------------------------------------

## Step 5 --- 加入 Whole-Leg Collision / Interference Check

這一步是 terrain-aware model 與舊 lowest-point model 最大的差異之一。

不能只檢查 active contact region。

需要檢查：

``` text
other rims
foot geometry
必要的 linkage / mechanism geometry
```

是否與 rectangle 發生 penetration / interference。

尤其先驗證：

``` text
lower rim still contacts ground
+
front part of leg-wheel hits obstacle vertical face
```

### 第一版

可以使用 geometry sampling：

``` text
sample point inside rectangle
→ penetration / collision
```

之後再視需求換成 analytical arc-vs-segment collision。

### 完成標準

可以正確辨認：

``` text
valid ground contact
但 configuration 因 vertical-face interference 而 invalid
```

------------------------------------------------------------------------

## Step 6 --- 組合 `query_contact()`

前面功能穩定後，再包成統一 interface。

概念：

``` python
result = query_contact(
    theta,
    beta,
    gamma,
    hip_pose,
    terrain,
)
```

Output：

``` text
ContactQueryResult
    candidates: list[ContactCandidate]
    collisions: list[CollisionInfo]
```

不要在這裡決定：

``` text
ROLL
SWING
```

也不要判斷整條 rolling path。

`query_contact()` 只回答 **single configuration geometry**。

------------------------------------------------------------------------

## Step 7 --- 加入 Rim Edge / Transition Margin

對每個 contact candidate：

$$
m_{\text{edge}} =
\text{distance from current contact parameter to nearest rim boundary}
$$

先做 deterministic margin。

例如：

``` text
edge_margin large
→ contact 位於 rim 中間

edge_margin small
→ near rim transition
```

### 完成標準

visualization 中能標出：

``` text
active/candidate rim
contact alpha
nearest rim boundary
edge margin
```

------------------------------------------------------------------------

## Step 8 --- 保留 Multiple Contact

不要在 contact detection 時強迫只選一個 candidate。

如果：

``` text
len(candidates) > 1
```

就保留全部。

第一版不需要解 contact force distribution，也不需要立即決定哪個是 active
support contact。

### 完成標準

人工建立一個可能雙接觸的 configuration，確認 query 不會把第二個 contact
丟掉。

------------------------------------------------------------------------

## Step 9 --- 建立 Visualization / Debug Tool

這一步很重要，因為幾何 code
很容易「數字看起來合理，但實際畫出來是錯的」。

每個 test case 至少畫：

``` text
leg geometry
rim geometry
ground
rectangle
contact candidates
collision / penetration points
active surface IDs
```

如果方便，再標：

``` text
rim name
alpha
gap
edge margin
```

### 完成標準

只看圖就能判斷 query result 是否合理。

------------------------------------------------------------------------

## Step 10 --- 建立 Day 3--5 Regression Tests

至少固定以下 cases。

### Case A --- Flat-ground normal contact

預期：

``` text
contact candidate exists
surface = ground
collision = false
```

### Case B --- Approaching obstacle but not touching

預期：

``` text
ground contact exists
obstacle collision = false
```

### Case C --- Vertical-face interference

預期：

``` text
ground contact may still exist
collision with obstacle_front_face = true
```

### Case D --- Obstacle-top contact

預期：

``` text
candidate surface = obstacle_top
collision = false
```

### Case E --- No contact

預期：

``` text
candidates = []
collision = false
```

### Case F --- Near rim transition

預期：

``` text
candidate exists
edge_margin < threshold
```

### Case G --- Multiple contact

若目前 geometry 可以人工構造：

``` text
len(candidates) > 1
```

------------------------------------------------------------------------

# 9. 建議交給 Codex 的任務順序

不要一次叫 Codex：

> 幫我完成 terrain-aware contact model。

這樣很容易讓它自己發明 architecture。

建議一次只交一個明確任務：

``` text
Task 1
請先閱讀既有 kinematic / geometry code，
整理目前可取得的 rim geometry API，不修改程式。

Task 2
建立 2D TerrainProfile 與 RectangleObstacle，
只處理 ground + one rectangle，並建立 unit tests。

Task 3
建立 world-frame / hip-frame geometry transform，
並畫出 leg + rectangle 驗證座標。

Task 4
建立 point-to-terrain surface gap query，
暫時不要做 contact classification。

Task 5
利用既有 rim sample geometry 建立 ContactCandidate detection。

Task 6
加入 rectangle vertical-face collision detection，
特別測試「ground contact 還存在但前 rim 已撞 obstacle」的 case。

Task 7
把前面功能包成 query_contact()，
回傳 candidates + collisions。

Task 8
加入 rim edge / transition margin。

Task 9
確認 multiple contact 不會被 query_contact() 強制刪成單一結果。

Task 10
建立完整 visualization 與 Day 3–5 regression tests。
```

每完成一個 task 都先：

``` text
run tests
+
visualize
+
確認 geometry
```

再進下一步。

------------------------------------------------------------------------

# 10. Day 3--5 暫時不要做的事情

為避免 scope 擴散，此階段先不要：

``` text
不要做完整 rolling feasibility search
不要決定 ROLL / SWING
不要做 Cartesian swing planner
不要做四腳 gait integration
不要做 support polygon
不要做 ABAD stability optimization
不要做 arbitrary point cloud terrain
不要做完整 3D terrain collision engine
不要重寫既有 kinematics
```

即使 $\gamma$ 與 3D geometry 已存在，Day 3--5 的目標仍是先把
**terrain-aware contact reasoning** 在可驗證的 2D rectangle case
建正確。

------------------------------------------------------------------------

# 11. Day 3--5 完成後如何接到 Day 6--7？

Day 3--5 解的是：

$$
q_k \rightarrow
\text{contact/collision state at one instant}
$$

Day 6--7 才把它串成：

$$
q_0 \rightarrow q_1 \rightarrow q_2
\rightarrow\cdots\rightarrow q_N
$$

對每個 $q_k$ 呼叫：

``` python
query_contact(q_k, terrain)
```

檢查整條 path 是否持續滿足：

``` text
valid terrain contact
no unacceptable collision
joint limits
workspace limits
continuous forward progress
contact/rim continuity
```

如果全部成立：

$$
\text{continuous rolling feasible}
$$

如果中途出現：

``` text
no contact
vertical-face interference
kinematic limit
invalid transition
```

則該 rolling path 不成立。

因此 Day 3--5 的 contact model 是後續：

``` text
rolling feasibility
→ roll-vs-swing selection
→ complete hybrid trajectory
```

的基礎。

------------------------------------------------------------------------

# 12. Paper 撰寫時可保留的研究脈絡

之後寫 paper 時，可以從下面這條 reasoning chain 回顧研究動機：

### 起點

Leg-wheel 的 contact 不像 point-foot robot。不同 rim 都可能成為支撐與
rolling contact，而且 contact point 可以沿 rim 連續移動。

### 舊方法的限制

Flat-ground contact model 可以利用最低有效 rim 推測 contact，但
discontinuous terrain 會產生：

``` text
terrain top contact
vertical-face interference
contact loss
rim transitions
multiple geometric contacts
```

因此 lowest-point information alone 不足以判斷 configuration
feasibility。

### 新 representation

建立：

$$
(q,\mathcal T) \rightarrow
\{\text{multi-rim contact candidates},\text{terrain interference}\}
$$

並保存：

``` text
rim
alpha
contact point
terrain surface
gap
rim-transition margin
collision information
```

### 為什麼這對 Hybrid Gait 必要？

因為「能不能繼續 rolling」不是由 obstacle height 的人工 threshold
決定，而應由一連串 terrain-aware contact states 的幾何可行性決定。

因此：

$$
\text{terrain-aware contact query} \rightarrow
\text{continuous rolling feasibility} \rightarrow
\text{ROLL / SWING sequence}
$$

這使 Hybrid gait 的決策建立在 **leg-wheel--terrain interaction
geometry** 上，而不是特定 rectangle 的 hard-coded gait rule。

------------------------------------------------------------------------

## 13. 目前最直接的下一步

**下一個工作先做 Step 0：盤點既有 geometry API。**

在寫任何新的 collision/contact code 前，先讓 Codex 閱讀目前的 kinematics
/ leg geometry implementation，整理：

``` text
目前可以直接取得哪些 rim / foot geometry？
哪些是 point？
哪些是 arc？
alpha 怎麼 parameterize？
能不能取得整段 rim samples？
哪些 geometry 已經包含 gamma？
哪些資料目前只有 visualization code 有？
```

盤點完成後，再做 Step 1 的 `TerrainProfile + RectangleObstacle`。

這樣可以最大程度避免重複實作學長已經完成的幾何功能，也能讓後面的
`query_contact()` 真正建立在現有 model 上。
