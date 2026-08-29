# New Hybrid Gait Research Plan & Two-Week Development Roadmap

> Purpose: This note summarizes the current research direction, software architecture, planning assumptions, implementation priorities, and short-term schedule for the new hybrid gait on the ABAD-enabled leg-wheel robot.
>
> Intended use: keep this file in the local workspace and provide it to Codex as persistent project context when implementing or modifying gait-planning code.
>
> Current date: 2026-08-23

---

## 1. Research Goal

The final objective is to develop an **offline terrain-aware hybrid gait planner** for the new leg-wheel robot with an additional ABAD degree of freedom. Given a complete known terrain, the planner precomputes the body, contact, stance/swing, and joint-reference trajectories for the entire traversal; at runtime, the robot only tracks this deterministic trajectory.

The target terrain is **left-right asymmetric, discontinuous rough terrain**, especially terrain composed of sparse rectangular obstacles with different heights on the two sides of the robot.

The terrain should satisfy at least one of the following:

- Pure wheel mode cannot pass the obstacle.
- Pure wheel mode can pass, but causes large body roll/pitch or unstable motion.
- Pure walking can traverse it, but requires unnecessary swing motions and therefore consumes more energy.
- Hybrid gait can exploit rolling wherever possible and only use swing repositioning when necessary.

The central motivation is:

> **Use rolling as much as possible for efficiency, and use stepping only when rolling becomes infeasible or unstable.**

The desired final comparison is therefore:

$$
\text{Wheel} \quad vs. \quad \text{Walk} \quad vs. \quad \text{Proposed Hybrid}
$$

with the proposed hybrid gait achieving:

- better terrain traversability than wheel mode,
- lower energy consumption than pure walking,
- reduced body attitude variation on asymmetric terrain,
- fewer unnecessary swing events.

---

## 2. Proposed Research Positioning

The new work should **not** be framed as simply “making one leg step onto a rectangular obstacle.”

A precise positioning is:

> **Offline multi-contact rolling–stepping gait planning for a leg-wheel robot with ABAD on known asymmetric discontinuous terrain.**

The planner explicitly uses:

1. **multi-rim contact information,**
2. **continuous rolling during stance,**
3. **selective swing repositioning,**
4. **ABAD-based lateral support adjustment.**

The main planning question is:

> **Given a known terrain, what sequence of rolling and stepping contacts should the robot use to traverse the terrain efficiently while satisfying collision-free motion and support-stability constraints?**

The work is explicitly limited to **offline trajectory planning**. It does not include online perception, depth-camera reconstruction, runtime terrain adaptation, receding-horizon replanning, or a real-time high-level planning requirement.

---

## 3. Relationship to Previous Work

### 3.1 Old Hybrid / WLW

The old hybrid gait already provides a useful software skeleton:

```text
wlw_open.cpp
    -> runtime loop / motor command

GaitSelector / Simple_fsm
    -> shared gait state, duty, swing phase

Hybrid::Initialize()
    -> initialize four-leg gait phases

Hybrid::Step()
    -> stance: LegModel::move()
    -> swing: HybridSwing::generate() / Swing_step()

next_eta[i] = {theta, beta}
    -> motor command
```

The important conceptual separation is:

```text
STANCE
    -> maintain rim contact
    -> use LegModel::move()
    -> body moves while contact point rolls

SWING
    -> leave ground
    -> reposition leg for next touchdown
```

The old code is therefore **not discarded**. It should be reused as the low-level gait-execution framework.

However, the old gait is mainly parameterized by:

- velocity,
- step length,
- stand height,
- gait phase.

It does **not** provide a general terrain-aware world-frame foothold planner.

---

## 4. Main Technical Contributions to Develop

The current plan is to focus on three technical elements.

### Contribution A — Terrain-Aware Multi-Rim Contact Representation

Old contact logic:

$$
(\theta,\beta,\text{slope})
\rightarrow
(S,\alpha,p_c)
$$

works mainly for flat or uniformly sloped surfaces because the contact point is determined using the lowest valid rim point relative to a ground plane.

The new planner should instead use:

$$
(q,\mathcal T)
\rightarrow
\mathcal C
$$

where:

$$
q=(\theta,\beta,\gamma)
$$

and:

$$
\mathcal T = \text{known terrain geometry}
$$

The output should contain one or more feasible contact candidates:

$$
\mathcal C=
\{(S_j,\alpha_j,p_j,\text{collision},\text{margin})\}
$$

The first implementation only needs to support:

```text
flat terrain
rectangular obstacle
```

Do not start with arbitrary point clouds or full 3D terrain.

---

### Contribution B — Rolling / Swing Hybrid Motion Selection

The planner should not assume:

```text
obstacle -> swing
```

Instead it should consider at least two motion primitives:

```text
ROLL
SWING
```

Potential motion patterns include:

```text
pure rolling

roll -> swing -> roll

full swing
```

A first rule-based planner is sufficient:

```text
if a collision-free continuous rolling trajectory exists:
    use ROLL
else:
    use SWING
```

Optimization can be added later if necessary.

Possible future cost:

$$
J =
w_E E
+
w_s N_{\text{swing}}
+
w_p J_{\text{posture}}
+
w_c J_{\text{collision}}
+
w_m J_{\text{stability}}
$$

but this should **not** be the first implementation priority.

---

### Contribution C — ABAD-Based Stability Adjustment

The ABAD DOF should primarily be used to change the lateral contact geometry.

For a swing leg $i$, the other three feet form the support polygon:

$$
\mathcal P_{\text{sup}}
=
\operatorname{ConvHull}\!\left(\{p_{c,j}\}_{j\ne i}\right)
$$

The stability requirement is:

$$
p_{\text{CoM},xy}
\in
\mathcal P_{\text{sup}}
$$

A more conservative condition is:

$$
d
\left(
p_{\text{CoM}},
\partial \mathcal P_{\text{sup}}
\right)
>
d_{\min}
$$

ABAD angle $\gamma$ changes the lateral foothold position:

$$
\gamma
\rightarrow
y_{\text{contact}}
\rightarrow
\mathcal P_{\text{sup}}
$$

The first version can be a simple quasi-static heuristic.

Do **not** begin with a full whole-body optimization.

---

## 5. New Planner Interface

The new planner is an **offline trajectory-level planner**, not a runtime selector that makes only the current ROLL/SWING decision.

### Planner input

```text
TerrainProfile
initial robot/body state
goal / traversal path
robot geometry and kinematics
gait constraints
```

The new planner should not use `stand_height` as the only terrain-related input.

The software should explicitly distinguish:

```text
stand_height
step_height / swing_clearance
target_foothold_height
target_foothold_position
```

Recommended high-level structures:

```cpp
struct ContactState {
    RimId rim;
    double alpha;
    Eigen::Vector3d point_world;
    std::string terrain_surface_id;
};

struct SwingTarget {
    Eigen::Vector3d target_position;
    RimId target_rim;
    double target_alpha;
    double clearance;
};

struct ContactCandidate {
    RimId rim;
    double alpha;
    Eigen::Vector3d point_world;
    double terrain_gap;
    double edge_margin;
    bool collision_free;
    std::string terrain_surface_id;
};

struct TerrainProfile {
    // first version:
    // flat + rectangular obstacles only
};

struct BodyTarget {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};
```

### Planner output

The output is a synchronized time series over the complete traversal:

```text
body pose trajectory
per-leg theta / beta / gamma trajectory
per-leg contact state and foothold
per-leg rim / alpha
per-leg ROLL / SWING mode and swing phase
stability margin
```

It is written as a deterministic `trajectory.csv`, or an equivalent file with an explicit schema, shared by simulation and the real robot. At minimum it includes time, body position/orientation, all four legs' joint references, mode/rim/alpha/contact metadata, footholds, and stability margin.

The important separation is:

```text
Complete known terrain + initial state + goal + constraints
        ↓
offline contact / motion planning for the complete traversal
        ↓
complete contact sequence and ROLL / SWING decisions
        ↓
body + stance / swing + ABAD trajectory planning
        ↓
complete theta / beta / gamma references
        ↓
deterministic trajectory file
        ↓
runtime playback and low-level joint tracking
```

Runtime does not perform high-level terrain replanning. IMU, encoder, motor-current, and Vicon data may be used for tracking and experimental measurement, but not for online contact/motion replanning.

---

## 6. Terrain Height Must Be Included From the Beginning

Even if the first test is on flat ground, the interface should allow:

$$
z_{\text{TD}}\neq 0
$$

Flat ground is simply:

$$
z_{\text{TD}}=0
$$

A 50 mm obstacle is:

$$
z_{\text{TD}}=0.05
$$

The same planner should therefore work for:

```text
flat
different touchdown height
single rectangular obstacle
multiple sparse obstacles
```

without replacing the whole gait logic.

This is important because the final rough-terrain planner should be an extension of the flat-ground planner rather than a separate special-purpose staircase behavior.

---

## 7. Swing Planning Decision

Do not spend large amounts of time comparing many swing curves.

Current options in old code include:

```text
LINEAR
CUBIC
FIVETIMES
OPTIMIZE
```

These mostly plan directly in joint space:

$$
\theta(t),\beta(t)
$$

For obstacle traversal, the preferred new swing representation is:

$$
p_c(t)=
\begin{bmatrix}
x_c(t) & y_c(t) & z_c(t)
\end{bmatrix}^{\mathsf T}
$$

followed by inverse kinematics:

$$
p_c(t)
\rightarrow
(\theta,\beta,\gamma)
$$

Recommended first method:

```text
Cartesian Bezier swing trajectory
```

because it allows direct control of:

- touchdown location,
- touchdown height,
- obstacle clearance,
- approach direction,
- touchdown velocity.

Use the old FIVETIMES trajectory only as a baseline if needed.

Do not spend the first two weeks improving `OPTIMIZE`.

---

## 8. Terrain-Aware Contact Query

The first new core module should be something like:

```cpp
std::vector<ContactCandidate> queryContact(
    const LegPose& q,
    const TerrainProfile& terrain
);
```

For each rim sample point:

$$
p_r(\alpha,q)
$$

transform to world coordinates:

$$
{}^{W}\!p_r
=
{}^{W}\!T_B
{}^{B}\!T_L(\gamma)
p_r(\alpha,\theta,\beta)
$$

For a height field:

$$
z=f(x,y)
$$

define terrain gap:

$$
d=z_r-f(x_r,y_r)
$$

Then approximately classify:

```text
|d| < epsilon
    -> contact candidate

d < -epsilon
    -> collision / penetration

d > epsilon
    -> free
```

For a rectangular obstacle:

$$
f(x,y)=
\begin{cases}
h_o, & (x,y)\in\text{obstacle footprint} \\
0, & \text{otherwise}
\end{cases}
$$

The first implementation may be sampling-based.

Analytical perfection is not required.

---

## 9. Contact Transition Information

The contact representation should preferably preserve more information than a simple rim label.

For example:

```text
rim
alpha
contact point
distance to rim boundary
collision state
terrain surface ID
```

Possible edge metric:

$$
m_{\text{edge}}
=
\text{distance to nearest rim-transition boundary}
$$

This can later be used to detect states where a small forward motion may cause passive transition to another rim.

No probability model is required initially.

A deterministic geometric margin is enough.

---

## 10. Rolling-Assisted Obstacle Traversal

One especially important experiment is to determine whether a rectangular obstacle can be climbed **without a full swing**.

Example:

```text
            _________
           |
___________|
```

For a fixed or prescribed hip trajectory, sweep:

$$
h_o
$$

and initial configuration:

$$
(\theta_0,\beta_0)
$$

Then check whether a continuous contact trajectory exists:

$$
q_0 \rightarrow q_1 \rightarrow \cdots \rightarrow q_N
$$

subject to:

```text
valid rim contact
no collision
joint limits
leg workspace
continuous forward progress
```

This may produce three regions:

```text
LOW obstacle
    -> continuous rolling

MEDIUM obstacle
    -> rolling-assisted hybrid / short swing

HIGH obstacle
    -> full swing required
```

This result may become one of the central figures of the paper.

---

## 11. Development Scope for the First Two Weeks

The goal of the first two weeks is **not** to finish the complete ICRA system.

The goal is:

> Build a research prototype that is mature enough to start systematic experiments in Week 3.

Expected outcome after two weeks:

- single-leg flat hybrid trajectory,
- terrain-aware rectangle contact/collision query,
- rolling feasibility result for a block,
- swing trajectory to different touchdown heights,
- first offline roll-vs-swing contact/motion sequence,
- complete precomputed four-leg trajectory and simulation over at least one simple obstacle,
- optional first ABAD stability heuristic.

---

# 12. Two-Week Development Schedule

## Day 1–2 — Freeze Research Architecture

### Objectives

Finalize:

```text
research question
planner inputs/outputs
software module boundaries
data structures
```

### Must finish

- Define `TerrainProfile`.
- Define `ContactState`.
- Define `ContactCandidate`.
- Define `SwingTarget`.
- Define body / foothold coordinate conventions.
- Draw one complete planning flowchart.

### Deliverable

```text
Research architecture is frozen.
No major architecture redesign after Day 2 unless absolutely necessary.
```

### Implementation status (2026-08-23)

- [x] The four core data structures are defined in
  `legwheel/planners/hybrid/types.py`.
- [x] Coordinates, units, module boundaries, and the planner flowchart are
  frozen in `hybrid_note/notes/day1_2_hybrid_gait_architecture/hybrid_gait_architecture_day1_2.md`.
- [x] Data-contract tests are defined in
  `tests/test_hybrid_planning_types.py`.

---

## Day 3–5 — 2D Terrain-Aware Contact Model

### Scope

Only support:

```text
flat terrain
single rectangular obstacle
```

### Implement

```cpp
queryContact(theta, beta, terrain)
```

or equivalent.

### Output

At minimum:

```text
rim
alpha
contact point
collision flag
terrain gap
edge margin
```

### Visualization

Generate plots showing:

- leg geometry,
- obstacle,
- active rim,
- contact point,
- collision points.

### Completion criterion

For a given $(\theta,\beta)$, the program can correctly determine whether the leg:

```text
contacts flat ground
contacts obstacle top
collides with obstacle face
has no valid contact
```

---

## Day 6–7 — Single-Leg Rolling Feasibility

### Task

Create a single-leg test with a rectangular obstacle.

Sweep:

```text
obstacle height
initial theta
initial beta
approach configuration
```

### Question

Can the robot keep continuous rim contact and roll onto the obstacle?

### Completion criterion

Generate a feasibility result such as:

```text
obstacle height vs rolling feasibility
```

or:

```text
initial configuration vs maximum rollable obstacle height
```

This is a major research checkpoint.

---

## Day 8–9 — Swing to Different Terrain Heights

### Method

Use one Cartesian Bezier swing planner.

### Inputs

```text
start contact
target contact
target z
clearance
swing duration
```

### Required checks

- IK feasibility,
- joint limits,
- obstacle clearance,
- touchdown configuration,
- touchdown velocity.

### Completion criterion

The same swing planner can generate trajectories for:

```text
zTD = 0
zTD = 20 mm
zTD = 40 mm
zTD = ...
```

without changing the planner structure.

---

## Day 10–11 — First Offline Roll-vs-Swing Trajectory Planning

Combine:

```text
rolling feasibility
+
swing planner
```

The planner expands a contact/motion sequence over the complete known terrain and traversal path. First rule:

```text
if continuous rolling is feasible:
    use ROLL
else:
    use SWING
```

Possible future refinement:

```text
ROLL
ROLL + SHORT SWING
FULL SWING
```

but this is optional for the first version.

### Completion criterion

Before robot execution, the single-leg planner produces the complete test-terrain sequence:

```text
ROLL segment(s)
SWING segment(s)
target contact / rim / alpha / foothold for every transition
```

This is not an online runtime decision; execution only reads the completed trajectory.

---

## Day 12 — Four-Leg Full-Trajectory Integration

Reuse existing:

```text
duty
swing_phase
Hybrid::Step()
next_eta
```

Do not redesign the full timing system.

### First test

```text
symmetric obstacle
gamma = 0
```

### Completion criterion

Generate a synchronized, precomputed body/contact/joint trajectory for all four legs and complete at least one simple obstacle traversal in simulation.

---

## Day 13–14 — Minimal ABAD Stability Adjustment + Trajectory Generation Freeze

### First ABAD logic

For each swing leg:

1. Compute support polygon from remaining three contacts.
2. Compute CoM projection.
3. Evaluate stability margin.
4. If margin is too small, adjust selected support-leg $\gamma$.
5. Recompute support polygon.

### First version

Rule-based / quasi-static is acceptable.

Example:

```text
swing right leg
    -> widen left/support-side footholds
    -> increase lateral stability margin
```

### Completion criterion

At minimum:

- support triangle is computed correctly,
- ABAD changes lateral foothold location,
- planner can show improved static stability margin.
- ABAD adjustments are included in the complete joint-reference trajectory,
- the deterministic trajectory schema shared by simulation and hardware is frozen.

### End of Day 14

Freeze the main architecture.

Start experiments instead of continually rewriting the gait.

---

# 13. Week 3 and Later — Experiment + Paper Phase

From Week 3 onward, experiments and paper writing should proceed in parallel.

## Experimental progression

Use the following order:

```text
flat hybrid
    ↓
single low obstacle
    ↓
single higher obstacle
    ↓
whole robot over one obstacle
    ↓
left-right asymmetric obstacle
    ↓
multiple sparse asymmetric obstacles
```

Do not begin with the hardest rough terrain.

---

## 14. Target Experimental Terrain

Recommended final terrain:

> **Known structured asymmetric rough terrain composed of sparse rectangular obstacles.**

Example:

```text
Left side:

________      ____________
        |____|

Right side:

____________        ______
            |______|
```

or different obstacle heights:

```text
Left:
_______┌─────┐________________
       │     │

Right:
______________┌───┐__________
              │   │
```

Advantages:

- repeatable,
- measurable,
- easy to parameterize,
- easy to compare wheel / walk / hybrid,
- directly tests ABAD on left-right asymmetry,
- less ambiguous than random rocks.

---

# 15. Main Experimental Baselines

Primary comparison:

```text
Wheel
Walk
Proposed Hybrid
```

Do not create an excessive number of combinations.

Suggested metrics:

```text
success rate
traversal time
average speed
energy consumption
Specific Resistance / COT
pitch RMS / peak
roll RMS / peak
minimum stability margin
number of swing events
rolling-distance ratio
```

Define:

$$
R_{\text{roll}}
=
\frac{\text{distance traveled under rolling support}}
{\text{total traversal distance}}
$$

This metric is important because it demonstrates that the hybrid gait genuinely exploits rolling rather than behaving like ordinary walking.

---

## 16. Recommended Ablation

First ablation:

```text
Hybrid without ABAD
vs.
Hybrid with ABAD
```

Use asymmetric terrain.

Compare:

```text
roll RMS
minimum stability margin
success rate
body attitude variation
```

Optional second ablation if time permits:

```text
fixed contact policy
vs.
terrain-aware contact policy
```

Do not perform a large Cartesian product of:

```text
Cubic × Fifth-order × Bezier × Rim × ABAD × Step Height
```

unless there is a strong paper-specific reason.

---

# 17. Minimum Viable Paper Contribution

A weak version would be:

> “We wrote a new trajectory so the robot can step onto a block.”

Avoid this framing.

A stronger paper contribution is:

> **An offline multi-contact rolling–stepping gait planner for a leg-wheel robot with ABAD. Given known asymmetric discontinuous terrain, it precomputes the complete contact/motion sequence, explicitly evaluates multi-rim contact and rolling feasibility, schedules swing only when rolling violates geometry, kinematic, or stability constraints, and adjusts lateral support geometry with ABAD.**

Potential contribution bullets:

1. **Terrain-aware multi-rim contact representation**
   - supports discontinuous obstacle geometry,
   - identifies contact and collision states.

2. **Rolling / swing hybrid motion selection**
   - exploits continuous rim rolling whenever feasible,
   - reduces unnecessary swing motions.

3. **ABAD-based lateral stability adaptation**
   - changes support geometry on asymmetric terrain,
   - maintains CoM within a safe support region.

4. **Experimental validation**
   - compares Wheel / Walk / Hybrid,
   - evaluates energy, stability, traversal performance.

---

# 18. Important Scope Limitations

For the first version, intentionally do **not** implement:

```text
arbitrary full 3D terrain map
online depth perception
unknown-terrain planning
runtime terrain adaptation
receding-horizon replanning
real-time high-level planning requirement
full whole-body optimization
MPC
reinforcement learning
many swing trajectory families
complex probabilistic contact estimation
full dynamic stability model
```

Initial assumptions are acceptable:

```text
complete terrain known before planning
structured rectangular obstacles
2D or 2.5D terrain representation
quasi-static support stability
sampling-based contact query
rule-based roll/swing selection
offline full-trajectory generation
runtime joint-reference tracking
```

The purpose is to obtain a working research prototype quickly.

---

# 19. Development Priorities

Use this order when deciding what to work on.

```text
1. Contact representation
2. Rolling feasibility
3. Swing to arbitrary touchdown height
4. Roll-vs-swing selection
5. Four-leg integration
6. ABAD stability
7. Hardware robustness
8. Optimization refinements
```

If time becomes insufficient, cut features from the bottom of this list first.

---

# 20. Coding Principles for Codex

When modifying the project, preserve the separation:

```text
Planner
    decides:
        where to contact
        which rim
        roll or swing
        target body/contact geometry

Kinematics
    computes:
        forward
        inverse
        contact geometry
        rolling motion

Gait executor
    handles:
        duty
        swing phase
        stance phase
        next_eta

Controller
    sends:
        theta / beta / gamma commands
```

Avoid burying terrain logic directly inside `Hybrid::Step()`.

Prefer small testable functions/modules.

Recommended first modules:

```cpp
queryContact(...)
checkCollision(...)
checkRollingFeasibility(...)
generateSwing(...)
computeSupportPolygon(...)
computeStabilityMargin(...)
adjustABAD(...)
```

---

# 21. Current Old-Code Reference

Important files:

```text
wlw_open.cpp
Simple_fsm.cpp
hybrid_gen.cpp
hybrid_swing.cpp
leg_model.cpp
```

Conceptual responsibilities:

```text
wlw_open.cpp
    runtime / ROS / motor publishing

Simple_fsm.cpp
    gait shared state

hybrid_gen.cpp
    gait timing + stance/swing dispatch

hybrid_swing.cpp
    swing trajectory generation

leg_model.cpp
    forward / inverse / contact_map / move
```

The current low-level hybrid execution should be reused where possible.

---

# 22. Key Research Checkpoints

Before moving to the next stage, answer these questions.

### Contact model checkpoint

```text
Can the planner identify which rim can touch a rectangular obstacle?
Can it detect vertical-face collision?
Can it distinguish top-surface contact from ground contact?
```

### Rolling checkpoint

```text
Can the robot continuously roll onto some low obstacles?
What is the maximum rollable obstacle height?
Which configurations enable or disable rolling?
```

### Swing checkpoint

```text
Can one swing planner reach different touchdown heights?
Can it guarantee obstacle clearance?
Can it land with a feasible contact rim?
```

### Hybrid checkpoint

```text
Can the offline planner produce an executable roll-vs-swing sequence for the complete terrain?
Are contact, mode, body, and four-leg joint trajectories time-synchronized?
Does hybrid use fewer swing events than walking?
```

### ABAD checkpoint

```text
Does gamma change support geometry?
Does it increase minimum stability margin?
Does it reduce body roll on asymmetric terrain?
```

### Paper checkpoint

```text
Can the results clearly show:
wheel fails or becomes unstable,
walk succeeds but costs more energy,
hybrid succeeds while preserving rolling efficiency?
```

---

# 23. Two-Week Success Definition

The two-week development period is considered successful if the following are available:

```text
[ ] single-leg flat hybrid planner
[ ] rectangular terrain representation
[ ] terrain-aware contact / collision query
[ ] rolling feasibility analysis
[ ] swing to arbitrary touchdown height
[ ] first offline roll-vs-swing contact/motion sequence planner
[ ] synchronized four-leg full trajectory file
[ ] four-leg obstacle simulation using the precomputed trajectory
[ ] basic stability polygon calculation
[ ] optional ABAD adjustment
```

The goal is **not** to have a polished final ICRA system.

The goal is to reach a stable experimental prototype so that Week 3 can focus on:

```text
hardware experiments
parameter tuning
data collection
paper writing
figure generation
```

---

# 24. Paper Writing Schedule

Do not wait for all experiments to finish.

## During Week 1

Write rough notes for:

```text
Introduction
Motivation
Related work
Problem statement
Planner overview
```

## During Week 2

Write:

```text
Contact representation
Rolling feasibility formulation
Swing planner
Hybrid decision strategy
ABAD stability formulation
```

## Week 3 onward

Replace placeholders with:

```text
experimental setup
results
plots
tables
discussion
limitations
```

The paper should evolve together with the implementation.

---

# 25. Central Research Story

Use the following as the default project narrative:

> Wheeled locomotion is highly efficient but is limited by discontinuous and asymmetric terrain. Pure walking improves terrain traversability but sacrifices the energy advantage of rolling. The proposed hybrid gait therefore exploits continuous rim rolling whenever contact remains feasible, and introduces swing repositioning only when rolling is blocked by terrain geometry or stability constraints. The additional ABAD degree of freedom is used to modify lateral support geometry, allowing the robot to maintain stability while traversing left-right asymmetric obstacles. The resulting gait aims to preserve the efficiency of rolling while retaining the obstacle-negotiation capability of walking.

---

# 26. One-Sentence Design Principle

> **Roll whenever possible, step only when necessary, and use ABAD to keep the support geometry stable.**

---

# 27. Immediate Next Task

The next implementation task should be:

```text
Build a 2D terrain-aware contact query for:
    1. flat ground
    2. one rectangular obstacle
```

Required output:

```text
rim
alpha
contact point
terrain gap
collision state
edge margin
```

Once this is reliable, use it to perform the first rolling-feasibility experiment.

---

## Reference Files in Current Project

- `old_hybrid_gait_planning_note.md`
- `hybrid_gen.cpp`
- `hybrid_swing.cpp`
- `Simple_fsm.cpp`
- `wlw_open.cpp`
- `leg_model.cpp`
- `Adaptive_Hybrid_Locomotion_for_a_Leg-Wheel_Transformable_Robot_on_Uneven_Terrain.pdf`
- `電子論文ver2_R12_Thesis_YaTing_Hsu-1.pdf`
- `2026ICRA_Lee,Hsing-Chen_2199.pdf`
- `ICRA_YenLi_Lai_Final.pdf`

When asking Codex to modify the gait, provide this note together with the relevant implementation files rather than asking it to infer the entire research logic from one source file.
