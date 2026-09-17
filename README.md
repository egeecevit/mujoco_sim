# MuJoCo: What It Is and What It's Good For
## Summary

MuJoCo is a physics engine for simulating robots and other jointed mechanical systems. It
was built for **control and machine learning**, not for games or visual effects, and that
shapes everything about it: it is fast, it never crashes on a hard contact, and its
dynamics are smooth enough to differentiate which is what optimization and reinforcement
learning need.

The central trade-off: MuJoCo treats contacts as **very stiff springs** rather than perfectly
rigid walls. This makes it fast and numerically reliable, but means objects always sink into
surfaces slightly and can creep under sustained load. It is excellent for *how a robot
moves*, and the wrong tool for *exactly how hard two surfaces press together*.

---

## What it actually computes

MuJoCo simulates systems in **generalized coordinates**: a robot arm with 7 joints is
represented by 7 numbers, not by the positions of 8 separate rigid bodies held together by
constraints. Joints are therefore satisfied exactly and for free links cannot drift apart,
because separation isn't representable.

At each timestep it solves Newton's second law in that coordinate system:

$$M\dot v + c = \tau + J^T f$$

| Term | Meaning |
|---|---|
| $M\dot v$ | mass × acceleration, for a jointed system |
| $c$ | forces that exist just because the system is moving and in gravity (Coriolis, centrifugal, gravity) |
| $\tau$ | forces you apply motors, springs, dampers |
| $J^T f$ | forces from **contacts and constraints** |

The first three terms are textbook and every engine computes them the same way. The last
term contact is where engines differ, and where MuJoCo made its distinctive choice.

---

## How it handles contact

The physically exact statement of contact is a set of either/or conditions: either two
objects touch and push, or they are apart and don't. Solving that exactly, with friction, is
an NP-hard problem that can have multiple valid answers or none at all. Engines that attempt
it can stall, jitter, or fail.

MuJoCo doesn't attempt it. It **relaxes** the problem into a *convex optimization problem*
a shape of problem with exactly one solution, always findable. Physically, this is the same
as saying every contact is a stiff spring-damper with a small amount of give.

Three consequences follow, and they explain most of MuJoCo's behaviour:

**1. It always returns an answer.** There is no configuration that makes the contact solver
fail. For running millions of simulations unattended which is what RL training is this
reliability matters more than exactness.

**2. Objects sink in slightly.** A resting object penetrates the floor by a small, predictable
amount. With default settings, under Earth gravity, that is about **0.2 mm**. You can reduce
it, but only by stiffening the simulation, which forces smaller timesteps and slower
execution. You cannot reduce it to zero that limit is disallowed by design.

**3. Friction is approximate.** A heavy object held in a gripper may slowly creep. This is a
property of the relaxation, not a bug or a tuning failure, and no parameter setting fully
removes it.

---

## The solver: finding the contact forces

Each step, the solver must answer one question: *what contact forces should act right now?*

It does this by searching for the forces that come as close as possible to letting the system
move the way it would with no contact at all, while still respecting the limits of each
contact a surface can push but not pull, and friction cannot exceed what the normal force
allows. Because MuJoCo relaxed the problem into a convex one, this search has a single
correct answer, and any downhill search is guaranteed to reach it. The solver stops when it
is close enough (`tolerance`) or has tried long enough (`iterations`).

Note that this search is needed only when running the simulation **forwards**. Running it
backwards computing which forces produced an observed motion has a direct formula and
needs no search at all.

| Solver | How it works | Use it when |
|---|---|---|
| **Newton** *(default)* | Uses the curvature of the problem, not just the slope, to jump almost directly to the answer. Converges in a handful of iterations. | Almost always. It is the best option in essentially every case. |
| **CG** (conjugate gradient) | Follows improving directions using slope only, never building the curvature matrix. Cheaper per iteration, more iterations needed. | Very large contact counts, where forming curvature is the bottleneck. |
| **PGS** (projected Gauss-Seidel) | Adjusts one contact force at a time, sweeping over all contacts repeatedly. Simple and steady, but slow to converge precisely. | Legacy. It was the former default and is retained for compatibility. |
| **NoSlip** | Not a solver an optional extra pass after the main one that re-solves *only* friction, to reduce the creep described above. | Rarely. It is an ad hoc correction and can destabilize models with complex contact. |

---

## The integrator: advancing time

The solver produces an acceleration. The integrator converts that into new velocities and
positions over one `timestep`. This is where most instability problems originate: if forces
change sharply within a single step, a naive update overshoots, and the simulation gains
energy and explodes.

| Integrator | How it works | Use it when |
|---|---|---|
| **Euler** *(default)* | Updates velocity from the current acceleration, then position from the **new** velocity. Cheap and adequate for most models. Joint damping is handled in a look ahead manner automatically. | General purpose. |
| **RK4** | Evaluates the dynamics four times per step and blends the results. More accurate per step, roughly four times the cost. | Smooth systems with little or no contact, where accuracy matters more than speed. |
| **implicit** / **implicitfast** | Looks ahead: estimates what the forces will be at the *end* of the step and folds that into an effective mass. This stops blow ups caused by velocity dependent forces. `implicitfast` is the cheaper variant and usually the better choice for robots. | A model goes unstable with strong damping, aerodynamic drag/lift, or fast spinning bodies. |
| **discrete** | Extends the same look ahead to position dependent forces, and solves the contacts using that same effective mass. | Very stiff springs or high gain actuators. |

The practical rule: if a simulation is unstable, either shrink `timestep` or move from `Euler`
to `implicitfast`. The second is usually cheaper than the first.

---

## Other settings that matter

| Setting | Default | What it controls |
|---|---|---|
| `timestep` | 0.002 s | Accuracy vs. speed. The main stability lever. |
| `cone` | `pyramidal` | Friction model. `elliptic` is more physically accurate but slower. |
| `solref` | `0.02 1` | How **soft** contacts are the first number is the response time constant. |
| `solimp` | `0.9 0.95 0.001 0.5 2` | How **strongly** constraints are enforced. |

`solref` and `solimp` are the contact tuning controls, and they can be set per contact one
model can have a rigid floor and a compliant fingertip.

---

## What it's good for

- **Reinforcement learning.** Fast, deterministic, and allocation free once running. MJX and
  MuJoCo Warp extend it to thousands of simulations in parallel on GPU. Most robot learning
  benchmarks are built on it.
- **Model based control (MPC, trajectory optimization).** Smooth dynamics and the ability to
  run the physics *backwards* asking "what forces produced this motion?" at low, fixed
  cost. Most engines cannot do this; it is MuJoCo's strongest technical advantage.
- **Robot design and evaluation.** Realistic actuators: motors, position/velocity servos,
  pneumatic and hydraulic cylinders, adhesion grippers, and biological muscles.
- **Biomechanics.** Tendons that route through 3D space and wrap around bones, plus muscle
  models MuJoCo is a serious musculoskeletal tool, not only a robotics one.
- **Sim-to-real transfer.** Fast enough to randomize physical parameters across thousands of
  runs, which is the standard method for making learned policies survive contact with real
  hardware.

## What it can't do

- **Precise contact forces.** Soft contacts mean penetration and creep are built in. Wrong
  tool for grasp force metrology or friction characterization.
- **Fast moving thin objects.** Collisions are checked at discrete points in time, so a
  bullet can pass through a wall. The only fix is a smaller timestep.
- **Closed loops without give.** MuJoCo requires tree shaped mechanisms. Parallel linkages
  (delta robots, four bars) are modelled with soft constraints, so they stretch under load.
- **Concave shapes directly.** Collision geometry must be convex; concave meshes need to be
  decomposed into convex pieces first.
- **Changing the model mid run.** The model is fixed once loaded no breaking, cutting, or
  spawning new objects during a simulation without rebuilding it.
- **Non-rigid-body physics.** No fluid dynamics (only a simple drag/lift approximation), no
  heat, no electromagnetics. Soft body support exists for ropes and cloth but is not
  engineering grade FEM.
- **Realistic rendering.** The built in renderer is for debugging and pixel based learning.
  No photorealism, no physically simulated lidar or camera noise. Vision heavy work pairs
  MuJoCo with an external renderer.

---

## Bottom line

MuJoCo is the strongest available choice for simulating jointed systems **inside an
optimization or learning loop**. Its approximations to contact are deliberate, and they buy
the speed, reliability and smoothness that make that loop possible.

**Choose MuJoCo** for reinforcement learning, model predictive control, trajectory
optimization, system identification, and biomechanics.

**Choose something else** when the contact force itself is the measurement you need, when
deformation is central to the problem, or when the simulated sensor is a camera.
