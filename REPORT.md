# MuJoCo: A Technical Assessment

## 1. What it is

MuJoCo (**Mu**lti-**Jo**int dynamics with **Co**ntact) simulates articulated rigid bodies
in **generalized (minimal) coordinates** and resolves contact by **convex optimization**
rather than complementarity. It was designed for model-based control and optimization —
trajectory optimization, system identification, reinforcement learning.

One design decision explains most of the engine: MuJoCo gives up exact non-penetrating
Coulomb contact, and in exchange gets a contact problem that is convex, always solvable,
unique, and analytically invertible. Sections 3–4 derive this; section 7 lists what it
costs.

---

## 2. The equation it solves

MuJoCo evaluates dynamics in continuous time, then integrates. The governing equation is
the standard multibody equation of motion:

$$M(q)\,\dot v + c(q,v) \;=\; \tau \;+\; J^T f$$

| Symbol | Meaning | Computed by |
|---|---|---|
| $M$ | joint-space inertia ($n_v \times n_v$) | Composite Rigid-Body algorithm; stored sparse, factored $L^TDL$ so $M^{-1}x$ is a back-substitution |
| $c$ | bias force: Coriolis, centrifugal, gravity | Recursive Newton–Euler with acceleration set to $0$ |
| $\tau$ | applied force | actuation + passive (spring-damper, fluid) + user-applied |
| $J$ | constraint Jacobian | maps joint velocity to constraint space; $J^T$ maps force back |
| $f$ | constraint force | the convex program of §3 |

$M$ is always invertible, so once $f$ is known both directions close immediately:

$$\dot v = M^{-1}(\tau + J^T f - c)
\qquad\qquad
\tau = M\dot v + c - J^T f$$

Two structural consequences of minimal coordinates. Joint constraints hold **exactly by
construction**, so there is no drift to stabilize inside the kinematic tree. And $n_q > n_v$
whenever the model has quaternions (a free joint has 7 position coordinates, 6 DoF);
velocities live in the tangent space, and integration respects that manifold.

Everything above is standard. $f$ is where MuJoCo differs from other engines.

---

## 3. How the constraint force is computed

### Why not complementarity

Classical frictional contact is posed as a complementarity problem: normal force $\ge 0$,
gap $\ge 0$, product $= 0$. With friction this is NP-hard, and solutions may fail to exist
or be non-unique. MuJoCo drops strict complementarity and regularizes instead.

### Primal problem

Forward dynamics is the solution of a convex program generalizing **Gauss's principle of
least constraint** — the realized acceleration is the one closest to the unconstrained
acceleration in the inertia metric:

$$(\dot v, \dot\omega) = \arg\min_{(x,y)}\;
\big\|x - M^{-1}(\tau - c)\big\|^2_M +
\big\|y - a_r\big\|^{\mathrm{Huber}(\eta)}_{R^{-1}}$$

$$\text{s.t.}\quad
J_\mathcal{E}x_\mathcal{E} - y_\mathcal{E} = 0,\quad
J_\mathcal{F}x_\mathcal{F} - y_\mathcal{F} = 0,\quad
J_\mathcal{C}x_\mathcal{C} - y_\mathcal{C} \in \mathcal{K}^*$$

$x$ is acceleration; $y$ is a slack variable in constraint space, which is what makes
constraints soft. $R \succ 0$ is a diagonal regularizer and $a_r$ a reference acceleration
that stabilizes constraints. $R \to 0$ would give hard constraints; **MuJoCo does not allow
that limit.** Physically, $R$ acts as an inverse deformation inertia and $a_r$ as an
unforced deformation acceleration — the soft contact is an unmodeled deformation whose
dynamics shape the force but are not integrated.

### Dual problem — what is actually solved

$$f = \arg\min_{\lambda}\;
\tfrac12 \lambda^T (A + R)\,\lambda + \lambda^T (a_u - a_r),
\qquad \lambda \in \Omega$$

$$A = J M^{-1} J^T \quad\text{(Delassus operator)}
\qquad
a_u = J M^{-1}(\tau - c) + \dot J v$$

$A$ is positive *semi*-definite, but $R$ is positive definite by construction, so $A+R$ is
positive definite and the cost is **strictly convex**. A unique solution always exists and
varies continuously with the state. The solver can be inaccurate; it cannot fail to return
an answer.

### Friction cones

For one contact of dimensionality $n$ (`condim`) with friction coefficients $\mu$:

$$\mathcal{K}_{\text{elliptic}} = \Big\{ f \in \mathbb{R}^n : f_1 \ge 0,\;
f_1^2 \ge \textstyle\sum_{i=2}^{n} f_i^2/\mu_{i-1}^2 \Big\}
\qquad
\mathcal{K}_{\text{pyramidal}} = \Big\{ f \in \mathbb{R}^{2(n-1)} : f \ge 0 \Big\}$$

Pyramidal (the default) makes the problem a **box-constrained QP**. Elliptic is the true
Coulomb cone and makes it a **second-order cone program** — more faithful, more expensive.

`condim` is 1, 3, 4 or 6 (never 2 or 5): normal only, $+$ two tangential, $+$ torsional,
$+$ two rolling. Elliptic uses $n$ force components, pyramidal $2(n-1)$ — so 1, 4, 6 or 10.
Native torsional and rolling friction is uncommon among engines and matters for pivoting
and soft-finger grasps.

### One constraint set

Equality constraints, dry friction loss, joint/tendon limits and contacts are not separate
subsystems. They are rows of the same $J$ in the same program, differing only in the
feasible set of their multiplier: $\lambda_\mathcal{E}$ unconstrained,
$|\lambda_\mathcal{F}| \le \eta$, $\lambda_\mathcal{C} \in \mathcal{K}$. One code path and
one set of tuning parameters covers all four.

---

## 4. Contact softness is a parameter

Every constraint approximately obeys, in constraint space:

$$a_c + d\,(b v + k r) = (1-d)\,a_u$$

$r$ is the residual (penetration depth, limit or equality violation), $d$ the **impedance**,
and $b, k$ the damping and stiffness of the virtual spring-damper defining $a_r = -bv - kr$.
The impedance interpolates between no constraint ($d \to 0$) and a hard constraint
($d \to 1$). It is clamped internally to $[10^{-4},\,0.9999]$ — never exactly 1.

The regularizer follows from the impedance:

$$R_{ii} = \frac{1 - d_i}{d_i}\,\hat A_{ii}$$

$\hat A$ is an *approximation* to $\mathrm{diag}(A)$, built from end-effector inertias frozen
at `qpos0`, so the sparse solver and inverse dynamics never form $A$. It assumes isotropy
and ignores kinematic coupling, so realized impedance drifts from specified impedance for
anisotropic inertias or long chains far from `qpos0`. The `diagexact` flag substitutes the
exact $A_{ii} = \|Y_i\|^2$ with $Y = JM^{-1/2}$.

$d$, $b$ and $k$ are set indirectly, via two attributes on every constraint-bearing element:

- **`solimp`** $=(d_0, d_\mathrm{w}, \text{width}, \text{midpoint}, \text{power})$, default
  `0.9 0.95 0.001 0.5 2`. Defines a sigmoid $d(r)$ with $d(0)=d_0$, $d(\text{width})=d_\mathrm{w}$.
- **`solref`** $=(\text{timeconst}, \text{dampratio})$, default `0.02 1` (critically damped,
  two timesteps), giving

$$b = \frac{2}{d_\mathrm{w}\cdot \text{timeconst}}
\qquad
k = \frac{d(r)}{d_\mathrm{w}^2 \cdot \text{timeconst}^2 \cdot \text{dampratio}^2}$$

**Practical consequence.** For constant impedance, the resting penetration depth is

$$r = a_u\,(1-d)\cdot \text{timeconst}^2 \cdot \text{dampratio}^2$$

A resting object always sinks into the floor by a known amount. Under Earth gravity with
defaults ($a_u = 9.81$, $d = 0.95$, timeconst $= 0.02$):
$r = 9.81 \times 0.05 \times 0.02^2 \approx 2\times10^{-4}\,\text{m}$, about 0.2 mm.
Shrinking `timeconst` or raising $d$ reduces it quadratically, but stiffens the system and
forces a smaller timestep. It cannot be driven to zero. This formula is the main handle for
tuning contact behaviour.

---

## 5. Solving and stepping

### Constraint solvers

Numerical solvers are needed **only in forward dynamics**. All accept either cone type and
dense or sparse Jacobians.

| Solver | Formulation | Method |
|---|---|---|
| **Newton** *(default)* | reduced primal | Exact Newton: analytic second derivatives, Cholesky factorization of the Hessian, rank-1 updates when constraint states switch, exact line search. Stops when cost improvement, gradient norm, **or** the Newton decrement $\tfrac12 g^TH^{-1}g$ drops below `tolerance`. |
| **CG** | reduced primal | Nonlinear conjugate gradient (Hager–Zhang), same exact line search, no setup cost. |
| **PGS** | dual | Projected Gauss-Seidel, one coordinate at a time. First-order convergence; one sweep ≈ one matrix-vector product. The former default. |
| **NoSlip** | — | Post-processing pass, not a solver: re-solves only the friction rows with $R=0$ to suppress soft-constraint slip. Documented as ad-hoc — the cascade no longer solves one well-defined problem and can destabilize multi-contact models. |

### Integrators

| Integrator | Update | Use for |
|---|---|---|
| `Euler` *(default)* | semi-implicit: $v_{t+h} = v_t + ha_t$, then $q_{t+h} = q_t + hv_{t+h}$; joint damping integrated implicitly unless `eulerdamp` is disabled | general purpose |
| `RK4` | fixed-step 4th-order Runge-Kutta | smooth, contact-light systems |
| `implicit` / `implicitfast` | $v_{t+h} = v_t + h\widehat M^{-1}Ma(v_t)$, $\widehat M \equiv M + hD$, $D \equiv -\partial(\tau - c + J^Tf)/\partial v$; `implicitfast` drops the Coriolis/centrifugal derivative | velocity-dependent instability: damping, lift/drag, tumbling bodies |
| `discrete` | $\widehat M a = f(q_t,v_t) - hKv_t + J^Tf_c$ with $\widehat M \equiv M + hD + h^2K$; constraints solved in that effective-inertia metric | stiff position-dependent forces |

The implicit family is the standard answer to instability at a given timestep: it folds the
velocity Jacobian $D$ into an effective inertia, at roughly one extra factorization.

### The step

Forward kinematics → CRB ($M$) and RNE ($c$) → collision detection (modified sweep-and-prune
broadphase, static AABB bounding-volume-hierarchy midphase, analytic primitives plus GJK/EPA
narrowphase) → assemble $J$, $R$, $a_r$ → solve for $f$ → integrate.

`mjModel` is immutable and `mjData` preallocated, so a step performs **no runtime memory
allocation** — the property that makes MuJoCo usable inside an MPC loop.

---

## 6. Analytic inverse dynamics

Given $(q, v, \dot v)$ rather than $(q, v, \tau)$, the constrained acceleration
$a_c = J\dot v + \dot J v$ is known while $a_u$ is not. The constraint force then solves:

$$f = \arg\min_{\lambda}\; \tfrac12 \lambda^T R\,\lambda + \lambda^T(a_c - a_r),
\qquad \lambda \in \Omega$$

The quadratic term is $R$ alone rather than $A+R$, and **$R$ is diagonal**. The problem
decouples into independent scalar problems: no matrix inversion, no factorization, no
iteration — closed form. The two formulations agree through

$$a_c = a_u + A f$$

which is Newton's second law projected into constraint space. This is why MuJoCo runs
backwards at fixed cost, and why it suits trajectory optimization over position sequences,
system identification, and estimation.

**Documented caveat.** Computing $\dot J v$ requires differentiating the constraint Jacobian
in time. MuJoCo does this for `connect` and `weld` equality constraints but **omits it for
contacts**, since differentiating the contact frame through the collision pipeline is
intractable. The term cancels in the identity above, so forward-inverse consistency is
unaffected — but its omission biases forward dynamics for any constraint whose Jacobian
varies with configuration.

---

## 7. Assessment

### Strengths

- **Articulated mechanisms.** Minimal coordinates: no constraint drift in the tree, no
  stabilization, state proportional to actual DoF.
- **Speed and determinism.** Allocation-free stepping, sparse tree-aware linear algebra,
  few-iteration convergence. MJX (JAX) and MuJoCo Warp extend this to large GPU-parallel
  rollout batches.
- **Model-based control.** Analytic inverse dynamics, finite-difference transition
  derivatives (`mjd_transitionFD`), and dynamics that are smooth when contacts use
  `solimp[0]=0`. On complementarity engines contact makes dynamics discontinuous; here
  gradient-based trajectory optimization and MPC are practical.
- **Actuation.** Motors, position/velocity servos, pneumatic and hydraulic cylinders,
  Hill-type muscles, adhesion — each built from a transmission, optional activation state,
  and gain/bias law. With 3D tendon routing and wrapping, this covers musculoskeletal
  modelling as well as robotics.
- **Uniform constraint semantics.** Limits, equalities, friction loss and contacts share one
  parameterization, so tuning intuition transfers between them.

### Limitations

Each follows from the model, not from a missing feature.

- **No exact non-penetration or exact Coulomb friction.** $R \to 0$ is disallowed, so
  penetration is structural (§4). Creep and slow slip under sustained load are properties of
  the convex relaxation, not tuning errors. NoSlip suppresses the symptom at the cost of a
  well-posed problem. Not an instrument for measuring absolute contact forces.
- **No kinematic loops.** Tree topology is required; loops are closed with soft equality
  constraints, so parallel mechanisms stretch under load.
- **No continuous collision detection.** Collision is evaluated once per fixed step at
  discrete configurations; there is no swept or time-of-impact test, so fast thin objects
  tunnel. The only mitigation is a smaller timestep.
- **Convex geometry only.** Concave meshes need convex decomposition, height fields, or SDF
  plugins.
- **No runtime topology change.** `mjModel` is constant after compilation — no fracture,
  cutting, or spawning/destroying bodies mid-episode. `mjSpec` allows procedural editing but
  still ends in a recompile.
- **Rigid multibody only.** Fluid interaction is a lumped ellipsoid model (drag, added mass,
  Magnus), not Navier-Stokes. No thermal, electromagnetic or acoustic physics.
- **`flex` is not structural FEM.** Cables, membranes, shells and soft solids are adequate
  for robotics contact, not for material analysis.
- **Rendering is not perception-grade.** The OpenGL rasterizer serves debugging and
  pixel-based RL: no physically based materials, ray tracing, lidar/radar physics, or sensor
  noise models.
- **Stiffness still constrains the timestep.** Extreme mass ratios, stiff contacts and
  high-gain controllers force small $h$; the implicit and `discrete` integrators widen the
  envelope without removing the limit.
- **Impedance is approximate** where tendon or flex coupling dominates, because $R$ uses the
  frozen diagonal approximation $\hat A$ (§4).

### Conclusion

MuJoCo is the strongest available tool for simulating articulated systems **inside an
optimization or learning loop**. The convex, regularized, invertible contact model is why it
is fast, why it always returns a solution, and why it is smooth enough for gradient-based
control — and equally why its contact forces should be treated as consistent and plausible
rather than physically exact.

Use it for reinforcement learning, MPC, trajectory optimization, system identification,
biomechanics, and sim-to-real with domain randomization. Use something else when the
quantity of interest is the contact force itself, when deformation is materially important,
or when the simulated sensor is a camera.

---

## Defaults

| Attribute | Default | Attribute | Default |
|---|---|---|---|
| `timestep` | `0.002` | `iterations` | `100` |
| `integrator` | `Euler` | `tolerance` | `1e-8` |
| `solver` | `Newton` | `ls_iterations` | `50` |
| `cone` | `pyramidal` | `impratio` | `1` |
| `jacobian` | `auto` | `solref` | `0.02 1` |
| | | `solimp` | `0.9 0.95 0.001 0.5 2` |

## Notation

| | |
|---|---|
| $q, v, \dot v$ | position ($n_q$), velocity ($n_v$), acceleration; $n_q > n_v$ with quaternions |
| $M, c, \tau$ | inertia matrix, bias force, applied force |
| $J, f$ | constraint Jacobian, constraint force |
| $A = JM^{-1}J^T$ | Delassus operator (inverse inertia in constraint space) |
| $R, d$ | diagonal regularizer, impedance $\in (0,1)$ |
| $a_u, a_c, a_r$ | unconstrained, constrained, reference acceleration in constraint space |
| $r$ | constraint residual (penetration depth, limit or equality violation) |
| $\mathcal{K}, \Omega$ | friction cone; feasible set of the dual multipliers |
| $h$ | integration timestep |

*Equations and defaults verified against the MuJoCo documentation sources
(`doc/computation/index.rst`, `doc/modeling.rst`, `doc/XMLreference.rst`,
`google-deepmind/mujoco@main`).*
