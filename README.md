# MPC Framework: Thermodynamic Chemical Reactor

A comprehensive structural outline and mathematical blueprint for a Model Predictive Control (MPC) framework tailored for an open-loop unstable, thermodynamically-driven, single-unit chemical reactor with high observability, state/output constraints, bounded uniform noise, and a controlled Relative Gain Array (RGA) strategy.

---

## 1. System Mathematical Model (State-Space Form)

Since the system is open-loop unstable and driven by thermodynamics (pure energy/mass balances without complex kinetic/transport PDEs), we represent it as a continuous-time (or discretized) linear or mildly nonlinear state-space system.

Let the state vector be $x \in \mathbb{X} \subset \mathbb{R}^n$, the control input be $u \in \mathbb{U} \subset \mathbb{R}^m$, and the process noise be $w \in \mathbb{W} \subset \mathbb{R}^n$ (bounded uniform noise, e.g., $\Vert w(t)\Vert_\infty \le \epsilon_w$).

### State-Space Equations

**Linearized / Nominal Form:**
$$\dot{x}(t) = A x(t) + B u(t) + w(t)$$
$$y(t) = C x(t)$$

**Mildly Nonlinear Form (if explicitly capturing thermodynamic state couplings):**
$$\dot{x}(t) = f(x(t)) + g(x(t))u(t) + w(t)$$
$$y(t) = h(x(t))$$

### Key Properties
* **Open-Loop Instability:** At least one eigenvalue of $A$ (or the Jacobian $\frac{\partial f}{\partial x}$) has a positive real part ($\mathrm{Re}(\lambda_i) > 0$).
* **Observability:** The observability matrix $$
\mathcal{O} = \begin{bmatrix} C^\top & (CA)^\top & \dots & (CA^{n-1})^\top \end{bmatrix}^\top
$$ has full rank $n$ (high inherent observability, ensuring the assumed state estimator provides an accurate full-state vector $\hat{x} \approx x$).
* **No Non-Minimum Phase Zeroes:** Transmission zeroes (or multivariable equivalents) lie strictly in the left half-plane, meaning no inverse-response behaviors that fight high-bandwidth tracking.

---

## 2. Relative Gain Array (RGA) Integration & Input Bundling

To bundle system states and control inputs while controlling interaction, we use the RGA matrix $\Lambda$:

$$\Lambda = G(0) \times \left(G(0)^{-1}\right)^\top$$

where $G(0)$ is the steady-state gain matrix.

### Initial Design ($\Lambda_{ij} \in [0.5, 0.7]$)
An RGA value in this range indicates moderate interaction between loops. This is deliberate: it prevents excessive loop-fighting while keeping the pairing meaningful enough to test decoupling structures.

### State/Input Bundling
We structure the input weighting matrix $R$ in the MPC cost function based on the RGA pairing inverse metrics, penalizing cross-coupling channels appropriately so that the optimizer favors decentralized authority along well-conditioned directions.

---

## 3. Problem Workflow & Theoretical Guarantees

Because the plant is open-loop unstable, standard MPC formulations can easily drive the system unbounded during constraint activation or estimation errors unless stability and robustness proofs are explicitly embedded.

### A. Input-to-State Stability (ISS) & Lipschitz Continuity
* **Lipschitz Continuity:** To guarantee that the controller and plant dynamics do not exhibit finite-time blow-ups or erratic jumps under bounded noise, the vector fields $f(x)$ and $g(x)$ (or matrices $A, B$) are globally Lipschitz continuous on compact sets $\mathbb{X}$ and $\mathbb{U}$:
  $$\Vert f(x_1) - f(x_2)\Vert \le L_f \Vert x_1 - x_2\Vert$$
* **ISS Bounds:** Given bounded uniform noise $w$, we establish Input-to-State Stability for the closed-loop system under the MPC feedback law $u = \kappa_{MPC}(x)$. The state trajectory satisfies:
  $$\Vert x(t)\Vert \le \beta(\Vert x(0)\Vert, t) + \gamma(\Vert w\Vert_\infty)$$
  where $\beta$ is a class $\mathcal{KL}$ function and $\gamma$ is a class $\mathcal{K}$ function scaling linearly/nonlinearly with the noise bound $\epsilon_w$.

### B. Control Lyapunov Functions (CLFs) for Stability
To ensure recursive feasibility and closed-loop stability for the unstable reactor, we augment the MPC terminal cost or terminal region using a local Control Lyapunov Function $V(x)$:
* **CLF Condition:** There exists a control law $u = k(x)$ such that:
  $$\frac{\partial V}{\partial x} (f(x) + g(x)k(x)) \le -\alpha V(x), \quad \alpha > 0$$
  In the optimization framework, this is integrated either as a terminal equality constraint ($x(N|t) = 0$), a terminal region constraint ($x(N|t) \in \Omega_f$ where $V(x) \le c$), or as a CLF-based descent constraint enforced at each horizon step to counteract open-loop instability.

### C. Control Barrier Functions (CBFs) for Safety & Constraints
Since the reactor features strict state and output constraints ($\mathbb{X}, \mathbb{Y}$), we utilize Control Barrier Functions to guarantee forward invariance of the safe set.

Let the safe operating envelope (e.g., maximum reactor temperature, pressure limits) be defined by the superlevel set of a continuously differentiable function $h(x) \ge 0$.

* **CBF Condition (Exponential/Reciprocal):**
  $$\dot{h}(x, u) + \alpha(h(x)) \ge 0 \quad \forall x \in \mathbb{X}$$
  The MPC optimization problem incorporates this as a real-time affine constraint on the control action vector $u$, ensuring that even in the presence of bounded uniform noise $w$, the states remain strictly inside physical safety boundaries.

---

## 4. The Quadratic Programming (QP) / Convex Optimization Problem

With the components above, the finite-horizon optimal control problem (FHOCP) solved at each sampling instant $t$ takes the form of a standard Convex Quadratic Program (QP):

$$\min_{U_t} \sum_{k=0}^{N-1} \left( \Vert x_{t+k|t} - x_r\Vert_{Q}^2 + \Vert u_{t+k|t}\Vert_{R}^2 \right) + \Vert x_{t+N|t}\Vert_{P}^2$$

### Subject to:
* **System Dynamics:**
  $$x_{t+k+1|t} = A x_{t+k|t} + B u_{t+k|t}$$
* **Input Constraints:**
  $$u_{min} \le u_{t+k|t} \le u_{max}$$
* **State & Output Constraints:**
  $$C x_{t+k|t} \in \mathbb{Y}, \quad x_{t+k|t} \in \mathbb{X}$$
* **CLF Terminal Stability Constraint:**
  $$V(x_{t+N|t}) \le \rho$$
* **CBF Safety Constraints:**
  $$A_{cbf} u_{t+k|t} \le b_{cbf}$$

---

## 5. Future Expansion Roadmap: Accommodating Increased RGA Values

To scale up the project later by increasing the RGA value (moving toward $\Lambda_{ij} \gg 1$, representing strong multivariable coupling and ill-conditioned plant dynamics):

1. **Transition from Diagonal to Full Weighting Matrices:**
   * *Initial ($\Lambda \approx 0.5-0.7$):* $R$ and $Q$ matrices can be largely diagonal, treating cross-coupling as mild disturbances.
   * *Increased RGA:* Shift $R$ and $Q$ to full, non-diagonal matrices optimized via Linear Matrix Inequalities (LMIs) or $\mathcal{H}_\infty$/$\mu$-synthesis tools to explicitly penalize directional sensitivity and input amplification.

2. **Incorporate Decentralized / Distributed MPC Structures:**
   * As RGA increases, centralized MPC can suffer from numerical sensitivity. You can split the single-unit reactor loops into sub-units using a Distributed Cooperative MPC architecture with consensus constraints on shared states.

3. **Robust Tube-MPC Augmentation:**
   * Higher RGA amplifies noise propagation through cross-coupling channels. Introduce a Tube MPC framework (nominal path planner + robust feedback tracking controller) to tightly bound the effects of the uniform bounded noise $w$ as interaction gains grow.
