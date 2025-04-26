# Theory notes

The derivations behind every model, design rule and metric used in State-Space Studio. Symbols follow the code: `A, B, C, D, E` are the plant matrices, `K` a state-feedback gain, `V` a prefilter, `w` the reference and `z` a disturbance.

- [1. State-space models and exact simulation](#1-state-space-models-and-exact-simulation)
- [2. Structural properties](#2-structural-properties)
- [3. The plants](#3-the-plants)
- [4. Pole placement with Ackermann's formula](#4-pole-placement-with-ackermanns-formula)
- [5. Prefilter and disturbance offsets](#5-prefilter-and-disturbance-offsets)
- [6. PI state feedback](#6-pi-state-feedback)
- [7. Linear-quadratic regulator](#7-linear-quadratic-regulator)
- [8. Performance metrics](#8-performance-metrics)

## 1. State-space models and exact simulation

Every plant is written as

```math
\dot{x}(t) = A x(t) + B u(t) + E z(t), \qquad y(t) = C x(t) + D u(t)
```

with the solution

```math
x(t) = e^{At} x(0) + \int_0^t e^{A(t-\tau)} \big(B u(\tau) + E z(\tau)\big) \, d\tau .
```

On a uniform grid with step $h$ the integral can be evaluated exactly when the input follows a known shape between samples. For a **zero-order hold** (input constant on $[t_k, t_{k+1})$, exact for steps on the grid) one matrix exponential gives both discrete matrices (Van Loan's method):

```math
\exp\left(\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} h\right)
= \begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix},
\qquad x_{k+1} = A_d x_k + B_d u_k .
```

For a **first-order hold** (input linear between samples, exact for ramps) the augmented matrix gains one more block:

```math
\exp\begin{bmatrix} A h & B h & 0 \\ 0 & 0 & I \\ 0 & 0 & 0 \end{bmatrix}
= \begin{bmatrix} A_d & \Gamma_1 & \Gamma_2 \\ \cdot & \cdot & \cdot \\ \cdot & \cdot & \cdot \end{bmatrix},
\qquad x_{k+1} = A_d x_k + (\Gamma_1 - \Gamma_2)\, u_k + \Gamma_2\, u_{k+1} .
```

`state_space_studio.lti.simulate` implements both; the tests compare them with `scipy.signal.lsim` (agreement to 1e-9) and with closed-form first-order step and ramp responses (1e-12).

## 2. Structural properties

**Stability.** With eigenvalues $\lambda_i$ of $A$:

| Condition | Class |
| --- | --- |
| all $\operatorname{Re}\lambda_i < 0$ | asymptotically stable |
| no $\operatorname{Re}\lambda_i > 0$, imaginary-axis eigenvalues simple | marginally stable |
| any $\operatorname{Re}\lambda_i > 0$, or a nontrivial Jordan block on the imaginary axis | unstable |

Repeated imaginary-axis eigenvalues alone do not establish instability: semisimple repeats are marginally stable. The eigenvalue-only helper reports these as undetermined because eigenvalues cannot reveal Jordan-block sizes.

**Controllability and observability** follow from the Kalman rank tests

```math
Q_c = \begin{bmatrix} B & AB & \cdots & A^{n-1}B \end{bmatrix}, \qquad
Q_o = \begin{bmatrix} C \\ CA \\ \vdots \\ CA^{n-1} \end{bmatrix}, \qquad
\operatorname{rank} Q_c = \operatorname{rank} Q_o = n .
```

All five plants pass both tests, so all closed-loop poles can be placed freely and all states could be reconstructed by an observer.

## 3. The plants

### RC ladder

Three RC stages; $v_i$ is the voltage over capacitor $C_i$, the source $u$ drives the first resistor:

```math
C_1 \dot{v}_1 = \frac{u - v_1}{R_1} - \frac{v_1 - v_2}{R_2}, \quad
C_2 \dot{v}_2 = \frac{v_1 - v_2}{R_2} - \frac{v_2 - v_3}{R_3}, \quad
C_3 \dot{v}_3 = \frac{v_2 - v_3}{R_3}, \quad y = v_3 .
```

With $R_i = 4\ \Omega$ and $C_i = 1$ F the poles are $-0.812, -0.389, -0.0495$ s⁻¹; the DC gain is 1 because no current flows at rest.

### Coupled two-input system

```math
A = \begin{bmatrix} -3 & 0 & 0 \\ 0 & -2 & 1 \\ 10 & -17 & 0 \end{bmatrix}, \quad
B = \begin{bmatrix} 4 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad
C = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix} .
```

$x_1$ is decoupled ($\lambda = -3$) but feeds $x_3$; the $(x_2, x_3)$ block has $s^2 + 2s + 17$, i.e. $-1 \pm 4j$. The plane $x_1 = 0$ is invariant, which is why the trajectories end on a spiral.

### Mass–spring–damper

$m\ddot{x} + b\dot{x} + cx = u$ with $m = 0.05$ kg, $b = 1$ N s/m, $c = 4$ N/m gives $s^2 + 20 s + 80$, so the poles are $-10 \pm \sqrt{20} = -5.53$ and $-14.47$ (overdamped, $\zeta = 1.12$) and the static deflection is $u/c = 0.25$ m.

### Three-mass torsional drivetrain

With $\varphi = (\varphi_1, \varphi_2, \varphi_3)^\top$ the equations of motion are

```math
M \ddot{\varphi} + D_v \dot{\varphi} + K_s \varphi = b\, T_1 + T_z, \qquad
M = \operatorname{diag}(J_1, J_2, J_3), \quad D_v = \operatorname{diag}(d_1, d_2, d_3),
```

```math
K_s = \begin{bmatrix} k_{12} & -k_{12} & 0 \\ -k_{12} & k_{12} + k_{23} & -k_{23} \\ 0 & -k_{23} & k_{23} \end{bmatrix}, \qquad
b = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} .
```

With $x = (\varphi, \dot{\varphi})$:

```math
A = \begin{bmatrix} 0 & I \\ -M^{-1} K_s & -M^{-1} D_v \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ M^{-1} b \end{bmatrix}, \quad
E = \begin{bmatrix} 0 \\ M^{-1} \end{bmatrix}, \quad
C = \begin{bmatrix} 0 & 0 & 1 & 0 & 0 & 0 \end{bmatrix} .
```

Because $K_s \mathbf{1} = 0$ (there is no spring to ground), $x = (1, 1, 1, 0, 0, 0)^\top$ satisfies $Ax = 0$: a **rigid-body mode** at $\lambda = 0$. The plant is therefore only marginally stable and a constant torque accelerates it to $\omega_\infty = T_1 / \sum d_i$. The remaining poles are

| Mode | Pole | $\omega_n$ [rad/s] | $f_d$ [Hz] | $\zeta$ |
| --- | --- | --: | --: | --: |
| rigid body (angle) | $0$ | – | – | – |
| rigid body (speed) | $-8.76$ | – | – | 1 |
| first torsional | $-9.20 \pm 46.6j$ | 47.5 | 7.42 | 0.194 |
| second torsional | $-10.3 \pm 104j$ | 104.5 | 16.55 | 0.099 |

### Three-tank process

Linearised mass balances with levels in percent: the pump fills tank 1, valves $k_{12}, k_{23}$ exchange liquid in proportion to the level difference and each tank drains with $k_i$:

```math
\begin{aligned}
\dot{h}_1 &= -(k_1 + k_{12}) h_1 + k_{12} h_2 + k_{in} u + k_z z \\
\dot{h}_2 &= k_{12} h_1 - (k_{12} + k_{23} + k_2) h_2 + k_{23} h_3 \\
\dot{h}_3 &= k_{23} h_2 - (k_{23} + k_3) h_3
\end{aligned}
```

The column sums of $A$ are $-k_1, -k_2, -k_3$: the valves only redistribute liquid, the total changes through the drains alone. The DC gain from pump to $h_3$ is 2.675.

## 4. Pole placement with Ackermann's formula

For a single-input pair $(A, B)$ and a desired characteristic polynomial $\varphi_d(s) = \prod (s - p_i)$,

```math
K = e_n^\top Q_c^{-1} \varphi_d(A), \qquad e_n^\top = \begin{bmatrix} 0 & \cdots & 0 & 1 \end{bmatrix},
```

places $\operatorname{eig}(A - BK) = \{p_i\}$, repeated poles included (SciPy's `place_poles` refuses pole multiplicities above $\operatorname{rank} B$). `state_space_studio.design.ackermann` evaluates $\varphi_d(A)$ with Horner's scheme and solves $Q_c^\top q = e_n$ instead of inverting $Q_c$. For the six-state drivetrain $Q_c$ spans many orders of magnitude, so the test suite checks the achieved characteristic polynomial (relative error below $10^{-9}$) rather than the individual repeated eigenvalues, which are ill-conditioned by nature.

**Dominant poles from a settling-time target.** The rule of thumb $t_s \approx 4 / (\zeta \omega_n)$ (2 % band) with $\zeta = 1$ and $t_s = 200$ s gives a double pole at $-\omega_n = -0.02$ s⁻¹. This is an envelope-based approximation, not an exact settling-time formula. For a critically damped pair the step response is

```math
y(t) = 1 - (1 + \omega_n t)\, e^{-\omega_n t},
```

which enters the 2 % band at $\omega_n t = 5.83$, i.e. **292 s** for the tanks and **0.58 s** for the drivetrain's pair at $-10$; the simulations measure 296–301 s and 0.62 s.

## 5. Prefilter and disturbance offsets

With $u = Vw - Kx$ the closed loop is $\dot{x} = (A - BK)x + BVw + Ez$. At steady state $\dot{x} = 0$:

```math
x_\infty = (BK - A)^{-1} (B V w + E z), \qquad y_\infty = C x_\infty .
```

Requiring $y_\infty = w$ for $z = 0$ gives the prefilter

```math
V = \left[ C (BK - A)^{-1} B \right]^{-1} .
```

In these plants, $C(BK - A)^{-1} E z$ is nonzero for the tested constant loads, leaving an output offset. A prefilter normalises reference tracking but does not guarantee disturbance rejection.

**The drivetrain's virtual spring.** For a rigid-body offset $\varphi_1 = \varphi_2 = \varphi_3 = \theta$ the feedback produces $u = -(K_1 + K_2 + K_3)\theta$, and the prefilter of this plant is exactly $V = K_1 + K_2 + K_3 = 1.067$ N m/rad. A load $T_{z3} = 1$ N m on the load inertia must be carried by both shafts, so at rest $\varphi_1 - \varphi_2 = \varphi_2 - \varphi_3 = -T_{z3}/k = -0.667$ rad and $u = -1$ N m. Inserting this twist into $u = -K x$:

```math
-1 = -\big(V \varphi_3 + K_1 (\varphi_1 - \varphi_3) + K_2 (\varphi_2 - \varphi_3)\big)
\;\Rightarrow\;
\varphi_3 = \frac{1 + (4/3)K_1 + (2/3)K_2}{V} \approx -36.6 \text{ rad}.
```

The large, opposite-signed angle gains turn a small twist into a large torque bias that only the weak virtual spring $V$ can balance. A load on the motor inertia causes no twist, hence only $1/V = 0.94$ rad.

## 6. PI state feedback

Add the integral of the control error as a state, $\dot{x}_i = e = w - y$, and place all $n + 1$ poles of

```math
A_a = \begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix}, \qquad
B_a = \begin{bmatrix} B \\ 0 \end{bmatrix}, \qquad
K_a = \begin{bmatrix} K_x & -K_i \end{bmatrix} .
```

The implemented law adds a proportional path on the error,

```math
u = K_p e + K_i x_i - K_{pi} x, \qquad K_{pi} = K_x - K_p C, \qquad K_p = \frac{\alpha}{-C A^{-1} B},
```

which expands to $u = -K_x x + K_i x_i + K_p w$: the state part is unchanged, so the closed-loop poles stay exactly where they were placed and $K_p$ only shapes the reference response (zeros). At any steady state $\dot{x}_i = 0$ forces $e = 0$, whatever constant disturbance acts on the plant — this is why the PI design is the only one with zero error after the leak.

## 7. Linear-quadratic regulator

Minimising $J = \int_0^\infty (x^\top Q x + u^\top R u)\, dt$ leads to the continuous-time algebraic Riccati equation

```math
A^\top P + P A - P B R^{-1} B^\top P + Q = 0, \qquad K = R^{-1} B^\top P,
```

solved with `scipy.linalg.solve_continuous_are`. For a scalar plant $\dot{x} = ax + bu$ this reduces to $K = \big(a + \sqrt{a^2 + b^2 q / r}\big)/b$, which the tests use as a reference. For tracking, the same prefilter as in section 5 is applied. Lowering $R$ (cheaper input) makes the loop faster and stiffer against disturbances, at the cost of larger pump commands — in the three-tank sweep the peak command exceeds 100 % (pump saturation) for $R < 0.75$.

## 8. Performance metrics

`state_space_studio.analysis.step_metrics` follows MATLAB's `stepinfo` conventions on a window that starts at the step:

| Metric | Definition |
| --- | --- |
| rise time | time from 10 % to 90 % of the final change (linear interpolation) |
| settling time | first time after which $\lvert y - y_\infty \rvert \le 2\ \%$ of the final change |
| overshoot | $\max(0, (y_{max} - y_\infty) / (y_\infty - y_0)) \cdot 100\ \%$ |
| steady-state error | target minus final value |
| IAE | $\int \lvert w - y \rvert\, dt$ after the reference step |
