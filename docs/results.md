# Results

The tables on this page are printed by `state-space-studio report` and every image is rendered by `state-space-studio figures`; the remaining numbers come from the same studies. The underlying experiments live in [`src/state_space_studio/scenarios.py`](../src/state_space_studio/scenarios.py) and are pinned by [`tests/test_scenarios.py`](../tests/test_scenarios.py).

- [Structural properties](#structural-properties)
- [Fundamentals](#fundamentals)
- [Drivetrain](#drivetrain)
- [Three-tank process](#three-tank-process)
- [Pole maps](#pole-maps)
- [Reproducing everything](#reproducing-everything)

## Structural properties

| Plant | n | Open-loop poles | Stability | rank Qc | rank Qo |
| --- | :-: | --- | --- | :-: | :-: |
| RC ladder network | 3 | -0.812, -0.389, -0.0495 | asymptotically stable | 3 | 3 |
| Coupled two-input system | 3 | -3, -1 ± 4j | asymptotically stable | 3 | 3 |
| Mass-spring-damper | 2 | -14.5, -5.53 | asymptotically stable | 2 | 2 |
| Three-mass torsional drivetrain | 6 | -10.3 ± 104j, -9.2 ± 46.6j, -8.76, 0 | marginally stable | 6 | 6 |
| Three-tank liquid-level process | 3 | -0.241, -0.0829, -0.00397 | asymptotically stable | 3 | 3 |

## Fundamentals

![Fundamentals](assets/fundamentals.png)

- **RC ladder.** The unit step charges the three capacitors one after another (each stage filters the previous one); all reach 1 V because no current flows at rest. Released from $v_1(0) = 0.5$ V with no source, the charge first spreads along the ladder and then drains through $R_1$. The slowest pole, −0.0495 s⁻¹, sets a 20.2 s time constant.
- **Coupled system.** From $x_0 = (0.2, 0, 0)$ the decoupled state $x_1$ decays at −3 while driving $x_3$; the $(x_2, x_3)$ pair then rings at 4 rad/s with decay rate 1 s⁻¹. From $x_0 = (0, 0.1, 0)$ only the oscillatory pair is excited, so the trajectory stays in the invariant plane $x_1 = 0$.
- **Mass–spring–damper.** Both poles are real (−5.53, −14.47). In the phase portrait every trajectory first collapses onto the slow eigen-direction and then slides to the origin; the unit step settles at the static deflection $u/c = 0.25$ m without overshoot.

![Coupled system under step and ramp inputs](assets/coupled_inputs.png)

Under the ramp $u = (t, 0)$ the decoupled state follows $x_1(t) = \tfrac{4}{3}\left(t - \tfrac{1}{3} + \tfrac{1}{3}e^{-3t}\right)$ — a constant lag of 1/3 s behind the scaled ramp, reproduced to within $10^{-3}$ by the first-order-hold simulation.

## Drivetrain

### Open loop

![Free response](assets/drivetrain_initial.png)

An initial twist excites both torsional modes (7.4 Hz and 16.6 Hz, damping ratios 0.19 and 0.10). Because there is no spring to ground the masses settle at a common, non-zero angle: the rigid-body mode at λ = 0 has no restoring force.

![Forced response](assets/drivetrain_open_loop.png)

A constant torque accelerates the whole shaft to $\omega_\infty = T / \sum d_i$ (435 rad/s for 10 N m, 43.5 rad/s for 1 N m) and the angles grow linearly — the open-loop plant cannot hold a position.

### Pole placement and prefilter

- Desired poles: −10 (×2), −100 (×4)
- K = [75.84, −211.7, 136.9, 0.7445, −0.86, 0.2695]
- V = 1.0667 (equal to $K_1 + K_2 + K_3$, see [theory, section 5](theory.md#5-prefilter-and-disturbance-offsets))
- Reference step: rise time 0.339 s, 2 % settling time 0.622 s, overshoot 0.0 %

| Case | φ1(1 s) [rad] | φ2(1 s) [rad] | φ3(1 s) [rad] | u(1 s) [N m] | max \|u\| [N m] |
| --- | --: | --: | --: | --: | --: |
| Reference step w = 50 rad | 49.966 | 49.965 | 49.963 | -0.000 | 53.33 |
| Load torque Tz1 = 1 N m | 0.937 | 0.937 | 0.937 | -1.000 | 1.06 |
| Load torque Tz2 = 1 N m | 47.642 | 48.307 | 48.306 | -1.000 | 8.96 |
| Load torque Tz3 = 1 N m | -37.884 | -37.216 | -36.548 | -1.000 | 6.50 |
| Tz2 = 0.5 N m and Tz3 = 1 N m | -14.063 | -13.062 | -12.395 | -1.500 | 3.99 |

![Closed-loop cases](assets/drivetrain_cases.png)

**Is the pole placement reasonable?** All poles are real and in the left half-plane, the dominant pair at −10 is ten times slower than the remaining four, so the response is dominated by a critically damped second-order behaviour with no overshoot. The fast poles at −100 cost actuator effort only during the first few milliseconds (the 53 N m peak is $V w$ at $t = 0$). Damping ratio 1, overshoot 0 %, and a 2 % settling time of 0.62 s (the $4/\sigma$ rule of thumb would say 0.4 s).

**Disturbances.** In every load case the motor ends at $u_\infty = -\sum T_z$, i.e. it exactly carries the load, and the velocities return to zero. The angular offset, however, depends strongly on where the load acts: 0.94 rad on the motor inertia but −36.5 rad on the load inertia, because the shaft twist is amplified by the large opposite-signed angle gains. Adding integral action on $\varphi_3$, or feeding the measured load torque forward, would remove the offset.

## Three-tank process

### Open loop

![Open loop](assets/three_tank_open_loop.png)

Released from (60, 50, 40) %, the levels first equalise through the valves (fast poles −0.241 and −0.083 s⁻¹) and then drain together (slow pole −0.004 s⁻¹, τ = 252 s). With the pump at 10 % the third tank reaches 26.7 %; after the step to 25 % it climbs to 66.8 % (steady state 66.9 %), and the leak $z = -5$ pulls it down to 43.3 %.

### Four controllers, one test

Reference 30 → 70 % at t = 500 s; leak z = −5 at t = 1000 s.

| Controller | Closed-loop poles | Rise [s] | Settling 2 % [s] | Error before leak [%] | Error after leak [%] | u range [%] | IAE [% s] |
| --- | --- | --: | --: | --: | --: | --: | --: |
| Pole placement + prefilter | -0.1, -0.02 (×2) | 170 | 301 | 0.02 | 46.75 | -3.2 … 43.9 | 22667 |
| PI state feedback | -0.2 (×2), -0.02 (×2) | 168 | 296 | 0.02 | 0.00 | 11.2 … 44.3 | 4330 |
| LQR Q = I, R = 2 | -0.241, -0.0847, -0.0145 | 155 | 284 | 0.04 | 6.32 | 11.2 … 67.0 | 6040 |
| LQR Q = diag(10, 1, 10), R = 5 | -0.242, -0.0901, -0.0223 | 104 | 192 | 0.00 | 3.84 | 11.2 … 103.2 | 4100 |

- Pole placement: K = [−5.529, 11.44, −5.789], V = 0.1886
- PI state feedback: Kx = [3.294, 2.764, 0.978], Ki = 0.0755, Kp = 0.3739, Kpi = [3.294, 2.764, 0.6042]
- LQR Q = I, R = 2: K = [0.3666, 0.2954, 0.2611], V = 1.3950
- LQR Q = diag(10, 1, 10), R = 5: K = [0.7881, 0.5041, 0.4324], V = 2.2996

![Controller comparison](assets/three_tank_controllers.png)

**Pole placement + prefilter.** The design asks for poles (−0.02, −0.02, −0.1) that are slower than two of the open-loop poles. To slow the plant down, the gains on $h_1$ and $h_3$ become negative, which makes the closed loop *more* sensitive to the leak than the open plant: the level drops by 46.75 points. The controller also commands a negative pump flow (−3.2 % at t ≈ 1055 s) shortly after the leak, which a real pump cannot deliver.

**PI state feedback.** The same dominant poles plus a pair at −0.2 for the integrator. Tracking is practically identical to pole placement (168 s rise, 296 s settling), but the integrator absorbs the leak completely and the pump command stays between 11 % and 44 %.

**LQR.** With all gains positive, LQR reinforces the plant's own dynamics instead of fighting them, so even the conservative design (Q = I, R = 2) keeps the droop to 6.3 points. The tuned design (heavier weights on $h_1$ and $h_3$) is the fastest of all four but peaks at 103 % pump command, i.e. it would saturate.

### LQR weight sweep

![LQR trade-off](assets/lqr_tradeoff.png)

Sweeping $R$ from 0.01 to 100 with $Q = I$ shows the classic trade-off: a smaller input weight shortens the settling time (84 s at $R = 0.01$, 468 s at $R = 100$) and shrinks the droop after the leak (0.46 % vs. 19.1 %), but the peak pump command grows from 28 % to 772 %. No value of $R$ reaches zero droop — that needs the integrator.

## Pole maps

![Pole maps](assets/pole_maps.png)

The drivetrain design moves the lightly damped torsional poles (ζ = 0.10 and 0.19) and the rigid-body pole at the origin to well-damped real locations. For the tanks, the ladder makes the different philosophies visible: pole placement drags all poles towards the slow target, PI adds the integrator pair at −0.2, and LQR leaves the fast plant pole where it is (−0.24) and only speeds up the slow drain mode.

## Reproducing everything

```bash
python -m pip install -e ".[dev]"
state-space-studio report                    # all tables on this page
state-space-studio figures --out docs/assets # all images on this page
python -m pytest                     # the assertions behind the numbers
```

The MATLAB studies in [`matlab/studies`](../matlab/studies) compute the same quantities with the Control System Toolbox (`acker`, `lqr`, `lsim`, `stepinfo`).
