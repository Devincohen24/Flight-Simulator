# Phase 1 — Core Physics Engine

This document explains the architecture, the physics, and the mathematics of the
Phase 1 core: the 6-degree-of-freedom (6DOF) rigid-body simulation, the
atmospheric model, the force/moment system, and the numerical integrator.

Everything here is **headless and dependency-free** — it builds and runs with
nothing but a C++17 compiler, so the dynamics can be validated automatically
(see `tests/`) before any rendering exists. All aircraft motion **emerges** from
integrating these equations; nothing is scripted.

---

## 1. Architecture

```
              +-------------------+
              |   Simulation      |  fixed-timestep driver (accumulator)
              |  (Simulation.*)   |
              +---------+---------+
                        | owns
        +---------------+----------------+-------------------+
        |               |                |                   |
  RigidBodyState   MassProperties   CompositeForceModel   Atmosphere
  (state vector)   (m, I, I^-1)     (sum of ForceModels)  (ISA sample)
                                          |
                                  +-------+--------+
                                  | GravityForce   |  (Phase 2 adds
                                  | (ForceModel)   |   Aero/Propulsion/Gear)
                                  +----------------+
                        |
                  integrate()  <-- RK4 / semi-implicit Euler (Integrator.hpp)
```

- **`RigidBodyState`** is the complete state: world position (NED), body-frame
  velocity, attitude quaternion, and body angular rates. The same struct also
  represents the *time derivative* of the state, which keeps the RK4 stage
  arithmetic clean.
- **`MassProperties`** holds mass and the inertia tensor, caching `I⁻¹`.
- **`ForceModel`** is the extensibility seam. Each physical effect (gravity now;
  aerodynamics, propulsion, and landing gear in Phase 2) implements `evaluate()`
  and returns a body-frame `Wrench` (force + moment). `CompositeForceModel`
  sums them. The integrator and rigid body never change as effects are added.
- **`Atmosphere`** provides ISA temperature/pressure/density/speed-of-sound,
  which Phase 2 uses for dynamic pressure and Mach.
- **`Simulation`** advances time with a **fixed** physics step (default 240 Hz)
  via an accumulator, so the dynamics are deterministic and stable independent
  of the (future) variable render frame rate.

### Coordinate frames

| Frame | Axes | Use |
|-------|------|-----|
| **Body** (b) | +X nose, +Y right wing, +Z down | velocities (u,v,w), rates (p,q,r), forces/moments |
| **World/NED** (n) | +X North, +Y East, +Z Down | position; gravity acts along +Z |
| **Render** (GL) | Y-up, right-handed | *only* at the rendering boundary (Milestone 3) |

Attitude is a unit quaternion `q = q_world_from_body`, so
`v_world = q.rotate(v_body)`.

---

## 2. The 6DOF equations of motion

All vectors are in the **body frame** unless noted. `m` = mass, `I` = inertia
tensor, `ω = (p,q,r)` = body angular rate, `v = (u,v,w)` = body velocity,
`F`, `M` = total force and moment, `q` = attitude quaternion.

**Translational (Newton, in a rotating frame):**

$$\dot{\mathbf v} = \frac{\mathbf F}{m} - \boldsymbol\omega \times \mathbf v$$

The `−ω×v` term is the transport (Coriolis) term that appears because the
velocity is expressed in the rotating body frame.

**Rotational (Euler's equation):**

$$\dot{\boldsymbol\omega} = I^{-1}\!\left(\mathbf M - \boldsymbol\omega \times (I\,\boldsymbol\omega)\right)$$

The `ω×(Iω)` term is the gyroscopic coupling; it is what makes an asymmetric
body tumble (the Dzhanibekov / tennis-racket effect) and is essential for
realistic spin and inertial coupling.

**Attitude kinematics (quaternion):**

$$\dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix}0 \\ \boldsymbol\omega\end{bmatrix}$$

Quaternions are used instead of Euler angles to avoid gimbal lock and to keep
attitude well-behaved at any orientation. The quaternion is renormalised after
every integration step to counter slow drift.

**Position (world frame):**

$$\dot{\mathbf p}_\text{ned} = R(q)\,\mathbf v$$

i.e. the body velocity rotated into NED.

These are implemented in `computeDerivative()` (`physics/RigidBody.cpp`).

---

## 3. Forces and moments

Currently the only model is **gravity**: a world-frame weight `m·g` along +Z
(down), rotated into the body frame, producing no moment about the CG:

$$\mathbf F_\text{body} = R(q)^{\top}\begin{bmatrix}0\\0\\m g\end{bmatrix}, \qquad \mathbf M_\text{body} = \mathbf 0$$

Phase 2 plugs in aerodynamic forces (lift, drag, side force from coefficient
tables), propulsion (thrust), and ground-contact (gear spring-damper + tire
friction) by adding more `ForceModel`s — the rest of the engine is untouched.

---

## 4. Atmosphere (International Standard Atmosphere)

Two layers are modelled:

**Troposphere** (0–11 km), lapse rate `L = 0.0065 K/m`:

$$T = T_0 - L h, \qquad p = p_0\left(\frac{T}{T_0}\right)^{g_0/(R L)}$$

**Lower stratosphere** (11–20 km), isothermal at `T = 216.65 K`:

$$p = p_{11}\,\exp\!\left(\frac{-g_0 (h - 11{,}000)}{R\,T}\right)$$

Density and speed of sound from the ideal-gas law:

$$\rho = \frac{p}{R\,T}, \qquad a = \sqrt{\gamma R T}$$

with `T₀ = 288.15 K`, `p₀ = 101325 Pa`, `R = 287.05287 J/(kg·K)`, `γ = 1.4`.
Validated against published ISA tables at 0, 5, and 11 km (`tests/test_atmosphere.cpp`).

---

## 5. Numerical integration

The default integrator is **classical fourth-order Runge–Kutta (RK4)** with a
fixed step `dt = 1/240 s`:

$$y_{n+1} = y_n + \tfrac{dt}{6}\,(k_1 + 2k_2 + 2k_3 + k_4)$$

RK4 is fourth-order accurate (error `O(dt⁵)` per step) and exact for the
constant-acceleration ballistic case, while remaining stable for the oscillatory
modes (phugoid, short-period, Dutch roll) that Phase 2 introduces. A
**semi-implicit (symplectic) Euler** option is included for performance
comparison and as a stability sanity check.

A **fixed** timestep is used (with an accumulator in `Simulation::advance`) for
determinism; the future renderer will run at a variable rate and interpolate
between physics states. The accumulator intentionally leaves a sub-step
remainder each frame, which the renderer will smooth out.

---

## 6. Validation

`tests/` contains a dependency-free harness with the following checks (all
passing):

- **Math** — dot/cross products, right-handedness, `Mat3` inverse round-trip on
  an inertia-like tensor, quaternion rotation, Euler↔quaternion round-trip.
- **Atmosphere** — ISA values at 0/5/11 km vs reference tables, isothermal
  stratosphere, negative-altitude clamping.
- **Rigid body** —
  - ballistic drop reproduces `z = ½gt²`, `v = gt`;
  - horizontal launch reproduces a parabola;
  - **energy conservation** for gravity-only flight (< 0.1 % drift over 10 s);
  - torque-free **symmetric** spin keeps a constant rate and unit quaternion;
  - torque-free **asymmetric** tumble conserves angular-momentum magnitude and
    rotational kinetic energy (validates the `ω×(Iω)` coupling);
  - gravity resolves correctly into a pitched-up body frame.

Run them with:

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

The headless demo (`./build/fsim_headless`) integrates a launched body and
prints the resulting parabola — apex, range, and impact speed all match
analytic mechanics, produced entirely by the integrator.

---

## 7. Next: Phase 2

Aerodynamic model (lift/drag/side-force and roll/pitch/yaw moments from
coefficient tables with angle of attack, sideslip and control deflections),
piston+propeller propulsion, and landing-gear ground handling — all added as new
`ForceModel`s on top of this unchanged core, with a data-driven Cessna 172 as
the first aircraft.
