# Phase 2 — Aerodynamics, Propulsion & Ground Handling

Phase 2 turns the bare 6DOF rigid body from Phase 1 into a flyable aircraft by
adding the forces that actually make it fly: aerodynamics, propulsion, and
landing-gear ground contact. Every new effect is a `ForceModel` plugged into the
unchanged `CompositeForceModel`/integrator from Phase 1, and the aircraft itself
is **data-driven** — defined entirely by `data/aircraft/cessna172.json` and
loaded through a small in-tree JSON reader. Adding another aircraft is a new data
file, nothing more.

All behaviour here is emergent: trim, stall, the phugoid, ground settling — none
of it is scripted. It all falls out of integrating these forces.

---

## 1. Architecture

```
   Aircraft (data)                 ForceModels (plug into Phase 1 engine)
   ----------------                -------------------------------------
   data/aircraft/*.json  --load--> Aircraft { mass, wing, aero,
                                              limits, propulsion, gear }
                                         |          |          |
                                         v          v          v
                                  AerodynamicsForce PropulsionForce LandingGearForce
                                         \          |          /
                                          \         |         /
                                       CompositeForceModel  +  GravityForce
                                                   |
                                              Simulation (RK4, fixed dt)

   Trim solver: damped Newton over (alpha, elevator, throttle) to find the
   level-flight equilibrium used to initialise the aircraft.
```

- **`core/Json`** — a dependency-free recursive-descent JSON reader (objects,
  arrays, numbers, strings, bools, null, escapes, `//` comments).
- **`aircraft/Aircraft`** — typed config structs (geometry, stability
  derivatives, control limits, propulsion, gear) with `Aircraft::load(path)`.
- **`aero/Aerodynamics`** — lift/drag/side-force and roll/pitch/yaw moments.
- **`propulsion/Propulsion`** — piston + propeller thrust.
- **`ground/LandingGear`** — spring-damper struts + tire friction.
- **`aircraft/Trim`** — solves for steady level flight.

---

## 2. Aerodynamic model

### 2.1 Flow angles and dynamic pressure

From the body-frame air-relative velocity `(u, v, w)` (= body velocity minus the
wind, which is zero until Phase 5):

$$V = \lVert\mathbf v_\text{air}\rVert,\quad
  \alpha = \operatorname{atan2}(w, u),\quad
  \beta = \arcsin\!\left(\frac{v}{V}\right),\quad
  \bar q = \tfrac12 \rho V^2$$

with ρ from the Phase 1 ISA model. Non-dimensional body rates use the span `b`
and mean chord `c`:

$$\hat p = \frac{p\,b}{2V},\quad \hat q = \frac{q\,c}{2V},\quad \hat r = \frac{r\,b}{2V}$$

### 2.2 Coefficient build-up (linear stability derivatives)

$$
\begin{aligned}
C_L &= C_{L0} + C_{L\alpha}\alpha + C_{Lq}\hat q + C_{L\delta_e}\delta_e + C_{L\text{flap}}\,\text{flap}\\
C_D &= C_{D0} + \frac{C_{L,\text{lin}}^2}{\pi e\,\mathrm{AR}} \quad(\text{drag polar})\\
C_Y &= C_{Y\beta}\beta + C_{Y\delta_r}\delta_r\\
C_l &= C_{l\beta}\beta + C_{lp}\hat p + C_{lr}\hat r + C_{l\delta_a}\delta_a + C_{l\delta_r}\delta_r\\
C_m &= C_{m0} + C_{m\alpha}\alpha + C_{mq}\hat q + C_{m\delta_e}\delta_e\\
C_n &= C_{n\beta}\beta + C_{np}\hat p + C_{nr}\hat r + C_{n\delta_a}\delta_a + C_{n\delta_r}\delta_r
\end{aligned}
$$

`AR = b²/S` and `e` is the Oswald efficiency factor. All derivatives come from
the aircraft data file (per-radian).

### 2.3 Stall (emergent, not scripted)

A sigmoid blend `σ(α)` (Beard & McLain) transitions lift and drag from the
linear model into a flat-plate model beyond the stall angle:

$$\sigma(\alpha)=\frac{1+e^{-M(\alpha-\alpha_0)}+e^{M(\alpha+\alpha_0)}}
{\left(1+e^{-M(\alpha-\alpha_0)}\right)\left(1+e^{M(\alpha+\alpha_0)}\right)}$$

$$C_L=(1-\sigma)\,C_{L,\text{lin}}+\sigma\,\big(2\,\mathrm{sign}(\alpha)\sin^2\alpha\cos\alpha\big),\quad
  C_D=(1-\sigma)\,C_{D,\text{lin}}+\sigma\,\big(2\sin^2\alpha\big)$$

So past the critical angle of attack `α₀`, `CL` peaks and falls while `CD` rises
sharply — the aircraft stalls on its own.

### 2.4 Forces and moments in the body frame

Lift `L = q̄ S C_L`, drag `D = q̄ S C_D`, side force `Y = q̄ S C_Y` are computed
in wind axes and rotated into the body frame through `α`, `β`:

$$
\begin{aligned}
F_x &= -D\cos\alpha\cos\beta - Y\cos\alpha\sin\beta + L\sin\alpha\\
F_y &= -D\sin\beta + Y\cos\beta\\
F_z &= -D\sin\alpha\cos\beta - Y\sin\alpha\sin\beta - L\cos\alpha
\end{aligned}
$$

Moments are already body-axis: `l = q̄ S b C_l`, `m = q̄ S c C_m`, `n = q̄ S b C_n`.

---

## 3. Propulsion (piston + propeller)

Available shaft power lapses with density and scales with throttle:

$$P = P_\max\,\big(\text{idle} + (1-\text{idle})\,\delta_t\big)\,\frac{\rho}{\rho_0}$$

Two thrust estimates are combined for envelope-wide stability:

$$T_\text{dyn} = \frac{\eta_\text{prop}\,P}{V_\text{fwd}},\qquad
  T_\text{static} = \sqrt[3]{2\rho A P^2}\ \ (A=\tfrac{\pi}{4}D^2)$$

The delivered thrust is `min(T_dyn, T_static)`: momentum-theory static thrust
caps the standing/low-speed case, while the energy form governs cruise. Thrust
acts along the body thrust axis at the thrust point, adding the moment `r × F`.

---

## 4. Landing gear (ground handling)

Each gear unit is a point on a spring-damper strut with a friction tire. For a
contact point `r` (body frame), with the ground a flat plane at a fixed
elevation (a terrain query replaces this in Milestone 3):

$$
d = p_z + h_\text{ground}\ (\text{penetration}),\qquad
N = \max(0,\; k\,d + c\,\dot d)
$$

Tire friction in the ground plane is saturated with `tanh` to avoid low-speed
chatter:

$$
F_\text{long} = -(\mu_\text{roll} + \text{brake}\cdot\mu_\text{brake})\,N\,\tanh\!\frac{v_\text{long}}{v_\epsilon},
\qquad
F_\text{lat} = -\mu_\text{side}\,N\,\tanh\!\frac{v_\text{lat}}{v_{\epsilon,\text{lat}}}
$$

The per-unit force is rotated into the body frame and applied at `r`. Taxi,
take-off roll, touchdown and ground loops all emerge from these forces — there
is no separate "on ground" state machine.

---

## 5. Trim

`trimLevelFlight()` solves for steady, wings-level flight at a target airspeed
and altitude by zeroing the residual `(F_x, F_z, M_y)` over the unknowns
`(α, δ_e, δ_t)` with a damped Newton iteration and finite-difference Jacobian.
For the C172 at 55 m/s / 1000 m it converges to `α ≈ 0.86°`, `δ_e ≈ −0.06`,
`δ_t ≈ 0.66` with a residual `~10⁻⁷` N — a genuine equilibrium.

---

## 6. Validation (all tests passing)

- **JSON** — scalars, nesting, arrays, escapes, comments, missing-key fallback.
- **Aircraft loading** — C172 mass/inertia/geometry/derivatives/gear from data.
- **Aerodynamics** — flow angles & `q̄`; lift up / drag aft; positive lift-curve
  slope; **stall** (CL falls and CD rises past the critical AoA); elevator pitch
  authority sign; weathercock (`Cnβ>0`) and dihedral (`Clβ<0`) signs.
- **Trim** — converges with tiny residual at cruise; integrating 20 s **from
  trim** holds altitude (±40 m) and airspeed (±5 m/s) hands-off (the residual
  excursion is the lightly-damped phugoid, exactly as expected).
- **Ground** — static load balance (gear reaction = weight) and dynamic settling
  (the aircraft rests on compressed struts without sinking through the ground).

The headless demo (`./build/fsim_headless`) trims the C172, flies it hands-off,
applies a nose-up elevator pulse (pitch-up, climb, airspeed bleed), then releases
to trim and shows the **phugoid** — all emergent.

---

## 7. Next: Phase 3

The OpenGL renderer — terrain, aircraft model, camera systems, and weather
visuals — consuming interpolated state from this (unchanged) headless core.
