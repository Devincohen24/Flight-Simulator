# Phase 5 — Advanced Weather

Phase 5 adds a dynamic atmosphere: altitude-dependent **wind layers** and
continuous **turbulence**. The key design point is that weather enters the
simulation through exactly one channel — the world-frame wind velocity in
`EnvironmentSample.windWorld` — which the aerodynamics already subtract from the
body velocity. So wind and gusts affect angle of attack, sideslip, airspeed,
drift and ground track **automatically**, with no special-case aerodynamics.

All stochastic / time-varying weather state lives outside the deterministic
physics core: the `WeatherSystem` is advanced once per physics step and the
resulting wind is held frozen across that step's RK4 sub-stages.

---

## 1. Architecture

```
   WeatherSystem.update(dt, altitude, airspeed)
        |
        +-- WindField.at(altitude)        -> steady layered wind (NED)
        +-- DrydenTurbulence.update(dt,V)  -> gust perturbation  (NED)
        |
        =  total wind (NED)
        v
   Simulation.setWind(wind)   -> EnvironmentSample.windWorld
        v
   Aerodynamics: v_air = v_body - R(q)^T * wind   ->  alpha, beta, q_bar, ...
```

- **`environment/Wind`** — `WindField`: a stack of `WindLayer`s with linear
  interpolation by altitude and clamping outside the range. Helper to add a
  layer from the aviation "wind FROM heading at speed" convention.
- **`environment/Turbulence`** — `DrydenTurbulence`: a seeded, frame-rate-
  independent gust generator.
- **`environment/Weather`** — `WeatherSystem`: sums steady wind + turbulence.

---

## 2. Wind layers

Wind is stored as the NED velocity of the air mass (the direction it moves *to*).
The aviation convention "wind FROM heading ψ at speed s" maps to

$$\mathbf w_\text{ned} = s\,\big(\cos(\psi+180^\circ),\ \sin(\psi+180^\circ),\ 0\big)$$

(heading measured clockwise from North: N = cos, E = sin). Between layers the
wind is linearly interpolated by altitude; below the lowest / above the highest
layer it is clamped. A non-zero wind produces crab and the airspeed/ground-speed
split with no extra code — the headless demo flies a heading of ~352° but tracks
~006° over the ground under an 8–14 m/s westerly.

---

## 3. Turbulence (Dryden / Gauss–Markov)

Continuous turbulence is generated per axis by passing white noise through a
shaping filter whose **stationary variance equals the intensity** `σ²` and whose
correlation length is the Dryden scale length `L`. We use the first-order
Gauss–Markov (exponentially-correlated) process — the discrete form of the
Dryden longitudinal filter — which is exact in variance and frame-rate
independent:

$$\beta = \frac{V}{L},\qquad \phi = e^{-\beta\,dt},\qquad
  g_{k+1} = \phi\,g_k + \sigma\sqrt{1-\phi^2}\;\,w,\quad w\sim\mathcal N(0,1)$$

Gusts are produced as NED perturbations (horizontal `σ` for N/E, a smaller
vertical `σ` for D) and added to the steady wind. Because the aerodynamics work
from the air-relative velocity, a vertical gust changes the angle of attack and a
horizontal gust changes airspeed/sideslip — exactly as real turbulence is felt.
Intensity presets (`light`, `moderate`, `severe`) follow the usual RMS
categories; scale lengths default to the MIL-F-8785C high-altitude value
(~533 m). The RNG is seeded, so runs are reproducible for tests and replays.

> The longitudinal axis is the exact Dryden filter; the lateral/vertical axes
> use the same first-order form (rather than the full second-order Dryden), which
> preserves variance and correlation length and is a standard, well-behaved
> approximation. Upgrading to the second-order filters is a localised change.

---

## 4. Validation (all 14 headless suites pass)

- **Wind** — "wind FROM heading" sign convention; altitude interpolation and
  end-clamping.
- **Turbulence** — zero intensity ⇒ exactly calm; long-run RMS matches the
  horizontal and vertical `σ`; identical seeds ⇒ bit-identical sequences
  (deterministic).
- **Weather** — with turbulence off the total wind equals the steady layer; with
  it on the wind deviates from the steady value.

The headless demo flies the C172 through an 8–14 m/s westerly with light chop and
reports the resulting crab (heading vs ground track) and eastward drift. The
viewer feeds the same weather into the sim each step and shows the wind on the
HUD; the GL build is verified to compile and link.

---

## 5. Next: Phase 6

Optimization: terrain quadtree level-of-detail, frustum-cull-driven draw
reduction, physics/throughput tuning, and graphics tuning.
