# Phase 4 — Cockpit Systems, Instruments & Navigation

Phase 4 adds the cockpit: a piston-engine systems model, the full flight-
instrument six-pack plus engine cluster, and a navigation computer (GPS,
waypoints, VOR). Every instrument reading is **derived from the simulated
state** — there are no independent gauge dynamics that could drift out of sync
with the physics. The numbers on the panel are the physics, presented.

The systems logic is headless and unit-tested; the visual panel is a Dear ImGui
overlay in the gated viewer.

---

## 1. Architecture

```
   Simulation ── state ──┐
   EngineSystem ─ engine ┤── computeInstruments() ── InstrumentReadings ── ImGui HUD
   Atmosphere ──── env ──┘
                          └─ NavigationComputer ── GpsFix / waypoint / VOR

   EngineSystem.update(dt, throttle, rho/rho0, p)   (advanced once per physics step)
```

- **`systems/Engine`** — stateful engine: RPM (spool lag), shaft power,
  manifold pressure, fuel flow and quantity. Advanced once per physics step.
- **`systems/Instruments`** — pure function `computeInstruments(state, env,
  engine, specificForce)` → all readings.
- **`systems/Navigation`** — local-tangent geodetic projection, great-circle
  bearing/distance, GPS fix, VOR radial + CDI.
- **`Simulation::specificForceBody()`** — the non-gravitational body
  acceleration (what accelerometers sense), feeding load factor and slip.

---

## 2. Engine systems

Power matches the propulsion force model so the tach and thrust agree:

$$P = P_\max\big(\text{idle} + (1-\text{idle})\,\delta_t\big)\frac{\rho}{\rho_0}$$

- **RPM** follows a first-order spool toward `idle + δ_t (max − idle)`:
  `rpm += (target − rpm)(1 − e^{−dt/τ})`.
- **Manifold pressure** ≈ ambient × (0.35 + 0.65 δ_t), reported in inHg.
- **Fuel flow** from brake specific fuel consumption: `ṁ = BSFC · P` (kg/s); the
  engine **quits the instant the tank runs dry**.

All parameters (`idle_rpm`, `max_rpm`, `spool_tau`, `bsfc`, `fuel_capacity`) are
in the aircraft data file.

---

## 3. Flight instruments

| Instrument | Formula |
|---|---|
| Airspeed (IAS) | `IAS = TAS · √(ρ/ρ₀)` — reads low at altitude |
| Mach | `TAS / a` |
| Altimeter | invert ISA from static pressure with a Kollsman setting: `h = (T₀/L)(1 − (p/p_baro)^{RL/g})` |
| Vertical speed | `−v_world,z` (up is −Z) |
| Attitude | pitch/bank from the attitude quaternion |
| Heading | yaw, wrapped to [0, 360) |
| Turn rate | `(q sinφ + r cosφ)/cosθ` |
| Slip/skid (ball) | lateral specific force `f_y/g` |
| Load factor | `n = −f_z/g` |
| AoA / sideslip | from the body-frame air-relative velocity |
| Tach / MP / fuel | from `EngineSystem` |

`f` is the body-frame specific force from `Simulation::specificForceBody()`, so
load factor and the slip ball respond correctly in turns and pull-ups.

---

## 4. Navigation

A local tangent plane fixes the NED origin to a latitude/longitude, projecting
NED positions back to geodetic coordinates:

$$\Delta\text{lat} = \frac{N}{R_\oplus},\qquad
  \Delta\text{lon} = \frac{E}{R_\oplus\cos(\text{lat}_0)}$$

- **GPS fix:** position, altitude, ground speed, and track over ground.
- **Waypoints:** great-circle (haversine) distance and initial bearing.
- **VOR:** the radial is the bearing **from** the station to the aircraft; the
  CDI deviation is the selected OBS course minus that radial, with a TO/FROM
  flag and ±10° full-scale clamp.

`LocalOrigin` can be swapped for a full WGS-84 model later without changing the
interface.

---

## 5. Validation (all 13 headless suites pass)

- **Engine** — spools to max RPM under full throttle, burns fuel, and quits when
  the tank empties.
- **Instruments** — IAS < TAS at altitude; pressure altitude matches geometric
  at standard baro; climb gives positive VS; attitude/heading extraction;
  load factor 1 g level and 2 g under a 2-g specific force; coordinated slip = 0.
- **Navigation** — one-degree great-circle distance (~111.2 km); cardinal
  bearings; NED→geodetic north offset; GPS track/altitude; waypoint
  bearing/distance (back-azimuth); VOR radial and CDI deviation.

The headless demo prints a live instrument panel and a GPS/nav fix while flying
the pulse/phugoid sequence. The viewer renders the same readings as a Dear ImGui
overlay (IAS/ALT/VS/HDG/attitude/AoA/g/RPM/MP/fuel/throttle + camera mode); the
viewer build (`-DFSIM_BUILD_RENDERER=ON`) is verified to compile and link.

---

## 6. Next: Phase 5

Advanced weather: altitude-dependent wind layers and continuous turbulence
(Dryden/von Kármán), injected into the air-relative velocity so they affect the
aerodynamics automatically.
