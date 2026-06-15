# Flight-Simulator

A professional-grade flight simulator built in **C++17 / OpenGL**, developed in
stages as a foundation for an X-Plane / MSFS-style experience. Aircraft motion is
**not scripted** — it emerges entirely from a 6-degree-of-freedom rigid-body
simulation driven by aerodynamic, propulsive, gravitational, and ground-contact
forces.

## Status

| Phase | Scope | State |
|-------|-------|-------|
| **1** | Core physics: 6DOF rigid body, ISA atmosphere, forces/moments, RK4 integrator | ✅ implemented & validated (headless) |
| **2** | Aerodynamics, propulsion, ground handling (data-driven Cessna 172) | ✅ implemented & validated (headless) |
| **3** | OpenGL renderer: terrain, aircraft, cameras, sky/fog, interpolation | ✅ render math + terrain validated (headless); GL backend builds with `-DFSIM_BUILD_RENDERER=ON` |
| 4 | Cockpit systems, instruments, navigation, engine systems | planned |
| 5 | Advanced weather: wind layers, Dryden turbulence | planned |
| 6 | Optimization: LOD, profiling, tuning | planned |

See [`docs/PHASE1_PHYSICS.md`](docs/PHASE1_PHYSICS.md),
[`docs/PHASE2_AERODYNAMICS.md`](docs/PHASE2_AERODYNAMICS.md), and
[`docs/PHASE3_RENDERING.md`](docs/PHASE3_RENDERING.md) for the architecture,
physics/render equations, and validation of the current systems.

The headless demo loads the data-driven Cessna 172, trims it for level cruise,
flies it hands-off, applies a nose-up elevator pulse, and shows the resulting
**phugoid** oscillation — all emergent from the aerodynamics, none scripted.

## Design principles

- **Emergent dynamics** — no scripted rotations or predetermined movement.
- **Data-driven aircraft** — aircraft are defined by data (mass, inertia, aero
  tables, engine decks), so new aircraft drop in without engine changes.
- **Physics / rendering separation** — the simulation runs headless; the
  renderer (Milestone 3) only consumes interpolated state.
- **Deterministic & stable** — fixed-timestep RK4 with quaternion attitude.

## Build & test

The Phase 1 physics core is headless and has **no third-party dependencies** —
just a C++17 compiler and CMake.

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure   # run the physics/math test suite
./build/fsim_headless                         # print a sample trajectory
```

The OpenGL renderer (added in Milestone 3) is gated behind
`-DFSIM_BUILD_RENDERER=ON` and will require GLFW/OpenGL; the headless core
always builds without them.

## Layout

```
src/core/        math (Vec3/Mat3/Quat/Mat4), constants, frames, controls, JSON
src/physics/     rigid body, equations of motion, atmosphere, integrator, sim driver
src/aircraft/    data-driven aircraft definition + loader, trim solver
src/aero/        aerodynamic force/moment model (stability derivatives + stall)
src/propulsion/  piston + propeller thrust model
src/ground/      landing-gear spring-damper + tire friction
src/terrain/     height-field terrain (shared by gear collision and rendering)
src/render/      cameras, NED↔GL, state interpolation, frustum (headless core)
src/render/gl/   OpenGL 3.3 backend: shaders, meshes, renderer (gated)
src/app/         headless demo + interactive viewer entry points
data/aircraft/   aircraft data files (e.g. cessna172.json)
tests/           dependency-free unit/validation tests
docs/            per-phase engineering documentation
```

### Interactive viewer (local machine with a GPU/display)

```bash
cmake -S . -B build -G Ninja -DFSIM_BUILD_RENDERER=ON
cmake --build build
./build/fsim_viewer       # fly the C172 over procedural terrain
```

Controls: arrows = elevator/ailerons, `Q`/`E` = rudder, `W`/`S` = throttle,
`F`/`G` = flaps, `B` = brakes, `1`–`4` = cameras, `R` = reset to trim, `Esc` =
quit. GLFW/glad are fetched automatically (see `docs/PHASE3_RENDERING.md` for
Linux X11 packages).
