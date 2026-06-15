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
| 2 | Aerodynamics, propulsion, ground handling (data-driven Cessna 172) | planned |
| 3 | OpenGL renderer: terrain, aircraft, cameras, weather visuals | planned |
| 4 | Cockpit systems, instruments, navigation, engine systems | planned |
| 5 | Advanced weather: wind layers, Dryden turbulence | planned |
| 6 | Optimization: LOD, profiling, tuning | planned |

See [`docs/PHASE1_PHYSICS.md`](docs/PHASE1_PHYSICS.md) for the architecture,
physics equations, and validation of the current core.

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
src/core/      math (Vec3/Mat3/Quat), constants, frame conventions
src/physics/   rigid body, equations of motion, atmosphere, integrator, sim driver
src/app/       headless demo entry point
tests/         dependency-free unit/validation tests
docs/          per-phase engineering documentation
```
