# Phase 3 — Rendering Engine

Phase 3 adds the visualisation layer: an OpenGL 3.3 renderer with terrain, an
aircraft model, multiple camera systems, sky/fog, and smooth frame-rate-
independent rendering of the fixed-timestep physics. The defining architectural
rule is a **strict boundary** between simulation and rendering: the physics core
never knows OpenGL exists, and the renderer never runs simulation logic. The
only place the two coordinate frames meet is one small conversion function.

Because the rendering math is the part that can be made *correct by
construction*, it lives in the headless core and is fully unit-tested without a
GPU. The OpenGL backend (`render/gl/**`) is real code that builds against
GLFW + glad, gated behind `-DFSIM_BUILD_RENDERER=ON`.

---

## 1. Architecture

```
   Simulation (fixed 240 Hz)
        | prev state, curr state
        v
   interpolateState(prev, curr, alpha)   <-- lerp position, slerp attitude
        | RenderState (NED)
        v
   Camera.update(RenderState)            <-- chase / cockpit / orbit / flyby
        | eye/center/up (NED) -> nedToGl -> lookAt + perspective
        v
   gl::Renderer.renderFrame(camera, RenderState)
        |  sky + fog
        |  terrain mesh  (sampled from the SAME HeightField the gear collides with)
        |  aircraft mesh (model matrix = NED->GL of pos + body->NED rotation)
        v
     OpenGL 3.3
```

**Testable core (`src/render`, `src/terrain`, `src/core/Mat4`)**
- `Mat4` — column-major 4×4, `perspective()`, `lookAt()`.
- `Coordinates` — the single NED↔GL conversion (`nedToGl`).
- `RenderState` / `interpolateState` — lerp + quaternion **slerp** between two
  physics states by the leftover accumulator fraction.
- `Camera` — four modes producing GL view/projection matrices.
- `Frustum` — Gribb–Hartmann plane extraction + AABB culling (for tile/object
  visibility and LOD in Phase 6).
- `terrain/Terrain` — `HeightField` interface, `FlatGround`, and a deterministic
  `ProceduralTerrain` (sinusoidal hills with a flat airfield disc).

**OpenGL backend (`src/render/gl`, built only with `FSIM_BUILD_RENDERER=ON`)**
- `Shader`, `Mesh` — thin RAII wrappers over GLSL programs and VAOs/VBOs.
- `MeshBuilders` — terrain grid from a `HeightField`; built-in aircraft mesh.
- `Renderer` — sky, sun lighting, distance fog, terrain re-centring as the
  aircraft flies, aircraft model-matrix construction.
- `app/viewer.cpp` — GLFW window, input mapping, the fixed-step + interpolation
  loop, and camera switching.

---

## 2. Coordinate frames and the NED→GL boundary

The simulation uses NED (`+X` North, `+Y` East, `+Z` Down). OpenGL uses a Y-up,
right-handed view frame. The conversion is a single proper rotation
(determinant `+1`, so handedness is preserved):

$$\mathbf p_\text{gl} = \big(\,p_E,\ -p_D,\ -p_N\,\big)$$

i.e. East→right, Up→up, North→into the screen. The aircraft **model matrix** is
built so a body-frame vertex `v_b` maps to GL world space as
`C\,(\mathbf p_\text{ned} + R(q)\,v_b)`, where `C` is the NED→GL linear map and
`R(q)` is the body→NED rotation from the attitude quaternion. This is the *only*
code that touches GL coordinates.

---

## 3. Fixed-step physics, interpolated rendering

The physics steps at a fixed `dt = 1/240 s` for determinism, but frames are
drawn at a variable rate. Each frame accumulates real time; whole physics steps
are consumed from the accumulator, and the **leftover fraction**
`α = accumulator / dt` interpolates between the previous and current physics
states — linear for position/velocity, **slerp** for attitude — eliminating
stutter without coupling the simulation rate to the frame rate.

---

## 4. Cameras

| Mode | Eye | Look-at | Up |
|------|-----|---------|----|
| Chase   | behind & above along the nose vector | aircraft | world up |
| Cockpit | pilot eyepoint (body offset)         | forward along body x | body up |
| Orbit   | azimuth/elevation/radius about aircraft | aircraft | world up |
| Flyby   | fixed world point                    | aircraft | world up |

Each resolves an eye/center/up triple in NED; the view matrix is
`lookAt(nedToGl(eye), nedToGl(center), nedToGlDir(up))` and the projection is a
standard perspective.

---

## 5. Terrain — shared by physics and rendering

Terrain is a `HeightField`: `height(north, east)` in MSL metres, with an
analytic surface normal from the height gradient. The **same** height field is
queried by the landing gear (ground contact, Phase 2) and sampled by the
renderer to build the terrain mesh — so the aircraft always touches down on
exactly the ground that is drawn. `ProceduralTerrain` is a deterministic sum of
sinusoids with a smoothly-blended flat airfield around the origin for take-off
and landing. The renderer re-centres and rebuilds the terrain patch as the
aircraft moves so the world appears unbounded.

---

## 6. Lighting & atmosphere

A single directional sun with Lambert diffuse + ambient, and exponential
distance fog blended to the sky colour for aerial depth perception. These are
deliberately simple; Phase 5 (weather) and Phase 6 (graphics tuning) extend
them.

---

## 7. Validation

The render math and terrain are unit-tested in the headless suite (no GPU):

- **Mat4** — identity/multiply, `lookAt` puts the target down `-Z`, `perspective`
  maps the near/far planes to NDC `∓1` and the FOV edge correctly.
- **Render** — NED↔GL axis mapping; interpolation endpoints and a slerped
  45° midpoint; chase camera is behind & above and keeps the aircraft in front
  of the camera in view space; cockpit camera looks along the body x-axis;
  frustum contains the target and culls points behind the camera.
- **Terrain** — flat/normal correctness; procedural determinism, flat airfield,
  and non-trivial distant hills; **terrain↔gear integration** (contact detection
  uses the queried height; the aircraft settles on 200 m ground without sinking
  through).

The OpenGL backend is verified to **compile and link** against GLFW + glad +
OpenGL (`-DFSIM_BUILD_RENDERER=ON`) and to fail gracefully where no display is
available. Rendering actual pixels requires a machine with a GPU/display — run
`fsim_viewer` locally.

---

## 8. Building & running the viewer (local machine)

```bash
cmake -S . -B build -G Ninja -DFSIM_BUILD_RENDERER=ON
cmake --build build
./build/fsim_viewer            # run from the repo root (loads data/aircraft/…)
```

GLFW and glad are fetched automatically. On Linux the X11 dev packages are
needed (`libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
libgl1-mesa-dev`); pass `-DFSIM_GLFW_WAYLAND=ON` to also build the Wayland
backend.

**Controls:** arrows = elevator/ailerons, `Q`/`E` = rudder, `W`/`S` = throttle,
`F`/`G` = flaps, `B` = brakes, `1`–`4` = camera modes, `R` = reset to trim,
`Esc` = quit.

---

## 9. Next: Phase 4

Cockpit systems and instruments (airspeed, altimeter, VSI, attitude, heading,
tachometer) driven purely by simulated state, navigation, and engine-systems
modelling.
