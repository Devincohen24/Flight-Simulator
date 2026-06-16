# Phase 6 — Optimization

Phase 6 is about making the simulator scale: adaptive terrain level-of-detail
(LOD), view-frustum culling driven by that LOD, lightweight profiling, and a
measured real-time performance margin for the flight-dynamics core. As with
every other phase, the parts that can be made *correct by construction* — the
LOD selection and the profiler accounting — live in the headless core and are
unit-tested; the GL renderer simply consumes the result.

---

## 1. Terrain quadtree LOD

The ground is covered by a quadtree over a large square region. Each node
subdivides while the camera is close relative to the node's size:

$$\text{subdivide if } \operatorname{dist}(\text{camera},\,\text{node}) < k\cdot\text{size}
  \quad\text{and}\quad \text{size} > \text{minTileSize}\ \text{and}\ \text{depth} < \text{maxDepth}$$

The leaves that survive are `TerrainTile`s — square patches each meshed at a
fixed grid resolution. Because near-camera leaves are small, they pack far more
triangles per metre than distant coarse leaves, so the triangle budget follows
screen-space importance instead of world area. Key properties (all tested):

- **Finer near the camera:** the leaf under the camera is at `minTileSize`;
  distant leaves are larger.
- **Exact tiling:** the selected leaves partition the region with no gaps or
  overlap (∑ leaf area == region area).
- **Proximity ⇒ more tiles:** a close camera produces many more leaves than a
  far one.

During selection, a node whose **GL-space bounding box** is entirely outside the
view frustum is discarded with its whole subtree, so off-screen terrain costs
nothing.

---

## 2. Frustum culling

The renderer builds the LOD chunk set when the camera moves more than half a
finest-tile, then each frame extracts the six frustum planes from the
view-projection matrix (Gribb–Hartmann, `render/Frustum.hpp`) and draws only the
chunks whose AABB intersects the frustum. The viewer HUD reports
`chunks drawn / total`, so the culling is visible at runtime.

This reuses exactly the `Frustum` and `AABB` types validated in Phase 3, and the
quadtree's own optional frustum pruning, so visibility is consistent between
selection and drawing.

---

## 3. Profiling

`core/Profiler` provides named timing accumulators (count / total / min / max /
average) and an RAII `ScopedTimer` that records a labelled span on scope exit:

```cpp
Profiler prof;
{ ScopedTimer t(prof, "physics_step"); sim.step(); }
auto s = prof.get("physics_step");   // s.average(), s.minVal, s.maxVal, ...
```

The accounting is pure (durations can be supplied directly), so it is
unit-tested without depending on wall-clock timing.

---

## 4. Performance

The flight-dynamics core is deterministic and fast. The benchmark in
`tests/test_performance.cpp`:

- confirms **determinism** — two simulations from identical inputs evolve
  bit-for-bit identically (there is no RNG in the core; weather's RNG is seeded
  and lives outside it);
- measures **throughput** — 60 s of simulated flight (14 400 RK4 steps of the
  full aero + propulsion + gravity model) runs in **~17 ms of wall time**, about
  **3500× real-time (~1.2 µs/step)**, and the test requires at least a 10×
  margin.

That headroom is the budget that lets the renderer, cockpit systems, weather and
(eventually) many simultaneous aircraft share a frame and still hit real time.

### Where the budget goes / further tuning

- **Physics:** RK4 at a fixed 240 Hz is already far inside budget; the fixed step
  keeps it deterministic. Contact-heavy ground phases could sub-step locally if
  ever needed, but the margin shows it is unnecessary for a single aircraft.
- **Rendering:** LOD + frustum culling bound the triangle count; terrain chunks
  rebuild only when the camera moves, not every frame. Natural next steps are
  mesh instancing, a chunk mesh cache keyed by tile, and shadow/post-processing
  passes — all isolated to the GL backend.

---

## 5. Validation (all 16 headless suites pass)

- **LOD** (`test_lod`) — finer tiles near the camera; exact region tiling;
  tile-count grows with proximity; frustum culling removes off-screen tiles.
- **Profiler** (`test_performance`) — count/total/min/max/average accounting.
- **Performance** (`test_performance`) — physics determinism and the
  faster-than-real-time throughput margin.

The renderer (`-DFSIM_BUILD_RENDERER=ON`) drives the terrain through the quadtree
with per-frame frustum culling and reports the drawn/total chunk counts on the
HUD; the GL build is verified to compile and link.

---

## 6. Status

This completes the six-phase roadmap: a deterministic 6DOF physics core, a
data-driven aerodynamic/propulsion/ground-handling model, an OpenGL rendering
engine with terrain and cameras, cockpit systems and navigation, dynamic
weather, and the optimization layer — a foundation built for expansion (new
aircraft are data files; new physical effects are `ForceModel`s; new render
features are isolated in the GL backend).
