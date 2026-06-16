// =============================================================================
//  render/gl/Renderer.hpp
//
//  The OpenGL renderer: sky, terrain and the aircraft model, with simple sun
//  lighting and distance fog. Terrain uses an adaptive quadtree level-of-detail
//  (Phase 6): the ground is split into chunks that are fine near the camera and
//  coarse far away, and each frame only the chunks inside the view frustum are
//  drawn. It consumes an interpolated RenderState and a Camera from the headless
//  core; it owns no simulation logic.
// =============================================================================
#pragma once

#include "render/gl/Shader.hpp"
#include "render/gl/Mesh.hpp"
#include "render/Camera.hpp"
#include "render/RenderState.hpp"
#include "render/Frustum.hpp"
#include "terrain/Terrain.hpp"
#include "terrain/Quadtree.hpp"

#include <memory>
#include <vector>

namespace fsim::gl {

class Renderer {
public:
    bool init(const HeightField* terrain);
    void resize(int width, int height);
    void renderFrame(const Camera& camera, const RenderState& aircraft);

    // Number of terrain chunks drawn last frame (after culling) -- for HUD/perf.
    int lastDrawnChunks() const { return lastDrawnChunks_; }
    int lastChunkCount()  const { return static_cast<int>(chunks_.size()); }

private:
    struct TerrainChunk {
        std::unique_ptr<Mesh> mesh;
        AABB bounds;       // GL-space bounding box for frustum culling
    };

    Shader shader_;
    Mesh   aircraftMesh_;
    const HeightField* terrain_{nullptr};

    int width_{1280}, height_{720};

    // Adaptive terrain LOD.
    QuadtreeParams qtParams_;
    int    tileSegments_{24};
    std::vector<TerrainChunk> chunks_;
    double lastBuildN_{1e30}, lastBuildE_{1e30};
    int    lastDrawnChunks_{0};

    void rebuildTerrainLod(const Vec3& cameraNed);
};

} // namespace fsim::gl
