// =============================================================================
//  render/gl/Renderer.hpp
//
//  The OpenGL renderer: draws the sky, the terrain patch (re-centred on the
//  aircraft as it flies), and the aircraft model, with simple directional sun
//  lighting and distance fog. It consumes an interpolated RenderState and a
//  Camera from the headless core; it owns no simulation logic.
// =============================================================================
#pragma once

#include "render/gl/Shader.hpp"
#include "render/gl/Mesh.hpp"
#include "render/Camera.hpp"
#include "render/RenderState.hpp"
#include "terrain/Terrain.hpp"

namespace fsim::gl {

class Renderer {
public:
    bool init(const HeightField* terrain);
    void resize(int width, int height);
    void renderFrame(const Camera& camera, const RenderState& aircraft);

private:
    Shader shader_;
    Mesh   terrainMesh_;
    Mesh   aircraftMesh_;
    const HeightField* terrain_{nullptr};

    int    width_{1280}, height_{720};
    double terrainExtent_{8000.0};
    int    terrainSegments_{160};
    double terrainCenterN_{1e30}, terrainCenterE_{1e30};

    void maybeRebuildTerrain(double north, double east);
};

} // namespace fsim::gl
