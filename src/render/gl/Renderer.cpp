// =============================================================================
//  render/gl/Renderer.cpp
// =============================================================================
#include "render/gl/Renderer.hpp"
#include "render/gl/MeshBuilders.hpp"
#include "render/Coordinates.hpp"

#include <cmath>

namespace fsim::gl {

namespace {

const char* kVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uViewProj;
out vec3 vNormal;
out vec3 vWorldPos;
void main() {
    vec4 wp = uModel * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uViewProj * wp;
}
)GLSL";

const char* kFragmentShader = R"GLSL(
#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
out vec4 FragColor;
uniform vec3  uLightDir;    // direction TO the sun (GL space, normalised)
uniform vec3  uColor;
uniform vec3  uCameraPos;
uniform vec3  uFogColor;
uniform float uFogDensity;
void main() {
    vec3 N = normalize(vNormal);
    float diff = max(dot(N, normalize(uLightDir)), 0.0);
    vec3 lit = uColor * (0.30 + 0.70 * diff);
    float dist = length(vWorldPos - uCameraPos);
    float fog = clamp(1.0 - exp(-uFogDensity * dist), 0.0, 1.0);
    FragColor = vec4(mix(lit, uFogColor, fog), 1.0);
}
)GLSL";

// Build the GL model matrix that maps a body-frame vertex to GL world space:
//   gl = C * (pos_ned + R(q) * vbody),   C = NED->GL linear map.
// So the linear part is C*R and the translation is nedToGl(pos).
Mat4 modelMatrix(const Vec3& posNed, const Quat& q) {
    const Mat3 R = q.toMatrix(); // body -> NED
    // C rows: (0,1,0), (0,0,-1), (-1,0,0).  CR[0]=R[1], CR[1]=-R[2], CR[2]=-R[0].
    Mat4 m = Mat4::identity();
    for (int j = 0; j < 3; ++j) {
        m(0, j) =  R.m[1][j];
        m(1, j) = -R.m[2][j];
        m(2, j) = -R.m[0][j];
    }
    const Vec3 t = nedToGl(posNed);
    m(0, 3) = t.x; m(1, 3) = t.y; m(2, 3) = t.z;
    return m;
}

} // namespace

bool Renderer::init(const HeightField* terrain) {
    terrain_ = terrain;
    if (!shader_.build(kVertexShader, kFragmentShader)) return false;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.53f, 0.70f, 0.92f, 1.0f); // sky blue

    std::vector<Vertex> v;
    std::vector<uint32_t> idx;
    buildAircraftMesh(v, idx);
    aircraftMesh_.upload(v, idx);
    return true;
}

void Renderer::resize(int width, int height) {
    width_  = width  > 1 ? width  : 1;
    height_ = height > 1 ? height : 1;
    glViewport(0, 0, width_, height_);
}

void Renderer::rebuildTerrainLod(const Vec3& cameraNed) {
    if (!terrain_) return;
    // Rebuild only when the camera has moved a finest-tile from the last build.
    if (std::fabs(cameraNed.x - lastBuildN_) < qtParams_.minTileSize * 0.5 &&
        std::fabs(cameraNed.y - lastBuildE_) < qtParams_.minTileSize * 0.5 &&
        !chunks_.empty())
        return;

    // Select adaptive tiles for the camera over a world-centred region.
    std::vector<TerrainTile> tiles;
    selectTerrainTiles(qtParams_, cameraNed, /*frustum=*/nullptr, 0.0, 0.0, tiles);

    chunks_.clear();
    chunks_.reserve(tiles.size());
    std::vector<Vertex> v;
    std::vector<uint32_t> idx;
    for (const TerrainTile& t : tiles) {
        buildTerrainMesh(*terrain_, t.centerNorth, t.centerEast, t.size,
                         tileSegments_, v, idx);
        TerrainChunk chunk;
        chunk.mesh = std::make_unique<Mesh>();
        chunk.mesh->upload(v, idx);
        // GL-space bounds: nedToGl(n,e,d)=(e,-d,-n), elevation = -d.
        const double h = t.size * 0.5;
        chunk.bounds.min = Vec3{t.centerEast - h, qtParams_.minElevation,
                                -(t.centerNorth + h)};
        chunk.bounds.max = Vec3{t.centerEast + h, qtParams_.maxElevation,
                                -(t.centerNorth - h)};
        chunks_.push_back(std::move(chunk));
    }
    lastBuildN_ = cameraNed.x;
    lastBuildE_ = cameraNed.y;
}

void Renderer::renderFrame(const Camera& camera, const RenderState& aircraft) {
    rebuildTerrainLod(camera.eyeNed());

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const double aspect = double(width_) / double(height_);
    const Mat4 viewProj = camera.viewProjection(aspect);
    const Frustum frustum(viewProj);

    shader_.use();
    shader_.setMat4("uViewProj", viewProj);
    shader_.setVec3("uCameraPos", nedToGl(camera.eyeNed()));
    // Afternoon sun, high in the GL sky.
    shader_.setVec3("uLightDir", Vec3{-0.4, 0.8, 0.3}.normalized());
    shader_.setVec3("uFogColor", Vec3{0.53, 0.70, 0.92});
    shader_.setFloat("uFogDensity", 1.0e-4f);

    // Terrain: draw only the LOD chunks inside the view frustum.
    shader_.setMat4("uModel", Mat4::identity());
    shader_.setVec3("uColor", Vec3{0.30, 0.55, 0.27}); // grass green
    lastDrawnChunks_ = 0;
    for (const TerrainChunk& c : chunks_) {
        if (!frustum.intersectsAABB(c.bounds)) continue;
        c.mesh->draw();
        ++lastDrawnChunks_;
    }

    // Aircraft.
    shader_.setMat4("uModel", modelMatrix(aircraft.positionWorld, aircraft.orientation));
    shader_.setVec3("uColor", Vec3{0.85, 0.85, 0.88});
    aircraftMesh_.draw();
}

} // namespace fsim::gl
