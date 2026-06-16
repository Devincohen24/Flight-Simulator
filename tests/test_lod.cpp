// =============================================================================
//  tests/test_lod.cpp -- terrain quadtree level-of-detail selection + culling.
// =============================================================================
#include "TestHarness.hpp"
#include "terrain/Quadtree.hpp"
#include "render/Camera.hpp"

#include <algorithm>

using namespace fsim;

static QuadtreeParams smallParams() {
    QuadtreeParams p;
    p.rootSize = 8192.0;
    p.minTileSize = 256.0;
    p.maxDepth = 6;
    p.splitFactor = 2.5;
    return p;
}

TEST_CASE(lod_finer_tiles_near_camera) {
    const QuadtreeParams p = smallParams();
    std::vector<TerrainTile> tiles;
    // Camera over the centre of the region.
    selectTerrainTiles(p, Vec3{0, 0, -500}, nullptr, 0.0, 0.0, tiles);

    CHECK(!tiles.empty());

    // The tile containing the camera should be at (or near) the finest size.
    double nearestSize = 1e30;
    double farthestSize = 0.0;
    for (const auto& t : tiles) {
        const double d = std::sqrt(t.centerNorth * t.centerNorth
                                 + t.centerEast * t.centerEast);
        if (d < 300.0) nearestSize = std::min(nearestSize, t.size);
        if (d > 2500.0) farthestSize = std::max(farthestSize, t.size);
    }
    CHECK(nearestSize <= p.minTileSize);   // finest detail under the camera
    CHECK(farthestSize > nearestSize);     // coarser tiles far away
}

TEST_CASE(lod_count_grows_with_proximity_detail) {
    const QuadtreeParams p = smallParams();
    std::vector<TerrainTile> low, high;
    // A very distant camera needs few tiles; a close one needs many.
    selectTerrainTiles(p, Vec3{100000, 0, -8000}, nullptr, 0.0, 0.0, low);
    selectTerrainTiles(p, Vec3{0, 0, -200},       nullptr, 0.0, 0.0, high);
    CHECK(high.size() > low.size());
}

TEST_CASE(lod_covers_region_area) {
    const QuadtreeParams p = smallParams();
    std::vector<TerrainTile> tiles;
    selectTerrainTiles(p, Vec3{0, 0, -500}, nullptr, 0.0, 0.0, tiles);
    // The selected leaves must tile the whole region exactly (no gaps/overlap):
    // total leaf area == root area.
    double area = 0.0;
    for (const auto& t : tiles) area += t.size * t.size;
    CHECK_NEAR(area, p.rootSize * p.rootSize, 1.0);
}

TEST_CASE(lod_frustum_culls_behind_camera) {
    const QuadtreeParams p = smallParams();

    // Camera at the region centre, looking North, narrow far plane.
    Camera cam;
    cam.mode = CameraMode::Flyby;
    cam.flybyPositionNed = Vec3{0, 0, -500};
    RenderState look; look.positionWorld = Vec3{4000, 0, -500}; // look north
    cam.update(look);
    const Frustum fr(cam.viewProjection(16.0 / 9.0));

    std::vector<TerrainTile> all, culled;
    selectTerrainTiles(p, Vec3{0, 0, -500}, nullptr, 0.0, 0.0, all);
    selectTerrainTiles(p, Vec3{0, 0, -500}, &fr,     0.0, 0.0, culled);
    // Culling against a forward-looking frustum removes tiles behind the camera.
    CHECK(culled.size() < all.size());
    CHECK(!culled.empty());
}

FSIM_TEST_MAIN()
