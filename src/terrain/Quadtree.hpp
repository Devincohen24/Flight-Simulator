// =============================================================================
//  terrain/Quadtree.hpp
//
//  Adaptive terrain level-of-detail. A quadtree over a square ground region is
//  refined where the camera is close and left coarse far away, so the rendered
//  triangle budget tracks screen-space importance rather than world area. Each
//  selected leaf is a TerrainTile (a square patch meshed at a fixed grid
//  resolution); smaller near-camera tiles therefore carry more detail per metre.
//
//  Selection also performs view-frustum culling: a node whose axis-aligned
//  bounding box (in GL space) lies entirely outside the frustum is discarded
//  along with its whole subtree.
//
//  This selection logic is pure and headless so it is unit-tested without a GPU;
//  the renderer simply meshes and draws whatever tiles it returns.
// =============================================================================
#pragma once

#include "render/Frustum.hpp"

#include <vector>

namespace fsim {

struct TerrainTile {
    double centerNorth{0.0};
    double centerEast{0.0};
    double size{0.0};   // side length, m
    int    lod{0};      // 0 = coarsest (root); larger = finer
};

struct QuadtreeParams {
    double rootSize{131072.0};  // m, side of the whole region (power-of-two)
    double minTileSize{512.0};  // m, finest leaf size
    int    maxDepth{8};
    double splitFactor{2.5};    // subdivide while distance < splitFactor * size
    double minElevation{-100.0};// for the culling bounding box
    double maxElevation{4000.0};
};

// Select the visible LOD tiles for a camera at 'cameraNed'. If 'frustum' is
// non-null, tiles outside the view frustum are culled. Tiles are appended to
// 'out'. The region is centred on (centerNorth, centerEast).
void selectTerrainTiles(const QuadtreeParams& params,
                        const Vec3& cameraNed,
                        const Frustum* frustum,
                        double centerNorth, double centerEast,
                        std::vector<TerrainTile>& out);

} // namespace fsim
