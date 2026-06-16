// =============================================================================
//  terrain/Quadtree.cpp -- see Quadtree.hpp.
// =============================================================================
#include "terrain/Quadtree.hpp"
#include "render/Coordinates.hpp"

#include <cmath>

namespace fsim {

namespace {

// Axis-aligned bounding box (GL space) of a square ground node spanning
// [cN-h, cN+h] x [cE-h, cE+h] with elevation in [minE, maxE].
AABB nodeAabbGl(double cN, double cE, double half, double minE, double maxE) {
    // nedToGl(n,e,d) = (e, -d, -n); elevation = -d.
    AABB box;
    box.min = Vec3{cE - half, minE, -(cN + half)};
    box.max = Vec3{cE + half, maxE, -(cN - half)};
    return box;
}

void recurse(const QuadtreeParams& p, const Vec3& cam, const Frustum* frustum,
             double cN, double cE, double size, int depth,
             std::vector<TerrainTile>& out) {
    const double half = size * 0.5;

    // Frustum cull the whole subtree if its bounds are off-screen.
    if (frustum) {
        const AABB box = nodeAabbGl(cN, cE, half, p.minElevation, p.maxElevation);
        if (!frustum->intersectsAABB(box)) return;
    }

    // Horizontal distance from the camera to this node's centre.
    const double dN = cN - cam.x, dE = cE - cam.y;
    const double dist = std::sqrt(dN * dN + dE * dE);

    const bool canSplit = (size > p.minTileSize) && (depth < p.maxDepth);
    if (canSplit && dist < p.splitFactor * size) {
        const double q = size * 0.25; // child centre offset
        recurse(p, cam, frustum, cN - q, cE - q, half, depth + 1, out);
        recurse(p, cam, frustum, cN - q, cE + q, half, depth + 1, out);
        recurse(p, cam, frustum, cN + q, cE - q, half, depth + 1, out);
        recurse(p, cam, frustum, cN + q, cE + q, half, depth + 1, out);
        return;
    }

    out.push_back(TerrainTile{cN, cE, size, depth});
}

} // namespace

void selectTerrainTiles(const QuadtreeParams& params,
                        const Vec3& cameraNed,
                        const Frustum* frustum,
                        double centerNorth, double centerEast,
                        std::vector<TerrainTile>& out) {
    recurse(params, cameraNed, frustum, centerNorth, centerEast,
            params.rootSize, 0, out);
}

} // namespace fsim
