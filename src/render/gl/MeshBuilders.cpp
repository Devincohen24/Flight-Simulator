// =============================================================================
//  render/gl/MeshBuilders.cpp
// =============================================================================
#include "render/gl/MeshBuilders.hpp"
#include "render/Coordinates.hpp"

#include <cmath>

namespace fsim::gl {

namespace {
Vertex makeVertex(const Vec3& posGl, const Vec3& nrmGl) {
    return Vertex{float(posGl.x), float(posGl.y), float(posGl.z),
                  float(nrmGl.x), float(nrmGl.y), float(nrmGl.z)};
}
} // namespace

void buildTerrainMesh(const HeightField& field,
                      double focusNorth, double focusEast,
                      double extent, int segments,
                      std::vector<Vertex>& outVerts,
                      std::vector<uint32_t>& outIndices) {
    outVerts.clear();
    outIndices.clear();
    if (segments < 1) segments = 1;

    const double step = extent / segments;
    // Snap the patch origin to the grid so it does not shimmer as we move.
    const double n0 = std::floor((focusNorth - extent * 0.5) / step) * step;
    const double e0 = std::floor((focusEast  - extent * 0.5) / step) * step;

    outVerts.reserve(static_cast<size_t>((segments + 1) * (segments + 1)));
    for (int j = 0; j <= segments; ++j) {
        for (int i = 0; i <= segments; ++i) {
            const double north = n0 + i * step;
            const double east  = e0 + j * step;
            const double h = field.height(north, east);
            // NED ground point: down = -elevation; convert to GL.
            const Vec3 posGl = nedToGl(Vec3{north, east, -h});
            const Vec3 nrmGl = nedToGlDir(field.normalNed(north, east, step));
            outVerts.push_back(makeVertex(posGl, nrmGl));
        }
    }

    const int stride = segments + 1;
    outIndices.reserve(static_cast<size_t>(segments * segments * 6));
    for (int j = 0; j < segments; ++j) {
        for (int i = 0; i < segments; ++i) {
            const uint32_t a = static_cast<uint32_t>(j * stride + i);
            const uint32_t b = static_cast<uint32_t>(j * stride + i + 1);
            const uint32_t c = static_cast<uint32_t>((j + 1) * stride + i);
            const uint32_t d = static_cast<uint32_t>((j + 1) * stride + i + 1);
            outIndices.insert(outIndices.end(), {a, c, b, b, c, d});
        }
    }
}

void buildAircraftMesh(std::vector<Vertex>& outVerts,
                       std::vector<uint32_t>& outIndices) {
    outVerts.clear();
    outIndices.clear();

    // Helper: add a flat triangle (body coords), normal from the winding.
    auto addTri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        const Vec3 nNed = (b - a).cross(c - a).normalized();
        const Vec3 nGl  = nedToGlDir(nNed);
        const uint32_t base = static_cast<uint32_t>(outVerts.size());
        outVerts.push_back(makeVertex(nedToGl(a), nGl));
        outVerts.push_back(makeVertex(nedToGl(b), nGl));
        outVerts.push_back(makeVertex(nedToGl(c), nGl));
        outIndices.insert(outIndices.end(), {base, base + 1, base + 2});
    };
    auto addQuad = [&](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        addTri(a, b, c);
        addTri(a, c, d);
    };

    // Dimensions (m), body frame: +x nose, +y right, +z down.
    const double noseX = 4.0, tailX = -4.0;
    const double halfSpan = 5.5, wingX = -0.2, wingChord = 1.4;
    const double finTopZ = -1.6, finX = -3.0;
    const double htSpan = 1.8;

    // Fuselage: a slender tetra-ish prism (nose to tail, small cross section).
    const Vec3 nose{noseX, 0, 0};
    const Vec3 tail{tailX, 0, 0};
    const double r = 0.5;
    const Vec3 fl{0, -r, 0}, fr{0, r, 0}, ft{0, 0, -r}, fb{0, 0, r};
    addTri(nose, fr, ft); addTri(nose, ft, fl);
    addTri(nose, fl, fb); addTri(nose, fb, fr);
    addTri(tail, ft, fr); addTri(tail, fl, ft);
    addTri(tail, fb, fl); addTri(tail, fr, fb);

    // Main wing (thin quad, both faces).
    const Vec3 wlf{wingX + wingChord * 0.5, -halfSpan, 0};
    const Vec3 wlb{wingX - wingChord * 0.5, -halfSpan, 0};
    const Vec3 wrf{wingX + wingChord * 0.5,  halfSpan, 0};
    const Vec3 wrb{wingX - wingChord * 0.5,  halfSpan, 0};
    addQuad(wlf, wrf, wrb, wlb);
    addQuad(wlb, wrb, wrf, wlf); // back face

    // Horizontal stabiliser.
    const Vec3 hlf{tailX + 1.0, -htSpan, 0};
    const Vec3 hlb{tailX, -htSpan, 0};
    const Vec3 hrf{tailX + 1.0,  htSpan, 0};
    const Vec3 hrb{tailX,  htSpan, 0};
    addQuad(hlf, hrf, hrb, hlb);
    addQuad(hlb, hrb, hrf, hlf);

    // Vertical fin (in the x-z plane).
    const Vec3 ftb{finX + 0.8, 0, 0};
    const Vec3 fbb{finX, 0, 0};
    const Vec3 ftt{finX + 0.4, 0, finTopZ};
    addTri(fbb, ftb, ftt);
    addTri(ftb, fbb, ftt);
}

} // namespace fsim::gl
