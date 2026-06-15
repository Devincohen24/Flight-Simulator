// =============================================================================
//  render/Frustum.hpp
//
//  View-frustum culling. The six clip planes are extracted from a combined
//  view-projection matrix (Gribb-Hartmann method) so that terrain tiles and
//  objects outside the camera's view can be skipped. Pure math in the headless
//  core, unit-tested without a GPU. Points/boxes are tested in GL world space
//  (the same space the view-projection maps from).
// =============================================================================
#pragma once

#include "core/Mat4.hpp"
#include "core/Math.hpp"

#include <array>
#include <cmath>

namespace fsim {

struct Plane {
    Vec3 normal{0, 0, 1};
    double d{0.0};
    double distance(const Vec3& p) const { return normal.dot(p) + d; }
};

struct AABB {
    Vec3 min{}, max{};
};

class Frustum {
public:
    // Build from a row-accessible view-projection matrix M (= proj * view).
    explicit Frustum(const Mat4& m) {
        // Rows of M: r0..r3, where r_i = (M(i,0), M(i,1), M(i,2), M(i,3)).
        auto row = [&](int i) {
            return std::array<double, 4>{m(i, 0), m(i, 1), m(i, 2), m(i, 3)};
        };
        const auto r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
        setPlane(0, r3, r0, +1); // left   : w + x
        setPlane(1, r3, r0, -1); // right  : w - x
        setPlane(2, r3, r1, +1); // bottom : w + y
        setPlane(3, r3, r1, -1); // top    : w - y
        setPlane(4, r3, r2, +1); // near   : w + z
        setPlane(5, r3, r2, -1); // far    : w - z
    }

    bool containsPoint(const Vec3& p) const {
        for (const Plane& pl : planes_)
            if (pl.distance(p) < 0.0) return false;
        return true;
    }

    // Conservative AABB test: visible unless entirely behind some plane.
    bool intersectsAABB(const AABB& box) const {
        for (const Plane& pl : planes_) {
            // The box's "positive vertex" w.r.t. this plane normal.
            const Vec3 pv{pl.normal.x >= 0 ? box.max.x : box.min.x,
                          pl.normal.y >= 0 ? box.max.y : box.min.y,
                          pl.normal.z >= 0 ? box.max.z : box.min.z};
            if (pl.distance(pv) < 0.0) return false; // fully outside this plane
        }
        return true;
    }

private:
    std::array<Plane, 6> planes_{};

    void setPlane(int i, const std::array<double, 4>& w,
                  const std::array<double, 4>& a, double sign) {
        Plane pl;
        pl.normal = Vec3{w[0] + sign * a[0], w[1] + sign * a[1], w[2] + sign * a[2]};
        pl.d = w[3] + sign * a[3];
        const double n = pl.normal.norm();
        if (n > 0.0) { pl.normal = pl.normal / n; pl.d /= n; }
        planes_[i] = pl;
    }
};

} // namespace fsim
