// =============================================================================
//  tests/test_mat4.cpp -- 4x4 matrix, projection and view-matrix math.
// =============================================================================
#include "TestHarness.hpp"
#include "core/Mat4.hpp"

using namespace fsim;

TEST_CASE(mat4_identity_multiply) {
    const Mat4 I = Mat4::identity();
    const Mat4 P = I * I;
    for (int i = 0; i < 16; ++i)
        CHECK_NEAR(P.m[i], I.m[i], 1e-12);
    const Vec3 v{3, -2, 7};
    const Vec3 r = I.transformPoint(v);
    CHECK_NEAR(r.x, 3.0, 1e-12);
    CHECK_NEAR(r.y, -2.0, 1e-12);
    CHECK_NEAR(r.z, 7.0, 1e-12);
}

TEST_CASE(mat4_lookat_puts_target_in_front) {
    // Camera at (0,0,10) looking at the origin, Y up. The origin should map to
    // (0,0,-10) in view space (down the -Z axis = in front of the camera).
    const Mat4 view = lookAt(Vec3{0, 0, 10}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    const Vec3 p = view.transformPoint(Vec3{0, 0, 0});
    CHECK_NEAR(p.x, 0.0, 1e-9);
    CHECK_NEAR(p.y, 0.0, 1e-9);
    CHECK_NEAR(p.z, -10.0, 1e-9);
}

TEST_CASE(mat4_perspective_maps_near_far) {
    const double n = 1.0, f = 100.0;
    const Mat4 P = perspective(radians(90.0), 1.0, n, f);
    // A point on the near plane (z = -n) maps to NDC z = -1; far plane to +1.
    const Vec3 nearNdc = P.transformPoint(Vec3{0, 0, -n});
    const Vec3 farNdc  = P.transformPoint(Vec3{0, 0, -f});
    CHECK_NEAR(nearNdc.z, -1.0, 1e-6);
    CHECK_NEAR(farNdc.z,   1.0, 1e-6);
    // 90-degree vertical FOV, unit aspect: edge of view at z=-n is at y=+n.
    const Vec3 edge = P.transformPoint(Vec3{0, n, -n});
    CHECK_NEAR(edge.y, 1.0, 1e-6);
}

FSIM_TEST_MAIN()
