// =============================================================================
//  tests/test_math.cpp -- Vec3, Mat3 and Quat correctness.
// =============================================================================
#include "TestHarness.hpp"
#include "core/Math.hpp"

using namespace fsim;

TEST_CASE(vec3_dot_and_cross) {
    const Vec3 a{1, 2, 3}, b{4, 5, 6};
    CHECK_NEAR(a.dot(b), 32.0, 1e-12);
    const Vec3 c = a.cross(b);          // (-3, 6, -3)
    CHECK_NEAR(c.x, -3.0, 1e-12);
    CHECK_NEAR(c.y,  6.0, 1e-12);
    CHECK_NEAR(c.z, -3.0, 1e-12);
    // x cross y = z for a right-handed frame.
    const Vec3 z = Vec3{1, 0, 0}.cross(Vec3{0, 1, 0});
    CHECK_NEAR(z.z, 1.0, 1e-12);
}

TEST_CASE(vec3_norm) {
    const Vec3 v{3, 4, 0};
    CHECK_NEAR(v.norm(), 5.0, 1e-12);
    CHECK_NEAR(v.normalized().norm(), 1.0, 1e-12);
}

TEST_CASE(mat3_inverse_identity) {
    const Mat3 I = Mat3::identity();
    const Mat3 Inv = I.inverse();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(Inv.m[i][j], (i == j) ? 1.0 : 0.0, 1e-12);
}

TEST_CASE(mat3_inverse_roundtrip) {
    // A non-trivial inertia-like tensor with an Ixz cross term.
    const Mat3 I = Mat3::inertia(1000.0, 3000.0, 3500.0, 120.0);
    const Mat3 Inv = I.inverse();
    // I * I^-1 == identity.
    Mat3 prod;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += I.m[i][k] * Inv.m[k][j];
            prod.m[i][j] = s;
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(prod.m[i][j], (i == j) ? 1.0 : 0.0, 1e-9);
}

TEST_CASE(quat_identity_rotation) {
    const Quat q = Quat::identity();
    const Vec3 v{1, 2, 3};
    const Vec3 r = q.rotate(v);
    CHECK_NEAR(r.x, 1.0, 1e-12);
    CHECK_NEAR(r.y, 2.0, 1e-12);
    CHECK_NEAR(r.z, 3.0, 1e-12);
}

TEST_CASE(quat_90deg_yaw) {
    // Yaw +90 deg about world Z. Body +X (nose, North) should map to +Y (East).
    const Quat q = Quat::fromEuler(0.0, 0.0, radians(90.0));
    const Vec3 nose = q.rotate(Vec3{1, 0, 0});
    CHECK_NEAR(nose.x, 0.0, 1e-9);
    CHECK_NEAR(nose.y, 1.0, 1e-9);
    CHECK_NEAR(nose.z, 0.0, 1e-9);
}

TEST_CASE(quat_euler_roundtrip) {
    const double roll = radians(20.0), pitch = radians(-12.0), yaw = radians(75.0);
    const Quat q = Quat::fromEuler(roll, pitch, yaw);
    const EulerAngles e = toEuler(q);
    CHECK_NEAR(e.roll,  roll,  1e-9);
    CHECK_NEAR(e.pitch, pitch, 1e-9);
    CHECK_NEAR(e.yaw,   yaw,   1e-9);
}

TEST_CASE(quat_rotate_inverse_roundtrip) {
    const Quat q = Quat::fromEuler(radians(30), radians(40), radians(50));
    const Vec3 v{2.5, -1.0, 4.0};
    const Vec3 back = q.rotateInverse(q.rotate(v));
    CHECK_NEAR(back.x, v.x, 1e-9);
    CHECK_NEAR(back.y, v.y, 1e-9);
    CHECK_NEAR(back.z, v.z, 1e-9);
}

FSIM_TEST_MAIN()
