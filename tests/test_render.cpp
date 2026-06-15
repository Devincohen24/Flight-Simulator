// =============================================================================
//  tests/test_render.cpp -- NED<->GL conversion, state interpolation, cameras,
//  and frustum culling (all GPU-free render math).
// =============================================================================
#include "TestHarness.hpp"
#include "render/Coordinates.hpp"
#include "render/RenderState.hpp"
#include "render/Camera.hpp"
#include "render/Frustum.hpp"

using namespace fsim;

TEST_CASE(ned_to_gl_axes) {
    // North -> into screen (-Z), East -> right (+X), Down -> down (-Y).
    const Vec3 north = nedToGl(Vec3{1, 0, 0});
    const Vec3 east  = nedToGl(Vec3{0, 1, 0});
    const Vec3 down  = nedToGl(Vec3{0, 0, 1});
    CHECK_NEAR(north.z, -1.0, 1e-12);
    CHECK_NEAR(east.x,   1.0, 1e-12);
    CHECK_NEAR(down.y,  -1.0, 1e-12);
    // NED "up" maps to GL +Y.
    const Vec3 up = nedToGl(Vec3{0, 0, -1});
    CHECK_NEAR(up.y, 1.0, 1e-12);
}

TEST_CASE(interpolation_endpoints_and_midpoint) {
    RigidBodyState a, b;
    a.positionWorld = Vec3{0, 0, 0};
    b.positionWorld = Vec3{10, 20, -30};
    a.orientation = Quat::fromEuler(0, 0, 0);
    b.orientation = Quat::fromEuler(0, 0, radians(90.0));

    const RenderState s0 = interpolateState(a, b, 0.0);
    const RenderState s1 = interpolateState(a, b, 1.0);
    const RenderState sm = interpolateState(a, b, 0.5);

    CHECK_NEAR(s0.positionWorld.x, 0.0, 1e-12);
    CHECK_NEAR(s1.positionWorld.z, -30.0, 1e-12);
    CHECK_NEAR(sm.positionWorld.y, 10.0, 1e-12);
    // Slerp midpoint of a 90-degree yaw is 45 degrees.
    const EulerAngles e = toEuler(sm.orientation);
    CHECK_NEAR(e.yaw, radians(45.0), 1e-9);
    CHECK_NEAR(sm.orientation.norm(), 1.0, 1e-12);
}

TEST_CASE(chase_camera_behind_and_above) {
    RenderState s;
    s.positionWorld = Vec3{0, 0, -500};        // 500 m up
    s.orientation = Quat::fromEuler(0, 0, 0);  // nose North

    Camera cam;
    cam.mode = CameraMode::Chase;
    cam.update(s);

    // Behind = south of the aircraft (smaller North); above = more negative Z.
    CHECK(cam.eyeNed().x < s.positionWorld.x);
    CHECK(cam.eyeNed().z < s.positionWorld.z);

    // In GL view space the aircraft must be in front of the camera (z < 0).
    const Mat4 view = cam.viewMatrix();
    const Vec3 acGl = nedToGl(s.positionWorld);
    CHECK(view.transformPoint(acGl).z < 0.0);
}

TEST_CASE(cockpit_camera_looks_forward) {
    RenderState s;
    s.orientation = Quat::fromEuler(0, 0, radians(30.0)); // yawed 30 deg
    Camera cam;
    cam.mode = CameraMode::Cockpit;
    cam.update(s);

    const Vec3 viewDir = (cam.centerNed() - cam.eyeNed()).normalized();
    const Vec3 bodyFwd = s.orientation.rotate(Vec3{1, 0, 0});
    CHECK_NEAR(viewDir.x, bodyFwd.x, 1e-9);
    CHECK_NEAR(viewDir.y, bodyFwd.y, 1e-9);
}

TEST_CASE(frustum_contains_and_culls) {
    Camera cam;
    cam.mode = CameraMode::Flyby;
    cam.flybyPositionNed = Vec3{-50, 0, -10}; // south & above, looking at origin
    RenderState s; s.positionWorld = Vec3{0, 0, 0};
    cam.update(s);

    const Frustum fr(cam.viewProjection(16.0 / 9.0));
    // The aircraft (at the look-at target) is visible.
    CHECK(fr.containsPoint(nedToGl(Vec3{0, 0, 0})));
    // A point far behind the camera is culled.
    CHECK(!fr.containsPoint(nedToGl(Vec3{-200, 0, -10})));
}

FSIM_TEST_MAIN()
