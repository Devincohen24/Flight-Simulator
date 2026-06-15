// =============================================================================
//  render/Camera.hpp
//
//  Camera systems. Each mode computes an eye/look-at/up triple in the NED world
//  frame from the aircraft's (interpolated) render state; the view matrix is
//  then produced by converting those vectors to GL space and forming a lookAt.
//  Projection is standard perspective. All of this is pure math living in the
//  headless core so it is unit-testable without a window.
//
//  Modes:
//    Chase   - trails behind and above the aircraft (external follow cam)
//    Cockpit - pilot eyepoint, looking forward along the body x-axis
//    Orbit   - circles the aircraft at a settable azimuth/elevation/radius
//    Flyby   - fixed world point tracking the aircraft (spotter cam)
// =============================================================================
#pragma once

#include "core/Mat4.hpp"
#include "render/RenderState.hpp"
#include "render/Coordinates.hpp"

namespace fsim {

enum class CameraMode { Chase, Cockpit, Orbit, Flyby };

class Camera {
public:
    CameraMode mode{CameraMode::Chase};

    // Projection.
    double fovY{radians(60.0)};
    double zNear{0.3};
    double zFar{50000.0};

    // Chase parameters.
    double chaseDistance{18.0}; // m behind
    double chaseHeight{5.0};    // m above

    // Cockpit eyepoint in the body frame (+x fwd, +z down -> up is negative z).
    Vec3 cockpitOffset{0.8, 0.0, -0.9};

    // Orbit parameters.
    double orbitRadius{25.0};
    double orbitAzimuth{radians(135.0)};   // bearing of the camera from aircraft
    double orbitElevation{radians(15.0)};  // above the horizontal

    // Flyby world position (NED).
    Vec3 flybyPositionNed{0.0, 0.0, -50.0};

    // Resolve eye/center/up (NED) for the current mode and aircraft state.
    void update(const RenderState& s);

    // GL-space view matrix (and combined view-projection).
    Mat4 viewMatrix() const;
    Mat4 viewProjection(double aspectRatio) const;

    // Resolved vectors (NED), exposed for tests / HUD.
    const Vec3& eyeNed()    const { return eyeNed_; }
    const Vec3& centerNed() const { return centerNed_; }
    const Vec3& upNed()     const { return upNed_; }

private:
    Vec3 eyeNed_{};
    Vec3 centerNed_{};
    Vec3 upNed_{0.0, 0.0, -1.0};
};

} // namespace fsim
