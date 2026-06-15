// =============================================================================
//  render/Camera.cpp -- see Camera.hpp for the camera-mode descriptions.
// =============================================================================
#include "render/Camera.hpp"

#include <cmath>

namespace fsim {

namespace {
const Vec3 kWorldUpNed{0.0, 0.0, -1.0}; // "up" is -Z in NED
} // namespace

void Camera::update(const RenderState& s) {
    const Vec3 pos = s.positionWorld;
    const Vec3 forwardNed = s.orientation.rotate(Vec3{1.0, 0.0, 0.0}); // body x in NED

    switch (mode) {
        case CameraMode::Chase: {
            // Trail behind along the (full 3D) nose direction, lifted "up".
            eyeNed_    = pos - forwardNed * chaseDistance + kWorldUpNed * chaseHeight;
            centerNed_ = pos;
            upNed_     = kWorldUpNed;
            break;
        }
        case CameraMode::Cockpit: {
            eyeNed_    = pos + s.orientation.rotate(cockpitOffset);
            centerNed_ = eyeNed_ + forwardNed;
            upNed_     = s.orientation.rotate(Vec3{0.0, 0.0, -1.0}); // body up
            break;
        }
        case CameraMode::Orbit: {
            const double ce = std::cos(orbitElevation);
            const double se = std::sin(orbitElevation);
            const Vec3 offset{orbitRadius * ce * std::cos(orbitAzimuth),
                              orbitRadius * ce * std::sin(orbitAzimuth),
                              -orbitRadius * se}; // up is -Z
            eyeNed_    = pos + offset;
            centerNed_ = pos;
            upNed_     = kWorldUpNed;
            break;
        }
        case CameraMode::Flyby: {
            eyeNed_    = flybyPositionNed;
            centerNed_ = pos;
            upNed_     = kWorldUpNed;
            break;
        }
    }
}

Mat4 Camera::viewMatrix() const {
    return lookAt(nedToGl(eyeNed_), nedToGl(centerNed_), nedToGlDir(upNed_));
}

Mat4 Camera::viewProjection(double aspectRatio) const {
    return perspective(fovY, aspectRatio, zNear, zFar) * viewMatrix();
}

} // namespace fsim
