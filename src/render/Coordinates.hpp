// =============================================================================
//  render/Coordinates.hpp
//
//  The single, isolated boundary between the simulation's NED world frame and
//  OpenGL's rendering frame. The physics NEVER uses GL coordinates; only the
//  renderer converts, and only here.
//
//    NED (sim):  +X North, +Y East, +Z Down
//    GL  (view): +X Right(East), +Y Up, +Z toward the viewer (out of screen)
//
//  Mapping (a proper, handedness-preserving rotation, det = +1):
//        gl.x =  ned.y      (East  -> right)
//        gl.y = -ned.z      (Up    -> up; NED down is +Z)
//        gl.z = -ned.x      (North -> into the screen, i.e. -Z)
//
//  The same linear map applies to directions (velocities, axes). Quaternion
//  attitudes are converted by conjugating the rotation with this map, but in
//  practice the renderer builds an aircraft model matrix from the body axes
//  rotated into NED and then converted per-axis, which is equivalent and
//  avoids a second quaternion convention.
// =============================================================================
#pragma once

#include "core/Math.hpp"

namespace fsim {

inline Vec3 nedToGl(const Vec3& ned) {
    return {ned.y, -ned.z, -ned.x};
}

// Same rotation for a direction vector (there is no translation component).
inline Vec3 nedToGlDir(const Vec3& ned) {
    return {ned.y, -ned.z, -ned.x};
}

} // namespace fsim
