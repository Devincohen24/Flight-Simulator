// =============================================================================
//  core/Frames.hpp
//
//  Documentation of the coordinate frames and their relationships. There is no
//  code here beyond small helpers; the value is in pinning the conventions so
//  every module agrees. Getting these wrong is the classic source of flight-sim
//  bugs (inverted controls, sign errors in moments, etc.).
//
//  BODY frame (b):   origin at the centre of gravity.
//      +X  forward, out the nose
//      +Y  right,   out the starboard wing
//      +Z  down,    through the belly
//    Body velocity components are named (u, v, w); body angular rates (p, q, r)
//    about (X, Y, Z) i.e. (roll, pitch, yaw) rates.
//
//  WORLD frame (n):  local-level North-East-Down (NED), treated as inertial for
//                    flight-dynamics purposes (flat-Earth assumption for now).
//      +X  North
//      +Y  East
//      +Z  Down   ->  gravity acts along +Z.
//
//  Attitude:  a Quat q == q_world_from_body, so
//      v_world = q.rotate(v_body)        (and v_body = q.rotateInverse(v_world)).
//
//  RENDER frame (Milestone 3): OpenGL is Y-up, right-handed. Conversion from NED
//  to GL happens only at the rendering boundary; the physics never uses it.
// =============================================================================
#pragma once

#include "core/Math.hpp"
#include "core/Constants.hpp"

namespace fsim {

// Gravity vector expressed in the WORLD (NED) frame: it points "down" (+Z).
inline Vec3 gravityWorld() {
    return {0.0, 0.0, constants::kStandardGravity};
}

} // namespace fsim
