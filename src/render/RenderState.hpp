// =============================================================================
//  render/RenderState.hpp
//
//  Bridges the fixed-timestep physics with variable-rate rendering. The physics
//  advances in discrete fixed steps (240 Hz); a frame may fall between two of
//  them. To avoid visual stutter the renderer interpolates between the previous
//  and current physics states by the fractional leftover time
//  (accumulator / dt), using linear interpolation for position/velocity and
//  spherical linear interpolation (slerp) for attitude.
// =============================================================================
#pragma once

#include "physics/RigidBody.hpp"

namespace fsim {

struct RenderState {
    Vec3 positionWorld{};   // interpolated NED position
    Quat orientation{Quat::identity()};
    Vec3 velocityBody{};
};

// Interpolate between two physics states. alpha = fractional time in [0,1]
// between 'previous' (alpha=0) and 'current' (alpha=1).
inline RenderState interpolateState(const RigidBodyState& previous,
                                    const RigidBodyState& current,
                                    double alpha) {
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    RenderState r;
    r.positionWorld = previous.positionWorld
                    + (current.positionWorld - previous.positionWorld) * alpha;
    r.velocityBody  = previous.velocityBody
                    + (current.velocityBody - previous.velocityBody) * alpha;
    r.orientation   = Quat::slerp(previous.orientation, current.orientation, alpha);
    return r;
}

} // namespace fsim
