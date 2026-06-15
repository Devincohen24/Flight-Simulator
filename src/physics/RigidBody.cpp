// =============================================================================
//  physics/RigidBody.cpp -- Newton-Euler equations of motion.
//  See RigidBody.hpp for the derivation and frame conventions.
// =============================================================================
#include "physics/RigidBody.hpp"

namespace fsim {

RigidBodyState computeDerivative(const RigidBodyState& s,
                                 const MassProperties& mp,
                                 const Wrench& w) {
    RigidBodyState d;

    // Position rate: body velocity expressed in the world (NED) frame.
    d.positionWorld = s.orientation.rotate(s.velocityBody);

    // Translational acceleration in the body frame:
    //   v_dot = F/m - omega x v
    d.velocityBody = w.force * (1.0 / mp.mass)
                   - s.angularVelocityBody.cross(s.velocityBody);

    // Attitude kinematics: q_dot = 1/2 * q (x) [0, omega].
    const Quat omegaQuat{0.0,
                         s.angularVelocityBody.x,
                         s.angularVelocityBody.y,
                         s.angularVelocityBody.z};
    d.orientation = (s.orientation * omegaQuat) * 0.5;

    // Rotational acceleration (Euler's equation):
    //   omega_dot = I^-1 ( M - omega x (I omega) )
    const Vec3 Iomega = mp.inertia * s.angularVelocityBody;
    d.angularVelocityBody =
        mp.inertiaInverse * (w.moment - s.angularVelocityBody.cross(Iomega));

    return d;
}

} // namespace fsim
