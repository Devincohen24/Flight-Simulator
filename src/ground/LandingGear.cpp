// =============================================================================
//  ground/LandingGear.cpp -- see LandingGear.hpp for the model description.
// =============================================================================
#include "ground/LandingGear.hpp"

#include <cmath>

namespace fsim {

namespace {
constexpr double kVelEpsLong = 1.0;  // m/s, longitudinal friction saturation
constexpr double kVelEpsLat  = 0.5;  // m/s, lateral friction saturation
const Vec3 kWorldUp{0.0, 0.0, -1.0}; // NED: up is -Z
} // namespace

Wrench LandingGearForce::evaluate(const RigidBodyState& state,
                                  const MassProperties& /*mass*/,
                                  const EnvironmentSample& /*env*/,
                                  const ControlInputs& controls) const {
    Wrench total;

    const Quat& q = state.orientation;
    const Vec3 vCgWorld    = q.rotate(state.velocityBody);
    const Vec3 omegaWorld  = q.rotate(state.angularVelocityBody);

    for (const GearUnit& g : gear_) {
        // Contact-point geometry in the world frame.
        const Vec3 rWorld = q.rotate(g.position);
        const Vec3 pWorld = state.positionWorld + rWorld;

        // Penetration below the (flat) ground plane. NED: down is +Z, and the
        // ground sits at NED z = -elevation, so penetration = p_z + elevation.
        const double penetration = pWorld.z + groundElevation_;
        if (penetration <= 0.0) continue; // wheel is above the ground

        // Velocity of the contact point (world frame).
        const Vec3 vContact = vCgWorld + omegaWorld.cross(rWorld);

        // Strut normal force: spring (compression) + damper (closing rate),
        // never tensile (the wheel cannot pull the aircraft down).
        double N = g.stiffness * penetration + g.damping * vContact.z;
        if (N < 0.0) N = 0.0;
        const Vec3 normalForceWorld{0.0, 0.0, -N}; // pushes the aircraft up

        // --- Tire friction in the ground plane ---
        Vec3 frictionWorld{0.0, 0.0, 0.0};

        // Forward (rolling) direction = body x-axis projected onto the ground.
        Vec3 forward = q.rotate(Vec3{1.0, 0.0, 0.0});
        forward.z = 0.0;
        const double fwdLen = forward.norm();
        if (fwdLen > 1e-6 && N > 0.0) {
            forward = forward / fwdLen;
            // Lateral direction completes a right-handed ground frame.
            const Vec3 lateral = kWorldUp.cross(forward).normalized();

            // Horizontal contact velocity components.
            const Vec3 vGround{vContact.x, vContact.y, 0.0};
            const double vLong = vGround.dot(forward);
            const double vLat  = vGround.dot(lateral);

            const double muLong = g.frictionRoll
                                + (g.braked ? controls.brake * g.frictionBrake : 0.0);
            const double fLong = -muLong * N * std::tanh(vLong / kVelEpsLong);
            const double fLat  = -g.frictionSide * N * std::tanh(vLat / kVelEpsLat);

            frictionWorld = forward * fLong + lateral * fLat;
        }

        // Combine, rotate into the body frame, and accumulate force + moment.
        const Vec3 forceWorld = normalForceWorld + frictionWorld;
        const Vec3 forceBody  = q.rotateInverse(forceWorld);
        total.force  += forceBody;
        total.moment += g.position.cross(forceBody);
    }

    return total;
}

bool LandingGearForce::anyContact(const RigidBodyState& state) const {
    const Quat& q = state.orientation;
    for (const GearUnit& g : gear_) {
        const Vec3 pWorld = state.positionWorld + q.rotate(g.position);
        if (pWorld.z + groundElevation_ > 0.0) return true;
    }
    return false;
}

} // namespace fsim
