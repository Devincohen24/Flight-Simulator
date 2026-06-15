// =============================================================================
//  physics/Integrator.hpp
//
//  Numerical integration of the 6DOF equations of motion. The default is a
//  classical fourth-order Runge-Kutta (RK4) scheme, chosen for its accuracy
//  and good stability for the moderately stiff dynamics of aircraft motion.
//  A semi-implicit Euler is also provided for performance comparison / sanity.
//
//  The integrator is generic over the force evaluation: it takes any callable
//  (state) -> Wrench, so gravity, aerodynamics, propulsion and ground contact
//  all flow through the same path. The quaternion is renormalised after each
//  full step to counter the slow drift caused by treating it as a 4-vector
//  during the stage arithmetic.
// =============================================================================
#pragma once

#include "physics/RigidBody.hpp"

#include <functional>

namespace fsim {

// A force functor returns the total body-frame wrench for a given state.
using ForceFunction = std::function<Wrench(const RigidBodyState&)>;

enum class IntegratorType { RK4, SemiImplicitEuler };

// Advance the state by dt seconds.
inline RigidBodyState integrate(const RigidBodyState& s0,
                                const MassProperties& mp,
                                const ForceFunction& forces,
                                double dt,
                                IntegratorType type = IntegratorType::RK4) {
    auto deriv = [&](const RigidBodyState& s) {
        return computeDerivative(s, mp, forces(s));
    };

    RigidBodyState out;

    if (type == IntegratorType::RK4) {
        const RigidBodyState k1 = deriv(s0);
        const RigidBodyState k2 = deriv(s0 + k1 * (dt * 0.5));
        const RigidBodyState k3 = deriv(s0 + k2 * (dt * 0.5));
        const RigidBodyState k4 = deriv(s0 + k3 * dt);
        out = s0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
    } else {
        // Semi-implicit (symplectic) Euler: update velocities first, then use
        // the new velocities to propagate position/attitude. More stable than
        // explicit Euler for oscillatory systems.
        const RigidBodyState d = deriv(s0);
        out = s0;
        out.velocityBody        = s0.velocityBody        + d.velocityBody * dt;
        out.angularVelocityBody = s0.angularVelocityBody + d.angularVelocityBody * dt;
        RigidBodyState s1 = out; // state with updated velocities
        const RigidBodyState d1 = computeDerivative(s1, mp, forces(s1));
        out.positionWorld = s0.positionWorld + d1.positionWorld * dt;
        out.orientation   = s0.orientation   + d1.orientation   * dt;
    }

    // Keep the attitude quaternion on the unit sphere.
    out.orientation = out.orientation.normalized();
    return out;
}

} // namespace fsim
