// =============================================================================
//  physics/RigidBody.hpp
//
//  6-degree-of-freedom rigid-body state and the Newton-Euler equations of
//  motion that produce its time derivative. This is the core of the simulator:
//  every aircraft motion emerges from integrating these equations under the
//  forces and moments supplied by gravity, aerodynamics, propulsion and ground
//  contact. No motion is ever scripted.
//
//  Equations of motion (all vectors in the BODY frame unless noted):
//
//    Translational (Newton, in a rotating body frame):
//        v_dot = F/m  -  omega x v
//
//    Rotational (Euler):
//        omega_dot = I^-1 ( M  -  omega x (I omega) )
//
//    Attitude kinematics (quaternion, q = q_world_from_body):
//        q_dot = 1/2 * q (x) [0, omega]
//
//    Position (world/NED):
//        p_dot = R(q) * v        (body velocity rotated into the world frame)
// =============================================================================
#pragma once

#include "core/Math.hpp"

namespace fsim {

// -----------------------------------------------------------------------------
// State of the rigid body. The SAME struct doubles as the *derivative* of the
// state during integration (position'=velocity, velocity'=accel, etc.), which
// is what makes the RK4 stage arithmetic below clean.
// -----------------------------------------------------------------------------
struct RigidBodyState {
    Vec3 positionWorld{};        // NED position, metres
    Vec3 velocityBody{};         // body-frame velocity (u, v, w), m/s
    Quat orientation{Quat::identity()}; // q_world_from_body
    Vec3 angularVelocityBody{};  // body rates (p, q, r), rad/s

    // Component-wise operations used by the RK4 integrator. The quaternion is
    // treated as a 4-vector here; the integrator renormalises after the step.
    RigidBodyState operator+(const RigidBodyState& o) const {
        return {positionWorld + o.positionWorld,
                velocityBody + o.velocityBody,
                orientation + o.orientation,
                angularVelocityBody + o.angularVelocityBody};
    }
    RigidBodyState operator*(double s) const {
        return {positionWorld * s,
                velocityBody * s,
                orientation * s,
                angularVelocityBody * s};
    }
};

// -----------------------------------------------------------------------------
// Mass properties. The inverse inertia tensor is cached because it is needed
// every derivative evaluation (4x per RK4 step) but only changes when fuel
// burns / stores are dropped.
// -----------------------------------------------------------------------------
struct MassProperties {
    double mass{1.0};              // kg
    Mat3   inertia{Mat3::identity()};      // body-frame inertia tensor, kg*m^2
    Mat3   inertiaInverse{Mat3::identity()};

    void setInertia(const Mat3& I) {
        inertia = I;
        inertiaInverse = I.inverse();
    }
    static MassProperties make(double mass, const Mat3& I) {
        MassProperties mp;
        mp.mass = mass;
        mp.setInertia(I);
        return mp;
    }
};

// -----------------------------------------------------------------------------
// A force and moment applied at the centre of gravity, expressed in the BODY
// frame. Force models accumulate into this. (Forces applied off the CG must
// also contribute a moment r x F; that bookkeeping lives in the force models.)
// -----------------------------------------------------------------------------
struct Wrench {
    Vec3 force{};   // N,   body frame
    Vec3 moment{};  // N*m, body frame

    Wrench& operator+=(const Wrench& o) { force += o.force; moment += o.moment; return *this; }
};

// Evaluate the time derivative of the state given the total wrench acting on it.
RigidBodyState computeDerivative(const RigidBodyState& s,
                                 const MassProperties& mp,
                                 const Wrench& w);

} // namespace fsim
