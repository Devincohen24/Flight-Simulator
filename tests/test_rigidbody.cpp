// =============================================================================
//  tests/test_rigidbody.cpp
//
//  Validates the 6DOF equations of motion and the RK4 integrator against
//  closed-form / conservation-law references. These checks are the safety net
//  that lets later phases (aero, propulsion) trust the core dynamics.
// =============================================================================
#include "TestHarness.hpp"
#include "physics/RigidBody.hpp"
#include "physics/Integrator.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"
#include "core/Constants.hpp"

#include <memory>

using namespace fsim;
using constants::kStandardGravity;

// --- 1. Ballistic drop: constant gravity should reproduce the kinematic
//        equations z = 1/2 g t^2 and v = g t exactly (RK4 is exact for this). --
TEST_CASE(ballistic_vertical_drop) {
    Simulation sim(1.0 / 240.0);
    MassProperties mp = MassProperties::make(10.0, Mat3::inertia(1, 1, 1));
    sim.setMassProperties(mp);
    sim.addForceModel(std::make_shared<GravityForce>());

    RigidBodyState s0;             // at rest, identity attitude, sea level
    sim.setState(s0);

    // Step an exact integer number of fixed steps so the comparison is against
    // the closed-form result (the accumulator in advance() intentionally leaves
    // a sub-step remainder, which is handled by render interpolation instead).
    const int N = 720;
    for (int i = 0; i < N; ++i) sim.step();
    const double T = N * sim.fixedTimeStep();

    const double expectedZ  = 0.5 * kStandardGravity * T * T; // down is +Z
    const double expectedVz = kStandardGravity * T;
    CHECK_NEAR(sim.state().positionWorld.z, expectedZ,  1e-4);
    CHECK_NEAR(sim.state().velocityBody.z,  expectedVz, 1e-6);
    // No horizontal drift.
    CHECK_NEAR(sim.state().positionWorld.x, 0.0, 1e-9);
    CHECK_NEAR(sim.state().positionWorld.y, 0.0, 1e-9);
}

// --- 2. Projectile with horizontal launch: parabolic trajectory. ------------
TEST_CASE(ballistic_projectile_parabola) {
    Simulation sim(1.0 / 480.0);
    sim.setMassProperties(MassProperties::make(2.0, Mat3::inertia(1, 1, 1)));
    sim.addForceModel(std::make_shared<GravityForce>());

    RigidBodyState s0;
    s0.velocityBody = Vec3{50.0, 0.0, 0.0}; // 50 m/s north, level attitude
    sim.setState(s0);

    const int N = 960;
    for (int i = 0; i < N; ++i) sim.step();
    const double T = N * sim.fixedTimeStep();

    CHECK_NEAR(sim.state().positionWorld.x, 50.0 * T, 1e-3);              // x = v t
    CHECK_NEAR(sim.state().positionWorld.z, 0.5 * kStandardGravity * T * T, 1e-3);
    CHECK_NEAR(sim.state().velocityBody.x, 50.0, 1e-6);                   // unchanged
}

// --- 3. Energy conservation for a conservative (gravity-only) trajectory. ---
TEST_CASE(energy_conservation_ballistic) {
    const MassProperties mp = MassProperties::make(5.0, Mat3::inertia(1, 1, 1));
    const ForceFunction forces = [&](const RigidBodyState& s) {
        const Vec3 weightWorld{0, 0, mp.mass * kStandardGravity};
        return Wrench{ s.orientation.rotateInverse(weightWorld), Vec3{0, 0, 0} };
    };

    RigidBodyState s;
    s.velocityBody = Vec3{30.0, 0.0, -40.0}; // launched up and forward

    auto energy = [&](const RigidBodyState& st) {
        const double speed2 = st.velocityBody.normSquared();
        const double altitude = -st.positionWorld.z;
        return 0.5 * mp.mass * speed2 + mp.mass * kStandardGravity * altitude;
    };
    const double E0 = energy(s);

    const double dt = 1.0 / 240.0;
    for (int i = 0; i < 2400; ++i)            // 10 s
        s = integrate(s, mp, forces, dt, IntegratorType::RK4);

    CHECK_NEAR(energy(s), E0, 1e-3 * E0);     // < 0.1% drift
}

// --- 4. Torque-free symmetric body spins at a constant rate; quaternion
//        stays normalised over many steps. ----------------------------------
TEST_CASE(torque_free_symmetric_spin) {
    const MassProperties mp = MassProperties::make(1.0, Mat3::inertia(2, 2, 2));
    const ForceFunction noForce = [](const RigidBodyState&) { return Wrench{}; };

    RigidBodyState s;
    s.angularVelocityBody = Vec3{0.0, 0.0, 1.0}; // 1 rad/s about body Z (yaw)

    const double dt = 1.0 / 240.0;
    const double T = 5.0;
    for (int i = 0; i < int(T / dt); ++i)
        s = integrate(s, mp, noForce, dt, IntegratorType::RK4);

    // Rate unchanged (no torque), attitude quaternion still unit length.
    CHECK_NEAR(s.angularVelocityBody.z, 1.0, 1e-9);
    CHECK_NEAR(s.orientation.norm(), 1.0, 1e-9);
    // After 5 s at 1 rad/s the body has yawed 5 rad; check the heading.
    const EulerAngles e = toEuler(s.orientation);
    double expectedYaw = std::fmod(5.0, 2.0 * kPi);
    if (expectedYaw > kPi) expectedYaw -= 2.0 * kPi;
    CHECK_NEAR(e.yaw, expectedYaw, 1e-4);
}

// --- 5. Torque-free ASYMMETRIC body: Euler's equation must conserve both the
//        magnitude of angular momentum (in the world frame) and rotational
//        kinetic energy. This exercises the omega x (I omega) coupling. ------
TEST_CASE(torque_free_asymmetric_conservation) {
    const Mat3 I = Mat3::inertia(1000.0, 3000.0, 3500.0); // distinct principal axes
    const MassProperties mp = MassProperties::make(1.0, I);
    const ForceFunction noForce = [](const RigidBodyState&) { return Wrench{}; };

    RigidBodyState s;
    s.angularVelocityBody = Vec3{0.6, 0.2, 0.9}; // tumbling about all three axes

    auto angularMomentumWorldMag = [&](const RigidBodyState& st) {
        const Vec3 Hbody = mp.inertia * st.angularVelocityBody;
        return st.orientation.rotate(Hbody).norm();
    };
    auto rotationalEnergy = [&](const RigidBodyState& st) {
        return 0.5 * st.angularVelocityBody.dot(mp.inertia * st.angularVelocityBody);
    };

    const double H0 = angularMomentumWorldMag(s);
    const double E0 = rotationalEnergy(s);

    const double dt = 1.0 / 480.0;
    for (int i = 0; i < 4800; ++i)            // 10 s of free tumbling
        s = integrate(s, mp, noForce, dt, IntegratorType::RK4);

    CHECK_NEAR(angularMomentumWorldMag(s), H0, 1e-4 * H0);
    CHECK_NEAR(rotationalEnergy(s),        E0, 1e-4 * E0);
    CHECK_NEAR(s.orientation.norm(),       1.0, 1e-9);
}

// --- 6. A body held at a banked attitude under gravity accelerates sideways:
//        confirms gravity is correctly resolved into the body frame. --------
TEST_CASE(gravity_resolves_into_body_frame) {
    // Pitch up 90 deg: body +X now points to world -Z (up). Gravity (world +Z)
    // should appear as -X in the body frame (pulling "backwards" along the nose).
    const MassProperties mp = MassProperties::make(1.0, Mat3::inertia(1, 1, 1));
    RigidBodyState s;
    s.orientation = Quat::fromEuler(0.0, radians(90.0), 0.0);

    const Vec3 weightWorld{0, 0, mp.mass * kStandardGravity};
    const Vec3 weightBody = s.orientation.rotateInverse(weightWorld);

    CHECK_NEAR(weightBody.x, -kStandardGravity, 1e-6);
    CHECK_NEAR(weightBody.y, 0.0, 1e-6);
    CHECK_NEAR(weightBody.z, 0.0, 1e-6);
}

FSIM_TEST_MAIN()
