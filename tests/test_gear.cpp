// =============================================================================
//  tests/test_gear.cpp -- landing-gear ground contact: static load balance and
//  dynamic settling. Confirms the aircraft rests on its gear in equilibrium and
//  does not fall through the ground -- all emerging from the strut spring-damper
//  forces fed through the rigid-body equations.
// =============================================================================
#include "TestHarness.hpp"
#include "aircraft/Aircraft.hpp"
#include "ground/LandingGear.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"
#include "core/Constants.hpp"

#include <memory>
#include <string>

using namespace fsim;
using constants::kStandardGravity;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

TEST_CASE(gear_static_load_balance) {
    const Aircraft ac = loadAC();
    LandingGearForce gear(ac.gear, /*groundElevation=*/0.0);

    // Total strut stiffness and the compression that balances weight.
    double K = 0.0;
    for (const auto& g : ac.gear) K += g.stiffness;
    const double W = ac.mass.mass * kStandardGravity;
    const double penEq = W / K;             // equal compression (gear all at z=1)

    // Place the aircraft level so every wheel is compressed by penEq.
    RigidBodyState s;
    s.positionWorld = Vec3{0.0, 0.0, -(1.0 - penEq)}; // gear body-z = 1.0

    EnvironmentSample env;
    const Wrench gearW = gear.evaluate(s, ac.mass, env, ControlInputs{});

    // Gear pushes up with the full weight; adding gravity gives ~zero net.
    const double netVertical = gearW.force.z + W; // body z = world z (level)
    CHECK_NEAR(netVertical, 0.0, 5.0);            // within 5 N
    CHECK(gearW.force.z < 0.0);                    // gear force is upward (-Z)
}

TEST_CASE(gear_settles_without_sinking_through) {
    const Aircraft ac = loadAC();

    Simulation sim(1.0 / 240.0);
    sim.setMassProperties(ac.mass);
    sim.addForceModel(std::make_shared<GravityForce>());
    sim.addForceModel(std::make_shared<LandingGearForce>(ac.gear, 0.0));

    // Start with the wheels just touching the ground (altitude 1.0 m, since the
    // gear sit 1.0 m below the CG), at rest.
    RigidBodyState s;
    s.positionWorld = Vec3{0.0, 0.0, -1.0};
    sim.setState(s);

    for (int i = 0; i < 8 * 240; ++i) sim.step(); // 8 s to settle

    const double altitude = -sim.state().positionWorld.z;
    const double vDown     = sim.state().velocityBody.z;
    const EulerAngles e    = toEuler(sim.state().orientation);

    CHECK(altitude > 0.5);                 // did NOT fall through the ground
    CHECK(altitude < 1.0);                 // settled onto compressed struts
    CHECK(std::fabs(vDown) < 0.5);         // came to rest vertically
    CHECK(std::fabs(e.pitch) < radians(8.0));
    CHECK(std::fabs(e.roll)  < radians(2.0));
}

FSIM_TEST_MAIN()
