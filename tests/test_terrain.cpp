// =============================================================================
//  tests/test_terrain.cpp -- height field correctness and the terrain/gear
//  integration (the aircraft touches down on exactly the queried ground).
// =============================================================================
#include "TestHarness.hpp"
#include "terrain/Terrain.hpp"
#include "aircraft/Aircraft.hpp"
#include "ground/LandingGear.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"

#include <memory>
#include <string>

using namespace fsim;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

TEST_CASE(flat_ground_height_and_normal) {
    FlatGround g(123.0);
    CHECK_NEAR(g.height(1000.0, -2000.0), 123.0, 1e-12);
    const Vec3 n = g.normalNed(0.0, 0.0);
    CHECK_NEAR(n.x, 0.0, 1e-9);
    CHECK_NEAR(n.y, 0.0, 1e-9);
    CHECK_NEAR(n.z, -1.0, 1e-9); // up in NED
}

TEST_CASE(procedural_terrain_deterministic_and_airfield_flat) {
    ProceduralTerrain t;
    // Deterministic: same query, same answer.
    CHECK_NEAR(t.height(3210.0, -870.0), t.height(3210.0, -870.0), 0.0);
    // Flat airfield around the origin (within the flat radius).
    CHECK_NEAR(t.height(0.0, 0.0), 0.0, 1e-9);
    CHECK_NEAR(t.height(500.0, 200.0), 0.0, 1e-6);
    // Rolling hills far away differ from the base elevation.
    CHECK(std::fabs(t.height(6000.0, 4000.0)) > 1.0);
    // Normal over flat airfield points up.
    const Vec3 n = t.normalNed(0.0, 0.0);
    CHECK_NEAR(n.z, -1.0, 1e-6);
}

TEST_CASE(gear_contact_uses_terrain_height) {
    const Aircraft ac = loadAC();
    FlatGround g(200.0);
    LandingGearForce gear(ac.gear, &g);

    RigidBodyState above;             // CG 5 m above the wheels' rest height
    above.positionWorld = Vec3{0, 0, -(200.0 + 6.0)};
    CHECK(!gear.anyContact(above));   // wheels are above the 200 m ground

    RigidBodyState below;
    below.positionWorld = Vec3{0, 0, -(200.0 + 0.5)}; // wheels pushed into ground
    CHECK(gear.anyContact(below));
}

TEST_CASE(aircraft_settles_on_elevated_ground) {
    const Aircraft ac = loadAC();
    auto terrain = std::make_shared<FlatGround>(200.0);

    Simulation sim(1.0 / 240.0);
    sim.setMassProperties(ac.mass);
    sim.addForceModel(std::make_shared<GravityForce>());
    sim.addForceModel(std::make_shared<LandingGearForce>(ac.gear, terrain.get()));

    // Start with the wheels just touching ground at 200 m elevation.
    RigidBodyState s;
    s.positionWorld = Vec3{0, 0, -(200.0 + 1.0)};
    sim.setState(s);

    for (int i = 0; i < 8 * 240; ++i) sim.step();

    const double altitude = -sim.state().positionWorld.z;
    CHECK(altitude > 200.5);            // rests above the 200 m ground
    CHECK(altitude < 201.0);            // struts compressed, not floating
    CHECK(std::fabs(sim.state().velocityBody.z) < 0.5);
}

FSIM_TEST_MAIN()
