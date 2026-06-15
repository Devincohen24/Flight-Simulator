// =============================================================================
//  tests/test_trim.cpp
//
//  Validates that the aero + propulsion model admits a steady level-flight
//  equilibrium (trim), and that integrating forward FROM that trimmed state
//  keeps the aircraft flying steadily -- i.e. the equilibrium is real and the
//  dynamics around it are well-behaved. This is the headline check that the
//  Phase 2 flight model is self-consistent.
// =============================================================================
#include "TestHarness.hpp"
#include "aircraft/Aircraft.hpp"
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"

#include <memory>
#include <string>

using namespace fsim;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

TEST_CASE(trim_converges_at_cruise) {
    const Aircraft ac = loadAC();
    const TrimResult t = trimLevelFlight(ac, 55.0, 1000.0);

    CHECK(t.converged);
    CHECK(t.residual < 1e-3);                 // net force/moment ~ 0
    CHECK(t.alpha > 0.0 && t.alpha < radians(10.0)); // sensible cruise AoA
    CHECK(t.controls.throttle > 0.1 && t.controls.throttle < 1.0);
}

TEST_CASE(trim_holds_steady_flight) {
    const Aircraft ac = loadAC();
    const TrimResult t = trimLevelFlight(ac, 55.0, 1000.0);
    CHECK(t.converged);

    Simulation sim(1.0 / 240.0);
    sim.setMassProperties(ac.mass);
    sim.addForceModel(std::make_shared<AerodynamicsForce>(ac));
    sim.addForceModel(std::make_shared<PropulsionForce>(ac.propulsion));
    sim.addForceModel(std::make_shared<GravityForce>());
    sim.setState(t.state);
    sim.setControls(t.controls);

    const double alt0   = -t.state.positionWorld.z;
    const double speed0  = t.state.velocityBody.norm();

    // Fly for 20 seconds hands-off.
    for (int i = 0; i < 20 * 240; ++i) sim.step();

    const double alt   = -sim.state().positionWorld.z;
    const double speed = sim.state().velocityBody.norm();

    // Lightly-damped phugoid means small excursions are expected, not zero.
    CHECK_NEAR(alt,   alt0,   40.0);  // altitude held within 40 m
    CHECK_NEAR(speed, speed0,  5.0);  // airspeed held within 5 m/s
    // Must not have departed controlled flight.
    const EulerAngles e = toEuler(sim.state().orientation);
    CHECK(std::fabs(e.pitch) < radians(20.0));
    CHECK(std::fabs(e.roll)  < radians(5.0));
}

FSIM_TEST_MAIN()
