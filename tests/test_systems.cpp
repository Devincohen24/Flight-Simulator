// =============================================================================
//  tests/test_systems.cpp -- engine systems and flight-instrument computations.
// =============================================================================
#include "TestHarness.hpp"
#include "aircraft/Aircraft.hpp"
#include "systems/Engine.hpp"
#include "systems/Instruments.hpp"
#include "physics/Atmosphere.hpp"
#include "core/Constants.hpp"

#include <string>

using namespace fsim;
using constants::kStandardGravity;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

static EnvironmentSample envAt(double altitude) {
    EnvironmentSample e;
    e.atmosphere = Atmosphere::sample(altitude);
    return e;
}

TEST_CASE(engine_spools_up_and_burns_fuel) {
    const Aircraft ac = loadAC();
    EngineSystem eng(ac.propulsion);
    eng.start();
    const double fuel0 = eng.state().fuelRemaining;

    for (int i = 0; i < 50; ++i) // 5 s at full throttle
        eng.update(0.1, /*throttle=*/1.0, /*densityRatio=*/1.0, /*ambient=*/101325.0);

    CHECK_NEAR(eng.state().rpm, ac.propulsion.maxRpm, 30.0); // near max RPM
    CHECK(eng.state().fuelFlow > 0.0);
    CHECK(eng.state().fuelRemaining < fuel0);               // fuel consumed
    CHECK(eng.state().manifoldPressure > 0.0);
}

TEST_CASE(engine_quits_when_out_of_fuel) {
    const Aircraft ac = loadAC();
    EngineSystem eng(ac.propulsion);
    eng.start();
    eng.setFuel(0.002); // grams of fuel

    eng.update(0.1, 1.0, 1.0, 101325.0); // burns the last fuel
    eng.update(0.1, 1.0, 1.0, 101325.0); // now dry
    CHECK(!eng.state().running);
    CHECK_NEAR(eng.state().fuelRemaining, 0.0, 1e-9);
}

TEST_CASE(airspeed_ias_vs_tas_with_altitude) {
    EngineState eng;
    RigidBodyState s;
    s.velocityBody = Vec3{60.0, 0.0, 0.0};

    // At sea level IAS == TAS.
    const InstrumentReadings sl = computeInstruments(s, envAt(0.0), eng,
                                                     Vec3{0, 0, -kStandardGravity});
    CHECK_NEAR(sl.trueAirspeed, 60.0, 1e-6);
    CHECK_NEAR(sl.indicatedAirspeed, 60.0, 0.1);

    // At altitude the thinner air makes IAS read lower than TAS.
    const InstrumentReadings hi = computeInstruments(s, envAt(3000.0), eng,
                                                     Vec3{0, 0, -kStandardGravity});
    CHECK(hi.indicatedAirspeed < hi.trueAirspeed - 5.0);
}

TEST_CASE(altimeter_and_vertical_speed) {
    EngineState eng;
    RigidBodyState s;
    s.positionWorld = Vec3{0, 0, -2000.0};        // 2000 m up
    s.velocityBody  = Vec3{50.0, 0.0, -5.0};      // climbing (body -z)

    const InstrumentReadings r = computeInstruments(s, envAt(2000.0), eng,
                                                    Vec3{0, 0, -kStandardGravity});
    CHECK_NEAR(r.altitudeMSL, 2000.0, 1e-6);
    CHECK_NEAR(r.pressureAltitude, 2000.0, 5.0); // matches geometric at std baro
    CHECK(r.verticalSpeed > 0.0);                // climbing
}

TEST_CASE(attitude_and_load_factor) {
    EngineState eng;
    RigidBodyState s;
    s.orientation = Quat::fromEuler(radians(20.0), radians(8.0), radians(90.0));
    s.velocityBody = Vec3{55.0, 0.0, 0.0};

    // Level 1-g: specific force straight "up" in the body frame.
    const InstrumentReadings r = computeInstruments(s, envAt(0.0), eng,
                                                    Vec3{0, 0, -kStandardGravity});
    CHECK_NEAR(r.bank, 20.0, 1e-6);
    CHECK_NEAR(r.pitch, 8.0, 1e-6);
    CHECK_NEAR(r.heading, 90.0, 1e-6);
    CHECK_NEAR(r.loadFactor, 1.0, 1e-9);
    CHECK_NEAR(r.slip, 0.0, 1e-9);

    // A 2-g pull shows on the accelerometer.
    const InstrumentReadings r2 = computeInstruments(s, envAt(0.0), eng,
                                                     Vec3{0, 0, -2.0 * kStandardGravity});
    CHECK_NEAR(r2.loadFactor, 2.0, 1e-9);
}

FSIM_TEST_MAIN()
