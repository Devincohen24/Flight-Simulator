// =============================================================================
//  tests/test_aircraft.cpp -- loading the data-driven aircraft definition.
// =============================================================================
#include "TestHarness.hpp"
#include "aircraft/Aircraft.hpp"

#include <string>

using namespace fsim;

static std::string dataPath() {
    return std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json";
}

TEST_CASE(load_cessna172) {
    const Aircraft ac = Aircraft::load(dataPath());
    CHECK(ac.name == "Cessna 172 Skyhawk");

    // Mass & inertia.
    CHECK_NEAR(ac.mass.mass, 1043.0, 1e-9);
    CHECK_NEAR(ac.mass.inertia.m[1][1], 1825.0, 1e-9); // Iyy
    // Inverse inertia was computed at load time.
    CHECK_NEAR(ac.mass.inertiaInverse.m[1][1], 1.0 / 1825.0, 1e-12);

    // Geometry & a couple of derivatives.
    CHECK_NEAR(ac.wing.area, 16.2, 1e-9);
    CHECK_NEAR(ac.wing.aspectRatio(), 11.0 * 11.0 / 16.2, 1e-9);
    CHECK_NEAR(ac.aero.CLalpha, 5.143, 1e-9);
    CHECK_NEAR(ac.aero.Cmde, -1.28, 1e-9);

    // Propulsion & gear.
    CHECK_NEAR(ac.propulsion.maxPower, 119000.0, 1e-6);
    CHECK(ac.gear.size() == 3);
    CHECK(ac.gear[0].name == "nose");
    CHECK(ac.gear[1].braked == true);
}

FSIM_TEST_MAIN()
