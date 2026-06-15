// =============================================================================
//  tests/test_atmosphere.cpp -- ISA model against published reference values.
// =============================================================================
#include "TestHarness.hpp"
#include "physics/Atmosphere.hpp"

using namespace fsim;

TEST_CASE(isa_sea_level) {
    const AtmosphereSample s = Atmosphere::sample(0.0);
    CHECK_NEAR(s.temperature, 288.15,   1e-6);
    CHECK_NEAR(s.pressure,    101325.0, 1e-3);
    CHECK_NEAR(s.density,     1.225,    1e-3);
    CHECK_NEAR(s.speedOfSound, 340.29,  0.1);   // ~340.3 m/s
}

TEST_CASE(isa_11km_tropopause) {
    // Reference ISA values at 11 km: T=216.65 K, p~22632 Pa, rho~0.3639 kg/m^3.
    const AtmosphereSample s = Atmosphere::sample(11000.0);
    CHECK_NEAR(s.temperature, 216.65, 1e-2);
    CHECK_NEAR(s.pressure,    22632.0, 50.0);
    CHECK_NEAR(s.density,     0.3639,  1e-3);
}

TEST_CASE(isa_5km) {
    // Reference ISA at 5 km: T=255.65 K, p~54019 Pa, rho~0.7361 kg/m^3.
    const AtmosphereSample s = Atmosphere::sample(5000.0);
    CHECK_NEAR(s.temperature, 255.65,  1e-2);
    CHECK_NEAR(s.pressure,    54019.0, 100.0);
    CHECK_NEAR(s.density,     0.7361,  2e-3);
}

TEST_CASE(isa_stratosphere_isothermal) {
    // Above the tropopause temperature is constant (isothermal layer).
    const AtmosphereSample a = Atmosphere::sample(12000.0);
    const AtmosphereSample b = Atmosphere::sample(15000.0);
    CHECK_NEAR(a.temperature, 216.65, 1e-2);
    CHECK_NEAR(b.temperature, 216.65, 1e-2);
    // Pressure must still decrease with altitude.
    CHECK(b.pressure < a.pressure);
}

TEST_CASE(isa_negative_altitude_clamped) {
    const AtmosphereSample s = Atmosphere::sample(-500.0);
    CHECK_NEAR(s.temperature, 288.15, 1e-6); // clamped to sea level
}

FSIM_TEST_MAIN()
