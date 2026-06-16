// =============================================================================
//  tests/test_performance.cpp -- profiler accumulation, physics determinism,
//  and a real-time-margin throughput benchmark for the flight-dynamics core.
// =============================================================================
#include "TestHarness.hpp"
#include "core/Profiler.hpp"
#include "aircraft/Aircraft.hpp"
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace fsim;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

static std::shared_ptr<Simulation> makeFlying(const Aircraft& ac, const TrimResult& trim) {
    auto sim = std::make_shared<Simulation>(1.0 / 240.0);
    sim->setMassProperties(ac.mass);
    sim->addForceModel(std::make_shared<AerodynamicsForce>(ac));
    sim->addForceModel(std::make_shared<PropulsionForce>(ac.propulsion));
    sim->addForceModel(std::make_shared<GravityForce>());
    sim->setState(trim.state);
    sim->setControls(trim.controls);
    return sim;
}

TEST_CASE(profiler_accumulates_stats) {
    Profiler prof;
    prof.add("x", 0.010);
    prof.add("x", 0.030);
    prof.add("x", 0.020);
    const Profiler::Stat s = prof.get("x");
    CHECK(s.count == 3);
    CHECK_NEAR(s.total, 0.060, 1e-12);
    CHECK_NEAR(s.average(), 0.020, 1e-12);
    CHECK_NEAR(s.minVal, 0.010, 1e-12);
    CHECK_NEAR(s.maxVal, 0.030, 1e-12);
}

TEST_CASE(physics_is_deterministic) {
    const Aircraft ac = loadAC();
    const TrimResult trim = trimLevelFlight(ac, 55.0, 1000.0);

    auto a = makeFlying(ac, trim);
    auto b = makeFlying(ac, trim);
    for (int i = 0; i < 5000; ++i) { a->step(); b->step(); }

    // Bit-identical evolution from identical inputs (no RNG in the core).
    CHECK_NEAR(a->state().positionWorld.x, b->state().positionWorld.x, 0.0);
    CHECK_NEAR(a->state().positionWorld.z, b->state().positionWorld.z, 0.0);
    CHECK_NEAR(a->state().velocityBody.x,  b->state().velocityBody.x,  0.0);
}

TEST_CASE(physics_runs_faster_than_realtime) {
    const Aircraft ac = loadAC();
    const TrimResult trim = trimLevelFlight(ac, 55.0, 1000.0);
    auto sim = makeFlying(ac, trim);

    const int steps = 240 * 60;            // 60 s of simulated time at 240 Hz
    const double simSeconds = steps * sim->fixedTimeStep();

    Profiler prof;
    const auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < steps; ++i) sim->step();
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double wall = std::chrono::duration<double>(t1 - t0).count();
    prof.add("physics_60s", wall);

    const double realtimeFactor = simSeconds / wall;
    std::printf("    [perf] 60 s sim in %.4f s wall  (%.0fx real-time, "
                "%.2f us/step)\n", wall, realtimeFactor, wall / steps * 1e6);

    // A single flight-dynamics model must run far faster than real time so the
    // budget is left for rendering and many aircraft. Require a healthy margin.
    CHECK(realtimeFactor > 10.0);
}

FSIM_TEST_MAIN()
