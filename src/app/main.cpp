// =============================================================================
//  app/main.cpp -- headless flight demonstration.
//
//  Loads the data-driven Cessna 172, trims it for level cruise, then flies it
//  with the full Phase 2 model (aerodynamics + propulsion + gravity). It first
//  flies hands-off to show the trimmed equilibrium holds, then applies a brief
//  nose-up elevator pulse so the pitch/altitude response can be seen to EMERGE
//  from the aerodynamics -- no motion is scripted.
//
//  Usage:  fsim_headless [path/to/aircraft.json]
// =============================================================================
#include "aircraft/Aircraft.hpp"
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"

#include <cstdio>
#include <memory>
#include <string>

using namespace fsim;

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1]
                                        : "data/aircraft/cessna172.json";
    Aircraft ac;
    try {
        ac = Aircraft::load(path);
    } catch (const std::exception& e) {
        std::printf("Failed to load aircraft '%s': %s\n", path.c_str(), e.what());
        std::printf("(run from the repo root, or pass the data-file path)\n");
        return 1;
    }

    const double cruise = 55.0;   // m/s true airspeed (~107 kt)
    const double alt0   = 1000.0; // m

    const TrimResult trim = trimLevelFlight(ac, cruise, alt0);
    std::printf("Aircraft: %s\n", ac.name.c_str());
    std::printf("Trim @ %.0f m/s, %.0f m:  converged=%s  residual=%.2e\n",
                cruise, alt0, trim.converged ? "yes" : "no", trim.residual);
    std::printf("  alpha=%.2f deg   elevator=%.3f   throttle=%.3f\n\n",
                degrees(trim.alpha), trim.controls.elevator, trim.controls.throttle);

    Simulation sim(1.0 / 240.0);
    sim.setMassProperties(ac.mass);
    sim.addForceModel(std::make_shared<AerodynamicsForce>(ac));
    sim.addForceModel(std::make_shared<PropulsionForce>(ac.propulsion));
    sim.addForceModel(std::make_shared<GravityForce>());
    sim.setState(trim.state);
    sim.setControls(trim.controls);

    std::printf("# %6s %10s %10s %9s %9s %9s\n",
                "t[s]", "alt[m]", "TAS[m/s]", "pitch[d]", "AoA[d]", "VS[m/s]");

    auto report = [&](double t) {
        const auto& s = sim.state();
        const EulerAngles e = toEuler(s.orientation);
        const EnvironmentSample env = sim.environment();
        const AeroState a = computeAerodynamics(ac, s, env, sim.controls());
        const double vs = -s.orientation.rotate(s.velocityBody).z; // climb rate
        std::printf("  %6.1f %10.2f %10.2f %9.2f %9.2f %9.2f\n",
                    t, -s.positionWorld.z, s.velocityBody.norm(),
                    degrees(e.pitch), degrees(a.alpha), vs);
    };

    // Phase A: hands-off for 6 s (should hold trim).
    for (int sec = 0; sec <= 6; ++sec) {
        report(sim.time());
        for (int i = 0; i < 240; ++i) sim.step();
    }

    // Phase B: 2-second nose-up elevator pulse (negative elevator = TE up).
    std::printf("# --- nose-up elevator pulse (2 s) ---\n");
    ControlInputs pulse = trim.controls;
    pulse.elevator = trim.controls.elevator - 0.25;
    sim.setControls(pulse);
    for (int i = 0; i < 2 * 240; ++i) {
        if (i % 240 == 0) report(sim.time());
        sim.step();
    }

    // Phase C: return to trim controls and observe the response settle.
    std::printf("# --- controls returned to trim ---\n");
    sim.setControls(trim.controls);
    for (int sec = 0; sec < 10; ++sec) {
        report(sim.time());
        for (int i = 0; i < 240; ++i) sim.step();
    }

    return 0;
}
