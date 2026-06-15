// =============================================================================
//  systems/Engine.cpp -- see Engine.hpp.
// =============================================================================
#include "systems/Engine.hpp"

#include <algorithm>
#include <cmath>

namespace fsim {

namespace {
constexpr double kPaToInHg = 1.0 / 3386.389; // pascals -> inches of mercury
} // namespace

void EngineSystem::update(double dt, double throttle, double densityRatio,
                          double ambientPressure) {
    throttle = std::clamp(throttle, 0.0, 1.0);

    // Out of fuel -> the engine quits.
    if (state_.fuelRemaining <= 0.0) {
        state_.fuelRemaining = 0.0;
        state_.running = false;
    }

    // Target RPM: idle..max with throttle when running, else windmilling to 0.
    const double targetRpm = state_.running
        ? cfg_.idleRpm + throttle * (cfg_.maxRpm - cfg_.idleRpm)
        : 0.0;

    // First-order spool response toward the target.
    const double a = (cfg_.spoolTau > 1e-6) ? (1.0 - std::exp(-dt / cfg_.spoolTau)) : 1.0;
    state_.rpm += (targetRpm - state_.rpm) * a;

    // Shaft power (matches the propulsion model).
    const double powerFraction = state_.running
        ? cfg_.idlePowerFraction + (1.0 - cfg_.idlePowerFraction) * throttle
        : 0.0;
    state_.power = cfg_.maxPower * powerFraction * densityRatio;

    // Manifold pressure: ambient at idle, rising with throttle (a simple
    // throttle-plate model). Reported in inches of mercury.
    state_.manifoldPressure = ambientPressure * (0.35 + 0.65 * throttle) * kPaToInHg;

    // Fuel burn; the engine quits the instant the tank runs dry.
    state_.fuelFlow = cfg_.bsfc * state_.power;
    state_.fuelRemaining = std::max(0.0, state_.fuelRemaining - state_.fuelFlow * dt);
    if (state_.fuelRemaining <= 0.0) {
        state_.fuelRemaining = 0.0;
        state_.running = false;
    }
}

} // namespace fsim
