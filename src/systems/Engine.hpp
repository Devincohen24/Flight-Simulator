// =============================================================================
//  systems/Engine.hpp
//
//  Piston-engine systems model: RPM (with spool-up lag), shaft power, manifold
//  pressure, fuel flow and fuel quantity. This is the STATEFUL companion to the
//  (stateless) propulsion force model -- it is advanced once per physics step
//  and feeds the cockpit instruments (tachometer, fuel gauges, MP). When the
//  tanks run dry the engine quits, which the demo/aircraft can react to.
//
//  Power matches the propulsion model so thrust and the tach agree:
//        P = P_max (idle + (1-idle) throttle) (rho/rho0)
//  Fuel flow from brake specific fuel consumption:
//        mdot = BSFC * P            (kg/s)
// =============================================================================
#pragma once

#include "aircraft/Aircraft.hpp"

namespace fsim {

struct EngineState {
    double rpm{0.0};
    double power{0.0};            // W, shaft power delivered
    double manifoldPressure{0.0}; // inHg
    double fuelFlow{0.0};         // kg/s
    double fuelRemaining{0.0};    // kg
    bool   running{false};
};

class EngineSystem {
public:
    explicit EngineSystem(const PropulsionConfig& cfg) : cfg_(cfg) {
        state_.fuelRemaining = cfg.fuelCapacity;
    }

    void start() { state_.running = true; }
    void shutdown() { state_.running = false; }
    void setFuel(double kg) { state_.fuelRemaining = kg; }

    const EngineState& state() const { return state_; }
    double fuelFraction() const {
        return cfg_.fuelCapacity > 0.0 ? state_.fuelRemaining / cfg_.fuelCapacity : 0.0;
    }

    // Advance the engine by dt seconds. ambientPressure in Pa (for manifold
    // pressure), densityRatio = rho/rho0 (for the power altitude lapse).
    void update(double dt, double throttle, double densityRatio, double ambientPressure);

private:
    PropulsionConfig cfg_;
    EngineState state_;
};

} // namespace fsim
