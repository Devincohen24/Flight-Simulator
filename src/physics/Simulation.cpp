// =============================================================================
//  physics/Simulation.cpp -- fixed-timestep driver. See Simulation.hpp.
// =============================================================================
#include "physics/Simulation.hpp"

namespace fsim {

EnvironmentSample Simulation::environment() const {
    EnvironmentSample env;
    // NED: down is +Z, so geometric altitude above MSL is the negated Z.
    const double altitude = -state_.positionWorld.z;
    env.atmosphere = Atmosphere::sample(altitude);
    env.windWorld  = Vec3{0.0, 0.0, 0.0}; // no wind until Phase 5
    return env;
}

Wrench Simulation::currentWrench() const {
    return forces_.total(state_, mass_, environment());
}

void Simulation::step() {
    const EnvironmentSample env = environment();

    // The force functor closes over the (constant-within-step) environment.
    // Each RK4 stage re-evaluates the forces at its trial state.
    const ForceFunction forceFn = [this, &env](const RigidBodyState& s) {
        return forces_.total(s, mass_, env);
    };

    state_ = integrate(state_, mass_, forceFn, fixedDt_, integrator_);
    time_ += fixedDt_;
}

int Simulation::advance(double wallClockSeconds) {
    accumulator_ += wallClockSeconds;
    int steps = 0;
    while (accumulator_ >= fixedDt_) {
        step();
        accumulator_ -= fixedDt_;
        ++steps;
    }
    return steps;
}

} // namespace fsim
