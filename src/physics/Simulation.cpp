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
    env.windWorld  = windWorld_; // set by the weather system (Phase 5)
    return env;
}

Wrench Simulation::currentWrench() const {
    return forces_.total(state_, mass_, environment(), controls_);
}

Vec3 Simulation::specificForceBody() const {
    const Wrench w = currentWrench();
    // Subtract the weight (gravity contributes m*g_body to the total force).
    const Vec3 gravityBody = state_.orientation.rotateInverse(
        Vec3{0.0, 0.0, mass_.mass * constants::kStandardGravity});
    return (w.force - gravityBody) * (1.0 / mass_.mass);
}

void Simulation::step() {
    const EnvironmentSample env = environment();

    // The force functor closes over the (constant-within-step) environment.
    // Each RK4 stage re-evaluates the forces at its trial state.
    const ForceFunction forceFn = [this, &env](const RigidBodyState& s) {
        return forces_.total(s, mass_, env, controls_);
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
