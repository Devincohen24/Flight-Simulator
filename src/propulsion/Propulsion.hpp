// =============================================================================
//  propulsion/Propulsion.hpp
//
//  Piston-engine + fixed-pitch propeller thrust model.
//
//  Available shaft power lapses with air density and scales with throttle:
//        P = P_max * (idle + (1 - idle) * throttle) * (rho / rho0)
//
//  Two thrust estimates are combined so the model is well-behaved across the
//  whole envelope:
//    * Dynamic (energy) thrust:   T_dyn = eta_prop * P / V_forward
//      (valid in forward flight; -> infinity as V -> 0, hence the cap below)
//    * Static thrust (momentum theory for an actuator disk of area A):
//        T_static = (2 rho A P^2)^(1/3)
//  The delivered thrust is min(T_dyn, T_static): static thrust limits the
//  standing/low-speed case, while T_dyn governs cruise where it is the smaller.
//
//  Thrust acts along the body-frame thrust axis applied at the thrust point,
//  contributing a moment r x F about the CG when the thrust line is offset.
// =============================================================================
#pragma once

#include "aircraft/Aircraft.hpp"
#include "physics/ForceModel.hpp"

namespace fsim {

struct ThrustState {
    double power{0.0};   // delivered shaft power, W
    double thrust{0.0};  // N, magnitude along the thrust axis
    Wrench wrench{};
};

ThrustState computeThrust(const PropulsionConfig& prop,
                          const RigidBodyState& state,
                          const EnvironmentSample& env,
                          const ControlInputs& controls);

class PropulsionForce final : public ForceModel {
public:
    explicit PropulsionForce(const PropulsionConfig& prop) : prop_(prop) {}

    Wrench evaluate(const RigidBodyState& state,
                    const MassProperties& /*mass*/,
                    const EnvironmentSample& env,
                    const ControlInputs& controls) const override {
        return computeThrust(prop_, state, env, controls).wrench;
    }
    const char* name() const override { return "propulsion"; }

private:
    PropulsionConfig prop_;
};

} // namespace fsim
