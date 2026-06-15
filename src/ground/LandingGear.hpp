// =============================================================================
//  ground/LandingGear.hpp
//
//  Landing-gear / ground-contact model. Each gear unit is a point connected to
//  the airframe through a spring-damper strut, with a tire that produces
//  longitudinal (rolling/braking) and lateral (cornering) friction. Taxi,
//  take-off roll, touchdown and ground loops all EMERGE from these forces fed
//  through the same rigid-body equations as everything else -- there is no
//  special "on ground" state machine.
//
//  For each gear contact point r (body frame):
//    world position    p = p_cg + R r
//    penetration       d = p_z + ground_elevation        (NED: down is +z)
//    contact velocity  v = R v_body + (R omega) x (R r)
//    strut normal force N = max(0, k d + c v_z)           (compression only)
//    normal force (world) acts upward: (0, 0, -N)
//
//  Tire friction in the ground plane, saturated with tanh to avoid numerical
//  chatter at low speed:
//    F_long = -(mu_roll + brake*mu_brake) N * tanh(v_long / v_eps)
//    F_lat  = -mu_side N * tanh(v_lat  / v_eps_lat)
//
//  The total per-unit force is rotated back into the body frame and applied at
//  r, contributing the moment r x F.
//
//  The ground is a flat plane at a fixed elevation for now; Milestone 3 will
//  replace ground_elevation with a terrain height query at p.
// =============================================================================
#pragma once

#include "aircraft/Aircraft.hpp"
#include "physics/ForceModel.hpp"

namespace fsim {

class LandingGearForce final : public ForceModel {
public:
    explicit LandingGearForce(std::vector<GearUnit> gear, double groundElevation = 0.0)
        : gear_(std::move(gear)), groundElevation_(groundElevation) {}

    void setGroundElevation(double e) { groundElevation_ = e; }

    Wrench evaluate(const RigidBodyState& state,
                    const MassProperties& mass,
                    const EnvironmentSample& env,
                    const ControlInputs& controls) const override;

    const char* name() const override { return "landing_gear"; }

    // True if any gear unit is currently in contact with the ground.
    bool anyContact(const RigidBodyState& state) const;

private:
    std::vector<GearUnit> gear_;
    double groundElevation_;
};

} // namespace fsim
