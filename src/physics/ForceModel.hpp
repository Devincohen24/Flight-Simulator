// =============================================================================
//  physics/ForceModel.hpp
//
//  Abstraction for anything that produces a force/moment on the aircraft. This
//  is the primary extensibility seam of the simulator: Phase 2 adds AeroModel,
//  Propulsion and LandingGear simply by implementing this interface and adding
//  them to the CompositeForceModel. The integrator and rigid body never change.
//
//  A force model is given the current state and environment and returns a
//  Wrench expressed in the BODY frame (forces applied off the CG are converted
//  to an equivalent force+moment by the model itself).
// =============================================================================
#pragma once

#include "physics/RigidBody.hpp"
#include "physics/Atmosphere.hpp"
#include "core/Constants.hpp"

#include <memory>
#include <vector>

namespace fsim {

// Snapshot of environmental conditions passed to every force model. Phase 5
// (wind, turbulence) will extend this with a wind vector queried per position.
struct EnvironmentSample {
    AtmosphereSample atmosphere{};
    Vec3 windWorld{};   // NED wind velocity, m/s (zero until Phase 5)
};

class ForceModel {
public:
    virtual ~ForceModel() = default;

    // Compute this model's contribution to the total wrench (body frame).
    virtual Wrench evaluate(const RigidBodyState& state,
                            const MassProperties& mass,
                            const EnvironmentSample& env) const = 0;

    // Human-readable label, useful for force breakdown / debugging.
    virtual const char* name() const = 0;
};

// -----------------------------------------------------------------------------
// Gravity: a body-frame force of magnitude m*g pointing "down" in the world,
// rotated into the body frame. Produces no moment about the CG.
// -----------------------------------------------------------------------------
class GravityForce final : public ForceModel {
public:
    Wrench evaluate(const RigidBodyState& state,
                    const MassProperties& mass,
                    const EnvironmentSample& /*env*/) const override {
        // Weight in the world frame points along +Z (down in NED).
        const Vec3 weightWorld{0.0, 0.0, mass.mass * constants::kStandardGravity};
        Wrench w;
        w.force  = state.orientation.rotateInverse(weightWorld); // world -> body
        w.moment = Vec3{0.0, 0.0, 0.0};
        return w;
    }
    const char* name() const override { return "gravity"; }
};

// -----------------------------------------------------------------------------
// CompositeForceModel: sums the contributions of all registered sub-models.
// This is what the Simulation hands to the integrator.
// -----------------------------------------------------------------------------
class CompositeForceModel {
public:
    void add(std::shared_ptr<ForceModel> model) { models_.push_back(std::move(model)); }

    Wrench total(const RigidBodyState& state,
                 const MassProperties& mass,
                 const EnvironmentSample& env) const {
        Wrench sum;
        for (const auto& m : models_) sum += m->evaluate(state, mass, env);
        return sum;
    }

    const std::vector<std::shared_ptr<ForceModel>>& models() const { return models_; }

private:
    std::vector<std::shared_ptr<ForceModel>> models_;
};

} // namespace fsim
