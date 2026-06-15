// =============================================================================
//  physics/Simulation.hpp
//
//  Top-level driver for the flight-dynamics model. Owns the aircraft state,
//  mass properties and the set of force models, and advances time with a FIXED
//  physics timestep using an accumulator. A fixed step is essential for
//  determinism and numerical stability: the renderer (Milestone 3) will run at
//  a variable frame rate and interpolate between physics states, but the
//  physics itself always steps by the same dt.
// =============================================================================
#pragma once

#include "physics/RigidBody.hpp"
#include "physics/ForceModel.hpp"
#include "physics/Integrator.hpp"
#include "core/Controls.hpp"

namespace fsim {

class Simulation {
public:
    // Default physics step: 240 Hz. Small enough to resolve control-surface
    // dynamics and ground contact accurately, cheap enough to run real-time.
    explicit Simulation(double fixedTimeStep = 1.0 / 240.0)
        : fixedDt_(fixedTimeStep) {}

    // --- Configuration ---
    void setState(const RigidBodyState& s)   { state_ = s; }
    void setMassProperties(const MassProperties& mp) { mass_ = mp; }
    void addForceModel(std::shared_ptr<ForceModel> m) { forces_.add(std::move(m)); }
    void setIntegrator(IntegratorType t)      { integrator_ = t; }
    void setControls(const ControlInputs& c) { controls_ = c; controls_.clamp(); }

    // --- Accessors ---
    const RigidBodyState& state() const { return state_; }
    const MassProperties& mass()  const { return mass_; }
    const ControlInputs&  controls() const { return controls_; }
    double time() const { return time_; }
    double fixedTimeStep() const { return fixedDt_; }

    // Sample the current environment at the aircraft's altitude. Altitude is
    // -Z in NED (down is positive), so altitude above sea level = -positionZ.
    EnvironmentSample environment() const;

    // Total force/moment currently acting on the aircraft (for HUD / debugging).
    Wrench currentWrench() const;

    // Non-gravitational acceleration in the body frame -- what an accelerometer
    // (and hence the load-factor/slip instruments) senses: (F_total - mg)/m.
    Vec3 specificForceBody() const;

    // Advance exactly one fixed physics step.
    void step();

    // Advance by an arbitrary wall-clock duration, consuming whole fixed steps
    // via an accumulator. Returns the number of physics steps executed.
    int advance(double wallClockSeconds);

private:
    double          fixedDt_;
    double          time_{0.0};
    double          accumulator_{0.0};
    RigidBodyState  state_{};
    MassProperties  mass_{};
    ControlInputs   controls_{};
    CompositeForceModel forces_{};
    IntegratorType  integrator_{IntegratorType::RK4};
};

} // namespace fsim
