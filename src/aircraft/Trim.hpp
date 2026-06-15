// =============================================================================
//  aircraft/Trim.hpp
//
//  Longitudinal trim solver. Finds the angle of attack, elevator deflection and
//  throttle that put the aircraft in steady, wings-level flight at a requested
//  airspeed and altitude (flight-path angle zero => level cruise).
//
//  Trim is the standard way to validate a flight-dynamics model: a correct aero
//  + propulsion model must admit an equilibrium where the net force and moment
//  vanish. We solve the 3-equation system
//        Fx_body = 0   (thrust balances drag)
//        Fz_body = 0   (lift balances weight)
//        My_body = 0   (pitching moment trimmed by the elevator)
//  for the three unknowns (alpha, elevator, throttle) with a damped
//  Newton iteration and a finite-difference Jacobian.
// =============================================================================
#pragma once

#include "aircraft/Aircraft.hpp"
#include "physics/RigidBody.hpp"
#include "core/Controls.hpp"

namespace fsim {

struct TrimResult {
    bool   converged{false};
    double residual{0.0};   // norm of the force/moment residual at the solution
    double alpha{0.0};      // trimmed angle of attack, rad
    RigidBodyState state{}; // level-flight state at the requested speed/altitude
    ControlInputs  controls{};
};

// Solve for level-flight trim at the given true airspeed (m/s) and altitude (m).
TrimResult trimLevelFlight(const Aircraft& ac, double airspeed, double altitude);

} // namespace fsim
