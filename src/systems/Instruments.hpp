// =============================================================================
//  systems/Instruments.hpp
//
//  Flight-instrument computations. Every reading is DERIVED from the simulated
//  state, atmosphere and engine -- there are no independent instrument dynamics
//  to drift out of sync with the physics. This is the data a six-pack + engine
//  cluster would display:
//
//    Airspeed indicator   - indicated (IAS) and true (TAS) airspeed, Mach
//    Altimeter            - pressure altitude with a Kollsman (baro) setting
//    Vertical speed       - rate of climb/descent
//    Attitude indicator   - pitch and bank from the attitude quaternion
//    Heading indicator    - magnetic-ish heading (true heading here)
//    Turn coordinator     - turn rate and slip/skid (the "ball")
//    Accelerometer        - load factor (g)
//    Tachometer / fuel    - from the engine system
//
//  Formulae:
//    IAS = TAS * sqrt(rho / rho0)                 (incompressible, low speed)
//    Mach = TAS / a
//    pressure altitude: invert the ISA troposphere from static pressure p with
//        sea-level reference = baro setting:
//        h = (T0/L) (1 - (p / p_baro)^(R L / g))
//    turn rate (yaw of the velocity vector):
//        psi_dot = (q sin(phi) + r cos(phi)) / cos(theta)
//    load factor: n = -f_z / g     (f = body specific force from accelerometers)
// =============================================================================
#pragma once

#include "physics/RigidBody.hpp"
#include "physics/ForceModel.hpp"   // EnvironmentSample
#include "systems/Engine.hpp"

namespace fsim {

struct InstrumentReadings {
    double indicatedAirspeed{0.0}; // m/s
    double trueAirspeed{0.0};      // m/s
    double mach{0.0};
    double pressureAltitude{0.0};  // m (with baro setting)
    double altitudeMSL{0.0};       // m (geometric)
    double verticalSpeed{0.0};     // m/s (+ = climb)
    double groundSpeed{0.0};       // m/s
    double pitch{0.0};             // deg
    double bank{0.0};              // deg (roll)
    double heading{0.0};           // deg [0,360)
    double turnRate{0.0};          // deg/s (+ = right)
    double slip{0.0};              // lateral g (ball); 0 = coordinated
    double loadFactor{1.0};        // g
    double angleOfAttack{0.0};     // deg
    double sideslip{0.0};          // deg
    // Engine cluster.
    double rpm{0.0};
    double manifoldPressure{0.0};  // inHg
    double fuelRemaining{0.0};     // kg
    double fuelFlow{0.0};          // kg/s
};

// Standard sea-level barometric pressure used as the default Kollsman setting.
constexpr double kStandardBaro = 101325.0; // Pa (29.92 inHg)

// Compute the full instrument panel. 'specificForceBody' is the non-gravitational
// acceleration in the body frame (what accelerometers sense); the Simulation can
// provide it via Simulation::specificForceBody().
InstrumentReadings computeInstruments(const RigidBodyState& state,
                                      const EnvironmentSample& env,
                                      const EngineState& engine,
                                      const Vec3& specificForceBody,
                                      double baroSetting = kStandardBaro);

} // namespace fsim
