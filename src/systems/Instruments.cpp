// =============================================================================
//  systems/Instruments.cpp -- see Instruments.hpp.
// =============================================================================
#include "systems/Instruments.hpp"
#include "core/Constants.hpp"

#include <cmath>

namespace fsim {

using namespace constants;

InstrumentReadings computeInstruments(const RigidBodyState& state,
                                      const EnvironmentSample& env,
                                      const EngineState& engine,
                                      const Vec3& specificForceBody,
                                      double baroSetting) {
    InstrumentReadings r;

    // --- Air data ---
    const Vec3 windBody = state.orientation.rotateInverse(env.windWorld);
    const Vec3 vAir = state.velocityBody - windBody;
    const double tas = vAir.norm();
    r.trueAirspeed = tas;

    const double densityRatio = env.atmosphere.density / kSeaLevelDensity;
    r.indicatedAirspeed = tas * std::sqrt(std::max(densityRatio, 0.0));
    r.mach = (env.atmosphere.speedOfSound > 0.0) ? tas / env.atmosphere.speedOfSound : 0.0;

    if (tas > 0.5) {
        r.angleOfAttack = degrees(std::atan2(vAir.z, vAir.x));
        double sb = vAir.y / tas;
        sb = sb < -1.0 ? -1.0 : (sb > 1.0 ? 1.0 : sb);
        r.sideslip = degrees(std::asin(sb));
    }

    // --- Altitude ---
    r.altitudeMSL = -state.positionWorld.z;
    // Pressure altitude: invert the ISA troposphere from static pressure.
    const double p = env.atmosphere.pressure;
    const double exponent = (kSpecificGasConstantAir * kTroposphereLapseRate)
                          / kStandardGravity;
    r.pressureAltitude = (kSeaLevelTemperature / kTroposphereLapseRate)
                       * (1.0 - std::pow(p / baroSetting, exponent));

    // --- Velocity-derived ---
    const Vec3 vWorld = state.orientation.rotate(state.velocityBody);
    r.verticalSpeed = -vWorld.z; // up is -Z
    r.groundSpeed   = std::sqrt(vWorld.x * vWorld.x + vWorld.y * vWorld.y);

    // --- Attitude ---
    const EulerAngles e = toEuler(state.orientation);
    r.pitch = degrees(e.pitch);
    r.bank  = degrees(e.roll);
    double hdg = degrees(e.yaw);
    if (hdg < 0.0) hdg += 360.0;
    r.heading = hdg;

    // --- Turn coordinator: yaw rate of the velocity vector, and slip ball ---
    const double phi = e.roll, theta = e.pitch;
    const double q = state.angularVelocityBody.y, rr = state.angularVelocityBody.z;
    const double cosTheta = std::cos(theta);
    if (std::fabs(cosTheta) > 1e-4)
        r.turnRate = degrees((q * std::sin(phi) + rr * std::cos(phi)) / cosTheta);

    // --- Accelerometer-derived ---
    r.loadFactor = -specificForceBody.z / kStandardGravity;
    r.slip       =  specificForceBody.y / kStandardGravity; // 0 when coordinated

    // --- Engine cluster ---
    r.rpm              = engine.rpm;
    r.manifoldPressure = engine.manifoldPressure;
    r.fuelRemaining    = engine.fuelRemaining;
    r.fuelFlow         = engine.fuelFlow;

    return r;
}

} // namespace fsim
