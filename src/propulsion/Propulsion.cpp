// =============================================================================
//  propulsion/Propulsion.cpp -- see Propulsion.hpp for the model description.
// =============================================================================
#include "propulsion/Propulsion.hpp"
#include "core/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace fsim {

ThrustState computeThrust(const PropulsionConfig& prop,
                          const RigidBodyState& state,
                          const EnvironmentSample& env,
                          const ControlInputs& controls) {
    ThrustState out;

    const Vec3 axis = prop.thrustAxis.normalized();

    // Density lapse of available power.
    const double densityRatio = env.atmosphere.density / constants::kSeaLevelDensity;
    const double powerFraction = prop.idlePowerFraction
                               + (1.0 - prop.idlePowerFraction) * controls.throttle;
    const double power = prop.maxPower * powerFraction * densityRatio;
    out.power = power;

    // Forward air-relative speed along the thrust axis.
    const Vec3 windBody = state.orientation.rotateInverse(env.windWorld);
    const Vec3 vAir = state.velocityBody - windBody;
    const double vForward = std::max(vAir.dot(axis), 0.0);

    // Static thrust from actuator-disk momentum theory.
    const double diskArea = kPi * 0.25 * prop.propDiameter * prop.propDiameter;
    const double tStatic = std::cbrt(2.0 * env.atmosphere.density * diskArea
                                     * power * power);

    // Dynamic (energy) thrust; guard the low-speed singularity with a floor.
    constexpr double kSpeedFloor = 1.0; // m/s
    const double tDynamic = prop.propEfficiency * power
                          / std::max(vForward, kSpeedFloor);

    // Delivered thrust: capped by the static-thrust ceiling at low speed.
    const double thrust = std::min(tDynamic, tStatic);
    out.thrust = thrust;

    Wrench wr;
    wr.force  = axis * thrust;
    wr.moment = prop.thrustPoint.cross(wr.force); // r x F about the CG
    out.wrench = wr;
    return out;
}

} // namespace fsim
