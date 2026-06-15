// =============================================================================
//  physics/Atmosphere.cpp  -- ISA implementation.
//
//  Troposphere (h <= 11 km), with lapse rate L:
//      T(h) = T0 - L h
//      p(h) = p0 * (T/T0)^( g0 / (R L) )
//  Isothermal lower stratosphere (11 km < h <= 20 km):
//      T(h) = T_tropopause (constant)
//      p(h) = p_tropopause * exp( -g0 (h - 11km) / (R T_tropopause) )
//  Density from the ideal gas law, speed of sound from a = sqrt(gamma R T).
// =============================================================================
#include "physics/Atmosphere.hpp"
#include "core/Constants.hpp"

#include <cmath>

namespace fsim {

using namespace constants;

AtmosphereSample Atmosphere::sample(double altitudeMeters) {
    if (altitudeMeters < 0.0) altitudeMeters = 0.0;

    AtmosphereSample s;
    s.altitude = altitudeMeters;

    if (altitudeMeters <= kTropopauseAltitude) {
        // --- Troposphere: linear temperature lapse ---
        const double T = kSeaLevelTemperature - kTroposphereLapseRate * altitudeMeters;
        const double exponent = kStandardGravity
                              / (kSpecificGasConstantAir * kTroposphereLapseRate);
        const double p = kSeaLevelPressure
                       * std::pow(T / kSeaLevelTemperature, exponent);
        s.temperature = T;
        s.pressure    = p;
    } else {
        // --- Lower stratosphere: isothermal layer above the tropopause ---
        const double T = kTropopauseTemperature;
        // Pressure at the tropopause (continuity with the troposphere layer).
        const double exponent = kStandardGravity
                              / (kSpecificGasConstantAir * kTroposphereLapseRate);
        const double pTropopause = kSeaLevelPressure
                                 * std::pow(kTropopauseTemperature / kSeaLevelTemperature,
                                            exponent);
        const double dh = altitudeMeters - kTropopauseAltitude;
        const double p = pTropopause
                       * std::exp(-kStandardGravity * dh
                                  / (kSpecificGasConstantAir * T));
        s.temperature = T;
        s.pressure    = p;
    }

    // Ideal gas law and speed of sound.
    s.density      = s.pressure / (kSpecificGasConstantAir * s.temperature);
    s.speedOfSound = std::sqrt(kRatioOfSpecificHeats
                               * kSpecificGasConstantAir * s.temperature);
    return s;
}

} // namespace fsim
