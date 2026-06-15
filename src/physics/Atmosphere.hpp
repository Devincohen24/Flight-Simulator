// =============================================================================
//  physics/Atmosphere.hpp
//
//  International Standard Atmosphere (ISA) model. Given a geometric altitude it
//  returns the local temperature, pressure, density and speed of sound. These
//  feed dynamic pressure (q_bar = 1/2 rho V^2), Mach number and engine lapse.
//
//  Two layers are modelled, which covers all general-aviation and most jet
//  cruise altitudes:
//    * Troposphere      (0 .. 11 km): temperature falls linearly with altitude.
//    * Lower stratosphere(11 .. 20 km): temperature constant (isothermal).
// =============================================================================
#pragma once

namespace fsim {

struct AtmosphereSample {
    double altitude    {0.0}; // m (geometric, the query altitude)
    double temperature {0.0}; // K
    double pressure    {0.0}; // Pa
    double density     {0.0}; // kg/m^3
    double speedOfSound{0.0}; // m/s
};

class Atmosphere {
public:
    // Sample the ISA at a geometric altitude in metres above mean sea level.
    // Negative altitudes (below sea level) are clamped to 0 for stability.
    static AtmosphereSample sample(double altitudeMeters);

    // Convenience accessors.
    static double density(double altitudeMeters)      { return sample(altitudeMeters).density; }
    static double speedOfSound(double altitudeMeters) { return sample(altitudeMeters).speedOfSound; }
};

} // namespace fsim
