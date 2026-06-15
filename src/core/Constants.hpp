// =============================================================================
//  core/Constants.hpp
//
//  Physical constants and reference values. Grouped so the provenance of every
//  number is documented in one place rather than scattered as magic literals.
//  SI units throughout (metres, kilograms, seconds, kelvin, pascals).
// =============================================================================
#pragma once

namespace fsim::constants {

// --- Gravity -----------------------------------------------------------------
// Standard gravitational acceleration at mean sea level (CODATA / ISO 80000).
constexpr double kStandardGravity = 9.80665; // m/s^2

// --- International Standard Atmosphere (ISA), sea-level reference -------------
constexpr double kSeaLevelTemperature = 288.15;   // K   (15 degC)
constexpr double kSeaLevelPressure    = 101325.0; // Pa
constexpr double kSeaLevelDensity     = 1.225;    // kg/m^3

// Troposphere temperature lapse rate (temperature decreases with altitude).
constexpr double kTroposphereLapseRate = 0.0065;  // K/m  (0 .. 11 km)
constexpr double kTropopauseAltitude   = 11000.0; // m
constexpr double kTropopauseTemperature = kSeaLevelTemperature
                                        - kTroposphereLapseRate * kTropopauseAltitude; // 216.65 K

// --- Gas properties of dry air ----------------------------------------------
constexpr double kSpecificGasConstantAir = 287.05287; // J/(kg*K)
constexpr double kRatioOfSpecificHeats   = 1.4;       // gamma, for speed of sound

} // namespace fsim::constants
