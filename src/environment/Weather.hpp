// =============================================================================
//  environment/Weather.hpp
//
//  Composite weather system: steady layered wind + continuous turbulence. It is
//  advanced once per physics step and produces the total NED wind velocity that
//  the Simulation hands to the aerodynamics (a frozen field within each RK4
//  step). This keeps all the stochastic / time-varying weather state out of the
//  deterministic physics core.
// =============================================================================
#pragma once

#include "environment/Wind.hpp"
#include "environment/Turbulence.hpp"

namespace fsim {

class WeatherSystem {
public:
    WindField&        wind()       { return wind_; }
    DrydenTurbulence& turbulence() { return turbulence_; }

    // Total wind (NED) at the given altitude, advancing turbulence by dt.
    Vec3 update(double dt, double altitude, double airspeed) {
        const Vec3 steady = wind_.at(altitude);
        const Vec3 gust   = turbulence_.update(dt, airspeed);
        lastWind_ = steady + gust;
        return lastWind_;
    }

    const Vec3& lastWind() const { return lastWind_; }

private:
    WindField        wind_;
    DrydenTurbulence turbulence_;
    Vec3             lastWind_{};
};

} // namespace fsim
