// =============================================================================
//  environment/Wind.hpp
//
//  Steady wind as a function of altitude: a stack of wind layers between which
//  the wind vector is linearly interpolated. Wind is stored as the NED velocity
//  of the air mass (the direction the air moves TO); the aviation "wind from a
//  heading" convention is provided as a helper. Because the aerodynamics use
//  the air-relative velocity (v_body - wind), a non-zero wind automatically
//  produces crab, drift and the airspeed/groundspeed split with no special code.
// =============================================================================
#pragma once

#include "core/Math.hpp"

#include <algorithm>
#include <vector>

namespace fsim {

struct WindLayer {
    double altitude{0.0};   // m MSL
    Vec3   velocityNed{};   // air-mass velocity (NED), m/s
};

class WindField {
public:
    // Add a layer from the aviation convention: wind FROM 'fromHeadingDeg'
    // (degrees, met. convention) at 'speed' m/s. The air moves toward the
    // reciprocal heading.
    void addLayerFromHeading(double altitude, double fromHeadingDeg, double speed) {
        const double toHeading = radians(fromHeadingDeg + 180.0);
        // Heading measured clockwise from North: N = cos, E = sin.
        Vec3 v{speed * std::cos(toHeading), speed * std::sin(toHeading), 0.0};
        addLayer({altitude, v});
    }

    void addLayer(const WindLayer& layer) {
        layers_.push_back(layer);
        std::sort(layers_.begin(), layers_.end(),
                  [](const WindLayer& a, const WindLayer& b) {
                      return a.altitude < b.altitude;
                  });
    }

    // Wind velocity (NED) at an altitude, linearly interpolated between layers
    // and clamped to the end layers outside the defined range.
    Vec3 at(double altitude) const {
        if (layers_.empty()) return Vec3{0, 0, 0};
        if (altitude <= layers_.front().altitude) return layers_.front().velocityNed;
        if (altitude >= layers_.back().altitude)  return layers_.back().velocityNed;
        for (std::size_t i = 1; i < layers_.size(); ++i) {
            if (altitude <= layers_[i].altitude) {
                const WindLayer& lo = layers_[i - 1];
                const WindLayer& hi = layers_[i];
                const double t = (altitude - lo.altitude) / (hi.altitude - lo.altitude);
                return lo.velocityNed + (hi.velocityNed - lo.velocityNed) * t;
            }
        }
        return layers_.back().velocityNed;
    }

    bool empty() const { return layers_.empty(); }

private:
    std::vector<WindLayer> layers_;
};

} // namespace fsim
