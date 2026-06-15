// =============================================================================
//  terrain/Terrain.hpp
//
//  Terrain as a height field: a function giving ground elevation (metres above
//  mean sea level) at a world (North, East) location. This is deliberately
//  decoupled from rendering -- the SAME height field is queried by the landing
//  gear for ground contact and by the renderer to build the terrain mesh, so
//  the aircraft always touches down on exactly the ground that is drawn.
//
//  Elevation is returned in MSL metres; the NED "down" coordinate of the ground
//  surface is therefore -height(north, east).
// =============================================================================
#pragma once

#include "core/Math.hpp"

namespace fsim {

class HeightField {
public:
    virtual ~HeightField() = default;

    // Ground elevation (m above MSL) at a world location.
    virtual double height(double north, double east) const = 0;

    // Outward (upward) surface normal in the NED frame, from the height
    // gradient. For flat ground this is (0, 0, -1).
    Vec3 normalNed(double north, double east, double eps = 1.0) const {
        const double hN = (height(north + eps, east) - height(north - eps, east)) / (2.0 * eps);
        const double hE = (height(north, east + eps) - height(north, east - eps)) / (2.0 * eps);
        // Surface normal pointing "up" (negative NED z).
        return Vec3{-hN, -hE, -1.0}.normalized();
    }
};

// Perfectly flat ground at a fixed elevation (e.g. a sea-level airfield).
class FlatGround final : public HeightField {
public:
    explicit FlatGround(double elevation = 0.0) : elevation_(elevation) {}
    double height(double, double) const override { return elevation_; }
private:
    double elevation_;
};

// Smooth procedural terrain: a deterministic sum of sinusoids (rolling hills),
// optionally with a flat region around an airfield so take-off/landing have a
// usable runway. No data files or noise tables required.
class ProceduralTerrain final : public HeightField {
public:
    struct Params {
        double baseElevation{0.0};     // m, mean ground level
        double amplitude{120.0};       // m, peak-to-mean hill height
        double wavelength{4000.0};     // m, dominant feature size
        double airfieldRadius{1500.0}; // m, flat zone radius around the origin
        double airfieldFalloff{1000.0};// m, blend distance from flat to hills
    };

    ProceduralTerrain() = default;
    explicit ProceduralTerrain(const Params& p) : p_(p) {}

    double height(double north, double east) const override;

private:
    Params p_;
};

} // namespace fsim
