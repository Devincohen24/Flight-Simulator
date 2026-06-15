// =============================================================================
//  systems/Navigation.hpp
//
//  Navigation computations on top of the flat-Earth simulation. A local-tangent
//  origin (lat/lon at the NED origin) lets us project the aircraft's NED
//  position back to geodetic coordinates for a GPS fix, compute great-circle
//  bearing/distance to waypoints, and resolve VOR radials with a CDI deviation.
//
//  The flat-Earth assumption (NED <-> lat/lon via a local tangent plane) is
//  accurate over the hundreds-of-km scale of a typical flight; a full WGS-84
//  model can replace LocalOrigin later without touching the interface.
// =============================================================================
#pragma once

#include "core/Math.hpp"
#include "physics/RigidBody.hpp"

#include <string>

namespace fsim {

// Geodetic coordinate; latitude/longitude stored in radians.
struct Geodetic {
    double latitude{0.0};
    double longitude{0.0};
    static Geodetic fromDegrees(double latDeg, double lonDeg) {
        return {radians(latDeg), radians(lonDeg)};
    }
};

// Great-circle distance (m) and initial bearing (rad, [0,2pi)) between points.
double greatCircleDistance(const Geodetic& a, const Geodetic& b);
double initialBearing(const Geodetic& from, const Geodetic& to);

struct Waypoint {
    std::string name;
    Geodetic    position;
};

struct GpsFix {
    Geodetic position;
    double    altitude{0.0};    // m MSL
    double    groundSpeed{0.0}; // m/s
    double    track{0.0};       // deg [0,360), course over ground
};

// VOR/CDI indication relative to a selected course (OBS).
struct VorIndication {
    double radial{0.0};     // deg, the radial the aircraft is on (FROM the station)
    double deviation{0.0};  // deg, course deviation (+ = fly right); clamped to +/-10
    bool   toFlag{false};   // true = TO the station, false = FROM
};

class NavigationComputer {
public:
    explicit NavigationComputer(const Geodetic& origin, double originElevation = 0.0)
        : origin_(origin), originElevation_(originElevation) {}

    // Project an NED position to geodetic coordinates (local tangent plane).
    Geodetic toGeodetic(const Vec3& nedPosition) const;

    // GPS fix from the aircraft state.
    GpsFix fix(const RigidBodyState& state) const;

    // Bearing (deg [0,360)) and distance (m) from the aircraft to a waypoint.
    void bearingDistanceTo(const RigidBodyState& state, const Waypoint& wp,
                           double& bearingDeg, double& distanceM) const;

    // VOR indication for a ground station given the selected OBS course (deg).
    VorIndication vor(const RigidBodyState& state, const Geodetic& station,
                      double obsCourseDeg) const;

private:
    Geodetic origin_;
    double   originElevation_;
};

} // namespace fsim
