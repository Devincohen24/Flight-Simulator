// =============================================================================
//  systems/Navigation.cpp -- see Navigation.hpp.
// =============================================================================
#include "systems/Navigation.hpp"

#include <algorithm>
#include <cmath>

namespace fsim {

namespace {
constexpr double kEarthRadius = 6371000.0; // m, mean radius

double wrap360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}
// Wrap an angle to (-180, 180].
double wrap180(double deg) {
    deg = wrap360(deg);
    if (deg > 180.0) deg -= 360.0;
    return deg;
}
} // namespace

double greatCircleDistance(const Geodetic& a, const Geodetic& b) {
    const double dLat = b.latitude - a.latitude;
    const double dLon = b.longitude - a.longitude;
    const double s = std::sin(dLat * 0.5) * std::sin(dLat * 0.5)
                   + std::cos(a.latitude) * std::cos(b.latitude)
                       * std::sin(dLon * 0.5) * std::sin(dLon * 0.5);
    return 2.0 * kEarthRadius * std::asin(std::min(1.0, std::sqrt(s)));
}

double initialBearing(const Geodetic& from, const Geodetic& to) {
    const double dLon = to.longitude - from.longitude;
    const double y = std::sin(dLon) * std::cos(to.latitude);
    const double x = std::cos(from.latitude) * std::sin(to.latitude)
                   - std::sin(from.latitude) * std::cos(to.latitude) * std::cos(dLon);
    double brg = std::atan2(y, x);
    if (brg < 0.0) brg += 2.0 * kPi;
    return brg;
}

Geodetic NavigationComputer::toGeodetic(const Vec3& nedPosition) const {
    // Local tangent plane: North -> latitude, East -> longitude.
    const double dLat = nedPosition.x / kEarthRadius;
    const double dLon = nedPosition.y / (kEarthRadius * std::cos(origin_.latitude));
    return {origin_.latitude + dLat, origin_.longitude + dLon};
}

GpsFix NavigationComputer::fix(const RigidBodyState& state) const {
    GpsFix f;
    f.position = toGeodetic(state.positionWorld);
    f.altitude = -state.positionWorld.z + originElevation_;

    const Vec3 vWorld = state.orientation.rotate(state.velocityBody);
    f.groundSpeed = std::sqrt(vWorld.x * vWorld.x + vWorld.y * vWorld.y);
    // Track over ground: bearing of the horizontal velocity (North=x, East=y).
    f.track = (f.groundSpeed > 1e-3) ? wrap360(degrees(std::atan2(vWorld.y, vWorld.x))) : 0.0;
    return f;
}

void NavigationComputer::bearingDistanceTo(const RigidBodyState& state,
                                           const Waypoint& wp,
                                           double& bearingDeg,
                                           double& distanceM) const {
    const Geodetic here = toGeodetic(state.positionWorld);
    distanceM  = greatCircleDistance(here, wp.position);
    bearingDeg = wrap360(degrees(initialBearing(here, wp.position)));
}

VorIndication NavigationComputer::vor(const RigidBodyState& state,
                                      const Geodetic& station,
                                      double obsCourseDeg) const {
    const Geodetic here = toGeodetic(state.positionWorld);
    VorIndication ind;

    // The radial is the bearing FROM the station TO the aircraft.
    ind.radial = wrap360(degrees(initialBearing(station, here)));

    // Deviation of the selected course from the current radial. The aircraft is
    // on the OBS radial when (radial - obs) == 0; TO/FROM resolves the ambiguity.
    const double diff = wrap180(ind.radial - obsCourseDeg);
    if (std::fabs(diff) <= 90.0) {
        ind.toFlag = false;          // FROM: flying away on the selected radial
        ind.deviation = diff;
    } else {
        ind.toFlag = true;           // TO: the reciprocal course points to the station
        ind.deviation = wrap180(180.0 - (ind.radial - obsCourseDeg));
    }
    // Full-scale CDI deflection is +/-10 degrees on a VOR.
    ind.deviation = std::clamp(ind.deviation, -10.0, 10.0);
    return ind;
}

} // namespace fsim
