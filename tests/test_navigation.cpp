// =============================================================================
//  tests/test_navigation.cpp -- geodetic projection, waypoint and VOR nav.
// =============================================================================
#include "TestHarness.hpp"
#include "systems/Navigation.hpp"

using namespace fsim;

TEST_CASE(great_circle_distance_one_degree) {
    // One degree of latitude is ~111.2 km along a great circle.
    const Geodetic a = Geodetic::fromDegrees(0.0, 0.0);
    const Geodetic b = Geodetic::fromDegrees(1.0, 0.0);
    CHECK_NEAR(greatCircleDistance(a, b), 111195.0, 100.0);
}

TEST_CASE(initial_bearing_cardinals) {
    const Geodetic o = Geodetic::fromDegrees(0.0, 0.0);
    CHECK_NEAR(degrees(initialBearing(o, Geodetic::fromDegrees(1.0, 0.0))), 0.0,  1e-3); // N
    CHECK_NEAR(degrees(initialBearing(o, Geodetic::fromDegrees(0.0, 1.0))), 90.0, 1e-2); // E
}

TEST_CASE(ned_to_geodetic_roundtrips_north) {
    const Geodetic origin = Geodetic::fromDegrees(40.0, -105.0);
    NavigationComputer nav(origin, 1600.0);
    // ~111195 m north should be about +1 degree latitude.
    const Geodetic g = nav.toGeodetic(Vec3{111195.0, 0.0, 0.0});
    CHECK_NEAR(degrees(g.latitude), 41.0, 0.05);
    CHECK_NEAR(degrees(g.longitude), -105.0, 1e-6);
}

TEST_CASE(gps_fix_track_and_altitude) {
    NavigationComputer nav(Geodetic::fromDegrees(0.0, 0.0), 0.0);
    RigidBodyState s;
    s.positionWorld = Vec3{0, 0, -1500.0};
    s.velocityBody  = Vec3{0.0, 50.0, 0.0}; // moving East (body y), wings level
    const GpsFix f = nav.fix(s);
    CHECK_NEAR(f.altitude, 1500.0, 1e-6);
    CHECK_NEAR(f.groundSpeed, 50.0, 1e-6);
    CHECK_NEAR(f.track, 90.0, 1e-6); // due East
}

TEST_CASE(waypoint_bearing_distance) {
    NavigationComputer nav(Geodetic::fromDegrees(0.0, 0.0), 0.0);
    // Aircraft 10 km north of the origin; waypoint at the origin -> bearing South.
    RigidBodyState s;
    s.positionWorld = Vec3{10000.0, 0.0, -1000.0};
    Waypoint wp{"ORIG", Geodetic::fromDegrees(0.0, 0.0)};
    double brg = 0, dist = 0;
    nav.bearingDistanceTo(s, wp, brg, dist);
    CHECK_NEAR(dist, 10000.0, 50.0);
    CHECK_NEAR(brg, 180.0, 0.5); // pointing back south
}

TEST_CASE(vor_radial_and_deviation) {
    NavigationComputer nav(Geodetic::fromDegrees(0.0, 0.0), 0.0);
    const Geodetic station = Geodetic::fromDegrees(0.0, 0.0);

    // Aircraft due north of the station -> on the 360 radial.
    RigidBodyState s;
    s.positionWorld = Vec3{20000.0, 0.0, -2000.0};
    const VorIndication ind = nav.vor(s, station, /*obs=*/0.0);
    CHECK_NEAR(ind.radial, 0.0, 0.5);
    CHECK_NEAR(ind.deviation, 0.0, 0.5); // centred CDI on the 360 radial
    CHECK(ind.toFlag == false);          // FROM the station

    // Off to the east of that radial with OBS 360 -> needle deflects.
    RigidBodyState s2;
    s2.positionWorld = Vec3{20000.0, 3000.0, -2000.0};
    const VorIndication ind2 = nav.vor(s2, station, 0.0);
    CHECK(std::fabs(ind2.deviation) > 1.0);
}

FSIM_TEST_MAIN()
