// =============================================================================
//  tests/test_weather.cpp -- wind layers and Dryden/Gauss-Markov turbulence.
// =============================================================================
#include "TestHarness.hpp"
#include "environment/Wind.hpp"
#include "environment/Turbulence.hpp"
#include "environment/Weather.hpp"

#include <cmath>

using namespace fsim;

TEST_CASE(wind_from_heading_convention) {
    WindField w;
    w.addLayerFromHeading(0.0, /*from*/0.0, 10.0); // wind FROM north -> air to south
    const Vec3 v = w.at(0.0);
    CHECK_NEAR(v.x, -10.0, 1e-9);
    CHECK_NEAR(v.y, 0.0, 1e-9);

    WindField w2;
    w2.addLayerFromHeading(0.0, /*from*/270.0, 10.0); // FROM west -> air to east
    const Vec3 v2 = w2.at(0.0);
    CHECK_NEAR(v2.x, 0.0, 1e-9);
    CHECK_NEAR(v2.y, 10.0, 1e-9);
}

TEST_CASE(wind_layer_interpolation_and_clamp) {
    WindField w;
    w.addLayer({0.0,    Vec3{-10.0, 0.0, 0.0}});
    w.addLayer({1000.0, Vec3{0.0, 10.0, 0.0}});

    const Vec3 mid = w.at(500.0);
    CHECK_NEAR(mid.x, -5.0, 1e-9);
    CHECK_NEAR(mid.y,  5.0, 1e-9);
    // Clamped outside the range.
    CHECK_NEAR(w.at(-200.0).x, -10.0, 1e-9);
    CHECK_NEAR(w.at(5000.0).y, 10.0, 1e-9);
}

TEST_CASE(turbulence_zero_intensity_is_calm) {
    DrydenTurbulence t(DrydenTurbulence::Params{}); // all sigma = 0
    for (int i = 0; i < 100; ++i) {
        const Vec3 g = t.update(0.02, 50.0);
        CHECK_NEAR(g.x, 0.0, 1e-12);
        CHECK_NEAR(g.y, 0.0, 1e-12);
        CHECK_NEAR(g.z, 0.0, 1e-12);
    }
}

TEST_CASE(turbulence_variance_matches_sigma) {
    DrydenTurbulence::Params p;
    p.sigmaHorizontal = 3.5; p.sigmaVertical = 2.1;
    p.scaleHorizontal = 533.0; p.scaleVertical = 533.0;
    DrydenTurbulence t(p, /*seed=*/42u);

    const int N = 200000;
    double sumX = 0, sumX2 = 0, sumZ2 = 0;
    for (int i = 0; i < N; ++i) {
        const Vec3 g = t.update(0.02, 50.0);
        sumX += g.x; sumX2 += g.x * g.x; sumZ2 += g.z * g.z;
    }
    const double meanX = sumX / N;
    const double varX  = sumX2 / N - meanX * meanX;
    const double varZ  = sumZ2 / N;
    // The gust is strongly autocorrelated (correlation time ~ L/V ~ 10 s), so
    // the sample-mean estimator has a standard error of ~0.2 m/s -- allow for it.
    CHECK_NEAR(meanX, 0.0, 0.5);
    CHECK_NEAR(std::sqrt(varX), 3.5, 0.25); // RMS ~ sigma_horizontal
    CHECK_NEAR(std::sqrt(varZ), 2.1, 0.20); // RMS ~ sigma_vertical
}

TEST_CASE(turbulence_deterministic_with_seed) {
    DrydenTurbulence a(DrydenTurbulence::moderate(), 7u);
    DrydenTurbulence b(DrydenTurbulence::moderate(), 7u);
    for (int i = 0; i < 1000; ++i) {
        const Vec3 ga = a.update(0.02, 55.0);
        const Vec3 gb = b.update(0.02, 55.0);
        CHECK_NEAR(ga.x, gb.x, 1e-12);
        CHECK_NEAR(ga.z, gb.z, 1e-12);
    }
}

TEST_CASE(weather_combines_steady_and_gust) {
    WeatherSystem wx;
    wx.wind().addLayer({0.0, Vec3{-8.0, 0.0, 0.0}});
    // No turbulence -> total wind equals the steady layer.
    const Vec3 calm = wx.update(0.02, 1000.0, 50.0);
    CHECK_NEAR(calm.x, -8.0, 1e-12);
    CHECK_NEAR(calm.y, 0.0, 1e-12);

    // With turbulence the wind deviates from the steady value.
    wx.turbulence().setParams(DrydenTurbulence::moderate());
    bool deviated = false;
    for (int i = 0; i < 200; ++i) {
        const Vec3 g = wx.update(0.02, 1000.0, 50.0);
        if (std::fabs(g.x - (-8.0)) > 0.5) { deviated = true; break; }
    }
    CHECK(deviated);
}

FSIM_TEST_MAIN()
