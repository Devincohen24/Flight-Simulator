// =============================================================================
//  app/main.cpp -- headless flight demonstration with cockpit instruments.
//
//  Loads the data-driven Cessna 172, trims it for level cruise, then flies it
//  with the full model (aerodynamics + propulsion + gravity) while driving the
//  engine system and reading out a live instrument panel and a GPS/nav fix.
//  A brief nose-up elevator pulse shows the pitch/airspeed response and the
//  resulting phugoid -- all emergent. Nothing is scripted.
//
//  Usage:  fsim_headless [path/to/aircraft.json]
// =============================================================================
#include "aircraft/Aircraft.hpp"
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"
#include "systems/Engine.hpp"
#include "systems/Instruments.hpp"
#include "systems/Navigation.hpp"
#include "environment/Weather.hpp"

#include <cstdio>
#include <memory>
#include <string>

using namespace fsim;

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1]
                                        : "data/aircraft/cessna172.json";
    Aircraft ac;
    try {
        ac = Aircraft::load(path);
    } catch (const std::exception& e) {
        std::printf("Failed to load aircraft '%s': %s\n", path.c_str(), e.what());
        std::printf("(run from the repo root, or pass the data-file path)\n");
        return 1;
    }

    const double cruise = 55.0;   // m/s true airspeed
    const double alt0   = 1000.0; // m

    const TrimResult trim = trimLevelFlight(ac, cruise, alt0);
    std::printf("Aircraft: %s\n", ac.name.c_str());
    std::printf("Trim @ %.0f m/s, %.0f m:  converged=%s  residual=%.2e  "
                "alpha=%.2f deg  elevator=%.3f  throttle=%.3f\n\n",
                cruise, alt0, trim.converged ? "yes" : "no", trim.residual,
                degrees(trim.alpha), trim.controls.elevator, trim.controls.throttle);

    Simulation sim(1.0 / 240.0);
    sim.setMassProperties(ac.mass);
    sim.addForceModel(std::make_shared<AerodynamicsForce>(ac));
    sim.addForceModel(std::make_shared<PropulsionForce>(ac.propulsion));
    sim.addForceModel(std::make_shared<GravityForce>());
    sim.setState(trim.state);
    sim.setControls(trim.controls);

    EngineSystem engine(ac.propulsion);
    engine.start();

    // Weather: a steady westerly crosswind (FROM 270 at 8 m/s) + light chop.
    WeatherSystem weather;
    weather.wind().addLayerFromHeading(0.0,    270.0, 8.0);
    weather.wind().addLayerFromHeading(3000.0, 270.0, 14.0); // stronger aloft
    weather.turbulence().setParams(DrydenTurbulence::light());

    // A VOR / waypoint somewhere ahead, with the field origin near Boulder, CO.
    NavigationComputer nav(Geodetic::fromDegrees(40.0, -105.0), alt0);
    Waypoint wp{"NORTH", Geodetic::fromDegrees(40.30, -105.0)}; // ~33 km north

    std::printf("# %5s %7s %7s %8s %7s %6s %6s %7s %6s %7s %8s\n",
                "t[s]", "IAS", "ALT", "VS", "PITCH", "BANK", "HDG",
                "RPM", "FUEL", "G", "WP[km]");

    auto report = [&](double t) {
        const EnvironmentSample env = sim.environment();
        const InstrumentReadings r = computeInstruments(
            sim.state(), env, engine.state(), sim.specificForceBody());
        double brg = 0, dist = 0;
        nav.bearingDistanceTo(sim.state(), wp, brg, dist);
        std::printf("  %5.1f %7.1f %7.0f %8.2f %7.1f %6.1f %6.0f %7.0f %6.1f %7.2f %8.1f\n",
                    t, r.indicatedAirspeed, r.altitudeMSL, r.verticalSpeed,
                    r.pitch, r.bank, r.heading, r.rpm, r.fuelRemaining,
                    r.loadFactor, dist / 1000.0);
    };

    auto stepOneSecond = [&]() {
        for (int i = 0; i < 240; ++i) {
            const EnvironmentSample env = sim.environment();
            engine.update(sim.fixedTimeStep(), sim.controls().throttle,
                          env.atmosphere.density / 1.225, env.atmosphere.pressure);
            // Advance the weather and feed the frozen wind to the next step.
            const double tas = sim.state().velocityBody.norm();
            const Vec3 wind = weather.update(sim.fixedTimeStep(),
                                             -sim.state().positionWorld.z, tas);
            sim.setWind(wind);
            sim.step();
        }
    };

    // Phase A: hands-off for 5 s.
    for (int s = 0; s <= 5; ++s) { report(sim.time()); stepOneSecond(); }

    // Phase B: 2 s nose-up elevator pulse.
    std::printf("# --- nose-up elevator pulse (2 s) ---\n");
    ControlInputs pulse = trim.controls;
    pulse.elevator = trim.controls.elevator - 0.25;
    sim.setControls(pulse);
    for (int s = 0; s < 2; ++s) { report(sim.time()); stepOneSecond(); }

    // Phase C: release to trim, observe the phugoid.
    std::printf("# --- controls returned to trim ---\n");
    sim.setControls(trim.controls);
    for (int s = 0; s < 10; ++s) { report(sim.time()); stepOneSecond(); }

    const auto gps = nav.fix(sim.state());
    const EulerAngles fe = toEuler(sim.state().orientation);
    std::printf("\nFinal GPS fix: %.4f, %.4f   alt %.0f m   gs %.1f m/s   trk %.0f deg\n",
                degrees(gps.position.latitude), degrees(gps.position.longitude),
                gps.altitude, gps.groundSpeed, gps.track);
    std::printf("Wind drift: heading %.0f deg vs ground track %.0f deg "
                "(crab from the %.1f m/s crosswind)\n",
                degrees(fe.yaw) < 0 ? degrees(fe.yaw) + 360 : degrees(fe.yaw),
                gps.track, weather.lastWind().norm());
    std::printf("Fuel burned: %.2f kg of %.0f kg\n",
                ac.propulsion.fuelCapacity - engine.state().fuelRemaining,
                ac.propulsion.fuelCapacity);
    return 0;
}
