// =============================================================================
//  aircraft/Aircraft.hpp
//
//  Data-driven aircraft definition. Everything that distinguishes one aircraft
//  from another -- mass, inertia, geometry, aerodynamic stability derivatives,
//  propulsion, landing gear -- lives in these plain structs, loaded from a JSON
//  data file (see data/aircraft/cessna172.json). Adding a new aircraft is just
//  a new data file; no engine code changes. This is the central expandability
//  requirement of the project.
//
//  Aerodynamic coefficients follow the standard fixed-wing linearised model
//  (e.g. Stevens & Lewis, Roskam, Beard & McLain). All angles are radians,
//  all derivatives per-radian, SI units throughout.
// =============================================================================
#pragma once

#include "core/Math.hpp"
#include "physics/RigidBody.hpp"

#include <string>
#include <vector>

namespace fsim {

// --- Wing / reference geometry ----------------------------------------------
struct WingGeometry {
    double area    {16.2};  // S,  reference wing area, m^2
    double span    {11.0};  // b,  wing span, m
    double chord   {1.49};  // c,  mean aerodynamic chord, m
    double oswald  {0.75};  // e,  Oswald span efficiency factor
    double aspectRatio() const { return span * span / area; } // AR = b^2 / S
};

// --- Longitudinal + lateral/directional stability & control derivatives -----
// Naming: C<force/moment><wrt>. e.g. CLalpha = dCL/dalpha, Cmde = dCm/d(elevator).
struct AeroCoefficients {
    // Lift
    double CL0{0.31}, CLalpha{5.143}, CLq{3.9}, CLde{0.43}, CLflap{0.7};
    // Drag (parasitic + induced via Oswald/AR)
    double CD0{0.031};
    // Side force
    double CYbeta{-0.31}, CYdr{0.187};
    // Roll moment
    double Clbeta{-0.089}, Clp{-0.47}, Clr{0.096}, Clda{-0.178}, Cldr{0.0147};
    // Pitch moment
    double Cm0{-0.015}, Cmalpha{-0.89}, Cmq{-12.4}, Cmde{-1.28};
    // Yaw moment
    double Cnbeta{0.065}, Cnp{-0.03}, Cnr{-0.099}, Cnda{-0.053}, Cndr{-0.0657};
    // Stall model (blends linear lift into a flat-plate model past alpha_stall)
    double alphaStall{0.2792};   // ~16 deg
    double stallBlendRate{20.0}; // sigmoid sharpness M
};

// --- Maximum control-surface deflections (rad) used to map [-1,1] -> radians -
struct ControlLimits {
    double elevatorMax{0.3491}; // 20 deg
    double aileronMax {0.3491}; // 20 deg
    double rudderMax  {0.2793}; // 16 deg
    double flapMax    {0.5236}; // 30 deg
};

// --- Piston + propeller propulsion ------------------------------------------
struct PropulsionConfig {
    double maxPower      {119000.0}; // W  (~160 hp, Lycoming O-320)
    double propDiameter  {1.93};     // m
    double propEfficiency{0.80};     // eta_prop at cruise advance ratio
    Vec3   thrustPoint   {0.0, 0.0, 0.0}; // application point in body frame (m)
    Vec3   thrustAxis    {1.0, 0.0, 0.0}; // direction of thrust in body frame
    double idlePowerFraction{0.05};  // residual power at zero throttle
};

// --- One landing-gear contact point (spring-damper strut + tire friction) ---
struct GearUnit {
    Vec3   position{0.0, 0.0, 1.0}; // contact point in body frame (m), +Z down
    double stiffness   {40000.0};   // strut spring constant, N/m
    double damping     {5000.0};    // strut damping, N*s/m
    double frictionRoll{0.04};      // rolling-resistance coefficient
    double frictionSide{0.7};       // lateral (cornering) friction coefficient
    double frictionBrake{0.5};      // additional longitudinal friction at full brake
    bool   braked      {false};     // do wheel brakes act on this unit?
    std::string name   {"gear"};
};

// --- Complete aircraft definition -------------------------------------------
struct Aircraft {
    std::string      name{"unnamed"};
    MassProperties   mass{};
    WingGeometry     wing{};
    AeroCoefficients aero{};
    ControlLimits    limits{};
    PropulsionConfig propulsion{};
    std::vector<GearUnit> gear{};

    // Load an aircraft definition from a JSON data file. Missing optional
    // fields fall back to the struct defaults above.
    static Aircraft load(const std::string& path);
};

} // namespace fsim
