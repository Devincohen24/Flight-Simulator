// =============================================================================
//  tests/test_aero.cpp -- aerodynamic model: angles, dynamic pressure, force
//  directions, control-surface signs, and emergent stall behaviour.
// =============================================================================
#include "TestHarness.hpp"
#include "aircraft/Aircraft.hpp"
#include "aero/Aerodynamics.hpp"
#include "physics/Atmosphere.hpp"
#include "core/Constants.hpp"

#include <string>

using namespace fsim;

static Aircraft loadAC() {
    return Aircraft::load(std::string(FSIM_SOURCE_DIR) + "/data/aircraft/cessna172.json");
}

// Build a state at airspeed V with angle of attack alpha (level wings).
static RigidBodyState flow(double V, double alpha) {
    RigidBodyState s;
    s.orientation  = Quat::fromEuler(0.0, alpha, 0.0);
    s.velocityBody = Vec3{V * std::cos(alpha), 0.0, V * std::sin(alpha)};
    return s;
}

static EnvironmentSample seaLevel() {
    EnvironmentSample env;
    env.atmosphere = Atmosphere::sample(0.0);
    return env;
}

TEST_CASE(aero_angles_and_dynamic_pressure) {
    const Aircraft ac = loadAC();
    const double V = 50.0;
    const AeroState a = computeAerodynamics(ac, flow(V, 0.0), seaLevel(), ControlInputs{});

    CHECK_NEAR(a.airspeed, V, 1e-9);
    CHECK_NEAR(a.alpha, 0.0, 1e-9);
    CHECK_NEAR(a.beta, 0.0, 1e-9);
    CHECK_NEAR(a.dynamicPressure, 0.5 * 1.225 * V * V, 1e-3);
    // At alpha=0 the lift coefficient is CL0, less a ~0.75% reduction from the
    // (symmetric) stall-blend sigmoid -- negligible and well within 1%.
    CHECK_NEAR(a.CL, ac.aero.CL0, 0.01);
}

TEST_CASE(aero_lift_and_drag_directions) {
    const Aircraft ac = loadAC();
    const AeroState a = computeAerodynamics(ac, flow(50.0, radians(3.0)), seaLevel(),
                                            ControlInputs{});
    // Lift acts "up" (body -Z) and drag "aft" (body -X) at small positive alpha.
    CHECK(a.wrench.force.z < 0.0);
    CHECK(a.wrench.force.x < 0.0);
}

TEST_CASE(aero_lift_curve_slope_positive) {
    const Aircraft ac = loadAC();
    const double cl2  = computeAerodynamics(ac, flow(50, radians(2.0)),  seaLevel(), {}).CL;
    const double cl8  = computeAerodynamics(ac, flow(50, radians(8.0)),  seaLevel(), {}).CL;
    CHECK(cl8 > cl2); // CL rises with alpha in the linear regime
}

TEST_CASE(aero_stall_drops_lift) {
    const Aircraft ac = loadAC();
    // Lift below the stall angle should exceed lift well past it.
    const double clPre  = computeAerodynamics(ac, flow(50, radians(12.0)), seaLevel(), {}).CL;
    const double clPost = computeAerodynamics(ac, flow(50, radians(30.0)), seaLevel(), {}).CL;
    CHECK(clPost < clPre);
    // Drag must rise sharply past the stall (separated flow).
    const double cdPre  = computeAerodynamics(ac, flow(50, radians(4.0)),  seaLevel(), {}).CD;
    const double cdPost = computeAerodynamics(ac, flow(50, radians(30.0)), seaLevel(), {}).CD;
    CHECK(cdPost > cdPre);
}

TEST_CASE(aero_elevator_pitch_authority) {
    const Aircraft ac = loadAC();
    const RigidBodyState s = flow(50.0, radians(2.0));
    ControlInputs up;   up.elevator = 0.0;
    ControlInputs down; down.elevator = 1.0; // TE down -> nose-down
    const double cm0 = computeAerodynamics(ac, s, seaLevel(), up).Cm;
    const double cm1 = computeAerodynamics(ac, s, seaLevel(), down).Cm;
    CHECK(cm1 < cm0); // positive elevator deflection reduces (nose-down) Cm
}

TEST_CASE(aero_directional_and_dihedral_signs) {
    const Aircraft ac = loadAC();
    // Positive sideslip (beta > 0): build a state with body-y velocity.
    RigidBodyState s;
    const double V = 50.0, beta = radians(8.0);
    s.velocityBody = Vec3{V * std::cos(beta), V * std::sin(beta), 0.0};
    const AeroState a = computeAerodynamics(ac, s, seaLevel(), ControlInputs{});
    CHECK(a.beta > 0.0);
    // Weathercock (Cnbeta>0) -> positive yaw moment; dihedral (Clbeta<0) -> negative roll.
    CHECK(a.Cn > 0.0);
    CHECK(a.Cl < 0.0);
}

FSIM_TEST_MAIN()
