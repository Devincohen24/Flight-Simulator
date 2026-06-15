// =============================================================================
//  aircraft/Trim.cpp -- damped Newton solve for level-flight trim.
// =============================================================================
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "physics/Atmosphere.hpp"
#include "core/Constants.hpp"

#include <array>
#include <cmath>

namespace fsim {

namespace {

// Build the level-flight state for a candidate (alpha) at the given airspeed
// and altitude: pitch attitude = alpha so the flight-path angle is zero.
RigidBodyState levelState(double alpha, double V, double altitude) {
    RigidBodyState s;
    s.positionWorld = Vec3{0.0, 0.0, -altitude};
    s.orientation   = Quat::fromEuler(0.0, alpha, 0.0);
    s.velocityBody  = Vec3{V * std::cos(alpha), 0.0, V * std::sin(alpha)};
    s.angularVelocityBody = Vec3{0.0, 0.0, 0.0};
    return s;
}

// Residual vector r = (Fx_body, Fz_body, My_body) for unknowns (alpha, de, thr).
Vec3 residual(const Aircraft& ac, double V, double altitude,
              double alpha, double elevator, double throttle) {
    const RigidBodyState s = levelState(alpha, V, altitude);

    EnvironmentSample env;
    env.atmosphere = Atmosphere::sample(altitude);

    ControlInputs c;
    c.elevator = elevator;
    c.throttle = throttle;

    Wrench w;
    w += computeAerodynamics(ac, s, env, c).wrench;
    w += computeThrust(ac.propulsion, s, env, c).wrench;
    // Gravity resolved into the body frame.
    const Vec3 weightWorld{0.0, 0.0, ac.mass.mass * constants::kStandardGravity};
    w.force += s.orientation.rotateInverse(weightWorld);

    return Vec3{w.force.x, w.force.z, w.moment.y};
}

} // namespace

TrimResult trimLevelFlight(const Aircraft& ac, double airspeed, double altitude) {
    TrimResult result;

    // Unknowns: x = [alpha, elevator, throttle]. Reasonable initial guess.
    std::array<double, 3> x{0.05, -0.05, 0.5};
    const std::array<double, 3> fdStep{1e-6, 1e-6, 1e-5};

    auto evalResidual = [&](const std::array<double, 3>& v) {
        return residual(ac, airspeed, altitude, v[0], v[1], v[2]);
    };

    Vec3 r = evalResidual(x);
    for (int iter = 0; iter < 60; ++iter) {
        const double rn = r.norm();
        if (rn < 1e-6) { result.converged = true; break; }

        // Finite-difference Jacobian J (columns = d r / d x_j).
        Mat3 J;
        for (int j = 0; j < 3; ++j) {
            std::array<double, 3> xp = x;
            xp[j] += fdStep[j];
            const Vec3 rp = evalResidual(xp);
            const Vec3 col = (rp - r) * (1.0 / fdStep[j]);
            J.m[0][j] = col.x;
            J.m[1][j] = col.y;
            J.m[2][j] = col.z;
        }

        // Newton step delta = -J^-1 r, with light damping for robustness.
        const Vec3 delta = J.inverse() * (r * -1.0);
        const double damping = 0.8;
        x[0] += damping * delta.x;
        x[1] += damping * delta.y;
        x[2] += damping * delta.z;

        // Keep the search physically sane.
        if (x[2] < 0.0) x[2] = 0.0;
        if (x[2] > 1.0) x[2] = 1.0;

        r = evalResidual(x);
    }

    result.residual = r.norm();
    result.converged = result.converged || (result.residual < 1e-4);
    result.alpha = x[0];
    result.state = levelState(x[0], airspeed, altitude);
    result.controls.elevator = x[1];
    result.controls.throttle = x[2];
    result.controls.clamp();
    return result;
}

} // namespace fsim
