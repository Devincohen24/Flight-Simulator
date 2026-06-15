// =============================================================================
//  aero/Aerodynamics.cpp -- see Aerodynamics.hpp for the model description.
// =============================================================================
#include "aero/Aerodynamics.hpp"
#include "core/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace fsim {

namespace {

// Sigmoid stall-blend factor (Beard & McLain, "Small Unmanned Aircraft").
// Returns ~0 below the stall angle and ~1 well beyond it, so lift transitions
// smoothly from the linear regime to a flat-plate regime.
double stallBlend(double alpha, double alpha0, double M) {
    const double e1 = std::exp(-M * (alpha - alpha0));
    const double e2 = std::exp( M * (alpha + alpha0));
    const double num = 1.0 + e1 + e2;
    const double den = (1.0 + e1) * (1.0 + e2);
    return num / den;
}

} // namespace

AeroState computeAerodynamics(const Aircraft& ac,
                              const RigidBodyState& state,
                              const EnvironmentSample& env,
                              const ControlInputs& controls) {
    const WingGeometry&     wing  = ac.wing;
    const AeroCoefficients& c     = ac.aero;
    const ControlLimits&    lim   = ac.limits;

    AeroState out;

    // --- Air-relative velocity in the body frame ---
    // v_air = v_body - R(q)^T * wind_world  (wind is zero until Phase 5).
    const Vec3 windBody = state.orientation.rotateInverse(env.windWorld);
    const Vec3 vAir = state.velocityBody - windBody;

    const double u = vAir.x, v = vAir.y, w = vAir.z;
    const double V = vAir.norm();
    out.airspeed = V;

    // Below a small airspeed the aerodynamic model is ill-conditioned (alpha,
    // beta and the non-dimensional rates all blow up); return no aero force.
    constexpr double kMinAirspeed = 0.5; // m/s
    if (V < kMinAirspeed) return out;

    const double alpha = std::atan2(w, u);
    double sinBeta = v / V;
    sinBeta = std::clamp(sinBeta, -1.0, 1.0);
    const double beta = std::asin(sinBeta);
    out.alpha = alpha;
    out.beta  = beta;

    const double qbar = 0.5 * env.atmosphere.density * V * V;
    out.dynamicPressure = qbar;

    // --- Control deflections (rad) ---
    const double de = controls.elevator * lim.elevatorMax; // +back-stick -> +pitch
    const double da = controls.aileron  * lim.aileronMax;
    const double dr = controls.rudder   * lim.rudderMax;

    // --- Non-dimensional body rates ---
    const double twoV  = 2.0 * V;
    const double phat = state.angularVelocityBody.x * wing.span  / twoV;
    const double qhat = state.angularVelocityBody.y * wing.chord / twoV;
    const double rhat = state.angularVelocityBody.z * wing.span  / twoV;

    // --- Lift coefficient with stall blend ---
    // Flap lift increment scales with the normalised flap setting [0,1].
    const double CL_linear = c.CL0 + c.CLalpha * alpha
                           + c.CLq * qhat + c.CLde * de
                           + c.CLflap * controls.flaps;
    const double sigma = stallBlend(alpha, c.alphaStall, c.stallBlendRate);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    const double CL_flat = 2.0 * std::copysign(1.0, alpha) * sa * sa * ca;
    const double CL = (1.0 - sigma) * CL_linear + sigma * CL_flat;

    // --- Drag: parabolic polar in the linear regime, flat-plate past stall ---
    const double AR = wing.aspectRatio();
    const double induced = (CL_linear * CL_linear) / (kPi * wing.oswald * AR);
    const double CD_linear = c.CD0 + induced;
    const double CD_flat = 2.0 * sa * sa;
    const double CD = (1.0 - sigma) * CD_linear + sigma * CD_flat;

    // --- Side force ---
    const double CY = c.CYbeta * beta + c.CYdr * dr;

    // --- Moments (body axis) ---
    const double Cl = c.Clbeta * beta + c.Clp * phat + c.Clr * rhat
                    + c.Clda * da + c.Cldr * dr;
    const double Cm = c.Cm0 + c.Cmalpha * alpha + c.Cmq * qhat + c.Cmde * de;
    const double Cn = c.Cnbeta * beta + c.Cnp * phat + c.Cnr * rhat
                    + c.Cnda * da + c.Cndr * dr;

    out.CL = CL; out.CD = CD; out.CY = CY;
    out.Cl = Cl; out.Cm = Cm; out.Cn = Cn;

    // --- Dimensional forces ---
    const double S = wing.area, b = wing.span, cbar = wing.chord;
    const double L = qbar * S * CL;   // lift
    const double D = qbar * S * CD;   // drag
    const double Yf = qbar * S * CY;  // side force

    // Wind-to-body rotation of (-D along wind x, Y along wind y, -L along wind z).
    const double cb = std::cos(beta), sb = std::sin(beta);
    Wrench wr;
    wr.force.x = -D * ca * cb - Yf * ca * sb + L * sa;
    wr.force.y = -D * sb      + Yf * cb;
    wr.force.z = -D * sa * cb - Yf * sa * sb - L * ca;

    // Moments about the CG.
    wr.moment.x = qbar * S * b    * Cl; // roll  (l)
    wr.moment.y = qbar * S * cbar * Cm; // pitch (m)
    wr.moment.z = qbar * S * b    * Cn; // yaw   (n)

    out.wrench = wr;
    return out;
}

} // namespace fsim
