// =============================================================================
//  aero/Aerodynamics.hpp
//
//  Aerodynamic force/moment model. Given the aircraft state, the air-relative
//  velocity, and the control deflections, it produces the body-frame forces and
//  moments using the standard linearised stability-derivative model plus a
//  non-linear stall blend so that stall and post-stall behaviour EMERGE rather
//  than being scripted.
//
//  Angle of attack and sideslip from the body-frame air-relative velocity
//  (u,v,w):
//        V     = |v_air|
//        alpha = atan2(w, u)
//        beta  = asin(v / V)
//  Dynamic pressure:
//        q_bar = 1/2 rho V^2
//  Non-dimensional body rates (b = span, c = chord):
//        p_hat = p b / (2V),  q_hat = q c / (2V),  r_hat = r b / (2V)
//
//  Coefficient build-up (per-radian derivatives):
//        CL = CL0 + CLa*alpha + CLq*q_hat + CLde*de + CLflap*df   (linear region)
//        CD = CD0 + CL_lin^2 / (pi e AR)                           (drag polar)
//        CY = CYb*beta + CYdr*dr
//        Cl = Clb*beta + Clp*p_hat + Clr*r_hat + Clda*da + Cldr*dr
//        Cm = Cm0 + Cma*alpha + Cmq*q_hat + Cmde*de
//        Cn = Cnb*beta + Cnp*p_hat + Cnr*r_hat + Cnda*da + Cndr*dr
//  A sigmoid blends CL/CD into a flat-plate model (CL=2 sa^2 ca, CD=2 sa^2)
//  beyond the stall angle (Beard & McLain formulation).
//
//  Forces are computed as lift/drag/side in wind axes and rotated into the body
//  frame via the alpha/beta wind-to-body DCM. Moments are already body-axis.
// =============================================================================
#pragma once

#include "aircraft/Aircraft.hpp"
#include "physics/ForceModel.hpp"

namespace fsim {

// Diagnostic breakdown of an aerodynamic evaluation (handy for HUD and tests).
struct AeroState {
    double airspeed{0.0};   // V, m/s (true air-relative)
    double alpha{0.0};      // angle of attack, rad
    double beta{0.0};       // sideslip angle, rad
    double dynamicPressure{0.0}; // q_bar, Pa
    double CL{0.0}, CD{0.0}, CY{0.0};
    double Cl{0.0}, Cm{0.0}, Cn{0.0};
    Wrench wrench{};        // resulting body-frame force & moment
};

// Compute the full aerodynamic breakdown. Exposed (not just the wrench) so the
// stall model, trim search, and unit tests can inspect intermediate quantities.
AeroState computeAerodynamics(const Aircraft& ac,
                              const RigidBodyState& state,
                              const EnvironmentSample& env,
                              const ControlInputs& controls);

// ForceModel wrapper so aerodynamics plugs into the CompositeForceModel.
class AerodynamicsForce final : public ForceModel {
public:
    explicit AerodynamicsForce(const Aircraft& ac) : aircraft_(ac) {}

    Wrench evaluate(const RigidBodyState& state,
                    const MassProperties& /*mass*/,
                    const EnvironmentSample& env,
                    const ControlInputs& controls) const override {
        return computeAerodynamics(aircraft_, state, env, controls).wrench;
    }
    const char* name() const override { return "aerodynamics"; }

private:
    Aircraft aircraft_;
};

} // namespace fsim
