// =============================================================================
//  core/Controls.hpp
//
//  Pilot control inputs, all normalised so the simulator is independent of any
//  particular hardware. Aerodynamic surfaces map these to physical deflections
//  using the per-aircraft maximum deflections from the data file.
// =============================================================================
#pragma once

namespace fsim {

struct ControlInputs {
    // Surface deflections follow the aerodynamic sign convention (positive =
    // trailing-edge down). The input layer (Phase 4) maps stick/pedal feel onto
    // these, e.g. stick-back -> negative elevator (nose-up).
    double aileron {0.0}; // [-1, 1]  (+ = right aileron TE down -> roll left)
    double elevator{0.0}; // [-1, 1]  (+ = TE down -> nose-down pitching moment)
    double rudder  {0.0}; // [-1, 1]  (+ = TE right -> nose-left yawing moment)
    double throttle{0.0}; // [ 0, 1]
    double flaps   {0.0}; // [ 0, 1]
    double brake   {0.0}; // [ 0, 1]  (wheel brakes)

    static double clampSym(double v)  { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }
    static double clampUnit(double v) { return v <  0.0 ?  0.0 : (v > 1.0 ? 1.0 : v); }

    void clamp() {
        aileron  = clampSym(aileron);
        elevator = clampSym(elevator);
        rudder   = clampSym(rudder);
        throttle = clampUnit(throttle);
        flaps    = clampUnit(flaps);
        brake    = clampUnit(brake);
    }
};

} // namespace fsim
