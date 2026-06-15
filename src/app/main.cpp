// =============================================================================
//  app/main.cpp -- headless demonstration driver.
//
//  With no renderer yet (Milestone 3), this program exercises the Phase 1
//  physics core and prints a trajectory so the dynamics can be inspected and
//  validated by eye / piped to a plot. It sets up a simple ballistic case: a
//  body launched forward and upward under gravity alone, integrated with the
//  fixed-timestep RK4 driver. The parabolic arc that prints is produced purely
//  by integrating the equations of motion -- nothing is scripted.
// =============================================================================
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"
#include "core/Math.hpp"

#include <cstdio>
#include <memory>

using namespace fsim;

int main() {
    Simulation sim(1.0 / 240.0);

    // A 10 kg point-like body with unit inertia (rotation is irrelevant here).
    sim.setMassProperties(MassProperties::make(10.0, Mat3::inertia(1.0, 1.0, 1.0)));
    sim.addForceModel(std::make_shared<GravityForce>());

    // Launch: 60 m/s forward (north) and 60 m/s up. In NED, "up" is -Z.
    RigidBodyState s0;
    s0.positionWorld = Vec3{0.0, 0.0, 0.0};
    s0.velocityBody  = Vec3{60.0, 0.0, -60.0};
    sim.setState(s0);

    std::printf("# Phase 1 physics demo: ballistic trajectory (RK4, dt=1/240 s)\n");
    std::printf("# %8s %12s %12s %12s %12s\n",
                "t[s]", "north[m]", "altitude[m]", "vN[m/s]", "vUp[m/s]");

    const double printInterval = 0.5;
    double nextPrint = 0.0;

    // Run until the body returns to (or below) the launch altitude.
    while (sim.time() < 30.0) {
        if (sim.time() >= nextPrint) {
            const auto& s = sim.state();
            std::printf("  %8.2f %12.2f %12.2f %12.2f %12.2f\n",
                        sim.time(),
                        s.positionWorld.x,
                        -s.positionWorld.z,        // altitude = -down
                        s.velocityBody.x,
                        -s.velocityBody.z);        // upward speed
            nextPrint += printInterval;
        }
        // Stop once we have come back down past the start (after going up).
        if (sim.time() > 1.0 && sim.state().positionWorld.z >= 0.0) break;
        sim.advance(sim.fixedTimeStep());
    }

    const auto& s = sim.state();
    std::printf("\nImpact: t=%.2f s, range=%.1f m, impact speed=%.1f m/s\n",
                sim.time(), s.positionWorld.x, s.velocityBody.norm());
    return 0;
}
