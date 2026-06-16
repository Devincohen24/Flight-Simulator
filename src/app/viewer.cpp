// =============================================================================
//  app/viewer.cpp -- interactive OpenGL flight viewer (Milestone 3 app).
//
//  Wires the headless simulation to the GL renderer:
//    * loads the data-driven Cessna 172 and trims it for level cruise,
//    * runs the physics at a FIXED 240 Hz with an accumulator,
//    * interpolates between physics states for smooth, frame-rate-independent
//      rendering,
//    * drives controls from the keyboard and switches camera modes,
//    * flies over procedural terrain that the landing gear also collides with.
//
//  Built only when FSIM_BUILD_RENDERER=ON (needs GLFW + OpenGL 3.3).
//
//  Controls:
//    Up/Down    elevator (pitch)      Left/Right  ailerons (roll)
//    Q/E        rudder (yaw)          W/S         throttle up/down
//    F/G        flaps down/up         B (hold)    wheel brakes
//    1/2/3/4    chase/cockpit/orbit/flyby camera  R  reset to trim
//    Esc        quit
// =============================================================================
#include "render/gl/Renderer.hpp"
#include "render/Camera.hpp"
#include "render/RenderState.hpp"
#include "physics/Simulation.hpp"
#include "physics/ForceModel.hpp"
#include "aircraft/Aircraft.hpp"
#include "aircraft/Trim.hpp"
#include "aero/Aerodynamics.hpp"
#include "propulsion/Propulsion.hpp"
#include "ground/LandingGear.hpp"
#include "terrain/Terrain.hpp"
#include "systems/Engine.hpp"
#include "systems/Instruments.hpp"
#include "environment/Weather.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace fsim;

namespace {

struct App {
    Simulation sim{1.0 / 240.0};
    ControlInputs controls;
    Camera camera;
    RigidBodyState prev, curr;
    TrimResult trim;
    EngineSystem* engine{nullptr};
    Vec3 wind{};
};

// Draw the instrument overlay (Dear ImGui) from the live simulation state.
void drawHud(const App& app, const gl::Renderer& renderer) {
    const EnvironmentSample env = app.sim.environment();
    const InstrumentReadings r = computeInstruments(
        app.sim.state(), env, app.engine->state(), app.sim.specificForceBody());

    const char* camName[] = {"Chase", "Cockpit", "Orbit", "Flyby"};

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("Instruments", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("IAS  %6.1f kt   (TAS %5.1f)", r.indicatedAirspeed * 1.94384,
                r.trueAirspeed * 1.94384);
    ImGui::Text("ALT  %6.0f ft   VS %+6.0f fpm", r.altitudeMSL * 3.28084,
                r.verticalSpeed * 196.85);
    ImGui::Text("HDG  %6.0f deg", r.heading);
    ImGui::Text("PITCH %+5.1f   BANK %+5.1f", r.pitch, r.bank);
    ImGui::Text("AoA  %5.1f deg  SLIP %+4.2f", r.angleOfAttack, r.slip);
    ImGui::Text("LOAD %5.2f g   TURN %+5.1f deg/s", r.loadFactor, r.turnRate);
    ImGui::Separator();
    ImGui::Text("RPM  %6.0f   MP %4.1f inHg", r.rpm, r.manifoldPressure);
    ImGui::Text("FUEL %6.1f kg  FLOW %.3f kg/s", r.fuelRemaining, r.fuelFlow);
    ImGui::Separator();
    ImGui::Text("THR %3.0f%%  FLAP %3.0f%%", app.controls.throttle * 100.0,
                app.controls.flaps * 100.0);
    ImGui::Text("WIND %5.1f m/s", app.wind.norm());
    ImGui::Text("Terrain chunks %d/%d drawn", renderer.lastDrawnChunks(),
                renderer.lastChunkCount());
    ImGui::Text("Camera: %s  [1-4]  R=reset", camName[int(app.camera.mode)]);
    ImGui::End();
}

App* g_app = nullptr;

void resetToTrim(App& app) {
    app.sim.setState(app.trim.state);
    app.controls = app.trim.controls;
    app.prev = app.curr = app.trim.state;
}

void keyCallback(GLFWwindow* win, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    App& app = *g_app;
    switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(win, GLFW_TRUE); break;
        case GLFW_KEY_1: app.camera.mode = CameraMode::Chase;   break;
        case GLFW_KEY_2: app.camera.mode = CameraMode::Cockpit; break;
        case GLFW_KEY_3: app.camera.mode = CameraMode::Orbit;   break;
        case GLFW_KEY_4: app.camera.mode = CameraMode::Flyby;   break;
        case GLFW_KEY_R: resetToTrim(app); break;
        default: break;
    }
}

void framebufferSizeCallback(GLFWwindow*, int w, int h);
gl::Renderer* g_renderer = nullptr;
void framebufferSizeCallback(GLFWwindow*, int w, int h) {
    if (g_renderer) g_renderer->resize(w, h);
}

// Poll continuous (held-key) control inputs and update the control state.
void pollControls(GLFWwindow* win, App& app, double dt) {
    ControlInputs& c = app.controls;
    const double rate = 1.5 * dt; // surface slew per second

    auto down = [&](int k) { return glfwGetKey(win, k) == GLFW_PRESS; };

    // Elevator: Up arrow = nose up = negative deflection (TE up).
    if (down(GLFW_KEY_UP))    c.elevator -= rate;
    if (down(GLFW_KEY_DOWN))  c.elevator += rate;
    // Ailerons.
    if (down(GLFW_KEY_LEFT))  c.aileron  -= rate;
    if (down(GLFW_KEY_RIGHT)) c.aileron  += rate;
    // Rudder.
    if (down(GLFW_KEY_E))     c.rudder   += rate;
    if (down(GLFW_KEY_Q))     c.rudder   -= rate;
    // Throttle.
    if (down(GLFW_KEY_W))     c.throttle += 0.4 * dt;
    if (down(GLFW_KEY_S))     c.throttle -= 0.4 * dt;
    // Flaps.
    if (down(GLFW_KEY_F))     c.flaps    += 0.3 * dt;
    if (down(GLFW_KEY_G))     c.flaps    -= 0.3 * dt;
    // Brakes (momentary).
    c.brake = down(GLFW_KEY_B) ? 1.0 : 0.0;

    // Self-centre the elevator/aileron/rudder slightly when released.
    if (!down(GLFW_KEY_UP) && !down(GLFW_KEY_DOWN))
        c.elevator += (app.trim.controls.elevator - c.elevator) * 2.0 * dt;
    if (!down(GLFW_KEY_LEFT) && !down(GLFW_KEY_RIGHT))
        c.aileron -= c.aileron * 2.0 * dt;
    if (!down(GLFW_KEY_Q) && !down(GLFW_KEY_E))
        c.rudder -= c.rudder * 2.0 * dt;

    c.clamp();
    app.sim.setControls(c);
}

} // namespace

int main(int argc, char** argv) {
    const std::string acPath = (argc > 1) ? argv[1]
                                          : "data/aircraft/cessna172.json";

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialise GLFW\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Flight Simulator", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (!gladLoadGL(glfwGetProcAddress)) {
        std::fprintf(stderr, "Failed to load OpenGL via glad\n");
        glfwTerminate();
        return 1;
    }

    // --- Simulation setup ---
    Aircraft ac;
    try {
        ac = Aircraft::load(acPath);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load aircraft '%s': %s\n", acPath.c_str(), e.what());
        glfwTerminate();
        return 1;
    }

    auto terrain = std::make_shared<ProceduralTerrain>();

    App app;
    app.sim.setMassProperties(ac.mass);
    app.sim.addForceModel(std::make_shared<AerodynamicsForce>(ac));
    app.sim.addForceModel(std::make_shared<PropulsionForce>(ac.propulsion));
    app.sim.addForceModel(std::make_shared<GravityForce>());
    app.sim.addForceModel(std::make_shared<LandingGearForce>(ac.gear, terrain.get()));

    EngineSystem engine(ac.propulsion);
    engine.start();
    app.engine = &engine;

    // Weather: a westerly breeze with light turbulence.
    WeatherSystem weather;
    weather.wind().addLayerFromHeading(0.0,    270.0, 6.0);
    weather.wind().addLayerFromHeading(3000.0, 270.0, 12.0);
    weather.turbulence().setParams(DrydenTurbulence::light());

    app.trim = trimLevelFlight(ac, 55.0, 800.0);
    resetToTrim(app);
    app.camera.mode = CameraMode::Chase;

    gl::Renderer renderer;
    if (!renderer.init(terrain.get())) {
        std::fprintf(stderr, "Renderer init failed\n");
        glfwTerminate();
        return 1;
    }
    int fbw = 1280, fbh = 720;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    renderer.resize(fbw, fbh);

    g_app = &app;
    g_renderer = &renderer;
    glfwSetKeyCallback(window, keyCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Dear ImGui instrument overlay (chains to the callbacks set above).
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // --- Main loop: fixed-step physics + interpolated rendering ---
    const double dt = app.sim.fixedTimeStep();
    double accumulator = 0.0;
    auto last = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        const auto now = std::chrono::high_resolution_clock::now();
        double frame = std::chrono::duration<double>(now - last).count();
        last = now;
        if (frame > 0.25) frame = 0.25; // avoid spiral-of-death after a stall
        accumulator += frame;

        glfwPollEvents();
        pollControls(window, app, frame);

        while (accumulator >= dt) {
            const EnvironmentSample env = app.sim.environment();
            engine.update(dt, app.controls.throttle,
                          env.atmosphere.density / 1.225, env.atmosphere.pressure);
            const double tas = app.sim.state().velocityBody.norm();
            app.wind = weather.update(dt, -app.sim.state().positionWorld.z, tas);
            app.sim.setWind(app.wind);
            app.prev = app.sim.state();
            app.sim.step();
            app.curr = app.sim.state();
            accumulator -= dt;
        }

        // Begin the ImGui frame and build the HUD.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawHud(app, renderer);
        ImGui::Render();

        // Draw the 3D scene, then the overlay.
        const double alpha = accumulator / dt;
        const RenderState rs = interpolateState(app.prev, app.curr, alpha);
        app.camera.update(rs);
        renderer.renderFrame(app.camera, rs);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}
