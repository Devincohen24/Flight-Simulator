cmake_minimum_required(VERSION 3.16)
project(FlightSimulator
        VERSION 0.1.0
        DESCRIPTION "Professional-grade flight simulator (C++/OpenGL)"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Warnings: treat the physics core strictly.
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(FSIM_BUILD_TESTS    "Build the headless physics/math tests" ON)
option(FSIM_BUILD_RENDERER "Build the OpenGL renderer (Milestone 3+)" OFF)

# ---------------------------------------------------------------------------
# fsim_core : headless simulation library (no graphics dependencies)
#   This is the foundation. It must build and run without any GPU/windowing
#   so the physics can be validated in CI / headless containers.
# ---------------------------------------------------------------------------
add_library(fsim_core
    src/core/Json.cpp
    src/physics/Atmosphere.cpp
    src/physics/RigidBody.cpp
    src/physics/Simulation.cpp
    src/aircraft/Aircraft.cpp
    src/aircraft/Trim.cpp
    src/aero/Aerodynamics.cpp
    src/propulsion/Propulsion.cpp
    src/ground/LandingGear.cpp
    src/terrain/Terrain.cpp
    src/terrain/Quadtree.cpp
    src/render/Camera.cpp
    src/systems/Engine.cpp
    src/systems/Instruments.cpp
    src/systems/Navigation.cpp
)
target_include_directories(fsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---------------------------------------------------------------------------
# Headless demo: integrates an aircraft state and prints a trajectory.
# ---------------------------------------------------------------------------
add_executable(fsim_headless src/app/main.cpp)
target_link_libraries(fsim_headless PRIVATE fsim_core)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(FSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Renderer (Milestone 3) — OpenGL 3.3 + GLFW. OFF by default so the headless
# foundation always builds with no GPU/windowing dependencies. Enable with
#   cmake -S . -B build -DFSIM_BUILD_RENDERER=ON
# GLFW and glad are fetched automatically (requires network on first config).
# ---------------------------------------------------------------------------
if(FSIM_BUILD_RENDERER)
    message(STATUS "Renderer enabled: fetching GLFW + glad and building fsim_viewer.")
    enable_language(C) # glad generates a C loader library
    include(FetchContent)

    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    # Default to the X11 backend; enabling Wayland needs wayland-scanner and the
    # wayland/xkbcommon dev packages. Override on the command line if desired.
    option(FSIM_GLFW_WAYLAND "Build the GLFW Wayland backend" OFF)
    if(NOT FSIM_GLFW_WAYLAND)
        set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    endif()
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4)
    FetchContent_Declare(glad
        GIT_REPOSITORY https://github.com/Dav1dde/glad.git
        GIT_TAG        v2.0.6
        SOURCE_SUBDIR  cmake)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.5)
    FetchContent_MakeAvailable(glfw glad imgui)

    # Generate an OpenGL 3.3 core loader target named glad_gl_core_33.
    glad_add_library(glad_gl_core_33 STATIC API gl:core=3.3)

    find_package(OpenGL REQUIRED)

    # Dear ImGui (no upstream CMake): build the core + GLFW/OpenGL3 backends.
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

    add_library(fsim_render
        src/render/gl/Shader.cpp
        src/render/gl/Renderer.cpp
        src/render/gl/MeshBuilders.cpp
    )
    target_include_directories(fsim_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(fsim_render
        PUBLIC fsim_core glad_gl_core_33 glfw OpenGL::GL)

    add_executable(fsim_viewer src/app/viewer.cpp)
    target_link_libraries(fsim_viewer PRIVATE fsim_render imgui)
endif()
