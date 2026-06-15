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
    src/physics/Atmosphere.cpp
    src/physics/RigidBody.cpp
    src/physics/Simulation.cpp
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
# Renderer (added at Milestone 3 — requires GLFW/OpenGL). Off by default so
# the headless foundation always builds.
# ---------------------------------------------------------------------------
if(FSIM_BUILD_RENDERER)
    message(STATUS "Renderer build requested (not yet implemented).")
endif()
