# Headless physics/math test suite. Each file is an independent executable
# registered with CTest so `ctest` runs them all.
set(FSIM_TESTS
    test_math
    test_atmosphere
    test_rigidbody
    test_json
    test_aircraft
    test_aero
    test_trim
    test_gear
    test_mat4
    test_render
    test_terrain
    test_systems
    test_navigation
)

foreach(test_name ${FSIM_TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE fsim_core)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    # Absolute path to the repo root so tests can locate data/ regardless of cwd.
    target_compile_definitions(${test_name} PRIVATE
        FSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
