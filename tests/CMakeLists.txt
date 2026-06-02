# tests/CMakeLists.txt - Test configuration

include(CTest)
include(GoogleTest)

# =============================================================================
# Original unit tests (existing tests)
# =============================================================================
add_executable(bp3d_tests
    test_main.cpp
    test_types.cpp
    test_rotation.cpp
    test_constraints.cpp
    test_algorithms.cpp
    test_json.cpp
)

target_link_libraries(bp3d_tests PRIVATE
    bp3d::bp3d
    GTest::gtest
    GTest::gmock
)

target_include_directories(bp3d_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# Apply sanitizers to tests
enable_sanitizers(bp3d_tests)

gtest_discover_tests(bp3d_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        LABELS "unit"
)

# =============================================================================
# Extended unit tests (internal utilities)
# =============================================================================
add_executable(bp3d_unit_tests
    test_main.cpp
    unit/test_geometry.cpp
    unit/test_free_space.cpp
    unit/test_sorting.cpp
    unit/test_obj_export.cpp
    unit/test_edge_cases.cpp
    unit/test_spatial_grid.cpp
)

target_link_libraries(bp3d_unit_tests PRIVATE
    bp3d::bp3d
    GTest::gtest
    GTest::gmock
)

target_include_directories(bp3d_unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

enable_sanitizers(bp3d_unit_tests)

gtest_discover_tests(bp3d_unit_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        LABELS "unit"
)

# =============================================================================
# Integration tests
# =============================================================================
add_executable(bp3d_integration_tests
    test_main.cpp
    integration/test_solver_integration.cpp
    integration/test_io_integration.cpp
)

target_link_libraries(bp3d_integration_tests PRIVATE
    bp3d::bp3d
    GTest::gtest
    GTest::gmock
)

target_include_directories(bp3d_integration_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

enable_sanitizers(bp3d_integration_tests)

gtest_discover_tests(bp3d_integration_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        LABELS "integration"
)

# =============================================================================
# System tests
# =============================================================================
add_executable(bp3d_system_tests
    test_main.cpp
    system/test_system.cpp
)

target_link_libraries(bp3d_system_tests PRIVATE
    bp3d::bp3d
    GTest::gtest
    GTest::gmock
)

target_include_directories(bp3d_system_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

enable_sanitizers(bp3d_system_tests)

gtest_discover_tests(bp3d_system_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        LABELS "system"
)

# =============================================================================
# Combined test target (runs all tests)
# =============================================================================
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS bp3d_tests bp3d_unit_tests bp3d_integration_tests bp3d_system_tests
    COMMENT "Running all tests"
)

# Copy test fixtures
file(COPY fixtures DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Exclude tests from clang-tidy: GoogleTest macros generate unavoidable noise,
# and the project gate (WarningsAsErrors) targets library and application code.
set_target_properties(
    bp3d_tests
    bp3d_unit_tests
    bp3d_integration_tests
    bp3d_system_tests
    PROPERTIES CXX_CLANG_TIDY ""
)
