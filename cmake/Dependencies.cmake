# Dependencies.cmake
# Fetches external dependencies using FetchContent

include(FetchContent)

# Third-party sources must not be analyzed by clang-tidy: they generate
# thousands of findings and our gate treats warnings as errors. Disable any
# globally-configured analyzer while the dependencies are made available, then
# restore it so our own targets are still checked.
set(_bp3d_saved_clang_tidy "${CMAKE_CXX_CLANG_TIDY}")
set(CMAKE_CXX_CLANG_TIDY "")

# nlohmann/json - JSON for Modern C++
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)

message(STATUS "Fetching nlohmann/json...")
FetchContent_MakeAvailable(nlohmann_json)

# GoogleTest - Testing framework (only if tests are enabled)
if(BP3D_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        GIT_SHALLOW    TRUE
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    
    # Disable installation of GTest
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

    message(STATUS "Fetching GoogleTest...")
    FetchContent_MakeAvailable(googletest)
endif()

# Restore the analyzer configuration for the project's own targets.
set(CMAKE_CXX_CLANG_TIDY "${_bp3d_saved_clang_tidy}")
unset(_bp3d_saved_clang_tidy)
