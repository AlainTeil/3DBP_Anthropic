# Dependencies.cmake
# Fetches external dependencies using FetchContent

include(FetchContent)

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
