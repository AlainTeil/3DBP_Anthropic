cmake_minimum_required(VERSION 3.20)

project(bp3d
    VERSION 1.0.0
    LANGUAGES CXX
    DESCRIPTION "High-performance 3D bin packing library"
)

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Please create a build directory.")
endif()

# C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BP3D_BUILD_TESTS "Build unit tests" ON)
option(BP3D_BUILD_CLI "Build CLI application" ON)
option(BP3D_BUILD_DOCS "Build documentation" OFF)
option(BP3D_BUILD_EXAMPLES "Build examples" OFF)
option(BP3D_ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
option(BP3D_BUILD_SHARED "Build shared library instead of static" OFF)
option(BP3D_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(BP3D_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(BP3D_ENABLE_TSAN "Enable ThreadSanitizer" OFF)

# Include CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(StaticAnalyzers)
include(Dependencies)
include(Sanitizers)

# Find threading library
find_package(Threads REQUIRED)

# Add subdirectories
add_subdirectory(src)

if(BP3D_BUILD_CLI)
    add_subdirectory(app)
endif()

if(BP3D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BP3D_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(BP3D_BUILD_DOCS)
    include(Doxygen)
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "bp3d configuration summary:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build tests:    ${BP3D_BUILD_TESTS}")
message(STATUS "  Build CLI:      ${BP3D_BUILD_CLI}")
message(STATUS "  Build docs:     ${BP3D_BUILD_DOCS}")
message(STATUS "  Clang-tidy:     ${BP3D_ENABLE_CLANG_TIDY}")
message(STATUS "  AddressSan:     ${BP3D_ENABLE_ASAN}")
message(STATUS "  UBSan:          ${BP3D_ENABLE_UBSAN}")
message(STATUS "  ThreadSan:      ${BP3D_ENABLE_TSAN}")
message(STATUS "  Shared lib:     ${BP3D_BUILD_SHARED}")
message(STATUS "")
