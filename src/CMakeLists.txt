# src/CMakeLists.txt
# Build the bp3d library

set(LIBRARY_SOURCES
    # Core
    logger.cpp
    
    # Algorithms
    algorithms/ffd.cpp
    algorithms/bfd.cpp
    algorithms/guillotine.cpp
    algorithms/extreme_point.cpp
    algorithms/shelf.cpp
    algorithms/parallel_solver.cpp
    
    # Constraints
    constraints/validator.cpp
    constraints/collision.cpp
    constraints/stacking.cpp
    constraints/weight.cpp
    
    # I/O
    io/json_serializer.cpp
    io/obj_exporter.cpp
    
    # Internal utilities
    internal/geometry.cpp
    internal/free_space.cpp
)

# Create library (static by default)
if(BP3D_BUILD_SHARED)
    add_library(bp3d SHARED ${LIBRARY_SOURCES})
    set_target_properties(bp3d PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
else()
    add_library(bp3d STATIC ${LIBRARY_SOURCES})
endif()

# Create alias for consistent usage
add_library(bp3d::bp3d ALIAS bp3d)

# Include directories
target_include_directories(bp3d
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/internal
)

# Link dependencies
target_link_libraries(bp3d
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        Threads::Threads
)

# Compiler features
target_compile_features(bp3d PUBLIC cxx_std_20)

# Apply warnings
set_project_warnings(bp3d)

# Apply sanitizers if enabled
enable_sanitizers(bp3d)

# Compiler definitions
target_compile_definitions(bp3d PRIVATE
    $<$<CONFIG:Debug>:BP3D_DEBUG>
)

# Position independent code (needed for shared library on some platforms)
set_target_properties(bp3d PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
