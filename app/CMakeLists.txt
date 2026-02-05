# app/CMakeLists.txt
# Build the bp3d CLI application

add_executable(bp3d-cli main.cpp)

target_link_libraries(bp3d-cli
    PRIVATE
        bp3d::bp3d
)

# Apply warnings to CLI as well
set_project_warnings(bp3d-cli)

# Apply sanitizers if enabled
enable_sanitizers(bp3d-cli)

# Set output name
set_target_properties(bp3d-cli PROPERTIES
    OUTPUT_NAME bp3d-cli
)
