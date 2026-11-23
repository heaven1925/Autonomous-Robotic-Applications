# Example 09: Mapping - probabilistic occupancy grid + frontier exploration
# (Kobuki + Arduino + 3x HC-SR04).
#
# The hardware independent cores live in 01_Sample/KobukiNativeVS/include:
#   OccupancyGrid.h    - probability grid, sensor model, Bayes update
#   FrontierExplorer.h - frontier detection, path planning, exploration
# src/RobotIO.h provides the real robot and a built-in simulator of the
# assignment floor plan ("Mapping.exe --sim"); src/MapView.h draws the live
# map window and saves the PNG files. See Mapping.md.
add_executable(Mapping
    src/main.cpp
    src/RobotIO.h
    src/MapView.h)

# gdiplus: PNG encoder for the saved maps (part of Windows).
target_link_libraries(Mapping PRIVATE KobukiDriver gdiplus)

# Maps are saved next to the sources unless --out says otherwise.
target_compile_definitions(Mapping PRIVATE
    MAPPING_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/output")

set_target_properties(Mapping PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Mapping PRIVATE /W4 /sdl /MP)
endif()
