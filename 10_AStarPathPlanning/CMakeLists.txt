# Example 10: A* path planning on a 0/1 grid map from a text file, keeping a
# robot-radius clearance, then driving the Kobuki along the path.
#
# The hardware independent core lives in 01_Sample/KobukiNativeVS/include:
#   AStarPlanner.h - map file reader, robot-radius inflation, A*, path checks
#   GdiCanvas.h    - off-screen drawing, window, PNG export (shared with 09)
#   PathFollower.h - turn on the spot, then drive straight along each leg
# src/Robot.h: real Kobuki + built-in simulator; src/PlanView.h: the picture.
# See AStar.md.
add_executable(AStar
    src/main.cpp
    src/Robot.h
    src/PlanView.h)

# gdiplus: PNG encoder for the saved pictures (part of Windows).
target_link_libraries(AStar PRIVATE KobukiDriver gdiplus)

# Default map and output folder are next to the sources.
target_compile_definitions(AStar PRIVATE
    ASTAR_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/maps"
    ASTAR_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/output")

set_target_properties(AStar PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(AStar PRIVATE /W4 /sdl /MP)
endif()
