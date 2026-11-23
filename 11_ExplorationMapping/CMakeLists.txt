# Example 11: Exploration mapping - Bayesian occupancy grid + frontier based
# exploration with A* (Kobuki + Arduino + one front HC-SR04, 360 deg look-arounds).
#
# Shared cores in 01_Sample/KobukiNativeVS/include:
#   AStarPlanner.h  - grid map files, A*        PathFollower.h - turn and drive
#   GdiCanvas.h     - window / PNG              ArduinoTriSensor.h - sonar board
# src/BayesGrid.h: cell probabilities, Bayes rule, sensor model;
# src/Explorer.h: frontier exploration; src/ExploreRobot.h: robot + simulator;
# src/ExploreView.h: picture. See ExplorationMapping.md.
add_executable(Explore
    src/main.cpp
    src/BayesGrid.h
    src/Explorer.h
    src/ExploreRobot.h
    src/ExploreView.h)

target_link_libraries(Explore PRIVATE KobukiDriver gdiplus)

# Arena, sensor model and output live next to the sources; the simulator's
# true map is maps/truth.txt (the arena built in the lab).
target_compile_definitions(Explore PRIVATE
    EXPLORE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    LAB_MAP="${CMAKE_CURRENT_SOURCE_DIR}/maps/truth.txt")

set_target_properties(Explore PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Explore PRIVATE /W4 /sdl /MP)
endif()
