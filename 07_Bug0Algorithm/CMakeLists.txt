# Example 07: Bug0 obstacle avoidance (Kobuki + Arduino + 3x HC-SR04).
#
# Two executables:
#   Bug0      - real hardware runner (Kobuki + Arduino serial ports).
#   Bug0Sim   - pure-software simulation/test harness, no hardware needed;
#               run this first (and after any planner change) to validate
#               the Bug0Planner decision logic before touching the robot.
add_executable(Bug0
    src/main.cpp)

target_link_libraries(Bug0 PRIVATE KobukiDriver)

set_target_properties(Bug0 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Bug0 PRIVATE /W4 /sdl /MP)
endif()

add_executable(Bug0Sim
    src/sim_main.cpp)

target_link_libraries(Bug0Sim PRIVATE KobukiDriver)

set_target_properties(Bug0Sim PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Bug0Sim PRIVATE /W4 /sdl /MP)
endif()
