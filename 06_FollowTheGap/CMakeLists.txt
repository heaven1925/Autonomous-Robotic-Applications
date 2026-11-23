# Example 06: Follow the Gap obstacle avoidance (simulated range sensor).
add_executable(FollowTheGap
    src/main.cpp)

target_link_libraries(FollowTheGap PRIVATE KobukiDriver)

set_target_properties(FollowTheGap PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(FollowTheGap PRIVATE /W4 /sdl /MP)
endif()
