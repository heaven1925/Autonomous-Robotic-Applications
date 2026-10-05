# Example 04: waypoint navigation with bumper obstacle avoidance.
add_executable(Example04
    src/main.cpp)

target_link_libraries(Example04 PRIVATE KobukiDriver)

set_target_properties(Example04 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Example04 PRIVATE /W4 /sdl /MP)
endif()
