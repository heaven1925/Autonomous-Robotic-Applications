# Example 03: waypoint navigation (0,0) -> (10,10) -> (0,10).
add_executable(Example03
    src/main.cpp)

target_link_libraries(Example03 PRIVATE KobukiDriver)

set_target_properties(Example03 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Example03 PRIVATE /W4 /sdl /MP)
endif()
