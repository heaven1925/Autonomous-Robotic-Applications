# Example 02: forward 2 m, spin 360 deg, return to the initial position.
add_executable(Example02
    src/main.cpp)

target_link_libraries(Example02 PRIVATE KobukiDriver)

set_target_properties(Example02 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Example02 PRIVATE /W4 /sdl /MP)
endif()
