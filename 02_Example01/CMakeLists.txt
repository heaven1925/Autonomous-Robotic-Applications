# Example 01: drive a 1 m square, stop at the initial position.
add_executable(Example01
    src/main.cpp)

target_link_libraries(Example01 PRIVATE KobukiDriver)

set_target_properties(Example01 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(Example01 PRIVATE /W4 /sdl /MP)
endif()
