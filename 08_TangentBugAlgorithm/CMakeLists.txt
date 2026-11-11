# Example 08: Tangent Bug navigation (Kobuki + Arduino + 3x HC-SR04).
#
# The decision core lives in 01_Sample/KobukiNativeVS/include/TangentBugPlanner.h
# (hardware independent). src/RobotIO.h provides two back-ends behind one
# interface: the real robot and a built-in simulator ("TangentBug.exe --sim"),
# so the same program runs with or without hardware. See USER_GUIDE.md.
add_executable(TangentBug
    src/main.cpp
    src/RobotIO.h)

target_link_libraries(TangentBug PRIVATE KobukiDriver)

set_target_properties(TangentBug PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(TangentBug PRIVATE /W4 /sdl /MP)
endif()
