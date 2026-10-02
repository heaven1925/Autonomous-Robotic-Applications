cmake_minimum_required(VERSION 3.21)

project(KobukiNative
    VERSION 1.0.0
    DESCRIPTION "Native Windows Kobuki driver demo (no ROS/ECL)"
    LANGUAGES CXX)

# Reusable driver library (serial port + protocol + Kobuki API).
add_library(KobukiDriver STATIC
    KobukiNativeVS/src/Kobuki.cpp
    KobukiNativeVS/src/KobukiProtocol.cpp
    KobukiNativeVS/src/SerialPort.cpp)

target_include_directories(KobukiDriver PUBLIC
    KobukiNativeVS/include)

set_target_properties(KobukiDriver PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_compile_definitions(KobukiDriver PUBLIC UNICODE _UNICODE)

if(MSVC)
    target_compile_options(KobukiDriver PRIVATE /W4 /sdl /MP)
endif()

# Interactive teleop demo.
add_executable(KobukiNativeVS
    KobukiNativeVS/src/main.cpp)

target_link_libraries(KobukiNativeVS PRIVATE KobukiDriver)

set_target_properties(KobukiNativeVS PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(KobukiNativeVS PRIVATE /W4 /sdl /MP)
endif()
