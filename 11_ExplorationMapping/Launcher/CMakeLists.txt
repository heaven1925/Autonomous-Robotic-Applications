# ExploreLauncher.exe - Windows application for 11_ExplorationMapping:
#   1. simulator / real robot   2. Kobuki + Arduino COM ports with a connection
#   check (Probe.h)   3. sensor pins + 40 cm calibration (SensorLink.h,
#   common/RangeCalibration.h)   4. method (11_01..11_04) and obstacle map; the run's live
#   map is drawn in the launcher (Explore_*.exe --ipc, LiveRun.h); benchmark and
#   comparison of the methods (results/results.csv).
# It starts the Explore_*.exe programs, so building it builds them too.
add_executable(ExploreLauncher WIN32
    src/main.cpp
    src/Theme.h
    src/LiveRun.h
    src/Probe.h
    src/ComPorts.h
    src/Results.h
    src/SensorLink.h
    src/Replay.h
    src/Teleop.h
    src/ArduinoCli.h
    src/MapFile.h
    src/Panels.h
    src/Tabs.h
    ../common/RangeCalibration.h)

target_include_directories(ExploreLauncher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../common   # RangeCalibration.h (shared with Explore_*.exe)
    ${CMAKE_SOURCE_DIR}/01_Sample/KobukiNativeVS/include)   # AStarPlanner.h (map files), GdiCanvas.h (PNG)

# KobukiDriver: KobukiProtocol (recognise Kobuki packets in the connection check)
target_link_libraries(ExploreLauncher PRIVATE KobukiDriver setupapi gdiplus shell32)
get_filename_component(explore_dir ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)   # 11_ExplorationMapping: maps, results, output
target_compile_definitions(ExploreLauncher PRIVATE UNICODE _UNICODE EXPLORE_DIR="${explore_dir}")

set_target_properties(ExploreLauncher PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY ${EXPLORE_BIN_DIR})

if(MSVC)
    target_compile_options(ExploreLauncher PRIVATE /W4 /sdl /MP /utf-8)
endif()

get_property(explore_methods GLOBAL PROPERTY EXPLORE_METHOD_TARGETS)
if(explore_methods)
    add_dependencies(ExploreLauncher ${explore_methods})
endif()
