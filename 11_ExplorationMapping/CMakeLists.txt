# Example 11: Exploration mapping - Bayesian occupancy grid (Kobuki + Arduino +
# one front HC-SR04, 360 deg look-arounds), four exploration methods that are
# compared on the same obstacle maps:
#
#   11_01_Frontier  Explore_Frontier.exe   nearest frontier (A*)
#   11_02_BFS       Explore_BFS.exe        breadth-first order from the start
#   11_03_Random    Explore_Random.exe     random reachable goals + look-around
#   11_04_TSP       Explore_TSP.exe        travelling-salesman tour of the frontier
#   Launcher        ExploreLauncher.exe    Windows app: choose method, obstacle
#                                          map, COM ports; benchmark + comparison
#
# common/ holds everything the methods share (grid, sensor model, robot +
# simulator, window, calibration, metrics); a method only adds its Strategy.
# All executables go to one folder (<build>/11_ExplorationMapping/bin) so the
# launcher finds them next to itself.
#
# Build flexibility: each method can be switched off, e.g.
#   cmake --preset x64-release -DEXPLORE_BUILD_RANDOM=OFF
# and a single one can be built:  cmake --build --preset x64-release --target Explore_BFS

option(EXPLORE_BUILD_FRONTIER "Build 11_01_Frontier (Explore_Frontier.exe)" ON)
option(EXPLORE_BUILD_BFS      "Build 11_02_BFS (Explore_BFS.exe)" ON)
option(EXPLORE_BUILD_RANDOM   "Build 11_03_Random (Explore_Random.exe)" ON)
option(EXPLORE_BUILD_TSP      "Build 11_04_TSP (Explore_TSP.exe)" ON)
option(EXPLORE_BUILD_LAUNCHER "Build the Windows launcher (ExploreLauncher.exe)" ON)

set(EXPLORE_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/bin)

# Shared core (header only): include path, Kobuki driver, GDI+ for the PNGs,
# and where the maps / sensor model / outputs live.
add_library(ExploreCore INTERFACE)
target_include_directories(ExploreCore INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/common)
target_link_libraries(ExploreCore INTERFACE KobukiDriver gdiplus)
target_compile_definitions(ExploreCore INTERFACE EXPLORE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# add_explore_method(<target> <sources...>): one exploration method = one exe.
function(add_explore_method target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} PRIVATE ExploreCore)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        RUNTIME_OUTPUT_DIRECTORY ${EXPLORE_BIN_DIR})
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /sdl /MP)
    endif()
    set_property(GLOBAL APPEND PROPERTY EXPLORE_METHOD_TARGETS ${target})   # the launcher depends on them
endfunction()

if(EXPLORE_BUILD_FRONTIER)
    add_subdirectory(11_01_Frontier)
endif()
if(EXPLORE_BUILD_BFS)
    add_subdirectory(11_02_BFS)
endif()
if(EXPLORE_BUILD_RANDOM)
    add_subdirectory(11_03_Random)
endif()
if(EXPLORE_BUILD_TSP)
    add_subdirectory(11_04_TSP)
endif()
if(EXPLORE_BUILD_LAUNCHER)
    add_subdirectory(Launcher)
endif()
