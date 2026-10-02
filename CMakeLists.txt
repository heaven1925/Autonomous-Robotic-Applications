cmake_minimum_required(VERSION 3.21)

project(AutonRobotSys
    DESCRIPTION "Autonomous robot systems workspace (Kobuki)"
    LANGUAGES CXX)

# Automatically add every top-level folder that contains a CMakeLists.txt.
# Adding a new project (e.g. 02_MyProject/) requires no changes here.
file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child}
       AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${child}/CMakeLists.txt)
        message(STATUS "Project: ${child}")
        add_subdirectory(${child})
    endif()
endforeach()
