// NavCommon.h - small types shared by the grid-navigation examples
// (10_AStarPathPlanning, 11_ExplorationMapping): angles, the robot pose in the
// map frame of AStarPlanner.h and the bumper state.
//
// Map frame: metres, origin at the bottom-left corner of the map, +x right,
// +y up, heading 0 = +x, counter-clockwise positive.
#pragma once

#include "AStarPlanner.h"

namespace nav {

constexpr double Pi = astar::Pi;
constexpr double Deg = Pi / 180.0;

inline double normalizeAngle(double a) {
    while (a > Pi) a -= 2.0 * Pi;
    while (a <= -Pi) a += 2.0 * Pi;
    return a;
}

struct Pose {
    double x = 0.0, y = 0.0, heading = 0.0;  // map frame
    astar::Point point() const { return { x, y }; }
};

enum class Bumper { None, Left, Center, Right };

} // namespace nav
