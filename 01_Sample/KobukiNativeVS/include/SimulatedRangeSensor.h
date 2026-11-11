// SimulatedRangeSensor.h - ray-casts a fixed obstacle layout into a RangeScan.
//
// Stands in for a real distance-sensor driver (2D LiDAR, ultrasonic array...)
// while none is attached yet. It only needs the robot's live pose (from
// Kobuki odometry/gyro) and a list of obstacles - it knows nothing about
// Kobuki itself. Replacing this with a real sensor driver later means
// writing a new file with the same job (produce a RangeScan) and changing
// one call in main.cpp; FollowTheGap.h and the robot motion loop stay as-is.
#pragma once

#include "RangeScan.h"

#include <cmath>
#include <variant>
#include <vector>

namespace ftg {

struct Pose2D {
    double x = 0.0, y = 0.0, theta = 0.0; // metres, metres, radians (world frame)
};

struct CircleObstacle   { double x, y, radius; };
struct SegmentObstacle  { double x1, y1, x2, y2; };
using Obstacle = std::variant<CircleObstacle, SegmentObstacle>;

struct SimScanParams {
    double angle_min = -120.0 * (3.14159265358979323846 / 180.0); // rad, robot-relative
    double angle_max =  120.0 * (3.14159265358979323846 / 180.0); // rad, robot-relative
    int    samples   = 241;   // 1 degree resolution over a 240 deg field of view
    double max_range = 4.0;   // m
};

namespace detail {

// Ray P = O + t*D (t >= 0) against segment Q = A + u*(B-A) (u in [0,1]).
// Returns t of the closest intersection, or -1 if there is none ahead.
inline double raySegmentDistance(double ox, double oy, double dx, double dy,
                                  double x1, double y1, double x2, double y2) {
    const double ex = x2 - x1, ey = y2 - y1;
    const double denom = dx * ey - dy * ex;
    if (std::abs(denom) < 1e-9) return -1.0; // parallel
    const double t = ((x1 - ox) * ey - (y1 - oy) * ex) / denom;
    const double u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) return t;
    return -1.0;
}

// Ray against a circle of radius r centred at (cx, cy). (dx, dy) must be
// a unit vector. Returns the distance to the nearest intersection ahead,
// or -1 if the ray misses the circle or it is entirely behind the origin.
inline double rayCircleDistance(double ox, double oy, double dx, double dy,
                                 double cx, double cy, double r) {
    const double fx = ox - cx, fy = oy - cy;
    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - r * r;
    const double disc = b * b - 4.0 * c;
    if (disc < 0.0) return -1.0;
    const double sq = std::sqrt(disc);
    const double t1 = (-b - sq) / 2.0;
    const double t2 = (-b + sq) / 2.0;
    if (t1 >= 0.0) return t1;
    if (t2 >= 0.0) return t2;
    return -1.0;
}

} // namespace detail

// Builds a RangeScan by ray-casting `obstacles` from the robot's current
// pose. This is the one function a real sensor driver would replace.
inline RangeScan simulateScan(const Pose2D& pose, const std::vector<Obstacle>& obstacles,
                               const SimScanParams& params = {}) {
    RangeScan scan;
    scan.angle_min = params.angle_min;
    scan.angle_max = params.angle_max;
    scan.max_range = params.max_range;
    scan.ranges.assign(static_cast<std::size_t>(params.samples), params.max_range);

    const double step = scan.angleStep();
    for (int i = 0; i < params.samples; ++i) {
        const double bearing = pose.theta + scan.angle_min + step * static_cast<double>(i);
        const double dx = std::cos(bearing), dy = std::sin(bearing);
        double best = params.max_range;
        for (const auto& obstacle : obstacles) {
            double hit = -1.0;
            if (const auto* c = std::get_if<CircleObstacle>(&obstacle))
                hit = detail::rayCircleDistance(pose.x, pose.y, dx, dy, c->x, c->y, c->radius);
            else if (const auto* s = std::get_if<SegmentObstacle>(&obstacle))
                hit = detail::raySegmentDistance(pose.x, pose.y, dx, dy, s->x1, s->y1, s->x2, s->y2);
            if (hit >= 0.0 && hit < best) best = hit;
        }
        scan.ranges[static_cast<std::size_t>(i)] = best;
    }
    return scan;
}

} // namespace ftg
