// RangeScan.h - platform-independent angular range-scan data structure.
//
// This is the common format every distance-sensor driver (simulated or
// real: 2D LiDAR, ultrasonic array, depth camera...) must produce. The
// Follow-the-Gap algorithm (FollowTheGap.h) only ever talks to this type,
// so swapping a simulated sensor for a real one later never touches the
// algorithm or the robot motion code.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ftg {

constexpr double Pi = 3.14159265358979323846;

inline double normalize(double a) {
    while (a > Pi)  a -= 2.0 * Pi;
    while (a <= -Pi) a += 2.0 * Pi;
    return a;
}

// Evenly spaced distance samples from angle_min to angle_max, robot-relative
// (radians, 0 = straight ahead, + = left / counter-clockwise).
struct RangeScan {
    double angle_min = 0.0;      // rad
    double angle_max = 0.0;      // rad
    double max_range = 0.0;      // m - value reported where nothing was detected
    std::vector<double> ranges;  // m, ranges[0] at angle_min ... ranges[N-1] at angle_max

    double angleStep() const {
        return ranges.size() > 1
            ? (angle_max - angle_min) / static_cast<double>(ranges.size() - 1)
            : 0.0;
    }

    double angleAt(std::size_t i) const { return angle_min + angleStep() * static_cast<double>(i); }

    // Nearest sample index for a given bearing (clamped to the scan's coverage).
    std::size_t indexAt(double angle) const {
        if (ranges.empty()) return 0;
        const double step = angleStep();
        if (step == 0.0) return 0;
        const double a = std::clamp(angle, angle_min, angle_max);
        const long idx = std::lround((a - angle_min) / step);
        return static_cast<std::size_t>(std::clamp<long>(idx, 0, static_cast<long>(ranges.size()) - 1));
    }
};

} // namespace ftg
