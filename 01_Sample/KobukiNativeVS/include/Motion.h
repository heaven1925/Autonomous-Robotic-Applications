// Motion.h - minimal motion helpers for the examples.
//
// Design: NO error handling in the examples. Every helper checks safety
// (bumper / cliff / wheel drop) itself; on any problem it stops the robot,
// prints the reason and exits the program. Example code stays linear:
//
//     kobuki::Kobuki robot;
//     connect(robot, argc, argv, "My scenario");
//     driveStraight(robot, 1.0, 0.0);   // 1 m, heading 0
//     turnTo(robot, 90 * Deg);          // face 90 deg
//     goTo(robot, 2.0, 1.0);            // drive to point (x, y)
//     stop(robot);
//
// All tunable values live in the global `cfg` (see struct Tuning).
#pragma once

#include "Kobuki.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace kobuki::motion {

constexpr double Pi  = 3.14159265358979323846;
constexpr double Deg = Pi / 180.0;   // 90 * Deg == 90 degrees in radians

// ----------------------------- tuning ------------------------------------
struct Tuning {
    double drive_speed  = 0.15; // m/s   - straight-line speed
    double turn_speed   = 0.60; // rad/s - max rotation speed
    double min_turn     = 0.10; // rad/s - min rotation speed (avoids stall)
    double turn_tol_deg = 0.9;  // deg   - stop tolerance for turns
    double heading_gain = 1.5;  // P gain that keeps straight lines straight
    int    loop_ms      = 10;   // control loop period (ms)
    bool   wheel_drop_check = false; // wheel drop -> exit (disabled for now)

    // obstacle avoidance (goToAvoid) - trapezoid wall-follow from the bump point
    // (wall = base of the trapezoid, legs at 45 deg to the wall):
    //   turn avoid_angle (135) -> avoid_distance -> parallel pass_distance ->
    //   45 deg back to the wall -> avoid_distance; repeat while it keeps bumping.
    double avoid_angle    = 135.0; // deg - turn at the bump point
    double avoid_distance = 0.30; // m   - diagonal leg
    double pass_distance  = 0.30; // m   - straight leg (parallel to old heading)
    double goal_tol       = 0.05; // m   - target counts as reached
};

inline Tuning cfg; // edit fields at the top of your main() to tune behaviour

// ----------------------------- small utils -------------------------------
inline double normalize(double a) {
    while (a > Pi)  a -= 2.0 * Pi;
    while (a <= -Pi) a += 2.0 * Pi;
    return a;
}

inline void wait() { std::this_thread::sleep_for(std::chrono::milliseconds(cfg.loop_ms)); }

inline void printPose(const Kobuki& robot, const char* label) {
    const auto o = robot.getOdometryData();
    std::cout << label << "  x=" << o.x << " m  y=" << o.y
              << " m  heading=" << robot.getHeading() / Deg << " deg\n";
}

inline void printClosure(const Kobuki& robot) {
    const auto o = robot.getOdometryData();
    std::cout << "Closure error: " << std::hypot(o.x, o.y) * 100.0 << " cm\n";
}

inline void stop(Kobuki& robot) {
    robot.setBaseControl(0.0, 0.0);
    robot.disable();
}

// Which bumper is pressed (Left/Right win over Center if both).
enum class Bump { None, Left, Center, Right };

inline Bump bumper(const Kobuki& robot) {
    const auto s = robot.getCoreSensorData();
    if (s.bumperLeft())   return Bump::Left;
    if (s.bumperRight())  return Bump::Right;
    if (s.bumperCenter()) return Bump::Center;
    return Bump::None;
}

// Safety guard: on cliff / wheel drop -> stop and exit the program.
// Bumper also exits, unless allowBump is true.
inline void guard(Kobuki& robot, bool allowBump = false) {
    const auto s = robot.getCoreSensorData();
    const char* why = nullptr;
    if (!allowBump && bumper(robot) != Bump::None) why = "Bumper hit";
    else if (s.cliffLeft() || s.cliffCenter() || s.cliffRight()) why = "Cliff detected";
    else if (cfg.wheel_drop_check && s.wheel_drop != 0) why = "Wheel drop";
    if (why) {
        stop(robot);
        std::cout << "\n!! " << why << " - stopped.\n";
        std::exit(1);
    }
}

// ----------------------------- connection --------------------------------
// Opens the COM port given on the command line, waits for the sensor
// stream and zeroes the odometry. Exits with a message on any failure.
inline void connect(Kobuki& robot, int argc, char** argv, const char* title) {
    std::cout << title << "\n\n";
    if (argc < 2) { std::cerr << "Usage: <program> <COM port>   (example: COM6)\n"; std::exit(1); }

    try {
        Parameters p;
        p.device_port = argv[1];
        p.command_rate_hz = 100; // max rate -> stop commands go out fast
        p.stop_on_bump = true;   // driver cuts forward speed on bumper press
        std::cout << "Opening " << argv[1] << "...\n";
        robot.init(p);
        robot.enable();
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        std::exit(1);
    }

    for (int i = 0; i < 30 && !robot.isAlive(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!robot.isAlive()) {
        std::cerr << "No Kobuki data on " << argv[1] << ". Check power/cable.\n";
        std::exit(1);
    }

    robot.resetOdometry();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    printPose(robot, "START");
}

// ----------------------------- motions -----------------------------------
// Drive `distance` metres while holding the absolute heading `headingRad`.
inline void driveStraight(Kobuki& robot, double distance, double headingRad) {
    const auto start = robot.getOdometryData();
    while (true) {
        guard(robot);
        const auto o = robot.getOdometryData();
        if (std::hypot(o.x - start.x, o.y - start.y) >= distance) break;
        const double correction = cfg.heading_gain * normalize(headingRad - robot.getHeading());
        robot.setBaseControl(cfg.drive_speed, correction);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

// Rotate in place to the absolute heading `headingRad`.
// allowBump = true: a pressed bumper does not exit (used while avoiding).
inline void turnTo(Kobuki& robot, double headingRad, bool allowBump = false) {
    const double tol = cfg.turn_tol_deg * Deg;
    while (true) {
        guard(robot, allowBump);
        const double error = normalize(headingRad - robot.getHeading());
        if (std::abs(error) < tol) break;
        double w = std::clamp(2.0 * error, -cfg.turn_speed, cfg.turn_speed);
        if (std::abs(w) < cfg.min_turn) w = (w < 0 ? -cfg.min_turn : cfg.min_turn);
        robot.setBaseControl(0.0, w);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

// Rotate by a RELATIVE angle (radians, + = left). Uses accumulated gyro
// deltas, so 360 deg and larger angles work.
inline void turnBy(Kobuki& robot, double angleRad) {
    const double tol = cfg.turn_tol_deg * Deg;
    const double dir = (angleRad >= 0.0) ? 1.0 : -1.0;
    double accumulated = 0.0;
    double previous = robot.getHeading();
    while (true) {
        guard(robot);
        const double current = robot.getHeading();
        accumulated += normalize(current - previous);
        previous = current;
        const double remaining = angleRad - accumulated;
        if (dir * remaining <= tol) break;
        double w = std::clamp(2.0 * remaining, -cfg.turn_speed, cfg.turn_speed);
        if (std::abs(w) < cfg.min_turn) w = (w < 0 ? -cfg.min_turn : cfg.min_turn);
        robot.setBaseControl(0.0, w);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

// Go to the odometry point (x, y): compute the angle to the target,
// turn towards it, then drive the straight-line distance.
inline void goTo(Kobuki& robot, double x, double y) {
    const auto o = robot.getOdometryData();
    const double angle    = std::atan2(y - o.y, x - o.x);
    const double distance = std::hypot(x - o.x, y - o.y);
    std::cout << "goTo(" << x << ", " << y << "): angle=" << angle / Deg
              << " deg, distance=" << distance << " m\n";
    turnTo(robot, angle);
    driveStraight(robot, distance, angle);
}

// ----------------------------- bumper avoidance --------------------------
// Like driveStraight, but a bumper hit does NOT exit: the robot stops at
// once and the pressed side is returned. Bump::None = distance reached.
// A bumper still pressed at the start is ignored until it is released.
inline Bump driveUntilBump(Kobuki& robot, double distance, double headingRad) {
    const auto start = robot.getOdometryData();
    bool armed = bumper(robot) == Bump::None;
    while (true) {
        guard(robot, true);
        const Bump b = bumper(robot);
        if (b == Bump::None) armed = true;
        else if (armed) {
            robot.setBaseControl(0.0, 0.0);
            return b;
        }
        const auto o = robot.getOdometryData();
        if (std::hypot(o.x - start.x, o.y - start.y) >= distance) break;
        const double correction = cfg.heading_gain * normalize(headingRad - robot.getHeading());
        robot.setBaseControl(cfg.drive_speed, correction);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
    return Bump::None;
}

// Reverse `distance` metres (bumper may still be pressed at the start).
inline void backUp(Kobuki& robot, double distance) {
    const auto start = robot.getOdometryData();
    while (true) {
        guard(robot, true);
        const auto o = robot.getOdometryData();
        if (std::hypot(o.x - start.x, o.y - start.y) >= distance) break;
        robot.setBaseControl(-cfg.drive_speed, 0.0);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

// Turn to `heading`, then drive `distance` until done or bumped.
inline Bump leg(Kobuki& robot, const char* name, double heading, double distance) {
    std::cout << "  " << name << ": heading=" << heading / Deg << " deg, " << distance << " m\n";
    turnTo(robot, heading, true);
    return driveUntilBump(robot, distance, heading);
}

// Wall-follow around an obstacle with a repeating trapezoid, starting
// right at the bump point. hb = heading at the bump (into the wall),
// s = side to go around (always left). The wall is the base of the trapezoid:
//   A: hb + 135 deg, avoid_distance  (away from the wall, 45 deg to it)
//   B: hb +  90 deg, pass_distance   (parallel to the wall)
//   C: hb +  45 deg, avoid_distance  (back towards the wall)
// Bump in A/B -> new obstacle in front: new wall frame, restart at A.
// Bump in C   -> wall still there: restart at A (same frame).
// C without a bump -> obstacle ended, return (re-aim at target).
inline void avoid(Kobuki& robot, Bump hit) {
    const double s = 1.0; // always correct to the LEFT (+ = counter-clockwise)
    double hb = robot.getHeading();
    std::cout << "  bump (" << (hit == Bump::Left ? "left" : hit == Bump::Right ? "right" : "center")
              << ") -> wall-follow to the left\n";
    while (true) {
        const double out  = normalize(hb + s * cfg.avoid_angle * Deg);
        const double wall = normalize(hb + s * 90.0 * Deg);
        const double in   = normalize(hb + s * (180.0 - cfg.avoid_angle) * Deg);
        if (leg(robot, "A out ", out,  cfg.avoid_distance) != Bump::None ||
            leg(robot, "B wall", wall, cfg.pass_distance)  != Bump::None) {
            hb = robot.getHeading();
            std::cout << "  new obstacle in front -> new wall\n";
            continue;
        }
        if (leg(robot, "C in  ", in, cfg.avoid_distance) == Bump::None) {
            std::cout << "  obstacle passed\n";
            return;
        }
    }
}

// goTo with obstacle avoidance: drive towards (x, y); on a bump
// wall-follow with avoid(), then re-aim at the target from the current
// position. Never gives up.
inline void goToAvoid(Kobuki& robot, double x, double y) {
    while (true) {
        const auto o = robot.getOdometryData();
        const double distance = std::hypot(x - o.x, y - o.y);
        if (distance < cfg.goal_tol) return;
        const double angle = std::atan2(y - o.y, x - o.x);
        std::cout << "goToAvoid(" << x << ", " << y << "): angle=" << angle / Deg
                  << " deg, distance=" << distance << " m\n";
        turnTo(robot, angle, true);
        const Bump hit = driveUntilBump(robot, distance, angle);
        if (hit == Bump::None) return; // reached
        avoid(robot, hit);
    }
}

} // namespace kobuki::motion
