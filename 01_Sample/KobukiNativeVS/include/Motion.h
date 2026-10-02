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
#include <iomanip>
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
    int    loop_ms      = 20;   // control loop period (ms)
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
    std::cout << std::fixed << std::setprecision(3)
              << label << "  x=" << o.x << " m  y=" << o.y
              << " m  heading=" << robot.getHeading() / Deg << " deg\n";
}

inline void printClosure(const Kobuki& robot) {
    const auto o = robot.getOdometryData();
    std::cout << std::fixed << std::setprecision(3)
              << "Closure error: " << std::hypot(o.x, o.y) * 100.0 << " cm\n";
}

inline void stop(Kobuki& robot) {
    robot.setBaseControl(0.0, 0.0);
    robot.disable();
}

// Safety guard: on bumper / cliff / wheel drop -> stop and exit the program.
inline void guard(Kobuki& robot) {
    const auto s = robot.getCoreSensorData();
    const char* why = nullptr;
    if (s.bumperLeft() || s.bumperCenter() || s.bumperRight()) why = "Bumper hit";
    else if (s.cliffLeft() || s.cliffCenter() || s.cliffRight()) why = "Cliff detected";
    else if (s.wheel_drop != 0) why = "Wheel drop";
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
        p.command_rate_hz = 50;
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
inline void turnTo(Kobuki& robot, double headingRad) {
    const double tol = cfg.turn_tol_deg * Deg;
    while (true) {
        guard(robot);
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
    std::cout << std::fixed << std::setprecision(3)
              << "goTo(" << x << ", " << y << "): angle=" << angle / Deg
              << " deg, distance=" << distance << " m\n";
    turnTo(robot, angle);
    driveStraight(robot, distance, angle);
}

} // namespace kobuki::motion
