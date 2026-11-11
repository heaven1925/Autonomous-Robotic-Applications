// Robot.h - the robot as the path follower sees it: a pose in the MAP frame,
// the bumper and a (v, w) velocity command. Back-ends:
//
//   KobukiRobot - real Kobuki: gyro heading + wheel encoder distance, turned
//                 into the map frame with the start pose from the map file.
//   SimRobot    - kinematic simulator of the same map: the robot is a disc,
//                 obstacle cells are solid (driving into one presses the
//                 bumper), optional obstacles that are NOT in the planner's
//                 map can be added to test re-planning.
//
// Map frame (AStarPlanner.h): metres, origin at the bottom-left corner of the
// map, +x right, +y up, heading 0 = +x, counter-clockwise positive.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "AStarPlanner.h"
#include "Kobuki.h"
#include "Motion.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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

struct Snapshot {
    Pose pose;
    Bumper bumper = Bumper::None;   // currently pressed bumper
    const char* fault = nullptr;    // non-null: safety event that must stop the run (cliff)
};

class Robot {
public:
    virtual ~Robot() = default;
    virtual bool open() = 0;              // connect; false = cannot run
    virtual Snapshot read() = 0;          // latest pose + safety state
    virtual void drive(double v, double w) = 0;
    virtual void stop() = 0;
    virtual double tick() = 0;            // wait one control period, return elapsed seconds
    virtual void shutdown() { stop(); }   // stop and release the robot at program end
    virtual void resetPose() {}           // "the robot is on the start pose now" (real robot: re-zero odometry)
    virtual void shiftPose(double, double) {}   // correct the position estimate (simulator: exact already)
    virtual const char* name() const = 0;
};

// ------------------------------------------------------------- hardware --
class KobukiRobot final : public Robot {
public:
    KobukiRobot(std::string port, Pose start) : port_(std::move(port)), start_(start) {}

    const char* name() const override { return "Kobuki"; }

    bool open() override {
        // Motion.h connect(): opens the port, waits for data, zeroes the
        // odometry; the driver itself cuts forward speed on a bumper press.
        char* argv[] = { const_cast<char*>("AStar"), const_cast<char*>(port_.c_str()) };
        kobuki::motion::connect(robot_, 2, argv, "Kobuki");
        resetPose();
        lastTick_ = std::chrono::steady_clock::now();
        return true;
    }

    void resetPose() override {
        const auto o = robot_.getOdometryData();
        lastDist_ = 0.5 * (o.left_distance + o.right_distance);
        heading0_ = robot_.getHeading();
        lastHeading_ = 0.0;
        ox_ = oy_ = 0.0;
        corrX_ = corrY_ = 0.0;
    }

    // Bumper against a KNOWN obstacle: the odometry has drifted by (dx, dy).
    void shiftPose(double dx, double dy) override {
        corrX_ += dx;
        corrY_ += dy;
    }

    Snapshot read() override {
        // Odometry frame (start = (0, 0), heading 0 = initial facing): heading
        // from the gyro, distance from the encoders, mid-point integration.
        const auto o = robot_.getOdometryData();
        const double dist = 0.5 * (o.left_distance + o.right_distance);
        const double heading = normalizeAngle(robot_.getHeading() - heading0_);
        const double d = dist - lastDist_;
        const double mid = lastHeading_ + 0.5 * normalizeAngle(heading - lastHeading_);
        ox_ += d * std::cos(mid);
        oy_ += d * std::sin(mid);
        lastDist_ = dist;
        lastHeading_ = heading;

        // ... turned into the map frame with the start pose.
        Snapshot s;
        const double c = std::cos(start_.heading), sn = std::sin(start_.heading);
        s.pose.x = start_.x + ox_ * c - oy_ * sn + corrX_;
        s.pose.y = start_.y + ox_ * sn + oy_ * c + corrY_;
        s.pose.heading = normalizeAngle(start_.heading + heading);

        const auto core = robot_.getCoreSensorData();
        if (core.cliffLeft() || core.cliffCenter() || core.cliffRight()) s.fault = "cliff detected";
        switch (kobuki::motion::bumper(robot_)) {
            case kobuki::motion::Bump::Left: s.bumper = Bumper::Left; break;
            case kobuki::motion::Bump::Right: s.bumper = Bumper::Right; break;
            case kobuki::motion::Bump::Center: s.bumper = Bumper::Center; break;
            default: break;
        }
        return s;
    }

    void drive(double v, double w) override { robot_.setBaseControl(v, w); }
    void stop() override { robot_.setBaseControl(0.0, 0.0); }
    void shutdown() override { kobuki::motion::stop(robot_); }

    double tick() override {
        kobuki::motion::wait();
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - lastTick_).count();
        lastTick_ = now;
        return dt;
    }

private:
    std::string port_;
    Pose start_;
    kobuki::Kobuki robot_;
    double ox_ = 0.0, oy_ = 0.0, lastDist_ = 0.0, lastHeading_ = 0.0, heading0_ = 0.0;
    double corrX_ = 0.0, corrY_ = 0.0;   // corrections from bumps against known obstacles
    std::chrono::steady_clock::time_point lastTick_;
};

// ------------------------------------------------------------ simulator --
class SimRobot final : public Robot {
public:
    struct Options {
        double radius = 0.15;        // robot body radius (m)
        bool walledBorder = false;   // the map border is a wall
        int speed = 4;               // roughly x real time; 0 = as fast as possible
        std::vector<astar::Cell> hidden;  // obstacles that exist but are not in the planner's map
    };

    SimRobot(astar::GridMap world, Pose start, Options opt) : world_(std::move(world)), pose_(start), opt_(std::move(opt)) {
        for (const auto& c : opt_.hidden) world_.setOccupied(c, true);
    }

    const char* name() const override { return "built-in simulator"; }

    bool open() override {
        std::cout << "Simulator: " << world_.cols() << " x " << world_.rows() << " cells of " << world_.cellSize() * 100.0
                  << " cm, robot radius " << opt_.radius * 100.0 << " cm";
        if (!opt_.hidden.empty()) std::cout << ", " << opt_.hidden.size() << " obstacle cell(s) NOT in the planner's map";
        std::cout << ".\n";
        return true;
    }

    Snapshot read() override {
        Snapshot s;
        s.pose = pose_;
        s.bumper = bumper_;
        return s;
    }

    void drive(double v, double w) override { v_ = v; w_ = w; }
    void stop() override { v_ = w_ = 0.0; }

    // 10 ms of simulated time per call.
    double tick() override {
        pose_.heading = normalizeAngle(pose_.heading + w_ * Dt);
        const double nx = pose_.x + v_ * std::cos(pose_.heading) * Dt;
        const double ny = pose_.y + v_ * std::sin(pose_.heading) * Dt;
        bumper_ = Bumper::None;
        astar::Point hit{}, oldHit{};
        const double gapNew = gap({ nx, ny }, hit), gapOld = gap(pose_.point(), oldHit);
        if (gapNew >= opt_.radius || gapNew > gapOld) {
            pose_.x = nx;   // free, or moving away from what it touches
            pose_.y = ny;
        } else if (v_ > 0.0) {
            // Solid obstacle: the robot stays where it is and the bumper is pressed.
            const double rel = normalizeAngle(std::atan2(hit.y - pose_.y, hit.x - pose_.x) - pose_.heading);
            if (std::abs(rel) < 90.0 * Deg)
                bumper_ = rel > 0.35 ? Bumper::Left : (rel < -0.35 ? Bumper::Right : Bumper::Center);
        }
        if (opt_.speed > 0 && ++sleepCounter_ % static_cast<unsigned>(opt_.speed) == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return Dt;
    }

private:
    static constexpr double Dt = 0.01;

    // Distance from p to the nearest obstacle cell (or the border if walled),
    // at most the robot radius; < radius means the robot disc at p overlaps
    // it. `hit` = the nearest obstacle point.
    double gap(astar::Point p, astar::Point& hit) const {
        const double C = world_.cellSize(), r = opt_.radius;
        double best = r;
        const astar::Cell at = world_.cellAt(p);
        const int w = static_cast<int>(std::ceil(r / C)) + 1;
        for (int y = at.y - w; y <= at.y + w; ++y) {
            for (int x = at.x - w; x <= at.x + w; ++x) {
                if (!world_.occupied({ x, y })) continue;
                double x0, y0, x1, y1;
                world_.bounds({ x, y }, x0, y0, x1, y1);
                const astar::Point q{ std::clamp(p.x, x0, x1), std::clamp(p.y, y0, y1) };
                const double d = astar::distance(p, q);
                if (d < best) { best = d; hit = q; }
            }
        }
        if (opt_.walledBorder) {
            const astar::Point border[4] = { { 0.0, p.y }, { world_.width(), p.y }, { p.x, 0.0 }, { p.x, world_.height() } };
            for (const auto& q : border) {
                const double d = astar::distance(p, q);
                if (d < best) { best = d; hit = q; }
            }
        }
        return best;
    }

    astar::GridMap world_;
    Pose pose_;
    Options opt_;
    double v_ = 0.0, w_ = 0.0;
    Bumper bumper_ = Bumper::None;
    unsigned sleepCounter_ = 0;
};

} // namespace nav
