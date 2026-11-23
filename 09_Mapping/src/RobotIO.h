// RobotIO.h - the robot as the mapping application sees it: a pose, three
// ultrasonic ranges, the bumper and a (v, w) velocity command. Back-ends:
//
//   HardwareRobot - real Kobuki (gyro heading + encoder distance) and the
//                   Arduino 3x HC-SR04 sensor board (same board as example 08).
//   SimRobot      - built-in simulator of the floor plan in the assignment
//                   figure, with the Arduino's sensor timing, so the program
//                   can be tried without hardware.
//
// Contract (same as 08_TangentBugAlgorithm):
//   * pose  : metres / radians, start = (0, 0), heading 0 = initial facing,
//             +y = robot's left (counter-clockwise positive).
//   * beams : metres measured from each SENSOR along its beam; echo = false
//             means the sensor heard nothing (-1 from the Arduino), valid =
//             false a missing / broken field.
//   * newData: true once per new sensor packet - the map is only updated
//             then, with the pose of that moment.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "ArduinoTriSensor.h"
#include "FrontierExplorer.h"
#include "Kobuki.h"
#include "Motion.h"
#include "OccupancyGrid.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace app {

enum class Bumper { None, Left, Center, Right };

struct Snapshot {
    explore::Pose pose;
    ogm::BeamReading left, front, right;  // as measured by the sensors
    bool newData = false;                 // a new sensor packet arrived since the previous read()
    bool fresh = true;                    // false: sensor stream stalled for too long
    Bumper bumper = Bumper::None;         // currently pressed bumper
    const char* fault = nullptr;          // non-null: hardware safety event that must stop the run (cliff)
};

class RobotIO {
public:
    virtual ~RobotIO() = default;
    virtual bool open() = 0;                 // connect; false = cannot run
    virtual Snapshot read() = 0;             // latest pose + ranges + safety state
    virtual void drive(double v, double w) = 0;
    virtual void stop() = 0;
    virtual double tick() = 0;               // wait one control period, return elapsed seconds
    virtual void shutdown() { stop(); }      // stop and release the robot at program end
    virtual const char* name() const = 0;

    // How the map is best shown: number of quarter turns applied to the
    // default view (robot's initial heading = up).
    virtual int viewQuarterTurns() const { return 0; }

    // Simulator only: which share of the world rectangle is really solid.
    // Returns false if unknown (real robot) or outside the simulated arena.
    virtual bool truthSolid(double, double, double, double, double&) const { return false; }

    // Simulator only: distance from a point to the nearest real surface,
    // negative inside a wall / obstacle. Same return convention.
    virtual bool truthDistance(double, double, double&) const { return false; }
};

// ------------------------------------------------------- Arduino sensors --
class SensorBoard {
public:
    explicit SensorBoard(std::string port) : port_(std::move(port)) {}

    bool open() {
        std::cout << "Opening sensor board on " << port_ << "...\n";
        try {
            sensor_.open(port_);
        } catch (const std::exception& e) {
            std::cerr << "ERROR opening sensor port: " << e.what() << "\n"
                      << "  (Is it open in Arduino IDE / another monitor window?)\n";
            return false;
        }
        // Opening the port resets the Arduino (~2 s boot) - wait for data.
        for (int i = 0; i < 40 && sensor_.readingCount() == 0; ++i) {
            sensor_.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (sensor_.readingCount() == 0) {
            std::cerr << "No sensor data on " << port_ << ". Check power/cable/sketch.\n";
            return false;
        }
        lastCount_ = sensor_.readingCount();
        lastPacket_ = std::chrono::steady_clock::now();
        return true;
    }

    // True if a new packet arrived since the previous call (drains the buffer
    // so the newest packet is the one used).
    bool poll() {
        for (int i = 0; i < 256 && sensor_.poll(); ++i) {}
        if (sensor_.readingCount() == lastCount_) return false;
        lastCount_ = sensor_.readingCount();
        lastPacket_ = std::chrono::steady_clock::now();
        return true;
    }

    double secondsSincePacket() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - lastPacket_).count();
    }

    void fill(Snapshot& s) const {
        const auto& r = sensor_.last();
        const double noEcho = sensor_.maxRangeMeters() - 1e-6;  // ArduinoTriSensor maps -1 to its maximum range
        s.left  = { r.leftM,  r.leftValid,  r.leftM  < noEcho };
        s.front = { r.frontM, r.frontValid, r.frontM < noEcho };
        s.right = { r.rightM, r.rightValid, r.rightM < noEcho };
    }

private:
    std::string port_;
    bug0::ArduinoTriSensor sensor_{4.0};
    std::size_t lastCount_ = 0;
    std::chrono::steady_clock::time_point lastPacket_;
};

// ------------------------------------------------------------- hardware --
class HardwareRobot final : public RobotIO {
public:
    HardwareRobot(std::string kobukiPort, std::string arduinoPort)
        : kobukiPort_(std::move(kobukiPort)), sensors_(std::move(arduinoPort)) {}

    const char* name() const override { return "Kobuki + Arduino"; }

    bool open() override {
        if (!sensors_.open()) return false;

        // Motion.h connect(): opens the port, waits for data, zeroes odometry
        // and the gyro heading (the robot's current spot becomes (0, 0)); the
        // driver itself cuts forward speed the moment a bumper is pressed.
        char* argv[] = { const_cast<char*>("Mapping"), const_cast<char*>(kobukiPort_.c_str()) };
        kobuki::motion::connect(robot_, 2, argv, "Kobuki");
        const auto o = robot_.getOdometryData();
        lastDist_ = 0.5 * (o.left_distance + o.right_distance);
        lastHeading_ = robot_.getHeading();
        lastTick_ = std::chrono::steady_clock::now();
        return true;
    }

    Snapshot read() override {
        Snapshot s;
        s.newData = sensors_.poll();
        s.fresh = sensors_.secondsSincePacket() < SensorTimeoutS;
        sensors_.fill(s);

        // Pose: heading from the gyro, distance from the wheel encoders,
        // integrated with the mid-point heading (see 08 RobotIO.h).
        const auto o = robot_.getOdometryData();
        const double dist = 0.5 * (o.left_distance + o.right_distance);
        const double heading = robot_.getHeading();
        const double d = dist - lastDist_;
        const double mid = lastHeading_ + 0.5 * ogm::normalizeAngle(heading - lastHeading_);
        x_ += d * std::cos(mid);
        y_ += d * std::sin(mid);
        lastDist_ = dist;
        lastHeading_ = heading;
        s.pose = { x_, y_, heading };

        const auto core = robot_.getCoreSensorData();
        if (core.cliffLeft() || core.cliffCenter() || core.cliffRight()) s.fault = "cliff detected";
        switch (kobuki::motion::bumper(robot_)) {
            case kobuki::motion::Bump::Left:   s.bumper = Bumper::Left; break;
            case kobuki::motion::Bump::Right:  s.bumper = Bumper::Right; break;
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
    static constexpr double SensorTimeoutS = 1.5;
    std::string kobukiPort_;
    SensorBoard sensors_;
    kobuki::Kobuki robot_;
    double x_ = 0.0, y_ = 0.0, lastDist_ = 0.0, lastHeading_ = 0.0;
    std::chrono::steady_clock::time_point lastTick_;
};

// ------------------------------------------------------------ simulator --
// The floor plan of the assignment figure: 16 x 24 cells of 0.5 m (8 x 12 m).
//   '#' wall cell, '.' free, 'S' start (the green cell; the robot faces "down").
// Two shapes do not follow the cells and are modelled exactly: the diagonal
// wall across the top-left corner and the round obstacle on the left wall.
// World frame = robot frame at the start: +x is "down" in the figure, +y is
// "right", so showing the map turned by two quarter turns reproduces the figure.
class SimRobot final : public RobotIO {
public:
    struct Options {
        double beamAngle = 60.0 * ogm::Pi / 180.0;
        double sensorOffset = 0.11;
        double robotRadius = 0.18;
        bool noise = false;   // +-1 cm range noise and occasional missed echoes
        int speed = 8;        // roughly x real time; 0 = as fast as possible
    };

    explicit SimRobot(Options opt) : opt_(opt) {
        // Edges between a wall cell and a non-wall cell.
        for (int r = 0; r < Rows; ++r) {
            for (int c = 0; c < Cols; ++c) {
                if (Map[r][c] != '#') continue;
                if (r > 0 && Map[r - 1][c] != '#')        edge(c, r, c + 1, r);
                if (r < Rows - 1 && Map[r + 1][c] != '#') edge(c, r + 1, c + 1, r + 1);
                if (c > 0 && Map[r][c - 1] != '#')        edge(c, r, c, r + 1);
                if (c < Cols - 1 && Map[r][c + 1] != '#') edge(c + 1, r, c + 1, r + 1);
            }
        }
        edge(DiagU0 + DiagDU, DiagV0, DiagU0, DiagV0 + DiagDV);  // diagonal wall
        circleX_ = CircleV * CellM - StartY;
        circleY_ = CircleU * CellM - StartX;
        for (auto& h : hist_) for (double& v : h) v = NoEcho;
    }

    const char* name() const override { return "built-in simulator"; }
    int viewQuarterTurns() const override { return 2; }

    bool open() override {
        std::cout << "Simulator: the floor plan of the assignment figure, 16 x 24 cells of 0.5 m (8 x 12 m).\n"
                  << "  The robot starts on the green cell facing into the room"
                  << (opt_.noise ? "; sensor noise ON.\n" : ".\n");
        sample();
        return true;
    }

    Snapshot read() override {
        Snapshot s;
        s.pose = pose_;
        s.left = beams_[0]; s.front = beams_[1]; s.right = beams_[2];
        s.newData = newData_;
        newData_ = false;
        s.bumper = bumper_;
        return s;
    }

    void drive(double v, double w) override { v_ = v; w_ = w; }
    void stop() override { v_ = w_ = 0.0; }

    // 10 ms of simulated time per call.
    double tick() override {
        pose_.heading = ogm::normalizeAngle(pose_.heading + w_ * Dt);
        const double nx = pose_.x + v_ * std::cos(pose_.heading) * Dt;
        const double ny = pose_.y + v_ * std::sin(pose_.heading) * Dt;
        double px = 0.0, py = 0.0;
        bumper_ = Bumper::None;
        if (nearest(nx, ny, px, py) >= opt_.robotRadius) {
            pose_.x = nx;
            pose_.y = ny;
        } else if (v_ > 0.0) {
            // Walls are solid: the robot stays where it is and the bumper is pressed.
            const double rel = ogm::normalizeAngle(std::atan2(py - pose_.y, px - pose_.x) - pose_.heading);
            bumper_ = rel > 0.35 ? Bumper::Left : (rel < -0.35 ? Bumper::Right : Bumper::Center);
        }
        simTime_ += Dt;
        if (simTime_ >= nextSample_ - 1e-9) sample();
        if (opt_.speed > 0 && ++sleepCounter_ % static_cast<unsigned>(opt_.speed) == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return Dt;
    }

    bool truthSolid(double x0, double y0, double x1, double y1, double& fraction) const override {
        double u, v;
        toFigure(0.5 * (x0 + x1), 0.5 * (y0 + y1), u, v);
        if (u < 0.0 || v < 0.0 || u >= Cols || v >= Rows) return false;
        const int n = 10;
        int solidCount = 0;
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                toFigure(x0 + (x1 - x0) * (i + 0.5) / n, y0 + (y1 - y0) * (j + 0.5) / n, u, v);
                if (solid(u, v)) ++solidCount;
            }
        fraction = static_cast<double>(solidCount) / (n * n);
        return true;
    }

    bool truthDistance(double x, double y, double& distance) const override {
        double u, v;
        toFigure(x, y, u, v);
        if (u < 0.0 || v < 0.0 || u >= Cols || v >= Rows) return false;
        double px = 0.0, py = 0.0;
        distance = nearest(x, y, px, py);
        if (solid(u, v)) distance = -distance;
        return true;
    }

private:
    struct Seg { double x1, y1, x2, y2; };

    static constexpr int Cols = 16, Rows = 24;
    static constexpr double CellM = 0.5;                      // metres per figure cell
    static constexpr double StartX = 8.5 * CellM;             // start = centre of the green cell (column 8, row 1),
    static constexpr double StartY = 1.5 * CellM;             //         in figure metres (x right, y down)
    static constexpr double DiagU0 = 1.0, DiagV0 = 1.0;       // diagonal wall: from (U0 + DU, V0) to (U0, V0 + DV),
    static constexpr double DiagDU = 4.7, DiagDV = 3.2;       //         in figure cells (measured from the figure)
    static constexpr double CircleU = 1.3, CircleV = 9.4;     // round obstacle: centre and radius in figure cells
    static constexpr double CircleR = 1.6;
    static constexpr double Dt = 0.01;
    static constexpr double SamplePeriod = 0.1;               // Arduino packet rate
    static constexpr double ConeHalf = 10.0 * ogm::Pi / 180.0;// HC-SR04-like beam
    static constexpr double MaxRange = 4.0;                   // beyond this the sensor hears nothing
    static constexpr double NoEcho = 1e9;

    static constexpr const char* Map[Rows] = {
        "################",
        "#.......S......#",
        "#..............#",
        "#..............#",
        "#..............#",
        "#.......###..###",
        "#.......#......#",
        "#.......#......#",
        "#.......#......#",
        "#.......#......#",
        "#.......#......#",
        "#.......########",
        "#.......#......#",
        "#..............#",
        "#..............#",
        "#.......#......#",
        "#.......#......#",
        "#.......########",
        "###.....#......#",
        "###.....#......#",
        "###............#",
        "##.............#",
        "##......#......#",
        "################",
    };

    // Figure cells (u right, v down) <-> world metres (x = down, y = right, origin at the start).
    static void toFigure(double x, double y, double& u, double& v) {
        u = (y + StartX) / CellM;
        v = (x + StartY) / CellM;
    }

    void edge(double u1, double v1, double u2, double v2) {
        walls_.push_back({ v1 * CellM - StartY, u1 * CellM - StartX, v2 * CellM - StartY, u2 * CellM - StartX });
    }

    static bool solid(double u, double v) {
        if (u < 0.0 || v < 0.0 || u >= Cols || v >= Rows) return true;
        if (Map[static_cast<int>(v)][static_cast<int>(u)] == '#') return true;
        if ((u - DiagU0) / DiagDU + (v - DiagV0) / DiagDV <= 1.0) return true;   // behind the diagonal wall
        return std::hypot(u - CircleU, v - CircleV) <= CircleR;                  // inside the round obstacle
    }

    // Same as the Arduino sketch: measure every beam (whole centimetres),
    // report the minimum of the last 2 samples per beam.
    void sample() {
        nextSample_ += SamplePeriod;
        const double rel[3] = { opt_.beamAngle, 0.0, -opt_.beamAngle };
        for (int k = 0; k < 3; ++k) {
            const double a = pose_.heading + rel[k];
            double r = cast(pose_.x + opt_.sensorOffset * std::cos(a), pose_.y + opt_.sensorOffset * std::sin(a), a);
            if (opt_.noise && r < MaxRange) {
                r += noise_(rng_);
                if (miss_(rng_) < 0.03) r = NoEcho;
            }
            hist_[k][histIndex_] = r >= MaxRange ? NoEcho : std::floor(std::max(r, 0.02) * 100.0) / 100.0;
            const double m = std::min(hist_[k][0], hist_[k][1]);
            beams_[k] = m >= NoEcho ? ogm::BeamReading{ MaxRange, true, false } : ogm::BeamReading{ m, true, true };
        }
        histIndex_ = (histIndex_ + 1) % 2;
        newData_ = true;
    }

    // Nearest echo inside the beam cone (5 rays across +-ConeHalf).
    double cast(double ox, double oy, double a0) const {
        double best = MaxRange;
        for (int k = 0; k < 5; ++k) {
            const double a = a0 - ConeHalf + ConeHalf * k / 2.0;
            const double dx = std::cos(a), dy = std::sin(a);
            for (const auto& s : walls_) {
                const double sx = s.x2 - s.x1, sy = s.y2 - s.y1;
                const double den = dx * sy - dy * sx;
                if (std::abs(den) < 1e-12) continue;
                const double t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / den;
                const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
                if (t >= 0.0 && u >= 0.0 && u <= 1.0) best = std::min(best, t);
            }
            // Round obstacle: first intersection of the ray with the circle.
            const double fx = ox - circleX_, fy = oy - circleY_;
            const double b = fx * dx + fy * dy;
            const double disc = b * b - (fx * fx + fy * fy - radius() * radius());
            if (disc >= 0.0) {
                const double t = -b - std::sqrt(disc);
                if (t >= 0.0) best = std::min(best, t);
            }
        }
        return best;
    }

    static double radius() { return CircleR * CellM; }

    // Distance from (x, y) to the nearest wall / obstacle surface, and that point.
    double nearest(double x, double y, double& px, double& py) const {
        double best = 1e9;
        for (const auto& s : walls_) {
            const double vx = s.x2 - s.x1, vy = s.y2 - s.y1;
            const double len2 = vx * vx + vy * vy;
            double t = len2 > 1e-12 ? ((x - s.x1) * vx + (y - s.y1) * vy) / len2 : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            const double qx = s.x1 + t * vx, qy = s.y1 + t * vy;
            const double d = std::hypot(x - qx, y - qy);
            if (d < best) { best = d; px = qx; py = qy; }
        }
        const double dc = std::hypot(x - circleX_, y - circleY_);
        if (dc - radius() < best && dc > 1e-9) {
            best = dc - radius();
            px = circleX_ + (x - circleX_) * radius() / dc;
            py = circleY_ + (y - circleY_) * radius() / dc;
        }
        return best;
    }

    Options opt_;
    std::vector<Seg> walls_;
    double circleX_ = 0.0, circleY_ = 0.0;
    explore::Pose pose_{};
    double v_ = 0.0, w_ = 0.0;
    double simTime_ = 0.0, nextSample_ = 0.0;
    double hist_[3][2]{};
    int histIndex_ = 0;
    ogm::BeamReading beams_[3];
    bool newData_ = false;
    Bumper bumper_ = Bumper::None;
    unsigned sleepCounter_ = 0;
    std::mt19937 rng_{ 12345 };
    std::normal_distribution<double> noise_{ 0.0, 0.01 };
    std::uniform_real_distribution<double> miss_{ 0.0, 1.0 };
};

} // namespace app
