// ExploreRobot.h - the robot as the exploration sees it: pose in the MAP
// frame, the HC-SR04 range(s), bumper, (v, w). The lab set-up uses ONE front
// sonar (Geometry::sideSensors = false); the robot turns 360 deg to look around.
// Slots 0 / 2 (left / right 45 deg) are kept for a 3-sensor board.
//
//   HardwareRobot - real Kobuki (gyro heading + encoder distance, turned into
//                   the map frame with the start pose) + the Arduino board.
//   SimRobot      - kinematic simulator: the true map is a 0/1 txt file
//                   (an obstacle map, obstacleMapReferences/mapN/mapN.txt),
//                   obstacle cells are solid, the sonar is ray-cast with a
//                   cone, noise and missed echoes, like the real board.
//                   The noise comes from a seeded generator: the same seed
//                   gives the same run (benchmark repeats use seeds 1..N).
//   SensorBoard   - the Arduino alone (used by the calibration mode).
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "AStarPlanner.h"
#include "ArduinoTriSensor.h"
#include "Kobuki.h"
#include "Motion.h"
#include "NavCommon.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace explore {

using nav::Bumper;
using nav::Deg;
using nav::Pose;

// One sonar reading: range from the SENSOR in metres; echo = false: the
// sensor heard nothing (-1); valid = false: field missing / broken packet.
struct Beam {
    double range = 0.0;
    bool valid = false;
    bool echo = false;
};

struct Snapshot {
    Pose pose;
    Beam beams[3];                 // 0 = left (+45), 1 = front, 2 = right (-45)
    bool newData = false;          // a new sensor packet arrived since the previous read()
    bool fresh = true;             // false: sensor stream stalled
    Bumper bumper = Bumper::None;
    const char* fault = nullptr;   // cliff
};

struct Geometry {
    double sideDeg = 45.0;         // side sensors (if fitted): +-45 deg from the front
    double offset = 0.11;          // sensor distance from the robot centre (m)
    double radius = 0.175;         // Kobuki body radius (m)
    bool sideSensors = false;      // false: only the FRONT sonar (the robot turns 360 deg to look around)
    double angle(int k) const { return k == 0 ? sideDeg * Deg : (k == 2 ? -sideDeg * Deg : 0.0); }
    bool uses(int k) const { return k == 1 || sideSensors; }
};

class Robot {
public:
    virtual ~Robot() = default;
    virtual bool open() = 0;
    virtual Snapshot read() = 0;
    virtual void drive(double v, double w) = 0;
    virtual void stop() = 0;
    virtual double tick() = 0;               // one control period, returns elapsed seconds
    virtual void shutdown() { stop(); }
    virtual void resetPose() {}               // the robot is on the start pose now
    virtual void shiftPose(double, double) {} // odometry correction (bump against a known obstacle)
    virtual const char* name() const = 0;
};

// ------------------------------------------------------- Arduino sensors --
class SensorBoard {
public:
    // scale: distance calibration (RangeCalibration.h, sensor_calibration.txt), 1 = none.
    explicit SensorBoard(std::string port, double scale = 1.0) : port_(std::move(port)), scale_(scale) {}

    bool open() {
        std::cout << "Opening sensor board on " << port_ << "...\n";
        try {
            sensor_.open(port_);
        } catch (const std::exception& e) {
            std::cerr << "ERROR opening sensor port: " << e.what() << "\n"
                      << "  (Is it open in Arduino IDE / another monitor window?)\n";
            return false;
        }
        for (int i = 0; i < 40 && sensor_.readingCount() == 0; ++i) {   // the Arduino reboots (~2 s)
            sensor_.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (sensor_.readingCount() == 0) {
            std::cerr << "No sensor data on " << port_ << ". Check power / cable / sketch.\n";
            return false;
        }
        lastCount_ = sensor_.readingCount();
        lastPacket_ = std::chrono::steady_clock::now();
        return true;
    }

    // True once per new packet.
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

    void fill(Beam beams[3]) const {
        const auto& r = sensor_.last();
        const double noEcho = sensor_.maxRangeMeters() - 1e-6;   // ArduinoTriSensor maps -1 to its max range
        const auto beam = [&](double m, bool valid) {
            const bool echo = m < noEcho;
            return Beam{ echo ? m * scale_ : m, valid, echo };
        };
        beams[0] = beam(r.leftM, r.leftValid);
        beams[1] = beam(r.frontM, r.frontValid);
        beams[2] = beam(r.rightM, r.rightValid);
    }

private:
    std::string port_;
    double scale_ = 1.0;
    bug0::ArduinoTriSensor sensor_{ 4.0 };
    std::size_t lastCount_ = 0;
    std::chrono::steady_clock::time_point lastPacket_;
};

// ------------------------------------------------------------- hardware --
class HardwareRobot final : public Robot {
public:
    HardwareRobot(std::string kobukiPort, std::string arduinoPort, Pose start, double rangeScale = 1.0)
        : kobukiPort_(std::move(kobukiPort)), sensors_(std::move(arduinoPort), rangeScale), start_(start) {}

    const char* name() const override { return "Kobuki + Arduino (HC-SR04)"; }

    bool open() override {
        if (!sensors_.open()) return false;
        char* argv[] = { const_cast<char*>("Explore"), const_cast<char*>(kobukiPort_.c_str()) };
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
        ox_ = oy_ = corrX_ = corrY_ = 0.0;
    }

    void shiftPose(double dx, double dy) override {
        corrX_ += dx;
        corrY_ += dy;
    }

    Snapshot read() override {
        Snapshot s;
        s.newData = sensors_.poll();
        s.fresh = sensors_.secondsSincePacket() < 1.5;
        sensors_.fill(s.beams);

        const auto o = robot_.getOdometryData();
        const double dist = 0.5 * (o.left_distance + o.right_distance);
        const double heading = nav::normalizeAngle(robot_.getHeading() - heading0_);
        const double d = dist - lastDist_;
        const double mid = lastHeading_ + 0.5 * nav::normalizeAngle(heading - lastHeading_);
        ox_ += d * std::cos(mid);
        oy_ += d * std::sin(mid);
        lastDist_ = dist;
        lastHeading_ = heading;

        const double c = std::cos(start_.heading), sn = std::sin(start_.heading);
        s.pose.x = start_.x + ox_ * c - oy_ * sn + corrX_;
        s.pose.y = start_.y + ox_ * sn + oy_ * c + corrY_;
        s.pose.heading = nav::normalizeAngle(start_.heading + heading);

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
    std::string kobukiPort_;
    SensorBoard sensors_;
    Pose start_;
    kobuki::Kobuki robot_;
    double ox_ = 0.0, oy_ = 0.0, lastDist_ = 0.0, lastHeading_ = 0.0, heading0_ = 0.0, corrX_ = 0.0, corrY_ = 0.0;
    std::chrono::steady_clock::time_point lastTick_;
};

// ------------------------------------------------------------ simulator --
class SimRobot final : public Robot {
public:
    struct Options {
        Geometry geo;
        bool walledBorder = true;
        bool noise = true;    // distance dependent noise, occasional missed echo / ghost reading
        int speed = 4;        // roughly x real time; 0 = as fast as possible
        unsigned seed = 2026; // noise generator seed
    };

    SimRobot(astar::GridMap world, Pose start, Options opt)
        : world_(std::move(world)), pose_(start), opt_(opt), rng_(opt.seed) {}

    const char* name() const override { return "built-in simulator"; }

    bool open() override {
        sample();
        return true;
    }

    // Calibration in the simulator: put / remove an obstacle cell.
    void setObstacle(astar::Cell c, bool on) { world_.setOccupied(c, on); }
    void place(Pose p) { pose_ = p; }

    Snapshot read() override {
        Snapshot s;
        s.pose = pose_;
        for (int k = 0; k < 3; ++k) s.beams[k] = beams_[k];
        s.newData = newData_;
        newData_ = false;
        s.bumper = bumper_;
        return s;
    }

    void drive(double v, double w) override { v_ = v; w_ = w; }
    void stop() override { v_ = w_ = 0.0; }

    double tick() override {
        pose_.heading = nav::normalizeAngle(pose_.heading + w_ * Dt);
        const double nx = pose_.x + v_ * std::cos(pose_.heading) * Dt;
        const double ny = pose_.y + v_ * std::sin(pose_.heading) * Dt;
        bumper_ = Bumper::None;
        astar::Point hit{}, oldHit{};
        const double gapNew = gap({ nx, ny }, hit), gapOld = gap(pose_.point(), oldHit);
        if (gapNew >= opt_.geo.radius || gapNew > gapOld) {
            pose_.x = nx;
            pose_.y = ny;
        } else if (v_ > 0.0) {
            const double rel = nav::normalizeAngle(std::atan2(hit.y - pose_.y, hit.x - pose_.x) - pose_.heading);
            if (std::abs(rel) < 90.0 * Deg)
                bumper_ = rel > 0.35 ? Bumper::Left : (rel < -0.35 ? Bumper::Right : Bumper::Center);
        }
        simTime_ += Dt;
        if (simTime_ >= nextSample_ - 1e-9) sample();
        if (opt_.speed > 0 && ++sleepCounter_ % static_cast<unsigned>(opt_.speed) == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return Dt;
    }

    // One raw reading of sensor k right now (calibration).
    Beam measure(int k) {
        const double a = pose_.heading + opt_.geo.angle(k);
        const astar::Point s{ pose_.x + opt_.geo.offset * std::cos(a), pose_.y + opt_.geo.offset * std::sin(a) };
        double r = cast(s, a);
        if (opt_.noise && r < MaxRange) {
            r += noise_(rng_) * (0.005 + 0.02 * r);   // HC-SR04: the noise grows with distance (~8 cm at 4 m)
            if (uni_(rng_) < 0.04) r = MaxRange;                      // missed echo
        }
        if (opt_.noise && uni_(rng_) < 0.02) r = 0.15 + uni_(rng_) * 1.2;   // ghost / cross-talk reading
        if (r >= MaxRange) return { 4.0, true, false };
        return { std::floor(std::max(r, 0.02) * 100.0) / 100.0, true, true };
    }

private:
    static constexpr double Dt = 0.01;
    static constexpr double SamplePeriod = 0.1;   // Arduino: ~10 Hz
    static constexpr double ConeHalf = 12.0 * Deg;
    static constexpr double MaxRange = 4.0;
    static constexpr double MaxIncidence = 55.0 * Deg;   // beyond this angle to the face normal: no echo

    void sample() {
        nextSample_ += SamplePeriod;
        for (int k = 0; k < 3; ++k) beams_[k] = opt_.geo.uses(k) ? measure(k) : Beam{};
        newData_ = true;
    }

    // Nearest echo inside the cone (7 rays), from the sensor position. Like a
    // real sonar, a ray that meets a flat face at a grazing angle (more than
    // MaxIncidence from the face normal) is reflected away and gives no echo;
    // corners always echo.
    double cast(astar::Point s, double a0) const {
        double best = MaxRange;
        const double cosMax = std::cos(MaxIncidence);
        for (int i = 0; i < 7; ++i) {
            const double a = a0 - ConeHalf + ConeHalf * i / 3.0;
            const double dx = std::cos(a), dy = std::sin(a);
            astar::Point prev = s;
            for (double t = 0.0; t < best; t += 0.005) {
                const astar::Point q{ s.x + dx * t, s.y + dy * t };
                const bool out = q.x < 0.0 || q.y < 0.0 || q.x > world_.width() || q.y > world_.height();
                const bool solid = (out && opt_.walledBorder) || world_.occupied(world_.cellAt(q));
                if (solid) {
                    const astar::Cell from = world_.cellAt(prev), to = world_.cellAt(q);
                    const bool xFace = from.x != to.x, yFace = from.y != to.y;
                    const double cosInc = xFace && yFace ? 1.0 : (xFace ? std::abs(dx) : std::abs(dy));
                    if (cosInc >= cosMax) best = t;   // echo from this surface
                    break;                            // grazing: the sound is reflected away
                }
                if (out) break;
                prev = q;
            }
        }
        return best;
    }

    double gap(astar::Point p, astar::Point& hit) const {
        const double C = world_.cellSize(), r = opt_.geo.radius;
        double best = r;
        const astar::Cell at = world_.cellAt(p);
        const int w = static_cast<int>(std::ceil(r / C)) + 1;
        for (int y = at.y - w; y <= at.y + w; ++y)
            for (int x = at.x - w; x <= at.x + w; ++x) {
                if (!world_.occupied({ x, y })) continue;
                double x0, y0, x1, y1;
                world_.bounds({ x, y }, x0, y0, x1, y1);
                const astar::Point q{ std::clamp(p.x, x0, x1), std::clamp(p.y, y0, y1) };
                const double d = astar::distance(p, q);
                if (d < best) { best = d; hit = q; }
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
    double v_ = 0.0, w_ = 0.0, simTime_ = 0.0, nextSample_ = 0.0;
    Beam beams_[3];
    bool newData_ = false;
    Bumper bumper_ = Bumper::None;
    unsigned sleepCounter_ = 0;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_{ 0.0, 1.0 };
    std::uniform_real_distribution<double> uni_{ 0.0, 1.0 };
};

} // namespace explore
