// RobotIO.h - the robot as the Tangent Bug application sees it: a pose, the
// three ultrasonic ranges and a (v, w) velocity command. Back-ends:
//
//   HardwareRobot - real Kobuki (gyro heading + encoder distance) and the
//                   Arduino 3x HC-SR04 sensor board.
//   SimRobot      - built-in simulator with the same sensor timing as the
//                   Arduino (10 Hz packets, min of the last 2 samples), so
//                   the program can be tried without hardware.
//
// Contract with the planner (TangentBugPlanner.h):
//   * pose  : metres / radians, start = (0, 0), heading 0 = initial facing,
//             +y = robot's left (counter-clockwise positive).
//   * range : metres measured from the robot CENTRE along each beam. The
//             hardware back-end adds the sensor mounting offset to every
//             echo; "no echo" (-1 from the Arduino) arrives as the maximum
//             range (= nothing seen), a missing/broken field as invalid.
//   * newData: true once per new sensor packet - the planner is stepped only
//             then, so it never re-uses a stale reading with a newer pose.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ArduinoTriSensor.h"
#include "Kobuki.h"
#include "Motion.h"
#include "TangentBugPlanner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace app {

struct Snapshot {
    tbug::Pose pose;              // see contract above
    tbug::SensorReading range;    // centre-referenced, metres (for the planner)
    tbug::SensorReading raw;      // as measured by the sensors, metres (for display)
    bool newData = false;         // a new sensor packet arrived since the previous read()
    bool fresh = true;            // false: sensor stream stalled for too long
    const char* fault = nullptr;  // non-null: hardware safety event (bumper, cliff, collision)
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
};

// Converts sensor-referenced readings to centre-referenced ones. Only real
// echoes are shifted; "nothing seen" (>= maxRange) stays "nothing seen".
inline tbug::SensorReading toCentre(const tbug::SensorReading& raw, double offset, double maxRange) {
    tbug::SensorReading c = raw;
    auto shift = [&](double r) { return r < maxRange ? r + offset : r; };
    c.left = shift(raw.left);
    c.front = shift(raw.front);
    c.right = shift(raw.right);
    return c;
}

// ------------------------------------------------------- Arduino sensors --
// Sensor board only (no Kobuki): used by the hardware back-end and by the
// "--sensors" test mode.
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

    // Returns true if a new packet arrived since the previous call. Drains
    // everything buffered so far (e.g. while the user was typing a target)
    // so the newest packet is always the one used.
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

    tbug::SensorReading reading() const {
        const auto& r = sensor_.last();
        tbug::SensorReading s;
        s.left = r.leftM;   s.leftValid = r.leftValid;
        s.front = r.frontM; s.frontValid = r.frontValid;
        s.right = r.rightM; s.rightValid = r.rightValid;
        return s;
    }

    double maxRange() const { return sensor_.maxRangeMeters(); }

private:
    std::string port_;
    bug0::ArduinoTriSensor sensor_{4.0};
    std::size_t lastCount_ = 0;
    std::chrono::steady_clock::time_point lastPacket_;
};

// ------------------------------------------------------------- hardware --
class HardwareRobot final : public RobotIO {
public:
    HardwareRobot(std::string kobukiPort, std::string arduinoPort, double sensorOffset)
        : kobukiPort_(std::move(kobukiPort)), sensors_(std::move(arduinoPort)), sensorOffset_(sensorOffset) {}

    const char* name() const override { return "Kobuki + Arduino"; }

    bool open() override {
        if (!sensors_.open()) return false;

        // Motion.h connect(): opens the port, waits for data, zeroes odometry
        // and the gyro heading (the robot's current spot becomes (0, 0)).
        char* argv[] = { const_cast<char*>("TangentBug"), const_cast<char*>(kobukiPort_.c_str()) };
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
        s.raw = sensors_.reading();
        s.range = toCentre(s.raw, sensorOffset_, sensors_.maxRange());

        // Pose: heading from the gyro (accurate, no wheel-slip drift),
        // distance from the wheel encoders, integrated with the mid-point
        // heading. The driver's own x/y uses the encoder heading, which
        // drifts away from the gyro heading the planner steers by.
        const auto o = robot_.getOdometryData();
        const double dist = 0.5 * (o.left_distance + o.right_distance);
        const double heading = robot_.getHeading();
        const double d = dist - lastDist_;
        const double mid = lastHeading_ + 0.5 * tbug::normalizeAngle(heading - lastHeading_);
        x_ += d * std::cos(mid);
        y_ += d * std::sin(mid);
        lastDist_ = dist;
        lastHeading_ = heading;
        s.pose = { x_, y_, heading };

        // Kobuki's own safety sensors, independent of the planner.
        const auto core = robot_.getCoreSensorData();
        if (kobuki::motion::bumper(robot_) != kobuki::motion::Bump::None) s.fault = "bumper hit";
        else if (core.cliffLeft() || core.cliffCenter() || core.cliffRight()) s.fault = "cliff detected";
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
    double sensorOffset_;
    kobuki::Kobuki robot_;
    double x_ = 0.0, y_ = 0.0, lastDist_ = 0.0, lastHeading_ = 0.0;
    std::chrono::steady_clock::time_point lastTick_;
};

// ------------------------------------------------------------ simulator --
// Demo room (metres, start at the origin facing +x):
//   - outer room 8 x 8 m: x in [-2, 6], y in [-4, 4]
//   - box in front of the start:      x 1.2..1.8, y -0.6..0.6
//   - long wall:                      x = 3.2,    y -2.5..1.2
//   - U-shaped pocket opening to -x:  x 3.8..5.0, y 1.8..3.2
// Try e.g. "4 0", "4.5 -1", "4.4 2.5" (inside the pocket), "-1 3".
class SimRobot final : public RobotIO {
public:
    const char* name() const override { return "built-in simulator"; }

    explicit SimRobot(double beamAngle) : beamAngle_(beamAngle) {
        rect(-2, -4, 6, 4);
        rect(1.2, -0.6, 1.8, 0.6);
        walls_.push_back({3.2, -2.5, 3.2, 1.2});
        walls_.push_back({3.8, 1.8, 5.0, 1.8});
        walls_.push_back({5.0, 1.8, 5.0, 3.2});
        walls_.push_back({5.0, 3.2, 3.8, 3.2});
        for (auto& h : hist_) for (double& v : h) v = MaxRange;
    }

    bool open() override {
        std::cout << "Simulator: demo room 8 x 8 m, robot at (0, 0) facing +x.\n"
                  << "  Obstacles: box x 1.2..1.8 y -0.6..0.6 | wall x=3.2 y -2.5..1.2 |\n"
                  << "             pocket x 3.8..5.0 y 1.8..3.2 (open toward -x)\n";
        sample();
        return true;
    }

    Snapshot read() override {
        Snapshot s;
        s.pose = pose_;
        s.range = s.raw = reading_;
        s.newData = newData_;
        newData_ = false;
        if (nearestWall() < RobotRadius) s.fault = "collision (simulated)";
        return s;
    }

    void drive(double v, double w) override { v_ = v; w_ = w; }
    void stop() override { v_ = w_ = 0.0; }

    // 10 ms of simulated time per call, ~5x faster than real time.
    double tick() override {
        pose_.heading = tbug::normalizeAngle(pose_.heading + w_ * Dt);
        pose_.x += v_ * std::cos(pose_.heading) * Dt;
        pose_.y += v_ * std::sin(pose_.heading) * Dt;
        simTime_ += Dt;
        if (simTime_ >= nextSample_ - 1e-9) sample();
        if (++sleepCounter_ % 5 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return Dt;
    }

private:
    struct Seg { double x1, y1, x2, y2; };
    static constexpr double Dt = 0.01;
    static constexpr double SamplePeriod = 0.1;                     // Arduino packet rate
    static constexpr double ConeHalf = 10.0 * tbug::Pi / 180.0;     // HC-SR04-like beam
    static constexpr double MaxRange = 4.0;
    static constexpr double RobotRadius = 0.18;

    void rect(double x1, double y1, double x2, double y2) {
        walls_.push_back({x1, y1, x2, y1}); walls_.push_back({x2, y1, x2, y2});
        walls_.push_back({x2, y2, x1, y2}); walls_.push_back({x1, y2, x1, y1});
    }

    // Same as the Arduino sketch: measure all beams, report the minimum of
    // the last 2 samples per beam.
    void sample() {
        nextSample_ += SamplePeriod;
        const double a[3] = { pose_.heading + beamAngle_, pose_.heading, pose_.heading - beamAngle_ };
        for (int k = 0; k < 3; ++k) hist_[k][histIndex_] = cast(a[k]);
        histIndex_ = (histIndex_ + 1) % 2;
        reading_.left = std::min(hist_[0][0], hist_[0][1]);
        reading_.front = std::min(hist_[1][0], hist_[1][1]);
        reading_.right = std::min(hist_[2][0], hist_[2][1]);
        reading_.leftValid = reading_.frontValid = reading_.rightValid = true;
        newData_ = true;
    }

    // Nearest echo inside the beam cone (5 rays across +-ConeHalf).
    double cast(double a0) const {
        double best = MaxRange;
        for (int k = 0; k < 5; ++k) {
            const double a = a0 - ConeHalf + ConeHalf * k / 2.0;
            const double dx = std::cos(a), dy = std::sin(a);
            for (const auto& s : walls_) {
                const double sx = s.x2 - s.x1, sy = s.y2 - s.y1;
                const double den = dx * sy - dy * sx;
                if (std::abs(den) < 1e-12) continue;
                const double t = ((s.x1 - pose_.x) * sy - (s.y1 - pose_.y) * sx) / den;
                const double u = ((s.x1 - pose_.x) * dy - (s.y1 - pose_.y) * dx) / den;
                if (t >= 0.0 && u >= 0.0 && u <= 1.0) best = std::min(best, t);
            }
        }
        return best;
    }

    double nearestWall() const {
        double best = 1e9;
        for (const auto& s : walls_) {
            const double vx = s.x2 - s.x1, vy = s.y2 - s.y1;
            const double len2 = vx * vx + vy * vy;
            double t = len2 > 1e-12 ? ((pose_.x - s.x1) * vx + (pose_.y - s.y1) * vy) / len2 : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            best = std::min(best, std::hypot(pose_.x - (s.x1 + t * vx), pose_.y - (s.y1 + t * vy)));
        }
        return best;
    }

    double beamAngle_;
    std::vector<Seg> walls_;
    tbug::Pose pose_{};
    double v_ = 0.0, w_ = 0.0;
    double simTime_ = 0.0, nextSample_ = 0.0;
    double hist_[3][2]{};
    int histIndex_ = 0;
    tbug::SensorReading reading_;
    bool newData_ = false;
    unsigned sleepCounter_ = 0;
};

} // namespace app
