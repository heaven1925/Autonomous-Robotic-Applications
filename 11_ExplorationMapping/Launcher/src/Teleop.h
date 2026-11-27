// Teleop.h - drive the Kobuki by hand from the launcher ("Elle sürüş ve sensör
// testi" panel) and show its sensors.
//
// Safety:
//   - the driver cuts forward speed on a bumper press (stop_on_bump)
//   - hold-to-drive: command() must be repeated; 300 ms without one -> stop
//     (so a hung window, a lost focus or a released key stops the robot)
//   - fixed moves (one cell forward, 90 deg turns) end on a bumper press, a
//     wheel drop, a cliff or a time-out
//   - speeds are limited to MaxLinear / MaxAngular
#pragma once

#include "Kobuki.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace teleop {

constexpr double Pi = 3.14159265358979323846;
constexpr double MaxLinear = 0.20, MaxAngular = 1.0;   // m/s, rad/s

enum class State { Closed, Opening, Open, Error };
enum class Goal { None, Move, Turn };

struct Status {
    State state = State::Closed;
    std::wstring error;
    bool alive = false;
    kobuki::CoreSensors::Data core;
    double headingDeg = 0.0, x = 0.0, y = 0.0, distance = 0.0;
    double v = 0.0, w = 0.0;           // command being sent
    Goal goal = Goal::None;
    double goalLeft = 0.0;             // m or deg still to go
    std::wstring note;                 // how the last fixed move ended
    bool seenLeft = false, seenCenter = false, seenRight = false;   // bumpers pressed at least once
};

class Robot {
public:
    ~Robot() { close(); }

    void open(const std::string& port) {
        close();
        stop_ = false;
        {
            std::lock_guard<std::mutex> lock(mu_);
            st_ = Status{};
            st_.state = State::Opening;
        }
        thread_ = std::thread([this, port] { loop(port); });
    }

    void close() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        std::lock_guard<std::mutex> lock(mu_);
        st_.state = State::Closed;
    }

    bool isOpen() const { return thread_.joinable() && !stop_; }

    // Hold-to-drive: call again within 300 ms to keep moving.
    void command(double v, double w) {
        std::lock_guard<std::mutex> lock(mu_);
        if (goal_ != Goal::None && (v != 0.0 || w != 0.0)) goal_ = Goal::None;   // a key takes over
        cmdV_ = std::clamp(v, -MaxLinear, MaxLinear);
        cmdW_ = std::clamp(w, -MaxAngular, MaxAngular);
        cmdTime_ = Clock::now();
    }

    void halt() {
        std::lock_guard<std::mutex> lock(mu_);
        goal_ = Goal::None;
        cmdV_ = cmdW_ = 0.0;
        st_.note = L"Durduruldu";
    }

    void move(double metres) { startGoal(Goal::Move, metres); }
    void turn(double degrees) { startGoal(Goal::Turn, degrees); }
    void resetPose() { resetPose_ = true; }
    void clearBumperTest() {
        std::lock_guard<std::mutex> lock(mu_);
        st_.seenLeft = st_.seenCenter = st_.seenRight = false;
    }

    Status status() const {
        std::lock_guard<std::mutex> lock(mu_);
        return st_;
    }

private:
    using Clock = std::chrono::steady_clock;

    void startGoal(Goal g, double amount) {
        std::lock_guard<std::mutex> lock(mu_);
        goal_ = g;
        goalAmount_ = amount;
        goalNew_ = true;
        st_.note.clear();
    }

    static double wrap(double a) {
        while (a > Pi) a -= 2.0 * Pi;
        while (a <= -Pi) a += 2.0 * Pi;
        return a;
    }

    void fail(const std::wstring& text) {
        std::lock_guard<std::mutex> lock(mu_);
        st_.state = State::Error;
        st_.error = text;
    }

    void loop(std::string port) {
        auto robot = std::make_unique<kobuki::Kobuki>();
        try {
            kobuki::Parameters p;
            p.device_port = port;
            p.command_rate_hz = 50;
            p.stop_on_bump = true;
            robot->init(p);
        } catch (const std::exception&) {
            fail(L"Port açılamadı (başka programda açık olabilir)");
            return;
        }
        for (int i = 0; i < 150 && !stop_ && !robot->isAlive(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (stop_) return;
        if (!robot->isAlive()) {
            fail(L"Kobuki veri göndermiyor (güç açık mı, doğru port mu?)");
            return;
        }
        robot->resetOdometry();
        robot->enable();
        {
            std::lock_guard<std::mutex> lock(mu_);
            st_.state = State::Open;
        }

        double goalStart = 0.0, goalTarget = 0.0;
        Clock::time_point goalT0{};
        double heading0 = robot->getHeading();
        while (!stop_) {
            const auto core = robot->getCoreSensorData();
            const auto odo = robot->getOdometryData();
            const double heading = robot->getHeading();
            const double dist = 0.5 * (odo.left_distance + odo.right_distance);
            if (resetPose_.exchange(false)) { robot->resetOdometry(); heading0 = heading; }

            double v = 0.0, w = 0.0;
            std::wstring note;
            Goal goal;
            double left = 0.0;
            {
                std::lock_guard<std::mutex> lock(mu_);
                goal = goal_;
                if (goal != Goal::None && goalNew_) {
                    goalNew_ = false;
                    goalStart = goal == Goal::Move ? dist : heading;
                    goalTarget = goal == Goal::Move ? goalAmount_ : goalAmount_ * Pi / 180.0;
                    goalT0 = Clock::now();
                }
                if (goal == Goal::None) {
                    const bool fresh = std::chrono::duration<double>(Clock::now() - cmdTime_).count() < 0.3;
                    if (fresh) { v = cmdV_; w = cmdW_; }
                }
            }
            if (goal != Goal::None) {
                const double elapsed = std::chrono::duration<double>(Clock::now() - goalT0).count();
                const bool unsafe = core.wheel_drop != 0 || core.cliff != 0;
                if (goal == Goal::Move) {
                    const double done = dist - goalStart;
                    left = goalTarget - done;
                    const double dir = goalTarget >= 0 ? 1.0 : -1.0;
                    if (left * dir <= 0.003) note = L"✓ Tamamlandı";
                    else if (dir > 0 && core.bumper != 0) note = L"Tampon: ileri hareket kesildi";
                    else v = dir * std::clamp(std::abs(left) * 1.2, 0.04, 0.12);
                } else {
                    const double err = wrap(goalStart + goalTarget - heading);
                    left = err * 180.0 / Pi;
                    if (std::abs(err) < 0.9 * Pi / 180.0) note = L"✓ Tamamlandı";
                    else w = (err > 0 ? 1.0 : -1.0) * std::clamp(std::abs(err) * 1.5, 0.15, 0.6);
                }
                if (note.empty() && unsafe) { note = L"Tekerlek / zemin sensörü: durdu"; v = w = 0.0; }
                if (note.empty() && elapsed > 12.0) { note = L"Zaman aşımı: durdu"; v = w = 0.0; }
            }
            robot->setBaseControl(v, w);
            {
                std::lock_guard<std::mutex> lock(mu_);
                if (!note.empty() && goal_ == goal) { goal_ = Goal::None; st_.note = note; }
                st_.alive = robot->isAlive();
                st_.core = core;
                st_.headingDeg = wrap(heading - heading0) * 180.0 / Pi;
                st_.x = odo.x;
                st_.y = odo.y;
                st_.distance = dist;
                st_.v = v;
                st_.w = w;
                st_.goal = goal_;
                st_.goalLeft = left;
                st_.seenLeft |= core.bumperLeft();
                st_.seenCenter |= core.bumperCenter();
                st_.seenRight |= core.bumperRight();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        robot->setBaseControl(0.0, 0.0);
        robot->disable();
    }

    mutable std::mutex mu_;
    Status st_;
    std::thread thread_;
    std::atomic<bool> stop_{ true }, resetPose_{ false };
    double cmdV_ = 0.0, cmdW_ = 0.0;
    Clock::time_point cmdTime_{};
    Goal goal_ = Goal::None;
    double goalAmount_ = 0.0;
    bool goalNew_ = false;
};

} // namespace teleop
