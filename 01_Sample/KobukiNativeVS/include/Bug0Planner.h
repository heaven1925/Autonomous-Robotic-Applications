// Bug0Planner.h - hardware-independent Bug0 decision core.
//
// This header has NO dependency on Kobuki, serial ports or Arduino: it only
// deals with plain doubles (pose, goal, sensor readings in metres) and
// returns a (v, w) velocity command plus the current state. That makes it
// usable both from the real hardware loop (07_Bug0Algorithm/src/main.cpp)
// and from a pure-software simulator (07_Bug0Algorithm/src/sim_main.cpp),
// satisfying "test the algorithm before touching real hardware".
//
// Sensors are assumed to look LEFT-FRONT, FRONT and RIGHT-FRONT (see
// Bug0.md section 3). A field is "valid" when the beam actually returned an
// echo; invalid/stale data is NEVER treated as "clear path" (Bug0.md,
// "Kontrol ve guvenlik ilkeleri").
//
// States (Bug0.md section 4): Init -> GoToGoal <-> AvoidObstacle ->
// FollowBoundary -> (GoToGoal | Recovery) -> GoalReached, with Fault as a
// terminal safe-stop. AvoidObstacle is transient: it only picks the side to
// follow and hands off to FollowBoundary on the next step() call.
#pragma once

#include <algorithm>
#include <cmath>

namespace bug0 {

constexpr double Pi = 3.14159265358979323846;

enum class State { Init, GoToGoal, AvoidObstacle, FollowBoundary, Recovery, GoalReached, Fault };
enum class Side { None, Left, Right };

struct Pose { double x = 0.0, y = 0.0, heading = 0.0; };
struct Goal { double x = 0.0, y = 0.0; };

struct SensorReading {
    double left = 0.0, front = 0.0, right = 0.0;
    bool leftValid = false, frontValid = false, rightValid = false;
};

struct Command { double v = 0.0, w = 0.0; };

struct Config {
    double d_stop      = 0.55;  // m   - front distance that forces a stop / avoid
    double d_slow       = 0.90;  // m   - front distance that starts slowing down
    double d_follow      = 0.60;  // m   - desired distance to the followed wall
    double d_clear       = 0.85;  // m   - side/front distance that counts as "open" again
    int    confirmReadings = 3;   // consecutive readings needed before changing decision (hysteresis)
    double robotRadius    = 0.18;  // m
    double sideEmergencyMargin = 0.22; // m - extra margin added to robotRadius for the hard side-stop
    double turnSpeed      = 0.5;   // rad/s - max in-place turn speed
    double minTurn        = 0.08;  // rad/s - min turn speed (avoids motor stall)
    double driveSpeed     = 0.15;  // m/s   - max straight-line speed
    double followSpeed    = 0.07;  // m/s   - speed while boundary-following
    double minDrive       = 0.04;  // m/s   - min forward speed when still allowed to move
    double goalTolerance  = 0.08;  // m     - goal counts as reached
    double headingTolGoTo = 10.0 * Pi / 180.0; // rad - must be aligned before driving straight
    double headingGain    = 2.0;   // P gain, turning towards the goal
    double wallGain        = 1.5;   // P gain, boundary distance control
    double maxFollowTimeS  = 30.0;  // s - safety: force recovery if stuck boundary-following
    double recoveryTimeS   = 2.0;   // s - duration of one recovery scan turn
    int    maxRecoveryAttempts = 4; // give up (Fault) after this many unsuccessful recoveries
    int    turnInPlaceTicks = 20;   // control ticks of forced pure rotation right after AvoidObstacle,
                                     // before any forward motion - a single ultrasonic ray can "see past"
                                     // the edge of a finite obstacle while the body still swings close to
                                     // its corner (Bug0.md "Kritik kısıt"); this is a deliberate, sensor-
                                     // independent safety margin rather than trusting the beam alone.
};

class Bug0Planner {
public:
    explicit Bug0Planner(Config cfg = {}) : cfg_(cfg) {}

    State state() const { return state_; }
    Side  followSide() const { return side_; }
    double distanceToGoal(const Pose& pose, const Goal& goal) const {
        return std::hypot(goal.x - pose.x, goal.y - pose.y);
    }

    // Advance the planner by dt seconds; returns the velocity command the
    // motion layer should apply for this tick.
    Command step(const SensorReading& s, const Pose& pose, const Goal& goal, double dt) {
        const double distToGoal = distanceToGoal(pose, goal);

        if (state_ != State::GoalReached && state_ != State::Fault && distToGoal < cfg_.goalTolerance) {
            state_ = State::GoalReached;
            return {0.0, 0.0};
        }

        switch (state_) {
            case State::Init:
                state_ = State::GoToGoal;
                resetCounters();
                [[fallthrough]];
            case State::GoToGoal:
                return doGoToGoal(s, pose, goal, distToGoal);
            case State::AvoidObstacle:
                return doAvoidObstacle(s);
            case State::FollowBoundary:
                return doFollowBoundary(s, pose, goal, dt);
            case State::Recovery:
                return doRecovery(s, dt);
            case State::GoalReached:
            case State::Fault:
            default:
                return {0.0, 0.0};
        }
    }

private:
    static double normalize(double a) {
        while (a > Pi) a -= 2.0 * Pi;
        while (a <= -Pi) a += 2.0 * Pi;
        return a;
    }

    void resetCounters() {
        frontCloseCount_ = 0;
        frontLostCount_ = 0;
        wallLostCount_ = 0;
        clearCount_ = 0;
        followTime_ = 0.0;
        turnInPlaceLeft_ = 0;
    }

    // ---------------------------------------------------------- GoToGoal --
    Command doGoToGoal(const SensorReading& s, const Pose& pose, const Goal& goal, double distToGoal) {
        // Hysteresis: only act on the front sensor after several consistent
        // readings - a single echo glitch must not change the state.
        if (s.frontValid && s.front < cfg_.d_stop) ++frontCloseCount_; else frontCloseCount_ = 0;
        if (!s.frontValid) ++frontLostCount_; else frontLostCount_ = 0;

        // Side beams are diagonal, so something they see very close is
        // almost touching the body right now - react on a single reading,
        // no hysteresis delay (e.g. a corridor wall the parallel front beam
        // can completely miss, see Bug0.md "Kritik kısıt").
        const double sideLimit = cfg_.robotRadius + cfg_.sideEmergencyMargin;
        const bool sideLeftClose = s.leftValid && s.left < sideLimit;
        const bool sideRightClose = s.rightValid && s.right < sideLimit;

        // Both sides close at once = a passage too narrow to trust (the
        // body is as wide as 2*robotRadius): avoiding into either side
        // would just drive further into the same squeeze, so stop and
        // back off instead of picking a side.
        if (sideLeftClose && sideRightClose) {
            state_ = State::Recovery;
            recoveryTimer_ = 0.0;
            return {0.0, 0.0};
        }

        if (frontCloseCount_ >= cfg_.confirmReadings || sideLeftClose || sideRightClose) {
            state_ = State::AvoidObstacle;
            return {0.0, 0.0};
        }
        if (frontLostCount_ >= cfg_.confirmReadings) {
            // Sensor data missing: do not drive blind, go re-evaluate slowly.
            state_ = State::Recovery;
            recoveryTimer_ = 0.0;
            return {0.0, 0.0};
        }

        const double goalHeading = std::atan2(goal.y - pose.y, goal.x - pose.x);
        const double headingErr = normalize(goalHeading - pose.heading);

        if (std::abs(headingErr) > cfg_.headingTolGoTo) {
            return { 0.0, clampTurn(cfg_.headingGain * headingErr) };
        }

        // Aligned: drive forward, slowing down as the front gets closer,
        // with a small heading correction to keep the line straight.
        double v = cfg_.driveSpeed;
        if (s.frontValid && s.front < cfg_.d_slow) {
            const double t = (s.front - cfg_.d_stop) / (cfg_.d_slow - cfg_.d_stop);
            v = std::max(cfg_.minDrive, cfg_.driveSpeed * std::clamp(t, 0.0, 1.0));
        }
        v = std::min(v, cfg_.driveSpeed * std::clamp(distToGoal / cfg_.goalTolerance / 4.0, 0.2, 1.0));
        const double w = clampTurn(cfg_.headingGain * headingErr);
        return { v, w };
    }

    // ------------------------------------------------------ AvoidObstacle --
    // Transient: pick the side with more clearance to turn into, lock the
    // opposite sensor as the "wall" sensor, and hand off to FollowBoundary.
    Command doAvoidObstacle(const SensorReading& s) {
        const bool haveLeft = s.leftValid;
        const bool haveRight = s.rightValid;

        if (haveLeft && haveRight) {
            side_ = (s.left >= s.right) ? Side::Right : Side::Left; // turn toward the clearer side
        } else if (haveLeft) {
            side_ = Side::Right; // only the left beam sees clear space -> turn left
        } else if (haveRight) {
            side_ = Side::Left;  // only the right beam sees clear space -> turn right
        } else {
            // Front blocked and both side beams blind: do not guess, recover.
            state_ = State::Recovery;
            recoveryTimer_ = 0.0;
            return {0.0, 0.0};
        }

        state_ = State::FollowBoundary;
        resetCounters();
        turnInPlaceLeft_ = cfg_.turnInPlaceTicks;
        return {0.0, 0.0}; // next tick starts the forced turn-in-place in FollowBoundary
    }

    // ------------------------------------------------------ FollowBoundary --
    Command doFollowBoundary(const SensorReading& s, const Pose& pose, const Goal& goal, double dt) {
        followTime_ += dt;
        if (followTime_ > cfg_.maxFollowTimeS) {
            state_ = State::Recovery;
            recoveryTimer_ = 0.0;
            return {0.0, 0.0};
        }

        const bool wallOnRight = (side_ == Side::Right);

        // Forced pure rotation right after entering: turn away from the wall
        // side for a fixed number of ticks before allowing any forward
        // motion, so the body has time to actually clear the obstacle's
        // corner instead of trusting a single ultrasonic ray not to have
        // already swept past the edge (see Config::turnInPlaceTicks).
        if (turnInPlaceLeft_ > 0) {
            --turnInPlaceLeft_;
            const double w = wallOnRight ? cfg_.turnSpeed : -cfg_.turnSpeed;
            return { 0.0, w };
        }

        const double wallReading = wallOnRight ? s.right : s.left;
        const bool wallValid = wallOnRight ? s.rightValid : s.leftValid;
        // "Seen" means the beam actually bounced off something close enough
        // to be the wall we are tracking - an out-of-range reading (>= d_clear)
        // is NOT the wall, it just means the beam currently points past it
        // (e.g. right at a corner). Only apply the distance P-control while
        // the wall is actually seen; otherwise a stale "too far" error would
        // swing the robot hard back towards the corner it just rounded.
        const bool wallSeen = wallValid && wallReading < cfg_.d_clear;

        // Departure check: front AND the wall-side beam both read clear for
        // several consecutive ticks -> the straight line ahead is probably
        // open again, Bug0's only criterion to leave the boundary.
        const bool frontClear = s.frontValid && s.front >= cfg_.d_clear;
        if (frontClear && !wallSeen) ++clearCount_; else clearCount_ = 0;
        if (clearCount_ >= cfg_.confirmReadings * 6) {
            state_ = State::GoToGoal;
            resetCounters();
            return {0.0, 0.0};
        }

        if (!wallSeen) ++wallLostCount_; else wallLostCount_ = 0;
        if (wallLostCount_ >= cfg_.confirmReadings * 4) {
            // Lost the wall for a good while and front is not clear either:
            // slow down and re-evaluate instead of driving blind.
            state_ = State::Recovery;
            recoveryTimer_ = 0.0;
            return {0.0, 0.0};
        }

        // Emergency: something is close in front while following - turn
        // away from the wall harder, regardless of the wall-distance P term.
        if (s.frontValid && s.front < cfg_.d_stop) {
            const double w = wallOnRight ? cfg_.turnSpeed : -cfg_.turnSpeed;
            return { 0.0, w };
        }

        // Side emergency: the tracked wall beam itself is reading close to
        // the robot's own radius - whatever the angle, that is too close to
        // keep driving forward. Stop and turn away immediately; this is a
        // direct, angle-independent backstop for corners a single ray can
        // under-estimate the true closest-point distance for.
        if (wallSeen && wallReading < cfg_.robotRadius + cfg_.sideEmergencyMargin) {
            const double w = wallOnRight ? cfg_.turnSpeed : -cfg_.turnSpeed;
            return { 0.0, w };
        }

        // Same check on the OPPOSITE side (e.g. the other wall of a narrow
        // corridor): if that beam suddenly reads close, stop and turn back
        // toward the tracked wall instead of squeezing further in.
        const double otherReading = wallOnRight ? s.left : s.right;
        const bool otherValid = wallOnRight ? s.leftValid : s.rightValid;
        if (otherValid && otherReading < cfg_.robotRadius + cfg_.sideEmergencyMargin) {
            const double w = wallOnRight ? -cfg_.turnSpeed : cfg_.turnSpeed;
            return { 0.0, w };
        }

        // Normal wall-following: P-control the distance to the wall only
        // while the wall is actually seen; otherwise turn gently toward
        // where it should be instead of over-correcting on a stale/far
        // reading (this is what kept pulling the robot into corners).
        double w = 0.0;
        if (wallSeen) {
            const double err = wallReading - cfg_.d_follow; // + = too far from the wall
            w = (wallOnRight ? -1.0 : 1.0) * cfg_.wallGain * err;
        } else {
            w = (wallOnRight ? -1.0 : 1.0) * cfg_.minTurn;
        }
        w = clampTurn(w);

        double v = cfg_.followSpeed;
        if (s.frontValid && s.front < cfg_.d_slow) {
            const double t = (s.front - cfg_.d_stop) / (cfg_.d_slow - cfg_.d_stop);
            v = std::max(cfg_.minDrive, cfg_.followSpeed * std::clamp(t, 0.0, 1.0));
        }
        (void)goal; (void)pose;
        return { v, w };
    }

    // ------------------------------------------------------------ Recovery --
    // Stop driving forward and slowly rotate in place for a short, bounded
    // time to get fresh sensor data, then go back to GoToGoal and let its
    // own hysteresis re-decide. Too many recoveries in a row -> Fault.
    Command doRecovery(const SensorReading& s, double dt) {
        recoveryTimer_ += dt;
        if (recoveryTimer_ > cfg_.recoveryTimeS) {
            ++recoveryAttempts_;
            if (recoveryAttempts_ > cfg_.maxRecoveryAttempts) {
                state_ = State::Fault;
                return {0.0, 0.0};
            }
            state_ = State::GoToGoal;
            resetCounters();
            return {0.0, 0.0};
        }
        if (s.frontValid && s.front < cfg_.d_stop) return {0.0, 0.0}; // still blocked, just wait
        return { 0.0, cfg_.minTurn }; // slow scan turn to the left
    }

    double clampTurn(double w) const {
        w = std::clamp(w, -cfg_.turnSpeed, cfg_.turnSpeed);
        if (std::abs(w) < cfg_.minTurn && std::abs(w) > 1e-6) w = (w < 0 ? -cfg_.minTurn : cfg_.minTurn);
        return w;
    }

    Config cfg_;
    State state_ = State::Init;
    Side side_ = Side::None;

    int frontCloseCount_ = 0;
    int frontLostCount_ = 0;
    int wallLostCount_ = 0;
    int clearCount_ = 0;
    double followTime_ = 0.0;
    double recoveryTimer_ = 0.0;
    int recoveryAttempts_ = 0;
    int turnInPlaceLeft_ = 0;
};

} // namespace bug0
