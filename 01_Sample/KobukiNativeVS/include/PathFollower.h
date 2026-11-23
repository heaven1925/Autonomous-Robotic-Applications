// PathFollower.h - drives a differential robot along a polyline of
// waypoints: "turn on the spot, then drive straight" for every leg.
//
//   Turn  : rotate in place until the robot faces the end of the leg
//           (heading error < turnTolDeg).
//   Drive : follow the straight leg A -> B. The heading set-point is the leg
//           direction corrected by the cross-track error (the robot steers
//           back onto the line, it does not just aim at B), the speed ramps
//           down near B. The leg is done once the robot is within goalTol of
//           B or has passed it.
// Legs come from A* corner points, so they are collision free by
// construction; the follower only has to stay on the line.
// Used by 10_AStarPathPlanning and 11_ExplorationMapping.
#pragma once

#include "AStarPlanner.h"
#include "NavCommon.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace nav {

struct FollowerConfig {
    double vMax = 0.15;        // m/s   straight-line speed (same as Motion.h)
    double vMin = 0.03;        // m/s   creep speed at the end of a leg
    double wMax = 0.8;         // rad/s turning speed limit
    double wMin = 0.15;        // rad/s smallest on-the-spot turning speed (Kobuki stalls below)
    double kTurn = 1.8;        // P gain, on-the-spot turn
    double kHeading = 2.5;     // P gain, heading while driving
    double kDecel = 0.6;       // 1/s   v <= kDecel * remaining distance
    double lookahead = 0.20;   // m     cross-track correction: atan(e / lookahead)
    double turnTolDeg = 3.0;   // deg   on-the-spot turn is done
    double realignDeg = 35.0;  // deg   heading error that stops driving and turns again
    double goalTol = 0.01;     // m     a waypoint counts as reached
};

struct Command {
    double v = 0.0, w = 0.0;
};

class PathFollower {
public:
    enum class Phase { Idle, Turn, Drive, Done };

    explicit PathFollower(FollowerConfig cfg = {}) : cfg_(cfg) {}

    // pts[0] is where the robot is now; legs pts[0]->pts[1]->...
    void setPath(std::vector<astar::Point> pts) {
        pts_ = std::move(pts);
        leg_ = 0;
        phase_ = pts_.size() >= 2 ? Phase::Turn : Phase::Done;
    }

    Command step(const Pose& p) {
        Command cmd;
        if (phase_ == Phase::Done || phase_ == Phase::Idle) return cmd;
        const astar::Point A = pts_[leg_], B = pts_[leg_ + 1];
        const double dx = B.x - A.x, dy = B.y - A.y;
        const double L = std::hypot(dx, dy);
        const double dir = std::atan2(dy, dx);

        // Position relative to the leg: along-track s, cross-track e (left +).
        const double rx = p.x - A.x, ry = p.y - A.y;
        const double s = L > 1e-9 ? (rx * dx + ry * dy) / L : 0.0;
        const double e = L > 1e-9 ? (dx * ry - dy * rx) / L : 0.0;
        const double remaining = L - s;

        if (remaining <= cfg_.goalTol || L < 1e-9) return nextLeg(p);

        if (phase_ == Phase::Turn) {
            // Face B itself (not just the leg direction), so a robot that is a
            // little off the line starts heading back towards it.
            const double err = normalizeAngle(std::atan2(B.y - p.y, B.x - p.x) - p.heading);
            if (std::abs(err) < cfg_.turnTolDeg * Deg) {
                phase_ = Phase::Drive;
            } else {
                const double w = std::clamp(cfg_.kTurn * std::abs(err), cfg_.wMin, cfg_.wMax);
                cmd.w = err > 0.0 ? w : -w;
                return cmd;
            }
        }

        // Drive: heading set-point = leg direction, pulled back onto the line.
        const double want = dir - std::atan2(e, cfg_.lookahead);
        const double err = normalizeAngle(want - p.heading);
        if (std::abs(err) > cfg_.realignDeg * Deg) {
            phase_ = Phase::Turn;
            return cmd;
        }
        cmd.v = std::clamp(cfg_.kDecel * remaining, cfg_.vMin, cfg_.vMax) * std::max(0.0, std::cos(err));
        cmd.w = std::clamp(cfg_.kHeading * err, -cfg_.wMax, cfg_.wMax);
        return cmd;
    }

    Phase phase() const { return phase_; }
    bool done() const { return phase_ == Phase::Done; }
    int leg() const { return static_cast<int>(leg_); }
    int legs() const { return pts_.size() >= 2 ? static_cast<int>(pts_.size()) - 1 : 0; }
    const std::vector<astar::Point>& points() const { return pts_; }

    // Largest distance from the planned line seen so far (m).
    double maxCrossTrack() const { return maxCross_; }
    void trackError(const Pose& p) {
        if (phase_ != Phase::Drive) return;
        const astar::Point A = pts_[leg_], B = pts_[leg_ + 1];
        const double L = astar::distance(A, B);
        if (L < 1e-9) return;
        const double e = ((B.x - A.x) * (p.y - A.y) - (B.y - A.y) * (p.x - A.x)) / L;
        maxCross_ = std::max(maxCross_, std::abs(e));
    }

private:
    Command nextLeg(const Pose&) {
        ++leg_;
        if (leg_ + 1 >= pts_.size()) {
            phase_ = Phase::Done;
            leg_ = pts_.size() >= 2 ? pts_.size() - 2 : 0;
        } else {
            phase_ = Phase::Turn;
        }
        return {};
    }

    FollowerConfig cfg_;
    std::vector<astar::Point> pts_;
    std::size_t leg_ = 0;
    Phase phase_ = Phase::Idle;
    double maxCross_ = 0.0;
};

} // namespace nav
