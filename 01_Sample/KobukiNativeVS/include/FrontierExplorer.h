// FrontierExplorer.h - frontier based exploration on an occupancy grid.
//
// Hardware independent, like the other planner headers: the occupancy grid,
// the robot pose and the three ranges in, a (v, w) velocity command out.
//
// This is step 3 of the lecture's mapping algorithm
// (09_Mapping/lecture-mapping.pdf, "Frontier Based Exploration"):
//   1. threshold the map into empty / occupied / unknown;
//   2. find the frontier - unknown cells with at least one empty neighbour -
//      and take the empty neighbours the robot can actually reach as goals;
//   3. pick the closest one, plan a path to it over empty cells and drive it;
//   4. loop until no frontier is left: in a finite world the map is then
//      complete. (The robot finally drives back to where it started.)
//
// What the lecture leaves open and this file decides:
//   * Path planning - Dijkstra over the empty cells. Cells closer to an
//     occupied cell than the robot radius (+ margin) are never entered; cells
//     a little farther cost extra, so paths keep to the middle of free space.
//   * The robot only sees three narrow cones ahead of it, so arriving at a
//     frontier it turns once on the spot ("scan") to look in every direction.
//   * Goals that turn out to be unreachable in practice are blacklisted, so
//     the loop always terminates.
//   * A reflex below the planner: never drive forward into something the
//     front sensor sees closer than dStop, whatever the map says.
#pragma once

#include "OccupancyGrid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace explore {

using ogm::Pi;
using ogm::normalizeAngle;

enum class State { Init, Scan, Plan, Drive, BackOff, Align, Done, Fault };
enum class GoalKind { None, Frontier, Home };
enum class FaultReason { None, TimeLimit, Stuck };

struct Pose { double x = 0.0, y = 0.0, heading = 0.0; };
struct Point { double x = 0.0, y = 0.0; };
struct Command { double v = 0.0, w = 0.0; };

// Ranges in metres from the robot CENTRE along each beam. "Nothing seen" is
// simply a large value; an invalid field is never treated as free space.
struct Ranges {
    double left = 0.0, front = 0.0, right = 0.0;
    bool leftValid = false, frontValid = false, rightValid = false;
};

struct Config {
    // --- robot / safety ---------------------------------------------------
    double robotRadius   = 0.18;  // m
    double lethalMargin  = 0.06;  // m - cells nearer than robotRadius + this to an occupied cell are never entered
    double inflation     = 0.45;  // m - extra path cost fades out at this distance from occupied cells
    double clearanceCost = 8.0;   //     cost multiplier right next to the forbidden zone
    double escapeRadius  = 0.30;  // m - forbidden cells this close to the robot may be crossed (to get out of them)
    double dStop         = 0.38;  // m - front range (from the centre) below which the robot never drives forward
    double dSlow         = 0.75;  // m - front range where slowing down starts
    double sideMargin    = 0.08;  // m - a side beam nearer than robotRadius + this -> creep

    // --- motion -----------------------------------------------------------
    double driveSpeed    = 0.18;  // m/s
    double minDrive      = 0.04;  // m/s
    double turnSpeed     = 0.60;  // rad/s
    double minTurn       = 0.08;  // rad/s
    double scanSpeed     = 0.50;  // rad/s - the look-around turn (sensor packets arrive at 10 Hz -> ~3 deg apart)
    double headingGain   = 2.0;
    double lookahead     = 0.30;  // m   - the robot steers at the path point this far ahead
    double turnInPlace   = 60.0 * Pi / 180.0; // rad - heading error above which it turns without driving
    double arriveDist    = 0.12;  // m
    double backOffDist   = 0.12;  // m   - reversed after a bumper hit / when stuck
    double backOffSpeed  = 0.08;  // m/s

    // --- exploration ------------------------------------------------------
    double frontierMinSize = 0.30; // m  - smaller frontier fragments (single unknown cells) are not worth a trip
    double exploreRadius   = 8.0;  // m  - frontiers farther than this from the start are ignored (open doors!)
    double scanSpacing     = 0.25; // m  - no second look-around turn closer than this to the previous one
    double replanPeriod    = 1.5;  // s  - the path is re-planned on the growing map this often while driving
    double stickyDist      = 0.60; // m  - keep heading for the same frontier while it only moves this much
    double blacklistRadius = 0.30; // m
    double blockedTimeS    = 3.0;  // s  - front blocked this long while trying to drive -> give the goal up
    double stuckTimeS      = 8.0;  // s  - no movement for this long while driving -> give the goal up
    int    maxStuck        = 12;   //      given-up goals / bumper hits before Fault(Stuck)
    double maxTimeS        = 1800.0; // s - whole-run limit -> Fault(TimeLimit); 0 = none
    bool   returnHome      = true;
};

class FrontierExplorer {
public:
    explicit FrontierExplorer(Config cfg = {}) : cfg_(cfg), lethal_(cfg.robotRadius + cfg.lethalMargin) {}

    State state() const { return state_; }
    GoalKind goal() const { return goal_; }
    FaultReason faultReason() const { return fault_; }
    bool finished() const { return state_ == State::Done || state_ == State::Fault; }

    // For display / logging.
    const std::vector<Point>& path() const { return path_; }
    const std::vector<Point>& frontier() const { return frontier_; }  // goal cells of the last plan
    bool hasTarget() const { return haveTarget_ && state_ == State::Drive; }
    Point target() const { return target_; }
    Point home() const { return home_; }
    int frontierCount() const { return clusters_; }  // separate frontiers in the last plan
    int scans() const { return scans_; }
    int plans() const { return plans_; }
    int givenUp() const { return stuck_; }
    double elapsed() const { return time_; }

    // The application saw a bumper press: back off and plan again (the
    // application also marks the touched spot occupied in the grid).
    void notifyBump(const Pose& pose) {
        if (finished() || state_ == State::BackOff) return;
        if (++stuck_ > cfg_.maxStuck) { fail(FaultReason::Stuck); return; }
        startBackOff(pose);
    }

    // Advance by dt seconds; returns the velocity command to apply.
    Command step(const ogm::OccupancyGrid& grid, const Pose& pose, const Ranges& r, double dt) {
        time_ += dt;
        if (finished()) return {};
        if (cfg_.maxTimeS > 0.0 && time_ > cfg_.maxTimeS) { fail(FaultReason::TimeLimit); return {}; }

        switch (state_) {
            case State::Init:
                home_ = { pose.x, pose.y };
                homeHeading_ = pose.heading;
                progressPos_ = home_;
                startScan(pose);
                [[fallthrough]];
            case State::Scan:    return doScan(pose);
            case State::Plan:    if (choosePath(grid, pose)) enterDrive(pose); return {};
            case State::Drive:   return doDrive(grid, pose, r, dt);
            case State::BackOff: return doBackOff(pose, dt);
            case State::Align:   return doAlign(pose);
            default:             return {};
        }
    }

private:
    static constexpr float Inf = std::numeric_limits<float>::infinity();

    static double dist(const Point& a, const Point& b) { return std::hypot(a.x - b.x, a.y - b.y); }
    static Point pos(const Pose& p) { return { p.x, p.y }; }

    double clampTurn(double w) const {
        w = std::clamp(w, -cfg_.turnSpeed, cfg_.turnSpeed);
        if (std::abs(w) < cfg_.minTurn && std::abs(w) > 1e-6) w = (w < 0 ? -cfg_.minTurn : cfg_.minTurn);
        return w;
    }

    void fail(FaultReason why) {
        state_ = State::Fault;
        fault_ = why;
        path_.clear();
    }

    // ---------------------------------------------------------------- Scan --
    // One full turn on the spot so the three forward cones sweep every
    // direction. Always safe for the round base.
    void startScan(const Pose& pose) {
        state_ = State::Scan;
        scanTurned_ = 0.0;
        scanLastHeading_ = pose.heading;
        path_.clear();
    }

    Command doScan(const Pose& pose) {
        scanTurned_ += std::abs(normalizeAngle(pose.heading - scanLastHeading_));
        scanLastHeading_ = pose.heading;
        if (scanTurned_ >= 2.0 * Pi) {
            lastScanPos_ = pos(pose);
            haveScan_ = true;
            ++scans_;
            state_ = State::Plan;
            return {};
        }
        return { 0.0, cfg_.scanSpeed };
    }

    // --------------------------------------------------------------- Drive --
    void enterDrive(const Pose& pose) {
        state_ = State::Drive;
        progress_ = 0;
        replanTimer_ = 0.0;
        blockedTimer_ = 0.0;
        stuckTimer_ = 0.0;
        progressPos_ = pos(pose);
    }

    Command doDrive(const ogm::OccupancyGrid& grid, const Pose& pose, const Ranges& r, double dt) {
        const Point here = pos(pose);

        // Watchdog: the robot has to get somewhere.
        if (dist(here, progressPos_) > 0.05) { progressPos_ = here; stuckTimer_ = 0.0; }
        else stuckTimer_ += dt;
        if (stuckTimer_ > cfg_.stuckTimeS) { giveUp(pose); return {}; }

        // The map grows while driving: re-plan on it regularly. The frontier
        // may have been seen (and so disappeared) from a distance already.
        replanTimer_ += dt;
        if (replanTimer_ >= cfg_.replanPeriod) {
            replanTimer_ = 0.0;
            if (!choosePath(grid, pose)) return {};
            progress_ = 0;
        }
        if (path_.empty()) { state_ = State::Plan; return {}; }

        if (dist(here, path_.back()) < cfg_.arriveDist) return arrived(pose);

        // Closest path point from the last one on, then the "carrot" one
        // lookahead further.
        std::size_t best = progress_;
        double bestD = std::numeric_limits<double>::infinity();
        for (std::size_t i = progress_; i < path_.size() && i < progress_ + 60; ++i) {
            const double d = dist(path_[i], here);
            if (d < bestD) { bestD = d; best = i; }
        }
        progress_ = best;
        std::size_t carrot = progress_;
        while (carrot + 1 < path_.size() && dist(path_[carrot], here) < cfg_.lookahead) ++carrot;

        const double err = normalizeAngle(std::atan2(path_[carrot].y - pose.y, path_[carrot].x - pose.x) - pose.heading);
        const double w = clampTurn(cfg_.headingGain * err);

        // Arc steering: full speed when aligned, slower the more the robot
        // has to turn, on the spot beyond turnInPlace.
        double v = 0.0;
        if (std::abs(err) < cfg_.turnInPlace) {
            v = cfg_.driveSpeed * std::cos(err * (0.5 * Pi / cfg_.turnInPlace));
            if (r.frontValid && r.front < cfg_.dSlow)
                v = std::min(v, cfg_.driveSpeed * (r.front - cfg_.dStop) / (cfg_.dSlow - cfg_.dStop));
            v = std::min(v, std::max(cfg_.minDrive, 1.5 * dist(here, path_.back())));
            const double side = cfg_.robotRadius + cfg_.sideMargin;
            if ((r.leftValid && r.left < side) || (r.rightValid && r.right < side)) v = std::min(v, cfg_.minDrive);
            v = std::max(v, cfg_.minDrive);
        }

        // Reflex below the planner: never drive into what the front beam sees.
        const bool frontBlocked = !r.frontValid || r.front < cfg_.dStop;
        if (v > 0.0 && frontBlocked) {
            v = 0.0;
            blockedTimer_ = std::abs(err) < 20.0 * Pi / 180.0 ? blockedTimer_ + dt : 0.0;
            if (blockedTimer_ > cfg_.blockedTimeS) { giveUp(pose); return {}; }
        } else {
            blockedTimer_ = 0.0;
        }
        return { v, w };
    }

    Command arrived(const Pose& pose) {
        if (goal_ == GoalKind::Home) {
            path_.clear();
            state_ = State::Align;
            return {};
        }
        // At the frontier: look around - unless that was just done right here
        // and the frontier is still there (the sensors cannot resolve it).
        if (!haveScan_ || dist(pos(pose), lastScanPos_) >= cfg_.scanSpacing) {
            startScan(pose);
            return {};
        }
        blacklist_.push_back(target_);
        state_ = State::Plan;
        return {};
    }

    // The current goal cannot be reached in practice: forget it, back off a
    // little and plan something else.
    void giveUp(const Pose& pose) {
        if (++stuck_ > cfg_.maxStuck) { fail(FaultReason::Stuck); return; }
        if (goal_ == GoalKind::Frontier && haveTarget_) blacklist_.push_back(target_);
        startBackOff(pose);
    }

    // ------------------------------------------------------------- BackOff --
    void startBackOff(const Pose& pose) {
        state_ = State::BackOff;
        backStart_ = pos(pose);
        backTimer_ = 0.0;
        path_.clear();
    }

    Command doBackOff(const Pose& pose, double dt) {
        backTimer_ += dt;
        if (dist(pos(pose), backStart_) >= cfg_.backOffDist || backTimer_ > 3.0) {
            state_ = State::Plan;
            return {};
        }
        return { -cfg_.backOffSpeed, 0.0 };
    }

    // --------------------------------------------------------------- Align --
    // Back home: face the way the robot faced at the start, then done.
    Command doAlign(const Pose& pose) {
        const double err = normalizeAngle(homeHeading_ - pose.heading);
        if (std::abs(err) < 3.0 * Pi / 180.0) {
            state_ = State::Done;
            return {};
        }
        return { 0.0, clampTurn(cfg_.headingGain * err) };
    }

    // ------------------------------------------------------------ planning --
    // Decide where to go next and plan the path there. Returns false when
    // there is nowhere left to go (the state is then Align / Done).
    bool choosePath(const ogm::OccupancyGrid& grid, const Pose& pose) {
        ++plans_;
        if (buildField(grid, pose)) {
            if (selectFrontier(grid)) { goal_ = GoalKind::Frontier; return true; }
            // No frontier left: the map is complete.
            if (cfg_.returnHome && dist(pos(pose), home_) >= cfg_.arriveDist && pathToHome(grid)) {
                goal_ = GoalKind::Home;
                return true;
            }
        }
        goal_ = GoalKind::None;
        haveTarget_ = false;
        path_.clear();
        state_ = cfg_.returnHome ? State::Align : State::Done;
        return false;
    }

    std::size_t idx(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(ww_) + static_cast<std::size_t>(x); }
    Point world(const ogm::OccupancyGrid& g, int x, int y) const { return { g.toWorld(wx0_ + x), g.toWorld(wy0_ + y) }; }

    // Snapshot of the known part of the grid, distance to the nearest
    // occupied cell ("clearance"), and the cost of the cheapest path from the
    // robot to every reachable empty cell (Dijkstra).
    bool buildField(const ogm::OccupancyGrid& g, const Pose& pose) {
        int bx0, by0, bx1, by1;
        if (!g.knownBounds(bx0, by0, bx1, by1)) return false;
        cell_ = g.cellSize();
        wx0_ = std::max(0, bx0 - 2);
        wy0_ = std::max(0, by0 - 2);
        ww_ = std::min(g.size() - 1, bx1 + 2) - wx0_ + 1;
        wh_ = std::min(g.size() - 1, by1 + 2) - wy0_ + 1;
        const std::size_t N = static_cast<std::size_t>(ww_) * static_cast<std::size_t>(wh_);

        if (kernel_.empty() || kernelCell_ != cell_) {
            kernel_.clear();
            kernelCell_ = cell_;
            const int kr = static_cast<int>(std::ceil(cfg_.inflation / cell_));
            for (int dy = -kr; dy <= kr; ++dy)
                for (int dx = -kr; dx <= kr; ++dx) {
                    const double d = std::hypot(dx, dy) * cell_;
                    if (d < cfg_.inflation) kernel_.push_back({ dx, dy, static_cast<float>(d) });
                }
        }

        cells_.resize(N);
        clear_.assign(N, static_cast<float>(cfg_.inflation));
        for (int y = 0; y < wh_; ++y)
            for (int x = 0; x < ww_; ++x)
                cells_[idx(x, y)] = g.state(wx0_ + x, wy0_ + y);
        for (int y = 0; y < wh_; ++y) {
            for (int x = 0; x < ww_; ++x) {
                if (cells_[idx(x, y)] != ogm::Cell::Occupied) continue;
                for (const KernelCell& k : kernel_) {
                    const int nx = x + k.dx, ny = y + k.dy;
                    if (nx < 0 || ny < 0 || nx >= ww_ || ny >= wh_) continue;
                    float& c = clear_[idx(nx, ny)];
                    if (k.d < c) c = k.d;
                }
            }
        }

        const int rx = g.toCell(pose.x) - wx0_, ry = g.toCell(pose.y) - wy0_;
        if (rx < 0 || ry < 0 || rx >= ww_ || ry >= wh_) return false;
        robot_ = pos(pose);

        cost_.assign(N, Inf);
        parent_.assign(N, -1);
        using Node = std::pair<float, int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        const int start = static_cast<int>(idx(rx, ry));
        cost_[static_cast<std::size_t>(start)] = 0.0f;
        open.push({ 0.0f, start });

        static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static const int DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
        const float cell = static_cast<float>(cell_);
        while (!open.empty()) {
            const Node node = open.top();
            open.pop();
            const std::size_t i = static_cast<std::size_t>(node.second);
            if (node.first > cost_[i]) continue;
            const int x = node.second % ww_, y = node.second / ww_;
            for (int k = 0; k < 8; ++k) {
                const int nx = x + DX[k], ny = y + DY[k];
                if (nx < 0 || ny < 0 || nx >= ww_ || ny >= wh_) continue;
                if (!passable(g, nx, ny)) continue;
                // No cutting across the corner of a blocked cell.
                if (k >= 4 && (!passable(g, nx, y) || !passable(g, x, ny))) continue;
                const std::size_t j = idx(nx, ny);
                const float step = (k >= 4 ? 1.41421356f : 1.0f) * cell * (1.0f + penalty(clear_[j]));
                const float c = node.first + step;
                if (c < cost_[j]) {
                    cost_[j] = c;
                    parent_[j] = node.second;
                    open.push({ c, static_cast<int>(j) });
                }
            }
        }
        return true;
    }

    // Only cells known to be empty are driven on, and none inside the
    // forbidden zone around occupied cells - except right around the robot,
    // so it can always leave such a zone it finds itself in.
    bool passable(const ogm::OccupancyGrid& g, int x, int y) const {
        const std::size_t i = idx(x, y);
        if (cells_[i] != ogm::Cell::Empty) return false;
        if (clear_[i] >= static_cast<float>(lethal_)) return true;
        return dist(world(g, x, y), robot_) <= cfg_.escapeRadius;
    }

    float penalty(float clearance) const {
        const float lethal = static_cast<float>(lethal_), infl = static_cast<float>(cfg_.inflation);
        if (clearance < lethal) return 30.0f;
        if (clearance >= infl) return 0.0f;
        const float t = (infl - clearance) / (infl - lethal);
        return static_cast<float>(cfg_.clearanceCost) * t * t;
    }

    bool touchesUnknown(int x, int y) const {
        static const int DX[4] = { 1, -1, 0, 0 };
        static const int DY[4] = { 0, 0, 1, -1 };
        for (int k = 0; k < 4; ++k) {
            const int nx = x + DX[k], ny = y + DY[k];
            if (nx < 0 || ny < 0 || nx >= ww_ || ny >= wh_) return true;
            if (cells_[idx(nx, ny)] == ogm::Cell::Unknown) return true;
        }
        return false;
    }

    bool blacklisted(const Point& p) const {
        for (const Point& b : blacklist_)
            if (dist(p, b) <= cfg_.blacklistRadius) return true;
        return false;
    }

    // Frontier goals = reachable empty cells next to an unknown cell. They
    // are grouped into connected frontiers; the robot heads for the middle of
    // the frontier that is cheapest to reach.
    bool selectFrontier(const ogm::OccupancyGrid& g) {
        const std::size_t N = cost_.size();
        mark_.assign(N, 0);
        frontier_.clear();
        const float lethal = static_cast<float>(lethal_);
        for (int y = 0; y < wh_; ++y) {
            for (int x = 0; x < ww_; ++x) {
                const std::size_t i = idx(x, y);
                if (cost_[i] == Inf || clear_[i] < lethal || !touchesUnknown(x, y)) continue;
                const Point w = world(g, x, y);
                if (dist(w, home_) > cfg_.exploreRadius || blacklisted(w)) continue;
                mark_[i] = 1;
                frontier_.push_back(w);
            }
        }

        clusters_ = 0;
        int bestRep = -1, stickyRep = -1;
        float bestCost = Inf, stickyCost = Inf;
        for (std::size_t s = 0; s < N; ++s) {
            if (mark_[s] != 1) continue;
            members_.clear();
            stack_.clear();
            stack_.push_back(static_cast<int>(s));
            mark_[s] = 2;
            int minx = ww_, maxx = -1, miny = wh_, maxy = -1;
            double sumx = 0.0, sumy = 0.0;
            while (!stack_.empty()) {
                const int i = stack_.back();
                stack_.pop_back();
                members_.push_back(i);
                const int x = i % ww_, y = i / ww_;
                minx = std::min(minx, x); maxx = std::max(maxx, x);
                miny = std::min(miny, y); maxy = std::max(maxy, y);
                sumx += x; sumy += y;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= ww_ || ny >= wh_) continue;
                        const std::size_t j = idx(nx, ny);
                        if (mark_[j] == 1) { mark_[j] = 2; stack_.push_back(static_cast<int>(j)); }
                    }
            }
            if (std::hypot(maxx - minx + 1, maxy - miny + 1) * cell_ < cfg_.frontierMinSize) continue;
            ++clusters_;

            // Representative: the member nearest to the middle of the frontier.
            const double cx = sumx / static_cast<double>(members_.size()), cy = sumy / static_cast<double>(members_.size());
            int rep = members_.front();
            double repD = std::numeric_limits<double>::infinity();
            for (const int i : members_) {
                const double d = std::hypot(i % ww_ - cx, i / ww_ - cy);
                if (d < repD) { repD = d; rep = i; }
            }
            const float c = cost_[static_cast<std::size_t>(rep)];
            if (c < bestCost) { bestCost = c; bestRep = rep; }
            if (goal_ == GoalKind::Frontier && haveTarget_ && c < stickyCost
                && dist(world(g, rep % ww_, rep / ww_), target_) <= cfg_.stickyDist) {
                stickyCost = c;
                stickyRep = rep;
            }
        }
        if (bestRep < 0) return false;

        // Keep going for the frontier already chosen unless another one is
        // clearly cheaper (no flip-flopping between two similar goals).
        int rep = bestRep;
        if (stickyRep >= 0 && stickyCost <= 1.5f * bestCost + 1.0f) rep = stickyRep;
        tracePath(g, rep);
        target_ = path_.back();
        haveTarget_ = true;
        return true;
    }

    bool pathToHome(const ogm::OccupancyGrid& g) {
        const int hx = g.toCell(home_.x) - wx0_, hy = g.toCell(home_.y) - wy0_;
        int rep = -1;
        if (hx >= 0 && hy >= 0 && hx < ww_ && hy < wh_ && cost_[idx(hx, hy)] != Inf) {
            rep = static_cast<int>(idx(hx, hy));
        } else {
            // Home cell itself not reachable on the map: nearest reachable cell.
            double bestD = std::numeric_limits<double>::infinity();
            for (int y = 0; y < wh_; ++y)
                for (int x = 0; x < ww_; ++x) {
                    if (cost_[idx(x, y)] == Inf) continue;
                    const double d = dist(world(g, x, y), home_);
                    if (d < bestD) { bestD = d; rep = static_cast<int>(idx(x, y)); }
                }
        }
        if (rep < 0) return false;
        tracePath(g, rep);
        target_ = path_.back();
        haveTarget_ = true;
        return true;
    }

    void tracePath(const ogm::OccupancyGrid& g, int goalIndex) {
        path_.clear();
        for (int i = goalIndex; i >= 0; i = parent_[static_cast<std::size_t>(i)])
            path_.push_back(world(g, i % ww_, i / ww_));
        std::reverse(path_.begin(), path_.end());
    }

    struct KernelCell { int dx, dy; float d; };

    Config cfg_;
    double lethal_;
    State state_ = State::Init;
    GoalKind goal_ = GoalKind::None;
    FaultReason fault_ = FaultReason::None;
    double time_ = 0.0;

    Point home_;
    double homeHeading_ = 0.0;

    // scan
    double scanTurned_ = 0.0, scanLastHeading_ = 0.0;
    Point lastScanPos_;
    bool haveScan_ = false;
    int scans_ = 0;

    // drive
    std::vector<Point> path_;
    std::size_t progress_ = 0;
    Point target_;
    bool haveTarget_ = false;
    double replanTimer_ = 0.0, blockedTimer_ = 0.0, stuckTimer_ = 0.0;
    Point progressPos_;
    int stuck_ = 0;

    // back off
    Point backStart_;
    double backTimer_ = 0.0;

    // planning field (window over the known part of the grid)
    int wx0_ = 0, wy0_ = 0, ww_ = 0, wh_ = 0;
    double cell_ = 0.05, kernelCell_ = 0.0;
    Point robot_;
    std::vector<KernelCell> kernel_;
    std::vector<ogm::Cell> cells_;
    std::vector<float> clear_, cost_;
    std::vector<int> parent_, stack_, members_;
    std::vector<std::uint8_t> mark_;
    std::vector<Point> frontier_, blacklist_;
    int clusters_ = 0, plans_ = 0;
};

} // namespace explore
