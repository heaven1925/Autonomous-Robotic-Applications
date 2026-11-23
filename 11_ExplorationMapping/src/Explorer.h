// Explorer.h - frontier based exploration on the Bayesian grid (ders notu):
//
//   1. Initialize grid       every cell P(Occupied) = 0.5
//   2. Update the grid       robot cell -> empty; every sonar reading -> Bayes
//                            update of the cells along the beam (BayesGrid.h)
//   3. Pick a next move      frontier = undecided cell with an empty neighbour;
//                            pick the one whose empty neighbour is closest by
//                            A* path (only through cells known to be empty),
//                            drive there (PathFollower.h), face the frontier
//                            and measure it (turn around if still undecided)
//   4. Loop                  until every cell is decided (< 0.3 or > 0.8) -
//                            or no frontier is left (cells that can never be
//                            seen, e.g. behind a wall, stay undecided)
//
// Sensor readings are integrated only when the robot has moved / turned
// enough (or 1 s passed) since the previous reading of the same sensor, so
// one stationary view is not counted again and again as new evidence.
#pragma once

#include "AStarPlanner.h"
#include "BayesGrid.h"
#include "ExploreRobot.h"
#include "NavCommon.h"
#include "PathFollower.h"

#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace explore {

using astar::Cell;
using astar::Point;

struct Config {
    double scanSpeed = 0.6;          // rad/s, turning on the spot to look around
    double integrateTurnDeg = 6.0;   // integrate a sensor again after this much turning...
    double integrateMoveM = 0.04;    // ...or this much driving...
    double integrateEveryS = 2.0;    // ...or this much time
    double frontStopM = 0.05;        // front reading (from the sensor) closer than this while driving: stop (a wall 1 cell ahead reads ~0.09)
    double backOffM = 0.10;
    double timeLimitS = 1500.0;
    int maxVisits = 3;               // a frontier still undecided after this many visits is given up
    int minDirect = 3;               // front readings taken while facing a target from its neighbour...
    int maxDirect = 12;              // ...more while they disagree with the cell's state (weak sensor model)
    bool verify = true;              // after the frontiers: re-measure occupied cells next to free space
    int maxBumps = 15;
    double planRadius = 0.15;        // robot radius for A* (cells are 0.40 m: no inflation)
};

enum class Phase { Scan, Plan, Drive, Face, Dwell, BackOff, Done };
enum class Finish { None, AllDecided, NoFrontier, TimeLimit, TooManyBumps };

inline const char* phaseName(Phase p) {
    switch (p) {
        case Phase::Scan: return "SCAN";
        case Phase::Plan: return "PLAN";
        case Phase::Drive: return "DRIVE";
        case Phase::Face: return "FACE";
        case Phase::Dwell: return "MEASURE";
        case Phase::BackOff: return "BACK_OFF";
        case Phase::Done: return "DONE";
    }
    return "?";
}

class Explorer {
public:
    Explorer(bayes::BayesGrid& grid, const bayes::SensorModel& model, Geometry geo, Config cfg = {})
        : grid_(grid), model_(model), geo_(geo), cfg_(cfg) {}

    // --- 2. update the grid ---------------------------------------------
    void observe(const Snapshot& s, double now) {
        const Cell here = grid_.arena().cellAt(s.pose.point());
        if (!(here == lastRobotCell_) && grid_.inside(here)) {
            grid_.markRobotCell(here);
            lastRobotCell_ = here;
        }
        if (!s.newData) return;
        for (int k = 0; k < 3; ++k) {
            if (!geo_.uses(k)) continue;   // single-sensor set-up: only the front sonar
            const Beam& b = s.beams[k];
            if (!b.valid) continue;
            const double a = s.pose.heading + geo_.angle(k);
            Last& l = last_[k];
            const bool moved = !l.set || std::abs(nav::normalizeAngle(a - l.angle)) >= cfg_.integrateTurnDeg * nav::Deg ||
                               std::hypot(s.pose.x - l.x, s.pose.y - l.y) >= cfg_.integrateMoveM ||
                               now - l.time >= cfg_.integrateEveryS;
            const Point sensor{ s.pose.x + geo_.offset * std::cos(a), s.pose.y + geo_.offset * std::sin(a) };
            shown_[k] = { sensor, a, b.range, b.echo, true };
            if (!moved) continue;
            grid_.integrate(sensor, a, b.range, b.echo, model_);
            l = { true, a, s.pose.x, s.pose.y, now };
            ++readings_;
        }
    }

    // --- 3. pick a next move / drive ------------------------------------
    nav::Command step(const Snapshot& s, double dt, double now, Robot& robot) {
        nav::Command cmd;
        if (phase_ == Phase::Done) return cmd;
        if (now > cfg_.timeLimitS) return finish(Finish::TimeLimit);

        const bool bumped = s.bumper != Bumper::None && lastBumper_ == Bumper::None;
        lastBumper_ = s.bumper;
        if (bumped && phase_ != Phase::BackOff) {
            onBump(s, robot);
            if (phase_ == Phase::Done) return cmd;
        }

        switch (phase_) {
            case Phase::Scan: {
                turned_ += std::abs(nav::normalizeAngle(s.pose.heading - lastHeading_));
                lastHeading_ = s.pose.heading;
                if (turned_ >= 2.0 * nav::Pi) { phase_ = Phase::Plan; ++scans_; break; }
                cmd.w = cfg_.scanSpeed;
                break;
            }
            case Phase::Plan:
                plan(s);
                break;
            case Phase::Drive: {
                // The way must still be known-empty (a new reading may have shown an obstacle).
                for (std::size_t i = pathProgress(s); i < path_.size(); ++i)
                    if (!grid_.empty(path_[i])) { event_ = "path blocked by a new reading - planning again"; phase_ = Phase::Plan; return cmd; }
                if (s.beams[1].valid && s.beams[1].echo && s.beams[1].range < cfg_.frontStopM) {
                    event_ = "something right in front - backing off";
                    startBackOff(s);
                    return cmd;
                }
                cmd = follower_.step(s.pose);
                if (follower_.done()) phase_ = Phase::Face;
                break;
            }
            case Phase::Face: {
                const Point t = grid_.arena().centre(target_);
                const double err = nav::normalizeAngle(std::atan2(t.y - s.pose.y, t.x - s.pose.x) - s.pose.heading);
                if (std::abs(err) < 4.0 * nav::Deg) { phase_ = Phase::Dwell; dwell_ = 0.0; direct_ = 0; agree_ = 0; break; }
                const double w = std::clamp(1.8 * std::abs(err), 0.15, 0.8);
                cmd.w = err > 0.0 ? w : -w;
                break;
            }
            case Phase::Dwell:
                // Facing the target from the neighbouring cell = exactly the
                // calibration set-up (robot in a cell, measured cell 1 ahead):
                // each front reading is a direct Method-A event for the target.
                // At least minDirect readings; stop once the cell is decided and
                // the last 3 readings agree with that decision, at most maxDirect.
                dwell_ += dt;
                if (s.newData && s.beams[1].valid && direct_ < cfg_.maxDirect) {
                    const bool hit = measureDirect(s);
                    ++direct_;
                    const auto st = grid_.state(target_);
                    const bool agrees = (st == bayes::State::Occupied && hit) || (st == bayes::State::Empty && !hit);
                    agree_ = agrees ? agree_ + 1 : 0;
                }
                if ((direct_ >= cfg_.minDirect && agree_ >= 3) || direct_ >= cfg_.maxDirect || dwell_ >= 4.0) {
                    if (!contains(verified_, target_)) verified_.push_back(target_);
                    if (verifying_ || grid_.state(target_) != bayes::State::Undecided) phase_ = Phase::Plan;
                    else startScan(s);   // frontier still undecided: look around from here
                }
                break;
            case Phase::BackOff:
                backTime_ += dt;
                if (std::hypot(s.pose.x - backStart_.x, s.pose.y - backStart_.y) < cfg_.backOffM && backTime_ < 2.5)
                    cmd.v = -0.08;
                else
                    phase_ = Phase::Plan;
                break;
            case Phase::Done:
                break;
        }
        return cmd;
    }

    void begin(const Snapshot& s) { startScan(s); }

    // --- status ----------------------------------------------------------
    Phase phase() const { return phase_; }
    Finish finished() const { return finish_; }
    bool hasTarget() const { return hasTarget_; }
    bool verifying() const { return verifying_; }
    int verifiedCount() const { return static_cast<int>(verified_.size()); }
    Cell target() const { return target_; }
    Cell goal() const { return goal_; }
    const std::vector<Cell>& path() const { return path_; }
    const std::vector<Cell>& frontier() const { return frontier_; }
    const std::vector<Cell>& givenUp() const { return givenUp_; }
    int scans() const { return scans_; }
    int plans() const { return plans_; }
    int bumps() const { return bumps_; }
    int corrections() const { return corrections_; }
    long readings() const { return readings_; }
    std::string takeEvent() { std::string e = event_; event_.clear(); return e; }

    struct Shown {
        Point sensor;
        double angle = 0.0, range = 0.0;
        bool echo = false, set = false;
    };
    const Shown& beam(int k) const { return shown_[k]; }

private:
    struct Last {
        bool set = false;
        double angle = 0.0, x = 0.0, y = 0.0, time = 0.0;
    };

    nav::Command finish(Finish f) {
        finish_ = f;
        phase_ = Phase::Done;
        hasTarget_ = false;
        path_.clear();
        return {};
    }

    void startScan(const Snapshot& s) {
        phase_ = Phase::Scan;
        turned_ = 0.0;
        lastHeading_ = s.pose.heading;
    }

    void startBackOff(const Snapshot& s) {
        phase_ = Phase::BackOff;
        backStart_ = s.pose;
        backTime_ = 0.0;
    }

    // One front reading while facing target_ from a neighbouring cell:
    // event "the reading lands in the target cell" -> P(hit|O) from the Gaussian
    // at the cell's distance, measured P(hit|E); otherwise the complements.
    bool measureDirect(const Snapshot& s) {
        const astar::GridMap& m = grid_.arena();
        const Point t = m.centre(target_);
        const double centreDist = std::hypot(t.x - s.pose.x, t.y - s.pose.y);
        const double nearFace = centreDist - 0.5 * m.cellSize() - geo_.offset;   // from the sensor
        const Beam& b = s.beams[1];
        const double L = m.cellSize();
        const bool hit = b.echo && b.range >= model_.windowStart(nearFace) && b.range <= model_.windowEnd(nearFace, L);
        const double pO = model_.pHitOccupied(nearFace, L), pE = model_.pHitEmpty;   // Gaussian (B) / experiment (A)
        if (hit) grid_.update(target_, pO, pE);
        else grid_.update(target_, 1.0 - pO, 1.0 - pE);
        ++readings_;
        return hit;
    }

    void plan(const Snapshot& s) {
        ++plans_;
        frontier_ = grid_.frontiers();
        if (grid_.allDecided() && !cfg_.verify) { finish(Finish::AllDecided); return; }

        const Cell here = grid_.arena().cellAt(s.pose.point());
        astar::GridMap planMap = grid_.planningMap();
        planMap.setOccupied(here, false);   // the robot stands here: always a valid start, whatever noise says
        const astar::CostMap cm(planMap, cfg_.planRadius, grid_.arena().walledBorder);

        // Targets: the frontier cells. When no frontier can be reached, the
        // occupied cells next to free space that were never measured face-on
        // (verification - a wrongly "occupied" cell may hide more free space).
        std::vector<Cell> frontierTargets, verifyTargets;
        for (const Cell& f : frontier_)
            if (!isGivenUp(f)) frontierTargets.push_back(f);
        if (cfg_.verify) {
            for (int y = 0; y < grid_.rows(); ++y)
                for (int x = 0; x < grid_.cols(); ++x) {
                    const Cell c{ x, y };
                    if (!grid_.occupied(c) || contains(verified_, c) || isGivenUp(c)) continue;
                    for (const Cell& n : grid_.neighbours4(c))
                        if (grid_.empty(n)) { verifyTargets.push_back(c); break; }
                }
        }

        // Closest target: the one whose empty neighbour has the shortest A* path.
        bool found = false;
        double best = 1e18;
        Cell bestF{}, bestN{};
        std::vector<Cell> bestPath;
        const auto choose = [&](const std::vector<Cell>& targets) {
            for (const Cell& f : targets) {
                for (const Cell& n : grid_.neighbours4(f)) {
                    if (!grid_.empty(n) && !(n == here)) continue;
                    double cost = 0.0;
                    std::vector<Cell> cells;
                    if (n == here) {
                        cells = { here };
                    } else {
                        const auto r = astar::search(cm, here, n, astar::Connectivity::Eight);
                        if (!r.found) continue;
                        cost = r.cost;
                        cells = r.path;
                    }
                    const double tie = 0.001 * std::hypot(f.x - here.x, f.y - here.y);
                    if (cost + tie < best) {
                        best = cost + tie;
                        bestF = f;
                        bestN = n;
                        bestPath = cells;
                        found = true;
                    }
                }
            }
        };
        verifying_ = false;
        choose(frontierTargets);
        if (!found && !verifyTargets.empty()) {
            choose(verifyTargets);
            verifying_ = found;
        }
        if (!found) {
            finish(grid_.allDecided() ? Finish::AllDecided : Finish::NoFrontier);
            return;
        }

        if (++visits_[key(bestF)] > cfg_.maxVisits) {
            givenUp_.push_back(bestF);
            event_ = "cell " + grid_.arena().coords.text(bestF) + " stays undecided after " +
                     std::to_string(cfg_.maxVisits) + " visits - giving it up";
            return;   // plan again next step
        }
        target_ = bestF;
        goal_ = bestN;
        hasTarget_ = true;
        path_ = bestPath;
        if (verifying_)
            event_ = "verify occupied " + grid_.arena().coords.text(bestF) + " face-on from " + grid_.arena().coords.text(bestN);
        else
            event_ = "frontier " + grid_.arena().coords.text(bestF) + " -> going to " + grid_.arena().coords.text(bestN) +
                     " (" + std::to_string(frontier_.size()) + " frontier cells)";

        if (bestPath.size() <= 1) {
            // Already in the right cell: measure from its centre (the calibration set-up).
            const Point c = grid_.arena().centre(here);
            if (std::hypot(c.x - s.pose.x, c.y - s.pose.y) < 0.03) { phase_ = Phase::Face; return; }
            follower_.setPath({ s.pose.point(), c });
            phase_ = Phase::Drive;
            return;
        }
        std::vector<Point> pts = astar::toPoints(grid_.arena(), astar::corners(bestPath));
        pts.front() = s.pose.point();   // start where the robot really is
        follower_.setPath(pts);
        phase_ = Phase::Drive;
    }

    // Index of the first path cell not yet reached (the robot's own cell or later).
    std::size_t pathProgress(const Snapshot& s) const {
        const Cell here = grid_.arena().cellAt(s.pose.point());
        for (std::size_t i = 0; i < path_.size(); ++i)
            if (path_[i] == here) return i + 1;
        return 0;
    }

    void onBump(const Snapshot& s, Robot& robot) {
        ++bumps_;
        if (bumps_ > cfg_.maxBumps) { finish(Finish::TooManyBumps); return; }
        Point u;
        double dist = 0.0;
        if (knownContact(s, u, dist)) {
            const double shift = dist - geo_.radius;
            robot.shiftPose(u.x * shift, u.y * shift);
            ++corrections_;
            event_ = "bumper: known obstacle, position corrected by " + std::to_string(static_cast<int>(std::lround(shift * 100.0))) + " cm";
        } else {
            const double rel = s.bumper == Bumper::Left ? 45.0 * nav::Deg : (s.bumper == Bumper::Right ? -45.0 * nav::Deg : 0.0);
            const double a = s.pose.heading + rel;
            const Cell own = grid_.arena().cellAt(s.pose.point());
            for (double d = geo_.radius + 0.02; d < geo_.radius + 0.5; d += 0.02) {
                const Cell c = grid_.arena().cellAt({ s.pose.x + d * std::cos(a), s.pose.y + d * std::sin(a) });
                if (c == own) continue;
                if (grid_.inside(c)) {
                    grid_.markBump(c);
                    event_ = "bumper: cell " + grid_.arena().coords.text(c) + " is occupied";
                }
                break;
            }
        }
        startBackOff(s);
    }

    // Bumper against a cell already known to be occupied (or the border wall):
    // the body is touching it, so the odometry has drifted.
    bool knownContact(const Snapshot& s, Point& u, double& dist) const {
        const double rel = s.bumper == Bumper::Left ? 45.0 * nav::Deg : (s.bumper == Bumper::Right ? -45.0 * nav::Deg : 0.0);
        const double a = s.pose.heading + rel;
        const astar::GridMap& m = grid_.arena();
        const Point p = s.pose.point();
        const double reach = geo_.radius + 0.12;
        dist = reach;
        bool found = false;
        const auto consider = [&](Point q) {
            const double d = astar::distance(p, q);
            if (d < 1e-6 || d >= dist) return;
            if (std::abs(nav::normalizeAngle(std::atan2(q.y - p.y, q.x - p.x) - a)) > 70.0 * nav::Deg) return;
            dist = d;
            u = { (q.x - p.x) / d, (q.y - p.y) / d };
            found = true;
        };
        const Cell at = m.cellAt(p);
        for (int y = at.y - 2; y <= at.y + 2; ++y)
            for (int x = at.x - 2; x <= at.x + 2; ++x) {
                if (!grid_.occupied({ x, y })) continue;
                double x0, y0, x1, y1;
                m.bounds({ x, y }, x0, y0, x1, y1);
                consider({ std::clamp(p.x, x0, x1), std::clamp(p.y, y0, y1) });
            }
        if (m.walledBorder) {
            consider({ 0.0, p.y });
            consider({ m.width(), p.y });
            consider({ p.x, 0.0 });
            consider({ p.x, m.height() });
        }
        return found;
    }

    static bool contains(const std::vector<Cell>& v, Cell c) {
        for (const Cell& k : v)
            if (k == c) return true;
        return false;
    }
    bool isGivenUp(Cell c) const {
        for (const Cell& g : givenUp_)
            if (g == c) return true;
        return false;
    }
    int key(Cell c) const { return c.y * 1000 + c.x; }

    bayes::BayesGrid& grid_;
    const bayes::SensorModel& model_;
    Geometry geo_;
    Config cfg_;
    nav::PathFollower follower_;

    Phase phase_ = Phase::Scan;
    Finish finish_ = Finish::None;
    Cell lastRobotCell_{ -1, -1 };
    Last last_[3];
    Shown shown_[3];
    double turned_ = 0.0, lastHeading_ = 0.0, dwell_ = 0.0, backTime_ = 0.0;
    nav::Pose backStart_;
    Bumper lastBumper_ = Bumper::None;
    bool hasTarget_ = false;
    Cell target_{}, goal_{};
    std::vector<Cell> path_, frontier_, givenUp_;
    std::map<int, int> visits_;
    int scans_ = 0, plans_ = 0, bumps_ = 0, corrections_ = 0, direct_ = 0, agree_ = 0;
    bool verifying_ = false;
    std::vector<Cell> verified_;
    long readings_ = 0;
    std::string event_;
};

} // namespace explore
