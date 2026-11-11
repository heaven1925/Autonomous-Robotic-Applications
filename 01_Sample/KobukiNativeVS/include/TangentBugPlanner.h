// TangentBugPlanner.h - hardware-independent (approximate) Tangent Bug core.
//
// Like Bug0Planner.h this header has NO dependency on Kobuki, serial ports or
// Arduino: plain doubles in (pose, goal, three range readings in metres),
// a (v, w) velocity command plus the current state out. The real hardware
// runner is 08_TangentBugAlgorithm/src/main.cpp.
//
// Sensing model: three fixed HC-SR04 beams at +beamAngle (left-front), 0
// (front) and -beamAngle (right-front), each with a finite sensing range R.
// Classic Tangent Bug assumes a continuous 360 deg range scan; with only three
// rays this is an APPROXIMATION (see 08_TangentBugAlgorithm/TangentBug.md):
//   * "Goal ray free"  - a beam whose cone contains the goal bearing reads at
//                        least min(R, d(x, goal)).
//   * Discontinuity O_i - a range jump between the front beam and a side
//                        beam that a flat, continuous obstacle could not
//                        produce: the blocking obstacle ends somewhere
//                        between the two rays on that side.
//   * Heuristic         - h_i = d(x, O_i) + d(O_i, goal); the robot moves
//                        toward the endpoint with the smallest h_i as long as
//                        that minimum keeps decreasing.
//   * Boundary follow   - started when h stops decreasing (local minimum).
//                        d_followed = min goal distance of the sensed points
//                        of the FOLLOWED boundary (echoes near the robot);
//                        d_reach = min goal distance of the free space
//                        currently visible along the beams. Leave toward the
//                        d_reach point as soon as d_reach < d_followed.
//   * No scanning       - the robot never stops to sweep the sensors around;
//                        every decision uses what the beams see while it
//                        drives. Heading changes are smooth arcs (arcTo), and
//                        the only in-place turns (escaping a close obstacle,
//                        Recovery, a U-turn out of a dead end) end the moment
//                        a free direction is seen.
//   * Unreachable goal  - a full lap back to the reference point (same
//                        direction, ~2 Pi of turning), consistent with the
//                        goal's winding number (inside the loop when circling
//                        an obstacle, outside when circling an enclosure),
//                        confirmed by a second lap in the opposite direction
//                        -> Fault(GoalUnreachable).
//   * Point memory      - every echo is also stored (world frame, odometry)
//                        for memoryTimeS. A single ray cannot tell whether
//                        the robot BODY fits past an obstacle corner it just
//                        grazes, so straight-line moves (to the goal or to an
//                        endpoint) additionally require the body corridor to
//                        be clear of remembered points.
//
// All hysteresis is TIME based (seconds), not tick based, so the behaviour
// is the same whether step() runs at 100 Hz (hardware loop) or 20 Hz (a
// simulator), and independent of the ~10 Hz Arduino update rate.
//
// Invalid sensor fields are NEVER treated as "clear path".
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace tbug {

constexpr double Pi = 3.14159265358979323846;
constexpr double Inf = std::numeric_limits<double>::infinity();

inline double normalizeAngle(double a) {
    while (a > Pi) a -= 2.0 * Pi;
    while (a <= -Pi) a += 2.0 * Pi;
    return a;
}

enum class State { Init, MotionToGoal, MoveToEndpoint, FollowBoundary, Recovery, GoalReached, Fault };
enum class Side { None, Left, Right };          // side of the robot the followed wall is on
enum class FaultReason { None, GoalUnreachable, NoSafeMotion, GoalBlocked, Timeout };

struct Pose { double x = 0.0, y = 0.0, heading = 0.0; };
struct Goal { double x = 0.0, y = 0.0; };
struct Point { double x = 0.0, y = 0.0; };

// Ranges in metres measured from the robot CENTRE along each beam (add the
// sensor mounting offset to raw sensor readings). "Nothing seen" = a value
// >= sensingRange; invalid fields are never treated as free space.
struct SensorReading {
    double left = 0.0, front = 0.0, right = 0.0;
    bool leftValid = false, frontValid = false, rightValid = false;
};

struct Command { double v = 0.0, w = 0.0; };

struct Config {
    // --- sensor geometry -------------------------------------------------
    double beamAngle      = 60.0 * Pi / 180.0; // rad - side beam angle from straight ahead (as mounted)
    double beamHalfWidth  = 12.0 * Pi / 180.0; // rad - usable half-cone for "goal ray free" checks
    double sensingRange   = 1.2;   // m - readings >= this count as "nothing seen" (R in the literature). Kept
                                   //     short on purpose: a HC-SR04 cone (~+-15 deg) is ~1 m wide at 2 m and
                                   //     would report a maze gap as a wall from afar; at ~1 m it sees through
    double jumpThreshold  = 0.30;  // m - extra side range (vs. a flat wall) that counts as a discontinuity

    // --- robot / safety distances (front beam distances unless noted) -----
    double robotRadius    = 0.18;  // m
    double d_stop         = 0.40;  // m - front distance that forbids driving forward (sensor ~29 cm from it)
    double d_slow         = 0.65;  // m - front distance where slowing down starts
    double d_stopFollow   = 0.32;  // m - front stop distance while boundary-following (slow; lets the robot
                                   //     arc around a door jamb without the opposite jamb turning it back)
    double d_follow       = 0.40;  // m - desired LATERAL distance to the followed wall (centre to wall); 0.4 m
                                   //     = the centre line of a 0.8 m maze corridor
    double wallSeenRange  = 0.80;  // m - side beam reading below this = "wall is seen"
    double sideEmergencyMargin = 0.14; // m - side beam < robotRadius + this -> stop & turn away
    double hardSideMargin = 0.08;  // m - side beam < robotRadius + this = really too close (in-place turn / Recovery);
                                   //     between this and sideEmergencyMargin a narrow passage is crept through, centred

    // --- speeds / gains --------------------------------------------------
    double driveSpeed     = 0.20;  // m/s
    double followSpeed    = 0.12;  // m/s
    double minDrive       = 0.04;  // m/s
    double turnSpeed      = 0.6;   // rad/s - HC-SR04 packets arrive at 10 Hz with ~0.1 s latency; 0.25 m/s +
                                   //         0.7 rad/s caused a collision in simulation, these values none
    double minTurn        = 0.08;  // rad/s
    double headingGain    = 2.0;
    double wallGain       = 1.5;
    double headingTol     = 10.0 * Pi / 180.0; // rad - aligned enough to drive straight
    double goalTolerance  = 0.08;  // m

    // --- Tangent Bug specific --------------------------------------------
    double endpointMaxStep = 0.80; // m - max distance driven toward one endpoint before re-evaluating
    double endpointMinStep = 0.15; // m - shorter than this -> no room, follow the boundary instead
    double endpointTol     = 0.10; // m - endpoint counts as reached
    double heuristicEps    = 0.03; // m - h must improve by more than this to keep moving to endpoints
    double sideSwitchMargin = 0.30;// m - the other side must be this much better to switch sides
    double leaveEps        = 0.05; // m - leave boundary when d_reach < d_followed - leaveEps
    double loopTol         = 0.30; // m - reference "gate" is set up once the robot is this far past the hit point
    double loopGate        = 0.60; // m - half-width of the gate line: lap closed when the robot crosses it again
    double loopMinTravel   = 1.50; // m - ...but only after following at least this far
    double goalBlockedRadius = 0.80; // m - "near the goal" for the blocked-goal check below
    double goalBlockedClear  = 0.05; // m - an echo closer than robotRadius + this to the goal = goal hugs an obstacle
    double goalBlockedTimeS  = 30.0; // s - total time near such a goal without reaching it -> Fault(GoalBlocked)
    double maxGoalTimeS      = 0.0;  // s - give up on one goal after this long -> Fault(Timeout); 0 = no limit

    // --- smooth (arc) motion ---------------------------------------------
    // Heading changes are driven as arcs: forward speed = vmax * cos(heading
    // error), so small corrections are smooth curves and only a target more
    // than 90 deg off turns in place. No scanning anywhere: every in-place
    // turn ends as soon as a usable direction is seen.
    double followReact     = 0.35; // m - while following, start curving away from an obstacle ahead this much
                                   //     before the front stop distance (rounds inner corners in an arc)
    double recoveryClear   = 0.60; // m - Recovery stops turning as soon as the front is free this far
    double recoveryMaxTurn = 2.0 * Pi; // rad - Recovery turned this much without finding a free direction -> attempt failed

    // --- passages (gaps) ---------------------------------------------------
    // A gap the beams see through is entered along a straight, body-wide
    // corridor found in the obstacle memory (pure computation - the robot
    // does not move to look for it).
    double passageSpan     = 70.0 * Pi / 180.0; // rad - search this far either side of the preferred direction
    double passageBeyond   = 0.80; // m - corridor length past the blocking point (gets the body through the gap)
    double preciseTol      = 15.0 * Pi / 180.0; // rad - through a gap: turn to within this before driving
    double passageWindow   = 20.0 * Pi / 180.0; // rad - corridors are only searched this close to a beam that
                                   //       currently sees far (never into directions nobody looked at)
    double carveHalf       = 6.0 * Pi / 180.0;  // rad - an ultrasonic cone reports its NEAREST echo, so
    double carveMargin     = 0.10; // m   everything nearer inside the cone is free: remembered points
    double carveMinDist    = 0.25; // m   there are erased (removes "phantom" points the cone smeared into
                                   //     gaps) - but never points this close to the robot (thin corners
                                   //     can be missed by a single packet; near points are safety-critical)
    double arcCheckLen     = 0.40; // m - every forward command is checked along its arc this far ahead
    double arcCheckMargin  = 0.02; // m   against remembered points (body radius + this): protects the
                                   //     blind sectors between the beams

    // --- obstacle point memory ---------------------------------------------
    double bodyMargin      = 0.05; // m - corridor half-width = robotRadius + this (fits ~0.5 m gaps)
    double memoryTimeS     = 60.0; // s - remembered echoes expire after this
    double memoryMinSpacing = 0.04;// m - a new echo closer than this to a stored one is not stored again
    double memoryMaxRange  = 1.00; // m - only echoes closer than this are stored (far ones are smeared by the cone)
    bool   followMemory    = false; // always follow the nearest remembered boundary point instead of the side
                                    //   beam. Off: with real (conical) beams remembered points are smeared,
                                    //   which made following worse in cluttered simulations. Remembered
                                    //   points are still used to arc around outer corners (wall not seen).
    double followMemoryRange = 0.80; // m - max distance of that remembered point

    // --- timing (seconds) --------------------------------------------------
    double confirmTimeS    = 0.25; // condition must hold this long before a state change
    double wallLostTurn    = 1.75 * Pi; // rad - turned this much without seeing the wall -> Recovery
                                   //       (rounding the tip of a thin wall needs ~Pi)
    double wallLostTimeS   = 60.0; // s - backstop for the same condition
    double stallTimeS      = 20.0; // s - did not get further than stallDist from one spot in this time
    double stallDist       = 0.30; // m   while following -> too tight (or jittering): U-turn and carry on
    double maxFollowTimeS  = 900.0;// safety cap on one boundary-following episode (must exceed one lap)
    int    maxRecoveryAttempts = 4; // recoveries without moving away from the spot -> Fault
    double progressReset   = 1.00; // m - moving this far from the last recovery spot resets the count
};

class TangentBugPlanner {
public:
    explicit TangentBugPlanner(Config cfg = {}) : cfg_(cfg) {}

    State state() const { return state_; }
    Side followSide() const { return side_; }
    FaultReason faultReason() const { return faultReason_; }
    double dFollowed() const { return dFollowed_; }
    double dReach() const { return dReach_; }
    double bestHeuristic() const { return hMin_; }
    Point endpoint() const { return endpoint_; }

    // Boundary-following loop bookkeeping, for logging / diagnostics.
    struct LoopInfo { bool haveRef; Point ref; double path, totalTurn, goalWind; bool confirmLap; };
    LoopInfo loopInfo() const { return { haveRef_, hitPoint_, followPath_, totalTurn_, goalWind_, confirmLap_ }; }
    double distanceToGoal(const Pose& p, const Goal& g) const { return std::hypot(g.x - p.x, g.y - p.y); }

    // Advance the planner by dt seconds; returns the velocity command the
    // motion layer should apply for this tick.
    Command step(const SensorReading& s, const Pose& pose, const Goal& goal, double dt) {
        time_ += dt;
        lastStepPos_ = pos(pose);
        remember(s, pose);

        const double distToGoal = distanceToGoal(pose, goal);
        // Recoveries only count toward Fault while the robot stays stuck in
        // one place; Tangent Bug paths may legitimately lead away from the
        // goal for a long time (around a deep cup), so goal distance is not
        // a progress measure here.
        if (recoveryAttempts_ > 0 && dist(pos(pose), recoverySpot_) > cfg_.progressReset) recoveryAttempts_ = 0;
        if (state_ != State::GoalReached && state_ != State::Fault && distToGoal < cfg_.goalTolerance) {
            state_ = State::GoalReached;
            return {};
        }

        if (cfg_.maxGoalTimeS > 0.0 && time_ > cfg_.maxGoalTimeS && state_ != State::GoalReached && state_ != State::Fault) {
            state_ = State::Fault;
            faultReason_ = FaultReason::Timeout;
            return {};
        }

        // A goal (almost) touching an obstacle cannot be reached with the
        // safety margins; without this check the robot would keep circling
        // around it. Time near the goal is accumulated, so an approach that
        // just needs a few seconds (e.g. straight at a goal 0.3 m in front of
        // a wall) is not affected.
        if (state_ != State::GoalReached && state_ != State::Fault && distToGoal < cfg_.goalBlockedRadius
            && goalHugsObstacle(s, pose, goal, distToGoal)) {
            nearBlockedGoalTime_ += dt;
            if (nearBlockedGoalTime_ > cfg_.goalBlockedTimeS) {
                state_ = State::Fault;
                faultReason_ = FaultReason::GoalBlocked;
                return {};
            }
        }

        Command cmd;
        switch (state_) {
            case State::Init:
                enterMotionToGoal(true);
                [[fallthrough]];
            case State::MotionToGoal:   cmd = doMotionToGoal(s, pose, goal, distToGoal, dt); break;
            case State::MoveToEndpoint: cmd = doMoveToEndpoint(s, pose, goal, distToGoal, dt); break;
            case State::FollowBoundary: cmd = doFollowBoundary(s, pose, goal, dt); break;
            case State::Recovery:       cmd = doRecovery(s, pose, dt); break;
            case State::GoalReached:
            case State::Fault:
            default:                    return {};
        }
        return arcGuard(pose, cmd);
    }

private:
    // ----------------------------------------------------------- helpers --
    static double normalize(double a) {
        while (a > Pi) a -= 2.0 * Pi;
        while (a <= -Pi) a += 2.0 * Pi;
        return a;
    }
    static double dist(const Point& a, const Point& b) { return std::hypot(a.x - b.x, a.y - b.y); }
    static Point pos(const Pose& p) { return {p.x, p.y}; }
    static Point pt(const Goal& g) { return {g.x, g.y}; }

    // Beam index: 0 = left, 1 = front, 2 = right.
    double beamRel(int i) const { return i == 0 ? cfg_.beamAngle : (i == 1 ? 0.0 : -cfg_.beamAngle); }
    static double reading(const SensorReading& s, int i) { return i == 0 ? s.left : (i == 1 ? s.front : s.right); }
    static bool valid(const SensorReading& s, int i) { return i == 0 ? s.leftValid : (i == 1 ? s.frontValid : s.rightValid); }
    double range(const SensorReading& s, int i) const { return std::clamp(reading(s, i), 0.0, cfg_.sensingRange); }
    bool hit(const SensorReading& s, int i) const { return valid(s, i) && reading(s, i) < cfg_.sensingRange; }
    double sideLimit() const { return cfg_.robotRadius + cfg_.sideEmergencyMargin; }
    double corridor() const { return cfg_.robotRadius + cfg_.bodyMargin; }

    static Point along(const Pose& p, double relAngle, double r) {
        const double a = p.heading + relAngle;
        return { p.x + r * std::cos(a), p.y + r * std::sin(a) };
    }

    double clampTurn(double w) const {
        w = std::clamp(w, -cfg_.turnSpeed, cfg_.turnSpeed);
        if (std::abs(w) < cfg_.minTurn && std::abs(w) > 1e-6) w = (w < 0 ? -cfg_.minTurn : cfg_.minTurn);
        return w;
    }

    // Speed limited by how close the front beam sees something.
    double frontLimitedSpeed(const SensorReading& s, double vmax) const {
        if (!s.frontValid || s.front >= cfg_.d_slow) return vmax;
        const double t = (s.front - cfg_.d_stop) / (cfg_.d_slow - cfg_.d_stop);
        return std::max(cfg_.minDrive, vmax * std::clamp(t, 0.0, 1.0));
    }

    // Nothing close straight ahead (front echo or remembered point): creeping
    // forward is allowed.
    bool aheadFree(const SensorReading& s, const Pose& pose) const {
        return !(s.frontValid && s.front < cfg_.d_stop) && clearLength(pos(pose), pose.heading, 0.3) >= 0.3;
    }

    // Smooth steering toward an absolute heading: drives an arc instead of
    // "turn in place, then go straight". Forward speed shrinks with the
    // heading error (cos), so large corrections become tight arcs and only
    // a target more than 90 deg off is turned to in place. Never drives
    // forward into something close ahead (front echo or remembered point).
    Command arcTo(const SensorReading& s, const Pose& pose, double target, double vmax, bool precise = false) const {
        const double err = normalize(target - pose.heading);
        const double w = clampTurn(cfg_.headingGain * err);
        double v = vmax * std::max(0.0, std::cos(err));
        if (precise && std::abs(err) > cfg_.preciseTol) v = 0.0; // line up before entering a tight gap
        v = std::min(v, frontLimitedSpeed(s, vmax));
        v = std::min(v, clearLength(pos(pose), pose.heading, 0.5));
        if (s.frontValid && s.front < cfg_.d_stop) v = 0.0;
        if (v < cfg_.minDrive) v = (v > 0.5 * cfg_.minDrive) ? cfg_.minDrive : 0.0;
        return { v, w };
    }

    // Time-based hysteresis: returns true once `cond` held for confirmTimeS.
    bool confirm(double& timer, bool cond, double dt) const {
        timer = cond ? timer + dt : 0.0;
        return timer >= cfg_.confirmTimeS;
    }

    // ------------------------------------------------------ point memory --
    struct MemPoint { double x = 0.0, y = 0.0, t = -1e9; };
    static constexpr std::size_t MemSize = 1024;

    bool alive(const MemPoint& m) const { return time_ - m.t <= cfg_.memoryTimeS; }

    void remember(const SensorReading& s, const Pose& pose) {
        // Free-space carving first: a beam reading r means nothing within
        // its cone up to r - erase remembered points there.
        for (int i = 0; i < 3; ++i) {
            if (!valid(s, i)) continue;
            const double r = range(s, i) - cfg_.carveMargin;
            if (r <= 0.0) continue;
            const double a = pose.heading + beamRel(i);
            for (auto& m : mem_) {
                if (!alive(m)) continue;
                const double dx = m.x - pose.x, dy = m.y - pose.y;
                const double d = std::hypot(dx, dy);
                if (d >= r || d < cfg_.carveMinDist) continue;
                if (std::abs(normalize(std::atan2(dy, dx) - a)) <= cfg_.carveHalf) m.t = -1e9;
            }
        }
        for (int i = 0; i < 3; ++i) {
            if (!hit(s, i) || reading(s, i) > cfg_.memoryMaxRange) continue;
            const Point p = along(pose, beamRel(i), range(s, i));
            bool duplicate = false;
            for (auto& m : mem_) {
                if (alive(m) && std::hypot(m.x - p.x, m.y - p.y) < cfg_.memoryMinSpacing) {
                    m.t = time_; // refresh instead of storing again
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            mem_[memNext_] = { p.x, p.y, time_ };
            memNext_ = (memNext_ + 1) % MemSize;
        }
    }

    // How far the robot body (a disc of radius corridor()) can travel from
    // `from` along absolute heading `a` before touching a remembered point,
    // capped at maxLen. Points behind the start are ignored (rotation in
    // place is always allowed for the round Kobuki).
    double clearLength(const Point& from, double a, double maxLen) const {
        const double dx = std::cos(a), dy = std::sin(a);
        const double hw = corridor();
        double limit = maxLen;
        for (const auto& m : mem_) {
            if (!alive(m)) continue;
            const double rx = m.x - from.x, ry = m.y - from.y;
            const double t = rx * dx + ry * dy;
            if (t < 0.0 || t > maxLen + hw) continue;
            const double lat = std::abs(rx * dy - ry * dx);
            if (lat >= hw) continue;
            limit = std::min(limit, t - std::sqrt(hw * hw - lat * lat));
        }
        return std::max(0.0, limit);
    }

    // Measured directly with the front beam (remembered points are smeared
    // by the beam cone and can land right next to a free goal): the goal is
    // inside the front cone and the echo comes from (almost) at the goal.
    bool goalHugsObstacle(const SensorReading& s, const Pose& pose, const Goal& goal, double distToGoal) const {
        if (!s.frontValid) return false;
        const double bearing = normalize(std::atan2(goal.y - pose.y, goal.x - pose.x) - pose.heading);
        if (std::abs(bearing) > cfg_.beamHalfWidth) return false;
        return range(s, 1) < distToGoal + cfg_.robotRadius + cfg_.goalBlockedClear;
    }

    // Last safety layer for every state: follow the commanded arc ahead
    // (arcCheckLen) and make sure the body does not run into a remembered
    // point - this also covers obstacles in the blind sectors between the
    // beams that were seen earlier. Slows down, or turns in place if needed.
    Command arcGuard(const Pose& pose, Command cmd) const {
        if (cmd.v <= 0.0) return cmd;
        const double r = cfg_.robotRadius + cfg_.arcCheckMargin;
        auto hits = [&](double v, double w) {
            const int n = 8;
            const double T = cfg_.arcCheckLen / std::max(v, 1e-3);
            for (int k = 1; k <= n; ++k) {
                const double t = T * k / n;
                double x, y;
                if (std::abs(w) < 1e-6) { x = pose.x + v * t * std::cos(pose.heading); y = pose.y + v * t * std::sin(pose.heading); }
                else {
                    const double R = v / w, h1 = pose.heading + w * t;
                    x = pose.x + R * (std::sin(h1) - std::sin(pose.heading));
                    y = pose.y - R * (std::cos(h1) - std::cos(pose.heading));
                }
                for (const auto& m : mem_)
                    if (alive(m) && std::hypot(m.x - x, m.y - y) < r) return true;
            }
            return false;
        };
        if (!hits(cmd.v, cmd.w)) return cmd;
        // Points the body ALREADY overlaps (e.g. smeared ones beside a gap)
        // must not freeze the robot: only block if the arc gets closer.
        double now = Inf;
        for (const auto& m : mem_) if (alive(m)) now = std::min(now, std::hypot(m.x - pose.x, m.y - pose.y));
        if (now < r) return cmd;
        // Softer alternatives first (slower, gentler, straight), then turn in place.
        const Command alts[] = { { 0.5 * cmd.v, cmd.w }, { cmd.v, 0.5 * cmd.w }, { 0.5 * cmd.v, 0.0 } };
        for (const auto& a : alts)
            if (a.v >= cfg_.minDrive && !hits(a.v, a.w)) return a;
        cmd.v = 0.0;
        return cmd;
    }

    // Straight, body-wide corridor of length `len` free of remembered points,
    // in the direction closest to `prefer` (searched both ways up to `span`).
    bool clearHeading(const Point& from, double prefer, double len, double span, double& out) const {
        const double step = 2.5 * Pi / 180.0;
        for (int k = 0; k * step <= span + 1e-9; ++k) {
            for (int sgn = 1; sgn >= -1; sgn -= 2) {
                if (k == 0 && sgn < 0) continue;
                const double th = normalize(prefer + sgn * k * step);
                if (clearLength(from, th, len) >= len) { out = th; return true; }
            }
        }
        return false;
    }

    // A wider corridor than the body needs is also free -> no precision needed.
    bool roomy(const Point& from, double th, double len) const {
        const double hw = corridor() + 0.15;
        for (const auto& m : mem_) {
            if (!alive(m)) continue;
            const double rx = m.x - from.x, ry = m.y - from.y;
            const double t = rx * std::cos(th) + ry * std::sin(th);
            if (t < 0.0 || t > len) continue;
            if (std::abs(rx * std::sin(th) - ry * std::cos(th)) < hw) return false;
        }
        return true;
    }

    void startMove(const Pose& pose, double th, double len, double h) {
        endpoint_ = { pose.x + len * std::cos(th), pose.y + len * std::sin(th) };
        hMin_ = h;
        precise_ = !roomy(pos(pose), th, len);
        state_ = State::MoveToEndpoint;
        moveTimer_ = 0.0;
        moveTimeout_ = 3.0 * len / cfg_.driveSpeed + Pi / cfg_.turnSpeed + 3.0;
        frontCloseTimer_ = goalFreeTimer_ = 0.0;
    }

    // The straight goal ray clips a remembered corner, but the front beam
    // looks through further: find a body-wide straight corridor near the goal
    // direction that gets past the block and take it (e.g. through a gap
    // that is not exactly on the goal ray).
    bool tryPassage(const SensorReading& s, const Pose& pose, const Goal& goal, double blockDist) {
        if (!s.frontValid || range(s, 1) < blockDist + 0.5) return false;
        const double dGoal = distanceToGoal(pose, goal);
        const double len = std::min({ blockDist + cfg_.passageBeyond, cfg_.sensingRange, dGoal });
        if (len < cfg_.endpointMinStep) return false;
        const double goalHeading = std::atan2(goal.y - pose.y, goal.x - pose.x);
        if (len > range(s, 1) - 0.2) return false;
        // Search around the front beam - the direction actually seen to be
        // open - preferring the heading closest to the goal.
        double th;
        if (!clearHeading(pos(pose), pose.heading, len, cfg_.passageWindow, th)) return false;
        for (double k = 0.0; k <= cfg_.passageWindow + 1e-9; k += 2.5 * Pi / 180.0) {
            const double cand = normalize(pose.heading + std::clamp(normalize(goalHeading - pose.heading), -k, k));
            if (clearLength(pos(pose), cand, len) >= len) { th = cand; break; }
        }
        const Point p{ pose.x + len * std::cos(th), pose.y + len * std::sin(th) };
        const double h = len + dist(p, pt(goal));
        if (h >= hMin_ - cfg_.heuristicEps) return false;
        side_ = preferredSide_ = (normalize(th - goalHeading) > 0.0) ? Side::Right : Side::Left;
        startMove(pose, th, len, h);
        return true;
    }

    // Goal ray is visible through some beam cone, nothing blocks it within
    // min(R, distance to goal), and the body corridor is clear in memory.
    bool goalRayFree(const SensorReading& s, const Pose& pose, const Goal& goal, double distToGoal) const {
        const double goalHeading = std::atan2(goal.y - pose.y, goal.x - pose.x);
        const double bearing = normalize(goalHeading - pose.heading);
        const double need = std::min(cfg_.sensingRange, distToGoal);
        for (int i = 0; i < 3; ++i) {
            if (!valid(s, i)) continue;
            if (std::abs(normalize(bearing - beamRel(i))) > cfg_.beamHalfWidth) continue;
            if (range(s, i) >= need && clearLength(pos(pose), goalHeading, need) >= need) return true;
        }
        return false;
    }

    // Endpoint candidate on one side (beam 0 or 2), or h = Inf if no
    // discontinuity is visible there. `blockDist` is how far straight ahead
    // the path is blocked (front echo or remembered points). The endpoint
    // estimate is where the blocking surface would cross the side ray if it
    // continued flat - the real edge lies between that point and the front
    // hit, so this is the conservative (outer) choice.
    struct Candidate { double h = Inf; Point p; double stepLen = 0.0; };
    Candidate endpointCandidate(const SensorReading& s, const Pose& pose, const Goal& goal, int i, double blockDist) const {
        Candidate c;
        if (!valid(s, i)) return c;
        const double flatWall = blockDist / std::cos(cfg_.beamAngle);
        const double r = range(s, i);
        const bool discontinuity = r >= cfg_.sensingRange || r > flatWall + cfg_.jumpThreshold;
        if (!discontinuity) return c;
        c.p = along(pose, beamRel(i), std::min(flatWall, r));
        c.h = dist(pos(pose), c.p) + dist(c.p, pt(goal));
        const double freeLen = r - sideLimit();
        c.stepLen = std::min({ cfg_.endpointMaxStep, freeLen,
                               clearLength(pos(pose), pose.heading + beamRel(i), cfg_.endpointMaxStep) });
        return c;
    }

    void enterMotionToGoal(bool resetHeuristic) {
        state_ = State::MotionToGoal;
        if (resetHeuristic) { hMin_ = Inf; preferredSide_ = Side::None; }
        blockedTimer_ = frontCloseTimer_ = lostTimer_ = goalFreeTimer_ = 0.0;
    }

    void enterRecovery() {
        if (recoveryAttempts_ == 0) recoverySpot_ = lastStepPos_;
        if (++recoveryAttempts_ > cfg_.maxRecoveryAttempts) {
            state_ = State::Fault;
            faultReason_ = FaultReason::NoSafeMotion;
            return;
        }
        state_ = State::Recovery;
        recoveryDir_ = 0.0;
        recoveryTurned_ = 0.0;
        recoveryOpenTimer_ = 0.0;
    }

    void enterFollowBoundary(Side wallSide, const Pose& pose, const SensorReading& s, const Goal& goal, bool turnFirst) {
        state_ = State::FollowBoundary;
        side_ = wallSide;
        preferredSide_ = wallSide;
        confirmLap_ = false;
        resetLoop(pose, goal);
        wallLostTime_ = 0.0;
        wallLostTurn_ = 0.0;
        uTurnLeft_ = 0.0;
        stallTimer_ = 0.0;
        stallPos_ = pos(pose);
        leaveTimer_ = 0.0;
        // Meeting the obstacle up close: turn away from it first - but only
        // until the front and the wall side are clear (escape turn below),
        // never a fixed-length rotation.
        escapeTurn_ = turnFirst;
        lastPose_ = pose;
        dFollowed_ = distanceToGoal(pose, goal);
        dReach_ = Inf;
        updateFollowed(s, pose, goal);
    }

    // Start loop-closure bookkeeping (hit point, turning, goal winding) afresh.
    void resetLoop(const Pose& pose, const Goal& goal) {
        hitPoint_ = pos(pose);
        followPath_ = 0.0;
        followTime_ = 0.0;
        leftHitArea_ = false;
        totalTurn_ = 0.0;
        haveHitHeading_ = false;
        haveRef_ = false;
        goalWind_ = 0.0;
        lastGoalBearing_ = std::atan2(goal.y - pose.y, goal.x - pose.x);
    }

    // Only echoes near the robot belong to the boundary being followed; a far
    // echo from some other obstacle closer to the goal must not lower
    // d_followed (that would make the leave condition unreachable).
    void updateFollowed(const SensorReading& s, const Pose& pose, const Goal& goal) {
        for (int i = 0; i < 3; ++i) {
            if (!hit(s, i) || reading(s, i) > cfg_.wallSeenRange) continue;
            dFollowed_ = std::min(dFollowed_, dist(along(pose, beamRel(i), range(s, i)), pt(goal)));
        }
    }

    // Nearest remembered echo within maxDist on the wall side of the robot
    // (the half-plane to the right for a wall on the right).
    bool nearestWallPoint(const Pose& pose, bool wallOnRight, double maxDist, Point& out) const {
        const double c = std::cos(pose.heading), sn = std::sin(pose.heading);
        double best = maxDist;
        bool found = false;
        for (const auto& m : mem_) {
            if (!alive(m)) continue;
            const double rx = m.x - pose.x, ry = m.y - pose.y;
            const double cross = c * ry - sn * rx; // > 0: point is on the left
            if (wallOnRight ? cross > -0.02 : cross < 0.02) continue;
            const double d = std::hypot(rx, ry);
            if (d < best) { best = d; out = { m.x, m.y }; found = true; }
        }
        return found;
    }

    // Minimum goal distance over the free space visible along the beams
    // (each beam's free part, shortened by the robot body and by remembered
    // points in the body corridor). `reachPoint_` = where that minimum is
    // attained (the robot's own position if nothing visible is closer).
    double computeReach(const SensorReading& s, const Pose& pose, const Goal& goal) {
        double best = dist(pos(pose), pt(goal));
        reachPoint_ = pos(pose);
        for (int i = 0; i < 3; ++i) {
            if (!valid(s, i)) continue;
            const double freeLen = range(s, i) - sideLimit();
            if (freeLen <= 0.0) continue;
            // Closest point to the goal on the beam's free segment. Not
            // clipped by remembered points here: the beam saw through (e.g. a
            // gap), and leaveBoundary() checks that the body fits.
            const double a = pose.heading + beamRel(i);
            const double t = std::clamp((goal.x - pose.x) * std::cos(a) + (goal.y - pose.y) * std::sin(a), 0.0, freeLen);
            const Point p = along(pose, beamRel(i), t);
            const double d = dist(p, pt(goal));
            if (d < best) { best = d; reachPoint_ = p; }
        }
        return best;
    }

    // Leave the boundary toward the point that achieved d_reach (Tangent
    // Bug's leaving rule), or straight to motion-to-goal if that point is
    // where the robot already is.
    // Returns false (keep following) if no body-wide straight corridor leads
    // toward the d_reach point yet.
    bool leaveBoundary(const Pose& pose, const Goal& goal) {
        const double d = dist(pos(pose), reachPoint_);
        if (d < cfg_.endpointMinStep) { enterMotionToGoal(true); return true; }
        const double a = std::atan2(reachPoint_.y - pose.y, reachPoint_.x - pose.x);
        const double len = std::min(d, cfg_.endpointMaxStep + cfg_.passageBeyond);
        double th;
        if (!clearHeading(pos(pose), a, len, cfg_.passageWindow, th)) return false;
        enterMotionToGoal(true);
        const Point p{ pose.x + len * std::cos(th), pose.y + len * std::sin(th) };
        startMove(pose, th, len, len + dist(p, pt(goal)));
        return true;
    }

    // The goal ray (front beam, robot aligned with the goal) is blocked within
    // `blockDist`. Pick the endpoint with the smallest heuristic and move to
    // it while the heuristic keeps decreasing; otherwise follow the boundary.
    // `close` = something is already inside d_stop, no room to manoeuvre.
    // Returns false when the right move is to keep approaching the obstacle
    // along the goal ray first (boundary following only starts near it).
    bool handleBlocked(const SensorReading& s, const Pose& pose, const Goal& goal, double blockDist, bool close) {
        if (tryPassage(s, pose, goal, blockDist)) return true;
        const bool approach = !close && blockDist > cfg_.d_slow;
        Candidate left = endpointCandidate(s, pose, goal, 0, blockDist);
        Candidate right = endpointCandidate(s, pose, goal, 2, blockDist);

        // Keep the previously chosen side unless the other one is clearly better.
        double hl = left.h, hr = right.h;
        if (preferredSide_ == Side::Right && hl < Inf) hl -= cfg_.sideSwitchMargin; // endpoint left -> wall on right
        if (preferredSide_ == Side::Left && hr < Inf) hr -= cfg_.sideSwitchMargin;

        if (hl < Inf || hr < Inf) {
            const bool goLeft = hl <= hr;
            const Candidate& c = goLeft ? left : right;
            const Side wallSide = goLeft ? Side::Right : Side::Left;
            if (!close && c.h < hMin_ - cfg_.heuristicEps && c.stepLen >= cfg_.endpointMinStep) {
                preferredSide_ = wallSide;
                side_ = wallSide;
                startMove(pose, pose.heading + beamRel(goLeft ? 0 : 2), c.stepLen, c.h);
                return true;
            }
            // Heuristic stopped decreasing (local minimum) or no room: follow
            // the boundary - once the obstacle is near enough to be followed.
            if (approach) { preferredSide_ = wallSide; return false; }
            enterFollowBoundary(wallSide, pose, s, goal, close);
            return true;
        }

        // No endpoint visible on either side: the obstacle spans the whole
        // sensed field - Tangent Bug falls back to boundary following, here
        // on the side with more room (or the side already chosen).
        if (!s.leftValid && !s.rightValid) { enterRecovery(); return true; }
        if (approach) return false;
        Side wallSide;
        if (preferredSide_ != Side::None) wallSide = preferredSide_;
        else if (!s.rightValid) wallSide = Side::Right;
        else if (!s.leftValid) wallSide = Side::Left;
        else wallSide = (s.left >= s.right) ? Side::Right : Side::Left;
        enterFollowBoundary(wallSide, pose, s, goal, true);
        return true;
    }

    // Both side beams inside the side limit = a narrow passage. If it is
    // still wide enough for the body and the front is open, creep through it
    // centred between the two sides; otherwise Recovery. Returns true if it
    // produced the command for this tick.
    bool squeeze(const SensorReading& s, Command& cmd) {
        const bool lc = s.leftValid && s.left < sideLimit();
        const bool rc = s.rightValid && s.right < sideLimit();
        if (!(lc && rc)) return false;
        const double hard = cfg_.robotRadius + cfg_.hardSideMargin;
        if (s.left < hard || s.right < hard || !s.frontValid || s.front < cfg_.d_stop) {
            enterRecovery();
            cmd = {};
            return true;
        }
        cmd = { cfg_.minDrive, clampTurn(cfg_.wallGain * (s.left - s.right)) };
        return true;
    }

    // -------------------------------------------------------- MotionToGoal --
    Command doMotionToGoal(const SensorReading& s, const Pose& pose, const Goal& goal, double distToGoal, double dt) {
        const bool leftClose = s.leftValid && s.left < sideLimit();
        const bool rightClose = s.rightValid && s.right < sideLimit();
        if (confirm(lostTimer_, !s.frontValid, dt)) { enterRecovery(); return {}; }

        // Not yet facing the goal: curve toward it (arc), no turn-then-go.
        // The goal-ray checks below need the front beam on the goal ray, so
        // they run once the heading error is small; meanwhile arcTo() never
        // drives forward into anything close ahead.
        const double goalHeading = std::atan2(goal.y - pose.y, goal.x - pose.x);
        const double err = normalize(goalHeading - pose.heading);
        if (std::abs(err) > cfg_.headingTol) {
            blockedTimer_ = frontCloseTimer_ = 0.0;
            if (Command sq; squeeze(s, sq)) return sq;
            if (leftClose != rightClose && aheadFree(s, pose)) {
                // Something brushes one side: keep curving, but away from it.
                const double r = leftClose ? s.left : s.right;
                if (r < cfg_.robotRadius + cfg_.hardSideMargin) {
                    enterFollowBoundary(leftClose ? Side::Left : Side::Right, pose, s, goal, true);
                    return {};
                }
                return { cfg_.minDrive, leftClose ? -0.5 * cfg_.turnSpeed : 0.5 * cfg_.turnSpeed };
            }
            const double vmax = cfg_.driveSpeed * std::clamp(distToGoal / (4.0 * cfg_.goalTolerance), 0.2, 1.0);
            return arcTo(s, pose, goalHeading, vmax);
        }

        // Aligned: the front beam looks along the goal ray.
        if (Command sq; squeeze(s, sq)) return sq;
        const double look = std::min(cfg_.sensingRange, distToGoal);
        const double memClear = clearLength(pos(pose), goalHeading, look);
        const double frontDist = s.frontValid ? range(s, 1) : 0.0;
        const double blockDist = std::min(frontDist < look ? frontDist : Inf, memClear < look ? memClear + corridor() : Inf);
        const bool blocked = blockDist < Inf;
        // "Close" = the obstacle on the goal ray is already inside d_stop (an
        // obstacle BEHIND a near goal does not count), or something is about
        // to touch the bumper.
        const bool touching = s.frontValid && s.front < cfg_.robotRadius + 0.05;
        const bool close = (blocked && blockDist < cfg_.d_stop) || touching;

        if (confirm(frontCloseTimer_, close, dt) || leftClose || rightClose) {
            if (!close) {
                // Something brushes one side only. Unless it is really close
                // (then follow it as a boundary), creep on while steering away
                // from it - turning that into a boundary-following episode
                // here ping-pongs with the leave condition.
                const double r = leftClose ? s.left : s.right;
                if (r >= cfg_.robotRadius + cfg_.hardSideMargin)
                    return { cfg_.minDrive, leftClose ? -0.5 * cfg_.turnSpeed : 0.5 * cfg_.turnSpeed };
                enterFollowBoundary(leftClose ? Side::Left : Side::Right, pose, s, goal, true);
                return {};
            }
            handleBlocked(s, pose, goal, blocked ? blockDist : frontDist, true);
            return {};
        }
        if (confirm(blockedTimer_, blocked, dt) && handleBlocked(s, pose, goal, blockDist, false)) return {};

        double v = frontLimitedSpeed(s, cfg_.driveSpeed);
        v = std::min(v, cfg_.driveSpeed * std::clamp(distToGoal / (4.0 * cfg_.goalTolerance), 0.2, 1.0));
        v = std::min(v, memClear); // never drive into remembered points
        if (blocked) v = std::min(v, cfg_.followSpeed); // unconfirmed obstacle: creep while deciding
        return { v, clampTurn(cfg_.headingGain * err) };
    }

    // ------------------------------------------------------ MoveToEndpoint --
    Command doMoveToEndpoint(const SensorReading& s, const Pose& pose, const Goal& goal, double distToGoal, double dt) {
        moveTimer_ += dt;
        const bool leftClose = s.leftValid && s.left < sideLimit();
        const bool rightClose = s.rightValid && s.right < sideLimit();

        if (confirm(goalFreeTimer_, goalRayFree(s, pose, goal, distToGoal), dt)) {
            enterMotionToGoal(false);
            return {};
        }

        const bool wallSideClose = side_ == Side::Right ? rightClose : leftClose;
        const bool otherSideClose = side_ == Side::Right ? leftClose : rightClose;
        // A close front echo only blocks the way to the endpoint if the robot
        // is (nearly) heading there; while it still turns toward the
        // endpoint the front beam looks at the obstacle it is leaving, and
        // arcTo() already refuses to drive forward into it.
        const double toTarget = normalize(std::atan2(endpoint_.y - pose.y, endpoint_.x - pose.x) - pose.heading);
        const bool frontOnPath = std::abs(toTarget) < 2.0 * cfg_.beamHalfWidth;
        if (confirm(frontCloseTimer_, frontOnPath && s.frontValid && s.front < cfg_.d_stop, dt) || wallSideClose) {
            enterFollowBoundary(side_, pose, s, goal, true);
            return {};
        }
        if (Command sq; squeeze(s, sq)) return sq;
        if (otherSideClose) {
            const double r = side_ == Side::Right ? s.left : s.right;
            if (r < cfg_.robotRadius + cfg_.hardSideMargin) { enterRecovery(); return {}; }
            // Creep on, steering away from that side (toward the wall side) -
            // only if nothing is close ahead; otherwise just turn.
            const double w = side_ == Side::Right ? -0.5 * cfg_.turnSpeed : 0.5 * cfg_.turnSpeed;
            return { aheadFree(s, pose) ? cfg_.minDrive : 0.0, w };
        }

        const double remaining = dist(pos(pose), endpoint_);
        if (remaining < cfg_.endpointTol || moveTimer_ > moveTimeout_) {
            enterMotionToGoal(false); // re-evaluate from here, keeping hMin_
            return {};
        }

        const double target = std::atan2(endpoint_.y - pose.y, endpoint_.x - pose.x);

        // New echoes now block the straight way to the endpoint: re-aim at a
        // nearby body-wide corridor, or re-evaluate if there is none.
        if (clearLength(pos(pose), target, remaining) < remaining - cfg_.endpointTol) {
            double th;
            if (!clearHeading(pos(pose), target, remaining, 30.0 * Pi / 180.0, th)) {
                enterMotionToGoal(false);
                return {};
            }
            endpoint_ = { pose.x + remaining * std::cos(th), pose.y + remaining * std::sin(th) };
            precise_ = true;
            return arcTo(s, pose, th, cfg_.driveSpeed, true);
        }
        // Curve toward the endpoint (arc), re-checking the goal ray every
        // packet above - as soon as it opens the robot heads for the goal.
        return arcTo(s, pose, target, cfg_.driveSpeed, precise_);
    }

    // ------------------------------------------------------ FollowBoundary --
    Command doFollowBoundary(const SensorReading& s, const Pose& pose, const Goal& goal, double dt) {
        followTime_ += dt;
        followPath_ += std::hypot(pose.x - lastPose_.x, pose.y - lastPose_.y);
        const double dHeading = normalize(pose.heading - lastPose_.heading);
        const double turned = std::abs(dHeading);
        // The in-place turn that starts a confirmation lap is not part of
        // the boundary traversal (a stall U-turn in a pocket is, and counts).
        if (!(confirmTurn_ && uTurnLeft_ > 0.0)) totalTurn_ += dHeading;
        lastPose_ = pose;
        const double goalBearing = std::atan2(goal.y - pose.y, goal.x - pose.x);
        goalWind_ += normalize(goalBearing - lastGoalBearing_);
        lastGoalBearing_ = goalBearing;
        if (followTime_ > cfg_.maxFollowTimeS) { enterRecovery(); return {}; }

        // Loop closure: back at the hit point after going around -> the goal
        // is enclosed by this obstacle (Tangent Bug's termination condition).
        // "Around" is checked topologically: a full lap around a boundary
        // turns the robot by ~ +-2 Pi in total, while coming back out of a
        // dead-end pocket the same way (after a U-turn) only turns ~ +-Pi -
        // in a narrow channel that path can pass right next to the hit point.
        // A real lap also passes the hit point in the SAME direction it left
        // it, while coming back out of a pocket passes it the opposite way.
        // The lap is detected with a GATE: a line through the hit point, across
        // the direction of travel there. A lap crosses it again in the same
        // direction; laps driven as smooth arcs never retrace the exact same
        // track, so "back within a few cm of the hit point" would miss them.
        const double dHit = dist(pos(pose), hitPoint_);
        bool gateCrossed = false;
        if (!haveHitHeading_ && dHit > cfg_.loopTol) {
            hitHeading_ = std::atan2(pose.y - hitPoint_.y, pose.x - hitPoint_.x);
            haveHitHeading_ = true;
            gateAlong_ = dHit;
        } else if (haveHitHeading_) {
            const double rx = pose.x - hitPoint_.x, ry = pose.y - hitPoint_.y;
            const double along = rx * std::cos(hitHeading_) + ry * std::sin(hitHeading_);
            const double lat = -rx * std::sin(hitHeading_) + ry * std::cos(hitHeading_);
            gateCrossed = gateAlong_ < 0.0 && along >= 0.0 && std::abs(lat) < cfg_.loopGate;
            gateAlong_ = along;
        }
        if (dHit > 2.0 * cfg_.loopTol) leftHitArea_ = true;
        if (haveRef_ && leftHitArea_ && followPath_ > cfg_.loopMinTravel && gateCrossed
            && std::abs(totalTurn_) > 1.5 * Pi) {
            // Is the goal really cut off by this loop? Going around an
            // obstacle (robot turns toward the wall side overall) the goal
            // must be INSIDE the loop; going around the inside of an
            // enclosure (turns away from the wall side) it must be OUTSIDE.
            // Anything else is a loop around free space / a mis-sensed
            // boundary, which says nothing about reachability.
            const bool aroundObstacle = (side_ == Side::Right) ? totalTurn_ < 0.0 : totalTurn_ > 0.0;
            const bool goalInside = std::abs(goalWind_) > Pi;
            if (aroundObstacle != goalInside) {
                // Inconsistent loop: re-plan, trying the other side next time.
                ++badLoops_;
                if (badLoops_ > cfg_.maxRecoveryAttempts) {
                    state_ = State::Fault;
                    faultReason_ = FaultReason::NoSafeMotion;
                    return {};
                }
                enterMotionToGoal(true);
                preferredSide_ = (side_ == Side::Right) ? Side::Left : Side::Right;
                return {};
            }
            if (!confirmLap_) {
                // Confirm with a lap in the opposite direction (wall on the
                // other side) - a door missed from one side is often found
                // from the other - before declaring the goal unreachable.
                confirmLap_ = true;
                side_ = (side_ == Side::Right) ? Side::Left : Side::Right;
                preferredSide_ = side_;
                resetLoop(pose, goal);
                uTurnLeft_ = Pi;
                confirmTurn_ = true;
                return {};
            }
            state_ = State::Fault;
            faultReason_ = FaultReason::GoalUnreachable;
            return {};
        }

        const bool wallOnRight = (side_ == Side::Right);
        const double away = wallOnRight ? cfg_.turnSpeed : -cfg_.turnSpeed; // turn away from the wall


        // Stall: alternating emergency turns in a passage too tight to drive
        // through. Treat it as a dead end - rotate 180 deg in place (always
        // safe for the round base) and keep following the boundary.
        if (uTurnLeft_ > 0.0) {
            uTurnLeft_ -= turned;
            // Stop turning as soon as the way ahead is open (after at least a
            // quarter turn so the robot really leaves the pocket wall).
            const bool openAhead = s.frontValid && s.front > cfg_.d_slow && uTurnLeft_ < 0.5 * Pi && !confirmTurn_;
            if (uTurnLeft_ > 0.0 && !openAhead) return { 0.0, away };
            uTurnLeft_ = 0.0;
            confirmTurn_ = false;
            escapeTurn_ = false;
            stallTimer_ = 0.0;
            stallTurn_ = 0.0;
            stallPos_ = pos(pose);
        }
        // Progress = leaving a stallDist circle, or turning a net ~180 deg (a
        // legitimate in-place turn at the end of a pocket). A livelock turns
        // back and forth with little net rotation.
        stallTimer_ += dt;
        stallTurn_ += dHeading;
        if (dist(pos(pose), stallPos_) > cfg_.stallDist || std::abs(stallTurn_) > 0.75 * Pi) {
            stallTimer_ = 0.0; stallTurn_ = 0.0; stallPos_ = pos(pose);
        }
        else if (stallTimer_ > cfg_.stallTimeS) {
            uTurnLeft_ = Pi;
            return { 0.0, away };
        }


        updateFollowed(s, pose, goal);
        dReach_ = computeReach(s, pose, goal);
        if (confirm(leaveTimer_, dReach_ < dFollowed_ - cfg_.leaveEps, dt) && leaveBoundary(pose, goal)) return {};

        const int wi = wallOnRight ? 2 : 0;
        const int oi = wallOnRight ? 0 : 2;
        const bool wallSeen = valid(s, wi) && reading(s, wi) < cfg_.wallSeenRange;
        Point q;
        const bool haveQ = (cfg_.followMemory || !wallSeen) && nearestWallPoint(pose, wallOnRight, cfg_.followMemoryRange, q);
        if (haveQ) dFollowed_ = std::min(dFollowed_, dist(q, pt(goal)));

        // The loop reference ("hit point") is taken once the robot actually
        // tracks the boundary at d_follow - where following was entered may be
        // far from the track (e.g. stopped short in the middle of a room),
        // and a lap would then never pass back near it.
        if (!haveRef_ && uTurnLeft_ <= 0.0) {
            const double lat = haveQ ? dist(pos(pose), q)
                             : (wallSeen ? reading(s, wi) * std::sin(cfg_.beamAngle) : Inf);
            if (std::abs(lat - cfg_.d_follow) < 0.1) {
                resetLoop(pose, goal);
                haveRef_ = true;
            }
        }
        wallLostTime_ = (wallSeen || haveQ) ? 0.0 : wallLostTime_ + dt;
        wallLostTurn_ = (wallSeen || haveQ) ? 0.0 : wallLostTurn_ + turned;
        if (wallLostTurn_ > cfg_.wallLostTurn || wallLostTime_ > cfg_.wallLostTimeS) { enterRecovery(); return {}; }

        // Emergencies: front blocked (echo or remembered point right ahead),
        // wall beam too close, or the opposite side too close.
        // Once an emergency rotation away from the wall has started, keep
        // turning the same way until both the front and the wall side are
        // clear again, ignoring the opposite side: rotating in place is safe
        // for the round base, and alternating left/right emergency turns in a
        // pocket narrower than the follow distance would otherwise livelock.
        // In a dead-end pocket this becomes an in-place U-turn.
        const bool memAhead = clearLength(pos(pose), pose.heading, 0.15) < 0.15;
        const bool frontBad = (s.frontValid && s.front < cfg_.d_stopFollow) || memAhead;

        // Narrow but passable passage (both sides inside the side limit, but
        // not dangerously close, front open): creep through centred, exactly
        // as MotionToGoal / MoveToEndpoint do, instead of spinning in it.
        const double hard = cfg_.robotRadius + cfg_.hardSideMargin;
        if (!frontBad && s.leftValid && s.rightValid && s.left < sideLimit() && s.right < sideLimit()
            && s.left >= hard && s.right >= hard) {
            escapeTurn_ = false;
            return { cfg_.minDrive, clampTurn(cfg_.wallGain * (s.left - s.right)) };
        }
        const bool wallBad = wallSeen && reading(s, wi) < sideLimit();
        if (frontBad || wallBad) escapeTurn_ = true;
        if (escapeTurn_) {
            const double hyst = 0.05;
            const bool frontOk = !memAhead && (!s.frontValid || s.front >= cfg_.d_stopFollow + hyst);
            const bool wallOk = !valid(s, wi) || reading(s, wi) >= sideLimit() + hyst;
            if (!(frontOk && wallOk)) return { 0.0, away };
            escapeTurn_ = false;
        }
        // Opposite side close (e.g. the far wall of a narrow passage): only a
        // really close echo forces an in-place turn; otherwise creep forward
        // while steering away from it - stopping and turning toward the
        // followed wall there just ping-pongs with the wall-side latch above.
        if (valid(s, oi) && reading(s, oi) < cfg_.robotRadius + cfg_.hardSideMargin) return { 0.0, -away };
        if (valid(s, oi) && reading(s, oi) < sideLimit()) return { aheadFree(s, pose) ? cfg_.minDrive : 0.0, -0.5 * away };

        double w;
        double v = frontLimitedSpeed(s, cfg_.followSpeed);
        if (haveQ) {
            // Follow the nearest remembered boundary point: drive along the
            // tangent to it, corrected toward/away from it to hold d_follow.
            // Rounding a corner the corner itself stays the nearest point, so
            // the robot arcs around it at d_follow instead of losing the wall.
            const double sgn = wallOnRight ? 1.0 : -1.0;
            const double dq = dist(pos(pose), q);
            const double tangent = std::atan2(q.y - pose.y, q.x - pose.x) + sgn * 0.5 * Pi;
            const double corr = std::clamp(cfg_.wallGain * (dq - cfg_.d_follow), -Pi / 3.0, Pi / 3.0);
            const double err = normalize(tangent - sgn * corr - pose.heading);
            w = cfg_.headingGain * err;
            v = std::abs(err) > Pi / 3.0 ? 0.0 : v * std::max(0.3, std::cos(err));
        } else if (wallSeen) {
            const double lateral = reading(s, wi) * std::sin(cfg_.beamAngle);
            w = (wallOnRight ? -1.0 : 1.0) * cfg_.wallGain * (lateral - cfg_.d_follow);
        } else {
            // Wall dropped out of the side beam (outer corner): arc around it.
            v = std::min(v, 0.75 * cfg_.followSpeed);
            w = (wallOnRight ? -1.0 : 1.0) * std::max(cfg_.minTurn, v / cfg_.d_follow);
        }

        // Obstacle ahead (inner corner, next wall): start curving away from
        // it followReact before the stop distance instead of driving up to
        // it and turning on the spot - keeps the sensors out of their
        // unreliable close range and rounds the corner in an arc.
        const double reactFar = cfg_.d_stopFollow + cfg_.followReact;
        if (s.frontValid && s.front < reactFar) {
            const double k = std::clamp((reactFar - s.front) / cfg_.followReact, 0.0, 1.0);
            w = (1.0 - k) * w + k * away;
            v = std::max(cfg_.minDrive, v * (1.0 - 0.6 * k));
        }
        return { v, clampTurn(w) };
    }

    // ------------------------------------------------------------ Recovery --
    // Find a free direction: rotate in place toward the side with more room
    // and leave the moment the way ahead is open (no scan to completion).
    // A full turn without any opening, or too many recoveries without
    // getting away from the spot -> Fault(NoSafeMotion).
    Command doRecovery(const SensorReading& s, const Pose& pose, double dt) {
        if (recoveryDir_ == 0.0) {
            const double l = s.leftValid ? s.left : 0.0, r = s.rightValid ? s.right : 0.0;
            recoveryDir_ = (l >= r) ? 1.0 : -1.0;
            recoveryLastHeading_ = pose.heading;
        }
        recoveryTurned_ += std::abs(normalize(pose.heading - recoveryLastHeading_));
        recoveryLastHeading_ = pose.heading;

        const double hard = cfg_.robotRadius + cfg_.hardSideMargin;
        const bool open = s.frontValid && s.front > cfg_.recoveryClear
                       && (!s.leftValid || s.left > hard) && (!s.rightValid || s.right > hard)
                       && clearLength(pos(pose), pose.heading, 0.5) >= 0.5;
        if (confirm(recoveryOpenTimer_, open, dt)) {
            enterMotionToGoal(true);
            return {};
        }
        if (recoveryTurned_ > cfg_.recoveryMaxTurn) {
            state_ = State::Fault;
            faultReason_ = FaultReason::NoSafeMotion;
            return {};
        }
        return { 0.0, recoveryDir_ * cfg_.turnSpeed };
    }

    Config cfg_;
    State state_ = State::Init;
    Side side_ = Side::None;
    Side preferredSide_ = Side::None;
    FaultReason faultReason_ = FaultReason::None;
    double time_ = 0.0;
    double nearBlockedGoalTime_ = 0.0;

    std::array<MemPoint, MemSize> mem_{};
    std::size_t memNext_ = 0;

    double hMin_ = Inf;
    Point endpoint_;
    bool precise_ = false;
    double moveTimer_ = 0.0, moveTimeout_ = 0.0;

    Point hitPoint_;
    Pose lastPose_;
    double followPath_ = 0.0, followTime_ = 0.0;
    bool leftHitArea_ = false;
    double dFollowed_ = Inf, dReach_ = Inf;
    Point reachPoint_;
    double wallLostTime_ = 0.0, wallLostTurn_ = 0.0;
    Point recoverySpot_, lastStepPos_;
    bool escapeTurn_ = false;
    double totalTurn_ = 0.0, hitHeading_ = 0.0, gateAlong_ = 0.0;
    double goalWind_ = 0.0, lastGoalBearing_ = 0.0;
    bool confirmLap_ = false, confirmTurn_ = false, haveRef_ = false;
    int badLoops_ = 0;
    bool haveHitHeading_ = false;
    double uTurnLeft_ = 0.0, stallTimer_ = 0.0, stallTurn_ = 0.0;
    Point stallPos_;


    double blockedTimer_ = 0.0, frontCloseTimer_ = 0.0, lostTimer_ = 0.0;
    double goalFreeTimer_ = 0.0, leaveTimer_ = 0.0;
    double recoveryDir_ = 0.0, recoveryTurned_ = 0.0, recoveryLastHeading_ = 0.0, recoveryOpenTimer_ = 0.0;
    int recoveryAttempts_ = 0;
};

} // namespace tbug
