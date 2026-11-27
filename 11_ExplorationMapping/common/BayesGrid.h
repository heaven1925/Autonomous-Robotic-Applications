// BayesGrid.h - Bayesian occupancy grid on the lab cells + the experimental
// HC-SR04 sensor model (ders notu: "Bayesian Mapping", "Method A").
//
// Every cell (i, j) stores P(Occupied); P(Empty) = 1 - P(Occupied).
//   start              : 0.5 (unknown)
//   P < EmptyBelow     : empty     (0.3)
//   P > OccupiedAbove  : occupied  (0.8)
//   otherwise          : undecided
//
// Bayes update rule for a measurement event s_n:
//   P(O|s_n) = P(s_n|O) P(O|s_n-1) / ( P(s_n|O) P(O|s_n-1) + P(s_n|E) P(E|s_n-1) )
//
// Sensor model (measured, see calibrate in main.cpp):
//   event "hit"    = the range reading falls inside this cell
//                    P(hit|O) = pHitOccupied   (e.g. 80 / 100 = 0.80)
//                    P(hit|E) = pHitEmpty      (e.g. 20 / 100 = 0.20)
//   event "passed" = the reading is beyond this cell (the beam went through)
//                    P(passed|O) = 1 - P(hit|O),  P(passed|E) = 1 - P(hit|E)
// Two ways a reading is used:
//   face-on  : robot in a cell, facing the measured cell 1 ahead - exactly
//              the calibration set-up, so P(hit|O), P(hit|E) are used as
//              measured (Explorer.h, measureDirect).
//   cone     : any other reading (integrate below). The sonar returns the
//              nearest surface anywhere in its ~24 deg cone: cells crossed
//              before z are "passed", the cells on the arc at z share the
//              "hit" by prior odds, cells behind z are not touched. These
//              readings are less reliable than the calibrated case, so they
//              use at most the lecture's confidence (0.80 / 0.20). Sound does
//              not go through a cell already believed occupied.
#pragma once

#include "AStarPlanner.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace bayes {

using astar::Cell;
using astar::Point;

constexpr double EmptyBelow = 0.3;
constexpr double OccupiedAbove = 0.8;

enum class State { Undecided, Empty, Occupied };

// ---------------------------------------------------------- sensor model --
// Standard normal CDF.
inline double phi(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

// HC-SR04 model, both lecture methods:
//   Method B (Gaussian): a surface at distance d (from the sensor) is detected
//     with probability pDetect and then read as z ~ N(d + bias, sigma(d)),
//     sigma(d) = sigma0 + sigmaPerM * d (the noise grows with distance).
//     P(reading in [w0, w1] | occupied) = pDetect * (Phi((w1-d-bias)/s) - Phi((w0-d-bias)/s))
//   Method A (experiment): P(reading lands in a cell | the cell is empty) =
//     pHitEmpty, counted with the cell empty (false / multipath readings).
// The calibration (main.cpp, --calibrate) measures all of them.
struct SensorModel {
    double sigma0 = 0.01;         // m: noise at 0 m          (default: sigma(4 m) = 0.10 m, the lecture example)
    double sigmaPerM = 0.0225;    // m per m: noise growth
    double bias = 0.0;            // m: mean reading - true distance
    double pDetect = 0.95;        // a surface in front of the sensor gives an echo near its distance
    double pHitEmpty = 0.20;      // P(reading lands in the cell | cell empty)
    double maxRange = 4.0;        // m: HC-SR04 range; beyond = no echo
    double hitRange = 1.0;        // m: cone hits farther than this are not used (the cone is too wide to say which cell)
    double noEchoRange = 0.80;    // m: "no echo" = free at least this far (shorter: grazing walls give no echo)
    double coneHalfDeg = 12.0;    // HC-SR04 beam: half opening angle (region 2 arc)
    double emptyHalfDeg = 6.0;    // inner part of the cone trusted as empty (region 1)
    double grazingDeg = 50.0;     // a wall hit flatter than this (angle from its normal) may give NO echo
    std::string source = "default (sigma(4 m) = 10 cm, P(hit|E) = 0.20 - run the calibration)";

    double sigma(double d) const { return sigma0 + sigmaPerM * std::max(d, 0.0); }

    // Method B: P(reading in [w0, w1] | surface at d).
    double pWindow(double d, double w0, double w1) const {
        const double s = sigma(d), mu = d + bias;
        return pDetect * (phi((w1 - mu) / s) - phi((w0 - mu) / s));
    }

    // The event "the reading lands in this cell": a cell whose near face is
    // s metres from the sensor and which is L deep along the beam. Readings
    // of its face scatter around s, so the window starts 2 sigma before it.
    double windowStart(double s) const { return s - 2.0 * sigma(s); }
    double windowEnd(double s, double L) const { return s + L - 2.0 * sigma(s); }
    double pHitOccupied(double s, double L) const { return pWindow(s, windowStart(s), windowEnd(s, L)); }

    // Cone readings (oblique, shared arc) are less reliable than the clean
    // face-on case the model describes: at most the lecture's 0.80 / 0.20.
    double coneHitOccupied(double s, double L) const { return std::min(pHitOccupied(s, L), 0.80); }
    double coneHitEmpty() const { return std::max(pHitEmpty, 0.20); }
};

// sensor_model.txt: "key value" lines (written by --calibrate).
inline bool loadSensorModel(const std::string& path, SensorModel& m, std::string& error) {
    std::ifstream in(path);
    if (!in) { error = "cannot open " + path; return false; }
    SensorModel r = m;
    std::string line;
    while (std::getline(in, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ss(line);
        std::string key;
        double v = 0.0;
        if (!(ss >> key)) continue;
        if (!(ss >> v)) { error = path + ": no number after '" + key + "'"; return false; }
        if (key == "sigma0") r.sigma0 = v;
        else if (key == "sigma_per_m") r.sigmaPerM = v;
        else if (key == "bias") r.bias = v;
        else if (key == "p_detect") r.pDetect = v;
        else if (key == "p_hit_empty") r.pHitEmpty = v;
        else if (key == "max_range") r.maxRange = v;
        else if (key == "hit_range") r.hitRange = v;
        else if (key == "no_echo_range") r.noEchoRange = v;
        else if (key == "cone_half_deg") r.coneHalfDeg = v;
        else if (key == "empty_half_deg") r.emptyHalfDeg = v;
        else { error = path + ": unknown key '" + key + "'"; return false; }
    }
    if (!(r.sigma0 > 0.0) || r.sigmaPerM < 0.0) { error = path + ": sigma0 must be > 0, sigma_per_m >= 0"; return false; }
    if (!(r.pDetect > 0.0 && r.pDetect <= 1.0 && r.pHitEmpty > 0.0 && r.pHitEmpty < 1.0)) {
        error = path + ": p_detect must be in (0, 1], p_hit_empty in (0, 1)";
        return false;
    }
    r.source = path;
    m = r;
    return true;
}

// Bayes update of P = 0.5 with one event of these likelihoods.
inline double posterior(double prior, double likeO, double likeE) {
    return likeO * prior / (likeO * prior + likeE * (1.0 - prior));
}

// ------------------------------------------------------------------ grid --
class BayesGrid {
public:
    explicit BayesGrid(const astar::GridMap& arena)
        : arena_(arena), p_(static_cast<std::size_t>(arena.cols()) * static_cast<std::size_t>(arena.rows()), 0.5),
          updates_(p_.size(), 0) {}

    const astar::GridMap& arena() const { return arena_; }
    int cols() const { return arena_.cols(); }
    int rows() const { return arena_.rows(); }
    bool inside(Cell c) const { return arena_.inside(c); }

    double p(Cell c) const { return p_[idx(c)]; }
    int updates(Cell c) const { return updates_[idx(c)]; }
    State state(Cell c) const {
        const double v = p(c);
        if (v < EmptyBelow) return State::Empty;
        if (v > OccupiedAbove) return State::Occupied;
        return State::Undecided;
    }
    bool empty(Cell c) const { return inside(c) && state(c) == State::Empty; }
    bool occupied(Cell c) const { return inside(c) && state(c) == State::Occupied; }

    int count(State s) const {
        int n = 0;
        for (int y = 0; y < rows(); ++y)
            for (int x = 0; x < cols(); ++x) n += state({ x, y }) == s ? 1 : 0;
        return n;
    }
    bool allDecided() const { return count(State::Undecided) == 0; }

    // Bayes update rule with the likelihoods of the observed event.
    void update(Cell c, double likeOccupied, double likeEmpty) {
        if (!inside(c)) return;
        double& v = p_[idx(c)];
        const double num = likeOccupied * v;
        v = num / (num + likeEmpty * (1.0 - v));
        v = std::clamp(v, 0.01, 0.99);   // never fully certain: later evidence can still change it
        ++updates_[idx(c)];
    }

    // The robot stands here: the cell is empty ("mark your current position as empty").
    void markRobotCell(Cell c) { update(c, 0.05, 0.95); }

    // The bumper touched something in this cell: occupied. A touch is physical
    // proof, so the cell goes straight to "occupied" (0.90) even if the sonar
    // had called it empty - one Bayes step from 0.01 would only reach 0.16 and
    // the robot would drive into it again. A face-on check can still clear it.
    void markBump(Cell c) {
        if (!inside(c)) return;
        update(c, 0.95, 0.05);
        double& v = p_[idx(c)];
        v = std::max(v, 0.90);
    }

    // One range reading. sensor = sensor position, angle = beam centre
    // (map frame), z = reading from the sensor (m), echo = false: no echo.
    //
    // The sonar reports the NEAREST surface anywhere inside its cone, so the
    // reading is interpreted with the lecture's regions:
    //   region 1 (empty)    : cells the inner cone (+-emptyHalfDeg) crossed
    //                         completely before z  -> event "passed"; a ray
    //                         stops at a face it meets flatter than grazingDeg
    //                         (a wall there would give no echo)
    //   region 2 (occupied) : cells on the arc at range z (+-coneHalfDeg) -
    //                         ONE of them reflected. The hit evidence is shared
    //                         by prior odds: an arc touching the border wall or
    //                         a cell already believed occupied explains the
    //                         reading, the other arc cells learn little.
    //   region 3            : behind z - not seen, unchanged.
    struct BeamResult {
        std::vector<Cell> passed, hit;
    };
    BeamResult integrate(Point sensor, double angle, double z, bool echo, const SensorModel& m) {
        BeamResult r;
        const double L = arena_.cellSize();
        // Echo: free until the surface (minus 2 sigma, the reading may be short);
        // hit at z only if close enough to say which cell. No echo: free for
        // noEchoRange (missed echoes are common).
        bool useHit = echo && z <= m.hitRange;
        const double depth = 2.0 * m.sigma(z);
        const double passedUntil = echo ? std::min(z, m.maxRange) - depth : m.noEchoRange;
        const Cell own = arena_.cellAt(sensor);

        // A reading whose centre ray would have to go THROUGH a known wall is
        // inconsistent (ghost, multipath, missed echo): its hit is not used.
        if (useHit) {
            const double dx = std::cos(angle), dy = std::sin(angle);
            for (double t = 0.01; t < z - depth; t += 0.01) {
                const Cell c = arena_.cellAt({ sensor.x + dx * t, sensor.y + dy * t });
                if (!(c == own) && inside(c) && state(c) == State::Occupied) { useHit = false; break; }
            }
        }

        // Region 2: arc cells (and whether the arc reaches the border wall).
        std::vector<Cell> arc;
        bool arcOutside = false;
        if (useHit) {
            for (double d = -m.coneHalfDeg; d <= m.coneHalfDeg + 1e-9; d += 2.0) {
                const double a = angle + d * astar::Pi / 180.0;
                const double t = z + depth;   // just behind the measured surface
                const Cell c = arena_.cellAt({ sensor.x + std::cos(a) * t, sensor.y + std::sin(a) * t });
                if (!inside(c)) { arcOutside = arcOutside || arena_.walledBorder; continue; }
                if (c == own || contains(arc, c)) continue;
                arc.push_back(c);
            }
        }

        // Region 1: cells the inner cone crossed completely before passedUntil
        // (with the distance at which the ray entered them).
        std::vector<double> entered;
        for (double d = -m.emptyHalfDeg; d <= m.emptyHalfDeg + 1e-9; d += 2.0) {
            const double a = angle + d * astar::Pi / 180.0;
            const double dx = std::cos(a), dy = std::sin(a);
            Cell current = own;
            double entry = 0.0;
            for (double t = 0.01; t <= passedUntil + 0.01; t += 0.01) {
                const Cell c = arena_.cellAt({ sensor.x + dx * t, sensor.y + dy * t });
                if (c == current) continue;
                if (t <= passedUntil && inside(current) && !contains(arc, current) && !contains(r.passed, current)) {
                    r.passed.push_back(current);   // the ray left `current` before the reflection
                    entered.push_back(entry);
                }
                // Angle between the ray and the normal of the face it just crossed.
                // Flatter than grazingDeg: if `c` is a wall the sound glances off it
                // without an echo, so the reading says nothing about `c` (or beyond).
                const double nx = c.x != current.x ? std::abs(dx) : 0.0, ny = c.y != current.y ? std::abs(dy) : 0.0;
                const bool grazing = std::max(nx, ny) < std::cos(m.grazingDeg * astar::Pi / 180.0);
                current = c;
                entry = t;
                if (!inside(c)) break;
                if (state(c) == State::Occupied) break;   // sound does not go through a known wall: a longer reading is a ghost / multipath
                if (grazing) break;
            }
        }

        // Region 1 update, event "the reading is beyond this cell":
        //   P(passed|O) = 1 - P(hit|O) at the cell's distance (Gaussian),
        //   P(passed|E) = 1 - P(hit|E).
        // "No echo" is weaker evidence (HC-SR04 misses echoes): half the hit probability.
        for (std::size_t i = 0; i < r.passed.size(); ++i) {
            const double pHitO = m.coneHitOccupied(entered[i], L), pHitE = m.coneHitEmpty();
            const double pO = echo ? 1.0 - pHitO : 1.0 - 0.5 * pHitO;
            const double pE = echo ? 1.0 - pHitE : 1.0 - 0.5 * pHitE;
            update(r.passed[i], pO, pE);
        }

        // Region 2 update: likelihood ratio P(hit|O) / P(hit|E) at distance z,
        // shared by prior odds (an arc touching the wall explains the reading).
        if (!arc.empty()) {
            const double ratio = m.coneHitOccupied(z, L) / m.coneHitEmpty();
            double total = arcOutside ? 99.0 : 0.0;   // the border wall is certainly there
            for (const Cell& c : arc) total += p(c) / (1.0 - p(c));
            for (const Cell& c : arc) {
                const double w = (p(c) / (1.0 - p(c))) / total;
                update(c, std::max(1.0 + (ratio - 1.0) * w, 1e-6), 1.0);
                r.hit.push_back(c);
            }
        }
        return r;
    }
    // Frontier cell = an undecided cell with at least one empty neighbour (4-neighbours).
    std::vector<Cell> frontiers() const {
        std::vector<Cell> f;
        for (int y = 0; y < rows(); ++y)
            for (int x = 0; x < cols(); ++x) {
                const Cell c{ x, y };
                if (state(c) != State::Undecided) continue;
                for (const Cell n : neighbours4(c))
                    if (empty(n)) { f.push_back(c); break; }
            }
        return f;
    }

    std::vector<Cell> neighbours4(Cell c) const {
        std::vector<Cell> n;
        const Cell cand[4] = { { c.x + 1, c.y }, { c.x - 1, c.y }, { c.x, c.y + 1 }, { c.x, c.y - 1 } };
        for (const Cell& k : cand)
            if (inside(k)) n.push_back(k);
        return n;
    }

    // Map for A*: only cells known to be EMPTY are free; occupied and
    // undecided cells are obstacles (the robot never drives into the unknown).
    astar::GridMap planningMap() const {
        astar::GridMap m = arena_;
        for (int y = 0; y < rows(); ++y)
            for (int x = 0; x < cols(); ++x) m.setOccupied({ x, y }, !empty({ x, y }));
        return m;
    }

    // Result written to the txt: 0 = empty, 1 = occupied or never decided.
    astar::GridMap resultMap() const { return planningMap(); }

private:
    static bool contains(const std::vector<Cell>& v, Cell c) { return std::find(v.begin(), v.end(), c) != v.end(); }
    std::size_t idx(Cell c) const { return static_cast<std::size_t>(arena_.index(c)); }

    astar::GridMap arena_;
    std::vector<double> p_;
    std::vector<int> updates_;
};

} // namespace bayes
