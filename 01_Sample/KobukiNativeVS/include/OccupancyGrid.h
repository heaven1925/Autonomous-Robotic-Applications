// OccupancyGrid.h - probabilistic occupancy grid mapping core.
//
// Like the planner headers next to it this file has NO dependency on Kobuki,
// serial ports, Arduino or Windows: robot poses and range readings in, a grid
// of occupancy probabilities out. The real robot / simulator glue lives in
// 09_Mapping/src.
//
// It implements the lecture's "Probabilistic OG Mapping Algorithm"
// (09_Mapping/lecture-mapping.pdf):
//   * every cell stores P(Occupied), initialised to 0.5 (= unknown);
//   * a range reading r splits the sensor cone into
//         region 1  (dist <  r) : probably empty
//         region 2  (dist ~= r) : probably occupied
//         region 3  (dist >  r) : hidden behind the echo, not updated;
//   * how much one reading is trusted for one cell is the lecture's sensor
//     correctness model
//         c = [ (R - r)/R + (B - a)/B ] / 2
//     (r = distance of the cell, a = its angle off the beam centreline,
//      R = maximum reliable range, B = cone half-angle): close to the sensor
//     and to the centreline = reliable, far away / at the cone edge = not;
//   * the cell is then updated with Bayes' rule
//         P(O|s) = P(s|O) P(O) / ( P(s|O) P(O) + P(s|E) P(E) );
//   * for moving and planning the probabilities are thresholded:
//         P > 0.7 occupied, P < 0.3 empty, anything else unknown.
//
// Two additions the lecture leaves to the implementer, both needed for a wide
// ultrasonic cone (HC-SR04) rather than a thin laser ray:
//   * the echo of a sonar comes from SOMEWHERE on the arc at distance r, not
//     necessarily from the centreline. Arc cells that are already believed
//     occupied "explain" the echo, so the other arc cells get proportionally
//     less evidence (weight = prior odds / largest prior odds on the arc).
//     Without this every wall grows a phantom layer in front of it.
//   * readings are only integrated after the robot moved or turned a little
//     (RangeMapper): Bayes' rule assumes independent measurements, and ten
//     identical readings per second from a standing robot are not.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ogm {

constexpr double Pi = 3.14159265358979323846;

inline double normalizeAngle(double a) {
    while (a > Pi) a -= 2.0 * Pi;
    while (a <= -Pi) a += 2.0 * Pi;
    return a;
}

enum class Cell : std::uint8_t { Unknown, Empty, Occupied };

struct GridConfig {
    double cellSize   = 0.05;  // m
    double halfExtent = 12.8;  // m - the grid covers [-halfExtent, +halfExtent] on both axes; the robot starts at its centre
    double pOccupied  = 0.70;  // P(Occupied) above this -> Occupied  (lecture threshold)
    double pEmpty     = 0.30;  // P(Occupied) below this -> Empty
    double pMin       = 0.02;  // probabilities are kept inside [pMin, pMax] so a cell can always change its mind
    double pMax       = 0.98;
};

struct SensorModel {
    double maxRange       = 2.0;               // m   - R: maximum reliable range. Farther echoes only say "empty up to R"
    double halfAngle      = 13.0 * Pi / 180.0; // rad - B: cone half-angle (the echo may come from anywhere inside it)
    double emptyHalfAngle = 8.0 * Pi / 180.0;  // rad - region 1 is only trusted this close to the centreline
    double noEchoRange    = 1.0;               // m   - "no echo at all" clears only this far: a smooth wall seen at a
                                               //       shallow angle also returns nothing
    double emptyMargin    = 0.02;              // m   - region 1 ends this far in front of the echo
    double hitTolerance   = 0.01;              // m   - region 2 starts this far in front of the echo (range noise)
    double hitDepth       = 0.10;              // m   - region 2 thickness: the obstacle is solid behind its surface
    double gainEmpty      = 0.60;              // P(s | true state) = 0.5 + 0.5 * gain * correctness, i.e. a cell the
    double gainOccupied   = 0.50;              //       model does not trust at all gets 0.5 = "no information"
};

// Lecture: P(correctness) = [ (R - r)/R + (B - a)/B ] / 2.
inline double sensorCorrectness(double r, double a, double R, double B) {
    return std::clamp(0.5 * ((R - r) / R + (B - std::abs(a)) / B), 0.0, 1.0);
}

class OccupancyGrid {
public:
    explicit OccupancyGrid(GridConfig cfg = {})
        : cfg_(cfg),
          n_(std::max(8, static_cast<int>(std::lround(2.0 * cfg.halfExtent / cfg.cellSize)))),
          p_(static_cast<std::size_t>(n_) * static_cast<std::size_t>(n_), 0.5f) {}

    const GridConfig& config() const { return cfg_; }
    int size() const { return n_; }                 // cells per side
    double cellSize() const { return cfg_.cellSize; }

    // World metres <-> cell index (same mapping on both axes).
    int toCell(double w) const { return static_cast<int>(std::floor((w + cfg_.halfExtent) / cfg_.cellSize)); }
    double toWorld(int i) const { return -cfg_.halfExtent + (static_cast<double>(i) + 0.5) * cfg_.cellSize; }
    bool inside(int ix, int iy) const { return ix >= 0 && iy >= 0 && ix < n_ && iy < n_; }

    double p(int ix, int iy) const { return inside(ix, iy) ? p_[index(ix, iy)] : 0.5; }

    Cell state(int ix, int iy) const {
        const double v = p(ix, iy);
        return v > cfg_.pOccupied ? Cell::Occupied : (v < cfg_.pEmpty ? Cell::Empty : Cell::Unknown);
    }

    // Bayes update of one cell. The sensor model is passed as P(s | Occupied);
    // P(s | Empty) = 1 - P(s | Occupied).
    void bayes(int ix, int iy, double pSensorGivenOccupied) {
        if (!inside(ix, iy)) return;
        float& cell = p_[index(ix, iy)];
        const double prior = cell;
        const double num = pSensorGivenOccupied * prior;
        const double posterior = num / (num + (1.0 - pSensorGivenOccupied) * (1.0 - prior));
        cell = static_cast<float>(std::clamp(posterior, cfg_.pMin, cfg_.pMax));
        touch(ix, iy);
    }

    // One range reading of one beam.
    //   (sx, sy) sensor position, axis = beam direction (world frame, rad),
    //   range = measured distance from the sensor (m),
    //   echo  = false: the sensor heard nothing at all.
    void integrateRange(double sx, double sy, double axis, double range, bool echo, const SensorModel& m) {
        const bool hit = echo && range <= m.maxRange;
        const double emptyEnd = hit ? range - m.emptyMargin
                                    : (echo ? m.maxRange : std::min(m.noEchoRange, m.maxRange));
        const double reach = hit ? range + m.hitDepth : emptyEnd;
        if (reach <= 0.0) return;

        // Bounding box of the cone sector (apex + points along the far arc).
        double xmin = sx, xmax = sx, ymin = sy, ymax = sy;
        for (int k = -2; k <= 2; ++k) {
            const double a = axis + m.halfAngle * static_cast<double>(k) / 2.0;
            const double px = sx + reach * std::cos(a), py = sy + reach * std::sin(a);
            xmin = std::min(xmin, px); xmax = std::max(xmax, px);
            ymin = std::min(ymin, py); ymax = std::max(ymax, py);
        }
        const int ix0 = std::max(0, toCell(xmin) - 1), ix1 = std::min(n_ - 1, toCell(xmax) + 1);
        const int iy0 = std::max(0, toCell(ymin) - 1), iy1 = std::min(n_ - 1, toCell(ymax) + 1);

        hits_.clear();
        double maxOdds = 0.0;
        for (int iy = iy0; iy <= iy1; ++iy) {
            const double dy = toWorld(iy) - sy;
            for (int ix = ix0; ix <= ix1; ++ix) {
                const double dx = toWorld(ix) - sx;
                const double d = std::hypot(dx, dy);
                if (d > reach) continue;
                const double a = d > 1e-6 ? std::abs(normalizeAngle(std::atan2(dy, dx) - axis)) : 0.0;
                if (a > m.halfAngle) continue;

                if (d < emptyEnd) {
                    // Region 1: probably empty.
                    if (a > m.emptyHalfAngle) continue;
                    const double c = sensorCorrectness(d, a, m.maxRange, m.halfAngle);
                    bayes(ix, iy, 0.5 - 0.5 * m.gainEmpty * c);
                } else if (hit && d >= range - m.hitTolerance) {
                    // Region 2: probably occupied - collected first, see below.
                    const double prior = p_[index(ix, iy)];
                    const double odds = prior / (1.0 - prior);
                    hits_.push_back({ix, iy, sensorCorrectness(range, a, m.maxRange, m.halfAngle), odds});
                    maxOdds = std::max(maxOdds, odds);
                }
            }
        }
        // The echo came from somewhere on this arc: the cells most likely to
        // be occupied already take the evidence, the rest proportionally less.
        for (const Hit& h : hits_)
            bayes(h.ix, h.iy, 0.5 + 0.5 * m.gainOccupied * h.c * (h.odds / maxOdds));
    }

    // "Mark your current position as high probability empty" (lecture step 2).
    void markEmptyDisc(double x, double y, double radius, double pValue = 0.05) {
        forDisc(x, y, radius, [&](float& c) { c = std::min(c, static_cast<float>(pValue)); });
    }

    // Something was touched here (bumper): occupied with high probability.
    void markOccupiedDisc(double x, double y, double radius, double pValue = 0.95) {
        forDisc(x, y, radius, [&](float& c) { c = std::max(c, static_cast<float>(pValue)); });
    }

    // Index range of every cell that was ever updated; false while the grid is untouched.
    bool knownBounds(int& ix0, int& iy0, int& ix1, int& iy1) const {
        if (bx1_ < bx0_) return false;
        ix0 = bx0_; iy0 = by0_; ix1 = bx1_; iy1 = by1_;
        return true;
    }

    // Number of cells currently in the given state (inside the known bounds).
    int count(Cell wanted) const {
        int n = 0;
        for (int iy = by0_; iy <= by1_; ++iy)
            for (int ix = bx0_; ix <= bx1_; ++ix)
                if (state(ix, iy) == wanted) ++n;
        return n;
    }

private:
    struct Hit { int ix, iy; double c, odds; };

    std::size_t index(int ix, int iy) const {
        return static_cast<std::size_t>(iy) * static_cast<std::size_t>(n_) + static_cast<std::size_t>(ix);
    }

    void touch(int ix, int iy) {
        bx0_ = std::min(bx0_, ix); bx1_ = std::max(bx1_, ix);
        by0_ = std::min(by0_, iy); by1_ = std::max(by1_, iy);
    }

    template <typename Fn>
    void forDisc(double x, double y, double radius, Fn&& fn) {
        const int ix0 = toCell(x - radius), ix1 = toCell(x + radius);
        const int iy0 = toCell(y - radius), iy1 = toCell(y + radius);
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int ix = ix0; ix <= ix1; ++ix) {
                if (!inside(ix, iy)) continue;
                if (std::hypot(toWorld(ix) - x, toWorld(iy) - y) > radius) continue;
                fn(p_[index(ix, iy)]);
                touch(ix, iy);
            }
        }
    }

    GridConfig cfg_;
    int n_;
    std::vector<float> p_;
    std::vector<Hit> hits_;
    int bx0_ = 1 << 30, by0_ = 1 << 30, bx1_ = -1, by1_ = -1;
};

// One beam of the sensor board. `range` is measured from the sensor itself.
struct BeamReading {
    double range = 0.0;  // m
    bool valid = false;  // false: field missing / broken packet -> ignored, never treated as free space
    bool echo = false;   // false: the sensor heard no echo (nothing within its own range)
};

// Feeds the three fixed beams of the robot (left-front at +beamAngle, front,
// right-front at -beamAngle; each sensor sits `sensorOffset` metres from the
// robot centre on its own beam line) into an OccupancyGrid.
class RangeMapper {
public:
    RangeMapper(OccupancyGrid& grid, SensorModel model, double beamAngle, double sensorOffset, double robotRadius)
        : grid_(grid), model_(model), beamAngle_(beamAngle), sensorOffset_(sensorOffset), robotRadius_(robotRadius) {}

    const SensorModel& model() const { return model_; }
    int updates() const { return updates_; }

    // Call once per new sensor packet with the pose of that moment. Returns
    // true if the readings were integrated (the robot moved or turned since
    // the last integrated packet).
    bool update(double x, double y, double heading,
                const BeamReading& left, const BeamReading& front, const BeamReading& right) {
        // The robot body is where the robot is: that space is empty.
        grid_.markEmptyDisc(x, y, robotRadius_ - 0.5 * grid_.cellSize());

        const bool moved = !have_ || std::hypot(x - lastX_, y - lastY_) >= MinMove
                        || std::abs(normalizeAngle(heading - lastHeading_)) >= MinTurn;
        if (!moved) return false;
        have_ = true;
        lastX_ = x; lastY_ = y; lastHeading_ = heading;

        const BeamReading* beams[3] = { &left, &front, &right };
        const double rel[3] = { beamAngle_, 0.0, -beamAngle_ };
        for (int i = 0; i < 3; ++i) {
            if (!beams[i]->valid) continue;
            const double a = heading + rel[i];
            grid_.integrateRange(x + sensorOffset_ * std::cos(a), y + sensorOffset_ * std::sin(a), a,
                                 beams[i]->range, beams[i]->echo, model_);
        }
        ++updates_;
        return true;
    }

private:
    static constexpr double MinMove = 0.02;              // m
    static constexpr double MinTurn = 1.5 * Pi / 180.0;  // rad

    OccupancyGrid& grid_;
    SensorModel model_;
    double beamAngle_, sensorOffset_, robotRadius_;
    bool have_ = false;
    double lastX_ = 0.0, lastY_ = 0.0, lastHeading_ = 0.0;
    int updates_ = 0;
};

// The coarse, deterministic "grid map" derived from the probability grid:
// big cells (about one robot each) that are occupied / empty / unknown - the
// representation in the lecture's figures and the one a grid path planner
// works on. Cell k covers [(k - 0.5) * cellSize, (k + 0.5) * cellSize] on its
// axis, so cell (0, 0) is centred on the robot's start position.
struct CellMap {
    double cellSize = 0.5;
    int ix0 = 0, iy0 = 0;  // index of the first stored cell
    int nx = 0, ny = 0;    // stored cells per axis
    std::vector<Cell> cells;

    static int indexOf(double w, double cellSize) { return static_cast<int>(std::floor(w / cellSize + 0.5)); }
    double centre(int i) const { return static_cast<double>(i) * cellSize; }

    Cell at(int ix, int iy) const {
        if (ix < ix0 || iy < iy0 || ix >= ix0 + nx || iy >= iy0 + ny) return Cell::Unknown;
        return cells[static_cast<std::size_t>(iy - iy0) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(ix - ix0)];
    }

    int count(Cell wanted) const { return static_cast<int>(std::count(cells.begin(), cells.end(), wanted)); }
};

struct CellMapConfig {
    double cellSize         = 0.50;  // m
    double emptyFraction    = 0.60;  // a big cell is empty when at least this share of its small cells is empty;
    double occupiedFraction = 0.03;  // otherwise occupied when at least this share of its small cells is ...
    int    minOccupiedCells = 3;     //   ... and at least this many; otherwise unknown
    double occupiedReach    = 0.06;  // m - occupied small cells are also counted this far OUTSIDE the big cell:
                                     //     sonar puts part of a wall surface a few cm in front of it, i.e. into
                                     //     the free neighbour (which stays empty - empty is decided first)
};

inline CellMap makeCellMap(const OccupancyGrid& g, const CellMapConfig& c) {
    CellMap m;
    m.cellSize = c.cellSize;
    int bx0, by0, bx1, by1;
    if (!g.knownBounds(bx0, by0, bx1, by1)) return m;

    m.ix0 = CellMap::indexOf(g.toWorld(bx0), c.cellSize);
    m.iy0 = CellMap::indexOf(g.toWorld(by0), c.cellSize);
    m.nx = CellMap::indexOf(g.toWorld(bx1), c.cellSize) - m.ix0 + 1;
    m.ny = CellMap::indexOf(g.toWorld(by1), c.cellSize) - m.iy0 + 1;
    m.cells.assign(static_cast<std::size_t>(m.nx) * static_cast<std::size_t>(m.ny), Cell::Unknown);

    for (int j = 0; j < m.ny; ++j) {
        const double y0 = (static_cast<double>(m.iy0 + j) - 0.5) * c.cellSize;
        const int fy0 = std::max(0, g.toCell(y0 + 1e-9)), fy1 = std::min(g.size() - 1, g.toCell(y0 + c.cellSize - 1e-9));
        for (int i = 0; i < m.nx; ++i) {
            const double x0 = (static_cast<double>(m.ix0 + i) - 0.5) * c.cellSize;
            const int fx0 = std::max(0, g.toCell(x0 + 1e-9)), fx1 = std::min(g.size() - 1, g.toCell(x0 + c.cellSize - 1e-9));
            int total = 0, occupied = 0, empty = 0;
            for (int fy = fy0; fy <= fy1; ++fy) {
                for (int fx = fx0; fx <= fx1; ++fx) {
                    ++total;
                    if (g.state(fx, fy) == Cell::Empty) ++empty;
                }
            }
            const int reach = static_cast<int>(std::lround(c.occupiedReach / g.cellSize()));
            for (int fy = std::max(0, fy0 - reach); fy <= std::min(g.size() - 1, fy1 + reach); ++fy)
                for (int fx = std::max(0, fx0 - reach); fx <= std::min(g.size() - 1, fx1 + reach); ++fx)
                    if (g.state(fx, fy) == Cell::Occupied) ++occupied;
            if (total == 0) continue;
            Cell& out = m.cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(m.nx) + static_cast<std::size_t>(i)];
            // Empty first: a wall surface usually lies ON a big-cell border, and
            // the strip of occupied small cells in front of it (sonar range
            // noise) must not turn the free cell next to the wall into a wall.
            // The wall cell behind the surface has no empty majority.
            const int need = std::max(c.minOccupiedCells, static_cast<int>(std::ceil(c.occupiedFraction * total)));
            if (static_cast<double>(empty) >= c.emptyFraction * total) out = Cell::Empty;
            else if (occupied >= need) out = Cell::Occupied;
        }
    }
    return m;
}

} // namespace ogm
