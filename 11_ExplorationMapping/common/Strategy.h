// Strategy.h - the exchangeable part of exploration mapping: step 3 of the
// lecture's algorithm, "Pick a next move".
//
//   1. Initialize grid       common (BayesGrid.h)
//   2. Update the grid       common (Explorer::observe, BayesGrid::integrate)
//   3. Pick a next move      >>> Strategy::next() <<<   one class per method:
//                              11_01_Frontier  nearest frontier (A* cost)
//                              11_02_BFS       breadth-first order from the start
//                              11_03_Random    random reachable cell + 360 deg look
//                              11_04_TSP       shortest tour through all frontiers
//   4. Loop                  common (Explorer: drive, face, measure, verify, bumps)
//
// A strategy only DECIDES. It gets a read-only view of the grid and returns a
// Goal (where to stand, what to look at, the cell path to get there); the
// Explorer drives, measures and calls next() again. So every method uses the
// same sensor model, Bayes update, robot, safety layer and window - the
// comparison measures only the decision rule.
#pragma once

#include "AStarPlanner.h"
#include "BayesGrid.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace explore {

using astar::Cell;
using astar::Point;

// ------------------------------------------------------------- the input --
// What the explorer knows when it asks for the next move.
struct PlanContext {
    const bayes::BayesGrid& grid;
    const astar::CostMap& cm;           // known-EMPTY cells are free; occupied and undecided cells are obstacles
    Cell here;                          // robot cell (always free in cm)
    Cell start;                         // start cell of the run
    const std::vector<Cell>& frontier;  // undecided cells with an empty 4-neighbour
    const std::vector<Cell>& givenUp;   // cells the explorer gave up on (still undecided after maxVisits)
    double now;                         // s since the start

    bool isGivenUp(Cell c) const { return std::find(givenUp.begin(), givenUp.end(), c) != givenUp.end(); }
};

// ------------------------------------------------------------ the output --
enum class Action {
    Measure,      // drive to `approach` (a neighbour of target), face `target`, measure it face-on;
                  // still undecided afterwards -> 360 deg look-around from there
    LookAround,   // drive to `approach` (== target) and turn 360 deg there
};

struct Goal {
    Action action = Action::Measure;
    Cell target{};               // cell to measure (Measure) / cell to stand on (LookAround)
    Cell approach{};             // where the robot stands
    std::vector<Cell> path;      // robot cell ... approach (one cell = already there)
    std::string why;             // console event text
};

// ---------------------------------------------------------- drawing hints --
enum class MarkKind { Order, Candidate, Goal };
struct Mark {
    Cell cell;
    std::wstring label;          // short text drawn in the cell corner ("1", "2", ...)
    MarkKind kind = MarkKind::Order;
};

struct StrategyView {
    std::vector<Mark> marks;            // e.g. visiting order of the next targets
    std::vector<Point> tour;            // polyline in the map frame (e.g. the TSP tour)
    std::wstring tourLabel;             // legend text for the polyline
    std::vector<std::wstring> info;     // lines for the side panel
};

// ---------------------------------------------------------- the interface --
class Strategy {
public:
    virtual ~Strategy() = default;

    virtual const char* id() const = 0;            // short id for files / CSV: "frontier", "bfs", ...
    virtual const char* name() const = 0;          // "11_01 Frontier"
    virtual const wchar_t* title() const = 0;      // window title part
    virtual const char* rule() const = 0;          // one-line description of the decision rule

    // Called once before the first move.
    virtual void begin(const bayes::BayesGrid&, Cell /*start*/) {}

    // The next move, or nullopt when the method has nothing left to explore.
    virtual std::optional<Goal> next(const PlanContext& ctx) = 0;

    // Non-empty when next() returned nullopt for a reason of its own (not
    // "no frontier left"), e.g. the random walk stopped learning anything.
    virtual std::string stopReason() const { return {}; }

    // Optional overlay for the window (called from the drawing code).
    virtual void describe(StrategyView&) const {}
};

// =============================================================== helpers ==
// Shared by the strategies (and the explorer's verification step).

struct Route {
    bool found = false;
    double cost = 0.0;           // metres
    std::vector<Cell> cells;     // from ... to
};

// A* (8-neighbour, no corner cutting) over the known-empty cells.
inline Route routeAStar(const astar::CostMap& cm, Cell from, Cell to) {
    Route r;
    if (from == to) { r.found = true; r.cells = { from }; return r; }
    const auto s = astar::search(cm, from, to, astar::Connectivity::Eight);
    if (!s.found) return r;
    r.found = true;
    r.cost = s.cost;
    r.cells = s.path;
    return r;
}

// Breadth-first search (4-neighbour, every step one cell) over the known-empty
// cells: the shortest path in number of cells.
inline Route routeBfs(const astar::CostMap& cm, Cell from, Cell to) {
    Route r;
    if (from == to) { r.found = true; r.cells = { from }; return r; }
    const astar::GridMap& m = cm.map();
    if (!m.inside(from) || !m.inside(to) || cm.obstacle(from) || !cm.traversable(to)) return r;
    const std::size_t n = static_cast<std::size_t>(m.cols()) * static_cast<std::size_t>(m.rows());
    std::vector<int> parent(n, -2);
    std::deque<Cell> queue{ from };
    parent[static_cast<std::size_t>(m.index(from))] = -1;
    while (!queue.empty()) {
        const Cell c = queue.front();
        queue.pop_front();
        if (c == to) break;
        const bool leaving = !cm.traversable(c);
        for (int k = 0; k < 4; ++k) {
            if (!astar::moveAllowed(cm, c, k, leaving)) continue;
            const Cell nb{ c.x + astar::NeighbourDx[k], c.y + astar::NeighbourDy[k] };
            int& p = parent[static_cast<std::size_t>(m.index(nb))];
            if (p != -2) continue;
            p = m.index(c);
            queue.push_back(nb);
        }
    }
    if (parent[static_cast<std::size_t>(m.index(to))] == -2) return r;
    for (int i = m.index(to); i != -1; i = parent[static_cast<std::size_t>(i)]) r.cells.push_back(m.cellOf(i));
    std::reverse(r.cells.begin(), r.cells.end());
    r.found = true;
    r.cost = static_cast<double>(r.cells.size() - 1) * m.cellSize();
    return r;
}

// The best place to measure each target from: the empty 4-neighbour with the
// shortest A* path from the robot (the robot's own cell counts, cost 0).
// Among all targets the cheapest one wins; ties go to the target nearest the
// robot in a straight line. This is the nearest-frontier rule of 11_01, also
// used by the verification step (re-measure occupied cells face-on).
struct Pick {
    Cell target{}, approach{};
    Route route;
};
inline std::optional<Pick> nearestTarget(const PlanContext& ctx, const std::vector<Cell>& targets) {
    std::optional<Pick> best;
    double bestCost = std::numeric_limits<double>::infinity();
    for (const Cell& f : targets) {
        for (const Cell& n : ctx.grid.neighbours4(f)) {
            if (!ctx.grid.empty(n) && !(n == ctx.here)) continue;
            Route route;
            if (n == ctx.here) {
                route.found = true;
                route.cells = { ctx.here };
            } else {
                route = routeAStar(ctx.cm, ctx.here, n);
                if (!route.found) continue;
            }
            const double tie = 0.001 * std::hypot(f.x - ctx.here.x, f.y - ctx.here.y);
            if (route.cost + tie < bestCost) {
                bestCost = route.cost + tie;
                best = Pick{ f, n, route };
            }
        }
    }
    return best;
}

// Every reachable target with its best approach (as in nearestTarget), sorted
// by path cost; equal costs keep the input order, so the first entry is the
// one nearestTarget() returns.
struct Ranked {
    Pick pick;
    double cost = 0.0;   // route cost + straight-line tie-break
};
inline std::vector<Ranked> rankTargets(const PlanContext& ctx, const std::vector<Cell>& targets) {
    std::vector<Ranked> out;
    for (const Cell& f : targets) {
        if (auto one = nearestTarget(ctx, { f })) {
            const double tie = 0.001 * std::hypot(f.x - ctx.here.x, f.y - ctx.here.y);
            out.push_back({ *one, one->route.cost + tie });
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const Ranked& a, const Ranked& b) { return a.cost < b.cost; });
    return out;
}

// Frontier cells the explorer has not given up on.
inline std::vector<Cell> openFrontier(const PlanContext& ctx) {
    std::vector<Cell> out;
    for (const Cell& f : ctx.frontier)
        if (!ctx.isGivenUp(f)) out.push_back(f);
    return out;
}

} // namespace explore
