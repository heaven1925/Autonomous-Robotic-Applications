// TspStrategy.h - 11_04: frontier exploration ordered by a travelling
// salesman tour (receding-horizon TSP, cf. Zlot 2002 / Kulich 2011).
//
// 11_01 always takes the NEAREST frontier - greedy, it may leave a corner
// behind and drive back for it later. Here every plan looks at ALL reachable
// frontier cells at once:
//
//   nodes      : every reachable frontier cell, measured from its best empty
//                neighbour (the "viewpoint", as in 11_01)
//   costs      : A* path lengths robot -> viewpoint and viewpoint -> viewpoint
//                (targets sharing a viewpoint cost 0 between them)
//   tour       : the shortest OPEN path from the robot through all nodes
//                (TspSolver.h: Held-Karp exact up to 13 nodes, else NN + 2-opt)
//   next move  : the FIRST node of that tour; after the measurement the map
//                has changed, so the tour is computed again (receding horizon)
//   stop       : no reachable frontier left
//
// The window shows the whole tour (purple) with the visiting order.
#pragma once

#include "Strategy.h"
#include "TspSolver.h"

#include <cstdio>
#include <string>
#include <vector>

namespace explore {

class TspStrategy final : public Strategy {
public:
    const char* id() const override { return "tsp"; }
    const char* name() const override { return "11_04 TSP"; }
    const wchar_t* title() const override { return L"frontier tour (travelling salesman)"; }
    const char* rule() const override {
        return "shortest tour through ALL reachable frontier cells (Held-Karp / 2-opt), go to its first stop, replan";
    }

    std::optional<Goal> next(const PlanContext& ctx) override {
        tourCells_.clear();
        tourPoints_.clear();
        const std::vector<Ranked> nodes = rankTargets(ctx, openFrontier(ctx));
        frontierCount_ = static_cast<int>(ctx.frontier.size());
        nodeCount_ = static_cast<int>(nodes.size());
        if (nodes.empty()) return std::nullopt;

        // Unique viewpoints and the A* costs between them.
        std::vector<Cell> views;
        std::vector<int> viewOf(nodes.size());
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const Cell a = nodes[i].pick.approach;
            const auto it = std::find(views.begin(), views.end(), a);
            viewOf[i] = static_cast<int>(it - views.begin());
            if (it == views.end()) views.push_back(a);
        }
        const std::size_t V = views.size();
        std::vector<std::vector<double>> vd(V, std::vector<double>(V, 0.0));
        for (std::size_t a = 0; a < V; ++a)
            for (std::size_t b = a + 1; b < V; ++b) {
                const Route r = routeAStar(ctx.cm, views[a], views[b]);
                const double c = r.found ? r.cost : 1e6;   // should not happen: both reachable from the robot
                vd[a][b] = vd[b][a] = c;
            }
        const std::size_t n = nodes.size();
        std::vector<double> d0(n);
        std::vector<std::vector<double>> d(n, std::vector<double>(n, 0.0));
        for (std::size_t i = 0; i < n; ++i) {
            d0[i] = nodes[i].cost;   // robot -> viewpoint (+ straight-line tie-break)
            for (std::size_t j = 0; j < n; ++j)
                d[i][j] = vd[static_cast<std::size_t>(viewOf[i])][static_cast<std::size_t>(viewOf[j])];
        }
        const tsp::Tour tour = tsp::solve(d0, d);
        exact_ = tour.exact;
        tourCost_ = tour.cost;

        const astar::GridMap& m = ctx.grid.arena();
        tourPoints_.push_back(m.centre(ctx.here));
        for (const int k : tour.order) {
            tourCells_.push_back(nodes[static_cast<std::size_t>(k)].pick.target);
            tourPoints_.push_back(m.centre(nodes[static_cast<std::size_t>(k)].pick.approach));
        }
        const Pick& first = nodes[static_cast<std::size_t>(tour.order.front())].pick;
        nearestIsFirst_ = tour.order.front() == 0;
        const auto& cs = m.coords;
        return Goal{ Action::Measure, first.target, first.approach, first.route.cells,
                     "TSP tour of " + std::to_string(n) + " frontier cells (" + std::to_string(V) + " viewpoints, " +
                         fmt(tour.cost) + " m, " + (tour.exact ? "Held-Karp" : "NN+2-opt") + "): first " + cs.text(first.target) +
                         " from " + cs.text(first.approach) + (nearestIsFirst_ ? "" : "  [not the nearest one]") };
    }

    void describe(StrategyView& v) const override {
        for (std::size_t i = 0; i < tourCells_.size() && i < 15; ++i)
            v.marks.push_back({ tourCells_[i], std::to_wstring(i + 1), i == 0 ? MarkKind::Order : MarkKind::Candidate });
        v.tour = tourPoints_;
        v.tourLabel = L"TSP tour";
        wchar_t b[160];
        swprintf(b, 160, L"frontier cells %d, tour stops %d", frontierCount_, nodeCount_);
        v.info.push_back(b);
        if (nodeCount_ > 0) {
            swprintf(b, 160, L"tour length %.2f m (%ls)", tourCost_, exact_ ? L"Held-Karp, exact" : L"nearest neighbour + 2-opt");
            v.info.push_back(b);
            v.info.push_back(nearestIsFirst_ ? L"first stop = nearest frontier" : L"first stop is NOT the nearest frontier");
        }
    }

private:
    static std::string fmt(double v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%.2f", v);
        return b;
    }

    std::vector<Cell> tourCells_;
    std::vector<Point> tourPoints_;
    int frontierCount_ = 0, nodeCount_ = 0;
    double tourCost_ = 0.0;
    bool exact_ = false, nearestIsFirst_ = true;
};

} // namespace explore
