// BfsStrategy.h - 11_02: breadth-first search (BFS) exploration.
//
// The grid is a graph: cells are nodes, 4-neighbours are edges. Instead of
// "the frontier nearest to the ROBOT" the cells are explored in BFS order
// from the START cell - level by level, like rings growing around the start:
//
//   queue <- start
//   while queue not empty:
//       c <- pop front                         (FIFO = breadth first)
//       for each 4-neighbour n of c not seen yet:
//           empty      -> push n (expanded later, one level deeper)
//           undecided  -> TARGET: the first one met is measured next, from c
//           occupied   -> wall, not expanded
//
// The robot goes to c (the BFS parent, known empty) and measures n face-on.
// Paths are BFS shortest paths too (4-neighbour, fewest cells). The search
// is repeated after every measurement on the updated grid, so a cell that
// became empty is expanded and the order stays breadth-first from the start.
//
// Systematic and complete, but the order ignores where the robot is: after a
// far target the next one may be back near the start (more driving than 11_01).
#pragma once

#include "Strategy.h"

#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace explore {

class BfsStrategy final : public Strategy {
public:
    const char* id() const override { return "bfs"; }
    const char* name() const override { return "11_02 BFS"; }
    const wchar_t* title() const override { return L"breadth-first order from the start"; }
    const char* rule() const override {
        return "explore cells in breadth-first (FIFO) order from the start cell; drive along BFS shortest paths";
    }

    void begin(const bayes::BayesGrid&, Cell start) override { start_ = start; }

    std::optional<Goal> next(const PlanContext& ctx) override {
        const bayes::BayesGrid& g = ctx.grid;
        const astar::GridMap& m = g.arena();
        order_.clear();
        expanded_ = 0;

        // BFS from the start cell over known-empty cells (the robot's cell if
        // the start is not empty for some reason).
        const Cell root = g.empty(start_) ? start_ : ctx.here;
        std::vector<char> seen(static_cast<std::size_t>(m.cols()) * static_cast<std::size_t>(m.rows()), 0);
        std::vector<int> depth(seen.size(), 0);
        std::deque<Cell> queue{ root };
        seen[static_cast<std::size_t>(m.index(root))] = 1;
        while (!queue.empty()) {
            const Cell c = queue.front();
            queue.pop_front();
            ++expanded_;
            for (int k = 0; k < 4; ++k) {
                const Cell n{ c.x + astar::NeighbourDx[k], c.y + astar::NeighbourDy[k] };
                if (!g.inside(n)) continue;
                char& flag = seen[static_cast<std::size_t>(m.index(n))];
                if (flag) continue;
                flag = 1;
                const int d = depth[static_cast<std::size_t>(m.index(c))] + 1;
                depth[static_cast<std::size_t>(m.index(n))] = d;
                const auto st = g.state(n);
                if (st == bayes::State::Empty || n == ctx.here) queue.push_back(n);
                else if (st == bayes::State::Undecided && !ctx.isGivenUp(n)) order_.push_back({ n, c, d });
            }
        }

        // The first target (in BFS order) whose parent the robot can reach.
        for (std::size_t i = 0; i < order_.size(); ++i) {
            const Item& t = order_[i];
            const Route r = routeBfs(ctx.cm, ctx.here, t.parent);
            if (!r.found) continue;
            chosen_ = static_cast<int>(i);
            const auto& cs = m.coords;
            return Goal{ Action::Measure, t.cell, t.parent, r.cells,
                         "BFS #" + std::to_string(i + 1) + " (depth " + std::to_string(t.depth) + "): " + cs.text(t.cell) +
                             " measured from its BFS parent " + cs.text(t.parent) + " (" + std::to_string(order_.size()) +
                             " cells in the queue)" };
        }
        chosen_ = -1;
        return std::nullopt;
    }

    void describe(StrategyView& v) const override {
        // BFS visiting order of the undecided cells (1 = next).
        for (std::size_t i = 0; i < order_.size() && i < 12; ++i)
            v.marks.push_back({ order_[i].cell, std::to_wstring(i + 1),
                                static_cast<int>(i) == chosen_ ? MarkKind::Order : MarkKind::Candidate });
        v.marks.push_back({ start_, L"", MarkKind::Goal });   // BFS root
        wchar_t b[160];
        swprintf(b, 160, L"BFS root = start, %d cells expanded, %d targets queued", expanded_, static_cast<int>(order_.size()));
        v.info.push_back(b);
        if (chosen_ >= 0) {
            swprintf(b, 160, L"numbers: BFS (FIFO) order; next target depth %d", order_[static_cast<std::size_t>(chosen_)].depth);
            v.info.push_back(b);
        }
    }

private:
    struct Item {
        Cell cell, parent;
        int depth = 0;
    };
    Cell start_{};
    std::vector<Item> order_;
    int chosen_ = -1, expanded_ = 0;
};

} // namespace explore
