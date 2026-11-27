// FrontierStrategy.h - 11_01: frontier based exploration (Yamauchi 1997), the
// method of the lecture and of the first version of example 11.
//
//   frontier cell : undecided (0.3 <= P <= 0.8) with a known-EMPTY 4-neighbour
//   next move     : the frontier cell whose empty neighbour has the SHORTEST
//                   A* path from the robot (greedy, decided again after every
//                   measurement); stand there, face the cell, measure it
//   stop          : no reachable frontier left
//
// Greedy "nearest first": cheap to compute and usually short, but it can
// leave a far corner for later and come back for it (compare 11_04_TSP).
#pragma once

#include "Strategy.h"

#include <cstdio>
#include <string>
#include <vector>

namespace explore {

class FrontierStrategy final : public Strategy {
public:
    const char* id() const override { return "frontier"; }
    const char* name() const override { return "11_01 Frontier"; }
    const wchar_t* title() const override { return L"nearest frontier (A* path cost)"; }
    const char* rule() const override {
        return "go to the frontier cell (undecided, next to known-empty space) with the shortest A* path, measure it";
    }

    std::optional<Goal> next(const PlanContext& ctx) override {
        ranking_ = rankTargets(ctx, openFrontier(ctx));
        frontierCount_ = static_cast<int>(ctx.frontier.size());
        if (ranking_.empty()) return std::nullopt;
        const Pick& p = ranking_.front().pick;
        const auto& cs = ctx.grid.arena().coords;
        return Goal{ Action::Measure, p.target, p.approach, p.route.cells,
                     "frontier " + cs.text(p.target) + " -> going to " + cs.text(p.approach) + " (" +
                         std::to_string(ctx.frontier.size()) + " frontier cells)" };
    }

    void describe(StrategyView& v) const override {
        // Ranking of the reachable frontier cells by path cost (1 = chosen).
        for (std::size_t i = 0; i < ranking_.size() && i < 9; ++i)
            v.marks.push_back({ ranking_[i].pick.target, std::to_wstring(i + 1), i == 0 ? MarkKind::Order : MarkKind::Candidate });
        wchar_t b[160];
        swprintf(b, 160, L"frontier cells %d, reachable %d", frontierCount_, static_cast<int>(ranking_.size()));
        v.info.push_back(b);
        if (!ranking_.empty()) {
            swprintf(b, 160, L"numbers: rank by A* path cost (1 = nearest, %.2f m)", ranking_.front().pick.route.cost);
            v.info.push_back(b);
        }
    }

private:
    std::vector<Ranked> ranking_;
    int frontierCount_ = 0;
};

} // namespace explore
