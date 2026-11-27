// RandomStrategy.h - 11_03: random exploration ("random walk" with random goals).
//
// The baseline every method should beat: no idea where the unknown is.
//
//   next move : a cell drawn UNIFORMLY at random from all known-empty cells
//               the robot can reach (not its own cell); drive there along
//               the A* path and turn 360 deg (look around)
//   stop      : no frontier left (nothing unknown next to free space), or
//               `maxIdle` look-arounds in a row decided no new cell
//               (stagnation: the remaining cells need a face-on look the
//               random walk does not do)
//
// It never aims at a cell, so cells that only a face-on measurement can
// decide may stay undecided - the success ratio shows that price. The draw
// uses its own seeded generator (--seed): the same seed gives the same walk.
#pragma once

#include "Strategy.h"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace explore {

class RandomStrategy final : public Strategy {
public:
    explicit RandomStrategy(unsigned seed, int maxIdle = 6) : rng_(seed), seed_(seed), maxIdle_(maxIdle) {}

    const char* id() const override { return "random"; }
    const char* name() const override { return "11_03 Random"; }
    const wchar_t* title() const override { return L"random reachable cell + 360 deg look-around"; }
    const char* rule() const override {
        return "drive to a uniformly random reachable empty cell and look around 360 deg; stop when nothing new is learnt";
    }

    std::optional<Goal> next(const PlanContext& ctx) override {
        const bayes::BayesGrid& g = ctx.grid;
        const int decided = g.count(bayes::State::Empty) + g.count(bayes::State::Occupied);
        if (moves_ > 0) idle_ = decided > lastDecided_ ? 0 : idle_ + 1;
        lastDecided_ = decided;
        open_ = static_cast<int>(openFrontier(ctx).size());
        if (open_ == 0) { stop_.clear(); return std::nullopt; }   // nothing unknown left: a normal finish
        if (stopped_) return std::nullopt;                          // stagnated before: stays stopped
        if (idle_ >= maxIdle_) {
            stopped_ = true;
            stop_ = "random walk: no new cell decided in the last " + std::to_string(maxIdle_) + " look-arounds";
            return std::nullopt;
        }

        // Candidates: every known-empty cell reachable from here.
        const astar::GridMap& m = g.arena();
        const std::vector<char> reach = astar::reachable(ctx.cm, ctx.here);
        candidates_.clear();
        for (int y = 0; y < g.rows(); ++y)
            for (int x = 0; x < g.cols(); ++x) {
                const Cell c{ x, y };
                if (c == ctx.here || !reach[static_cast<std::size_t>(m.index(c))] || !ctx.cm.traversable(c)) continue;
                candidates_.push_back(c);
            }
        Cell goal = ctx.here;   // nowhere to go: look around where it stands
        if (!candidates_.empty()) {
            std::uniform_int_distribution<std::size_t> pick(0, candidates_.size() - 1);
            goal = candidates_[pick(rng_)];
        }
        const Route r = routeAStar(ctx.cm, ctx.here, goal);
        if (!r.found) return std::nullopt;
        ++moves_;
        goal_ = goal;
        return Goal{ Action::LookAround, goal, goal, r.cells,
                     "random goal #" + std::to_string(moves_) + ": " + m.coords.text(goal) + " (1 of " +
                         std::to_string(candidates_.size()) + " reachable cells) - drive there and look around" };
    }

    std::string stopReason() const override { return stop_; }

    void describe(StrategyView& v) const override {
        if (moves_ > 0) v.marks.push_back({ goal_, std::to_wstring(moves_), MarkKind::Order });
        wchar_t b[160];
        swprintf(b, 160, L"seed %u, random goals %d, candidates %d", seed_, moves_, static_cast<int>(candidates_.size()));
        v.info.push_back(b);
        swprintf(b, 160, L"look-arounds without a new cell: %d / %d", idle_, maxIdle_);
        v.info.push_back(b);
        swprintf(b, 160, L"open frontier cells %d (number: goal count)", open_);
        v.info.push_back(b);
    }

private:
    std::mt19937 rng_;
    unsigned seed_;
    int maxIdle_;
    int moves_ = 0, idle_ = 0, lastDecided_ = 0, open_ = 0;
    bool stopped_ = false;
    Cell goal_{};
    std::vector<Cell> candidates_;
    std::string stop_;
};

} // namespace explore
