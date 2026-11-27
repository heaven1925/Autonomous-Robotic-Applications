// TspSolver.h - open travelling salesman path: start at node "robot", visit
// every node once, do NOT return (the robot stays where it ends).
//
//   n <= ExactLimit : Held-Karp dynamic programming, exact optimum
//                     dp[S][j] = shortest path from the robot through the set S ending at j
//                     dp[{j}][j] = d0[j];  dp[S][j] = min_{i in S-j} dp[S-j][i] + d[i][j]
//                     O(2^n * n^2): 13 nodes ~ 1.4 M steps, < 10 ms
//   larger          : nearest neighbour construction + 2-opt improvement
//                     (reverse a segment while the path gets shorter)
//
// d0[j]  : cost robot -> node j,   d[i][j] : cost node i -> node j (symmetric not required)
#pragma once

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace tsp {

constexpr int ExactLimit = 13;

struct Tour {
    std::vector<int> order;   // node indices in visiting order
    double cost = 0.0;
    bool exact = false;
};

inline double pathCost(const std::vector<double>& d0, const std::vector<std::vector<double>>& d, const std::vector<int>& order) {
    if (order.empty()) return 0.0;
    double c = d0[static_cast<std::size_t>(order[0])];
    for (std::size_t k = 1; k < order.size(); ++k) c += d[static_cast<std::size_t>(order[k - 1])][static_cast<std::size_t>(order[k])];
    return c;
}

inline Tour heldKarp(const std::vector<double>& d0, const std::vector<std::vector<double>>& d) {
    const int n = static_cast<int>(d0.size());
    constexpr double Inf = std::numeric_limits<double>::infinity();
    const std::size_t full = (std::size_t{ 1 } << n);
    std::vector<double> dp(full * static_cast<std::size_t>(n), Inf);
    std::vector<int> prev(full * static_cast<std::size_t>(n), -1);
    const auto at = [n](std::size_t S, int j) { return S * static_cast<std::size_t>(n) + static_cast<std::size_t>(j); };
    for (int j = 0; j < n; ++j) dp[at(std::size_t{ 1 } << j, j)] = d0[static_cast<std::size_t>(j)];
    for (std::size_t S = 1; S < full; ++S) {
        for (int j = 0; j < n; ++j) {
            if (!(S & (std::size_t{ 1 } << j))) continue;
            const double here = dp[at(S, j)];
            if (here == Inf) continue;
            for (int k = 0; k < n; ++k) {
                if (S & (std::size_t{ 1 } << k)) continue;
                const std::size_t T = S | (std::size_t{ 1 } << k);
                const double c = here + d[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)];
                if (c < dp[at(T, k)] - 1e-12) { dp[at(T, k)] = c; prev[at(T, k)] = j; }
            }
        }
    }
    Tour t;
    t.exact = true;
    int last = 0;
    double best = Inf;
    for (int j = 0; j < n; ++j)
        if (dp[at(full - 1, j)] < best - 1e-12) { best = dp[at(full - 1, j)]; last = j; }
    t.cost = best;
    std::size_t S = full - 1;
    for (int j = last; j != -1;) {
        t.order.push_back(j);
        const int p = prev[at(S, j)];
        S &= ~(std::size_t{ 1 } << j);
        j = p;
    }
    std::reverse(t.order.begin(), t.order.end());
    return t;
}

inline Tour nearestNeighbour2Opt(const std::vector<double>& d0, const std::vector<std::vector<double>>& d) {
    const int n = static_cast<int>(d0.size());
    Tour t;
    std::vector<char> used(static_cast<std::size_t>(n), 0);
    int cur = -1;
    for (int step = 0; step < n; ++step) {
        int best = -1;
        double bestC = std::numeric_limits<double>::infinity();
        for (int j = 0; j < n; ++j) {
            if (used[static_cast<std::size_t>(j)]) continue;
            const double c = cur < 0 ? d0[static_cast<std::size_t>(j)] : d[static_cast<std::size_t>(cur)][static_cast<std::size_t>(j)];
            if (c < bestC - 1e-12) { bestC = c; best = j; }
        }
        used[static_cast<std::size_t>(best)] = 1;
        t.order.push_back(best);
        cur = best;
    }
    // 2-opt: reverse order[i..k] while that shortens the open path.
    bool improved = true;
    double cost = pathCost(d0, d, t.order);
    while (improved) {
        improved = false;
        for (int i = 0; i < n - 1; ++i)
            for (int k = i + 1; k < n; ++k) {
                std::vector<int> cand = t.order;
                std::reverse(cand.begin() + i, cand.begin() + k + 1);
                const double c = pathCost(d0, d, cand);
                if (c < cost - 1e-9) { t.order = std::move(cand); cost = c; improved = true; }
            }
    }
    t.cost = cost;
    return t;
}

inline Tour solve(const std::vector<double>& d0, const std::vector<std::vector<double>>& d) {
    if (d0.empty()) return {};
    if (static_cast<int>(d0.size()) <= ExactLimit) return heldKarp(d0, d);
    return nearestNeighbour2Opt(d0, d);
}

} // namespace tsp
