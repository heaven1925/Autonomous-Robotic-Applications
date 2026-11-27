// Metrics.h - how good is a map, and the results file every run appends to.
//
// The three comparison criteria (all four methods, every obstacle map):
//   1. Speed          time_s: time until the method finished the map by itself
//                     (simulated seconds in --sim, wall-clock seconds on the robot)
//   2. Success ratio  correct cells / observable cells of the reference map
//   3. Obstacle maps  on how many different maps (map1..map4) the run finished
//                     by itself with a success ratio >= the threshold (launcher)
//
// Observable cell: a free cell, or an occupied cell with a free 4-neighbour,
// i.e. a face the sonar can see from free space. A wall cell surrounded by
// wall cells (the four outer corners, a 2-cell thick wall) can never be seen
// by any method - counting it would only lower every method by the same
// constant. accuracy_all (all cells, undecided = wrong) is written as well.
//
// results.csv: one line per run, header written when the file is new.
#pragma once

#include "AStarPlanner.h"
#include "BayesGrid.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace metrics {

using astar::Cell;

inline bool observable(const astar::GridMap& truth, Cell c) {
    if (!truth.occupied(c)) return truth.inside(c);
    const Cell n[4] = { { c.x + 1, c.y }, { c.x - 1, c.y }, { c.x, c.y + 1 }, { c.x, c.y - 1 } };
    for (const Cell& k : n)
        if (truth.inside(k) && !truth.occupied(k)) return true;
    return false;
}

struct Score {
    int cells = 0, correct = 0, wrong = 0, undecided = 0;          // all cells
    int observable = 0, correctObs = 0, wrongObs = 0, undecidedObs = 0;
    int freeTotal = 0, freeFound = 0;                               // free cells decided empty
    int occTotal = 0, occFound = 0;                                 // observable occupied cells decided occupied
    std::vector<Cell> wrongCells;

    double successRatio() const { return observable ? static_cast<double>(correctObs) / observable : 0.0; }
    double accuracyAll() const { return cells ? static_cast<double>(correct) / cells : 0.0; }
};

inline Score score(const bayes::BayesGrid& g, const astar::GridMap& truth) {
    Score s;
    for (int y = 0; y < g.rows(); ++y)
        for (int x = 0; x < g.cols(); ++x) {
            const Cell c{ x, y };
            const bool occ = truth.occupied(c), obs = observable(truth, c);
            const auto st = g.state(c);
            ++s.cells;
            if (obs) ++s.observable;
            if (occ) { if (obs) ++s.occTotal; } else ++s.freeTotal;
            if (st == bayes::State::Undecided) {
                ++s.undecided;
                if (obs) ++s.undecidedObs;
                continue;
            }
            const bool right = (st == bayes::State::Occupied) == occ;
            if (right) {
                ++s.correct;
                if (obs) ++s.correctObs;
                if (occ && obs) ++s.occFound;
                if (!occ) ++s.freeFound;
            } else {
                ++s.wrong;
                if (obs) ++s.wrongObs;
                s.wrongCells.push_back(c);
            }
        }
    return s;
}

// One run = one line of results.csv.
struct RunRecord {
    std::string timestamp;       // 2026-11-24 10:15:02
    std::string algorithm;       // frontier / bfs / random / tsp
    std::string map;             // map1 ... (name of the reference map), "-" if none
    std::string mode;            // sim / real
    unsigned seed = 0;
    bool noise = true, verify = true;
    std::string finish;          // finishId()
    bool completed = false;      // finished by itself
    double timeS = 0.0, drivenM = 0.0;
    bool scored = false;         // a reference map was given
    Score s;
    int bumps = 0, scans = 0, plans = 0, goals = 0, faceOn = 0;
    long readings = 0;
    std::string output;          // the saved map txt
};

inline const char* CsvHeader =
    "timestamp,algorithm,map,mode,seed,noise,verify,finish,completed,time_s,driven_m,"
    "cells,observable,correct_obs,wrong_obs,undecided_obs,success_ratio,correct,wrong,undecided,accuracy_all,"
    "free_found,free_total,occ_found,occ_total,bumps,scans,plans,goals,face_on,readings,output";

inline bool appendCsv(const std::filesystem::path& path, const RunRecord& r) {
    std::error_code ec;
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), ec);
    const bool fresh = !std::filesystem::exists(path, ec) || std::filesystem::file_size(path, ec) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out) return false;
    if (fresh) out << CsvHeader << "\n";
    const auto num = [](double v, int prec) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(prec) << v;
        return o.str();
    };
    const Score& s = r.s;
    const auto opt = [&](double v, int prec) { return r.scored ? num(v, prec) : std::string(); };
    const auto cnt = [&](int v) { return r.scored ? std::to_string(v) : std::string(); };
    std::string file = r.output;
    for (char& ch : file) if (ch == ',') ch = ';';
    out << r.timestamp << ',' << r.algorithm << ',' << r.map << ',' << r.mode << ',' << r.seed << ','
        << (r.noise ? 1 : 0) << ',' << (r.verify ? 1 : 0) << ',' << r.finish << ',' << (r.completed ? 1 : 0) << ','
        << num(r.timeS, 1) << ',' << num(r.drivenM, 2) << ','
        << s.cells << ',' << cnt(s.observable) << ',' << cnt(s.correctObs) << ',' << cnt(s.wrongObs) << ','
        << cnt(s.undecidedObs) << ',' << opt(s.successRatio(), 4) << ',' << cnt(s.correct) << ',' << cnt(s.wrong) << ','
        << s.undecided << ',' << opt(s.accuracyAll(), 4) << ','
        << cnt(s.freeFound) << ',' << cnt(s.freeTotal) << ',' << cnt(s.occFound) << ',' << cnt(s.occTotal) << ','
        << r.bumps << ',' << r.scans << ',' << r.plans << ',' << r.goals << ',' << r.faceOn << ',' << r.readings << ','
        << file << "\n";
    return static_cast<bool>(out);
}

} // namespace metrics
