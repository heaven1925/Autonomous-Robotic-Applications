// Replay.h - a finished run for the "Geçmiş" tab.
//
//   <map>.replay     the run's state lines (common/Ipc.h), saved by the launcher
//                    next to the map -> every frame can be shown again
//   <map>_prob.txt   written by every Explore_*.exe -> only the final grid
//                    (runs started from the command line / the benchmark)
#pragma once

#include "LiveRun.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace replay {

struct Frame {
    live::State state;         // without events (they are in Recording::events)
    std::size_t events = 0;    // number of events up to this frame
};

struct Recording {
    bool loaded = false, animated = false;
    std::vector<Frame> frames;
    std::vector<std::string> events;
    double duration = 0.0;

    // Frame shown at time t (the last one that started at or before t).
    std::size_t indexAt(double t) const {
        std::size_t lo = 0, hi = frames.size();
        while (lo + 1 < hi) {
            const std::size_t mid = (lo + hi) / 2;
            (frames[mid].state.t <= t ? lo : hi) = mid;
        }
        return lo;
    }

    // The state to draw for frame i, with the events up to it (last 200).
    live::State view(std::size_t i) const {
        if (frames.empty()) return {};
        i = std::min(i, frames.size() - 1);
        live::State s = frames[i].state;
        const std::size_t n = frames[i].events, from = n > 200 ? n - 200 : 0;
        s.events.assign(events.begin() + static_cast<std::ptrdiff_t>(from), events.begin() + static_cast<std::ptrdiff_t>(n));
        const bool last = i + 1 == frames.size();
        if (!last) { s.done = false; s.completed = false; }
        return s;
    }
};

inline Recording load(const std::filesystem::path& file) {
    Recording r;
    std::ifstream in(file, std::ios::binary);
    if (!in) return r;
    live::Parser parser;
    parser.maxEvents = 0;
    live::State cur;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!parser.feed(line, cur)) continue;
        Frame f;
        f.events = cur.events.size();
        f.state = cur;
        f.state.events.clear();
        f.state.console.clear();
        r.frames.push_back(std::move(f));
    }
    if (r.frames.empty()) return r;
    r.events.assign(cur.events.begin(), cur.events.end());
    // The end of the run arrives after the last frame.
    live::State& last = r.frames.back().state;
    last.done = cur.done;
    last.completed = cur.completed;
    last.finish = cur.finish;
    last.result = cur.result;
    last.saved = cur.saved;
    last.endTime = cur.endTime;
    last.endDriven = cur.endDriven;
    r.frames.back().events = r.events.size();
    for (Frame& f : r.frames) { f.state.header = cur.header; f.state.cell = cur.cell; f.state.cols = cur.cols; f.state.rows = cur.rows; }
    r.duration = cur.done ? cur.endTime : last.t;
    r.loaded = true;
    r.animated = r.frames.size() > 1;
    return r;
}

// Final grid only, from <map>_prob.txt ("0.99#" occupied, "0.01 " empty, "0.50?" undecided).
inline Recording fromProbFile(const std::filesystem::path& file, int cols, int rows) {
    Recording r;
    std::ifstream in(file);
    if (!in) return r;
    live::State s;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream row(line);
        std::vector<std::string> tok;
        std::string t;
        while (row >> t) tok.push_back(t);
        // "0.99#" is one token, "0.01" + nothing; an undecided / occupied tag sticks to the number.
        if (static_cast<int>(tok.size()) != cols + 1) continue;   // the column header line has `cols` tokens
        for (int x = 0; x < cols; ++x) {
            const std::string& v = tok[static_cast<std::size_t>(x + 1)];
            const double p = std::atof(v.c_str());
            s.p.push_back(static_cast<int>(p * 100.0 + 0.5));
            s.states += v.back() == '#' ? 'o' : (v.back() == '?' ? 'u' : 'e');
        }
    }
    if (static_cast<int>(s.p.size()) != cols * rows) return r;
    s.frame = true;
    s.done = true;
    s.staticMap = true;
    s.phase = "DONE";
    Frame f;
    f.state = s;
    r.frames.push_back(f);
    r.loaded = true;
    return r;
}

} // namespace replay
