// Ipc.h - "--ipc": the exploration program runs without its own window and
// streams its state to stdout, one line per item, so the launcher can draw
// the live map in its own window. Commands come back on stdin ("stop").
//
// Cells are internal grid cells (x = column, y = row counted from the TOP
// line of the map file), points are metres in the map frame.
//
//   @ARENA cols rows cell startX startY yUp base heading      once
//   @METHOD <id> <name...>                                    once
//   @REF <map name>                                           once ("-" = none)
//   frame (~10 Hz):
//   @F t phase x y heading driven goals scans bumps readings sonarCm hasTarget action tx ty ax ay verifying
//   @P  p0 p1 ...          P(occupied) of every cell in percent (row by row)
//   @S  eeou...            state of every cell: e empty (P < 0.3), o occupied (P > 0.8), u undecided
//   @FR x y x y ...        frontier cells
//   @PA x y x y ...        current path (cells)
//   @TO x y x y ...        method's polyline (metres), e.g. the TSP tour
//   @MK x y kind label ... method's marks (kind 0 order, 1 candidate, 2 goal; label "-" = none)
//   @BM sx sy angle range echo   last sonar beam
//   @I <text>              method's info line (several)
//   @E                     end of frame
//   @EV <text>             event (as on the console)
//   @DONE finish completed time driven      end of the run
//   @RESULT <text>
//   @SAVED <path of the map txt>
// Every other line is plain console text (errors, the model table ...).
#pragma once

#include "BayesGrid.h"
#include "Console.h"
#include "ExploreRobot.h"
#include "Explorer.h"
#include "Strategy.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

namespace ipc {

// stdin: "stop" -> the run stops like Ctrl+C (robot stopped, map saved).
inline void listenStdin() {
    std::thread([] {
        std::string line;
        while (std::getline(std::cin, line))
            if (line.find("stop") != std::string::npos) app::g_quit = true;   // tolerant: a BOM or CR may surround it
    }).detach();
}

inline std::string oneLine(std::string s) {
    for (char& c : s) if (c == '\n' || c == '\r') c = ' ';
    return s;
}

inline void header(const astar::GridMap& arena, astar::Cell start, double headingDeg, const explore::Strategy& st,
                   const std::string& refName) {
    std::cout << "@ARENA " << arena.cols() << ' ' << arena.rows() << ' ' << arena.cellSize() << ' ' << start.x << ' ' << start.y << ' '
              << (arena.coords.yUp ? 1 : 0) << ' ' << arena.coords.base << ' ' << headingDeg << "\n"
              << "@METHOD " << st.id() << ' ' << st.name() << "\n"
              << "@REF " << refName << "\n"
              << std::flush;
}

inline void event(double now, const std::string& text) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.0f", now);
    std::cout << "@EV [" << b << " s] " << oneLine(text) << "\n" << std::flush;
}

inline std::string narrow(const std::wstring& w) {
    std::string s;
    for (wchar_t c : w) s += c < 128 ? static_cast<char>(c) : '?';
    return s;
}

inline void frame(const bayes::BayesGrid& g, const explore::Explorer& e, const explore::Strategy& st, const explore::Snapshot& s,
                  double now, double driven) {
    char b[256];
    const auto& be = s.beams[1];
    const int sonar = be.valid && be.echo ? static_cast<int>(be.range * 100.0 + 0.5) : -1;
    std::snprintf(b, sizeof(b), "@F %.2f %s %.3f %.3f %.4f %.2f %d %d %d %ld %d %d %d %d %d %d %d %d\n", now,
                  explore::phaseName(e.phase()), s.pose.x, s.pose.y, s.pose.heading, driven, e.goals(), e.scans(), e.bumps(),
                  e.readings(), sonar, e.hasTarget() ? 1 : 0, e.action() == explore::Action::LookAround ? 1 : 0, e.target().x,
                  e.target().y, e.goal().x, e.goal().y, e.verifying() ? 1 : 0);
    std::string out = b;
    out += "@P";
    for (int y = 0; y < g.rows(); ++y)
        for (int x = 0; x < g.cols(); ++x) out += ' ' + std::to_string(static_cast<int>(g.p({ x, y }) * 100.0 + 0.5));
    out += "\n@S ";
    for (int y = 0; y < g.rows(); ++y)
        for (int x = 0; x < g.cols(); ++x) {
            const auto cs = g.state({ x, y });
            out += cs == bayes::State::Empty ? 'e' : (cs == bayes::State::Occupied ? 'o' : 'u');
        }
    out += "\n@FR";
    for (const auto& c : e.frontier())
        if (g.state(c) == bayes::State::Undecided) out += ' ' + std::to_string(c.x) + ' ' + std::to_string(c.y);
    out += "\n@PA";
    for (const auto& c : e.path()) out += ' ' + std::to_string(c.x) + ' ' + std::to_string(c.y);
    explore::StrategyView v;
    st.describe(v);
    out += "\n@TO";
    for (const auto& p : v.tour) {
        std::snprintf(b, sizeof(b), " %.3f %.3f", p.x, p.y);
        out += b;
    }
    out += "\n@MK";
    for (const auto& m : v.marks) {
        std::string label = narrow(m.label);
        if (label.empty()) label = "-";
        out += ' ' + std::to_string(m.cell.x) + ' ' + std::to_string(m.cell.y) + ' ' + std::to_string(static_cast<int>(m.kind)) + ' ' + label;
    }
    const auto& bm = e.beam(1);
    if (bm.set) {
        std::snprintf(b, sizeof(b), "\n@BM %.3f %.3f %.4f %.3f %d", bm.sensor.x, bm.sensor.y, bm.angle, bm.range, bm.echo ? 1 : 0);
        out += b;
    }
    for (const auto& line : v.info) out += "\n@I " + oneLine(narrow(line));
    out += "\n@E\n";
    std::cout << out << std::flush;
}

inline void done(explore::Finish f, bool completed, double now, double driven, const std::string& result, const std::string& saved) {
    std::cout << "@DONE " << explore::finishId(f) << ' ' << (completed ? 1 : 0) << ' ' << now << ' ' << driven << "\n"
              << "@RESULT " << oneLine(result) << "\n"
              << "@SAVED " << saved << "\n"
              << std::flush;
}

} // namespace ipc
