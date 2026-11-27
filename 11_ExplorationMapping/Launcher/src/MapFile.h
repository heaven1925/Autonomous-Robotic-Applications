// MapFile.h - obstacle maps for the map editor ("Haritalar" tab): write a map
// in the format of obstacleMapReferences/mapN/mapN.txt and check it.
#pragma once

#include "AStarPlanner.h"

#include <filesystem>
#include <fstream>
#include <queue>
#include <string>
#include <vector>

namespace mapfile {

struct Check {
    int inner = 0;          // obstacles inside the border
    int freeCells = 0;
    int unreachable = 0;    // free cells the robot cannot drive to from the start
    int observable = 0;     // free, or occupied with a free 4-neighbour (as in Metrics.h)
};

inline Check check(const astar::GridMap& m) {
    Check c;
    const int cols = m.cols(), rows = m.rows();
    std::vector<char> seen(static_cast<std::size_t>(cols * rows), 0);
    const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    if (m.start && !m.occupied(*m.start)) {
        std::queue<astar::Cell> q;
        q.push(*m.start);
        seen[static_cast<std::size_t>(m.index(*m.start))] = 1;
        while (!q.empty()) {
            const astar::Cell k = q.front();
            q.pop();
            for (int i = 0; i < 4; ++i) {
                const astar::Cell n{ k.x + dx[i], k.y + dy[i] };
                if (!m.inside(n) || m.occupied(n) || seen[static_cast<std::size_t>(m.index(n))]) continue;
                seen[static_cast<std::size_t>(m.index(n))] = 1;
                q.push(n);
            }
        }
    }
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const astar::Cell k{ x, y };
            const bool border = x == 0 || y == 0 || x == cols - 1 || y == rows - 1;
            if (m.occupied(k)) {
                if (!border) ++c.inner;
                for (int i = 0; i < 4; ++i)
                    if (m.inside({ x + dx[i], y + dy[i] }) && !m.occupied({ x + dx[i], y + dy[i] })) { ++c.observable; break; }
            } else {
                ++c.freeCells;
                ++c.observable;
                if (!seen[static_cast<std::size_t>(m.index(k))]) ++c.unreachable;
            }
        }
    return c;
}

// True if the robot cannot drive to this free cell from the start.
inline std::vector<char> unreachableMask(const astar::GridMap& m) {
    const int cols = m.cols(), rows = m.rows();
    std::vector<char> seen(static_cast<std::size_t>(cols * rows), 0), out(static_cast<std::size_t>(cols * rows), 0);
    const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    if (m.start && !m.occupied(*m.start)) {
        std::queue<astar::Cell> q;
        q.push(*m.start);
        seen[static_cast<std::size_t>(m.index(*m.start))] = 1;
        while (!q.empty()) {
            const astar::Cell k = q.front();
            q.pop();
            for (int i = 0; i < 4; ++i) {
                const astar::Cell n{ k.x + dx[i], k.y + dy[i] };
                if (!m.inside(n) || m.occupied(n) || seen[static_cast<std::size_t>(m.index(n))]) continue;
                seen[static_cast<std::size_t>(m.index(n))] = 1;
                q.push(n);
            }
        }
    }
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = !m.occupied(m.cellOf(static_cast<int>(i))) && !seen[i];
    return out;
}

// obstacleMapReferences/<name>/<name>.txt
inline bool save(const std::filesystem::path& mapsDir, const std::string& name, const astar::GridMap& m, std::string& error) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(mapsDir / name, ec);
    const fs::path file = mapsDir / name / (name + ".txt");
    std::ofstream out(file);
    if (!out) { error = "cannot write " + file.string(); return false; }
    std::string inner;
    for (int y = 1; y + 1 < m.rows(); ++y)
        for (int x = 1; x + 1 < m.cols(); ++x)
            if (m.occupied({ x, y })) inner += " " + m.coords.text({ x, y });
    char cell[32];
    std::snprintf(cell, sizeof(cell), "%.2f", m.cellSize());
    out << "# Engel haritasi " << name << " (ExploreLauncher harita duzenleyicisi). Alan " << m.cols() << " sutun x " << m.rows()
        << " satir, " << static_cast<int>(m.cellSize() * 100.0 + 0.5) << " cm hucreler.\n"
        << "# Koordinatlar (x, y), 0'dan: x saga, y YUKARI (alt satir y = 0).\n"
        << "# 1 = engel, 0 = bos, * = baslangic, robot +y'ye bakar.\n"
        << "# Ic engel:" << (inner.empty() ? " yok" : inner) << "\n"
        << "# Simulatorde gercek dunya; gercek robotta cikan haritanin karsilastirildigi referans.\n"
        << "name " << name << "\n"
        << "cell " << cell << "\n"
        << "coords " << m.coords.keyword() << "\n"
        << "border " << (m.walledBorder ? "wall" : "open") << "\n"
        << "heading " << static_cast<int>(m.headingDeg.value_or(90.0) + 0.5) << "\n";
    for (int y = 0; y < m.rows(); ++y) {
        for (int x = 0; x < m.cols(); ++x) {
            const bool start = m.start && m.start->x == x && m.start->y == y;
            out << (x ? " " : "") << (start ? "*" : (m.occupied({ x, y }) ? "1" : "0"));
        }
        out << "\n";
    }
    if (!out) { error = "write error: " + file.string(); return false; }
    return true;
}

} // namespace mapfile
