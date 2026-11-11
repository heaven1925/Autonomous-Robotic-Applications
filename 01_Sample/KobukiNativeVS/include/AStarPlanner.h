// AStarPlanner.h - grid path planning for a round robot, hardware independent.
//
//   GridMap  - the map from a text file of 0 / 1 cells (1 = obstacle) plus
//              optional "cell", "start", "goal", "heading" lines.
//   CostMap  - configuration space: every free cell whose CENTRE is closer
//              than the robot radius to an obstacle is "inflated" (forbidden
//              for the robot centre), so a path through free cells keeps the
//              whole robot body off the obstacles.
//   search() - A* on the cell grid (8- or 4-connected, no corner cutting,
//              octile / Manhattan heuristic). hWeight = 0 gives Dijkstra.
//   corners(), shortcut(), minClearance(), nearestSafeCell() - path post
//              processing and checks.
//
// Coordinates
//   Cell {x, y}: x = column (0 = left), y = row (0 = FIRST line of the file).
//   Point {x, y}: metres in the map frame - origin at the bottom-left corner
//   of the map, +x to the right, +y up (towards the first line), heading 0 =
//   +x, counter-clockwise positive. The centre of cell (c, r) is
//   ((c + 0.5) * cell, (rows - r - 0.5) * cell).
//
// Map file example (10 x 4 cells of 20 cm, start top-left, goal right):
//   # comment                (also: % comment, // comment)
//   cell 0.20                cell size in metres (default 0.20)
//   start 0 0                column row   (or put an S in the grid)
//   goal  9 2                column row   (or put a G in the grid)
//   heading 0                robot's initial heading, degrees (0 = right, 90 = up)
//   0 0 0 0 0 0 0 0 0 0
//   0 0 0 1 1 0 0 0 0 0      cells separated by spaces, commas - or nothing
//   0 0 0 1 1 0 0 0 0 0
//   0 0 0 0 0 0 0 0 0 0
#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

namespace astar {

constexpr double Pi = 3.14159265358979323846;
constexpr double Sqrt2 = 1.41421356237309504880;

struct Cell {
    int x = 0, y = 0;  // column, row
    bool operator==(const Cell&) const = default;
};

struct Point {
    double x = 0.0, y = 0.0;  // metres, map frame
};

inline double distance(Point a, Point b) { return std::hypot(b.x - a.x, b.y - a.y); }

// ------------------------------------------------------------------ map --
class GridMap {
public:
    GridMap() = default;
    GridMap(int cols, int rows, double cellSize)
        : cols_(cols), rows_(rows), cell_(cellSize), occ_(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0) {}

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    double cellSize() const { return cell_; }
    double width() const { return cols_ * cell_; }
    double height() const { return rows_ * cell_; }
    bool empty() const { return cols_ <= 0 || rows_ <= 0; }

    bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < cols_ && c.y < rows_; }
    int index(Cell c) const { return c.y * cols_ + c.x; }
    Cell cellOf(int index) const { return { index % cols_, index / cols_ }; }

    bool occupied(Cell c) const { return inside(c) && occ_[static_cast<std::size_t>(index(c))] != 0; }
    void setOccupied(Cell c, bool on) {
        if (inside(c)) occ_[static_cast<std::size_t>(index(c))] = on ? 1 : 0;
    }
    int obstacleCount() const { return static_cast<int>(std::count(occ_.begin(), occ_.end(), std::uint8_t{1})); }

    Point centre(Cell c) const { return { (c.x + 0.5) * cell_, (rows_ - c.y - 0.5) * cell_ }; }
    Cell cellAt(Point p) const {
        return { static_cast<int>(std::floor(p.x / cell_)), rows_ - 1 - static_cast<int>(std::floor(p.y / cell_)) };
    }

    // Square of the cell in metres: [x0, x1] x [y0, y1].
    void bounds(Cell c, double& x0, double& y0, double& x1, double& y1) const {
        x0 = c.x * cell_;
        x1 = x0 + cell_;
        y0 = (rows_ - c.y - 1) * cell_;
        y1 = y0 + cell_;
    }

    // Values read from the file (all optional).
    std::optional<Cell> start, goal;
    std::optional<double> headingDeg;
    std::string name;

private:
    int cols_ = 0, rows_ = 0;
    double cell_ = 0.20;
    std::vector<std::uint8_t> occ_;
};

namespace detail {

inline std::string lower(std::string s) {
    for (char& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
}

inline std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// "start (3, 4)" / "start: 3 4" / "start=3,4" -> numbers after the keyword.
inline std::vector<double> numbers(const std::string& text, bool& ok) {
    std::string t = text;
    for (char& ch : t)
        if (ch == ',' || ch == ';' || ch == ':' || ch == '=' || ch == '(' || ch == ')' || ch == '[' || ch == ']') ch = ' ';
    std::istringstream in(t);
    std::vector<double> v;
    std::string word;
    ok = true;
    while (in >> word) {
        try {
            std::size_t used = 0;
            v.push_back(std::stod(word, &used));
            if (used != word.size()) ok = false;
        } catch (...) {
            ok = false;
        }
    }
    return v;
}

} // namespace detail

// Reads a map; false + a message naming the line on any error.
inline bool parseMap(std::istream& in, GridMap& map, std::string& error) {
    map = GridMap{};
    std::vector<std::string> rows;
    std::optional<Cell> gridStart, gridGoal;
    std::optional<double> cellSize;
    std::string line;
    int lineNo = 0;

    const auto fail = [&](const std::string& msg) {
        error = "line " + std::to_string(lineNo) + ": " + msg;
        return false;
    };

    while (std::getline(in, line)) {
        ++lineNo;
        if (lineNo == 1 && line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xEF) line.erase(0, 3);  // UTF-8 BOM
        for (const char* mark : { "#", "%", "//" }) {
            const auto at = line.find(mark);
            if (at != std::string::npos) line.erase(at);
        }
        line = detail::trim(line);
        if (line.empty()) continue;

        // Keyword line?
        if (std::isalpha(static_cast<unsigned char>(line[0])) && line.size() > 1 &&
            std::isalpha(static_cast<unsigned char>(line[1]))) {
            std::size_t end = 0;
            while (end < line.size() && (std::isalpha(static_cast<unsigned char>(line[end])) || line[end] == '_')) ++end;
            const std::string key = detail::lower(line.substr(0, end));
            if (key == "name") {
                map.name = detail::trim(line.substr(end));
                continue;
            }
            bool ok = true;
            const std::vector<double> v = detail::numbers(line.substr(end), ok);
            if (!ok) return fail("cannot read the numbers after '" + key + "'");
            if (key == "cell" || key == "cell_size" || key == "cellsize" || key == "resolution") {
                if (v.size() != 1 || !(v[0] > 0.0) || v[0] > 10.0) return fail("'cell' needs one size in metres, e.g. cell 0.20");
                cellSize = v[0];
            } else if (key == "start" || key == "goal") {
                if (v.size() != 2 || v[0] != std::floor(v[0]) || v[1] != std::floor(v[1]))
                    return fail("'" + key + "' needs a column and a row (whole numbers), e.g. " + key + " 0 0");
                const Cell c{ static_cast<int>(v[0]), static_cast<int>(v[1]) };
                (key == "start" ? map.start : map.goal) = c;
            } else if (key == "heading") {
                if (v.size() != 1) return fail("'heading' needs one angle in degrees (0 = right, 90 = up)");
                map.headingDeg = v[0];
            } else {
                return fail("unknown keyword '" + key + "' (known: cell, start, goal, heading, name)");
            }
            continue;
        }

        // Grid row: 0 / 1 (S = start, G = goal), separated by spaces / commas or not
        // at all; braces are ignored, so "{0,1,0,0}," (C array style) works too.
        std::string row;
        for (char ch : line) {
            if (ch == ' ' || ch == '\t' || ch == ',' || ch == ';' || ch == '{' || ch == '}') continue;
            const char up = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
            if (up != '0' && up != '1' && up != 'S' && up != 'G')
                return fail(std::string("unexpected character '") + ch + "' (cells must be 0 = free or 1 = obstacle)");
            if (up == 'S' || up == 'G') {
                auto& slot = up == 'S' ? gridStart : gridGoal;
                if (slot) return fail(std::string("more than one ") + up + " in the grid");
                slot = Cell{ static_cast<int>(row.size()), static_cast<int>(rows.size()) };
            }
            row.push_back(up);
        }
        if (!rows.empty() && row.size() != rows.front().size())
            return fail("this row has " + std::to_string(row.size()) + " cells, the first row has " +
                        std::to_string(rows.front().size()));
        rows.push_back(row);
    }
    if (rows.empty()) { error = "no grid rows found (expected lines of 0 / 1)"; return false; }

    GridMap m(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()), cellSize.value_or(0.20));
    for (int y = 0; y < m.rows(); ++y)
        for (int x = 0; x < m.cols(); ++x)
            m.setOccupied({ x, y }, rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] == '1');
    m.start = map.start ? map.start : gridStart;   // keyword lines win over S / G letters
    m.goal = map.goal ? map.goal : gridGoal;
    m.headingDeg = map.headingDeg;
    m.name = map.name;
    map = std::move(m);
    return true;
}

inline bool loadMap(const std::string& path, GridMap& map, std::string& error) {
    std::ifstream in(path);
    if (!in) { error = "cannot open " + path; return false; }
    if (!parseMap(in, map, error)) { error = path + ", " + error; return false; }
    return true;
}

// ------------------------------------------------------------- cost map --
enum class CellState : std::uint8_t { Free = 0, Inflated = 1, Obstacle = 2 };

class CostMap {
public:
    // radius: robot radius (+ any safety margin) in metres.
    // walledBorder: treat the map border as a wall too (default: the map is a
    // window onto a bigger free floor - the robot centre just stays inside).
    CostMap(GridMap map, double radius, bool walledBorder = false)
        : map_(std::move(map)), radius_(radius), walled_(walledBorder) {
        rebuild();
    }

    const GridMap& map() const { return map_; }
    double radius() const { return radius_; }
    bool walledBorder() const { return walled_; }

    CellState state(Cell c) const {
        if (!map_.inside(c)) return CellState::Obstacle;
        return state_[static_cast<std::size_t>(map_.index(c))];
    }
    bool obstacle(Cell c) const { return state(c) == CellState::Obstacle; }
    bool inflated(Cell c) const { return state(c) == CellState::Inflated; }
    bool traversable(Cell c) const { return state(c) == CellState::Free; }
    int count(CellState s) const { return static_cast<int>(std::count(state_.begin(), state_.end(), s)); }

    // An obstacle found while driving (bumper): add it and recompute.
    void addObstacle(Cell c) {
        if (!map_.inside(c)) return;
        map_.setOccupied(c, true);
        rebuild();
    }

    // Distance (m) from p to the nearest obstacle surface (or the border if
    // walled), or `limit` if nothing is closer than that.
    double clearance(Point p, double limit) const {
        double best = limit;
        if (walled_) best = std::min({ best, p.x, map_.width() - p.x, p.y, map_.height() - p.y });
        const double C = map_.cellSize();
        const int w = static_cast<int>(std::ceil(limit / C)) + 1;
        const Cell at = map_.cellAt(p);
        for (int y = at.y - w; y <= at.y + w; ++y) {
            for (int x = at.x - w; x <= at.x + w; ++x) {
                if (!map_.occupied({ x, y })) continue;
                double x0, y0, x1, y1;
                map_.bounds({ x, y }, x0, y0, x1, y1);
                const double dx = std::max({ x0 - p.x, 0.0, p.x - x1 });
                const double dy = std::max({ y0 - p.y, 0.0, p.y - y1 });
                best = std::min(best, std::hypot(dx, dy));
            }
        }
        return best;
    }

    // True if the robot centre can move straight from a to b keeping at
    // least `need` metres from every obstacle (checked every 1 cm).
    bool segmentClear(Point a, Point b, double need) const {
        const double len = distance(a, b);
        const int steps = std::max(1, static_cast<int>(std::ceil(len / 0.01)));
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            const Point p{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
            if (!inMap(p) || clearance(p, need + 0.01) < need - 1e-9) return false;
        }
        return true;
    }

    bool inMap(Point p) const { return p.x >= 0.0 && p.y >= 0.0 && p.x <= map_.width() && p.y <= map_.height(); }

private:
    void rebuild() {
        state_.assign(static_cast<std::size_t>(map_.cols()) * static_cast<std::size_t>(map_.rows()), CellState::Free);
        for (int y = 0; y < map_.rows(); ++y) {
            for (int x = 0; x < map_.cols(); ++x) {
                const Cell c{ x, y };
                CellState& s = state_[static_cast<std::size_t>(map_.index(c))];
                if (map_.occupied(c)) s = CellState::Obstacle;
                else if (clearance(map_.centre(c), radius_ + 0.01) < radius_ - 1e-9) s = CellState::Inflated;
            }
        }
    }

    GridMap map_;
    double radius_;
    bool walled_;
    std::vector<CellState> state_;
};

// ------------------------------------------------------------------- A* --
enum class Connectivity { Four = 4, Eight = 8 };

struct SearchResult {
    bool found = false;
    std::vector<Cell> path;          // start ... goal (cell centres are the waypoints)
    double cost = 0.0;               // path length in metres
    int expanded = 0;                // cells taken off the open list (closed set size)
    int generated = 0;               // entries pushed onto the open list
    std::vector<int> order;          // per cell: expansion order, -1 = never expanded
    double milliseconds = 0.0;
};

// Neighbour offsets: 4 straight moves first, then the diagonals.
inline constexpr int NeighbourDx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
inline constexpr int NeighbourDy[8] = { 0, 0, -1, 1, -1, 1, -1, 1 };

// Admissible, consistent estimate of the remaining path length (metres).
inline double heuristic(Cell a, Cell b, Connectivity conn, double cellSize) {
    const double dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    if (conn == Connectivity::Four) return (dx + dy) * cellSize;                             // Manhattan
    return (std::max(dx, dy) + (Sqrt2 - 1.0) * std::min(dx, dy)) * cellSize;               // octile
}

// A move from `from` in direction k is allowed if the target is free and, for
// a diagonal, both cells beside it are free too (the robot must not cut a
// corner of an inflated / obstacle cell). leaving = the robot starts inside
// the inflated zone (it is already there): then only real obstacles beside a
// diagonal step count, so it can always get out the way it got in.
inline bool moveAllowed(const CostMap& cm, Cell from, int k, bool leaving = false) {
    const Cell to{ from.x + NeighbourDx[k], from.y + NeighbourDy[k] };
    if (!cm.traversable(to)) return false;
    if (k >= 4) {
        const Cell side1{ from.x + NeighbourDx[k], from.y }, side2{ from.x, from.y + NeighbourDy[k] };
        if (leaving) return !cm.obstacle(side1) && !cm.obstacle(side2);
        if (!cm.traversable(side1) || !cm.traversable(side2)) return false;
    }
    return true;
}

// A*: f = g + w * h. The start cell may lie in the inflated zone (the robot
// is already there, e.g. after a bump) - it only has to move out of it; the
// goal must be traversable. hWeight = 1: A*, 0: Dijkstra (for comparison).
inline SearchResult search(const CostMap& cm, Cell start, Cell goal, Connectivity conn = Connectivity::Eight,
                           double hWeight = 1.0) {
    const auto t0 = std::chrono::steady_clock::now();
    const GridMap& m = cm.map();
    const double C = m.cellSize();
    const std::size_t n = static_cast<std::size_t>(m.cols()) * static_cast<std::size_t>(m.rows());
    SearchResult r;
    r.order.assign(n, -1);
    if (!m.inside(start) || !m.inside(goal) || cm.obstacle(start) || !cm.traversable(goal)) return r;

    constexpr double Inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(n, Inf);
    std::vector<int> parent(n, -1);
    std::vector<char> closed(n, 0);

    struct Entry {
        double f, g;
        int index;
    };
    // Smallest f first; on a tie the larger g (the entry closer to the goal),
    // which keeps A* from expanding whole plateaus of equal f.
    const auto worse = [](const Entry& a, const Entry& b) {
        if (std::abs(a.f - b.f) > 1e-9) return a.f > b.f;
        return a.g < b.g;
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(worse)> open(worse);

    const int s = m.index(start), goalIndex = m.index(goal);
    g[static_cast<std::size_t>(s)] = 0.0;
    open.push({ hWeight * heuristic(start, goal, conn, C), 0.0, s });
    r.generated = 1;
    const int moves = conn == Connectivity::Four ? 4 : 8;

    while (!open.empty()) {
        const Entry e = open.top();
        open.pop();
        const auto ei = static_cast<std::size_t>(e.index);
        if (closed[ei]) continue;                      // stale entry (a shorter way was found later)
        closed[ei] = 1;
        r.order[ei] = r.expanded++;
        if (e.index == goalIndex) break;

        const Cell c = m.cellOf(e.index);
        const bool leaving = !cm.traversable(c);   // only the start cell can be inflated
        for (int k = 0; k < moves; ++k) {
            if (!moveAllowed(cm, c, k, leaving)) continue;
            const Cell nb{ c.x + NeighbourDx[k], c.y + NeighbourDy[k] };
            const auto ni = static_cast<std::size_t>(m.index(nb));
            if (closed[ni]) continue;
            const double ng = g[ei] + (k < 4 ? C : Sqrt2 * C);
            if (ng < g[ni] - 1e-12) {
                g[ni] = ng;
                parent[ni] = e.index;
                open.push({ ng + hWeight * heuristic(nb, goal, conn, C), ng, static_cast<int>(ni) });
                ++r.generated;
            }
        }
    }

    if (closed[static_cast<std::size_t>(goalIndex)]) {
        r.found = true;
        r.cost = g[static_cast<std::size_t>(goalIndex)];
        for (int i = goalIndex; i != -1; i = parent[static_cast<std::size_t>(i)]) r.path.push_back(m.cellOf(i));
        std::reverse(r.path.begin(), r.path.end());
    }
    r.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

// ------------------------------------------------------ path processing --
inline double length(const std::vector<Point>& pts) {
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) L += distance(pts[i - 1], pts[i]);
    return L;
}

// Cells where the A* path changes direction (plus start and goal): the robot
// drives straight between them. Same route as the cell path, fewer stops.
inline std::vector<Cell> corners(const std::vector<Cell>& path) {
    if (path.size() <= 2) return path;
    std::vector<Cell> out{ path.front() };
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const int dx1 = path[i].x - path[i - 1].x, dy1 = path[i].y - path[i - 1].y;
        const int dx2 = path[i + 1].x - path[i].x, dy2 = path[i + 1].y - path[i].y;
        if (dx1 != dx2 || dy1 != dy2) out.push_back(path[i]);
    }
    out.push_back(path.back());
    return out;
}

inline std::vector<Point> toPoints(const GridMap& m, const std::vector<Cell>& cells) {
    std::vector<Point> pts;
    pts.reserve(cells.size());
    for (const Cell& c : cells) pts.push_back(m.centre(c));
    return pts;
}

// Line-of-sight shortening ("string pulling"): from each kept point jump to
// the farthest later point of the path that can be reached in a straight
// line with `need` metres of clearance. Gives fewer, longer, any-angle legs.
inline std::vector<Point> shortcut(const CostMap& cm, const std::vector<Point>& pts, double need) {
    if (pts.size() <= 2) return pts;
    std::vector<Point> out{ pts.front() };
    std::size_t i = 0;
    while (i + 1 < pts.size()) {
        std::size_t j = pts.size() - 1;
        while (j > i + 1 && !cm.segmentClear(pts[i], pts[j], need)) --j;
        out.push_back(pts[j]);
        i = j;
    }
    return out;
}

// Smallest distance (m) between the robot centre and any obstacle along the
// polyline (sampled every 1 cm), capped at `limit`.
inline double minClearance(const CostMap& cm, const std::vector<Point>& pts, double limit = 1.0) {
    double best = limit;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const double len = distance(pts[i], pts[i + 1]);
        const int steps = std::max(1, static_cast<int>(std::ceil(len / 0.01)));
        for (int k = 0; k <= steps; ++k) {
            const double t = static_cast<double>(k) / steps;
            best = std::min(best, cm.clearance({ pts[i].x + (pts[i + 1].x - pts[i].x) * t,
                                                 pts[i].y + (pts[i + 1].y - pts[i].y) * t }, limit));
        }
    }
    if (pts.size() == 1) best = cm.clearance(pts.front(), limit);
    return best;
}

// Every cell the robot can reach from `from` (same move rules as A*).
inline std::vector<char> reachable(const CostMap& cm, Cell from, Connectivity conn = Connectivity::Eight) {
    const GridMap& m = cm.map();
    std::vector<char> seen(static_cast<std::size_t>(m.cols()) * static_cast<std::size_t>(m.rows()), 0);
    if (!m.inside(from) || cm.obstacle(from)) return seen;
    std::vector<Cell> stack{ from };
    seen[static_cast<std::size_t>(m.index(from))] = 1;
    const int moves = conn == Connectivity::Four ? 4 : 8;
    while (!stack.empty()) {
        const Cell c = stack.back();
        stack.pop_back();
        const bool leaving = !cm.traversable(c);
        for (int k = 0; k < moves; ++k) {
            if (!moveAllowed(cm, c, k, leaving)) continue;
            const Cell nb{ c.x + NeighbourDx[k], c.y + NeighbourDy[k] };
            char& flag = seen[static_cast<std::size_t>(m.index(nb))];
            if (!flag) { flag = 1; stack.push_back(nb); }
        }
    }
    return seen;
}

// The traversable cell reachable from `from` that is closest to `target`
// (used when the requested goal is too close to an obstacle). False if the
// robot cannot move at all.
inline bool nearestSafeCell(const CostMap& cm, Cell from, Cell target, Cell& out,
                            Connectivity conn = Connectivity::Eight) {
    const GridMap& m = cm.map();
    const std::vector<char> seen = reachable(cm, from, conn);
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int y = 0; y < m.rows(); ++y) {
        for (int x = 0; x < m.cols(); ++x) {
            const Cell c{ x, y };
            if (!seen[static_cast<std::size_t>(m.index(c))] || !cm.traversable(c)) continue;
            const double d = std::hypot(x - target.x, y - target.y);
            if (d < best - 1e-9) { best = d; out = c; found = true; }
        }
    }
    return found;
}

} // namespace astar
