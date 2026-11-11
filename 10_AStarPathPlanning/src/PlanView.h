// PlanView.h - draws the map, the A* search and the robot (live window and
// PNG file). Uses the shared GdiCanvas.h; colours follow 09_Mapping.
//
//   left  : the grid, cell by cell
//             dark red     obstacle (1 in the map file)
//             light red    inflated: closer than the robot radius to an
//                          obstacle - the robot CENTRE must not go there
//             light blue   cells A* expanded (closed set)
//             white        free
//           the A* path (blue), the robot footprint along it (green
//           outline = robot radius on both sides), start, goal, and while
//           driving the robot and its trail.
//   right : numbers (map, plan, A* vs Dijkstra, run) and the waypoints.
#pragma once

#include "AStarPlanner.h"
#include "GdiCanvas.h"
#include "Robot.h"

#include <algorithm>
#include <cmath>
#include <cwchar>
#include <string>
#include <vector>

namespace view {

constexpr COLORREF ColObstacle = RGB(128, 0, 0);
constexpr COLORREF ColMarked   = RGB(70, 0, 0);       // obstacle found by the bumper
constexpr COLORREF ColInflated = RGB(246, 205, 205);
constexpr COLORREF ColExpanded = RGB(214, 229, 250);
constexpr COLORREF ColFree     = RGB(255, 255, 255);
constexpr COLORREF ColGrid     = RGB(190, 194, 208);
constexpr COLORREF ColPath     = RGB(38, 98, 214);
constexpr COLORREF ColRoute    = RGB(232, 120, 0);     // shortened route (when different from the A* path)
constexpr COLORREF ColFootprint= RGB(70, 160, 60);
constexpr COLORREF ColRobot    = RGB(88, 153, 28);
constexpr COLORREF ColTrail    = RGB(20, 110, 20);
constexpr COLORREF ColStart    = RGB(88, 153, 28);
constexpr COLORREF ColGoal     = RGB(232, 120, 0);
constexpr COLORREF ColText     = RGB(34, 34, 34);
constexpr COLORREF ColMuted    = RGB(105, 105, 105);
constexpr COLORREF ColBack     = RGB(246, 246, 244);

struct PlanScene {
    const astar::CostMap* cost = nullptr;
    const std::vector<int>* expanded = nullptr;           // per cell: A* expansion order (-1 = no)
    const std::vector<astar::Cell>* cells = nullptr;      // A* cell path
    const std::vector<astar::Point>* route = nullptr;     // waypoints the robot drives (may equal the cell path)
    bool routeIsShortcut = false;                         // draw the route in its own colour
    const std::vector<astar::Point>* trail = nullptr;     // where the robot has been
    const std::vector<astar::Cell>* marked = nullptr;     // obstacles found by the bumper
    astar::Cell start, goal;
    bool hasRequestedGoal = false;                        // goal was moved: show where it was asked for
    astar::Cell requestedGoal;
    bool hasRobot = false;
    nav::Pose pose;
    double robotRadius = 0.15;
    int activeLeg = -1;
    std::wstring title, status;
    std::vector<std::wstring> info;                       // right-hand panel, one line each ("" = gap, "#..." = heading)
};

struct GridView {
    int ox = 0, oy = 0, px = 1;  // top-left pixel of cell (0, 0), pixels per cell
    int cols = 0, rows = 0;
    double cell = 0.2;

    POINT cellTopLeft(astar::Cell c) const { return { ox + c.x * px, oy + c.y * px }; }
    POINT toPixel(astar::Point p) const {
        return { ox + static_cast<LONG>(std::lround(p.x / cell * px)),
                 oy + static_cast<LONG>(std::lround((rows - p.y / cell) * px)) };
    }
    int metres(double m) const { return std::max(1, static_cast<int>(std::lround(m / cell * px))); }
};

inline GridView fitGrid(const astar::GridMap& m, const RECT& area) {
    GridView g;
    g.cols = m.cols();
    g.rows = m.rows();
    g.cell = m.cellSize();
    const int labelW = 26, labelH = 20;
    const int w = area.right - area.left - labelW, h = area.bottom - area.top - labelH;
    g.px = std::max(3, std::min(w / std::max(1, g.cols), h / std::max(1, g.rows)));
    g.ox = area.left + labelW + (w - g.px * g.cols) / 2;
    g.oy = area.top + labelH + (h - g.px * g.rows) / 2;
    return g;
}

inline void drawCross(Canvas& c, POINT p, int r, COLORREF col, int width) {
    c.line(p.x - r, p.y - r, p.x + r + 1, p.y + r + 1, col, width);
    c.line(p.x - r, p.y + r, p.x + r + 1, p.y - r - 1, col, width);
}

inline void drawGrid(Canvas& c, const PlanScene& s, const GridView& g) {
    const astar::CostMap& cm = *s.cost;
    const astar::GridMap& m = cm.map();

    // Cells.
    for (int y = 0; y < m.rows(); ++y) {
        for (int x = 0; x < m.cols(); ++x) {
            const astar::Cell cell{ x, y };
            COLORREF col = ColFree;
            switch (cm.state(cell)) {
                case astar::CellState::Obstacle: col = ColObstacle; break;
                case astar::CellState::Inflated: col = ColInflated; break;
                default:
                    if (s.expanded && (*s.expanded)[static_cast<std::size_t>(m.index(cell))] >= 0) col = ColExpanded;
                    break;
            }
            if (s.marked && std::find(s.marked->begin(), s.marked->end(), cell) != s.marked->end()) col = ColMarked;
            const POINT p = g.cellTopLeft(cell);
            c.fill(p.x, p.y, p.x + g.px, p.y + g.px, col);
        }
    }
    // Grid lines and frame.
    for (int x = 0; x <= m.cols(); ++x) c.line(g.ox + x * g.px, g.oy, g.ox + x * g.px, g.oy + m.rows() * g.px, ColGrid);
    for (int y = 0; y <= m.rows(); ++y) c.line(g.ox, g.oy + y * g.px, g.ox + m.cols() * g.px, g.oy + y * g.px, ColGrid);
    c.frame(g.ox - 1, g.oy - 1, g.ox + m.cols() * g.px + 2, g.oy + m.rows() * g.px + 2, ColMuted);

    // Column / row numbers (every one if there is room, else every 5th).
    const int step = g.px >= 16 ? 1 : (g.px >= 7 ? 5 : 10);
    wchar_t buf[16];
    for (int x = 0; x < m.cols(); x += step) {
        swprintf(buf, 16, L"%d", x);
        c.text(g.ox + x * g.px + g.px / 2 - (x >= 10 ? 7 : 4), g.oy - 20, buf, ColMuted);
    }
    for (int y = 0; y < m.rows(); y += step) {
        swprintf(buf, 16, L"%d", y);
        c.text(g.ox - (y >= 10 ? 22 : 14), g.oy + y * g.px + g.px / 2 - 9, buf, ColMuted);
    }

    // Robot footprint along the route: everything the robot body sweeps.
    const std::vector<astar::Point>* drive = s.route;
    if (drive && drive->size() >= 2) {
        const int rr = g.metres(s.robotRadius);
        for (std::size_t i = 0; i + 1 < drive->size(); ++i) {
            const astar::Point a = (*drive)[i], b = (*drive)[i + 1];
            const double len = astar::distance(a, b);
            if (len < 1e-9) continue;
            const double nx = -(b.y - a.y) / len * s.robotRadius, ny = (b.x - a.x) / len * s.robotRadius;
            for (int side : { -1, 1 }) {
                const POINT p = g.toPixel({ a.x + side * nx, a.y + side * ny });
                const POINT q = g.toPixel({ b.x + side * nx, b.y + side * ny });
                c.line(p.x, p.y, q.x, q.y, ColFootprint, 1);
            }
        }
        for (const auto& p : *drive) {
            const POINT q = g.toPixel(p);
            c.circle(q.x, q.y, rr, ColFootprint, 1, false);
        }
    }

    // A* cell path (blue), and the driven route if it was shortened (orange).
    if (s.cells && s.cells->size() >= 2) {
        std::vector<POINT> pts;
        for (const auto& cell : *s.cells) pts.push_back(g.toPixel(m.centre(cell)));
        c.polyline(pts, ColPath, std::max(2, g.px / 8));
        for (const auto& q : pts) c.circle(q.x, q.y, std::max(2, g.px / 9), ColPath, 1, true);
    }
    if (s.routeIsShortcut && drive && drive->size() >= 2) {
        std::vector<POINT> pts;
        for (const auto& p : *drive) pts.push_back(g.toPixel(p));
        c.polyline(pts, ColRoute, std::max(2, g.px / 10));
    }

    // Requested (unsafe) goal, start, goal.
    if (s.hasRequestedGoal) {
        const POINT q = g.toPixel(m.centre(s.requestedGoal));
        drawCross(c, q, std::max(4, g.px / 4), ColMuted, 2);
    }
    {
        const POINT q = g.toPixel(m.centre(s.start));
        c.circle(q.x, q.y, std::max(5, g.px / 3), ColStart, 2, false);
        c.text(q.x + g.px / 3 + 2, q.y - g.px / 2 - 2, L"S", ColStart, true);
    }
    {
        const POINT q = g.toPixel(m.centre(s.goal));
        drawCross(c, q, std::max(5, g.px / 3), ColGoal, 3);
        c.text(q.x + g.px / 3 + 2, q.y - g.px / 2 - 2, L"G", ColGoal, true);
    }

    // Trail and robot.
    if (s.trail && s.trail->size() >= 2) {
        std::vector<POINT> pts;
        pts.reserve(s.trail->size());
        for (const auto& p : *s.trail) pts.push_back(g.toPixel(p));
        c.polyline(pts, ColTrail, 2);
    }
    if (s.hasRobot) {
        const POINT r = g.toPixel(s.pose.point());
        const int rr = g.metres(s.robotRadius);
        c.circle(r.x, r.y, rr, ColRobot, 2, false);
        c.circle(r.x, r.y, std::max(2, rr / 4), ColRobot, 1, true);
        const POINT nose = g.toPixel({ s.pose.x + s.robotRadius * std::cos(s.pose.heading),
                                       s.pose.y + s.robotRadius * std::sin(s.pose.heading) });
        c.line(r.x, r.y, nose.x, nose.y, ColTrail, 3);
    }
}

inline void legendItem(Canvas& c, int& x, int y, COLORREF col, const wchar_t* label, bool line = false) {
    if (line) {
        c.line(x, y + 10, x + 16, y + 10, col, 3);
    } else {
        c.fill(x, y + 3, x + 14, y + 17, col);
        c.frame(x, y + 3, x + 14, y + 17, ColGrid);
    }
    c.text(x + 21, y, label, ColMuted);
    x += 36 + 7 * static_cast<int>(std::wcslen(label));
}

inline void render(Canvas& c, const PlanScene& s) {
    const int W = c.width(), H = c.height();
    c.fill(0, 0, W, H, ColBack);
    c.text(18, 10, s.title, ColText, true);
    c.text(18, 34, s.status, ColMuted);

    const int top = 64, bottom = H - 44, margin = 18, panelW = std::min(330, W / 3);
    const RECT mapArea{ margin, top, W - margin - panelW - 12, bottom };
    drawGrid(c, s, fitGrid(s.cost->map(), mapArea));

    // Right-hand panel.
    int y = top + 6;
    const int x = W - margin - panelW;
    for (const auto& line : s.info) {
        if (line.empty()) { y += 8; continue; }
        if (line[0] == L'#') { y += 4; c.text(x, y, line.substr(1), ColText, true); y += 24; continue; }
        if (y > bottom - 18) { c.text(x, y, L"...", ColMuted); break; }
        c.text(x, y, line, ColMuted);
        y += 19;
    }

    int lx = margin;
    const int ly = H - 30;
    legendItem(c, lx, ly, ColObstacle, L"obstacle");
    legendItem(c, lx, ly, ColInflated, L"within robot radius");
    legendItem(c, lx, ly, ColExpanded, L"expanded by A*");
    legendItem(c, lx, ly, ColPath, L"A* path", true);
    if (s.routeIsShortcut) legendItem(c, lx, ly, ColRoute, L"shortened route", true);
    legendItem(c, lx, ly, ColFootprint, L"robot footprint", true);
    if (s.marked && !s.marked->empty()) legendItem(c, lx, ly, ColMarked, L"found by the bumper");
    if (s.trail) legendItem(c, lx, ly, ColTrail, L"driven", true);
}

} // namespace view
