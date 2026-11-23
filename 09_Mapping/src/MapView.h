// MapView.h - draws the map (live window + PNG files). Windows GDI only, no
// external libraries.
//
// One picture, two panels showing the same area:
//   left  - the occupancy grid itself: P(Occupied) of every small cell
//           (white = empty, grey = unknown, dark red = occupied), with the
//           robot, its planned path and the frontier it is heading for;
//   right - the grid map derived from it: big cells that are occupied /
//           empty / unknown, drawn like the assignment figure (dark red
//           walls, white free cells, the robot's cell in green).
//
// Everything is drawn into an off-screen bitmap (Canvas, GdiCanvas.h); the
// window just shows that bitmap and the PNG export saves it, so the saved
// picture is exactly what was on screen - and the program also works without
// a window.
#pragma once

#include "GdiCanvas.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cwchar>
#include <string>
#include <vector>

#include "FrontierExplorer.h"
#include "OccupancyGrid.h"

namespace view {

// Colours of the assignment figure.
constexpr COLORREF ColOccupied = RGB(128, 0, 0);
constexpr COLORREF ColEmpty    = RGB(255, 255, 255);
constexpr COLORREF ColUnknown  = RGB(212, 212, 212);
constexpr COLORREF ColGridLine = RGB(139, 146, 176);
constexpr COLORREF ColRobot    = RGB(88, 153, 28);
constexpr COLORREF ColPath     = RGB(38, 98, 214);
constexpr COLORREF ColTarget   = RGB(232, 120, 0);
constexpr COLORREF ColFrontier = RGB(0, 168, 190);
constexpr COLORREF ColText     = RGB(34, 34, 34);
constexpr COLORREF ColMuted    = RGB(105, 105, 105);
constexpr COLORREF ColPage     = RGB(246, 246, 244);

// What to draw.
struct Scene {
    const ogm::OccupancyGrid* grid = nullptr;
    const ogm::CellMap* cells = nullptr;
    explore::Pose pose;
    double robotRadius = 0.18;
    const std::vector<explore::Point>* path = nullptr;      // planned path (may be null)
    const std::vector<explore::Point>* frontier = nullptr;  // frontier goal cells (may be null)
    bool hasTarget = false;
    explore::Point target;
    int quarterTurns = 0;   // 0: the robot's initial heading points up; each step turns the picture 90 deg
    std::wstring title, status;
};

// World metres -> pixels of one panel. Default orientation: +x (the robot's
// initial heading) up, +y (its left) to the left.
struct Viewport {
    int k = 0;
    double uMin = 0.0, vMax = 0.0, scale = 1.0;
    int ox = 0, oy = 0, w = 0, h = 0;  // content rectangle in pixels

    void toView(double x, double y, double& u, double& v) const {
        double xr = x, yr = y;
        switch (k & 3) {
            case 1: xr = -y; yr = x; break;
            case 2: xr = -x; yr = -y; break;
            case 3: xr = y; yr = -x; break;
            default: break;
        }
        u = -yr;
        v = xr;
    }

    void toWorld(double u, double v, double& x, double& y) const {
        const double xr = v, yr = -u;
        switch (k & 3) {
            case 1: x = yr; y = -xr; break;
            case 2: x = -xr; y = -yr; break;
            case 3: x = -yr; y = xr; break;
            default: x = xr; y = yr; break;
        }
    }

    POINT px(double x, double y) const {
        double u, v;
        toView(x, y, u, v);
        return { ox + static_cast<LONG>(std::lround((u - uMin) * scale)), oy + static_cast<LONG>(std::lround((vMax - v) * scale)) };
    }
};

// Fit the cell map (plus one cell of margin, at least 3 x 3 m) into `area`,
// with a whole number of pixels per big cell so the grid lines stay crisp.
inline Viewport fitViewport(const ogm::CellMap& m, int quarterTurns, const RECT& area) {
    Viewport vp;
    vp.k = quarterTurns;
    const double C = m.cellSize;
    double x0 = -1.5, x1 = 1.5, y0 = -1.5, y1 = 1.5;
    if (m.nx > 0 && m.ny > 0) {
        x0 = (m.ix0 - 0.5) * C; x1 = (m.ix0 + m.nx - 0.5) * C;
        y0 = (m.iy0 - 0.5) * C; y1 = (m.iy0 + m.ny - 0.5) * C;
    }
    const auto grow = [&](double& lo, double& hi) {
        while (hi - lo < 3.0 - 1e-9) { lo -= C; hi += C; }  // whole cells, so the view starts on a cell edge
        lo -= C;
        hi += C;
    };
    grow(x0, x1);
    grow(y0, y1);

    double uA, vA, uB, vB;
    vp.toView(x0, y0, uA, vA);
    vp.toView(x1, y1, uB, vB);
    const double uLo = std::min(uA, uB), uHi = std::max(uA, uB), vLo = std::min(vA, vB), vHi = std::max(vA, vB);

    const double fit = std::min((area.right - area.left) / (uHi - uLo), (area.bottom - area.top) / (vHi - vLo));
    const double perCell = std::max(2.0, std::floor(fit * C));
    vp.scale = perCell / C;
    vp.w = static_cast<int>(std::lround((uHi - uLo) * vp.scale));
    vp.h = static_cast<int>(std::lround((vHi - vLo) * vp.scale));
    vp.ox = area.left + ((area.right - area.left) - vp.w) / 2;
    vp.oy = area.top + ((area.bottom - area.top) - vp.h) / 2;
    vp.uMin = uLo;
    vp.vMax = vHi;
    return vp;
}

inline std::uint32_t shade(double p) {
    // 0 -> white, 0.5 -> unknown grey, 1 -> dark red.
    const auto mix = [](int a, int b, double t) { return static_cast<std::uint32_t>(std::lround(a + (b - a) * t)); };
    if (p <= 0.5) {
        const double t = (0.5 - p) / 0.5;
        const std::uint32_t g = mix(212, 255, t);
        return (g << 16) | (g << 8) | g;
    }
    const double t = (p - 0.5) / 0.5;
    return (mix(212, 128, t) << 16) | (mix(212, 0, t) << 8) | mix(212, 0, t);
}

// Left panel: the probability grid with robot, path and frontier.
inline void drawOccupancyGrid(Canvas& c, const Scene& s, const Viewport& vp) {
    c.flush();
    for (int py = 0; py < vp.h; ++py) {
        for (int pxl = 0; pxl < vp.w; ++pxl) {
            double x, y;
            vp.toWorld(vp.uMin + (pxl + 0.5) / vp.scale, vp.vMax - (py + 0.5) / vp.scale, x, y);
            c.pixel(vp.ox + pxl, vp.oy + py, shade(s.grid->p(s.grid->toCell(x), s.grid->toCell(y))));
        }
    }
    c.frame(vp.ox - 1, vp.oy - 1, vp.ox + vp.w + 1, vp.oy + vp.h + 1, ColGridLine);

    if (s.frontier) {
        for (const auto& f : *s.frontier) {
            const POINT p = vp.px(f.x, f.y);
            c.fill(p.x - 1, p.y - 1, p.x + 1, p.y + 1, ColFrontier);
        }
    }
    if (s.path && s.path->size() >= 2) {
        std::vector<POINT> pts;
        pts.reserve(s.path->size());
        for (const auto& q : *s.path) pts.push_back(vp.px(q.x, q.y));
        c.polyline(pts, ColPath, 2);
    }
    if (s.hasTarget) {
        const POINT t = vp.px(s.target.x, s.target.y);
        c.line(t.x - 6, t.y - 6, t.x + 7, t.y + 7, ColTarget, 3);
        c.line(t.x - 6, t.y + 6, t.x + 7, t.y - 7, ColTarget, 3);
    }
    const POINT home = vp.px(0.0, 0.0);
    c.line(home.x - 5, home.y, home.x + 6, home.y, ColText, 1);
    c.line(home.x, home.y - 5, home.x, home.y + 6, ColText, 1);

    const POINT r = vp.px(s.pose.x, s.pose.y);
    const int rr = std::max(3, static_cast<int>(std::lround(s.robotRadius * vp.scale)));
    c.circle(r.x, r.y, rr, ColRobot, 1, true);
    const POINT nose = vp.px(s.pose.x + 1.6 * s.robotRadius * std::cos(s.pose.heading),
                             s.pose.y + 1.6 * s.robotRadius * std::sin(s.pose.heading));
    c.line(r.x, r.y, nose.x, nose.y, RGB(20, 70, 0), 2);
}

// Right panel: the grid map, drawn like the assignment figure.
inline void drawCellMap(Canvas& c, const Scene& s, const Viewport& vp) {
    const ogm::CellMap& m = *s.cells;
    const double C = m.cellSize;
    c.fill(vp.ox, vp.oy, vp.ox + vp.w, vp.oy + vp.h, ColUnknown);

    // Every big cell that touches the view (cells outside the map are unknown).
    double xa, ya, xb, yb;
    vp.toWorld(vp.uMin, vp.vMax, xa, ya);
    vp.toWorld(vp.uMin + vp.w / vp.scale, vp.vMax - vp.h / vp.scale, xb, yb);
    const int ix0 = ogm::CellMap::indexOf(std::min(xa, xb) + 0.5 * C, C), ix1 = ogm::CellMap::indexOf(std::max(xa, xb) - 0.5 * C, C);
    const int iy0 = ogm::CellMap::indexOf(std::min(ya, yb) + 0.5 * C, C), iy1 = ogm::CellMap::indexOf(std::max(ya, yb) - 0.5 * C, C);
    const int rx = ogm::CellMap::indexOf(s.pose.x, C), ry = ogm::CellMap::indexOf(s.pose.y, C);

    for (int iy = iy0; iy <= iy1; ++iy) {
        for (int ix = ix0; ix <= ix1; ++ix) {
            const POINT a = vp.px((ix - 0.5) * C, (iy - 0.5) * C), b = vp.px((ix + 0.5) * C, (iy + 0.5) * C);
            const int left = std::min(a.x, b.x), right = std::max(a.x, b.x);
            const int top = std::min(a.y, b.y), bottom = std::max(a.y, b.y);
            COLORREF col = ColUnknown;
            const ogm::Cell st = m.at(ix, iy);
            if (ix == rx && iy == ry) col = ColRobot;
            else if (st == ogm::Cell::Occupied) col = ColOccupied;
            else if (st == ogm::Cell::Empty) col = ColEmpty;
            c.fill(left, top, right, bottom, col);
            c.frame(left, top, right + 1, bottom + 1, ColGridLine);
        }
    }
    c.frame(vp.ox - 1, vp.oy - 1, vp.ox + vp.w + 1, vp.oy + vp.h + 1, ColGridLine);
}

inline void legendItem(Canvas& c, int& x, int y, COLORREF col, const wchar_t* label) {
    c.fill(x, y + 3, x + 14, y + 17, col);
    c.frame(x, y + 3, x + 14, y + 17, ColGridLine);
    c.text(x + 20, y, label, ColMuted);
    x += 34 + 8 * static_cast<int>(std::wcslen(label));
}

// The whole picture: header, both panels, legend.
inline void render(Canvas& c, const Scene& s) {
    const int W = c.width(), H = c.height();
    c.fill(0, 0, W, H, ColPage);
    c.text(18, 10, s.title, ColText, true);
    c.text(18, 34, s.status, ColMuted);

    const int top = 92, bottom = H - 44, gap = 22, margin = 18;
    const int panelW = (W - 2 * margin - gap) / 2;
    const RECT leftArea{ margin, top, margin + panelW, bottom };
    const RECT rightArea{ margin + panelW + gap, top, W - margin, bottom };

    wchar_t buf[96];
    swprintf(buf, 96, L"Occupancy grid  -  P(occupied), %.0f cm cells", s.grid->cellSize() * 100.0);
    c.text(leftArea.left, top - 26, buf, ColText);
    swprintf(buf, 96, L"Grid map  -  %.0f cm cells", s.cells->cellSize * 100.0);
    c.text(rightArea.left, top - 26, buf, ColText);

    drawOccupancyGrid(c, s, fitViewport(*s.cells, s.quarterTurns, leftArea));
    drawCellMap(c, s, fitViewport(*s.cells, s.quarterTurns, rightArea));

    int x = margin;
    const int y = H - 30;
    legendItem(c, x, y, ColOccupied, L"occupied");
    legendItem(c, x, y, ColEmpty, L"empty");
    legendItem(c, x, y, ColUnknown, L"unknown");
    legendItem(c, x, y, ColRobot, L"robot");
    legendItem(c, x, y, ColPath, L"planned path");
    legendItem(c, x, y, ColFrontier, L"frontier");
    legendItem(c, x, y, ColTarget, L"goal");
}

} // namespace view
