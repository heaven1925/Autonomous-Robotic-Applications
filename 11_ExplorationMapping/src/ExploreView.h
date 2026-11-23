// ExploreView.h - draws the Bayesian grid while exploring (window + PNG).
//
//   cell colour  : P(Occupied), white (0) -> grey (0.5, unknown) -> dark red (1)
//   cell text    : P(Occupied) with two decimals
//   cyan frame   : frontier cell, orange frame: the frontier being explored
//   blue line    : A* path to the frontier's empty neighbour
//   green        : robot and its sonar beam (to the measured range)
#pragma once

#include "BayesGrid.h"
#include "Explorer.h"
#include "GdiCanvas.h"

#include <algorithm>
#include <cmath>
#include <cwchar>
#include <string>
#include <vector>

namespace view {

constexpr COLORREF ColText   = RGB(34, 34, 34);
constexpr COLORREF ColMuted  = RGB(105, 105, 105);
constexpr COLORREF ColBack   = RGB(246, 246, 244);
constexpr COLORREF ColGrid   = RGB(150, 156, 180);
constexpr COLORREF ColFront  = RGB(0, 168, 190);
constexpr COLORREF ColTarget = RGB(232, 120, 0);
constexpr COLORREF ColPath   = RGB(38, 98, 214);
constexpr COLORREF ColRobot  = RGB(88, 153, 28);
constexpr COLORREF ColBeam   = RGB(20, 140, 60);
constexpr COLORREF ColNoEcho = RGB(170, 200, 170);

struct ExploreScene {
    const bayes::BayesGrid* grid = nullptr;
    const explore::Explorer* explorer = nullptr;
    nav::Pose pose;
    double robotRadius = 0.175;
    double noEchoRange = 0.8;
    astar::Cell start;
    std::wstring title, status;
    std::vector<std::wstring> info;
};

inline COLORREF shade(double p) {
    const auto mix = [](int a, int b, double t) { return static_cast<int>(std::lround(a + (b - a) * t)); };
    if (p <= 0.5) {
        const int g = mix(212, 255, (0.5 - p) / 0.5);
        return RGB(g, g, g);
    }
    const double t = (p - 0.5) / 0.5;
    return RGB(mix(212, 128, t), mix(212, 0, t), mix(212, 0, t));
}

inline void renderExplore(Canvas& c, const ExploreScene& s) {
    const int W = c.width(), H = c.height();
    c.fill(0, 0, W, H, ColBack);
    c.text(18, 10, s.title, ColText, true);
    c.text(18, 34, s.status, ColMuted);

    const bayes::BayesGrid& g = *s.grid;
    const astar::GridMap& m = g.arena();
    const int panelW = std::min(360, W / 3), top = 70, bottom = H - 40;
    const int areaW = W - panelW - 60, areaH = bottom - top - 24;
    const int px = std::max(8, std::min(areaW / m.cols(), areaH / m.rows()));
    const int ox = 18 + 28 + (areaW - px * m.cols()) / 2, oy = top + 22;
    const auto toPx = [&](astar::Point p) {
        return POINT{ ox + static_cast<LONG>(std::lround(p.x / m.cellSize() * px)),
                      oy + static_cast<LONG>(std::lround((m.rows() - p.y / m.cellSize()) * px)) };
    };

    wchar_t buf[64];
    for (int y = 0; y < m.rows(); ++y) {
        for (int x = 0; x < m.cols(); ++x) {
            const astar::Cell cell{ x, y };
            const int x0 = ox + x * px, y0 = oy + y * px;
            const double p = g.p(cell);
            c.fill(x0, y0, x0 + px, y0 + px, shade(p));
            swprintf(buf, 64, L"%.2f", p);
            c.text(x0 + px / 2 - 15, y0 + px / 2 - 9, buf, p > 0.65 ? RGB(255, 255, 255) : ColMuted);
        }
    }
    for (int x = 0; x <= m.cols(); ++x) c.line(ox + x * px, oy, ox + x * px, oy + m.rows() * px, ColGrid);
    for (int y = 0; y <= m.rows(); ++y) c.line(ox, oy + y * px, ox + m.cols() * px, oy + y * px, ColGrid);
    c.frame(ox - 2, oy - 2, ox + m.cols() * px + 3, oy + m.rows() * px + 3, m.walledBorder ? RGB(90, 0, 0) : ColGrid);
    for (int x = 0; x < m.cols(); ++x) {
        swprintf(buf, 64, L"%d", m.coords.colLabel(x));
        c.text(ox + x * px + px / 2 - 4, oy - 20, buf, ColMuted);
    }
    for (int y = 0; y < m.rows(); ++y) {
        swprintf(buf, 64, L"%d", m.coords.rowLabel(y));
        c.text(ox - (m.coords.rowLabel(y) >= 10 ? 24 : 16), oy + y * px + px / 2 - 9, buf, ColMuted);
    }

    // Start cell, frontier, target, path.
    {
        const int x0 = ox + s.start.x * px, y0 = oy + s.start.y * px;
        c.text(x0 + 4, y0 + 2, L"*", ColRobot, true);
    }
    const explore::Explorer& e = *s.explorer;
    for (const auto& f : e.frontier()) {
        const int x0 = ox + f.x * px, y0 = oy + f.y * px;
        if (g.state(f) != bayes::State::Undecided) continue;
        c.frame(x0 + 2, y0 + 2, x0 + px - 1, y0 + px - 1, ColFront);
        c.frame(x0 + 3, y0 + 3, x0 + px - 2, y0 + px - 2, ColFront);
    }
    if (e.hasTarget()) {
        const astar::Cell t = e.target();
        const int x0 = ox + t.x * px, y0 = oy + t.y * px;
        for (int k = 2; k <= 5; ++k) c.frame(x0 + k, y0 + k, x0 + px - k + 1, y0 + px - k + 1, ColTarget);
    }
    if (e.path().size() >= 2) {
        std::vector<POINT> pts;
        for (const auto& cell : e.path()) pts.push_back(toPx(m.centre(cell)));
        c.polyline(pts, ColPath, 3);
    }

    // Robot and beams.
    for (int k = 0; k < 3; ++k) {
        const auto& b = e.beam(k);
        if (!b.set) continue;
        const double len = b.echo ? b.range : s.noEchoRange;
        const POINT a = toPx(b.sensor);
        const POINT q = toPx({ b.sensor.x + len * std::cos(b.angle), b.sensor.y + len * std::sin(b.angle) });
        c.line(a.x, a.y, q.x, q.y, b.echo ? ColBeam : ColNoEcho, 2);
        if (b.echo) c.circle(q.x, q.y, 4, ColBeam, 1, true);
    }
    const POINT r = toPx(s.pose.point());
    const int rr = std::max(4, static_cast<int>(std::lround(s.robotRadius / m.cellSize() * px)));
    c.circle(r.x, r.y, rr, ColRobot, 2, false);
    const POINT nose = toPx({ s.pose.x + s.robotRadius * std::cos(s.pose.heading), s.pose.y + s.robotRadius * std::sin(s.pose.heading) });
    c.line(r.x, r.y, nose.x, nose.y, ColRobot, 3);

    // Right-hand panel.
    int y = top;
    const int x = W - panelW - 10;
    for (const auto& line : s.info) {
        if (line.empty()) { y += 8; continue; }
        if (line[0] == L'#') { y += 4; c.text(x, y, line.substr(1), ColText, true); y += 24; continue; }
        if (y > bottom - 18) break;
        c.text(x, y, line, ColMuted);
        y += 19;
    }

    // Legend.
    int lx = 18;
    const int ly = H - 28;
    const auto item = [&](COLORREF col, const wchar_t* label, bool frameOnly) {
        if (frameOnly) { c.frame(lx, ly + 3, lx + 14, ly + 17, col); c.frame(lx + 1, ly + 4, lx + 13, ly + 16, col); }
        else { c.fill(lx, ly + 3, lx + 14, ly + 17, col); c.frame(lx, ly + 3, lx + 14, ly + 17, ColGrid); }
        c.text(lx + 20, ly, label, ColMuted);
        lx += 34 + 7 * static_cast<int>(std::wcslen(label));
    };
    item(shade(0.05), L"empty (P < 0.3)", false);
    item(shade(0.5), L"undecided", false);
    item(shade(0.95), L"occupied (P > 0.8)", false);
    item(ColFront, L"frontier", true);
    item(ColTarget, L"target frontier", true);
    item(ColPath, L"A* path", false);
}

} // namespace view
