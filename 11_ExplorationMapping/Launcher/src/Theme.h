// Theme.h - the launcher's look: a light, card based theme drawn with GDI+
// (anti-aliased shapes, rounded corners, ClearType text). Every coordinate is
// in "base units" (96 dpi); the painter multiplies by the DPI scale.
#pragma once

#include "GdiCanvas.h"   // windows.h + gdiplus.h (with the min/max fix)

#include <map>
#include <memory>
#include <string>

namespace ui {

using Gdiplus::ARGB;

// ---------------------------------------------------------------- palette --
namespace col {
constexpr ARGB Bg = 0xFFF3F5F9;
constexpr ARGB Card = 0xFFFFFFFF;
constexpr ARGB Border = 0xFFE2E8F0;
constexpr ARGB Text = 0xFF0F172A;
constexpr ARGB Muted = 0xFF64748B;
constexpr ARGB Faint = 0xFF94A3B8;
constexpr ARGB Accent = 0xFF2563EB;
constexpr ARGB AccentDark = 0xFF1D4ED8;
constexpr ARGB AccentSoft = 0xFFDBEAFE;
constexpr ARGB AccentFaint = 0xFFEFF6FF;
constexpr ARGB Green = 0xFF16A34A;
constexpr ARGB GreenSoft = 0xFFDCFCE7;
constexpr ARGB Red = 0xFFDC2626;
constexpr ARGB RedSoft = 0xFFFEE2E2;
constexpr ARGB Amber = 0xFFD97706;
constexpr ARGB AmberSoft = 0xFFFEF3C7;
constexpr ARGB Track = 0xFFE5E7EB;
constexpr ARGB Hover = 0xFFF1F5F9;
constexpr ARGB Purple = 0xFF7C3AED;
constexpr ARGB Obstacle = 0xFFC81E1E;
} // namespace col

enum class Align { Left, Center, Right };

class Painter {
public:
    Painter(HDC dc, double scale) : g(dc), k_(static_cast<float>(scale)) {
        g.SetSmoothingMode(Gdiplus::SmoothingModeAntiAlias);
        g.SetTextRenderingHint(Gdiplus::TextRenderingHintClearTypeGridFit);
        g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeHalf);
    }

    Gdiplus::Graphics g;
    float S(double v) const { return static_cast<float>(v) * k_; }

    Gdiplus::Font& font(float size, bool bold = false) {
        const int key = static_cast<int>(size * 10.0f) * 2 + (bold ? 1 : 0);
        auto& f = fonts_[key];
        if (!f) f = std::make_unique<Gdiplus::Font>(L"Segoe UI", S(size), bold ? Gdiplus::FontStyleBold : Gdiplus::FontStyleRegular,
                                                    Gdiplus::UnitPixel);
        return *f;
    }

    void rect(double x, double y, double w, double h, ARGB c) {
        Gdiplus::SolidBrush b{ Gdiplus::Color(c) };
        g.FillRectangle(&b, S(x), S(y), S(w), S(h));
    }

    void round(double x, double y, double w, double h, double r, ARGB fill, ARGB stroke = 0, double width = 1.0) {
        Gdiplus::GraphicsPath p;
        roundPath(p, S(x), S(y), S(w), S(h), S(r));
        if (fill) { Gdiplus::SolidBrush b{ Gdiplus::Color(fill) }; g.FillPath(&b, &p); }
        if (stroke) { Gdiplus::Pen pen(Gdiplus::Color(stroke), S(width)); g.DrawPath(&pen, &p); }
    }

    // White card with a hairline border and a soft shadow.
    void card(double x, double y, double w, double h) {
        round(x, y + 1.5, w, h, 12, 0x0F0F172A);
        round(x, y, w, h, 12, col::Card, col::Border);
    }

    void line(double x1, double y1, double x2, double y2, ARGB c, double width = 1.0) {
        Gdiplus::Pen pen(Gdiplus::Color(c), S(width));
        pen.SetStartCap(Gdiplus::LineCapRound);
        pen.SetEndCap(Gdiplus::LineCapRound);
        g.DrawLine(&pen, S(x1), S(y1), S(x2), S(y2));
    }

    void circle(double cx, double cy, double r, ARGB fill, ARGB stroke = 0, double width = 1.0) {
        if (fill) { Gdiplus::SolidBrush b{ Gdiplus::Color(fill) }; g.FillEllipse(&b, S(cx - r), S(cy - r), S(2 * r), S(2 * r)); }
        if (stroke) { Gdiplus::Pen pen(Gdiplus::Color(stroke), S(width)); g.DrawEllipse(&pen, S(cx - r), S(cy - r), S(2 * r), S(2 * r)); }
    }

    void text(const std::wstring& s, double x, double y, double w, double h, float size, ARGB c, bool bold = false,
              Align a = Align::Left, bool wrap = false, bool middle = true) {
        Gdiplus::StringFormat f;
        f.SetAlignment(a == Align::Left ? Gdiplus::StringAlignmentNear : (a == Align::Center ? Gdiplus::StringAlignmentCenter : Gdiplus::StringAlignmentFar));
        f.SetLineAlignment(middle ? Gdiplus::StringAlignmentCenter : Gdiplus::StringAlignmentNear);
        f.SetTrimming(Gdiplus::StringTrimmingEllipsisCharacter);
        if (!wrap) f.SetFormatFlags(Gdiplus::StringFormatFlagsNoWrap);
        Gdiplus::SolidBrush b{ Gdiplus::Color(c) };
        const Gdiplus::RectF r(S(x), S(y), S(w), S(h));
        g.DrawString(s.c_str(), -1, &font(size, bold), r, &f, &b);
    }

    double measure(const std::wstring& s, float size, bool bold = false) {
        Gdiplus::RectF out;
        g.MeasureString(s.c_str(), -1, &font(size, bold), Gdiplus::PointF(0, 0), &out);
        return out.Width / k_;
    }

    // Pill shaped badge; returns its width.
    double pill(const std::wstring& s, double x, double y, double h, ARGB fg, ARGB bg, float size = 12.5f) {
        const double w = measure(s, size, true) + 18;
        round(x, y, w, h, h / 2, bg);
        text(s, x, y, w, h, size, fg, true, Align::Center);
        return w;
    }

    void progress(double x, double y, double w, double h, double frac, ARGB c) {
        round(x, y, w, h, h / 2, col::Track);
        if (frac > 0.0) round(x, y, std::max(h, w * std::min(frac, 1.0)), h, h / 2, c);
    }

private:
    static void roundPath(Gdiplus::GraphicsPath& p, float x, float y, float w, float h, float r) {
        r = std::min({ r, w / 2, h / 2 });
        const float d = 2 * r;
        if (d <= 0.5f) { p.AddRectangle(Gdiplus::RectF(x, y, w, h)); return; }
        p.AddArc(x, y, d, d, 180, 90);
        p.AddArc(x + w - d, y, d, d, 270, 90);
        p.AddArc(x + w - d, y + h - d, d, d, 0, 90);
        p.AddArc(x, y + h - d, d, d, 90, 90);
        p.CloseFigure();
    }

    float k_;
    std::map<int, std::unique_ptr<Gdiplus::Font>> fonts_;
};

} // namespace ui
