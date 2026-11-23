// GdiCanvas.h - small Windows GDI toolkit shared by the map examples
// (09_Mapping, 10_AStarPathPlanning). No external libraries.
//
//   Canvas         - off-screen 32-bit bitmap with a GDI device context:
//                    rectangles, lines, circles, text, direct pixels, PNG export.
//   MapWindow      - plain window that shows a Canvas and reports key presses.
//   GdiPlusSession - starts GDI+ (needed only for the PNG encoder).
//
// Everything is drawn into the off-screen Canvas; the window only shows that
// bitmap and the PNG export saves it, so the saved picture is exactly what was
// on screen - and a program also works without any window.
// Link with gdiplus (target_link_libraries(... gdiplus)).
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <objidl.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// GDI+ (PNG encoder) uses min/max, which NOMINMAX removes.
namespace Gdiplus { using std::min; using std::max; }
#include <gdiplus.h>

namespace view {

// GDI+ must be started once before any PNG is saved.
class GdiPlusSession {
public:
    GdiPlusSession() {
        Gdiplus::GdiplusStartupInput input;
        Gdiplus::GdiplusStartup(&token_, &input, nullptr);
    }
    ~GdiPlusSession() { Gdiplus::GdiplusShutdown(token_); }
    GdiPlusSession(const GdiPlusSession&) = delete;
    GdiPlusSession& operator=(const GdiPlusSession&) = delete;

private:
    ULONG_PTR token_ = 0;
};

// Off-screen 32-bit bitmap with a GDI device context to draw into.
class Canvas {
public:
    Canvas(int w, int h) : w_(w), h_(h) {
        BITMAPINFO bi{};
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = w;
        bi.bmiHeader.biHeight = -h;  // top-down rows
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        bi.bmiHeader.biCompression = BI_RGB;
        dc_ = CreateCompatibleDC(nullptr);
        void* bits = nullptr;
        bitmap_ = CreateDIBSection(dc_, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        bits_ = static_cast<std::uint32_t*>(bits);
        oldBitmap_ = SelectObject(dc_, bitmap_);
        font_ = makeFont(-15, FW_NORMAL);
        fontBold_ = makeFont(-17, FW_SEMIBOLD);
        oldFont_ = SelectObject(dc_, font_);
        SetBkMode(dc_, TRANSPARENT);
    }

    ~Canvas() {
        SelectObject(dc_, oldFont_);
        SelectObject(dc_, oldBitmap_);
        DeleteObject(font_);
        DeleteObject(fontBold_);
        DeleteObject(bitmap_);
        DeleteDC(dc_);
    }

    Canvas(const Canvas&) = delete;
    Canvas& operator=(const Canvas&) = delete;

    int width() const { return w_; }
    int height() const { return h_; }
    HDC dc() const { return dc_; }

    void fill(int x0, int y0, int x1, int y1, COLORREF c) {
        RECT r{ x0, y0, x1, y1 };
        SetDCBrushColor(dc_, c);
        FillRect(dc_, &r, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
    }

    void frame(int x0, int y0, int x1, int y1, COLORREF c) {
        RECT r{ x0, y0, x1, y1 };
        SetDCBrushColor(dc_, c);
        FrameRect(dc_, &r, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
    }

    void line(int x0, int y0, int x1, int y1, COLORREF c, int width = 1) {
        const HPEN pen = CreatePen(PS_SOLID, width, c);
        const HGDIOBJ old = SelectObject(dc_, pen);
        MoveToEx(dc_, x0, y0, nullptr);
        LineTo(dc_, x1, y1);
        SelectObject(dc_, old);
        DeleteObject(pen);
    }

    void polyline(const std::vector<POINT>& pts, COLORREF c, int width) {
        if (pts.size() < 2) return;
        const HPEN pen = CreatePen(PS_SOLID, width, c);
        const HGDIOBJ old = SelectObject(dc_, pen);
        Polyline(dc_, pts.data(), static_cast<int>(pts.size()));
        SelectObject(dc_, old);
        DeleteObject(pen);
    }

    // Circle outline (fill = false) or filled disc.
    void circle(int cx, int cy, int r, COLORREF c, int width, bool filled) {
        const HPEN pen = CreatePen(PS_SOLID, width, c);
        const HGDIOBJ oldPen = SelectObject(dc_, pen);
        SetDCBrushColor(dc_, c);
        const HGDIOBJ oldBrush = SelectObject(dc_, GetStockObject(filled ? DC_BRUSH : NULL_BRUSH));
        Ellipse(dc_, cx - r, cy - r, cx + r + 1, cy + r + 1);
        SelectObject(dc_, oldBrush);
        SelectObject(dc_, oldPen);
        DeleteObject(pen);
    }

    void text(int x, int y, const std::wstring& s, COLORREF c, bool bold = false) {
        const HGDIOBJ old = SelectObject(dc_, bold ? fontBold_ : font_);
        SetTextColor(dc_, c);
        TextOutW(dc_, x, y, s.c_str(), static_cast<int>(s.size()));
        SelectObject(dc_, old);
    }

    // Direct pixel access (0x00RRGGBB). Call flush() first after GDI drawing.
    void flush() const { GdiFlush(); }
    void pixel(int x, int y, std::uint32_t rgb) {
        if (x >= 0 && y >= 0 && x < w_ && y < h_) bits_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)] = rgb;
    }

    bool savePng(const std::wstring& path) const {
        GdiFlush();
        Gdiplus::Bitmap bitmap(w_, h_, w_ * 4, PixelFormat32bppRGB, reinterpret_cast<BYTE*>(bits_));
        CLSID png{};
        if (FAILED(CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &png))) return false;
        return bitmap.Save(path.c_str(), &png, nullptr) == Gdiplus::Ok;
    }

private:
    static HFONT makeFont(int height, int weight) {
        return CreateFontW(height, 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
    }

    int w_, h_;
    HDC dc_ = nullptr;
    HBITMAP bitmap_ = nullptr;
    HGDIOBJ oldBitmap_ = nullptr, oldFont_ = nullptr;
    HFONT font_ = nullptr, fontBold_ = nullptr;
    std::uint32_t* bits_ = nullptr;
};

// A plain window that shows a Canvas. All calls on the thread that created it.
class MapWindow {
public:
    MapWindow() = default;
    ~MapWindow() { if (hwnd_) DestroyWindow(hwnd_); }
    MapWindow(const MapWindow&) = delete;
    MapWindow& operator=(const MapWindow&) = delete;

    bool create(int w, int h, const wchar_t* title, COLORREF background = RGB(246, 246, 244)) {
        WNDCLASSW wc{};
        wc.lpfnWndProc = &MapWindow::proc;
        wc.hInstance = GetModuleHandleW(nullptr);
        wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
        wc.lpszClassName = L"KobukiMapView";
        RegisterClassW(&wc);

        const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
        RECT r{ 0, 0, w, h };
        AdjustWindowRect(&r, style, FALSE);
        back_ = std::make_unique<Canvas>(w, h);
        back_->fill(0, 0, w, h, background);
        hwnd_ = CreateWindowExW(0, wc.lpszClassName, title, style, CW_USEDEFAULT, CW_USEDEFAULT,
                                r.right - r.left, r.bottom - r.top, nullptr, nullptr, wc.hInstance, this);
        if (!hwnd_) return false;
        // Show it without taking the keyboard away from the console.
        ShowWindow(hwnd_, SW_SHOWNOACTIVATE);
        SetWindowPos(hwnd_, HWND_TOP, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
        return true;
    }

    bool isOpen() const { return hwnd_ != nullptr; }
    bool closedByUser() const { return closed_; }

    void present(const Canvas& c) {
        if (!hwnd_) return;
        BitBlt(back_->dc(), 0, 0, back_->width(), back_->height(), c.dc(), 0, 0, SRCCOPY);
        InvalidateRect(hwnd_, nullptr, FALSE);
    }

    // Handle pending window messages; call regularly.
    void pump() {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    // Last key pressed while the window had the focus (virtual-key code), 0 = none.
    int takeKey() {
        const int k = key_;
        key_ = 0;
        return k;
    }

private:
    static LRESULT CALLBACK proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
        if (msg == WM_NCCREATE) {
            const auto* cs = reinterpret_cast<const CREATESTRUCTW*>(lp);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
        }
        auto* self = reinterpret_cast<MapWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        if (self) {
            switch (msg) {
                case WM_PAINT: {
                    PAINTSTRUCT ps;
                    const HDC dc = BeginPaint(hwnd, &ps);
                    BitBlt(dc, 0, 0, self->back_->width(), self->back_->height(), self->back_->dc(), 0, 0, SRCCOPY);
                    EndPaint(hwnd, &ps);
                    return 0;
                }
                case WM_ERASEBKGND:
                    return 1;
                case WM_KEYDOWN:
                    self->key_ = static_cast<int>(wp);
                    return 0;
                case WM_CLOSE:
                    self->closed_ = true;
                    DestroyWindow(hwnd);
                    return 0;
                case WM_NCDESTROY:
                    self->hwnd_ = nullptr;
                    break;
                default:
                    break;
            }
        }
        return DefWindowProcW(hwnd, msg, wp, lp);
    }

    HWND hwnd_ = nullptr;
    std::unique_ptr<Canvas> back_;
    bool closed_ = false;
    int key_ = 0;
};

// Picture size that fits the screen (the window adds its own frame).
inline void pictureSize(int& w, int& h) {
    w = 1180;
    h = 800;
    RECT work{};
    if (SystemParametersInfoW(SPI_GETWORKAREA, 0, &work, 0)) {
        w = std::clamp(static_cast<int>(work.right - work.left) - 40, 760, w);
        h = std::clamp(static_cast<int>(work.bottom - work.top) - 70, 520, h);
    }
}

} // namespace view
