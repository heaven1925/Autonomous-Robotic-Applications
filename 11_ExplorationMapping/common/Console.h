// Console.h - small console helpers shared by the exploration programs:
// Ctrl+C flag, key polling (console + map window), dates, wide strings.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "GdiCanvas.h"

#include <atomic>
#include <cctype>
#include <conio.h>
#include <ctime>
#include <string>

namespace app {

inline std::atomic<bool> g_quit{ false };

inline BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) { g_quit = true; return TRUE; }
    return FALSE;
}

inline std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

inline std::string dateText(bool seconds = false) {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), seconds ? "%Y-%m-%d %H:%M:%S" : "%Y-%m-%d %H:%M", &tm);
    return buf;
}

inline std::wstring wide(const std::string& s) { return std::wstring(s.begin(), s.end()); }

// Key from the map window or the console (upper case, VK_RETURN for Enter), 0 = none.
inline int pollKey(view::MapWindow& window) {
    int k = window.takeKey();
    if (k != 0) return k;
    if (!_kbhit()) return 0;
    k = _getch();
    if (k == 0 || k == 224) { (void)_getch(); return 0; }
    return k == '\r' ? VK_RETURN : std::toupper(k);
}

// Console only: true on Enter, false on Q / Esc / Ctrl+C.
inline bool waitEnter() {
    while (true) {
        if (g_quit) return false;
        if (_kbhit()) {
            const int k = _getch();
            if (k == '\r') return true;
            if (k == 'q' || k == 'Q' || k == 27) return false;
        }
        Sleep(20);
    }
}

} // namespace app
