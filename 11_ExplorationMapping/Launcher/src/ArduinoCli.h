// ArduinoCli.h - compile and upload arduino/sensor/sensor.ino from the launcher
// with arduino-cli ("Sketch'i yükle" in step 3).
//
// arduino-cli is looked for in:
//   <repo>\.tools\arduino-cli\arduino-cli.exe   (arduino\setup-arduino-cli.ps1 puts it there)
//   PATH
//   Arduino IDE 2 (it ships an arduino-cli):
//     %LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
//     %ProgramFiles%\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace arduino {

struct Board {
    const wchar_t* name;
    const wchar_t* fqbn;
};
inline const std::vector<Board>& boards() {
    static const std::vector<Board> b = {
        { L"Uno", L"arduino:avr:uno" },
        { L"Nano", L"arduino:avr:nano" },
        { L"Nano (eski bootloader)", L"arduino:avr:nano:cpu=atmega328old" },
        { L"Mega 2560", L"arduino:avr:mega" },
    };
    return b;
}

inline std::wstring env(const wchar_t* name) {
    wchar_t b[1024];
    const DWORD n = GetEnvironmentVariableW(name, b, 1024);
    return n > 0 && n < 1024 ? std::wstring(b, n) : std::wstring();
}

// Empty path = not found.
inline std::filesystem::path findCli(const std::filesystem::path& repoRoot) {
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<fs::path> candidates = { repoRoot / ".tools" / "arduino-cli" / "arduino-cli.exe" };
    wchar_t found[MAX_PATH * 2];
    if (SearchPathW(nullptr, L"arduino-cli.exe", nullptr, MAX_PATH * 2, found, nullptr) > 0) candidates.push_back(found);
    const wchar_t* ide = L"Arduino IDE\\resources\\app\\lib\\backend\\resources\\arduino-cli.exe";
    if (const std::wstring l = env(L"LOCALAPPDATA"); !l.empty()) candidates.push_back(fs::path(l) / "Programs" / ide);
    if (const std::wstring p = env(L"ProgramFiles"); !p.empty()) candidates.push_back(fs::path(p) / ide);
    for (const fs::path& c : candidates)
        if (fs::exists(c, ec)) return c;
    return {};
}

// Runs a command line without a window; every output line goes to onLine.
// Returns the exit code, -1 if it could not be started, -2 if cancelled.
inline int runCaptured(std::wstring cmd, const std::wstring& workDir, const std::function<void(const std::string&)>& onLine,
                       const std::atomic<bool>& cancel) {
    SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
    HANDLE rd = nullptr, wr = nullptr;
    if (!CreatePipe(&rd, &wr, &sa, 0)) return -1;
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
    const HANDLE nul = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, 0, nullptr);
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nul;
    si.hStdOutput = wr;
    si.hStdError = wr;
    PROCESS_INFORMATION pi{};
    const BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                   workDir.empty() ? nullptr : workDir.c_str(), &si, &pi);
    CloseHandle(wr);
    if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
    if (!ok) { CloseHandle(rd); return -1; }
    CloseHandle(pi.hThread);
    std::string buf;
    char chunk[2048];
    bool cancelled = false;
    while (true) {
        DWORD avail = 0;
        if (!PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr)) break;   // pipe closed: the process ended
        if (cancel && !cancelled) { TerminateProcess(pi.hProcess, 9); cancelled = true; }
        if (avail == 0) { Sleep(30); continue; }
        DWORD n = 0;
        if (!ReadFile(rd, chunk, sizeof(chunk), &n, nullptr) || n == 0) break;
        buf.append(chunk, n);
        std::size_t nl;
        while ((nl = buf.find_first_of("\r\n")) != std::string::npos) {
            if (nl > 0) onLine(buf.substr(0, nl));
            buf.erase(0, nl + 1);
        }
    }
    if (!buf.empty()) onLine(buf);
    CloseHandle(rd);
    WaitForSingleObject(pi.hProcess, 5000);
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hProcess);
    return cancelled ? -2 : static_cast<int>(code);
}

} // namespace arduino
