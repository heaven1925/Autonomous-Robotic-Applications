// Probe.h - "is the right device on this COM port?" Listens to a port for up
// to a few seconds and recognises what is talking. It only READS - nothing is
// sent, so the robot never moves.
//
//   Kobuki  : streams feedback packets on its own (~50 Hz): header AA 55,
//             XOR checksum (KobukiProtocol), basic sensor data -> battery voltage
//   Arduino : sensor.ino prints {"l":..,"f":..,"r":..} lines at ~10 Hz -> front
//             range and the line rate. Plain numbers = another sketch is loaded.
//   Nothing : port opened but silent (power off, wrong cable, board hung)
//   Busy    : the port is open in another program (Arduino IDE, Explore_*.exe)
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "KobukiProtocol.h"

#include <chrono>
#include <cstdio>
#include <string>

namespace probe {

enum class Kind { Untested, Testing, Kobuki, Arduino, WrongSketch, Silent, Busy, Missing, Unknown };

struct Result {
    Kind kind = Kind::Untested;
    double battery = 0.0;   // V (Kobuki)
    int frontCm = -2;       // last front reading (Arduino), -1 = no echo
    double hz = 0.0;        // packets / lines per second
    // Kobuki core sensors of the last packet (pre-flight check)
    int bumper = 0, wheelDrop = 0, cliff = 0, charger = 0;
    // Arduino: sensor.ino v2 sends the echo time ("us"); lines without an echo
    bool hasUs = false;
    int lines = 0, noEcho = 0;
    std::wstring detail;    // short text for the screen
};

inline std::wstring fmt(const wchar_t* f, double v) {
    wchar_t b[64];
    swprintf(b, 64, f, v);
    return b;
}

// Listen to `port` (e.g. L"COM6") for at most `seconds`.
inline Result listen(const std::wstring& port, double seconds = 3.5) {
    Result r;
    const std::wstring path = L"\\\\.\\" + port;
    const HANDLE h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD e = GetLastError();
        if (e == ERROR_ACCESS_DENIED || e == ERROR_SHARING_VIOLATION) { r.kind = Kind::Busy; r.detail = L"Port başka bir programda açık"; }
        else { r.kind = Kind::Missing; r.detail = L"Port bulunamadı (takılı değil?)"; }
        return r;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;   // as the programs do: do not reset the Arduino
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = dcb.fOutX = dcb.fInX = FALSE;
    SetCommState(h, &dcb);
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 50;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR);

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const auto since = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    kobuki::KobukiProtocol protocol;
    std::string line;
    int kobukiPackets = 0, jsonLines = 0, plainLines = 0;
    double firstKobuki = -1.0, lastKobuki = 0.0, firstJson = -1.0, lastJson = 0.0;
    std::size_t bytes = 0;
    while (since() < seconds) {
        std::uint8_t buf[512];
        DWORD n = 0;
        if (!ReadFile(h, buf, sizeof(buf), &n, nullptr)) break;
        if (n == 0) continue;
        bytes += n;
        protocol.append(buf, n);
        for (const auto& payload : protocol.extractPayloads()) {
            kobuki::CoreSensors::Data core;
            if (!kobuki::KobukiProtocol::parseCoreSensors(payload, core)) continue;
            ++kobukiPackets;
            r.battery = core.batteryVoltage();
            r.bumper = core.bumper;
            r.wheelDrop = core.wheel_drop;
            r.cliff = core.cliff;
            r.charger = core.charger;
            if (firstKobuki < 0) firstKobuki = since();
            lastKobuki = since();
        }
        for (DWORD i = 0; i < n; ++i) {
            const char c = static_cast<char>(buf[i]);
            if (c != '\n') { if (c != '\r' && line.size() < 200) line += c; continue; }
            const auto f = line.find("\"f\":");
            if (f != std::string::npos) {
                ++jsonLines;
                r.frontCm = std::atoi(line.c_str() + f + 4);
                if (r.frontCm < 0) ++r.noEcho;
                if (line.find("\"us\":") != std::string::npos) r.hasUs = true;
                if (firstJson < 0) firstJson = since();
                lastJson = since();
            } else if (!line.empty() && line.find_first_not_of("0123456789 -\t") == std::string::npos) {
                ++plainLines;
            }
            line.clear();
        }
        if (kobukiPackets >= 25 || jsonLines >= 6) break;   // recognised - no need to wait longer
    }
    CloseHandle(h);

    if (kobukiPackets >= 3) {
        r.kind = Kind::Kobuki;
        r.hz = lastKobuki > firstKobuki ? (kobukiPackets - 1) / (lastKobuki - firstKobuki) : 0.0;
        r.detail = L"Kobuki yanıt veriyor · batarya " + fmt(L"%.1f V", r.battery);
        if (r.battery > 0.0 && r.battery < 13.5) r.detail += L" (düşük, şarj edin)";
    } else if (jsonLines >= 2) {
        r.kind = Kind::Arduino;
        r.lines = jsonLines;
        r.hz = lastJson > firstJson ? (jsonLines - 1) / (lastJson - firstJson) : 0.0;
        r.detail = L"Arduino sensör akışı · ön " + (r.frontCm >= 0 ? std::to_wstring(r.frontCm) + L" cm" : std::wstring(L"yankı yok")) +
                   L" · " + fmt(L"%.0f Hz", r.hz);
    } else if (plainLines >= 2) {
        r.kind = Kind::WrongSketch;
        r.detail = L"Arduino başka bir program çalıştırıyor (JSON yok) - sensor.ino yükleyin";
    } else if (bytes == 0) {
        r.kind = Kind::Silent;
        r.detail = L"Veri gelmiyor - güç / kablo / kart?";
    } else {
        r.kind = Kind::Unknown;
        r.detail = L"Tanınmayan veri (" + std::to_wstring(bytes) + L" bayt)";
    }
    return r;
}

} // namespace probe
