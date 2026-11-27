// SensorLink.h - keeps the Arduino's COM port open while the calibration panel
// (step 3) is shown: live readings for the screen and the 40 cm calibration,
// and the pin commands of sensor.ino v2.
//
//   data line : {"l":-1,"f":40,"r":-1,"us":2332}   (an old sketch has no "us")
//   "?"       -> {"ver":2,"trig":11,"echo":12}
//   "PINS t e"-> the same line with the new pins, or {"err":"pins"}
// The board is asked "?" after its first data line (it may have just rebooted)
// up to three times; no answer = an old sketch without pin commands.
//
// A background thread reads the port; it posts `msg` to the window when there
// is something new (at most one message waiting at a time) and the window
// takes a snapshot().
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sensor {

struct Sample {
    std::uint64_t seq = 0;
    int cm = -1;     // "f": whole cm, -1 = no echo
    long us = -2;    // "us": echo time, -1 = no echo, -2 = not sent (old sketch)
    bool echo() const { return us >= 0 || (us == -2 && cm >= 0); }
};

enum class State { Closed, Opening, Waiting, Streaming, Error };

struct Snapshot {
    State state = State::Closed;
    std::wstring port, error;
    std::vector<Sample> recent;      // newest last
    std::uint64_t seq = 0;           // data lines so far
    double hz = 0.0;
    bool hasUs = false;
    bool cfgKnown = false, cfgUnsupported = false;
    int ver = 0, trig = -1, echo = -1;
    int pinErrors = 0, cfgReplies = 0;
    int otherLines = 0;              // lines that are neither data nor config
};

class Link {
public:
    ~Link() { close(); }

    bool open(const std::wstring& port, HWND hwnd, UINT msg) {
        close();
        {
            std::lock_guard<std::mutex> lock(mu_);
            snap_ = Snapshot{};
            snap_.port = port;
            snap_.state = State::Opening;
            outbox_.clear();
            times_.clear();
        }
        hwnd_ = hwnd;
        msg_ = msg;
        stop_ = false;
        pending_ = false;
        thread_ = std::thread([this, port] { loop(port); });
        return true;
    }

    void close() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        std::lock_guard<std::mutex> lock(mu_);
        snap_.state = State::Closed;
    }

    bool isOpen() const { return thread_.joinable() && !stop_; }
    std::wstring port() const {
        std::lock_guard<std::mutex> lock(mu_);
        return snap_.port;
    }

    void send(const std::string& line) {
        std::lock_guard<std::mutex> lock(mu_);
        outbox_.push_back(line + "\n");
    }

    // Called by the window when it handles `msg`.
    Snapshot snapshot() {
        pending_ = false;
        std::lock_guard<std::mutex> lock(mu_);
        return snap_;
    }

private:
    using Clock = std::chrono::steady_clock;

    void notify() {
        if (!pending_.exchange(true)) PostMessageW(hwnd_, msg_, 0, 0);
    }

    void fail(const std::wstring& text) {
        {
            std::lock_guard<std::mutex> lock(mu_);
            snap_.state = State::Error;
            snap_.error = text;
        }
        notify();
    }

    static int field(const std::string& line, const char* key, int def) {
        const auto p = line.find(key);
        return p == std::string::npos ? def : std::atoi(line.c_str() + p + std::char_traits<char>::length(key));
    }

    void parse(const std::string& line) {
        std::lock_guard<std::mutex> lock(mu_);
        if (line.find("\"f\":") != std::string::npos) {
            Sample s;
            s.seq = ++snap_.seq;
            s.cm = field(line, "\"f\":", -1);
            if (line.find("\"us\":") != std::string::npos) {
                s.us = std::atol(line.c_str() + line.find("\"us\":") + 5);
                snap_.hasUs = true;
            }
            snap_.recent.push_back(s);
            if (snap_.recent.size() > 300) snap_.recent.erase(snap_.recent.begin(), snap_.recent.begin() + 100);
            snap_.state = State::Streaming;
            times_.push_back(Clock::now());
            if (times_.size() > 21) times_.pop_front();
            if (times_.size() >= 3)
                snap_.hz = static_cast<double>(times_.size() - 1) /
                           std::max(1e-3, std::chrono::duration<double>(times_.back() - times_.front()).count());
        } else if (line.find("\"trig\":") != std::string::npos) {
            snap_.ver = field(line, "\"ver\":", 0);
            snap_.trig = field(line, "\"trig\":", -1);
            snap_.echo = field(line, "\"echo\":", -1);
            snap_.cfgKnown = true;
            snap_.cfgUnsupported = false;
            ++snap_.cfgReplies;
        } else if (line.find("\"err\"") != std::string::npos) {
            ++snap_.pinErrors;
        } else if (!line.empty()) {
            ++snap_.otherLines;
        }
    }

    void loop(std::wstring port) {
        const std::wstring path = L"\\\\.\\" + port;
        const HANDLE h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            const DWORD e = GetLastError();
            fail(e == ERROR_ACCESS_DENIED || e == ERROR_SHARING_VIOLATION ? L"Port başka bir programda açık" : L"Port açılamadı");
            return;
        }
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;   // as Probe.h and the programs: do not reset the Arduino
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = dcb.fOutX = dcb.fInX = FALSE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = MAXDWORD;
        to.ReadTotalTimeoutConstant = 50;
        to.WriteTotalTimeoutConstant = 300;
        SetCommTimeouts(h, &to);
        PurgeComm(h, PURGE_RXCLEAR);
        {
            std::lock_guard<std::mutex> lock(mu_);
            snap_.state = State::Waiting;
        }
        notify();

        const auto t0 = Clock::now();
        const auto since = [](Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); };
        Clock::time_point lastQuery{};
        int queries = 0;
        std::uint64_t seen = 0;
        std::string line;
        bool warnedSilent = false;
        while (!stop_) {
            // Commands waiting to go out, and the "?" query.
            std::vector<std::string> out;
            bool streaming = false, cfgKnown = false;
            {
                std::lock_guard<std::mutex> lock(mu_);
                out.swap(outbox_);
                streaming = snap_.seq > 0;
                cfgKnown = snap_.cfgKnown;
            }
            if (streaming && !cfgKnown) {
                if (queries < 3 && (queries == 0 || since(lastQuery) > 1.2)) {
                    out.insert(out.begin(), "?\n");
                    ++queries;
                    lastQuery = Clock::now();
                } else if (queries == 3 && since(lastQuery) > 1.5) {
                    {
                        std::lock_guard<std::mutex> lock(mu_);
                        snap_.cfgUnsupported = true;
                    }
                    queries = 4;   // stop asking
                    notify();
                }
            }
            for (const std::string& s : out) {
                DWORD w = 0;
                WriteFile(h, s.data(), static_cast<DWORD>(s.size()), &w, nullptr);
            }
            if (!out.empty()) notify();

            std::uint8_t buf[512];
            DWORD n = 0;
            if (!ReadFile(h, buf, sizeof(buf), &n, nullptr)) {
                CloseHandle(h);
                fail(L"Okuma hatası (kablo çıktı mı?)");
                return;
            }
            for (DWORD i = 0; i < n; ++i) {
                const char c = static_cast<char>(buf[i]);
                if (c == '\n') { parse(line); line.clear(); }
                else if (c != '\r' && line.size() < 200) line += c;
            }
            std::uint64_t seq;
            {
                std::lock_guard<std::mutex> lock(mu_);
                seq = snap_.seq + static_cast<std::uint64_t>(snap_.cfgReplies + snap_.pinErrors);
            }
            if (seq != seen) { seen = seq; notify(); }
            if (!warnedSilent && seq == 0 && since(t0) > 4.0) {   // the board may reboot for ~2 s first
                warnedSilent = true;
                {
                    std::lock_guard<std::mutex> lock(mu_);
                    snap_.error = L"Veri gelmiyor - güç / kablo / sketch?";
                }
                notify();
            }
        }
        CloseHandle(h);
    }

    mutable std::mutex mu_;
    Snapshot snap_;
    std::vector<std::string> outbox_;
    std::deque<Clock::time_point> times_;
    std::thread thread_;
    std::atomic<bool> stop_{ true }, pending_{ false };
    HWND hwnd_ = nullptr;
    UINT msg_ = 0;
};

} // namespace sensor
