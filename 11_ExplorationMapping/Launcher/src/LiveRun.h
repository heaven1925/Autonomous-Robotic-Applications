// LiveRun.h - runs one Explore_*.exe with --ipc (no window of its own),
// reads its state lines (common/Ipc.h) and keeps the latest frame for the
// launcher to draw. stop() writes "stop" to its stdin: the program stops the
// robot, saves the map and exits by itself.
//
// The state lines are kept (record) so the launcher can save them next to the
// map as <map>.replay; Replay.h plays such a file back with the same Parser.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace live {

struct CellXY { int x = 0, y = 0; };
struct PointXY { double x = 0.0, y = 0.0; };
struct Mark { CellXY c; int kind = 0; std::string label; };

struct State {
    // header
    bool header = false;
    int cols = 0, rows = 0, base = 0;
    double cell = 0.4, heading = 0.0;
    CellXY start;
    bool yUp = false;
    std::string methodId, methodName, ref;
    // latest frame
    bool frame = false;
    double t = 0.0, x = 0.0, y = 0.0, h = 0.0, driven = 0.0;
    std::string phase;
    int goals = 0, scans = 0, bumps = 0, sonarCm = -1;
    long readings = 0;
    bool hasTarget = false, lookAround = false, verifying = false;
    CellXY target, approach;
    std::vector<int> p;              // P(occupied) in percent
    std::string states;              // 'e' / 'o' / 'u' per cell
    std::vector<CellXY> frontier, path;
    std::vector<PointXY> tour;
    std::vector<Mark> marks;
    bool beam = false, echo = false;
    double bsx = 0, bsy = 0, bangle = 0, brange = 0;
    std::vector<std::string> info;
    // events / end
    std::deque<std::string> events, console;
    bool done = false, completed = false;
    std::string finish, result, saved;
    double endTime = 0.0, endDriven = 0.0;
    bool staticMap = false;   // history: only the final grid is known (no robot, no path)
};

// Turns the lines of common/Ipc.h into a State. feed() returns true when a
// frame is complete (@E).
struct Parser {
    State pending;                  // frame being assembled
    std::size_t maxEvents = 200;    // 0 = keep all (replay)

    static std::vector<double> numbers(std::istringstream& in) {
        std::vector<double> v;
        double d;
        while (in >> d) v.push_back(d);
        return v;
    }

    bool feed(const std::string& line, State& state) {
        State& f = pending;
        if (line.size() < 2 || line[0] != '@') {
            if (line.find_first_not_of(" \t") == std::string::npos) return false;
            state.console.push_back(line);
            while (state.console.size() > 60) state.console.pop_front();
            return false;
        }
        const auto sp = line.find(' ');
        const std::string key = line.substr(1, sp == std::string::npos ? std::string::npos : sp - 1);
        const std::string rest = sp == std::string::npos ? std::string() : line.substr(sp + 1);
        std::istringstream in(rest);
        if (key == "ARENA") {
            int yUp = 0;
            in >> state.cols >> state.rows >> state.cell >> state.start.x >> state.start.y >> yUp >> state.base >> state.heading;
            state.yUp = yUp != 0;
            state.header = true;
        } else if (key == "METHOD") {
            in >> state.methodId;
            std::getline(in >> std::ws, state.methodName);
        } else if (key == "REF") {
            state.ref = rest;
        } else if (key == "F") {
            int hasT = 0, look = 0, ver = 0;
            in >> f.t >> f.phase >> f.x >> f.y >> f.h >> f.driven >> f.goals >> f.scans >> f.bumps >> f.readings >> f.sonarCm >> hasT >> look >>
                f.target.x >> f.target.y >> f.approach.x >> f.approach.y >> ver;
            f.hasTarget = hasT != 0;
            f.lookAround = look != 0;
            f.verifying = ver != 0;
            f.info.clear();
            f.beam = false;
        } else if (key == "P") {
            f.p.clear();
            int v;
            while (in >> v) f.p.push_back(v);
        } else if (key == "S") {
            in >> f.states;
        } else if (key == "FR" || key == "PA") {
            const auto v = numbers(in);
            auto& out = key == "FR" ? f.frontier : f.path;
            out.clear();
            for (std::size_t i = 0; i + 1 < v.size(); i += 2) out.push_back({ static_cast<int>(v[i]), static_cast<int>(v[i + 1]) });
        } else if (key == "TO") {
            const auto v = numbers(in);
            f.tour.clear();
            for (std::size_t i = 0; i + 1 < v.size(); i += 2) f.tour.push_back({ v[i], v[i + 1] });
        } else if (key == "MK") {
            f.marks.clear();
            Mark m;
            while (in >> m.c.x >> m.c.y >> m.kind >> m.label) {
                if (m.label == "-") m.label.clear();
                f.marks.push_back(m);
            }
        } else if (key == "BM") {
            int echo = 0;
            in >> f.bsx >> f.bsy >> f.bangle >> f.brange >> echo;
            f.echo = echo != 0;
            f.beam = true;
        } else if (key == "I") {
            f.info.push_back(rest);
        } else if (key == "E") {
            State& s = state;
            s.frame = true;
            s.t = f.t; s.x = f.x; s.y = f.y; s.h = f.h; s.driven = f.driven; s.phase = f.phase;
            s.goals = f.goals; s.scans = f.scans; s.bumps = f.bumps; s.readings = f.readings; s.sonarCm = f.sonarCm;
            s.hasTarget = f.hasTarget; s.lookAround = f.lookAround; s.verifying = f.verifying;
            s.target = f.target; s.approach = f.approach;
            s.p = f.p; s.states = f.states; s.frontier = f.frontier; s.path = f.path; s.tour = f.tour; s.marks = f.marks;
            s.beam = f.beam; s.echo = f.echo; s.bsx = f.bsx; s.bsy = f.bsy; s.bangle = f.bangle; s.brange = f.brange;
            s.info = f.info;
            return true;
        } else if (key == "EV") {
            state.events.push_back(rest);
            while (maxEvents && state.events.size() > maxEvents) state.events.pop_front();
        } else if (key == "DONE") {
            int c = 0;
            in >> state.finish >> c >> state.endTime >> state.endDriven;
            state.completed = c != 0;
            state.done = true;
        } else if (key == "RESULT") {
            state.result = rest;
        } else if (key == "SAVED") {
            state.saved = rest;
        }
        return false;
    }
};

class Run {
public:
    ~Run() { detach(); }

    bool running() const { return running_; }
    DWORD exitCode() const { return exitCode_; }

    // notify: window that gets `msgFrame` (new frame, throttled) and `msgEnd` (process ended).
    bool start(const std::wstring& exe, const std::wstring& args, const std::wstring& workDir, HWND notify, UINT msgFrame, UINT msgEnd) {
        if (running_) return false;
        detach();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State{};
            record_.clear();
        }
        SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
        HANDLE outRead = nullptr, outWrite = nullptr, inRead = nullptr, inWrite = nullptr;
        if (!CreatePipe(&outRead, &outWrite, &sa, 0)) return false;
        if (!CreatePipe(&inRead, &inWrite, &sa, 0)) { CloseHandle(outRead); CloseHandle(outWrite); return false; }
        SetHandleInformation(outRead, HANDLE_FLAG_INHERIT, 0);   // our ends are not inherited
        SetHandleInformation(inWrite, HANDLE_FLAG_INHERIT, 0);
        STARTUPINFOW si{};
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = inRead;
        si.hStdOutput = outWrite;
        si.hStdError = outWrite;
        PROCESS_INFORMATION pi{};
        std::wstring cmd = L"\"" + exe + L"\" " + args;
        const BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, workDir.c_str(), &si, &pi);
        CloseHandle(outWrite);
        CloseHandle(inRead);
        if (!ok) { CloseHandle(outRead); CloseHandle(inWrite); return false; }
        CloseHandle(pi.hThread);
        process_ = pi.hProcess;
        stdin_ = inWrite;
        running_ = true;
        exitCode_ = STILL_ACTIVE;
        reader_ = std::thread([this, outRead, notify, msgFrame, msgEnd] { readLoop(outRead, notify, msgFrame, msgEnd); });
        return true;
    }

    // Polite stop: the program stops the robot and saves the map.
    void stop() {
        if (!running_ || !stdin_) return;
        const char cmd[] = "stop\n";
        DWORD written = 0;
        WriteFile(stdin_, cmd, sizeof(cmd) - 1, &written, nullptr);
    }
    void kill() {
        if (running_ && process_) TerminateProcess(process_, 4);
    }

    State snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        frameQueued_ = false;
        return state_;
    }

    // The run's state lines as a replay file (Replay.h).
    bool saveRecording(const std::wstring& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (record_.empty()) return false;
        std::ofstream out(path.c_str(), std::ios::binary);
        if (!out) return false;
        for (const std::string& l : record_) out << l << "\n";
        return static_cast<bool>(out);
    }

    void detach() {
        if (reader_.joinable()) reader_.join();
        if (stdin_) { CloseHandle(stdin_); stdin_ = nullptr; }
        if (process_) { CloseHandle(process_); process_ = nullptr; }
    }

private:
    void readLoop(HANDLE out, HWND notify, UINT msgFrame, UINT msgEnd) {
        std::string buf, line;
        Parser parser;
        char chunk[4096];
        DWORD n = 0;
        while (ReadFile(out, chunk, sizeof(chunk), &n, nullptr) && n > 0) {
            buf.append(chunk, n);
            std::size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos) {
                line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                bool post = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!line.empty() && line[0] == '@') record_.push_back(line);
                    if (parser.feed(line, state_) && !frameQueued_) { frameQueued_ = true; post = true; }
                }
                if (post) PostMessageW(notify, msgFrame, 0, 0);
            }
        }
        CloseHandle(out);
        WaitForSingleObject(process_, 5000);
        DWORD code = 0;
        GetExitCodeProcess(process_, &code);
        exitCode_ = code;
        running_ = false;
        PostMessageW(notify, msgEnd, code, 0);
    }

    std::mutex mutex_;
    State state_;
    std::vector<std::string> record_;   // the run's state lines, for <map>.replay
    bool frameQueued_ = false;
    std::thread reader_;
    HANDLE process_ = nullptr, stdin_ = nullptr;
    std::atomic<bool> running_{ false };
    std::atomic<DWORD> exitCode_{ 0 };
};

} // namespace live
