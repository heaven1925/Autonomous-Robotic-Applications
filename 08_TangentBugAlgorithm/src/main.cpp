// Ornek 08 - Tangent Bug navigasyonu (Kobuki + Arduino + 3x HC-SR04).
//
//   Gercek robot  : TangentBug.exe <Kobuki COM portu> <Arduino COM portu>
//   Simulasyon    : TangentBug.exe --sim                (donanim gerekmez)
//   Sensor testi  : TangentBug.exe --sensors <Arduino COM portu>  (Kobuki gerekmez)
//   Kolay yol     : .\run.ps1 8   /   .\run.ps1 8 -Sim   /   .\run.ps1 8 -SensorTest
//
// Program basladigi nokta (0, 0), o anki burun yonu +x kabul edilir
// (odometri ve jiroskop sifirlanir). Klavyeden "x y" (metre) hedefleri
// sirayla istenir; robot her hedefe TangentBugPlanner ile gider, sonra bir
// sonraki hedefi sorar. Koordinatlar hep baslangic noktasina goredir.
//
//   q           -> cikis
//   Ctrl+C      -> hareket sirasinda: mevcut hedefi iptal et, robotu durdur
//
// Guvenlik (planlayicidan bagimsiz): tampon / ucurum / sensor akisi kesilmesi
// -> robot durur ve program cikar.
#include "RobotIO.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

// --- Robota ozgu ayarlar (montaj olculeri) --------------------------------
constexpr double Deg = tbug::Pi / 180.0;
constexpr double SensorOffsetM = 0.11;   // robot merkezinden sensorlere uzaklik (m)
constexpr double SideBeamDeg = 60.0;     // yan sensorlerin one gore acisi (derece)
constexpr double RobotRadiusM = 0.18;    // Kobuki govde yaricapi (cap 351 mm)

constexpr double HudPeriodS = 0.25; // canli durum satirinin yenilenme periyodu (s)
constexpr double MaxDtS = 0.25;     // planlayiciya tek adimda verilen azami dt (paketler ~0.1 s'de bir)

std::atomic<bool> g_cancel{false};

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) { g_cancel = true; return TRUE; }
    return FALSE;
}

const char* stateName(tbug::State s) {
    switch (s) {
        case tbug::State::Init: return "INIT";
        case tbug::State::MotionToGoal: return "MOTION_TO_GOAL";
        case tbug::State::MoveToEndpoint: return "MOVE_TO_ENDPOINT";
        case tbug::State::FollowBoundary: return "FOLLOW_BOUNDARY";
        case tbug::State::Recovery: return "RECOVERY";
        case tbug::State::GoalReached: return "GOAL_REACHED";
        case tbug::State::Fault: return "FAULT";
    }
    return "?";
}

// "2 0", "2, 0", "2 , 0" ve ondalik virgul ("0,5 1") kabul edilir; tek parca
// "2,0" belirsiz oldugu icin reddedilir (2.0 mi, yoksa 2 ve 0 mi?).
bool parseTarget(const std::string& line, double& x, double& y) {
    std::istringstream iss(line);
    std::vector<double> vals;
    std::string tok;
    while (iss >> tok) {
        while (!tok.empty() && tok.back() == ',') tok.pop_back();
        if (tok.empty()) continue;
        std::replace(tok.begin(), tok.end(), ',', '.');
        try {
            std::size_t used = 0;
            vals.push_back(std::stod(tok, &used));
            if (used != tok.size()) return false;
        } catch (...) {
            return false;
        }
    }
    if (vals.size() != 2) return false;
    x = vals[0];
    y = vals[1];
    return true;
}

enum class Outcome { Reached, Unreachable, NoSafeMotion, Cancelled, SafetyStop };

// Tek bir hedefe Tangent Bug ile git.
Outcome navigate(app::RobotIO& robot, const tbug::Config& cfg, const tbug::Goal& goal) {
    tbug::TangentBugPlanner planner(cfg);
    tbug::State lastState = tbug::State::Init;
    app::Snapshot s = robot.read();
    tbug::Pose prev = s.pose;
    tbug::Command cmd;
    double pathLength = 0.0, sinceHud = HudPeriodS, sinceStep = 0.0;

    auto summary = [&](const char* result) {
        std::cout << "\n  Result      : " << result << "\n"
                  << "  Final pose  : (" << std::fixed << std::setprecision(2) << s.pose.x << ", " << s.pose.y
                  << ")  heading " << std::setprecision(0) << s.pose.heading / Deg << " deg\n"
                  << "  Path length : " << std::setprecision(2) << pathLength << " m\n";
    };

    while (true) {
        s = robot.read();
        if (s.fault) {
            robot.stop();
            summary(s.fault);
            return Outcome::SafetyStop;
        }
        if (!s.fresh) {
            robot.stop();
            summary("sensor stream lost");
            return Outcome::SafetyStop;
        }
        if (g_cancel) {
            robot.stop();
            summary("cancelled (Ctrl+C)");
            return Outcome::Cancelled;
        }

        pathLength += std::hypot(s.pose.x - prev.x, s.pose.y - prev.y);
        prev = s.pose;

        // The planner only runs when a new sensor packet (~10 Hz) arrives, with
        // the pose of that moment; in between the last command keeps running.
        // Feeding it the same stale reading 10x with newer poses would smear
        // its obstacle memory (verified in simulation with Arduino timing).
        if (s.newData) {
            cmd = planner.step(s.range, s.pose, goal, std::clamp(sinceStep, 0.0, MaxDtS));
            sinceStep = 0.0;
        }

        if (planner.state() != lastState) {
            std::cout << "\n  [" << stateName(lastState) << " -> " << stateName(planner.state()) << "]";
            if (planner.state() == tbug::State::FollowBoundary)
                std::cout << " wall on " << (planner.followSide() == tbug::Side::Right ? "right" : "left");
            else if (planner.state() == tbug::State::MoveToEndpoint)
                std::cout << " toward (" << std::fixed << std::setprecision(2) << planner.endpoint().x << ", "
                          << planner.endpoint().y << ")";
            std::cout << "\n";
            lastState = planner.state();
        }

        if (planner.state() == tbug::State::GoalReached) {
            robot.stop();
            summary("GOAL REACHED");
            return Outcome::Reached;
        }
        if (planner.state() == tbug::State::Fault) {
            robot.stop();
            switch (planner.faultReason()) {
                case tbug::FaultReason::GoalUnreachable:
                    summary("GOAL UNREACHABLE (boundary circled both ways, no passage found)");
                    return Outcome::Unreachable;
                case tbug::FaultReason::GoalBlocked:
                    summary("GOAL TOO CLOSE TO AN OBSTACLE (keep targets >= 0.4 m from obstacles)");
                    return Outcome::Unreachable;
                case tbug::FaultReason::Timeout:
                    summary("GAVE UP (time limit for one target reached)");
                    return Outcome::Unreachable;
                default:
                    summary("STOPPED (no safe way forward)");
                    return Outcome::NoSafeMotion;
            }
        }

        robot.drive(cmd.v, cmd.w);

        if (sinceHud >= HudPeriodS) {
            sinceHud = 0.0;
            auto cm = [](bool valid, double r) { return valid ? (r >= 4.0 ? -1 : static_cast<int>(std::lround(r * 100.0))) : -2; };
            std::cout << "\r  " << std::left << std::setw(16) << stateName(planner.state()) << std::right
                      << " L " << std::setw(3) << cm(s.raw.leftValid, s.raw.left)
                      << " F " << std::setw(3) << cm(s.raw.frontValid, s.raw.front)
                      << " R " << std::setw(3) << cm(s.raw.rightValid, s.raw.right) << " cm"
                      << std::fixed << std::setprecision(2)
                      << " | (" << s.pose.x << ", " << s.pose.y << ") "
                      << std::setprecision(0) << std::setw(4) << s.pose.heading / Deg << " deg"
                      << " | goal " << std::setprecision(2) << planner.distanceToGoal(s.pose, goal) << " m   "
                      << std::flush;
        }

        const double dt = robot.tick();
        sinceHud += dt;
        sinceStep += dt;
    }
}

// Sadece Arduino: sensor degerlerini planlayicinin gorecegi bicimde yazdirir.
int sensorTest(const std::string& port) {
    app::SensorBoard board(port);
    if (!board.open()) return 1;
    SetConsoleCtrlHandler(consoleHandler, TRUE);
    std::cout << "\nSensor test - Ctrl+C to quit. Mounting: offset " << SensorOffsetM * 100.0
              << " cm from the centre, side beams +-" << SideBeamDeg << " deg.\n"
              << "  raw    = what each sensor measures (cm)\n"
              << "  centre = what the planner uses: distance from the robot centre (m)\n"
              << "  -1 = no echo (treated as free up to the sensing range), -2 = missing field\n\n";
    std::size_t packets = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (!g_cancel) {
        if (board.poll()) {
            ++packets;
            const auto raw = board.reading();
            const auto c = app::toCentre(raw, SensorOffsetM, board.maxRange());
            auto cm = [](bool v, double r) { return v ? (r >= 4.0 ? -1 : static_cast<int>(std::lround(r * 100.0))) : -2; };
            auto m = [](bool v, double r) { std::ostringstream o; if (!v) o << "  inv"; else if (r >= 4.0) o << " free"; else o << std::fixed << std::setprecision(2) << r; return o.str(); };
            const double rate = packets / std::max(1e-3, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            std::cout << "\r  raw  L " << std::setw(4) << cm(raw.leftValid, raw.left)
                      << "  F " << std::setw(4) << cm(raw.frontValid, raw.front)
                      << "  R " << std::setw(4) << cm(raw.rightValid, raw.right) << " cm"
                      << "   | centre  L " << m(c.leftValid, c.left) << "  F " << m(c.frontValid, c.front)
                      << "  R " << m(c.rightValid, c.right) << " m"
                      << "   | " << std::fixed << std::setprecision(1) << rate << " Hz   " << std::flush;
        } else if (board.secondsSincePacket() > 1.5) {
            std::cout << "\r  !! no sensor packet for " << std::fixed << std::setprecision(1)
                      << board.secondsSincePacket() << " s                                        " << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "\nBye.\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "=== Tangent Bug navigation (Kobuki + 3x HC-SR04) ===\n\n";

    const std::string mode = argc >= 2 ? argv[1] : "";
    if (mode == "--sensors") {
        if (argc < 3) { std::cerr << "Usage: " << argv[0] << " --sensors <Arduino COM port>\n"; return 1; }
        return sensorTest(argv[2]);
    }

    std::unique_ptr<app::RobotIO> robot;
    if (mode == "--sim") {
        robot = std::make_unique<app::SimRobot>(SideBeamDeg * Deg);
    } else if (argc >= 3) {
        robot = std::make_unique<app::HardwareRobot>(argv[1], argv[2], SensorOffsetM);
    } else {
        std::cerr << "Usage: " << argv[0] << " <Kobuki COM port> <Arduino COM port>\n"
                  << "       " << argv[0] << " --sim\n"
                  << "       " << argv[0] << " --sensors <Arduino COM port>\n";
        return 1;
    }
    if (!robot->open()) return 1;
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    // Planlayici ayarlari: varsayilanlar TangentBugPlanner.h'de simulasyonla
    // (Arduino zamanlamasiyla) dogrulanmis degerlerdir; mesafeler robot
    // MERKEZINDEN olculur (RobotIO sensor ofsetini ekler).
    tbug::Config cfg;
    cfg.beamAngle = SideBeamDeg * Deg; // yan sensorlerin one gore montaj acisi
    cfg.sensingRange = 1.2;            // m, bundan uzak okumalar "bos" sayilir (uzaktan bosluk kapali gorunmesin)
    cfg.robotRadius = RobotRadiusM;    // Kobuki govde yaricapi
    cfg.driveSpeed = 0.20;             // m/s, hedefe duz gidis
    cfg.followSpeed = 0.12;            // m/s, sinir takibi
    cfg.turnSpeed = 0.6;               // rad/s (simulasyonda carpismasiz dogrulanan ust sinir)
    cfg.maxGoalTimeS = 600.0;          // s, tek hedef icin azami sure -> vazgec

    std::cout << "\nRunning on: " << robot->name() << "\n"
              << "Start point = (0, 0), +x = the direction the robot faced at start, +y = its left.\n"
              << "Enter targets as \"x y\" in metres (e.g. \"2 0\" or \"1,5 -0,5\"); q = quit.\n"
              << "Ctrl+C while moving cancels the current target.\n";

    int exitCode = 0;
    while (true) {
        const auto pose = robot->read().pose;
        std::cout << "\nRobot at (" << std::fixed << std::setprecision(2) << pose.x << ", " << pose.y
                  << ").  Target x y > " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            if (g_cancel) { // Ctrl+C at the prompt interrupts the read; just ask again
                g_cancel = false;
                std::cin.clear();
                std::cout << "\n  (type q to quit)\n";
                continue;
            }
            break; // end of input
        }
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        if (line == "q" || line == "Q" || line == "quit" || line == "exit") break;

        double gx = 0.0, gy = 0.0;
        if (!parseTarget(line, gx, gy)) {
            std::cout << "  Invalid input - enter two numbers, e.g. \"2 0\".\n";
            continue;
        }

        std::cout << "  Going to (" << gx << ", " << gy << ")...\n";
        g_cancel = false;
        const Outcome o = navigate(*robot, cfg, { gx, gy });
        g_cancel = false;
        if (o == Outcome::SafetyStop) {
            std::cout << "\n!! Safety stop - check the robot and its surroundings, then restart.\n";
            exitCode = 2;
            break;
        }
    }

    robot->shutdown();
    std::cout << "\nBye.\n";
    return exitCode;
}
