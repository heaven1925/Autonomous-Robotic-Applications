// ExploreApp.h - the program around an exploration method. Every method's
// main.cpp is one line:  return app::run(argc, argv, makeMyStrategy);
//
//   Kalibrasyon  : Explore_X.exe --calibrate <Arduino COM> [--trials 100] [--cal-cells 1,2,3,5,10]
//                  Explore_X.exe --calibrate --sim
//   Kesif (sim)  : Explore_X.exe --sim [--map obstacleMapReferences/map2/map2.txt] [secenekler]
//   Kesif (robot): Explore_X.exe <Kobuki COM> <Arduino COM> [--map ...referans...] [secenekler]
//   Kolay yol    : ExploreLauncher.exe (pencereden secim) ya da .\run.ps1 11 -Algo bfs -Sim
//
//   Secenekler:
//     --arena <txt>   alan: boyut, hucre, baslangic (*), yon (varsayilan maps/arena.txt)
//     --map <txt>     engel haritasi (obstacleMapReferences/mapN/mapN.txt):
//                     --sim: simulatordeki gercek dunya (varsayilan map3)
//                     robot: cikan haritanin karsilastirildigi referans (verilmezse puan yok)
//     --model <txt>   sensor modeli (varsayilan sensor_model.txt; --calibrate buraya yazar)
//     --range-cal <txt>  40 cm mesafe kalibrasyonu (varsayilan sensor_calibration.txt;
//                     ExploreLauncher 3. adim yazar). Gercek robotta okumalar ham_cm * scale.
//     --seed <n>      simulator gurultusu ve rastgele yontemin tohumu (varsayilan 2026)
//     --result <csv>  sonuc satirinin eklenecegi dosya (varsayilan results/results.csv)
//     --no-result     sonuc satiri yazma
//     --no-gui  --fast  --go  --time <s>  --no-noise  --no-verify  --out <klasor>
//     --snapshot <s>  bu (simulasyon) saniyesinde pencere resmini de kaydet (belge / kontrol icin)
//     --ipc           pencere yok; durum stdout'a satir satir (Ipc.h), stdin "stop" = dur.
//                     ExploreLauncher canli haritayi bununla kendi penceresinde cizer.
#pragma once

#include "BayesGrid.h"
#include "Calibration.h"
#include "Console.h"
#include "ExploreRobot.h"
#include "ExploreView.h"
#include "Explorer.h"
#include "Ipc.h"
#include "Metrics.h"
#include "RangeCalibration.h"
#include "Strategy.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace app {

using astar::Cell;
using astar::Point;
using nav::Deg;

constexpr double SideBeamDeg = 45.0;   // yalniz 3 sensorlu kart icin (yan sensorler); laboratuvar duzeninde tek ON sensor
constexpr double SensorOffsetM = 0.11; // sensorlerin robot merkezine uzakligi
constexpr double KobukiRadiusM = 0.175;
constexpr double CliffAbortS = 1.5;
constexpr unsigned DefaultSeed = 2026;

struct Options {
    bool sim = false, calibrate = false, gui = true, fast = false, go = false, noise = true, verify = true;
    bool writeResult = true;
    bool ipc = false;                // --ipc: stream the state to the launcher (no own window)
    std::string kobukiPort, arduinoPort, arenaPath, mapPath, modelPath, rangeCalPath, outDir, resultPath;
    unsigned seed = DefaultSeed;
    int trials = 100;
    std::vector<int> calCells{ 1, 2, 3, 5, 10 };
    double timeLimit = 0.0;
    std::vector<double> snapshots;   // --snapshot: save the picture at these times (s)
};

using StrategyFactory = std::function<std::unique_ptr<explore::Strategy>(const Options&)>;

inline std::string folder(const char* sub) {
#ifdef EXPLORE_DIR
    return std::string(EXPLORE_DIR) + sub;
#else
    return std::string(".") + sub;
#endif
}

inline void usage(const char* exe) {
    std::cerr << "Usage: " << exe << " --calibrate <Arduino COM> [--trials N] [--cal-cells 1,2,3,5,10]   sensor model\n"
              << "       " << exe << " --calibrate --sim\n"
              << "       " << exe << " --sim [options]                                    explore in the simulator\n"
              << "       " << exe << " <Kobuki COM> <Arduino COM> [options]               explore with the robot\n"
              << "Options: --arena txt  --map txt  --model txt  --range-cal txt  --seed n  --result csv  --no-result\n"
              << "         --no-gui  --fast  --go  --time s  --no-noise  --no-verify  --out dir  --snapshot s\n";
}

inline bool parseArgs(int argc, char** argv, Options& o) {
    std::vector<std::string> ports;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto text = [&](std::string& out) {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };
        const auto integer = [&](int& out) {
            if (i + 1 >= argc) return false;
            try { out = std::stoi(argv[++i]); } catch (...) { return false; }
            return out > 0;
        };
        if (a == "--sim") o.sim = true;
        else if (a == "--calibrate") o.calibrate = true;
        else if (a == "--no-gui") o.gui = false;
        else if (a == "--fast") o.fast = true;
        else if (a == "--go") o.go = true;
        else if (a == "--ipc") { o.ipc = true; o.gui = false; o.go = true; }
        else if (a == "--no-noise") o.noise = false;
        else if (a == "--no-verify") o.verify = false;
        else if (a == "--no-result") o.writeResult = false;
        else if (a == "--arena") { if (!text(o.arenaPath)) return false; }
        else if (a == "--map" || a == "--truth") { if (!text(o.mapPath)) return false; }
        else if (a == "--model") { if (!text(o.modelPath)) return false; }
        else if (a == "--range-cal") { if (!text(o.rangeCalPath)) return false; }
        else if (a == "--out") { if (!text(o.outDir)) return false; }
        else if (a == "--result") { if (!text(o.resultPath)) return false; }
        else if (a == "--trials") { if (!integer(o.trials)) return false; }
        else if (a == "--seed") {
            int v = 0;
            if (!integer(v)) return false;
            o.seed = static_cast<unsigned>(v);
        }
        else if (a == "--cal-cells") {
            if (i + 1 >= argc) return false;
            o.calCells.clear();
            std::string t = argv[++i];
            for (char& ch : t) if (ch == ',' || ch == ';') ch = ' ';
            std::istringstream in(t);
            int k = 0;
            while (in >> k) { if (k < 1) return false; o.calCells.push_back(k); }
            if (o.calCells.empty()) return false;
        }
        else if (a == "--snapshot") {
            if (i + 1 >= argc) return false;
            try { o.snapshots.push_back(std::stod(argv[++i])); } catch (...) { return false; }
        }
        else if (a == "--time") {
            if (i + 1 >= argc) return false;
            try { o.timeLimit = std::stod(argv[++i]); } catch (...) { return false; }
        }
        else if (!a.empty() && a[0] == '-') { std::cerr << "Unknown option: " << a << "\n"; return false; }
        else ports.push_back(a);
    }
    if (o.arenaPath.empty()) o.arenaPath = folder("/maps/arena.txt");
    if (o.modelPath.empty()) o.modelPath = folder("/sensor_model.txt");
    if (o.rangeCalPath.empty()) o.rangeCalPath = folder("/sensor_calibration.txt");
    if (o.resultPath.empty()) o.resultPath = folder("/results/results.csv");
    if (o.mapPath.empty() && o.sim) o.mapPath = folder("/obstacleMapReferences/map3/map3.txt");
    if (o.calibrate) {
        if (o.sim) return ports.empty();
        if (ports.size() != 1) return false;
        o.arduinoPort = ports[0];
        return true;
    }
    if (o.sim) return ports.empty();
    if (ports.size() != 2) return false;
    o.kobukiPort = ports[0];
    o.arduinoPort = ports[1];
    return true;
}

// ================================================================ output ===
inline bool saveMapTxt(const std::filesystem::path& path, const bayes::BayesGrid& g, Cell start, double headingDeg,
                       const std::vector<Cell>& undecided, const explore::Strategy& strategy) {
    std::ofstream out(path);
    if (!out) return false;
    const astar::GridMap& a = g.arena();
    out << "# 11_ExplorationMapping result (" << strategy.name() << "), " << dateText() << "\n"
        << "# 0 = empty (P < 0.3), 1 = occupied (P > 0.8) or never decided, * = start\n";
    if (!undecided.empty()) {
        out << "# undecided cells (written as 1):";
        for (const Cell& c : undecided) out << " " << a.coords.text(c);
        out << "\n";
    }
    out << "# A* (10_AStarPathPlanning): put a / on the goal cell, then .\\run.ps1 10 -Map <this file>\n"
        << "name explored\n"
        << "cell " << a.cellSize() << "\n"
        << "coords " << a.coords.keyword() << "\n"
        << "border " << (a.walledBorder ? "wall" : "open") << "\n"
        << "heading " << headingDeg << "\n";
    for (int y = 0; y < a.rows(); ++y) {
        for (int x = 0; x < a.cols(); ++x) {
            const Cell c{ x, y };
            out << (x ? " " : "");
            if (c == start) out << '*';
            else out << (g.empty(c) ? '0' : '1');
        }
        out << "\n";
    }
    return static_cast<bool>(out);
}

inline void writeProbabilities(std::ostream& out, const bayes::BayesGrid& g) {
    const astar::GridMap& a = g.arena();
    out << "      ";
    for (int x = 0; x < a.cols(); ++x) out << std::setw(6) << a.coords.colLabel(x);
    out << "\n";
    out << std::fixed << std::setprecision(2);
    for (int y = 0; y < a.rows(); ++y) {
        out << std::setw(4) << a.coords.rowLabel(y) << "  ";
        for (int x = 0; x < a.cols(); ++x) {
            const Cell c{ x, y };
            const char tag = g.state(c) == bayes::State::Empty ? ' ' : (g.state(c) == bayes::State::Occupied ? '#' : '?');
            out << std::setw(5) << g.p(c) << tag;
        }
        out << "\n";
    }
}

inline std::string mapName(const astar::GridMap& m, const std::string& path) {
    if (!m.name.empty()) return m.name;
    return std::filesystem::path(path).stem().string();
}

// ================================================================== run ====
inline int run(int argc, char** argv, const StrategyFactory& makeStrategy) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(argv[0]); return 1; }
    SetConsoleCtrlHandler(consoleHandler, TRUE);
    if (opt.ipc) ipc::listenStdin();

    astar::GridMap arena;
    std::string error;
    if (!astar::loadMap(opt.arenaPath, arena, error)) { std::cerr << "ARENA ERROR: " << error << "\n"; return 1; }
    if (!arena.start) { std::cerr << "ARENA ERROR: no start cell (*) in " << opt.arenaPath << "\n"; return 1; }

    const explore::Geometry geo{ SideBeamDeg, SensorOffsetM, KobukiRadiusM, false };   // front sensor only

    // 40 cm distance calibration of the real sensor (the simulator needs none).
    rangecal::Calibration rangeCal;
    if (!opt.sim) {
        if (rangecal::load(opt.rangeCalPath, rangeCal))
            std::cout << std::fixed << std::setprecision(4) << "Range cal: cm = raw cm * " << rangeCal.scale << "  ("
                      << std::setprecision(0) << rangeCal.referenceCm << " cm block, " << rangeCal.date << ", speed of sound "
                      << rangeCal.speedOfSound() << " m/s)  [" << opt.rangeCalPath << "]\n";
        else
            std::cout << "!! Range calibration: " << opt.rangeCalPath
                      << " missing or invalid - raw HC-SR04 distances are used (ExploreLauncher step 3 calibrates).\n";
    }
    if (opt.calibrate) {
        CalibrationOptions co;
        co.sim = opt.sim;
        co.noise = opt.noise;
        co.arduinoPort = opt.arduinoPort;
        co.modelPath = opt.modelPath;
        co.trials = opt.trials;
        co.rangeScale = rangeCal.scale;
        co.cells = opt.calCells;
        co.geo = geo;
        return calibrate(co, arena);
    }

    const std::unique_ptr<explore::Strategy> strategy = makeStrategy(opt);
    std::cout << "=== Exploration mapping " << strategy->name() << ": Bayesian occupancy grid (Kobuki + 1x HC-SR04, 360 deg look-arounds) ===\n"
              << "Method   : " << strategy->rule() << "\n\n";
    bayes::SensorModel model;
    if (!bayes::loadSensorModel(opt.modelPath, model, error))
        std::cout << "!! Sensor model: " << error << "\n   Using the default Gaussian model (sigma(4 m) = 10 cm, P(hit|E) = 0.20) - run the calibration first (.\\run.ps1 11 -Calibrate).\n";
    const Cell start = *arena.start;
    const double headingDeg = arena.headingDeg.value_or(0.0);
    const astar::CoordStyle& cs = arena.coords;

    // Reference (obstacle) map: the simulated world, or what the real run is scored against.
    astar::GridMap truth;
    bool haveTruth = false;
    std::string refName = "-";
    if (!opt.mapPath.empty()) {
        if (!astar::loadMap(opt.mapPath, truth, error)) { std::cerr << "OBSTACLE MAP ERROR: " << error << "\n"; return 1; }
        if (truth.cols() != arena.cols() || truth.rows() != arena.rows()) {
            std::cerr << "ERROR: the obstacle map " << opt.mapPath << " is " << truth.rows() << " x " << truth.cols()
                      << ", the arena " << arena.rows() << " x " << arena.cols() << ".\n";
            return 1;
        }
        if (truth.occupied(start)) { std::cerr << "ERROR: the start cell is an obstacle in " << opt.mapPath << ".\n"; return 1; }
        haveTruth = true;
        refName = mapName(truth, opt.mapPath);
    }

    std::cout << std::fixed << std::setprecision(2)
              << "Arena    : " << opt.arenaPath << "\n"
              << "           " << arena.rows() << " rows x " << arena.cols() << " columns of " << arena.cellSize() * 100.0
              << " cm, start " << cs.text(start) << ", heading " << std::setprecision(0) << headingDeg << " deg"
              << (arena.walledBorder ? ", border wall" : "") << "\n"
              << "Obstacles: " << (haveTruth ? refName + "  (" + opt.mapPath + ")" : std::string("none given - the run is not scored")) << "\n"
              << std::setprecision(2)
              << "Sensor   : one HC-SR04 facing forward, " << SensorOffsetM * 100.0
              << " cm from the centre; the robot turns 360 deg to look around\n"
              << "Model    : [" << model.source << "]\n"
              << "Rule     : every cell starts at 0.50;  P < " << bayes::EmptyBelow << " empty,  P > " << bayes::OccupiedAbove << " occupied\n";
    printModelTable(model, arena.cellSize());
    std::cout << std::endl;
    if (opt.ipc) ipc::header(arena, start, headingDeg, *strategy, refName);

    // --- robot -------------------------------------------------------------
    const nav::Pose startPose{ arena.centre(start).x, arena.centre(start).y, headingDeg * Deg };
    std::unique_ptr<explore::Robot> robot;
    if (opt.sim) {
        astar::GridMap world(arena.cols(), arena.rows(), arena.cellSize());
        for (int y = 0; y < arena.rows(); ++y)
            for (int x = 0; x < arena.cols(); ++x) world.setOccupied({ x, y }, truth.occupied({ x, y }));
        explore::SimRobot::Options so;
        so.geo = geo;
        so.walledBorder = arena.walledBorder;
        so.noise = opt.noise;
        so.speed = (opt.fast || (!opt.gui && !opt.ipc)) ? 0 : 4;   // --ipc: the launcher shows it live
        so.seed = opt.seed;
        robot = std::make_unique<explore::SimRobot>(world, startPose, so);
        std::cout << "Simulator: obstacle map " << refName << ", seed " << opt.seed
                  << (opt.noise ? " (sensor noise, missed echoes, ghost readings)" : " (no noise)") << "\n";
    } else {
        robot = std::make_unique<explore::HardwareRobot>(opt.kobukiPort, opt.arduinoPort, startPose, rangeCal.scale);
    }

    // --- grid, explorer, window -------------------------------------------
    bayes::BayesGrid grid(arena);
    explore::Config cfg;
    if (opt.timeLimit > 0.0) cfg.timeLimitS = opt.timeLimit;
    cfg.verify = opt.verify;
    explore::Explorer explorer(grid, model, geo, *strategy, cfg);

    view::GdiPlusSession gdiPlus;
    int picW = 1180, picH = 800;
    if (opt.gui) view::pictureSize(picW, picH);
    view::Canvas canvas(picW, picH);
    view::MapWindow window;
    const std::wstring windowTitle = wide(strategy->name()) + L" Exploration Mapping - " + strategy->title() +
                                     (opt.sim ? L" (simulator, " : L" (robot, ") + wide(refName) + L")";
    if (opt.gui && !window.create(picW, picH, windowTitle.c_str()))
        std::cout << "(could not open the window - continuing without it)\n";

    if (!robot->open()) return 1;
    if (!opt.sim) {
        std::cout << "\nPlace the robot with its CENTRE on cell " << cs.text(start) << " facing " << std::setprecision(0)
                  << headingDeg << " deg (as in the arena file).\n"
                  << (opt.go ? "--go: starting in 3 s (Q / close the window to cancel).\n" : "Press ENTER to start, Q to quit.\n")
                  << std::flush;
        if (opt.ipc) ipc::event(0.0, "Kobuki and Arduino connected - the robot starts in 3 s");
        const auto t0 = std::chrono::steady_clock::now();
        while (true) {
            window.pump();
            const int key = pollKey(window);
            if (key == VK_RETURN) break;
            if (opt.go && std::chrono::steady_clock::now() - t0 >= std::chrono::seconds(3)) break;
            if (key == 'Q' || key == VK_ESCAPE || g_quit || window.closedByUser()) { robot->shutdown(); return 0; }
            robot->tick();
        }
    }
    robot->resetPose();

    explore::Snapshot s = robot->read();
    explorer.begin(s, start);
    std::cout << "\nExploring. Q / close the window = stop. (Ctrl+C also stops.)\n";

    const std::filesystem::path outDir = opt.outDir.empty() ? std::filesystem::path(folder("/output")) / strategy->id()
                                                            : std::filesystem::path(opt.outDir);
    using Clock = std::chrono::steady_clock;
    auto lastHud = Clock::now() - std::chrono::seconds(10), lastView = lastHud, lastIpc = lastHud;
    double now = 0.0, dt = 0.0, driven = 0.0, cliffTime = 0.0;
    Point lastPoint = s.pose.point();
    const char* stopReason = nullptr;

    const auto info = [&]() {
        std::vector<std::wstring> v;
        wchar_t b[200];
        v.push_back(L"#" + wide(strategy->name()));
        {   // the rule, word-wrapped to the panel
            std::istringstream words(strategy->rule());
            std::string word, line;
            while (words >> word) {
                if (!line.empty() && line.size() + 1 + word.size() > 52) { v.push_back(wide(line)); line.clear(); }
                line += (line.empty() ? "" : " ") + word;
            }
            if (!line.empty()) v.push_back(wide(line));
        }
        explore::StrategyView sv;
        strategy->describe(sv);
        for (const auto& line : sv.info) v.push_back(line);
        v.push_back(L"#Bayesian grid");
        swprintf(b, 200, L"empty %d   occupied %d   undecided %d", grid.count(bayes::State::Empty),
                 grid.count(bayes::State::Occupied), grid.count(bayes::State::Undecided));
        v.push_back(b);
        swprintf(b, 200, L"Gaussian: sigma(1 m) %.1f cm, sigma(4 m) %.1f cm", model.sigma(1.0) * 100.0, model.sigma(4.0) * 100.0);
        v.push_back(b);
        swprintf(b, 200, L"P(hit|O) 1 cell ahead %.2f   P(hit|E) %.2f", model.pHitOccupied(0.09, arena.cellSize()), model.pHitEmpty);
        v.push_back(b);
        if (haveTruth) {
            const metrics::Score sc = metrics::score(grid, truth);
            v.push_back(L"#Score vs " + wide(refName));
            swprintf(b, 200, L"success %.1f %%  (%d / %d observable cells)", sc.successRatio() * 100.0, sc.correctObs, sc.observable);
            v.push_back(b);
            swprintf(b, 200, L"wrong %d   undecided %d   all cells %d / %d", sc.wrong, sc.undecided, sc.correct, sc.cells);
            v.push_back(b);
        }
        v.push_back(L"#Exploration");
        swprintf(b, 200, L"frontier cells %d", static_cast<int>(explorer.frontier().size()));
        v.push_back(b);
        if (explorer.hasTarget()) {
            const bool look = explorer.action() == explore::Action::LookAround;
            v.push_back((look ? L"look around at " : L"target ") + wide(cs.text(explorer.target())) +
                        (look ? L"" : L"  via " + wide(cs.text(explorer.goal()))));
        }
        if (explorer.verifying()) v.push_back(L"verifying occupied cells face-on");
        swprintf(b, 200, L"goals %d, look-arounds %d, face-on checks %d", explorer.goals(), explorer.scans(),
                 explorer.verifiedCount());
        v.push_back(b);
        swprintf(b, 200, L"readings used %ld, bumps %d, position fixes %d", explorer.readings(), explorer.bumps(), explorer.corrections());
        v.push_back(b);
        swprintf(b, 200, L"t %.0f s, driven %.1f m", now, driven);
        v.push_back(b);
        const auto& be = s.beams[1];
        if (!be.valid) v.push_back(L"front sonar: --");
        else if (!be.echo) v.push_back(L"front sonar: no echo");
        else { swprintf(b, 200, L"front sonar: %.0f cm", be.range * 100.0); v.push_back(b); }
        return v;
    };
    const auto draw = [&](const std::wstring& status) {
        view::ExploreScene sc;
        sc.grid = &grid;
        sc.explorer = &explorer;
        sc.pose = s.pose;
        sc.robotRadius = KobukiRadiusM;
        sc.noEchoRange = model.noEchoRange;
        sc.start = start;
        sc.title = wide(strategy->name()) + L"  -  " + strategy->title() + L"  -  " + wide(refName);
        sc.status = status;
        sc.info = info();
        strategy->describe(sc.plan);
        sc.truth = haveTruth ? &truth : nullptr;
        view::renderExplore(canvas, sc);
        window.present(canvas);
    };
    const auto statusText = [&]() {
        std::wostringstream w;
        w << explore::phaseName(explorer.phase()) << L"   |   cell " << wide(cs.text(arena.cellAt(s.pose.point())))
          << std::fixed << std::setprecision(0) << L"   " << s.pose.heading / Deg << L" deg   |   t " << now << L" s";
        return w.str();
    };

    while (true) {
        s = robot->read();
        driven += astar::distance(lastPoint, s.pose.point());
        lastPoint = s.pose.point();

        if (g_quit) { stopReason = "stopped by the user (Ctrl+C)"; break; }
        window.pump();
        const int key = pollKey(window);
        if (key == 'Q' || key == VK_ESCAPE) { stopReason = "stopped by the user (Q)"; break; }
        if (window.closedByUser()) { stopReason = "window closed"; break; }
        if (!s.fresh) { stopReason = "sensor stream lost (Arduino)"; break; }
        if (s.fault) {   // cliff: stop now, give up only if it lasts
            robot->stop();
            if (cliffTime == 0.0) std::cout << "\n  " << s.fault << " - stopped, waiting " << CliffAbortS << " s...\n";
            cliffTime += dt;
            if (cliffTime >= CliffAbortS) { stopReason = s.fault; break; }
            dt = robot->tick();
            continue;
        }
        cliffTime = 0.0;

        explorer.observe(s, now);
        const nav::Command cmd = explorer.step(s, dt, now, *robot);
        const std::string ev = explorer.takeEvent();
        if (!ev.empty()) {
            if (opt.ipc) ipc::event(now, ev);
            else std::cout << "\n  [" << std::setprecision(0) << now << " s] " << ev << "\n";
        }
        if (explorer.phase() == explore::Phase::Done) break;
        robot->drive(cmd.v, cmd.w);

        const auto t = Clock::now();
        if (opt.ipc && std::chrono::duration<double>(t - lastIpc).count() >= 0.1) {
            lastIpc = t;
            ipc::frame(grid, explorer, *strategy, s, now, driven);
        }
        if (!opt.ipc && std::chrono::duration<double>(t - lastHud).count() >= 0.25) {
            lastHud = t;
            std::cout << "\r  " << std::left << std::setw(8) << explore::phaseName(explorer.phase()) << std::right
                      << " cell " << std::setw(7) << cs.text(arena.cellAt(s.pose.point())) << " | empty " << std::setw(2)
                      << grid.count(bayes::State::Empty) << " occ " << std::setw(2) << grid.count(bayes::State::Occupied)
                      << " undecided " << std::setw(2) << grid.count(bayes::State::Undecided) << " | t " << std::setprecision(0)
                      << now << " s   " << std::flush;
        }
        if (window.isOpen() && std::chrono::duration<double>(t - lastView).count() >= 0.15) {
            lastView = t;
            draw(statusText());
        }
        for (double& at : opt.snapshots) {
            if (at < 0.0 || now < at) continue;
            draw(statusText());
            std::error_code ec;
            std::filesystem::create_directories(outDir, ec);
            const auto file = outDir / (std::string("snapshot_") + strategy->id() + "_" + refName + "_t" +
                                        std::to_string(static_cast<int>(at)) + ".png");
            if (canvas.savePng(file.wstring())) std::cout << "\n  snapshot: " << file.string() << "\n";
            at = -1.0;
        }
        dt = robot->tick();
        now += dt;
    }
    robot->stop();

    // --- result --------------------------------------------------------------
    std::vector<Cell> undecided;
    for (int y = 0; y < arena.rows(); ++y)
        for (int x = 0; x < arena.cols(); ++x)
            if (grid.state({ x, y }) == bayes::State::Undecided) undecided.push_back({ x, y });
    std::string result = stopReason ? stopReason : "";
    switch (explorer.finished()) {
        case explore::Finish::AllDecided: result = "MAPPING COMPLETE - every cell decided"; break;
        case explore::Finish::NoFrontier: result = undecided.empty() ? "MAPPING COMPLETE - every cell decided"
                                                                     : "MAPPING COMPLETE - no frontier left (some cells can never be seen)"; break;
        case explore::Finish::Stagnated: result = "MAPPING COMPLETE - " + strategy->stopReason(); break;
        case explore::Finish::TimeLimit: result = "STOPPED - time limit"; break;
        case explore::Finish::TooManyBumps: result = "STOPPED - too many bumps"; break;
        default: break;
    }
    draw(wide(result));

    std::error_code fsError;
    const std::filesystem::path& dir = outDir;
    std::filesystem::create_directories(dir, fsError);
    const std::string base = std::string("explored_") + strategy->id() + "_" + (refName == "-" ? "nomap" : refName) +
                             (opt.sim ? "_s" + std::to_string(opt.seed) : std::string("_real")) + "_" + timestamp();
    std::string stem = base;
    for (int k = 2; std::filesystem::exists(dir / (stem + ".txt"), fsError); ++k)   // two runs in the same second
        stem = base + "-" + std::to_string(k);
    const auto txt = dir / (stem + ".txt"), prob = dir / (stem + "_prob.txt"), png = dir / (stem + ".png");
    const bool okTxt = saveMapTxt(txt, grid, start, headingDeg, undecided, *strategy);
    bool okProb = false;
    {
        std::ofstream p(prob);
        if (p) {
            p << "# P(Occupied) per cell after exploration (" << strategy->name() << "), " << dateText() << "\n"
              << "# tag: ' ' empty (P < 0.3)  '#' occupied (P > 0.8)  '?' undecided\n"
              << "# sensor model: sigma(d) = " << model.sigma0 << " + " << model.sigmaPerM << " d, P(detect) = " << model.pDetect
              << ", P(hit|E) = " << model.pHitEmpty << "\n";
            writeProbabilities(p, grid);
            okProb = static_cast<bool>(p);
        }
    }
    const bool okPng = canvas.savePng(png.wstring());

    const bool completed = explore::finishedNormally(explorer.finished());
    std::cout << "\n\n--- Summary (" << strategy->name() << ") ---\n"
              << "Result      : " << result << "\n"
              << std::setprecision(1)
              << "Time        : " << now << " s" << (completed ? "  (speed: time to complete the map)" : "  (NOT completed)")
              << ", driven " << driven << " m\n"
              << "Cells       : " << grid.count(bayes::State::Empty) << " empty, " << grid.count(bayes::State::Occupied)
              << " occupied, " << undecided.size() << " undecided\n"
              << "Exploration : " << explorer.goals() << " goals, " << explorer.scans() << " look-arounds, " << explorer.plans()
              << " plans, " << explorer.verifiedCount() << " face-on checks, " << explorer.readings()
              << " sonar readings used, " << explorer.bumps() << " bumps\n";
    if (!undecided.empty()) {
        std::cout << "Undecided   :";
        for (const Cell& c : undecided) std::cout << " " << cs.text(c);
        std::cout << "  (written as 1)\n";
    }
    std::cout << "\nP(Occupied):\n";
    writeProbabilities(std::cout, grid);

    metrics::Score sc;
    if (haveTruth) {
        sc = metrics::score(grid, truth);
        std::cout << "\nCompared with the obstacle map " << refName << ": " << sc.correct << " correct, " << sc.wrong << " wrong, "
                  << sc.undecided << " undecided";
        for (const Cell& c : sc.wrongCells) std::cout << " " << cs.text(c);
        std::cout << "\nSuccess ratio: " << std::setprecision(1) << sc.successRatio() * 100.0 << " %  (" << sc.correctObs << " / "
                  << sc.observable << " observable cells; all cells " << sc.accuracyAll() * 100.0 << " %)\n";
    }
    std::cout << "\nSaved:\n  " << txt.string() << "   (0/1 map for A*)\n  " << prob.string() << "\n  " << png.string() << "\n";
    if (!okTxt || !okProb || !okPng) std::cout << "  !! some files could not be written to " << dir.string() << "\n";

    if (opt.writeResult) {
        metrics::RunRecord r;
        r.timestamp = dateText(true);
        r.algorithm = strategy->id();
        r.map = refName;
        r.mode = opt.sim ? "sim" : "real";
        r.seed = opt.seed;
        r.noise = opt.sim ? opt.noise : true;
        r.verify = opt.verify;
        r.finish = explore::finishId(explorer.finished());
        r.completed = completed;
        r.timeS = now;
        r.drivenM = driven;
        r.scored = haveTruth;
        r.s = sc;
        if (!haveTruth) r.s.cells = arena.cols() * arena.rows(), r.s.undecided = static_cast<int>(undecided.size());
        r.bumps = explorer.bumps();
        r.scans = explorer.scans();
        r.plans = explorer.plans();
        r.goals = explorer.goals();
        r.faceOn = explorer.verifiedCount();
        r.readings = explorer.readings();
        r.output = txt.string();
        if (metrics::appendCsv(opt.resultPath, r)) std::cout << "  " << opt.resultPath << "   (result line appended)\n";
        else std::cout << "  !! could not append to " << opt.resultPath << "\n";
    }
    if (opt.ipc) {
        ipc::frame(grid, explorer, *strategy, s, now, driven);
        ipc::done(explorer.finished(), completed, now, driven, result, txt.string());
    }

    robot->shutdown();
    if (window.isOpen() && !g_quit) {
        std::cout << "\nThe window stays open - close it (or press Q / Enter) to exit.\n";
        while (window.isOpen() && !g_quit) {
            window.pump();
            const int k = pollKey(window);
            if (k == 'Q' || k == VK_RETURN || k == VK_ESCAPE) break;
            Sleep(30);
        }
    }
    return completed ? 0 : 2;
}

} // namespace app
