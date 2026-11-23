// Ornek 11 - Exploration Mapping: frontier tabanli kesif + Bayes doluluk
// izgarasi (Kobuki + tek HC-SR04 onde; robot yerinde 360 derece donerek olcer).
//
//   Kalibrasyon  : Explore.exe --calibrate <Arduino COM> [--trials 100] [--cal-cells 1,2,3,5,10]
//                  Explore.exe --calibrate --sim
//   Keşif (sim)  : Explore.exe --sim [secenekler]
//   Keşif (robot): Explore.exe <Kobuki COM> <Arduino COM> [secenekler]
//   Kolay yol    : .\run.ps1 11 -Calibrate / .\run.ps1 11 -Sim / .\run.ps1 11
//
//   Secenekler:
//     --arena <txt>  alan: boyut, hucre, baslangic (*), yon (varsayilan maps/arena.txt)
//     --truth <txt>  yalniz --sim: gercek harita (varsayilan maps/truth.txt)
//     --model <txt>  sensor modeli (varsayilan sensor_model.txt; --calibrate buraya yazar)
//     --no-gui  --fast  --go  --time <s>  --no-noise  --out <klasor>
//     --no-verify    frontier bitince dolu hucreleri karsidan yeniden olcme
//
// Ders notundaki adimlar:
//   1. Initialize grid : her hucre P(Dolu) = 0.5
//   2. Update the grid : robotun hucresi bos; her sonar olcumu Bayes kurali ile
//                        isindaki hucreleri gunceller (P(olay|Dolu), P(olay|Bos)
//                        deneysel kalibrasyondan)
//   3. Pick a next move: frontier = karar verilmemis ve bos komsusu olan hucre;
//                        en yakin frontier'in bos komsusuna A* ile git, frontier'a
//                        don ve olc
//   4. Loop            : tum hucreler < 0.3 (bos) ya da > 0.8 (dolu) olunca biter;
//                        hic gorulemeyen hucre kalirsa frontier bitince durur.
// Sonuc 10_AStarPathPlanning'in harita bicimindeki txt'ye yazilir.
#include "BayesGrid.h"
#include "ExploreRobot.h"
#include "ExploreView.h"
#include "Explorer.h"

#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <conio.h>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using astar::Cell;
using astar::Point;
using nav::Deg;

constexpr double SideBeamDeg = 45.0;   // yalniz 3 sensorlu kart icin (yan sensorler); laboratuvar duzeninde tek ON sensor
constexpr double SensorOffsetM = 0.11; // sensorlerin robot merkezine uzakligi
constexpr double KobukiRadiusM = 0.175;
constexpr double CliffAbortS = 1.5;

std::atomic<bool> g_quit{ false };

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) { g_quit = true; return TRUE; }
    return FALSE;
}

struct Options {
    bool sim = false, calibrate = false, gui = true, fast = false, go = false, noise = true, verify = true;
    std::string kobukiPort, arduinoPort, arenaPath, truthPath, modelPath, outDir;
    int trials = 100;
    std::vector<int> calCells{ 1, 2, 3, 5, 10 };   // box positions for the calibration (cells ahead)
    double timeLimit = 0.0;
};

std::string folder(const char* sub) {
#ifdef EXPLORE_DIR
    return std::string(EXPLORE_DIR) + sub;
#else
    return std::string(".") + sub;
#endif
}

void usage(const char* exe) {
    std::cerr << "Usage: " << exe << " --calibrate <Arduino COM> [--trials N] [--cal-cells 1,2,3,5,10]   sensor model\n"
              << "       " << exe << " --calibrate --sim\n"
              << "       " << exe << " --sim [options]                                    explore in the simulator\n"
              << "       " << exe << " <Kobuki COM> <Arduino COM> [options]               explore with the robot\n"
              << "Options: --arena txt  --truth txt (sim)  --model txt  --no-gui  --fast  --go  --time s  --no-noise  --no-verify  --out dir\n";
}

bool parseArgs(int argc, char** argv, Options& o) {
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
        else if (a == "--no-noise") o.noise = false;
        else if (a == "--no-verify") o.verify = false;
        else if (a == "--arena") { if (!text(o.arenaPath)) return false; }
        else if (a == "--truth") { if (!text(o.truthPath)) return false; }
        else if (a == "--model") { if (!text(o.modelPath)) return false; }
        else if (a == "--out") { if (!text(o.outDir)) return false; }
        else if (a == "--trials") { if (!integer(o.trials)) return false; }
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
        else if (a == "--time") {
            if (i + 1 >= argc) return false;
            try { o.timeLimit = std::stod(argv[++i]); } catch (...) { return false; }
        }
        else if (!a.empty() && a[0] == '-') { std::cerr << "Unknown option: " << a << "\n"; return false; }
        else ports.push_back(a);
    }
    if (o.arenaPath.empty()) o.arenaPath = folder("/maps/arena.txt");
    if (o.modelPath.empty()) o.modelPath = folder("/sensor_model.txt");
    if (o.truthPath.empty()) {
#ifdef LAB_MAP
        o.truthPath = LAB_MAP;
#else
        o.truthPath = "truth.txt";
#endif
    }
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

std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

std::string dateText() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tm);
    return buf;
}

std::wstring wide(const std::string& s) { return std::wstring(s.begin(), s.end()); }

int pollKey(view::MapWindow& window) {
    int k = window.takeKey();
    if (k != 0) return k;
    if (!_kbhit()) return 0;
    k = _getch();
    if (k == 0 || k == 224) { (void)_getch(); return 0; }
    return k == '\r' ? VK_RETURN : std::toupper(k);
}

bool waitEnter() {
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

// ============================================================ calibration ==
// Ders notu, "Building a Probabilistic Sensor Model":
//   Method B (Gaussian): kutu sirayla 1, 2, 3, 5, 10 hucre ileride; her
//     mesafede N olcum -> ortalama, sigma, algilama orani. sigma(d) dogrusu
//     sigma0 + k*d en kucuk kareler ile uydurulur. Hucreye dusme olasiligi
//     P(olay|Dolu) bu Gaussian'in hucre araligi uzerinden integralidir.
//   Method A (deney): hucre BOS iken N olcumun kaci hucreye dusuyor -> P(olay|Bos).
// Sonunda slayttaki ornek (z = 4.0 m, aralik [3.9, 4.1] m) ve 0.5 -> 4 m tablo yazilir.

// One Bayes step from P = 0.5.
double fromHalf(double likeO, double likeE) { return bayes::posterior(0.5, likeO, likeE); }

void printModelTable(const bayes::SensorModel& m, double cell) {
    std::cout << std::fixed << std::setprecision(3)
              << "\nSensor model: sigma(d) = " << m.sigma0 << " + " << m.sigmaPerM << " * d  m,  bias " << m.bias
              << " m,  P(detect) = " << m.pDetect << ",  P(hit|E) = " << m.pHitEmpty << "\n\n"
              << "  distance   sigma    P(hit|O)   P(hit|E)   P(O) after a hit   P(O) after no hit\n"
              << "  (cell of " << std::setprecision(2) << cell << " m, Gaussian integrated over the cell)\n";
    for (double d : { 0.5, 1.0, 2.0, 3.0, 4.0 }) {
        const double pO = m.pHitOccupied(d, cell), pE = m.pHitEmpty;
        std::cout << std::setprecision(1) << "  " << std::setw(5) << d << " m  " << std::setprecision(3) << std::setw(7)
                  << m.sigma(d) << "   " << std::setw(7) << pO << "    " << std::setw(7) << pE << "        "
                  << std::setw(6) << fromHalf(pO, pE) << "             " << std::setw(6) << fromHalf(1.0 - pO, 1.0 - pE) << "\n";
    }
    // The lecture's example: z = 4.0 m, event window [3.9, 4.1] m.
    const double pO = m.pWindow(4.0, 3.9, 4.1), pE = m.pHitEmpty;
    std::cout << "\n  Lecture example, cell at 4.0 m, event = reading in [3.9, 4.1] m (Gaussian, sigma "
              << m.sigma(4.0) * 100.0 << " cm):\n"
              << "    P(z|O) = " << pO << "   P(z|E) = " << pE << "\n"
              << "    Bayes from 0.5:  hit -> P(O|z) = " << fromHalf(pO, pE) << "   no hit -> " << fromHalf(1.0 - pO, 1.0 - pE)
              << "   (thresholds: > 0.8 occupied, < 0.3 empty)\n";
}

int calibrate(const Options& o, const astar::GridMap& arena) {
    const double cell = arena.cellSize();
    std::cout << "=== Sensor calibration: Gaussian noise model (Method B) + P(hit|E) (Method A) ===\n\n"
              << "Front sensor only. The robot stands with its CENTRE in the middle of a cell, facing along the grid.\n"
              << "A box (flat face to the robot) is put k cells ahead, its face on the near edge of that cell.\n"
              << o.trials << " readings per position.\n";

    // Box positions: k cells ahead -> distance from the sensor to the box face.
    std::vector<int> cells = o.calCells;
    const auto faceDist = [&](int k) { return cell * (k - 0.5) - SensorOffsetM; };

    std::unique_ptr<explore::SensorBoard> board;
    std::unique_ptr<explore::SimRobot> sim;
    const int maxK = *std::max_element(cells.begin(), cells.end());
    const Cell robotCell{ arena.cols() / 2, 1 };
    if (o.sim) {
        explore::SimRobot::Options so;
        so.geo = { SideBeamDeg, SensorOffsetM, KobukiRadiusM, false };
        so.walledBorder = false;
        so.noise = o.noise;
        so.speed = 0;
        astar::GridMap open(arena.cols(), maxK + 6, cell);
        sim = std::make_unique<explore::SimRobot>(open, nav::Pose{ open.centre(robotCell).x, open.centre(robotCell).y, -90.0 * Deg }, so);
        sim->open();
        std::cout << "(simulator: open room, the box is placed / removed automatically)\n";
    } else {
        board = std::make_unique<explore::SensorBoard>(o.arduinoPort);
        if (!board->open()) return 1;
    }

    const auto read = [&](explore::Beam& b) {
        if (sim) { b = sim->measure(1); return true; }
        while (!g_quit) {
            if (!board->poll()) { Sleep(5); continue; }
            explore::Beam beams[3];
            board->fill(beams);
            b = beams[1];
            if (b.valid) return true;
        }
        return false;
    };

    // --- Method B: box at each distance ------------------------------------
    struct Fit { double d, mean, sigma, detect; };
    std::vector<Fit> fits;
    for (int k : cells) {
        const double d = faceDist(k);
        std::cout << std::fixed << std::setprecision(2) << "\nBox " << k << " cell(s) ahead (face " << d << " m from the sensor).";
        if (sim) {
            for (int j = 1; j < maxK + 6; ++j) sim->setObstacle({ robotCell.x, robotCell.y + j }, false);
            sim->setObstacle({ robotCell.x, robotCell.y + k }, true);
            std::cout << "\n";
        } else {
            std::cout << " Press ENTER (Q = cancel)." << std::endl;
            if (!waitEnter()) return 0;
        }
        std::vector<double> boxReadings;   // readings that belong to the box (within 0.25 m)
        int total = 0, noEcho = 0;
        while (total < o.trials) {
            explore::Beam b;
            if (!read(b)) return 0;
            ++total;
            if (!b.echo) { ++noEcho; continue; }
            if (std::abs(b.range - d) <= 0.25) boxReadings.push_back(b.range);
        }
        Fit f{ d, 0.0, 0.0, static_cast<double>(boxReadings.size()) / total };
        for (double v : boxReadings) f.mean += v;
        if (!boxReadings.empty()) f.mean /= static_cast<double>(boxReadings.size());
        for (double v : boxReadings) f.sigma += (v - f.mean) * (v - f.mean);
        f.sigma = boxReadings.size() > 1 ? std::sqrt(f.sigma / static_cast<double>(boxReadings.size() - 1)) : 0.0;
        std::cout << "  mean " << std::setprecision(3) << f.mean << " m, sigma " << f.sigma * 100.0 << " cm, detected "
                  << boxReadings.size() << " / " << total << " (no echo " << noEcho << ")\n";
        if (boxReadings.size() >= 3) fits.push_back(f);
    }
    if (fits.empty()) { std::cerr << "ERROR: the box was never detected - check the sensor / box position.\n"; return 2; }

    // sigma(d) = sigma0 + k d (least squares), bias = mean(mean - d), P(detect) = mean detection rate.
    bayes::SensorModel m;
    double sx = 0, sy = 0, sxx = 0, sxy = 0, bias = 0, det = 0;
    for (const Fit& f : fits) {
        sx += f.d; sy += f.sigma; sxx += f.d * f.d; sxy += f.d * f.sigma;
        bias += f.mean - f.d;
        det += f.detect;
    }
    const double n = static_cast<double>(fits.size());
    double slope = 0.0, icpt = sy / n;
    if (fits.size() >= 2 && sxx * n - sx * sx > 1e-12) {
        slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        icpt = (sy - slope * sx) / n;
    }
    if (slope < 0.0) { slope = 0.0; icpt = sy / n; }
    m.sigma0 = std::max(icpt, 0.003);   // HC-SR04 resolution ~3 mm
    m.sigmaPerM = slope;
    m.bias = bias / n;
    m.pDetect = std::clamp(det / n, 0.05, 0.99);

    // --- Method A: the cell 1 ahead empty -------------------------------------
    const double s1 = faceDist(1);
    const double w0 = m.windowStart(s1), w1 = m.windowEnd(s1, cell);
    std::cout << std::setprecision(2) << "\nCELL EMPTY: remove the box; keep the cells in front free. Event: reading in ["
              << w0 << ", " << w1 << "] m (the cell 1 ahead).";
    if (sim) {
        for (int j = 1; j < maxK + 6; ++j) sim->setObstacle({ robotCell.x, robotCell.y + j }, false);
        std::cout << "\n";
    } else {
        std::cout << " Press ENTER (Q = cancel)." << std::endl;
        if (!waitEnter()) return 0;
    }
    int inCell = 0, total = 0;
    while (total < o.trials) {
        explore::Beam b;
        if (!read(b)) return 0;
        ++total;
        if (b.echo && b.range >= w0 && b.range <= w1) ++inCell;
    }
    const double minP = std::max(0.01, 1.0 / (2.0 * o.trials));   // 0 would decide a cell for ever
    m.pHitEmpty = std::max(static_cast<double>(inCell) / total, minP);
    std::cout << "  in the cell: " << inCell << " / " << total << "  ->  P(hit|E) = " << std::setprecision(3) << m.pHitEmpty << "\n";

    printModelTable(m, cell);

    std::ofstream out(o.modelPath);
    if (!out) { std::cerr << "ERROR: cannot write " << o.modelPath << "\n"; return 1; }
    out << std::fixed << std::setprecision(4)
        << "# HC-SR04 sensor model (front sensor) - " << dateText() << (sim ? "  (SIMULATOR)" : "") << "\n"
        << "# Method B (Gaussian): reading ~ N(d + bias, sigma0 + sigma_per_m * d), detected with p_detect\n";
    for (const Fit& f : fits)
        out << "#   box at " << f.d << " m: mean " << f.mean << ", sigma " << f.sigma << ", detected " << f.detect << "\n";
    out << "# Method A: cell empty -> " << inCell << " / " << total << " readings in the cell\n"
        << "sigma0 " << m.sigma0 << "\n"
        << "sigma_per_m " << m.sigmaPerM << "\n"
        << "bias " << m.bias << "\n"
        << "p_detect " << m.pDetect << "\n"
        << "p_hit_empty " << m.pHitEmpty << "\n";
    std::cout << "\nSaved: " << o.modelPath << "  (the exploration uses this file)\n";
    return 0;
}

// ================================================================ output ===
bool saveMapTxt(const std::filesystem::path& path, const bayes::BayesGrid& g, Cell start, double headingDeg,
                const std::vector<Cell>& undecided) {
    std::ofstream out(path);
    if (!out) return false;
    const astar::GridMap& a = g.arena();
    out << "# 11_ExplorationMapping result, " << dateText() << "\n"
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

void writeProbabilities(std::ostream& out, const bayes::BayesGrid& g) {
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

} // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(argv[0]); return 1; }
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    astar::GridMap arena;
    std::string error;
    if (!astar::loadMap(opt.arenaPath, arena, error)) { std::cerr << "ARENA ERROR: " << error << "\n"; return 1; }
    if (!arena.start) { std::cerr << "ARENA ERROR: no start cell (*) in " << opt.arenaPath << "\n"; return 1; }

    if (opt.calibrate) return calibrate(opt, arena);

    std::cout << "=== Exploration mapping: Bayesian occupancy grid + frontier exploration (Kobuki + 1x HC-SR04, 360 deg look-arounds) ===\n\n";
    bayes::SensorModel model;
    if (!bayes::loadSensorModel(opt.modelPath, model, error))
        std::cout << "!! Sensor model: " << error << "\n   Using the default Gaussian model (sigma(4 m) = 10 cm, P(hit|E) = 0.20) - run the calibration first (.\\run.ps1 11 -Calibrate).\n";
    const Cell start = *arena.start;
    const double headingDeg = arena.headingDeg.value_or(0.0);
    const astar::CoordStyle& cs = arena.coords;
    std::cout << std::fixed << std::setprecision(2)
              << "Arena    : " << opt.arenaPath << "\n"
              << "           " << arena.rows() << " rows x " << arena.cols() << " columns of " << arena.cellSize() * 100.0
              << " cm, start " << cs.text(start) << ", heading " << std::setprecision(0) << headingDeg << " deg"
              << (arena.walledBorder ? ", border wall" : "") << "\n"
              << std::setprecision(2)
              << "Sensor   : one HC-SR04 facing forward, " << SensorOffsetM * 100.0
              << " cm from the centre; the robot turns 360 deg to look around\n"
              << "Model    : [" << model.source << "]\n"
              << "Rule     : every cell starts at 0.50;  P < " << bayes::EmptyBelow << " empty,  P > " << bayes::OccupiedAbove << " occupied\n";
    printModelTable(model, arena.cellSize());
    std::cout << std::endl;

    // --- robot -------------------------------------------------------------
    const explore::Geometry geo{ SideBeamDeg, SensorOffsetM, KobukiRadiusM, false };   // front sensor only
    const nav::Pose startPose{ arena.centre(start).x, arena.centre(start).y, headingDeg * Deg };
    std::unique_ptr<explore::Robot> robot;
    astar::GridMap truth;
    if (opt.sim) {
        if (!astar::loadMap(opt.truthPath, truth, error)) { std::cerr << "TRUTH MAP ERROR: " << error << "\n"; return 1; }
        if (truth.cols() != arena.cols() || truth.rows() != arena.rows()) {
            std::cerr << "ERROR: the true map " << opt.truthPath << " is " << truth.rows() << " x " << truth.cols()
                      << ", the arena " << arena.rows() << " x " << arena.cols() << ".\n";
            return 1;
        }
        truth = astar::GridMap(truth);   // copy
        astar::GridMap world(arena.cols(), arena.rows(), arena.cellSize());
        for (int y = 0; y < arena.rows(); ++y)
            for (int x = 0; x < arena.cols(); ++x) world.setOccupied({ x, y }, truth.occupied({ x, y }));
        if (world.occupied(start)) { std::cerr << "ERROR: the start cell is an obstacle in the true map.\n"; return 1; }
        explore::SimRobot::Options so;
        so.geo = geo;
        so.walledBorder = arena.walledBorder;
        so.noise = opt.noise;
        so.speed = (opt.fast || !opt.gui) ? 0 : 4;
        robot = std::make_unique<explore::SimRobot>(world, startPose, so);
        std::cout << "Simulator: true map " << opt.truthPath << (opt.noise ? " (sensor noise, missed echoes, ghost readings)" : "") << "\n";
    } else {
        robot = std::make_unique<explore::HardwareRobot>(opt.kobukiPort, opt.arduinoPort, startPose);
    }

    // --- grid, explorer, window -------------------------------------------
    bayes::BayesGrid grid(arena);
    explore::Config cfg;
    if (opt.timeLimit > 0.0) cfg.timeLimitS = opt.timeLimit;
    cfg.verify = opt.verify;
    explore::Explorer explorer(grid, model, geo, cfg);

    view::GdiPlusSession gdiPlus;
    int picW = 1180, picH = 800;
    if (opt.gui) view::pictureSize(picW, picH);
    view::Canvas canvas(picW, picH);
    view::MapWindow window;
    if (opt.gui && !window.create(picW, picH, L"11 Exploration Mapping - Bayes grid + frontier"))
        std::cout << "(could not open the window - continuing without it)\n";

    if (!robot->open()) return 1;
    if (!opt.sim) {
        std::cout << "\nPlace the robot with its CENTRE on cell " << cs.text(start) << " facing " << std::setprecision(0)
                  << headingDeg << " deg (as in the arena file).\n"
                  << (opt.go ? "--go: starting in 3 s (Q / close the window to cancel).\n" : "Press ENTER to start, Q to quit.\n")
                  << std::flush;
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
    explorer.begin(s);
    std::cout << "\nExploring. Q / close the window = stop. (Ctrl+C also stops.)\n";

    using Clock = std::chrono::steady_clock;
    auto lastHud = Clock::now() - std::chrono::seconds(10), lastView = lastHud;
    double now = 0.0, dt = 0.0, driven = 0.0, cliffTime = 0.0;
    Point lastPoint = s.pose.point();
    const char* stopReason = nullptr;

    const auto info = [&]() {
        std::vector<std::wstring> v;
        wchar_t b[160];
        v.push_back(L"#Bayesian grid");
        swprintf(b, 160, L"empty %d   occupied %d   undecided %d", grid.count(bayes::State::Empty),
                 grid.count(bayes::State::Occupied), grid.count(bayes::State::Undecided));
        v.push_back(b);
        swprintf(b, 160, L"Gaussian: sigma(1 m) %.1f cm, sigma(4 m) %.1f cm", model.sigma(1.0) * 100.0, model.sigma(4.0) * 100.0);
        v.push_back(b);
        swprintf(b, 160, L"P(hit|O) 1 cell ahead %.2f   P(hit|E) %.2f", model.pHitOccupied(0.09, arena.cellSize()), model.pHitEmpty);
        v.push_back(b);
        v.push_back(L"P < 0.30 empty,  P > 0.80 occupied");
        v.push_back(L"#Exploration");
        swprintf(b, 160, L"frontier cells %d", static_cast<int>(explorer.frontier().size()));
        v.push_back(b);
        if (explorer.hasTarget()) {
            v.push_back(L"target " + wide(cs.text(explorer.target())) + L"  via " + wide(cs.text(explorer.goal())));
        }
        if (explorer.verifying()) v.push_back(L"verifying occupied cells face-on");
        swprintf(b, 160, L"look-arounds %d, plans %d, face-on checks %d", explorer.scans(), explorer.plans(),
                 explorer.verifiedCount());
        v.push_back(b);
        swprintf(b, 160, L"readings used %ld", explorer.readings());
        v.push_back(b);
        swprintf(b, 160, L"bumps %d, position fixes %d", explorer.bumps(), explorer.corrections());
        v.push_back(b);
        swprintf(b, 160, L"t %.0f s, driven %.1f m", now, driven);
        v.push_back(b);
        v.push_back(L"#Sonar (cm from the sensor)");
        const wchar_t* names[3] = { L"left 45", L"front", L"right 45" };
        for (int k = 0; k < 3; ++k) {
            if (!geo.uses(k)) continue;
            const auto& be = s.beams[k];
            if (!be.valid) swprintf(b, 160, L"%ls: --", names[k]);
            else if (!be.echo) swprintf(b, 160, L"%ls: no echo", names[k]);
            else swprintf(b, 160, L"%ls: %.0f", names[k], be.range * 100.0);
            v.push_back(b);
        }
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
        sc.title = L"Exploration mapping  -  Bayesian occupancy grid + frontier exploration";
        sc.status = status;
        sc.info = info();
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
        if (!ev.empty()) std::cout << "\n  [" << std::setprecision(0) << now << " s] " << ev << "\n";
        if (explorer.phase() == explore::Phase::Done) break;
        robot->drive(cmd.v, cmd.w);

        const auto t = Clock::now();
        if (std::chrono::duration<double>(t - lastHud).count() >= 0.25) {
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
        dt = robot->tick();
        now += dt;
    }
    robot->stop();

    // --- result --------------------------------------------------------------
    std::vector<Cell> undecided;
    for (int y = 0; y < arena.rows(); ++y)
        for (int x = 0; x < arena.cols(); ++x)
            if (grid.state({ x, y }) == bayes::State::Undecided) undecided.push_back({ x, y });
    const char* result = stopReason;
    switch (explorer.finished()) {
        case explore::Finish::AllDecided: result = "MAPPING COMPLETE - every cell decided"; break;
        case explore::Finish::NoFrontier: result = undecided.empty() ? "MAPPING COMPLETE - every cell decided"
                                                                    : "MAPPING COMPLETE - no frontier left (some cells can never be seen)"; break;
        case explore::Finish::TimeLimit: result = "STOPPED - time limit"; break;
        case explore::Finish::TooManyBumps: result = "STOPPED - too many bumps"; break;
        default: break;
    }
    draw(wide(result));

    std::error_code fsError;
    const std::filesystem::path dir = opt.outDir.empty() ? std::filesystem::path(folder("/output")) : std::filesystem::path(opt.outDir);
    std::filesystem::create_directories(dir, fsError);
    const std::string stem = "explored_" + timestamp();
    const auto txt = dir / (stem + ".txt"), prob = dir / (stem + "_prob.txt"), png = dir / (stem + ".png");
    const bool okTxt = saveMapTxt(txt, grid, start, headingDeg, undecided);
    bool okProb = false;
    {
        std::ofstream p(prob);
        if (p) {
            p << "# P(Occupied) per cell after exploration, " << dateText() << "\n"
              << "# tag: ' ' empty (P < 0.3)  '#' occupied (P > 0.8)  '?' undecided\n"
              << "# sensor model: sigma(d) = " << model.sigma0 << " + " << model.sigmaPerM << " d, P(detect) = " << model.pDetect
              << ", P(hit|E) = " << model.pHitEmpty << "\n";
            writeProbabilities(p, grid);
            okProb = static_cast<bool>(p);
        }
    }
    const bool okPng = canvas.savePng(png.wstring());

    std::cout << "\n\n--- Summary ---\n"
              << "Result      : " << result << "\n"
              << std::setprecision(1)
              << "Time        : " << now << " s, driven " << driven << " m\n"
              << "Cells       : " << grid.count(bayes::State::Empty) << " empty, " << grid.count(bayes::State::Occupied)
              << " occupied, " << undecided.size() << " undecided\n"
              << "Exploration : " << explorer.scans() << " look-arounds, " << explorer.plans() << " plans, "
              << explorer.verifiedCount() << " face-on checks, "
              << explorer.readings() << " sonar readings used, " << explorer.bumps() << " bumps\n";
    if (!undecided.empty()) {
        std::cout << "Undecided   :";
        for (const Cell& c : undecided) std::cout << " " << cs.text(c);
        std::cout << "  (written as 1)\n";
    }
    std::cout << "\nP(Occupied):\n";
    writeProbabilities(std::cout, grid);
    if (opt.sim) {
        int right = 0, wrong = 0, open = 0;
        std::vector<Cell> wrongCells;
        for (int y = 0; y < arena.rows(); ++y)
            for (int x = 0; x < arena.cols(); ++x) {
                const Cell c{ x, y };
                const auto st = grid.state(c);
                if (st == bayes::State::Undecided) { ++open; continue; }
                if ((st == bayes::State::Occupied) == truth.occupied(c)) ++right;
                else { ++wrong; wrongCells.push_back(c); }
            }
        std::cout << "\nCompared with the true map: " << right << " correct, " << wrong << " wrong, " << open << " undecided";
        for (const Cell& c : wrongCells) std::cout << " " << cs.text(c);
        std::cout << "\n";
    }
    std::cout << "\nSaved:\n  " << txt.string() << "   (0/1 map for A*)\n  " << prob.string() << "\n  " << png.string() << "\n";
    if (!okTxt || !okProb || !okPng) std::cout << "  !! some files could not be written to " << dir.string() << "\n";

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
    const bool ok = explorer.finished() == explore::Finish::AllDecided || explorer.finished() == explore::Finish::NoFrontier;
    return ok ? 0 : 2;
}
