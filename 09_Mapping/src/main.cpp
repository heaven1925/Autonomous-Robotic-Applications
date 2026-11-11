// Ornek 09 - Haritalama (Mapping): olasiliksal Occupancy Grid + frontier
// tabanli kesif (Kobuki + Arduino + 3x HC-SR04).
//
//   Gercek robot : Mapping.exe <Kobuki COM portu> <Arduino COM portu> [secenekler]
//   Simulasyon   : Mapping.exe --sim [secenekler]          (donanim gerekmez)
//   Kolay yol    : .\run.ps1 9   /   .\run.ps1 9 -Sim   /   .\run.ps1 9 -Manual
//
//   Secenekler:
//     --manual      robotu klavyeden sur (W/A/S/D, X = dur); harita yine cikarilir
//     --no-gui      harita penceresini acma (dosyalar yine kaydedilir)
//     --fast        simulasyonu bekleme yapmadan, olabildigince hizli kostur
//     --noise       simulasyonda sensor gurultusu / kacan yankilar
//     --grid <m>    sagdaki "grid map" hucre boyutu (varsayilan 0.50 m)
//     --radius <m>  baslangictan bu kadar uzaktaki frontier'lar yok sayilir
//     --time <s>    kesif icin azami sure
//     --out <dir>   haritalarin kaydedilecegi klasor
//
// Ne yapar (ders notundaki "Probabilistic OG Mapping Algorithm"):
//   1. Tum hucreler P(dolu) = 0.5 (bilinmiyor) ile baslar; robotun durdugu
//      nokta (0, 0), baktigi yon +x kabul edilir.
//   2. Her yeni sensor paketinde harita guncellenir: robotun bulundugu yer
//      "bos", her sonar okumasi sensor modeli + Bayes kurali ile islenir
//      (OccupancyGrid.h).
//   3. Harita esiklenir (bos / dolu / bilinmiyor), frontier'lar bulunur, en
//      yakinina yol planlanir ve gidilir (FrontierExplorer.h).
//   4. Frontier kalmayinca harita tamamdir: robot baslangica doner, harita
//      PNG + metin olarak kaydedilir.
//
// Guvenlik (planlayicidan bagimsiz): ucurum sensoru ya da sensor akisinin
// kesilmesi -> robot durur, o ana kadarki harita kaydedilir. Tampona carpma:
// carpilan nokta haritaya "dolu" yazilir, robot biraz geri cekilip yeniden
// planlar. Q / Ctrl+C / pencereyi kapatmak: dur ve haritayi kaydet.
#include "MapView.h"
#include "RobotIO.h"

#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <conio.h>
#include <cstdio>
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

// --- Robota ozgu ayarlar (montaj olculeri; 08_TangentBug ile ayni) ---------
constexpr double Deg = ogm::Pi / 180.0;
constexpr double SensorOffsetM = 0.11;   // robot merkezinden sensorlere uzaklik (m)
constexpr double SideBeamDeg   = 60.0;   // yan sensorlerin one gore acisi (derece)
constexpr double RobotRadiusM  = 0.18;   // Kobuki govde yaricapi
constexpr double RangeBiasM    = 0.005;  // sketch mesafeyi tam cm'ye asagi yuvarlar: ortalama 0.5 cm kisa olcer

// --- Harita ayarlari ------------------------------------------------------
constexpr double FineCellM      = 0.05;   // olasilik izgarasinin hucre boyutu (m)
constexpr double GridCellM      = 0.50;   // turetilen "grid map" hucre boyutu (m)
constexpr double ExploreRadiusM = 8.0;    // gercek robotta kesif yaricapi (acik kapidan cikip gitmesin)
constexpr double MaxTimeS       = 1800.0; // gercek robotta azami kesif suresi (s)

constexpr double HudPeriodS  = 0.25;  // canli durum satirinin yenilenme periyodu (s)
constexpr double ViewPeriodS = 0.20;  // harita penceresinin yenilenme periyodu (s)

std::atomic<bool> g_quit{false};

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) { g_quit = true; return TRUE; }
    return FALSE;
}

struct Options {
    bool sim = false, manual = false, gui = true, fast = false, noise = false;
    std::string kobukiPort, arduinoPort, outDir;
    double gridCell = GridCellM;
    double radius = 0.0;   // 0 = varsayilan
    double maxTime = 0.0;  // 0 = varsayilan
};

void usage(const char* exe) {
    std::cerr << "Usage: " << exe << " <Kobuki COM port> <Arduino COM port> [options]\n"
              << "       " << exe << " --sim [options]\n"
              << "Options: --manual  --no-gui  --fast  --noise  --grid <m>  --radius <m>  --time <s>  --out <dir>\n";
}

bool parseArgs(int argc, char** argv, Options& o) {
    std::vector<std::string> ports;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto number = [&](double& out) {
            if (i + 1 >= argc) return false;
            try { out = std::stod(argv[++i]); } catch (...) { return false; }
            return out > 0.0;
        };
        if (a == "--sim") o.sim = true;
        else if (a == "--manual") o.manual = true;
        else if (a == "--no-gui") o.gui = false;
        else if (a == "--fast") o.fast = true;
        else if (a == "--noise") o.noise = true;
        else if (a == "--grid") { if (!number(o.gridCell)) return false; }
        else if (a == "--radius") { if (!number(o.radius)) return false; }
        else if (a == "--time") { if (!number(o.maxTime)) return false; }
        else if (a == "--out") { if (i + 1 >= argc) return false; o.outDir = argv[++i]; }
        else if (!a.empty() && a[0] == '-') return false;
        else ports.push_back(a);
    }
    if (o.sim) return ports.empty();
    if (ports.size() != 2) return false;
    o.kobukiPort = ports[0];
    o.arduinoPort = ports[1];
    return true;
}

const char* stateName(explore::State s) {
    switch (s) {
        case explore::State::Init: return "INIT";
        case explore::State::Scan: return "SCAN";
        case explore::State::Plan: return "PLAN";
        case explore::State::Drive: return "DRIVE";
        case explore::State::BackOff: return "BACK_OFF";
        case explore::State::Align: return "ALIGN";
        case explore::State::Done: return "DONE";
        case explore::State::Fault: return "FAULT";
    }
    return "?";
}

// Sensorden olculen mesafeleri robot MERKEZINE gore mesafeye cevirir (refleks
// icin). Yanki yoksa "cok uzak" kabul edilir; gecersiz alan gecersiz kalir.
explore::Ranges centreRanges(const app::Snapshot& s) {
    const auto centre = [](const ogm::BeamReading& b) { return b.echo ? b.range + SensorOffsetM : 9.9; };
    explore::Ranges r;
    r.left = centre(s.left);   r.leftValid = s.left.valid;
    r.front = centre(s.front); r.frontValid = s.front.valid;
    r.right = centre(s.right); r.rightValid = s.right.valid;
    return r;
}

// Durum satiri icin: cm; -1 = yanki yok, -2 = gecersiz alan.
int cm(const ogm::BeamReading& b) {
    if (!b.valid) return -2;
    return b.echo ? static_cast<int>(std::lround(b.range * 100.0)) : -1;
}

// Tampon bir seye degdi: degen noktayi haritaya "dolu" olarak yaz.
void markBump(ogm::OccupancyGrid& grid, const explore::Pose& pose, app::Bumper b) {
    const double rel = b == app::Bumper::Left ? 45.0 * Deg : (b == app::Bumper::Right ? -45.0 * Deg : 0.0);
    const double a = pose.heading + rel, d = RobotRadiusM + 0.05;
    grid.markOccupiedDisc(pose.x + d * std::cos(a), pose.y + d * std::sin(a), 0.06);
}

// Klavye: harita penceresinden ya da konsoldan gelen son tus (buyuk harf).
// Ok tuslari W/A/S/D'ye cevrilir. 0 = tus yok.
int pollKey(view::MapWindow& window) {
    int k = window.takeKey();
    switch (k) {
        case VK_UP: return 'W';
        case VK_DOWN: return 'S';
        case VK_LEFT: return 'A';
        case VK_RIGHT: return 'D';
        default: break;
    }
    if (k != 0) return k;
    if (!_kbhit()) return 0;
    k = _getch();
    if (k == 0 || k == 224) {
        switch (_getch()) {
            case 72: return 'W';
            case 80: return 'S';
            case 75: return 'A';
            case 77: return 'D';
            default: return 0;
        }
    }
    return std::toupper(k);
}

// Elle surus: son basilan tus gecerli kalir (01_Sample teleop'u gibi).
struct Teleop {
    double v = 0.0, w = 0.0;

    void key(int k) {
        switch (k) {
            case 'W': v = 0.15; w = 0.0; break;
            case 'S': v = -0.10; w = 0.0; break;
            case 'A': v = 0.0; w = 0.6; break;
            case 'D': v = 0.0; w = -0.6; break;
            case 'X': case ' ': v = 0.0; w = 0.0; break;
            default: break;
        }
    }

    // Ondeki engele dogru ileri gidilmez (ayni refleks, otonom moddaki gibi).
    explore::Command command(const explore::Ranges& r, double dStop) const {
        const bool blocked = !r.frontValid || r.front < dStop;
        return { (v > 0.0 && blocked) ? 0.0 : v, w };
    }
};

std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

// Grid map'i 0/1 metin haritasi olarak kaydeder (ekrandaki yonelimle):
//   0 = bos, 1 = dolu YA DA bilinmiyor (gorulmemis yer guvenli sayilmaz).
// Bicim 10_AStarPathPlanning harita dosyasiyla aynidir (cell / start /
// heading satirlari + 0/1 izgara), yani dosya dogrudan A*'a verilebilir:
//   AStar.exe <bu dosya> --goal c,r
bool saveCellMapText(const std::filesystem::path& path, const ogm::CellMap& m, int quarterTurns) {
    if (m.nx <= 0 || m.ny <= 0) return false;
    view::Viewport vp;
    vp.k = quarterTurns;
    // Hucre indekslerini goruntu satir/sutununa cevir (90 derecelik donusler tam sayi kalir).
    const auto toRowCol = [&](int ix, int iy, int& col, int& row) {
        double u, v;
        vp.toView(ix, iy, u, v);
        col = static_cast<int>(std::lround(u));
        row = static_cast<int>(std::lround(-v));
    };
    int c0, r0, c1, r1;
    toRowCol(m.ix0, m.iy0, c0, r0);
    toRowCol(m.ix0 + m.nx - 1, m.iy0 + m.ny - 1, c1, r1);
    const int colMin = std::min(c0, c1), rowMin = std::min(r0, r1);
    const int cols = std::abs(c1 - c0) + 1, rows = std::abs(r1 - r0) + 1;

    std::vector<std::vector<int>> grid(static_cast<std::size_t>(rows), std::vector<int>(static_cast<std::size_t>(cols), 1));
    int counts[3] = {};  // bilinmiyor, bos, dolu
    for (int iy = m.iy0; iy < m.iy0 + m.ny; ++iy) {
        for (int ix = m.ix0; ix < m.ix0 + m.nx; ++ix) {
            int col, row;
            toRowCol(ix, iy, col, row);
            const ogm::Cell st = m.at(ix, iy);
            ++counts[static_cast<int>(st)];
            grid[static_cast<std::size_t>(row - rowMin)][static_cast<std::size_t>(col - colMin)] = st == ogm::Cell::Empty ? 0 : 1;
        }
    }
    int startCol, startRow;
    toRowCol(0, 0, startCol, startRow);   // robotun basladigi hucre
    double u, v;
    vp.toView(1.0, 0.0, u, v);            // robotun ilk yonu (+x) resimde nereye bakiyor
    const long heading = std::lround(std::atan2(v, u) * 180.0 / ogm::Pi);

    std::ofstream out(path);
    if (!out) return false;
    out << "# Kobuki grid map (09_Mapping): 0 = empty, 1 = occupied or unknown\n"
        << "# " << cols << " columns x " << rows << " rows; " << counts[1] << " empty, " << counts[2] << " occupied, "
        << counts[0] << " unknown cells (unknown written as 1)\n"
        << "# orientation as on screen, first grid line = row 0; start = where the robot started\n"
        << "# usable as an A* map: AStar.exe <this file> --goal <column>,<row>\n"
        << "cell " << m.cellSize << "\n"
        << "start " << startCol - colMin << " " << startRow - rowMin << "\n"
        << "heading " << heading << "\n";
    for (const auto& line : grid) {
        for (std::size_t i = 0; i < line.size(); ++i) out << (i ? " " : "") << line[i];
        out << "\n";
    }
    return static_cast<bool>(out);
}

// Yalnizca simulasyon: olasilik izgarasindaki "dolu" hucreler gercek yuzeye ne
// kadar yakin? (+ = yuzeyin onunde, bos alanda; - = duvarin icinde)
void compareFineWithTruth(const app::RobotIO& robot, const ogm::OccupancyGrid& g) {
    int bx0, by0, bx1, by1;
    if (!g.knownBounds(bx0, by0, bx1, by1)) return;
    int inside = 0, near5 = 0, near10 = 0, near20 = 0, beyond = 0, total = 0;
    for (int iy = by0; iy <= by1; ++iy) {
        for (int ix = bx0; ix <= bx1; ++ix) {
            if (g.state(ix, iy) != ogm::Cell::Occupied) continue;
            double d = 0.0;
            if (!robot.truthDistance(g.toWorld(ix), g.toWorld(iy), d)) continue;
            ++total;
            if (d <= 0.0) ++inside;
            else if (d <= 0.05) ++near5;
            else if (d <= 0.10) ++near10;
            else if (d <= 0.20) ++near20;
            else ++beyond;
        }
    }
    if (total == 0) return;
    const auto pct = [&](int n) { return 100.0 * n / total; };
    std::cout << "\nOccupied " << g.cellSize() * 100.0 << " cm cells vs. the real surfaces (" << total << " cells):\n"
              << std::fixed << std::setprecision(0)
              << "  inside the wall " << pct(inside) << "%, up to 5 cm in front " << pct(near5) << "%, 5-10 cm "
              << pct(near10) << "%, 10-20 cm " << pct(near20) << "%, farther " << pct(beyond) << "%\n";
}

// Yalnizca simulasyon: cikan grid map'i gercek planla hucre hucre karsilastir.
void compareWithTruth(const app::RobotIO& robot, const ogm::CellMap& m) {
    int wall[3] = {}, open[3] = {}, mixed[3] = {};  // indeks: 0 bilinmiyor, 1 bos, 2 dolu
    bool any = false;
    const double C = m.cellSize;
    for (int iy = m.iy0 - 40; iy < m.iy0 + m.ny + 40; ++iy) {
        for (int ix = m.ix0 - 40; ix < m.ix0 + m.nx + 40; ++ix) {
            double solid = 0.0;
            if (!robot.truthSolid((ix - 0.5) * C, (iy - 0.5) * C, (ix + 0.5) * C, (iy + 0.5) * C, solid)) continue;
            any = true;
            int* bucket = solid >= 0.90 ? wall : (solid <= 0.02 ? open : mixed);
            ++bucket[static_cast<int>(m.at(ix, iy))];
        }
    }
    if (!any) return;
    const auto sum = [](const int* b) { return b[0] + b[1] + b[2]; };
    std::cout << std::fixed << std::setprecision(2) << "\nGrid map vs. the simulated floor plan (" << C << " m cells):\n"
              << "  wall cells  : " << std::setw(3) << sum(wall) << " -> " << wall[2] << " occupied, " << wall[0]
              << " unknown (not visible from inside), " << wall[1] << " wrongly empty\n"
              << "  free cells  : " << std::setw(3) << sum(open) << " -> " << open[1] << " empty, " << open[0]
              << " unknown, " << open[2] << " wrongly occupied\n"
              << "  mixed cells : " << std::setw(3) << sum(mixed) << " -> " << mixed[2] << " occupied, " << mixed[1]
              << " empty, " << mixed[0] << " unknown  (cut by the diagonal wall / round obstacle)\n";
}

std::filesystem::path outputDir(const Options& o) {
    if (!o.outDir.empty()) return o.outDir;
#ifdef MAPPING_OUTPUT_DIR
    return MAPPING_OUTPUT_DIR;
#else
    return "map_output";
#endif
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "=== Mapping: occupancy grid + frontier exploration (Kobuki + 3x HC-SR04) ===\n\n";

    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(argv[0]); return 1; }

    std::unique_ptr<app::RobotIO> robot;
    if (opt.sim) {
        app::SimRobot::Options so;
        so.beamAngle = SideBeamDeg * Deg;
        so.sensorOffset = SensorOffsetM;
        so.robotRadius = RobotRadiusM;
        so.noise = opt.noise;
        so.speed = opt.fast ? 0 : 8;
        robot = std::make_unique<app::SimRobot>(so);
    } else {
        robot = std::make_unique<app::HardwareRobot>(opt.kobukiPort, opt.arduinoPort);
    }
    if (!robot->open()) return 1;
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    // Harita: olasilik izgarasi + sensor modeli (varsayilanlar OccupancyGrid.h'de).
    ogm::GridConfig gridCfg;
    gridCfg.cellSize = FineCellM;
    ogm::OccupancyGrid grid(gridCfg);
    ogm::RangeMapper mapper(grid, ogm::SensorModel{}, SideBeamDeg * Deg, SensorOffsetM, RobotRadiusM);
    ogm::CellMapConfig cellCfg;
    cellCfg.cellSize = opt.gridCell;

    // Kesif planlayicisi (varsayilanlar FrontierExplorer.h'de).
    explore::Config exploreCfg;
    exploreCfg.robotRadius = RobotRadiusM;
    exploreCfg.exploreRadius = opt.radius > 0.0 ? opt.radius : (opt.sim ? 20.0 : ExploreRadiusM);
    exploreCfg.maxTimeS = opt.maxTime > 0.0 ? opt.maxTime : (opt.sim ? 3600.0 : MaxTimeS);
    explore::FrontierExplorer explorer(exploreCfg);

    // Harita resmi + (istenirse) penceresi.
    view::GdiPlusSession gdiPlus;
    int picW = 1180, picH = 800;
    if (opt.gui) view::pictureSize(picW, picH);
    view::Canvas canvas(picW, picH);
    view::MapWindow window;
    if (opt.gui && !window.create(picW, picH, L"09 Mapping - occupancy grid + frontier exploration"))
        std::cout << "(could not open the map window - continuing without it)\n";
    int viewTurns = robot->viewQuarterTurns();

    std::cout << "\nRunning on: " << robot->name() << "\n"
              << "Start point = (0, 0), +x = the direction the robot faces now, +y = its left.\n";
    if (opt.manual)
        std::cout << "MANUAL mode - drive with W/A/S/D or the arrow keys, X or Space = stop.\n";
    else
        std::cout << "AUTONOMOUS mode - the robot looks around, drives to the nearest frontier and repeats\n"
                  << "until no frontier is left, then returns to the start.\n";
    std::cout << "Q = stop and save the map, R = rotate the picture. (Ctrl+C also stops.)\n\n";

    using Clock = std::chrono::steady_clock;
    const auto seconds = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b - a).count(); };
    auto lastHud = Clock::now() - std::chrono::seconds(10), lastView = lastHud;

    Teleop teleop;
    app::Snapshot s;
    explore::Pose prev;
    bool first = true;
    app::Bumper lastBumper = app::Bumper::None;
    explore::State lastState = explore::State::Init;
    explore::Point lastTarget;
    bool haveLastTarget = false;
    double dt = 0.0, runTime = 0.0, pathLength = 0.0;
    int bumps = 0;
    const char* stopReason = nullptr;
    ogm::CellMap cells;

    const auto makeScene = [&](const std::wstring& status) {
        view::Scene scene;
        scene.grid = &grid;
        scene.cells = &cells;
        scene.pose = s.pose;
        scene.robotRadius = RobotRadiusM;
        scene.path = &explorer.path();
        scene.frontier = opt.manual ? nullptr : &explorer.frontier();
        scene.hasTarget = explorer.hasTarget();
        scene.target = explorer.target();
        scene.quarterTurns = viewTurns;
        scene.title = L"Mapping - probabilistic occupancy grid (Bayes) + frontier exploration";
        scene.status = status;
        return scene;
    };

    const auto statusText = [&](const char* state) {
        std::wostringstream w;
        w << std::fixed << std::setprecision(1) << state << L"   |   t = " << runTime << L" s   |   driven "
          << pathLength << L" m   |   explored " << grid.count(ogm::Cell::Empty) * FineCellM * FineCellM << L" m2";
        if (!opt.manual) w << L"   |   frontiers left: " << explorer.frontierCount() << L"   |   look-arounds: " << explorer.scans();
        return w.str();
    };

    while (true) {
        s = robot->read();
        if (first) { prev = s.pose; first = false; }
        pathLength += std::hypot(s.pose.x - prev.x, s.pose.y - prev.y);
        prev = s.pose;

        // Guvenlik: planlayicidan bagimsiz.
        if (s.fault) { stopReason = s.fault; break; }
        if (!s.fresh) { stopReason = "sensor stream lost"; break; }
        if (g_quit) { stopReason = "stopped by the user (Ctrl+C)"; break; }

        // Harita guncellemesi: yalnizca yeni sensor paketinde, o anki pozla.
        if (s.newData) {
            if (s.left.echo) s.left.range += RangeBiasM;
            if (s.front.echo) s.front.range += RangeBiasM;
            if (s.right.echo) s.right.range += RangeBiasM;
            mapper.update(s.pose.x, s.pose.y, s.pose.heading, s.left, s.front, s.right);
        }

        const bool bumped = s.bumper != app::Bumper::None && lastBumper == app::Bumper::None;
        lastBumper = s.bumper;
        if (bumped) {
            ++bumps;
            markBump(grid, s.pose, s.bumper);
            teleop.key('X');
            if (!opt.manual) explorer.notifyBump(s.pose);
            std::cout << "\n  Bumper hit at (" << std::fixed << std::setprecision(2) << s.pose.x << ", " << s.pose.y
                      << ") - marked occupied.\n";
        }

        window.pump();
        const int key = pollKey(window);
        if (key == 'Q') { stopReason = "stopped by the user (Q)"; break; }
        if (key == 'R') viewTurns = (viewTurns + 1) & 3;
        if (window.closedByUser()) { stopReason = "map window closed"; break; }

        // Hiz komutu: elle surus ya da kesif planlayicisi.
        const explore::Ranges ranges = centreRanges(s);
        explore::Command cmd;
        const char* stateText = "MANUAL";
        if (opt.manual) {
            teleop.key(key);
            cmd = teleop.command(ranges, exploreCfg.dStop);
        } else {
            cmd = explorer.step(grid, s.pose, ranges, dt);
            stateText = stateName(explorer.state());

            // Onemli olaylari ekrana yaz.
            const explore::State st = explorer.state();
            if (st != lastState) {
                std::cout << std::fixed << std::setprecision(2);
                if (st == explore::State::Scan)
                    std::cout << "\n  Looking around at (" << s.pose.x << ", " << s.pose.y << ")...\n";
                else if (st == explore::State::BackOff)
                    std::cout << "\n  Cannot get there - backing off and planning again.\n";
                else if (st == explore::State::Align)
                    std::cout << "\n  Back at the start - turning to the initial heading.\n";
                lastState = st;
            }
            if (explorer.hasTarget()) {
                const explore::Point t = explorer.target();
                if (!haveLastTarget || std::hypot(t.x - lastTarget.x, t.y - lastTarget.y) > 0.6) {
                    std::cout << std::fixed << std::setprecision(2) << "\n  "
                              << (explorer.goal() == explore::GoalKind::Home ? "No frontier left - map complete. Returning to the start"
                                                                             : "Heading for the frontier at")
                              << " (" << t.x << ", " << t.y << "), " << explorer.frontierCount() << " frontier(s) left.\n";
                    lastTarget = t;
                    haveLastTarget = true;
                }
            }
            if (explorer.finished()) break;
        }
        robot->drive(cmd.v, cmd.w);

        // Canli durum satiri + harita penceresi (gercek zamana gore).
        const auto now = Clock::now();
        if (seconds(lastHud, now) >= HudPeriodS) {
            lastHud = now;
            std::cout << "\r  " << std::left << std::setw(9) << stateText << std::right
                      << " L " << std::setw(3) << cm(s.left) << " F " << std::setw(3) << cm(s.front)
                      << " R " << std::setw(3) << cm(s.right) << " cm"
                      << std::fixed << std::setprecision(2) << " | (" << s.pose.x << ", " << s.pose.y << ") "
                      << std::setprecision(0) << std::setw(4) << s.pose.heading / Deg << " deg"
                      << std::setprecision(1) << " | explored " << grid.count(ogm::Cell::Empty) * FineCellM * FineCellM
                      << " m2 | t " << runTime << " s   " << std::flush;
        }
        if (window.isOpen() && seconds(lastView, now) >= ViewPeriodS) {
            lastView = now;
            cells = ogm::makeCellMap(grid, cellCfg);
            view::render(canvas, makeScene(statusText(stateText)));
            window.present(canvas);
        }

        dt = robot->tick();
        runTime += dt;
    }
    robot->stop();

    // --- Sonuc: haritayi ciz, kaydet, ozetle --------------------------------
    const char* result = stopReason;
    if (!result) {
        if (explorer.state() == explore::State::Done) result = "EXPLORATION COMPLETE (no frontier left)";
        else if (explorer.faultReason() == explore::FaultReason::TimeLimit) result = "STOPPED (time limit reached)";
        else result = "STOPPED (robot got stuck too often)";
    }
    cells = ogm::makeCellMap(grid, cellCfg);
    view::render(canvas, makeScene(statusText(opt.manual ? "MANUAL" : stateName(explorer.state()))));
    window.present(canvas);

    std::error_code fsError;
    const std::filesystem::path dir = outputDir(opt);
    std::filesystem::create_directories(dir, fsError);
    const std::string stem = "map_" + timestamp();
    const std::filesystem::path fullPng = dir / (stem + ".png"), gridPng = dir / (stem + "_grid.png"),
                                gridTxt = dir / (stem + "_grid.txt");
    const bool savedFull = canvas.savePng(fullPng.wstring());
    bool savedGrid = false;
    {
        // Yalnizca grid map: once genis bir tuvale ciz, sonra icerigi kirp.
        view::Canvas big(1000, 1000);
        auto scene = makeScene(L"");
        big.fill(0, 0, big.width(), big.height(), RGB(255, 255, 255));
        const RECT area{ 8, 8, big.width() - 8, big.height() - 8 };
        const view::Viewport vp = view::fitViewport(cells, viewTurns, area);
        view::drawCellMap(big, scene, vp);
        view::Canvas cropped(vp.w + 16, vp.h + 16);
        cropped.fill(0, 0, cropped.width(), cropped.height(), RGB(255, 255, 255));
        BitBlt(cropped.dc(), 8, 8, vp.w + 1, vp.h + 1, big.dc(), vp.ox, vp.oy, SRCCOPY);
        savedGrid = cropped.savePng(gridPng.wstring());
    }
    const bool savedTxt = saveCellMapText(gridTxt, cells, viewTurns);

    std::cout << "\n\n--- Summary ---\n"
              << "Result        : " << result << "\n"
              << std::fixed << std::setprecision(2)
              << "Final pose    : (" << s.pose.x << ", " << s.pose.y << ")  heading " << std::setprecision(0)
              << s.pose.heading / Deg << " deg\n"
              << std::setprecision(1)
              << "Run time      : " << runTime << " s\n"
              << "Path length   : " << pathLength << " m\n"
              << "Explored area : " << grid.count(ogm::Cell::Empty) * FineCellM * FineCellM << " m2 empty, "
              << grid.count(ogm::Cell::Occupied) << " occupied cells of " << FineCellM * 100.0 << " cm\n"
              << "Grid map      : " << cells.count(ogm::Cell::Occupied) << " occupied, " << cells.count(ogm::Cell::Empty)
              << " empty cells of " << std::setprecision(2) << cells.cellSize << " m\n";
    if (!opt.manual)
        std::cout << "Exploration   : " << explorer.scans() << " look-around turns, " << explorer.plans() << " plans, "
                  << explorer.givenUp() << " goals given up\n";
    std::cout << "Bumper hits   : " << bumps << "\n";
    compareFineWithTruth(*robot, grid);
    compareWithTruth(*robot, cells);

    std::cout << "\nSaved:\n";
    if (savedFull) std::cout << "  " << fullPng.string() << "\n";
    if (savedGrid) std::cout << "  " << gridPng.string() << "\n";
    if (savedTxt) std::cout << "  " << gridTxt.string() << "\n";
    if (!savedFull || !savedGrid || !savedTxt) std::cout << "  !! could not write some files to " << dir.string() << "\n";

    robot->shutdown();

    // Harita penceresi, kullanici kapatana kadar acik kalir.
    if (window.isOpen() && !g_quit) {
        std::cout << "\nThe map window stays open - close it (or press Q / Enter) to exit.\n";
        while (window.isOpen() && !g_quit) {
            window.pump();
            const int key = pollKey(window);
            if (key == 'Q' || key == VK_RETURN) break;
            if (key == 'R') {
                viewTurns = (viewTurns + 1) & 3;
                view::render(canvas, makeScene(statusText("FINISHED")));
                window.present(canvas);
            }
            Sleep(30);
        }
    }
    std::cout << "\nBye.\n";
    const bool ok = !stopReason && explorer.state() == explore::State::Done;
    return (ok || (opt.manual && stopReason && !s.fault && s.fresh)) ? 0 : 2;
}
