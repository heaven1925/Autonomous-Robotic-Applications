// Ornek 10 - A* ile yol planlama: .txt dosyasindaki 0/1 harita uzerinde,
// robot yaricapi kadar pay birakarak en kisa yolu bulur ve Kobuki'yi o yol
// boyunca surer.
//
//   Yalniz plan  : AStar.exe [harita.txt] [secenekler]
//   Simulasyon   : AStar.exe [harita.txt] --sim [secenekler]
//   Gercek robot : AStar.exe [harita.txt] <Kobuki COM portu> [secenekler]
//   Kolay yol    : .\run.ps1 10 -Sim   /   .\run.ps1 10 -PlanOnly   /   .\run.ps1 10
//   (harita verilmezse maps/lab.txt - laboratuvar labirenti - kullanilir)
//
//   Secenekler:
//     --start c,r    baslangic hucresi (sutun, satir); dosyadakini ezer
//     --goal c,r     hedef hucresi
//     --heading d    robotun baslangictaki yonu, derece (0 = saga, 90 = yukari)
//     --cell m       hucre boyutu (m); dosyadakini ezer (varsayilan 0.20)
//     --radius m     robot yaricapi (varsayilan 0.15)
//     --margin m     yaricapa eklenecek ek guvenlik payi (varsayilan 0)
//     --4            4 komsulu arama (yalniz duz hareket); varsayilan 8 komsulu
//     --smooth       A* yolunu "gorus hatti" ile kisalt (daha az donus)
//     --walled       harita kenarini da duvar say
//     --strict       hedef engele cok yakinsa en yakin guvenli hucreye kaydirma, dur
//     --block c,r    (yalniz --sim) haritada OLMAYAN bir engel ekle: robot
//                    tampona carpinca onu haritaya ekleyip yeniden planlar
//     --no-gui  --fast  --out <klasor>
//
// Adimlar:
//   1. Harita okunur: 1 = engel, 0 = bos; her hucre `cell` metre (20 cm).
//   2. Konfigurasyon uzayi: merkezi bir engele robot yaricapindan (15 cm)
//      yakin olan bos hucreler "sisirilir" - robotun MERKEZI oraya giremez.
//      Boylece merkezin gectigi her yerde govde engellere degmez.
//   3. A*: f = g + h; g = baslangictan gelinen yol (duz 1, capraz sqrt(2)
//      hucre), h = hedefe "octile" mesafe (kabul edilebilir + tutarli).
//      Kose kesme yok: capraz adim icin yanindaki iki hucre de serbest olmali.
//   4. Yol, yon degistirdigi hucrelere (kose noktalarina) indirgenir; robot
//      her kosede yerinde donup bir sonraki koseye duz gider.
//   5. Tampon bir seye degerse (haritada olmayan engel): robot durur, 10 cm
//      geri cekilir, o hucreyi engel olarak ekler ve bulundugu yerden yeniden
//      A* calistirir.
#include "PathFollower.h"
#include "PlanView.h"
#include "Robot.h"

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
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using astar::Cell;
using astar::Point;
using nav::Deg;

// --- Ayarlar ---------------------------------------------------------------
constexpr double DefaultRadiusM = 0.15;   // robot yaricapi (odev: 15 cm; Kobuki'nin gercegi ~17.5 cm)
constexpr double SmoothExtraM   = 0.05;   // --smooth: kestirmeler yaricaptan bu kadar fazla bosluk birakir
constexpr double BackOffM       = 0.10;   // tampondan sonra geri cekilme (m)
constexpr double BackOffSpeed   = 0.08;   // m/s
constexpr int    MaxReplans     = 10;     // bu kadar carpmadan sonra vazgec
constexpr double KobukiBodyM    = 0.175;  // Kobuki govdesinin gercek yaricapi (tampon temasi icin)
constexpr double MaxDriftM      = 0.12;   // tampon temasinda kabul edilen en buyuk konum hatasi
constexpr double CliffAbortS    = 1.5;    // ucurum sinyali bu kadar surerse dur (kisa sinyal: bekle, devam)
constexpr double HudPeriodS     = 0.25;
constexpr double ViewPeriodS    = 0.10;

std::atomic<bool> g_quit{false};

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) { g_quit = true; return TRUE; }
    return FALSE;
}

struct Options {
    std::string mapPath, port, outDir;
    bool sim = false, gui = true, fast = false, smooth = false, four = false, walled = false, strict = false, go = false;
    std::optional<Cell> start, goal;
    std::optional<double> heading, cell;
    double radius = DefaultRadiusM, margin = 0.0;
    std::vector<Cell> hidden;

    bool hardware() const { return !port.empty(); }
    bool planOnly() const { return !sim && !hardware(); }
};

void usage(const char* exe) {
    std::cerr << "Usage: " << exe << " [map.txt] [options]                 plan only\n"
              << "       " << exe << " [map.txt] --sim [options]           plan + drive in the simulator\n"
              << "       " << exe << " [map.txt] <Kobuki COM> [options]    plan + drive the real robot\n"
              << "Options: --start c,r  --goal c,r  --heading deg  --cell m  --radius m  --margin m\n"
              << "         --4  --smooth  --walled  --strict  --block c,r (sim)  --no-gui  --fast  --out dir  --go (real robot: start after 3 s, no Enter)\n";
}

bool isPort(const std::string& s) {
    if (s.size() < 4) return false;
    std::string up = s.substr(0, 3);
    for (char& ch : up) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    return up == "COM" && std::all_of(s.begin() + 3, s.end(), [](char ch) { return std::isdigit(static_cast<unsigned char>(ch)); });
}

// "3,4" / "3;4" / "3 4" (tek arguman) ya da "3" "4" (iki arguman).
bool readCell(int argc, char** argv, int& i, Cell& out) {
    if (i + 1 >= argc) return false;
    std::string t = argv[++i];
    for (char& ch : t)
        if (ch == ',' || ch == ';' || ch == ':' || ch == '(' || ch == ')') ch = ' ';
    std::istringstream in(t);
    int a = 0, b = 0;
    if (in >> a) {
        if (in >> b) { out = { a, b }; return true; }
        if (i + 1 < argc) {
            std::istringstream next(argv[i + 1]);
            if (next >> b) { ++i; out = { a, b }; return true; }
        }
    }
    return false;
}

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto number = [&](double& out, bool allowZero) {
            if (i + 1 >= argc) return false;
            try { out = std::stod(argv[++i]); } catch (...) { return false; }
            return allowZero ? out >= 0.0 : out > 0.0;
        };
        Cell c;
        double d = 0.0;
        if (a == "--sim") o.sim = true;
        else if (a == "--plan") {}
        else if (a == "--no-gui") o.gui = false;
        else if (a == "--fast") o.fast = true;
        else if (a == "--smooth") o.smooth = true;
        else if (a == "--4") o.four = true;
        else if (a == "--8") o.four = false;
        else if (a == "--walled") o.walled = true;
        else if (a == "--strict") o.strict = true;
        else if (a == "--go") o.go = true;
        else if (a == "--start") { if (!readCell(argc, argv, i, c)) return false; o.start = c; }
        else if (a == "--goal") { if (!readCell(argc, argv, i, c)) return false; o.goal = c; }
        else if (a == "--block") { if (!readCell(argc, argv, i, c)) return false; o.hidden.push_back(c); }
        else if (a == "--heading") {
            if (i + 1 >= argc) return false;
            try { o.heading = std::stod(argv[++i]); } catch (...) { return false; }
        }
        else if (a == "--cell") { if (!number(d, false)) return false; o.cell = d; }
        else if (a == "--radius") { if (!number(o.radius, false)) return false; }
        else if (a == "--margin") { if (!number(o.margin, true)) return false; }
        else if (a == "--out") { if (i + 1 >= argc) return false; o.outDir = argv[++i]; }
        else if (!a.empty() && a[0] == '-') { std::cerr << "Unknown option: " << a << "\n"; return false; }
        else if (isPort(a)) { if (!o.port.empty()) return false; o.port = a; }
        else { if (!o.mapPath.empty()) return false; o.mapPath = a; }
    }
    if (o.sim && o.hardware()) { std::cerr << "--sim and a COM port cannot be used together.\n"; return false; }
    if (!o.hidden.empty() && !o.sim) { std::cerr << "--block only works with --sim.\n"; return false; }
    return true;
}

std::string defaultMap() {
#ifdef ASTAR_MAPS_DIR
    return std::string(ASTAR_MAPS_DIR) + "/lab.txt";
#else
    return "maps/lab.txt";
#endif
}

std::filesystem::path outputDir(const Options& o) {
    if (!o.outDir.empty()) return o.outDir;
#ifdef ASTAR_OUTPUT_DIR
    return ASTAR_OUTPUT_DIR;
#else
    return "astar_output";
#endif
}

std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

// Hucreler haritanin "coords" satirindaki yazimla gosterilir (orn. satir,sutun 1'den).
astar::CoordStyle g_coords;
std::string cellText(Cell c) { return g_coords.text(c); }
std::wstring wide(const std::string& s) { return std::wstring(s.begin(), s.end()); }

// Ayni haritayi farkli hucre boyutuyla yeniden kurar (--cell).
astar::GridMap withCellSize(const astar::GridMap& m, double cell) {
    astar::GridMap out(m.cols(), m.rows(), cell);
    for (int y = 0; y < m.rows(); ++y)
        for (int x = 0; x < m.cols(); ++x) out.setOccupied({ x, y }, m.occupied({ x, y }));
    out.start = m.start;
    out.goal = m.goal;
    out.headingDeg = m.headingDeg;
    out.name = m.name;
    out.coords = m.coords;
    out.walledBorder = m.walledBorder;
    return out;
}

const char* headingName(double deg) {
    const double a = std::fmod(std::fmod(deg, 360.0) + 360.0, 360.0);
    if (std::abs(a) < 1e-9) return "right (increasing column)";
    if (std::abs(a - 90.0) < 1e-9) return "up (towards the first row)";
    if (std::abs(a - 180.0) < 1e-9) return "left (towards the first column)";
    if (std::abs(a - 270.0) < 1e-9) return "down (increasing row)";
    return "see the angle";
}

// Konsol / metin dosyasi icin haritanin ASCII cizimi.
//   #  engel          +  robot merkezi giremez (engele yaricaptan yakin)
//   .  bos            o  A* yolu            *  baslangic     /  hedef
//   X  tamponla bulunan engel                R  surus sonunda robotun hucresi
void writeGrid(std::ostream& out, const astar::CostMap& cm, const std::vector<Cell>& path, Cell start, Cell goal,
               const std::vector<Cell>& marked, const std::optional<Cell>& robot = std::nullopt) {
    const astar::GridMap& m = cm.map();
    std::vector<char> ch(static_cast<std::size_t>(m.cols()) * static_cast<std::size_t>(m.rows()), '.');
    for (int y = 0; y < m.rows(); ++y)
        for (int x = 0; x < m.cols(); ++x) {
            const Cell c{ x, y };
            char& k = ch[static_cast<std::size_t>(m.index(c))];
            if (cm.obstacle(c)) k = '#';
            else if (cm.inflated(c)) k = '+';
        }
    for (const Cell& c : marked) if (m.inside(c)) ch[static_cast<std::size_t>(m.index(c))] = 'X';
    for (const Cell& c : path) if (m.inside(c)) ch[static_cast<std::size_t>(m.index(c))] = 'o';
    if (robot && m.inside(*robot)) ch[static_cast<std::size_t>(m.index(*robot))] = 'R';
    if (m.inside(start)) ch[static_cast<std::size_t>(m.index(start))] = '*';
    if (m.inside(goal)) ch[static_cast<std::size_t>(m.index(goal))] = '/';

    const int base = m.coords.base;   // numbers as in the map file (0- or 1-based)
    if (m.cols() - 1 + base >= 10) {
        out << "      ";
        for (int x = 0; x < m.cols(); ++x) {
            const int n = x + base;
            out << (n >= 10 ? static_cast<char>('0' + (n / 10) % 10) : ' ') << ' ';
        }
        out << "\n";
    }
    out << "      ";
    for (int x = 0; x < m.cols(); ++x) out << (x + base) % 10 << ' ';
    out << "\n";
    for (int y = 0; y < m.rows(); ++y) {
        out << std::setw(4) << m.coords.rowLabel(y) << "  ";
        for (int x = 0; x < m.cols(); ++x) out << ch[static_cast<std::size_t>(m.index({ x, y }))] << ' ';
        out << "\n";
    }
}

// Klavye: pencereden ya da konsoldan gelen son tus (buyuk harf), 0 = yok.
int pollKey(view::MapWindow& window) {
    int k = window.takeKey();
    if (k != 0) return k;
    if (!_kbhit()) return 0;
    k = _getch();
    if (k == 0 || k == 224) { (void)_getch(); return 0; }
    return k == '\r' ? VK_RETURN : std::toupper(k);
}

// Bir planin her sey dahil sonucu.
struct Plan {
    astar::SearchResult search;              // A*
    astar::SearchResult dijkstra;            // ayni arama h = 0 ile (karsilastirma icin)
    std::vector<Cell> corners;               // yolun yon degistirdigi hucreler
    std::vector<Point> route;                // robotun surecegi noktalar
    double clearance = 0.0;                  // rota boyunca engele en yakin mesafe (m)
};

Plan makePlan(const astar::CostMap& cm, Cell start, Cell goal, const Options& o) {
    const auto conn = o.four ? astar::Connectivity::Four : astar::Connectivity::Eight;
    Plan p;
    p.search = astar::search(cm, start, goal, conn, 1.0);
    p.dijkstra = astar::search(cm, start, goal, conn, 0.0);
    if (!p.search.found) return p;
    p.corners = astar::corners(p.search.path);
    p.route = o.smooth ? astar::shortcut(cm, astar::toPoints(cm.map(), p.search.path), cm.radius() + SmoothExtraM)
                       : astar::toPoints(cm.map(), p.corners);
    p.clearance = astar::minClearance(cm, p.route, 2.0);
    return p;
}

// Tampon, haritada ZATEN bilinen bir engele (ya da duvar kenara) mi degdi?
// Tamponun baktigi yon +-70 derece icinde, govde yaricapi + MaxDriftM'den
// yakin bilinen en yakin engel noktasini arar. Bulursa: u = merkezden o
// noktaya birim vektor, dist = tahmini merkezden uzaklik. Gercekte merkez
// engelden tam govde yaricapi kadar uzakta oldugu icin konum tahmini
// u * (dist - govde yaricapi) kadar kaydirilir (odometri kaymasi duzeltilir).
bool knownContact(const astar::CostMap& cm, const nav::Pose& pose, nav::Bumper b, double body, Point& u,
                  double& dist) {
    const double rel = b == nav::Bumper::Left ? 45.0 * Deg : (b == nav::Bumper::Right ? -45.0 * Deg : 0.0);
    const double a = pose.heading + rel;
    const astar::GridMap& m = cm.map();
    const Point p = pose.point();
    const double reach = body + MaxDriftM;
    dist = reach;
    bool found = false;
    const auto consider = [&](Point q) {
        const double d = astar::distance(p, q);
        if (d < 1e-6 || d >= dist) return;
        if (std::abs(nav::normalizeAngle(std::atan2(q.y - p.y, q.x - p.x) - a)) > 70.0 * Deg) return;
        dist = d;
        u = { (q.x - p.x) / d, (q.y - p.y) / d };
        found = true;
    };
    const Cell at = m.cellAt(p);
    const int w = static_cast<int>(std::ceil(reach / m.cellSize())) + 1;
    for (int y = at.y - w; y <= at.y + w; ++y)
        for (int x = at.x - w; x <= at.x + w; ++x) {
            if (!m.occupied({ x, y })) continue;
            double x0, y0, x1, y1;
            m.bounds({ x, y }, x0, y0, x1, y1);
            consider({ std::clamp(p.x, x0, x1), std::clamp(p.y, y0, y1) });
        }
    if (cm.walledBorder()) {
        consider({ 0.0, p.y });
        consider({ m.width(), p.y });
        consider({ p.x, 0.0 });
        consider({ p.x, m.height() });
    }
    return found;
}

// Tampon: degilen noktanin hucresi (robotun kendi hucresi degil).
std::optional<Cell> bumpCell(const astar::CostMap& cm, const nav::Pose& pose, nav::Bumper b, double radius) {
    const double rel = b == nav::Bumper::Left ? 45.0 * Deg : (b == nav::Bumper::Right ? -45.0 * Deg : 0.0);
    const double a = pose.heading + rel;
    const astar::GridMap& m = cm.map();
    const Cell own = m.cellAt(pose.point());
    for (double d = radius + 0.02; d < radius + 2.0 * m.cellSize(); d += 0.02) {
        const Cell c = m.cellAt({ pose.x + d * std::cos(a), pose.y + d * std::sin(a) });
        if (!(c == own)) {
            if (!m.inside(c)) return std::nullopt;
            return c;
        }
    }
    return std::nullopt;
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "=== A* path planning on a 0/1 grid map (Kobuki) ===\n\n";

    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(argv[0]); return 1; }
    if (opt.mapPath.empty()) opt.mapPath = defaultMap();

    // --- 1. Harita --------------------------------------------------------
    astar::GridMap map;
    std::string error;
    if (!astar::loadMap(opt.mapPath, map, error)) { std::cerr << "MAP ERROR: " << error << "\n"; return 1; }
    if (opt.cell) map = withCellSize(map, *opt.cell);
    g_coords = map.coords;
    if (opt.start) map.start = map.coords.fromUser(opt.start->x, opt.start->y);   // --start / --goal are written
    if (opt.goal) map.goal = map.coords.fromUser(opt.goal->x, opt.goal->y);       // like the map file ("coords")
    for (Cell& c : opt.hidden) c = map.coords.fromUser(c.x, c.y);
    const bool walled = opt.walled || map.walledBorder;
    if (opt.heading) map.headingDeg = opt.heading;
    const std::string mapName = map.name.empty() ? std::filesystem::path(opt.mapPath).stem().string() : map.name;

    std::cout << std::fixed << std::setprecision(2)
              << "Map      : " << opt.mapPath << "\n"
              << "           " << map.cols() << " x " << map.rows() << " cells of " << map.cellSize() * 100.0 << " cm = "
              << map.width() << " x " << map.height() << " m, " << map.obstacleCount() << " obstacle cells\n"
              << "           cells written as (" << map.coords.order() << "), counting from " << map.coords.base
              << (walled ? "; the map border is a wall" : "") << "\n";

    if (!map.start || !map.goal) {
        std::cerr << "MAP ERROR: no " << (!map.start ? "start" : "goal") << " cell - add a line like '"
                  << (!map.start ? "start 0 0" : "goal 9 2") << "' to the map file (or an "
                  << (!map.start ? "*" : "/") << " in the grid), or use --" << (!map.start ? "start" : "goal") << " c,r\n";
        return 1;
    }
    const Cell start = *map.start;
    Cell goal = *map.goal;
    for (const auto& [label, c] : { std::pair{ "start", start }, std::pair{ "goal", goal } }) {
        if (!map.inside(c)) {
            std::cerr << "ERROR: the " << label << " cell " << cellText(c) << " is outside the map (" << map.rows() << " rows x "
                      << map.cols() << " columns; cells are written as " << map.coords.order() << ", counting from "
                      << map.coords.base << ").\n";
            return 1;
        }
        if (map.occupied(c)) {
            std::cerr << "ERROR: the " << label << " cell " << cellText(c) << " is an obstacle (1 in the map).\n";
            return 1;
        }
    }

    // --- 2. Konfigurasyon uzayi: robot yaricapi kadar sisirme ---------------
    const double clearanceNeed = opt.radius + opt.margin;
    astar::CostMap cm(map, clearanceNeed, walled);
    std::cout << "Robot    : radius " << opt.radius * 100.0 << " cm";
    if (opt.margin > 0.0) std::cout << " + " << opt.margin * 100.0 << " cm margin";
    std::cout << " -> " << cm.count(astar::CellState::Inflated)
              << " free cells are too close to an obstacle for the robot centre\n";

    const auto conn = opt.four ? astar::Connectivity::Four : astar::Connectivity::Eight;
    if (cm.inflated(start))
        std::cout << "WARNING  : the start cell " << cellText(start) << " is closer than "
                  << clearanceNeed * 100.0 << " cm to an obstacle - the robot may already touch it.\n";

    std::optional<Cell> requestedGoal;
    if (!cm.traversable(goal)) {
        const double gap = cm.clearance(map.centre(goal), 1.0);
        std::cout << "GOAL     : the centre of " << cellText(goal) << " is only " << gap * 100.0
                  << " cm from an obstacle (< " << clearanceNeed * 100.0 << " cm) - the robot would hit it there.\n";
        Cell safe;
        if (opt.strict || !astar::nearestSafeCell(cm, start, goal, safe, conn)) {
            std::cerr << "ERROR: no safe goal" << (opt.strict ? " (--strict)" : "") << ". Choose another goal cell.\n";
            return 3;
        }
        std::cout << "           using the nearest safe cell " << cellText(safe) << " instead (--strict to refuse).\n";
        requestedGoal = goal;
        goal = safe;
    }

    const Point startPoint = map.centre(start);
    const double headingDeg = map.headingDeg.value_or(0.0);
    std::cout << "Start    : " << cellText(start) << " = (" << startPoint.x << ", " << startPoint.y << ") m, facing "
              << std::setprecision(0) << headingDeg << " deg = " << headingName(headingDeg) << "\n"
              << std::setprecision(2) << "Goal     : " << cellText(goal) << " = (" << map.centre(goal).x << ", "
              << map.centre(goal).y << ") m\n\n";

    // --- 3. A* --------------------------------------------------------------
    Plan plan = makePlan(cm, start, goal, opt);
    std::vector<Cell> marked;
    if (plan.search.found) {
        std::cout << "A* (" << (opt.four ? "4" : "8") << "-connected): path of " << plan.search.path.size() << " cells, "
                  << plan.search.cost << " m, " << plan.corners.size() - 1 << " straight legs\n"
                  << "           expanded " << plan.search.expanded << " cells (Dijkstra would expand "
                  << plan.dijkstra.expanded << "), " << std::setprecision(3) << plan.search.milliseconds << " ms\n"
                  << std::setprecision(2);
        if (opt.smooth)
            std::cout << "Shortened : " << plan.route.size() - 1 << " legs, " << astar::length(plan.route) << " m\n";
        std::cout << "Clearance: the robot centre stays at least " << std::setprecision(1) << plan.clearance * 100.0
                  << " cm from every obstacle (needs >= " << clearanceNeed * 100.0 << " cm)";
        if (plan.clearance < clearanceNeed - 1e-9 && cm.inflated(start)) std::cout << " - only while leaving the start";
        std::cout << "\n\n" << std::setprecision(2);
    } else {
        std::cout << "A*: NO PATH from " << cellText(start) << " to " << cellText(goal) << " after expanding "
                  << plan.search.expanded << " cells.\n"
                  << "    Every way is blocked or narrower than the robot (" << 2 * clearanceNeed * 100.0 << " cm).\n\n";
    }
    std::cout << "Legend: # obstacle   + too close for the robot centre   . free   o A* path   * start   / goal\n";
    writeGrid(std::cout, cm, plan.search.path, start, goal, marked);
    std::cout << "\n";

    // --- Resim + pencere ---------------------------------------------------
    view::GdiPlusSession gdiPlus;
    int picW = 1180, picH = 800;
    if (opt.gui) view::pictureSize(picW, picH);
    view::Canvas canvas(picW, picH);
    view::MapWindow window;
    if (opt.gui && !window.create(picW, picH, L"10 A* path planning"))
        std::cout << "(could not open the window - continuing without it)\n";

    std::vector<Point> trail;
    nav::Snapshot snap;
    snap.pose = { startPoint.x, startPoint.y, headingDeg * Deg };
    bool showRobot = false;
    nav::PathFollower follower;
    const char* phaseText = "PLANNED";
    double runTime = 0.0, driven = 0.0, finalError = -1.0;
    int bumps = 0, replans = 0, corrections = 0;

    const auto infoLines = [&]() {
        std::vector<std::wstring> v;
        wchar_t b[160];
        v.push_back(L"#Map");
        v.push_back(wide(std::filesystem::path(opt.mapPath).filename().string()));
        swprintf(b, 160, L"%d x %d cells of %.0f cm  (%.2f x %.2f m)", map.cols(), map.rows(), map.cellSize() * 100.0,
                 map.width(), map.height());
        v.push_back(b);
        swprintf(b, 160, L"%d obstacle cells, %d too close (radius %.0f cm)", cm.count(astar::CellState::Obstacle),
                 cm.count(astar::CellState::Inflated), clearanceNeed * 100.0);
        v.push_back(b);
        v.push_back(L"#A* plan");
        swprintf(b, 160, L"start %ls   goal %ls   (%ls)", wide(cellText(start)).c_str(), wide(cellText(goal)).c_str(),
                 wide(map.coords.order()).c_str());
        v.push_back(b);
        if (requestedGoal) {
            swprintf(b, 160, L"goal moved from %ls: too close", wide(cellText(*requestedGoal)).c_str());
            v.push_back(b);
        }
        if (plan.search.found) {
            swprintf(b, 160, L"%d cells, %.2f m, %d legs (%ls)", static_cast<int>(plan.search.path.size()), plan.search.cost,
                     static_cast<int>(plan.corners.size()) - 1, opt.four ? L"4-conn" : L"8-conn");
            v.push_back(b);
            if (opt.smooth) {
                swprintf(b, 160, L"shortened route: %d legs, %.2f m", static_cast<int>(plan.route.size()) - 1, astar::length(plan.route));
                v.push_back(b);
            }
            swprintf(b, 160, L"min clearance %.1f cm (>= %.0f cm)", plan.clearance * 100.0, clearanceNeed * 100.0);
            v.push_back(b);
        } else {
            v.push_back(L"NO PATH");
        }
        swprintf(b, 160, L"expanded: A* %d, Dijkstra %d", plan.search.expanded, plan.dijkstra.expanded);
        v.push_back(b);
        if (showRobot) {
            v.push_back(L"#Run");
            swprintf(b, 160, L"t %.1f s, driven %.2f m", runTime, driven);
            v.push_back(b);
            swprintf(b, 160, L"bumps %d, re-plans %d, position fixes %d", bumps, replans, corrections);
            v.push_back(b);
            swprintf(b, 160, L"max distance from the line %.1f cm", follower.maxCrossTrack() * 100.0);
            v.push_back(b);
            if (finalError >= 0.0) {
                swprintf(b, 160, L"final position error %.1f cm", finalError * 100.0);
                v.push_back(b);
            }
        }
        v.push_back(L"#Waypoints (cell = " + wide(map.coords.order()) + L")");
        const auto& pts = showRobot ? follower.points() : plan.route;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Cell c = map.cellAt(pts[i]);
            swprintf(b, 160, L"%ls%2d  %ls   %.2f, %.2f m", showRobot && static_cast<int>(i) == follower.leg() + 1 ? L">" : L" ",
                     static_cast<int>(i), wide(cellText(c)).c_str(), pts[i].x, pts[i].y);
            v.push_back(b);
        }
        return v;
    };

    const auto draw = [&](const std::wstring& status) {
        view::PlanScene s;
        s.cost = &cm;
        s.expanded = &plan.search.order;
        s.cells = &plan.search.path;
        s.route = showRobot ? &follower.points() : &plan.route;
        s.routeIsShortcut = opt.smooth;
        s.trail = showRobot ? &trail : nullptr;
        s.marked = &marked;
        s.start = start;
        s.goal = goal;
        s.hasRequestedGoal = requestedGoal.has_value();
        if (requestedGoal) s.requestedGoal = *requestedGoal;
        s.hasRobot = showRobot;
        s.pose = snap.pose;
        s.robotRadius = opt.radius;
        s.title = L"A* path planning  -  " + wide(mapName) + L"  (robot radius " +
                  std::to_wstring(static_cast<int>(std::lround(opt.radius * 100.0))) + L" cm)";
        s.status = status;
        s.info = infoLines();
        view::render(canvas, s);
        window.present(canvas);
    };

    const auto statusText = [&]() {
        std::wostringstream w;
        w << std::fixed << std::setprecision(2) << phaseText;
        if (showRobot) {
            const Cell c = map.cellAt(snap.pose.point());
            w << L"   |   leg " << follower.leg() + 1 << L"/" << follower.legs() << L"   |   (" << snap.pose.x << L", "
              << snap.pose.y << L") m, cell (" << c.x << L"," << c.y << L"), " << std::setprecision(0)
              << snap.pose.heading / Deg << L" deg   |   " << std::setprecision(1) << L"t " << runTime << L" s";
        } else {
            w << L"   |   Q / close the window = exit";
        }
        return w.str();
    };

    // --- Kayit ----------------------------------------------------------------
    std::error_code fsError;
    const std::filesystem::path dir = outputDir(opt);
    std::filesystem::create_directories(dir, fsError);
    const std::string stem = mapName + "_" + timestamp();

    const auto saveText = [&](const std::filesystem::path& path, bool withRun, const char* result) {
        std::ofstream out(path);
        if (!out) return false;
        out << std::fixed << std::setprecision(2)
            << "A* path plan (10_AStarPathPlanning)\n"
            << "map        : " << opt.mapPath << "\n"
            << "grid       : " << map.cols() << " x " << map.rows() << " cells of " << map.cellSize() << " m\n"
            << "robot      : radius " << opt.radius << " m, margin " << opt.margin << " m, "
            << (opt.four ? "4" : "8") << "-connected" << (opt.smooth ? ", shortened route" : "") << "\n"
            << "start      : " << cellText(start) << "   goal: " << cellText(goal);
        if (requestedGoal) out << "   (requested " << cellText(*requestedGoal) << ", too close to an obstacle)";
        out << "\n";
        if (plan.search.found)
            out << "path       : " << plan.search.path.size() << " cells, " << plan.search.cost << " m, min clearance "
                << plan.clearance * 100.0 << " cm\n";
        else
            out << "path       : NONE\n";
        out << "expanded   : A* " << plan.search.expanded << ", Dijkstra " << plan.dijkstra.expanded << "\n";
        if (withRun)
            out << "run        : " << result << ", " << runTime << " s, driven " << driven << " m, bumps " << bumps
                << ", re-plans " << replans << ", final error " << finalError * 100.0 << " cm\n";
        out << "\nlegend: # obstacle   + too close for the robot centre   . free   o path   * start   / goal   X found by the bumper\n";
        writeGrid(out, cm, plan.search.path, start, goal, marked);
        out << "\nwaypoints (" << map.coords.order() << "  ->  x, y in metres; origin = bottom-left corner of the map):\n";
        const auto& pts = withRun ? follower.points() : plan.route;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Cell c = map.cellAt(pts[i]);
            out << std::setw(3) << i << "  " << std::setw(8) << cellText(c) << "  ->  " << pts[i].x << ", " << pts[i].y << "\n";
        }
        out << "\ncell path:\n";
        for (std::size_t i = 0; i < plan.search.path.size(); ++i)
            out << cellText(plan.search.path[i]) << (i + 1 < plan.search.path.size() ? (i % 10 == 9 ? "\n" : " ") : "\n");
        return static_cast<bool>(out);
    };

    draw(statusText());
    const auto planPng = dir / (stem + "_plan.png"), planTxt = dir / (stem + "_plan.txt");
    const bool savedPlan = canvas.savePng(planPng.wstring()) && saveText(planTxt, false, "");
    std::cout << (savedPlan ? "Saved    : " : "!! could not save ") << planPng.string() << "\n"
              << "           " << planTxt.string() << "\n";

    SetConsoleCtrlHandler(consoleHandler, TRUE);
    const auto keepWindowOpen = [&](const char* why) {
        if (!window.isOpen() || g_quit) return;
        std::cout << "\n" << why << " The window stays open - close it (or press Q / Enter) to exit.\n";
        while (window.isOpen() && !g_quit) {
            window.pump();
            const int key = pollKey(window);
            if (key == 'Q' || key == VK_RETURN || key == VK_ESCAPE) break;
            Sleep(30);
        }
    };

    if (!plan.search.found) { keepWindowOpen("No path."); return 3; }
    if (opt.planOnly()) {
        std::cout << "\nPlan only (no robot). Run with --sim, or with the Kobuki COM port, to drive it.\n";
        keepWindowOpen("");
        return 0;
    }

    // --- 4. Surus ----------------------------------------------------------------
    std::unique_ptr<nav::Robot> robot;
    const nav::Pose startPose{ startPoint.x, startPoint.y, headingDeg * Deg };
    if (opt.sim) {
        nav::SimRobot::Options so;
        so.radius = opt.radius;
        so.walledBorder = walled;
        so.speed = (opt.fast || !window.isOpen()) ? 0 : 4;
        so.hidden = opt.hidden;
        for (const Cell& c : so.hidden)
            if (!map.inside(c)) { std::cerr << "ERROR: --block " << cellText(c) << " is outside the map.\n"; return 1; }
        robot = std::make_unique<nav::SimRobot>(map, startPose, so);
    } else {
        robot = std::make_unique<nav::KobukiRobot>(opt.port, startPose);
    }

    if (!robot->open()) return 1;
    if (opt.hardware()) {
        std::cout << "\nPlace the robot with its CENTRE on cell " << cellText(start) << ", facing " << headingName(headingDeg)
                  << ".\nEvery cell is " << map.cellSize() * 100.0 << " cm; the map's top-left corner is the corner of cell "
                  << cellText({ 0, 0 }) << ".\n"
                  << (opt.go ? "--go: starting in 3 s (Q / close the window to cancel).\n"
                             : "Press ENTER to start driving, Q to quit.\n")
                  << std::flush;
        // --go: Enter yerine 3 s geri sayim (program etkilesimsiz baslatildiginda).
        const auto waitStart = std::chrono::steady_clock::now();
        while (true) {
            window.pump();
            const int key = pollKey(window);
            if (key == VK_RETURN) break;
            if (opt.go && std::chrono::steady_clock::now() - waitStart >= std::chrono::seconds(3)) break;
            if (key == 'Q' || key == VK_ESCAPE || g_quit || window.closedByUser()) {
                std::cout << "Not started.\n";
                robot->shutdown();
                return 0;
            }
            robot->tick();
        }
    }
    robot->resetPose();   // the robot may have been moved into place: this spot = the start cell
    snap = robot->read();

    follower.setPath(plan.route);
    showRobot = true;
    std::cout << "\nDriving " << plan.route.size() - 1 << " legs on " << robot->name()
              << ". Space = pause, Q = stop. (Ctrl+C also stops.)\n";

    using Clock = std::chrono::steady_clock;
    const auto seconds = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b - a).count(); };
    auto lastHud = Clock::now() - std::chrono::seconds(10), lastView = lastHud;
    const double timeLimit = 120.0 + 20.0 * astar::length(plan.route) / 0.15;

    enum class Mode { Drive, BackOff, Done, Failed } mode = Mode::Drive;
    const char* result = nullptr;
    nav::Bumper lastBumper = nav::Bumper::None;
    nav::Pose backStart{};
    double backTime = 0.0, dt = 0.0, cliffTime = 0.0;
    bool paused = false;
    int lastLeg = -1;
    trail.push_back(snap.pose.point());
    Point lastPoint = snap.pose.point();

    while (true) {
        snap = robot->read();
        driven += astar::distance(lastPoint, snap.pose.point());
        lastPoint = snap.pose.point();
        if (astar::distance(trail.back(), lastPoint) >= 0.01) trail.push_back(lastPoint);

        // Guvenlik.
        // Ucurum: hemen dur. Sinyal CliffAbortS'den uzun surerse bitir; kisa
        // bir sinyalde (koyu / parlak zemin yanlis alarmi) bekleyip devam et.
        if (snap.fault) {
            robot->stop();
            if (cliffTime == 0.0) std::cout << "\n  " << snap.fault << " - stopped, waiting " << CliffAbortS << " s...\n";
            cliffTime += dt;
            if (cliffTime >= CliffAbortS) { result = snap.fault; mode = Mode::Failed; break; }
            window.pump();
            dt = robot->tick();
            continue;
        }
        if (cliffTime > 0.0) {
            std::cout << "  Cliff signal gone after " << std::setprecision(1) << cliffTime << " s - continuing.\n";
            cliffTime = 0.0;
        }
        if (g_quit) { result = "stopped by the user (Ctrl+C)"; mode = Mode::Failed; break; }
        window.pump();
        const int key = pollKey(window);
        if (key == 'Q' || key == VK_ESCAPE) { result = "stopped by the user (Q)"; mode = Mode::Failed; break; }
        if (window.closedByUser()) { result = "window closed"; mode = Mode::Failed; break; }
        if (key == ' ') {
            paused = !paused;
            std::cout << (paused ? "\n  Paused - Space to continue.\n" : "\n  Continuing.\n");
        }
        if (runTime > timeLimit) { result = "time limit reached"; mode = Mode::Failed; break; }

        // Tampon: yeni bir carpma mi?
        const bool bumped = snap.bumper != nav::Bumper::None && lastBumper == nav::Bumper::None;
        lastBumper = snap.bumper;
        if (bumped && mode == Mode::Drive) {
            ++bumps;
            robot->stop();
            std::cout << "\n  Bumper hit at (" << std::fixed << std::setprecision(2) << snap.pose.x << ", " << snap.pose.y << ")";
            const double body = opt.sim ? opt.radius : KobukiBodyM;
            Point u;
            double dist = 0.0;
            const auto hit = bumpCell(cm, snap.pose, snap.bumper, opt.radius);
            if (knownContact(cm, snap.pose, snap.bumper, body, u, dist)) {
                // Bilinen engel: yeni bir sey yok, konum tahmini kaymis - duzelt.
                const double shift = dist - body;
                robot->shiftPose(u.x * shift, u.y * shift);
                snap = robot->read();
                ++corrections;
                std::cout << " - a known obstacle: position corrected by " << std::setprecision(1) << shift * 100.0
                          << " cm to (" << std::setprecision(2) << snap.pose.x << ", " << snap.pose.y << ")";
            } else if (hit && !cm.obstacle(*hit)) {
                cm.addObstacle(*hit);
                marked.push_back(*hit);
                std::cout << " - cell " << cellText(*hit) << " is blocked, added to the map";
            }
            std::cout << ". Backing off " << BackOffM * 100.0 << " cm.\n";
            mode = Mode::BackOff;
            backStart = snap.pose;
            backTime = 0.0;
        }

        nav::Command cmd;
        if (mode == Mode::Drive) {
            phaseText = follower.phase() == nav::PathFollower::Phase::Turn ? "TURN" : "DRIVE";
            cmd = follower.step(snap.pose);
            follower.trackError(snap.pose);
            if (follower.leg() != lastLeg && !follower.done()) {
                lastLeg = follower.leg();
                const Point to = follower.points()[static_cast<std::size_t>(lastLeg + 1)];
                const Cell c = map.cellAt(to);
                std::cout << "\n  Leg " << lastLeg + 1 << "/" << follower.legs() << ": to cell " << cellText(c) << " ("
                          << std::fixed << std::setprecision(2) << to.x << ", " << to.y << ") m\n";
            }
            if (follower.done()) { mode = Mode::Done; break; }
        } else if (mode == Mode::BackOff) {
            phaseText = "BACK_OFF";
            backTime += dt;
            if (astar::distance(backStart.point(), snap.pose.point()) < BackOffM && backTime < 3.0) {
                cmd.v = -BackOffSpeed;
            } else {
                // Bulundugu yerden yeniden planla.
                robot->stop();
                const Cell from = map.cellAt(snap.pose.point());
                if (++replans > MaxReplans) { result = "too many bumps - giving up"; mode = Mode::Failed; break; }
                if (!cm.traversable(goal)) { result = "the goal is blocked"; mode = Mode::Failed; break; }
                plan = makePlan(cm, from, goal, opt);
                if (!plan.search.found) { result = "no path left after the new obstacle"; mode = Mode::Failed; break; }
                // Start the new route where the robot really is: straight on to the
                // second point if that line is safe, else first to the cell centre.
                std::vector<Point> pts = plan.route;
                if (pts.size() >= 2 && (!cm.traversable(from) || cm.segmentClear(snap.pose.point(), pts[1], cm.radius())))
                    pts.front() = snap.pose.point();
                else
                    pts.insert(pts.begin(), snap.pose.point());
                follower.setPath(pts);
                lastLeg = -1;
                std::cout << "  Re-planned from " << cellText(from) << ": " << plan.search.path.size() << " cells, "
                          << std::setprecision(2) << plan.search.cost << " m (A* expanded " << plan.search.expanded << ").\n";
                mode = Mode::Drive;
            }
        }
        if (paused) cmd = {};
        robot->drive(cmd.v, cmd.w);

        const auto now = Clock::now();
        if (seconds(lastHud, now) >= HudPeriodS) {
            lastHud = now;
            const Cell c = map.cellAt(snap.pose.point());
            std::cout << "\r  " << std::left << std::setw(9) << (paused ? "PAUSED" : phaseText) << std::right << std::fixed
                      << std::setprecision(2) << " (" << snap.pose.x << ", " << snap.pose.y << ") m  cell "
                      << std::setw(7) << cellText(c) << std::setprecision(0) << std::setw(5) << snap.pose.heading / Deg
                      << " deg | t " << std::setprecision(1) << runTime << " s   " << std::flush;
        }
        if (window.isOpen() && seconds(lastView, now) >= ViewPeriodS) {
            lastView = now;
            draw(statusText());
        }

        dt = robot->tick();
        if (!paused) runTime += dt;
    }
    robot->stop();

    // --- 5. Sonuc ------------------------------------------------------------------
    finalError = astar::distance(snap.pose.point(), map.centre(goal));
    if (mode == Mode::Done) { result = "GOAL REACHED"; phaseText = "GOAL REACHED"; }
    else phaseText = "STOPPED";
    draw(statusText());

    const auto runPng = dir / (stem + "_run.png"), runTxt = dir / (stem + "_run.txt");
    const bool savedRun = canvas.savePng(runPng.wstring()) && saveText(runTxt, true, result);

    const Cell endCell = map.cellAt(snap.pose.point());
    std::cout << "\n\n--- Summary ---\n"
              << "Result        : " << result << "\n"
              << std::fixed << std::setprecision(2)
              << "Final pose    : (" << snap.pose.x << ", " << snap.pose.y << ") m, cell " << cellText(endCell)
              << ", heading " << std::setprecision(0) << snap.pose.heading / Deg << " deg\n"
              << std::setprecision(1)
              << "Goal error    : " << finalError * 100.0 << " cm from the centre of " << cellText(goal) << "\n"
              << "Run time      : " << runTime << " s\n"
              << "Driven        : " << driven << " m (planned " << std::setprecision(2) << astar::length(plan.route) << " m)\n"
              << std::setprecision(1)
              << "Off the line  : max " << follower.maxCrossTrack() * 100.0 << " cm\n"
              << "Bumper hits   : " << bumps << ", re-plans " << replans << ", position fixes " << corrections << "\n";
    if (!marked.empty()) {
        std::cout << "Found cells   :";
        for (const Cell& c : marked) std::cout << " " << cellText(c);
        std::cout << "\n";
    }
    std::cout << "\nLegend: # obstacle   + too close   o last A* path   * start   / goal   X found by the bumper   R robot\n";
    writeGrid(std::cout, cm, plan.search.path, start, goal, marked, endCell);
    std::cout << "\n" << (savedRun ? "Saved         : " : "!! could not save ") << runPng.string() << "\n"
              << "                " << runTxt.string() << "\n";

    robot->shutdown();
    keepWindowOpen(mode == Mode::Done ? "Goal reached." : "Stopped.");
    std::cout << "\nBye.\n";
    return mode == Mode::Done ? 0 : 2;
}
