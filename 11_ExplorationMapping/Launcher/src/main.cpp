// ExploreLauncher - 11 Exploration Mapping'in Windows uygulamasi.
//
// "Calistir" sekmesi, dort adim:
//   1. Calisma modu    : Simulasyon / Gercek robot
//   2. Baglanti        : (gercek robot) Kobuki ve Arduino COM portu + baglanti
//                        kontrolu: port dinlenir, Kobuki paketleri / Arduino JSON
//                        satirlari taninir (Probe.h); "Portlari tara" ikisini bulur.
//                        Simulasyonda: sensor gurultusu / hizli.
//   3. Sensor kalibrasyonu : (gercek robot) sagda bir panel acar: HC-SR04'un Trig /
//                        Echo pinleri (sensor.ino v2'ye yazilir, EEPROM), anlik okuma
//                        (yanki suresi us, ham cm, kalibreli cm) ve 40 cm blokla
//                        kalibrasyon -> sensor_calibration.txt (SensorLink.h,
//                        common/RangeCalibration.h). Dort yontem de bu olcegi kullanir.
//   4. Algoritma + harita : 11_01 Frontier / 11_02 BFS / 11_03 Random / 11_04 TSP,
//                        obstacleMapReferences/mapN
//   Baslat -> secilen Explore_*.exe --ipc ile penceresiz calisir; durumu
//   (LiveRun.h) bu pencerede canli cizilir: P(Dolu) % ile hucreler, robot,
//   sonar, yol, frontier, yontemin katmani; Kesif / Dogruluk / Basari %'leri.
//   Baslat (gercek robot) once "Hazirlik kontrolu" panelini acar: iki port
//   yeniden dinlenir; batarya, tampon, sensor akisi, kalibrasyon tek listede.
//   2. adimdaki "Elle surus" paneli (Teleop.h): ok tuslari / WASD ile surus,
//   sabit hareketler, Kobuki sensorleri, tampon testi.
//   3. adim panelinde "Sketch'i yukle" (ArduinoCli.h): arduino-cli ile derle + yukle.
// "Karsilastirma" sekmesi: benchmark (simulasyon, pencere yok) ve 3 olcutle
// siralama (hiz, basari orani, basarili engel haritasi), yontem x harita.
// "Gecmis" sekmesi (Tabs.h, Replay.h): results.csv'deki kosular; launcher'dan
// baslatilanlar <harita>.replay ile kare kare yeniden izlenir.
// "Haritalar" sekmesi (Tabs.h, MapFile.h): engel haritasi duzenleyici (map5..map8).
//
// Everything is drawn with GDI+ (Theme.h) - no standard controls - so the
// whole window follows one modern theme.
//
// Command line: --report (write results/comparison_<date>.md and exit).
// Test hooks: --screenshot <png> [--delay s] [--tab 1] [--real] [--auto-start] [--stop-after s]
//             [--cal] [--arduino COMx] [--auto-cal s] [--cal-file txt] [--pins t,e]
//             [--drive] [--kobuki COMx] [--preflight] [--tab 2|3] [--verify s].
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "Theme.h"   // windows.h, gdiplus.h, GdiCanvas.h

#include <shellapi.h>
#include <windowsx.h>

#include "AStarPlanner.h"
#include "ArduinoCli.h"
#include "ComPorts.h"
#include "LiveRun.h"
#include "MapFile.h"
#include "Probe.h"
#include "RangeCalibration.h"
#include "Replay.h"
#include "Results.h"
#include "SensorLink.h"
#include "Teleop.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#pragma comment(linker, "\"/manifestdependency:type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

namespace fs = std::filesystem;
using namespace ui;

namespace {

// ------------------------------------------------------------------ data --
struct Method {
    std::string id;
    std::wstring exe, name, tag, desc;
};
const std::vector<Method> Methods = {
    { "frontier", L"Explore_Frontier.exe", L"11_01 Frontier", L"en yakın frontier",
      L"Bilinmeyenin sınırındaki hücrelerden A* yolu en kısa olana gider, ona dönüp karşıdan ölçer." },
    { "bfs", L"Explore_BFS.exe", L"11_02 BFS", L"başlangıçtan katman katman",
      L"Hücreleri başlangıç hücresinden genişlik öncelikli (FIFO) sırayla ölçer; yollar BFS en kısa yoludur." },
    { "random", L"Explore_Random.exe", L"11_03 Random", L"rastgele hedef + 360°",
      L"Ulaşılabilir rastgele bir hücreye gidip 360° tarar; yeni bilgi gelmeyince durur. Temel karşılaştırma." },
    { "tsp", L"Explore_TSP.exe", L"11_04 TSP", L"en kısa frontier turu",
      L"Tüm frontier hücrelerinden geçen en kısa turu (Held-Karp / 2-opt) hesaplar, ilk durağa gider, yeniden planlar." },
};

struct MapRef {
    std::wstring name, path, desc;
    astar::GridMap grid;
    int inner = 0;
};

enum class Act {
    None, TabRun, ModeSim, Noise, Fast, KobukiPort, ArduinoPort, Test, Scan, Method, Map, Start, Stop,
    OpenPicture, BenchAlgo, BenchMap, RepsMinus, RepsPlus, ThrMinus, ThrPlus, BenchNoise, BenchRun, BenchCancel, Report,
    OpenResults, Archive, Filter,
    CalOpen, CalClose, TrigPin, EchoPin, WritePins, DefaultPins, CalStart, CalCancel, LinkRetry,
    // pre-flight check
    PreGo, PreRetry, PreCancel,
    // manual drive
    DriveOpen, DriveBtn, DriveStop, DriveSpeed, DriveMove, DriveTurn, DriveReset, BumperReset, DriveRetry,
    // sketch upload, calibration check
    Upload, UploadCancel, UploadClose, Board, VerifyRef, Verify,
    // history
    HistFilter, HistRow, HistPlay, HistSpeed, HistSeek, HistPicture, HistFolder,
    // maps
    MapPage, EditSelect, EditCell, EditNew, EditSave, EditRevert, EditDelete, EditUse,
};

enum class Panel { None, Cal, Drive, Preflight };   // what the right side of the run tab shows (real robot)

struct Hit {
    double x, y, w, h;
    Act act;
    int arg;
};

constexpr UINT WmFrame = WM_APP + 1, WmRunEnd = WM_APP + 2, WmProbe = WM_APP + 3, WmBenchStep = WM_APP + 4,
               WmBenchDone = WM_APP + 5, WmScanDone = WM_APP + 6, WmSensor = WM_APP + 7, WmUpLine = WM_APP + 8,
               WmUpDone = WM_APP + 9;
constexpr UINT_PTR TimerKill = 1, TimerShot = 2, TimerStart = 3, TimerStop = 4, TimerCal = 5, TimerDrive = 6, TimerReplay = 7,
                   TimerVerify = 8;
constexpr int VerifySamples = 20;      // "Kontrol et": readings with an echo
constexpr int MaxMaps = 8;
constexpr int KeyFwd = 1, KeyBack = 2, KeyLeft = 4, KeyRight = 8;

// Step 3: 40 cm calibration.
constexpr int CalSamples = 50;         // readings with an echo (~5 s at 10 Hz)
constexpr double CalMaxSeconds = 20.0;
constexpr int CalMaxNoEcho = 25;
constexpr int DefaultTrig = 11, DefaultEcho = 12;
constexpr double BW = 1360, BH = 820;   // window size in base units

struct PortStatus {
    std::wstring port;
    probe::Result r;
};

struct Job {
    int method = 0, map = 0;
    unsigned seed = 1;
    std::wstring args, log, label;
};

struct App {
    HINSTANCE inst = nullptr;
    HWND hwnd = nullptr;
    double scale = 1.0;
    std::unique_ptr<view::Canvas> canvas;
    fs::path dir, bin;
    std::vector<MapRef> maps;
    std::vector<ports::Port> ports;
    std::vector<Hit> hits;
    Act hoverAct = Act::None;
    int hoverArg = -1;
    bool tracking = false;

    // selection
    int tab = 0;
    bool real = false, noise = true, fast = false;
    int method = 0, map = 0;
    PortStatus kobuki, arduino;
    int probing = 0;
    bool scanning = false;
    std::wstring scanText;
    int scanned = 0;   // ports listened to in the last scan (0 = no scan yet)

    // step 3: sensor pins, live readings, 40 cm calibration
    sensor::Link link;
    sensor::Snapshot sens;
    Panel panel = Panel::None;
    bool calAutoShown = false;
    double clickX = 0.0, clickY = 0.0;   // last click / drag position, base units
    Act dragAct = Act::None;
    int trig = DefaultTrig, echo = DefaultEcho;   // pins as chosen on the screen
    bool pinsDirty = false, pinWait = false, pinOk = false;
    int pinReplies = 0, pinErrors = 0;
    ULONGLONG pinSent = 0;
    std::wstring pinMsg;
    rangecal::Calibration cal;
    fs::path calFile;
    struct CalRun {
        bool active = false;
        std::uint64_t lastSeq = 0;
        std::vector<double> raw, us;   // raw cm, echo time (-1 if not sent)
        int noEcho = 0;
        ULONGLONG t0 = 0;
        bool verify = false;           // "Kontrol et": measure only, do not save
    } calRun;
    std::wstring calError;
    int verifyRef = 40;                // cm
    struct Verify {
        bool have = false, ok = false;
        double ref = 0.0, mean = 0.0, err = 0.0;
    } verify;

    // sketch upload (arduino-cli)
    struct Up {
        bool running = false, done = false, ok = false, show = false, setup = false;
        std::vector<std::wstring> log;
        std::thread th;
        std::atomic<bool> cancel{ false };
    } up;
    int board = 0;

    // pre-flight check
    bool preChecking = false;

    // manual drive
    teleop::Robot bot;
    teleop::Status drive;
    int driveKeys = 0, driveBtn = 0, driveLast = 0, driveSpeed = 0;

    // history tab
    int histFilter = 0, histRow = -1, histScroll = 0;
    replay::Recording rec;
    live::State recView;
    std::size_t recIndex = 0;
    double recT = 0.0, histBarX = 0.0, histBarW = 1.0;
    bool recPlaying = false;
    int recSpeed = 2;
    ULONGLONG recTick = 0;

    // maps tab (editor)
    struct Draft {
        bool active = false, isNew = false, dirty = false, editable = false;
        std::wstring name;
        astar::GridMap grid;
    } draft;
    int paintValue = -1, mapPage = 0;
    std::wstring mapMsg;

    // run
    live::Run run;
    live::State live;
    bool haveLive = false, stopping = false;
    int runMethod = 0, runMap = 0;
    bool runReal = false;

    // benchmark / results
    std::vector<results::Row> rows;
    std::vector<Job> jobs;
    std::thread bench;
    std::atomic<bool> benchCancel{ false };
    bool benchRunning = false;
    std::size_t benchIndex = 0;
    std::vector<bool> benchAlgo, benchMap;
    int reps = 3, threshold = 95, filter = 0;
    bool benchNoise = true;
    std::wstring lastReport;

    // test hooks
    std::wstring screenshot;
    double shotDelay = 1.5, stopAfter = 0.0, autoCal = 0.0;
    bool autoStart = false, reportOnly = false;
    int hookTrig = 0, hookEcho = 0;   // --pins t,e: write these pins once the board answers
    double hookVerify = 0.0;
} g;

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}
std::string utf8(const std::wstring& w) {
    if (w.empty()) return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
    std::string s(static_cast<std::size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
    return s;
}
std::wstring fmt(const wchar_t* f, double v) {
    wchar_t b[512];
    swprintf(b, 512, f, v);
    return b;
}
std::wstring q(const std::wstring& s) { return L"\"" + s + L"\""; }
fs::path resultsCsv() { return g.dir / "results" / "results.csv"; }
bool exeExists(int m) { return fs::exists(g.bin / Methods[static_cast<std::size_t>(m)].exe); }
std::wstring methodName(const std::string& id) {
    for (const Method& m : Methods)
        if (m.id == id) return m.name;
    return widen(id);
}
void repaint() { InvalidateRect(g.hwnd, nullptr, FALSE); }
bool busy() { return g.run.running() || g.benchRunning; }

void hit(double x, double y, double w, double h, Act a, int arg = 0) { g.hits.push_back({ x, y, w, h, a, arg }); }
bool hovered(Act a, int arg = 0) { return g.hoverAct == a && g.hoverArg == arg; }

// ------------------------------------------------------------------ maps --
// Observable cell of a reference map: free, or occupied with a free 4-neighbour
// (the same definition as common/Metrics.h).
bool observable(const astar::GridMap& m, int x, int y) {
    if (!m.occupied({ x, y })) return m.inside({ x, y });
    const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    for (int k = 0; k < 4; ++k)
        if (m.inside({ x + dx[k], y + dy[k] }) && !m.occupied({ x + dx[k], y + dy[k] })) return true;
    return false;
}

void loadMaps() {
    g.maps.clear();
    std::error_code ec;
    std::vector<fs::path> files;
    for (const auto& d : fs::directory_iterator(g.dir / "obstacleMapReferences", ec)) {
        if (!d.is_directory()) continue;
        for (const auto& f : fs::directory_iterator(d.path(), ec))
            if (f.path().extension() == ".txt") files.push_back(f.path());
    }
    std::sort(files.begin(), files.end());
    for (const fs::path& f : files) {
        MapRef m;
        std::ifstream in(f);
        std::string error;
        if (!in || !astar::parseMap(in, m.grid, error)) continue;
        m.path = f.wstring();
        m.name = m.grid.name.empty() ? f.stem().wstring() : widen(m.grid.name);
        std::wstring list;
        for (int y = 1; y + 1 < m.grid.rows(); ++y)
            for (int x = 1; x + 1 < m.grid.cols(); ++x)
                if (m.grid.occupied({ x, y })) { ++m.inner; list += L" " + widen(m.grid.coords.text({ x, y })); }
        m.desc = m.inner ? std::to_wstring(m.inner) + L" iç engel:" + list : L"iç engel yok";
        g.maps.push_back(m);
    }
}

// ---------------------------------------------------------------- settings --
fs::path iniPath() { return g.dir / "results" / "launcher.ini"; }
void setIni(const wchar_t* k, const std::wstring& v) { WritePrivateProfileStringW(L"launcher", k, v.c_str(), iniPath().c_str()); }
std::wstring getIni(const wchar_t* k, const std::wstring& def) {
    wchar_t b[512];
    GetPrivateProfileStringW(L"launcher", k, def.c_str(), b, 512, iniPath().c_str());
    return b;
}
int getIniInt(const wchar_t* k, int def) { return _wtoi(getIni(k, std::to_wstring(def)).c_str()); }

void saveSettings() {
    if (!g.screenshot.empty()) return;   // test runs leave the user's settings alone
    std::error_code ec;
    fs::create_directories(iniPath().parent_path(), ec);
    setIni(L"real", g.real ? L"1" : L"0");
    setIni(L"noise", g.noise ? L"1" : L"0");
    setIni(L"fast", g.fast ? L"1" : L"0");
    setIni(L"method", std::to_wstring(g.method));
    setIni(L"map", std::to_wstring(g.map));
    setIni(L"kobuki", g.kobuki.port);
    setIni(L"arduino", g.arduino.port);
    setIni(L"trig", std::to_wstring(g.trig));
    setIni(L"echo", std::to_wstring(g.echo));
    setIni(L"board", std::to_wstring(g.board));
    setIni(L"driveSpeed", std::to_wstring(g.driveSpeed));
    setIni(L"reps", std::to_wstring(g.reps));
    setIni(L"threshold", std::to_wstring(g.threshold));
    setIni(L"benchNoise", g.benchNoise ? L"1" : L"0");
    std::wstring a, m;
    for (bool b : g.benchAlgo) a += b ? L"1" : L"0";
    for (bool b : g.benchMap) m += b ? L"1" : L"0";
    setIni(L"benchAlgos", a);
    setIni(L"benchMaps", m);
}

void loadSettings() {
    g.real = getIniInt(L"real", 0) == 1;
    g.noise = getIniInt(L"noise", 1) == 1;
    g.fast = getIniInt(L"fast", 0) == 1;
    g.method = std::clamp(getIniInt(L"method", 0), 0, static_cast<int>(Methods.size()) - 1);
    int defMap = 0;
    for (std::size_t i = 0; i < g.maps.size(); ++i) if (g.maps[i].name == L"map3") defMap = static_cast<int>(i);
    g.map = g.maps.empty() ? 0 : std::clamp(getIniInt(L"map", defMap), 0, static_cast<int>(g.maps.size()) - 1);
    g.kobuki.port = getIni(L"kobuki", L"");
    g.arduino.port = getIni(L"arduino", L"");
    g.trig = std::clamp(getIniInt(L"trig", DefaultTrig), 2, 13);
    g.echo = std::clamp(getIniInt(L"echo", DefaultEcho), 2, 13);
    if (g.trig == g.echo) { g.trig = DefaultTrig; g.echo = DefaultEcho; }
    g.board = std::clamp(getIniInt(L"board", 0), 0, static_cast<int>(arduino::boards().size()) - 1);
    g.driveSpeed = std::clamp(getIniInt(L"driveSpeed", 0), 0, 1);
    g.reps = std::clamp(getIniInt(L"reps", 3), 1, 20);
    g.threshold = std::clamp(getIniInt(L"threshold", 95), 50, 100);
    g.benchNoise = getIniInt(L"benchNoise", 1) == 1;
    const std::wstring a = getIni(L"benchAlgos", L""), m = getIni(L"benchMaps", L"");
    g.benchAlgo.assign(Methods.size(), true);
    g.benchMap.assign(g.maps.size(), true);
    for (std::size_t i = 0; i < g.benchAlgo.size(); ++i) g.benchAlgo[i] = (i < a.size() ? a[i] == L'1' : true) && exeExists(static_cast<int>(i));
    for (std::size_t i = 0; i < g.benchMap.size(); ++i) g.benchMap[i] = i < m.size() ? m[i] == L'1' : true;
}

// ----------------------------------------------------------------- ports --
// Probe one port in the background; the result comes back as WmProbe.
struct ProbeMsg {
    int role;            // 0 Kobuki, 1 Arduino
    std::wstring port;
    probe::Result r;
};

void probeRole(int role) {
    PortStatus& ps = role == 0 ? g.kobuki : g.arduino;
    if (ps.port.empty() || g.run.running()) return;
    ps.r = probe::Result{};
    ps.r.kind = probe::Kind::Testing;
    ps.r.detail = L"Port dinleniyor...";
    g.link.close();   // the calibration panel may hold the Arduino port
    g.bot.close();    // ... and the drive panel the Kobuki port
    ++g.probing;
    const std::wstring port = ps.port;
    const HWND hwnd = g.hwnd;
    std::thread([role, port, hwnd] {
        auto* m = new ProbeMsg{ role, port, probe::listen(port) };
        if (!PostMessageW(hwnd, WmProbe, 0, reinterpret_cast<LPARAM>(m))) delete m;
    }).detach();
    repaint();
}

// "Portları tara": listen to every port (except Bluetooth, whose open can hang)
// at the same time and pick the Kobuki and the Arduino by what they send.
struct ScanMsg {
    std::vector<std::pair<std::wstring, probe::Result>> found;
};
void scanPorts() {
    if (g.scanning || g.run.running()) return;
    g.ports = ports::list();
    std::vector<std::wstring> list;
    for (const auto& p : g.ports)
        if (ports::upper(p.hardwareId).find(L"BTHENUM") == std::wstring::npos) list.push_back(p.name);
    g.link.close();
    g.bot.close();
    g.scanning = true;
    g.scanText = L"Taranıyor: " + std::to_wstring(list.size()) + L" port";
    const HWND hwnd = g.hwnd;
    std::thread([list, hwnd] {
        auto* msg = new ScanMsg;
        std::mutex mu;
        std::vector<std::thread> th;
        for (const auto& port : list)
            th.emplace_back([&mu, msg, port] {
                const probe::Result r = probe::listen(port, 3.5);
                std::lock_guard<std::mutex> lock(mu);
                msg->found.push_back({ port, r });
            });
        for (auto& t : th) t.join();
        std::sort(msg->found.begin(), msg->found.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        if (!PostMessageW(hwnd, WmScanDone, 0, reinterpret_cast<LPARAM>(msg))) delete msg;
    }).detach();
    repaint();
}


void onScanDone(ScanMsg* m) {
    g.scanning = false;
    std::wstring kob, ard;
    for (const auto& [port, r] : m->found) {
        if (r.kind == probe::Kind::Kobuki && kob.empty()) { kob = port; g.kobuki = { port, r }; }
        else if ((r.kind == probe::Kind::Arduino || r.kind == probe::Kind::WrongSketch) && ard.empty()) { ard = port; g.arduino = { port, r }; }
    }
    // A role not found keeps its port; if that port was listened to, show what it said.
    for (const auto& [port, r] : m->found) {
        if (kob.empty() && port == g.kobuki.port) g.kobuki.r = r;
        if (ard.empty() && port == g.arduino.port) g.arduino.r = r;
    }
    g.scanned = static_cast<int>(m->found.size());
    g.scanText = L"Tarama: " + std::to_wstring(m->found.size()) + L" port dinlendi";
    delete m;
    saveSettings();
    repaint();
}

// Status of a role for the screen.
struct Badge {
    std::wstring label, detail;
    ARGB fg, bg;
};
Badge badge(int role) {
    const PortStatus& ps = role == 0 ? g.kobuki : g.arduino;
    const probe::Kind want = role == 0 ? probe::Kind::Kobuki : probe::Kind::Arduino;
    const probe::Result& r = ps.r;
    if (ps.port.empty())
        return { L"Port seçin", g.scanned ? L"Taramada bulunamadı - kablo / güç?" : L"Listeden seçin ya da Portları tara", col::Muted,
                 col::Hover };
    if (r.kind == probe::Kind::Untested) return { L"Test edilmedi", L"Bağlantıyı test edin", col::Muted, col::Hover };
    if (r.kind == probe::Kind::Testing) return { L"Test ediliyor", r.detail, col::Accent, col::AccentSoft };
    if (r.kind == want) return { L"✓ Bağlı", r.detail, col::Green, col::GreenSoft };
    if (r.kind == probe::Kind::Kobuki) return { L"✗ Bu port Kobuki", L"Portlar ters seçilmiş olabilir", col::Red, col::RedSoft };
    if (r.kind == probe::Kind::Arduino) return { L"✗ Bu port Arduino", L"Portlar ters seçilmiş olabilir", col::Red, col::RedSoft };
    if (r.kind == probe::Kind::WrongSketch && role == 1) return { L"! Yanlış program", r.detail, col::Amber, col::AmberSoft };
    return { L"✗ Hata", r.detail, col::Red, col::RedSoft };
}
bool connectionOk() {
    return g.kobuki.r.kind == probe::Kind::Kobuki && g.arduino.r.kind == probe::Kind::Arduino && g.kobuki.port != g.arduino.port;
}

void choosePort(int role, double bx, double by) {
    g.ports = ports::list();
    HMENU menu = CreatePopupMenu();
    const std::wstring& current = role == 0 ? g.kobuki.port : g.arduino.port;
    for (std::size_t i = 0; i < g.ports.size(); ++i) {
        const auto& p = g.ports[i];
        std::wstring s = p.name + L"     " + p.friendly;
        if (p.ftdi) s += L"   (FTDI: Kobuki?)";
        if (p.arduino) s += L"   (CH340: Arduino?)";
        AppendMenuW(menu, MF_STRING | (p.name == current ? MF_CHECKED : 0), 1000 + i, s.c_str());
    }
    if (g.ports.empty()) AppendMenuW(menu, MF_STRING | MF_GRAYED, 1, L"COM port bulunamadı");
    POINT pt{ static_cast<LONG>(bx * g.scale), static_cast<LONG>(by * g.scale) };
    ClientToScreen(g.hwnd, &pt);
    const int cmd = TrackPopupMenu(menu, TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN, pt.x, pt.y, 0, g.hwnd, nullptr);
    DestroyMenu(menu);
    if (cmd < 1000) return;
    PortStatus& ps = role == 0 ? g.kobuki : g.arduino;
    ps.port = g.ports[static_cast<std::size_t>(cmd - 1000)].name;
    ps.r = probe::Result{};
    saveSettings();
    probeRole(role);   // test right away
}

// ------------------------------------------------ step 3: sensor + 40 cm --
double rawCm(const sensor::Sample& s) { return s.us >= 0 ? s.us * rangecal::CmPerUs : static_cast<double>(s.cm); }
const sensor::Sample* lastSample() { return g.sens.recent.empty() ? nullptr : &g.sens.recent.back(); }
bool streaming() { return g.sens.state == sensor::State::Streaming; }

// The Arduino port is held open only while the calibration panel is shown
// (the probes and the Explore_*.exe programs need it free).
void syncLink() {
    const bool idle = g.real && g.tab == 0 && !g.run.running() && g.probing == 0 && !g.scanning;
    // Kobuki: only while the drive panel is shown.
    const bool wantBot = idle && g.panel == Panel::Drive && !g.kobuki.port.empty();
    if (wantBot) {
        if (!g.bot.isOpen()) {
            g.bot.open(utf8(g.kobuki.port));
            g.drive = teleop::Status{};
            g.drive.state = teleop::State::Opening;
            SetTimer(g.hwnd, TimerDrive, 50, nullptr);
        }
    } else if (g.bot.isOpen()) {
        KillTimer(g.hwnd, TimerDrive);
        g.bot.close();
        g.drive = teleop::Status{};
        g.driveKeys = g.driveBtn = g.driveLast = 0;
    }
    // Arduino: calibration panel, and the drive panel (to show the front sensor).
    const bool want = idle && (g.panel == Panel::Cal || g.panel == Panel::Drive) && !g.up.running && !g.arduino.port.empty();
    if (want) {
        if (!g.link.isOpen() || g.link.port() != g.arduino.port) {
            g.link.open(g.arduino.port, g.hwnd, WmSensor);
            g.sens = sensor::Snapshot{};
            g.sens.port = g.arduino.port;
            g.sens.state = sensor::State::Opening;
        }
    } else if (g.link.isOpen()) {
        g.link.close();
        g.sens = sensor::Snapshot{};
        g.calRun.active = false;
        g.pinWait = false;
    }
}

void choosePin(bool trig, double bx, double by) {
    HMENU menu = CreatePopupMenu();
    const int cur = trig ? g.trig : g.echo, other = trig ? g.echo : g.trig;
    for (int pin = 2; pin <= 13; ++pin) {
        std::wstring s = L"D" + std::to_wstring(pin);
        if (pin == (trig ? DefaultTrig : DefaultEcho)) s += L"     varsayılan";
        if (pin == other) s += trig ? L"     (Echo)" : L"     (Trig)";
        AppendMenuW(menu, MF_STRING | (pin == cur ? MF_CHECKED : 0) | (pin == other ? MF_GRAYED : 0), 2000 + pin, s.c_str());
    }
    POINT pt{ static_cast<LONG>(bx * g.scale), static_cast<LONG>(by * g.scale) };
    ClientToScreen(g.hwnd, &pt);
    const int cmd = TrackPopupMenu(menu, TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN, pt.x, pt.y, 0, g.hwnd, nullptr);
    DestroyMenu(menu);
    if (cmd < 2000) return;
    (trig ? g.trig : g.echo) = cmd - 2000;
    g.pinsDirty = !g.sens.cfgKnown || g.trig != g.sens.trig || g.echo != g.sens.echo;
    g.pinMsg.clear();
    saveSettings();
}

void writePins() {
    if (!g.sens.cfgKnown || g.pinWait || g.trig == g.echo) return;
    g.pinReplies = g.sens.cfgReplies;
    g.pinErrors = g.sens.pinErrors;
    g.pinSent = GetTickCount64();
    g.pinWait = true;
    g.pinOk = false;
    g.pinMsg = L"Karta yazılıyor...";
    g.link.send("PINS " + std::to_string(g.trig) + " " + std::to_string(g.echo));
}

// Age of the calibration in hours (-1: none / unknown date).
double calAgeHours() {
    if (!g.cal.valid) return -1.0;
    int Y = 0, M = 0, D = 0, h = 0, m = 0;
    if (swscanf_s(widen(g.cal.date).c_str(), L"%d-%d-%d %d:%d", &Y, &M, &D, &h, &m) != 5) return -1.0;
    SYSTEMTIME then{}, now;
    then.wYear = static_cast<WORD>(Y); then.wMonth = static_cast<WORD>(M); then.wDay = static_cast<WORD>(D);
    then.wHour = static_cast<WORD>(h); then.wMinute = static_cast<WORD>(m);
    GetLocalTime(&now);
    FILETIME a, b;
    if (!SystemTimeToFileTime(&then, &a) || !SystemTimeToFileTime(&now, &b)) return -1.0;
    const auto u = [](const FILETIME& f) { return (static_cast<unsigned long long>(f.dwHighDateTime) << 32) | f.dwLowDateTime; };
    return std::max(0.0, (static_cast<double>(u(b)) - static_cast<double>(u(a))) / 1e7 / 3600.0);
}
std::wstring ageText(double hours) {
    if (hours < 0) return L"";
    if (hours < 2.0 / 60.0) return L"az önce";
    if (hours < 1.0) return fmt(L"%.0f dk önce", hours * 60.0);
    if (hours < 24.0) return fmt(L"%.0f saat önce", std::floor(hours));
    return fmt(L"%.0f gün önce", std::floor(hours / 24.0));
}
constexpr double CalStaleHours = 24.0;
bool calStale() { return calAgeHours() > CalStaleHours; }
// The board's pins are not the ones the calibration was made with.
bool calPinsChanged() { return g.cal.valid && g.sens.cfgKnown && (g.sens.trig != g.cal.trig || g.sens.echo != g.cal.echo); }

// "Kontrol et": is the calibration still right? Block at 40 or 80 cm.
void startVerify() {
    if (!streaming() || g.calRun.active || !g.cal.valid) return;
    g.calRun = {};
    g.calRun.active = true;
    g.calRun.verify = true;
    g.calRun.lastSeq = g.sens.seq;
    g.calRun.t0 = GetTickCount64();
    g.verify = {};
}
void finishVerify() {
    auto& r = g.calRun;
    r.active = false;
    std::vector<double> v = r.raw;
    std::sort(v.begin(), v.end());
    const double med = v[v.size() / 2];
    double sum = 0.0;
    int n = 0;
    for (double x : r.raw)
        if (std::abs(x - med) <= 3.0) { sum += x; ++n; }
    g.verify.have = true;
    g.verify.ref = g.verifyRef;
    g.verify.mean = g.cal.cm(sum / n);
    g.verify.err = g.verify.mean - g.verify.ref;
    g.verify.ok = std::abs(g.verify.err) <= std::max(1.0, 0.025 * g.verify.ref);   // 1 cm at 40, 2 cm at 80
}

void startCalibration() {
    if (!streaming() || g.calRun.active) return;
    g.verify = {};
    g.calRun = {};
    g.calRun.active = true;
    g.calRun.lastSeq = g.sens.seq;   // from the next reading on
    g.calRun.t0 = GetTickCount64();
    g.calError.clear();
}

std::wstring nowText() {
    SYSTEMTIME t;
    GetLocalTime(&t);
    wchar_t b[32];
    swprintf(b, 32, L"%04d-%02d-%02d %02d:%02d", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute);
    return b;
}

// 50 readings of the block at 40 cm: drop readings more than 3 cm from the
// median (a ghost echo off the floor / another object), mean of the rest ->
// scale = 40 / mean. Saved to sensor_calibration.txt.
void finishCalibration() {
    auto& r = g.calRun;
    r.active = false;
    std::vector<double> v = r.raw;
    std::sort(v.begin(), v.end());
    const double med = v[v.size() / 2];
    if (std::abs(med - rangecal::ReferenceCm) > 15.0) {
        g.calError = fmt(L"Blok 40 cm'de görünmüyor: okumaların ortası %.1f cm. Bloğun yerini ve sensörün yönünü kontrol edin.", med);
        return;
    }
    double sum = 0.0, sumUs = 0.0;
    int nUs = 0;
    std::vector<double> kept;
    for (std::size_t i = 0; i < r.raw.size(); ++i) {
        if (std::abs(r.raw[i] - med) > 3.0) continue;
        kept.push_back(r.raw[i]);
        sum += r.raw[i];
        if (r.us[i] >= 0.0) { sumUs += r.us[i]; ++nUs; }
    }
    if (kept.size() * 10 < r.raw.size() * 6) {
        g.calError = L"Okumalar çok dağınık (yarıdan fazlası ortadan 3 cm uzak). Blok sabit ve sensöre dik mi?";
        return;
    }
    const double mean = sum / static_cast<double>(kept.size());
    double var = 0.0;
    for (double x : kept) var += (x - mean) * (x - mean);
    const double sd = kept.size() > 1 ? std::sqrt(var / static_cast<double>(kept.size() - 1)) : 0.0;
    rangecal::Calibration c;
    c.scale = rangecal::ReferenceCm / mean;
    if (!(c.scale >= rangecal::MinScale && c.scale <= rangecal::MaxScale)) {
        g.calError = fmt(L"Ölçek × %.3f beklenen aralığın (0.85 - 1.15) dışında: ölçüm şüpheli, tekrar deneyin.", c.scale);
        return;
    }
    c.valid = true;
    c.rawMeanCm = mean;
    c.meanUs = nUs ? sumUs / nUs : -1.0;
    c.sigmaCm = sd * c.scale;
    c.samples = static_cast<int>(kept.size());
    c.rejected = static_cast<int>(r.raw.size() - kept.size());
    c.trig = g.sens.cfgKnown ? g.sens.trig : g.trig;
    c.echo = g.sens.cfgKnown ? g.sens.echo : g.echo;
    c.date = utf8(nowText());
    std::error_code ec;
    fs::create_directories(g.calFile.parent_path(), ec);
    if (!rangecal::save(g.calFile, c)) {
        g.calError = L"Kaydedilemedi: " + g.calFile.wstring();
        return;
    }
    g.cal = c;
}

void onSensor() {
    g.sens = g.link.snapshot();
    // Pins: the board is the truth; follow it unless the user is editing.
    if (g.sens.cfgKnown && !g.pinsDirty && !g.pinWait) { g.trig = g.sens.trig; g.echo = g.sens.echo; }
    if (g.sens.cfgKnown && g.hookTrig && !g.pinWait) {   // test hook --pins
        g.trig = g.hookTrig;
        g.echo = g.hookEcho;
        g.hookTrig = g.hookEcho = 0;
        g.pinsDirty = true;
        writePins();
    }
    if (g.pinWait) {
        if (g.sens.pinErrors > g.pinErrors) {
            g.pinWait = false;
            g.pinMsg = L"✗ Kart pinleri reddetti (D2..D13 ve farklı olmalı)";
        } else if (g.sens.cfgReplies > g.pinReplies) {
            g.pinWait = false;
            g.pinOk = g.sens.trig == g.trig && g.sens.echo == g.echo;
            g.pinMsg = g.pinOk ? L"✓ Pinler karta yazıldı (EEPROM: kart yeniden başlasa da kalır)" : L"✗ Kart farklı pinler bildirdi";
            if (g.pinOk) { g.pinsDirty = false; saveSettings(); }
        } else if (GetTickCount64() - g.pinSent > 2500) {
            g.pinWait = false;
            g.pinMsg = L"✗ Kart yanıt vermedi - sensor.ino v2 yüklü mü?";
        }
    }
    auto& r = g.calRun;
    if (r.active) {
        for (const auto& smp : g.sens.recent) {
            if (smp.seq <= r.lastSeq) continue;
            r.lastSeq = smp.seq;
            if (!smp.echo()) { ++r.noEcho; continue; }
            r.raw.push_back(rawCm(smp));
            r.us.push_back(smp.us >= 0 ? static_cast<double>(smp.us) : -1.0);
        }
        const bool tooLong = static_cast<double>(GetTickCount64() - r.t0) / 1000.0 > CalMaxSeconds;
        if (r.verify) {
            if (r.raw.size() >= static_cast<std::size_t>(VerifySamples)) finishVerify();
            else if (r.noEcho > CalMaxNoEcho || tooLong) r.active = false;
        }
        else if (r.raw.size() >= static_cast<std::size_t>(CalSamples)) finishCalibration();
        else if (r.noEcho > CalMaxNoEcho) { r.active = false; g.calError = L"Yankı alınamıyor: blok çok uzak / eğik ya da sensör bağlı değil."; }
        else if (tooLong) { r.active = false; g.calError = L"Zaman aşımı: yeterli ölçüm gelmedi."; }
    }
    if (g.sens.state == sensor::State::Error) r.active = false;
    repaint();
}

// Open the calibration panel once, the first time both devices answer and the
// sensor has no calibration yet (step 3 follows step 2).
void maybeShowCalibration() {
    if (g.real && connectionOk() && !g.cal.valid && !g.calAutoShown && !g.run.running() && g.tab == 0 && g.panel == Panel::None) {
        g.panel = Panel::Cal;
        g.calAutoShown = true;
        g.haveLive = false;
    }
}

// ------------------------------------------------ sketch upload (arduino-cli) --
fs::path repoRoot() { return g.dir.parent_path(); }
fs::path cliPath() { return arduino::findCli(repoRoot()); }

// setup = false: compile + upload arduino/sensor/sensor.ino to the Arduino port.
// setup = true : run arduino/setup-arduino-cli.ps1 (downloads arduino-cli + the AVR core).
void startUpload(bool setup) {
    if (g.up.running || g.run.running()) return;
    const fs::path cli = cliPath();
    std::vector<std::wstring> cmds, labels;   // labels: the short form shown in the log
    if (setup) {
        if (MessageBoxW(g.hwnd, L"arduino-cli bulunamadı.\n\nŞimdi indirilsin mi? (~180 MB: arduino-cli depo içindeki .tools klasörüne, "
                                L"AVR çekirdeği Arduino IDE'nin kullandığı %LOCALAPPDATA%\\Arduino15 klasörüne kurulur.)",
                        L"Sketch'i yükle", MB_ICONQUESTION | MB_OKCANCEL) != IDOK)
            return;
        cmds.push_back(L"powershell -NoProfile -ExecutionPolicy Bypass -File " + q((g.dir / "arduino" / "setup-arduino-cli.ps1").wstring()));
        labels.push_back(L"arduino\\setup-arduino-cli.ps1");
    } else {
        if (g.arduino.port.empty()) return;
        const std::wstring fqbn = arduino::boards()[static_cast<std::size_t>(g.board)].fqbn;
        const std::wstring sketch = q((g.dir / "arduino" / "sensor").wstring());
        cmds.push_back(q(cli.wstring()) + L" compile --fqbn " + fqbn + L" " + sketch);
        cmds.push_back(q(cli.wstring()) + L" upload -p " + g.arduino.port + L" --fqbn " + fqbn + L" " + sketch);
        labels.push_back(L"arduino-cli compile --fqbn " + fqbn + L" arduino\\sensor");
        labels.push_back(L"arduino-cli upload -p " + g.arduino.port + L" --fqbn " + fqbn + L" arduino\\sensor");
    }
    if (g.up.th.joinable()) g.up.th.join();
    g.up.log.clear();
    g.up.running = true;
    g.up.done = g.up.ok = false;
    g.up.show = true;
    g.up.setup = setup;
    g.up.cancel = false;
    g.calRun.active = false;
    const HWND hwnd = g.hwnd;
    const std::wstring work = g.dir.wstring();
    g.up.th = std::thread([cmds, labels, hwnd, work] {
        int code = 0;
        for (std::size_t i = 0; i < cmds.size(); ++i) {
            const std::wstring& cmd = cmds[i];
            PostMessageW(hwnd, WmUpLine, 0, reinterpret_cast<LPARAM>(new std::wstring(L"> " + labels[i])));
            code = arduino::runCaptured(cmd, work, [hwnd](const std::string& line) {
                auto* w = new std::wstring(widen(line));
                if (!PostMessageW(hwnd, WmUpLine, 0, reinterpret_cast<LPARAM>(w))) delete w;
            }, g.up.cancel);
            if (code != 0) break;
        }
        PostMessageW(hwnd, WmUpDone, static_cast<WPARAM>(code), 0);
    });
}

void chooseBoard(double bx, double by) {
    HMENU menu = CreatePopupMenu();
    const auto& b = arduino::boards();
    for (std::size_t i = 0; i < b.size(); ++i)
        AppendMenuW(menu, MF_STRING | (static_cast<int>(i) == g.board ? MF_CHECKED : 0), 3000 + i, (std::wstring(b[i].name) + L"     " + b[i].fqbn).c_str());
    POINT pt{ static_cast<LONG>(bx * g.scale), static_cast<LONG>(by * g.scale) };
    ClientToScreen(g.hwnd, &pt);
    const int cmd = TrackPopupMenu(menu, TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN, pt.x, pt.y, 0, g.hwnd, nullptr);
    DestroyMenu(menu);
    if (cmd < 3000) return;
    g.board = cmd - 3000;
    saveSettings();
}

// ---------------------------------------------------------- pre-flight check --
// Everything that makes a real run fail half-way, in one list. Both ports are
// listened to again when the panel opens (Probe.h, read-only).
struct PreItem {
    int level;   // 0 ok, 1 warning, 2 problem, 3 being checked
    std::wstring title, detail;
};

std::vector<PreItem> preflightItems() {
    std::vector<PreItem> v;
    const probe::Result& k = g.kobuki.r;
    const probe::Result& a = g.arduino.r;
    const bool kb = k.kind == probe::Kind::Testing, ab = a.kind == probe::Kind::Testing;
    const bool kOk = k.kind == probe::Kind::Kobuki, aOk = a.kind == probe::Kind::Arduino;

    v.push_back(exeExists(g.method) ? PreItem{ 0, L"Program", Methods[static_cast<std::size_t>(g.method)].exe + L" hazır" }
                                    : PreItem{ 2, L"Program", Methods[static_cast<std::size_t>(g.method)].exe + L" derlenmemiş" });
    // Kobuki
    if (g.kobuki.port.empty()) v.push_back({ 2, L"Kobuki bağlantısı", L"Port seçilmedi (2. adım)" });
    else if (kb) v.push_back({ 3, L"Kobuki bağlantısı", g.kobuki.port + L" dinleniyor..." });
    else if (kOk) v.push_back({ 0, L"Kobuki bağlantısı", g.kobuki.port + fmt(L" · %.0f Hz", k.hz) });
    else v.push_back({ 2, L"Kobuki bağlantısı", g.kobuki.port + L": " + (k.detail.empty() ? std::wstring(L"doğrulanmadı") : k.detail) });
    if (kOk) {
        if (k.battery >= 14.5) v.push_back({ 0, L"Batarya", fmt(L"%.1f V", k.battery) });
        else if (k.battery >= 13.5) v.push_back({ 1, L"Batarya", fmt(L"%.1f V - düşük, uzun koşu yarıda kalabilir", k.battery) });
        else v.push_back({ 2, L"Batarya", fmt(L"%.1f V - çok düşük, önce şarj edin", k.battery) });
        if (k.bumper) v.push_back({ 2, L"Tampon", L"Basılı: robot bir şeye dayanmış" });
        else if (k.wheelDrop) v.push_back({ 2, L"Tekerlekler", L"Tekerlek havada (robot yerde değil)" });
        else if (k.cliff) v.push_back({ 1, L"Zemin sensörü", L"Boşluk algılıyor (masa kenarı / koyu zemin?)" });
        else v.push_back({ 0, L"Tampon ve tekerlekler", L"Tampon serbest, tekerlekler yerde" });
        if (k.charger) v.push_back({ 1, L"Şarj", L"Adaptör / istasyon takılı: kabloyu çıkarın" });
    }
    // Arduino
    if (g.arduino.port.empty()) v.push_back({ 2, L"Arduino bağlantısı", L"Port seçilmedi (2. adım)" });
    else if (g.arduino.port == g.kobuki.port) v.push_back({ 2, L"Arduino bağlantısı", L"Kobuki ile aynı port seçilmiş" });
    else if (ab) v.push_back({ 3, L"Arduino bağlantısı", g.arduino.port + L" dinleniyor..." });
    else if (aOk) v.push_back({ 0, L"Arduino bağlantısı", g.arduino.port + fmt(L" · %.0f Hz · ", a.hz) + (a.hasUs ? L"sensor.ino v2" : L"eski sensor.ino") });
    else v.push_back({ 2, L"Arduino bağlantısı", g.arduino.port + L": " + (a.detail.empty() ? std::wstring(L"doğrulanmadı") : a.detail) });
    if (aOk) {
        if (a.lines > 0 && a.noEcho * 2 > a.lines) v.push_back({ 1, L"Ön sensör", L"Yankı yok: sensör bağlı mı, pinler doğru mu? (3. adım)" });
        else if (a.frontCm >= 0 && a.frontCm < 12) v.push_back({ 1, L"Ön sensör", std::to_wstring(a.frontCm) + L" cm: sensörün önü kapalı" });
        else v.push_back({ 0, L"Ön sensör", a.frontCm >= 0 ? L"Okuyor: " + std::to_wstring(a.frontCm) + L" cm" : std::wstring(L"Okuyor") });
    }
    // Calibration
    if (!g.cal.valid) v.push_back({ 1, L"Sensör kalibrasyonu", L"Yapılmadı: ham HC-SR04 mesafeleri kullanılır (3. adım)" });
    else {
        const std::wstring text = fmt(L"ham × %.4f · ", g.cal.scale) + ageText(calAgeHours());
        if (calStale()) v.push_back({ 1, L"Sensör kalibrasyonu", text + L" - bir günden eski, yenilemeniz önerilir" });
        else v.push_back({ 0, L"Sensör kalibrasyonu", text });
    }
    return v;
}

void launchRun();

// Real robot: open the check panel and listen to both ports again.
void openPreflight() {
    g.panel = Panel::Preflight;
    g.haveLive = false;
    g.preChecking = true;
    g.kobuki.r = probe::Result{};
    g.arduino.r = probe::Result{};
    probeRole(0);
    probeRole(1);
    if (g.probing == 0) g.preChecking = false;   // no port chosen at all
}

// ------------------------------------------------------------------- run --
void startRun() {
    if (busy() || g.maps.empty()) return;
    if (!exeExists(g.method)) {
        MessageBoxW(g.hwnd, (Methods[static_cast<std::size_t>(g.method)].exe + L" derlenmemiş.\n.\\run.ps1 launcher ile derleyin.").c_str(),
                    L"Exploration Mapping", MB_ICONWARNING);
        return;
    }
    if (g.real) openPreflight();
    else launchRun();
}

void launchRun() {
    if (busy() || g.maps.empty() || !exeExists(g.method)) return;
    const MapRef& map = g.maps[static_cast<std::size_t>(g.map)];
    std::wstring args;
    if (g.real) {
        if (g.kobuki.port.empty() || g.arduino.port.empty() || g.kobuki.port == g.arduino.port) return;
        args = g.kobuki.port + L" " + g.arduino.port + L" --map " + q(map.path) + L" --range-cal " + q(g.calFile.wstring()) + L" --ipc";
    } else {
        args = L"--sim --map " + q(map.path) + L" --ipc";
        if (!g.noise) args += L" --no-noise";
        if (g.fast) args += L" --fast";
    }
    const std::wstring exe = (g.bin / Methods[static_cast<std::size_t>(g.method)].exe).wstring();
    g.panel = Panel::None;   // the program opens both ports itself
    syncLink();
    if (!g.run.start(exe, args, g.bin.wstring(), g.hwnd, WmFrame, WmRunEnd)) {
        MessageBoxW(g.hwnd, (L"Başlatılamadı: " + exe).c_str(), L"Exploration Mapping", MB_ICONERROR);
        return;
    }
    g.live = live::State{};
    g.haveLive = true;
    g.stopping = false;
    g.runMethod = g.method;
    g.runMap = g.map;
    g.runReal = g.real;
    saveSettings();
    repaint();
}

// The finished run's state lines next to its map, for the history tab.
void saveReplay() {
    g.live = g.run.snapshot();
    if (!g.live.saved.empty()) g.run.saveRecording(fs::path(widen(g.live.saved)).replace_extension(".replay").wstring());
}

void stopRun() {
    if (!g.run.running() || g.stopping) return;
    g.run.stop();
    g.stopping = true;
    SetTimer(g.hwnd, TimerKill, 10000, nullptr);
    repaint();
}

void reloadResults() { g.rows = results::load(resultsCsv()); }

// ------------------------------------------------------------- benchmark --
bool spawnLogged(const std::wstring& exe, const std::wstring& args, HANDLE out, PROCESS_INFORMATION& pi) {
    std::wstring cmd = q(exe) + L" " + args;
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
    const HANDLE nul = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, 0, nullptr);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nul;
    si.hStdOutput = out;
    si.hStdError = out;
    const std::wstring work = g.bin.wstring();
    const BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, work.c_str(), &si, &pi);
    if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
    return ok != FALSE;
}

void benchWorker(HWND hwnd) {
    for (std::size_t i = 0; i < g.jobs.size() && !g.benchCancel; ++i) {
        const Job& j = g.jobs[i];
        PostMessageW(hwnd, WmBenchStep, i, 0);
        SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
        const HANDLE out = CreateFileW(j.log.c_str(), GENERIC_WRITE, FILE_SHARE_READ, &sa, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        PROCESS_INFORMATION pi{};
        const fs::path exe = g.bin / Methods[static_cast<std::size_t>(j.method)].exe;
        if (spawnLogged(exe.wstring(), j.args, out, pi)) {
            CloseHandle(pi.hThread);
            while (WaitForSingleObject(pi.hProcess, 100) == WAIT_TIMEOUT)
                if (g.benchCancel) TerminateProcess(pi.hProcess, 3);   // simulator only: safe to kill
            CloseHandle(pi.hProcess);
        }
        if (out != INVALID_HANDLE_VALUE) CloseHandle(out);
        PostMessageW(hwnd, WmBenchStep, i, 1);
    }
    PostMessageW(hwnd, WmBenchDone, g.benchCancel ? 1 : 0, 0);
}

void startBenchmark() {
    if (busy()) return;
    g.jobs.clear();
    const fs::path logs = g.dir / "results" / "logs";
    std::error_code ec;
    fs::create_directories(logs, ec);
    for (int s = 1; s <= g.reps; ++s)
        for (std::size_t m = 0; m < g.maps.size(); ++m) {
            if (!g.benchMap[m]) continue;
            for (std::size_t a = 0; a < Methods.size(); ++a) {
                if (!g.benchAlgo[a] || !exeExists(static_cast<int>(a))) continue;
                Job j;
                j.method = static_cast<int>(a);
                j.map = static_cast<int>(m);
                j.seed = static_cast<unsigned>(s);
                j.args = L"--sim --fast --no-gui --map " + q(g.maps[m].path) + L" --seed " + std::to_wstring(s) + L" --result " +
                         q(resultsCsv().wstring());
                if (!g.benchNoise) j.args += L" --no-noise";
                j.log = (logs / (widen(Methods[a].id) + L"_" + g.maps[m].name + L"_s" + std::to_wstring(s) + L".log")).wstring();
                j.label = Methods[a].name + L" · " + g.maps[m].name + L" · seed " + std::to_wstring(s);
                g.jobs.push_back(j);
            }
        }
    if (g.jobs.empty()) {
        MessageBoxW(g.hwnd, L"En az bir yöntem ve bir harita seçin.", L"Benchmark", MB_ICONINFORMATION);
        return;
    }
    if (g.bench.joinable()) g.bench.join();
    g.benchCancel = false;
    g.benchRunning = true;
    g.benchIndex = 0;
    saveSettings();
    g.bench = std::thread(benchWorker, g.hwnd);
    repaint();
}

std::string filterMode() { return g.filter == 0 ? "sim" : (g.filter == 1 ? "real" : ""); }
std::vector<std::string> methodOrder() {
    std::vector<std::string> o;
    for (const Method& m : Methods) o.push_back(m.id);
    return o;
}

bool saveReport(bool open) {
    const results::Comparison c = results::compare(g.rows, filterMode(), g.threshold / 100.0, methodOrder());
    if (c.methods.empty()) {
        if (open) MessageBoxW(g.hwnd, L"Raporlanacak koşu yok.", L"Rapor", MB_ICONINFORMATION);
        return false;
    }
    std::map<std::string, std::string> names;
    for (const Method& m : Methods) names[m.id] = utf8(m.name);
    SYSTEMTIME t;
    GetLocalTime(&t);
    wchar_t stamp[32];
    swprintf(stamp, 32, L"%04d%02d%02d_%02d%02d%02d", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    const fs::path file = g.dir / "results" / (L"comparison_" + std::wstring(stamp) + L".md");
    if (!results::writeReport(file, g.rows, c, filterMode(), g.threshold / 100.0, names)) return false;
    g.lastReport = file.wstring();
    if (open) ShellExecuteW(g.hwnd, L"open", file.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    return true;
}

void archiveResults() {
    if (!fs::exists(resultsCsv())) return;
    if (MessageBoxW(g.hwnd, L"results.csv arşivlenip boş bir tablo başlatılsın mı?\n(Dosya silinmez, results_<tarih>.csv olarak saklanır.)",
                    L"Sonuçlar", MB_ICONQUESTION | MB_OKCANCEL) != IDOK)
        return;
    SYSTEMTIME t;
    GetLocalTime(&t);
    wchar_t stamp[32];
    swprintf(stamp, 32, L"%04d%02d%02d_%02d%02d%02d", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    std::error_code ec;
    fs::rename(resultsCsv(), g.dir / "results" / (L"results_" + std::wstring(stamp) + L".csv"), ec);
    reloadResults();
}

void openFolder(const fs::path& p) {
    std::error_code ec;
    fs::create_directories(p, ec);
    ShellExecuteW(g.hwnd, L"open", p.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

// ================================================================ drawing ===
void stepTitle(Painter& P, double x, double y, int n, const std::wstring& title, bool active = true) {
    P.circle(x + 12, y + 12, 12, active ? col::Accent : col::Track);
    P.text(std::to_wstring(n), x, y, 24, 24, 12.5f, active ? 0xFFFFFFFF : col::Muted, true, Align::Center);
    P.text(title, x + 34, y, 300, 24, 15.0f, active ? col::Text : col::Faint, true);
}

enum class Btn { Primary, Secondary, Danger };
void button(Painter& P, double x, double y, double w, double h, const std::wstring& label, Btn style, Act a, int arg = 0,
            bool enabled = true, float size = 13.5f) {
    const bool hv = enabled && hovered(a, arg);
    ARGB fill = col::Card, stroke = col::Border, fg = col::Text;
    switch (style) {
        case Btn::Primary: fill = hv ? col::AccentDark : col::Accent; stroke = 0; fg = 0xFFFFFFFF; break;
        case Btn::Danger: fill = hv ? 0xFFB91C1C : col::Red; stroke = 0; fg = 0xFFFFFFFF; break;
        case Btn::Secondary: fill = hv ? col::Hover : col::Card; break;
    }
    if (!enabled) { fill = col::Track; stroke = 0; fg = col::Faint; }
    P.round(x, y, w, h, 10, fill, stroke);
    P.text(label, x, y, w, h, size, fg, true, Align::Center);
    if (enabled) hit(x, y, w, h, a, arg);
}

void segmented(Painter& P, double x, double y, double w, double h, const std::vector<std::wstring>& labels, int sel, Act a,
               bool enabled = true) {
    P.round(x, y, w, h, 10, col::Hover);
    const double sw = (w - 8) / static_cast<double>(labels.size());
    for (std::size_t i = 0; i < labels.size(); ++i) {
        const double sx = x + 4 + sw * static_cast<double>(i);
        const bool on = static_cast<int>(i) == sel;
        if (on) P.round(sx, y + 4, sw, h - 8, 8, col::Card, col::Border);
        else if (enabled && hovered(a, static_cast<int>(i))) P.round(sx, y + 4, sw, h - 8, 8, 0xFFE8EDF3);
        P.text(labels[i], sx, y + 4, sw, h - 8, 13.5f, on ? (enabled ? col::Text : col::Muted) : col::Muted, on, Align::Center);
        if (enabled) hit(sx, y + 4, sw, h - 8, a, static_cast<int>(i));
    }
}

void toggle(Painter& P, double x, double y, const std::wstring& label, bool on, Act a, bool enabled = true) {
    P.round(x, y + 2, 36, 20, 10, !enabled ? col::Track : (on ? col::Accent : 0xFFCBD5E1));
    P.circle(on ? x + 26 : x + 10, y + 12, 7.5, 0xFFFFFFFF);
    P.text(label, x + 44, y, 200, 24, 13.0f, enabled ? col::Text : col::Faint);
    if (enabled) hit(x, y, 44 + P.measure(label, 13.0f) + 4, 24, a);
}

void chip(Painter& P, double x, double y, double w, double h, const std::wstring& label, bool on, Act a, int arg, bool enabled = true) {
    const bool hv = enabled && hovered(a, arg);
    P.round(x, y, w, h, 9, on ? col::AccentFaint : (hv ? col::Hover : col::Card), on ? col::Accent : col::Border, on ? 1.5 : 1.0);
    P.round(x + 10, y + h / 2 - 8, 16, 16, 4, on ? col::Accent : col::Card, on ? 0 : 0xFFCBD5E1, 1.5);
    if (on) {
        P.line(x + 14, y + h / 2, x + 17, y + h / 2 + 3, 0xFFFFFFFF, 2);
        P.line(x + 17, y + h / 2 + 3, x + 22, y + h / 2 - 3, 0xFFFFFFFF, 2);
    }
    P.text(label, x + 32, y, w - 36, h, 13.0f, enabled ? col::Text : col::Faint, on);
    if (enabled) hit(x, y, w, h, a, arg);
}

void stepper(Painter& P, double x, double y, const std::wstring& label, const std::wstring& value, Act minus, Act plus, bool enabled) {
    P.text(label, x, y, 150, 32, 13.0f, col::Muted);
    const double bx = x + 160;
    P.round(bx, y, 120, 32, 9, col::Card, col::Border);
    P.text(L"−", bx, y, 34, 32, 16.0f, enabled && hovered(minus) ? col::Accent : col::Muted, true, Align::Center);
    P.text(value, bx + 34, y, 52, 32, 13.5f, col::Text, true, Align::Center);
    P.text(L"+", bx + 86, y, 34, 32, 16.0f, enabled && hovered(plus) ? col::Accent : col::Muted, true, Align::Center);
    if (!enabled) return;
    hit(bx, y, 34, 32, minus);
    hit(bx + 86, y, 34, 32, plus);
}

// Colour of a cell by P(occupied): white -> slate (unknown) -> dark red.
ARGB cellColour(double p) {
    const auto mix = [](int a, int b, double t) { return static_cast<int>(std::lround(a + (b - a) * std::clamp(t, 0.0, 1.0))); };
    int r, gg, b;
    if (p <= 0.5) {
        const double t = p / 0.5;
        r = mix(255, 203, t); gg = mix(255, 213, t); b = mix(255, 225, t);
    } else {
        const double t = (p - 0.5) / 0.5;
        r = mix(203, 153, t); gg = mix(213, 27, t); b = mix(225, 27, t);
    }
    return 0xFF000000u | (static_cast<ARGB>(r) << 16) | (static_cast<ARGB>(gg) << 8) | static_cast<ARGB>(b);
}

void miniMap(Painter& P, const astar::GridMap& m, double x, double y, double w, double h) {
    const double c = std::min(w / m.cols(), h / m.rows());
    const double ox = x + (w - c * m.cols()) / 2, oy = y + (h - c * m.rows()) / 2;
    for (int yy = 0; yy < m.rows(); ++yy)
        for (int xx = 0; xx < m.cols(); ++xx)
            P.rect(ox + xx * c + 0.4, oy + yy * c + 0.4, c - 0.8, c - 0.8, m.occupied({ xx, yy }) ? col::Obstacle : 0xFFF1F5F9);
    if (m.start) P.circle(ox + (m.start->x + 0.5) * c, oy + (m.start->y + 0.5) * c, c * 0.32, col::Green);
}

// --- header ---------------------------------------------------------------
void drawHeader(Painter& P) {
    P.rect(0, 0, BW, 64, col::Card);
    P.rect(0, 63, BW, 1, col::Border);
    P.round(24, 14, 36, 36, 10, col::Accent);
    P.text(L"11", 24, 14, 36, 36, 15.0f, 0xFFFFFFFF, true, Align::Center);
    P.text(L"Exploration Mapping", 72, 10, 400, 26, 19.0f, col::Text, true);
    P.text(L"Kobuki · tek HC-SR04 · Bayes doluluk ızgarası · 4 keşif yöntemi", 72, 36, 600, 18, 12.5f, col::Muted);
    segmented(P, BW - 24 - 520, 14, 520, 38, { L"Çalıştır", L"Karşılaştırma", L"Geçmiş", L"Haritalar" }, g.tab, Act::TabRun,
              !g.run.running() && !g.benchRunning && !g.up.running);
}

// --- live map -----------------------------------------------------------
struct Score {
    int obs = 0, decided = 0, correct = 0, wrong = 0;
    double explored() const { return obs ? static_cast<double>(decided) / obs : 0.0; }
    double precision() const { return decided ? static_cast<double>(correct) / decided : 0.0; }
    double success() const { return obs ? static_cast<double>(correct) / obs : 0.0; }
};
// Keşif = decided / observable cells, Doğruluk = correct / decided,
// Başarı oranı = correct / observable (criterion 2, as in results.csv).
Score liveScore(const live::State& s, const astar::GridMap& truth) {
    Score sc;
    if (static_cast<int>(s.states.size()) != truth.cols() * truth.rows()) return sc;
    for (int y = 0; y < truth.rows(); ++y)
        for (int x = 0; x < truth.cols(); ++x) {
            if (!observable(truth, x, y)) continue;
            ++sc.obs;
            const char st = s.states[static_cast<std::size_t>(y * truth.cols() + x)];
            if (st == 'u') continue;
            ++sc.decided;
            if ((st == 'o') == truth.occupied({ x, y })) ++sc.correct;
            else ++sc.wrong;
        }
    return sc;
}

// End of a run in Turkish (finish ids of common/Explorer.h).
std::wstring finishTr(const std::string& f) {
    if (f == "all_decided") return L"Harita tamamlandı: tüm hücreler karar verildi";
    if (f == "no_frontier") return L"Harita tamamlandı: frontier kalmadı (görülemeyen hücreler kararsız)";
    if (f == "stagnated") return L"Harita tamamlandı: yeni hücre öğrenilmiyor (Random durdu)";
    if (f == "time_limit") return L"Durdu: süre sınırı";
    if (f == "too_many_bumps") return L"Durdu: çok fazla tampon teması";
    return L"Durduruldu (harita kaydedildi)";
}

std::wstring phaseTr(const std::string& p) {
    if (p == "SCAN") return L"360° tarama";
    if (p == "PLAN") return L"Planlama";
    if (p == "DRIVE") return L"Sürüş";
    if (p == "FACE") return L"Hedefe dönüş";
    if (p == "MEASURE") return L"Karşıdan ölçüm";
    if (p == "BACK_OFF") return L"Geri çekilme";
    if (p == "DONE") return L"Bitti";
    return widen(p);
}

void kpi(Painter& P, double x, double y, double w, const std::wstring& label, const std::wstring& value, const std::wstring& sub,
         double frac, ARGB c) {
    P.card(x, y, w, 96);
    P.text(label, x + 16, y + 10, w - 32, 20, 12.0f, col::Muted, true);
    P.text(value, x + 16, y + 30, w - 32, 38, 26.0f, frac >= 0 ? c : col::Text, true);
    if (frac >= 0) P.progress(x + 16, y + 76, w - 32, 6, frac, c);
    else P.text(sub, x + 16, y + 68, w - 32, 20, 11.5f, col::Muted);
}

// What the map card and the side cards show: the running program (run tab)
// or a recorded run (history tab).
struct LiveView {
    const live::State* s = nullptr;
    bool show = false;        // a run is shown; otherwise the reference map
    int map = 0, method = 0;  // map = -1: the run's obstacle map does not exist any more
    bool real = false, running = false, stopping = false, replay = false;
    DWORD exitCode = 0;
};
LiveView runView() {
    return { &g.live, g.haveLive, g.haveLive ? g.runMap : g.map, g.haveLive ? g.runMethod : g.method, g.haveLive ? g.runReal : g.real,
             g.run.running(), g.stopping, false, g.run.exitCode() };
}

void drawLive(Painter& P, double x0, double y0, const LiveView& v) {
    const live::State& s = *v.s;
    const bool running = v.running;
    const bool showRun = v.show;
    const MapRef* ref = (g.maps.empty() || v.map < 0 || v.map >= static_cast<int>(g.maps.size())) ? nullptr : &g.maps[static_cast<std::size_t>(v.map)];
    const Method& method = Methods[static_cast<std::size_t>(std::clamp(v.method, 0, static_cast<int>(Methods.size()) - 1))];

    // KPI tiles.
    const double tw = (BW - 20 - x0 - 4 * 12) / 5.0;
    Score sc;
    const bool have = showRun && s.frame && ref;
    if (have) sc = liveScore(s, ref->grid);
    const auto pct = [](double v) { return fmt(L"%%%.0f", v * 100.0); };
    kpi(P, x0, y0, tw, L"KEŞİF", have ? pct(sc.explored()) : L"—", L"", have ? sc.explored() : 0.0, col::Accent);
    kpi(P, x0 + (tw + 12), y0, tw, L"DOĞRULUK", have && sc.decided ? pct(sc.precision()) : L"—", L"", have ? sc.precision() : 0.0,
        sc.wrong ? col::Amber : col::Green);
    kpi(P, x0 + 2 * (tw + 12), y0, tw, L"BAŞARI ORANI", have ? pct(sc.success()) : L"—", L"", have ? sc.success() : 0.0, col::Purple);
    const double t = s.done ? s.endTime : s.t;
    kpi(P, x0 + 3 * (tw + 12), y0, tw, L"SÜRE", have ? fmt(L"%.0f s", t) : L"—",
        have ? fmt(L"%.0f dk ", std::floor(t / 60.0)) + fmt(L"%02.0f sn", std::fmod(t, 60.0)) + (v.real ? L"" : L" (sim.)")
             : L"haritayı tamamlama süresi",
        -1, col::Text);
    kpi(P, x0 + 4 * (tw + 12), y0, tw, L"YOL", have ? fmt(L"%.1f m", s.driven) : L"—",
        have ? std::to_wstring(s.goals) + L" hedef · " + std::to_wstring(s.bumps) + L" tampon"
             : L"sürülen mesafe",
        -1, col::Text);

    // Map card.
    const double my = y0 + 108, mw = 600, mh = BH - 20 - my;
    P.card(x0, my, mw, mh);
    P.text(showRun ? (v.replay ? (s.staticMap ? L"Kaydedilen harita" : L"Kayıtlı koşu") : L"Canlı harita") : L"Engel haritası (referans)", x0 + 20,
           my + 14, 300, 24, 15.0f, col::Text, true);
    const std::wstring sub = method.name + L"  ·  " + (ref ? ref->name : L"-") + L"  ·  " +
                             (v.real ? L"gerçek robot" : L"simülasyon") +
                             (showRun ? L"" : (ref ? L"  ·  " + ref->desc : L""));
    P.text(sub, x0 + 20, my + 38, 420, 18, 12.0f, col::Muted);
    std::wstring ptxt;
    ARGB pfg = col::Accent, pbg = col::AccentSoft;
    if (!showRun) { ptxt = v.replay ? L"Kayıt dosyası yok" : L"Hazır"; pfg = col::Muted; pbg = col::Hover; }
    else if (s.done) {
        ptxt = s.completed ? L"✓ Harita tamamlandı" : L"Durduruldu";
        pfg = s.completed ? col::Green : col::Amber;
        pbg = s.completed ? col::GreenSoft : col::AmberSoft;
    } else if (v.replay) ptxt = phaseTr(s.phase);
    else if (running && !s.frame) ptxt = v.real ? L"Bağlanıyor..." : L"Başlıyor...";
    else if (!running) { ptxt = L"Program kapandı"; pfg = col::Red; pbg = col::RedSoft; }
    else ptxt = (v.stopping ? L"Durduruluyor · " : L"") + phaseTr(s.phase);
    const double pw = P.measure(ptxt, 12.5f, true) + 18;
    P.pill(ptxt, x0 + mw - 20 - pw, my + 16, 26, pfg, pbg);
    if (s.done && !s.saved.empty() && !v.replay)
        button(P, x0 + mw - 20 - pw - 108, my + 14, 96, 30, L"Resmi aç", Btn::Secondary, Act::OpenPicture, 0, true, 12.5f);

    if (!ref) {
        P.text(L"Bu koşunun engel haritası artık yok (silinmiş ya da adı değişmiş).", x0, my + 120, mw, 24, 13.0f, col::Faint, false,
               Align::Center);
        return;
    }
    const astar::GridMap& truth = ref->grid;
    const int cols = truth.cols(), rows = truth.rows();
    const double areaX = x0 + 44, areaY = my + 66, areaW = mw - 64, areaH = mh - 66 - 64;
    const double c = std::floor(std::min(areaW / cols, areaH / rows));
    const double gx = areaX + (areaW - c * cols) / 2, gy = areaY + (areaH - c * rows) / 2;
    const double cellM = s.header ? s.cell : truth.cellSize();
    const auto mx = [&](double x) { return gx + x / cellM * c; };
    const auto myy = [&](double y) { return gy + (rows - y / cellM) * c; };
    const bool showCells = showRun && s.frame && static_cast<int>(s.p.size()) == cols * rows;

    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const double cx = gx + x * c, cy = gy + y * c;
            if (showCells) {
                const double p = s.p[static_cast<std::size_t>(y * cols + x)] / 100.0;
                P.rect(cx, cy, c, c, cellColour(p));
                P.text(fmt(L"%%%.0f", p * 100.0), cx, cy, c, c, 12.0f, p > 0.62 ? 0xFFFFFFFF : col::Muted, false, Align::Center);
            } else {
                const bool occ = truth.occupied({ x, y });
                P.rect(cx, cy, c, c, occ ? col::Obstacle : 0xFFFFFFFF);
                P.text(std::to_wstring(truth.coords.colLabel(x)) + L"," + std::to_wstring(truth.coords.rowLabel(y)), cx, cy, c, c, 11.5f,
                       occ ? 0xFFFFFFFF : col::Faint, false, Align::Center);
            }
        }
    for (int x = 0; x <= cols; ++x) P.line(gx + x * c, gy, gx + x * c, gy + rows * c, 0x40334155, 1);
    for (int y = 0; y <= rows; ++y) P.line(gx, gy + y * c, gx + cols * c, gy + y * c, 0x40334155, 1);
    P.round(gx - 1, gy - 1, cols * c + 2, rows * c + 2, 3, 0, 0xFF334155, 1.5);
    for (int x = 0; x < cols; ++x)
        P.text(std::to_wstring(truth.coords.colLabel(x)), gx + x * c, gy + rows * c + 4, c, 18, 12.0f, col::Muted, true, Align::Center);
    for (int y = 0; y < rows; ++y)
        P.text(std::to_wstring(truth.coords.rowLabel(y)), gx - 30, gy + y * c, 24, c, 12.0f, col::Muted, true, Align::Right);
    if (truth.start) P.text(L"★", gx + truth.start->x * c + 3, gy + truth.start->y * c + 2, 18, 18, 12.0f, col::Green, true);

    if (showCells) {
        const auto inner = [&](live::CellXY k, ARGB colr, double width, double inset) {
            P.round(gx + k.x * c + inset, gy + k.y * c + inset, c - 2 * inset, c - 2 * inset, 5, 0, colr, width);
        };
        for (const auto& f : s.frontier) inner(f, 0xFF06B6D4, 2.5, 4);
        // Wrong decisions against the reference map.
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < cols; ++x) {
                const char st = s.states.size() == static_cast<std::size_t>(cols * rows) ? s.states[static_cast<std::size_t>(y * cols + x)] : 'u';
                if (st == 'u' || (st == 'o') == truth.occupied({ x, y })) continue;
                const double cx = gx + x * c, cy = gy + y * c;
                P.line(cx + 8, cy + 8, cx + c - 8, cy + c - 8, col::Red, 3);
                P.line(cx + c - 8, cy + 8, cx + 8, cy + c - 8, col::Red, 3);
            }
        if (s.hasTarget) inner(s.target, 0xFFF97316, 3.5, 3);
        if (s.tour.size() >= 2) {   // the method's polyline (TSP tour), dashed
            Gdiplus::Pen pen(Gdiplus::Color(col::Purple), P.S(2.2));
            pen.SetDashStyle(Gdiplus::DashStyleDash);
            std::vector<Gdiplus::PointF> pts;
            for (const auto& p : s.tour) pts.push_back({ P.S(mx(p.x)), P.S(myy(p.y)) });
            P.g.DrawLines(&pen, pts.data(), static_cast<INT>(pts.size()));
        }
        if (s.path.size() >= 2) {
            Gdiplus::Pen pen(Gdiplus::Color(col::Accent), P.S(3.5));
            pen.SetLineJoin(Gdiplus::LineJoinRound);
            std::vector<Gdiplus::PointF> pts;
            for (const auto& k : s.path) pts.push_back({ P.S(gx + (k.x + 0.5) * c), P.S(gy + (k.y + 0.5) * c) });
            P.g.DrawLines(&pen, pts.data(), static_cast<INT>(pts.size()));
        }
        for (const auto& m : s.marks) {
            const double cx = gx + m.c.x * c, cy = gy + m.c.y * c;
            if (m.kind == 2) { P.circle(cx + c / 2, cy + c / 2, c * 0.2, 0, col::Purple, 2.5); continue; }
            if (m.label.empty()) continue;
            const std::wstring l = widen(m.label);
            const double w = 10 + 7.5 * static_cast<double>(l.size());
            P.round(cx + 3, cy + 3, w, 17, 6, m.kind == 0 ? col::Purple : 0xFFA78BFA);
            P.text(l, cx + 3, cy + 3, w, 17, 10.5f, 0xFFFFFFFF, true, Align::Center);
        }
        if (s.beam && !s.staticMap) {
            const double len = s.echo ? s.brange : 0.8;
            const double ex = s.bsx + len * std::cos(s.bangle), ey = s.bsy + len * std::sin(s.bangle);
            P.line(mx(s.bsx), myy(s.bsy), mx(ex), myy(ey), s.echo ? 0xCC15803D : 0x6615803D, 2.5);
            if (s.echo) P.circle(mx(ex), myy(ey), 4.5, 0xFF15803D);
        }
        if (!s.staticMap) {
            const double rr = 0.175 / cellM * c;
            P.circle(mx(s.x), myy(s.y), rr, 0x5522C55E, 0xFF15803D, 2.5);
            P.line(mx(s.x), myy(s.y), mx(s.x + 0.175 * std::cos(s.h)), myy(s.y + 0.175 * std::sin(s.h)), 0xFF15803D, 3.5);
        }
    }

    // Legend.
    double lx = x0 + 20;
    const double ly = my + mh - 34;
    const auto item = [&](ARGB colr, const std::wstring& label, int kind) {
        if (kind == 0) P.round(lx, ly + 3, 14, 14, 3, colr, col::Border);
        else if (kind == 1) P.round(lx, ly + 3, 14, 14, 3, 0, colr, 2.2);
        else P.line(lx, ly + 10, lx + 14, ly + 10, colr, 3);
        P.text(label, lx + 20, ly, 120, 20, 12.0f, col::Muted);
        lx += 30 + P.measure(label, 12.0f);
    };
    if (showCells) {
        item(cellColour(0.02), L"boş", 0);
        item(cellColour(0.5), L"kararsız", 0);
        item(cellColour(0.97), L"dolu", 0);
        item(0xFF06B6D4, L"frontier", 1);
        item(0xFFF97316, L"hedef", 1);
        item(col::Accent, L"yol", 2);
        if (!s.tour.empty()) item(col::Purple, L"tur", 2);
        item(col::Red, L"yanlış", 2);
    } else {
        item(col::Obstacle, L"engel", 0);
        item(0xFFFFFFFF, L"boş", 0);
        P.text(L"★ başlangıç (1,1), robot +y yönüne bakar", lx, ly, 360, 20, 12.0f, col::Muted);
    }
}

void drawSide(Painter& P, double x0, double y0, const LiveView& v) {
    const double w = BW - 20 - x0;
    const live::State& s = *v.s;
    const bool showRun = v.show;
    const Method& m = Methods[static_cast<std::size_t>(std::clamp(v.method, 0, static_cast<int>(Methods.size()) - 1))];

    const double h1 = 210;
    P.card(x0, y0, w, h1);
    P.text(L"Yöntem", x0 + 18, y0 + 14, w - 36, 22, 15.0f, col::Text, true);
    P.pill(m.name, x0 + 18, y0 + 42, 24, col::Accent, col::AccentSoft, 12.0f);
    P.text(m.desc, x0 + 18, y0 + 74, w - 36, 56, 12.0f, col::Muted, false, Align::Left, true, false);
    double iy = y0 + 132;
    if (showRun && s.frame) {
        for (const auto& line : s.info) {
            if (iy > y0 + h1 - 22) break;
            P.text(L"· " + widen(line), x0 + 18, iy, w - 36, 18, 11.5f, col::Text);
            iy += 19;
        }
    } else if (!showRun && !v.replay) {
        P.text(L"Başlat'a basınca yöntemin kararları (sıra, tur, kuyruk) burada ve haritada görünür.", x0 + 18, iy, w - 36, 60, 11.5f,
               col::Faint, false, Align::Left, true, false);
    }

    const double y1 = y0 + h1 + 12, h2 = BH - 20 - y1;
    P.card(x0, y1, w, h2);
    P.text(L"Olaylar", x0 + 18, y1 + 14, w - 36, 22, 15.0f, col::Text, true);
    std::vector<std::pair<std::wstring, ARGB>> lines;
    if (!showRun && v.replay) {
        lines = { { L"Bu koşu için kayıt bulunamadı.", col::Muted } };
    } else if (!showRun) {
        lines = { { L"1. Çalışma modunu seçin.", col::Muted },
                  { L"2. Gerçek robotta portları seçin ve", col::Muted },
                  { L"    bağlantıyı doğrulayın (iki yeşil ✓).", col::Muted },
                  { L"3. Sensörü 40 cm blokla kalibre edin", col::Muted },
                  { L"    (gerçek robot; pinler de orada).", col::Muted },
                  { L"4. Algoritmayı ve haritayı seçin.", col::Muted },
                  { L"5. Başlat.", col::Muted } };
    } else {
        for (const auto& e : s.events) lines.push_back({ widen(e), col::Text });
        if (s.done) {
            lines.push_back({ finishTr(s.finish), s.completed ? col::Green : col::Amber });
            if (!s.saved.empty()) lines.push_back({ L"Kaydedildi: " + fs::path(widen(s.saved)).filename().wstring(), col::Muted });
        }
        if (s.staticMap) lines.push_back({ L"Bu koşunun kare kaydı yok: yalnız son harita.", col::Muted });
        if (!v.running && !s.frame && !v.replay) {   // ended before the first frame: show why
            for (const auto& c : s.console) lines.push_back({ widen(c), col::Red });
            if (s.console.empty()) lines.push_back({ L"Program başlamadan kapandı (çıkış " + std::to_wstring(v.exitCode) + L").", col::Red });
        }
    }
    const double lh = 19;
    const auto fit = static_cast<std::size_t>((h2 - 50) / lh);
    const std::size_t from = lines.size() > fit ? lines.size() - fit : 0;
    double ly = y1 + 44;
    for (std::size_t i = from; i < lines.size(); ++i) {
        P.text(lines[i].first, x0 + 18, ly, w - 36, lh, 11.5f, lines[i].second);
        ly += lh;
    }
}

void dropdown(Painter& P, double x, double y, double w, double h, const std::wstring& value, bool placeholder, Act a, int arg,
              bool enabled) {
    P.round(x, y, w, h, 9, enabled && hovered(a, arg) ? col::Hover : col::Card, col::Border);
    P.text(value, x + 12, y, w - 34, h, 13.5f, !enabled || placeholder ? col::Faint : col::Text, true);
    P.text(L"▾", x + w - 26, y, 20, h, 12.0f, col::Muted, false, Align::Center);
    if (enabled) hit(x, y, w, h, a, arg);
}

void tile(Painter& P, double x, double y, double w, double h, const std::wstring& label, const std::wstring& value,
          const std::wstring& sub, ARGB c) {
    P.round(x, y, w, h, 10, 0xFFF8FAFC, col::Border);
    P.text(label, x + 12, y + 8, w - 24, 18, 11.0f, col::Muted, true);
    P.text(value, x + 12, y + 28, w - 24, 32, 21.0f, c, true);
    P.text(sub, x + 12, y + h - 24, w - 24, 18, 11.0f, col::Muted);
}

#include "Panels.h"   // calibration, manual drive, pre-flight check (right side of the run tab)

void drawRunTab(Painter& P) {
    const double x0 = 20, w = 380;
    const bool locked = g.run.running();

    // 1. Mode.
    double y = 80;
    P.card(x0, y, w, 104);
    stepTitle(P, x0 + 16, y + 14, 1, L"Çalışma modu");
    segmented(P, x0 + 16, y + 48, w - 32, 42, { L"Simülasyon", L"Gerçek robot" }, g.real ? 1 : 0, Act::ModeSim, !locked);

    // 2. Connection (real robot) / simulator settings.
    y = 196;
    const double h2 = 206;
    P.card(x0, y, w, h2);
    if (!g.real) {
        stepTitle(P, x0 + 16, y + 14, 2, L"Simülatör");
        toggle(P, x0 + 16, y + 52, L"Sensör gürültüsü", g.noise, Act::Noise, !locked);
        P.text(L"gürültü, kaçan yankı, hayalet okuma", x0 + 60, y + 76, w - 76, 18, 11.5f, col::Muted);
        toggle(P, x0 + 16, y + 106, L"Hızlı (beklemeden)", g.fast, Act::Fast, !locked);
        P.text(L"kapalıyken ~2.5 × gerçek zaman, izlenebilir", x0 + 60, y + 130, w - 76, 18, 11.5f, col::Muted);
        P.text(L"Gerçek robotta burada Kobuki ve Arduino portları seçilip doğrulanır.", x0 + 18, y + 160, w - 36, 36, 11.5f, col::Faint,
               false, Align::Left, true, false);
    } else {
        stepTitle(P, x0 + 16, y + 14, 2, L"Bağlantı");
        if (!g.scanText.empty()) P.text(g.scanText, x0 + 160, y + 14, w - 178, 24, 11.0f, col::Faint, false, Align::Right);
        for (int role = 0; role < 2; ++role) {
            const double ry = y + 48 + role * 58;
            const PortStatus& ps = role == 0 ? g.kobuki : g.arduino;
            const Act a = role == 0 ? Act::KobukiPort : Act::ArduinoPort;
            P.text(role == 0 ? L"Kobuki" : L"Arduino", x0 + 18, ry, 70, 34, 13.5f, col::Text, true);
            dropdown(P, x0 + 92, ry, 112, 34, ps.port.empty() ? L"Seçin" : ps.port, ps.port.empty(), a, 0, !locked);
            const Badge b = badge(role);
            const double bw = std::min(148.0, P.measure(b.label, 12.0f, true) + 18);
            P.round(x0 + w - 18 - bw, ry + 5, bw, 24, 12, b.bg);
            P.text(b.label, x0 + w - 18 - bw, ry + 5, bw, 24, 12.0f, b.fg, true, Align::Center);
            P.text(b.detail, x0 + 18, ry + 36, w - 36, 18, 11.5f, b.fg == col::Green ? col::Muted : b.fg);
        }
        const bool canTest = !locked && !g.scanning && g.probing == 0;
        button(P, x0 + 16, y + 164, 100, 34, g.probing ? L"Test..." : L"Test et", Btn::Secondary, Act::Test, 0,
               canTest && (!g.kobuki.port.empty() || !g.arduino.port.empty()), 12.5f);
        button(P, x0 + 124, y + 164, 116, 34, g.scanning ? L"Taranıyor..." : L"Portları tara", Btn::Secondary, Act::Scan, 0, canTest, 12.5f);
        button(P, x0 + 248, y + 164, w - 264, 34, g.panel == Panel::Drive ? L"Sürüşü kapat" : L"Elle sürüş  ›", Btn::Secondary, Act::DriveOpen, 0,
               canTest && !g.kobuki.port.empty(), 12.5f);
    }

    // 3. Sensor calibration.
    y = 414;
    const double h3 = 110;
    P.card(x0, y, w, h3);
    stepTitle(P, x0 + 16, y + 14, 3, L"Sensör kalibrasyonu", g.real);
    if (!g.real) {
        P.text(L"Simülasyonda gerekmez (sanal sensör).", x0 + 18, y + 48, w - 36, 20, 12.5f, col::Faint);
        P.text(L"Gerçek robotta: Trig / Echo pinleri, anlık okuma ve 40 cm blokla kalibrasyon.", x0 + 18, y + 70, w - 36, 34, 11.5f,
               col::Faint, false, Align::Left, true, false);
    } else {
        const bool calOpen = g.panel == Panel::Cal;
        button(P, x0 + w - 16 - 96, y + 11, 96, 30, calOpen ? L"Kapat" : L"Aç  ›", calOpen || g.cal.valid ? Btn::Secondary : Btn::Primary,
               Act::CalOpen, 0, !locked && !g.arduino.port.empty() && !g.up.running, 12.5f);
        P.text(L"Pinler", x0 + 18, y + 48, 56, 22, 12.0f, col::Muted);
        std::wstring pins = L"Trig D" + std::to_wstring(g.trig) + L" · Echo D" + std::to_wstring(g.echo);
        if (!g.sens.cfgKnown) pins += (g.trig == DefaultTrig && g.echo == DefaultEcho) ? L"  (varsayılan)" : L"  (seçilen)";
        P.text(pins, x0 + 74, y + 48, 220, 22, 12.5f, col::Text, true);
        const sensor::Sample* last = lastSample();
        if (streaming() && last && last->echo())
            P.text(L"anlık " + fmt(L"%.1f cm", g.cal.valid ? g.cal.cm(rawCm(*last)) : rawCm(*last)), x0 + w - 18 - 120, y + 48, 120, 22, 12.0f,
                   col::Accent, true, Align::Right);
        if (g.cal.valid) {
            const bool stale = calStale() || calPinsChanged();
            P.text((stale ? L"! 40 cm = " : L"✓ 40 cm = ") + (g.cal.meanUs > 0 ? fmt(L"%.0f µs", g.cal.meanUs) : fmt(L"%.1f ham cm", g.cal.rawMeanCm)) +
                       fmt(L" · × %.4f · ", g.cal.scale) + (calPinsChanged() ? std::wstring(L"pinler değişti") : ageText(calAgeHours())),
                   x0 + 18, y + 74, w - 36, 22, 12.5f, stale ? col::Amber : col::Green, true);
        }
        else
            P.text(L"! Kalibre edilmedi: bloğu 40 cm'ye koyup ölçün", x0 + 18, y + 74, w - 36, 22, 12.5f, col::Amber, true);
    }

    // 4. Method and map.
    y = 536;
    const double h4 = 200;
    P.card(x0, y, w, h4);
    stepTitle(P, x0 + 16, y + 14, 4, L"Algoritma ve harita");
    const int pages = (static_cast<int>(g.maps.size()) + 3) / 4;
    g.mapPage = std::clamp(g.mapPage, 0, std::max(0, pages - 1));
    if (pages > 1) {   // more than four maps: page through them
        P.text(std::to_wstring(g.mapPage + 1) + L" / " + std::to_wstring(pages), x0 + w - 16 - 100, y + 14, 44, 24, 11.5f, col::Muted, false,
               Align::Center);
        button(P, x0 + w - 16 - 124, y + 12, 26, 28, L"‹", Btn::Secondary, Act::MapPage, -1, !locked && g.mapPage > 0, 14.0f);
        button(P, x0 + w - 16 - 26, y + 12, 26, 28, L"›", Btn::Secondary, Act::MapPage, 1, !locked && g.mapPage + 1 < pages, 14.0f);
    }
    const double mt = (w - 32 - 3 * 8) / 4.0;
    for (std::size_t i = 0; i < Methods.size(); ++i) {
        const double tx = x0 + 16 + static_cast<double>(i) * (mt + 8), ty = y + 46;
        const bool on = static_cast<int>(i) == g.method, ok = exeExists(static_cast<int>(i));
        const bool hv = hovered(Act::Method, static_cast<int>(i)) && !locked;
        const std::wstring& name = Methods[i].name;
        const std::size_t sp = name.find(L' ');
        P.round(tx, ty, mt, 52, 10, on ? col::AccentFaint : (hv ? col::Hover : col::Card), on ? col::Accent : col::Border, on ? 1.6 : 1.0);
        P.text(ok ? name.substr(0, sp) : L"derlenmemiş", tx, ty + 6, mt, 16, 10.5f, col::Muted, false, Align::Center);
        P.text(name.substr(sp + 1), tx, ty + 22, mt, 24, 13.5f, ok ? (on ? col::AccentDark : col::Text) : col::Faint, true, Align::Center);
        if (!locked && ok) hit(tx, ty, mt, 52, Act::Method, static_cast<int>(i));
    }
    for (std::size_t i = static_cast<std::size_t>(g.mapPage) * 4; i < g.maps.size() && i < static_cast<std::size_t>(g.mapPage) * 4 + 4; ++i) {
        const double tx = x0 + 16 + static_cast<double>(i % 4) * (mt + 8), ty = y + 108;
        const bool on = static_cast<int>(i) == g.map;
        const bool hv = hovered(Act::Map, static_cast<int>(i)) && !locked;
        P.round(tx, ty, mt, 80, 10, on ? col::AccentFaint : (hv ? col::Hover : col::Card), on ? col::Accent : col::Border, on ? 1.6 : 1.0);
        miniMap(P, g.maps[i].grid, tx + 6, ty + 6, mt - 12, 52);
        P.text(g.maps[i].name, tx, ty + 58, mt, 20, 12.0f, on ? col::AccentDark : col::Text, true, Align::Center);
        if (!locked) hit(tx, ty, mt, 80, Act::Map, static_cast<int>(i));
    }

    // Start / stop.
    if (g.run.running())
        button(P, x0, BH - 20 - 52, w, 52, g.stopping ? L"Durduruluyor..." : L"■   Durdur", Btn::Danger, Act::Stop, 0, !g.stopping, 15.0f);
    else
        button(P, x0, BH - 20 - 52, w, 52, g.real ? L"▶   Robotta başlat" : L"▶   Simülasyonu başlat", Btn::Primary, Act::Start, 0,
               !g.benchRunning && g.probing == 0 && !g.scanning && !g.calRun.active && !g.up.running && g.panel != Panel::Preflight, 15.0f);

    const Panel panel = g.real && !g.run.running() ? g.panel : Panel::None;
    if (panel == Panel::Cal) drawCalibration(P, 416, 80);
    else if (panel == Panel::Drive) drawDrive(P, 416, 80);
    else if (panel == Panel::Preflight) drawPreflight(P, 416, 80);
    else {
        const LiveView v = runView();
        drawLive(P, 416, 80, v);
        drawSide(P, 416 + 612, 188, v);
    }
}

// --- comparison tab ------------------------------------------------------
void drawCompareTab(Painter& P) {
    const double x0 = 20, w = 380;
    const bool idle = !g.benchRunning;
    double y = 80;
    P.card(x0, y, w, BH - 20 - y);
    P.text(L"Benchmark (simülasyon)", x0 + 18, y + 14, w - 36, 24, 15.0f, col::Text, true);
    P.text(L"Seçilen yöntemler × haritalar × seed 1..N pencere açmadan art arda koşar; her koşu results.csv'ye eklenir.",
           x0 + 18, y + 42, w - 36, 40, 12.0f, col::Muted, false, Align::Left, true, false);
    P.text(L"Yöntemler", x0 + 18, y + 90, w, 20, 12.5f, col::Muted, true);
    for (std::size_t i = 0; i < Methods.size(); ++i)
        chip(P, x0 + 16 + static_cast<double>(i % 2) * 178, y + 114 + static_cast<double>(i / 2) * 44, 170, 36, Methods[i].name,
             g.benchAlgo[i], Act::BenchAlgo, static_cast<int>(i), idle && exeExists(static_cast<int>(i)));
    P.text(L"Engel haritaları", x0 + 18, y + 210, w, 20, 12.5f, col::Muted, true);
    const double cw = (w - 32 - 3 * 8) / 4.0;
    for (std::size_t i = 0; i < g.maps.size() && i < static_cast<std::size_t>(MaxMaps); ++i)
        chip(P, x0 + 16 + static_cast<double>(i % 4) * (cw + 8), y + 234 + static_cast<double>(i / 4) * 42, cw, 36, g.maps[i].name, g.benchMap[i],
             Act::BenchMap, static_cast<int>(i), idle);
    if (g.maps.size() > 4) y += 42;   // a second row of maps pushes the rest down
    stepper(P, x0 + 18, y + 290, L"Tekrar (seed 1..N)", std::to_wstring(g.reps), Act::RepsMinus, Act::RepsPlus, idle);
    stepper(P, x0 + 18, y + 332, L"Başarı eşiği", L"%" + std::to_wstring(g.threshold), Act::ThrMinus, Act::ThrPlus, true);
    toggle(P, x0 + 18, y + 378, L"Gürültülü sensör", g.benchNoise, Act::BenchNoise, idle);
    int nAlgo = 0, nMap = 0;
    for (bool b : g.benchAlgo) nAlgo += b;
    for (bool b : g.benchMap) nMap += b;
    const int total = nAlgo * nMap * g.reps;
    if (g.benchRunning) {
        button(P, x0 + 16, y + 418, w - 32, 46, L"■   İptal", Btn::Danger, Act::BenchCancel, 0, true, 14.0f);
        const double frac = g.jobs.empty() ? 0.0 : static_cast<double>(g.benchIndex) / static_cast<double>(g.jobs.size());
        P.progress(x0 + 16, y + 476, w - 32, 8, frac, col::Accent);
        const std::wstring label = g.benchIndex < g.jobs.size() ? g.jobs[g.benchIndex].label : L"";
        P.text(std::to_wstring(std::min(g.benchIndex + 1, g.jobs.size())) + L" / " + std::to_wstring(g.jobs.size()) + L"   " + label,
               x0 + 18, y + 490, w - 36, 20, 12.0f, col::Muted);
    } else {
        button(P, x0 + 16, y + 418, w - 32, 46, L"▶   Benchmark başlat (" + std::to_wstring(total) + L" koşu)", Btn::Primary, Act::BenchRun, 0,
               total > 0 && !g.run.running(), 14.0f);
    }
    const double by = BH - 20 - 16 - 3 * 44 + 6;
    button(P, x0 + 16, by, w - 32, 38, L"Rapor kaydet (.md)", Btn::Secondary, Act::Report, 0, !g.rows.empty());
    button(P, x0 + 16, by + 44, w - 32, 38, L"Sonuç klasörü", Btn::Secondary, Act::OpenResults);
    button(P, x0 + 16, by + 88, w - 32, 38, L"Sonuçları arşivle", Btn::Secondary, Act::Archive, 0, !g.rows.empty() && idle);

    // Ranking card.
    const double rx = 416, rw = BW - 20 - rx;
    const results::Comparison c = results::compare(g.rows, filterMode(), g.threshold / 100.0, methodOrder());
    y = 80;
    const double h1 = 300;
    P.card(rx, y, rw, h1);
    P.text(L"Sıralama", rx + 20, y + 14, 300, 24, 15.0f, col::Text, true);
    P.text(L"önce başarılı harita sayısı, sonra başarı oranı, sonra hız", rx + 20, y + 38, 500, 18, 12.0f, col::Muted);
    segmented(P, rx + rw - 20 - 360, y + 14, 360, 38, { L"Simülasyon", L"Gerçek robot", L"Tümü" }, g.filter, Act::Filter);
    const double cx[] = { rx + 20, rx + 70, rx + 250, rx + 530, rx + 740, rx + 860 };
    const double hy = y + 70;
    P.text(L"SIRA", cx[0], hy, 50, 18, 11.0f, col::Faint, true);
    P.text(L"YÖNTEM", cx[1], hy, 170, 18, 11.0f, col::Faint, true);
    P.text(L"1) HIZ · ORT. SÜRE", cx[2], hy, 260, 18, 11.0f, col::Faint, true);
    P.text(L"2) BAŞARI ORANI", cx[3], hy, 200, 18, 11.0f, col::Faint, true);
    P.text(L"3) BAŞARILI HARİTA", cx[4], hy, 140, 18, 11.0f, col::Faint, true);
    P.text(L"KOŞU", cx[5], hy, 60, 18, 11.0f, col::Faint, true);
    double maxT = 1.0;
    for (const auto& s : c.methods) maxT = std::max(maxT, s.meanTime());
    double ry = hy + 26;
    if (c.methods.empty())
        P.text(L"Henüz koşu yok - soldan benchmark başlatın ya da Çalıştır sekmesinden koşun.", rx + 20, ry + 20, rw - 40, 24, 13.0f, col::Faint);
    for (const auto& s : c.methods) {
        if (ry > y + h1 - 40) break;
        P.rect(rx + 16, ry - 4, rw - 32, 1, col::Border);
        const bool first = s.rank == 1;
        P.circle(cx[0] + 14, ry + 18, 14, first ? col::Accent : col::Hover);
        P.text(std::to_wstring(s.rank), cx[0], ry + 4, 28, 28, 13.0f, first ? 0xFFFFFFFF : col::Muted, true, Align::Center);
        P.text(methodName(s.algorithm), cx[1], ry + 4, 170, 28, 14.0f, col::Text, true);
        if (s.completed) {
            P.text(fmt(L"%.0f s", s.meanTime()), cx[2], ry + 4, 70, 28, 14.0f, col::Text, true);
            P.progress(cx[2] + 74, ry + 15, 180, 8, s.meanTime() / maxT, first ? col::Green : 0xFF94A3B8);
        } else {
            P.text(L"tamamlanmadı", cx[2], ry + 4, 200, 28, 13.0f, col::Red);
        }
        if (s.scored) {
            P.text(fmt(L"%%%.1f", s.meanSuccess() * 100.0), cx[3], ry + 4, 70, 28, 14.0f, col::Text, true);
            P.progress(cx[3] + 74, ry + 15, 120, 8, s.meanSuccess(), s.meanSuccess() * 100.0 + 1e-9 >= g.threshold ? col::Green : col::Amber);
        } else {
            P.text(L"—", cx[3], ry + 4, 70, 28, 14.0f, col::Faint);
        }
        const bool all = s.mapsTried > 0 && s.mapsPassed == s.mapsTried;
        P.pill(std::to_wstring(s.mapsPassed) + L" / " + std::to_wstring(s.mapsTried), cx[4], ry + 6, 24, all ? col::Green : col::Amber,
               all ? col::GreenSoft : col::AmberSoft, 12.5f);
        P.text(std::to_wstring(s.completed) + L"/" + std::to_wstring(s.runs), cx[5], ry + 4, 60, 28, 13.0f, col::Muted);
        ry += 44;
    }

    // Method x map matrix.
    y = 80 + h1 + 12;
    const double h2 = BH - 20 - y;
    P.card(rx, y, rw, h2);
    P.text(L"Yöntem × engel haritası", rx + 20, y + 14, 400, 24, 15.0f, col::Text, true);
    P.text(L"ort. süre · ort. başarı oranı · eşiği geçen koşu", rx + 20, y + 38, 500, 18, 12.0f, col::Muted);
    const std::size_t nm = std::max<std::size_t>(1, c.maps.size());
    const double colW = (rw - 40 - 170) / static_cast<double>(nm);
    for (std::size_t i = 0; i < c.maps.size(); ++i)
        P.text(widen(c.maps[i]), rx + 20 + 170 + colW * static_cast<double>(i), y + 66, colW, 18, 12.0f, col::Faint, true, Align::Center);
    double my = y + 90;
    for (const auto& s : c.methods) {
        if (my > y + h2 - 50) break;
        P.text(methodName(s.algorithm), rx + 20, my, 170, 48, 14.0f, col::Text, true);
        for (std::size_t i = 0; i < c.maps.size(); ++i) {
            const double bx = rx + 20 + 170 + colW * static_cast<double>(i) + 4;
            const auto it = s.perMap.find(c.maps[i]);
            if (it == s.perMap.end()) { P.round(bx, my, colW - 8, 48, 9, col::Hover); continue; }
            const results::Cell& k = it->second;
            const bool pass = k.mapPassed();
            P.round(bx, my, colW - 8, 48, 9, pass ? col::GreenSoft : col::RedSoft);
            P.text((k.completed ? fmt(L"%.0f s", k.meanTime()) : std::wstring(L"bitmedi")) + L"  ·  " +
                       (k.scored ? fmt(L"%%%.0f", k.meanSuccess() * 100.0) : std::wstring(L"—")),
                   bx, my + 4, colW - 8, 22, 13.0f, col::Text, true, Align::Center);
            P.text(std::to_wstring(k.passed) + L"/" + std::to_wstring(k.runs) + (pass ? L"  ✓ başarılı" : L"  ✗"), bx, my + 24, colW - 8, 18,
                   11.5f, pass ? col::Green : col::Red, true, Align::Center);
        }
        my += 56;
    }
    if (!g.lastReport.empty())
        P.text(L"Son rapor: " + fs::path(g.lastReport).filename().wstring(), rx + 20, y + h2 - 28, rw - 40, 18, 11.5f, col::Muted);
}

#include "Tabs.h"   // history (replay) and map editor tabs

void paint(HDC dc) {
    RECT rc;
    GetClientRect(g.hwnd, &rc);
    if (!g.canvas || g.canvas->width() != rc.right || g.canvas->height() != rc.bottom)
        g.canvas = std::make_unique<view::Canvas>(std::max<int>(1, rc.right), std::max<int>(1, rc.bottom));
    {
        Painter P(g.canvas->dc(), g.scale);
        g.hits.clear();
        P.rect(0, 0, BW + 10, BH + 10, col::Bg);
        drawHeader(P);
        if (g.tab == 0) drawRunTab(P);
        else if (g.tab == 1) drawCompareTab(P);
        else if (g.tab == 2) drawHistoryTab(P);
        else drawMapsTab(P);
    }
    BitBlt(dc, 0, 0, rc.right, rc.bottom, g.canvas->dc(), 0, 0, SRCCOPY);
}

// ================================================================ input ====
void act(Act a, int arg, const Hit& h) {
    switch (a) {
        case Act::TabRun:
            g.tab = arg;
            if (g.tab == 1 || g.tab == 2) reloadResults();
            if (g.tab == 2) openHistory();
            if (g.tab == 3 && !g.draft.active) editSelect(g.map);
            if (g.tab != 2) { g.recPlaying = false; KillTimer(g.hwnd, TimerReplay); }
            break;
        case Act::ModeSim:
            g.real = arg == 1;
            if (!g.real) g.panel = Panel::None;
            if (g.real) {
                if (g.kobuki.port.empty() && g.arduino.port.empty()) scanPorts();
                else {
                    if (g.kobuki.r.kind == probe::Kind::Untested) probeRole(0);
                    if (g.arduino.r.kind == probe::Kind::Untested) probeRole(1);
                }
            }
            if (!g.run.running()) g.haveLive = false;
            saveSettings();
            break;
        case Act::Noise: g.noise = !g.noise; saveSettings(); break;
        case Act::Fast: g.fast = !g.fast; saveSettings(); break;
        case Act::KobukiPort: choosePort(0, h.x, h.y + h.h + 2); break;
        case Act::ArduinoPort: choosePort(1, h.x, h.y + h.h + 2); break;
        case Act::Test: probeRole(0); probeRole(1); break;
        case Act::Scan: scanPorts(); break;
        case Act::Method:
            g.method = arg;
            if (!g.run.running()) g.haveLive = false;
            saveSettings();
            break;
        case Act::Map:
            g.map = arg;
            if (!g.run.running()) g.haveLive = false;   // show the newly chosen reference map
            saveSettings();
            break;
        case Act::Start: startRun(); break;
        case Act::Stop: stopRun(); break;
        case Act::OpenPicture: {
            const fs::path png = fs::path(widen(g.live.saved)).replace_extension(".png");
            if (fs::exists(png)) ShellExecuteW(g.hwnd, L"open", png.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
            break;
        }
        case Act::BenchAlgo: g.benchAlgo[static_cast<std::size_t>(arg)] = !g.benchAlgo[static_cast<std::size_t>(arg)]; break;
        case Act::BenchMap: g.benchMap[static_cast<std::size_t>(arg)] = !g.benchMap[static_cast<std::size_t>(arg)]; break;
        case Act::RepsMinus: g.reps = std::max(1, g.reps - 1); break;
        case Act::RepsPlus: g.reps = std::min(20, g.reps + 1); break;
        case Act::ThrMinus: g.threshold = std::max(50, g.threshold - 5); break;
        case Act::ThrPlus: g.threshold = std::min(100, g.threshold + 5); break;
        case Act::BenchNoise: g.benchNoise = !g.benchNoise; break;
        case Act::BenchRun: startBenchmark(); break;
        case Act::BenchCancel: g.benchCancel = true; break;
        case Act::Report: saveReport(true); break;
        case Act::OpenResults: openFolder(g.dir / "results"); break;
        case Act::Archive: archiveResults(); break;
        case Act::Filter: g.filter = arg; break;
        case Act::CalOpen:
            g.panel = g.panel == Panel::Cal ? Panel::None : Panel::Cal;
            if (g.panel == Panel::Cal && !g.run.running()) g.haveLive = false;
            break;
        case Act::CalClose: g.panel = Panel::None; break;
        // pre-flight check
        case Act::PreGo: launchRun(); break;
        case Act::PreRetry: openPreflight(); break;
        case Act::PreCancel: g.panel = Panel::None; g.preChecking = false; break;
        // manual drive
        case Act::DriveOpen:
            g.panel = g.panel == Panel::Drive ? Panel::None : Panel::Drive;
            if (g.panel == Panel::Drive && !g.run.running()) g.haveLive = false;
            break;
        case Act::DriveBtn: g.driveBtn = arg; g.dragAct = Act::DriveBtn; break;
        case Act::DriveStop: g.driveKeys = g.driveBtn = 0; g.bot.halt(); break;
        case Act::DriveSpeed: g.driveSpeed = arg; saveSettings(); break;
        case Act::DriveMove: g.bot.move(arg / 100.0); break;
        case Act::DriveTurn: g.bot.turn(arg); break;
        case Act::DriveReset: g.bot.resetPose(); break;
        case Act::BumperReset: g.bot.clearBumperTest(); break;
        case Act::DriveRetry: g.bot.close(); break;   // syncLink() below opens it again
        // sketch upload, calibration check
        case Act::Upload: startUpload(cliPath().empty()); break;
        case Act::UploadCancel: g.up.cancel = true; break;
        case Act::UploadClose: g.up.show = false; break;
        case Act::Board: chooseBoard(h.x, h.y + h.h + 2); break;
        case Act::VerifyRef: g.verifyRef = arg == 0 ? 40 : 80; g.verify = {}; break;
        case Act::Verify: startVerify(); break;
        // history
        case Act::HistFilter: g.histFilter = arg; g.histScroll = 0; break;
        case Act::HistRow: histSelect(arg); break;
        case Act::HistPlay: histPlay(!g.recPlaying); break;
        case Act::HistSpeed: g.recSpeed = arg; break;
        case Act::HistSeek: histSeek(); g.dragAct = Act::HistSeek; break;
        case Act::HistPicture: histOpen(true); break;
        case Act::HistFolder: histOpen(false); break;
        // maps
        case Act::MapPage: g.mapPage += arg; break;
        case Act::EditSelect: editSelect(arg); break;
        case Act::EditCell: editCell(arg, true); g.dragAct = Act::EditCell; break;
        case Act::EditNew: editNew(); break;
        case Act::EditSave: editSave(); break;
        case Act::EditRevert: editRevert(); break;
        case Act::EditDelete: editDelete(); break;
        case Act::EditUse: editUse(); break;
        case Act::TrigPin: choosePin(true, h.x, h.y + h.h + 2); break;
        case Act::EchoPin: choosePin(false, h.x, h.y + h.h + 2); break;
        case Act::WritePins: writePins(); break;
        case Act::DefaultPins:
            g.trig = DefaultTrig;
            g.echo = DefaultEcho;
            g.pinsDirty = !g.sens.cfgKnown || g.trig != g.sens.trig || g.echo != g.sens.echo;
            g.pinMsg.clear();
            saveSettings();
            break;
        case Act::CalStart: startCalibration(); break;
        case Act::CalCancel: g.calRun.active = false; break;
        case Act::LinkRetry: g.link.close(); break;   // syncLink() below opens it again
        default: break;
    }
    syncLink();
    repaint();
}

const Hit* hitAt(int px, int py) {
    const double x = px / g.scale, y = py / g.scale;
    for (auto it = g.hits.rbegin(); it != g.hits.rend(); ++it)
        if (x >= it->x && x < it->x + it->w && y >= it->y && y < it->y + it->h) return &*it;
    return nullptr;
}

void takeScreenshot() {
    RECT rc;
    GetClientRect(g.hwnd, &rc);
    view::Canvas canvas(rc.right, rc.bottom);
    PrintWindow(g.hwnd, canvas.dc(), PW_CLIENTONLY | 0x2 /* PW_RENDERFULLCONTENT */);
    canvas.savePng(g.screenshot);
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_PAINT: {
            PAINTSTRUCT ps;
            const HDC dc = BeginPaint(hwnd, &ps);
            paint(dc);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;
        case WM_MOUSEMOVE: {
            if (!g.tracking) {
                TRACKMOUSEEVENT t{ sizeof(t), TME_LEAVE, hwnd, 0 };
                TrackMouseEvent(&t);
                g.tracking = true;
            }
            const Hit* h = hitAt(GET_X_LPARAM(lp), GET_Y_LPARAM(lp));
            const Act a = h ? h->act : Act::None;
            const int arg = h ? h->arg : -1;
            if ((wp & MK_LBUTTON) && g.dragAct != Act::None) {   // dragging: scrub the replay / paint map cells
                g.clickX = GET_X_LPARAM(lp) / g.scale;
                g.clickY = GET_Y_LPARAM(lp) / g.scale;
                if (g.dragAct == Act::HistSeek) { histSeek(); repaint(); }
                else if (g.dragAct == Act::EditCell && a == Act::EditCell) { editCell(arg, false); repaint(); }
            }
            if (a != g.hoverAct || arg != g.hoverArg) { g.hoverAct = a; g.hoverArg = arg; repaint(); }
            return 0;
        }
        case WM_MOUSELEAVE:
            g.tracking = false;
            if (g.hoverAct != Act::None) { g.hoverAct = Act::None; g.hoverArg = -1; repaint(); }
            return 0;
        case WM_SETCURSOR:
            if (LOWORD(lp) == HTCLIENT && g.hoverAct != Act::None) { SetCursor(LoadCursorW(nullptr, IDC_HAND)); return TRUE; }
            break;
        case WM_LBUTTONDOWN:
            g.clickX = GET_X_LPARAM(lp) / g.scale;
            g.clickY = GET_Y_LPARAM(lp) / g.scale;
            g.dragAct = Act::None;
            SetCapture(hwnd);
            if (const Hit* h = hitAt(GET_X_LPARAM(lp), GET_Y_LPARAM(lp))) {
                const Hit copy = *h;
                act(copy.act, copy.arg, copy);
            }
            return 0;
        case WM_LBUTTONUP:
            if (GetCapture() == hwnd) ReleaseCapture();
            if (g.dragAct == Act::DriveBtn) g.driveBtn = 0;
            g.dragAct = Act::None;
            g.paintValue = -1;
            repaint();
            return 0;
        case WM_CAPTURECHANGED:
            if (g.dragAct == Act::DriveBtn) g.driveBtn = 0;
            g.dragAct = Act::None;
            g.paintValue = -1;
            return 0;
        case WM_MOUSEWHEEL:
            if (g.tab == 2) {
                g.histScroll -= GET_WHEEL_DELTA_WPARAM(wp) / WHEEL_DELTA * 2;
                repaint();
            }
            return 0;
        case WM_KEYDOWN:
        case WM_KEYUP:
            if (g.tab == 0 && g.panel == Panel::Drive && g.bot.isOpen()) {
                int bit = 0;
                switch (wp) {
                    case VK_UP: case 'W': bit = KeyFwd; break;
                    case VK_DOWN: case 'S': bit = KeyBack; break;
                    case VK_LEFT: case 'A': bit = KeyLeft; break;
                    case VK_RIGHT: case 'D': bit = KeyRight; break;
                    case VK_SPACE: case VK_ESCAPE: g.driveKeys = g.driveBtn = 0; g.bot.halt(); break;
                    default: break;
                }
                if (msg == WM_KEYDOWN) g.driveKeys |= bit;
                else g.driveKeys &= ~bit;
                return 0;
            }
            if (g.tab == 2 && msg == WM_KEYDOWN && wp == VK_SPACE) { histPlay(!g.recPlaying); repaint(); return 0; }
            break;
        case WM_KILLFOCUS:
        case WM_ENTERMENULOOP:
            g.driveKeys = g.driveBtn = 0;   // never keep driving without the window
            break;
        case WmFrame:
            g.live = g.run.snapshot();
            repaint();
            return 0;
        case WmRunEnd:
            saveReplay();
            g.run.detach();
            g.stopping = false;
            KillTimer(hwnd, TimerKill);
            reloadResults();
            syncLink();
            repaint();
            return 0;
        case WmProbe: {
            std::unique_ptr<ProbeMsg> m(reinterpret_cast<ProbeMsg*>(lp));
            g.probing = std::max(0, g.probing - 1);
            PortStatus& ps = m->role == 0 ? g.kobuki : g.arduino;
            if (ps.port == m->port) ps.r = m->r;
            if (g.probing == 0) g.preChecking = false;
            if (g.probing == 0) maybeShowCalibration();
            syncLink();
            repaint();
            return 0;
        }
        case WmScanDone:
            onScanDone(reinterpret_cast<ScanMsg*>(lp));
            maybeShowCalibration();
            syncLink();
            repaint();
            return 0;
        case WmSensor:
            onSensor();
            return 0;
        case WmUpLine: {
            std::unique_ptr<std::wstring> line(reinterpret_cast<std::wstring*>(lp));
            g.up.log.push_back(*line);
            if (g.up.log.size() > 400) g.up.log.erase(g.up.log.begin(), g.up.log.begin() + 100);
            repaint();
            return 0;
        }
        case WmUpDone:
            if (g.up.th.joinable()) g.up.th.join();
            g.up.running = false;
            g.up.done = true;
            g.up.ok = wp == 0;
            if (g.up.ok && !g.up.setup) {   // the board now runs sensor.ino: forget the old answers, test the port again
                g.pinMsg.clear();
                g.pinsDirty = false;
                probeRole(1);
            }
            syncLink();
            repaint();
            return 0;
        case WmBenchStep:
            g.benchIndex = wp + (lp ? 1 : 0);
            if (lp) reloadResults();
            repaint();
            return 0;
        case WmBenchDone:
            if (g.bench.joinable()) g.bench.join();
            g.benchRunning = false;
            g.benchIndex = g.jobs.size();
            reloadResults();
            repaint();
            return 0;
        case WM_TIMER:
            if (wp == TimerKill) {
                KillTimer(hwnd, TimerKill);
                if (g.run.running() &&
                    MessageBoxW(hwnd, L"Program 10 s içinde kapanmadı. Zorla sonlandırılsın mı?\n(Gerçek robotta önce robotu elle durdurun.)",
                                L"Durdur", MB_ICONWARNING | MB_YESNO) == IDYES)
                    g.run.kill();
            } else if (wp == TimerStop) {
                KillTimer(hwnd, TimerStop);
                stopRun();
            } else if (wp == TimerCal) {
                if (streaming()) { KillTimer(hwnd, TimerCal); startCalibration(); repaint(); }
            } else if (wp == TimerVerify) {
                if (streaming() && !g.calRun.active) { KillTimer(hwnd, TimerVerify); startVerify(); repaint(); }
            } else if (wp == TimerDrive) {
                driveTick();
            } else if (wp == TimerReplay) {
                replayTick();
            } else if (wp == TimerStart) {
                KillTimer(hwnd, TimerStart);
                startRun();
            } else if (wp == TimerShot) {
                if (g.benchRunning) return 0;
                KillTimer(hwnd, TimerShot);
                takeScreenshot();
                PostMessageW(hwnd, WM_CLOSE, 0, 0);
            }
            return 0;
        case WM_CLOSE:
            if (g.run.running()) {
                if (g.screenshot.empty() &&
                    MessageBoxW(hwnd, L"Keşif hâlâ çalışıyor. Durdurulup kapatılsın mı?", L"Exploration Mapping", MB_ICONQUESTION | MB_OKCANCEL) != IDOK)
                    return 0;
                g.run.stop();
                for (int i = 0; i < 50 && g.run.running(); ++i) Sleep(100);   // let it stop the robot and save
                if (!g.run.running()) saveReplay();
                g.run.kill();
            }
            g.benchCancel = true;
            if (g.bench.joinable()) g.bench.join();
            g.up.cancel = true;
            if (g.up.th.joinable()) g.up.th.join();
            KillTimer(hwnd, TimerDrive);
            g.bot.close();
            g.link.close();
            saveSettings();
            DestroyWindow(hwnd);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            break;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

fs::path exeFolder() {
    wchar_t b[MAX_PATH * 4];
    GetModuleFileNameW(nullptr, b, MAX_PATH * 4);
    return fs::path(b).parent_path();
}

} // namespace

int WINAPI wWinMain(HINSTANCE inst, HINSTANCE, PWSTR, int show) {
    SetProcessDPIAware();
    g.inst = inst;
    g.bin = exeFolder();
#ifdef EXPLORE_DIR
    g.dir = fs::path(widen(EXPLORE_DIR)).make_preferred();
#else
    g.dir = g.bin;
#endif
    bool startReal = false, openCal = false, openDrive = false, openPre = false;
    std::wstring kobukiPort;
    int startTab = -1;
    std::wstring arduinoPort, calFile;
    int argc = 0;
    wchar_t** argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    for (int i = 1; i < argc; ++i) {
        const std::wstring a = argv[i];
        if (a == L"--screenshot" && i + 1 < argc) g.screenshot = argv[++i];
        else if (a == L"--delay" && i + 1 < argc) g.shotDelay = _wtof(argv[++i]);
        else if (a == L"--tab" && i + 1 < argc) startTab = _wtoi(argv[++i]);
        else if (a == L"--real") startReal = true;
        else if (a == L"--auto-start") g.autoStart = true;
        else if (a == L"--stop-after" && i + 1 < argc) g.stopAfter = _wtof(argv[++i]);
        else if (a == L"--report") g.reportOnly = true;
        else if (a == L"--cal") openCal = true;
        else if (a == L"--drive") openDrive = true;
        else if (a == L"--preflight") openPre = true;
        else if (a == L"--kobuki" && i + 1 < argc) kobukiPort = argv[++i];
        else if (a == L"--verify" && i + 1 < argc) g.hookVerify = _wtof(argv[++i]);
        else if (a == L"--arduino" && i + 1 < argc) arduinoPort = argv[++i];
        else if (a == L"--auto-cal" && i + 1 < argc) g.autoCal = _wtof(argv[++i]);
        else if (a == L"--cal-file" && i + 1 < argc) calFile = argv[++i];
        else if (a == L"--pins" && i + 1 < argc) swscanf_s(argv[++i], L"%d,%d", &g.hookTrig, &g.hookEcho);
    }
    LocalFree(argv);

    view::GdiPlusSession gdiPlus;
    loadMaps();
    loadSettings();
    reloadResults();
    if (startTab >= 0) g.tab = std::clamp(startTab, 0, 3);
    if (startReal) g.real = true;
    if (!arduinoPort.empty()) g.arduino.port = arduinoPort;
    if (!kobukiPort.empty()) g.kobuki.port = kobukiPort;
    g.calFile = calFile.empty() ? g.dir / "sensor_calibration.txt" : fs::path(calFile);
    rangecal::load(g.calFile, g.cal);
    if (openCal) { g.panel = Panel::Cal; g.calAutoShown = true; }
    if (openDrive) g.panel = Panel::Drive;
    if (g.reportOnly) return saveReport(false) ? 0 : 1;   // --report: results/comparison_<date>.md (simulator runs)

    // Scale: the screen's DPI, but never bigger than the work area allows.
    {
        const HDC sdc = GetDC(nullptr);
        g.scale = GetDeviceCaps(sdc, LOGPIXELSX) / 96.0;
        ReleaseDC(nullptr, sdc);
        RECT work{};
        if (SystemParametersInfoW(SPI_GETWORKAREA, 0, &work, 0))
            g.scale = std::min({ g.scale, (work.right - work.left - 16) / BW, (work.bottom - work.top - 48) / BH });
        g.scale = std::max(g.scale, 0.6);
    }

    WNDCLASSW wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = inst;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hIcon = LoadIconW(nullptr, IDI_APPLICATION);
    wc.lpszClassName = L"ExploreLauncher";
    RegisterClassW(&wc);
    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    RECT r{ 0, 0, static_cast<LONG>(BW * g.scale), static_cast<LONG>(BH * g.scale) };
    AdjustWindowRect(&r, style, FALSE);
    g.hwnd = CreateWindowExW(0, wc.lpszClassName, L"11 Exploration Mapping", style, CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left,
                             r.bottom - r.top, nullptr, nullptr, inst, nullptr);
    if (!g.hwnd) return 1;
    ShowWindow(g.hwnd, show);
    UpdateWindow(g.hwnd);
    if (g.real && g.screenshot.empty()) {   // re-check the saved ports
        if (!g.kobuki.port.empty()) probeRole(0);
        if (!g.arduino.port.empty()) probeRole(1);
    }
    if (g.tab == 2) openHistory();
    if (g.tab == 3) editSelect(g.map);
    if (openPre) openPreflight();
    syncLink();
    repaint();
    if (g.hookVerify > 0.0) SetTimer(g.hwnd, TimerVerify, static_cast<UINT>(g.hookVerify * 1000), nullptr);
    if (g.autoCal > 0.0) SetTimer(g.hwnd, TimerCal, static_cast<UINT>(g.autoCal * 1000), nullptr);
    if (g.autoStart) SetTimer(g.hwnd, TimerStart, 300, nullptr);
    if (g.stopAfter > 0.0) SetTimer(g.hwnd, TimerStop, static_cast<UINT>(g.stopAfter * 1000), nullptr);
    if (!g.screenshot.empty()) SetTimer(g.hwnd, TimerShot, static_cast<UINT>(g.shotDelay * 1000), nullptr);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    g.canvas.reset();
    return 0;
}
