// Ornek 06 - Gercek bir HC-SR04 mesafe sensoruyle "Follow the Gap" (Boslugu Takip Et) algoritmasi.
//
//   Kullanim: FollowTheGap.exe <Kobuki COM portu> <Arduino COM portu>
//   (ya da basitce: .\run.ps1 6  -> derler, her iki portu da bulur ve baslatir)
//
// Ne yapar:
//   1. Odometri sifir noktasindan (0,0) baslar ve hedefi (x y) kullanicidan sorar.
//   2. Arduino'daki HC-SR04'u surekli okur; en guncel mesafe tek satirda
//      canli gosterilir ve robotun karar verdigi deger tam olarak budur.
//   3. Hedefe dogru ilerler. Tam onde 0.70 m'den yakin VE hedeften once
//      olan bir sey = engel demektir: robot durur. (Hedefin arkasindaki bir
//      duvar tarama tetiklemez.)
//   4. Sensoru okuyarak yavasca SOLA doner - baslangic yonunden 90
//      dereceden fazla, hedef yonunden de 90 dereceden fazla sapmaz;
//      boylece asla geriye dogru yonelmez.
//   5. Robotun tum govdesinin (yaricap + guvenlik payi) bilinen her
//      noktadan temiz gectigi ILK yon bulunur bulunmaz tarama durur, o yone
//      donulur ve o yonde, hedefe giden duz cizginin de engeli temiz
//      gecmesine yetecek kadar ilerlenir (gecis sirasinda tekrar taranmaz).
//   6. Gorulen engel noktalari hafizaya (dunya koordinatlari) kaydedilir:
//      gecisten sonra hedefe giden yol hala bu engele yakinsa robot yeniden
//      taramak yerine ayni bosluk yonunde biraz daha ilerler; yeni bir
//      tarama gerekirse hafizadaki noktalarla baslar.
//   7. Solda uyan bosluk yoksa once ayni yerden SAGA tarar; ancak iki
//      tarafta da bosluk yoksa biraz geri gidip tekrar tarar.
//      Tampona (bumper) carpma: geri gider, nerede carptigini hatirlar ve tarar.
//
// Guvenlik: ucurum (cliff) algilanmasi programi durdurur (Motion.h icindeki guard()),
// sensor verisinin kaybolmasi, cok fazla tarama yapilmasi, zaman asimi ve Ctrl+C de ayni sekilde durdurur.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ArduinoRangeSensor.h"
#include "FollowTheGap.h"
#include "Motion.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace kobuki::motion;

namespace {

// --------------------------------------------------------- ayar sabitleri --
constexpr double TriggerDistance = 0.70;        // m - sensor onde bu kadar yakin okursa = engel var demektir (erken fark etmek icin)
constexpr double PassStopDistance = 0.40;       // m - bosluktan gecerken sadece bu kadar yakin (ve yolu kapatan) engelde durulur
constexpr double GoalTolerance   = 0.03;        // m - hedefe sadece bu kadar yaklasinca "ulasildi" sayilir
constexpr double ApproachGain    = 0.6;         // 1/s - son yaklasma: hiz <= kazanc * hedefe kalan mesafe
constexpr double MinApproachV    = 0.03;        // m/s - ... ama bundan daha yavas olmaz
constexpr int    TriggerReadings = 2;           // ardisik kac yakin okuma gerektigi (hatali tekil olcumleri eler)
constexpr double BodyCheckRange  = 1.00;        // m - bu mesafeye kadar olan sensor okumalari robotun govdesine karsi kontrol edilir
constexpr double SensorOffset    = 0.15;        // m - sensor, robotun merkezinin bu kadar onunde duruyor
constexpr double AssumedDepth    = 0.35;        // m - engelin, sensorun gordugu yuzeyin arkasinda bu kadar derin oldugu varsayilir
constexpr double ScanMaxAngle    = 90.0 * Deg;  // rad - bir tarafa bundan daha fazla taranmaz (ya da yonelinmez)
constexpr double ScanSpeed       = 0.35;        // rad/s - iki sensor okumasi arasinda ~2 derecelik donus
constexpr double BumpBackUp      = 0.12;        // m - tampona carptiktan sonra geri gidilecek mesafe
constexpr double NoGapBackUp     = 0.15;        // m - 90 derece icinde uyan bosluk yoksa geri gidilecek mesafe
constexpr int    MaxNoGapBackUps = 2;           // geri gitme korudur (arkada sensor/ucurum kontrolu yok) - ust uste bu kadar sonra dur
constexpr int    MaxScans        = 8;           // bu kadar tarama sonrasi vazgecilir
constexpr double ExtensionStep   = 0.25;        // m - gecisten sonra yol hala engele yakinsa bosluk yonunde ek ilerleme
constexpr int    MaxExtensions   = 3;           // en fazla bu kadar ek ilerleme
constexpr std::size_t MaxMemPoints = 300;       // hafizada tutulacak en fazla engel noktasi
constexpr double SensorTimeoutS  = 1.5;         // s - bu kadar sure Arduino'dan veri gelmezse -> dur
constexpr double HudPeriodS      = 0.2;         // s - canli durum satirinin yenilenme periyodu

std::atomic<bool> g_abort{false};  // Ctrl+C basilinca true olur; tum dongulerde kontrol edilir
BOOL WINAPI onCtrlC(DWORD type) {
    // Konsol kapatma olaylarini (Ctrl+C dahil) yakalayan Windows geri cagirim fonksiyonu.
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT || type == CTRL_CLOSE_EVENT) {
        g_abort = true;
        return TRUE;
    }
    return FALSE;
}

// Verilen zaman noktasindan bu yana gecen saniyeyi dondurur (zaman asimi kontrolleri icin).
double secondsSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

// --------------------------------------------------- canli terminal arayuzu --
// Tum calisma boyunca sensor okumasini bu sinif yonetir: her kontrol dongusu
// tick() cagirir; bu fonksiyon Arduino'yu okur, en guncel degeri durum
// satirinda gosterir, kat edilen yolu tutar ve Ctrl+C / sensor-kaybi
// durdurmalarini uygular.
class Live {
public:
    // Kurucu: robot, sensor ve hedef (x,y) referanslarini saklar; baslangic
    // konumunu yol uzunlugu hesabi icin kaydeder.
    Live(kobuki::Kobuki& robot, ftg::ArduinoRangeSensor& sensor, double goalX, double goalY)
        : robot_(robot), sensor_(sensor), goalX_(goalX), goalY_(goalY), closest_(sensor.maxRangeMeters()) {
        const auto o = robot.getOdometryData();
        prevX_ = o.x;
        prevY_ = o.y;
    }

    double range() const { return sensor_.lastRangeMeters(); }       // en son olculen mesafe (m)
    double pathLength() const { return path_; }                     // su ana kadar katedilen toplam yol (m)
    double closestSeen() const { return closest_; }                 // gorulen en yakin mesafe (m)

    // Son cagridan bu yana gercekten yeni bir sensor okumasi geldiyse true dondurur.
    bool tick(const char* mode) {
        if (g_abort) {
            // Kullanici Ctrl+C ile iptal etti: robotu durdur, mesaj goster ve cik.
            stop(robot_);
            event("Aborted by user - robot stopped.");
            std::exit(1);
        }
        sensor_.poll();  // Arduino'dan gelen yeni baytlari oku
        const bool fresh = sensor_.readingCount() != lastCount_;  // yeni bir tam okuma geldi mi?
        if (fresh) {
            lastCount_ = sensor_.readingCount();
            lastData_ = std::chrono::steady_clock::now();
            closest_ = (std::min)(closest_, range());  // en yakin okumayi guncelle
        } else if (secondsSince(lastData_) > SensorTimeoutS) {
            // Uzun suredir hic veri gelmedi: sensor baglantisi kopmus olabilir, guvenli dur.
            stop(robot_);
            event("!! No data from the distance sensor - robot stopped.");
            std::exit(1);
        }
        const auto o = robot_.getOdometryData();
        path_ += std::hypot(o.x - prevX_, o.y - prevY_);  // kat edilen mesafeyi topla
        prevX_ = o.x;
        prevY_ = o.y;
        if (secondsSince(lastPrint_) >= HudPeriodS) {
            print(mode);  // durum satirini belirli araliklarla yenile
            lastPrint_ = std::chrono::steady_clock::now();
        }
        return fresh;
    }

    // Durum satirinin uzerine yazilmayan, tek seferlik bir mesaj basar (ornek: "Engel bulundu").
    void event(const std::string& msg) {
        std::cout << '\r' << std::string(96, ' ') << '\r' << msg << '\n' << std::flush;
    }

private:
    // Tek satirlik canli durum bilgisini (mod, mesafe, konum, yonelim, hedefe kalan mesafe) yazdirir.
    void print(const char* mode) {
        const auto o = robot_.getOdometryData();
        std::ostringstream s;
        s << std::fixed << std::setprecision(2);
        s << '[' << std::left << std::setw(5) << mode << "] sensor ";
        if (range() >= sensor_.maxRangeMeters() - 1e-6) s << '>' << sensor_.maxRangeMeters();
        else s << range();
        s << " m | pos (" << o.x << ", " << o.y << ") | heading "
          << std::setprecision(0) << robot_.getHeading() / Deg << " deg | to goal "
          << std::setprecision(2) << std::hypot(goalX_ - o.x, goalY_ - o.y) << " m";
        std::string line = s.str();
        if (line.size() < 96) line.resize(96, ' ');  // onceki (daha uzun) satiri tamamen silmek icin bosluklarla doldur
        std::cout << '\r' << line << std::flush;
    }

    kobuki::Kobuki& robot_;
    ftg::ArduinoRangeSensor& sensor_;
    double goalX_, goalY_;                 // hedef koordinatlari
    double prevX_ = 0.0, prevY_ = 0.0, path_ = 0.0, closest_;  // yol hesabi ve en yakin okuma icin durum
    std::size_t lastCount_ = 0;
    std::chrono::steady_clock::time_point lastData_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastPrint_ = std::chrono::steady_clock::now();
};

// -------------------------------------------------------------- hareketler --
// Mutlak bir yone donene kadar yerinde doner (Kobuki yuvarlak oldugu icin
// yerinde donmek yeni bir seye carpmaz - burada basili bir tampon yok sayilir).
void turnToLive(kobuki::Kobuki& robot, Live& live, double headingRad) {
    const double tol = cfg.turn_tol_deg * Deg;
    while (true) {
        guard(robot, true);
        live.tick("TURN");
        const double error = normalize(headingRad - robot.getHeading());
        if (std::abs(error) < tol) break;  // hedef yone yeterince yaklasildi, dur
        double w = std::clamp(2.0 * error, -cfg.turn_speed, cfg.turn_speed);
        if (std::abs(w) < cfg.min_turn) w = (w < 0 ? -cfg.min_turn : cfg.min_turn);  // motor takilmasin diye asgari hiz
        robot.setBaseControl(0.0, w);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

// Belirtilen mesafe kadar geri gider (sensor/HUD'u canli tutarak).
void backUpLive(kobuki::Kobuki& robot, Live& live, double distance) {
    const auto start = robot.getOdometryData();
    while (true) {
        guard(robot, true);
        live.tick("BACK");
        const auto o = robot.getOdometryData();
        if (std::hypot(o.x - start.x, o.y - start.y) >= distance) break;  // istenen mesafe kadar geri gidildi
        robot.setBaseControl(-0.10, 0.0);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
}

enum class PassEnd { Done, Goal, Obstacle, Bumped };  // Gecisin nasil bittigi: bitti/hedefe ulasildi/engel/carpma

// `headingRad` yonunde, en fazla `distance` metre kadar duz ilerler. Bu,
// bulunan bosluga "bagli" bir gecistir: sadece kalan gecis yolunu gercekten
// kapatan yakin bir engelde durur; uzaktaki seyler ya da bosluk kenarindan
// gelen yansimalar gecisi bolmez.
PassEnd driveAlong(kobuki::Kobuki& robot, Live& live, double headingRad, double distance,
                   double goalX, double goalY, double goalTol, double speed, double clearance, Bump& bump) {
    const auto start = robot.getOdometryData();
    PassEnd end = PassEnd::Done;
    while (true) {
        guard(robot, true);
        bump = bumper(robot);
        if (bump != Bump::None) { end = PassEnd::Bumped; break; }  // tampona carpildi, hemen dur
        live.tick("PASS");
        const auto o = robot.getOdometryData();
        if (std::hypot(goalX - o.x, goalY - o.y) < goalTol) { end = PassEnd::Goal; break; }  // hedefe ulasildi
        const double done = std::hypot(o.x - start.x, o.y - start.y);
        if (done >= distance) break;  // istenen mesafe kadar gidildi
        // Engel yakin mi ve kalan gecis yolunun icinde mi? (merkezden uzaklik < kalan yol + govde payi)
        if (live.range() <= PassStopDistance && live.range() + SensorOffset < distance - done + clearance) {
            end = PassEnd::Obstacle;
            break;
        }
        const double correction = cfg.heading_gain * normalize(headingRad - robot.getHeading());  // dogrusal hizi korumak icin yon duzeltmesi
        robot.setBaseControl(speed, correction);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
    return end;
}

// -------------------------------------------------------- bosluk geometrisi --
// Taranan bir nokta; tarama basladigindaki yone gore acisi (rad, + = sol)
// ve robotun MERKEZINE olan uzakligi (m) ile tutulur.
struct ScanPoint { double angle, range; };

// Robot `th` yonunde giderken, olculen her nokta yoldan en az `clearance`
// kadar uzakta mi? (R uzakligindaki bir nokta icin |aci - th| >= asin(c / R) gerekir.)
bool bodyFits(const std::vector<ScanPoint>& pts, double th, double clearance) {
    for (const auto& p : pts) {
        if (std::abs(p.angle - th) < std::asin((std::min)(1.0, clearance / p.range))) return false;  // bu nokta yolu kesiyor
    }
    return true;
}

// (px,py) noktasindan (ax,ay)-(bx,by) dogru parcasina olan en kisa mesafe.
double distToSegment(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    const double t = len2 > 0.0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

// `th` yonunde ne kadar ilerlenmeli ki, oradan hedefe giden duz cizgi de
// robotun govdesini, etrafindan dolasilan engelden (sola gecerken sagdaki,
// saga gecerken soldaki noktalardan, her biri AssumedDepth kadar derin
// varsayilarak) temiz tutsun. dir: +1 = bosluk solda, -1 = bosluk sagda.
double passDistanceFor(const std::vector<ScanPoint>& pts, double th, double goalBearing,
                       double goalDist, double clearance, int dir) {
    const double gx = goalDist * std::cos(goalBearing), gy = goalDist * std::sin(goalBearing);
    for (double d = 0.3; d < 1.2 + 1e-9; d += 0.05) {  // 0.3 m'den 1.2 m'ye kadar artan mesafeleri dene
        const double ax = d * std::cos(th), ay = d * std::sin(th);
        bool clear = true;
        for (const auto& p : pts) {
            if (dir * (p.angle - th) >= 0.0) continue; // yolun bosluk tarafinda - robot bu noktanin etrafindan dolanmiyor
            for (const double extra : {0.0, AssumedDepth}) {
                const double r = p.range + extra;
                if (distToSegment(r * std::cos(p.angle), r * std::sin(p.angle), ax, ay, gx, gy) < clearance)
                    clear = false;  // bu mesafe guvenli degil, bir sonrakini dene
            }
        }
        if (clear) return d;  // ilk guvenli mesafeyi bulundu
    }
    return 1.2;  // hicbiri tam guvenli degilse en uzun denenen mesafeyi kullan
}

// ------------------------------------------------------- engel hafizasi --
// Taramalarda gorulen engel noktalari dunya (odometri) koordinatlarinda
// saklanir; boylece ayni engel tekrar tekrar taranmaz: yeni bir tarama bu
// noktalarla baslar, gecisten sonra hedefe giden yol bunlara gore kontrol edilir.
struct MemPoint { double x, y, deepX, deepY; };  // gorulen yuzey noktasi + AssumedDepth kadar arkasi

// `ref` yonune gore olculen noktalari (robot merkezi sx,sy iken) hafizaya ekler.
void remember(std::vector<MemPoint>& mem, double sx, double sy, double ref, const std::vector<ScanPoint>& pts) {
    for (const auto& p : pts) {
        const double a = ref + p.angle;
        const double x = sx + p.range * std::cos(a), y = sy + p.range * std::sin(a);
        const bool known = std::any_of(mem.begin(), mem.end(),
                                       [&](const MemPoint& m) { return std::hypot(m.x - x, m.y - y) < 0.03; });
        if (known) continue;  // bu nokta zaten hafizada
        const double deep = p.range + AssumedDepth;
        mem.push_back({x, y, sx + deep * std::cos(a), sy + deep * std::sin(a)});
    }
    if (mem.size() > MaxMemPoints) mem.erase(mem.begin(), mem.end() - MaxMemPoints);  // en eskileri at
}

// Hafizadaki, robota yakin (BodyCheckRange + sensor kaymasi icindeki) noktalari
// `ref` yonune gore tarama noktalarina cevirir. Robotun govdesi icine dusen
// noktalar (odometri kaymasindan kalma, gecersiz) atlanir.
std::vector<ScanPoint> recall(const std::vector<MemPoint>& mem, double cx, double cy, double ref, double minRange) {
    std::vector<ScanPoint> pts;
    for (const auto& m : mem) {
        const double r = std::hypot(m.x - cx, m.y - cy);
        if (r < minRange || r > BodyCheckRange + SensorOffset) continue;
        pts.push_back({normalize(std::atan2(m.y - cy, m.x - cx) - ref), r});
    }
    return pts;
}

// (px,py)'den hedefe (gx,gy) giden duz yol, hafizadaki her engel noktasindan
// (ve varsayilan derinliginden) en az `clearance` uzakta mi?
bool lineClear(const std::vector<MemPoint>& mem, double px, double py, double gx, double gy, double clearance) {
    for (const auto& m : mem) {
        if (distToSegment(m.x, m.y, px, py, gx, gy) < clearance ||
            distToSegment(m.deepX, m.deepY, px, py, gx, gy) < clearance)
            return false;
    }
    return true;
}

// `ref` yonunden `dir` tarafina (+1 = sol, -1 = sag) yavasca doner, sensoru
// okur ve robotun govdesinin bilinen her noktadan (hafiza + yeni olcumler)
// temiz gectigi ILK yonu bulur bulmaz durur - en kucuk donusle gecilebilen
// bosluk. Bir yon sadece, sensorun o yonun biraz otesine (`look` kadar) de
// bakmis olmasi durumunda kabul edilir (sonarin ~15 derecelik huzmesi
// nedeniyle), boylece gorulmemis hicbir sey yolun yaninda kalmaz.
// ScanMaxAngle'dan fazla, ya da hedeften 90 dereceden fazla donulmez.
// Yeni olcumler `points`'e eklenir (cagiran bunlari hafizaya kaydeder).
// Donen deger: bulunan yonun `ref`'e gore acisi (rad, + = sol).
std::optional<double> scanForGap(kobuki::Kobuki& robot, Live& live, double ref, double goalBearing,
                                 double clearance, std::vector<ScanPoint>& points, int dir) {
    const double limit = (std::min)(ScanMaxAngle, 90.0 * Deg + dir * goalBearing);
    const double look = std::asin((std::min)(1.0, clearance / (BodyCheckRange + SensorOffset)));
    const double tol = cfg.turn_tol_deg * Deg;

    while (true) {
        guard(robot, true);
        const bool fresh = live.tick("SCAN");
        const double ang = normalize(robot.getHeading() - ref);  // ref'e gore isaretli aci
        const double phi = (std::max)(0.0, dir * ang);           // `dir` tarafina su ana kadar donulen aci

        if (fresh) {
            if (live.range() <= BodyCheckRange) points.push_back({ang, live.range() + SensorOffset});  // yeni olcumu ekle

            // Taranan alanda, ref'e en yakin yonden baslayarak ilk uyan yonu ara.
            for (double m = look; m <= phi - look + 1e-9; m += 1.0 * Deg) {
                if (bodyFits(points, dir * m, clearance)) {
                    robot.setBaseControl(0.0, 0.0);
                    return dir * m;  // ilk uyan bosluk bulundu, tarama biter
                }
            }
        }
        if (phi >= limit - tol) break;  // izin verilen en fazla donus acisina ulasildi
        robot.setBaseControl(0.0, dir * ScanSpeed);
        wait();
    }
    robot.setBaseControl(0.0, 0.0);
    return std::nullopt;  // bu tarafta uyan bir bosluk bulunamadi
}

// Tampona carpildi: geri git ve engelin kabaca nerede oldugunu dondur,
// boylece ultrasonik sensor onu goremese bile bir sonraki tarama bu
// noktadan kacinir.
ScanPoint backOffBump(kobuki::Kobuki& robot, Live& live, Bump b, double robotRadius) {
    robot.setBaseControl(0.0, 0.0);
    const char* side = b == Bump::Left ? "left" : b == Bump::Right ? "right" : "front";
    live.event(std::string("Bumper hit (") + side + ") - backing up and scanning.");
    backUpLive(robot, live, BumpBackUp);
    const double angle = b == Bump::Left ? 45.0 * Deg : b == Bump::Right ? -45.0 * Deg : 0.0;  // carpilan tarafa gore tahmini engel yonu
    return {angle, robotRadius + BumpBackUp + 0.02};
}

// ----------------------------------------------------------- ana gorev --
bool driveWithFollowTheGap(kobuki::Kobuki& robot, Live& live, double goalX, double goalY,
                           const ftg::FollowTheGapParams& p, double timeoutSeconds) {
    const auto startTime = std::chrono::steady_clock::now();
    const double clearance = p.robot_radius + p.safety_margin;  // robotun govdesi + guvenlik payi
    int closeReadings = 0; // TriggerDistance'dan yakin ardisik taze okuma sayisi
    int scans = 0;              // toplam yapilan tarama sayisi
    int noGapBackUps = 0; // hicbir bosluk bulamayan ardisik tarama sayisi
    bool scanNow = false;       // bir sonraki dongude hemen tarama yapilsin mi
    std::vector<ScanPoint> bumpPoints;  // tampon carpmasindan gelen tahmini engel noktalari
    std::vector<MemPoint> memory;       // gorulen engel noktalari (dunya koordinatlari)

    while (true) {
        guard(robot, true); // ucurum programi durdurur; tampon asagida ayrica ele alinir
        if (const Bump b = bumper(robot); b != Bump::None) {
            bumpPoints = {backOffBump(robot, live, b, p.robot_radius)};  // carpma oldu: geri git, taramayi tetikle
            scanNow = true;
        }
        const bool fresh = live.tick("DRIVE");

        const auto o = robot.getOdometryData();
        const double goalDist = std::hypot(goalX - o.x, goalY - o.y);
        if (goalDist < GoalTolerance) return true;  // hedefe ulasildi
        if (secondsSince(startTime) > timeoutSeconds) {
            live.event("Timeout - stopping.");  // zaman asimi - dur
            return false;
        }

        const double goalDir = std::atan2(goalY - o.y, goalX - o.x);
        const double bearing = normalize(goalDir - robot.getHeading());
        // Sadece hedefe giden yoldaki bir engel onemlidir - robot hedefe
        // donerken sensorun yanindan "gecip giden" bir sey degil.
        const bool facingGoal = std::abs(bearing) < p.slow_turn_rad;
        // Engel ancak hedeften ONCE ise onemlidir: hedefe yaklasirken gorulen,
        // hedefin arkasindaki bir duvar tarama tetiklemez.
        const bool beforeGoal = live.range() + SensorOffset < goalDist + p.robot_radius;
        if (fresh)
            closeReadings = (facingGoal && beforeGoal && live.range() <= TriggerDistance) ? closeReadings + 1 : 0;

        if (!scanNow && closeReadings < TriggerReadings) {
            const double w = std::clamp(p.turn_gain * bearing, -p.max_turn, p.max_turn);
            const double slow = std::clamp(std::abs(bearing) / p.slow_turn_rad, 0.0, 1.0);
            // Son yaklasmada yavaslar, boylece robot hedefin TAM UZERINDE
            // durur, hedefi gecip gitmez.
            const double vMax = (std::min)(p.max_speed, (std::max)(MinApproachV, ApproachGain * goalDist));
            const double v = closeReadings > 0 ? 0.0 : vMax * (1.0 - slow);
            robot.setBaseControl(v, w);
            wait();
            continue;
        }

        // ---- engel: dur, once SOLA, bulunamazsa SAGA tara; ilk uyan bosluga git ----
        robot.setBaseControl(0.0, 0.0);
        closeReadings = 0;
        const bool rescan = scanNow;  // bu bir tampon/geri-gitme sonrasi yeniden tarama mi?
        scanNow = false;
        if (++scans > MaxScans) {
            live.event("Too many scans without reaching the goal - stopping.");  // cok fazla tarama yapildi, hedefe ulasilamadi - dur
            return false;
        }
        const double scanStart = robot.getHeading();
        const double goalBearing = normalize(goalDir - scanStart);

        // Taramaya hafizadaki noktalar + (varsa) tampon noktasiyla basla.
        std::vector<ScanPoint> points = recall(memory, o.x, o.y, scanStart, p.robot_radius);
        points.insert(points.end(), bumpPoints.begin(), bumpPoints.end());
        bumpPoints.clear();

        std::ostringstream msg;
        if (rescan) msg << "Scanning left again for a gap the robot fits through...";
        else msg << std::fixed << std::setprecision(2) << "Obstacle " << live.range()
                 << " m ahead - scanning left for a gap the robot fits through...";
        live.event(msg.str());

        int dir = +1;  // +1 = bosluk solda, -1 = bosluk sagda
        std::optional<double> gapOffset = scanForGap(robot, live, scanStart, goalBearing, clearance, points, dir);
        if (!gapOffset) {
            // Solda yok: geri gitmeden once ayni yerden SAG tarafi tara.
            live.event("No gap on the left - scanning right...");
            dir = -1;
            turnToLive(robot, live, scanStart);
            gapOffset = scanForGap(robot, live, scanStart, goalBearing, clearance, points, dir);
        }
        remember(memory, o.x, o.y, scanStart, points);  // gorulen her seyi hafizaya kaydet

        if (!gapOffset) {
            if (++noGapBackUps > MaxNoGapBackUps) {
                live.event("No gap the robot fits through on either side - stopping.");  // iki tarafta da uyan bosluk yok - dur
                return false;
            }
            live.event("No gap fits on either side - backing up a little to scan again.");  // biraz geri gidip tekrar tara
            turnToLive(robot, live, scanStart);
            backUpLive(robot, live, NoGapBackUp);
            scanNow = true;
            continue;
        }
        noGapBackUps = 0;  // bosluk bulundu, sayaci sifirla

        const double gapHeading = normalize(scanStart + *gapOffset);  // bosluk yonu (mutlak) - gecis boyunca bu yonde kalinir
        const double pass = passDistanceFor(points, *gapOffset, goalBearing, goalDist, clearance, dir);
        std::ostringstream found;
        found << std::fixed << std::setprecision(0) << "Gap fits at " << std::abs(*gapOffset) / Deg
              << " deg " << (dir > 0 ? "left" : "right") << " (goal at " << goalBearing / Deg
              << " deg) - driving through for " << std::setprecision(2) << pass << " m.";
        live.event(found.str());

        turnToLive(robot, live, gapHeading);  // bulunan bosluk yonune don
        Bump b = Bump::None;
        PassEnd end = driveAlong(robot, live, gapHeading, pass, goalX, goalY, GoalTolerance, p.max_speed, clearance, b);
        // Gecis bitti ama hedefe giden duz yol hala hafizadaki engele cok yakin:
        // yeniden taramak yerine ayni bosluk yonunde biraz daha ilerle.
        for (int i = 0; i < MaxExtensions && end == PassEnd::Done; ++i) {
            const auto q = robot.getOdometryData();
            if (lineClear(memory, q.x, q.y, goalX, goalY, clearance)) break;
            live.event("Path to the goal still passes the remembered obstacle - continuing through the gap.");
            end = driveAlong(robot, live, gapHeading, ExtensionStep, goalX, goalY, GoalTolerance, p.max_speed, clearance, b);
        }
        if (end == PassEnd::Bumped) {
            bumpPoints = {backOffBump(robot, live, b, p.robot_radius)};  // gecis sirasinda carpildi, yeniden tara
            scanNow = true;
        }
        // Done / Obstacle: dongu tekrar hedefe doner, engellenirse yeniden tetiklenir.
    }
}

// "2 0", "2, 0", "2 , 0" ve ondalik virgul ("0,5 1") kabul edilir; tek parca
// "2,0" belirsiz oldugu icin reddedilir (2.0 mi, yoksa 2 ve 0 mi?).
bool parseTarget(const std::string& line, double& x, double& y) {
    std::istringstream iss(line);
    std::vector<double> vals;
    std::string tok;
    while (iss >> tok) {
        while (!tok.empty() && tok.back() == ',') tok.pop_back();  // ayirici virgul
        if (tok.empty()) continue;
        std::replace(tok.begin(), tok.end(), ',', '.');  // ondalik virgul
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

// Hedefi kullanicidan ister; gecersiz girdide tekrar sorar, girdi akisi kapaliysa duzgunce cikar.
void readTarget(double& x, double& y) {
    while (true) {
        std::cout << "Target coordinates (x y) in metres, e.g. \"1 0.5\": " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) {
            std::cerr << "\nNo input - exiting.\n";
            std::exit(1);
        }
        if (parseTarget(line, x, y)) return;
        std::cout << "  Please enter two numbers, e.g.: 1 0.5\n";
    }
}

// Arduino'yu acar ve ilk okumayi bekler (port acildiginda Uno sifirlanir,
// bu yuzden birkac saniye surer). Basarisiz olursa ipucuyla birlikte cikar.
void openSensor(ftg::ArduinoRangeSensor& sensor, const char* port) {
    try {
        std::cout << "Opening distance sensor on " << port << "..." << std::flush;
        sensor.open(port);
    } catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << "\n";
        std::exit(1);
    }
    const auto t0 = std::chrono::steady_clock::now();
    while (sensor.readingCount() == 0) {
        sensor.poll();
        if (secondsSince(t0) > 6.0) {
            std::cerr << "\nERROR: the Arduino on " << port << " sends no data. Is the sensor sketch uploaded and "
                         "the front sensor wired (TRIG=11, ECHO=12)?\n";
            std::exit(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << " OK (" << std::fixed << std::setprecision(2) << sensor.lastRangeMeters() << " m)\n";
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "=== Follow the Gap - Kobuki + HC-SR04 ===\n"
              << "The robot starts at (0,0), drives to your target and, when it sees an obstacle,\n"
              << "scans to the left and passes through a gap that fits its body. Ctrl+C stops it.\n\n";

    if (argc < 3) {
        std::cerr << "Usage: <program> <Kobuki COM port> <Arduino COM port>   (example: COM6 COM9)\n"
                     "       or run  .\\run.ps1 6  to detect the ports automatically.\n";
        return 1;
    }
    SetConsoleCtrlHandler(onCtrlC, TRUE);  // Ctrl+C yakalayicisini kaydet

    // 1. Once sensor (robota henuz dokunulmadi), sonra hedef.
    ftg::ArduinoRangeSensor sensor;
    openSensor(sensor, argv[2]);

    double x1, y1;
    readTarget(x1, y1);

    // 2. Kobuki'ye baglan; odometri burada sifirlanir, yani bu nokta tam olarak (0,0)'dir.
    kobuki::Kobuki robot;
    connect(robot, argc, argv, "");

    // ---- ince ayar: gercek robota gore bu degerlerle oynayin ----
    ftg::FollowTheGapParams params;
    params.robot_radius  = 0.18;  // m     - Kobuki'nin capi 35 cm
    params.safety_margin = 0.03;  // m     - HER iki tarafa eklenen ekstra pay (sonar huzmesi zaten ekstra pay katiyor)
    params.max_speed     = 0.15;  // m/s
    cfg.loop_ms = 50;             // kontrol dongusu periyodu
    const double timeoutSeconds = 180.0;  // tum gorev icin azami sure

    while (sensor.poll()) {} // kullanicinin girdisi beklenirken biriken eski verileri at
    Live live(robot, sensor, x1, y1);

    std::cout << "\nGoing from (0, 0) to (" << x1 << ", " << y1 << ")\n";
    const bool reached = driveWithFollowTheGap(robot, live, x1, y1, params, timeoutSeconds);
    stop(robot);
    const auto end = robot.getOdometryData();

    std::cout << "\n\n--- Summary ---\n"
              << "Result             : " << (reached ? "GOAL REACHED" : "STOPPED (goal not reached)") << "\n"
              << std::fixed << std::setprecision(2)
              << "Distance to goal   : " << std::hypot(x1 - end.x, y1 - end.y) * 100.0 << " cm\n"
              << "Path length        : " << live.pathLength() << " m\n"
              << "Closest reading    : " << live.closestSeen() << " m\n";
    printPose(robot, "Final pose       :");
    return reached ? 0 : 1;
}

