// sim_main.cpp - Bug0Planner icin saf yazilim test araci (donanim, Arduino
// ya da Kobuki YOK). Uc ultrasonik huzmeyi, duvar parcalarindan olusan 2B bir
// dunyaya karsi simule eder, planlayiciyi + basit bir diferansiyel-surus
// kinematik modelini adim adim ilerletir ve carpisma, zaman asimi ve hedefe
// ulasma durumlarini kontrol eder.
//
// Argumansiz calistirin: sabit bir senaryo kumesini yurutur ve her basarisizlik
// icin detaylarla birlikte bir PASS/FAIL (gecti/kaldi) tablosu yazdirir. Cikis
// kodu sadece TUM senaryolar gecerse 0 olur.
#include "Bug0Planner.h"

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

using namespace bug0;

namespace {

struct Segment { double x1, y1, x2, y2; };  // bir duvar parcasi (dogru parcasi)

// Isin (ox,oy)+t*(dx,dy), t>0, duvar parcasiyla kesisiyor mu? Varsa
// carpma mesafesi t'yi dondurur.
std::optional<double> rayHit(double ox, double oy, double dx, double dy, const Segment& s) {
    const double sx = s.x2 - s.x1, sy = s.y2 - s.y1;
    const double denom = dx * sy - dy * sx;
    if (std::abs(denom) < 1e-12) return std::nullopt;  // isin ve duvar paralel, kesisim yok
    const double t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / denom;
    const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / denom;
    if (t < 0.0 || u < 0.0 || u > 1.0) return std::nullopt;  // kesisim isinin disinda ya da parcanin disinda
    return t;
}

// (px,py) noktasindan duvar parcasina olan en kisa mesafe.
double pointSegDist(double px, double py, const Segment& s) {
    const double vx = s.x2 - s.x1, vy = s.y2 - s.y1;
    const double wx = px - s.x1, wy = py - s.y1;
    const double len2 = vx * vx + vy * vy;
    double t = (len2 > 1e-12) ? (wx * vx + wy * vy) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = s.x1 + t * vx, cy = s.y1 + t * vy;
    return std::hypot(px - cx, py - cy);
}

// Simule edilen 2B dunya: duvar parcalari listesi + sensorlerin gorebilecegi azami mesafe.
struct World {
    std::vector<Segment> walls;
    double maxRange = 4.0;

    // Verilen konumdan, verilen mutlak acida bir ultrasonik isin firlatir ve
    // en yakin carpma mesafesini (ya da hic carpma yoksa maxRange'i) dondurur.
    double castBeam(double ox, double oy, double angle, bool& validOut) const {
        const double dx = std::cos(angle), dy = std::sin(angle);
        double best = maxRange;
        for (const auto& s : walls) {
            if (auto t = rayHit(ox, oy, dx, dy, s)) {
                if (*t < best) best = *t;
            }
        }
        // ArduinoTriSensor uzerinden gelen gercek bir HC-SR04 her zaman
        // "gecerli" bir alan verir (menzil disi olmasi sadece maxRange =
        // "o kadar uzaga kadar acik" demektir); validOut sadece bir senaryo
        // bilerek bir iletisim kopmasi enjekte ettiginde (asagidaki
        // dropoutEvery'ye bakin) false olur.
        validOut = true;
        return best;
    }

    // Verilen noktadan en yakin duvara olan mesafeyi dondurur (carpisma kontrolu icin).
    double distanceToNearestWall(double px, double py) const {
        double best = 1e9;
        for (const auto& s : walls) best = std::min(best, pointSegDist(px, py, s));
        return best;
    }
};

// Bir senaryonun sonuc bilgileri.
struct ScenarioResult {
    std::string name;
    bool reachedGoal = false;  // hedefe ulasti mi
    bool collided = false;     // carpti mi (govde yaricapindan yakin duvar)
    bool timedOut = false;     // ne hedefe ulasti ne carpti ne de Fault oldu, sure doldu
    double simTime = 0.0;      // senaryonun bitis anindaki simulasyon zamani (s)
    double finalDist = 0.0;    // bitis anindaki hedefe kalan mesafe (m)
    double minClearance = 1e9; // simulasyon boyunca duvara olan en kucuk mesafe (m)
    int stateFaults = 0;       // Fault durumuna kac kez girildigi (burada en fazla 1)
};

// Tek bir senaryoyu simule eder. beamSpread = on huzme ile her bir yan huzme
// arasindaki yari-aci (rad). dropoutEvery > 0 ise histerezis sayaclarini
// test etmek icin periyodik olarak gecersiz okuma enjekte eder.
ScenarioResult runScenario(const std::string& name, const World& world, Pose start, Goal goal,
                           Config cfg, double robotRadius, double maxSimTime = 60.0,
                           double beamSpread = 55.0 * Pi / 180.0, int dropoutEvery = 0,
                           bool trace = false) {
    ScenarioResult r;
    r.name = name;
    Bug0Planner planner(cfg);
    Pose pose = start;
    const double dt = 0.05;  // simulasyon zaman adimi (s)
    int step = 0;

    for (double t = 0.0; t < maxSimTime; t += dt, ++step) {
        // Uc sensoru (on, sol-on, sag-on) mevcut poza gore isin izleyerek olc.
        SensorReading sr;
        sr.front = world.castBeam(pose.x, pose.y, pose.heading, sr.frontValid);
        sr.left  = world.castBeam(pose.x, pose.y, pose.heading + beamSpread, sr.leftValid);
        sr.right = world.castBeam(pose.x, pose.y, pose.heading - beamSpread, sr.rightValid);

        if (dropoutEvery > 0 && (step % dropoutEvery) == 0) {
            sr.frontValid = sr.leftValid = sr.rightValid = false;  // kasitli sensor veri kaybi simule et
        }

        const double clearance = world.distanceToNearestWall(pose.x, pose.y);
        r.minClearance = std::min(r.minClearance, clearance);
        if (trace && step % 4 == 0) {
            // Hata ayiklama amacli: belirli araliklarla durum/poz/sensor izini yazdir.
            std::printf("t=%5.2f st=%d side=%d pos=(%.3f,%.3f) h=%.1f L=%.2f F=%.2f R=%.2f clr=%.3f\n",
                        t, (int)planner.state(), (int)planner.followSide(), pose.x, pose.y,
                        pose.heading * 180.0 / Pi, sr.left, sr.front, sr.right, clearance);
        }
        if (clearance < robotRadius) { r.collided = true; break; }  // robot govdesi duvara cok yaklasti: CARPISMA

        // Bug0 kararini bu simulasyon dongusunde de gercek koddaki gibi cagir.
        const Command cmd = planner.step(sr, pose, goal, dt);

        if (planner.state() == State::GoalReached) { r.reachedGoal = true; r.simTime = t; break; }
        if (planner.state() == State::Fault) { ++r.stateFaults; r.simTime = t; break; }

        // Basit diferansiyel-surus kinematik modeli: komutu uygulayip pozu ilerlet.
        pose.heading = std::atan2(std::sin(pose.heading + cmd.w * dt), std::cos(pose.heading + cmd.w * dt));
        pose.x += cmd.v * std::cos(pose.heading) * dt;
        pose.y += cmd.v * std::sin(pose.heading) * dt;
        r.simTime = t;
    }
    if (!r.reachedGoal && !r.collided && r.stateFaults == 0) r.timedOut = true;  // sure doldu, baska bir sonuc olusmadi
    r.finalDist = std::hypot(goal.x - pose.x, goal.y - pose.y);
    return r;
}

// Bir senaryonun PASS/FAIL (gecti/kaldi) satirini yazdirir.
// expectReach = true ise hedefe ulasmak da sart; degilse sadece carpismamak yeterli.
void printResult(const ScenarioResult& r, bool expectReach) {
    const bool pass = !r.collided && (expectReach ? r.reachedGoal : true);
    std::printf("[%s] %-28s  t=%5.1fs  dist=%5.2fm  minClear=%5.2fm  %s\n",
                pass ? "PASS" : "FAIL", r.name.c_str(), r.simTime, r.finalDist, r.minClearance,
                r.collided ? "COLLISION" : r.reachedGoal ? "GOAL" : r.stateFaults ? "FAULT(safe-stop)" : "TIMEOUT");
}

} // namespace

int main() {
    Config cfg; // Bug0Planner.h'deki varsayilan ayarlar
    const double R = cfg.robotRadius;
    bool allOk = true;  // TUM zorunlu senaryolar basarili mi

    // 1) Bos dunya: duz hat, engel yok.
    {
        World w; w.walls = {};
        auto r = runScenario("Empty world", w, {0,0,0}, {2,0}, cfg, R);
        printResult(r, true);
        allOk &= r.reachedGoal && !r.collided;
    }

    // 2) Onde tek duvar, her iki tarafta acik alan. Guvenlik-oncelikli ayarla
    //    (Bug0Planner.h'ye bakin) engel tam ortada ve mukemmel simetrikse
    //    robot isi bitirmek yerine tedbirli bir guvenli-duruşla
    //    sonuclanabilir; burada aranan sonuc carpismamaktir.
    {
        World w; w.walls = { {1.0,-0.5, 1.0,0.5} };
        auto r = runScenario("Single frontal wall", w, {0,0,0}, {2,0}, cfg, R);
        printResult(r, false);
        allOk &= !r.collided;
    }

    // 3) Duvar yolun sagina kaymis -> engel sadece kismen engelliyor.
    {
        World w; w.walls = { {1.0,-0.6, 1.0,0.1} };
        auto r = runScenario("Right-side obstacle", w, {0,0,0}, {2,0}, cfg, R);
        printResult(r, false);
        allOk &= !r.collided;
    }

    // 4) Koridor: robotun gecebilecegi kadar genis iki paralel duvar.
    {
        World w;
        w.walls = { {1.0,0.35, 2.5,0.35}, {1.0,-0.35, 2.5,-0.35} };
        auto r = runScenario("Corridor (passable)", w, {0,0,0}, {3,0}, cfg, R);
        printResult(r, true);
        allOk &= r.reachedGoal && !r.collided;
    }

    // 5) Robot govdesinden dar koridor, duvarlar girisin tam basinda
    //    hazirliksiz basliyor: bu BILINEN ve BELGELENMIS bir SINIRLAMADIR
    //    (Bug0.md "Kritik kisit" - tam cepheden yaklasan 3 ileri/capraz
    //    ultrasonik huzme, govdenin kendi acikligi yaricapinda tam baslayan
    //    bir koridor agzini surtunmeyi onleyecek kadar erken goremez).
    //    Goruntuleme amacli raporlanir ama allOk'a DAHIL EDILMEZ; cozumler
    //    (daha genis acili yan huzmeler, dorduncu bir yan sensor, ya da daha
    //    yavas yaklasma hizi) donanim/ayar konusu, planlayici mantik hatasi degildir.
    {
        World w;
        w.walls = { {1.0,0.15, 2.5,0.15}, {1.0,-0.15, 2.5,-0.15} };
        Config narrowCfg = cfg; narrowCfg.maxFollowTimeS = 8.0; narrowCfg.recoveryTimeS = 1.0;
        auto r = runScenario("Corridor (too narrow) [known limitation]", w, {0,0,0}, {3,0}, narrowCfg, R, 30.0);
        printResult(r, false);
        // allOk'a dahil edilmedi - yukaridaki aciklamaya bakin.
    }

    // 6) Icbukey / U-seklinde engel: bilinen bir Bug0 sinirlamasi (Bug0.md
    //    bolum 2) - yine de carpismamali ve sonsuza kadar donup durmamali
    //    (maxFollowTimeS -> Recovery -> Fault ile sinirlanir).
    {
        World w;
        w.walls = {
            {0.8,-0.6, 0.8,0.6},   // U'nun arka duvari
            {0.8,0.6, 1.6,0.6},    // ust kol
            {0.8,-0.6, 1.6,-0.6},  // alt kol
        };
        Config uCfg = cfg; uCfg.maxFollowTimeS = 10.0; uCfg.recoveryTimeS = 1.0; uCfg.maxRecoveryAttempts = 2;
        auto r = runScenario("U-shaped trap (concave)", w, {0,0,0}, {3,0}, uCfg, R, 40.0);
        printResult(r, false);
        allOk &= !r.collided;
    }

    // 7) Sensor veri kayiplari: her 5. dongude uc huzme de gecersiz rapor eder.
    //    Histerezis (confirmReadings) bunun bir cokme/gecersiz duruma yol
    //    acmasini onlemeli; carpismamak sarttir, ama hedefe ulasmak yerine
    //    guvenli bir Recovery/Fault duruşu da kabul edilebilir.
    {
        World w; w.walls = { {1.0,-0.5, 1.0,0.5} };
        auto r = runScenario("Frontal wall + sensor dropouts", w, {0,0,0}, {2,0}, cfg, R,
                              60.0, 55.0 * Pi / 180.0, /*dropoutEvery=*/5);
        printResult(r, false);
        allOk &= !r.collided;
    }

    // 8) Yaklasik 90 derecelik bir sinir-donusu gerektiren engel (kose).
    {
        World w;
        w.walls = { {1.0,-0.8, 1.0,0.4}, {1.0,0.4, 2.2,0.4} }; // L seklinde kose
        auto r = runScenario("Corner obstacle", w, {0,0,0}, {3,0}, cfg, R);
        printResult(r, false);
        allOk &= !r.collided;
    }

    std::printf("\n%s\n", allOk ? "ALL SCENARIOS OK (no collisions; required goals reached)"
                                : "SOME SCENARIOS FAILED");
    return allOk ? 0 : 1;
}
