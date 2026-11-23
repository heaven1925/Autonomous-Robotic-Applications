// Ornek 07 - Gercek Kobuki uzerinde Bug0 engelden kacinma, Arduino'dan
// okunan 3 gercek HC-SR04 ultrasonik sensor (sol-on, on, sag-on) ile.
// Karar mantiginin kendisi Bug0Planner.h icinde yasar ve donanimdan
// bagimsizdir; bu dosya sadece onu gercek robota ve sensore baglar, ve
// Motion.h'daki Kobuki'ye ozgu guvenlik kontrollerini (tampon / ucurum /
// tekerlek dusmesi -> dur) uygular.
//
//   Kullanim: Bug0.exe <Kobuki COM portu> <Arduino COM portu>
//
// Ne yapar:
//   1. Arduino sensor akisini ve Kobuki'yi acar, odometriyi sifirlar.
//   2. Hedefi (x y) stdin'den sorar.
//   3. Her kontrol dongusunde en guncel sensor okumasi ve odometri
//      pozuyla Bug0Planner::step() cagirir, donen (v, w) hiz komutunu
//      Kobuki::setBaseControl() ile uygular ve tek satirlik canli bir
//      durum bilgisi yazdirir.
//   4. Tampon / ucurum / tekerlek dusmesi durumunda Bug0 durumundan
//      BAGIMSIZ olarak hemen durur (Motion.h guard()) - bu, Bug0.md'deki
//      "SafetySupervisor" katmanidir, bilerek planlayicidan ayri tutulur.
//   5. Sensor zaman asiminda, Ctrl+C'de ya da Bug0Planner GoalReached /
//      Fault durumuna ulastiginda durur.
//
// HENUZ gercek donanimda dogrulanmadi (bkz. sohbet ozeti / Bug0.md
// "bilinen kisitlar"): arduino/sensor/sensor.ino icindeki sensor pin
// numaralari, tam montaj acilari (asagidaki beamSpreadRad) ve Config
// icindeki ayarlanmis mesafe esikleri hepsi Bug0.md'de tarif edilen
// geometriyi varsayar ve guvenmeden once fiziksel robotta dogrulanmalidir.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ArduinoTriSensor.h"
#include "Bug0Planner.h"
#include "Motion.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace kobuki::motion;
using namespace bug0;

namespace {

constexpr double SensorTimeoutS = 1.5; // bu kadar sure yeni okuma gelmezse -> dur, cik
constexpr double HudPeriodS = 0.2;     // canli durum satirinin yenilenme periyodu (s)
constexpr double BeamSpreadDeg = 55.0; // sol/sag huzmenin one gore acisi (montaj varsayimi)

std::atomic<bool> g_stopRequested{false};  // Ctrl+C basilinca true olur

// Konsol kontrol olaylarini (Ctrl+C) yakalayan Windows geri cagirim fonksiyonu.
BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT) { g_stopRequested = true; return TRUE; }
    return FALSE;
}

// Bug0 durumunu okunabilir bir metne cevirir (HUD/ekran yazdirmalari icin).
const char* stateName(State s) {
    switch (s) {
        case State::Init: return "INIT";
        case State::GoToGoal: return "GO_TO_GOAL";
        case State::AvoidObstacle: return "AVOID_OBSTACLE";
        case State::FollowBoundary: return "FOLLOW_BOUNDARY";
        case State::Recovery: return "RECOVERY";
        case State::GoalReached: return "GOAL_REACHED";
        case State::Fault: return "FAULT";
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

} // namespace

int main(int argc, char** argv) {
    std::cout << "Bug0 obstacle avoidance (3x HC-SR04 + Kobuki)\n\n";
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <Kobuki COM port> <Arduino COM port>\n";
        return 1;
    }

    // Once sensoru ac (henuz robota dokunulmuyor) ve ilk okumayi bekle.
    ArduinoTriSensor sensor;
    std::cout << "Opening sensor on " << argv[2] << "...\n";
    try {
        sensor.open(argv[2]);
    } catch (const std::exception& e) {
        std::cerr << "ERROR opening sensor port: " << e.what() << "\n";
        return 1;
    }
    for (int i = 0; i < 30 && sensor.readingCount() == 0; ++i) {
        sensor.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (sensor.readingCount() == 0) {
        std::cerr << "No sensor data on " << argv[2] << ". Check power/cable/sketch.\n";
        return 1;  // Arduino'dan hic veri gelmedi - guc/kablo/kodu kontrol et
    }

    // Hedef koordinatlarini kullanicidan oku (bkz. parseTarget; gecersiz
    // girdide tekrar sorulur, girdi akisi kapanirsa cikilir).
    double gx = 0.0, gy = 0.0;
    while (true) {
        std::cout << "Target x y (m), e.g. \"2 0\": " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) {
            std::cerr << "\nNo input - exiting.\n";
            return 1;
        }
        if (parseTarget(line, gx, gy)) break;
        std::cout << "  Invalid target, please enter two numbers.\n";
    }

    // Kobuki'ye baglan (burada odometri sifirlanir), Ctrl+C yakalayicisini kaydet.
    kobuki::Kobuki robot;
    connect(robot, argc, argv, "Bug0 obstacle avoidance");
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    // Bug0 planlayicisinin ayar degerleri: robot yaricapi, hiz sinirlari ve
    // mesafe esikleri (d_stop/d_slow/d_follow/d_clear) burada belirlenir.
    Config bug0Cfg;
    bug0Cfg.robotRadius = 0.18;
    bug0Cfg.goalTolerance = 0.05;
    bug0Cfg.driveSpeed = 0.15;
    bug0Cfg.followSpeed = 0.10;
    bug0Cfg.turnSpeed = 0.5;
    bug0Cfg.d_stop = 0.30;
    bug0Cfg.d_slow = 0.60;
    bug0Cfg.d_follow = 0.35;
    bug0Cfg.d_clear = 0.55;
    Bug0Planner planner(bug0Cfg);
    const Goal goal{ gx, gy };
    const double beamSpreadRad = BeamSpreadDeg * bug0::Pi / 180.0;  // derece -> radyan

    std::cout << "Going from (0, 0) to (" << gx << ", " << gy << ")\n";

    // Dongu icinde kullanilacak zaman/sayac degiskenleri.
    const auto t0 = std::chrono::steady_clock::now();
    auto lastReadingTime = t0;   // en son yeni sensor verisinin geldigi an
    auto lastHud = t0;           // HUD'un en son yenilendigi an
    std::size_t lastReadingCount = sensor.readingCount();
    double pathLength = 0.0;     // kat edilen toplam yol (m)
    auto prevOdom = robot.getOdometryData();
    State lastState = State::Init;  // durum degisikligini tespit etmek icin onceki durum

    while (true) {
        guard(robot); // tampon / ucurum / tekerlek dusmesi -> dur ve cik

        if (g_stopRequested) {
            stop(robot);
            std::cout << "\nCtrl+C - stopping.\n";
            return 1;
        }

        // En guncel sensor verisini al; uzun suredir veri gelmiyorsa guvenli dur.
        sensor.poll();
        const auto now = std::chrono::steady_clock::now();
        if (sensor.readingCount() != lastReadingCount) {
            lastReadingCount = sensor.readingCount();
            lastReadingTime = now;
        }
        const double sinceReading = std::chrono::duration<double>(now - lastReadingTime).count();
        if (sinceReading > SensorTimeoutS) {
            stop(robot);
            std::cout << "\n!! Sensor timeout (" << sinceReading << " s) - stopped.\n";
            return 1;
        }

        // Odometriden gercek pozu al, kat edilen yolu guncelle.
        const auto odom = robot.getOdometryData();
        pathLength += std::hypot(odom.x - prevOdom.x, odom.y - prevOdom.y);
        prevOdom = odom;
        const Pose pose{ odom.x, odom.y, robot.getHeading() };

        // Arduino'dan gelen son 3 sensor okumasini Bug0Planner'in bekledigi
        // bicime (SensorReading) aktar.
        const auto& tr = sensor.last();
        SensorReading sr;
        sr.left = tr.leftM;       sr.leftValid = tr.leftValid;
        sr.front = tr.frontM;     sr.frontValid = tr.frontValid;
        sr.right = tr.rightM;     sr.rightValid = tr.rightValid;

        // ASIL BUG0 KARARI burada verilir: planlayici mevcut sensor/poz/hedef
        // bilgisine gore bir (v, w) hiz komutu doner.
        const double dt = kobuki::motion::cfg.loop_ms / 1000.0;
        const Command cmd = planner.step(sr, pose, goal, dt);

        // Durum degistiyse ekrana bildir (ör. GO_TO_GOAL -> AVOID_OBSTACLE).
        if (planner.state() != lastState) {
            std::cout << "\n[STATE] " << stateName(lastState) << " -> " << stateName(planner.state());
            if (planner.state() == State::FollowBoundary)
                std::cout << " (wall on " << (planner.followSide() == Side::Right ? "right" : "left") << ")";
            std::cout << "\n";
            lastState = planner.state();
        }

        if (planner.state() == State::GoalReached) {
            // Hedefe ulasildi: durdur, ozet bilgiyi yazdir ve basariyla cik.
            stop(robot);
            std::cout << "\nResult           : GOAL REACHED\n";
            std::cout << "Path length      : " << pathLength << " m\n";
            printPose(robot, "Final pose      :");
            return 0;
        }
        if (planner.state() == State::Fault) {
            // Guvenli bir ilerleme yolu bulunamadi: durdur ve hata koduyla cik.
            stop(robot);
            std::cout << "\nResult           : FAULT (safety stop - could not find a safe way forward)\n";
            std::cout << "Path length      : " << pathLength << " m\n";
            printPose(robot, "Final pose      :");
            return 1;
        }

        // Planlayicinin dondurdugu hiz komutunu gercek robota uygula.
        robot.setBaseControl(cmd.v, cmd.w);

        // Belirli araliklarla canli durum satirini (durum, 3 sensor, poz, hedefe kalan mesafe) yazdir.
        if (std::chrono::duration<double>(now - lastHud).count() >= HudPeriodS) {
            lastHud = now;
            std::cout << "\r[" << stateName(planner.state()) << "] "
                      << "L " << (sr.leftValid ? sr.left : -1.0) << "m  "
                      << "F " << (sr.frontValid ? sr.front : -1.0) << "m  "
                      << "R " << (sr.rightValid ? sr.right : -1.0) << "m  | "
                      << "pos (" << std::fixed << std::setprecision(2) << pose.x << ", " << pose.y << ") "
                      << "head " << std::setprecision(0) << pose.heading / Deg << " deg | "
                      << "to goal " << std::setprecision(2) << planner.distanceToGoal(pose, goal) << " m   "
                      << std::flush;
        }

        wait();  // bir sonraki kontrol dongusune kadar bekle (cfg.loop_ms)
        (void)beamSpreadRad; // sadece belgeleme amacli tutuluyor; gercek montaj acisi donanimda sabittir
    }
}
