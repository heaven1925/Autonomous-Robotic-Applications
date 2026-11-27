// Calibration.h - experimental HC-SR04 sensor model (ders notu, "Building a
// Probabilistic Sensor Model"). Shared by all four methods: the sensor model
// belongs to the hardware, not to the exploration strategy, so every method
// reads the same sensor_model.txt.
//
//   Method B (Gaussian): kutu sirayla 1, 2, 3, 5, 10 hucre ileride; her
//     mesafede N olcum -> ortalama, sigma, algilama orani. sigma(d) dogrusu
//     sigma0 + k*d en kucuk kareler ile uydurulur. Hucreye dusme olasiligi
//     P(olay|Dolu) bu Gaussian'in hucre araligi uzerinden integralidir.
//   Method A (deney): hucre BOS iken N olcumun kaci hucreye dusuyor -> P(olay|Bos).
// Sonunda slayttaki ornek (z = 4.0 m, aralik [3.9, 4.1] m) ve 0.5 -> 4 m tablo yazilir.
#pragma once

#include "BayesGrid.h"
#include "Console.h"
#include "ExploreRobot.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace app {

struct CalibrationOptions {
    bool sim = false, noise = true;
    std::string arduinoPort, modelPath;
    int trials = 100;
    double rangeScale = 1.0;   // 40 cm distance calibration (RangeCalibration.h)
    std::vector<int> cells{ 1, 2, 3, 5, 10 };   // box positions (cells ahead)
    explore::Geometry geo;
};

// One Bayes step from P = 0.5.
inline double fromHalf(double likeO, double likeE) { return bayes::posterior(0.5, likeO, likeE); }

inline void printModelTable(const bayes::SensorModel& m, double cell) {
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

inline int calibrate(const CalibrationOptions& o, const astar::GridMap& arena) {
    const double cell = arena.cellSize();
    std::cout << "=== Sensor calibration: Gaussian noise model (Method B) + P(hit|E) (Method A) ===\n\n"
              << "Front sensor only. The robot stands with its CENTRE in the middle of a cell, facing along the grid.\n"
              << "A box (flat face to the robot) is put k cells ahead, its face on the near edge of that cell.\n"
              << o.trials << " readings per position.\n";

    // Box positions: k cells ahead -> distance from the sensor to the box face.
    const std::vector<int>& cells = o.cells;
    const auto faceDist = [&](int k) { return cell * (k - 0.5) - o.geo.offset; };

    std::unique_ptr<explore::SensorBoard> board;
    std::unique_ptr<explore::SimRobot> sim;
    const int maxK = *std::max_element(cells.begin(), cells.end());
    const astar::Cell robotCell{ arena.cols() / 2, 1 };
    if (o.sim) {
        explore::SimRobot::Options so;
        so.geo = o.geo;
        so.walledBorder = false;
        so.noise = o.noise;
        so.speed = 0;
        astar::GridMap open(arena.cols(), maxK + 6, cell);
        sim = std::make_unique<explore::SimRobot>(open, nav::Pose{ open.centre(robotCell).x, open.centre(robotCell).y, -90.0 * nav::Deg }, so);
        sim->open();
        std::cout << "(simulator: open room, the box is placed / removed automatically)\n";
    } else {
        board = std::make_unique<explore::SensorBoard>(o.arduinoPort, o.rangeScale);
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
    std::cout << "\nSaved: " << o.modelPath << "  (all four exploration methods use this file)\n";
    return 0;
}

} // namespace app
