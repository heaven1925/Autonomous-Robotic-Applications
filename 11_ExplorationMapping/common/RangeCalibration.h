// RangeCalibration.h - HC-SR04 distance calibration against one known distance
// (40 cm = one block / one cell). Written by ExploreLauncher (step 3 "Sensor
// kalibrasyonu"), read by every Explore_*.exe on the real robot.
//
//   raw cm        = echo time [us] * 0.01715   (343 m/s, there and back)
//                   (an old sketch without "us" sends whole cm: raw cm = "f")
//   calibrated cm = raw cm * scale,   scale = 40 / mean(raw cm at 40 cm)
//
// One point fixes one parameter. The HC-SR04's error is mostly a scale (the
// speed of sound changes with temperature: 331 m/s at 0 C, 343 m/s at 20 C),
// so the point is used for the scale. "343 * scale" is the speed of sound the
// calibration implies - a quick sanity check (about 330..355 m/s indoors).
//
// File (sensor_calibration.txt, next to sensor_model.txt):
//   # comments
//   scale 0.98321
//   reference_cm 40.0
//   raw_mean_cm 40.68
//   ... (see save())
#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace rangecal {

constexpr double CmPerUs = 0.0343 / 2.0;   // nominal: 343 m/s, there and back
constexpr double ReferenceCm = 40.0;       // one block / one cell
constexpr double MinScale = 0.85, MaxScale = 1.15;

struct Calibration {
    bool valid = false;
    double scale = 1.0;
    double referenceCm = ReferenceCm;
    double rawMeanCm = 0.0;     // mean raw reading at the reference distance
    double meanUs = -1.0;       // mean echo time (-1: old sketch, whole cm only)
    double sigmaCm = 0.0;       // spread of the calibrated readings
    int samples = 0, rejected = 0;
    int trig = 11, echo = 12;   // Arduino pins at calibration time
    std::string date;

    double cm(double rawCm) const { return rawCm * scale; }
    double speedOfSound() const { return 343.0 * scale; }   // m/s
};

inline bool load(const std::filesystem::path& file, Calibration& c) {
    c = Calibration{};
    std::ifstream in(file);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream s(line);
        std::string key;
        s >> key;
        if (key == "date") { std::getline(s >> std::ws, c.date); continue; }
        double v = 0.0;
        if (!(s >> v)) continue;
        if (key == "scale") c.scale = v;
        else if (key == "reference_cm") c.referenceCm = v;
        else if (key == "raw_mean_cm") c.rawMeanCm = v;
        else if (key == "mean_us") c.meanUs = v;
        else if (key == "sigma_cm") c.sigmaCm = v;
        else if (key == "samples") c.samples = static_cast<int>(v);
        else if (key == "rejected") c.rejected = static_cast<int>(v);
        else if (key == "trig") c.trig = static_cast<int>(v);
        else if (key == "echo") c.echo = static_cast<int>(v);
    }
    c.valid = std::isfinite(c.scale) && c.scale >= MinScale && c.scale <= MaxScale;
    if (!c.valid) c.scale = 1.0;
    return c.valid;
}

inline bool save(const std::filesystem::path& file, const Calibration& c) {
    std::ofstream out(file);
    if (!out) return false;
    out << std::fixed
        << "# HC-SR04 mesafe kalibrasyonu (ExploreLauncher) - " << c.date << "\n"
        << "# Blok yuzu sensorun onunden " << std::setprecision(1) << c.referenceCm << " cm uzakta; " << c.samples
        << " olcum (" << c.rejected << " aykiri olcum atildi).\n"
        << "# cm = ham_cm * scale,  ham_cm = echo_us * 0.01715  (ses hizi " << std::setprecision(0) << c.speedOfSound()
        << " m/s)\n"
        << "# Tum Explore_*.exe programlari gercek robotta bu dosyayi okur.\n"
        << std::setprecision(5) << "scale " << c.scale << "\n"
        << std::setprecision(1) << "reference_cm " << c.referenceCm << "\n"
        << std::setprecision(2) << "raw_mean_cm " << c.rawMeanCm << "\n"
        << std::setprecision(1) << "mean_us " << c.meanUs << "\n"
        << std::setprecision(2) << "sigma_cm " << c.sigmaCm << "\n"
        << "samples " << c.samples << "\n"
        << "rejected " << c.rejected << "\n"
        << "trig " << c.trig << "\n"
        << "echo " << c.echo << "\n"
        << "date " << c.date << "\n";
    return static_cast<bool>(out);
}

} // namespace rangecal
