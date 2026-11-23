// ArduinoTriSensor.h - reads three HC-SR04 ultrasonic sensors (left-front,
// front, right-front) from an Arduino over serial, one JSON line per cycle:
//   {"l":<cm>,"f":<cm>,"r":<cm>}
// -1 in any field means "no echo / out of range / timeout" for that beam.
//
// Sketch: 07_Bug0Algorithm/arduino/sensor/sensor.ino, printing at ~10 Hz.
//
// This header only turns raw serial bytes into the latest three distances
// plus per-beam validity flags and a reading counter (used by callers to
// detect stale data). It has no notion of Bug0, robot geometry or motion -
// see Bug0Planner.h for the decision logic that consumes these readings.
#pragma once

#include "SerialPort.h"

#include <cstdint>
#include <optional>
#include <sstream>
#include <string>

namespace bug0 {

// leftM/frontM/rightM are always meaningful distances: a -1 (no echo /
// out of range) from the sensor is mapped to maxRangeMeters(), i.e. "clear
// at least that far" - the same convention 06_FollowTheGap's
// ArduinoRangeSensor uses. The *Valid flags only mean "this field was
// present in the last parsed packet"; they go false on a genuine comms
// problem (malformed/missing packet), not on a normal long-range echo
// timeout. Staleness (no NEW packet for a while) is a separate check done
// by the caller via readingCount().
struct TriReading {
    double leftM = 0.0, frontM = 0.0, rightM = 0.0;
    bool leftValid = false, frontValid = false, rightValid = false;
};

class ArduinoTriSensor {
public:
    explicit ArduinoTriSensor(double maxRangeMeters = 4.0) : maxRangeM_(maxRangeMeters) {}

    void open(const std::string& portName) { port_.open(portName); }

    // Drains whatever the Arduino has sent since the last call and keeps
    // the newest complete line's values. Returns true if any bytes were
    // read at all (even a partial line) - same contract as
    // ArduinoRangeSensor::poll() in 06_FollowTheGap.
    bool poll() {
        std::uint8_t buf[256];
        const std::size_t n = port_.read(buf, sizeof(buf));
        if (n == 0) return false;
        rxBuffer_.append(reinterpret_cast<char*>(buf), n);

        bool got = false;
        std::size_t newlinePos;
        while ((newlinePos = rxBuffer_.find('\n')) != std::string::npos) {
            const std::string line = rxBuffer_.substr(0, newlinePos);
            rxBuffer_.erase(0, newlinePos + 1);
            if (parseLine(line)) got = true;
        }
        if (got) ++readingCount_;
        return true;
    }

    std::size_t readingCount() const { return readingCount_; }
    const TriReading& last() const { return last_; }
    double maxRangeMeters() const { return maxRangeM_; }

private:
    bool parseLine(const std::string& line) {
        const auto l = parseField(line, "\"l\":");
        const auto f = parseField(line, "\"f\":");
        const auto r = parseField(line, "\"r\":");
        if (!l && !f && !r) return parsePlain(line);

        TriReading next;
        if (l) { next.leftValid  = true; next.leftM  = (*l >= 0) ? *l / 100.0 : maxRangeM_; }
        if (f) { next.frontValid = true; next.frontM = (*f >= 0) ? *f / 100.0 : maxRangeM_; }
        if (r) { next.rightValid = true; next.rightM = (*r >= 0) ? *r / 100.0 : maxRangeM_; }
        last_ = next;
        return true;
    }

    // Fallback for a plain test sketch printing "<front> <left> <right>" in cm
    // per line (exactly three integers, nothing else). 0 or negative = no echo.
    bool parsePlain(const std::string& line) {
        std::istringstream iss(line);
        long v[3];
        if (!(iss >> v[0] >> v[1] >> v[2])) return false;
        std::string rest;
        if (iss >> rest) return false;
        auto m = [&](long cm) { return cm > 0 ? cm / 100.0 : maxRangeM_; };
        TriReading next;
        next.frontM = m(v[0]); next.leftM = m(v[1]); next.rightM = m(v[2]);
        next.leftValid = next.frontValid = next.rightValid = true;
        last_ = next;
        return true;
    }

    static std::optional<int> parseField(const std::string& line, const char* key) {
        const auto pos = line.find(key);
        if (pos == std::string::npos) return std::nullopt;
        try {
            return std::stoi(line.substr(pos + std::string(key).size()));
        } catch (...) {
            return std::nullopt;
        }
    }

    kobuki::SerialPort port_;
    double maxRangeM_;
    std::string rxBuffer_;
    TriReading last_;
    std::size_t readingCount_ = 0;
};

} // namespace bug0
