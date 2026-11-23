// ArduinoRangeSensor.h - reads a single forward-facing HC-SR04 ultrasonic
// sensor from an Arduino Uno over serial. This is the "write one new file"
// half of the real-sensor swap promised when the simulated sensor was
// built (see Kobuki_Follow_the_Gap_Sensor_Mimarisi.md section 4 and
// FollowTheGap.h's header comment); the other half is main.cpp physically
// rotating the robot to sweep this single fixed beam across an arc and
// building a real multi-sample RangeScan from the measurements (a static
// single ultrasonic sensor has no other way to "see" sideways - see the
// architecture doc sections 3.1 and 6).
//
// Sketch: 06_FollowTheGap/arduino/sensor/sensor.ino, printing
// `{"center":<cm>}` once per ~100 ms (-1 = no echo / out of range).
#pragma once

#include "SerialPort.h"

#include <cstdint>
#include <optional>
#include <string>

namespace ftg {

// Opens the Arduino's serial port and keeps the most recent `{"center":N}`
// reading, converted to metres (-1 / no echo maps to maxRangeMeters()).
class ArduinoRangeSensor {
public:
    explicit ArduinoRangeSensor(double maxRangeMeters = 4.0)
        : maxRangeM_(maxRangeMeters), lastRangeM_(maxRangeMeters) {}

    void open(const std::string& portName) { port_.open(portName); }

    // Drains whatever the Arduino has sent since the last call and keeps
    // the newest complete line's value (older lines in the same batch are
    // stale and discarded - only "now" matters for a reactive loop).
    // Returns true if any bytes were read at all (even a partial line) -
    // callers that need to flush a backlog of OLD readings (e.g. after
    // not polling for a while, see scanAround() in main.cpp) can call this
    // in a loop until it returns false to know the serial buffer is empty.
    bool poll() {
        std::uint8_t buf[256];
        const std::size_t n = port_.read(buf, sizeof(buf));
        if (n == 0) return false;
        rxBuffer_.append(reinterpret_cast<char*>(buf), n);

        std::optional<int> latest;
        std::size_t newlinePos;
        while ((newlinePos = rxBuffer_.find('\n')) != std::string::npos) {
            const std::string line = rxBuffer_.substr(0, newlinePos);
            rxBuffer_.erase(0, newlinePos + 1);
            if (const auto cm = parseCenter(line)) latest = cm;
        }
        if (latest) {
            lastRangeM_ = (*latest < 0) ? maxRangeM_ : static_cast<double>(*latest) / 100.0;
            ++readingCount_;
        }
        return true;
    }

    // Number of complete readings received so far - changes whenever a
    // genuinely new measurement arrives (used to timestamp scan samples).
    std::size_t readingCount() const { return readingCount_; }
    double lastRangeMeters() const { return lastRangeM_; }
    double maxRangeMeters() const { return maxRangeM_; }

private:
    // Accepts the 06 sketch's `{"center":N}` and also the front beam of the
    // 07 three-sensor sketch's `{"l":..,"f":N,"r":..}`, so one Arduino sketch
    // serves both apps.
    static std::optional<int> parseCenter(const std::string& line) {
        for (const std::string key : {"\"center\":", "\"f\":"}) {
            const auto pos = line.find(key);
            if (pos == std::string::npos) continue;
            try {
                return std::stoi(line.substr(pos + key.size()));
            } catch (...) {
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    kobuki::SerialPort port_;
    double maxRangeM_;
    std::string rxBuffer_;
    double lastRangeM_;
    std::size_t readingCount_ = 0;
};

} // namespace ftg

