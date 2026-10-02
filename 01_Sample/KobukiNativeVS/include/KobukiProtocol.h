#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace kobuki {

class CoreSensors {
public:
    struct Data {
        std::uint16_t time_stamp = 0;
        std::uint8_t bumper = 0;
        std::uint8_t wheel_drop = 0;
        std::uint8_t cliff = 0;
        std::uint16_t left_encoder = 0;
        std::uint16_t right_encoder = 0;
        std::int8_t left_pwm = 0;
        std::int8_t right_pwm = 0;
        std::uint8_t buttons = 0;
        std::uint8_t charger = 0;
        std::uint8_t battery = 0;
        std::uint8_t overcurrent = 0;

        double batteryVoltage() const { return static_cast<double>(battery) * 0.1; }
        bool bumperRight() const { return (bumper & 0x01) != 0; }
        bool bumperCenter() const { return (bumper & 0x02) != 0; }
        bool bumperLeft() const { return (bumper & 0x04) != 0; }
        bool cliffRight() const { return (cliff & 0x01) != 0; }
        bool cliffCenter() const { return (cliff & 0x02) != 0; }
        bool cliffLeft() const { return (cliff & 0x04) != 0; }
    };
};

class Inertia {
public:
    struct Data {
        // Factory-calibrated z-axis gyro values from feedback sub-payload 0x04.
        // Kobuki reports angle and angle rate in hundredths of a degree.
        std::int16_t angle = 0;
        std::int16_t angle_rate = 0;
        std::uint8_t acc[3] = {0, 0, 0};

        double angleDegrees() const;
        double angularVelocityDegreesPerSecond() const;
        double angleRadians() const;
        double angularVelocityRadiansPerSecond() const;
    };
};

struct OdometryData {
    // Integrated encoder odometry in a local 2-D frame.
    double x = 0.0;                   // metres
    double y = 0.0;                   // metres
    double heading = 0.0;             // radians, encoder-derived
    double linear_velocity = 0.0;     // m/s
    double angular_velocity = 0.0;    // rad/s, encoder-derived
    double left_distance = 0.0;       // metres since reset
    double right_distance = 0.0;      // metres since reset
};

class OdometryEstimator {
public:
    static constexpr double WheelBaseMeters = 0.230;
    static constexpr double TicksToMeters = 0.00008529209049773756;

    void reset();
    bool update(const CoreSensors::Data& sensors);
    const OdometryData& data() const { return data_; }

private:
    bool initialized_ = false;
    std::uint16_t lastTimestamp_ = 0;
    std::uint16_t lastLeftEncoder_ = 0;
    std::uint16_t lastRightEncoder_ = 0;
    OdometryData data_{};
};

class KobukiProtocol {
public:
    static constexpr double WheelBaseMeters = 0.230;

    // Returns Kobuki protocol values: speed [mm/s], radius [mm].
    static std::pair<std::int16_t, std::int16_t> velocityToSpeedRadius(
        double linearVelocityMetersPerSecond,
        double angularVelocityRadiansPerSecond);

    static std::vector<std::uint8_t> makeBaseControlPacket(
        double linearVelocityMetersPerSecond,
        double angularVelocityRadiansPerSecond);

    void append(const std::uint8_t* data, std::size_t size);
    std::vector<std::vector<std::uint8_t>> extractPayloads();

    static bool parseCoreSensors(const std::vector<std::uint8_t>& payload, CoreSensors::Data& data);
    static bool parseInertia(const std::vector<std::uint8_t>& payload, Inertia::Data& data);

private:
    std::vector<std::uint8_t> buffer_;
};

} // namespace kobuki
