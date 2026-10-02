#include "KobukiProtocol.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kobuki {
namespace {

constexpr double Pi = 3.141592653589793238462643383279502884;

std::int16_t clampToInt16(double value) {
    const double high = static_cast<double>((std::numeric_limits<std::int16_t>::max)());
    const double low = static_cast<double>((std::numeric_limits<std::int16_t>::min)());
    if (value > high) return (std::numeric_limits<std::int16_t>::max)();
    if (value < low) return (std::numeric_limits<std::int16_t>::min)();
    // kobuki_core historically truncates when converting to short.
    return static_cast<std::int16_t>(value);
}

void pushInt16LE(std::vector<std::uint8_t>& out, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    out.push_back(static_cast<std::uint8_t>(raw & 0xFF));
    out.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
}

std::uint16_t readUInt16LE(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8);
}

std::int16_t readInt16LE(const std::uint8_t* p) {
    return static_cast<std::int16_t>(readUInt16LE(p));
}

double normalizeAngle(double angle) {
    while (angle > Pi) angle -= 2.0 * Pi;
    while (angle <= -Pi) angle += 2.0 * Pi;
    return angle;
}

} // namespace

double Inertia::Data::angleDegrees() const {
    return static_cast<double>(angle) / 100.0;
}

double Inertia::Data::angularVelocityDegreesPerSecond() const {
    return static_cast<double>(angle_rate) / 100.0;
}

double Inertia::Data::angleRadians() const {
    return angleDegrees() * Pi / 180.0;
}

double Inertia::Data::angularVelocityRadiansPerSecond() const {
    return angularVelocityDegreesPerSecond() * Pi / 180.0;
}

void OdometryEstimator::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
    lastLeftEncoder_ = 0;
    lastRightEncoder_ = 0;
    data_ = OdometryData{};
}

bool OdometryEstimator::update(const CoreSensors::Data& sensors) {
    if (!initialized_) {
        lastTimestamp_ = sensors.time_stamp;
        lastLeftEncoder_ = sensors.left_encoder;
        lastRightEncoder_ = sensors.right_encoder;
        initialized_ = true;
        return false;
    }

    // The firmware counters wrap at 65535. Casting the wrapped difference to
    // int16_t gives the correct signed delta for normal 50 Hz updates.
    const auto leftTicks = static_cast<std::int16_t>(
        static_cast<std::uint16_t>(sensors.left_encoder - lastLeftEncoder_));
    const auto rightTicks = static_cast<std::int16_t>(
        static_cast<std::uint16_t>(sensors.right_encoder - lastRightEncoder_));
    const auto deltaTimeMs = static_cast<std::uint16_t>(sensors.time_stamp - lastTimestamp_);

    lastTimestamp_ = sensors.time_stamp;
    lastLeftEncoder_ = sensors.left_encoder;
    lastRightEncoder_ = sensors.right_encoder;

    const double left = static_cast<double>(leftTicks) * TicksToMeters;
    const double right = static_cast<double>(rightTicks) * TicksToMeters;
    const double distance = 0.5 * (left + right);
    const double deltaHeading = (right - left) / WheelBaseMeters;

    // Integrate the exact differential-drive arc in the robot's local frame.
    double localX = 0.0;
    double localY = 0.0;
    if (std::abs(deltaHeading) < 1e-12) {
        localX = distance;
    } else {
        const double radius = distance / deltaHeading;
        localX = radius * std::sin(deltaHeading);
        localY = radius * (1.0 - std::cos(deltaHeading));
    }

    const double c = std::cos(data_.heading);
    const double s = std::sin(data_.heading);
    data_.x += c * localX - s * localY;
    data_.y += s * localX + c * localY;
    data_.heading = normalizeAngle(data_.heading + deltaHeading);
    data_.left_distance += left;
    data_.right_distance += right;

    if (deltaTimeMs > 0) {
        const double dt = static_cast<double>(deltaTimeMs) / 1000.0;
        data_.linear_velocity = distance / dt;
        data_.angular_velocity = deltaHeading / dt;
    }

    return true;
}

std::pair<std::int16_t, std::int16_t> KobukiProtocol::velocityToSpeedRadius(
    double vx, double wz) {
    constexpr double epsilon = 0.0001;
    double speed = 0.0;
    double radius = 0.0;

    // Same conversion used by kobuki_core DiffDrive.
    if (std::abs(wz) < epsilon) {
        radius = 0.0;
        speed = 1000.0 * vx;
    } else {
        radius = vx * 1000.0 / wz;
        if (std::abs(vx) < epsilon || std::abs(radius) <= 1.0) {
            speed = 1000.0 * WheelBaseMeters * wz / 2.0;
            radius = 1.0;
        } else if (radius > 0.0) {
            speed = (radius + 1000.0 * WheelBaseMeters / 2.0) * wz;
        } else {
            speed = (radius - 1000.0 * WheelBaseMeters / 2.0) * wz;
        }
    }

    return { clampToInt16(speed), clampToInt16(radius) };
}

std::vector<std::uint8_t> KobukiProtocol::makeBaseControlPacket(double vx, double wz) {
    const auto [speed, radius] = velocityToSpeedRadius(vx, wz);

    std::vector<std::uint8_t> packet;
    packet.reserve(10);
    packet.push_back(0xAA);
    packet.push_back(0x55);
    packet.push_back(0x06); // payload bytes
    packet.push_back(0x01); // Base Control sub-payload id
    packet.push_back(0x04); // data length
    pushInt16LE(packet, speed);
    pushInt16LE(packet, radius);

    std::uint8_t checksum = 0;
    for (std::size_t i = 2; i < packet.size(); ++i) checksum ^= packet[i];
    packet.push_back(checksum);
    return packet;
}

void KobukiProtocol::append(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) return;
    buffer_.insert(buffer_.end(), data, data + size);
}

std::vector<std::vector<std::uint8_t>> KobukiProtocol::extractPayloads() {
    std::vector<std::vector<std::uint8_t>> payloads;

    while (buffer_.size() >= 4) {
        std::size_t header = 0;
        while (header + 1 < buffer_.size() &&
               !(buffer_[header] == 0xAA && buffer_[header + 1] == 0x55)) {
            ++header;
        }

        if (header > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header));
        }
        if (buffer_.size() < 4) break;
        if (!(buffer_[0] == 0xAA && buffer_[1] == 0x55)) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        const std::size_t payloadLength = buffer_[2];
        const std::size_t totalLength = payloadLength + 4; // AA 55 LEN PAYLOAD CHECKSUM
        if (buffer_.size() < totalLength) break;

        std::uint8_t checksum = 0;
        for (std::size_t i = 2; i < totalLength; ++i) checksum ^= buffer_[i];
        if (checksum != 0) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        payloads.emplace_back(buffer_.begin() + 3,
                              buffer_.begin() + 3 + static_cast<std::ptrdiff_t>(payloadLength));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(totalLength));
    }

    return payloads;
}

bool KobukiProtocol::parseCoreSensors(const std::vector<std::uint8_t>& payload, CoreSensors::Data& out) {
    std::size_t i = 0;
    while (i + 2 <= payload.size()) {
        const std::uint8_t id = payload[i];
        const std::size_t length = payload[i + 1];
        const std::size_t dataStart = i + 2;
        const std::size_t dataEnd = dataStart + length;
        if (dataEnd > payload.size()) return false;

        if (id == 0x01 && length >= 15) {
            const std::uint8_t* d = payload.data() + dataStart;
            out.time_stamp = readUInt16LE(d + 0);
            out.bumper = d[2];
            out.wheel_drop = d[3];
            out.cliff = d[4];
            out.left_encoder = readUInt16LE(d + 5);
            out.right_encoder = readUInt16LE(d + 7);
            out.left_pwm = static_cast<std::int8_t>(d[9]);
            out.right_pwm = static_cast<std::int8_t>(d[10]);
            out.buttons = d[11];
            out.charger = d[12];
            out.battery = d[13];
            out.overcurrent = d[14];
            return true;
        }
        i = dataEnd;
    }
    return false;
}

bool KobukiProtocol::parseInertia(const std::vector<std::uint8_t>& payload, Inertia::Data& out) {
    std::size_t i = 0;
    while (i + 2 <= payload.size()) {
        const std::uint8_t id = payload[i];
        const std::size_t length = payload[i + 1];
        const std::size_t dataStart = i + 2;
        const std::size_t dataEnd = dataStart + length;
        if (dataEnd > payload.size()) return false;

        if (id == 0x04 && length >= 7) {
            const std::uint8_t* d = payload.data() + dataStart;
            out.angle = readInt16LE(d + 0);
            out.angle_rate = readInt16LE(d + 2);
            out.acc[0] = d[4];
            out.acc[1] = d[5];
            out.acc[2] = d[6];
            return true;
        }
        i = dataEnd;
    }
    return false;
}

} // namespace kobuki
