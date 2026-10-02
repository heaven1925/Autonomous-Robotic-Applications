#include "KobukiProtocol.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

static std::vector<std::uint8_t> wrapPayload(const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> p{0xAA, 0x55, static_cast<std::uint8_t>(payload.size())};
    p.insert(p.end(), payload.begin(), payload.end());
    std::uint8_t cs = 0;
    for (std::size_t i = 2; i < p.size(); ++i) cs ^= p[i];
    p.push_back(cs);
    return p;
}

int main() {
    using kobuki::KobukiProtocol;

    const auto straight = KobukiProtocol::makeBaseControlPacket(0.15, 0.0);
    const std::vector<std::uint8_t> expected{0xAA,0x55,0x06,0x01,0x04,0x96,0x00,0x00,0x00,0x95};
    assert(straight == expected);

    const auto rotate = KobukiProtocol::velocityToSpeedRadius(0.0, 1.0);
    assert(rotate.first == 115);
    assert(rotate.second == 1);

    // One feedback payload containing Basic Sensor Data + Inertial Sensor Data.
    std::vector<std::uint8_t> feedback{
        0x01, 0x0F,
        0x34,0x12, // timestamp
        0x06,      // bumper center+left
        0x00,      // wheel drop
        0x01,      // cliff right
        0x78,0x56, // left encoder
        0xBC,0x9A, // right encoder
        0xF6,      // left pwm -10
        0x14,      // right pwm 20
        0x03,      // buttons
        0x06,      // charger
        0xA7,      // battery 16.7V
        0x02,      // overcurrent
        0x04, 0x07,
        0x28,0x23, // angle = 9000 -> 90.00 deg
        0xE8,0x03, // angle rate = 1000 -> 10.00 deg/s
        0x00,0x00,0x00
    };
    const auto frame = wrapPayload(feedback);

    KobukiProtocol parser;
    parser.append(frame.data(), 3);
    assert(parser.extractPayloads().empty());
    parser.append(frame.data() + 3, frame.size() - 3);
    const auto payloads = parser.extractPayloads();
    assert(payloads.size() == 1);

    kobuki::CoreSensors::Data data;
    assert(KobukiProtocol::parseCoreSensors(payloads[0], data));
    assert(data.time_stamp == 0x1234);
    assert(data.left_encoder == 0x5678);
    assert(data.right_encoder == 0x9ABC);
    assert(data.left_pwm == -10);
    assert(data.batteryVoltage() > 16.69 && data.batteryVoltage() < 16.71);
    assert(data.bumperCenter() && data.bumperLeft() && !data.bumperRight());
    assert(data.cliffRight());

    kobuki::Inertia::Data gyro;
    assert(KobukiProtocol::parseInertia(payloads[0], gyro));
    assert(gyro.angle == 9000);
    assert(gyro.angle_rate == 1000);
    assert(std::abs(gyro.angleDegrees() - 90.0) < 1e-9);
    assert(std::abs(gyro.angularVelocityDegreesPerSecond() - 10.0) < 1e-9);

    kobuki::OdometryEstimator odom;
    kobuki::CoreSensors::Data s0;
    s0.time_stamp = 1000;
    s0.left_encoder = 1000;
    s0.right_encoder = 1000;
    assert(!odom.update(s0)); // first packet establishes the baseline

    kobuki::CoreSensors::Data s1 = s0;
    s1.time_stamp = 2000;
    s1.left_encoder = 1117;
    s1.right_encoder = 1117;
    assert(odom.update(s1));
    const auto forward = odom.data();
    assert(forward.x > 0.0099 && forward.x < 0.0101);
    assert(std::abs(forward.y) < 1e-9);
    assert(std::abs(forward.heading) < 1e-9);
    assert(forward.linear_velocity > 0.0099 && forward.linear_velocity < 0.0101);

    odom.reset();
    s0.time_stamp = 3000;
    s0.left_encoder = 2000;
    s0.right_encoder = 2000;
    assert(!odom.update(s0));
    s1 = s0;
    s1.time_stamp = 4000;
    s1.left_encoder = 1900;
    s1.right_encoder = 2100;
    assert(odom.update(s1));
    const auto turn = odom.data();
    assert(turn.heading > 0.07 && turn.heading < 0.08);
    assert(turn.angular_velocity > 0.07 && turn.angular_velocity < 0.08);

    std::cout << "Protocol, gyro and odometry tests passed.\n";
    return 0;
}
