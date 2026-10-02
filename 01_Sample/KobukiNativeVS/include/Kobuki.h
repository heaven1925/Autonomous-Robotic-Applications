#pragma once

#include "KobukiProtocol.h"
#include "SerialPort.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace kobuki {

struct Parameters {
    std::string device_port = "COM3";
    unsigned int command_rate_hz = 20;
};

class Kobuki {
public:
    Kobuki();
    ~Kobuki();

    Kobuki(const Kobuki&) = delete;
    Kobuki& operator=(const Kobuki&) = delete;

    // API intentionally resembles kobuki_core for teaching purposes.
    void init(Parameters& parameters);
    bool enable();
    bool disable();

    bool isEnabled() const { return enabled_.load(); }
    bool isAlive() const;
    bool isConnected() const { return serial_.isOpen(); }

    void setBaseControl(const double& linearVelocity, const double& angularVelocity);

    CoreSensors::Data getCoreSensorData() const;
    Inertia::Data getInertiaData() const;

    // Factory-calibrated gyro values, relative to the most recent reset.
    double getHeading() const;          // radians
    double getAngularVelocity() const;  // rad/s

    // Encoder-based differential-drive odometry.
    OdometryData getOdometryData() const;
    void resetOdometry();

private:
    void readerLoop();
    void commandLoop();
    void sendCurrentCommand(bool forceStop = false);

    Parameters parameters_;
    SerialPort serial_;
    KobukiProtocol protocol_;

    std::atomic<bool> running_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<std::uint64_t> lastRxMilliseconds_{0};

    mutable std::mutex dataMutex_;
    CoreSensors::Data coreSensors_{};
    Inertia::Data inertia_{};
    OdometryEstimator odometry_{};
    bool gyroInitialized_ = false;
    double headingOffsetRadians_ = 0.0;

    std::mutex commandMutex_;
    double targetLinearVelocity_ = 0.0;
    double targetAngularVelocity_ = 0.0;
    std::mutex writeMutex_;

    std::thread readerThread_;
    std::thread commandThread_;
};

} // namespace kobuki
