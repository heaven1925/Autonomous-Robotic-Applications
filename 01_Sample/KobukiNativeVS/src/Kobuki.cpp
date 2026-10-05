#include "Kobuki.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace kobuki {
namespace {
constexpr double Pi = 3.141592653589793238462643383279502884;

std::uint64_t nowMilliseconds() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

double normalizeAngle(double angle) {
    while (angle > Pi) angle -= 2.0 * Pi;
    while (angle <= -Pi) angle += 2.0 * Pi;
    return angle;
}
}

Kobuki::Kobuki() = default;

Kobuki::~Kobuki() {
    try { disable(); } catch (...) {}
    running_.store(false);
    if (readerThread_.joinable()) readerThread_.join();
    if (commandThread_.joinable()) commandThread_.join();
    serial_.close();
}

void Kobuki::init(Parameters& parameters) {
    if (running_.load()) throw std::runtime_error("Kobuki is already initialized.");
    if (parameters.command_rate_hz == 0 || parameters.command_rate_hz > 100) {
        throw std::runtime_error("command_rate_hz must be between 1 and 100.");
    }

    parameters_ = parameters;
    serial_.open(parameters_.device_port);
    running_.store(true);
    enabled_.store(false);
    lastRxMilliseconds_.store(0);

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        coreSensors_ = CoreSensors::Data{};
        inertia_ = Inertia::Data{};
        odometry_.reset();
        gyroInitialized_ = false;
        headingOffsetRadians_ = 0.0;
    }

    readerThread_ = std::thread(&Kobuki::readerLoop, this);
    commandThread_ = std::thread(&Kobuki::commandLoop, this);
}

bool Kobuki::enable() {
    if (!serial_.isOpen()) return false;
    enabled_.store(true);
    return true;
}

bool Kobuki::disable() {
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        targetLinearVelocity_ = 0.0;
        targetAngularVelocity_ = 0.0;
    }
    enabled_.store(false);
    if (serial_.isOpen()) {
        try { sendCurrentCommand(true); } catch (...) { return false; }
    }
    return true;
}

bool Kobuki::isAlive() const {
    const std::uint64_t last = lastRxMilliseconds_.load();
    if (last == 0) return false;
    return nowMilliseconds() - last < 500;
}

void Kobuki::setBaseControl(const double& linearVelocity, const double& angularVelocity) {
    std::lock_guard<std::mutex> lock(commandMutex_);
    targetLinearVelocity_ = linearVelocity;
    targetAngularVelocity_ = angularVelocity;
}

CoreSensors::Data Kobuki::getCoreSensorData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return coreSensors_;
}

Inertia::Data Kobuki::getInertiaData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return inertia_;
}

double Kobuki::getHeading() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (!gyroInitialized_) return 0.0;
    return normalizeAngle(inertia_.angleRadians() - headingOffsetRadians_);
}

double Kobuki::getAngularVelocity() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return inertia_.angularVelocityRadiansPerSecond();
}

OdometryData Kobuki::getOdometryData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return odometry_.data();
}

void Kobuki::resetOdometry() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    odometry_.reset();
    if (gyroInitialized_) {
        headingOffsetRadians_ = inertia_.angleRadians();
    }
}

void Kobuki::readerLoop() {
    std::array<std::uint8_t, 512> bytes{};
    try {
        while (running_.load()) {
            const std::size_t n = serial_.read(bytes.data(), bytes.size());
            if (n == 0) continue;

            protocol_.append(bytes.data(), n);
            auto payloads = protocol_.extractPayloads();
            for (const auto& payload : payloads) {
                CoreSensors::Data coreData;
                Inertia::Data inertiaData;
                const bool hasCore = KobukiProtocol::parseCoreSensors(payload, coreData);
                const bool hasInertia = KobukiProtocol::parseInertia(payload, inertiaData);
                bool bumpEdge = false;

                if (hasCore || hasInertia) {
                    std::lock_guard<std::mutex> lock(dataMutex_);
                    if (hasCore) {
                        bumpEdge = coreData.bumper != 0 && coreSensors_.bumper == 0;
                        coreSensors_ = coreData;
                        odometry_.update(coreData);
                    }
                    if (hasInertia) {
                        inertia_ = inertiaData;
                        if (!gyroInitialized_) {
                            headingOffsetRadians_ = inertia_.angleRadians();
                            gyroInitialized_ = true;
                        }
                    }
                    lastRxMilliseconds_.store(nowMilliseconds());
                }

                // Emergency stop on the bumper press itself: cut forward
                // speed and send it right now, not on the next command tick.
                if (bumpEdge && parameters_.stop_on_bump) {
                    bool wasForward = false;
                    {
                        std::lock_guard<std::mutex> lock(commandMutex_);
                        if (targetLinearVelocity_ > 0.0) {
                            targetLinearVelocity_ = 0.0;
                            wasForward = true;
                        }
                    }
                    if (wasForward && enabled_.load()) sendCurrentCommand();
                }
            }
        }
    } catch (...) {
        running_.store(false);
        enabled_.store(false);
    }
}

void Kobuki::commandLoop() {
    const auto period = std::chrono::milliseconds(1000 / parameters_.command_rate_hz);
    try {
        while (running_.load()) {
            sendCurrentCommand(!enabled_.load());
            std::this_thread::sleep_for(period);
        }
    } catch (...) {
        running_.store(false);
        enabled_.store(false);
    }
}

void Kobuki::sendCurrentCommand(bool forceStop) {
    double vx = 0.0;
    double wz = 0.0;
    if (!forceStop) {
        std::lock_guard<std::mutex> lock(commandMutex_);
        vx = targetLinearVelocity_;
        wz = targetAngularVelocity_;
    }
    const auto packet = KobukiProtocol::makeBaseControlPacket(vx, wz);
    std::lock_guard<std::mutex> writeLock(writeMutex_);
    serial_.write(packet.data(), packet.size());
}

} // namespace kobuki
