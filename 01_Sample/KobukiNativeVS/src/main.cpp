#include "Kobuki.h"
#include "SerialPort.h"

#include <conio.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr double RadToDeg = 57.2957795130823208768;

void printSensors(const kobuki::CoreSensors::Data& d) {
    std::cout << "\n--- Core Sensors ---\n"
              << "Timestamp      : " << d.time_stamp << " ms\n"
              << "Left encoder   : " << d.left_encoder << "\n"
              << "Right encoder  : " << d.right_encoder << "\n"
              << "Battery        : " << std::fixed << std::setprecision(1) << d.batteryVoltage() << " V\n"
              << "Bumper L/C/R   : " << d.bumperLeft() << "/" << d.bumperCenter() << "/" << d.bumperRight() << "\n"
              << "Cliff  L/C/R   : " << d.cliffLeft() << "/" << d.cliffCenter() << "/" << d.cliffRight() << "\n"
              << "Wheel drop     : " << static_cast<int>(d.wheel_drop) << "\n"
              << "Buttons        : " << static_cast<int>(d.buttons) << "\n"
              << "Charger state  : " << static_cast<int>(d.charger) << "\n"
              << "Overcurrent    : " << static_cast<int>(d.overcurrent) << "\n";
}

void printGyro(const kobuki::Kobuki& robot) {
    const auto g = robot.getInertiaData();
    std::cout << "\n--- Gyro / Inertial Sensor ---\n"
              << std::fixed << std::setprecision(4)
              << "Raw angle      : " << g.angle << " (0.01 deg units)\n"
              << "Raw angle rate : " << g.angle_rate << " (0.01 deg/s units)\n"
              << "Angle          : " << g.angleDegrees() << " deg\n"
              << "Angle rate     : " << g.angularVelocityDegreesPerSecond() << " deg/s\n"
              << "Heading        : " << robot.getHeading() << " rad ("
              << robot.getHeading() * RadToDeg << " deg, zeroed at init/reset)\n"
              << "Angular vel.   : " << robot.getAngularVelocity() << " rad/s\n";
}

void printOdometry(const kobuki::OdometryData& o) {
    std::cout << "\n--- Encoder Odometry ---\n"
              << std::fixed << std::setprecision(4)
              << "x              : " << o.x << " m\n"
              << "y              : " << o.y << " m\n"
              << "heading        : " << o.heading << " rad (" << o.heading * RadToDeg << " deg)\n"
              << "linear vel.    : " << o.linear_velocity << " m/s\n"
              << "angular vel.   : " << o.angular_velocity << " rad/s\n"
              << "left distance  : " << o.left_distance << " m\n"
              << "right distance : " << o.right_distance << " m\n";
}

std::string choosePort(int argc, char** argv) {
    if (argc >= 2) return argv[1];

    const auto ports = kobuki::SerialPort::listPorts();
    std::cout << "Detected COM ports:";
    if (ports.empty()) std::cout << " none";
    std::cout << "\n";
    for (const auto& p : ports) std::cout << "  " << p << "\n";

    std::cout << "\nEnter Kobuki COM port (example COM4): ";
    std::string port;
    std::cin >> port;
    return port;
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "Kobuki Native Windows / Visual Studio Demo\n"
              << "No ROS, WSL, ECL or external libraries required.\n\n";

    try {
        const std::string port = choosePort(argc, argv);

        kobuki::Parameters parameters;
        parameters.device_port = port;
        parameters.command_rate_hz = 20;

        kobuki::Kobuki robot;
        std::cout << "Opening " << port << " at 115200 8N1...\n";
        robot.init(parameters);
        robot.enable();

        std::cout << "Waiting for Kobuki sensor stream...\n";
        for (int i = 0; i < 30 && !robot.isAlive(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (robot.isAlive()) {
            std::cout << "Kobuki is streaming sensor data.\n";
            printSensors(robot.getCoreSensorData());
            printGyro(robot);
            printOdometry(robot.getOdometryData());
        } else {
            std::cout << "WARNING: COM port opened, but no valid Kobuki sensor packet was received.\n"
                      << "Check the COM port, USB cable, FTDI driver, and robot power.\n";
        }

        std::cout << "\nControls:\n"
                  << "  W : forward  (0.15 m/s)\n"
                  << "  S : backward (-0.15 m/s)\n"
                  << "  A : rotate left  (0.8 rad/s)\n"
                  << "  D : rotate right (-0.8 rad/s)\n"
                  << "  X or SPACE : stop\n"
                  << "  R : print core sensors\n"
                  << "  G : print gyro / heading\n"
                  << "  O : print encoder odometry\n"
                  << "  Z : reset odometry and gyro heading zero\n"
                  << "  Q : stop and quit\n\n"
                  << "For the first test, keep the robot wheels off the ground or place it in a clear area.\n";

        while (true) {
            const int key = _getch();
            switch (key) {
            case 'w': case 'W':
                robot.setBaseControl(0.15, 0.0);
                std::cout << "FORWARD\n";
                break;
            case 's': case 'S':
                robot.setBaseControl(-0.15, 0.0);
                std::cout << "BACKWARD\n";
                break;
            case 'a': case 'A':
                robot.setBaseControl(0.0, 0.8);
                std::cout << "ROTATE LEFT\n";
                break;
            case 'd': case 'D':
                robot.setBaseControl(0.0, -0.8);
                std::cout << "ROTATE RIGHT\n";
                break;
            case 'x': case 'X': case ' ':
                robot.setBaseControl(0.0, 0.0);
                std::cout << "STOP\n";
                break;
            case 'r': case 'R':
                printSensors(robot.getCoreSensorData());
                break;
            case 'g': case 'G':
                printGyro(robot);
                break;
            case 'o': case 'O':
                printOdometry(robot.getOdometryData());
                break;
            case 'z': case 'Z':
                robot.resetOdometry();
                std::cout << "ODOMETRY + GYRO HEADING RESET\n";
                break;
            case 'q': case 'Q':
                robot.disable();
                std::cout << "STOP / EXIT\n";
                return 0;
            default:
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
}
