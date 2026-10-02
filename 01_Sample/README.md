# Kobuki Native Windows - CMake / VS Code

This project controls a Kobuki base directly from a native Windows C++ program.
It does **not** require ROS, WSL, ECL, colcon, or `kobuki_core`.

It implements the documented Kobuki USB/serial protocol and exposes a small API that resembles `kobuki_core` for teaching.

## New: odometry and gyro

The project now also reads Kobuki's factory-calibrated inertial sensor packet and calculates differential-drive odometry from the wheel encoders.

```cpp
kobuki::Parameters parameters;
parameters.device_port = "COM4";

kobuki::Kobuki robot;
robot.init(parameters);
robot.enable();

robot.setBaseControl(0.15, 0.0);

auto sensors = robot.getCoreSensorData();
auto gyro = robot.getInertiaData();
auto odom = robot.getOdometryData();

double gyroHeading = robot.getHeading();          // rad
double gyroRate = robot.getAngularVelocity();     // rad/s

robot.resetOdometry();
robot.setBaseControl(0.0, 0.0);
robot.disable();
```

`OdometryData` contains:

- `x`, `y` in metres
- `heading` in radians (encoder-derived)
- `linear_velocity` in m/s
- `angular_velocity` in rad/s (encoder-derived)
- `left_distance`, `right_distance` in metres

The gyro API contains:

- raw `angle` and `angle_rate`
- `angleDegrees()`
- `angularVelocityDegreesPerSecond()`
- `angleRadians()`
- `angularVelocityRadiansPerSecond()`
- `robot.getHeading()` returns gyro heading relative to init/reset
- `robot.getAngularVelocity()` returns calibrated z-axis gyro rate in rad/s

## Requirements

- Windows 10/11
- Visual Studio 2026 **Build Tools** (or full IDE) with the C++ workload (MSVC v145 toolset, Windows SDK)
- CMake 3.21+ and Ninja (both ship with the VS C++ workload)
- Visual Studio Code with the **C/C++** and **CMake Tools** extensions
- Kobuki connected by USB and powered on
- FTDI USB serial driver (Windows commonly installs it automatically)

## Build (VS Code + CMake)

The whole workspace builds from the repository root (`AutonRobotSysWS/CMakeLists.txt`);
every top-level folder containing a `CMakeLists.txt` is picked up automatically.

1. Open the workspace folder in VS Code and install the recommended extensions (C/C++, CMake Tools).
2. Select the configure preset `x64-debug` (or `x64-release`) when prompted, and a Visual Studio amd64 kit.
3. Run **CMake: Build** (F7). The executable is written to `out/build/<preset>/01_Sample/KobukiNativeVS.exe`.

### One-command build + run (recommended)

From the repository root:

```text
.\run.ps1                          # build + run KobukiNativeVS, auto-detect robot COM port
.\run.ps1 KobukiNativeVS -Port COM4
.\run.ps1 -Config release
.\run.ps1 -ListPorts               # show available COM ports
```

In VS Code: **Terminal -> Run Task -> "Robot: Build & Run (auto COM)"** (or Ctrl+Shift+B).

Command-line alternative (from a *Developer Command Prompt* / after `vcvars64.bat`):

```text
cmake --preset x64-debug
cmake --build --preset x64-debug
```

## Run

1. Connect and power on the Kobuki.
2. Run `.\run.ps1` from the repository root - the FTDI COM port is detected automatically.
3. Or run the executable manually and enter the COM port when asked:

```text
out\build\x64-debug\01_Sample\KobukiNativeVS.exe COM4
```

## Controls

- `W` forward
- `S` backward
- `A` rotate left
- `D` rotate right
- `X` or Space stop
- `R` print current basic sensor data
- `G` print gyro / heading
- `O` print encoder odometry
- `Z` reset encoder odometry and gyro heading zero
- `Q` stop and exit

## Protocol implemented

- Serial: 115200 baud, 8 data bits, 1 stop bit, no parity
- Packet header: `AA 55`
- XOR checksum
- Base Control command (`0x01`)
- Basic Sensor Data feedback (`0x01`, 15 data bytes)
- Inertial Sensor feedback (`0x04`, 7 data bytes)
- Linear/angular velocity conversion matching Kobuki's DiffDrive logic
- Wheelbase: 230 mm
- Encoder conversion: 0.00008529209049773756 m/tick
- Gyro angle/rate conversion: 0.01 degree / 0.01 degree-per-second units -> SI units

References:
- https://kobuki.readthedocs.io/en/devel/protocol.html
- https://kobuki.readthedocs.io/en/devel/conversions.html

## Important distinction

This is a native Windows educational driver built around Kobuki's documented serial protocol. It is **not** the official `kobuki_core` library. This avoids the Linux/ECL dependency chain while keeping the application-facing API familiar.

## Safety

For the first test, place the robot in a clear area or lift the drive wheels off the ground. Press `X`/Space to stop before quitting. The destructor and `Q` path also send a stop command.
