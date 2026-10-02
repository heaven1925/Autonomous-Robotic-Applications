// Example 01 - drive a 1 m square, stop back at the start.
#include "Motion.h"

using namespace kobuki::motion;

int main(int argc, char** argv) {
    kobuki::Kobuki robot;
    connect(robot, argc, argv, "Example 01 - 1 m square");

    // ---- tuning: play with these to match reality ----
    cfg.drive_speed  = 0.40;  // m/s
    cfg.turn_speed   = 0.60;  // rad/s
    cfg.turn_tol_deg = 0.9;   // deg

    const double side = 1.0;  // m

    for (int i = 0; i < 4; ++i) {
        std::cout << "\n[Side " << i + 1 << "/4]\n";
        driveStraight(robot, side, normalize(i * 90 * Deg));   // 0 / 90 / 180 / 270
        turnTo(robot, normalize((i + 1) * 90 * Deg));
        printPose(robot, "  pose");
    }

    stop(robot);
    std::cout << "\nSquare complete.\n";
    printPose(robot, "FINAL");
    printClosure(robot);
    return 0;
}
