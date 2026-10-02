// Example 02 - forward 2 m, spin 360 deg, return to the start point.
#include "Motion.h"

using namespace kobuki::motion;

int main(int argc, char** argv) {
    kobuki::Kobuki robot;
    connect(robot, argc, argv, "Example 02 - forward 2 m, spin 360, return");

    // ---- tuning: play with these to match reality ----
    cfg.drive_speed  = 0.15;  // m/s
    cfg.turn_speed   = 0.60;  // rad/s
    cfg.turn_tol_deg = 0.7;   // deg

    std::cout << "\n[1/3] Forward 2 m\n";
    driveStraight(robot, 2.0, 0.0);
    printPose(robot, "  pose");

    std::cout << "\n[2/3] Spin 360 deg\n";
    turnBy(robot, 360 * Deg);
    printPose(robot, "  pose");

    std::cout << "\n[3/3] Return to start\n";
    goTo(robot, 0.0, 0.0);

    stop(robot);
    std::cout << "\nDone.\n";
    printPose(robot, "FINAL");
    printClosure(robot);
    return 0;
}
