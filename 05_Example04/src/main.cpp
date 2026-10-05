// Example 04 - waypoint navigation with bumper obstacle avoidance.
// Same as Example 03, but an obstacle may be in the way: on a bump the
// robot wall-follows with a repeating trapezoid (45 deg out, parallel,
// 45 deg back in) until the obstacle ends, then re-aims at the target.
#include "Motion.h"

using namespace kobuki::motion;

// Ask the user for one coordinate pair, e.g. "2 2" or "0.5 1".
static void readPoint(const char* name, double& x, double& y) {
    std::cout << name << " (x y) [m]: ";
    while (!(std::cin >> x >> y)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "  Invalid input. Enter two numbers, e.g.: 2 1.5\n"
                  << name << " (x y) [m]: ";
    }
}

int main(int argc, char** argv) {
    std::cout << "Example 04 - waypoints with obstacle avoidance\n\n";

    double x1, y1, x2, y2;
    readPoint("Point 1", x1, y1);
    readPoint("Point 2", x2, y2);

    kobuki::Kobuki robot;
    connect(robot, argc, argv, "");

    // ---- tuning ----
    cfg.drive_speed    = 0.20;  // m/s
    cfg.turn_speed     = 0.80;  // rad/s
    cfg.turn_tol_deg   = 0.7;   // deg
    cfg.avoid_angle    = 135.0; // deg - turn at the bump point
    cfg.avoid_distance = 0.30;  // m   - diagonal leg
    cfg.pass_distance  = 0.30;  // m   - straight leg, then re-aim

    std::cout << "\n[1/2] Go to (" << x1 << ", " << y1 << ")\n";
    goToAvoid(robot, x1, y1);
    printPose(robot, "  pose");

    //std::cout << "\n[2/2] Go to (" << x2 << ", " << y2 << ")\n";
    //goToAvoid(robot, x2, y2);
    //printPose(robot, "  pose");

    stop(robot);
    std::cout << "\nDone.\n";
    printPose(robot, "FINAL");
    return 0;
}
