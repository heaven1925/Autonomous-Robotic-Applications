// Example 03 - waypoint navigation with console input.
// Two target points are read from the console; the robot drives to
// point 1, then to point 2. The angle to each target is computed and
// printed by goTo().
#include "Motion.h"
#include <conio.h>

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
    std::cout << "Example 03 - drive to two waypoints\n\n";

    // 1. Get the two target points first (robot is not moving yet).
    double x1, y1, x2, y2, x3, y3;
    readPoint("Point 1", x1, y1);
    readPoint("Point 2", x2, y2);
    readPoint("Point 3", x3, y3);

    // 2. Connect and run the mission.
    kobuki::Kobuki robot;
    connect(robot, argc, argv, "");

    // ---- tuning: play with these to match reality ----
    cfg.drive_speed  = 0.15;  // m/s
    cfg.turn_speed   = 0.60;  // rad/s
    cfg.turn_tol_deg = 0.7;   // deg

    std::cout << "\n[1/2] Go to (" << x1 << ", " << y1 << ")\n";
    goTo(robot, x1, y1);
    printPose(robot, "  pose");

    //std::cout << "\nPress 'a' to go to point 2...\n";
    //while (_getch() != 'a') {}

    std::cout << "\n[2/2] Go to (" << x2 << ", " << y2 << ")\n";
    goTo(robot, x2, y2);
    printPose(robot, "  pose");

    std::cout << "\n[3/3] Go to (" << x3 << ", " << y3 << ")\n";
    goTo(robot, x3, y3);
    printPose(robot, "  pose");

    stop(robot);
    std::cout << "\nDone.\n";
    printPose(robot, "FINAL");
    return 0;
}
