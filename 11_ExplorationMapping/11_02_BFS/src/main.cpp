// 11_02 BFS - breadth-first exploration order from the start cell.
// Everything except the decision rule is common (../common): Bayes grid,
// sensor model, robot / simulator, window, calibration, metrics.
//   Explore_BFS.exe --sim [--map ../obstacleMapReferences/map2/map2.txt] [--seed 1]
//   Explore_BFS.exe <Kobuki COM> <Arduino COM> [--map ...]
//   or: ExploreLauncher.exe / .\run.ps1 11 -Algo bfs -Sim
#include "ExploreApp.h"
#include "BfsStrategy.h"

int main(int argc, char** argv) {
    return app::run(argc, argv, [](const app::Options& opt) -> std::unique_ptr<explore::Strategy> {
        (void)opt;
        return std::make_unique<explore::BfsStrategy>();
    });
}
