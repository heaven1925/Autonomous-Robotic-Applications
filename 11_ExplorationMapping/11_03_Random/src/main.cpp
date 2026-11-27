// 11_03 Random - random reachable goals + 360 deg look-arounds (baseline).
// Everything except the decision rule is common (../common): Bayes grid,
// sensor model, robot / simulator, window, calibration, metrics.
//   Explore_Random.exe --sim [--map ../obstacleMapReferences/map2/map2.txt] [--seed 1]
//   Explore_Random.exe <Kobuki COM> <Arduino COM> [--map ...]
//   or: ExploreLauncher.exe / .\run.ps1 11 -Algo random -Sim
#include "ExploreApp.h"
#include "RandomStrategy.h"

int main(int argc, char** argv) {
    return app::run(argc, argv, [](const app::Options& opt) -> std::unique_ptr<explore::Strategy> {
        (void)opt;
        return std::make_unique<explore::RandomStrategy>(opt.seed);
    });
}
