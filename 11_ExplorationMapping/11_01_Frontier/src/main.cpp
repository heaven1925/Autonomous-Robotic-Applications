// 11_01 Frontier - nearest frontier exploration (the method of the lecture).
// Everything except the decision rule is common (../common): Bayes grid,
// sensor model, robot / simulator, window, calibration, metrics.
//   Explore_Frontier.exe --sim [--map ../obstacleMapReferences/map2/map2.txt] [--seed 1]
//   Explore_Frontier.exe <Kobuki COM> <Arduino COM> [--map ...]
//   or: ExploreLauncher.exe / .\run.ps1 11 -Algo frontier -Sim
#include "ExploreApp.h"
#include "FrontierStrategy.h"

int main(int argc, char** argv) {
    return app::run(argc, argv, [](const app::Options& opt) -> std::unique_ptr<explore::Strategy> {
        (void)opt;
        return std::make_unique<explore::FrontierStrategy>();
    });
}
