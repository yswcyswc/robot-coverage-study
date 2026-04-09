#include "map_io.h"
#include "test.h"
#include "frontier_planner.h"
#include "random_planner.h"
#include "lawnmower.h"
// #include "stc_planner.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: ./coverage_demo <map_file> <frontier|random|lawnmower|stc>\n";
        return 1;
    }

    try {
        const fs::path map_path(argv[1]);
        std::string algo_choice = argv[2];
        
        GridMap grid_map = loadMap(map_path.string());
        CoverageResult result;

        // 4 algorithms
        if (algo_choice == "frontier") {
            result = runFrontierCoverage(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else if (algo_choice == "random") {
            result = runRandomTraversal(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells, 500);
        } else if (algo_choice == "lawnmower") {
            result = runLawnmowerTraversal(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        // } else if (algo_choice == "stc") {
        //     result = runSTCCoverage(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else {
            std::cerr << "Unknown algorithm: " << algo_choice << "\n";
            return 1;
        }

        // Output handling
        fs::create_directories(fs::path("outputs") / "trajectories");
        fs::path trajectory_path = fs::path("outputs") / "trajectories" /
                                   (map_path.stem().string() + "_" + algo_choice + "_trajectory.csv");

        std::ofstream output(trajectory_path);
        output << "step,row,col\n";
        for (size_t i = 0; i < result.trajectory.size(); ++i) {
            output << i << "," << result.trajectory[i].row << "," << result.trajectory[i].col << "\n";
        }

        std::cout << "----------------------------\n";
        std::cout << "Map: " << map_path.string() << "\n";
        std::cout << "Algorithm: " << algo_choice << "\n";
        std::cout << "Total steps: " << result.total_steps << "\n";
        std::cout << "Trajectory: " << trajectory_path.string() << "\n";
        std::cout << "----------------------------\n";

    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
