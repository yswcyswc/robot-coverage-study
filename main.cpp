#include "map_io.h"
#include "test.h"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: coverage_demo <map_file>\n";
        return 1;
    }

    try {
        const fs::path map_path(argv[1]);
        GridMap grid_map = loadMap(map_path.string());
        CoverageResult result = runTestTraversal(grid_map.rows, grid_map.cols, grid_map.start);

        fs::create_directories("outputs");
        fs::path trajectory_path = fs::path("outputs") / (map_path.stem().string() + "_trajectory.csv");

        std::ofstream output(trajectory_path);
        output << "step,row,col\n";
        for (size_t i = 0; i < result.trajectory.size(); ++i) {
            output << i << "," << result.trajectory[i].row << "," << result.trajectory[i].col << "\n";
        }

        std::cout << "Map: " << map_path.string() << "\n";
        std::cout << "Trajectory saved to: " << trajectory_path.string() << "\n";
        std::cout << "Algorithm: test\n";
        std::cout << "Total steps: " << result.total_steps << "\n";
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
