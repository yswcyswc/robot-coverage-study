#include "map_io.h"
#include "test.h"
#include "frontier_planner.h"
#include "random_planner.h"
#include "lawnmower.h"
#include "stc.h"
#include "stc_twostep.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <chrono>

namespace fs = std::filesystem;

int countReachableFreeCells(int rows, int cols, Point start,
                            const std::vector<std::string>& grid) {
    if (start.row < 0 || start.row >= rows || start.col < 0 || start.col >= cols ||
        grid[start.row][start.col] != '.') {
        return 0;
    }

    std::queue<Point> frontier;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    int reachable = 0;

    frontier.push(start);
    visited[start.row][start.col] = true;

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!frontier.empty()) {
        Point curr = frontier.front();
        frontier.pop();
        ++reachable;

        for (int i = 0; i < 4; ++i) {
            int next_row = curr.row + dr[i];
            int next_col = curr.col + dc[i];

            if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols) {
                continue;
            }

            if (grid[next_row][next_col] != '.' || visited[next_row][next_col]) {
                continue;
            }

            visited[next_row][next_col] = true;
            frontier.push({next_row, next_col});
        }
    }

    return reachable;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: ./coverage_demo <map_file> <frontier|random|lawnmower|stc|stc_twostep>\n";
        return 1;
    }

    try {
        const fs::path map_path(argv[1]);
        std::string algo_choice = argv[2];
        
        GridMap grid_map = loadMap(map_path.string());
        CoverageResult result;

        auto start_time = std::chrono::high_resolution_clock::now();

        // Planner selection
        if (algo_choice == "frontier") {
            result = runFrontierCoverage(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else if (algo_choice == "random") {
            result = runRandomTraversal(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells, 500);
        } else if (algo_choice == "lawnmower") {
            result = runLawnmowerTraversal(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else if (algo_choice == "stc") {
            result = runSTCCoverage(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else if (algo_choice == "stc_twostep") {
            result = runSTCTwoStepCoverage(grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);
        } else {
            std::cerr << "Unknown algorithm: " << algo_choice << "\n";
            return 1;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        
        // Calculate planning time (in milliseconds)
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double planning_time_ms = duration.count() / 1000.0;

        int reachable_free_cells = countReachableFreeCells(
            grid_map.rows, grid_map.cols, grid_map.start, grid_map.cells);

        std::set<std::pair<int, int>> unique_visited;
        for (const Point& p : result.trajectory) {
            unique_visited.insert({p.row, p.col});
        }

        int unique_visited_cells = unique_visited.size();
        int revisit_count = result.trajectory.size() - unique_visited_cells;
        double revisit_percentage = 0.0;
        if (!result.trajectory.empty()) {
            revisit_percentage = 100.0 * revisit_count / result.trajectory.size();
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
        std::cout << "Planning Time: " << planning_time_ms << " ms\n"; // New Metric
        std::cout << "Total steps: " << result.total_steps << "\n";
        std::cout << "Reachable free cells: " << reachable_free_cells << "\n";
        std::cout << "Unique visited cells: " << unique_visited_cells << "\n";
        std::cout << "Revisit count: " << revisit_count << "\n";
        std::cout << "Revisit percentage: " << revisit_percentage << "%\n";
        std::cout << "Trajectory: " << trajectory_path.string() << "\n";
        std::cout << "----------------------------\n";

    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
