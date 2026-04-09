#include "random_planner.h"
#include <random>
#include <algorithm>

CoverageResult runRandomTraversal(int rows, int cols, Point start, const std::vector<std::string>& grid, int max_steps) {
    CoverageResult result;
    Point current = start;
    result.trajectory.push_back(current);

    // random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    for (int step = 0; step < max_steps; ++step) {
        std::vector<Point> neighbors;

        // find all valid adjacent empty cells
        for (int i = 0; i < 4; ++i) {
            int nr = current.row + dr[i];
            int nc = current.col + dc[i];

            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc] == '.') {
                neighbors.push_back({nr, nc});
            }
        }

        if (neighbors.empty()) break; // Trapped

        // pick one neighbor at random
        std::uniform_int_distribution<> dis(0, neighbors.size() - 1);
        current = neighbors[dis(gen)];
        
        result.trajectory.push_back(current);
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}