#include "frontier_planner.h"
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

// check if a cell is valid and not an obstacle
bool isValid(int r, int c, int rows, int cols, const std::vector<std::string>& grid) {
    return r >= 0 && r < rows && c >= 0 && c < cols && grid[r][c] == '.';
}

// A* Search to find the shortest path from 'current' to 'target'
std::vector<Point> planPathAStar(Point current, Point target, int rows, int cols, const std::vector<std::string>& grid) {
    auto dist = [](Point a, Point b) { return std::abs(a.row - b.row) + std::abs(a.col - b.col); };
    
    std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;
    std::map<std::pair<int, int>, int> gScore;

    pq.push({0, {current.row, current.col}});
    gScore[{current.row, current.col}] = 0;

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!pq.empty()) {
        auto [row, col] = pq.top().second;
        pq.pop();

        if (row == target.row && col == target.col) {
            std::vector<Point> path;
            std::pair<int, int> curr = {row, col};
            while (curr.first != current.row || curr.second != current.col) {
                path.push_back({curr.first, curr.second});
                curr = parent[curr];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 4; ++i) {
            int nr = row + dr[i], nc = col + dc[i];
            if (isValid(nr, nc, rows, cols, grid)) {
                int tentative_g = gScore[{row, col}] + 1;
                if (gScore.find({nr, nc}) == gScore.end() || tentative_g < gScore[{nr, nc}]) {
                    gScore[{nr, nc}] = tentative_g;
                    parent[{nr, nc}] = {row, col};
                    pq.push({tentative_g + dist({nr, nc}, target), {nr, nc}});
                }
            }
        }
    }
    return {};
}

CoverageResult runFrontierCoverage(int rows, int cols, Point start, const std::vector<std::string>& grid) {
    CoverageResult result;
    std::set<std::pair<int, int>> visited;
    Point current = start;
    
    visited.insert({current.row, current.col});
    result.trajectory.push_back(current);

    while (true) {
        std::vector<Point> frontiers;
        int dr[] = {0, 0, 1, -1};
        int dc[] = {1, -1, 0, 0};

        // find frontier cells
        for (auto const& [vr, vc] : visited) {
            for (int i = 0; i < 4; ++i) {
                int nr = vr + dr[i], nc = vc + dc[i];
                if (isValid(nr, nc, rows, cols, grid) && visited.find({nr, nc}) == visited.end()) {
                    frontiers.push_back({nr, nc});
                }
            }
        }

        if (frontiers.empty()) break; // All reachable cells visited

        // select nearest frontier
        Point nearest = frontiers[0];
        int minDist = 1e9;
        for (const auto& f : frontiers) {
            int d = std::abs(f.row - current.row) + std::abs(f.col - current.col);
            if (d < minDist) {
                minDist = d;
                nearest = f;
            }
        }

        // plan path
        std::vector<Point> path = planPathAStar(current, nearest, rows, cols, grid);
        for (const auto& p : path) {
            result.trajectory.push_back(p);
            visited.insert({p.row, p.col});
            current = p;
        }
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}