#include "frontier_planner.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <set>

// Helper to convert 2D coordinates to a 1D index for flat vectors
inline int getIdx(int r, int c, int cols) {
    return r * cols + c;
}

// A* Search to find the shortest path from 'current' to 'target'
std::vector<Point> planPathAStarOptimized(Point current, Point target, int rows, int cols, const std::vector<std::string>& grid) {
    int total_cells = rows * cols;
    std::vector<int> gScore(total_cells, 1e9);
    std::vector<int> parent(total_cells, -1);
    
    // priority queue stores {fScore, index}
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

    int startIdx = getIdx(current.row, current.col, cols);
    int targetIdx = getIdx(target.row, target.col, cols);

    gScore[startIdx] = 0;
    pq.push({std::abs(current.row - target.row) + std::abs(current.col - target.col), startIdx});

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!pq.empty()) {
        int currIdx = pq.top().second;
        pq.pop();

        if (currIdx == targetIdx) {
            std::vector<Point> path;
            int temp = currIdx;
            while (temp != startIdx) {
                path.push_back({temp / cols, temp % cols});
                temp = parent[temp];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        int r = currIdx / cols;
        int c = currIdx % cols;

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc] == '.') {
                int neighborIdx = getIdx(nr, nc, cols);
                int tentative_g = gScore[currIdx] + 1;

                if (tentative_g < gScore[neighborIdx]) {
                    gScore[neighborIdx] = tentative_g;
                    parent[neighborIdx] = currIdx;
                    int fScore = tentative_g + std::abs(nr - target.row) + std::abs(nc - target.col);
                    pq.push({fScore, neighborIdx});
                }
            }
        }
    }
    return {};
}

// BFS to find the true nearest frontier
Point findNearestFrontierBFS(Point start, int rows, int cols, const std::vector<std::string>& grid, const std::vector<bool>& visited) {
    std::queue<int> q;
    std::vector<bool> explored(rows * cols, false);
    
    int startIdx = getIdx(start.row, start.col, cols);
    q.push(startIdx);
    explored[startIdx] = true;

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!q.empty()) {
        int currIdx = q.front();
        q.pop();
        int r = currIdx / cols;
        int c = currIdx % cols;

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                int nIdx = getIdx(nr, nc, cols);
                if (grid[nr][nc] == '.' && !explored[nIdx]) {
                    // if neighbor is unvisited, the current cell (r,c) is a frontier
                    if (!visited[nIdx]) {
                        return {nr, nc};
                    }
                    explored[nIdx] = true;
                    q.push(nIdx);
                }
            }
        }
    }
    return {-1, -1}; // no more frontiers
}

CoverageResult runFrontierCoverage(int rows, int cols, Point start, const std::vector<std::string>& grid) {
    CoverageResult result;
    std::vector<bool> visited(rows * cols, false);
    Point current = start;
    
    visited[getIdx(current.row, current.col, cols)] = true;
    result.trajectory.push_back(current);

    while (true) {
        Point nearest = findNearestFrontierBFS(current, rows, cols, grid, visited);
        
        if (nearest.row == -1) break;

        std::vector<Point> path = planPathAStarOptimized(current, nearest, rows, cols, grid);
        
        for (const auto& p : path) {
            result.trajectory.push_back(p);
            visited[getIdx(p.row, p.col, cols)] = true;
            current = p;
        }
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}