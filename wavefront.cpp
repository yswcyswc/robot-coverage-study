#include "wavefront.h"

#include <queue>
#include <string>
#include <vector>


/*
Planning Paths of Complete Coverage of an Unstructured Environment by a Mobile Robot
Zelinsky, Jarvis, Byrne, Yuta

Pseudocode:
1. Choose a fixed goal cell and build a distance transform from it.
2. Start at the robot start cell.
3. At each cell, inspect unvisited neighbors in descending distance-transform order.
4. Move to the highest-valued unvisited neighbor first.
5. Backtrack only after all higher-priority neighbors have been covered.
*/

bool isFreeWavefrontCell(int row, int col, int r, int c,
                         const std::vector<std::string>& grid) {
    return row >= 0 && row < r && col >= 0 && col < c && grid[row][col] == '.';
}

std::vector<std::vector<int>> buildDistanceTransform(Point goal, int r, int c,
                                                     const std::vector<std::string>& grid) {
    std::vector<std::vector<int>> dist(r, std::vector<int>(c, -1));
    std::queue<Point> frontier;

    frontier.push(goal);
    dist[goal.row][goal.col] = 0;

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!frontier.empty()) {
        Point curr = frontier.front();
        frontier.pop();

        for (int i = 0; i < 4; ++i) {
            int nextR = curr.row + dr[i];
            int nextC = curr.col + dc[i];

            if (!isFreeWavefrontCell(nextR, nextC, r, c, grid) ||
                dist[nextR][nextC] != -1) {
                continue;
            }

            dist[nextR][nextC] = dist[curr.row][curr.col] + 1;
            frontier.push({nextR, nextC});
        }
    }

    return dist;
}

Point findWavefrontGoal(Point start, int r, int c, const std::vector<std::string>& grid) {
    std::queue<Point> frontier;
    std::vector<std::vector<bool>> seen(r, std::vector<bool>(c, false));

    frontier.push(start);
    seen[start.row][start.col] = true;

    Point goal = start;
    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};

    while (!frontier.empty()) {
        Point curr = frontier.front();
        frontier.pop();
        goal = curr;

        for (int i = 0; i < 4; ++i) {
            int nextR = curr.row + dr[i];
            int nextC = curr.col + dc[i];

            if (!isFreeWavefrontCell(nextR, nextC, r, c, grid) || seen[nextR][nextC]) {
                continue;
            }

            seen[nextR][nextC] = true;
            frontier.push({nextR, nextC});
        }
    }

    return goal;
}

void buildWavefrontCoverage(Point curr,
                            const std::vector<std::vector<int>>& dist,
                            std::vector<std::vector<bool>>& visited,
                            std::vector<Point>& trajectory) {
    visited[curr.row][curr.col] = true;

    int dr[] = {0, 0, 1, -1};
    int dc[] = {1, -1, 0, 0};
    
    while (true) {
        Point next = {-1, -1};
        int best_dist = -1;

        for (int i = 0; i < 4; ++i) {
            int nextR = curr.row + dr[i];
            int nextC = curr.col + dc[i];

            if (nextR < 0 || nextR >= dist.size() || nextC < 0 || nextC >= dist[0].size()) {
                continue;
            }

            if (dist[nextR][nextC] == -1 || visited[nextR][nextC]) {
                continue;
            }

            if (dist[nextR][nextC] > best_dist) {
                best_dist = dist[nextR][nextC];
                next = {nextR, nextC};
            }
        }

        if (next.row == -1) {
            break;
        }

        trajectory.push_back(next);
        buildWavefrontCoverage(next, dist, visited, trajectory);
        trajectory.push_back(curr);
    }
}

CoverageResult runWavefrontCoverage(int r, int c, Point start,
                                    const std::vector<std::string>& grid) {
    CoverageResult result;

    if (!isFreeWavefrontCell(start.row, start.col, r, c, grid)) {
        return result;
    }

    Point goal = findWavefrontGoal(start, r, c, grid);
    std::vector<std::vector<int>> dist = buildDistanceTransform(goal, r, c, grid);
    if (dist[start.row][start.col] == -1) {
        return result;
    }

    result.trajectory.push_back(start);
    std::vector<std::vector<bool>> visited(r, std::vector<bool>(c, false));
    buildWavefrontCoverage(start, dist, visited, result.trajectory);

    result.total_steps = result.trajectory.size() - 1;
    return result;
}
