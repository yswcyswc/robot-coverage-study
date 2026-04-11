#include "lawnmower.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <utility>
#include <vector>

/*
Boustrophedon-inspired lawnmower sweep
https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/choset_howie_1997_5/choset_howie_1997_5.pdf

1. Find all free cells reachable from the start cell.
2. Build a sweep order row by row:
   - for one row, visit reachable free cells left-to-right
   - for the next row, visit reachable free cells right-to-left
3. For each next sweep target:
   - if the robot is already there, continue
   - otherwise, run A* on free cells to get a shortest path
   - append that path to the trajectory
4. Mark visited cells along the way and stop when every reachable free cell
   has been covered.
*/

bool isFreeCell(int row, int col, int rows, int cols,
                const std::vector<std::string>& grid) {
    return row >= 0 && row < rows && col >= 0 && col < cols && grid[row][col] == '.';
}

int heuristic(Point a, Point b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

std::vector<Point> buildAStarPath(Point start, Point goal, int rows, int cols,
                                  const std::vector<std::string>& grid) {
    struct SearchNode {
        int f;
        int g;
        Point p;
    };

    struct CompareNode {
        bool operator()(const SearchNode& a, const SearchNode& b) const {
            return a.f > b.f;
        }
    };

    std::priority_queue<SearchNode, std::vector<SearchNode>, CompareNode> frontier;
    std::vector<std::vector<int>> best_g(rows, std::vector<int>(cols, -1));
    std::vector<std::vector<Point>> parent(rows, std::vector<Point>(cols, {-1, -1}));

    const int dr[] = {0, 0, 1, -1};
    const int dc[] = {1, -1, 0, 0};

    best_g[start.row][start.col] = 0;
    frontier.push({heuristic(start, goal), 0, start});

    while (!frontier.empty()) {
        SearchNode node = frontier.top();
        frontier.pop();
        Point current = node.p;

        if (node.g != best_g[current.row][current.col]) {
            continue;
        }

        if (current.row == goal.row && current.col == goal.col) {
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int nextR = current.row + dr[i];
            int nextC = current.col + dc[i];

            if (!isFreeCell(nextR, nextC, rows, cols, grid)) {
                continue;
            }

            int next_g = node.g + 1;
            if (best_g[nextR][nextC] != -1 && next_g >= best_g[nextR][nextC]) {
                continue;
            }

            best_g[nextR][nextC] = next_g;
            parent[nextR][nextC] = current;
            frontier.push({next_g + heuristic({nextR, nextC}, goal), next_g, {nextR, nextC}});
        }
    }

    if (best_g[goal.row][goal.col] == -1) {
        return {};
    }

    std::vector<Point> path;
    for (Point current = goal;
        !(current.row == start.row && current.col == start.col);
         current = parent[current.row][current.col]) {
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::set<std::pair<int, int>> findReachableCells(int rows, int cols, Point start,
                                                 const std::vector<std::string>& grid) {
    std::set<std::pair<int, int>> reachable;
    if (!isFreeCell(start.row, start.col, rows, cols, grid)) {
        return reachable;
    }

    std::queue<Point> frontier;
    frontier.push(start);
    reachable.insert({start.row, start.col});

    const int dr[] = {0, 0, 1, -1};
    const int dc[] = {1, -1, 0, 0};

    while (!frontier.empty()) {
        Point current = frontier.front();
        frontier.pop();

        for (int i = 0; i < 4; ++i) {
            const int nextR = current.row + dr[i];
            const int nextC = current.col + dc[i];
            const std::pair<int, int> next = {nextR, nextC};

            if (!isFreeCell(nextR, nextC, rows, cols, grid) || reachable.count(next) > 0) {
                continue;
            }

            reachable.insert(next);
            frontier.push({nextR, nextC});
        }
    }

    return reachable;
}

std::vector<Point> buildSweepTargets(int rows, int cols,
                                     const std::set<std::pair<int, int>>& reachable) {
    std::vector<Point> targets;

    for (int row = 0; row < rows; ++row) {
        std::vector<int> cols_in_row;
        for (int col = 0; col < cols; ++col) {
            if (reachable.count({row, col}) > 0) {
                cols_in_row.push_back(col);
            }
        }

        if (row % 2 == 1) {
            std::reverse(cols_in_row.begin(), cols_in_row.end());
        }

        for (int col : cols_in_row) {
            targets.push_back({row, col});
        }
    }

    return targets;
}

CoverageResult runLawnmowerTraversal(int rows, int cols, Point start,
                                     const std::vector<std::string>& grid) {
    CoverageResult result;
    std::set<std::pair<int, int>> reachable = findReachableCells(rows, cols, start, grid);

    if (reachable.empty()) {
        return result;
    }

    std::vector<Point> sweep_targets = buildSweepTargets(rows, cols, reachable);
    std::set<std::pair<int, int>> covered;

    Point current = start;
    result.trajectory.push_back(current);
    covered.insert({current.row, current.col});

    for (const Point& target : sweep_targets) {
        if (covered.count({target.row, target.col}) > 0) {
            continue;
        }

        std::vector<Point> path = buildAStarPath(current, target, rows, cols, grid);
        if (path.empty()) {
            continue;
        }

        for (const Point& step : path) {
            result.trajectory.push_back(step);
            covered.insert({step.row, step.col});
            current = step;
        }
    }

    result.total_steps = result.trajectory.size() - 1;
    return result;
}
