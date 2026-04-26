#include "lawnmower.h"

#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

/*
Grid boustrophedon decomposition with lawnmower coverage.
https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/choset_howie_1997_5/choset_howie_1997_5.pdf

This is a discrete, simplified version of the boustrophedon idea from the above paper:
1. Sweep the grid column by column.
2. In each column, extract contiguous free vertical intervals.
3. Compare current intervals with previous-column intervals.
   - no overlap: a new cell opens
   - one-to-one overlap: the same cell continues
   - one-to-many overlap: a cell splits
   - many-to-one overlap: cells merge
4. Build a cell adjacency graph from those split/merge/touch events.
5. Run a DFS-like walk over the cell graph.
6. Clean each cell with a local zigzag sweep, using BFS to transfer between
   cells or around obstacles.
*/

struct Interval {
    int top;
    int bottom;
    int cell_id;
};

struct DecompositionCell {
    int id;
    std::vector<Point> points;
    std::set<int> neighbors;
};

static bool samePoint(Point a, Point b) {
    return a.row == b.row && a.col == b.col;
}

static bool isFree(int row, int col, int rows, int cols,
                   const std::vector<std::string>& grid) {
    return row >= 0 && row < rows && col >= 0 && col < cols && grid[row][col] == '.';
}

static int manhattan(Point a, Point b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

static bool intervalsOverlap(const Interval& a, const Interval& b) {
    return a.top <= b.bottom && b.top <= a.bottom;
}

static std::set<std::pair<int, int>> findReachable(
    int rows,
    int cols,
    Point start,
    const std::vector<std::string>& grid) {
    std::set<std::pair<int, int>> reachable;
    if (!isFree(start.row, start.col, rows, cols, grid)) {
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
            Point next = {current.row + dr[i], current.col + dc[i]};
            if (!isFree(next.row, next.col, rows, cols, grid) ||
                reachable.count({next.row, next.col}) > 0) {
                continue;
            }

            reachable.insert({next.row, next.col});
            frontier.push(next);
        }
    }

    return reachable;
}

static std::vector<Interval> extractColumnIntervals(
    int col,
    int rows,
    const std::set<std::pair<int, int>>& reachable) {
    std::vector<Interval> intervals;
    int row = 0;

    while (row < rows) {
        while (row < rows && reachable.count({row, col}) == 0) {
            ++row;
        }

        if (row >= rows) {
            break;
        }

        int top = row;
        while (row + 1 < rows && reachable.count({row + 1, col}) > 0) {
            ++row;
        }

        intervals.push_back({top, row, -1});
        ++row;
    }

    return intervals;
}

static int createCell(std::vector<DecompositionCell>& cells) {
    int id = static_cast<int>(cells.size());
    cells.push_back({id, {}, {}});
    return id;
}

static void connectCells(std::vector<DecompositionCell>& cells, int a, int b) {
    if (a == b || a < 0 || b < 0) {
        return;
    }

    cells[a].neighbors.insert(b);
    cells[b].neighbors.insert(a);
}

static void addIntervalPoints(DecompositionCell& cell, int col, const Interval& interval) {
    for (int row = interval.top; row <= interval.bottom; ++row) {
        cell.points.push_back({row, col});
    }
}

static std::vector<DecompositionCell> decomposeByColumnSweep(
    int rows,
    int cols,
    const std::set<std::pair<int, int>>& reachable) {
    std::vector<DecompositionCell> cells;
    std::vector<Interval> previous;

    for (int col = 0; col < cols; ++col) {
        std::vector<Interval> current = extractColumnIntervals(col, rows, reachable);
        std::vector<std::vector<int>> overlaps(current.size());
        std::map<int, int> previous_overlap_count;

        for (int i = 0; i < static_cast<int>(current.size()); ++i) {
            for (const Interval& prev : previous) {
                if (!intervalsOverlap(current[i], prev)) {
                    continue;
                }

                overlaps[i].push_back(prev.cell_id);
                previous_overlap_count[prev.cell_id] += 1;
            }
        }

        for (int i = 0; i < static_cast<int>(current.size()); ++i) {
            int assigned_id = -1;

            if (overlaps[i].size() == 1 && previous_overlap_count[overlaps[i][0]] == 1) {
                assigned_id = overlaps[i][0];
            } else {
                assigned_id = createCell(cells);
                for (int previous_id : overlaps[i]) {
                    connectCells(cells, assigned_id, previous_id);
                }
            }

            current[i].cell_id = assigned_id;
            addIntervalPoints(cells[assigned_id], col, current[i]);
        }

        previous = current;
    }

    return cells;
}

static int findCellContainingPoint(const std::vector<DecompositionCell>& cells, Point p) {
    for (const DecompositionCell& cell : cells) {
        for (Point candidate : cell.points) {
            if (samePoint(candidate, p)) {
                return cell.id;
            }
        }
    }

    return -1;
}

static void buildDfsWalk(const std::vector<DecompositionCell>& cells,
                         int cell_id,
                         std::vector<bool>& visited,
                         std::vector<int>& path) {
    visited[cell_id] = true;
    path.push_back(cell_id);

    for (int neighbor_id : cells[cell_id].neighbors) {
        if (visited[neighbor_id]) {
            continue;
        }

        buildDfsWalk(cells, neighbor_id, visited, path);
    }
}

static std::vector<Point> buildBfsPath(
    Point start, Point goal, int rows, int cols,
    const std::vector<std::string>& grid) {
    if (samePoint(start, goal)) {
        return {};
    }

    std::queue<Point> frontier;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Point>> parent(rows, std::vector<Point>(cols, {-1, -1}));

    const int dr[] = {0, 0, 1, -1};
    const int dc[] = {1, -1, 0, 0};

    frontier.push(start);
    visited[start.row][start.col] = true;

    while (!frontier.empty()) {
        Point current = frontier.front();
        frontier.pop();

        if (samePoint(current, goal)) {
            break;
        }

        for (int i = 0; i < 4; ++i) {
            Point next = {current.row + dr[i], current.col + dc[i]};

            if (!isFree(next.row, next.col, rows, cols, grid) ||
                visited[next.row][next.col]) {
                continue;
            }

            visited[next.row][next.col] = true;
            parent[next.row][next.col] = current;
            frontier.push(next);
        }
    }

    if (!visited[goal.row][goal.col]) {
        return {};
    }

    std::vector<Point> path;
    for (Point current = goal; !samePoint(current, start);
         current = parent[current.row][current.col]) {
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

static void appendPath(CoverageResult& result,
                       const std::vector<Point>& path,
                       std::set<std::pair<int, int>>& covered,
                       Point& current) {
    for (Point step : path) {
        result.trajectory.push_back(step);
        covered.insert({step.row, step.col});
        current = step;
    }
}

static Point nearestPointInCell(Point current, const DecompositionCell& cell) {
    Point nearest = cell.points.front();
    int best_distance = manhattan(current, nearest);

    for (Point p : cell.points) {
        int distance = manhattan(current, p);
        if (distance < best_distance) {
            best_distance = distance;
            nearest = p;
        }
    }

    return nearest;
}

static std::vector<Point> buildCellSweep(const DecompositionCell& cell, Point current) {
    std::map<int, std::vector<int>> rows_by_col;
    for (Point p : cell.points) {
        rows_by_col[p.col].push_back(p.row);
    }

    std::vector<int> columns;
    for (const auto& entry : rows_by_col) {
        columns.push_back(entry.first);
    }

    std::vector<std::vector<Point>> candidates;
    for (int direction : {1, -1}) {
        std::vector<int> ordered_columns = columns;
        if (direction == -1) {
            std::reverse(ordered_columns.begin(), ordered_columns.end());
        }

        for (bool first_down : {true, false}) {
            std::vector<Point> sweep;
            bool down = first_down;

            for (int col : ordered_columns) {
                std::vector<int> ordered_rows = rows_by_col[col];
                std::sort(ordered_rows.begin(), ordered_rows.end());
                if (!down) {
                    std::reverse(ordered_rows.begin(), ordered_rows.end());
                }

                for (int row : ordered_rows) {
                    sweep.push_back({row, col});
                }

                down = !down;
            }

            candidates.push_back(sweep);
        }
    }

    auto score = [&](const std::vector<Point>& sweep) {
        if (sweep.empty()) {
            return 0;
        }
        return manhattan(current, sweep.front());
    };

    return *std::min_element(candidates.begin(), candidates.end(),
                             [&](const auto& a, const auto& b) {
                                 return score(a) < score(b);
                             });
}

static void moveToCell(CoverageResult& result,
                       Point& current,
                       const DecompositionCell& cell,
                       int rows,
                       int cols,
                       const std::vector<std::string>& grid,
                       std::set<std::pair<int, int>>& covered) {
    for (Point p : cell.points) {
        if (samePoint(current, p)) {
            return;
        }
    }

    Point entry = nearestPointInCell(current, cell);
    std::vector<Point> path = buildBfsPath(current, entry, rows, cols, grid);
    appendPath(result, path, covered, current);
}

static void cleanCell(CoverageResult& result,
                      Point& current,
                      const DecompositionCell& cell,
                      int rows,
                      int cols,
                      const std::vector<std::string>& grid,
                      std::set<std::pair<int, int>>& covered) {
    std::vector<Point> sweep = buildCellSweep(cell, current);

    for (Point target : sweep) {
        if (covered.count({target.row, target.col}) > 0) {
            continue;
        }

        std::vector<Point> path = buildBfsPath(current, target, rows, cols, grid);
        appendPath(result, path, covered, current);
    }
}

CoverageResult runLawnmowerTraversal(int rows, int cols, Point start,
                                     const std::vector<std::string>& grid) {
    CoverageResult result;
    std::set<std::pair<int, int>> reachable = findReachable(rows, cols, start, grid);
    if (reachable.empty()) {
        return result;
    }

    std::vector<DecompositionCell> cells = decomposeByColumnSweep(rows, cols, reachable);
    int start_cell = findCellContainingPoint(cells, start);
    if (start_cell < 0) {
        return result;
    }

    std::vector<int> cell_walk;
    std::vector<bool> visited_cells(cells.size(), false);
    buildDfsWalk(cells, start_cell, visited_cells, cell_walk);

    Point current = start;
    std::set<std::pair<int, int>> covered;
    result.trajectory.push_back(current);
    covered.insert({current.row, current.col});

    for (int cell_id : cell_walk) {
        moveToCell(result, current, cells[cell_id], rows, cols, grid, covered);
        cleanCell(result, current, cells[cell_id], rows, cols, grid, covered);
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}
