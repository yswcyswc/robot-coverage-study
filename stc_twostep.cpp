#include "stc_twostep.h"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <string>
#include <utility>
#include <vector>

/*
Two-step STC optimization variant.

This variant is enhancing the Spanning Tree Coverage idea in:
https://ieeexplore.ieee.org/document/1013479 where this paper only considers 
one sensor.

This keeps the same strict 2x2 major-cell assumption as stc.cpp, but optimizes
how the spanning tree is grown. Instead of scanning adjacent major cells in a
fixed order only, each adjacent candidate is scored by looking one extra major
cell beyond it. The robot still moves only between adjacent major cells; the
second-neighbor information only influences which branch is explored first.

Our hypothesis is that this two-step sensing reduces unnecessary backtracking
by choosing branches that lead to more future free space for some maps.
*/

struct TwoStepCandidate {
    Point cell;
    int direction_index;
    int score;
};

static bool laSamePoint(Point a, Point b) {
    return a.row == b.row && a.col == b.col;
}

static bool laLessPoint(Point a, Point b) {
    if (a.row != b.row) {
        return a.row < b.row;
    }
    return a.col < b.col;
}

static bool laIsFreeCell(int row, int col, int rows, int cols,
                         const std::vector<std::string>& grid) {
    return row >= 0 && row < rows && col >= 0 && col < cols && grid[row][col] == '.';
}

static bool laIsValidMajorCell(int cell_row, int cell_col, int rows, int cols,
                               const std::vector<std::string>& grid) {
    int base_row = 2 * cell_row;
    int base_col = 2 * cell_col;

    if (base_row + 1 >= rows || base_col + 1 >= cols) {
        return false;
    }

    return laIsFreeCell(base_row, base_col, rows, cols, grid) &&
           laIsFreeCell(base_row, base_col + 1, rows, cols, grid) &&
           laIsFreeCell(base_row + 1, base_col, rows, cols, grid) &&
           laIsFreeCell(base_row + 1, base_col + 1, rows, cols, grid);
}

static Point laNW(Point cell) {
    return {2 * cell.row, 2 * cell.col};
}

static Point laNE(Point cell) {
    return {2 * cell.row, 2 * cell.col + 1};
}

static Point laSW(Point cell) {
    return {2 * cell.row + 1, 2 * cell.col};
}

static Point laSE(Point cell) {
    return {2 * cell.row + 1, 2 * cell.col + 1};
}

static char laDirection(Point from, Point to) {
    if (from.row == to.row && from.col < to.col) {
        return 'E';
    }
    if (from.row == to.row && from.col > to.col) {
        return 'W';
    }
    if (from.row < to.row && from.col == to.col) {
        return 'S';
    }
    return 'N';
}

static std::pair<std::pair<int, int>, std::pair<int, int>> laMakeEdge(Point a, Point b) {
    if (laLessPoint(b, a)) {
        Point tmp = a;
        a = b;
        b = tmp;
    }
    return {{a.row, a.col}, {b.row, b.col}};
}

static bool laHasEdge(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges,
                      Point a, Point b) {
    return edges.count(laMakeEdge(a, b)) > 0;
}

static void laAppendPoint(std::vector<Point>& trajectory, Point p) {
    if (trajectory.empty() || !laSamePoint(trajectory.back(), p)) {
        trajectory.push_back(p);
    }
}

static void laAppendInterpolatedMove(std::vector<Point>& trajectory, Point from, Point to) {
    int row_diff = to.row - from.row;
    int col_diff = to.col - from.col;

    if (std::abs(row_diff) + std::abs(col_diff) <= 1) {
        laAppendPoint(trajectory, to);
        return;
    }

    if (row_diff != 0 && col_diff != 0) {
        Point mid = (row_diff > 0) ^ (col_diff > 0) ? Point{from.row, to.col}
                                                    : Point{to.row, from.col};
        laAppendPoint(trajectory, mid);
        laAppendPoint(trajectory, to);
        return;
    }

    if (row_diff != 0) {
        int step = row_diff > 0 ? 1 : -1;
        for (int row = from.row + step; row != to.row + step; row += step) {
            laAppendPoint(trajectory, {row, from.col});
        }
        return;
    }

    int step = col_diff > 0 ? 1 : -1;
    for (int col = from.col + step; col != to.col + step; col += step) {
        laAppendPoint(trajectory, {from.row, col});
    }
}

static void laAppendSegment(std::vector<Point>& trajectory,
                            const std::vector<Point>& segment) {
    if (segment.empty()) {
        return;
    }

    Point current = trajectory.back();
    for (Point step : segment) {
        laAppendInterpolatedMove(trajectory, current, step);
        current = step;
    }
}

static std::vector<Point> laMoveAcrossEdge(Point from, Point to) {
    char dir = laDirection(from, to);

    if (dir == 'E') {
        return {laSE(from), laSW(to)};
    }
    if (dir == 'W') {
        return {laNW(from), laNE(to)};
    }
    if (dir == 'S') {
        return {laSW(from), laNW(to)};
    }
    return {laNE(from), laSE(to)};
}

static std::vector<Point> laBuildRoundTrip(Point last, Point pivot) {
    char dir = laDirection(last, pivot);

    if (dir == 'E') {
        return {laSE(pivot), laNE(pivot)};
    }
    if (dir == 'W') {
        return {laNW(pivot), laSW(pivot)};
    }
    if (dir == 'S') {
        return {laSW(pivot), laSE(pivot)};
    }
    return {laNE(pivot), laNW(pivot)};
}

static Point laFindIntermediateNode(
    Point from, Point to,
    const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges) {
    const int dr[] = {1, 0, -1, 0};
    const int dc[] = {0, 1, 0, -1};

    for (int i = 0; i < 4; ++i) {
        Point candidate = {from.row + dr[i], from.col + dc[i]};
        if (laHasEdge(edges, from, candidate) && laHasEdge(edges, candidate, to)) {
            return candidate;
        }
    }

    return from;
}

static bool laValidUnvisitedMajor(Point cell,
                                  const std::vector<std::vector<int>>& visit_counts,
                                  int cell_rows, int cell_cols, int rows, int cols,
                                  const std::vector<std::string>& grid) {
    if (cell.row < 0 || cell.row >= cell_rows ||
        cell.col < 0 || cell.col >= cell_cols ||
        visit_counts[cell.row][cell.col] != 0) {
        return false;
    }

    return laIsValidMajorCell(cell.row, cell.col, rows, cols, grid);
}

static int laTwoStepScore(Point current,
                            int direction_index,
                            const std::vector<std::vector<int>>& visit_counts,
                            int cell_rows, int cell_cols, int rows, int cols,
                            const std::vector<std::string>& grid) {
    const int dr[] = {1, 0, -1, 0};
    const int dc[] = {0, 1, 0, -1};
    Point candidate = {current.row + dr[direction_index], current.col + dc[direction_index]};

    int score = 0;
    for (int i = 0; i < 4; ++i) {
        Point neighbor = {candidate.row + dr[i], candidate.col + dc[i]};
        if (laValidUnvisitedMajor(neighbor, visit_counts, cell_rows, cell_cols, rows, cols, grid)) {
            score += 2;
        }
    }

    Point straight_ahead = {candidate.row + dr[direction_index],
                            candidate.col + dc[direction_index]};
    if (laValidUnvisitedMajor(straight_ahead, visit_counts, cell_rows, cell_cols, rows, cols, grid)) {
        score += 3;
    }

    return score;
}

static std::vector<TwoStepCandidate> laBuildCandidates(
    Point current,
    const std::vector<std::vector<int>>& visit_counts,
    int cell_rows, int cell_cols, int rows, int cols,
    const std::vector<std::string>& grid) {
    const int dr[] = {1, 0, -1, 0};
    const int dc[] = {0, 1, 0, -1};
    std::vector<TwoStepCandidate> candidates;

    for (int i = 0; i < 4; ++i) {
        Point next = {current.row + dr[i], current.col + dc[i]};
        if (!laValidUnvisitedMajor(next, visit_counts, cell_rows, cell_cols, rows, cols, grid)) {
            continue;
        }

        candidates.push_back({
            next,
            i,
            laTwoStepScore(current, i, visit_counts, cell_rows, cell_cols, rows, cols, grid),
        });
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const TwoStepCandidate& a, const TwoStepCandidate& b) {
                  if (a.score != b.score) {
                      return a.score > b.score;
                  }
                  return a.direction_index < b.direction_index;
              });

    return candidates;
}

static bool laHasUnvisitedNeighbor(Point cell, const std::vector<std::vector<int>>& visit_counts, int cell_rows, int cell_cols, int rows, int cols, const std::vector<std::string>& grid) {
    return !laBuildCandidates(cell, visit_counts, cell_rows, cell_cols, rows, cols, grid).empty();
}

static void laBuildCoverageRoute(
    Point current,
    std::vector<std::vector<int>>& visit_counts,
    std::vector<Point>& route,
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges,
    int cell_rows, int cell_cols, int rows, int cols,
    const std::vector<std::string>& grid) {
    route.push_back(current);
    bool found_child = false;

    std::vector<TwoStepCandidate> candidates =
        laBuildCandidates(current, visit_counts, cell_rows, cell_cols, rows, cols, grid);

    for (const TwoStepCandidate& candidate : candidates) {
        if (visit_counts[candidate.cell.row][candidate.cell.col] != 0) {
            continue;
        }

        edges.insert(laMakeEdge(current, candidate.cell));
        visit_counts[candidate.cell.row][candidate.cell.col] = 1;
        found_child = true;
        laBuildCoverageRoute(candidate.cell, visit_counts, route, edges,
                             cell_rows, cell_cols, rows, cols, grid);
    }

    if (!found_child) {
        for (int i = static_cast<int>(route.size()) - 1; i >= 0; --i) {
            Point node = route[i];

            if (visit_counts[node.row][node.col] == 2) {
                continue;
            }

            visit_counts[node.row][node.col] += 1;
            route.push_back(node);

            if (laHasUnvisitedNeighbor(node, visit_counts, cell_rows, cell_cols, rows, cols, grid)) {
                break;
            }
        }
    }
}

CoverageResult runSTCTwoStepCoverage(int rows, int cols, Point start, const std::vector<std::string>& grid) {
    CoverageResult result;

    if (rows % 2 != 0 || cols % 2 != 0) {
        return result;
    }

    Point start_cell = {start.row / 2, start.col / 2};
    if (!laIsValidMajorCell(start_cell.row, start_cell.col, rows, cols, grid)) {
        return result;
    }

    int cell_rows = rows / 2;
    int cell_cols = cols / 2;

    std::vector<std::vector<int>> visit_counts(cell_rows, std::vector<int>(cell_cols, 0));
    std::vector<Point> route;
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> edges;

    visit_counts[start_cell.row][start_cell.col] = 1;
    laBuildCoverageRoute(start_cell, visit_counts, route, edges,
                         cell_rows, cell_cols, rows, cols, grid);

    if (route.empty()) {
        return result;
    }

    result.trajectory.push_back(start);

    for (int i = 0; i + 1 < static_cast<int>(route.size()); ++i) {
        int distance = std::abs(route[i].row - route[i + 1].row) +
                       std::abs(route[i].col - route[i + 1].col);

        std::vector<Point> segment;
        if (distance == 0) {
            if (i == 0) {
                continue;
            }
            segment = laBuildRoundTrip(route[i - 1], route[i]);
        } else if (distance == 1) {
            segment = laMoveAcrossEdge(route[i], route[i + 1]);
        } else if (distance == 2) {
            Point mid = laFindIntermediateNode(route[i], route[i + 1], edges);
            std::vector<Point> first = laMoveAcrossEdge(route[i], mid);
            std::vector<Point> second = laMoveAcrossEdge(mid, route[i + 1]);
            segment.insert(segment.end(), first.begin(), first.end());
            segment.insert(segment.end(), second.begin(), second.end());
        }

        laAppendSegment(result.trajectory, segment);
    }

    if (route.size() >= 2 && laSamePoint(route.back(), start_cell)) {
        std::vector<Point> closing_segment = laBuildRoundTrip(route[route.size() - 2], start_cell);
        laAppendSegment(result.trajectory, closing_segment);
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}
