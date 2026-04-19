#include "stc.h"

#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

/*
STC1-style strict Spanning Tree Coverage implementation.

This file follows the main idea of the STC1 algorithm from:
https://ieeexplore.ieee.org/document/1013479
with some simplification on 

The implementation is "strict" because it only allows 2x2 major cells whose
four original grid cells are all free. It does not implement STC2's partially
occupied major cells or edge-type-dependent motion rules.

High-level flow:
1. Compress the original grid into 2x2 major cells.
2. Keep only major cells whose four sub-cells are obstacle-free.
3. Recursively build a spanning tree over those valid major cells.
4. Convert each spanning-tree edge into concrete moves through the original
   grid cells, tracing around the tree to form the coverage trajectory.
*/

struct EdgeKey {
    Point a;
    Point b;
};

bool samePoint(Point a, Point b) {
    return a.row == b.row && a.col == b.col;
}

bool lessPoint(Point a, Point b) {
    if (a.row != b.row) {
        return a.row < b.row;
    }
    return a.col < b.col;
}

bool isFreeSTCCell(int row, int col, int rows, int cols,
                   const std::vector<std::string>& grid) {
    return row >= 0 && row < rows && col >= 0 && col < cols && grid[row][col] == '.';
}

bool isValidMajorCell(int cell_row, int cell_col, int rows, int cols,
                      const std::vector<std::string>& grid) {
    int base_row = 2 * cell_row;
    int base_col = 2 * cell_col;

    if (base_row + 1 >= rows || base_col + 1 >= cols) {
        return false;
    }
    // robot only enters a 2 * 2 block if that entire block is safe. If any of the 4 cells has
    // an obstacle, the whole cell is rejected
    return isFreeSTCCell(base_row, base_col, rows, cols, grid) &&
           isFreeSTCCell(base_row, base_col + 1, rows, cols, grid) &&
           isFreeSTCCell(base_row + 1, base_col, rows, cols, grid) &&
           isFreeSTCCell(base_row + 1, base_col + 1, rows, cols, grid);
}

Point getNW(Point cell) {
    return {2 * cell.row, 2 * cell.col};
}

Point getNE(Point cell) {
    return {2 * cell.row, 2 * cell.col + 1};
}

Point getSW(Point cell) {
    return {2 * cell.row + 1, 2 * cell.col};
}

Point getSE(Point cell) {
    return {2 * cell.row + 1, 2 * cell.col + 1};
}

char getDirection(Point from, Point to) {
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

EdgeKey makeEdge(Point a, Point b) {
    if (lessPoint(b, a)) {
        Point tmp = a;
        a = b;
        b = tmp;
    }
    return {a, b};
}

bool hasEdge(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges,
             Point a, Point b) {
    EdgeKey edge = makeEdge(a, b);
    return edges.count({{edge.a.row, edge.a.col}, {edge.b.row, edge.b.col}}) > 0;
}

void appendPoint(std::vector<Point>& trajectory, Point p) {
    if (trajectory.empty() || !samePoint(trajectory.back(), p)) {
        trajectory.push_back(p);
    }
}

void appendInterpolatedMove(std::vector<Point>& trajectory, Point from, Point to) {
    int row_diff = to.row - from.row;
    int col_diff = to.col - from.col;

    if (std::abs(row_diff) + std::abs(col_diff) <= 1) {
        appendPoint(trajectory, to);
        return;
    }

    if (row_diff != 0 && col_diff != 0) {
        Point mid;
        if ((row_diff > 0) ^ (col_diff > 0)) {
            mid = {from.row, to.col};
        } else {
            mid = {to.row, from.col};
        }
        appendPoint(trajectory, mid);
        appendPoint(trajectory, to);
        return;
    }

    if (row_diff != 0) {
        int step = row_diff > 0 ? 1 : -1;
        for (int row = from.row + step; row != to.row + step; row += step) {
            appendPoint(trajectory, {row, from.col});
        }
        return;
    }

    int step = col_diff > 0 ? 1 : -1;
    for (int col = from.col + step; col != to.col + step; col += step) {
        appendPoint(trajectory, {from.row, col});
    }
}

void appendSegment(std::vector<Point>& trajectory, const std::vector<Point>& segment) {
    if (segment.empty()) {
        return;
    }

    Point curr = trajectory.back();
    for (const Point& step : segment) {
        appendInterpolatedMove(trajectory, curr, step);
        curr = step;
    }
}

std::vector<Point> moveAcrossEdge(Point from, Point to) {
    char dir = getDirection(from, to);

    if (dir == 'E') {
        return {getSE(from), getSW(to)};
    }
    if (dir == 'W') {
        return {getNW(from), getNE(to)};
    }
    if (dir == 'S') {
        return {getSW(from), getNW(to)};
    }
    return {getNE(from), getSE(to)};
}

std::vector<Point> buildRoundTrip(Point last, Point pivot) {
    char dir = getDirection(last, pivot);

    if (dir == 'E') {
        return {getSE(pivot), getNE(pivot)};
    }
    if (dir == 'W') {
        return {getNW(pivot), getSW(pivot)};
    }
    if (dir == 'S') {
        return {getSW(pivot), getSE(pivot)};
    }
    return {getNE(pivot), getNW(pivot)};
}

Point findIntermediateNode(
    Point from,
    Point to,
    const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges) {
    int dr[] = {1, 0, -1, 0};
    int dc[] = {0, 1, 0, -1};

    for (int i = 0; i < 4; ++i) {
        Point candidate = {from.row + dr[i], from.col + dc[i]};
        if (hasEdge(edges, from, candidate) && hasEdge(edges, candidate, to)) {
            return candidate;
        }
    }

    return from;
}

bool hasUnvisitedNeighbor(Point cell,
                          const std::vector<std::vector<int>>& visit_counts,
                          int cell_rows, int cell_cols,
                          int rows, int cols,
                          const std::vector<std::string>& grid) {
    int dr[] = {1, 0, -1, 0};
    int dc[] = {0, 1, 0, -1};

    for (int i = 0; i < 4; ++i) {
        Point next = {cell.row + dr[i], cell.col + dc[i]};
        if (next.row < 0 || next.row >= cell_rows || next.col < 0 || next.col >= cell_cols) {
            continue;
        }

        if (isValidMajorCell(next.row, next.col, rows, cols, grid) &&
            visit_counts[next.row][next.col] == 0) {
            return true;
        }
    }

    return false;
}

void buildCoverageRoute(
    Point curr,
    std::vector<std::vector<int>>& visit_counts,
    std::vector<Point>& route,
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges,
    int cell_rows, int cell_cols,
    int rows, int cols,
    const std::vector<std::string>& grid) {
    route.push_back(curr);

    int dr[] = {1, 0, -1, 0};
    int dc[] = {0, 1, 0, -1};
    bool found_child = false;

    for (int i = 0; i < 4; ++i) {
        Point next = {curr.row + dr[i], curr.col + dc[i]};
        if (next.row < 0 || next.row >= cell_rows || next.col < 0 || next.col >= cell_cols) {
            continue;
        }

        if (!isValidMajorCell(next.row, next.col, rows, cols, grid) ||
            visit_counts[next.row][next.col] != 0) {
            continue;
        }

        EdgeKey edge = makeEdge(curr, next);
        edges.insert({{edge.a.row, edge.a.col}, {edge.b.row, edge.b.col}});
        visit_counts[next.row][next.col] = 1;
        found_child = true;
        buildCoverageRoute(next, visit_counts, route, edges, cell_rows, cell_cols, rows, cols, grid);
    }

    if (!found_child) {
        for (int i = static_cast<int>(route.size()) - 1; i >= 0; --i) {
            Point node = route[i];

            if (visit_counts[node.row][node.col] == 2) {
                continue;
            }

            visit_counts[node.row][node.col] += 1;
            route.push_back(node);

            if (hasUnvisitedNeighbor(node, visit_counts, cell_rows, cell_cols, rows, cols, grid)) {
                break;
            }
        }
    }
}

CoverageResult runSTCCoverage(int rows, int cols, Point start,
                              const std::vector<std::string>& grid) {
    CoverageResult result;

    if (rows % 2 != 0 || cols % 2 != 0) {
        return result;
    }

    Point start_cell = {start.row / 2, start.col / 2};
    if (!isValidMajorCell(start_cell.row, start_cell.col, rows, cols, grid)) {
        return result;
    }

    int cell_rows = rows / 2;
    int cell_cols = cols / 2;

    std::vector<std::vector<int>> visit_counts(cell_rows, std::vector<int>(cell_cols, 0));
    std::vector<Point> route;
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> edges;

    visit_counts[start_cell.row][start_cell.col] = 1;
    buildCoverageRoute(start_cell, visit_counts, route, edges,
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
            segment = buildRoundTrip(route[i - 1], route[i]);
        } else if (distance == 1) {
            segment = moveAcrossEdge(route[i], route[i + 1]);
        } else if (distance == 2) {
            Point mid = findIntermediateNode(route[i], route[i + 1], edges);
            std::vector<Point> first = moveAcrossEdge(route[i], mid);
            std::vector<Point> second = moveAcrossEdge(mid, route[i + 1]);
            segment.insert(segment.end(), first.begin(), first.end());
            segment.insert(segment.end(), second.begin(), second.end());
        }

        appendSegment(result.trajectory, segment);
    }

    if (route.size() >= 2 && samePoint(route.back(), start_cell)) {
        std::vector<Point> closing_segment = buildRoundTrip(route[route.size() - 2], start_cell);
        appendSegment(result.trajectory, closing_segment);
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}
