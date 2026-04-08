#ifndef TEST_H
#define TEST_H

#include <vector>

struct Point {
    int row;
    int col;
};

struct CoverageResult {
    std::vector<Point> trajectory;
    int total_steps = 0;
};

CoverageResult runTestTraversal(int rows, int cols, Point start);

#endif
