#include "test.h"

CoverageResult runTestTraversal(int rows, int cols, Point start) {
    CoverageResult result;

    if (rows <= 0 || cols <= 0) {
        return result;
    }

    int start_row = start.row;
    if (start_row < 0) {
        start_row = 0;
    }
    if (start_row >= rows) {
        start_row = rows - 1;
    }

    int start_col = start.col;
    if (start_col < 0) {
        start_col = 0;
    }
    if (start_col >= cols) {
        start_col = cols - 1;
    }

    result.trajectory.push_back({start_row, start_col});

    for (int col = start_col - 1; col >= 0; --col) {
        result.trajectory.push_back({start_row, col});
    }
    for (int row = start_row - 1; row >= 0; --row) {
        result.trajectory.push_back({row, 0});
    }

    for (int row = 0; row < rows; ++row) {
        if (row % 2 == 0) {
            int begin_col = (row == 0) ? 1 : 0;
            for (int col = begin_col; col < cols; ++col) {
                result.trajectory.push_back({row, col});
            }
        } else {
            for (int col = cols - 2; col >= 0; --col) {
                result.trajectory.push_back({row, col});
            }
        }

        if (row < rows - 1) {
            int edge_col = (row % 2 == 0) ? cols - 1 : 0;
            result.trajectory.push_back({row + 1, edge_col});
        }
    }

    result.total_steps = static_cast<int>(result.trajectory.size()) - 1;
    return result;
}
