#ifndef BOUSTROPHEDON_GRID_H
#define BOUSTROPHEDON_GRID_H

#include <string>
#include <vector>

#include "test.h"

CoverageResult runBoustrophedonGridTraversal(int rows, int cols, Point start,
                                             const std::vector<std::string>& grid);

#endif
