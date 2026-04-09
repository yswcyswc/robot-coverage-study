#ifndef BOUSTROPHEDON_H
#define BOUSTROPHEDON_H

#include <string>
#include <vector>

#include "test.h"

CoverageResult runLawnmowerTraversal(int rows, int cols, Point start,
                                     const std::vector<std::string>& grid);

#endif
