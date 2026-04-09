#ifndef RANDOM_PLANNER_H
#define RANDOM_PLANNER_H

#include <vector>
#include <string>
#include "test.h"

CoverageResult runRandomTraversal(int rows, int cols, Point start, 
                                  const std::vector<std::string>& grid, int max_steps = 5000);

#endif