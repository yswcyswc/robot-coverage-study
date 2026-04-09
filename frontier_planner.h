#ifndef FRONTIER_PLANNER_H
#define FRONTIER_PLANNER_H

#include <vector>
#include <string>
#include <set>
#include "test.h" // Reusing Point and CoverageResult structs

CoverageResult runFrontierCoverage(int rows, int cols, Point start, 
                                   const std::vector<std::string>& grid);

#endif