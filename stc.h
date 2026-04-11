#ifndef STC_H
#define STC_H

#include <string>
#include <vector>

#include "test.h"

CoverageResult runSTCCoverage(int rows, int cols, Point start,
                              const std::vector<std::string>& grid);

#endif
