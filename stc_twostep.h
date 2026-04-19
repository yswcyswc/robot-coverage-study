#ifndef STC_TWOSTEP_H
#define STC_TWOSTEP_H

#include <string>
#include <vector>

#include "test.h"

CoverageResult runSTCTwoStepCoverage(int rows, int cols, Point start,
                                     const std::vector<std::string>& grid);

#endif
