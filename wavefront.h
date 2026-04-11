#ifndef WAVEFRONT_H
#define WAVEFRONT_H

#include <string>
#include <vector>

#include "test.h"

CoverageResult runWavefrontCoverage(int rows, int cols, Point start,
                                    const std::vector<std::string>& grid);

#endif
