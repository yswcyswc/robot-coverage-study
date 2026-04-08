#ifndef MAP_IO_H
#define MAP_IO_H

#include <string>
#include <vector>

#include "test.h"

struct GridMap {
    int rows = 0;
    int cols = 0;
    Point start{0, 0};
    std::vector<std::string> cells;
};

GridMap loadMap(const std::string& map_path);

#endif
