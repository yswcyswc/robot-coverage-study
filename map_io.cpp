#include "map_io.h"

#include <fstream>
#include <stdexcept>

GridMap loadMap(const std::string& map_path) {
    std::ifstream input(map_path);
    if (!input.is_open()) {
        throw std::runtime_error("Could not open map file: " + map_path);
    }

    GridMap grid_map;
    input >> grid_map.rows >> grid_map.cols;
    input >> grid_map.start.row >> grid_map.start.col;

    if (grid_map.rows <= 0 || grid_map.cols <= 0) {
        throw std::runtime_error("Map dimensions must be positive.");
    }

    grid_map.cells.resize(grid_map.rows);
    for (int row = 0; row < grid_map.rows; ++row) {
        input >> grid_map.cells[row];
        if ((int)grid_map.cells[row].size() != grid_map.cols) {
            throw std::runtime_error("Each map row must have exactly cols characters.");
        }
    }

    return grid_map;
}
