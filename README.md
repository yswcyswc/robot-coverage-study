# Final Project

## Layout

- `main.cpp` - runs the demo
- `test.cpp` / `test.h` - very simple traversal algorithm only
- `map_io.cpp` / `map_io.h` - map loading
- `maps/` - test maps
- `visualizer.py` - animates the moving robot and saves the final result
- `outputs/` - generated trajectory CSVs and images

## Build

```bash
g++ -std=c++17 -O2 main.cpp map_io.cpp frontier_planner.cpp random_planner.cpp # to be updated
```

## Run

```bash
./a.out maps/open_room.txt frontier # replace the last argument with the planner used
./a.out maps/narrow_corridor.txt frontier # replace the last argument with the planner used
./a.out maps/cluttered_room.txt frontier # replace the last argument with the planner used
```

## Visualize

```bash
python3 visualizer.py maps/open_room.txt frontier # replace the last argument
python3 visualizer.py maps/narrow_corridor.txt frontier # replace the last argument
python3 visualizer.py maps/cluttered_room.txt frontier # replace the last argument
```
