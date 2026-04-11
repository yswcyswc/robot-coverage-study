# Final Project

## Layout

- `main.cpp` - runs the demo
- `test.cpp` / `test.h` - very simple traversal algorithm only
- `map_io.cpp` / `map_io.h` - map loading
- `maps/` - test maps
- `visualizer.py` - animates the moving robot and saves the final result
- `outputs/trajectories/` - generated trajectory CSVs
- `outputs/images/` - generated coverage images
- `outputs/gifs/` - generated coverage animations

## Build

```bash
g++ -std=c++17 -O2 main.cpp map_io.cpp frontier_planner.cpp random_planner.cpp lawnmower.cpp wavefront.cpp stc.cpp
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

## Results

Results below are from one run of each planner on the three default maps.
`random` is stochastic, so its numbers will vary between runs.

### Open Room

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 77 | 76 | 76 | 2 | 2.56% |
| random | 500 | 76 | 60 | 441 | 88.02% |
| lawnmower | 83 | 76 | 76 | 8 | 9.52% |
| wavefront | 260 | 76 | 76 | 185 | 70.88% |
| stc | 68 | 76 | 64 | 5 | 7.25% |

### Narrow Corridor

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 72 | 53 | 53 | 20 | 27.40% |
| random | 500 | 53 | 49 | 452 | 90.22% |
| lawnmower | 79 | 53 | 53 | 27 | 33.75% |
| wavefront | 122 | 53 | 53 | 70 | 56.91% |
| stc | 0 | 53 | 0 | 0 | 0.00% |

### Cluttered Room

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 113 | 96 | 96 | 18 | 15.79% |
| random | 500 | 96 | 45 | 456 | 91.02% |
| lawnmower | 239 | 96 | 96 | 144 | 60.00% |
| wavefront | 312 | 96 | 96 | 217 | 69.33% |
| stc | 93 | 96 | 94 | 0 | 0.00% |
