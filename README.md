# Final Project

## Layout

- `main.cpp` - runs the demo
- `test.cpp` / `test.h` - very simple traversal algorithm only
- `map_io.cpp` / `map_io.h` - map loading
- `boustrophedon_grid.cpp` / `boustrophedon_grid.h` - grid boustrophedon decomposition planner
- `maps/` - test maps
- `visualizer.py` - saves coverage images and GIF animations
- `outputs/trajectories/` - generated trajectory CSVs
- `outputs/images/` - generated coverage images
- `outputs/gifs/` - generated coverage animations

## Build

```bash
g++ -std=c++17 -O2 main.cpp map_io.cpp frontier_planner.cpp random_planner.cpp lawnmower.cpp stc.cpp
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

The visualizer saves output files instead of opening a live popup window:
- `outputs/images/` for final PNGs
- `outputs/gifs/` for animated GIFs

## Results

Results below are from one run of each planner on the three default maps.
`random` is stochastic, so its numbers will vary between runs.

### Open Room

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 77 | 76 | 76 | 2 | 2.56% |
| random | 500 | 76 | 70 | 431 | 86.03% |
| lawnmower | 83 | 76 | 76 | 8 | 9.52% |
| stc | 68 | 76 | 64 | 5 | 7.25% |

### Narrow Corridor

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 72 | 53 | 53 | 20 | 27.40% |
| random | 500 | 53 | 33 | 468 | 93.41% |
| lawnmower | 79 | 53 | 53 | 27 | 33.75% |
| stc | 0 | 53 | 0 | 0 | 0.00% |

### Cluttered Room

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 127 | 92 | 92 | 36 | 28.13% |
| random | 500 | 92 | 51 | 450 | 89.82% |
| lawnmower | 167 | 92 | 92 | 76 | 45.24% |
| stc | 92 | 92 | 92 | 1 | 1.08% |

### Larger Room

| Algorithm | Total Steps | Reachable Free Cells | Unique Visited Cells | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: |
| frontier | 2350 | 2172 | 2172 | 179 | 7.61% |
| random | 500 | 2172 | 82 | 419 | 83.63% |
| lawnmower | 2715 | 2172 | 2172 | 544 | 20.03% |
| stc | 2188 | 2172 | 2172 | 17 | 0.78% |
