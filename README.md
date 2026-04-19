# Final Project

## Layout

- `main.cpp` - runs the demo
- `test.cpp` / `test.h` - very simple traversal algorithm only
- `map_io.cpp` / `map_io.h` - map loading
- `lawnmower.cpp` / `lawnmower.h` - lawnmower planner using grid boustrophedon decomposition
- `maps/` - test maps
- `visualizer.py` - saves coverage images and GIF animations
- `outputs/trajectories/` - generated trajectory CSVs
- `outputs/images/` - generated coverage images
- `outputs/gifs/` - generated coverage animations

## Build

```bash
g++ -std=c++17 -O2 main.cpp map_io.cpp frontier_planner.cpp random_planner.cpp lawnmower.cpp stc.cpp stc_twostep.cpp
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

Results below are from one run of each planner on every map in `maps/`, except `large_300.txt` which was intentionally skipped.
`random` is stochastic, so its numbers will vary between runs.

### Apartment (10x12)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 1.267 | 127 | 108 | 108 | 100.00% | 20 | 15.63% |
| random | 0.838 | 500 | 108 | 105 | 97.22% | 396 | 79.04% |
| lawnmower | 1.174 | 158 | 108 | 108 | 100.00% | 51 | 32.08% |
| stc | 0.681 | 20 | 108 | 20 | 18.52% | 1 | 4.76% |
| stc_twostep | 0.713 | 20 | 108 | 20 | 18.52% | 1 | 4.76% |

### Cluttered Room (10x12)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 1.051 | 127 | 92 | 92 | 100.00% | 36 | 28.13% |
| random | 0.171 | 500 | 92 | 77 | 83.70% | 424 | 84.63% |
| lawnmower | 1.064 | 113 | 92 | 92 | 100.00% | 22 | 19.30% |
| stc | 0.714 | 92 | 92 | 92 | 100.00% | 1 | 1.08% |
| stc_twostep | 0.892 | 96 | 92 | 92 | 100.00% | 5 | 5.15% |

### Large Map 2 (50x50)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 577.097 | 2147 | 1540 | 1540 | 100.00% | 608 | 28.31% |
| random | 0.179 | 500 | 1540 | 122 | 7.92% | 379 | 75.65% |
| lawnmower | 34.948 | 2616 | 1540 | 1540 | 100.00% | 1077 | 41.15% |
| stc | 1.201 | 1804 | 1540 | 1540 | 100.00% | 265 | 14.68% |
| stc_twostep | 1.053 | 1564 | 1540 | 1540 | 100.00% | 25 | 1.60% |

### Large Room (50x50)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 1,412.210 | 2350 | 2172 | 2172 | 100.00% | 179 | 7.61% |
| random | 0.183 | 500 | 2172 | 101 | 4.65% | 400 | 79.84% |
| lawnmower | 50.414 | 2591 | 2172 | 2172 | 100.00% | 420 | 16.20% |
| stc | 2.203 | 2188 | 2172 | 2172 | 100.00% | 17 | 0.78% |
| stc_twostep | 3.497 | 2287 | 2172 | 2152 | 99.08% | 136 | 5.94% |

### Larger Room (50x50)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 1,352.170 | 2350 | 2172 | 2172 | 100.00% | 179 | 7.61% |
| random | 0.117 | 500 | 2172 | 144 | 6.63% | 357 | 71.26% |
| lawnmower | 41.924 | 2591 | 2172 | 2172 | 100.00% | 420 | 16.20% |
| stc | 1.986 | 2188 | 2172 | 2172 | 100.00% | 17 | 0.78% |
| stc_twostep | 1.800 | 2287 | 2172 | 2152 | 99.08% | 136 | 5.94% |

### Maze 100 (100x100)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 11,926.800 | 8536 | 6136 | 6136 | 100.00% | 2401 | 28.12% |
| random | 0.208 | 500 | 6136 | 39 | 0.64% | 462 | 92.22% |
| lawnmower | 220.266 | 9243 | 6136 | 6136 | 100.00% | 3108 | 33.62% |
| stc | 3.237 | 6329 | 6136 | 6106 | 99.51% | 224 | 3.54% |
| stc_twostep | 3.891 | 6453 | 6136 | 6006 | 97.88% | 448 | 6.94% |

### Narrow Corridor (8x10)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 0.494 | 72 | 53 | 53 | 100.00% | 20 | 27.40% |
| random | 0.144 | 500 | 53 | 43 | 81.13% | 458 | 91.42% |
| lawnmower | 1.024 | 95 | 53 | 53 | 100.00% | 43 | 44.79% |
| stc | 0.815 | 0 | 53 | 0 | 0.00% | 0 | 0.00% |
| stc_twostep | 0.015 | 0 | 53 | 0 | 0.00% | 0 | 0.00% |

### Open Room (8x10)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 0.966 | 77 | 76 | 76 | 100.00% | 2 | 2.56% |
| random | 0.125 | 500 | 76 | 74 | 97.37% | 427 | 85.23% |
| lawnmower | 0.997 | 86 | 76 | 76 | 100.00% | 11 | 12.64% |
| stc | 0.749 | 68 | 76 | 64 | 84.21% | 5 | 7.25% |
| stc_twostep | 4.030 | 64 | 76 | 64 | 84.21% | 1 | 1.54% |

### Snake Maze (10x10)

| Algorithm | Planning Time (ms) | Total Steps | Reachable Free Cells | Unique Visited Cells | Coverage % | Revisit Count | Revisit % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| frontier | 0.352 | 63 | 64 | 64 | 100.00% | 0 | 0.00% |
| random | 0.206 | 500 | 64 | 25 | 39.06% | 476 | 95.01% |
| lawnmower | 1.322 | 63 | 64 | 64 | 100.00% | 0 | 0.00% |
| stc | 0.669 | 0 | 64 | 0 | 0.00% | 0 | 0.00% |
| stc_twostep | 0.010 | 0 | 64 | 0 | 0.00% | 0 | 0.00% |
