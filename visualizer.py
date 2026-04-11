from pathlib import Path
import csv
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np


def load_map(map_path):
    with open(map_path, "r", encoding="utf-8") as f:
        rows, cols = map(int, f.readline().split())
        start_row, start_col = map(int, f.readline().split())
        grid = [list(f.readline().strip()) for _ in range(rows)]

    return rows, cols, (start_row, start_col), grid


def load_trajectory(csv_path):
    path = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            path.append((int(row["row"]), int(row["col"])))
    return path


def compute_frontier(rows, cols, grid, visited):
    frontier = set()
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    for row, col in visited:
        for dr, dc in directions:
            nr = row + dr
            nc = col + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] == "." and (nr, nc) not in visited:
                    frontier.add((nr, nc))

    return frontier


def build_image(rows, cols, grid, visited, frontier):
    image = np.ones((rows, cols, 3), dtype=float)
    image[:] = np.array([0.95, 0.95, 0.92])

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == "#":
                image[r, c] = np.array([0.15, 0.15, 0.15])

    for r, c in visited:
        image[r, c] = np.array([0.64, 0.82, 0.93])

    for r, c in frontier:
        image[r, c] = np.array([0.97, 0.77, 0.24])

    return image


def main():
    if len(sys.argv) < 2:
        print("Usage: python visualizer.py <map_file> <optional: algorithm_name>")
        sys.exit(1)

    map_path = Path(sys.argv[1])

    algo_name = sys.argv[2] if len(sys.argv) > 2 else "frontier" 

    trajectory_path = Path("outputs") / "trajectories" / f"{map_path.stem}_{algo_name}_trajectory.csv"
    
    if not trajectory_path.exists():
        print(f"Missing trajectory file: {trajectory_path}")
        sys.exit(1)

    rows, cols, start, grid = load_map(map_path)
    trajectory = load_trajectory(trajectory_path)
    Path("outputs").mkdir(exist_ok=True)
    Path("outputs/trajectories").mkdir(parents=True, exist_ok=True)
    Path("outputs/images").mkdir(parents=True, exist_ok=True)
    Path("outputs/gifs").mkdir(parents=True, exist_ok=True)
    output_png = Path("outputs") / "images" / f"{map_path.stem}_coverage_{algo_name}.png"
    output_gif = Path("outputs") / "gifs" / f"{map_path.stem}_coverage_{algo_name}.gif"

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title(f"Coverage Path: {map_path.stem}")
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.grid(color="white", linewidth=0.5)

    empty_visited = set()
    empty_frontier = compute_frontier(rows, cols, grid, empty_visited)
    image_artist = ax.imshow(build_image(rows, cols, grid, empty_visited, empty_frontier), origin="upper")
    path_line, = ax.plot([], [], color="#c44536", linewidth=2, label="trajectory")
    start_marker = ax.scatter([start[1]], [start[0]], color="#2a9d8f", s=70, label="start", zorder=3)
    robot_marker = ax.scatter([], [], color="#1d3557", s=90, label="robot", zorder=4)
    frontier_patch = plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#f7c53d",
                                markersize=10, linestyle="", label="frontier")
    ax.legend(handles=[start_marker, robot_marker, path_line, frontier_patch])

    def update(frame):
        partial = trajectory[: frame + 1]
        visited = set(partial)
        frontier = compute_frontier(rows, cols, grid, visited)
        image_artist.set_data(build_image(rows, cols, grid, visited, frontier))

        xs = [col for _, col in partial]
        ys = [row for row, _ in partial]
        path_line.set_data(xs, ys)

        robot_row, robot_col = partial[-1]
        robot_marker.set_offsets([[robot_col, robot_row]])
        return image_artist, path_line, robot_marker

    animation = FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=200,
        blit=False,
        repeat=False,
    )

    plt.tight_layout()
    try:
        animation.save(output_gif, writer="pillow", fps=5)
        print(f"Saved animation to {output_gif}")
    except Exception as exc:
        print(f"Could not save GIF automatically: {exc}")

    final_visited = set(trajectory)
    final_frontier = compute_frontier(rows, cols, grid, final_visited)
    final_image = build_image(rows, cols, grid, final_visited, final_frontier)
    image_artist.set_data(final_image)
    path_line.set_data([col for _, col in trajectory], [row for row, _ in trajectory])
    plt.savefig(output_png, dpi=200)
    print(f"Saved image to {output_png}")
    plt.close(fig)


if __name__ == "__main__":
    main()
