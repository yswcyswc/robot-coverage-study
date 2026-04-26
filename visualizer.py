from pathlib import Path
import csv
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

MAX_GIF_FRAMES = 120


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


def is_valid_major_cell(cell_row, cell_col, rows, cols, grid):
    base_row = 2 * cell_row
    base_col = 2 * cell_col
    if base_row + 1 >= rows or base_col + 1 >= cols:
        return False

    return (
        grid[base_row][base_col] == "."
        and grid[base_row][base_col + 1] == "."
        and grid[base_row + 1][base_col] == "."
        and grid[base_row + 1][base_col + 1] == "."
    )


def major_cell_points(cell):
    row, col = cell
    return [
        (2 * row, 2 * col),
        (2 * row, 2 * col + 1),
        (2 * row + 1, 2 * col),
        (2 * row + 1, 2 * col + 1),
    ]


def compute_twostep_layers(rows, cols, grid, current, visited):
    if rows % 2 != 0 or cols % 2 != 0:
        return set(), set()

    current_major = (current[0] // 2, current[1] // 2)
    visited_major = {(row // 2, col // 2) for row, col in visited}
    cell_rows = rows // 2
    cell_cols = cols // 2
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    immediate = set()
    second_step = set()

    if is_valid_major_cell(current_major[0], current_major[1], rows, cols, grid):
        immediate.add(current_major)

    for dr, dc in directions:
        neighbor = (current_major[0] + dr, current_major[1] + dc)
        if not (0 <= neighbor[0] < cell_rows and 0 <= neighbor[1] < cell_cols):
            continue
        if neighbor in visited_major:
            continue
        if not is_valid_major_cell(neighbor[0], neighbor[1], rows, cols, grid):
            continue

        immediate.add(neighbor)

        ahead = (neighbor[0] + dr, neighbor[1] + dc)
        if (
            0 <= ahead[0] < cell_rows
            and 0 <= ahead[1] < cell_cols
            and ahead not in visited_major
            and is_valid_major_cell(ahead[0], ahead[1], rows, cols, grid)
        ):
            second_step.add(ahead)

        for side_dr, side_dc in directions:
            candidate = (neighbor[0] + side_dr, neighbor[1] + side_dc)
            if (
                0 <= candidate[0] < cell_rows
                and 0 <= candidate[1] < cell_cols
                and candidate not in visited_major
                and candidate != current_major
                and is_valid_major_cell(candidate[0], candidate[1], rows, cols, grid)
            ):
                second_step.add(candidate)

    second_step -= immediate
    return immediate, second_step


def compute_twostep_frontier(rows, cols, grid, trajectory):
    discovered = set()
    for index, current in enumerate(trajectory):
        visited = set(trajectory[: index + 1])
        immediate, second_step = compute_twostep_layers(rows, cols, grid, current, visited)
        discovered.update(immediate)
        discovered.update(second_step)

    return discovered


def compute_twostep_frontier_history(rows, cols, grid, trajectory):
    discovered = set()
    visited = set()
    history = []

    for current in trajectory:
        visited.add(current)
        immediate, second_step = compute_twostep_layers(rows, cols, grid, current, visited)
        discovered.update(immediate)
        discovered.update(second_step)
        history.append(set(discovered))

    return history


def compute_twostep_frontier_snapshots(rows, cols, grid, trajectory, frame_indices):
    wanted = set(int(index) for index in frame_indices)
    wanted.add(len(trajectory) - 1)
    discovered = set()
    visited = set()
    snapshots = {}

    for index, current in enumerate(trajectory):
        visited.add(current)
        immediate, second_step = compute_twostep_layers(rows, cols, grid, current, visited)
        discovered.update(immediate)
        discovered.update(second_step)

        if index in wanted:
            snapshots[index] = set(discovered)

    return snapshots


def build_image(rows, cols, grid, visited, frontier, immediate=None, second_step=None):
    immediate = immediate or set()
    second_step = second_step or set()
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

    for cell in second_step:
        for r, c in major_cell_points(cell):
            if 0 <= r < rows and 0 <= c < cols and grid[r][c] == "." and (r, c) not in visited:
                image[r, c] = np.array([0.97, 0.77, 0.24])

    for cell in immediate:
        for r, c in major_cell_points(cell):
            if 0 <= r < rows and 0 <= c < cols and grid[r][c] == "." and (r, c) not in visited:
                image[r, c] = np.array([0.97, 0.77, 0.24])

    return image


def main():
    if len(sys.argv) < 2:
        print("Usage: python visualizer.py <map_file> <optional: algorithm_name>")
        sys.exit(1)

    map_path = Path(sys.argv[1])

    algo_name = sys.argv[2] if len(sys.argv) > 2 else "frontier"
    extra_args = sys.argv[3:]
    save_gif = "--no-gif" not in extra_args

    max_gif_frames = MAX_GIF_FRAMES
    if "--frames" in extra_args:
        frame_arg_index = extra_args.index("--frames") + 1
        if frame_arg_index >= len(extra_args):
            print("Missing frame count after --frames")
            sys.exit(1)
        max_gif_frames = max(1, int(extra_args[frame_arg_index]))

    trajectory_path = Path("outputs") / "trajectories" / f"{map_path.stem}_{algo_name}_trajectory.csv"
    
    if not trajectory_path.exists():
        print(f"Missing trajectory file: {trajectory_path}")
        sys.exit(1)

    rows, cols, start, grid = load_map(map_path)
    trajectory = load_trajectory(trajectory_path)
    if not trajectory:
        print(f"Trajectory is empty: {trajectory_path}; rendering stationary start state.")
        trajectory = [start]

    Path("outputs").mkdir(exist_ok=True)
    Path("outputs/trajectories").mkdir(parents=True, exist_ok=True)
    Path("outputs/images").mkdir(parents=True, exist_ok=True)
    Path("outputs/gifs").mkdir(parents=True, exist_ok=True)
    output_png = Path("outputs") / "images" / f"{map_path.stem}_coverage_{algo_name}.png"
    output_gif = Path("outputs") / "gifs" / f"{map_path.stem}_coverage_{algo_name}.gif"
    frame_count = min(len(trajectory), max_gif_frames)
    frame_indices = np.linspace(0, len(trajectory) - 1, frame_count, dtype=int)
    twostep_frontier_snapshots = {}
    if algo_name == "stc_twostep":
        twostep_frontier_snapshots = compute_twostep_frontier_snapshots(
            rows, cols, grid, trajectory, frame_indices)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title(f"Coverage Path: {map_path.stem}")
    if rows <= 30 and cols <= 30:
        ax.set_xticks(range(cols))
        ax.set_yticks(range(rows))
        ax.grid(color="white", linewidth=0.5)
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)

    empty_visited = set()
    empty_frontier = compute_frontier(rows, cols, grid, empty_visited)
    image_artist = ax.imshow(build_image(rows, cols, grid, empty_visited, empty_frontier), origin="upper")
    path_line, = ax.plot([], [], color="#c44536", linewidth=2, label="trajectory")
    start_marker = ax.scatter([start[1]], [start[0]], color="#2a9d8f", s=70, label="start", zorder=3)
    robot_marker = ax.scatter([], [], color="#1d3557", s=90, label="robot", zorder=4)
    frontier_patch = plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#f7c53d",
                                markersize=10, linestyle="", label="frontier")
    legend_handles = [start_marker, robot_marker, path_line, frontier_patch]
    ax.legend(
        handles=legend_handles,
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        borderaxespad=0,
    )

    final_visited = set(trajectory)
    final_frontier = compute_frontier(rows, cols, grid, final_visited)
    final_immediate = set()
    final_second_step = set()
    if algo_name == "stc_twostep":
        final_frontier = set()
        final_immediate = twostep_frontier_snapshots[len(trajectory) - 1]
    final_image = build_image(
        rows,
        cols,
        grid,
        final_visited,
        final_frontier,
        final_immediate,
        final_second_step,
    )
    image_artist.set_data(final_image)
    path_line.set_data([col for _, col in trajectory], [row for row, _ in trajectory])
    robot_row, robot_col = trajectory[-1]
    robot_marker.set_offsets([[robot_col, robot_row]])
    plt.tight_layout(rect=[0, 0, 0.82, 1])
    plt.savefig(output_png, dpi=200)
    print(f"Saved image to {output_png}")

    def update(frame_index):
        partial = trajectory[: frame_index + 1]
        visited = set(partial)
        frontier = compute_frontier(rows, cols, grid, visited)
        immediate = set()
        second_step = set()
        if algo_name == "stc_twostep":
            frontier = set()
            immediate = twostep_frontier_snapshots[frame_index]
        image_artist.set_data(build_image(rows, cols, grid, visited, frontier, immediate, second_step))

        xs = [col for _, col in partial]
        ys = [row for row, _ in partial]
        path_line.set_data(xs, ys)

        robot_row, robot_col = partial[-1]
        robot_marker.set_offsets([[robot_col, robot_row]])
        return image_artist, path_line, robot_marker

    if save_gif:
        animation = FuncAnimation(
            fig,
            update,
            frames=frame_indices,
            interval=200,
            blit=False,
            repeat=False,
        )

        plt.tight_layout(rect=[0, 0, 0.82, 1])
        try:
            if len(trajectory) > max_gif_frames:
                print(
                    f"Trajectory has {len(trajectory)} steps; "
                    f"saving sampled GIF with {max_gif_frames} frames."
                )
            animation.save(output_gif, writer="pillow", fps=5)
            print(f"Saved animation to {output_gif}")
        except Exception as exc:
            print(f"Could not save GIF automatically: {exc}")
    else:
        print("Skipped GIF because --no-gif was provided.")

    plt.close(fig)


if __name__ == "__main__":
    main()
