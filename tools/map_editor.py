#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import numpy as np


@dataclass(frozen=True)
class MapMeta:
    resolution: float
    origin: Tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PGM map editor (matplotlib-based)")
    parser.add_argument("--width", type=float, default=5.0, help="Map width in meters")
    parser.add_argument("--height", type=float, default=5.0, help="Map height in meters")
    parser.add_argument("--resolution", type=float, default=0.05, help="Meters per cell")
    parser.add_argument(
        "--origin",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "THETA"),
        help="Origin [x y theta] in meters/radians",
    )
    parser.add_argument("--output-dir", type=str, default="tools", help="Output directory")
    parser.add_argument("--basename", type=str, default="map", help="Output base name")
    return parser.parse_args()


def clamp(value: int, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, value))


def apply_brush(grid: np.ndarray, row: int, col: int, value: int, radius: int) -> None:
    height, width = grid.shape
    row_min = clamp(row - radius, 0, height - 1)
    row_max = clamp(row + radius, 0, height - 1)
    col_min = clamp(col - radius, 0, width - 1)
    col_max = clamp(col + radius, 0, width - 1)
    grid[row_min : row_max + 1, col_min : col_max + 1] = value


def save_pgm(path: Path, grid: np.ndarray) -> None:
    height, width = grid.shape
    header = f"P5\n{width} {height}\n255\n"
    with path.open("wb") as file:
        file.write(header.encode("ascii"))
        # Flip vertically so the file matches ROS map_server expectations.
        file.write(np.flipud(grid).tobytes())


def save_yaml(path: Path, image_path: str, meta: MapMeta) -> None:
    x, y, theta = meta.origin
    content = (
        f"image: {image_path}\n"
        f"resolution: {meta.resolution}\n"
        f"origin: [{x:.6f}, {y:.6f}, {theta:.6f}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    path.write_text(content, encoding="ascii")


def main() -> None:
    args = parse_args()

    if args.width <= 0.0 or args.height <= 0.0:
        raise SystemExit("width and height must be positive")
    if args.resolution <= 0.0:
        raise SystemExit("resolution must be positive")

    map_width_meters = args.width
    map_height_meters = args.height
    map_width_cells = max(1, int(round(map_width_meters / args.resolution)))
    map_height_cells = max(1, int(round(map_height_meters / args.resolution)))
    grid = np.full((map_height_cells, map_width_cells), 254, dtype=np.uint8)
    map_width = map_width_cells
    map_height = map_height_cells
    state = {"drawing": False, "value": 0, "resolution": args.resolution}

    def map_extent() -> tuple:
        resolution = state["resolution"]
        return map_width * resolution, map_height * resolution

    def brush_radius_from_resolution(resolution: float) -> int:
        brush_width_meters = resolution
        brush_width_cells = max(1.0, brush_width_meters / resolution)
        return int(round((brush_width_cells - 1.0) / 2.0))

    brush_radius = brush_radius_from_resolution(state["resolution"])

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("PGM Map Editor")
    plt.subplots_adjust(left=0.05, right=0.75, bottom=0.05, top=0.95)

    extent_width, extent_height = map_extent()
    image = ax.imshow(
        grid,
        cmap="gray",
        origin="lower",
        vmin=0,
        vmax=255,
        interpolation="nearest",
        extent=[0.0, extent_width, 0.0, extent_height],
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Left: obstacle, Right: free")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    fig.text(0.8, 0.88, f"resolution: {args.resolution}", fontsize=10)
    fig.text(0.8, 0.83, f"size: {map_width_meters} x {map_height_meters} m", fontsize=9)
    origin_x_box = TextBox(
        plt.axes([0.8, 0.76, 0.18, 0.05]),
        "origin x",
        initial=str(args.origin[0]),
    )
    origin_y_box = TextBox(
        plt.axes([0.8, 0.69, 0.18, 0.05]),
        "origin y",
        initial=str(args.origin[1]),
    )
    origin_t_box = TextBox(
        plt.axes([0.8, 0.62, 0.18, 0.05]),
        "origin t",
        initial=str(args.origin[2]),
    )

    save_button = Button(plt.axes([0.8, 0.5, 0.18, 0.06]), "Save")
    clear_button = Button(plt.axes([0.8, 0.42, 0.18, 0.06]), "Clear")

    def parse_meta() -> MapMeta:
        origin = (float(origin_x_box.text), float(origin_y_box.text), float(origin_t_box.text))
        return MapMeta(resolution=args.resolution, origin=origin)


    def on_press(event) -> None:
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        if event.button == 1:
            state["value"] = 0
        elif event.button == 3:
            state["value"] = 254
        else:
            return
        state["drawing"] = True
        resolution = state["resolution"]
        row = clamp(int(event.ydata / resolution), 0, map_height - 1)
        col = clamp(int(event.xdata / resolution), 0, map_width - 1)
        apply_brush(grid, row, col, state["value"], brush_radius)
        image.set_data(grid)
        fig.canvas.draw_idle()

    def on_release(event) -> None:
        state["drawing"] = False

    def on_move(event) -> None:
        if not state["drawing"] or event.inaxes != ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        resolution = state["resolution"]
        row = clamp(int(event.ydata / resolution), 0, map_height - 1)
        col = clamp(int(event.xdata / resolution), 0, map_width - 1)
        apply_brush(grid, row, col, state["value"], brush_radius)
        image.set_data(grid)
        fig.canvas.draw_idle()

    def on_save(event) -> None:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = output_dir / f"{args.basename}.pgm"
        yaml_path = output_dir / f"{args.basename}.yaml"
        meta = parse_meta()
        save_pgm(pgm_path, grid)
        save_yaml(yaml_path, pgm_path.name, meta)
        ax.set_title(f"Saved to {pgm_path} and {yaml_path}")
        fig.canvas.draw_idle()

    def on_clear(event) -> None:
        grid[:, :] = 254
        image.set_data(grid)
        fig.canvas.draw_idle()

    def clamp_view(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def clamp_window(center: float, span: float, min_value: float, max_value: float) -> tuple:
        half = span / 2.0
        min_edge = center - half
        max_edge = center + half

        if min_edge < min_value:
            min_edge = min_value
            max_edge = min_value + span
        if max_edge > max_value:
            max_edge = max_value
            min_edge = max_value - span

        min_edge = clamp_view(min_edge, min_value, max_value)
        max_edge = clamp_view(max_edge, min_value, max_value)
        return min_edge, max_edge

    def on_scroll(event) -> None:
        if event.inaxes != ax:
            return
        if event.button not in ("up", "down"):
            return

        zoom_factor = 0.9 if event.button == "up" else 1.1
        current_xlim = ax.get_xlim()
        current_ylim = ax.get_ylim()
        center_x = event.xdata if event.xdata is not None else sum(current_xlim) / 2.0
        center_y = event.ydata if event.ydata is not None else sum(current_ylim) / 2.0

        x_span = (current_xlim[1] - current_xlim[0]) * zoom_factor
        y_span = (current_ylim[1] - current_ylim[0]) * zoom_factor
        span = max(x_span, y_span)
        min_span = state["resolution"]

        if span < min_span:
            return

        extent_width, extent_height = map_extent()
        x_min, x_max = clamp_window(center_x, span, 0.0, extent_width)
        y_min, y_max = clamp_window(center_y, span, 0.0, extent_height)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_move)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    save_button.on_clicked(on_save)
    clear_button.on_clicked(on_clear)

    plt.show()


if __name__ == "__main__":
    main()

# python3 tools/map_editor.py --width 100 --height 100 --resolution 0.05 --origin 0 0 0 --output-dir tools --basename map