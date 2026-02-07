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
    parser.add_argument("--width", type=int, default=100, help="Map width in cells")
    parser.add_argument("--height", type=int, default=100, help="Map height in cells")
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

    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")

    grid = np.full((args.height, args.width), 254, dtype=np.uint8)
    brush_radius = 0

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("PGM Map Editor")
    plt.subplots_adjust(left=0.05, right=0.75, bottom=0.05, top=0.95)

    image = ax.imshow(
        grid,
        cmap="gray",
        origin="lower",
        vmin=0,
        vmax=255,
        interpolation="nearest",
    )
    ax.set_title("Left: obstacle, Right: free")
    ax.set_xlabel("x (cells)")
    ax.set_ylabel("y (cells)")

    resolution_box = TextBox(
        plt.axes([0.8, 0.85, 0.18, 0.05]),
        "resolution",
        initial=str(args.resolution),
    )
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

    state = {"drawing": False, "value": 0}

    def parse_meta() -> MapMeta:
        resolution = float(resolution_box.text)
        origin = (float(origin_x_box.text), float(origin_y_box.text), float(origin_t_box.text))
        return MapMeta(resolution=resolution, origin=origin)

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
        row = int(event.ydata)
        col = int(event.xdata)
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
        row = int(event.ydata)
        col = int(event.xdata)
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

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_move)
    save_button.on_clicked(on_save)
    clear_button.on_clicked(on_clear)

    plt.show()


if __name__ == "__main__":
    main()
