#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import yaml


@dataclass(frozen=True)
class MapMeta:
    width: float
    height: float
    resolution: float
    origin: tuple[float, float, float]


DEFAULT_SCHEMA = """map:
  width: 10.0
  height: 10.0
  resolution: 0.05
  origin: [0.0, 0.0, 0.0]

lines:
  - start: [1.0, 1.0]
    end: [9.0, 1.0]
    thickness: 0.20
  - start: [1.0, 1.0]
    end: [1.0, 9.0]
    thickness: 0.20
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="YAMLスキーマを監視してリアルタイムに地図をプレビューするツール"
    )
    parser.add_argument(
        "--schema",
        type=str,
        default="tools/map_schema.yaml",
        help="監視対象のYAMLスキーマ",
    )
    parser.add_argument("--output-dir", type=str, default="tools", help="出力先ディレクトリ")
    parser.add_argument("--basename", type=str, default="map", help="出力ファイル名ベース")
    parser.add_argument(
        "--poll-interval-ms",
        type=int,
        default=300,
        help="YAML変更監視間隔 (ms)",
    )
    return parser.parse_args()


def ensure_schema_file(schema_path: Path) -> None:
    if schema_path.exists():
        return
    schema_path.parent.mkdir(parents=True, exist_ok=True)
    schema_path.write_text(DEFAULT_SCHEMA, encoding="utf-8")


def parse_pair(value: Any, key_name: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{key_name} must be [x, y]")
    return float(value[0]), float(value[1])


def parse_origin(value: Any) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("map.origin must be [x, y, theta]")
    return float(value[0]), float(value[1]), float(value[2])


def load_schema(schema_path: Path) -> tuple[MapMeta, list[dict[str, float]]]:
    raw = yaml.safe_load(schema_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("root must be a mapping")

    map_raw = raw.get("map")
    if not isinstance(map_raw, dict):
        raise ValueError("map must be a mapping")

    width = float(map_raw.get("width"))
    height = float(map_raw.get("height"))
    resolution = float(map_raw.get("resolution"))
    origin = parse_origin(map_raw.get("origin"))

    if width <= 0.0 or height <= 0.0:
        raise ValueError("map.width and map.height must be positive")
    if resolution <= 0.0:
        raise ValueError("map.resolution must be positive")

    lines_raw = raw.get("lines", [])
    if not isinstance(lines_raw, list):
        raise ValueError("lines must be a list")

    lines: list[dict[str, float]] = []
    for index, line in enumerate(lines_raw):
        if not isinstance(line, dict):
            raise ValueError(f"lines[{index}] must be a mapping")
        start_x, start_y = parse_pair(line.get("start"), f"lines[{index}].start")
        end_x, end_y = parse_pair(line.get("end"), f"lines[{index}].end")
        thickness = float(line.get("thickness"))
        if thickness <= 0.0:
            raise ValueError(f"lines[{index}].thickness must be positive")
        lines.append(
            {
                "start_x": start_x,
                "start_y": start_y,
                "end_x": end_x,
                "end_y": end_y,
                "thickness": thickness,
            }
        )

    return MapMeta(width=width, height=height, resolution=resolution, origin=origin), lines


def clamp(value: int, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, value))


def draw_line_on_grid(grid: np.ndarray, meta: MapMeta, line: dict[str, float]) -> None:
    start = np.array([line["start_x"], line["start_y"]], dtype=float)
    end = np.array([line["end_x"], line["end_y"]], dtype=float)
    thickness = line["thickness"]
    radius = thickness / 2.0

    min_x = min(start[0], end[0]) - radius
    max_x = max(start[0], end[0]) + radius
    min_y = min(start[1], end[1]) - radius
    max_y = max(start[1], end[1]) + radius

    height_cells, width_cells = grid.shape
    col_min = clamp(int(np.floor(min_x / meta.resolution)), 0, width_cells - 1)
    col_max = clamp(int(np.ceil(max_x / meta.resolution)), 0, width_cells - 1)
    row_min = clamp(int(np.floor(min_y / meta.resolution)), 0, height_cells - 1)
    row_max = clamp(int(np.ceil(max_y / meta.resolution)), 0, height_cells - 1)

    segment = end - start
    segment_length_sq = float(np.dot(segment, segment))

    for row in range(row_min, row_max + 1):
        center_y = (row + 0.5) * meta.resolution
        for col in range(col_min, col_max + 1):
            center_x = (col + 0.5) * meta.resolution
            point = np.array([center_x, center_y], dtype=float)

            if segment_length_sq == 0.0:
                closest = start
            else:
                t = float(np.dot(point - start, segment) / segment_length_sq)
                t = max(0.0, min(1.0, t))
                closest = start + t * segment

            distance = float(np.linalg.norm(point - closest))
            if distance <= radius:
                grid[row, col] = 0


def render_grid(meta: MapMeta, lines: list[dict[str, float]]) -> np.ndarray:
    width_cells = max(1, int(round(meta.width / meta.resolution)))
    height_cells = max(1, int(round(meta.height / meta.resolution)))
    grid = np.full((height_cells, width_cells), 254, dtype=np.uint8)
    for line in lines:
        draw_line_on_grid(grid, meta, line)
    return grid


def save_pgm(path: Path, grid: np.ndarray) -> None:
    height, width = grid.shape
    header = f"P5\n{width} {height}\n255\n"
    with path.open("wb") as file:
        file.write(header.encode("ascii"))
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
    schema_path = Path(args.schema)
    ensure_schema_file(schema_path)

    state: dict[str, Any] = {
        "meta": None,
        "grid": None,
        "schema_mtime": None,
        "status": "",
    }

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("YAML Map Schema Editor")
    plt.subplots_adjust(right=0.78)

    placeholder = np.full((10, 10), 254, dtype=np.uint8)
    image = ax.imshow(
        placeholder,
        cmap="gray",
        origin="lower",
        vmin=0,
        vmax=255,
        interpolation="nearest",
        extent=[0.0, 10.0, 0.0, 10.0],
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    status_text = fig.text(0.80, 0.80, "", fontsize=9)
    fig.text(0.80, 0.90, f"schema:\n{schema_path}", fontsize=9)

    save_button = Button(plt.axes([0.80, 0.72, 0.16, 0.06]), "Save")

    def update_preview(force: bool = False) -> None:
        try:
            mtime = schema_path.stat().st_mtime
        except FileNotFoundError:
            state["status"] = f"schema not found: {schema_path}"
            ax.set_title(state["status"])
            status_text.set_text(state["status"])
            fig.canvas.draw_idle()
            return

        if not force and state["schema_mtime"] == mtime:
            return

        try:
            meta, lines = load_schema(schema_path)
            grid = render_grid(meta, lines)
        except Exception as error:
            state["status"] = f"schema error: {error}"
            ax.set_title(state["status"])
            status_text.set_text(state["status"])
            fig.canvas.draw_idle()
            return

        state["meta"] = meta
        state["grid"] = grid
        state["schema_mtime"] = mtime
        state["status"] = f"loaded lines: {len(lines)}"

        image.set_data(grid)
        image.set_extent([0.0, meta.width, 0.0, meta.height])
        ax.set_xlim(0.0, meta.width)
        ax.set_ylim(0.0, meta.height)
        ax.set_title(state["status"])
        status_text.set_text(state["status"])
        fig.canvas.draw_idle()

    def on_save(event) -> None:
        if state["grid"] is None or state["meta"] is None:
            state["status"] = "save failed: valid schema is not loaded"
            ax.set_title(state["status"])
            status_text.set_text(state["status"])
            fig.canvas.draw_idle()
            return

        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = output_dir / f"{args.basename}.pgm"
        yaml_path = output_dir / f"{args.basename}.yaml"

        save_pgm(pgm_path, state["grid"])
        save_yaml(yaml_path, pgm_path.name, state["meta"])
        state["status"] = f"saved: {pgm_path.name}, {yaml_path.name}"
        ax.set_title(state["status"])
        status_text.set_text(state["status"])
        fig.canvas.draw_idle()

    save_button.on_clicked(on_save)

    timer = fig.canvas.new_timer(interval=max(50, args.poll_interval_ms))
    timer.add_callback(update_preview)
    timer.start()

    update_preview(force=True)
    plt.show()


if __name__ == "__main__":
    main()
