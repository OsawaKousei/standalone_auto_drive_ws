#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DIAG_PREFIX = "[ransac_diag]"
KV_PATTERN = re.compile(r"([a-zA-Z0-9_]+)=([^\s]+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RANSAC debug timeline visualizer (map + scan + stage1 segments)"
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("test/localization/logs/localization_test.log"),
        help="Path to localization_test.log",
    )
    parser.add_argument(
        "--diag-log",
        type=Path,
        default=Path("test/localization/logs/ransac_diag.log"),
        help="Path to ransac_diag log",
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
        help="Scenario toml path (auto-resolved from log header when omitted)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/ransac_diag/timeline"),
        help="Output directory for timeline frames",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Render every N-th lidar frame",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Max frames to render (0 = all)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="FPS for GIF export",
    )
    parser.add_argument(
        "--no-gif",
        action="store_true",
        help="Disable GIF export (always saves PNG sequence)",
    )
    return parser.parse_args()


def parse_header_and_rows(log_path: Path) -> Tuple[Dict[str, str], List[str], List[Dict[str, str]]]:
    if not log_path.exists():
        raise FileNotFoundError(f"Log not found: {log_path}")

    header: Dict[str, str] = {}
    columns: List[str] = []
    rows: List[Dict[str, str]] = []

    with log_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue

            if line.startswith("#"):
                content = line[1:].strip()
                if "=" in content:
                    key, value = content.split("=", 1)
                    header[key.strip()] = value.strip()
                elif content.startswith("columns:"):
                    columns = [col.strip() for col in content.split(":", 1)[1].split(",")]
                continue

            if not columns:
                columns = [col.strip() for col in line.split(",")]
                continue

            parts = [part.strip() for part in next(csv.reader([line]))]
            if len(parts) != len(columns) or parts == columns:
                continue
            rows.append(dict(zip(columns, parts)))

    if not columns:
        raise ValueError("CSV columns not found in localization log")

    return header, columns, rows


def parse_scenario_and_map(scenario_path: Path) -> Tuple[Path, float, Tuple[float, float]]:
    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario not found: {scenario_path}")

    map_yaml_rel = None
    in_map_section = False
    for raw in scenario_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            in_map_section = line == "[map]"
            continue
        if in_map_section and line.startswith("yaml_path"):
            map_yaml_rel = line.split("=", 1)[1].strip().strip('"')
            break

    if map_yaml_rel is None:
        raise ValueError(f"yaml_path not found in scenario: {scenario_path}")

    map_yaml_path = (scenario_path.parent / map_yaml_rel).resolve()
    image_rel = None
    resolution = None
    origin_xy = None
    for raw in map_yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("image:"):
            image_rel = line.split(":", 1)[1].strip()
        elif line.startswith("resolution:"):
            resolution = float(line.split(":", 1)[1].strip())
        elif line.startswith("origin:"):
            raw_values = line.split(":", 1)[1].strip().removeprefix("[").removesuffix("]")
            vals = [float(v.strip()) for v in raw_values.split(",")]
            origin_xy = (vals[0], vals[1])

    if image_rel is None or resolution is None or origin_xy is None:
        raise ValueError(f"Invalid map yaml: {map_yaml_path}")

    map_image_path = (map_yaml_path.parent / image_rel).resolve()
    return map_image_path, resolution, origin_xy


def parse_points_robot(serialized: str) -> List[Tuple[float, float]]:
    if not serialized:
        return []
    points: List[Tuple[float, float]] = []
    for token in serialized.split(";"):
        if not token:
            continue
        parts = token.split(":")
        if len(parts) != 2:
            continue
        try:
            points.append((float(parts[0]), float(parts[1])))
        except ValueError:
            continue
    return points


def parse_segments(serialized: str) -> List[Tuple[float, float, float, float]]:
    if not serialized:
        return []
    segments: List[Tuple[float, float, float, float]] = []
    for token in serialized.split("|"):
        if not token:
            continue
        parts = token.split(":")
        if len(parts) != 4:
            continue
        try:
            x1, y1, x2, y2 = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
        except ValueError:
            continue
        segments.append((x1, y1, x2, y2))
    return segments


def transform_point_robot_to_world(x_r: float, y_r: float, pose_x: float, pose_y: float, theta: float) -> Tuple[float, float]:
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return pose_x + (cos_t * x_r) - (sin_t * y_r), pose_y + (sin_t * x_r) + (cos_t * y_r)


def parse_diag_rows(diag_log_path: Path) -> List[Dict[str, str]]:
    if not diag_log_path.exists():
        return []
    rows: List[Dict[str, str]] = []
    for raw in diag_log_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or DIAG_PREFIX not in line:
            continue
        row: Dict[str, str] = {}
        for key, value in KV_PATTERN.findall(line):
            row[key] = value
        if row:
            rows.append(row)
    return rows


def resolve_scenario_path(header: Dict[str, str], scenario_arg: Optional[Path]) -> Path:
    if scenario_arg is not None:
        return scenario_arg.resolve()
    raw = header.get("scenario_config")
    if raw:
        return Path(raw).resolve()
    return Path("test/localization/configs/localization.toml").resolve()


def build_lidar_frames(rows: List[Dict[str, str]]) -> List[Dict[str, object]]:
    frames: List[Dict[str, object]] = []
    for row in rows:
        if row.get("lidar_updated") != "1":
            continue

        points_robot = parse_points_robot(row.get("scan_points_robot", ""))
        true_x = float(row["true_x"])
        true_y = float(row["true_y"])
        true_theta = float(row["true_theta"])

        points_world = [
            transform_point_robot_to_world(x_r, y_r, true_x, true_y, true_theta)
            for x_r, y_r in points_robot
        ]

        frames.append(
            {
                "step": int(row["step"]),
                "time": float(row["time"]),
                "true_pose": (true_x, true_y, true_theta),
                "est_pose": (float(row["est_x"]), float(row["est_y"]), float(row["est_theta"])),
                "scan_world": points_world,
            }
        )
    return frames


def render_timeline(
    map_image_path: Path,
    map_resolution: float,
    map_origin: Tuple[float, float],
    lidar_frames: List[Dict[str, object]],
    diag_rows: List[Dict[str, str]],
    out_dir: Path,
    stride: int,
    max_frames: int,
    fps: int,
    disable_gif: bool,
) -> Dict[str, object]:
    image = plt.imread(map_image_path)
    if image.ndim == 3:
        image = image[..., 0]

    h, w = image.shape[:2]
    x0, y0 = map_origin
    extent = [x0, x0 + w * map_resolution, y0, y0 + h * map_resolution]

    frame_count = min(len(lidar_frames), len(diag_rows)) if diag_rows else len(lidar_frames)
    if frame_count <= 0:
        raise RuntimeError("No lidar frames available for timeline rendering")

    indices = list(range(0, frame_count, max(stride, 1)))
    if max_frames > 0:
        indices = indices[:max_frames]

    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    saved_paths: List[Path] = []
    for idx_out, idx_src in enumerate(indices):
        frame = lidar_frames[idx_src]
        diag = diag_rows[idx_src] if idx_src < len(diag_rows) else {}

        fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
        ax.imshow(image, cmap="gray", origin="upper", extent=extent)

        scan_world = frame["scan_world"]
        if scan_world:
            arr = np.asarray(scan_world, dtype=float)
            ax.scatter(arr[:, 0], arr[:, 1], s=4, c="cyan", alpha=0.6, label="scan")

        true_x, true_y, true_theta = frame["true_pose"]
        est_x, est_y, _ = frame["est_pose"]
        ax.scatter([true_x], [true_y], c="lime", s=32, label="true")
        ax.scatter([est_x], [est_y], c="magenta", s=28, label="estimate")

        segments = parse_segments(diag.get("stage1_segments", ""))
        for x1r, y1r, x2r, y2r in segments:
            x1w, y1w = transform_point_robot_to_world(x1r, y1r, true_x, true_y, true_theta)
            x2w, y2w = transform_point_robot_to_world(x2r, y2r, true_x, true_y, true_theta)
            ax.plot([x1w, x2w], [y1w, y2w], color="yellow", linewidth=2.0, alpha=0.9)

        stage1 = diag.get("stage1_extracted", "?")
        stage2 = diag.get("stage2_matches", "?")
        gate = diag.get("gate_passed", "?")
        accepted = diag.get("accepted", "?")
        reject = diag.get("reject", "-")

        ax.set_title(
            "RANSAC timeline "
            f"step={frame['step']} time={frame['time']:.2f}s\n"
            f"stage1={stage1} stage2={stage2} gate={gate} accepted={accepted} reject={reject}"
        )
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
        ax.legend(loc="upper right")

        out_path = frames_dir / f"frame_{idx_out:04d}.png"
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        saved_paths.append(out_path)

    gif_path = out_dir / "ransac_timeline.gif"
    gif_saved = False
    if not disable_gif and saved_paths:
        try:
            from PIL import Image

            images = [Image.open(path) for path in saved_paths]
            duration_ms = int(1000 / max(fps, 1))
            images[0].save(
                gif_path,
                save_all=True,
                append_images=images[1:],
                duration=duration_ms,
                loop=0,
            )
            gif_saved = True
            for image_obj in images:
                image_obj.close()
        except Exception as exc:
            print(f"GIF export skipped: {exc}")

    summary = {
        "rendered_frames": len(saved_paths),
        "frame_dir": str(frames_dir),
        "gif_path": str(gif_path) if gif_saved else "",
        "gif_saved": gif_saved,
        "used_diag_rows": bool(diag_rows),
        "aligned_frame_count": frame_count,
    }

    summary_path = out_dir / "timeline_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    args = parse_args()

    header, _columns, rows = parse_header_and_rows(args.log)
    scenario_path = resolve_scenario_path(header, args.scenario)
    map_image_path, map_resolution, map_origin = parse_scenario_and_map(scenario_path)

    lidar_frames = build_lidar_frames(rows)
    diag_rows = parse_diag_rows(args.diag_log)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = render_timeline(
        map_image_path=map_image_path,
        map_resolution=map_resolution,
        map_origin=map_origin,
        lidar_frames=lidar_frames,
        diag_rows=diag_rows,
        out_dir=out_dir,
        stride=args.stride,
        max_frames=args.max_frames,
        fps=args.fps,
        disable_gif=args.no_gif,
    )

    print(f"Rendered frames: {summary['rendered_frames']}")
    print(f"Frame directory: {summary['frame_dir']}")
    if summary["gif_saved"]:
        print(f"GIF: {summary['gif_path']}")
    else:
        print("GIF: not saved")
    print(f"Summary: {out_dir / 'timeline_summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
