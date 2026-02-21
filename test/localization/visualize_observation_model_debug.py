#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize Hough vs Hough+RANSAC observation debug CSV")
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("test/localization/logs/observation_model_debug.csv"),
        help="Path to observation model debug csv",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis"),
        help="Output directory",
    )
    return parser.parse_args()


def parse_point(raw: str) -> Tuple[float, float]:
    x_raw, y_raw = raw.split(":", 1)
    return float(x_raw), float(y_raw)


def parse_segment(raw: str) -> np.ndarray | None:
    raw = raw.strip()
    if not raw:
        return None
    parts = [part.strip() for part in raw.split(";") if part.strip()]
    if len(parts) != 2:
        return None
    p0 = parse_point(parts[0])
    p1 = parse_point(parts[1])
    return np.asarray([p0, p1], dtype=float)


def parse_scan_points(raw: str) -> np.ndarray:
    points: List[Tuple[float, float]] = []
    for item in raw.split(";"):
        item = item.strip()
        if not item:
            continue
        points.append(parse_point(item))
    return np.asarray(points, dtype=float) if points else np.zeros((0, 2), dtype=float)


def parse_debug_csv(csv_path: Path) -> Tuple[Dict[str, str], List[Dict[str, str]]]:
    header: Dict[str, str] = {}
    rows: List[Dict[str, str]] = []
    columns: List[str] = []

    with csv_path.open("r", encoding="utf-8") as handle:
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
            if parts == columns or len(parts) != len(columns):
                continue
            rows.append({key: value for key, value in zip(columns, parts)})

    if not rows:
        raise ValueError(f"No rows parsed from {csv_path}")
    return header, rows


def parse_map_metadata(scenario_path: Path) -> Tuple[Path, float, Tuple[float, float]]:
    map_yaml_rel = None
    in_map_section = False
    with scenario_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                in_map_section = line == "[map]"
                continue
            if in_map_section and line.startswith("yaml_path"):
                map_yaml_rel = line.split("=", 1)[1].strip().strip('"')
                break

    if map_yaml_rel is None:
        raise ValueError(f"Failed to parse map yaml path from scenario: {scenario_path}")

    map_yaml = (scenario_path.parent / map_yaml_rel).resolve()
    image_rel = None
    resolution = None
    origin_xy = None
    with map_yaml.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line.startswith("image:"):
                image_rel = line.split(":", 1)[1].strip()
            elif line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                raw = line.split(":", 1)[1].strip().removeprefix("[").removesuffix("]")
                vals = [float(value.strip()) for value in raw.split(",")]
                origin_xy = (vals[0], vals[1])

    if image_rel is None or resolution is None or origin_xy is None:
        raise ValueError(f"Failed to parse map yaml: {map_yaml}")

    return (map_yaml.parent / image_rel).resolve(), resolution, origin_xy


def draw_map(ax, map_image_path: Path, resolution: float, origin_xy: Tuple[float, float]) -> None:
    image = plt.imread(map_image_path)
    if image.ndim == 3:
        image = image[..., 0]
    h, w = image.shape[:2]
    x0, y0 = origin_xy
    extent = [x0, x0 + w * resolution, y0, y0 + h * resolution]
    ax.imshow(image, cmap="gray", origin="upper", extent=extent)


def main() -> int:
    args = parse_args()
    if not args.csv.exists():
        print(f"csv not found: {args.csv}", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)

    header, rows = parse_debug_csv(args.csv)
    scenario_rel = header.get("scenario_config", "test/localization/configs/localization.toml")
    scenario_path = Path(scenario_rel)
    if not scenario_path.is_absolute():
        scenario_path = (Path.cwd() / scenario_path).resolve()
    map_image_path, map_resolution, map_origin = parse_map_metadata(scenario_path)

    scan_world = parse_scan_points(header.get("scan_world", ""))

    map_segments: List[np.ndarray] = []
    hough_segments: List[np.ndarray] = []
    ransac_segments: List[np.ndarray] = []
    hough_gate: List[int] = []
    ransac_gate: List[int] = []
    hough_valid: List[int] = []
    ransac_valid: List[int] = []

    for row in rows:
        map_seg = parse_segment(row.get("map_seg", ""))
        if map_seg is not None:
            map_segments.append(map_seg)
        h_seg = parse_segment(row.get("hough_seg", ""))
        if h_seg is not None:
            hough_segments.append(h_seg)
        r_seg = parse_segment(row.get("ransac_seg", ""))
        if r_seg is not None:
            ransac_segments.append(r_seg)

        hough_gate.append(int(float(row.get("hough_gate", "0"))))
        ransac_gate.append(int(float(row.get("ransac_gate", "0"))))
        hough_valid.append(int(float(row.get("hough_valid", "0"))))
        ransac_valid.append(int(float(row.get("ransac_valid", "0"))))

    hough_gate_arr = np.asarray(hough_gate, dtype=int)
    ransac_gate_arr = np.asarray(ransac_gate, dtype=int)
    hough_valid_arr = np.asarray(hough_valid, dtype=int)
    ransac_valid_arr = np.asarray(ransac_valid, dtype=int)

    fig, ax = plt.subplots(figsize=(9, 9), dpi=120)
    draw_map(ax, map_image_path, map_resolution, map_origin)
    for seg in map_segments:
        ax.plot(seg[:, 0], seg[:, 1], color="gray", lw=1.0, alpha=0.6)
    if scan_world.size > 0:
        ax.scatter(scan_world[:, 0], scan_world[:, 1], s=8, c="green", alpha=0.30, label="scan points")
    for seg in hough_segments:
        ax.plot(seg[:, 0], seg[:, 1], color="blue", lw=1.4, alpha=0.9)
    for seg in ransac_segments:
        ax.plot(seg[:, 0], seg[:, 1], color="orange", lw=1.4, alpha=0.9)
    ax.set_title("Feature Extraction Overlay (Map/Hough/RANSAC)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    extraction_path = args.out_dir / "observation_feature_extraction_overlay.png"
    fig.savefig(extraction_path)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(15, 7), dpi=120)
    for ax, title, gate_arr in [
        (axes[0], "Hough Matching", hough_gate_arr),
        (axes[1], "Hough+RANSAC Matching", ransac_gate_arr),
    ]:
        draw_map(ax, map_image_path, map_resolution, map_origin)
        for idx, seg in enumerate(map_segments):
            passed = gate_arr[idx] == 1 if idx < gate_arr.size else False
            color = "blue" if passed else "red"
            alpha = 0.9 if passed else 0.45
            ax.plot(seg[:, 0], seg[:, 1], color=color, lw=1.8, alpha=alpha)
        if scan_world.size > 0:
            ax.scatter(scan_world[:, 0], scan_world[:, 1], s=5, c="green", alpha=0.20)
        ax.set_title(title)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fig.tight_layout()
    compare_path = args.out_dir / "observation_matching_hough_vs_ransac.png"
    fig.savefig(compare_path)
    plt.close(fig)

    metrics = {
        "line_count": int(len(rows)),
        "hough": {
            "valid_count": int(np.sum(hough_valid_arr)),
            "gate_pass_count": int(np.sum(hough_gate_arr)),
            "gate_pass_rate": float(np.mean(hough_gate_arr)) if hough_gate_arr.size > 0 else 0.0,
        },
        "ransac": {
            "valid_count": int(np.sum(ransac_valid_arr)),
            "gate_pass_count": int(np.sum(ransac_gate_arr)),
            "gate_pass_rate": float(np.mean(ransac_gate_arr)) if ransac_gate_arr.size > 0 else 0.0,
        },
    }
    metrics_path = args.out_dir / "observation_model_debug_metrics.json"
    with metrics_path.open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)

    print(f"Saved extraction overlay: {extraction_path}")
    print(f"Saved matching comparison: {compare_path}")
    print(f"Saved debug metrics: {metrics_path}")
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
