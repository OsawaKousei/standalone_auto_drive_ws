#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser(description="Analyze control test log and generate plots/metrics")
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("test/control/logs/control_test.log"),
        help="Path to control log file",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/control/logs/analysis"),
        help="Directory to save plots and metrics",
    )
    parser.add_argument(
        "--trajectory-source",
        choices=["true", "odom"],
        default="true",
        help="Which trajectory to use as followed path on map/curvature fit",
    )
    return parser.parse_args()


def parse_header_and_rows(log_path: Path) -> Tuple[Dict[str, str], List[str], np.ndarray]:
    header: Dict[str, str] = {}
    columns: List[str] = []
    data_rows: List[List[float]] = []

    with log_path.open("r", encoding="utf-8") as f:
        for raw_line in f:
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

            parts = [part.strip() for part in line.split(",")]
            if len(parts) != len(columns):
                continue
            try:
                data_rows.append([float(value) for value in parts])
            except ValueError:
                continue

    if not columns:
        raise ValueError("CSV columns not found in log")
    if not data_rows:
        raise ValueError("No data rows found in log")

    return header, columns, np.asarray(data_rows, dtype=float)


def parse_path_points(path_raw: str) -> np.ndarray:
    points: List[Tuple[float, float]] = []
    for item in path_raw.split(";"):
        item = item.strip()
        if not item:
            continue
        x_raw, y_raw = item.split(":", 1)
        points.append((float(x_raw), float(y_raw)))
    if not points:
        raise ValueError("Parsed path is empty")
    return np.asarray(points, dtype=float)


def parse_map_metadata(scenario_path: Path) -> Tuple[Path, float, Tuple[float, float]]:
    map_yaml_rel = None
    in_map_section = False
    with scenario_path.open("r", encoding="utf-8") as f:
        for raw_line in f:
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
        raise ValueError(f"Failed to parse map.yaml path from scenario: {scenario_path}")

    scenario_dir = scenario_path.parent
    map_yaml_path = (scenario_dir / map_yaml_rel).resolve()

    image_rel = None
    resolution = None
    origin_xy = None

    with map_yaml_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("image:"):
                image_rel = line.split(":", 1)[1].strip()
            elif line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                raw = line.split(":", 1)[1].strip()
                raw = raw.removeprefix("[").removesuffix("]")
                vals = [float(v.strip()) for v in raw.split(",")]
                origin_xy = (vals[0], vals[1])

    if image_rel is None or resolution is None or origin_xy is None:
        raise ValueError(f"Failed to parse map yaml: {map_yaml_path}")

    map_image_path = (map_yaml_path.parent / image_rel).resolve()
    return map_image_path, resolution, origin_xy


def compute_curvature(path_xy: np.ndarray) -> np.ndarray:
    n = len(path_xy)
    if n < 3:
        return np.zeros(n, dtype=float)

    curvature = np.zeros(n, dtype=float)
    for i in range(1, n - 1):
        p_prev = path_xy[i - 1]
        p = path_xy[i]
        p_next = path_xy[i + 1]

        a = np.linalg.norm(p - p_prev)
        b = np.linalg.norm(p_next - p)
        c = np.linalg.norm(p_next - p_prev)
        if a <= 1e-12 or b <= 1e-12 or c <= 1e-12:
            continue

        area2 = (
            (p[0] - p_prev[0]) * (p_next[1] - p_prev[1])
            - (p[1] - p_prev[1]) * (p_next[0] - p_prev[0])
        )
        curvature[i] = area2 / (a * b * c)

    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]
    return curvature


def resample_by_normalized_arclength(path_xy: np.ndarray, values: np.ndarray, n_samples: int) -> np.ndarray:
    if len(path_xy) != len(values):
        raise ValueError("path_xy and values length mismatch")
    if len(path_xy) == 1:
        return np.full(n_samples, values[0], dtype=float)

    seg = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    arclen = np.concatenate([[0.0], np.cumsum(seg)])
    total = arclen[-1]
    if total <= 1e-12:
        return np.full(n_samples, float(np.mean(values)), dtype=float)

    s_norm = arclen / total
    target = np.linspace(0.0, 1.0, n_samples)
    return np.interp(target, s_norm, values)


def count_oscillation(signal: np.ndarray, threshold: float = 1e-2) -> int:
    valid = np.where(np.abs(signal) >= threshold, signal, 0.0)
    signs = np.sign(valid)
    signs = signs[signs != 0.0]
    if len(signs) < 2:
        return 0
    return int(np.sum(signs[1:] * signs[:-1] < 0.0))


def save_map_plot(
    out_path: Path,
    map_image_path: Path,
    map_resolution: float,
    map_origin: Tuple[float, float],
    planned_path_xy: np.ndarray,
    followed_path_xy: np.ndarray,
) -> None:
    image = plt.imread(map_image_path)
    if image.ndim == 3:
        image = image[..., 0]

    height, width = image.shape[:2]
    x0, y0 = map_origin
    extent = [x0, x0 + width * map_resolution, y0, y0 + height * map_resolution]

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.imshow(image, cmap="gray", origin="lower", extent=extent)
    ax.plot(planned_path_xy[:, 0], planned_path_xy[:, 1], "b-", lw=1.5, label="planned path")
    ax.plot(followed_path_xy[:, 0], followed_path_xy[:, 1], "r-", lw=1.5, label="followed path")
    ax.set_title("Planned vs Followed Path on Map")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="best")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_timeseries_plot(
    out_path: Path, time_s: np.ndarray, y: np.ndarray, title: str, y_label: str
) -> None:
    fig, ax = plt.subplots(figsize=(10, 4), dpi=120)
    ax.plot(time_s, y, lw=1.2)
    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(y_label)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    log_path = args.log
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if not log_path.exists():
        print(f"log file not found: {log_path}", file=sys.stderr)
        return 1

    header, columns, data = parse_header_and_rows(log_path)
    col_idx = {name: idx for idx, name in enumerate(columns)}

    required_cols = [
        "time",
        "cross_track",
        "track_heading_err",
        "cmd_v",
        "cmd_vy",
        "cmd_w",
        "true_x",
        "true_y",
        "odom_x",
        "odom_y",
        "track_err",
    ]
    missing = [c for c in required_cols if c not in col_idx]
    if missing:
        print(f"missing required columns: {missing}", file=sys.stderr)
        return 1

    if "path" not in header:
        print(
            "log header does not contain planned path ('# path=...'). "
            "Please regenerate log with the latest control_test_app.",
            file=sys.stderr,
        )
        return 1

    scenario_cfg_rel = header.get("scenario_config", "test/control/configs/control.toml")
    scenario_cfg_path = Path(scenario_cfg_rel)
    if not scenario_cfg_path.is_absolute():
        scenario_cfg_path = (Path.cwd() / scenario_cfg_path).resolve()

    planned_path_xy = parse_path_points(header["path"])

    time_s = data[:, col_idx["time"]]
    cross_track = data[:, col_idx["cross_track"]]
    heading_error = data[:, col_idx["track_heading_err"]]
    cmd_v = data[:, col_idx["cmd_v"]]
    cmd_vy = data[:, col_idx["cmd_vy"]]
    cmd_w = data[:, col_idx["cmd_w"]]
    track_err = data[:, col_idx["track_err"]]

    followed_xy = (
        np.stack([data[:, col_idx["true_x"]], data[:, col_idx["true_y"]]], axis=1)
        if args.trajectory_source == "true"
        else np.stack([data[:, col_idx["odom_x"]], data[:, col_idx["odom_y"]]], axis=1)
    )

    speed = np.hypot(cmd_v, cmd_vy)
    dt = np.diff(time_s)
    dt_safe = np.where(dt <= 1e-12, np.nan, dt)

    accel = np.diff(speed) / dt_safe
    angular_accel = np.diff(cmd_w) / dt_safe
    jerk = np.diff(accel) / dt_safe[1:] if len(accel) > 1 else np.asarray([], dtype=float)

    planned_curvature = compute_curvature(planned_path_xy)
    followed_curvature = compute_curvature(followed_xy)
    n_compare = 300
    planned_curv_rs = resample_by_normalized_arclength(planned_path_xy, planned_curvature, n_compare)
    followed_curv_rs = resample_by_normalized_arclength(followed_xy, followed_curvature, n_compare)
    curvature_rmse = float(np.sqrt(np.mean((planned_curv_rs - followed_curv_rs) ** 2)))
    curvature_fitness = float(math.exp(-curvature_rmse))

    metrics = {
        "rmse": float(np.sqrt(np.mean(track_err**2))),
        "max_deviation": float(np.max(cross_track)),
        "mean_speed": float(np.mean(speed)),
        "max_speed": float(np.max(speed)),
        "max_angular_speed": float(np.max(np.abs(cmd_w))),
        "max_acceleration": float(np.nanmax(np.abs(accel))) if len(accel) > 0 else 0.0,
        "max_angular_acceleration": float(np.nanmax(np.abs(angular_accel)))
        if len(angular_accel) > 0
        else 0.0,
        "max_jerk": float(np.nanmax(np.abs(jerk))) if len(jerk) > 0 else 0.0,
        "curvature_fitness": curvature_fitness,
        "curvature_rmse": curvature_rmse,
        "oscillation_count": count_oscillation(cmd_w),
    }

    map_image_path, map_resolution, map_origin = parse_map_metadata(scenario_cfg_path)
    save_map_plot(
        out_dir / "map_planned_vs_followed.png",
        map_image_path,
        map_resolution,
        map_origin,
        planned_path_xy,
        followed_xy,
    )

    save_timeseries_plot(
        out_dir / "cross_track_timeseries.png",
        time_s,
        cross_track,
        "Cross Track Error",
        "cross-track [m]",
    )
    save_timeseries_plot(
        out_dir / "heading_error_timeseries.png",
        time_s,
        heading_error,
        "Heading Error",
        "heading error [rad]",
    )

    accel_time = time_s[1:] if len(time_s) > 1 else np.asarray([], dtype=float)
    ang_accel_time = time_s[1:] if len(time_s) > 1 else np.asarray([], dtype=float)

    if len(accel) > 0:
        save_timeseries_plot(
            out_dir / "acceleration_timeseries.png",
            accel_time,
            accel,
            "Acceleration",
            "acceleration [m/s^2]",
        )
    if len(angular_accel) > 0:
        save_timeseries_plot(
            out_dir / "angular_acceleration_timeseries.png",
            ang_accel_time,
            angular_accel,
            "Angular Acceleration",
            "angular acceleration [rad/s^2]",
        )

    metrics_path = out_dir / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")

    print("Analysis complete")
    print(f"  log: {log_path}")
    print(f"  output_dir: {out_dir}")
    print(f"  metrics: {metrics_path}")
    for key, value in metrics.items():
        print(f"  {key}: {value}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
