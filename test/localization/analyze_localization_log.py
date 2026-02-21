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
    parser = argparse.ArgumentParser(
        description="Analyze localization test log and generate map/error metrics"
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("test/localization/logs/localization_test.log"),
        help="Path to localization test log",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis"),
        help="Directory to save plots and metrics",
    )
    return parser.parse_args()


def parse_header_and_rows(log_path: Path) -> Tuple[Dict[str, str], List[str], List[List[str]]]:
    header: Dict[str, str] = {}
    columns: List[str] = []
    data_rows: List[List[str]] = []

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
            if len(parts) != len(columns):
                continue
            if parts == columns:
                continue
            data_rows.append(parts)

    if not columns:
        raise ValueError("CSV columns not found in log")
    if not data_rows:
        raise ValueError("No data rows found in log")

    return header, columns, data_rows


def column_as_float(rows: List[List[str]], index: int) -> np.ndarray:
    values = [float(row[index]) for row in rows]
    return np.asarray(values, dtype=float)


def normalize_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


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
        raise ValueError(f"Failed to parse map.yaml path from scenario: {scenario_path}")

    map_yaml_path = (scenario_path.parent / map_yaml_rel).resolve()
    image_rel = None
    resolution = None
    origin_xy = None

    with map_yaml_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line.startswith("image:"):
                image_rel = line.split(":", 1)[1].strip()
            elif line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                raw = line.split(":", 1)[1].strip().removeprefix("[").removesuffix("]")
                values = [float(value.strip()) for value in raw.split(",")]
                origin_xy = (values[0], values[1])

    if image_rel is None or resolution is None or origin_xy is None:
        raise ValueError(f"Failed to parse map yaml: {map_yaml_path}")

    map_image_path = (map_yaml_path.parent / image_rel).resolve()
    return map_image_path, resolution, origin_xy


def save_map_plot(
    out_path: Path,
    map_image_path: Path,
    map_resolution: float,
    map_origin: Tuple[float, float],
    true_xy: np.ndarray,
    est_xy: np.ndarray,
) -> None:
    image = plt.imread(map_image_path)
    if image.ndim == 3:
        image = image[..., 0]

    height, width = image.shape[:2]
    x0, y0 = map_origin
    extent = [x0, x0 + width * map_resolution, y0, y0 + height * map_resolution]

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.imshow(image, cmap="gray", origin="upper", extent=extent)
    ax.plot(true_xy[:, 0], true_xy[:, 1], "b-", lw=1.5, label="true trajectory")
    ax.plot(est_xy[:, 0], est_xy[:, 1], "r-", lw=1.5, label="estimated trajectory")
    ax.set_title("True vs Estimated Trajectory on Map")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="best")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_update_colored_map_plot(
    out_path: Path,
    map_image_path: Path,
    map_resolution: float,
    map_origin: Tuple[float, float],
    true_xy: np.ndarray,
    est_xy: np.ndarray,
    update_true_xy: np.ndarray,
    update_improved_mask: np.ndarray,
) -> None:
    image = plt.imread(map_image_path)
    if image.ndim == 3:
        image = image[..., 0]

    height, width = image.shape[:2]
    x0, y0 = map_origin
    extent = [x0, x0 + width * map_resolution, y0, y0 + height * map_resolution]

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.imshow(image, cmap="gray", origin="upper", extent=extent)
    ax.plot(true_xy[:, 0], true_xy[:, 1], "b-", lw=1.2, label="true trajectory")
    ax.plot(est_xy[:, 0], est_xy[:, 1], "r-", lw=1.0, alpha=0.9, label="estimated trajectory")

    improved_points = update_true_xy[update_improved_mask]
    worsened_points = update_true_xy[~update_improved_mask]
    if improved_points.size > 0:
        ax.scatter(
            improved_points[:, 0],
            improved_points[:, 1],
            c="blue",
            s=16,
            alpha=0.9,
            label="update improved",
        )
    if worsened_points.size > 0:
        ax.scatter(
            worsened_points[:, 0],
            worsened_points[:, 1],
            c="red",
            s=16,
            alpha=0.9,
            label="update worsened",
        )

    ax.set_title("Update Effect on Map (Improved vs Worsened)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="best")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_timeseries_plot(
    out_path: Path,
    time_s: np.ndarray,
    values: np.ndarray,
    title: str,
    y_label: str,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 4), dpi=120)
    ax.plot(time_s, values, lw=1.2)
    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(y_label)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_update_error_triplet_plot(
    out_path: Path,
    update_time_s: np.ndarray,
    e_before: np.ndarray,
    e_after: np.ndarray,
    delta_e: np.ndarray,
    title: str,
    y_label: str,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), dpi=120, sharex=True)

    axes[0].plot(update_time_s, e_before, lw=1.2, color="tab:orange")
    axes[0].set_ylabel(f"before {y_label}")
    axes[0].grid(alpha=0.25)

    axes[1].plot(update_time_s, e_after, lw=1.2, color="tab:green")
    axes[1].set_ylabel(f"after {y_label}")
    axes[1].grid(alpha=0.25)

    axes[2].plot(update_time_s, delta_e, lw=1.2, color="tab:blue")
    axes[2].axhline(0.0, color="black", linestyle="--", linewidth=1.0, alpha=0.7)
    axes[2].set_ylabel(f"delta {y_label}")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(alpha=0.25)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def ratio(mask: np.ndarray) -> float:
    if mask.size == 0:
        return 0.0
    return float(np.mean(mask.astype(float)))


def safe_mean(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.mean(values))


def bucket_stats(dist_goal: np.ndarray, delta_pos: np.ndarray, delta_heading: np.ndarray) -> Dict[str, float]:
    return {
        "count": int(dist_goal.size),
        "mean_delta_pos_err": safe_mean(delta_pos),
        "mean_delta_heading_err": safe_mean(delta_heading),
        "position_improvement_rate": ratio(delta_pos > 0.0),
        "heading_improvement_rate": ratio(delta_heading > 0.0),
        "position_degradation_rate": ratio(delta_pos < 0.0),
        "heading_degradation_rate": ratio(delta_heading < 0.0),
        "mean_dist_goal": safe_mean(dist_goal),
    }


def build_relative_poses(x: np.ndarray, y: np.ndarray, theta: np.ndarray) -> np.ndarray:
    rel = np.zeros((len(x) - 1, 3), dtype=float)
    for i in range(1, len(x)):
        dx_w = x[i] - x[i - 1]
        dy_w = y[i] - y[i - 1]
        c = math.cos(theta[i - 1])
        s = math.sin(theta[i - 1])
        rel[i - 1, 0] = c * dx_w + s * dy_w
        rel[i - 1, 1] = -s * dx_w + c * dy_w
        rel[i - 1, 2] = ((theta[i] - theta[i - 1] + math.pi) % (2.0 * math.pi)) - math.pi
    return rel


def compute_rpe(
    true_x: np.ndarray,
    true_y: np.ndarray,
    true_theta: np.ndarray,
    est_x: np.ndarray,
    est_y: np.ndarray,
    est_theta: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    if len(true_x) < 2:
        return np.zeros(0, dtype=float), np.zeros(0, dtype=float)

    true_rel = build_relative_poses(true_x, true_y, true_theta)
    est_rel = build_relative_poses(est_x, est_y, est_theta)
    rel_error = est_rel - true_rel
    rel_error[:, 2] = normalize_angle(rel_error[:, 2])

    trans_error = np.hypot(rel_error[:, 0], rel_error[:, 1])
    rot_error = np.abs(rel_error[:, 2])
    return trans_error, rot_error


def main() -> int:
    args = parse_args()
    if not args.log.exists():
        print(f"log file not found: {args.log}", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)

    header, columns, rows = parse_header_and_rows(args.log)
    col_idx = {name: idx for idx, name in enumerate(columns)}

    required_cols = [
        "time",
        "dist_goal",
        "true_x",
        "true_y",
        "true_theta",
        "est_x",
        "est_y",
        "est_theta",
        "pos_err",
        "heading_err",
        "lidar_updated",
        "pre_update_pos_err",
        "post_update_pos_err",
        "delta_pos_err",
        "pre_update_heading_err",
        "post_update_heading_err",
        "delta_heading_err",
    ]
    missing = [name for name in required_cols if name not in col_idx]
    if missing:
        print(f"missing required columns: {missing}", file=sys.stderr)
        return 1

    scenario_cfg_rel = header.get("scenario_config", "test/localization/configs/localization.toml")
    scenario_cfg_path = Path(scenario_cfg_rel)
    if not scenario_cfg_path.is_absolute():
        scenario_cfg_path = (Path.cwd() / scenario_cfg_path).resolve()

    map_image_path, map_resolution, map_origin = parse_map_metadata(scenario_cfg_path)

    time_s = column_as_float(rows, col_idx["time"])
    dist_goal = column_as_float(rows, col_idx["dist_goal"])
    true_x = column_as_float(rows, col_idx["true_x"])
    true_y = column_as_float(rows, col_idx["true_y"])
    true_theta = column_as_float(rows, col_idx["true_theta"])
    est_x = column_as_float(rows, col_idx["est_x"])
    est_y = column_as_float(rows, col_idx["est_y"])
    est_theta = column_as_float(rows, col_idx["est_theta"])
    pos_err = column_as_float(rows, col_idx["pos_err"])
    heading_err = column_as_float(rows, col_idx["heading_err"])
    lidar_updated = column_as_float(rows, col_idx["lidar_updated"])

    pre_update_pos_err = column_as_float(rows, col_idx["pre_update_pos_err"])
    post_update_pos_err = column_as_float(rows, col_idx["post_update_pos_err"])
    delta_pos_err = column_as_float(rows, col_idx["delta_pos_err"])
    pre_update_heading_err = column_as_float(rows, col_idx["pre_update_heading_err"])
    post_update_heading_err = column_as_float(rows, col_idx["post_update_heading_err"])
    delta_heading_err = column_as_float(rows, col_idx["delta_heading_err"])

    true_xy = np.stack([true_x, true_y], axis=1)
    est_xy = np.stack([est_x, est_y], axis=1)

    save_map_plot(
        args.out_dir / "map_true_vs_estimated.png",
        map_image_path,
        map_resolution,
        map_origin,
        true_xy,
        est_xy,
    )
    save_timeseries_plot(
        args.out_dir / "position_error_timeseries.png",
        time_s,
        pos_err,
        "Position Error Time Series",
        "position error [m]",
    )
    save_timeseries_plot(
        args.out_dir / "heading_error_timeseries.png",
        time_s,
        heading_err,
        "Heading Error Time Series",
        "heading error [rad]",
    )

    update_mask = (
        (lidar_updated > 0.5)
        & np.isfinite(pre_update_pos_err)
        & np.isfinite(post_update_pos_err)
        & np.isfinite(delta_pos_err)
        & np.isfinite(pre_update_heading_err)
        & np.isfinite(post_update_heading_err)
        & np.isfinite(delta_heading_err)
    )

    if not np.any(update_mask):
        print("no lidar update rows with pre/post/delta errors found", file=sys.stderr)
        return 1

    update_time_s = time_s[update_mask]
    update_true_xy = true_xy[update_mask]
    update_dist_goal = dist_goal[update_mask]

    update_pre_pos = pre_update_pos_err[update_mask]
    update_post_pos = post_update_pos_err[update_mask]
    update_delta_pos = delta_pos_err[update_mask]

    update_pre_heading = pre_update_heading_err[update_mask]
    update_post_heading = post_update_heading_err[update_mask]
    update_delta_heading = delta_heading_err[update_mask]

    save_update_error_triplet_plot(
        args.out_dir / "update_position_error_triplet.png",
        update_time_s,
        update_pre_pos,
        update_post_pos,
        update_delta_pos,
        "Observation Update Effect (Position Error)",
        "[m]",
    )
    save_update_error_triplet_plot(
        args.out_dir / "update_heading_error_triplet.png",
        update_time_s,
        update_pre_heading,
        update_post_heading,
        update_delta_heading,
        "Observation Update Effect (Heading Error)",
        "[rad]",
    )
    save_update_colored_map_plot(
        args.out_dir / "map_update_improvement_colored.png",
        map_image_path,
        map_resolution,
        map_origin,
        true_xy,
        est_xy,
        update_true_xy,
        update_delta_pos > 0.0,
    )

    ate = float(np.mean(pos_err))
    rmse = float(np.sqrt(np.mean(np.square(pos_err))))

    rpe_trans, rpe_rot = compute_rpe(true_x, true_y, true_theta, est_x, est_y, est_theta)
    rpe = float(np.sqrt(np.mean(np.square(rpe_trans)))) if rpe_trans.size > 0 else 0.0
    rpe_rot_rmse = float(np.sqrt(np.mean(np.square(rpe_rot)))) if rpe_rot.size > 0 else 0.0

    dist_median = float(np.median(update_dist_goal))
    near_mask = update_dist_goal <= dist_median
    far_mask = update_dist_goal > dist_median

    near_stats = bucket_stats(
        update_dist_goal[near_mask],
        update_delta_pos[near_mask],
        update_delta_heading[near_mask],
    )
    far_stats = bucket_stats(
        update_dist_goal[far_mask],
        update_delta_pos[far_mask],
        update_delta_heading[far_mask],
    )

    metrics = {
        "samples": int(len(pos_err)),
        "duration_seconds": float(time_s[-1] if len(time_s) > 0 else 0.0),
        "final_position_error": float(pos_err[-1]),
        "final_heading_error": float(heading_err[-1]),
        "ate": ate,
        "rpe": rpe,
        "rmse": rmse,
        "rpe_rot_rmse": rpe_rot_rmse,
        "max_position_error": float(np.max(pos_err)),
        "max_heading_error": float(np.max(heading_err)),
        "observation_update": {
            "count": int(update_delta_pos.size),
            "mean_delta_pos_err": safe_mean(update_delta_pos),
            "mean_delta_heading_err": safe_mean(update_delta_heading),
            "position_improvement_rate": ratio(update_delta_pos > 0.0),
            "heading_improvement_rate": ratio(update_delta_heading > 0.0),
            "position_degradation_rate": ratio(update_delta_pos < 0.0),
            "heading_degradation_rate": ratio(update_delta_heading < 0.0),
            "position_effective_rate": ratio(update_delta_pos >= 0.0),
            "heading_effective_rate": ratio(update_delta_heading >= 0.0),
            "distance_bucket_threshold": dist_median,
            "near": near_stats,
            "far": far_stats,
        },
        "result": header.get("result", "unknown"),
    }

    metrics_path = args.out_dir / "metrics.json"
    with metrics_path.open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)

    print(f"Saved map plot: {args.out_dir / 'map_true_vs_estimated.png'}")
    print(f"Saved position error plot: {args.out_dir / 'position_error_timeseries.png'}")
    print(f"Saved heading error plot: {args.out_dir / 'heading_error_timeseries.png'}")
    print(f"Saved update position triplet plot: {args.out_dir / 'update_position_error_triplet.png'}")
    print(f"Saved update heading triplet plot: {args.out_dir / 'update_heading_error_triplet.png'}")
    print(f"Saved update colored map plot: {args.out_dir / 'map_update_improvement_colored.png'}")
    print(f"Saved metrics: {metrics_path}")
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
