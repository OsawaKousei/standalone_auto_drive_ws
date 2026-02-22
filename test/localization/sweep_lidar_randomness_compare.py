#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class ExperimentTarget:
    name: str
    localization_config_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Narrow LiDAR FOV to realistic range, increase LiDAR randomness level, "
            "and compare localization algorithms with ATE trend plots."
        )
    )
    parser.add_argument(
        "--base-scenario",
        type=Path,
        default=Path("test/localization/configs/localization.toml"),
        help="Base scenario TOML path",
    )
    parser.add_argument(
        "--base-lidar-config",
        type=Path,
        default=Path("configs/defaults/sensor/lidar.toml"),
        help="Base lidar TOML path",
    )
    parser.add_argument(
        "--localization-app",
        type=Path,
        default=Path("build/localization_test_app"),
        help="Path to localization_test_app",
    )
    parser.add_argument(
        "--analyzer-script",
        type=Path,
        default=Path("test/localization/analyze_localization_log.py"),
        help="Path to analyze_localization_log.py",
    )
    parser.add_argument(
        "--noise-levels",
        type=str,
        default="0.01,0.02,0.04,0.06,0.08,0.10",
        help="Comma-separated range_noise_stddev levels",
    )
    parser.add_argument(
        "--fov-deg",
        type=float,
        default=270.0,
        help="LiDAR field-of-view in degrees (centered at 0)",
    )
    parser.add_argument(
        "--eval-runs",
        type=int,
        default=1,
        help="Number of repeated runs per setting (aggregated by median)",
    )
    parser.add_argument(
        "--max-attempts-per-run",
        type=int,
        default=3,
        help="Maximum retry attempts for each run when app/analyze fails",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Run app with --render (default: off)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/lidar_randomness_sweep"),
        help="Output directory for sweep results",
    )
    return parser.parse_args()


def to_abs(path: Path, workspace_root: Path) -> Path:
    return path if path.is_absolute() else (workspace_root / path).resolve()


def run_command(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)


def parse_noise_levels(raw: str) -> list[float]:
    levels: list[float] = []
    for token in raw.split(","):
        value = float(token.strip())
        if value < 0.0:
            raise ValueError("noise levels must be non-negative")
        levels.append(value)
    if not levels:
        raise ValueError("noise levels are empty")
    return levels


def parse_key_values(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def build_lidar_config(base_lidar_text: str, min_angle: float, max_angle: float, noise_stddev: float) -> str:
    params = parse_key_values(base_lidar_text)
    params["min_angle"] = f"{min_angle:.12g}"
    params["max_angle"] = f"{max_angle:.12g}"
    params["range_noise_stddev"] = f"{noise_stddev:.12g}"

    order = ["ray_count", "min_angle", "max_angle", "max_range", "range_step", "range_noise_stddev"]
    lines: list[str] = []
    for key in order:
        if key in params:
            lines.append(f"{key} = {params[key]}")

    for key, value in params.items():
        if key not in order:
            lines.append(f"{key} = {value}")

    return "\n".join(lines) + "\n"


def build_scenario_text(base_scenario_text: str, localization_cfg_path: str, lidar_cfg_path: str) -> str:
    lines = base_scenario_text.splitlines()
    current_section = ""

    for idx, raw_line in enumerate(lines):
        line = raw_line.strip()
        if line.startswith("[") and line.endswith("]"):
            current_section = line.strip("[]")
            continue

        if "=" not in line:
            continue

        key = line.split("=", 1)[0].strip()
        if key != "config_path":
            continue

        if current_section == "localization":
            prefix = raw_line.split("config_path", 1)[0]
            lines[idx] = f'{prefix}config_path = "{localization_cfg_path}"'
        elif current_section == "lidar_sensor":
            prefix = raw_line.split("config_path", 1)[0]
            lines[idx] = f'{prefix}config_path = "{lidar_cfg_path}"'

    return "\n".join(lines) + "\n"


def load_metrics(metrics_path: Path) -> dict[str, Any]:
    return json.loads(metrics_path.read_text(encoding="utf-8"))


def aggregate_rows(rows: list[dict[str, Any]], noise_levels: list[float], targets: list[ExperimentTarget]) -> list[dict[str, Any]]:
    summary: list[dict[str, Any]] = []
    for target in targets:
        for noise in noise_levels:
            subset = [r for r in rows if r["algorithm"] == target.name and abs(r["noise_stddev"] - noise) < 1e-12]
            if not subset:
                continue
            ate_values = [float(r["ate"]) for r in subset if r["result"] == "success"]
            rmse_values = [float(r["rmse"]) for r in subset if r["result"] == "success"]
            success_rate = sum(1 for r in subset if r["result"] == "success") / len(subset)

            summary.append(
                {
                    "algorithm": target.name,
                    "noise_stddev": noise,
                    "runs": len(subset),
                    "success_rate": success_rate,
                    "ate_median": statistics.median(ate_values) if ate_values else None,
                    "ate_mean": statistics.fmean(ate_values) if ate_values else None,
                    "rmse_median": statistics.median(rmse_values) if rmse_values else None,
                }
            )
    return summary


def plot_ate_trend(summary: list[dict[str, Any]], out_path: Path, title: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 5), dpi=120)

    algorithms = sorted({row["algorithm"] for row in summary})
    for algorithm in algorithms:
        subset = [row for row in summary if row["algorithm"] == algorithm]
        subset.sort(key=lambda x: x["noise_stddev"])

        x = np.asarray([row["noise_stddev"] for row in subset], dtype=float)
        y = np.asarray([
            float("nan") if row["ate_median"] is None else float(row["ate_median"])
            for row in subset
        ], dtype=float)
        ax.plot(x, y, marker="o", label=f"{algorithm} (median ATE)")

    ax.set_title(title)
    ax.set_xlabel("LiDAR range_noise_stddev")
    ax.set_ylabel("ATE (lower is better)")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_success_trend(summary: list[dict[str, Any]], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5), dpi=120)

    algorithms = sorted({row["algorithm"] for row in summary})
    for algorithm in algorithms:
        subset = [row for row in summary if row["algorithm"] == algorithm]
        subset.sort(key=lambda x: x["noise_stddev"])
        x = np.asarray([row["noise_stddev"] for row in subset], dtype=float)
        y = np.asarray([float(row["success_rate"]) for row in subset], dtype=float)
        ax.plot(x, y, marker="o", label=f"{algorithm} success rate")

    ax.set_title("Success Rate vs LiDAR Randomness")
    ax.set_xlabel("LiDAR range_noise_stddev")
    ax.set_ylabel("Success rate")
    ax.set_ylim(0.0, 1.05)
    ax.grid(alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    workspace_root = Path(__file__).resolve().parents[2]

    base_scenario_path = to_abs(args.base_scenario, workspace_root)
    base_lidar_path = to_abs(args.base_lidar_config, workspace_root)
    localization_app = to_abs(args.localization_app, workspace_root)
    analyzer_script = to_abs(args.analyzer_script, workspace_root)
    out_dir = to_abs(args.out_dir, workspace_root)

    if args.eval_runs <= 0:
        print("--eval-runs must be >= 1", file=sys.stderr)
        return 1
    if args.max_attempts_per_run <= 0:
        print("--max-attempts-per-run must be >= 1", file=sys.stderr)
        return 1

    if not base_scenario_path.exists() or not base_lidar_path.exists():
        print("base scenario or lidar config not found", file=sys.stderr)
        return 1
    if not localization_app.exists() or not analyzer_script.exists():
        print("localization app or analyzer script not found", file=sys.stderr)
        return 1

    noise_levels = parse_noise_levels(args.noise_levels)

    out_dir.mkdir(parents=True, exist_ok=True)
    temp_dir = out_dir / "tmp_configs"
    temp_dir.mkdir(parents=True, exist_ok=True)
    scenario_temp_dir = base_scenario_path.parent

    base_scenario_text = base_scenario_path.read_text(encoding="utf-8")
    base_lidar_text = base_lidar_path.read_text(encoding="utf-8")

    half_fov_rad = (args.fov_deg * np.pi / 180.0) / 2.0
    min_angle = -half_fov_rad
    max_angle = half_fov_rad

    targets = [
        ExperimentTarget(
            "hough_ransac",
            workspace_root / "test/localization/configs/localization/ekf_hough_ransac.toml",
        ),
        ExperimentTarget(
            "hough_only",
            workspace_root / "test/localization/configs/localization/ekf_hough.toml",
        ),
    ]

    rows: list[dict[str, Any]] = []

    print("LiDAR randomness sweep start")
    print(f"  fov_deg: {args.fov_deg}")
    print(f"  noise_levels: {noise_levels}")
    print(f"  eval_runs: {args.eval_runs}")
    print(f"  max_attempts_per_run: {args.max_attempts_per_run}")

    for noise in noise_levels:
        lidar_abs = temp_dir / f"lidar_noise_{noise:.4f}.toml"
        lidar_abs.write_text(build_lidar_config(base_lidar_text, min_angle, max_angle, noise), encoding="utf-8")

        for target in targets:
            scenario_abs = scenario_temp_dir / f"_tmp_scenario_{target.name}_noise_{noise:.4f}.toml"
            scenario_abs.write_text(
                build_scenario_text(
                    base_scenario_text,
                    localization_cfg_path=str(target.localization_config_path),
                    lidar_cfg_path=str(lidar_abs),
                ),
                encoding="utf-8",
            )

            for run_index in range(args.eval_runs):
                run_succeeded = False
                final_status = "app_failed"
                for attempt in range(args.max_attempts_per_run):
                    app_command = [str(localization_app), str(scenario_abs)]
                    app_command.append("--render" if args.render else "--no-render")
                    app_result = run_command(app_command, cwd=workspace_root)
                    if app_result.returncode != 0:
                        final_status = "app_failed"
                        continue

                    run_log = out_dir / f"run_{target.name}_noise_{noise:.4f}_idx{run_index}.log"
                    run_log.write_text(
                        (workspace_root / "test/localization/logs/localization_test.log").read_text(encoding="utf-8"),
                        encoding="utf-8",
                    )

                    analysis_dir = out_dir / f"analysis_{target.name}_noise_{noise:.4f}_idx{run_index}"
                    analysis_result = run_command(
                        [sys.executable, str(analyzer_script), "--log", str(run_log), "--out-dir", str(analysis_dir)],
                        cwd=workspace_root,
                    )
                    metrics_path = analysis_dir / "metrics.json"
                    if analysis_result.returncode != 0 or not metrics_path.exists():
                        final_status = "analyze_failed"
                        continue

                    metrics = load_metrics(metrics_path)
                    result = str(metrics.get("result", "unknown"))
                    observation = metrics.get("observation_update") or {}
                    rows.append(
                        {
                            "algorithm": target.name,
                            "noise_stddev": noise,
                            "run_index": run_index,
                            "result": result,
                            "ate": float(metrics.get("ate", "nan")),
                            "rmse": float(metrics.get("rmse", "nan")),
                            "rpe": float(metrics.get("rpe", "nan")),
                            "position_improvement_rate": float(observation.get("position_improvement_rate", "nan")),
                            "update_count": int(observation.get("count", 0)),
                        }
                    )
                    print(
                        f"  {target.name} noise={noise:.4f} run={run_index} attempt={attempt}: result={result} "
                        f"ate={float(metrics.get('ate', float('nan'))):.6f}"
                    )
                    run_succeeded = True
                    break

                if not run_succeeded:
                    rows.append(
                        {
                            "algorithm": target.name,
                            "noise_stddev": noise,
                            "run_index": run_index,
                            "result": final_status,
                            "ate": None,
                            "rmse": None,
                            "rpe": None,
                            "position_improvement_rate": None,
                            "update_count": None,
                        }
                    )
                    print(
                        f"  {target.name} noise={noise:.4f} run={run_index}: {final_status} "
                        f"after {args.max_attempts_per_run} attempts"
                    )

    csv_path = out_dir / "sweep_raw_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "algorithm",
                "noise_stddev",
                "run_index",
                "result",
                "ate",
                "rmse",
                "rpe",
                "position_improvement_rate",
                "update_count",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    summary_rows = aggregate_rows(rows, noise_levels, targets)
    summary_path = out_dir / "sweep_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "fov_deg": args.fov_deg,
                "fov_min_angle_rad": min_angle,
                "fov_max_angle_rad": max_angle,
                "noise_levels": noise_levels,
                "eval_runs": args.eval_runs,
                "summary": summary_rows,
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    ate_plot_path = out_dir / "ate_vs_lidar_randomness.png"
    plot_ate_trend(summary_rows, ate_plot_path, title="ATE Trend vs LiDAR Randomness (Narrow FOV)")

    success_plot_path = out_dir / "success_rate_vs_lidar_randomness.png"
    plot_success_trend(summary_rows, success_plot_path)

    print("Sweep complete")
    print(f"  raw_csv: {csv_path}")
    print(f"  summary_json: {summary_path}")
    print(f"  ate_plot: {ate_plot_path}")
    print(f"  success_plot: {success_plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
