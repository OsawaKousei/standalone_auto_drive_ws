#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run time-series A/B comparison: Hough vs Hough+RANSAC")
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path("build/localization_test_app"),
        help="Path to localization_test_app binary",
    )
    parser.add_argument(
        "--analyzer",
        type=Path,
        default=Path("test/localization/analyze_localization_log.py"),
        help="Path to localization log analyzer script",
    )
    parser.add_argument(
        "--hough-scenario",
        type=Path,
        default=Path("test/localization/configs/localization_hough.toml"),
        help="Scenario using hough_line observation model",
    )
    parser.add_argument(
        "--ransac-scenario",
        type=Path,
        default=Path("test/localization/configs/localization_hough_ransac.toml"),
        help="Scenario using hough_ransac_line observation model",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/ab_compare"),
        help="Directory for A/B logs and reports",
    )
    parser.add_argument(
        "--retry",
        type=int,
        default=3,
        help="Retry count when a simulation run fails",
    )
    return parser.parse_args()


def run_with_retry(command: list[str], retry_count: int) -> None:
    last_error = None
    for _ in range(retry_count):
        try:
            subprocess.run(command, check=True)
            return
        except subprocess.CalledProcessError as error:
            last_error = error
    raise RuntimeError(f"Command failed after {retry_count} retries: {' '.join(command)}") from last_error


def run_case(
    case_name: str,
    scenario: Path,
    binary: Path,
    analyzer: Path,
    out_dir: Path,
    retry_count: int,
) -> Dict[str, Any]:
    case_dir = out_dir / case_name
    case_dir.mkdir(parents=True, exist_ok=True)

    run_with_retry([str(binary), str(scenario)], retry_count)

    latest_log = Path("test/localization/logs/localization_test.log")
    case_log = case_dir / "localization_test.log"
    shutil.copy2(latest_log, case_log)

    subprocess.run(
        [
            sys.executable,
            str(analyzer),
            "--log",
            str(case_log),
            "--out-dir",
            str(case_dir / "analysis"),
        ],
        check=True,
    )

    metrics_path = case_dir / "analysis" / "metrics.json"
    return json.loads(metrics_path.read_text(encoding="utf-8"))


def nested_get(data: Dict[str, Any], key_path: str) -> float:
    current: Any = data
    for key in key_path.split("."):
        if not isinstance(current, dict) or key not in current:
            return 0.0
        current = current[key]
    return float(current)


def main() -> int:
    args = parse_args()

    if not args.binary.exists():
        print(f"binary not found: {args.binary}", file=sys.stderr)
        return 1
    if not args.analyzer.exists():
        print(f"analyzer not found: {args.analyzer}", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)

    metrics_hough = run_case(
        case_name="hough",
        scenario=args.hough_scenario,
        binary=args.binary,
        analyzer=args.analyzer,
        out_dir=args.out_dir,
        retry_count=args.retry,
    )

    metrics_ransac = run_case(
        case_name="hough_ransac",
        scenario=args.ransac_scenario,
        binary=args.binary,
        analyzer=args.analyzer,
        out_dir=args.out_dir,
        retry_count=args.retry,
    )

    compare_keys = [
        "ate",
        "rmse",
        "rpe",
        "observation_update.position_improvement_rate",
        "observation_update.position_degradation_rate",
        "observation_update.heading_improvement_rate",
        "observation_update.heading_degradation_rate",
        "observation_update.mean_delta_pos_err",
        "observation_update.mean_delta_heading_err",
    ]

    comparison: Dict[str, Any] = {
        "hough": metrics_hough,
        "hough_ransac": metrics_ransac,
        "delta": {},
    }

    for key in compare_keys:
        hough_val = nested_get(metrics_hough, key)
        ransac_val = nested_get(metrics_ransac, key)
        comparison["delta"][key] = {
            "hough": hough_val,
            "hough_ransac": ransac_val,
            "hough_ransac_minus_hough": ransac_val - hough_val,
        }

    report_path = args.out_dir / "comparison_report.json"
    report_path.write_text(json.dumps(comparison, indent=2), encoding="utf-8")

    plot_keys = [
        "ate",
        "rmse",
        "rpe",
        "observation_update.position_degradation_rate",
        "observation_update.heading_degradation_rate",
        "observation_update.mean_delta_pos_err",
    ]
    hough_values = [nested_get(metrics_hough, key) for key in plot_keys]
    ransac_values = [nested_get(metrics_ransac, key) for key in plot_keys]

    x = range(len(plot_keys))
    width = 0.38
    fig, ax = plt.subplots(figsize=(12, 5), dpi=120)
    ax.bar([idx - width / 2 for idx in x], hough_values, width=width, label="hough")
    ax.bar([idx + width / 2 for idx in x], ransac_values, width=width, label="hough_ransac")
    ax.set_xticks(list(x))
    ax.set_xticklabels([key.replace("observation_update.", "") for key in plot_keys], rotation=25, ha="right")
    ax.set_title("Hough vs Hough+RANSAC (Time-Series A/B)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    plot_path = args.out_dir / "comparison_barplot.png"
    fig.savefig(plot_path)
    plt.close(fig)

    print(f"Saved comparison report: {report_path}")
    print(f"Saved comparison plot: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
