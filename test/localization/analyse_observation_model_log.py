#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize observation model evaluation metrics JSON"
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        default=Path(
            "test/localization/logs/analysis/observation_model_test/observation_model_eval_metrics.json"
        ),
        help="Path to observation_model_eval_metrics.json",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/observation_model_test"),
        help="Directory to save figures and summary",
    )
    return parser.parse_args()


def load_metrics(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_model(metrics: Dict) -> Tuple[int, Dict]:
    frame_count = int(metrics.get("frame_count", 0))
    model = metrics.get("model")
    if not isinstance(model, dict):
        raise ValueError("metrics['model'] is missing or invalid")
    return frame_count, model


def save_metric_bars(frame_count: int, model: Dict, out_path: Path) -> None:
    keys = [
        ("success_rate", "success rate", False, "ratio"),
        ("mean_score", "mean score", False, "score"),
        ("mean_residual_rmse", "mean residual rmse", True, "rmse"),
        ("mean_nis", "mean nis", True, "nis"),
        ("mean_measurement_count", "mean measurement count", False, "count"),
        ("mean_runtime_ms", "mean runtime [ms]", True, "ms"),
        ("p95_runtime_ms", "p95 runtime [ms]", True, "ms"),
    ]

    fig, axes = plt.subplots(3, 3, figsize=(15, 10), dpi=120)
    axes = axes.ravel()

    for i, (key, title, lower_better, unit) in enumerate(keys):
        value = float(model.get(key, 0.0))
        bars = axes[i].bar([model.get("label", "model")], [value], color=["tab:blue"])
        axes[i].set_title(title)
        axes[i].grid(axis="y", alpha=0.25)
        for bar in bars:
            axes[i].text(
                bar.get_x() + bar.get_width() / 2.0,
                value,
                f"{value:.4f}",
                ha="center",
                va="bottom",
                fontsize=9,
            )
        if lower_better:
            axes[i].set_ylabel(f"lower is better [{unit}]")
        else:
            axes[i].set_ylabel(f"higher is better [{unit}]")

    counts = [
        int(model.get("success_count", 0)),
        int(model.get("no_update_count", 0)),
        int(model.get("error_count", 0)),
    ]
    axes[7].bar(["success", "no_update", "error"], counts, color=["tab:green", "tab:orange", "tab:red"])
    axes[7].set_title("frame outcome counts")
    axes[7].grid(axis="y", alpha=0.25)

    axes[8].axis("off")
    info_lines = [
        f"label: {model.get('label', 'unknown')}",
        f"config_path: {model.get('config_path', 'unknown')}",
        f"frame_count: {frame_count}",
        f"total_frames: {int(model.get('total_frames', 0))}",
    ]
    axes[8].text(0.0, 0.95, "\n".join(info_lines), va="top", ha="left", fontsize=10)

    fig.suptitle("Observation Model Evaluation: Single Model Metrics")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_summary(frame_count: int, model: Dict, out_path: Path) -> None:

    lines: List[str] = []
    lines.append("# Observation Model Evaluation Summary")
    lines.append("")
    lines.append(f"- frame_count: {frame_count}")
    lines.append(f"- label: {model.get('label', 'unknown')}")
    lines.append(f"- config_path: {model.get('config_path', 'unknown')}")
    lines.append(f"- total_frames: {int(model.get('total_frames', 0))}")
    lines.append(f"- success_count: {int(model.get('success_count', 0))}")
    lines.append(f"- no_update_count: {int(model.get('no_update_count', 0))}")
    lines.append(f"- error_count: {int(model.get('error_count', 0))}")
    lines.append(f"- success_rate: {float(model.get('success_rate', 0.0)):.6f}")
    lines.append(f"- mean_score: {float(model.get('mean_score', 0.0)):.6f}")
    lines.append(
        f"- mean_residual_rmse: {float(model.get('mean_residual_rmse', 0.0)):.6f}"
    )
    lines.append(f"- mean_nis: {float(model.get('mean_nis', 0.0)):.6f}")
    lines.append(
        f"- mean_measurement_count: {float(model.get('mean_measurement_count', 0.0)):.6f}"
    )
    lines.append(f"- mean_runtime_ms: {float(model.get('mean_runtime_ms', 0.0)):.6f}")
    lines.append(f"- p95_runtime_ms: {float(model.get('p95_runtime_ms', 0.0)):.6f}")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    metrics = load_metrics(args.metrics)
    frame_count, model = parse_model(metrics)

    save_metric_bars(frame_count, model, args.out_dir / "model_metrics.png")
    save_summary(frame_count, model, args.out_dir / "summary.md")

    print("Observation model evaluation plots generated.")
    print(f"  metrics: {args.metrics}")
    print(f"  out_dir: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
