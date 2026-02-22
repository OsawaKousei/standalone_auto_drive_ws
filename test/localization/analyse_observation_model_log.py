#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize observation model evaluation metrics JSON"
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        default=Path("test/localization/logs/observation_model_eval_metrics.json"),
        help="Path to observation_model_eval_metrics.json",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/observation_model_eval"),
        help="Directory to save figures and summary",
    )
    return parser.parse_args()


def load_metrics(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_model_comparison(metrics: Dict, out_path: Path) -> None:
    models: List[Dict] = metrics.get("models", [])
    if len(models) < 2:
        raise ValueError("metrics['models'] must have at least 2 entries")

    labels = [m.get("label", "unknown") for m in models]
    keys = [
        ("success_rate", "success rate", False),
        ("mean_score", "mean score", False),
        ("mean_residual_rmse", "mean residual rmse", True),
        ("mean_nis", "mean nis", True),
        ("mean_measurement_count", "mean measurement count", False),
        ("mean_runtime_ms", "mean runtime [ms]", True),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 8), dpi=120)
    axes = axes.ravel()

    for i, (key, title, lower_better) in enumerate(keys):
        values = [float(model.get(key, 0.0)) for model in models]
        bars = axes[i].bar(labels, values, color=["tab:blue", "tab:orange"])
        axes[i].set_title(title)
        axes[i].grid(axis="y", alpha=0.25)
        for bar, value in zip(bars, values):
            axes[i].text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height(),
                f"{value:.4f}",
                ha="center",
                va="bottom",
                fontsize=9,
            )
        if lower_better:
            axes[i].set_ylabel("lower is better")
        else:
            axes[i].set_ylabel("higher is better")

    fig.suptitle("Observation Model Evaluation: Model Comparison")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_delta_plot(metrics: Dict, out_path: Path) -> None:
    comparison: Dict = metrics.get("comparison", {})
    if not comparison:
        raise ValueError("metrics['comparison'] is missing")

    delta_keys = [
        "success_rate_delta",
        "mean_score_delta",
        "mean_residual_rmse_delta",
        "mean_nis_delta",
        "mean_measurement_count_delta",
        "mean_runtime_ms_delta",
    ]
    labels = [name.replace("_delta", "") for name in delta_keys]
    values = [float(comparison.get(name, 0.0)) for name in delta_keys]
    colors = ["tab:green" if value >= 0.0 else "tab:red" for value in values]

    y = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(10, 6), dpi=120)
    bars = ax.barh(y, values, color=colors)
    ax.set_yticks(y, labels)
    ax.axvline(0.0, color="black", linestyle="--", linewidth=1.0)
    ax.set_title("Model B - Model A Deltas")
    ax.grid(axis="x", alpha=0.25)

    for bar, value in zip(bars, values):
        x_pos = value + (0.01 if value >= 0.0 else -0.01)
        ax.text(
            x_pos,
            bar.get_y() + bar.get_height() / 2.0,
            f"{value:.6f}",
            va="center",
            ha="left" if value >= 0.0 else "right",
            fontsize=9,
        )

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_summary(metrics: Dict, out_path: Path) -> None:
    frame_count = int(metrics.get("frame_count", 0))
    models: List[Dict] = metrics.get("models", [])
    comparison: Dict = metrics.get("comparison", {})

    lines: List[str] = []
    lines.append("# Observation Model Evaluation Summary")
    lines.append("")
    lines.append(f"- frame_count: {frame_count}")
    for model in models:
        lines.append(
            "- "
            f"{model.get('label', 'unknown')}: "
            f"success_rate={float(model.get('success_rate', 0.0)):.6f}, "
            f"mean_score={float(model.get('mean_score', 0.0)):.6f}, "
            f"mean_residual_rmse={float(model.get('mean_residual_rmse', 0.0)):.6f}, "
            f"mean_nis={float(model.get('mean_nis', 0.0)):.6f}, "
            f"mean_runtime_ms={float(model.get('mean_runtime_ms', 0.0)):.6f}"
        )

    if comparison:
        lines.append("")
        lines.append("## Deltas (model B - model A)")
        for key, value in comparison.items():
            lines.append(f"- {key}: {float(value):.6f}")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    metrics = load_metrics(args.metrics)

    save_model_comparison(metrics, args.out_dir / "model_comparison.png")
    save_delta_plot(metrics, args.out_dir / "comparison_deltas.png")
    save_summary(metrics, args.out_dir / "summary.md")

    print("Observation model evaluation plots generated.")
    print(f"  metrics: {args.metrics}")
    print(f"  out_dir: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
