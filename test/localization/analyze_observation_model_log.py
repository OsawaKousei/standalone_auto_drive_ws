#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
import numpy as np

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
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Path to observation_model_eval.csv (optional; auto-resolved from --metrics when omitted)",
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


def resolve_csv_path(metrics_path: Path, csv_arg: Optional[Path]) -> Path:
    if csv_arg is not None:
        return csv_arg
    return metrics_path.parent / "observation_model_eval.csv"


def load_csv_records(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    if not path.exists():
        return [], []
    with path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = [dict(row) for row in reader]
        columns = list(reader.fieldnames or [])
    return columns, rows


def column_as_float(rows: List[Dict[str, str]], key: str) -> np.ndarray:
    values: List[float] = []
    for row in rows:
        raw = row.get(key, "")
        try:
            values.append(float(raw))
        except (TypeError, ValueError):
            values.append(float("nan"))
    return np.asarray(values, dtype=float)


def rolling_mean(values: np.ndarray, window: int) -> np.ndarray:
    if values.size == 0:
        return np.zeros(0, dtype=float)
    if window <= 1:
        return values.copy()
    out = np.empty_like(values)
    for i in range(values.size):
        lo = max(0, i - window + 1)
        chunk = values[lo : i + 1]
        finite = chunk[np.isfinite(chunk)]
        out[i] = np.mean(finite) if finite.size > 0 else np.nan
    return out


def finite(values: np.ndarray) -> np.ndarray:
    return values[np.isfinite(values)]


def ratio(mask: np.ndarray) -> float:
    if mask.size == 0:
        return 0.0
    return float(np.mean(mask.astype(float)))


def safe_mean(values: np.ndarray) -> float:
    f = finite(values)
    if f.size == 0:
        return 0.0
    return float(np.mean(f))


def safe_median(values: np.ndarray) -> float:
    f = finite(values)
    if f.size == 0:
        return 0.0
    return float(np.median(f))

def save_metric_triplet_plot(
    out_path: Path,
    frame_index: np.ndarray,
    values: np.ndarray,
    title: str,
    y_label: str,
    lower_is_better: bool,
) -> None:
    if values.size == 0:
        return

    smoothed = rolling_mean(values, window=20)
    delta = np.diff(values, prepend=values[0])

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), dpi=120, sharex=True)

    axes[0].plot(frame_index, values, lw=1.2, color="tab:blue")
    axes[0].set_ylabel(y_label)
    axes[0].grid(alpha=0.25)

    axes[1].plot(frame_index, smoothed, lw=1.2, color="tab:orange")
    axes[1].set_ylabel("rolling mean")
    axes[1].grid(alpha=0.25)

    improve_mask = delta < 0.0 if lower_is_better else delta > 0.0
    colors = np.where(improve_mask, "tab:green", "tab:red")
    axes[2].bar(frame_index, delta, width=0.8, color=colors, alpha=0.8)
    axes[2].axhline(0.0, color="black", linestyle="--", linewidth=1.0, alpha=0.7)
    axes[2].set_ylabel("delta")
    axes[2].set_xlabel("frame")
    axes[2].grid(alpha=0.25)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_distribution_plots(out_path: Path, metric_values: List[Tuple[str, np.ndarray]]) -> None:
    valid_items = [(name, finite(values)) for name, values in metric_values if finite(values).size > 0]
    if not valid_items:
        return

    rows = 2
    cols = 2
    fig, axes = plt.subplots(rows, cols, figsize=(12, 8), dpi=120)
    axes = np.asarray(axes).ravel()

    for i, (name, values) in enumerate(valid_items[: rows * cols]):
        ax = axes[i]
        ax.hist(values, bins=30, alpha=0.8, color="tab:blue", edgecolor="white")
        median = float(np.median(values))
        p95 = float(np.percentile(values, 95))
        ax.axvline(median, color="tab:orange", linestyle="--", linewidth=1.0, label=f"median={median:.4f}")
        ax.axvline(p95, color="tab:red", linestyle=":", linewidth=1.0, label=f"p95={p95:.4f}")
        ax.set_title(f"{name} distribution")
        ax.grid(alpha=0.2)
        ax.legend(loc="best", fontsize=8)

    for j in range(len(valid_items), rows * cols):
        axes[j].axis("off")

    fig.suptitle("Observation Model Metric Distributions")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_runtime_cdf(out_path: Path, runtime_ms: np.ndarray) -> None:
    values = np.sort(finite(runtime_ms))
    if values.size == 0:
        return
    y = np.linspace(0.0, 1.0, values.size)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=120)
    ax.plot(values, y, lw=1.5, color="tab:blue")
    ax.set_title("Runtime CDF")
    ax.set_xlabel("runtime [ms]")
    ax.set_ylabel("cdf")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def save_status_timeline(out_path: Path, status_codes: np.ndarray) -> None:
    if status_codes.size == 0:
        return
    frame_idx = np.arange(status_codes.size, dtype=float)
    fig, ax = plt.subplots(figsize=(10, 3), dpi=120)
    ax.scatter(frame_idx, status_codes, s=10, alpha=0.85)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["error", "no_update", "success"])
    ax.set_xlabel("frame")
    ax.set_title("Frame Status Timeline")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def build_advanced_stats(rows: List[Dict[str, str]]) -> Dict:
    if not rows:
        return {}

    status = np.asarray([row.get("status", "") for row in rows], dtype=object)
    frame_idx = column_as_float(rows, "frame")

    success_mask = status == "success"
    no_update_mask = status == "no_update"
    error_mask = status == "error"

    score = column_as_float(rows, "score")
    residual = column_as_float(rows, "residual_rmse")
    nis = column_as_float(rows, "nis")
    measurement_count = column_as_float(rows, "measurement_count")
    runtime_ms = column_as_float(rows, "runtime_ms")

    success_score = score[success_mask]
    success_residual = residual[success_mask]
    success_nis = nis[success_mask]
    success_measurement_count = measurement_count[success_mask]

    delta_score = np.diff(success_score) if success_score.size >= 2 else np.zeros(0, dtype=float)
    delta_residual = np.diff(success_residual) if success_residual.size >= 2 else np.zeros(0, dtype=float)
    delta_nis = np.diff(success_nis) if success_nis.size >= 2 else np.zeros(0, dtype=float)

    return {
        "rows": int(len(rows)),
        "frame_start": int(frame_idx[0]) if frame_idx.size > 0 and np.isfinite(frame_idx[0]) else 0,
        "frame_end": int(frame_idx[-1]) if frame_idx.size > 0 and np.isfinite(frame_idx[-1]) else 0,
        "status": {
            "success_rate": ratio(success_mask),
            "no_update_rate": ratio(no_update_mask),
            "error_rate": ratio(error_mask),
        },
        "score": {
            "median": safe_median(success_score),
            "p95": float(np.percentile(finite(success_score), 95)) if finite(success_score).size > 0 else 0.0,
            "delta_improvement_rate": ratio(delta_score > 0.0),
        },
        "residual_rmse": {
            "median": safe_median(success_residual),
            "p95": float(np.percentile(finite(success_residual), 95)) if finite(success_residual).size > 0 else 0.0,
            "delta_improvement_rate": ratio(delta_residual < 0.0),
        },
        "nis": {
            "median": safe_median(success_nis),
            "p95": float(np.percentile(finite(success_nis), 95)) if finite(success_nis).size > 0 else 0.0,
            "delta_improvement_rate": ratio(delta_nis < 0.0),
        },
        "measurement_count": {
            "mean": safe_mean(success_measurement_count),
            "median": safe_median(success_measurement_count),
        },
        "runtime_ms": {
            "median": safe_median(runtime_ms),
            "p95": float(np.percentile(finite(runtime_ms), 95)) if finite(runtime_ms).size > 0 else 0.0,
        },
    }


def save_advanced_plots(csv_rows: List[Dict[str, str]], out_dir: Path) -> Dict:
    if not csv_rows:
        return {}

    status = np.asarray([row.get("status", "") for row in csv_rows], dtype=object)
    success_mask = status == "success"
    status_codes = np.zeros(status.size, dtype=float)
    status_codes[status == "no_update"] = 1.0
    status_codes[status == "success"] = 2.0

    frame_all = np.arange(status.size, dtype=float)
    frame_success = frame_all[success_mask]

    score = column_as_float(csv_rows, "score")
    residual = column_as_float(csv_rows, "residual_rmse")
    nis = column_as_float(csv_rows, "nis")
    measurement_count = column_as_float(csv_rows, "measurement_count")
    runtime_ms = column_as_float(csv_rows, "runtime_ms")

    save_status_timeline(out_dir / "status_timeline.png", status_codes)

    if np.any(success_mask):
        save_metric_triplet_plot(
            out_dir / "score_triplet.png",
            frame_success,
            score[success_mask],
            "Score Evolution (Success Frames): value / rolling mean / delta",
            "score",
            lower_is_better=False,
        )
        save_metric_triplet_plot(
            out_dir / "residual_rmse_triplet.png",
            frame_success,
            residual[success_mask],
            "Residual RMSE Evolution (Success Frames): value / rolling mean / delta",
            "rmse",
            lower_is_better=True,
        )
        save_metric_triplet_plot(
            out_dir / "nis_triplet.png",
            frame_success,
            nis[success_mask],
            "NIS Evolution (Success Frames): value / rolling mean / delta",
            "nis",
            lower_is_better=True,
        )
        save_metric_triplet_plot(
            out_dir / "measurement_count_triplet.png",
            frame_success,
            measurement_count[success_mask],
            "Measurement Count Evolution (Success Frames): value / rolling mean / delta",
            "count",
            lower_is_better=False,
        )

    save_metric_triplet_plot(
        out_dir / "runtime_triplet.png",
        frame_all,
        runtime_ms,
        "Runtime Evolution (All Frames): value / rolling mean / delta",
        "runtime [ms]",
        lower_is_better=True,
    )

    save_distribution_plots(
        out_dir / "metric_distributions.png",
        [
            ("score", score[success_mask]),
            ("residual_rmse", residual[success_mask]),
            ("nis", nis[success_mask]),
            ("runtime_ms", runtime_ms),
        ],
    )
    save_runtime_cdf(out_dir / "runtime_cdf.png", runtime_ms)

    return build_advanced_stats(csv_rows)


def save_summary(frame_count: int, model: Dict, advanced: Dict, out_path: Path) -> None:

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

    if advanced:
        lines.append("")
        lines.append("## Advanced metrics from CSV")
        lines.append("")
        lines.append(f"- rows: {int(advanced.get('rows', 0))}")
        lines.append(f"- frame_start: {int(advanced.get('frame_start', 0))}")
        lines.append(f"- frame_end: {int(advanced.get('frame_end', 0))}")

        status = advanced.get("status", {})
        lines.append(f"- success_rate_csv: {float(status.get('success_rate', 0.0)):.6f}")
        lines.append(f"- no_update_rate_csv: {float(status.get('no_update_rate', 0.0)):.6f}")
        lines.append(f"- error_rate_csv: {float(status.get('error_rate', 0.0)):.6f}")

        score = advanced.get("score", {})
        lines.append(f"- score_median: {float(score.get('median', 0.0)):.6f}")
        lines.append(f"- score_p95: {float(score.get('p95', 0.0)):.6f}")
        lines.append(
            f"- score_delta_improvement_rate: {float(score.get('delta_improvement_rate', 0.0)):.6f}"
        )

        residual = advanced.get("residual_rmse", {})
        lines.append(f"- residual_median: {float(residual.get('median', 0.0)):.6f}")
        lines.append(f"- residual_p95: {float(residual.get('p95', 0.0)):.6f}")
        lines.append(
            f"- residual_delta_improvement_rate: {float(residual.get('delta_improvement_rate', 0.0)):.6f}"
        )

        nis = advanced.get("nis", {})
        lines.append(f"- nis_median: {float(nis.get('median', 0.0)):.6f}")
        lines.append(f"- nis_p95: {float(nis.get('p95', 0.0)):.6f}")
        lines.append(
            f"- nis_delta_improvement_rate: {float(nis.get('delta_improvement_rate', 0.0)):.6f}"
        )

        runtime = advanced.get("runtime_ms", {})
        lines.append(f"- runtime_median_ms: {float(runtime.get('median', 0.0)):.6f}")
        lines.append(f"- runtime_p95_ms: {float(runtime.get('p95', 0.0)):.6f}")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    metrics = load_metrics(args.metrics)
    frame_count, model = parse_model(metrics)

    csv_path = resolve_csv_path(args.metrics, args.csv)
    _, csv_rows = load_csv_records(csv_path)

    advanced_stats = save_advanced_plots(csv_rows, args.out_dir)
    save_summary(frame_count, model, advanced_stats, args.out_dir / "summary.md")

    print("Observation model evaluation plots generated.")
    print(f"  metrics: {args.metrics}")
    if csv_rows:
        print(f"  csv: {csv_path}")
        print("  advanced_plots: enabled")
    else:
        print(f"  csv: {csv_path} (not found; advanced plots skipped)")
    print(f"  out_dir: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
