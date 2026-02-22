#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
from collections import Counter
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DIAG_PREFIX = "[ransac_diag]"
KV_PATTERN = re.compile(r"([a-zA-Z0-9_]+)=([^\s]+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze RANSAC-specific diagnostic logs")
    parser.add_argument(
        "--run",
        action="store_true",
        help="Run localization_test_app and capture diagnostics before analysis",
    )
    parser.add_argument(
        "--app",
        default="./build/localization_test_app",
        help="Command to run when --run is enabled",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Add --render when running app with --run",
    )
    parser.add_argument(
        "--diag-log",
        type=Path,
        default=Path("test/localization/logs/ransac_diag.log"),
        help="Path to ransac diagnostic log",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/ransac_diag"),
        help="Output directory for plots and metrics",
    )
    return parser.parse_args()


def run_and_capture(app_command: str, with_render: bool, diag_log_path: Path) -> int:
    args = app_command.split()
    if with_render:
        args.append("--render")

    completed = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = completed.stdout.splitlines()
    diag_lines = [line for line in lines if DIAG_PREFIX in line]

    diag_log_path.parent.mkdir(parents=True, exist_ok=True)
    diag_log_path.write_text("\n".join(diag_lines) + ("\n" if diag_lines else ""), encoding="utf-8")

    print(f"Captured diagnostics: {len(diag_lines)} lines -> {diag_log_path}")
    print(f"localization_test_app exit code: {completed.returncode}")
    return completed.returncode


def parse_diag_lines(diag_log_path: Path) -> List[Dict[str, str]]:
    if not diag_log_path.exists():
        raise FileNotFoundError(f"Diagnostic log not found: {diag_log_path}")

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


def as_int(rows: List[Dict[str, str]], key: str, default: int = 0) -> np.ndarray:
    values = []
    for row in rows:
        raw = row.get(key)
        try:
            values.append(int(raw) if raw is not None else default)
        except ValueError:
            values.append(default)
    return np.asarray(values, dtype=int)


def parse_segments(raw: str) -> List[Tuple[float, float, float, float]]:
    if not raw:
        return []

    segments: List[Tuple[float, float, float, float]] = []
    for token in raw.split("|"):
        parts = token.split(":")
        if len(parts) != 4:
            continue
        try:
            x1, y1, x2, y2 = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        except ValueError:
            continue
        segments.append((x1, y1, x2, y2))
    return segments


def save_timeseries(path: Path, title: str, y_label: str, values: np.ndarray) -> None:
    if values.size == 0:
        return
    x = np.arange(values.size)
    fig, ax = plt.subplots(figsize=(10, 4), dpi=120)
    ax.plot(x, values, lw=1.2)
    ax.set_title(title)
    ax.set_xlabel("diag frame index")
    ax.set_ylabel(y_label)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_histogram(path: Path, title: str, values: np.ndarray) -> None:
    if values.size == 0:
        return
    fig, ax = plt.subplots(figsize=(8, 4), dpi=120)
    bins = np.arange(values.min(), values.max() + 2) - 0.5
    ax.hist(values, bins=bins, edgecolor="white", alpha=0.9)
    ax.set_title(title)
    ax.set_xlabel("count")
    ax.set_ylabel("frequency")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_reject_bar(path: Path, reject_counter: Counter) -> None:
    if not reject_counter:
        return
    labels = list(reject_counter.keys())
    values = [reject_counter[label] for label in labels]

    fig, ax = plt.subplots(figsize=(9, 4), dpi=120)
    ax.bar(labels, values, alpha=0.9)
    ax.set_title("Reject reason counts")
    ax.set_ylabel("count")
    ax.grid(axis="y", alpha=0.25)
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_stage1_segment_grid(path: Path, rows: List[Dict[str, str]], max_frames: int = 9) -> None:
    frames = [(index, parse_segments(row.get("stage1_segments", ""))) for index, row in enumerate(rows)]
    frames = [(index, segments) for index, segments in frames if segments]
    if not frames:
        return

    sample_count = min(max_frames, len(frames))
    if sample_count == 1:
        chosen = [frames[0]]
    else:
        sample_indices = np.linspace(0, len(frames) - 1, num=sample_count).round().astype(int)
        chosen = [frames[idx] for idx in sample_indices]

    cols = 3
    rows_count = int(np.ceil(sample_count / cols))
    fig, axes = plt.subplots(rows_count, cols, figsize=(12, 4 * rows_count), dpi=120)
    axes = np.atleast_1d(axes).reshape(rows_count, cols)

    for plot_index in range(rows_count * cols):
        ax = axes[plot_index // cols, plot_index % cols]
        if plot_index >= sample_count:
            ax.axis("off")
            continue

        frame_index, segments = chosen[plot_index]
        xs: List[float] = []
        ys: List[float] = []
        for x1, y1, x2, y2 in segments:
            ax.plot([x1, x2], [y1, y2], "-", linewidth=2.0)
            xs.extend([x1, x2])
            ys.extend([y1, y2])

        if xs and ys:
            x_min = min(xs)
            x_max = max(xs)
            y_min = min(ys)
            y_max = max(ys)
            pad = 0.3
            ax.set_xlim(x_min - pad, x_max + pad)
            ax.set_ylim(y_min - pad, y_max + pad)

        ax.set_aspect("equal")
        ax.set_title(f"frame={frame_index}, segments={len(segments)}")
        ax.grid(alpha=0.25)

    fig.suptitle("Stage1 extracted segments (robot frame)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def build_metrics(rows: List[Dict[str, str]]) -> Dict:
    stage1 = as_int(rows, "stage1_extracted")
    stage2 = as_int(rows, "stage2_matches")
    gate = as_int(rows, "gate_passed")
    accepted = as_int(rows, "accepted")

    reject_counter = Counter(row.get("reject", "") for row in rows if "reject" in row)

    segment_counts = np.asarray(
        [len(parse_segments(row.get("stage1_segments", ""))) for row in rows], dtype=int
    )

    segment_lengths: List[float] = []
    for row in rows:
        for x1, y1, x2, y2 in parse_segments(row.get("stage1_segments", "")):
            segment_lengths.append(float(np.hypot(x2 - x1, y2 - y1)))
    segment_lengths_arr = np.asarray(segment_lengths, dtype=float)

    metrics = {
        "frames": int(len(rows)),
        "accepted_count": int(np.sum(accepted == 1)),
        "accepted_rate": float(np.mean(accepted == 1)) if accepted.size > 0 else 0.0,
        "stage1_extracted": {
            "mean": float(np.mean(stage1)) if stage1.size > 0 else 0.0,
            "median": float(np.median(stage1)) if stage1.size > 0 else 0.0,
            "min": int(np.min(stage1)) if stage1.size > 0 else 0,
            "max": int(np.max(stage1)) if stage1.size > 0 else 0,
            "distribution": {str(k): int(v) for k, v in sorted(Counter(stage1).items())},
        },
        "stage2_matches": {
            "mean": float(np.mean(stage2)) if stage2.size > 0 else 0.0,
            "median": float(np.median(stage2)) if stage2.size > 0 else 0.0,
            "zero_rate": float(np.mean(stage2 == 0)) if stage2.size > 0 else 0.0,
            "distribution": {str(k): int(v) for k, v in sorted(Counter(stage2).items())},
        },
        "gate_passed": {
            "mean": float(np.mean(gate)) if gate.size > 0 else 0.0,
            "median": float(np.median(gate)) if gate.size > 0 else 0.0,
            "zero_rate": float(np.mean(gate == 0)) if gate.size > 0 else 0.0,
            "distribution": {str(k): int(v) for k, v in sorted(Counter(gate).items())},
        },
        "reject_reasons": dict(reject_counter),
        "stage1_segments": {
            "frames_with_segments_rate": float(np.mean(segment_counts > 0))
            if segment_counts.size > 0
            else 0.0,
            "mean_segments_per_frame": float(np.mean(segment_counts)) if segment_counts.size > 0 else 0.0,
            "mean_segment_length": float(np.mean(segment_lengths_arr))
            if segment_lengths_arr.size > 0
            else 0.0,
            "median_segment_length": float(np.median(segment_lengths_arr))
            if segment_lengths_arr.size > 0
            else 0.0,
        },
    }
    return metrics


def main() -> int:
    args = parse_args()

    if args.run:
        run_and_capture(args.app, args.render, args.diag_log)

    rows = parse_diag_lines(args.diag_log)
    if not rows:
        raise RuntimeError(f"No {DIAG_PREFIX} lines found in {args.diag_log}")

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    stage1 = as_int(rows, "stage1_extracted")
    stage2 = as_int(rows, "stage2_matches")
    gate = as_int(rows, "gate_passed")

    save_timeseries(out_dir / "stage1_extracted_timeseries.png", "Stage1 extracted lines", "count", stage1)
    save_timeseries(out_dir / "stage2_matches_timeseries.png", "Stage2 matches", "count", stage2)
    save_timeseries(out_dir / "gate_passed_timeseries.png", "Gate passed", "count", gate)
    save_histogram(out_dir / "stage1_extracted_histogram.png", "Stage1 extracted distribution", stage1)
    save_histogram(out_dir / "stage2_matches_histogram.png", "Stage2 matches distribution", stage2)
    save_stage1_segment_grid(out_dir / "stage1_segments_grid.png", rows)

    reject_counter = Counter(row.get("reject", "") for row in rows if "reject" in row)
    save_reject_bar(out_dir / "reject_reasons.png", reject_counter)

    metrics = build_metrics(rows)
    metrics_path = out_dir / "ransac_diag_metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Parsed frames: {metrics['frames']}")
    print(f"Accepted updates: {metrics['accepted_count']} ({metrics['accepted_rate']:.3f})")
    print(f"Saved metrics: {metrics_path}")
    print(f"Saved plots: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
