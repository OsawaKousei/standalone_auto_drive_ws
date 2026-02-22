#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare runtime localization metrics between hough+rancac and hough-only runs"
    )
    parser.add_argument(
        "--ransac-metrics",
        type=Path,
        default=Path("test/localization/logs/analysis/localization_eval_ransac/metrics.json"),
    )
    parser.add_argument(
        "--hough-metrics",
        type=Path,
        default=Path("test/localization/logs/analysis/localization_eval_hough_only/metrics.json"),
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/runtime_compare"),
    )
    parser.add_argument("--ransac-label", type=str, default="hough_ransac")
    parser.add_argument("--hough-label", type=str, default="hough_only")
    return parser.parse_args()


def to_abs(path: Path, workspace_root: Path) -> Path:
    return path if path.is_absolute() else (workspace_root / path).resolve()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def metric_value(metrics: dict[str, Any], key: str) -> float:
    if key.startswith("observation_update."):
        _, subkey = key.split(".", 1)
        return float(metrics.get("observation_update", {}).get(subkey, 0.0))
    return float(metrics.get(key, 0.0))


def build_comparison(ransac: dict[str, Any], hough: dict[str, Any]) -> dict[str, Any]:
    lower_better = [
        "ate",
        "rmse",
        "rpe",
        "rpe_rot_rmse",
        "final_position_error",
        "final_heading_error",
    ]
    higher_better = [
        "observation_update.count",
        "observation_update.position_improvement_rate",
        "observation_update.heading_improvement_rate",
        "observation_update.position_effective_rate",
        "observation_update.heading_effective_rate",
    ]

    metrics_out: list[dict[str, Any]] = []

    for key in lower_better:
        rv = metric_value(ransac, key)
        hv = metric_value(hough, key)
        winner = "hough_ransac" if rv < hv else ("hough_only" if hv < rv else "tie")
        diff = rv - hv
        metrics_out.append(
            {
                "metric": key,
                "direction": "lower_is_better",
                "ransac": rv,
                "hough_only": hv,
                "winner": winner,
                "signed_diff_ransac_minus_hough": diff,
            }
        )

    for key in higher_better:
        rv = metric_value(ransac, key)
        hv = metric_value(hough, key)
        winner = "hough_ransac" if rv > hv else ("hough_only" if hv > rv else "tie")
        diff = rv - hv
        metrics_out.append(
            {
                "metric": key,
                "direction": "higher_is_better",
                "ransac": rv,
                "hough_only": hv,
                "winner": winner,
                "signed_diff_ransac_minus_hough": diff,
            }
        )

    ransac_wins = sum(1 for item in metrics_out if item["winner"] == "hough_ransac")
    hough_wins = sum(1 for item in metrics_out if item["winner"] == "hough_only")

    if ransac_wins > hough_wins:
        overall = "hough_ransac"
    elif hough_wins > ransac_wins:
        overall = "hough_only"
    else:
        overall = "tie"

    return {
        "result_status": {
            "hough_ransac": ransac.get("result", "unknown"),
            "hough_only": hough.get("result", "unknown"),
        },
        "score": {
            "hough_ransac_wins": ransac_wins,
            "hough_only_wins": hough_wins,
            "overall_winner": overall,
        },
        "metrics": metrics_out,
    }


def plot_core_metrics(ransac: dict[str, Any], hough: dict[str, Any], out_path: Path, ransac_label: str, hough_label: str) -> None:
    keys = ["ate", "rmse", "rpe", "final_position_error", "final_heading_error"]
    r_vals = [metric_value(ransac, key) for key in keys]
    h_vals = [metric_value(hough, key) for key in keys]

    x = range(len(keys))
    width = 0.38

    fig, ax = plt.subplots(figsize=(9, 5), dpi=120)
    ax.bar([xi - width / 2 for xi in x], r_vals, width=width, label=ransac_label)
    ax.bar([xi + width / 2 for xi in x], h_vals, width=width, label=hough_label)
    ax.set_xticks(list(x))
    ax.set_xticklabels(keys, rotation=20)
    ax.set_title("Core Error Metrics (lower is better)")
    ax.set_ylabel("Value")
    ax.grid(axis="y", alpha=0.2)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def write_markdown(out_path: Path, comparison: dict[str, Any], ransac_label: str, hough_label: str, fig_path: Path) -> None:
    lines = [
        "# Runtime Localization Comparison",
        "",
        "## Summary",
        f"- Result status: {ransac_label}={comparison['result_status']['hough_ransac']}, {hough_label}={comparison['result_status']['hough_only']}",
        f"- Wins: {ransac_label}={comparison['score']['hough_ransac_wins']}, {hough_label}={comparison['score']['hough_only_wins']}",
        f"- Overall winner: **{comparison['score']['overall_winner']}**",
        "",
        "## Metric-by-metric",
        "",
        "| metric | direction | hough_ransac | hough_only | winner |",
        "|---|---|---:|---:|---|",
    ]

    for item in comparison["metrics"]:
        lines.append(
            f"| {item['metric']} | {item['direction']} | {item['ransac']:.6f} | {item['hough_only']:.6f} | {item['winner']} |"
        )

    lines.extend([
        "",
        "## Visualization",
        f"- {fig_path}",
    ])

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    workspace_root = Path(__file__).resolve().parents[2]

    ransac_path = to_abs(args.ransac_metrics, workspace_root)
    hough_path = to_abs(args.hough_metrics, workspace_root)
    out_dir = to_abs(args.out_dir, workspace_root)

    if not ransac_path.exists() or not hough_path.exists():
        raise FileNotFoundError("Metrics files are missing. Run evaluations first.")

    out_dir.mkdir(parents=True, exist_ok=True)

    ransac_metrics = read_json(ransac_path)
    hough_metrics = read_json(hough_path)

    comparison = build_comparison(ransac_metrics, hough_metrics)

    fig_path = out_dir / "core_metrics_bar.png"
    plot_core_metrics(ransac_metrics, hough_metrics, fig_path, args.ransac_label, args.hough_label)

    summary_path = out_dir / "runtime_comparison_summary.json"
    summary_path.write_text(json.dumps(comparison, indent=2, ensure_ascii=False), encoding="utf-8")

    report_path = out_dir / "runtime_comparison_report.md"
    write_markdown(report_path, comparison, args.ransac_label, args.hough_label, fig_path)

    print("Runtime comparison complete")
    print(f"  summary: {summary_path}")
    print(f"  report: {report_path}")
    print(f"  figure: {fig_path}")
    print(f"  overall winner: {comparison['score']['overall_winner']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
