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
        description="Compare top optimization results (best ATE) between hough+rancac and hough-only."
    )
    parser.add_argument(
        "--ransac-result",
        type=Path,
        default=Path("test/localization/logs/optuna/optuna_localization_result.json"),
        help="Path to hough+rancac optimization result JSON",
    )
    parser.add_argument(
        "--hough-only-result",
        type=Path,
        default=Path("test/localization/logs/optuna_hough_only/optuna_hough_only_result.json"),
        help="Path to hough-only optimization result JSON",
    )
    parser.add_argument(
        "--ransac-label",
        type=str,
        default="hough_ransac",
        help="Display label for ransac model",
    )
    parser.add_argument(
        "--hough-only-label",
        type=str,
        default="hough_only",
        help="Display label for hough-only model",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("test/localization/logs/analysis/optuna_compare_top"),
        help="Directory to save top-result comparison outputs",
    )
    return parser.parse_args()


def to_abs(path: Path, workspace_root: Path) -> Path:
    if path.is_absolute():
        return path
    return (workspace_root / path).resolve()


def load_result_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if "best_value" not in data:
        raise KeyError(f"best_value is missing: {path}")
    return data


def decide_superiority(ransac_best: float, hough_best: float) -> dict[str, Any]:
    if hough_best <= 0.0 or ransac_best <= 0.0:
        return {
            "winner": "inconclusive",
            "confidence": "low",
            "reason": "Invalid best ATE value detected.",
            "margin_abs": None,
            "margin_ratio": None,
        }

    margin_abs = ransac_best - hough_best
    margin_ratio = margin_abs / ransac_best

    if margin_abs > 0.0:
        winner = "hough_only"
    elif margin_abs < 0.0:
        winner = "hough_ransac"
    else:
        winner = "tie"

    improvement_abs = abs(margin_abs)
    improvement_ratio = abs(margin_ratio)

    if improvement_ratio >= 0.20:
        confidence = "high"
    elif improvement_ratio >= 0.10:
        confidence = "medium"
    else:
        confidence = "low"

    if winner == "tie":
        reason = "Top ATE values are equal."
    elif winner == "hough_only":
        reason = "Top ATE of hough-only is lower than hough+rancac."
    else:
        reason = "Top ATE of hough+rancac is lower than hough-only."

    return {
        "winner": winner,
        "confidence": confidence,
        "reason": reason,
        "margin_abs": improvement_abs,
        "margin_ratio": improvement_ratio,
        "signed_margin_ransac_minus_hough": margin_abs,
    }


def plot_top_ate_bar(labels: list[str], values: list[float], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=120)
    bars = ax.bar(labels, values)
    ax.set_title("Top ATE Comparison (lower is better)")
    ax.set_ylabel("Best ATE")
    ax.grid(axis="y", alpha=0.2)

    max_v = max(values)
    y_offset = max_v * 0.03 if max_v > 0.0 else 0.01
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2.0, value + y_offset, f"{value:.6f}", ha="center")

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def write_markdown_report(
    out_path: Path,
    ransac_label: str,
    hough_label: str,
    ransac_result: dict[str, Any],
    hough_result: dict[str, Any],
    decision: dict[str, Any],
    figure_path: Path,
) -> None:
    lines = [
        "# Top Result Comparison Report",
        "",
        "## Summary",
        f"- Winner: **{decision['winner']}**",
        f"- Confidence: **{decision['confidence']}**",
        f"- Reason: {decision['reason']}",
        f"- Absolute margin (ATE): {decision['margin_abs']:.6f}" if decision["margin_abs"] is not None else "- Absolute margin (ATE): N/A",
        f"- Relative margin: {decision['margin_ratio']:.2%}" if decision["margin_ratio"] is not None else "- Relative margin: N/A",
        "",
        "## Top Metrics",
        "",
        f"| Metric | {ransac_label} | {hough_label} |",
        "|---|---:|---:|",
        f"| best_ATE | {float(ransac_result['best_value']):.6f} | {float(hough_result['best_value']):.6f} |",
        f"| best_trial | {ransac_result.get('best_trial', 'N/A')} | {hough_result.get('best_trial', 'N/A')} |",
        f"| n_trials | {ransac_result.get('n_trials', 'N/A')} | {hough_result.get('n_trials', 'N/A')} |",
        f"| n_complete | {ransac_result.get('n_complete', 'N/A')} | {hough_result.get('n_complete', 'N/A')} |",
        f"| final_config_mode | {ransac_result.get('final_config_mode', 'N/A')} | {hough_result.get('final_config_mode', 'N/A')} |",
        "",
        "## Note",
        "- This comparison uses only top optimization results (best_value), not trial distribution.",
        "",
        "## Artifact",
        f"- Top ATE bar chart: {figure_path}",
    ]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    workspace_root = Path(__file__).resolve().parents[2]

    ransac_path = to_abs(args.ransac_result, workspace_root)
    hough_path = to_abs(args.hough_only_result, workspace_root)
    out_dir = to_abs(args.out_dir, workspace_root)

    if not ransac_path.exists():
        raise FileNotFoundError(f"ransac result file not found: {ransac_path}")
    if not hough_path.exists():
        raise FileNotFoundError(f"hough-only result file not found: {hough_path}")

    out_dir.mkdir(parents=True, exist_ok=True)

    ransac_result = load_result_json(ransac_path)
    hough_result = load_result_json(hough_path)

    ransac_best = float(ransac_result["best_value"])
    hough_best = float(hough_result["best_value"])

    decision = decide_superiority(ransac_best, hough_best)

    fig_path = out_dir / "top_ate_comparison.png"
    plot_top_ate_bar(
        labels=[args.ransac_label, args.hough_only_label],
        values=[ransac_best, hough_best],
        out_path=fig_path,
    )

    summary = {
        "inputs": {
            "ransac_result": str(ransac_path),
            "hough_only_result": str(hough_path),
        },
        "ransac": {
            "label": args.ransac_label,
            "best_ate": ransac_best,
            "best_trial": ransac_result.get("best_trial"),
            "n_trials": ransac_result.get("n_trials"),
            "n_complete": ransac_result.get("n_complete"),
            "final_config_mode": ransac_result.get("final_config_mode"),
        },
        "hough_only": {
            "label": args.hough_only_label,
            "best_ate": hough_best,
            "best_trial": hough_result.get("best_trial"),
            "n_trials": hough_result.get("n_trials"),
            "n_complete": hough_result.get("n_complete"),
            "final_config_mode": hough_result.get("final_config_mode"),
        },
        "decision": decision,
        "artifact": {
            "top_ate_comparison": str(fig_path),
        },
    }

    summary_path = out_dir / "top_comparison_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    report_path = out_dir / "top_comparison_report.md"
    write_markdown_report(
        out_path=report_path,
        ransac_label=args.ransac_label,
        hough_label=args.hough_only_label,
        ransac_result=ransac_result,
        hough_result=hough_result,
        decision=decision,
        figure_path=fig_path,
    )

    print("Top-result comparison complete")
    print(f"  summary: {summary_path}")
    print(f"  report: {report_path}")
    print(f"  figure: {fig_path}")
    print(f"  winner: {decision['winner']} (confidence={decision['confidence']})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
