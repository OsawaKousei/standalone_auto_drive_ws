#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

def parse_log(path: Path):
    rows = []
    result = None
    error = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if line.startswith("# result="):
                    result = line.split("=", 1)[1].strip()
                if line.startswith("# error="):
                    error = line.split("=", 1)[1].strip()
                continue
            if line.startswith("step,"):
                continue
            rows.append(line)

    records = []
    for row in csv.reader(rows):
        if len(row) < 14:
            raise ValueError(f"Unexpected column count: {len(row)}")
        record = {
            "step": int(row[0]),
            "dist_before": float(row[1]),
            "dist_after": float(row[2]),
            "pos_err": float(row[3]),
            "head_err": float(row[4]),
            "score": float(row[5]),
            "true_x": float(row[6]),
            "true_y": float(row[7]),
            "true_theta": float(row[8]),
            "est_x": float(row[9]),
            "est_y": float(row[10]),
            "est_theta": float(row[11]),
            "v": float(row[12]),
            "w": float(row[13]),
        }
        records.append(record)

    return records, result, error


def stats(values):
    if not values:
        return 0.0, 0.0, 0.0
    mean = sum(values) / len(values)
    rms = math.sqrt(sum(v * v for v in values) / len(values))
    max_v = max(values)
    return mean, rms, max_v


def main():
    parser = argparse.ArgumentParser(description="Verify localization control log.")
    parser.add_argument(
        "log_path",
        nargs="?",
        default="logs/localization_control_lidar_demo.log",
        help="Path to the log file",
    )
    args = parser.parse_args()

    path = Path(args.log_path)
    if not path.exists():
        raise SystemExit(f"Log file not found: {path}")

    records, result, error = parse_log(path)
    if not records:
        raise SystemExit("No records found in log.")

    pos_values = [r["pos_err"] for r in records]
    head_values = [r["head_err"] for r in records]
    dist_after = [r["dist_after"] for r in records]

    pos_mean, pos_rms, pos_max = stats(pos_values)
    head_mean, head_rms, head_max = stats(head_values)

    print(f"Records: {len(records)}")
    print(f"Final distance: {dist_after[-1]:.4f}")
    print("Position error: mean={:.4f} m, RMS={:.4f} m, max={:.4f} m".format(
        pos_mean, pos_rms, pos_max
    ))
    print("Heading error:  mean={:.4f} rad, RMS={:.4f} rad, max={:.4f} rad".format(
        head_mean, head_rms, head_max
    ))

    if result:
        print(f"Result: {result}")
    if error:
        print(f"Error: {error}")

    if result and result != "success":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
