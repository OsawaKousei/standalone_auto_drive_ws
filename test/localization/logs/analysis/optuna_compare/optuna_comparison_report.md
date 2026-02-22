# Localization Optuna Comparison Report

## Summary
- Winner: **hough_only**
- Confidence: **high**
- Reason: Median ATE difference bootstrap 95% CI is entirely above 0 and success-rate gap is acceptable.

## Core Metrics

| Metric | hough_ransac | hough_only |
|---|---:|---:|
| Total trials | 200 | 200 |
| Successful trials | 88 | 165 |
| Success rate | 0.4400 | 0.8250 |
| Best ATE | 0.105821 | 0.072291 |
| Median ATE | 0.134400 | 0.089981 |
| Mean ATE | 0.145768 | 0.093031 |
| Std ATE | 0.034207 | 0.013291 |
| P10 ATE | 0.114371 | 0.078416 |
| P25 ATE | 0.120456 | 0.083746 |
| P75 ATE | 0.157518 | 0.099597 |

## Difference (ransac - hough_only)
- Median ATE diff: 0.044419
- Mean ATE diff: 0.052737
- Best ATE diff: 0.033530
- Success-rate diff: -0.385000
- Bootstrap median diff 95% CI: [0.038543, 0.053428]

## Artifacts
- Histogram: /workspace/test/localization/logs/analysis/optuna_compare/ate_histogram.png
- Boxplot: /workspace/test/localization/logs/analysis/optuna_compare/ate_boxplot.png
- ECDF: /workspace/test/localization/logs/analysis/optuna_compare/ate_ecdf.png
- Success rate: /workspace/test/localization/logs/analysis/optuna_compare/success_rate.png
