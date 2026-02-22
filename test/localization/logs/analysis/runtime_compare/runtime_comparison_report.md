# Runtime Localization Comparison

## Summary
- Result status: hough_ransac=success, hough_only=success
- Wins: hough_ransac=3, hough_only=8
- Overall winner: **hough_only**

## Metric-by-metric

| metric | direction | hough_ransac | hough_only | winner |
|---|---|---:|---:|---|
| ate | lower_is_better | 0.121792 | 0.102259 | hough_only |
| rmse | lower_is_better | 0.139211 | 0.128818 | hough_only |
| rpe | lower_is_better | 0.023713 | 0.012222 | hough_only |
| rpe_rot_rmse | lower_is_better | 0.010558 | 0.008917 | hough_only |
| final_position_error | lower_is_better | 0.102422 | 0.145357 | hough_ransac |
| final_heading_error | lower_is_better | 0.000553 | 0.080934 | hough_ransac |
| observation_update.count | higher_is_better | 102.000000 | 96.000000 | hough_ransac |
| observation_update.position_improvement_rate | higher_is_better | 0.656863 | 0.750000 | hough_only |
| observation_update.heading_improvement_rate | higher_is_better | 0.666667 | 0.687500 | hough_only |
| observation_update.position_effective_rate | higher_is_better | 0.676471 | 0.750000 | hough_only |
| observation_update.heading_effective_rate | higher_is_better | 0.686275 | 0.687500 | hough_only |

## Visualization
- /workspace/test/localization/logs/analysis/runtime_compare/core_metrics_bar.png
