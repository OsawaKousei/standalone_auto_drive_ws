# analyze_localization_log.py 出力仕様

対象スクリプト: `test/localization/analyze_localization_log.py`

## 1. 入出力

- 入力ログ（既定）: `test/localization/logs/localization_test.log`
- 出力ディレクトリ（既定）: `test/localization/logs/analysis/localization_test`
- 実行例:
  - `python3 test/localization/analyze_localization_log.py`

## 2. 生成物

以下のファイルを出力します。

- `map_true_vs_estimated.png`
  - map 上に真値軌跡（青）と推定軌跡（赤）を重ねて表示
- `position_error_timeseries.png`
  - 位置誤差（`pos_err`）の時系列
- `heading_error_timeseries.png`
  - 方位誤差（`heading_err`）の時系列
- `update_position_error_triplet.png`
  - 観測更新時刻における位置誤差の `e_before / e_after / delta_e` 時系列
- `update_heading_error_triplet.png`
  - 観測更新時刻における方位誤差の `e_before / e_after / delta_e` 時系列
- `map_update_improvement_colored.png`
  - 観測更新時刻点を map 上で色分け（改善=青、悪化=赤）
- `metrics.json`
  - 主要メトリクス

## 3. メトリクス定義

- `ate`
  - Absolute Trajectory Error の平均値
  - `mean( ||p_est - p_true|| )`
- `rmse`
  - 推定位置誤差の RMSE
  - `sqrt(mean( ||p_est - p_true||^2 ))`
- `rpe`
  - Relative Pose Error の並進成分 RMSE
  - 連続ステップ間の相対移動差の RMSE
- `rpe_rot_rmse`
  - Relative Pose Error の回転成分 RMSE

### 3.1 観測更新（`lidar_updated=1`）の評価

- `observation_update.mean_delta_pos_err`
  - 位置誤差改善量の平均（`pre_update_pos_err - post_update_pos_err`）
- `observation_update.mean_delta_heading_err`
  - 方位誤差改善量の平均（`pre_update_heading_err - post_update_heading_err`）
- `observation_update.position_improvement_rate`
  - 位置誤差が改善した割合（`delta_pos_err > 0`）
- `observation_update.heading_improvement_rate`
  - 方位誤差が改善した割合（`delta_heading_err > 0`）
- `observation_update.position_degradation_rate`
  - 位置誤差が悪化した割合（`delta_pos_err < 0`）
- `observation_update.heading_degradation_rate`
  - 方位誤差が悪化した割合（`delta_heading_err < 0`）
- `observation_update.position_effective_rate`
  - 位置誤差で非悪化だった割合（`delta_pos_err >= 0`）
- `observation_update.heading_effective_rate`
  - 方位誤差で非悪化だった割合（`delta_heading_err >= 0`）

### 3.2 更新間隔（遠距離/近距離）別の性能

- `observation_update.distance_bucket_threshold`
  - `dist_goal` の中央値（近距離/遠距離の境界）
- `observation_update.near`, `observation_update.far`
  - 各距離帯での件数、平均改善量、改善率、悪化率

## 4. 実装依存性に関する方針

- 解析はログ列（真値姿勢・推定姿勢）だけを利用します。
- EKF 固有の内部状態やパラメータには依存しません。
- `ILocalizer` 実装を差し替えても、同一ログスキーマなら解析可能です。
