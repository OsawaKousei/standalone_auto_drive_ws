# analyze_localization_log.py 出力仕様

対象スクリプト: `test/localization/analyze_localization_log.py`

## 1. 入出力

- 入力ログ（既定）: `test/localization/logs/localization_test.log`
- 出力ディレクトリ（既定）: `test/localization/logs/analysis`
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

## 4. 実装依存性に関する方針

- 解析はログ列（真値姿勢・推定姿勢）だけを利用します。
- EKF 固有の内部状態やパラメータには依存しません。
- `ILocalizer` 実装を差し替えても、同一ログスキーマなら解析可能です。
