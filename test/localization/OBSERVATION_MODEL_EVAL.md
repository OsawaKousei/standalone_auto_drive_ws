# observation_model_debug_app 仕様

対象アプリ: `build/observation_model_debug_app`

## 1. 目的

- `IObservationModel::buildUpdateInput` を**同一入力**で比較評価する。
- 制御・経路生成・運動モデルの影響を除外し、観測モデルの性能差だけを計測する。

## 2. 入力

- 既定ログ: `test/localization/logs/localization_test.log`
  - `lidar_updated=1` の行のみ評価対象
- 既定シナリオ: `test/localization/configs/localization.toml`
  - `[map].yaml_path` から地図を読み込む
- 既定モデルA: `test/localization/configs/localization/ekf_hough.toml`
- 既定モデルB: `test/localization/configs/localization/ekf_hough_ransac.toml`

## 3. 実行例

```bash
./build/observation_model_debug_app
```

```bash
./build/observation_model_debug_app \
  --log test/localization/logs/localization_test.log \
  --scenario test/localization/configs/localization.toml \
  --config-a test/localization/configs/localization/ekf_hough.toml \
  --config-b test/localization/configs/localization/ekf_hough_ransac.toml \
  --label-a hough \
  --label-b hough_ransac \
  --out-csv test/localization/logs/observation_model_eval.csv \
  --out-json test/localization/logs/observation_model_eval_metrics.json
```

## 4. 出力

- CSV: `test/localization/logs/observation_model_eval.csv`
  - 1フレーム × 1モデルごとの評価結果
  - 主列: `status`, `score`, `residual_rmse`, `nis`, `measurement_count`, `runtime_ms`
- JSON: `test/localization/logs/observation_model_eval_metrics.json`
  - モデル別集計（成功率、平均残差、平均NIS、処理時間）
  - モデルB - モデルA の差分

## 5. ステータス定義

- `success`: 観測更新入力を生成できた
- `no_update`: 観測不足やゲートで更新が成立しなかった
- `error`: 入力不整合などで処理失敗

## 6. 差分可視化

対象スクリプト: `test/localization/plot_observation_model_eval.py`

```bash
python3 test/localization/plot_observation_model_eval.py
```

既定出力先:

- `test/localization/logs/analysis/observation_model_eval/model_comparison.png`
- `test/localization/logs/analysis/observation_model_eval/comparison_deltas.png`
- `test/localization/logs/analysis/observation_model_eval/summary.md`
