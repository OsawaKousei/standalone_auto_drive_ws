# Observation Model 2実装 徹底比較レポート

## 1. 目的

`simple_line_association` と `ransac_line_association` の2実装を、既存のテスト用コードと同一ログ入力で比較し、

- 更新成立率（success/no_update）
- 観測品質（score / residual RMSE / NIS）
- 観測量（measurement count）
- 計算時間（mean / p95 / p99）
  を定量評価する。

## 2. 比較対象

- 実装A: `SimpleLineAssociationModel`（`src/features/localization/observation_model/simple_line_association_model.cpp`）
- 実装B: `RansacLineAssociationModel`（`src/features/localization/observation_model/ransac_line_association_model.cpp`）

## 3. 使用したテストコードと設定

- 評価アプリ: `test/localization/observation_model_test_app.cpp`
- 解析スクリプト: `test/localization/analyze_observation_model_log.py`
- シナリオ: `test/localization/configs/localization.toml`
- Simple設定: `test/localization/configs/localization/ekf_sla.toml`
- RANSAC設定: `test/localization/configs/localization/ekf_ransac.toml`
- 入力ログ: `test/localization/logs/localization_test.log`

## 4. 実行手順（再現用）

```bash
# Simple
./build/observation_model_test_app \
  --scenario test/localization/configs/localization.toml \
  --config localization/ekf_sla.toml \
  --label simple_line_association \
  --out-csv test/localization/logs/analysis/observation_model_compare/simple/observation_model_eval.csv \
  --out-json test/localization/logs/analysis/observation_model_compare/simple/observation_model_eval_metrics.json

# RANSAC
./build/observation_model_test_app \
  --scenario test/localization/configs/localization.toml \
  --config localization/ekf_ransac.toml \
  --label ransac_line_association \
  --out-csv test/localization/logs/analysis/observation_model_compare/ransac/observation_model_eval.csv \
  --out-json test/localization/logs/analysis/observation_model_compare/ransac/observation_model_eval_metrics.json

# 可視化・集計
uv run test/localization/analyze_observation_model_log.py \
  --metrics test/localization/logs/analysis/observation_model_compare/simple/observation_model_eval_metrics.json \
  --csv test/localization/logs/analysis/observation_model_compare/simple/observation_model_eval.csv \
  --out-dir test/localization/logs/analysis/observation_model_compare/simple

uv run test/localization/analyze_observation_model_log.py \
  --metrics test/localization/logs/analysis/observation_model_compare/ransac/observation_model_eval_metrics.json \
  --csv test/localization/logs/analysis/observation_model_compare/ransac/observation_model_eval.csv \
  --out-dir test/localization/logs/analysis/observation_model_compare/ransac
```

## 5. 結果サマリ（99フレーム）

| 指標                                 |         Simple |         RANSAC | 差分解釈                         |
| ------------------------------------ | -------------: | -------------: | -------------------------------- |
| success_rate                         | 1.0000 (99/99) | 0.7071 (70/99) | Simpleが+29フレーム分安定        |
| no_update_rate                       |         0.0000 |         0.2929 | RANSACは約3割で更新不成立        |
| mean_score (success)                 |         0.9698 |         0.9929 | RANSAC成功時はゲート通過率高     |
| mean_residual_rmse (success)         |        0.08053 |        0.05226 | RANSAC成功時の残差は小さい       |
| mean_nis (success)                   |         1.1086 |         1.0745 | 両者ほぼ同等（RANSAC僅差で低い） |
| mean_measurement_count (success)     |         7.6162 |         4.7143 | Simpleの方が観測本数が多い       |
| mean_runtime_ms (all frames)         |        0.05137 |       10.88701 | RANSACは約191倍遅い              |
| p95_runtime_ms (all frames)          |        0.05579 |       16.31544 | RANSACは約234倍遅い              |
| p99_runtime_ms (all frames, CSV算出) |        0.07363 |       13.48618 | RANSACの裾が重い                 |

## 6. 失敗モード分析（RANSAC）

`ransac_diag.log` を解析した結果:

- accepted: 70
- reject=no_consensus: 29
- reject=min_observations: 0
- reject=insufficient_after_gate: 0

**示唆**

- 主失敗要因は「線抽出不足」や「EKFゲート不足」ではなく、**対応付けRANSACでの合意形成失敗**。
- no_update 29件は、ほぼ `stage=associate reject=no_consensus` に一致。

## 7. 実装差分と結果の因果

### 7.1 Simpleの性質

- スキャン点を地図線へ最近傍・距離閾値で直接バケツ分配し、各線で回帰→EKFゲート。
- 処理経路が短く、分岐が少ないため、
  - 成功率が高い（今回100%）
  - 実行時間が非常に短い（~0.05ms/frame）
- 一方で、対応ミスを抑えるための探索的検証が少なく、成功時の残差品質はRANSACより悪化しうる。

### 7.2 RANSACの性質

- Stage1: 点群から複数線抽出（局所PCA条件＋inlier連続性）
- Stage2: 線ペア仮説から姿勢候補生成し、文脈ゲートとinlier数で最良仮説選択
- Stage4相当: EKFゲートで最終採択
- 多段で頑健化されるため、**成功時品質（RMSE, score）が高い**。
- ただし、探索空間が広く分岐も多いので、
  - 実行時間は大幅増
  - 今回設定では約3割で合意形成に失敗し no_update

## 8. 実運用に向けた結論

### 8.1 単一モデルとしての選択

- **リアルタイム性・更新継続性重視**: Simple優位
- **更新成立時の幾何整合品質重視**: RANSAC優位

### 8.2 本データセットに対する推奨

現状パラメータでは、RANSACは品質面で魅力がある一方、

- 更新欠落（29.3%）
- 計算時間（mean 10.9ms, p95 16.3ms）
  が運用上のリスク。

したがって、**現時点のデフォルト運用はSimple推奨**。
RANSACを本採用するなら、まず `stage=associate no_consensus` を削減するパラメータ再チューニングが必要。

## 9. 追加チューニング優先度（RANSAC）

1. `stage2.translation_ransac_iterations`, `line_angle_threshold`, `line_rho_threshold` の見直し（合意形成率改善）
2. `min_pose_inliers` と `segment_margin` の再調整（過剰reject抑制）
3. `stage1.min_inlier_points`, `point_distance_threshold` のバランス調整（抽出線の安定化）
4. 必要なら `use_ekf_gate` を段階導入（ただし今回は主因ではない）

## 10. 生成物

- Simple集計: `test/localization/logs/analysis/observation_model_compare/simple/summary.md`
- RANSAC集計: `test/localization/logs/analysis/observation_model_compare/ransac/summary.md`
- RANSAC診断ログ: `test/localization/logs/analysis/observation_model_compare/ransac/ransac_diag.log`
- 本レポート: `docs/observation_model_2実装_徹底比較レポート.md`
