# RANSAC Observation Model デバッグガイド

## 1. 目的

このドキュメントは、`ransac_line_association` 観測モデルの不具合調査を迅速化するための実践ガイドです。
今回の失敗事例（`localization_test_app` で即座に破綻）を踏まえ、**どこに着目すべきか**を整理しています。

---

## 2. 今回の分析概要（何が起きたか）

### 実行条件

- テストシナリオ: `test/localization/configs/localization.toml`
- ローカライザ設定: `test/localization/configs/localization/ekf_ransac.toml`
- 観測モデル: `ransac_line_association`

### 観測された症状

- `./build/localization_test_app --render` で衝突終了
- `metrics.json` で失敗（`result: failure`）
- 観測更新は18回発生しているが、更新品質が不安定

### 時系列の重要ポイント

- **step 19（初回LiDAR更新）で破綻開始**
  - 位置誤差: `0.0687 -> 10.3748`（大ジャンプ）
  - heading誤差も急増
- 以後も一部更新で回復するが、step 299 でも再破綻

ログ根拠:

- `test/localization/logs/localization_test.log`
- `test/localization/logs/analysis/localization_test/metrics.json`

---

## 3. 根本原因（今回の時点）

今回の実装とログ挙動を突き合わせた結果、主因は次の3点です。

### A. Stage2対応付けの制約不足（重複マッチ許容）

`evaluatePoseHypothesis` は「各観測線が最良の地図線を選ぶ」方式で、
**複数観測線が同一地図線へ重複対応**できます。

これにより、誤姿勢仮説でもインライア数が水増しされ、
RANSACで誤仮説が勝ちやすくなります。

対象実装:

- `src/features/localization/observation_model/ransac_line_association_model.cpp`
  - `evaluatePoseHypothesis`
  - `matches.emplace_back(...)`

### B. 仮説受理条件が弱い

設定が `min_pose_inliers = 3` のため、
初期の偶然一致だけで仮説が受理される余地があります。

対象設定:

- `test/localization/configs/localization/ekf_ransac.toml`

### C. EKFゲート無効（防波堤がない）

`use_ekf_gate = false` なので、誤対応の大きな更新がそのままEKFに入ります。

対象設定/実装:

- `test/localization/configs/localization/ekf_ransac.toml`
- `src/features/localization/observation_model/ransac_line_association_model.cpp`
  - `if (config_.useEkfGate && !gateLineObservation(...))`

補足:

- `score` はゲート通過率由来で、ゲート無効時は高止まりしやすく、品質指標として弱いです。

---

## 4. デバッグ時に最優先で着目するポイント

## 4.1 Stage1（点群→線分）

見るべき値:

- 抽出線分数
- 各線分の `supportPointCount`
- 各線分の `mse`
- 抽出線分長（短すぎないか）

確認観点:

- 初回更新時に「壁らしい長線分」が取れているか
- ノイズ線分が上位に来ていないか
- `min_inlier_points` が環境密度に対して適切か

## 4.2 Stage2（線分→地図対応付け）

見るべき値:

- 仮説ごとの `matches.size()`
- `mapIndex` の重複率
- 各マッチの `angleDiff / rhoDiff`
- 最終採択仮説と予測姿勢との差（`dx, dy, dtheta`）

確認観点:

- **同一地図線への重複対応が多発していないか**
- 採択仮説の位置/姿勢が予測から突然飛んでいないか

## 4.3 EKF更新入力

見るべき値:

- `obs_update_applied` の直前直後の誤差差分
- 更新ごとの `pre_update_pos_err / post_update_pos_err`
- 更新直後に共分散が不自然に縮んでいないか

確認観点:

- 1回の更新で誤差が数m単位で悪化するケースを検知できるか
- `score` と実品質が乖離していないか

---

## 5. まず実施すべき安全側の切り分け

1. `use_ekf_gate = true` で再実行し、破綻頻度を確認
2. `min_pose_inliers` を増やす（例: 3 -> 5 or 6）
3. Stage2で地図線の重複マッチを禁止（1対1対応）
4. 採択仮説に対して「予測姿勢からの最大許容差」を導入
5. 更新適用前にサニティチェック（大ジャンプなら更新棄却）

この5項目で、今回のような初回大破綻は大きく抑制できる可能性が高いです。

---

## 6. 再現と確認の最小手順

### 実行

1. `./build/localization_test_app --render`
2. `uv run test/localization/analyze_localization_log.py`

### 確認ファイル

- `test/localization/logs/localization_test.log`
- `test/localization/logs/analysis/localization_test/metrics.json`

### まず見る指標

- `observation_update.count`
- `final_position_error`
- `result`
- 更新イベントごとの `delta_pos_err`

---

## 7. 今後の改善方針（実装優先順位）

1. Stage2対応を1対1マッチ制約へ変更（重複禁止）
2. 仮説採択時に「予測姿勢整合チェック」を追加
3. ゲートON時の`score`定義を見直し（品質指標を実態寄りに）
4. 更新棄却時の理由ログ（角度超過/距離超過/整合失敗）を出力

この順で進めると、破綻を止めつつ原因追跡の観測性も上がります。
