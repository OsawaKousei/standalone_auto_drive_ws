# analyze_control_log.py 出力仕様

対象スクリプト: `test/control/analyze_control_log.py`

## 1. 入出力

- 入力ログ（既定）: `test/control/logs/control_test.log`
- 出力ディレクトリ（既定）: `test/control/logs/analysis`
- 実行例:
  - `python3 test/control/analyze_control_log.py`
  - `python3 test/control/analyze_control_log.py --trajectory-source odom`

### 1.1 必須ログ情報

ログヘッダに以下が必要です。

- `# scenario_config=...`
- `# path=...`（生成経路の点列 `x:y;x:y;...`）

CSV列は少なくとも以下を必要とします。

- `time`, `cross_track`, `cmd_v`, `cmd_vy`, `cmd_w`
- `true_x`, `true_y`, `true_theta`
- `odom_x`, `odom_y`, `odom_theta`

## 2. 生成プロット

以下の PNG を `test/control/logs/analysis` に出力します。

1. `map_planned_vs_followed.png`
   - map画像の上に、
     - 生成経路（planned, 青）
     - 追従軌跡（followed, 赤）
       を重ねて表示
   - map は `origin="upper"` で描画（上下反転修正済み）

2. `cross_track_timeseries.png`
   - 横偏差 `cross_track` の時系列

3. `heading_error_timeseries.png`
   - 経路接線に対する角度偏差の時系列
   - 定義: 追従姿勢角 − 最近傍経路点の接線角
   - 角度は $[-\pi, \pi)$ に正規化

4. `acceleration_timeseries.png`
   - 追従軌跡から求めた加速度（速度の時間微分）

5. `angular_acceleration_timeseries.png`
   - 追従軌跡から求めた角加速度（角速度の時間微分）

## 3. メトリクス定義

出力ファイル: `test/control/logs/analysis/metrics.json`

### 3.1 基本偏差

- `rmse`
  - 横偏差の RMSE
  - $\sqrt{\frac{1}{N}\sum_{i=1}^{N} e_i^2}$, $e_i = cross\_track_i$

- `max_deviation`
  - 横偏差の最大値

- `heading_error_rmse`
  - 経路接線に対する角度偏差の RMSE

- `heading_error_max`
  - 経路接線に対する角度偏差の絶対値最大

### 3.2 速度・加速度系

`--trajectory-source` で選ばれた追従軌跡（`true` または `odom`）から算出します。

- `mean_speed`
  - 平均速度（位置差分から計算）
- `max_speed`
  - 最大速度（絶対値最大）
- `max_angular_speed`
  - 最大角速度（姿勢角の差分から計算）
- `max_acceleration`
  - 最大加速度（速度の差分）
- `max_angular_acceleration`
  - 最大角加速度（角速度の差分）
- `max_jerk`
  - 最大ジャーク（加速度の差分）

### 3.3 経路形状・振動

- `curvature_rmse`
  - 生成経路と追従軌跡の曲率差 RMSE
  - 両者を正規化弧長で再サンプル（300点）して比較

- `curvature_fitness`
  - 曲率適合度: $\exp(-curvature\_rmse)$
  - 1 に近いほど曲率一致度が高い

- `oscillation_count`
  - `cmd_w` の符号反転回数（しきい値 $|cmd_w| \ge 0.01$）

### 3.4 到達時間

- `goal_reach_time`
  - ログヘッダ `# result=success` の場合: 最終時刻 `time[-1]`
  - 失敗の場合: `-1.0`

## 4. 重要な注意点

- 本スクリプトは matplotlib を `Agg` バックエンドで使用し、GUIを開かずに画像保存します。
- `heading_error_timeseries` は「推定誤差」ではなく「経路接線に対する誤差」です。
- 時間差分 $\Delta t \le 0$ の区間は `NaN` 扱いになり、最大値計算では有限値のみを対象にします。
