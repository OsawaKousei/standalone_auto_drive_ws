# localization_test.log スキーマ仕様

対象ログ: `test/localization/logs/localization_test.log`

## 1. ファイル構成

1. コメントヘッダ行（`#` で開始）
2. CSVカラム名行
3. 各ステップのCSVデータ行
4. 終了メタ情報（`# result=...`、必要に応じて `# error=...`）

### 1.1 ヘッダ例

- `# localization_test log`
- `# scenario_config=test/localization/configs/localization.toml`
- `# path_point_count=...`
- `# path=<x:y;x:y;...>`
- `# columns: step,time,...,scan_points_robot`

## 2. CSVカラム定義

カラム順は固定です。

`step,time,dist_goal,true_x,true_y,true_theta,est_x,est_y,est_theta,pos_err,heading_err,pre_update_pos_err,post_update_pos_err,delta_pos_err,pre_update_heading_err,post_update_heading_err,delta_heading_err,score,cov_xx,cov_yy,cov_tt,cmd_v,cmd_vy,cmd_w,odom_df,odom_dl,odom_dtheta,lidar_updated,scan_count,scan_min_angle,scan_angle_inc,scan_max_range,scan_ranges,scan_points_robot`

| カラム名                                             | 型     | 単位       | 説明                                                     |
| ---------------------------------------------------- | ------ | ---------- | -------------------------------------------------------- |
| `step`                                               | int    | -          | シミュレーションステップ番号（0始まり）                  |
| `time`                                               | double | s          | 経過時刻。`step_seconds * (step + 1)`                    |
| `dist_goal`                                          | double | m          | 真値姿勢からゴール位置までの距離                         |
| `true_x`, `true_y`, `true_theta`                     | double | m / rad    | シミュレーター真値姿勢                                   |
| `est_x`, `est_y`, `est_theta`                        | double | m / rad    | `ILocalizer::estimate()` が返した推定姿勢                |
| `pos_err`                                            | double | m          | 真値と推定の位置誤差                                     |
| `heading_err`                                        | double | rad        | 真値と推定の方位誤差（正規化後の絶対値）                 |
| `pre_update_pos_err`                                 | double | m          | 観測更新直前（predict 後）の位置誤差                     |
| `post_update_pos_err`                                | double | m          | 観測更新直後の位置誤差                                   |
| `delta_pos_err`                                      | double | m          | 位置誤差改善量 `pre - post`（正なら改善）                |
| `pre_update_heading_err`                             | double | rad        | 観測更新直前（predict 後）の方位誤差                     |
| `post_update_heading_err`                            | double | rad        | 観測更新直後の方位誤差                                   |
| `delta_heading_err`                                  | double | rad        | 方位誤差改善量 `pre - post`（正なら改善）                |
| `score`                                              | double | -          | `ILocalizer::estimate()` の `score`                      |
| `cov_xx`, `cov_yy`, `cov_tt`                         | double | -          | 推定共分散対角成分                                       |
| `cmd_v`, `cmd_vy`, `cmd_w`                           | double | m/s, rad/s | 制御入力                                                 |
| `odom_df`, `odom_dl`, `odom_dtheta`                  | double | m / rad    | `predictOdometry` 入力差分                               |
| `lidar_updated`                                      | int    | -          | 当該ステップで `update(scan, map)` 実行時 `1`            |
| `scan_count`                                         | int    | -          | スキャン点数（`ranges.size()`）                          |
| `scan_min_angle`, `scan_angle_inc`, `scan_max_range` | double | rad / m    | `LidarScan` のメタ情報                                   |
| `scan_ranges`                                        | string | m          | `LidarScan.ranges` を `;` 区切りで直列化                 |
| `scan_points_robot`                                  | string | m          | スキャン点群をロボット座標系 `(x:y)` で `;` 区切り直列化 |

## 3. 点群保持ポリシー

- `scan_ranges` はシミュレーター出力 (`LidarScan`) の順序・値をそのまま保持します。
- `scan_points_robot` は `ranges/min_angle/angle_increment` から復元したロボット座標系点群です。
- 点群はワールド座標へ変換せず、センサー出力形状を保持します。

## 4. 観測更新評価ポリシー

- 観測更新評価は `lidar_updated=1` の行のみ対象とします。
- `delta_pos_err` と `delta_heading_err` は、それぞれ更新による改善量です。
  - `delta > 0`: 改善
  - `delta < 0`: 悪化
