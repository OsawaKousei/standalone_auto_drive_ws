# control_test.log スキーマ仕様

対象ログ: `test/control/logs/control_test.log`

## 1. ファイル構成

本ログは以下の順で構成されます。

1. コメントヘッダ行（`#` で開始）
2. CSVカラム名行
3. 各ステップのCSVデータ行
4. 終了メタ情報（`# result=...`、必要に応じて `# error=...`）

### 1.1 ヘッダ例

- `# control_test log`
- `# scenario_config=test/control/configs/control.toml`
- `# path_point_count=411`
- `# columns: step,time,...,odom_dtheta`

## 2. CSVカラム定義

カラム順は固定です。

`step,time,dist_goal,cross_track,nearest_idx,true_x,true_y,true_theta,odom_x,odom_y,odom_theta,track_err,track_heading_err,cmd_v,cmd_vy,cmd_w,odom_df,odom_dl,odom_dtheta`

| カラム名            | 型     | 単位  | 説明                                                            |
| ------------------- | ------ | ----- | --------------------------------------------------------------- |
| `step`              | int    | -     | シミュレーションステップ番号（0始まり）                         |
| `time`              | double | s     | 経過時刻。`step_seconds * (step + 1)`                           |
| `dist_goal`         | double | m     | 真値姿勢 (`true_pose`) からゴール位置までの距離                 |
| `cross_track`       | double | m     | 真値姿勢から経路点列への最短距離                                |
| `nearest_idx`       | int    | -     | オドメトリ姿勢 (`odometry_pose`) から最も近い経路点インデックス |
| `true_x`            | double | m     | 真値姿勢の X                                                    |
| `true_y`            | double | m     | 真値姿勢の Y                                                    |
| `true_theta`        | double | rad   | 真値姿勢のヨー角                                                |
| `odom_x`            | double | m     | オドメトリ推定姿勢の X                                          |
| `odom_y`            | double | m     | オドメトリ推定姿勢の Y                                          |
| `odom_theta`        | double | rad   | オドメトリ推定姿勢のヨー角                                      |
| `track_err`         | double | m     | 真値姿勢とオドメトリ姿勢の位置誤差（ユークリッド距離）          |
| `track_heading_err` | double | rad   | 真値姿勢とオドメトリ姿勢の方位誤差（正規化後の絶対値）          |
| `cmd_v`             | double | m/s   | 制御コマンド前進速度                                            |
| `cmd_vy`            | double | m/s   | 制御コマンド横速度                                              |
| `cmd_w`             | double | rad/s | 制御コマンド角速度                                              |
| `odom_df`           | double | m     | オドメトリ差分（前進成分）                                      |
| `odom_dl`           | double | m     | オドメトリ差分（横成分）                                        |
| `odom_dtheta`       | double | rad   | オドメトリ差分（方位成分）                                      |

## 3. 終了ステータス行

シミュレーション終了時に以下のコメント行が追記されます。

- `# result=success` または `# result=failure`
- 失敗時のみ: `# error=<エラー内容>`

## 4. 補足

- 1行の内容が長いため、ターミナル表示では折り返されて見える場合があります。
- ログ自体は通常の1行CSVとして保存されます。
