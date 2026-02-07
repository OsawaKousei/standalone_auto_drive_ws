# 可視化リファクタ案

## 目的

マップ描画とロボット／スキャン描画を分離し、責務を明確にする。将来の衝突判定でロボット外形を差し込む際に、マップ描画の流れを組み替えなくても済むようにする。

## 現状

Visualizer の `renderFrame` が次の処理を一度に担っている:

- matplotlib の環境設定
- マップとスキャン入力の検証
- マップ描画
- パス描画
- ロボット姿勢の描画（点＋進行方向）
- スキャン点の描画

この構造では、ロボット描画（外形や衝突オーバーレイ）を拡張しようとすると `renderFrame` が肥大化しやすい。

## 提案方針

`renderFrame` はマップと座標系のベース描画に限定し、姿勢とスキャン描画は専用関数へ移す。呼び出し側で小さな関数を順に呼び、1フレームを組み立てる。
`types::Footprint` は今導入する。

### API 変更案（ヘッダ）

- `renderFrame(map)` はマップ背景と座標系の設定のみ行う。
- `renderPose` は削除し、`renderRobot(pose, footprint)` に置き換える。
- `renderScan(pose, ranges)` は姿勢入力で点計算するシグネチャにする。現時点の実装はモックデータで十分。
- `renderPath(path)` は実描画を行う実装にする。

### API スケッチ（C++ 擬似コード）

```cpp
class Visualizer {
public:
   [[nodiscard]] auto renderFrame(const types::MapData &map) const -> Status;
   [[nodiscard]] auto renderPath(std::span<const types::Point> path) const -> Status;
   [[nodiscard]] auto renderRobot(const types::Pose &pose,
                                                 const types::Footprint &footprint) const -> Status;
   [[nodiscard]] auto renderScan(const types::Pose &pose,
                                                std::span<const double> ranges) const -> Status;
};
```

`types::Footprint` は今導入する。

## 実装ステップ案

1. `renderFrame` からマップ描画のみを抽出し、`renderFrame(map)` にする。
2. 姿勢／進行方向の描画を `renderRobot(pose, footprint)` に移す。
3. スキャン点の描画を `renderScan(pose, ranges)` に移す（内部で姿勢から点計算。現時点はモックで可）。
4. `renderPath(path)` を実描画に切り替える。
5. 呼び出し側を更新してフレームを合成する:
   - `renderFrame(map)`
   - `renderPath(path)`
   - `renderRobot(pose, footprint)`
   - `renderScan(pose, ranges)`
6. 旧 `renderFrame(map, pose, path, ranges)` は残さない。

## 影響ファイル

- [src/features/visualization/visualizer.hpp](../src/features/visualization/visualizer.hpp)
- [src/features/visualization/visualizer.cpp](../src/features/visualization/visualizer.cpp)
- デモ／テストの呼び出し元（例: [test/visualization_demo.cpp](../test/visualization_demo.cpp)）

## 理由

- 責務の分離が明確になる
- ロボット外形や衝突オーバーレイの追加が容易
- テスト粒度を細かくできる
