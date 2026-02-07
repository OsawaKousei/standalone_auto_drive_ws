# planning_visualization_demo クラッシュのバグ修正レポート

## 概要

`planning_visualization_demo` が `renderFrame()` 実行時にセグメンテーションフォルトでクラッシュする問題が発生しました。原因は、`matplotlibcpp` のインタプリタが異なる翻訳単位 (TU) で二重に初期化されることによるプロセス内競合でした。修正により、可視化処理は単一の TU で完結し、クラッシュは解消されました。

## 発生状況

- 現象: `./build/planning_visualization_demo` 実行時に `Rendering frame...` の直後でクラッシュ
- 再現性: 100%
- 対象: `planning_visualization_demo` のみ
- 影響: 画像生成の失敗、デモが完走できない

## 原因

### 1. matplotlibcpp の二重インタプリタ初期化

`matplotlibcpp` はヘッダ実装で、各 TU ごとに内部シングルトンが生成されます。以下の構成が問題でした。

- `visualizer.cpp` で `matplotlibcpp` を使用
- `planning_visualization_demo.cpp` でも `matplotlibcpp` を直接 `#include` し、`save/close` を呼び出し

このため、**1プロセス内に2つの Python インタプリタが初期化**され、Python のグローバル状態と matplotlib の内部状態が競合し、`renderFrame()` 中にセグメンテーションフォルトが発生しました。

### 2. 既存の visualization_demo が動作する理由

`visualization_demo` は `matplotlibcpp` を直接使わず、`Visualizer` 経由でのみ描画します。つまり、**1つの TU (`visualizer.cpp`) に処理が集約**されており、二重初期化が起きません。

## 解決方法

### 方針

`matplotlibcpp` を直接呼ぶのをやめ、可視化の全 API を `Visualizer` に集約して **単一 TU でのインタプリタ利用**に統一しました。

### 実施内容

1. `Visualizer` に画像保存用の API を追加
   - `saveFigure(std::string_view path)` を追加
   - `matplotlibcpp::save` と `matplotlibcpp::close` を `visualizer.cpp` 内で実行

2. `planning_visualization_demo.cpp` から `matplotlibcpp.h` の直接インクルードを削除
   - `Visualizer::saveFigure()` を使用

## 変更点

- 追加: `Visualizer::saveFigure(std::string_view path)`
- 更新: `planning_visualization_demo.cpp` は `matplotlibcpp` に直接依存しない

## 影響範囲

- 可視化系は `Visualizer` 経由に統一され、複数 TU での Python インタプリタ初期化が回避される
- `visualization_demo` の挙動は変化なし
- `planning_visualization_demo` はクラッシュせず `planning_path.png` を生成

## 再発防止策

- `matplotlibcpp` を直接利用するコードは新規追加しない
- 可視化関連の API は `Visualizer` に集約する
- 可視化デモは `Visualizer` の公開 API のみを使用する

## 動作確認

- `./build/planning_visualization_demo` が正常終了し `planning_path.png` を生成
- `./build/visualization_demo` が従来通り動作
