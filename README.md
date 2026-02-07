# ワークスペース概要

自律走行アルゴリズム PoC のための C++20 プロジェクトです。

## 前提条件

- ホストに Docker が導入済み
- VS Code の DevContainer 機能が利用可能
- 実行にはマップファイル（YAML/PGM）が必要

## 使い始める手順（最初に必ず実施）

1. ホスト側で初回ビルドを実行して `compile_commands.json` と依存関係を生成する。
   - `./scripts/build.sh`
2. ビルド完了後に DevContainer を開く。IntelliSense は生成された compile commands を前提とするため、先にビルドが必要。
3. 以降の編集・開発は DevContainer 内で行い、ビルドが必要になったらホストで `./scripts/build.sh` を再実行する。

## よく使うコマンド

- ビルド（ホスト）: `./scripts/build.sh`
- 実行（コンテナ）:
  - `./build/auto_drive_app`
  - `./build/auto_drive_log_replay`

## マップの準備（必須）

実行には **マップファイル（YAML/PGM）が必須** です。既定のマップは `tools/map.yaml` と `tools/map.pgm` で、`tools/map_editor.py` を使って作成・更新できます。

## 自律走行スタックの実装概要

本ワークスペースでは、計画・制御・自己位置推定・センサ・物理・可視化を分離した構成で自律走行スタックを実装しています。主要モジュールは以下に配置されています。

- 計画: グリッドベースの経路探索（A\* / Dijkstra）と衝突判定を実装（[src/features/planning](src/features/planning)）
- 制御: Pure Pursuit による追従制御（[src/features/control](src/features/control)）
- 自己位置推定: EKF による推定とユーティリティ（[src/features/localization](src/features/localization)）
- シミュレーション: 二輪モデル、Lidar 生成、衝突チェック（[src/features/simulation](src/features/simulation)）
- 可視化: matplotlib-cpp を使ったフレーム描画（[src/features/visualization](src/features/visualization)）
- 共有データ型と入出力: マップ読み込み、幾何ユーティリティ（[src/shared](src/shared)）

## 主要アプリ

- [src/auto_drive_app.cpp](src/auto_drive_app.cpp): 計画 → 自己位置推定 → 制御 → 物理更新 → 可視化を 1 ループで回す統合デモです。マップ読込、A\* で経路生成、EKF 推定の予測/更新、Pure Pursuit による指令生成、Unicycle モデルで状態更新、Lidar シミュレーションと衝突チェックを行い、フレーム描画とログ出力を行います。ログは [logs/localization_control_lidar_demo.log](logs/localization_control_lidar_demo.log) に、最終図は [localization_control_lidar_path.png](localization_control_lidar_path.png) に保存されます。
- [src/auto_drive_log_replay.cpp](src/auto_drive_log_replay.cpp): 上記ログを読み込み、真値/推定軌跡、計画経路、スキャン点を再描画するリプレイツールです。ログとマップを指定して可視化し、最終図を [localization_control_lidar_log_replay.png](localization_control_lidar_log_replay.png) に保存します。

## ディレクトリ構成

- `src/` — エントリポイントと機能モジュール（shared/types, result など）
- `test/` — デモ/テストのエントリポイント
- `docs/` — ビルド・コーディング・テストのガイドライン
- `scripts/` — 自動化スクリプト（主要エントリポイント: `build.sh`）
- `tools/` — 解析/補助ツール（例: マップ編集）
- `build/` — ホスト側コンテナビルドの生成物出力先

## 開発メモ

- ビルドは使い捨てコンテナで実行し、成果物はホストの `build/` に出力されます。
- コーディングは DevContainer 内で行い、ホスト環境は最小限に保ちます。
- 可視化は Python/NumPy に依存します（matplotlib-cpp 経由）。
- マップは `tools/map_editor.py` を使って作成できます（出力は `tools/map.yaml` / `tools/map.pgm`）。
- 詳細は [docs/BuildStrategy.md](docs/BuildStrategy.md) と [docs/CodingGuideline.md](docs/CodingGuideline.md) を参照してください。
