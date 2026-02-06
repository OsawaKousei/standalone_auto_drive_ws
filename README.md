# ワークスペース概要

コンテナ化されたビルド環境と DevContainer を組み合わせた C++20 プロジェクトです。fmt による出力と tl::expected による結果ハンドリングを使い、挨拶機能を示す小さなアプリケーションを含みます。

## 前提条件

- ホストに Docker が導入済み（ビルドスクリプトで使用）
- VS Code の DevContainer 機能が利用可能（コーディングと IntelliSense 用）

## 使い始める手順（最初に必ず実施）

1. ホスト側で初回ビルドを実行して `compile_commands.json` と依存関係を生成する。
   - `./scripts/build.sh`
2. ビルド完了後に DevContainer を開く。IntelliSense は生成された compile commands を前提とするため、先にビルドが必要。
3. 以降の編集・開発は DevContainer 内で行い、ビルドが必要になったらホストで `./scripts/build.sh` を再実行する。

## よく使うコマンド

- ビルド（ホスト）: `./scripts/build.sh`
- 実行（ホスト）: `./build/app`

## ディレクトリ構成

- `src/` — エントリポイントと機能（例: greeting サービス）
- `docs/` — ビルド・コーディング・テストのガイドライン
- `scripts/` — 自動化スクリプト（主要エントリポイント: `build.sh`）
- `build/` — ホスト側コンテナビルドの生成物出力先

## 開発メモ

- ビルドは使い捨てコンテナで実行し、成果物はホストの `build/` に出力されます。
- コーディングは DevContainer 内で行い、ホスト環境は最小限に保ちます。
- 詳細は [docs/BuildStrategy.md](docs/BuildStrategy.md) と [docs/CodingGuideline.md](docs/CodingGuideline.md) を参照してください。
