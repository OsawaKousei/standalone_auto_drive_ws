# observation_model リファクタリング提案（包括版）

## 1. 対象と目的

対象: `src/features/localization/observation_model`

- `simple_line_association_model.*`
- `ransac_line_association_model.*`
- `line_extractor.*`
- `ransac_core.*`

目的:

1. 同一責務の重複実装を統合し、保守コストを削減する
2. 設定値の意味と実際の挙動の乖離を解消する
3. 推論ループ内の副作用（ログ出力など）を分離し、処理を純化する
4. テスト可能性（ユニット分割）を高め、観測モデルの差分評価を安定化する

---

## 2. 現状の詳細診断

## 2.1 責務分離の崩れ（単一ファイル集中）

`ransac_line_association_model.cpp` に以下が同居しており、関心分離が崩れています。

- Lidar点群生成（`collectScanPoints`）
- スキャン線分抽出（`extractLinesFromPointsRansac`）
- 幾何生成（`buildMapLineFromInliers`）
- 候補生成（`buildCandidatePairs`）
- 観測構築とゲーティング（`buildObservations`）
- デバッグCSV出力（`appendUpdateDebugCsv`）

結果:

- 単体テスト粒度が粗く、失敗原因の切り分けが難しい
- リファクタ時の波及範囲が過大
- シンプル版との共通化が進まない

## 2.2 幾何ロジックの重複

以下が重複しています。

- 線分→`MapLine`構築
  - `line_extractor.cpp::buildMapLine`
  - `ransac_line_association_model.cpp::buildMapLineFromInliers`
- ローカル線→地図座標変換
  - `ransac_line_association_model.cpp::transformLocalLineToMap`
  - `ransac_core.cpp::transformScanLineToMap`
- 観測ゲート処理（候補→`LineObservation`→gate）
  - simple と ransac でほぼ同型

結果:

- 同一バグ修正が複数箇所に必要
- 数値閾値や正規化の挙動差が混入しやすい

## 2.3 設定定義の不整合（重要）

### (A) 定義と実行時挙動の乖離

`RansacLineAssociationModel` では設定を受けつつ、実行時に強いクランプ・再計算を行っています。

- `minInlierRatio` を `0.002 ~ 0.08` に再クランプ
- `minInliers` を毎フレーム `remainingPoints` 依存で再設定
- ゲート閾値に下限固定（例: `kCandidateAngleGateMin`）

これにより、設定ファイル上の `ransac.min_inlier_ratio`（例: 0.29）がそのまま効かないケースがあります。

### (B) パラメータの二重管理

- `SimpleLineAssociationModelConfig.mapLineExtraction`
- `RansacLineExtractionConfig.maxLines/minSegmentLength`
- `RansacLineAssociationModelConfig.baseObservation.mapLineExtraction`

同じ意味の値が複数構造体を経由して複製され、追跡性が低い状態です。

### (C) `const` フィールド多用

設定構造体のメンバが `const` で宣言され、値オブジェクトとしての柔軟性（代入・後段調整・段階的構築）が落ちています。

- `MapLineExtractionConfig`
- `RansacLineExtractionConfig`
- `SimpleLineAssociationModelConfig`
- `RansacLineAssociationModelConfig`
- `RansacConfig` など

## 2.4 本番パスの副作用

`buildUpdateInput` 内で毎フレーム CSV 追記を実施。

- ファイルI/Oと mutex が推定処理に混在
- 実行環境依存（書き込み権限、パス存在）
- パフォーマンス観測がI/Oに汚染される

## 2.5 アルゴリズム効率の改善余地

- `sampleTwoDistinct` が重複サンプルを即失敗扱い（再抽選しない）
- 一意制約チェックに `std::find` 線形探索を多用
- 候補ペア数増加時の計算量上昇に対する最適化余地あり

---

## 3. 提案アーキテクチャ（統合方針）

## 3.1 層構造の再編

### A. Geometry/Math 共通層

新規: `observation_model/line_geometry.hpp(.cpp)`

提供関数:

- `toMapLineFromSegment(start, end)`
- `toMapLineFromModelAndSpan(model, minProj, maxProj)`
- `transformLineModelLocalToMap(line, pose)`
- `projectionRangeOnLine(segment, mapLine)`

目的: 重複実装の一本化と数値規約の統一。

### B. Feature Extraction 層

新規:

- `map_line_provider.hpp(.cpp)`（既存 `line_extractor` を内包）
- `scan_line_extractor.hpp(.cpp)`（RANSAC線抽出を移設）

I/F:

- `IMapLineProvider::extract(map)`
- `IScanLineExtractor::extract(scan, predictedPose, config)`

### C. Association 層

新規: `line_association_engine.hpp(.cpp)`

責務:

- 候補生成（角度・距離・投影重なり）
- RANSACペア評価（`ransac_core` 呼び出し）
- 一意対応選択

### D. Observation Construction 層

新規: `line_observation_builder.hpp(.cpp)`

責務:

- `LinePairMatch` + `scanLines/mapLines` から `LineObservation` 生成
- ノイズ付与
- gate判定
- `ObservationUpdateInput` 変換

### E. Diagnostics 層（副作用分離）

新規:

- `observation_diagnostics_sink.hpp`
- `NullDiagnosticsSink`（既定）
- `CsvDiagnosticsSink`（デバッグ時のみ注入）

`buildUpdateInput` は純粋に近い処理へ寄せ、I/Oは外部注入へ移す。

## 3.2 モデル実装の位置付け

- `SimpleLineAssociationModel`: バケット方式 + 共通 ObservationBuilder 利用
- `RansacLineAssociationModel`: ScanLineExtractor + AssociationEngine + 共通 ObservationBuilder 利用

差分は「候補集合の作り方」に限定し、最終観測生成の実装差を解消。

---

## 4. 設定モデルの再設計（不適切定義の改善）

## 4.1 推奨Config再編

```cpp
struct ObservationCommonConfig {
  double measurementNoiseRange;
  double measurementNoiseAngle;
  double gateThreshold;
  std::size_t minObservations;
};

struct AssociationConfig {
  double maxAssociationDistance;
  double segmentMargin;
  double candidateAngleGate;
  double candidateRhoGate;
};

struct MapLineConfig {
  int maxLines;
  double minSegmentLength;
};

struct ScanLineRansacConfig {
  int maxLines;
  int maxIterations;
  double inlierDistance;
  std::size_t minInliers;
  double minInlierRatio;
  double minInlierSpan;
  double mergeRho;
  double mergeTheta;
};

struct PairRansacConfig {
  int maxIterations;
  std::size_t minInliers;
  double minInlierRatio;
  double inlierAngleThreshold;
  double inlierRhoThreshold;
  double maxTranslationDelta;
  double maxRotationDelta;
  double maxMeanResidual;
};
```

ポイント:

- `const` メンバをやめ、値オブジェクトとして自然に扱う
- 「設定値」と「実行時導出値」を分離
  - 設定: Config
  - 導出: RuntimeParams（フレームごとに計算）
- 暗黙クランプを最小化し、必要な下限はパース時バリデーションで明示

## 4.2 「設定値が効かない」問題の解消

方針:

1. 既定値の適用はパーサのみ
2. 実行中に上書きクランプする場合は、
   - 別名フィールド（例: `adaptiveMinInlierRatio`）として明示
   - diagnostics に「effective値」を記録

---

## 5. 段階的リファクタリング計画

## Phase 0（安全網）

- 現行挙動を固定化するリグレッションテストを追加
- 評価アプリの比較指標をCIで保存（success率、平均NIS、runtime）

## Phase 1（純粋関数の抽出）

- Geometry関数を `line_geometry` へ移設
- simple/ransac の重複箇所を置換
- 挙動差分ゼロを確認

## Phase 2（観測生成の共通化）

- `line_observation_builder` 導入
- simple/ransac 両方で利用
- gate通過率と残差統計の同等性を確認

## Phase 3（RANSACパイプライン分離）

- `extractLinesFromPointsRansac` を `scan_line_extractor` へ分離
- `buildCandidatePairs` を `line_association_engine` へ移設

## Phase 4（診断I/O分離）

- CSV書き出しを `DiagnosticsSink` 化
- 本番デフォルトは `NullDiagnosticsSink`

## Phase 5（Config再設計）

- 新Config導入
- 旧Config互換レイヤーを `localizer_factory` で吸収
- 旧キー警告を出して段階移行

---

## 6. テスト戦略（追加提案）

## 6.1 ユニットテスト

1. Geometry

- 座標変換の可逆性（近似）
- 直線正規化（rho符号規約）

2. ScanLineExtractor

- 人工点群で本数・最小長・重複抑制を検証

3. AssociationEngine

- 候補ゲート（角度/距離/投影）の境界条件

4. ObservationBuilder

- 同入力に対し simple/ransac で同一ノイズ付与・gate判定

## 6.2 統合テスト

- 既存 `observation_model_debug_app` のA/B比較を回帰テスト化
- 判定基準例:
  - success率: 既存比 -2%以内
  - 平均NIS: 既存比 +10%以内
  - runtime: +15%以内

---

## 7. 優先度付き改善バックログ

P0（最優先）

- 幾何関数の重複統合
- 設定値と実効値の乖離可視化
- デバッグI/O分離

P1

- 候補生成・観測生成の共通化
- Config二重構造の解消

P2

- サンプリング効率改善（重複時再抽選）
- 線形探索ベースの一意制約を集合ベースへ置換

---

## 8. 期待効果

- コード量: observation_model内の重複ロジックを20〜35%削減
- 障害解析: 失敗点がモジュール単位で特定可能
- 設定運用: 「値を変えたのに効かない」事象の減少
- 性能: I/O分離により推定処理のジッタ低減

---

## 9. 実装開始時の具体タスク（最初の1スプリント）

1. `line_geometry` 新設 + 既存2箇所置換
2. `appendUpdateDebugCsv` を `DiagnosticsSink` 化
3. `extractLinesFromPointsRansac` を別ファイルへ移設
4. `buildObservations` を共通化し simple/ransac へ適用
5. 回帰指標を `observation_model_debug_app` で採取し差分確認

以上。
