# 線分ベース観測モデル 分割・責務再設計案

## 1. 目的

本ドキュメントは、以下 3 要素の責務を明確化し、将来の線分ベース観測モデル（simple 以外を含む）へ拡張しやすい構造を定義する。

- `simple_line_association_model`
- `line_extractor`
- 新規 `observation_model` 配下 util

併せて、`simple_line_association_model` の処理フローと、モデル固有のコアロジックを明確化する。

---

## 2. 現状整理（要点）

現状では `localizer_util` に以下が混在している。

- EKF と観測の両方で使う処理（例: 角度正規化）
- 線分観測モデル専用の幾何・統計処理

一方、利用実態としては以下が確認できる。

- `normalizeAngle` は EKF 側でも使用
- 線分関連（`MapLine`, `fitLine`, `gateLineObservation`, `buildMeasurementData` など）は観測モデル側でのみ使用
- `line_extractor` と `simple_line_association_model` で線幾何の式レベル重複（法線計算、投影、線距離判定）が存在

結論として、`localizer_util` は「ローカライザ共通最小セット」に絞り、線分観測の汎用処理は `observation_model` 配下 util に集約するのが妥当。

---

## 3. 目標アーキテクチャ

### 3.1 モジュール責務

#### A) simple_line_association_model

責務: 観測モデル戦略（アルゴリズム固有）

- スキャン点をどの map line に割り当てるか（最近傍・区間内・閾値）
- 観測採用条件（`minObservations` など）
- スコア定義（gate 通過率など）

責務外:

- 線分幾何の基本演算
- 線フィット数式実装
- 観測行列化（H, R, residual 構築）

#### B) line_extractor

責務: 地図から線分特徴を生成する前処理

- occupancy grid から境界エッジ抽出
- 区間マージ
- map line 列への変換

責務外:

- センサ観測との対応付け
- EKF 更新用データ生成

#### C) 新規 observation_model util（推奨: `observation_model/line_observation_util.*`）

責務: 線分ベース観測で共有される汎用処理

- 線モデル正規化（rho/alpha 規約）
- 期待観測生成（pose 依存）
- ゲーティング（Mahalanobis 判定）
- 観測ノイズ反映
- `ObservationUpdateInput` への変換
- map 整合性/署名チェック

責務外:

- どの候補を採用するかという戦略判断
- 特定モデル専用の抽出・対応付けアルゴリズム

### 3.2 util 配置の判定原則（厳格版）

原則:

- 「純粋関数かどうか」ではなく、「複数モデルでの再利用が確実かどうか」で判断する
- 現時点で `simple` と「線分検出RANSAC + 対応付けRANSAC（2段）」の双方で共有が見込める処理のみ util に置く
- 片方でしか使わない処理は、たとえ計算処理でも各モデル内に置く

採用条件（すべて満たすこと）:

1. 入出力がモデル戦略に依存しない（閾値意味・探索順・候補選択規則を含まない）
2. `simple` と 2段RANSAC の双方で同一数式・同一意味で利用できる
3. 近い将来に最低2実装で利用される具体シナリオを説明できる

除外条件（いずれかに該当したら util 化しない）:

- 候補生成、対応付け、インライア選別など「戦略の核」に触れる
- しきい値の意味がモデル固有（例: RANSAC反復・インライア率）
- 現状単一実装でしか呼ばれていない

### 3.3 simple と 2段RANSAC を前提にした配置マトリクス

新 util に置く（共有確度が高い）:

- `toLineModel`（rho/alpha 正規化）
- `makeExpectedLine`（地図線分 + pose から期待観測）
- `gateLineObservation`（共通ゲーティング）
- `applyObservationNoiseFromMse`（共通ノイズ反映）
- `buildMeasurementData`（EKF更新入力への変換）
- `mapHasConsistentGrid` / `mapSignatureFromMap` / `signatureMatches`

モデル側に残す（戦略依存）:

- `simple`: scan点のバケット割当、最近傍 line 選択、採否判定フロー
- 2段RANSAC: scan線分抽出のサンプリング戦略、線分マッチングのRANSAC反復、インライア選抜

要再評価（現時点では util 化しない）:

- `fitLine`（2段RANSACで最終線推定に使う可能性はあるが、用途と統計モデルが一致するまで保留）
- 線方向投影や点線距離の低レベルヘルパ（現状は呼び出し統一より可読性低下リスクが高い）

---

## 4. localizer_util に残すもの

`localizer_util` は「localizer 共通」に限定する。

最低限残す候補:

- `normalizeAngle`

必要に応じて将来追加するもの:

- EKF ローカライザ一般に関わる共通処理のみ（観測モデル専用処理は追加しない）

---

## 5. 分割後のファイル構成（提案）

- [src/features/localization/localizer_util.hpp](../src/features/localization/localizer_util.hpp)
  - `normalizeAngle` のみ（または localizer 共通最小セット）
- [src/features/localization/localizer_util.cpp](../src/features/localization/localizer_util.cpp)
- 新規: `src/features/localization/observation_model/line_observation_util.hpp`
- 新規: `src/features/localization/observation_model/line_observation_util.cpp`
- [src/features/localization/observation_model/line_extractor.hpp](../src/features/localization/observation_model/line_extractor.hpp)
- [src/features/localization/observation_model/line_extractor.cpp](../src/features/localization/observation_model/line_extractor.cpp)
- [src/features/localization/observation_model/simple_line_association_model.hpp](../src/features/localization/observation_model/simple_line_association_model.hpp)
- [src/features/localization/observation_model/simple_line_association_model.cpp](../src/features/localization/observation_model/simple_line_association_model.cpp)

---

## 6. simple_line_association_model の処理フロー（分割後）

1. create
   - map の署名を util で作成
   - `line_extractor` で map line を抽出
   - 抽出結果と設定を保持してモデル生成

2. buildUpdateInput
   - map 署名一致を util で検証
   - scan 各点を map 座標へ変換
   - 戦略ロジックで候補 line へバケット分配

- 各バケットをモデル内の線推定処理へ投入（`fitLine` は共有実績が固まるまでモデル側）
- util で期待観測生成・ノイズ設定・ゲート判定
- 観測数が閾値未満なら no update
- util で `ObservationUpdateInput` に変換して返却

### フロー図

```mermaid
flowchart TD
  A[create(map, config)] --> B[line_extractor: map lines]
  A --> C[util: map signature]
  B --> D[SimpleLineAssociationModel instance]
  C --> D

  D --> E[buildUpdateInput(scan, map, pose, P)]
  E --> F[util: signature check]
  F --> G[bucket assignment by strategy]
  G --> H[model: line fitting/estimation]
  H --> I[util: makeExpectedLine]
  I --> J[util: applyObservationNoise]
  J --> K[util: gateLineObservation]
  K --> L{enough observations?}
  L -- no --> M[return nullopt]
  L -- yes --> N[util: buildMeasurementData]
  N --> O[return ObservationUpdateInput]
```

---

## 7. simple_line_association_model のコアロジック定義

本モデルのコアは「候補対応の戦略」にある。

具体的には以下がコア:

- 点をどの map line に割り当てるか
  - 線分投影区間条件
  - 距離閾値条件
  - 複数候補時の最良選択規則
- 観測採否の閾値設計
  - `minObservations`
  - スコア定義（候補数に対する gate 通過率）

逆にコアではない（汎用化対象）:

- 線観測の期待値生成
- Mahalanobis ゲート計算
- EKF 入力行列の組み立て

この分離により、将来 `ransac` 系や別の線分関連付け方式を導入しても、コア差分は「対応戦略」に限定できる。

---

## 8. 重複解消の具体ポイント

### 8.1 util へ寄せる優先候補（厳格版）

- `toLineModel`
- `makeExpectedLine`
- `gateLineObservation`
- `applyObservationNoiseFromMse`
- `buildMeasurementData`
- map 整合性 / 署名チェック

※ 低レベル幾何ヘルパは「2実装以上で同一シグネチャ利用」が確認できるまでモデル内に留める。

### 8.2 期待効果

- 数式規約の一貫化（rho/alpha 正規化の揺れ防止）
- バグ修正の一箇所化
- 新観測モデル追加時の実装量削減

---

## 9. 段階的移行計画

### Phase 1: util 新設（挙動不変）

- `line_observation_util` を追加
- `localizer_util` 内の観測専用処理から、共有確度が高いもののみ移設
- 既存呼び出しを差し替え

### Phase 2: line_extractor 側の幾何重複吸収

- `line_extractor` と `simple` で実際に重複している式を棚卸し
- 2段RANSAC導入時も共有できると判断できたものだけ util へ昇格

### Phase 3: localizer_util 最小化

- `normalizeAngle` 以外の観測専用依存を排除
- include 依存を localizer 方向に閉じる

### Phase 4: 回帰確認

- ビルド・既存テスト・ログ比較（成功率、観測数、スコア）で差分検証

---

## 10. 非目標（今回やらないこと）

- アルゴリズム挙動自体の変更（閾値や対応戦略の再設計）
- 観測スコア定義の刷新
- RANSAC モデルの同時全面改修

---

## 11. 完了条件

- `localizer_util` が localizer 共通最小責務に限定される
- 線分観測汎用処理が `observation_model` 配下 util に集約される
- `simple_line_association_model` は戦略ロジック中心の読みやすい構造になる
- 既存テスト・主要実行パスで回帰なし

---

## 12. 設定スキーマ再検討（改修後の推奨）

### 12.1 再検討が必要な理由

現状のローカライズ設定は、構造上は次のように責務が混在している。

- `ekf.*` に状態遷移ノイズ（localizer責務）と観測ノイズ（observation責務）が同居
- `association.*` に観測モデル固有パラメータが配置

実装分割後（`localizer_util` 最小化 + `observation_model` util 導入）との整合性を高めるため、
「ローカライザ設定」と「観測モデル設定」を分離したスキーマへ寄せるのが望ましい。

### 12.2 現行読み取りキー（実装準拠）

現行実装が読む必須キー:

- `ekf.process_noise_translation`
- `ekf.process_noise_rotation`
- `ekf.measurement_noise_range`
- `ekf.measurement_noise_angle`
- `line_extraction.max_lines`
- `line_extraction.min_segment_length`
- `association.max_association_distance`
- `association.segment_margin`
- `association.gate_threshold`
- `association.min_observations`
- `initial_covariance.xx|yy|tt`
- `observation_model`（未指定時は `simple_line_association`）

### 12.3 推奨スキーマ（v2案）

方針:

- `localizer` と `observation` をトップレベルで分離
- `observation.model` でモデル種別を宣言
- `observation.<model_name>` にモデル固有設定を閉じ込める
- 今後の 2段RANSAC 追加時は `observation.ransac_line_association` を追加するだけにする

例:

```toml
[localizer]
algorithm = "ekf"

[localizer.ekf]
process_noise_translation = 0.05
process_noise_rotation = 0.03

[localizer.initial_covariance]
xx = 0.5
yy = 0.5
tt = 0.2

[observation]
model = "simple_line_association"

[observation.common]
measurement_noise_range = 0.12
measurement_noise_angle = 0.12
gate_threshold = 6.0
min_observations = 3

[observation.map_line_extraction]
max_lines = 40
min_segment_length = 0.8

[observation.simple_line_association]
max_association_distance = 0.3
segment_margin = 0.3
```

### 12.4 simple と 2段RANSAC を見据えた設定境界

`common` に置く（共有見込みが高い）:

- `measurement_noise_range`
- `measurement_noise_angle`
- `gate_threshold`
- `min_observations`
- `map_line_extraction.*`

モデル固有に置く:

- simple: `max_association_distance`, `segment_margin`
- 2段RANSAC: `scan_line_ransac.*`, `matching_ransac.*`（反復回数、inlier条件など）

### 12.5 互換移行戦略

Phase A（互換読み取り）:

- 新スキーマ優先で読み取り
- 未指定なら現行キー（`ekf.*`, `association.*`, `line_extraction.*`）へフォールバック

Phase B（警告）:

- 旧キー使用時に deprecation 警告を出力

Phase C（統一）:

- テスト・サンプル設定を v2 に移行
- 旧キー削除

### 12.6 バリデーション方針

- 値域チェックはパーサで一元化（負値禁止、最小値制約など）
- モデル別の必須キーは `observation.model` ごとに分岐して検証
- 「設定未使用」検出（不要キー警告）を導入し、設定ドリフトを防止
