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
- 点群の線フィット
- 期待観測生成（pose 依存）
- ゲーティング（Mahalanobis 判定）
- 観測ノイズ反映
- `ObservationUpdateInput` への変換
- map 整合性/署名チェック
- （追加推奨）線への投影値・点線距離など幾何ヘルパ

責務外:

- どの候補を採用するかという戦略判断

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
   - 各バケットを util の線フィットへ投入
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
  G --> H[util: fitLine]
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

- 線フィット数式
- 線観測の期待値生成
- Mahalanobis ゲート計算
- EKF 入力行列の組み立て

この分離により、将来 `ransac` 系や別の線分関連付け方式を導入しても、コア差分は「対応戦略」に限定できる。

---

## 8. 重複解消の具体ポイント

### 8.1 util へ寄せる優先候補

- 線法線計算（`alpha -> nx, ny`）
- 点と線モデルの距離計算
- 線方向投影計算
- 線分から `MapLine` 生成（規約統一）

### 8.2 期待効果

- 数式規約の一貫化（rho/alpha 正規化の揺れ防止）
- バグ修正の一箇所化
- 新観測モデル追加時の実装量削減

---

## 9. 段階的移行計画

### Phase 1: util 新設（挙動不変）

- `line_observation_util` を追加
- `localizer_util` 内の観測専用処理を移設
- 既存呼び出しを差し替え

### Phase 2: line_extractor 側の幾何重複吸収

- `buildMapLineFromSegment` 周辺を util ヘルパ利用へ変更
- 数式重複を削減

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
