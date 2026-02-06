# **モダンC++20 自律走行システム 詳細開発計画書**

## **0\. プロジェクトコンセプト**

本プロジェクトは、自律走行のコアアルゴリズム（経路生成・追従・自己位置推定）を、**C++20の最新機能と「Restricted OOP（制限されたオブジェクト指向）」設計**を用いて実装する技術実証（PoC）である。

従来の「動的な状態管理」を廃し、\*\*「不変性（Immutability）」**と**「型安全性（Type Safety）」\*\*を徹底することで、メモリ安全性が保証され、かつ認知的負荷の低い堅牢なソフトウェアアーキテクチャを確立することを主眼とする。

## ---

**1\. 技術スタック (Modern C++ Architecture)**

* **Language**: C++20 (Ubuntu 22.04 / GCC 11+ or Clang 14+)  
* **Build System**: CMake 3.20+  
* **Key Libraries**:  
  * fmt: 型安全かつ高速なフォーマット出力（iostream禁止）  
  * tl-expected: RustライクなResult型によるエラーハンドリング（例外禁止）  
  * matplotlib-cpp: シミュレーション結果の可視化  
* **Design Pattern**:  
  * **Feature-First Packaging**: 機能ごとにヘッダ・実装を凝集させる。  
  * **Restricted OOP**: struct (Data) と class (Service) の完全分離。  
  * **Dependency Injection (DI)**: コンストラクタによる依存性の注入。

## ---

**2\. システムアーキテクチャ**

システム全体を「状態（Data）」と「計算（Service）」に二分し、データフローを一方向へ整流化する。

### **2.1 データモデル (Immutable Data)**

すべてのデータ構造体は struct で定義し、メンバは原則 const とする。状態更新は「変数の書き換え」ではなく「新しい状態の生成」として表現する。

### **2.2 サービスモデル (Stateless Services)**

ロジックはステートレスなサービスクラスに集約する。

* **Input**: const 参照（Data Object）  
* **Output**: tl::expected\<Result, Error\>  
* **Error Handling**: 戻り値チェックを \[\[nodiscard\]\] で強制。

## ---

**3\. ディレクトリ構成 (Feature Cohesion)**

機能単位の凝集度を高めるため、関連ファイルを同一ディレクトリに配置する「Package by Feature」を採用する。

Plaintext

/autonomous\_driving\_modern  
├── CMakeLists.txt              \# ルートビルド設定  
├── main.cpp                    \# エントリポイント (DIコンテナ構成とメインループ)  
│  
├── shared/                     \# 全機能共通の定義 (Kernel)  
│   ├── types.hpp               \# Pose, Point, MapData 等のData Object  
│   ├── result.hpp              \# Result\<T\> エイリアス, エラー定義  
│   └── math\_utils.hpp          \# 共通計算ロジック  
│  
└── feature/                    \# 機能モジュール  
    ├── planning/               \# 経路計画機能  
    │   ├── i\_planner.hpp       \# インターフェース  
    │   ├── astar\_planner.hpp   \# 実装ヘッダ  
    │   └── astar\_planner.cpp   \# 実装ロジック (A\*)  
    │  
    ├── control/                \# 制御機能  
    │   ├── i\_controller.hpp  
    │   ├── pure\_pursuit.hpp  
    │   └── pure\_pursuit.cpp    \# 実装ロジック (Pure Pursuit)  
    │  
    ├── localization/           \# 自己位置推定機能  
    │   ├── i\_localizer.hpp  
    │   ├── ransac\_localizer.hpp  
    │   └── ransac\_localizer.cpp \# 実装ロジック (RANSAC)  
    │  
    ├── simulation/             \# 物理・センサモデル機能  
    │   ├── i\_physics.hpp       \# 物理モデル I/F  
    │   ├── unicycle\_model.hpp  \# 実装ヘッダ  
    │   ├── unicycle\_model.cpp  \# 実装ロジック (二輪モデル)  
    │   ├── i\_sensor.hpp        \# センサ I/F  
    │   ├── lidar\_sim.hpp       \# 実装ヘッダ  
    │   └── lidar\_sim.cpp       \# 実装ロジック (Ray Casting)  
    │  
    └── visualization/          \# 可視化機能  
        ├── visualizer.hpp  
        └── visualizer.cpp      \# matplotlibラッパー

## ---

**4\. モジュール詳細設計**

### **4.0 共通データ (shared/types.hpp)**

\+1

C++

namespace ad::types {  
    struct Point { const double x; const double y; };  
    struct Pose { const double x; const double y; const double theta; };  
    struct Twist { const double v; const double w; }; // 線速度, 角速度  
    struct MapData {  
        const int width;  
        const int height;  
        const double resolution;  
        const std::vector\<int8\_t\> grid; // 占有格子  
    };  
    using Path \= std::vector\<Point\>;  
}

### **4.1 シミュレーション機能 (feature/simulation)**

* **Physics Service (UnicycleModel)**:  
  * 役割: 現在の状態と入力から、次のタイムステップの状態を計算する純粋関数。  
  * 入力: CurrentPose, CurrentTwist, CommandTwist  
  * 出力: NextPose, NextTwist (積分計算と物理制約の適用)  
* **Sensor Service (LidarSim)**:  
  * 役割: ロボット位置からマップに対してRay Castingを行い、距離データを生成する。  
  * 技術: std::views::iota を用いてレイごとの計算を並列化・パイプライン化する。

### **4.2 経路生成機能 (feature/planning)**

* **Planner Service (AStarPlanner)**:  
  * 役割: スタートからゴールまでの衝突のない経路を探索する。  
  * 実装: ノード管理に生ポインタを使用せず、インデックスまたは値ベースで管理しメモリリークを防ぐ。

### **4.3 経路追従機能 (feature/control)**

* **Controller Service (PurePursuitController)**:  
  * 役割: 経路と現在位置から、適切な指令値（速度・角速度）を計算する。  
  * 実装: パスデータは std::span\<const Point\> で受け取り、ゼロコピーで参照する。

### **4.4 自己位置推定機能 (feature/localization)**

* **Localizer Service (RansacLocalizer)**:  
  * 役割: ノイズを含むLiDARデータと地図を照合し、位置補正を行う。  
  * 実装: RANSACの反復処理に std::ranges を活用し、宣言的に記述する。

## ---

**5\. 開発・検証ロードマップ**

テスト駆動に近い形で、ボトムアップに実装を進める。

| Step | 機能 | 対象ファイル | 検証内容 (テスト/実行) |
| :---- | :---- | :---- | :---- |
| **1** | **基盤構築** | shared/\*, CMakeLists.txt | コンパイルが通り、基本型が利用可能であること。 |
| **2** | **物理・可視化** | feature/simulation, visualization | **Test**: ロボットが指令値通りに動き、物理制約（速度制限等）を守ること。 **App**: ウィンドウ上にマップとロボットが描画され、動くこと。 |
| **3** | **経路生成** | feature/planning | **Test**: 障害物を回避したパスが計算されること。 **App**: スタートからゴールへの線が描画されること。 |
| **4** | **経路追従** | feature/control | **Test**: パスの曲率に応じて適切な角速度が出力されること。 **App**: ロボットが生成されたパス上を滑らかに移動すること。 |
| **5** | **センサ・推定** | feature/simulation (Lidar), feature/localization | **Test**: Ray Castingで正しい距離が返るか。位置ズレが補正されるか。 **App**: センサ点群の描画と、自己位置推定による補正挙動の確認。 |
| **6** | **統合** | main.cpp | 全モジュールをDIで結合し、自律走行（Start→Goal）が完遂すること。 |

## ---

**6\. 品質保証とツール設定**

コーディング規約を機械的に強制する。

1. **Clang-Format**: 保存時に自動整形を行い、スタイル論争を排除する。  
2. **Clang-Tidy**: modernize-\* フィルタを適用し、古いC++構文（生ポインタ、C形式キャスト等）を警告・エラー化する。  
3. **Sanitizers**: ビルド時に \-fsanitize=address,undefined を付与し、実行時のメモリ違反を検出する。