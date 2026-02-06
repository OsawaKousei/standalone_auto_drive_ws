# **モダン C++コーディング規約 (C++20 / Ubuntu 22.04)**

## **1\. 基本原則 (Core Principles)**

- **Simple is Best:** 書くのが短いコードではなく、読む際の認知的負荷が最小のコードを最良とする
- **Immutable by Default:** データは不変であるべき。変数は原則 const とし、変更が必要な場合は「既存の変更（Mutation）」ではなく「新しい値の生成（Creation）」を選択する
- **Restricted OOP:** クラスの役割を「データの定義」と「振る舞いの定義」に厳格に分離する
- **Resource is Lifecycle (RAII):** すべてのリソース寿命をスコープに紐付け、生のリソース管理を禁止する

## **2\. アーキテクチャ：制限されたオブジェクト指向 (Restricted OOP)**

C++のクラスを以下の 2 種類に厳格に分類し、中間的な「データもロジックも持つステートフルなクラス」を禁止する。

### **2.1 Data Object (Aggregate Struct)**

- **役割:** データの構造定義
- **実装:** struct を使用する
- **ルール:** メンバ変数は原則 const。ロジックを持たず、継承も禁止

### **2.2 Service/Resource Object (Operation)**

- **役割:** ビジネスロジックの実行、またはリソース（ファイル、通信等）の保持
- **実装:** class を使用する
- **ルール:**
  - **Stateless:** インスタンス化後の内部状態変更を禁止。依存関係はコンストラクタで受け取る（DI）
  - **Composition over Inheritance:** 実装継承を禁止。多態性が必要な場合はインターフェース（純粋仮想関数のみの抽象クラス）の継承のみ許可

## **3\. リソース管理とスマートポインタ (Ownership Policy)**

生ポインタの所有・管理を完全に排除する。

- **new / delete の禁止:** 必ず std::make_unique または std::make_shared を使用する
- **std::unique_ptr をデフォルトとする:** 単一の所有権を基本とし、リソースの寿命を一箇所で管理する
- **std::shared_ptr の限定使用:** 非同期処理やキャッシュなど、寿命の共有が避けられない場合のみ使用する
- **生ポインタ (T) の完全排除:**「所有権を持たない参照」であっても、生ポインタの使用を禁止する
  - 必ず存在する参照には **T&** を使用する
  - 存在しない可能性がある（null を許容する）参照には **std::optional\<std::reference_wrapper\<T\>\>** を使用する
  - スマートポインタから .get() で生ポインタを取り出す操作も原則禁止する

## **4\. データ構造と操作 (Data Structures)**

データの集合は、不変性を保ちつつ一貫した方法で扱う

- **std::vector への統一:** 固定長・可変長にかかわらず、配列シンタックスは **const std::vector\<T\>** に統一する。std::array や C スタイル配列は使用しない
- **IIFE (即時実行ラムダ) による生成:**
  - ソートや複雑なフィルタリングが必要な場合、IIFE 内部で一時的に非 const なコピーを作成・加工し、最終結果を const として返す
  - 例: const auto sorted \= \[&\]{ auto v \= raw; std::ranges::sort(v); return v; }();
- **std::span の活用:** 連続するメモリ領域を関数に渡す際は、所有権を持たない参照として std::span\<const T\> を使用する
- **std::string_view の使用:** 文字列の読み取り専用アクセスには std::string_view を使用する

## **5\. 制御フローと構文 (Modern Syntax)**

- **宣言的記述 (C++20 Ranges):** 複雑なデータ変換には std::ranges を使用し、パイプライン演算子で記述する
- **手続き的ループの制限:** インデックス管理を伴う for 文を禁止。範囲ベース for または std::ranges を使用する
- **出力の統一:** std::cout や printf を禁止し、すべて **fmt::print** を使用する
- **Result パターンによるエラー処理:** 予測可能なエラーには例外を使用せず、戻り値に tl::expected\<T, E\> を付与する

## **6\. 命名規則とファイル構造**

| 対象          | 命名規則            | 例                         |
| :------------ | :------------------ | :------------------------- |
| クラス/構造体 | PascalCase          | UserProfile, UserService   |
| 関数/変数     | camelCase           | getUser(), userData        |
| 定数          | kPascalCase / UPPER | kMaxRetry, DEFAULT_TIMEOUT |
| ファイル名    | snake_case          | user_repository.hpp        |

- **Colocation:** 関連する Data Object と Service Object は同じディレクトリに配置する。

## **7\. 強制と自動化 (Tooling)**

- **Clang-Format:** コードスタイルの自動整形。
- **Clang-Tidy:** 静的解析による古い構文の検知と modernize-\* の適用。生ポインタの使用をエラーとして検知する
- **Sanitizers:** アドレスエラーや未定義動作をランタイムで検知
