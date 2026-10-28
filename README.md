# 概要

自律走行アルゴリズムの開発と検証のための、軽量で依存関係の少ない、CPP向けの2Dシミュレーター。
Dockerを活用して構築されており、VScodeのdevcontainerが使用できればそれ以外の環境構築を必要としません。ただし、ホストマシンでビルド成果物を直接実行することも可能になっています。
シミュレーターコア及びサンプルの自律走行スタックはCPPで記述されています。描画は既定でRustのwgpuバックエンドをC ABI経由で使用し、従来のPython（matplotlib）もビルド時に選択できます。
configファイルの編集によって、mapやロボットの形状・使用するアルゴリズムやそのパラメータを編集できるようになっており、実用的な簡易検証環境となることを期待しています。

※このシミュレーターはアルゴリズムの学習・検証を主な目的として作成されており、実機へのデプロイやROSとの統合は想定されておらず、実装に関してもデスクトップアプリケーションとしてのモジュール性と可読性、拡張性に重点が置かれています。

## このシミュレーターで想定する問題設定

このシミュレーターでは静的な２D環境を想定しています。
地図がグリッドマップで与えられ、センサーとして抽象化されたオドメトリセンサーとLidarセンサーを用いることを想定します。
ロボットは１台で、スタートとゴールのみがワールド座標系で与えられ、ゴール到達判定はシミュレーターが行います。
ロボットが壁に衝突した場合は即座に失敗とみなし、シミュレーションを終了します。
経路生成・自己位置推定・経路追従の３つのアルゴリズムが実装されることを想定します。
経路生成はシミュレーション開始時に一度だけ行い、その後は経路追従アルゴリズムに委ねます。
オドメトリセンサーとLidarセンサーが異なる周期で値を返す状況を想定し、自己位置推定はこれら２つのセンサーによる入力を区別して受け取ります。センサーの周期はconfigで設定できます。
ロボットの形状はconfigである程度自由に定義できます。運動モデルに関しては事前に実装されたモデルをconfigで選択します。
オドメトリセンサー・Lidar・ロボットの制御コマンドに対する追従性に対してシンプルなノイズを付加します。この度合いはパラメータによって制御できます。

# 動作環境・バージョン

### 動作要件

- dockerが使用できること

### 推奨要件

- VScodeのdevcontainer拡張機能が使用できること
- 実機またはWSLのUbuntu22.04

※動作確認は推奨要件の環境で行っています。devcontainerで開発を行う場合、VScodeのインテリセンスが動作することを確認しています。

# 前提条件

C++での開発経験がある読者を想定する

# コンテンツ

## 導入方法

### devcontainer使用の場合（推奨）

1. [このレポジトリ](https://github.com/OsawaKousei/standalone_auto_drive_ws)からソースコードを入手する
1. VScodeでcloneしたワークスペースを開く
1. ワークスペースをdevcontainerで開く（例：リモートエクスプローラ->「開発コンテナー」を選択->新しい開発コンテナー(+)->コンテナーで現在のフォルダを開く）
   ※初回は時間がかかる
1. ビルド：`bash scripts/build_internal.sh`
1. 実行：`./build/auto_drive_app`
   ※ウィンドウが立ち上がり、自律走行のデモが実行されれば成功

#### devcontainerからホスト画面へ表示する場合

LinuxホストのローカルX11／XWaylandセッションを使用します。ホストに `xauth` が必要です（Ubuntuでは `sudo apt-get install xauth`）。ホストのデスクトップ端末で `DISPLAY` が設定されていることを確認してください。認証ファイルが標準の `~/.Xauthority` 以外にある場合は、そのセッションの `XAUTHORITY` もVS Codeへ引き継いでください。環境変数を設定し直した場合は、ホスト側のVS Codeを終了してデスクトップ端末から `code .` で開き直します。

コンテナの起動前に `prepare-x11.sh` がホスト上で対象ディスプレイの認証Cookieを抽出し、`.devcontainer/.xauthority` に保存します。このファイルとX11ソケットをコンテナへ読み取り専用でマウントし、ホストの `DISPLAY` とコンテナ用の `XAUTHORITY` を設定します。生成した認証ファイルはGit管理の対象外です。

MesaのソフトウェアVulkan描画では、コンテナとホストのIPC名前空間が異なるため、X11の共有メモリ（MIT-SHM）への接続が `BadAccess`（`X_ShmAttach`）で失敗する場合があります。`containerEnv` の `MESA_VK_WSI_DEBUG=noshm` でMesaの共有メモリ転送を無効にし、通常のX11転送で表示します。この設定はIPC名前空間の分離を維持します。

現在のコンテナですぐに表示を確認する場合は、次のコマンドを実行します。

```bash
MESA_VK_WSI_DEBUG=noshm ./build/auto_drive_app
```

設定を反映するには、VS Codeの **Dev Containers: Rebuild and Reopen in Container** を実行してください。再構築後、コンテナ内で `./build/auto_drive_app` を実行できます。ディスプレイ番号が変わった場合も再構築が必要です。起動前にCookieが見つからない場合は、ホストの `XAUTHORITY` を確認してください。Waylandソケットの転送だけでは、X11を使用する現在のバックエンドには接続できません。

### ホストで実行する場合（追加の依存関係をインストールする必要があります）

1. [このレポジトリ](https://github.com/OsawaKousei/standalone_auto_drive_ws)からソースコードを入手する
1. 実行用のVulkanローダー・ドライバーとX11環境を用意する（Ubuntuでは `libvulkan1 mesa-vulkan-drivers libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6 libxrender1`）。Python製ツールも使用する場合は `uv sync`
1. ビルド：`bash scripts/build.sh`
1. 実行：`./build/auto_drive_app`
   ※コンテナ内で生成されるGCC 12でコンパイルされたバイナリを実行できる環境が必要です

## 描画バックエンド

既定は [rust-wgpu-plotting v0.1.0](https://github.com/OsawaKousei/rust-wgpu-plotting/releases/tag/v0.1.0) の Linux x86_64 バイナリです。CMake の構成時に `ad-visualization-v0.1.0-linux-x86_64.tar.gz` を取得し、固定した SHA-256 を検証して `build/_deps/ad_visualization_bundle-src/` に展開します。初回は他の C++ 依存関係の取得にもネットワーク接続が必要です。`ref/` へのソース配置や Rust ツールチェーンは不要です。wgpu を選択した C++ アプリのビルド・実行には Python・NumPy・Matplotlib も不要です。

Docker／devcontainer は C++ ビルドツールと Vulkan・X11 の実行ライブラリを導入します。既存の開発コンテナは **Dev Containers: Rebuild and Reopen in Container** で再構築してください。

コンテナ内で直接ビルドする場合：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DAD_VISUALIZATION_BACKEND=wgpu
cmake --build build -j4
./build/auto_drive_app
```

Linux x86_64 / Vulkan / X11 を対象にしています。PNG 保存にも Vulkan アダプターが必要ですが、画面表示は不要です。GPU のない環境では Mesa のソフトウェア Vulkan ドライバーを使用できます。実行時はビルドしたアプリと `build/_deps/ad_visualization_bundle-src/` を同じ位置関係で保持してください。

手動取得済みの Release を使用する場合は、`.tar.gz.sha256` でアーカイブを検証し、展開先で `sha256sum -c SHA256SUMS` を実行した後、`-DAD_VISUALIZATION_ROOT=/absolute/path/to/ad-visualization-v0.1.0-linux-x86_64` を CMake に指定します。この指定では自動ダウンロードとアーカイブのハッシュ検証を行いません。

以前のソースビルドのキャッシュを引き継がずにクリーンビルドを確認する場合は、未使用のビルドディレクトリを指定します：

```bash
cmake -S . -B build/release-clean -DCMAKE_BUILD_TYPE=Debug \
  -DAD_VISUALIZATION_BACKEND=wgpu -DAD_TEST_RENDER_OUTPUT=ON -DAD_TEST_RENDER_WINDOW=ON
cmake --build build/release-clean -j4
ctest --test-dir build/release-clean -R '^visualizer_(cpu|png)$' --output-on-failure
xvfb-run -a ctest --test-dir build/release-clean -R '^visualizer_window$' --output-on-failure
```

ウィンドウテストには `xvfb`・`xauth` が必要です（devcontainer に同梱）。ホスト／コンテナ間の CMake キャッシュは共有せず、環境を切り替える場合も新しいビルドディレクトリを使用してください。

従来の描画は `-DAD_VISUALIZATION_BACKEND=matplotlib` で選択できます。スクリプトを使用する場合は `AD_VISUALIZATION_BACKEND=matplotlib bash scripts/build_internal.sh` または `AD_VISUALIZATION_BACKEND=matplotlib bash scripts/build.sh` とします。この場合はPython・NumPy・Matplotlibが必要です。Python製の解析・地図編集ツールの依存も引き続き `uv sync` で導入します。

wgpuでは座標軸がメートル単位となり、占有セルは黒、空きセルは白で表示します。地図のセル境界が明示されるため、従来のMatplotlib画像とは表示位置・配色に差があります。ウィンドウ操作とVisualizerの破棄は同じスレッドで行ってください。

C++アダプターのCPUテストはGPUなしで実行できます。PNG・ウィンドウの統合テストは明示的に有効化します。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DAD_VISUALIZATION_BACKEND=wgpu \
  -DAD_TEST_RENDER_OUTPUT=ON -DAD_TEST_RENDER_WINDOW=ON
cmake --build build --target visualizer_test -j4
ctest --test-dir build -R '^visualizer_(cpu|png)$' --output-on-failure
xvfb-run -a ctest --test-dir build -R '^visualizer_window$' --output-on-failure
```

## 使い方

### メインとなるアプリケーションについて

#### auto_drive_app

scenario.tomlファイルに基づいて自律走行のシミュレーションを行います。
実行方法：`./build/auto_drive_app`
実行ログをauto_drive_app.logとして保存します。
終了時のスナップショットをauto_drive_app_result.pngとして保存します。

#### auto_drive_log_replay

replay.tomlに基づいて、auto_drive_appの結果をリプレイできます。
実行方法：`./build/auto_drive_log_replay`
終了時のスナップショットをauto_drive_log_replay_result.pngとして保存します。

### configについて

configs/scenario.toml及びconfigs/replay.tomlが起点となるconfigで、メインのアプリケーションで参照されています。
実際にこれらのファイルにデフォルトで記載されている内容よりも遥かに多くの設定項目が必要ですが、記載が無い項目は内部でconfigs/defaults以下のconfigファイル群にフォールバックされます。
ユーザーは変更したい項目についてのみ記述することで、設定を反映できます。
具体的な設定項目やその意味については、デフォルトコンフィグのコメントを参照してください。

### mapについて

シミュレーターが使用する地図はROS 2で使用されるパターンを参考に、yaml形式のメタファイルとpgm形式の実体で構成されますが、互換性の検証はしていません。
付属のmap_editor.pyを使用する（`uv run tools/map_editor.py`）ことで、mapを作成できます。これは、map_schema.yamlに記述されているマップの定義をリアルタイムでレンダリングし、任意のタイミングでこれを所定の形式にエクスポートできます。

### 基本的なディレクトリ構造について

src/以下に実装が置かれています。shared/以下には共通の部品や型定義が、features/以下には機能別の実装が置かれています。
control/：経路追従
localization/：自己位置推定
planning/：経路生成
simulation/：センサーや運動モデル、衝突判定
visualization/：描画系

※重要なポイントとして、本プロジェクトでは、シミュレーターとして統合されたオブジェクトやインタフェースは存在しません。ロボットの運動計算・衝突判定・センサーの再現・描画はそれぞれ完全に独立しており、メインアプリケーションが自己位置推定アルゴリズムの実行と併せてこれらを適切に呼び出す責務を負います。

### サンプルとなるアルゴリズムの実装状況

経路追従：Pure Pursuit
自己位置推定：EKF
経路生成：A\*
