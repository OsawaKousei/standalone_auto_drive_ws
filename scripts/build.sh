#!/bin/bash
# scripts/build.sh (ホスト側実行用)

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="cpp-simple-builder"
UV_CACHE_DIR="${HOME}/.cache/uv"

# 1. uv のキャッシュディレクトリが存在することを確認（マウントエラー防止）
mkdir -p "${UV_CACHE_DIR}"

# 2. イメージがなければビルド（Dockerfileの変更検知はDockerに任せる）
echo "1. ビルド用イメージを確認中..."
docker build -t "${IMAGE_NAME}" "${PROJECT_ROOT}/docker"

echo "2. コンテナを起動してビルドを開始します..."

# 3. docker run の実行
# --user: ホストユーザーとして実行し、権限問題を回避
# -v (workspace): ソースコードのマウント
# -v (uv cache): ホストのuvキャッシュを共有して高速化
docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${PROJECT_ROOT}:/workspace" \
    -v "${UV_CACHE_DIR}:/root/.cache/uv" \
    -e "HOME=/tmp" \
    "${IMAGE_NAME}" \
    /bin/bash /workspace/scripts/build_internal.sh

# 成果物の確認（ホスト側からの視点）
if [ -f "${PROJECT_ROOT}/build/auto_drive_app" ]; then
    echo "=== ホスト側: ビルド完了を確認しました ==="
else
    echo "❌ ビルドに失敗しました。"
    exit 1
fi