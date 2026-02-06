#!/bin/bash
# scripts/build.sh (権限対応版)

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="cpp-simple-builder"

# 1. ビルド用コンテナイメージを作成
docker build -t ${IMAGE_NAME} "${PROJECT_ROOT}/docker"

echo "3. コンテナ内でビルドを実行中..."
# --user $(id -u):$(id -g) を追加してホストユーザーとして書き込む
docker run --rm \
    --user $(id -u):$(id -g) \
    -v "${PROJECT_ROOT}:/workspace" \
    ${IMAGE_NAME} \
    bash -c "cmake -B build -S . -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)"

if [ -f "${PROJECT_ROOT}/build/app" ]; then
    echo "=== ビルド成功 ==="
    echo "実行ファイル: ${PROJECT_ROOT}/build/app"
else
    echo "❌ ビルドに失敗しました。"
    exit 1
fi