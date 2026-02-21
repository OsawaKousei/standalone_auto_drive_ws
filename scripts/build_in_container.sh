#!/bin/bash
# scripts/build_internal.sh
# ※このスクリプトはコンテナ内部で実行してください

set -e # エラーが発生したら停止

# プロジェクトルートの取得
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "0. Git safe.directory を設定中..."
if command -v git >/dev/null 2>&1; then
    git config --global --add safe.directory "${PROJECT_ROOT}" || true
    git config --global --add safe.directory "${PROJECT_ROOT}/.git" || true

    if [ -d "${BUILD_DIR}/_deps" ]; then
        while IFS= read -r -d '' dep_dir; do
            git config --global --add safe.directory "${dep_dir}" || true
        done < <(find "${BUILD_DIR}/_deps" -maxdepth 1 -type d -name "*-src" -print0)
    fi
fi

echo "1. uv sync で依存関係を確認中..."
# コンテナ内の .venv_docker が最新であることを保証
# (ホストのキャッシュをマウントしていれば一瞬で終わります)
uv sync

echo "2. CMake 構成中..."
# Dockerfileで設定した環境変数 (CC=gcc-12, CXX=g++-12) が自動で使われます
cmake -B "${BUILD_DIR}" -S "${PROJECT_ROOT}" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DENABLE_SANITIZER=OFF \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "3. ビルド実行中..."
cmake --build "${BUILD_DIR}" -j$(nproc)

# 成果物の確認
if [ -f "${BUILD_DIR}/auto_drive_app" ]; then
    echo "========================="
    echo "   🎉 ビルド成功 🎉"
    echo "========================="
    echo "実行ファイル: ${BUILD_DIR}/auto_drive_app"
else
    echo "❌ 成果物が見つかりません。ビルドに失敗した可能性があります。"
    exit 1
fi