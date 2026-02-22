#!/bin/bash
# scripts/build.sh (ホスト側実行用)

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="cpp-simple-builder"

# ホスト側の uv キャッシュパスと Python インストール先のパス
UV_CACHE_HOST="${HOME}/.cache/uv"
UV_PYTHON_HOST="${HOME}/.local/share/uv/python"
mkdir -p "${UV_CACHE_HOST}"
mkdir -p "${UV_PYTHON_HOST}"

echo "1. ビルド用イメージを確認中..."
docker build -t "${IMAGE_NAME}" "${PROJECT_ROOT}/docker"

echo "2. コンテナを起動し、使い捨て環境でビルドを開始します..."

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${PROJECT_ROOT}:/workspace" \
    -v "${UV_CACHE_HOST}:/tmp/.cache/uv" \
    -v "${UV_PYTHON_HOST}:/tmp/.uv_python" \
    -e "UV_CACHE_DIR=/tmp/.cache/uv" \
    -e "UV_PYTHON_INSTALL_DIR=/tmp/.uv_python" \
    -e "HOME=/tmp" \
    -w /workspace \
    "${IMAGE_NAME}" \
    bash -c '
        set -e
        echo "0. Git safe.directory を設定中..."
        git config --global --add safe.directory /workspace || true
        if [ -d build/_deps ]; then
            find build/_deps -maxdepth 1 -type d -name "*-src" -exec git config --global --add safe.directory {} \; || true
        fi

        echo "1. 使い捨て仮想環境を構築中 (/tmp/.venv_build)..."
        export UV_PROJECT_ENVIRONMENT=/tmp/.venv_build
        export VIRTUAL_ENV=/tmp/.venv_build
        export PATH=/tmp/.venv_build/bin:$PATH
        
        # [重要] システムのPythonを無視し、uvが管理する完全なPythonを強制する
        export UV_PYTHON_PREFERENCE=only-managed

        uv sync --active

        echo "2. CMake 構成中..."
        # [重要] CMakeにも仮想環境のPythonだけを見るように強制する
        cmake -B build -S . \
            -DCMAKE_BUILD_TYPE=Debug \
            -DENABLE_SANITIZER=OFF \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DPython3_FIND_VIRTUALENV=ONLY

        echo "3. ビルド実行中..."
        cmake --build build -j$(nproc)
    '

# 成果物の確認
if [ -f "${PROJECT_ROOT}/build/auto_drive_app" ]; then
    echo "========================="
    echo "   　　ビルド成功 　　　 "
    echo "========================="
    echo "実行ファイル: ${PROJECT_ROOT}/build/auto_drive_app"
else
    echo "❌ 成果物が見つかりません。ビルドに失敗した可能性があります。"
    exit 1
fi