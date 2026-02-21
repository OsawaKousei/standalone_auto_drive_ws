#!/bin/bash
set -e

# .venv_docker 内の Python を明示的に指定
VENV_PYTHON="/workspace/.venv_docker/bin/python"

# 1. Pythonの標準ライブラリがある「ベースパス」を取得して PYTHONHOME にセット
export PYTHONHOME=$(${VENV_PYTHON} -c "import sys; print(sys.base_prefix)")

# 2. numpy や matplotlib がある「仮想環境のパス」を取得して PYTHONPATH にセット
export PYTHONPATH=$(${VENV_PYTHON} -c "import site; print(':'.join(site.getsitepackages()))")

echo "=== Python Runtime Config ==="
echo "PYTHONHOME: ${PYTHONHOME}"
echo "PYTHONPATH: ${PYTHONPATH}"
echo "============================="

# 渡されたコマンド（引数）を実行
exec "$@"