#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
import importlib
import sys
modules = ["numpy", "matplotlib", "tkinter"]
missing = []
for mod in modules:
    try:
        importlib.import_module(mod)
        print(f"[ok] {mod}")
    except Exception as exc:
        print(f"[fail] {mod}: {exc}")
        missing.append(mod)
if missing:
    sys.exit(1)
print("[ok] visualization dependencies are available")
PY

echo "[hint] If any dependency is missing, install via: sudo apt-get update && sudo apt-get install -y python3-numpy python3-matplotlib python3-tk"
