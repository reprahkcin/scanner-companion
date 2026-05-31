#!/usr/bin/env bash
# 3D Scanner Control Panel Launcher
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

SYSTEM_PYTHON="${SYSTEM_PYTHON:-/usr/bin/python3}"
VENV_DIR="$ROOT_DIR/.venv"
VENV_PYTHON="$VENV_DIR/bin/python"
APP_ENTRY="$ROOT_DIR/scanner_control.py"

print_header() {
	echo "Starting 3D Scanner Control Panel..."
	echo "====================================="
	echo "Project dir : $ROOT_DIR"
	echo "System Python: $SYSTEM_PYTHON"
}

create_venv() {
	echo "[setup] Creating virtual environment at $VENV_DIR"
	"$SYSTEM_PYTHON" -m venv --system-site-packages "$VENV_DIR"
}

doctor_imports() {
	"$VENV_PYTHON" - <<'PY'
import importlib
import sys

required = ["tkinter", "serial", "cv2", "picamera2", "PIL"]
missing = []

for name in required:
		try:
				importlib.import_module(name)
				print(f"[ok] {name}")
		except Exception as exc:
				missing.append((name, str(exc)))
				print(f"[missing] {name}: {exc}")

if missing:
		print("\nDependency check failed.")
		print("Install Raspberry Pi OS packages, then rerun ./run_scanner.sh:")
		print("  sudo apt update")
		print("  sudo apt install -y python3-opencv python3-picamera2 python3-pil python3-serial")
		sys.exit(1)

print("\nDependency check passed.")
PY
}

usage() {
	cat <<'EOF'
Usage: ./run_scanner.sh [--doctor]

Options:
	--doctor   Validate interpreter + imports and exit.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
	usage
	exit 0
fi

print_header

if [[ ! -x "$SYSTEM_PYTHON" ]]; then
	echo "[error] System Python not found at $SYSTEM_PYTHON"
	exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
	create_venv
fi

echo "[check] Python in use: $($VENV_PYTHON --version)"
echo "[check] Interpreter path: $(readlink -f "$VENV_PYTHON")"

if ! doctor_imports; then
	echo "[repair] Rebuilding virtual environment once and retrying checks..."
	rm -rf "$VENV_DIR"
	create_venv
	doctor_imports
fi

if [[ "${1:-}" == "--doctor" ]]; then
	echo "[done] Environment is healthy."
	exit 0
fi

echo "[launch] Starting scanner UI..."
exec "$VENV_PYTHON" "$APP_ENTRY"
