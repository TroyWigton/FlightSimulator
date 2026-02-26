#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON_BIN:-python3}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Error: '$PYTHON_BIN' was not found on PATH. Install Python 3.10+ and retry."
  exit 1
fi

if [[ ! -d ".venv" ]]; then
  echo "Creating virtual environment in .venv ..."
  "$PYTHON_BIN" -m venv .venv
fi

VENV_PY=".venv/bin/python"

echo "Ensuring dependencies are installed ..."
"$VENV_PY" -m pip install -r requirements.txt

case "${1:-}" in
  --setup-only)
    echo "Environment setup complete."
    exit 0
    ;;
  --test)
    exec "$VENV_PY" -m pytest -q
    ;;
esac

echo "Starting flight simulator ..."
exec "$VENV_PY" -m flightsim
