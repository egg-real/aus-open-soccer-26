#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="aus-open-soccer-logic.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

PI_USER="${PI_USER:-$(whoami)}"
WORKDIR="${WORKDIR:-$PI_DIR}"
PYTHON="${PYTHON:-/home/dsa/env/bin/python}"

if [[ ! -f "$PI_DIR/logic.py" ]]; then
  echo "error: expected logic.py at $PI_DIR/logic.py" >&2
  exit 1
fi

sed \
  -e "s|@USER@|$PI_USER|g" \
  -e "s|@WORKDIR@|$WORKDIR|g" \
  -e "s|@PYTHON@|$PYTHON|g" \
  "$SCRIPT_DIR/aus-open-soccer-logic.service" \
  | sudo tee "$SERVICE_PATH" >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo "Installed $SERVICE_NAME"
echo "  user:     $PI_USER"
echo "  workdir:  $WORKDIR"
echo "  python:   $PYTHON"
echo
echo "Start now:  sudo systemctl start aus-open-soccer-logic"
echo "View logs:  journalctl -u aus-open-soccer-logic -f"
echo "Stop:       sudo systemctl stop aus-open-soccer-logic"
