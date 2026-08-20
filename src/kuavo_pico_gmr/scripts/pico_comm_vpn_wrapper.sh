#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
export PYTHONPATH="$PACKAGE_DIR/kuavo_gmr:$PACKAGE_DIR:${PYTHONPATH}"

exec python3 "$PACKAGE_DIR/kuavo_gmr/pico_comm_vpn.py" "$@"
