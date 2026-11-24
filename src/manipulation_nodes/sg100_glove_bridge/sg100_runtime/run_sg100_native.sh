#!/usr/bin/env bash
set -euo pipefail
sg100_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec "${SG100_PYTHON:-$sg100_base/.venv_sg100_native/bin/python}" "$sg100_base/sg100_live_native.py" "$@"
