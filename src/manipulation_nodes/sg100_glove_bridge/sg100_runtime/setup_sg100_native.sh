#!/usr/bin/env bash
set -euo pipefail
sg100_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"${SG100_SYSTEM_PYTHON:-python3.12}" -m venv "$sg100_base/.venv_sg100_native"
sg100_requirements="$sg100_base/requirements-sg100-native.lock.txt"
if [[ ! -f "$sg100_requirements" ]]; then
    sg100_requirements="$sg100_base/requirements-sg100-native.txt"
fi
"$sg100_base/.venv_sg100_native/bin/python" -m pip install -r "$sg100_requirements"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$sg100_base/.venv_sg100_native/bin/python" -c \
  'import sys,numpy,scipy,pinocchio; print(sys.version); print("NumPy",numpy.__version__,"SciPy",scipy.__version__,"Pinocchio",pinocchio.__version__)'
