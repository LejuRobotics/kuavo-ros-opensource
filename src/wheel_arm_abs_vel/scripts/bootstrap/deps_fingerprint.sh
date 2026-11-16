#!/usr/bin/env bash
# Hash the pinned Noble dependency inputs before reusing a prepared underlay.
set -e -o pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
sha256sum deps_fingerprint.sh deps_setup.bash setup_noble.sh ros_noble.sh jobs.py \
  noble-python.lock noble-apt-packages.txt noetic-noble.rosinstall | sha256sum | cut -d ' ' -f 1
