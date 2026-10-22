#!/bin/sh
if [ $# -eq 0 ]; then
  echo "usage: env.sh command [args...]" >&2
  exit 1
fi
# shellcheck disable=SC1091
. "$(cd "$(dirname "$0")" && pwd)/setup.sh"
exec "$@"
