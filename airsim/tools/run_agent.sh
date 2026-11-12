#!/usr/bin/env bash
set -euo pipefail
archive_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
agent_binary="$archive_dir/.local/bin/MicroXRCEAgent"
if [[ ! -x "$agent_binary" ]]; then
  echo 'Build the bundled agent first: bash airsim/tools/build.sh agent' >&2
  exit 1
fi
export LD_LIBRARY_PATH="$archive_dir/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$agent_binary" udp4 -p "${1:-8888}"
