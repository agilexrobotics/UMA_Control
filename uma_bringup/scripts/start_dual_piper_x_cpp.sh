#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAUNCH_FILE=demo_dual_piper_x.launch.py
exec bash "${SCRIPT_DIR}/start_dual_nero_cpp.sh" "$@"
