#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

bash "$SCRIPT_DIR/can_activate.sh" left_arm 1000000 "1-2.4:1.0"
bash "$SCRIPT_DIR/can_activate.sh" right_arm 1000000 "1-2.1:1.0"
bash "$SCRIPT_DIR/can_activate.sh" left_tego 1000000 "1-5:1.0"
bash "$SCRIPT_DIR/can_activate.sh" right_tego 1000000 "1-3:1.0"
