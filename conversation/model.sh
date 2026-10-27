#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# The current laptop has little disk space. Override this to retain weights across reboot.
export HF_HOME="${CONVERSATION_CACHE:-/tmp/robot-conversation-models}"
export PATH="$PWD/.tools:$PATH"
exec pixi run -e train python conversation/model_server.py "$@"
