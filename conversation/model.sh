#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Keep downloaded models across restarts; CONVERSATION_CACHE can select another disk.
export HF_TOKEN_PATH="${HF_TOKEN_PATH:-${HF_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/huggingface}/token}"
export HF_HOME="${CONVERSATION_CACHE:-${HF_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/huggingface}}"
export PATH="$PWD/.tools:$PATH"
model_env=train
for arg in "$@"; do
    if [[ "$arg" == parakeet || "$arg" == --asr=parakeet ]]; then
        model_env=speech
    fi
done
exec pixi run -e "$model_env" python conversation/model_server.py "$@"
