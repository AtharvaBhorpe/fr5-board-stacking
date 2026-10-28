#!/usr/bin/env bash
# Local, pinned llama.cpp Gemma QAT server; launch the HTTP/ROS adapter separately.
set -euo pipefail
cd "$(dirname "$0")/.."
model_dir="${GEMMA_QAT_DIR:-$HOME/.cache/robot-conversation-comparison/hub/models--google--gemma-4-E2B-it-qat-q4_0-gguf/snapshots/675cff42a74c774d6cb76f76d8eacb49b48c9b93}"
export LD_LIBRARY_PATH="$PWD/.pixi/envs/train/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "${LLAMA_SERVER:-$PWD/.tools/llama-b11934/llama-server}" \
  --model "$model_dir/gemma-4-E2B_q4_0-it.gguf" \
  --mmproj "$model_dir/gemma-4-E2B-it-mmproj.gguf" \
  --host 127.0.0.1 --port 8770 --ctx-size 8192 --parallel 1 \
  --n-gpu-layers 99 --threads 4 --batch-size 512 --ubatch-size 128 \
  --image-max-tokens 280 --reasoning off "$@"
