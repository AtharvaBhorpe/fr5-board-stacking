#!/usr/bin/env bash
# Gemma 4 E2B (QAT q4_0 GGUF) on llama.cpp: OpenAI-style API on http://127.0.0.1:8770.   pixi run -e models llm
# The planner (planner.py) and the model server (/ask, natural speech) use it. Extra arguments go to llama-server.
# The first start downloads the model (4.3 GB) into the Hugging Face cache. Later starts work offline.
set -euo pipefail
read -r model mmproj < <(python - <<'EOF'
from huggingface_hub import hf_hub_download
get = lambda f: hf_hub_download("google/gemma-4-E2B-it-qat-q4_0-gguf", f, revision="675cff42a74c774d6cb76f76d8eacb49b48c9b93")
print(get("gemma-4-E2B_q4_0-it.gguf"), get("gemma-4-E2B-it-mmproj.gguf"))
EOF
)
exec llama-server --model "$model" --mmproj "$mmproj" \
  --host 127.0.0.1 --port 8770 --ctx-size 8192 --parallel 1 \
  --n-gpu-layers 99 --threads 4 --batch-size 512 --ubatch-size 128 \
  --image-max-tokens 280 --reasoning off "$@"
