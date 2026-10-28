# Robot conversation (branch `feat/robot-conversation`)

Chat and push-to-talk (RViz panel or web UI) for the FR5 cell. Fixed sentence patterns become **jobs for the main
controller** (`feat/task-main-controller`); the local model only answers questions and never moves the robot.

```
panel / web chat / voice --/conversation/input--> bridge.py --policy.py--> task_link.py --> agent bridge
   (<ws>/live_logs/session_*/bridge/cmd, tools/agent_bridge.py) --> /task/add_job, /task/control
status: /task/status (JSON) -> chat replies + /conversation/batch_state;  scene: /vision/boards (boards + zones)
```

## Start (simulation)

```bash
conversation/model.sh --backend llama      # optional: questions + speech (see below)
conversation/demo.sh                      # main controller sim (trial_up --sim) + bridge + RViz, domain 62
```

Real robot: start the controller with `tools/trial_up.sh` (see its README), then `pixi run conversation` here on the
same `ROS_DOMAIN_ID`. The bridge finds the newest trial session itself (`HK_BRIDGE_DIR` overrides).

## What you can say

| Text | Controller command |
|---|---|
| `move two variant B boards to drop off b`, `pick variant A`, `move all var c` | `transfer VARb drop_off_b 2` ... (no zone: `drop_off_<letter>` if seen, the only drop-off zone, else it asks) |
| any controller command verbatim: `process VARa processing_1 drop_off_a 1`, `clear drop_off_a drop_off_b`, `board b7 drop_off_a`, `prio j3 5`, `cancel j3` | sent as typed |
| `stop` / `cancel` | `stop` (smooth stop now, then paused; always sent, never gated) |
| `cancel batch` / `cancel all` | `clear_queue` (a held board is placed first) |
| `pause`, `resume` / `continue`, `home` | same |
| `status` | read from `/task/status` |
| anything else | question for the local model (images + detections + status), no motion |

Jobs are refused with a reason when no `/task/status` arrived for 3 s or motion / skills are not ready; the
controller validates everything else and its answer is read back. Finished / paused jobs and new alerts are
announced in the chat.

## Checks

```bash
pixi run conversation-check                       # unit tests (no ROS)
python3 conversation/policy.py                    # command patterns self-check
pixi run python conversation/sim_check.py         # end-to-end with conversation/demo.sh running (13 checks)
```

## First setup / after reboot

```bash
pixi install --locked -e train
# Optional: conversation/setup_audio.sh (eSpeak for synthetic ASR benchmark fixtures only)
conversation/model.sh --download
```

Qwen3-VL-2B-Instruct runs in BF16 on CUDA. Faster Whisper `base.en` runs on CPU in int8; Phonon 31M English produces
spoken output. PulseAudio/PipeWire's `parec` captures the default microphone only while PTT is held, at most
20 seconds. Microphone content and conversation context stay in memory. Phonon stays loaded on CPU and streams
24 kHz audio through `pacat`; Stop, clearing chat and starting PTT interrupt speech. Health reports
`speech_output.engine`, playback state and errors. There is no silent fallback to eSpeak.

The existing CUDA training environment is reused because a separate PyTorch environment exhausted disk space.
The model cache currently lives in **`/tmp/robot-conversation-models`** (about 4.2 GB): **it disappears on reboot**.
To retain models, set `CONVERSATION_CACHE` to a writable location with sufficient free space before downloading
and on subsequent launches. No old datasets or model caches were removed. The optional audio setup script
extracts eSpeak under ignored `.tools/` for synthetic ASR fixtures; live playback uses Phonon and `pacat`.

### Experimental Gemma simulation backend

The default remains Qwen. Gemma requires a separate environment with Transformers 5.21.0 and
bitsandbytes 0.50.2; do not upgrade the existing Pixi train environment for this experiment.
With the comparison environment and model cache present, start Gemma **before** GPU vision:

```bash
HF_HOME=/home/ub26/.cache/robot-conversation-comparison PATH="$PWD/.tools:$PATH" \
  /tmp/gemma-comparison-env/bin/python conversation/model_server.py \
  --backend gemma --asr-cache /tmp/robot-conversation-models/hub
# In another terminal:
conversation/demo.sh
```

Gemma uses NF4/BF16 on GPU, a CPU per-layer embedding lookup, and offloaded unused audio modules.
The embedding wrapper transfers selected rows rather than the whole table each token. Peak model loading
still needs more GPU memory than steady inference. The `/tmp` environment and Whisper cache are temporary.
This backend is experimental: successful ordinary simulated picks do not resolve the blank-image false
approvals found in the separate robustness tests. The interactive service now uses Phonon for speech output.

### Gemma QAT with llama.cpp

The alternative `--backend llama` uses the same HTTP interface and robot policy, but forwards model
requests to a separate localhost llama.cpp server. It does not load Gemma through Transformers.
`conversation/gemma_qat.sh` pins the Google QAT Q4_0 checkpoint and vision projector and expects
llama.cpp build b11934 in `.tools/llama-b11934` (or set `LLAMA_SERVER`).

```bash
# Terminal 1: quantized multimodal model, port 8770
conversation/gemma_qat.sh
# Terminal 2: existing conversation/audio HTTP adapter, port 8766
conversation/model.sh --backend llama
# Terminal 3: mock hardware only
conversation/demo.sh
```

Model weights are read from the pinned Hugging Face comparison cache; `GEMMA_QAT_DIR` overrides it.
The launcher uses CUDA libraries from the existing Pixi train environment and binds to loopback.
The adapter verifies that llama-server is ready at startup. Stop the adapter and llama-server separately.
Requests use deterministic decoding, disabled thinking, and no cross-request prompt-cache reuse.
Measured results and live-speech verification: [QAT retry report](results/comparison/GEMMA-QAT.md).
`pixi run -e train python conversation/check_speech.py` performs an audible playback/interruption check
without opening the microphone.

## Parakeet speech recognition

Speech output now uses pinned `gradium/phonon-40m-multilingual` on CPU Q8, with
English text normalization. It retains queued, interruptible playback and
pronounces `VARa/b/c` as “variant A, variant B, variant C.” The model requires
Hugging Face access and is licensed CC BY-NC 4.0; commercial deployment requires
appropriate permission from Gradium. The launcher preserves the existing HF
credential path when selecting an alternate model cache. The default cache is now persistent.

Run `conversation/model.sh --backend llama --asr parakeet --download` once to
download NVIDIA `parakeet-tdt-0.6b-v3`, then omit `--download` for cache-only starts.
The launcher selects the isolated Pixi `speech` environment (Transformers 5),
leaving the existing `train` environment available for Whisper and VLM work.
Parakeet uses CUDA BF16 by default; `--asr-device cpu` selects CPU FP32.
The existing Phonon output and bounded robot command parser remain in use.
This is push-to-talk transcription after release, not continuous listening.

Check saved synthetic recordings without commanding a robot:
`python3 conversation/check_asr.py --output conversation/results/voice-batch/parakeet.json`.
Human microphone/accent accuracy must be tested separately. To revert, restart
the adapter with `conversation/model.sh --backend llama --asr whisper`.
