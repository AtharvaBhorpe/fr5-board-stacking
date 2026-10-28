# Local robot conversation — MVP1

Branch: `feat/robot-conversation`. English text and push-to-talk in the existing RViz panel.
Multi-board scheduling is documented in [BATCHES.md](BATCHES.md), including validation scope and deployment options.
This implements the team's **LLM Agent** box. Existing calibrated vision, wrist depth and the robot controller
still own geometry and motion. This is a supervised prototype, not an autonomous manipulation policy.

## Start the demo on this laptop

From `vision1-overhead`, three terminals for the current Gemma setup:

```bash
# Terminal 1: pinned Gemma QAT on the GPU
conversation/gemma_qat.sh

# Terminal 2: local model adapter, speech recognition, and Phonon
conversation/model.sh --backend llama

# Terminal 3: isolated MoveIt mock robot, dynamic replay vision, bridge and RViz
conversation/demo.sh
```

Wait for `READY` in terminal 2 and `SIMULATION` in RViz. The demo uses ROS domain **62**, localhost-only.
It does not connect to the FR5. Do not run a second copy of the demo on the same domain.
Ctrl+C in terminal 3 stops the processes launched by that script. Model services are separate.
If the development demo is already open, use it instead of starting another copy.

In RViz, type `status`, `what do you see?`, or `pick variant B`. **Hold to talk** captures your microphone;
release to transcribe. An accepted pick request runs through pick, placement, and release automatically. The screenshot capture hook only captures Qt widgets;
OpenGL scene contents may not appear in that capture, even though RViz renders them on screen.

The default demo has two B boards. Use **Reset 2 B**, then `pick two variant B boards`, or
**Reset A/B/C**, then `sort all visible boards`. Reset is accepted only while the controller
and batch are idle. **Cancel batch** works during and between picks. Progress appears above chat.
The simulated image removes a source board after mock-controller completion; the real GPU detector
and Gemma then process the changed image. It does not simulate contact, vacuum physics, destination
stack growth, wrist depth, or physical placement accuracy. Camera and robot hardware remain disconnected.

For VARa's composite recorded scene:

```bash
conversation/demo.sh samples/composite_vara_dest.png
```

The composite image is a synthetic test fixture, not a live observation of the robot scene.

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

## Supported behavior

- Explicit commands such as `pick VARa`, `please pick variant B`, `pick and place variant C` request one pick.
- `status`, `stop`, `cancel`, and `clear chat` have bounded local meanings.
- Other text is conversational. It can never become a motion command through model-generated output.
  Ask explicitly again after clarifying a variant; MVP1 does not turn a bare `B` follow-up into an action.
- A task requires fresh controller and vision timestamps, known hardware, calibration, a freely pickable variant,
  its destination, a fresh overhead image, and a successful model image check. Changed target poses are rejected.
- Replay is permitted only with `allow_replay:=true` **and** SIMULATION hardware. Real operation requires
  vision's explicit `live: true`. No automatic task retry or remote real-hardware confirmation.
- The controller rejects expired requests and duplicate task IDs. Result messages are correlated with the request.
- Pressing PTT interrupts spoken output, not robot motion. **Stop** bypasses inference and requests cancellation
  of the pick script's active action. Check the log for cancellation completion. This is not the pendant e-stop.
- Closing/clearing chat does not cancel an existing task. Unknown or unavailable speech/model services do not
  dispatch a pick. A recognized phrase is not proof that the user intended it; inspect the visible transcript.

## Real-robot test handoff

1. Stop the isolated demo and its replay node. Keep the local model server running.
2. Use the existing real-robot, overhead GPU vision and D405 startup procedure from the main README, all on
   the same intended ROS domain. Do not set `CONVERSATION_SIM`. Use live C920 input, not a sample file.
3. Launch this branch's `robot/rviz.sh` in that domain, then:

   ```bash
   pixi run conversation
   ```

4. Verify the panel says **REAL FR5**, both cameras stream, the gripper state is valid, and the expected variant
   and template are present. Clear simulation extra arguments: no `--speed 0.3` or `--no-wrist-height`.
5. Test `status` and a scene question first. Test PTT transcription of variant names without requesting motion
   (for example, `what is variant B?`). Lab-noise and the operator's microphone/voice remain unvalidated.
6. Request one variant. The request permits approach, pick descent, grip, lift, carry, placement, release,
   and return to park without confirmation pauses. Use `--hover-only` for a
   contact-free alignment check. Repeat per variant only after inspecting results.

Dynamic D405 pick/drop heights and placement accuracy are inherited from the existing controller and were not
physically revalidated during this session. The integration blocks fully occluded targets instead of automatically
asking the existing controller to clear covering boards.

## Interfaces

| Interface | Type | Contract |
|---|---|---|
| `/conversation/input` | `std_msgs/String` | English text |
| `/conversation/control` | `std_msgs/String` | `ptt_start`, `ptt_stop`, `clear` |
| `/conversation/output` | `std_msgs/String` JSON | `role`, `text`, `phase` for RViz |
| `/conversation/task` | `std_msgs/String` JSON | `id`, `variant`, Unix `stamp`; panel checks before starting |
| `/pick_panel/state` | `std_msgs/String` JSON | `stamp`, `hardware`, `busy`, `waiting`, `task_id`, `phase`, `detail` |
| `/vision/boards` | existing JSON | adds `live` provenance flag; existing geometry unchanged |
| `127.0.0.1:8766` | local HTTP | GET health/device; POST `/ask`, `/speak`, `/silence`, `/listen/start`, `/listen/stop`, `/transcribe` |

The HTTP service binds loopback only. Do not expose it or ROS task topics to untrusted clients; this MVP has no
remote authentication. `/transcribe` accepts mono 16-bit 16 kHz WAV for repeatable synthetic-speech tests.

## Checks

```bash
pixi run conversation-check
# With isolated demo open and IDLE, the following automate confirmations ONLY after verifying SIMULATION:
ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_check.py --variant VARb --chat
ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_negative_check.py
# With VARa composite scene:
ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_cancel_check.py
# Local VLM and generated speech, no robot commands:
PATH="$PWD/.tools:$PATH" pixi run -e train python conversation/benchmark.py
```

Results are under `conversation/results/`. Synthetic speech tests are not live microphone tests; the 30 VLM
turns repeat three prompts on one recorded scene, and the 30 ASR turns repeat ten generated utterances.
The separate policy and ROS-negative suites cover unsupported commands and dispatch boundaries.
These finite checks do not establish general VLM accuracy, physical success, or full PTT-to-speech latency.

The subsequent [VLM/Phonon comparison](results/comparison/REPORT.md) and
[MoveIt/vision comparison](results/sim-comparison/REPORT.md) record Gemma and Qwen separately.
Wake-word listening remains MVP2; German remains MVP3.

### Experimental Gemma simulation backend

The default remains Qwen. Gemma requires a separate environment with Transformers 5.21.0 and
bitsandbytes 0.50.2; do not upgrade the existing Pixi train environment for this experiment.
With the comparison environment and model cache present, start Gemma **before** GPU vision:

```bash
HF_HOME=/home/ub26/.cache/robot-conversation-comparison PATH="$PWD/.tools:$PATH" \
  /tmp/gemma-comparison-env/bin/python conversation/model_server.py \
  --backend gemma --asr-cache /tmp/robot-conversation-models/hub
# In another terminal:
conversation/demo.sh samples/composite_vara_dest.png
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
conversation/demo.sh samples/composite_vara_dest.png
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
