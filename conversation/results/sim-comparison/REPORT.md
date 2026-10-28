# Gemma vs Qwen: vision, RViz and MoveIt mock integration

Follow-up: [Gemma QAT/llama.cpp and live Phonon](../comparison/GEMMA-QAT.md) records the newer optimized
deployment. This report preserves the Transformers comparison.

Date: 2026-10-27. Branch: `feat/robot-conversation`. No real hardware connected or commanded by these tests.

Both backends completed all three ordinary simulated pick/place cases. This supports an experimental Gemma integration, but does not overturn its blank-image failures in the [separate robustness comparison](../comparison/REPORT.md). Keep Qwen as the default; neither model's free-form answers establish hidden counts or metric measurements.

## Matched final runs

Both used `samples/composite_vara_dest.png`, GPU segmentation, the existing conversation bridge and policy, the native RViz pick panel, MoveIt, mock ros2_control and fake gripper I/O. ROS domain 61 was localhost-only. The runner checks `hardware == SIMULATION` before dispatch and throughout execution; confirmation requests are automated only there. Each model ran A, B and C sequentially; the simulation was restarted before the Qwen set.

| Test | Gemma 4 E2B | Qwen3-VL-2B |
|---|---:|---:|
| Chat → image check → VARa pick/place | PASS, 28.39 s | PASS, 28.54 s |
| Chat → image check → VARb pick/place | PASS, 21.37 s | PASS, 20.91 s |
| Chat → image check → VARc pick/place | PASS, 24.44 s | PASS, 22.90 s |
| Refusal/status suite | 10/10, zero dispatches | 10/10, zero dispatches |
| Stop during accepted action | PASS; controller result received | PASS; controller result received |
| Duplicate task ID | Suppressed | Suppressed |
| Vision message delivery rate | 5.001 Hz | 5.005 Hz |
| Maximum observed vision message gap | 0.250 s | 0.231 s |
| Vision samples / observation window | 200 / 40 s | 195 / 40 s |
| Model health: allocated GPU memory | 1,853 MiB | 4,091 MiB |

Each pick time is one run, not a median or statistical performance claim. It includes model checking, process startup, planning, motion, gripper waits and scripted confirmations. It is not VLM inference latency. Message rates use first-to-last delivery; the 40-second window also includes discovery, explaining fewer Qwen samples despite approximately 5 Hz steady delivery. Rate windows overlapped normal pick runs, not a sustained maximum-rate VLM stress test.

The raw JSON files in `gemma/` and `qwen/` contain controller phases, final mock hardware identity, conversation output, runtime health, vision delivery, cancellation and refusal evidence. Success means the panel reported `completed` after the sequence ran. The image fixture stays static throughout; it does not simulate a changing scene as the board moves.

## Deployment differences

- Qwen: BF16 GPU, Transformers 4.57.6, existing Pixi train environment.
- Gemma: NF4/BF16 compute, Transformers 5.21.0 and bitsandbytes 0.50.2 in the isolated comparison environment. Pinned model revision remains `3e22461f65e89153144f8adb70e3b8c2cc9845a7`.
- Gemma's large per-layer embedding table now stays in system RAM. Token lookup runs on CPU and only selected rows return to GPU. Vision and language computation remain on GPU; unused audio modules are offloaded. Its steady memory figure therefore differs substantially from the earlier all-GPU embedding benchmark. It is not evidence that Gemma's complete model is smaller than Qwen.
- Load Gemma before starting vision: initial weight loading still needs more VRAM than steady inference. The loader's optional allocator preallocation is disabled for this backend.
- Both retain CPU Whisper and eSpeak. Phonon was tested separately on saved replies and is not part of these control-path runs.

The refusal/status tests largely exercise deterministic policy before model inference. The cancellation test injects a bounded task and sends conversational `stop`; cancellation intentionally bypasses the VLM. Passing these tests validates the shared stack under each deployment, not superior model reasoning.

## Failures found and fixes

1. The first Gemma integration request failed with a CPU/GPU index mismatch. The embedding module's inherited device hook was moving indices back to GPU. The CPU wrapper now removes that hook before moving the lookup table. The failed request dispatched no task; `gemma/setup-error.json` preserves it.
2. An initial simulated pick got stuck after a scene-service response was lost (`failed to send response (timeout)` in the scene manager). That run was terminated before motion. The pick script now bounds service discovery and response waits to five seconds and stops on failure rather than waiting indefinitely. Three new unit cases cover missing service, lost response and successful response. The full suite is **65/65 passed**.
3. An attempted negative-suite run overlapped the tail of that aborted setup sequence and interfered with B/C test conversation output. Those files remain under `gemma/setup-attempts/` and are excluded. All final pick and refusal runs were executed sequentially, without overlapping test clients.

No automatic retry was added to the real task controller. Final Gemma and Qwen pick cases passed after the setup fixes. The results above do not silently count failed setup attempts as successful trials.

## Reproduce

Use the backend launch commands in [conversation README](../../README.md). Start the composite-fixture demo, wait for idle `SIMULATION`, and run each command sequentially:

```bash
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_check.py --chat --variant VARa
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_check.py --chat --variant VARb
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_check.py --chat --variant VARc
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_negative_check.py --output /tmp/negative.json
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_cancel_check.py
```

`measure_vision.py --output /tmp/vision-rate.json` may run concurrently because it only subscribes. Do not overlap multiple action/conversation test clients. `sim_check.py` now ends promptly on a model error or refusal rather than waiting the full action timeout.

Still unverified: physical placement/grasp accuracy, real suction feedback, moving/live-camera scenes, German, native Gemma audio, live microphone latency, and Phonon streaming cancellation. The earlier negative-image results remain a reason not to promote Gemma as an independent visual safety check.
