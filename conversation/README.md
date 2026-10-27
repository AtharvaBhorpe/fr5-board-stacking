# Local robot conversation — MVP1

Branch: `feat/robot-conversation`. English text and push-to-talk in the existing RViz panel.
This implements the team's **LLM Agent** box. Existing calibrated vision, wrist depth and the robot controller
still own geometry and motion. This is a supervised prototype, not an autonomous manipulation policy.

## Start the demo on this laptop

From `vision1-overhead`, two terminals:

```bash
# Terminal 1: model and speech service (cache-only, no cloud inference)
conversation/model.sh

# Terminal 2: isolated MoveIt mock robot, GPU replay vision, bridge and RViz
conversation/demo.sh
```

Wait for `READY` in terminal 1 and `SIMULATION` in RViz. The demo uses ROS domain **61**, localhost-only.
It does not connect to the FR5. Do not run a second copy of the demo on the same domain.
Ctrl+C in terminal 2 stops the processes launched by that script. Terminal 1 can stay running for real testing.
If the development demo is already open, use it instead of starting another copy.

In RViz, type `status`, `what do you see?`, or `pick variant B`. **Hold to talk** captures your microphone;
release to transcribe. Press **Confirm** at each descent. The screenshot capture hook only captures Qt widgets;
OpenGL scene contents may not appear in that capture, even though RViz renders them on screen.

For VARa's composite recorded scene:

```bash
conversation/demo.sh samples/composite_vara_dest.png
```

The composite image is a synthetic test fixture, not a live observation of the robot scene.

## First setup / after reboot

```bash
pixi install --locked -e train
conversation/setup_audio.sh
conversation/model.sh --download
```

Qwen3-VL-2B-Instruct runs in BF16 on CUDA. Faster Whisper `base.en` runs on CPU in int8; eSpeak NG produces
spoken output. PulseAudio/PipeWire's `parec` captures the default microphone only while PTT is held, at most
20 seconds. Microphone content and conversation context stay in memory. Text-to-speech is deliberately basic.

The existing CUDA training environment is reused because a separate PyTorch environment exhausted disk space.
The model cache currently lives in **`/tmp/robot-conversation-models`** (about 4.2 GB): **it disappears on reboot**.
To retain models, set `CONVERSATION_CACHE` to a writable location with sufficient free space before downloading
and on subsequent launches. No old datasets or model caches were removed. Audio setup extracts an executable
under ignored `.tools/`; it does not install system packages. It uses this Ubuntu host's eSpeak shared library/data.

## Supported behavior

- Explicit commands such as `pick VARa`, `please pick variant B`, `pick and place variant C` request one pick.
- `status`, `stop`, `cancel`, and `clear chat` have bounded local meanings.
- Other text is conversational. It can never become a motion command through model-generated output.
  Ask explicitly again after clarifying a variant; MVP1 does not turn a bare `B` follow-up into an action.
- A task requires fresh controller and vision timestamps, known hardware, calibration, a freely pickable variant,
  its destination, a fresh overhead image, and a successful model image check. Changed target poses are rejected.
- Replay is permitted only with `allow_replay:=true` **and** SIMULATION hardware. Real operation requires
  vision's explicit `live: true`. No pick queue, automatic task retry, or remote real-hardware confirmation.
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
6. Request one variant. The request permits approach motion. Physically check the cup and press **Confirm**
   at each descent; the model cannot press it on REAL hardware. Repeat per variant only after inspecting results.

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
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_check.py --variant VARb --chat
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_negative_check.py
# With VARa composite scene:
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run python conversation/sim_cancel_check.py
# Local VLM and generated speech, no robot commands:
PATH="$PWD/.tools:$PATH" pixi run -e train python conversation/benchmark.py
```

Results are under `conversation/results/`. Synthetic speech tests are not live microphone tests; the 30 VLM
turns repeat three prompts on one recorded scene, and the 30 ASR turns repeat ten generated utterances.
The separate policy and ROS-negative suites cover unsupported commands and dispatch boundaries.
These finite checks do not establish general VLM accuracy, physical success, or full PTT-to-speech latency.

Gemma was not benchmarked within this implementation window. Qwen is a working baseline, not a measured winner
over Gemma. Wake-word listening remains MVP2; German remains MVP3.
