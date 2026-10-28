# MVP1 verification — 2026-10-27

Historical MVP snapshot. The development session described below has ended. Subsequent
[Gemma/Qwen simulation results](sim-comparison/REPORT.md) and
[VLM/Phonon comparison](comparison/REPORT.md) supersede the Gemma and session-status notes here.

Implemented on `feat/robot-conversation`, based on vision commit `45b6a9f`. No real hardware commanded.

- Policy/unit checks: **62 passed** (command parsing, refusal responses, stale/replay/invalid observations, model-result validation).
- Live ROS negative cases: **10 passed, zero task dispatches**, including negation, ambiguous targets, unsupported counts/motion and remote confirmation.
- RViz-panel → MoveIt mock pick/place: **VARa, VARb, VARc passed**.
- Chat → local Qwen image check → RViz → MoveIt mock pick/place: **VARa, VARb, VARc passed**.
- Stop during an accepted action: controller returned a result; script reported stopped. Replaying its task ID did not start another task.
- VLM: **30 turns**, complete-response p95 **0.63 s**; 20/20 positive image checks agreed on the recorded fixture.
- ASR: **30 synthetic utterances**, p95 **0.34 s**, 30/30 parsed intent categories matched, zero wrong pick actions. Some transcripts were inaccurate but remained non-actionable.
- Concurrent GPU vision: 200 messages over a 40-second observation window, measured delivered rate 5.12 Hz, maximum inter-message gap 0.93 s. This run overlapped the earlier prompt-version benchmark; final prompt-version benchmark also ran with GPU vision active but had no second dedicated rate trace.
- Native panel compiled successfully. Qt-widget screenshot inspected; OpenGL scene capture was blank in that mechanism, so no screenshot claim about the 3D scene. MoveIt completion is supported by controller/task results.
- Runtime health reports Qwen on **cuda:0, BF16**. Measured GPU process allocation approximately 4.65 GB Qwen and 650 MB vision. Whisper base.en is CPU int8.

## Limits / not passed yet

- Live microphone/operator voice and lab noise are untested. PTT plumbing is implemented; generated WAV ASR is tested separately.
- Full PTT-release-to-spoken-answer p95 is unmeasured. Component timings do not prove the full 3/5-second acceptance target.
- VLM benchmark repeats three prompts on one scene; ASR repeats ten synthetic utterances. This is not a broad accuracy evaluation or a general guarantee against wrong actions.
- No Gemma comparison, wake word, German, physical validation or real-robot cancellation test.
- Final action-cancellation code was exercised by the VARc end-to-end and cancellation runs; earlier VARa/VARb success runs preceded that change.
- A final conversational-history refusal rule was added after the end-to-end runs and checked by the final policy and ROS-negative suites; the pick path was unchanged.
- Model weights are in temporary storage and must be redownloaded after reboot unless the cache is relocated.

## Current interactive session

The isolated simulation remains open on localhost ROS domain 61. Try `status`, `what do you see?`, then `pick variant B` in RViz. Confirm descents manually.
Before starting real hardware, stop this development session with:

```bash
python /tmp/stop-conversation-dev.py
```

This stops only the recorded simulation/vision/RViz/bridge PIDs after checking their command lines. The local model server remains available.
Follow `conversation/README.md` for the real-hardware handoff and clean launch commands.
