# Workspace workflow notes

## Robot conversation workflow — 2026-10-27

- User wants to understand the Gemma-powered duck conversation pattern and whether Gemma or Qwen3-VL can provide multimodal conversation and reasoning for this stack.
- User invoked architect, ponytail, then loop-me. Current work is workflow specification; implementation choices remain open.
- Reference: [Build Overhead Camera Vision 1](t3-thread://v1/42da2fb6-3384-4664-bab5-eefeeec1f620). History read through position 1715.
- `vision1-overhead` contains both overhead C920 vision and wrist D405 vision in one Pixi workspace. `fr5-board-stacking` supplies robot motion and gripper services.
- Local GPU checked: NVIDIA RTX 5060 Laptop GPU, 8151 MiB VRAM.
- Overhead output: `/vision/boards`, `/vision/detect`, `/vision/image`. JSON includes timestamp, source, calibration status, boards, occluded boards, rejected candidates, and destinations.
- `robot/pick_one.py` owns the supervised sequence and wrist depth use for pick and destination heights.
- `rviz_panel/src/pick_panel.cpp` accepts variant requests, stop, and confirm on `/pick_panel/request`; it refuses topic-based confirmation unless hardware is MOCK. A variant request can initiate motion before the later descent confirmation.
- `/pick_panel/log` provides textual feedback. It is not a correlated task-result API.
- Canonical terms: conversation = user dialogue; observation = timestamped sensor evidence; task request = asking the existing controller to execute; motion target = metric pose owned by the controller.
- Round 1 accepted: scene/status conversation plus explicit pick requests; RViz text and push-to-talk with visible transcript; requests may start the sequence while preserving existing manual descent confirmations.
- Round 2 accepted: local-only initially; one pick by variant/status/scene/stop, clarify ambiguity; no queue or automatic retry; short spoken answers and task updates plus full text; push-to-talk interrupts speech but not robot motion.
- User requested wake-word/always-listening conversation as a possible MVP2. MVP1 is push-to-talk; background listening is deferred.
- Round 3 accepted: English through MVP2, German in MVP3; target first useful response within 3 seconds and complete short answer within 5 seconds; overhead image plus detections/status, relevant wrist images, and withhold pick requests on target disagreement.
- Round 4 accepted: session-only memory without saved audio/images; offline/mock cases, latency checks, and supervised per-variant hardware validation.
- User requires all development on a new branch. Created `feat/robot-conversation` from vision branch commit `45b6a9f`; moved these notes and the workflow spec into that checkout.
- User confirmed the consolidated scope and requested MVP1 simulation testing within 30 minutes. Implementation and evidence are in conversation/README.md and conversation/results/REPORT.md.

## Research status

- The found Gemma Robot Duck demo appears to concern Open Duck Mini, distinct from Pollen Microduck. The official video was located but its transcript and exact integration code were not retrieved. Exact demo internals remain unverified.
- Official Gemma docs confirm Gemma 4 E2B/E4B text, image and audio inputs with text output and thinking support. Qwen's Qwen3-VL-4B-Instruct model card documents image/text conversation and video understanding. These capabilities do not establish robot accuracy or local latency.
- No inference benchmark, model installation, robot connection, or robot command was performed for this assessment.
- Architect's default review model configuration is absent; capability discovery did not list the requested model IDs and Grok is disabled. No multi-model review was run.
