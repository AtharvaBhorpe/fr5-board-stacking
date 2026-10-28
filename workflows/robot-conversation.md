# Robot conversation and task assistance

Status: CONFIRMED by user; MVP1 implemented on feat/robot-conversation. See conversation/README.md and conversation/results for verification and remaining limits.

## Intent

Use multimodal conversation and reasoning with the existing FR5 board-handling stack, informed by the Gemma duck demonstration and possible Qwen3-VL alternatives.

## Development branch and scope

- All implementation belongs on new branch `feat/robot-conversation`, created from `feat/vision1-overhead` at `45b6a9f`, in the `vision1-overhead` checkout.
- This is the LLM Agent in the supplied team architecture image. The existing panel and pick script serve as the current main control loop; calibrated vision, wrist depth, movement, and gripper controllers retain their responsibilities.
- No implementation on `feat/vision1-overhead` or `feat/robot-motion-gripper`. If changes to the robot workspace become necessary, use a separate feature branch there as well.
- No merge or push is included in this specification task.

## Existing integration surface

- Observations: overhead images and `/vision/boards` JSON; wrist camera images and controller-owned D405 depth measurements.
- Execution: jobs for the main controller (feat/task-main-controller) through its agent bridge (conversation/task_link.py).
- Requests: `/pick_panel/request` accepts `VARa`, `VARb`, `VARc`, `stop`, `confirm`. Real-hardware confirmation is restricted to the panel button.
- Feedback: `/pick_panel/log`.

## Decision tree

1. SETTLED: scene/status conversation plus assisted pick requests (option B). Multi-step task orchestration is outside the initial outcome.
2. SETTLED: RViz text and push-to-talk voice with a visible transcript.
3. SETTLED: an explicit pick request may start the existing sequence, including approach motion; existing manual descent confirmations remain.
4. SETTLED: local-only initially; bounded commands with clarification; no task queue or automatic retries; short spoken responses plus text.
5. SETTLED: English through MVP2, German added in MVP3; warm response targets of 3 seconds to first useful response and 5 seconds to complete a short answer; images plus structured observations, with target disagreements withholding dispatch.
6. SETTLED: session-only conversation memory and the proposed offline/mock, latency, and supervised hardware acceptance checks. Consolidated scope confirmed. Choose runtime/model through implementation evidence.

## Accepted MVP1 behavior

- Run locally after initial downloads. No cloud inference in MVP1.
- Support one pick by explicit variant, status/scene questions, and stop. Clarify ambiguous references such as "that white board"; reject unsupported counts and motion instructions rather than silently reducing them to one pick.
- While a pick runs, allow conversation and stop, but reject another pick instead of queueing it. After a failure, report the failure and wait for explicit user direction; no autonomous retry/recovery.
- Speak short answers and task transitions, with detailed logs remaining in text. Push-to-talk interrupts speech output; it does not itself cancel robot motion. Keep the direct Stop button independent of inference.

## Possible MVP2 — deferred

- User wants always-listening conversation with a trigger/wake word, possibly starting in MVP2.
- MVP1 remains push-to-talk. Do not implement background microphone monitoring yet.
- Wake word, listening-window duration, follow-up turns, false-activation handling, and interruption behavior must be specified before MVP2 implementation.

## Accepted language and perception behavior

- English speech and text through MVP2; add German in MVP3. Display VARa/VARb/VARc verbatim and verify spoken variant-name recognition explicitly.
- Show listening/transcribing/thinking states immediately. Target first useful response within 3 seconds and a complete short response within 5 seconds after release on warm inference, measured alongside the existing vision workload. These are targets, not demonstrated performance.
- Use timestamped overhead snapshots plus structured detections/status by default, with wrist imagery for relevant questions. Images support explanation; calibrated vision and the controller remain authoritative for actionable geometry. If imagery and detections disagree about a requested target, explain and ask for review rather than guessing or overriding detection.

## Candidate shapes for later comparison

- A: one conversational sidecar reads snapshots, returns answers or bounded task requests, and uses the existing panel interface. Recommended starting candidate; no second motion controller.
- B: a dedicated task orchestrator owns queued multi-step jobs and invokes the existing robot sequence. Consider only if the chosen workflow actually requires multiple picks, cancellation across steps, and correlated results.

Current recommendation: A. The accepted single-pick scope and no-queue policy do not justify B. This is a single-agent assessment, not a completed multi-model architect review.

## Execution boundaries

- Calibrated perception and the controller own metric targets, depth, and motion parameters.
- Image interpretation can explain observations but cannot establish hidden board identity, exact stack count, or physical clearance by assertion.
- Execute only against revalidated current observations; distinguish historical/replay images from live evidence.
- Report observed controller outcomes; never equate a generated answer with task completion.
- Keep the existing direct Stop control available independently of model inference.

## Accepted memory and acceptance criteria

- Keep conversation context in memory for the current session, cleared on restart or an explicit Clear chat action. Do not save microphone recordings or camera snapshots in the conversation layer. Existing robot logs remain under their existing policy. Clearing chat does not cancel an active robot task.
- Remember dialogue, but obtain fresh observations for every actionable request. A previous successful pick or an old frame is not evidence of current availability.
- Validate on recorded scenes and mock execution first, then an explicitly supervised real-robot trial. Offline/mock success must not be reported as real-robot validation.
- Minimum offline acceptance set: 30 English text/voice cases covering each variant, paraphrases, ambiguous white boards, unavailable targets, unsupported counts/motion requests, busy state, stale/replay observations, visual disagreement, stop, inference failure, and controller failure. Every case has an expected answer category and expected dispatch count before the run.
- Require zero wrong-variant, duplicate, unsolicited, stale/replay-derived, or remote-confirm dispatches in that set. Any such failure blocks readiness; this finite test is not a general safety guarantee.
- Measure at least 30 warm turns with the existing vision workload active: target p95 first useful response <=3 s and complete short response <=5 s, maintaining the configured 5 Hz vision workload without sustained slowdown. Report cold-start and failure timing separately. A missed target is reported, not silently waived.
- Physical validation: one supervised pick/place per variant through the conversation interface; verify manual descent checkpoints and controller-reported completion. Check cancellation in mock first and under a separately agreed supervised hardware procedure. Do not intentionally provoke hazardous hardware failures.
- Before implementation, present the consolidated MVP1 specification for final shared-understanding confirmation, as required by the invoked grilling skill.

## Implementation decisions to resolve without another product interview

- Select model/runtime through a Gemma versus Qwen3-VL comparison using the same scenes, commands, and resource limits; do not assume small-model latency or recognition quality.
- Define bounded task messages and correlate requests to controller outcomes; the current log topic alone is not a reliable structured completion contract.
- Use acquisition timestamps and source provenance for freshness, fail closed for missing actionable evidence, and recheck before dispatch. Preserve controller checks at execution time.
- Keep cancellation and direct panel controls responsive independently of inference. A stop request is not a claim that the robot has physically stopped; report confirmation or uncertainty from controller evidence.
- Implement timeouts and reject late model results after cancellation or changed task state. Do not execute model-generated motion parameters or shell commands.
