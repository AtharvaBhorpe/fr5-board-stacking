# Bounded multi-board scheduling

Commands accepted through the existing text or push-to-talk interface:

- `Pick two variant B boards`
- `Pick 3 VARc`
- `Sort all visible boards`
- `Status`
- `Stop` / `Cancel`, or the always-available **Cancel batch** button

The plan is a snapshot of the initially visible, calibrated source boards (maximum ten).
Sort order is A, then B, then C; no boards added later are appended. Source stacks and
occluded boards are not inferred. Missing destinations, uncertain dimensions,
out-of-calibration source boards, and overlapping/duplicate detections block a batch.

Each pick uses the existing fresh-image VLM check, pose-change check, and deterministic
single-pick controller. Only one task is dispatched at a time. A matching controller
completion must be followed by three distinct fresh perception frames showing exactly
one fewer source board of the picked variant and unchanged other source counts. Failure
to verify this within ten seconds ends the batch without retry. This verifies source
removal, not destination placement accuracy. Static replay cannot satisfy this check.

Controller rejection/failure, cancellation, stale controller status, missing acknowledgement,
changed hardware identity, model refusal, and perception failure discard the remainder.
Cancellation sends the existing Stop request; it is not a physical emergency stop.
Clearing chat or starting a new PTT capture invalidates in-flight work and prevents batch
continuation; it does not cancel a controller motion already running. Use Stop for that.

Progress appears in chat and on `/conversation/batch_state` as JSON:
`total`, `completed`, `phase`, `remaining`, `reason`.
Each batch also carries a unique `id` for correlation.
Phases are checking, executing, verifying, completed, failed, and stopped.
Spoken progress uses ordered narration: "Picking the first board", "First board placed",
"Picking the second board", and a final completion message. Placement wording reflects
controller completion plus source-removal verification, not measured placement accuracy.
Normal speech is queued in order; cancellation or a new PTT capture interrupts stale speech.
A second batch or pick request is rejected while the worker is active.

The English recognizer uses counted-board vocabulary and beam search. Only voice transcripts
matching a complete counted pick command can repair the observed final-noun errors "volts" or
"bolts" to "boards". The original and interpreted text are shown in chat. Numbers and variants
are never supplied or changed by this repair; unrelated and negated text stays unchanged.

## Validation and use

The real robot's running bridge was not replaced during this implementation. Batches on
REAL FR5 are disabled by default. Simulation bridges accept them normally. After supervised
batch validation, the explicit deployment option is:

```bash
ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 pixi run conversation --ros-args -p enable_real_batches:=true
```

Do not run this alongside an existing conversation bridge. Depth calibration remains
pending; the fixed single-board pick setting is not a general stack-height solution.

Checks:

```bash
python3 -m unittest discover -s conversation/tests
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 pixi run python conversation/check_batch_ros.py
```

The ROS integration check uses real ROS topic transport and the actual bridge worker with
a simulated controller/perception source and an explicit VLM stub. It checks two-pick
completion, stop-on-controller-failure, cancellation, and refusal to repeat on an unchanged scene.
It does not launch MoveIt, evaluate VLM accuracy, or validate physical batch execution.
Evidence is written to `results/batch-ros.json`.

## Interactive MoveIt demo

`conversation/demo.sh` uses domain 62 with fake hardware and a changing, labeled test image.
The normal GPU vision detector, Gemma QAT image checks, RViz dispatcher, and MoveIt execute the
batch. Use **Reset 2 B** or **Reset A/B/C** before a new request. No live cameras are opened.

While that demo is idle, run a reproducible end-to-end check:

```bash
ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 pixi run python conversation/check_batch_moveit.py counted
# Other cases: mixed, cancel, unchanged, missing
```

These checks perform actual MoveIt mock trajectories. The simulated image removes boards on
successful task completion; it does not model contact, vacuum physics, destination stack growth,
or placement accuracy. See `results/batch-moveit/` for recorded outcomes.
