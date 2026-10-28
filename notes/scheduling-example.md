# Scheduling example for the VLM / agent (real robot, 2026-10-28)

How the main controller runs two jobs in parallel, recorded on the real FR5 (`tools/schedule_log.py`, timeline below).
For VLM fine-tuning / skill design: **the agent only decides WHAT and the PRIORITY; the controller decides the ORDER.**

## The task (voice)

> "Process variant A and drop off in drop-off zone A with high priority. Transfer one variant B to drop-off zone B
> with low priority."

## What the agent sent (both at once, within the same second)

| Job | Command (agent bridge / `tc`) | = AddJob fields |
|---|---|---|
| j1 | `process VARa processing_1 drop_off_a 1 10` | type process, select_variant VARa, zone processing_1, to drop_off_a, count 1, priority 10 |
| j2 | `transfer VARb drop_off_b 1 5` | type transfer, select_variant VARb, to drop_off_b, count 1, priority 5 |

## What happened (real robot, t = seconds after the jobs were added)

| t (s) | Robot | Why |
|---|---|---|
| 0 - 34 | j1: pick VARa, place it on processing_1 | highest priority job, its board available |
| 34 | j1 -> **WAITING** (board is being processed), **j2 starts at once** | a waiting job does not block: the controller takes the next runnable job |
| 34 - 68 | j2: pick VARb, place it on drop_off_b | meanwhile the operator processed VARa: processing_1 PROCESSING (t 44) -> DONE (t 61) |
| 68 | j2 delivered; **j1 resumes first** (prio 10 > 5) | the processed board is ready, higher priority |
| 68 - 107 | j1: pick VARa from processing_1, place it on drop_off_a | |
| 107 | both DONE (1 delivered each) | |

The robot was never idle while work was possible (gaps between actions 0.1 - 0.2 s). The VLM run that left the robot
idle until job A was completely done had serialised the jobs: job B was only added after job A finished.

## Rules for the VLM / agent

1. **Submit all jobs of an instruction at once.** Never wait for a job to finish before adding the next one; the
   controller interleaves them (a job waiting for processing lets the others run).
2. **Express urgency as `priority`** (higher runs first, ties: queue order), not as submission order or delays.
   "High / low priority" -> e.g. 10 / 5. Priorities can be changed later (`prio JOB N`).
3. **One job per instruction part**: "process X then drop-off A" = ONE `process` job with `zone` + `to` (not a process
   job plus a separate transfer job). "Transfer one B" = `transfer` with `count 1` (count = boards to DELIVER; no
   count given -> 1; 0 = all matching).
4. **Read the outcome from the job, not from the robot motion**: a job is finished when `delivered == count` or state
   DONE (`delivered`, `to_fault`, `lost`, `skipped` counters; `message` gives the reason when partial).
5. **Do not micro-manage skills** (pick / place / home): only jobs + control commands (`pause`, `resume`, `stop`,
   `cancel`, `prio`). The controller handles retries, failures and the processing hand-over.
6. Names: variants `VARa` (large white), `VARb` (small white), `VARc` (red), `UNKWN` (never picked unless the board id
   is named); zones `processing_1`, `drop_off_a`, `drop_off_b` (from the camera, see `tc world`); the fault zone is
   automatic.

## Notes from this run

- The last place (VARa on drop_off_a) reported NOT_VERIFIED (the camera did not confirm it within 3 s): logged as a
  warning, the job continues and counts it delivered (by design).
- j2 shows DONE only when the controller finishes its current action (here at t 107), although it delivered its
  board at t 68: check `delivered == count`, not only the state.

## Timeline (recorder output)

| clock | t (s) | kind | what |
|---|---|---|---|
| 10:14:48 | - | AGENT | voice task: 'process variant A -> drop-off A, high priority; transfer one variant B -> drop-off B, low priority'. Decision: submit BOTH jobs at once (never wait for job 1): process VARa via processing_1 -> drop_off_a count 1 prio 10; transfer VARb -> drop_off_b count 1 prio 5. The controller interleaves: while VARa is in processing (operator), the transfer runs; the processed VARa (prio 10) goes first when DONE |
| 10:14:48 |     0.0 | JOB | j1 added: process VARa via processing_1 -> drop_off_a count 1 prio 10 |
| 10:14:48 |     0.0 | JOB | j1 QUEUED (delivered 0, fault 0, lost 0, skipped 0) |
| 10:14:48 |     0.0 | EVENT | info: job j1 added: process 1 x VARa in processing_1 -> drop_off_a (prio 10) |
| 10:14:48 |     0.0 | JOB | j1 RUNNING (delivered 0, fault 0, lost 0, skipped 0) |
| 10:14:48 |     0.0 | ACTION | start pick job j1 board b1 (to_zone) |
| 10:14:48 |     0.0 | JOB | j2 added: transfer VARb -> drop_off_b count 1 prio 5 |
| 10:14:48 |     0.0 | JOB | j2 QUEUED (delivered 0, fault 0, lost 0, skipped 0) |
| 10:14:48 |     0.0 | EVENT | info: job j2 added: transfer 1 x VARb -> drop_off_b (prio 5) |
| 10:15:04 |    16.3 | ACTION | idle |
| 10:15:04 |    16.3 | EVENT | info: j1: picked b1 (to zone) |
| 10:15:04 |    16.4 | ACTION | start place job j1 board b1 -> processing_1 |
| 10:15:16 |    27.7 | ZONE | processing_1 LOADED count 0 board b1 |
| 10:15:22 |    34.4 | ACTION | idle |
| 10:15:22 |    34.4 | EVENT | info: j1: placed b1 in processing_1 |
| 10:15:22 |    34.5 | JOB | j1 WAITING (delivered 0, fault 0, lost 0, skipped 0) |
| 10:15:22 |    34.5 | JOB | j2 RUNNING (delivered 0, fault 0, lost 0, skipped 0) |
| 10:15:22 |    34.5 | ACTION | start pick job j2 board b2 (to_target) |
| 10:15:32 |    44.3 | ZONE | processing_1 PROCESSING count 0 board b1 |
| 10:15:38 |    49.9 | ACTION | idle |
| 10:15:38 |    49.9 | EVENT | info: j2: picked b2 (to target) |
| 10:15:38 |    50.1 | ACTION | start place job j2 board b2 -> drop_off_b |
| 10:15:49 |    61.1 | ZONE | processing_1 DONE count 0 board b1 |
| 10:15:51 |    62.9 | ZONE | drop_off_b OK count 1 |
| 10:15:56 |    67.8 | JOB | j2 RUNNING (delivered 1, fault 0, lost 0, skipped 0) |
| 10:15:56 |    67.8 | ACTION | idle |
| 10:15:56 |    67.8 | EVENT | info: j2: placed b2 in drop_off_b |
| 10:15:56 |    67.9 | JOB | j1 RUNNING (delivered 0, fault 0, lost 0, skipped 0) |
| 10:15:56 |    67.9 | ACTION | start pick job j1 board b1 (to_out) |
| 10:16:07 |    79.2 | ZONE | processing_1 EMPTY count 0 |
| 10:16:10 |    82.1 | ACTION | idle |
| 10:16:10 |    82.1 | EVENT | info: j1: picked b1 (to out) |
| 10:16:10 |    82.3 | ACTION | start place job j1 board b1 -> drop_off_a |
| 10:16:27 |    98.9 | ZONE | drop_off_a OK count 1 |
| 10:16:35 |   106.9 | JOB | j1 RUNNING (delivered 1, fault 0, lost 0, skipped 0) |
| 10:16:35 |   106.9 | ACTION | idle |
| 10:16:35 |   106.9 | EVENT | warn: place b1 failed: NOT_VERIFIED - placed b1, but the camera does not confirm it at the target |
| 10:16:35 |   106.9 | EVENT | warn: b1: placed but not confirmed by the camera - continuing |
| 10:16:35 |   106.9 | EVENT | info: j1: placed b1 in drop_off_a |
| 10:16:35 |   106.9 | JOB | j1 DONE (delivered 1, fault 0, lost 0, skipped 0) - 1 delivered of 1, 0 to fault, 0 lost, 0 skipped (1 attempts) |
| 10:16:35 |   106.9 | JOB | j2 DONE (delivered 1, fault 0, lost 0, skipped 0) - 1 delivered of 1, 0 to fault, 0 lost, 0 skipped (1 attempts) |
| 10:16:35 |   106.9 | EVENT | info: job j1 DONE: 1 delivered of 1, 0 to fault, 0 lost, 0 skipped (1 attempts) |
| 10:16:35 |   106.9 | EVENT | info: job j2 DONE: 1 delivered of 1, 0 to fault, 0 lost, 0 skipped (1 attempts) |
