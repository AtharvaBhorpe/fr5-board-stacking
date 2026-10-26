# FR5 execution board

Working budget: 12 hours, as reported by the team.
Current access: at home; no robot workspace access.
Workflow: one laptop, one branch, sequential work.
Branch: feat/fr5-demo-integration.
Move a card to Done only after recording evidence.
Doing limit: one implementation task.
Owners below are suggested.

## Ready next


## Doing

## Waiting for hardware

- [ ] T06 — Person B: Validate C920 settings and calibration loading. Acceptance: intended resolution/focus and calibrated CameraInfo confirmed; fresh workspace images checked.
- [ ] T07 — Person A: Establish camera-to-robot transform, TCP, surface heights and marker-to-placement offsets. Acceptance: independent measured points validate the geometry.
- [ ] T08 — Person A: Validate physical approach, descent, lift and waiting motions with held-board collision geometry. Requires T07.
- [ ] T09 — Person A: Validate vacuum operation, grasp/release evidence and fault behaviour. Acceptance: command success is distinguished from actual board pickup.
- [ ] T10 — Both: Complete one supervised VARa cycle. Requires T08 and T09. Acceptance: source → profile → retreat → 2-second wait → retrieve → destination.
- [ ] T11 — Both: Run the cycle using current validated perception targets. Requires T04, T07 and T10.
- [ ] T12 — Both: Validate all variants and final placement. Requires T11. Acceptance: record repeated results and measured error against the agreed 5 mm criterion.

## Queued / dependencies

- [ ] T03 — Person B: Show perception in the browser. Requires T02. Acceptance: observed tags/candidates are clearly separate from simulated job state.
- [ ] T04 — Person B: Resolve source variant identification and ambiguous detections. Acceptance: distinguish VARa/VARb by validated geometry and VARc by colour; ambiguous scenes do not create execution targets.
- [ ] T05 — Person A: Prepare calibration and target-generation tools offline. Acceptance: explicit coordinate frames and height handling; unknown physical values remain unset.
- [ ] T13 — Both: Freeze and rehearse the demonstration; prepare slides and backup recording. Requires a clearly documented tested capability.
- [ ] OPTIONAL — AI text interpretation and voice through validated task commands.
- [ ] OPTIONAL — Browser 3D digital twin.
- [ ] OPTIONAL — Wrist D405 integration.

## Done — evidence scope matters

- [x] T02 — Person A: Connect perception to ROS image input with recorded-image replay. Acceptance: incoming images produce timestamped scene observations, including missing detections. Evidence: artifacts/replay-check/verified-scenes.json — all seven historical images passed expected tag/candidate counts and replay metadata checks in the user-run validation. Robot execution remains disabled; live-camera reliability and physical accuracy remain pending.
- [x] Shared branch, Pixi build and fake bringup verified.
- [x] Obsolete camera post removed. Commit: 1e12342.
- [x] C920 acquisition and provisional intrinsic calibration completed. Physical workspace validation remains T06.
- [x] Offline destination detection: IDs 1/2/16 detected in 7/7 historical images using contrast preprocessing.
- [x] Offline source candidate detection implemented; misses/extra candidates remain unresolved. Commit: 257bf81.
- [x] Simulated coordinator: complete cycle, 2-second wait and injected grasp-failure handling demonstrated. Commit: 180a688.
- [x] Exact text commands accepted for VARa/VARb/VARc; VARd rejected. Commit: 7c53f00.
- [x] Fake MoveIt, fake gripper and recorded fake joint waypoint checks demonstrated. Commits: 61665c4, 1e30917, df2fae2.
- [x] Full fake ROS coordinator integration passed. Synthetic pos1/pos2 geometry and observations. Commit: f5aa6a5.
- [x] Browser submission, progress and invalid-command rejection demonstrated. Source commit remains T01.
- [x] T01 — Person B: Commit the tested scripts/demo_web.py. Acceptance: browser UI is tracked in Git; record commit.

