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

- [ ] T04 — Person B: Resolve source variant identification and ambiguous detections. Partial evidence: user-run classify_board.py synthetic tests passed for VARa/VARb/VARc across rotations, both windings and cyclic corner ordering; rejected outdated dimensions, large red-board dimensions, unsupported colour, crossed corner order, nonhorizontal corners, NaN coordinates and ambiguous size matches. Execution remained disabled. The 5 mm dimension tolerance is provisional and separate from final-placement tolerance. Adapter evidence: user-run classify_scene.py missing-calibration check returned uncalibrated for all three candidates in the first historical scene; original scene and destination tags stayed unchanged, identities/grasp poses stayed unset, and robot targets stayed empty. Integration evidence: user screenshot confirmed synthetic projection plus classification passed for candidates a/VARa, b/VARb, c/VARc and b2/VARb, including reconstructed metric-coordinate assertions and explicit multiple-VARb reporting. Operational configuration was not changed; original candidate identities/grasp poses remained unset and execution stayed disabled. This is an ideal downward-camera, zero-distortion synthetic consistency check, not physical accuracy evidence. Selection evidence: user screenshot confirmed all ten select_source.py diagnostic checks passed: unique requested match, empty scene, unavailable variant, multiple matches, duplicate IDs, uncalibrated candidate, rejected candidate, rejected image region, missing classification and unsupported variant. Selection remains diagnostic with execution_allowed=false; no freshness or hardware authorization is provided. Adapter and prior checklist committed as 8cd8a55. Pending: integration with current observations, resolving rejected source regions, and physical accuracy validation. Synthetic horizontal-plane checks do not prove a real board is flat when its corners were projected onto an assumed plane.


## Waiting for hardware

- [ ] T06 — Person B: Validate C920 settings and calibration loading. Acceptance: intended resolution/focus and calibrated CameraInfo confirmed; fresh workspace images checked.
- [ ] T07 — Person A: Establish camera-to-robot transform, TCP, surface heights and marker-to-placement offsets. Acceptance: independent measured points validate the geometry.
- [ ] T08 — Person A: Validate physical approach, descent, lift and waiting motions with held-board collision geometry. Requires T07.
- [ ] T09 — Person A: Validate vacuum operation, grasp/release evidence and fault behaviour. Acceptance: command success is distinguished from actual board pickup.
- [ ] T10 — Both: Complete one supervised VARa cycle. Requires T08 and T09. Acceptance: source → profile → retreat → 2-second wait → retrieve → destination.
- [ ] T11 — Both: Run the cycle using current validated perception targets. Requires T04, T07 and T10.
- [ ] T12 — Both: Validate all variants and final placement. Requires T11. Acceptance: record repeated results and measured error against the agreed 5 mm criterion.

## Queued / dependencies

- [ ] T05 — Person A: Prepare calibration and target-generation tools offline. Acceptance: explicit coordinate frames and height handling; unknown physical values remain unset. Partial evidence: user-run plane_geometry.py synthetic checks passed for optical axes, table-plane intersection, raised-plane XY changes, camera translation, and rejection of missing transform/height or a plane behind the camera. perception_geometry.yaml records corrected board/tag dimensions with unmeasured geometry left null. User-run load_perception_geometry.py check loaded 1920x1080 intrinsics and five distortion coefficients; source/profile/destination conversions correctly rejected missing workspace intrinsic validation, camera transform and support heights. User-run estimate_charuco_pose.py found planar pose candidates in 17/17 calibration archive images (16 with 24 corners; one with 9); best-candidate corner RMS ranged from 0.118 to 0.464 px. Evidence: artifacts/charuco-pose/report.json. These are calibration-training-image diagnostics, not independent physical accuracy validation. Visual inspection of uploaded images 001, 006, 008 and 017: corner/axis overlays are plausible in 006/008/017; 001 has visible blur and a horizontal image discontinuity and should not be used as an extrinsic reference. Uploaded report confirms image 008 has close planar candidate errors (0.414 vs 0.686 px); pose ambiguity remains unresolved. Update: independent XY checker and camera-transform composition now have user-run synthetic checks; see Calibration progress below. Pending: metric target generation, measured camera transform, destination heights/offsets, TCP verification and independent physical validation.

- [ ] T13 — Both: Freeze and rehearse the demonstration; prepare slides and backup recording. Requires a clearly documented tested capability.
- [ ] OPTIONAL — AI text interpretation and voice through validated task commands.
- [ ] OPTIONAL — Browser 3D digital twin.
- [ ] OPTIONAL — Wrist D405 integration.

## Done — evidence scope matters

- [x] T03 — Person B: Show perception in the browser. Requires T02. Acceptance: observed tags/candidates are clearly separate from simulated job state. Evidence: scene_bridge.py serves historical replay at http://127.0.0.1:8082/api/scene; user screenshots verified waiting, receiving with IDs 1/2/16, and stale after publisher shutdown (183.719 seconds since receipt), retaining the last scene with execution_allowed=false. UI integration: demo_web.py polls the scene API once per second in a separate Historical replay panel. The UI session reports browser fixture checks passed for receiving, null, stale, error, HTTP/network failure, malformed JSON, timeout, recovery, and independent job controls; no execution requests issued. Integrated ROS replay verification: user screenshots showed stale at 485.3 seconds with detection lists cleared, then receiving at 6.5 seconds with destination IDs 1/16/2 and two white plus one red source candidate; source variants remained unverified. Physical/live-camera perception remains pending.

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

## Calibration progress — 2026-10-26

This evidence supplements T05; physical calibration and target generation remain incomplete. User ran all reported code and checks. No physical motion is established by these results.

- Independent XY checker: check_workspace_points.py committed as 013148a. User-run synthetic checks passed, including a known 5 mm XY error and rejection of a point declared used for transform fitting. The checker uses supplied measured Z and does not independently test Z accuracy.
- Scene correction: 797770d records measured 40 mm profile thickness and robot mounting face 60 mm above the tabletop. All six packages built successfully; uploaded RViz screenshot showed Global Status: Ok. Rendering/build success does not validate physical contact or the TCP.
- Spacer: centred beneath robot base; inferred height 20 mm. Approximately 150 x 150 mm footprint is unmeasured. Spacer collision geometry remains pending; the team expects it to be outside the intended motion path, but clearance has not been physically verified.
- Transform composition: user-run compose_camera_transform.py synthetic test passed for combined rotation/translation, point-coordinate chaining, and invalid-transform rejection. No measured camera-to-base transform exists. Commit not confirmed in this conversation.
- Perception reference frame: user-run configuration check passed with workcell.frame_id=base_link and identity T_base_link_workcell. Legacy T_workcell_camera consequently denotes camera-to-base_link and remains null. Source/profile conversion correctly rejected missing camera transform and intrinsic workspace validation. Commit not confirmed in this conversation.
- Board-placement template: config/calibration/board_placement.template.json committed as 19a330c. Pose source, measurement method, board axes and transform intentionally remain unset.

### Current geometry contract

| Quantity | Value / status |
| --- | --- |
| Reference frame | Existing URDF base_link; physical axis correspondence still to verify |
| Table/source support Z | -0.060 m; source assumes boards rest directly on tabletop |
| Profile thickness | 0.040 m |
| Profile support Z | -0.020 m |
| Single-board top Z | Source -0.054 m; profile -0.014 m |
| Destination support/marker heights | Unmeasured; remain unset |
| Camera extrinsics | Unmeasured; remain unset |
| Intrinsics | Provisional; workspace validation pending |
| TCP and final placement | Physical validation pending |

The profile visual spans -0.060 to -0.020 m. Its collision box has a deliberate 1 mm underside clearance and preserves the top at -0.020 m. Collision clearance is not a measured surface offset. Old contact waypoints are not validated by the geometry correction.

### Next execution sequence

1. Offline: finish testing metric source-variant classification through pixel-to-plane conversion and scene-level ambiguity handling (T04). The standalone classifier's synthetic checks have passed.
2. On site: secure webcam, verify camera settings/intrinsics, and verify base_link physical axes (T06/T07).
3. Capture the ChArUco board at a measured pose; explicitly identify model origin/axes and account for backing thickness. Compose a candidate camera-to-base transform.
4. Check independent measured points across source, profile and destination regions. Do not reuse points for fitting or choosing the transform and then claim them as independent validation.
5. Measure destination heights and marker-to-placement offsets; verify TCP/contact heights and spacer clearance separately.
6. Generate and inspect calibrated targets, then progress through supervised physical motion, vacuum checks and the VARa cycle (T08-T10).
7. Validate perception-driven execution, all variants and final placement before rehearsal (T11-T13).

