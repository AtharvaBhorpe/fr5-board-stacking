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

- [ ] T04 — Person B: Resolve source variant identification and ambiguous detections. Partial evidence: user-run classify_board.py synthetic tests passed for VARa/VARb/VARc across rotations, both windings and cyclic corner ordering; rejected outdated dimensions, large red-board dimensions, unsupported colour, crossed corner order, nonhorizontal corners, NaN coordinates and ambiguous size matches. Execution remained disabled. The 5 mm dimension tolerance is provisional and separate from final-placement tolerance. Adapter evidence: user-run classify_scene.py missing-calibration check returned uncalibrated for all three candidates in the first historical scene; original scene and destination tags stayed unchanged, identities/grasp poses stayed unset, and robot targets stayed empty. Integration evidence: user screenshot confirmed synthetic projection plus classification passed for candidates a/VARa, b/VARb, c/VARc and b2/VARb, including reconstructed metric-coordinate assertions and explicit multiple-VARb reporting. Operational configuration was not changed; original candidate identities/grasp poses remained unset and execution stayed disabled. This is an ideal downward-camera, zero-distortion synthetic consistency check, not physical accuracy evidence. Selection evidence: user screenshot confirmed all ten select_source.py diagnostic checks passed: unique requested match, empty scene, unavailable variant, multiple matches, duplicate IDs, uncalibrated candidate, rejected candidate, rejected image region, missing classification and unsupported variant. Selection remains diagnostic with execution_allowed=false; no freshness or hardware authorization is provided. Adapter and prior checklist committed as 8cd8a55. Offline command evidence: user-run inspect_source.py requested VARa from the first historical scene and returned status=blocked, reason=uncalibrated_candidates, candidate=None and execution_allowed=false. Report: artifacts/source-inspection/first-VARa.json. Pending: integration with current observations, resolving rejected source regions, and physical accuracy validation. Synthetic horizontal-plane checks do not prove a real board is flat when its corners were projected onto an assumed plane.


## Waiting for hardware

- [ ] T06 — Person B: Validate C920 settings and calibration loading. Acceptance: intended resolution/focus and calibrated CameraInfo confirmed; fresh workspace images checked.
- [ ] T07 — Person A: Establish camera-to-robot transform, TCP, surface heights and marker-to-placement offsets. Acceptance: independent measured points validate the geometry.
- [ ] T08 — Person A: Validate physical approach, descent, lift and waiting motions with held-board collision geometry. Requires T07.
- [ ] T09 — Person A: Validate vacuum operation, grasp/release evidence and fault behaviour. Acceptance: command success is distinguished from actual board pickup.
- [ ] T10 — Both: Complete one supervised VARa cycle. Requires T08 and T09. Acceptance: source → profile → retreat → 2-second wait → retrieve → destination.
- [ ] T11 — Both: Run the cycle using current validated perception targets. Requires T04, T07 and T10.
- [ ] T12 — Both: Validate all variants and final placement. Requires T11. Acceptance: record repeated results and measured error against the agreed 5 mm criterion.

## Queued / dependencies

- [ ] T05 — Person A: Prepare calibration and target-generation tools offline. Acceptance: explicit coordinate frames and height handling; unknown physical values remain unset. Partial evidence: user-run plane_geometry.py synthetic checks passed for optical axes, table-plane intersection, raised-plane XY changes, camera translation, and rejection of missing transform/height or a plane behind the camera. perception_geometry.yaml records corrected board/tag dimensions with unmeasured geometry left null. User-run load_perception_geometry.py check loaded 1920x1080 intrinsics and five distortion coefficients; source/profile/destination conversions correctly rejected missing workspace intrinsic validation, camera transform and support heights. User-run estimate_charuco_pose.py found planar pose candidates in 17/17 calibration archive images (16 with 24 corners; one with 9); best-candidate corner RMS ranged from 0.118 to 0.464 px. Evidence: artifacts/charuco-pose/report.json. These are calibration-training-image diagnostics, not independent physical accuracy validation. Visual inspection of uploaded images 001, 006, 008 and 017: corner/axis overlays are plausible in 006/008/017; 001 has visible blur and a horizontal image discontinuity and should not be used as an extrinsic reference. Uploaded report confirms image 008 has close planar candidate errors (0.414 vs 0.686 px); pose ambiguity remains unresolved. Update: independent XY checker and camera-transform composition now have user-run synthetic checks; see Calibration progress below. Pending: metric target generation, measured camera transform, marker-plane heights and destination-offset integration, TCP verification and independent physical validation.

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

### Latest physical measurements — short workspace access

- Latest user correction: ALL source boards and destination template sheets are 5 mm thick, superseding the earlier 6 mm assumption for all variants. Source-board top Z is therefore -0.055 m and profile-board top Z is -0.015 m. A single 5 mm destination template directly on the tabletop would have support Z=-0.055 m and a placed 5 mm board top Z=-0.050 m; user subsequently confirmed all templates rest directly on the tabletop without extra layers. Earlier synthetic fixtures using 6 mm remain mathematical test evidence, not current physical geometry. User screenshot confirms the configuration update completed: VARa=[0.295, 0.210, 0.005], VARb/VARc=[0.135, 0.210, 0.005]; the script also updated the legacy scene board entry to VARa. User subsequently ran the destination-height update and reported support=-0.055 m and placed-board top=-0.050 m for VARa/VARb/VARc. Marker-plane heights remain unset; paper/adhesive thickness is unmeasured. Camera distortion coefficient 0.006891 is unrelated and was not targeted by the update.

- User measured the centred robot spacer as 155 x 155 x 20 mm. This supersedes the earlier approximate, unmeasured 150 x 150 mm footprint. Spacer collision geometry has not yet been added.
- User confirmed corrected destination offsets from tag centre to intended board centre, straight along each template's length away from the tag, with zero sideways offset: VARb/tag 2=150 mm, VARc/tag 16=140 mm, VARa/tag 1=150 mm. These supersede the earlier 140/130/140 mm values. The black crosses are NOT intended drop locations. These were initially template-relative measurements; signed local-frame offsets have now been stored as described below. Direction mapping was subsequently established in the orientation evidence below. Desired board yaw is now stored as documented below; marker-plane heights remain pending and no placement targets are generated. Destination support heights were subsequently confirmed and populated.
- Inspected both uploaded tool/ruler photos. The relaxed cup lip is around the 11 cm ruler region, but the ruler-zero reference relative to the modeled flange/adapter face is not established precisely. No TCP change or physical TCP validation is justified by these photos alone. Template thickness is now confirmed as 5 mm, directly on the tabletop.

### Current geometry contract

Desired board orientation: user confirmed the 210 mm side runs along each template away from its tag. User-supplied placed-board reference image 264854f7-Image_1.png is visually consistent: VARa's 295 mm side runs across the template; VARb/VARc's 135 mm sides run across their templates. Define board +X along size_m[0], +Y along the 210 mm side away from the tag, +Z up. Combined with the observed tag directions, desired board yaw relative to the local marker frame is +pi/2 for tags 1/2 and -pi/2 for tag 16. User-run configuration update confirmed board yaw +90 degrees for tags 1/2 and -90 degrees for tag 16. Board orientation is stored; no TCP pose or robot target was generated. Physical placement remains unvalidated. Photo documents intended arrangement only, not measured placement error, automated execution or TCP orientation.

Destination orientation evidence: user-run corner annotation detected IDs 1, 2 and 16 in their respective phone photos. Direct inspection of artifacts/destination-tag-directions/VARa-directions.png, VARb-directions.png and VARc-directions.png shows the template direction toward intended placement is -U for VARa/tag 1, -U for VARb/tag 2, and +U for VARc/tag 16, where U follows detected corner 0 toward corner 1. Combining this visual direction association with user-confirmed distances gives longitudinal offsets -0.150 m, -0.150 m and +0.140 m respectively, with zero sideways offset. These signs describe the corner-derived local direction, not base_link X. User-run configuration update confirmed stored offset_xy_m values: tag 1=[-0.150, 0], tag 2=[-0.150, 0], tag 16=[+0.140, 0]. Convention: origin at black-square centre, +X along reconstructed corner 0 to 1, +Z base_link up, +Y=Z cross X. board_yaw_rad was subsequently populated as documented below; physical_placement_validated remains false for every entry; execution remains disabled. Existing perception code does not yet consume these entries. Metric corner reconstruction, marker-plane heights and physical placement verification remain pending; desired board orientation is now stored. Destination support heights were subsequently populated as -0.055 m. Do not use black crosses as targets.

Overlap ROI check: user-run image 04 (18-15-23.984036) with ROI [1250, 330, 1840, 980] produced two white candidates, zero red candidates and five rejected regions. Uploaded JSON and overlay confirm rejected merged white-board and red-board outlines remain inside the ROI; the other three rejected regions correspond visually to robot parts. Destination IDs 1/2/16 were each detected once. Evidence: artifacts/scenes-roi-overlap-check/01-Photo from 2026-10-25 18-15-23.984036.json and matching PNG. This confirms rejection for this overlap example, not general overlap detection. The robot still enters this ROI in this pose; final observation pose/ROI validation remains pending. No detector thresholds or default ROI were changed.

Historical source-region review: inspected scene JSON for all seven captures and annotated images 01, 04, 05 and 06. Image 01 rejected contours include the robot body and wrist; image 04 includes visibly overlapping white/red boards, so its rejection must not be tuned away. Image 05 contains additional visible boards as well as robot contours, so four white candidates are not established false positives. Image 06 shows three separated source outlines with two rejected robot regions at the left ROI boundary. User-run ROI [1250, 330, 1840, 980] test on image 06 passed the intended diagnostic: two white candidates, one red candidate, zero rejected regions, destination IDs 1/2/16 with no missing/unexpected/duplicate tags. Uploaded JSON and annotated image confirm all three source outlines remain within the search region. Evidence: artifacts/scenes-roi-check/01-Photo from 2026-10-25 18-16-17.901466.json and matching PNG. Variant identities and grasp poses remain null; metric_calibrated=false and execution_allowed=false. This ROI is specific to the historical capture; no current workspace ROI or physical classification is validated. Uploaded first-VARa.json is the earlier 18-14-31 scene with the original ROI and still correctly reports uncalibrated_candidates; it is not an inspection of the new ROI result. Screenshot confirms inspect_source.py committed as adb2cd0.

| Quantity | Value / status |
| --- | --- |
| Reference frame | Existing URDF base_link; physical axis correspondence still to verify |
| Table/source support Z | -0.060 m; source assumes boards rest directly on tabletop |
| Profile thickness | 0.040 m |
| Profile support Z | -0.020 m |
| Single-board top Z | Source -0.055 m; profile -0.015 m, using corrected 5 mm board thickness |
| Destination support Z | -0.055 m for all three templates; user configuration output confirmed |
| Placed-board top Z | -0.050 m for one 5 mm board on a template |
| Destination marker plane Z | Unset; paper/adhesive height approximation still unresolved |
| Camera extrinsics | Unmeasured; remain unset |
| Intrinsics | Provisional; workspace validation pending |
| TCP and final placement | Physical validation pending |

The profile visual spans -0.060 to -0.020 m. Its collision box has a deliberate 1 mm underside clearance and preserves the top at -0.020 m. Collision clearance is not a measured surface offset. Old contact waypoints are not validated by the geometry correction.

### Next execution sequence

1. Offline: finish testing metric source-variant classification through pixel-to-plane conversion and scene-level ambiguity handling (T04). The standalone classifier's synthetic checks have passed.
2. On site: secure webcam, verify camera settings/intrinsics, and verify base_link physical axes (T06/T07).
3. Capture the ChArUco board at a measured pose; explicitly identify model origin/axes and account for backing thickness. Compose a candidate camera-to-base transform.
4. Check independent measured points across source, profile and destination regions. Do not reuse points for fitting or choosing the transform and then claim them as independent validation.
5. Integrate the stored destination offsets with metric marker geometry; resolve marker-plane heights and apply the stored board orientation, then verify TCP/contact heights and spacer clearance separately.
6. Generate and inspect calibrated targets, then progress through supervised physical motion, vacuum checks and the VARa cycle (T08-T10).
7. Validate perception-driven execution, all variants and final placement before rehearsal (T11-T13).

