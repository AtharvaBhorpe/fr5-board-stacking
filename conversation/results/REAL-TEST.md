# First supervised real-robot test — 2026-10-27

Operator confirmed being at the physical emergency stop, a clear workspace, and a visible variant B board.
Simulation and replay processes were stopped before real bringup. One robot-description publisher was
verified; the panel reported REAL FR5. Live C920 data was calibrated and explicitly `live: true`; the D405
was streaming. Gemma QAT/llama.cpp and local Phonon were used with the normal guarded conversation bridge.

1. **Hover-only at 10% speed passed.** The real controller reached the hover pose, and D405 measured
   approximately 99.1 mm cup-to-board gap. No descent or grip was requested in this stage. The operator
   confirmed the cup was centered and the path below clear. Evidence: `real-hover-varb.json`.
2. VARb disappeared from overhead detections while the arm was above it. The agent returned the arm to
   the previously exercised camera-clear pose at 10%; the controller reported success. Live detection
   then reacquired VARb at approximately x=-0.196, y=-0.431 m, inside the calibrated region. No different
   variant was substituted.
3. **Supervised VARb pick/place completed.** The chat request passed the Gemma image check and dispatched
   through the RViz panel to the existing deterministic pick script, with `--speed 0.1`. Both descent
   confirmations came from the operator's local RViz button; no remote confirmation was sent.
4. Controller evidence: pick descent succeeded; vacuum achieved in 0.61 s; lift and carry succeeded;
   placement descent succeeded; release succeeded with a 2-second blow-off; empty lift and park succeeded.
   The script exited 0. Elapsed time including human confirmation waits was 65.24 s.
5. **Operator visually confirmed clean release and correct placement** on the variant B destination template.

Evidence: `real-pick-varb.json` plus operator visual confirmation. This establishes one successful real
VARb cycle, not repeatability, measured placement accuracy, or physical validation of A/C.
The request was injected as text on the conversation topic;
it was not an end-to-end voice-triggered robot test. The user separately reported that PTT works with
their connected earphones; the built-in laptop microphone issue remains unresolved.

Phonon pronunciation now expands VARa/VARb/VARc to “variant A/B/C” only in spoken text. The bridge reports
“task completed” so hover-only completion is not falsely announced as completed pick/place.

## Automatic pick descent follow-up

After the operator confirmed variant B was back in the pick area and hands were clear, one real cycle
ran at `--speed 0.1`. Hover proceeded directly to the 103 mm pick descent, grip, lift, and carry.
Vacuum was achieved in 0.60 s. The only prompt was placement, confirmed by the operator's RViz button.
Release, empty lift, and park succeeded; exit code 0, elapsed 54.94 s including placement confirmation.
Final gripper state was IDLE with vacuum and blow-off off. Evidence: `real-pick-varb-auto-descent.json`.

D405 measured the pick gap at 99.1 mm but returned no destination-template measurement; placement used
the existing `--stack` fallback. The operator visually confirmed clean release and correct placement
for this automatic-descent cycle. This is a second successful real variant B cycle; only this second
cycle exercised automatic pick descent.
The running bridge still spoke the old confirmation wording during this test; the source wording had
already been corrected. This was a text-triggered test, not an end-to-end voice test.

## Fixed single-board pick-height diagnostic

The subsequent fully automatic wrist-height pick failed with NO_VACUUM. The operator reported a centered
cup stopping approximately 2 mm above the board. Motion logs recorded a 101.2 mm descent to z=-0.0561 m;
earlier successful picks ended near z=-0.0582 m. This supports a height-estimation problem but is not a
calibration measurement. The operator confirmed a single flat source board, clear destination, hands clear,
and emergency-stop readiness for one diagnostic cycle.

After returning the empty arm to the previously tested camera-clear pose at 10% speed, live calibrated
VARb was reacquired. The diagnostic used `--speed 0.1 --fixed-pick-height --press 3`: nominal pick target
z=-0.0580 m and 103 mm descent, while retaining wrist placement measurements. Vacuum was achieved in
0.57 s. Carry, automatic placement descent, release, empty lift, and park all succeeded without confirmation
pauses; exit 0, elapsed 48.61 s. Final gripper IDLE, vacuum/blow-off off. The wrist measured the destination
stack top at -0.0540 m. Evidence: `real-pick-varb-fixed-height.json`. Operator visual placement confirmation
was supplied: the operator confirmed clean release/correct placement and audible spoken updates, while
noting that the release appeared higher than an earlier test. This was text-triggered, not a full
voice-triggered validation. This opt-in diagnostic applies
only to the confirmed single flat board and does not validate source stacks or fix wrist calibration.

Missing audio was traced to Handy holding the ALSA speaker device, causing PipeWire's 'Device or resource
busy' error. With explicit permission Handy was closed; the speaker node then ran, its monitor recorded
non-silent Phonon output, and the operator confirmed voice was audible.
