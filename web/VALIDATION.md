# Web workcell validation — 2026-10-28

## Verified

- Production Vite build; existing 102 conversation/controller tests pass.
- Headless gateway running with MoveIt mock hardware on domain 62, no RViz.
- Desktop 1512×982 and mobile-width 390×844 Chrome screenshots reviewed.
  Ten loaded robot meshes, twelve URDF joints, live overhead JPEG, no horizontal
  overflow and no browser JavaScript/resource errors.
- Browser typed request “pick two variant B boards” completed both actual
  MoveIt mock pick/place cycles, including Gemma scene gates and source removal.
  Four Phonon browser synthesis responses succeeded; browser playback ran.
- Variant A button accepted after scene reset; Stop task interrupted the pick
  and the headless controller reported stopped. Demo restored to two B boards.
- Browser AudioWorklet captured synthetic microphone audio, produced mono
  16-bit 16 kHz WAV, and released it to the transcription API. This test used an
  ASR response stub and did not send robot commands. Pointer cancellation
  discarded audio without submitting a transcript.
- Offline service worker shell loads; pick, sort and microphone controls disable.
  Offline input is not replayed when connectivity returns.
- Missing token, cross-origin control and invalid control requests rejected.
  Mesh traversal rejected; authorized model mesh served successfully.
- HTTPS certificate validated against the generated project-local CA.

## Pending / limits

- The restart cleared the old `/tmp` model cache. Parakeet TDT v3 is downloading
  again over a slow network. Typed Gemma chat and Phonon output are available
  during background loading; microphone enables only when ASR is ready.
- Complete browser-microphone -> actual Parakeet -> robot integration is pending
  that download. Run `cd web && node tests/voice.mjs` when ready.
- Physical phone/tablet certificate trust, installation, microphone quality and
  actual-device playback require operator testing. Chrome mobile viewport tests
  are not physical iOS/Android tests.
- Real robot execution is deliberately unavailable in this web MVP. Only
  simulation is enabled. Stop is software cancellation, not emergency stop.
- 3D board Z is a display plane, not a measured contact height. The simulated
  camera world does not model suction contact or destination stack growth.

Machine-readable logs and screenshots are in ignored `web/test-results/`:
`browser.json`, `typed.json`, `cancel.json`, `microphone.json`, `offline.json`.
