# Workcell PWA

A local web replacement for the RViz interaction panel, on branch `feat/robot-conversation`.
This version executes **simulation only**, on ROS domain 62. RViz is not required.
Downloaded models are retained in the normal Hugging Face cache. Background ASR
loading keeps typed chat and TTS available; the microphone enables when ready.
It uses the existing `robot/pick_one.py`, MoveIt mock hardware, vision pipeline,
Gemma scene checks, bounded conversation scheduler, Parakeet, and Phonon 40M.

## Start

Close the existing RViz demo first. From the vision1-overhead directory:

```bash
conversation/gemma_qat.sh
# Separate terminal; add --download if the model has not been downloaded:
conversation/model.sh --backend llama --asr parakeet --download --background-asr
# Separate terminal:
npm --prefix web ci
web/run.sh
```

Open http://localhost:8088. “Install app” appears when the browser offers PWA
installation; the browser's install menu also works. The shell is cached, but
all state and commands require the running laptop. Commands are never queued
for offline execution. Models, robot control, and vision run on the laptop.

## Phone and tablet

Use the same reachable Wi-Fi network. `web/run.sh` creates a local CA and a
90-day HTTPS server certificate covering the laptop's current addresses.
It does not modify any system certificate store. Private keys remain in ignored
`web/.local/`, with owner-only file permissions.

1. On the laptop, choose **Connect a device**. Download the CA certificate and
   transfer it to the phone (or download it from the HTTPS page after reviewing
   the browser's certificate warning).
2. Install and trust `web/.local/ca.crt` on your own phone. On iOS, install the
   profile under General → VPN & Device Management, then enable full trust under
   General → About → Certificate Trust Settings. Android's certificate-install
   menu varies by vendor.
3. Copy the paired link from the laptop and open it on the phone. It contains a
   private session token in the URL fragment, removed after pairing. Do not share
   it with others. Tokens rotate each time the gateway restarts.
4. Enable voice replies, grant microphone permission, and hold **Hold to talk**.
   Audio is captured by that device, transcribed locally on the laptop, and
   replies play in that device's browser. Releasing sends the utterance; a cancelled
   touch or switching away cancels the recording. Recordings stop after 20 seconds.
5. Use Add to Home Screen / Install. Remove the local CA when testing ends.

Network isolation on guest/public Wi-Fi can prevent phone access. Restart after
an address change to renew the server certificate. Physical iOS/Android trust,
microphone, and installation require testing on the actual device.

## Interfaces

- `gateway.py`: authenticated same-origin HTTP API, JPEG camera snapshots, URDF
  meshes, live joint states, chat/log history, and a headless pick-process owner.
- `src/main.js`: Three.js URDF view, bounded controls, browser microphone, and
  ordered browser audio replies. Enable voice replies per browser session.
- `run.sh`: launches MoveIt simulation, synthetic image world, actual GPU vision,
  conversation bridge with browser speech, and gateway. It refuses an existing
  robot publisher to avoid duplicate controllers.
- `/api/state` and all control/audio/mesh endpoints require a paired token.
  Plain HTTP binds only to loopback; TLS binds to the LAN on port 8443.
- Execution is rejected unless hardware identifies as SIMULATION and joint,
  vision, calibration, destination, and controller checks pass. Stop requests
  SIGINT cancellation of the pick script, not a physical emergency stop.

3D board height is a display plane, not a depth measurement. The mock world
verifies source removal but does not model suction physics or destination stacks.
The wrist feed appears when that topic is present; it is absent in this demo.
This app does not replace RViz's full planning-scene editing or debugging tools.

## Checks

```bash
npm --prefix web run build
npm --prefix web test
python3 web/tests/gateway.py
# With services running; executes two simulated picks from synthetic microphone audio:
(cd web && node tests/voice.mjs)
```

Browser screenshots and results live in ignored `web/test-results/`. Automated
Chrome checks do not establish real phone microphone or headset quality.
