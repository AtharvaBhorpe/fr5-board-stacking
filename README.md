# FR5 board stacking — perception branch and team handover

**Branch:** `feat/perception-geometric-detector` · **Handover:** 28 October 2026 · **Final model:** YOLO11s-seg, cloud run `yolo_cloud_001`

This branch contains the camera, dataset, training, evaluation and live perception
work for the FR5 board-sorting demonstration. The deployment path is now
**YOLO11s instance segmentation on an NVIDIA GPU**. Earlier geometric detection
and Grounding DINO/SAM2 experiments remain available, but they are not required
to run the YOLO workflow.

The tested interface to motion provides **board class, snapshot-local instance
ID, centre `(u, v)` in the original Logitech image, and long-axis image angle**.
The fresh-snapshot service waits for a newly captured image after the request.
Perception does not itself send trajectories, operate suction, or publish
independently validated robot poses.

![Historical workcell overview](docs/images/perception/hardware-workcell.jpg)

*Historical arrangement, before later board/bin/ROI changes. The current search
covers both table halves through a full-image ROI; see its limitations below.*

## What is ready and what is not

| Capability | Current status |
|---|---|
| Load the final four-class checkpoint on CUDA | Tested |
| Markerless board instance segmentation | Tested offline and through the ROS interface |
| Original-image centre pixels and image angle | Tested, with supported-outline filtering |
| Multiple same-class proposals | Separate IDs in one snapshot; no persistent tracking |
| Continuous output plus fresh-snapshot ROS service | Tested |
| Stale/repeated timestamp rejection and worker-failure recovery | Tested with isolated replay |
| 24–30 coordinate updates/second | **29.7/s in a 20-second saved-image ROS replay**; live camera rate still needs checking |
| Debug overlay at 24–30 FPS | No; fast mode deliberately limits it to **1 image/second** |
| Every touching/overlapping board separated correctly | **Not solved** |
| Reliable human-safety stop from hand predictions | **Not validated; brown fixtures cause false hands** |
| Automatic pixel-to-robot XYZ/yaw conversion | Requires separately validated geometry and motion integration |
| Autonomous suction verification, placement and obstacle avoidance | Motion/task-layer responsibility, not established by these detections |
| Speech/VLM selection | Planned integration; not part of the tested YOLO launch |

Do not describe the model as 98% accurate or every green rectangle as a verified
pick. The evaluation below contains concrete class, overlap and background
failures. A supported image quadrilateral does not prove a unique topmost board.

## Contents

1. [Quick start on another laptop](#quick-start-on-another-laptop)
2. [Final weights and provenance](#final-weights-and-provenance)
3. [Architecture and environments](#architecture-and-environments)
4. [Classes, hardware and full-table ROI](#classes-hardware-and-full-table-roi)
5. [Camera commissioning](#camera-commissioning)
6. [Live inference, viewers and shutdown](#live-inference-viewers-and-shutdown)
7. [ROS interface for motion](#ros-interface-for-motion)
8. [Metric calibration and pick/place integration](#metric-calibration-and-pickplace-integration)
9. [Offline testing and regression checks](#offline-testing-and-regression-checks)
10. [Measured accuracy, latency and limitations](#measured-accuracy-latency-and-limitations)
11. [Capture, CVAT, review and augmentation](#capture-cvat-review-and-augmentation)
12. [Training and evaluating a replacement](#training-and-evaluating-a-replacement)
13. [Troubleshooting](#troubleshooting)
14. [Team handover checklist](#team-handover-checklist)
15. [Photo and results galleries](#photo-and-results-galleries)
16. [Repository map and further reading](#repository-map-and-further-reading)

## Quick start on another laptop

The reference machine was Ubuntu 24.04 with an RTX 4070 Laptop GPU and 8 GB VRAM.
Use Linux x86-64 with a working NVIDIA driver. CPU-only, AMD-only and Mac/MPS
deployment are not implemented by this CUDA-specific adapter. Cameras and the
robot are not needed for the first offline checks.

### 1. Clone the feature branch and submodules

```bash
git clone --branch feat/perception-geometric-detector --single-branch \
  --recurse-submodules https://github.com/AtharvaBhorpe/fr5-board-stacking.git
cd fr5-board-stacking
git status --short --branch
```

For an existing clean checkout already on this branch:

```bash
git fetch origin
git pull --ff-only origin feat/perception-geometric-detector
git submodule update --init --recursive
```

Preserve local edits before updating. Do not reset another person's work. All
commands in this README run from the repository root, not its parent directory.

### 2. Install Pixi and inspect the driver

Use the [official Pixi installation instructions](https://pixi.sh), then:

```bash
pixi --version
nvidia-smi
```

The `.tools/pixi` bootstrap binary used in earlier chat commands is ignored by Git
and will not exist after a fresh clone. Use `pixi` from PATH. Do not copy the old
laptop's `.pixi` directory or mix system ROS/Jazzy/Conda base into these terminals.

### 3. Install the locked environments

```bash
pixi install --frozen
pixi install --frozen -e camera-ros
pixi install --frozen -e perception-gpu
```

The GPU environment includes the base perception feature. The separate CPU
inspection environment is optional:

```bash
pixi install --frozen -e perception
```

Keep the committed `pixi.lock`. Do not upgrade Torch or Ultralytics to reproduce
these results. Initial downloads depend on the network and can be large. The
prebuilt PyTorch runtime does not require a system CUDA compiler toolkit; it
does require a compatible functioning NVIDIA driver.

### 4. Build the team's ROS packages

```bash
pixi run --frozen build
```

This creates `install/setup.sh`, used by the default environment, and builds the
team's pinned FR5/workcell/gripper packages. It does not move the robot.

### 5. Download and verify the final weights

Download [the final checkpoint](https://github.com/AtharvaBhorpe/fr5-board-stacking/releases/download/perception-yolo11s-cloud-001/fr5-yolo11s-cloud-001-best.pt).
It is a release asset rather than a large binary in Git history. This block reads
the committed manifest, verifies the byte count and SHA256, and installs it at the
path already used by fast mode. It refuses to overwrite a different checkpoint.

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib, json, os, tempfile, urllib.request

root = Path.cwd()
manifest = json.loads((root / "config/yolo_checkpoint.json").read_text())
target = root / manifest["runtime_path"]
expected = manifest["sha256"]
if target.exists():
    assert hashlib.sha256(target.read_bytes()).hexdigest() == expected, \
        "Different weights exist; preserve them before continuing."
    print("Already verified:", target)
else:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as output:
            temporary = Path(output.name)
            with urllib.request.urlopen(manifest["download_url"], timeout=60) as response:
                payload = response.read(manifest["size_bytes"] + 1)
            assert len(payload) == manifest["size_bytes"], "Unexpected checkpoint size"
            assert hashlib.sha256(payload).hexdigest() == expected, "SHA256 mismatch"
            output.write(payload)
        os.link(temporary, target)
        print("Installed verified checkpoint:", target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
PY
```

For a USB/manual transfer, put the same verified file at
`.local/brev_results/yolo_cloud_001/weights/best.pt`, then run:

```bash
sha256sum .local/brev_results/yolo_cloud_001/weights/best.pt
```

Expected SHA256:

```text
67ee9c19255eea65462b4289ebb6b8f3cf0f88347ff5325e5dded77a3da64a58
```

### 6. Verify actual CUDA computation

```bash
pixi run --frozen -e perception-gpu check-cuda
```

Stop if this fails. The adapter refuses silent CPU fallback. The reference runtime
was PyTorch `2.7.1+cu128`. The CUDA version in `nvidia-smi` describes driver support;
it need not equal the bundled Torch runtime's version.

### 7. Run an included example without cameras or robot

```bash
pixi run --frozen -e perception-gpu python scripts/yolo_pixels.py \
  --settings config/yolo_live_fast.json \
  --image docs/samples/scene_02.jpg \
  --output-dir .local/offline_yolo/teammate_smoke_001

xdg-open .local/offline_yolo/teammate_smoke_001/overlay.jpg
python3 -m json.tool .local/offline_yolo/teammate_smoke_001/report.json
```

The output folder must be new; use `_002` on repeat. The CLI writes `overlay.jpg`,
`report.json` and `mask_001.png` etc. The sample JPEG retains its original
1920×1080 pixels. It includes known confusers: successful execution is not an
accuracy pass. See the gallery and per-frame review before choosing demo cases.
The first inference includes GPU/model warmup and can be much slower than steady
state; use the sustained ROS replay below to measure throughput.

### 8. Check and start cameras

After connecting the correct USB cables/hub:

```bash
pixi run --frozen -e camera-ros ros2 topic info /scene/image_raw
```

Reuse an existing single publisher. If none exists, configure the device as below,
then choose ONE of these alternatives in a long-running terminal:

```bash
# Logitech only
pixi run --frozen -e camera-ros scene-camera
```

```bash
# Alternative: Logitech + wrist D405
pixi run --frozen -e camera-ros cameras
```

### 9. Start live YOLO in another terminal

```bash
cd /path/to/fr5-board-stacking
pixi run --frozen live-yolo-fast
```

Wait for `READY`. The GPU model loads once. This command starts perception only;
the motion teammate can run their own commissioned controller independently.

### 10. Request fresh coordinates and view the overlay

```bash
pixi run --frozen read-yolo-pixels
```

In a separate terminal:

```bash
pixi run --frozen -e camera-ros ros2 run image_view image_view --ros-args \
  -r image:=/perception/boards_pixels/debug_image
```

The overlay refreshes at 1 FPS in fast mode. Measure coordinate output separately:

```bash
pixi run --frozen ros2 topic hz /perception/boards_pixels --window 60
```

## Final weights and provenance

| Item | Value |
|---|---|
| Model | YOLO11s-seg, four-class instance segmentation |
| Deploy | `best.pt` from `yolo_cloud_001` |
| Release | [perception-yolo11s-cloud-001](https://github.com/AtharvaBhorpe/fr5-board-stacking/releases/tag/perception-yolo11s-cloud-001) |
| Size | 20,535,524 bytes, approximately 20.5 MB |
| Parameters in the saved unfused model | 10,083,836 |
| Ultralytics / Torch | `8.3.228` / `2.7.1+cu128` |
| Training GPU | NVIDIA L40S 48 GB |
| Training input / batch / seed | 960 / 32 / 42 |
| Requested / completed epochs | 100 / 78, early stopping with patience 20 |
| Best epoch | 58, from the preserved training log |
| Training duration | Approximately 16.6 minutes on that cloud run |
| Dataset | 147 train originals + 588 appearance variants = 735 train; 48 val; 46 test |
| Manifest | [config/yolo_checkpoint.json](config/yolo_checkpoint.json) |

`last.pt` remains a local training artifact and is not needed for deployment. A
stripped saved checkpoint can contain `epoch: -1`; use the log/CSV for historical
best epoch rather than treating `-1` as a failure. Checkpoint weights and runtime
configuration are separate: changing a config does not retrain the model.

Choose the launch deliberately:

| Task/config | Checkpoint and processing | Use |
|---|---|---|
| `live-yolo-fast` / `yolo_live_fast.json` | **Final cloud best**, network 960, FP16, compact geometry | Recommended handover starting point |
| `live-yolo-pixels` / `yolo_live.json` | **Old pilot path**, network 640, full-resolution masks | Historical pilot; does not select final cloud weights |
| `live-b-pixels` | Earlier Grounding DINO/SAM2 settings | Optional legacy experiment |

For a cloud-checkpoint/full-resolution comparison, create another settings file:

```bash
python3 - <<'PY'
from pathlib import Path
import json
settings = json.loads(Path("config/yolo_live_fast.json").read_text())
settings.update(fp16=False, geometry_max_width=0)
path = Path(".local/yolo_cloud_full_resolution.json")
path.parent.mkdir(exist_ok=True)
with path.open("x") as stream:
    json.dump(settings, stream, indent=2)
print(path)
PY

pixi run --frozen ros2 launch config/live_yolo_pixels.launch.py \
  settings:=.local/yolo_cloud_full_resolution.json period_s:=0.01
```

Only run one server on the shared perception topic/service names at a time.

## Architecture and environments

```mermaid
flowchart LR
  L[Logitech: v4l2_camera] -->|Image and CameraInfo| R[Latest-frame ROS receiver]
  R -->|Lossless RAM-backed frame| G[Persistent CUDA YOLO11s-seg worker]
  G --> M[Instance masks and classes]
  M --> Q[Supported outline checks]
  Q --> P[Original-image pixels and axis]
  Q --> U[Separate uncertain proposals]
  P --> T[ROS topic and fresh service]
  T --> C[Motion-side consumer]
  C --> H[Validated mapping and instance selection]
  H --> V[MoveIt and gripper state machine]
  D[D405: realsense2_camera] --> W[Optional wrist depth verification]
  W --> H
```

| Environment | Responsibility | Example |
|---|---|---|
| default | ROS Humble, MoveIt, bridge/service and robot packages | `pixi run --frozen live-yolo-fast` |
| `camera-ros` | Standard camera drivers, viewers, calibrator | `pixi run --frozen -e camera-ros cameras` |
| `perception-gpu` | CUDA/Torch/YOLO, offline evaluation and training | `pixi run --frozen -e perception-gpu check-cuda` |
| `perception` | OpenCV/geometric inspection | `pixi run --frozen -e perception hardware` |

ROS messages cross the default/camera environment boundary normally. The GPU
worker uses a clean environment because it has a different Python runtime. A
lossless frame file and JSON pipe connect it to the ROS bridge. Do not point ROS
Python at GPU site-packages or manually source the environments into each other.

Fast inference does the following:

1. Retain only the newest serialized camera message, then decode it when selected.
2. Preserve its source timestamp/frame and send BGR pixels to the persistent worker.
3. Run YOLO with network input 960 and FP16 on CUDA.
4. Remove network letterboxing on GPU and measure compact 640×360 masks for 1080p input.
5. Check components, region/image boundaries and supported quadrilateral geometry.
6. Restore centres/corners to original 1920×1080 pixel coordinates.
7. Publish accepted observations, separate uncertain proposals and hand predictions.
8. Render a debug image only when subscribed and at most once/second in fast mode.

The model still sees network input 960. Compact mask measurement is not the same
as reducing network input to 640; that smaller-network trial lost quality and was
not selected. CUDA inference, CPU geometry, ROS transport and camera capture are
different timing components. A detector-only FPS is not a live system FPS.

## Classes, hardware and full-table ROI

| Stable interface ID | Name | Old label | Reference dimensions, length × width × thickness |
|---|---|---|---|
| 0 | `VARa` | `large_white` | 295 × 210 × 4 mm |
| 1 | `VARb` | `small_white` | 210 × 152 × 4 mm |
| 2 | `VARc` | `small_red` | 210 × 142 × 4 mm |
| 3 | `hand` | `hand` | No board dimensions |

The final checkpoint stores these canonical names. Importers accept older aliases
and remap IDs by name; arbitrary CVAT class ordering must not be assumed correct.
A predicted size class is not an independent millimetre measurement. Red may be
marker colouring rather than the substrate's base colour. White size-class
confusion is observed in the test set.

Hardware roles:

- **Logitech C920 identified by the current device configuration:** scene context
  and main YOLO input. Early VU0060 label photos did not determine optical calibration.
- **Wrist RealSense D405:** RGB/aligned depth for close-up/height verification.
  Its depth is not automatically fused into the fast Logitech detector.
- **FR5 + suction tool:** controlled by the motion stack, not the pixel adapter.
- **AR0234/Arducam modules:** not included in this deployed pipeline.

`config/perception_full_table.toml` uses a **full-image** pickup region. Both table
halves and visible background are searched. This is not a measured table-edge
polygon, and image-edge-clipped masks may still be rejected. The previous narrow
pickup polygon is not the active YOLO scene config.

Whole-table detection does not distinguish a waiting board from a board already
placed in a destination. The task controller must retain that state. Tagged brown
fixtures can themselves become spurious board/hand masks; a green outline alone
must not trigger a new pickup.

## Camera commissioning

### Discovery, domains and clocks

Camera, perception and motion must use the same ROS domain and compatible
middleware/discovery settings. For a single laptop, if the team has not selected
another domain, use this in each terminal:

```bash
export ROS_DOMAIN_ID=0
unset ROS_DISCOVERY_SERVER
```

Do not copy domain 173/localhost-only settings from the isolated replay into a
multi-computer deployment. Synchronize clocks across machines. Matching topic
names alone does not align camera exposure and measured robot pose.

### Device paths, focus and permissions

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/video*
```

Edit `video_device` in `config/logitech_ros.yaml` to match the actual Logitech.
The current stable path identifies the original C920. Another webcam has another
path; select its real video stream rather than the laptop camera or metadata node.

Current camera settings:

```yaml
pixel_format: YUYV
output_encoding: bgr8
image_size: [1920, 1080]
camera_frame_id: scene_camera_optical_frame
focus_automatic_continuous: false
focus_absolute: 35
```

The YAML does not explicitly fix frame period, exposure or white balance. Check
the live controls rather than assuming they are locked. Focus 35 belongs to the
original lens/setup, not every replacement webcam.

If D405 USB permission is denied, an operator can install the included rule:

```bash
sudo install -m 0644 config/99-realsense-d405.rules \
  /etc/udev/rules.d/99-realsense-d405.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb \
  --attr-match=idVendor=8086 --attr-match=idProduct=0b5b
```

Reconnect if needed and check device-access group membership. This grants device
permissions; it is not a package installation. Test the actual hub/cables with
both cameras running, because bandwidth/power behaviour can differ from individual tests.

### Saved intrinsic calibration on another laptop

The configured camera-info URL reads from `ROS_HOME/camera_info`. Install the
matching saved file there before launching the camera:

```bash
mkdir -p "${ROS_HOME:-$HOME/.ros}/camera_info"
install -m 0644 config/calibration/logitech_scene.yaml \
  "${ROS_HOME:-$HOME/.ros}/camera_info/logitech_scene.yaml"
```

Preserve any calibration already stored there for another camera. Alternatively
set `camera_info_url` explicitly to the intended file. Reuse these intrinsics only
with the matching lens/focus/resolution. Copying a YAML does not validate a new
camera mount or camera-to-robot transform.

### Launch one driver per device

Run each long-lived command in a separate terminal, choosing the needed alternative:

```bash
# Logitech only
pixi run --frozen -e camera-ros scene-camera
```

```bash
# D405 only, if the scene camera is already running
pixi run --frozen -e camera-ros wrist-camera
```

```bash
# Alternative: both cameras together
pixi run --frozen -e camera-ros cameras
```

### Verify publishers, source header and parameters

```bash
pixi run --frozen -e camera-ros ros2 topic info /scene/image_raw
pixi run --frozen -e camera-ros ros2 topic info /wrist/wrist_camera/color/image_raw
pixi run --frozen -e camera-ros ros2 topic info \
  /wrist/wrist_camera/aligned_depth_to_color/image_raw

pixi run --frozen -e camera-ros ros2 topic echo \
  /scene/image_raw --once --field header
pixi run --frozen -e camera-ros ros2 topic echo /scene/camera_info --once

pixi run --frozen -e camera-ros ros2 param get \
  /scene/v4l2_camera focus_automatic_continuous
pixi run --frozen -e camera-ros ros2 param get /scene/v4l2_camera focus_absolute
pixi run --frozen -e camera-ros ros2 param list \
  /scene/v4l2_camera | grep -E 'exposure|white_balance|time_per_frame'
```

The scene image must have `frame_id: scene_camera_optical_frame`; the adapter
rejects another frame. Do not crop/rectify/resize a published stream without
updating its pixel/intrinsic contract.

| Stream | Topic |
|---|---|
| Scene BGR | `/scene/image_raw` |
| Scene intrinsics | `/scene/camera_info` |
| Wrist RGB | `/wrist/wrist_camera/color/image_raw` |
| Wrist intrinsics | `/wrist/wrist_camera/color/camera_info` |
| Wrist depth aligned to RGB | `/wrist/wrist_camera/aligned_depth_to_color/image_raw` |
| Aligned depth intrinsics | `/wrist/wrist_camera/aligned_depth_to_color/camera_info` |

The D405 launch requests 848×480 at 30 FPS, alignment/synchronization enabled,
pointcloud/hole filling disabled. RealSense internal synchronization is not
hardware synchronization with the Logitech. Check depth encoding/units at the
consumer; do not blindly apply the earlier raw SDK scale to ROS depth values.

## Live inference, viewers and shutdown

Recommended:

```bash
pixi run --frozen live-yolo-fast
```

Equivalent explicit launch:

```bash
pixi run --frozen ros2 launch config/live_yolo_pixels.launch.py \
  settings:=config/yolo_live_fast.json period_s:=0.01 max_age_s:=2.0
```

`period_s` is a minimum processing-loop period, not a camera FPS setting. A 5-FPS
camera cannot yield 30 new observations/s by reducing this value. The worker
uses the newest frame rather than accumulating an inference backlog.

`max_age_s` is an error-rejection threshold. The fresh service additionally requires
source time after the request. Motion may need a stricter age budget than the
default 2 seconds; agree that policy with the motion owner.

Useful viewers, each in its own terminal:

```bash
pixi run --frozen -e camera-ros view-scene
```

```bash
pixi run --frozen -e camera-ros ros2 run image_view image_view --ros-args \
  -r image:=/perception/boards_pixels/debug_image
```

```bash
pixi run --frozen -e camera-ros view-wrist
```

```bash
pixi run --frozen -e camera-ros view-depth
```

The raw viewer follows capture; the overlay follows the preview cap. Additional
viewers incur additional image transport/copy work. First measure performance
with only the windows the demo needs.

Prefer Ctrl+C in the corresponding launch terminal. To inspect this process family:

```bash
pgrep -af '[p]ython.*live_b_pixels\.py (serve|worker)'
nvidia-smi
```

YOLO deliberately shares the bridge script name `live_b_pixels.py`. That name
does not imply Grounding DINO/SAM2 is running. Do not use broad `pkill python`
commands that could stop a teammate's motion/controller process.

## ROS interface for motion

| Endpoint | Type | Purpose |
|---|---|---|
| `/perception/boards_pixels` | `std_msgs/msg/String` | Continuous JSON observations, including error packets |
| `/perception/get_boards_pixels` | `std_srvs/srv/Trigger` | New processed image after request, JSON in `message` |
| `/perception/boards_pixels/debug_image` | `sensor_msgs/msg/Image` | Annotated BGR image retaining its original source header |

No custom message package is required. Dense masks are used internally and saved
by offline inspection; there is no separate public dense-mask ROS topic in this
handover. The coordinate messages provide supported corners/contours and metadata.

### Getting a fresh observation

```bash
pixi run --frozen read-yolo-pixels
```

Raw service call:

```bash
pixi run --frozen ros2 service call \
  /perception/get_boards_pixels std_srvs/srv/Trigger '{}'
```

Continuous topic inspection:

```bash
pixi run --frozen ros2 topic echo /perception/boards_pixels
```

The reader's `handoff()` validates timestamp/frame, pixel bounds, angle range,
confidence and identities. It rejects hands inside the board array and uncertain
objects presented as accepted coordinates. Reuse this validation instead of
parsing only two numbers from an unchecked JSON packet.

### Coordinate semantics

- `centre_px = [u, v]`: original image pixels, top-left origin, u right and v down.
  For 1080p input, `0 <= u < 1920`, `0 <= v < 1080`.
- `orientation_image_deg`: long-axis image angle from +u/right, clockwise in image
  coordinates, in `[0, 180)`. This is an axis with 180° ambiguity, not printed-face
  direction and not robot yaw.
- `supported_pixel_corners`: quadrilateral supported by the mask. Its centre is
  the diagonal intersection; perspective projection need not produce a perfect rectangle.
- Enclosing-rectangle fields and `mask_centroid_px` are diagnostics and may differ
  from the supported projected-rectangle centre.
- `instance_id`: unique within this snapshot, **not a persistent track**. Reacquiring
  the same physical board generates another snapshot/instance identity.
- `candidate_id`: identifies that inference's proposal. Numbers can have gaps after
  filtering; they are not physical serial numbers.
- `geometry_image_size` and `geometry_pixel_scale`: disclose compact measurement.
  Fast output is still in original image pixels, not 640×360 pixels.

“Camera frame” identifies the source image here. These are not 3D metres in an
optical frame. Do not put raw `u, v` directly into `PoseStamped.position.x/y`.

### Representative schema

This shortened example is illustrative. The complete saved response is
[example_observation.json](docs/handover_data/example_observation.json); its old
timestamp will correctly fail a live freshness check.

```json
{
  "snapshot_id": "example_snapshot",
  "source_timestamp_sec": 1793170240.140921,
  "source_stamp": {"sec": 1793170240, "nanosec": 140921059},
  "frame_id": "scene_camera_optical_frame",
  "coordinate_system": "raw_image_pixels",
  "model_backend": "yolo11_seg",
  "image_size": [1920, 1080],
  "geometry_image_size": [640, 360],
  "geometry_pixel_scale": [3.0, 3.0],
  "objects": [{
    "instance_id": "example_snapshot:1",
    "candidate_id": "candidate_1",
    "class_name": "VARa",
    "class_id": 0,
    "confidence": 0.9653,
    "centre_px": [1369.86, 828.19],
    "orientation_image_deg": 5.71,
    "outline_kind": "supported_mask_quadrilateral",
    "layer_order_verified": false,
    "instance_isolation_verified": false
  }],
  "hand_count": 4,
  "uncertain_count": 6,
  "human_safety_validated": false,
  "execution_allowed": false
}
```

The continuous packet additionally includes `status`, `hand_detections`,
`uncertain_objects`, rejected proposals, observation age and transfer/worker/pipeline
timings. The reader returns a narrower validated handoff summary.

`execution_allowed: false` is intentional: detection does not certify calibrated
robot execution. It is not a switch to flip to finish commissioning. Motion needs
its own validated coordinate conversion, instance selection and execution policy.

### Motion teammate's integration responsibilities

1. Request new perception when the scene is visible and the robot is ready.
2. Validate freshness and choose the requested class using task state.
3. Resolve conflicting/duplicate proposals and select one unique, graspable top instance.
4. Convert supported centre/corners/axis into a validated robot/workcell frame,
   accounting for the correct support and board height.
5. Plan pregrasp, approach and retreat using MoveIt and the verified TCP/tool model.
6. Execute the existing suction interface and verify the pick with available
   seal/contact feedback or deliberate visual confirmation.
7. Recheck orientation if the board can rotate/slip relative to the cup.
8. Place at the registered destination with current stack height and board-to-tool
   orientation accounted for; verify and update task state.
9. Reacquire before each subsequent pick and invalidate coordinates after scene changes.

MoveIt Servo consumes metric motion commands in its configured reference frame,
not image pixel goals. A validated visual-servo controller can consume this
interface, but the interface is not itself that controller.

## Metric calibration and pick/place integration

Pixel inference can run without robot joints. A physical pick needs geometry:
one RGB pixel describes a ray, not a unique 3D point.

| Route | Required data | Scope |
|---|---|---|
| Planar homography | Matched image/contact points, intrinsics, unchanged camera and plane | Surveyed plane only |
| Camera ray / plane intersection | Intrinsics, verified camera pose, correct surface plane | Correct measured surface height |
| Wrist RGB-D / hand-eye | Valid depth/intrinsics, hand-eye, measured TF at source image time | Visible surfaces with checked depth/registration |

For homography, undistort raw pixels with `cv2.undistortPoints(..., P=K)` before
using `H_undistorted_pixels_to_base_xy`. Transform supported corners/axis endpoints
as well as centre, then compute orientation in the robot plane. Image angle is not yaw.

A tabletop homography does not automatically correct a 4-mm board, growing stack,
raised rail or suspended object. Use the actual object/support height and validate
its error budget. An RGB mask's rectangle does not establish the topmost layer.

### Existing calibration artifacts

| File | Meaning / limitation |
|---|---|
| `config/calibration/logitech_scene.yaml` | Saved 1920×1080 intrinsics, original camera/focus |
| `config/calibration/logitech_intrinsics_validation.json` | Reprojection: median .292 px, max .825 px, 29 checked frames |
| `config/calibration/wrist_handeye_candidate_002.json` | Candidate flange-to-wrist optical transform; 32 fitting + 8 held-out poses |
| `config/calibration/logitech_extrinsics_candidate_001.json` | Candidate scene registration; independent physical accuracy unverified |
| `config/handeye_session.json` | Read-only capture settings |
| `config/sorting_task.json` | Class/bin settings and explicitly incomplete metric prerequisites |

Held-out hand-eye consistency was approximately .53–4.61 mm translation and
.10–.85° rotation. That is not an absolute cup-to-board positioning validation.
Its validation/execution flags remain false.

### Reference survey reminders

- Physically verify the relaxed suction-cup TCP/tool geometry before contact surveys.
- Pendant **flange** XYZ is not cup contact XYZ. Include flange orientation and
  transform through the tool offset.
- The importer handles the documented 180° relationship between pendant base and
  `base_link`; do not negate coordinates twice downstream.
- Earlier **F5 and V3 were on the raised rail**, not the tabletop plane.
- Fit with F points and independently validate V points; fitting residual alone
  is not an independent accuracy check.
- Use [the flange survey guide](docs/pendant_flange_homography.md) for exact import,
  marking and solve steps, including `--exclude F5 V3`. Re-measure duplicate or
  uncertain physical records rather than editing coordinates to make a fit pass.

The current modelled flange offset is 112.9 mm. That model value is not a fresh
physical measurement. Preserve survey frame, tool geometry and measurement provenance.

### Destination tags

Current `config/sorting_task.json` records:

| Class | Destination tag ID |
|---|---:|
| VARa | 1 |
| VARb | 2 |
| VARc | 16 |

Dictionary `DICT_5X5_100`, outer black square 39 mm, reported tag-centre to
placement-centre distance 138–139 mm (nominal 138.5), support 4 mm above tabletop.
Verify these against the final physical bins.

`T_tag_placement` remains null. A distance alone does not give offset direction or
orientation. Complete and validate bin transforms, tabletop pose and stack height
before automatic metric placement. A destination changed between runs must be
reacquired and its valid transform frozen for the current run.

### Run alongside the team's motion stack

Check another laptop with fake hardware first:

```bash
pixi run --frozen demo
```

The existing `pixi run --frozen robot` task starts real hardware. Repository rules
require current-session human confirmation, clear workspace, e-stop supervision
and reduced speed for new motion. Keep perception in a separate process so a
perception restart does not restart robot control.

Do not pendant-jog/free-drive while a ROS driver actively owns the command
interfaces. Coordinate controller ownership with the motion owner. The optional
Fairino command server can conflict with the motion driver's single-client realtime
port; do not start both blindly.

## Offline testing and regression checks

### Unit and standard ROS calibration checks

```bash
pixi run --frozen -e perception-gpu python -m unittest discover -s tests -v
pixi run --frozen -e camera-ros test-ros-calibration
```

At handover, 131 tests were discovered: 130 passed in the GPU environment and one
camera-calibration integration test skipped there because its package belongs to
`camera-ros`. That test then passed in `camera-ros`. All 131 were exercised in
their appropriate environments. This is not 131 physical pick/place trials.

### Isolated 30-FPS replay with an included crowded image

```bash
pixi run --frozen python scripts/check_yolo_ros_replay.py \
  --settings config/yolo_live_fast.json \
  --image docs/samples/benchmark_crowded.png \
  --samples 15 --min-boards 1 --input-fps 30 --min-update-fps 24 \
  --stream-seconds 20
```

It publishes saved frames in localhost-only domain 173, starts the persistent GPU
worker and sends no robot/camera command. It checks:

- Sustained coordinate cadence separately from sequential service calls.
- Positive accepted board output and hands excluded from board coordinates.
- New source images after requests, unique snapshot identities and debug headers.
- Repeated source timestamps do not produce repeated “new” observations.
- Frozen/stopped camera data fails freshness checks.
- Reconnection recovers output.
- Killing this test worker invalidates results instead of serving cached boards.

New run folders appear under `.local/yolo_ros_replay_*`, with log, observations
and `result.json`. Passing recovery checks does not override a failed FPS threshold.
See [the recorded final run](docs/handover_data/speed_replay.json).

### Process all 13 teammate JPEGs with one model load

These original files are included under `docs/samples/`. They have no supplied
ground-truth annotation masks, so this is inference/visual review, not scored accuracy.

```bash
pixi run --frozen -e perception-gpu python - <<'PY'
from pathlib import Path
import json, sys, tomllib
import cv2
sys.path.insert(0, str(Path("scripts").resolve()))
from yolo_pixels import YoloModels, draw_hands
from b_live_geometry import measure, draw

settings = json.loads(Path("config/yolo_live_fast.json").read_text())
config = tomllib.loads(Path(settings["scene_config"]).read_text())
model = YoloModels(settings)
output = Path(".local/offline_yolo/sim_suite_001")
output.mkdir(parents=True, exist_ok=False)
for path in sorted(Path("docs/samples").glob("scene_*.jpg")):
    image = cv2.imread(str(path))
    assert image is not None, path
    report, accepted, _, _ = model.detect(image, config, include_masks=False)
    objects = [measure(candidate, mask) for candidate, mask in accepted]
    uncertain = [measure(candidate) for candidate in report["uncertain_candidates"]]
    report.update(source_image=path.name, objects=objects, uncertain_objects=uncertain)
    (output / (path.stem + ".json")).write_text(json.dumps(report, indent=2))
    visual = draw(image.copy(), uncertain + objects)
    draw_hands(visual, report["hand_detections"])
    assert cv2.imwrite(str(output / (path.stem + ".jpg")), visual)
    print(path.name, "coordinates:", len(objects), "uncertain:", len(uncertain))
print("Results:", output.resolve())
PY
```

Use a new output folder on repeat. Proposal counts are not accuracy percentages.
For scoring, use annotated masks/classes and compare precision, recall, mask IoU
and supported centre/angle errors.

## Measured accuracy, latency and limitations

### Full-resolution standard mask metrics

| Split | Images | Mask mAP50 | Mask mAP50–95 |
|---|---:|---:|---:|
| Validation | 48 | 86.06% | 73.03% |
| Test | 46 | 75.98% | 57.91% |

At confidence .25 and class-correct mask IoU >= .5:

| Test class | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|
| VARa | 98 | 40 | 89 | 71.0% | 52.4% |
| VARb | 69 | 29 | 33 | 70.4% | 67.6% |
| VARc | 72 | 8 | 24 | 90.0% | 75.0% |
| hand | 78 | 18 | 18 | 81.25% | 81.25% |

These compare exported annotation rows, which can include fragments/near-duplicates.
Nearby frames are correlated, and some test images appeared in earlier pilot
investigation. Do not present these as a pristine independent industrial trial
or a physical grasp success rate.

### Fast-mode tradeoff

| Test class | Full-resolution TP / FP / FN | Fast TP / FP / FN |
|---|---|---|
| VARa | 98 / 40 / 89 | 94 / 44 / 93 |
| VARb | 69 / 29 / 33 | 69 / 29 / 33 |
| VARc | 72 / 8 / 24 | 70 / 10 / 26 |
| hand | 78 / 18 / 18 | 78 / 18 / 18 |

For matched annotations with supported quadrilaterals, full-resolution centre
error was 2.34 px median / 6.47 px p95 (71 comparisons); fast was 5.09 px / 8.51 px
(68 comparisons). Image-angle error was .39° / 1.70° versus .63° / 1.94°. The
subsets differ. These are annotation-pixel errors, not robot millimetres.

### Throughput

| Measurement | Result and scope |
|---|---|
| Final crowded ROS replay | 29.7 coordinate updates/s over the full 20-second window |
| Median detector time | 18.8 ms in that crowded run |
| Source-to-result age | 35.1 ms median, 48.2 ms p95 |
| Empty synthetic 1080p replay | 29.8 updates/s, zero board coordinates |
| Debug preview | Capped at 1 image/s in fast mode |
| Actual Logitech + USB + inference | Needs hardware verification |

The speed changes crop CPU mask work, skip unused appearance statistics, avoid
full-HD mask transfer, use FP16/RAM transfer, and separate camera reception from
blocking service work. No TensorRT export or new CUDA toolkit is required for
these measurements. Faster processing does not resolve model class/layer failures.

### Teammate ZIP: concrete failures

All 13 JPEGs ran in both modes. Each mode produced 115 raw board proposals in
total; full resolution emitted 55 supported-coordinate proposals and fast 54.
Those are not true-positive or unique-physical-board counts.

Visual inspection found:

1. Ten false hand masks on brown fixtures across nine images.
2. Brown fixtures mistaken for boards, and some red masks including brown backing.
3. Duplicate VARa/VARb coordinates on one white board in `scene_01` in full-resolution mode.
4. Offset/perpendicular overlap masks remaining merged or uncertain.
5. Some obvious isolated rotated boards withheld by outline checks.

See [the visual report](docs/handover_data/sim_scenes_report.md) and
[per-frame CSV](docs/handover_data/sim_scenes_visual_review.csv). These samples were
not used for training or threshold tuning during this evaluation.

Before treating coordinates as picks, resolve conflicting proposals, use task
state to distinguish fixtures/placed boards, and withhold unresolved overlaps.
Analogous correctly labelled brown-fixture negatives/red-on-brown cases are useful
future training data. Globally lowering confidence/geometry limits does not solve
selecting the wrong instance or layer.

## Capture, CVAT, review and augmentation

### Save camera images with S and preserve metadata

With the existing scene driver publishing, run:

```bash
pixi run --frozen -e camera-ros python scripts/capture_board_dataset.py \
  --scene 0 --notes "Final camera mount; new board arrangements and lighting"
```

Focus the UI window. **S** saves the displayed original frame, **N/P** switch the
intended guide and **Q** exits. The terminal prints the session and saved file
paths. A resized preview does not resize saved pixels. Verify the first saved
image instead of assuming that opening the window established capture.

If wrist and robot feedback are available, add their optional capture flags:

```bash
pixi run --frozen -e camera-ros python scripts/capture_board_dataset.py \
  --scene 0 --with-wrist --with-robot \
  --notes "Final setup; preserve associated wrist/depth/robot feedback"
```

Typical session:

```text
.local/datasets/webcam_capture_.../
  images/                         # convenient scene-image upload folder for CVAT
  samples/capture_.../
    capture_....png               # original scene pixels
    metadata.json                # source time, header, camera info, notes, hashes
    wrist_rgb.png                # optional, if a nearby sample exists
    wrist_depth.npy              # optional original array; unit status in metadata
  configuration_snapshot/        # configuration/calibration copies and hashes
```

The tool preserves source headers/timestamps, resolution, image hash, observed
camera parameters when available, nearby CameraInfo, optional wrist/joint/TF data,
and explicit missing-data warnings. Timestamp association within 100 ms is not
hardware synchronization. Camera parameters are observed periodically, not
physically remeasured on every image. Intended guide layouts/group suggestions
are not annotations or measured object poses.

### Scenes that address current failures

Capture distinct physical arrangements rather than a long burst of the same one:

- Each class alone across table positions and rotations.
- Large/small white boards in the same frame, including similar apparent sizes
  at different depths in the oblique camera view.
- Same-colour touching, offset stacks and perpendicular overlap, with both layer orders.
- Partly exposed lower boards, top-board corners and edges, and realistic robot occlusions.
- Red marker streaks/coverage, pale red, shadows, glare and normal lighting variation.
- Actual hands entering/leaving and partly covering boards, labelled separately.
- Brown tagged fixtures, cables, robot links and empty table as background confusers.
- Boards placed on destination fixtures and already-placed boards with task context retained.

Fully hidden boards cannot be recovered from the RGB image. Do not invent their
visible masks. A board exposing more than some percentage is not automatically
graspable; visible suction area and layer/identity ambiguity matter separately.

### CVAT annotation steps

1. Use image instance segmentation with exactly `VARa`, `VARb`, `VARc`, `hand`.
   Old aliases can be imported, but new exports should use canonical names.
2. Upload the session's `images/` files; retain originals and metadata locally.
3. Use the available SAM assistant to propose **one physical instance** at a time.
   Correct prompts or masks when two touching boards become one mask.
4. Label visible board surfaces. Exclude the robot, hands, background and visible
   lower-board parts from a top board's mask.
5. A lower board can have a separate visible mask when its identity is known.
   Keep the visible-mask policy consistent; do not draw a full hidden rectangle
   through an occluding robot or a different board.
6. Label each actual hand with `hand`. Brown cardboard, sleeves and tags are not
   hands. Fixed destination fixtures stay background unless explicitly defined
   as target substrates.
7. Save, reload and inspect. Every target in an image must be checked before
   declaring that image completely reviewed.
8. Export **Ultralytics YOLO Segmentation 1.0 with images enabled**. Preserve a
   CVAT backup and preferably a COCO master too. CVAT XML is not the YOLO export.
9. Inspect exported polygons: disconnected masks may be joined awkwardly by
   format conversion. Automatic masks do not certify label correctness.

### Merge new exports, open the browser tool and build

The input names below are examples; replace them with the actual downloaded ZIPs.
All output names are deliberately new to preserve older review and training runs.

```bash
mkdir -p .local/datasets/cvat_exports

pixi run --frozen python scripts/prepare_yolo_pilot.py merge \
  --zip .local/datasets/cvat_exports/export_a.zip \
  --extra-zip .local/datasets/cvat_exports/export_b.zip \
  --output-zip .local/datasets/cvat_exports/combined_001.zip

pixi run --frozen python scripts/prepare_yolo_pilot.py inventory \
  --zip .local/datasets/cvat_exports/combined_001.zip \
  --plan .local/datasets/cvat_exports/review_draft_001.json

pixi run --frozen -e perception-gpu python scripts/review_yolo_plan.py \
  --zip .local/datasets/cvat_exports/combined_001.zip \
  --plan .local/datasets/cvat_exports/review_draft_001.json \
  --output-plan .local/datasets/cvat_exports/review_completed_001.json
```

Open the printed localhost URL; the port is selected automatically. Keep that
terminal running until the plan is saved. Old chat/browser URLs will not work
after restarting the server.

In the browser:

1. Select images from one physical board arrangement.
2. Enter a group such as `session02_layout_07`.
3. Choose `train`, `val` or `test` for the entire group.
4. Click **Confirm selected reviewed + assign group/split**. This is the group
   button; there is no separate button literally named “Group”.
5. Confirm empty scenes only after checking that no target board or hand is present.
   Missing labels do not imply a negative scene.
6. Repeat for other groups, then click **Save new plan**.

The UI fills review metadata; it does not assess mask/class correctness for you.

```bash
pixi run --frozen python scripts/prepare_yolo_pilot.py build \
  --zip .local/datasets/cvat_exports/combined_001.zip \
  --plan .local/datasets/cvat_exports/review_completed_001.json \
  --output-dir .local/datasets/reviewed_next_001
```

`exported_instances` counts annotation rows, not guaranteed physical boards; a
fragmented mask/export can affect that number. The builder checks class remapping,
format/bounds, duplicate polygons, explicit negatives, review state and group/split
isolation. Human label review is still required.

### Add exports without mixing earlier splits

```bash
pixi run --frozen python scripts/extend_yolo_review.py \
  --previous-zip .local/datasets/cvat_exports/combined_001.zip \
  --previous-plan .local/datasets/cvat_exports/review_completed_001.json \
  --new-zip .local/datasets/cvat_exports/export_new.zip \
  --output-zip .local/datasets/cvat_exports/combined_002.zip \
  --output-plan .local/datasets/cvat_exports/review_draft_002.json
```

Open the same browser tool using the new ZIP/plan and another new output-plan
filename. Earlier rows are locked and retain their reviewed flags, groups and
splits; review only the new rows. Exact duplicates are deduplicated. Changed
nonempty labels for an existing image require separate correction review rather
than silently replacing historical annotations.

### Split rules and synthetic appearance variants

- Keep one physical arrangement entirely in one split, including hand/robot pose
  changes and lighting/exposure variants. New hand position alone does not make
  independent board geometry.
- Validation selects models/checkpoints/settings; test is held out for evaluation.
- An original and its “butchered” augmented image still share scene geometry.
  Do not put them into different splits.
- Keep prior test assignments stable rather than moving difficult examples into train.
- Original dataset group diversity matters more than raw correlated frame count.

```bash
pixi run --frozen -e perception-gpu python scripts/augment_board_dataset.py \
  --dataset .local/datasets/reviewed_next_001 \
  --output-dir .local/datasets/augmented_next_001
```

This adds `dim`, `bright`, `shadow` and `surface_marks` variants to **training only**.
Geometry/labels stay unchanged; val/test remain original. Seeds and lineage are
recorded and a preview is produced. Inspect that preview. Appearance augmentation
does not create new overlap configurations or fix incorrect size classes.

## Training and evaluating a replacement

Training/annotation do not need continuous robot access. They read a frozen local
dataset. Hardware is required for new capture, physical calibration and actual
closed-loop pick/place checks. Teammates can work on motion during an offline run.

### Local run

```bash
pixi run --frozen -e perception-gpu python scripts/train_yolo_pilot.py \
  --data .local/datasets/augmented_next_001/data.yaml \
  --output-dir .local/training/yolo_next_001 \
  --epochs 100 --batch 2 --imgsz 960
```

Batch 2 is a conservative starting point for the 8-GB laptop, not a guarantee of
memory use or a claim that bigger batch means better accuracy. The cloud batch
32 run used roughly 22 GB; do not copy it to the laptop blindly. Stop competing
GPU inference/training processes before measuring available VRAM.

The script checks CUDA, reviewed manifest and canonical labels, and starts from
pretrained YOLO11s segmentation weights. It uses seed 42, AMP, patience 20,
no mosaic and separate-instance mask handling. The first weight download needs
network access. A changed dataset requires a new run; do not silently swap data
inside an optimizer resume.

### Brev alternative, not a second mandatory main run

Brev is not required for local inference. Choose local or cloud training according
to available GPU/time. See [the cloud workflow](docs/brev_training.md) for packaging,
transfer, checksums, remote Pixi setup, detached training and result retrieval.

The final cloud command was:

```bash
pixi run python scripts/train_yolo_pilot.py \
  --data dataset/data.yaml --output-dir outputs/yolo_cloud_001 \
  --epochs 100 --batch 32 --imgsz 960
```

For unattended SSH use the documented `nohup`/log workflow. Download checkpoint,
CSV, plots and training log before stopping the instance. Closing the laptop or
disconnecting SSH does not stop cloud billing. No active cloud instance is needed
to deploy this final model.

### Moving a dataset to another machine

Full CVAT ZIPs, training datasets, ROS bags and training caches are not committed.
Get them separately from the dataset owner if reproducing training or the 48/46
evaluation. The Git branch includes small runnable examples and shareable reports.

Built `data.yaml` uses an absolute `path`. Verify transferred data/manifest first,
then rebase that path on the new machine:

```bash
pixi run --frozen -e perception-gpu python - <<'PY'
from pathlib import Path
import yaml
path = Path(".local/datasets/augmented_next_001/data.yaml")
config = yaml.safe_load(path.read_text())
config["path"] = str(path.parent.resolve())
path.write_text(yaml.safe_dump(config, sort_keys=False))
print(config["path"])
PY
```

This local path edit does not change or authorize changes to images, labels,
historical group assignments or old test splits. Preserve transfer provenance.

### Evaluate before promoting a replacement

Full-resolution standard metrics plus label/prediction comparisons:

```bash
pixi run --frozen -e perception-gpu python scripts/evaluate_yolo_boards.py \
  --weights .local/training/yolo_next_001/weights/best.pt \
  --dataset .local/datasets/augmented_next_001 \
  --output-dir .local/evaluation/yolo_next_full_001 \
  --imgsz 960 --confidence 0.25
```

Fast-adapter operating-point evaluation:

```bash
pixi run --frozen -e perception-gpu python scripts/evaluate_yolo_fast.py \
  --settings config/yolo_live_fast.json \
  --dataset .local/datasets/augmented_next_001 \
  --output-dir .local/evaluation/yolo_fast_recheck_001
```

The latter evaluates the checkpoint named by that settings file. For a candidate,
make a separate settings file pointing to it first; do not claim to evaluate new
weights while accidentally using the old final checkpoint. It is a fixed-confidence
mask-IoU evaluation, not an mAP sweep. Full-resolution diagnostic mask export also
makes its timing unsuitable as a live-FPS measurement.

Promote only after checking per-class errors, instance separation, centre/angle
errors, fixture negatives, live contract/failure checks, speed and physical tests.
Lower training loss alone is insufficient. Preserve the previous checkpoint and
settings for rollback.

## Troubleshooting

| Symptom | Check / response |
|---|---|
| `.tools/pixi` missing | Install Pixi and use `pixi` from PATH; the bootstrap binary is ignored by Git. |
| Missing setup script or FR5 packages | Initialize submodules, install default environment, run `pixi run --frozen build`. |
| Checkpoint missing | Download the release asset, verify SHA256 and install at the canonical path. |
| Pilot selected accidentally | Use `live-yolo-fast`; historical `live-yolo-pixels` uses the old pilot config. |
| CUDA unavailable | Check driver, correct GPU environment and `check-cuda`; silent CPU fallback is disabled. |
| Camera busy | Reuse an existing publisher; close another USB preview app or duplicate camera driver. |
| Publisher count 0 | Start/check the camera driver, device path and permissions. An image extractor is only a subscriber. |
| Duplicate services/publishers | Stop the duplicate launch in its own terminal. |
| Nodes cannot see camera | Check ROS domain, discovery, middleware, localhost-only restrictions and clock consistency. |
| Wrong scene frame | Fix source frame configuration; do not relabel pixels as `base_link`. |
| `sequence size exceeds remaining buffer` | Observed during DDS/CLI discovery. Verify actual data delivery/function; the message alone proves neither a healthy stream nor failed calibration. |
| Overlay seems slow | Fast overlay is capped at 1 FPS; measure source and coordinate rates independently. |
| Coordinate stream truly slow | Check capture rate, detector/transfer/pipeline fields, GPU load, duplicate workers and extra viewers. |
| No fresh service result | Missing/frozen/stale image, clock/frame mismatch, failed worker or model still loading. Inspect server output. |
| Many orange boxes | Masks do not support full accepted outlines. Inspect raw masks/occlusion before relaxing geometry. |
| Brown fixture labelled hand | Known model failure. Do not use it as a validated human-safety signal. |
| Duplicate VARa/VARb on one board | Known class/identity conflict. Resolve before issuing a pick. |
| Red mask includes brown backing | Known segmentation failure; high class confidence does not validate the outline/top layer. |
| Dataset lacks `data.yaml` | Export Ultralytics YOLO Segmentation with images, not CVAT XML or labels-only. |
| Output directory exists | Use a new numbered directory; previous results are intentionally preserved. |
| `rg` not installed | Use `grep` for simple filters; ripgrep is not a runtime dependency. |

### YUYV → BGR warning and source frame rate

The warning denotes CPU colour conversion on each captured frame. A local
OpenCV proxy on synthetic 1080p YUYV took .16 ms median and 6.0 ms p95 across
200 samples. This is not a measurement of the actual camera driver. Conversion
can contribute overhead, but did not explain the earlier mask/executor bottlenecks.

Raw 1920×1080 YUYV at 30 FPS carries approximately **124.4 MB/s**, before USB
overhead. Check the camera's advertised format/rate modes and actual hub/cable.

```bash
pixi run --frozen -e camera-ros ros2 topic hz /scene/image_raw --window 60
```

If the needed rate requires MJPEG, use a standard driver supporting that format,
for example `usb_cam`, after inspecting the actual device modes. Do not set MJPG
blindly in an unsupported driver. ROS image compression after capture does not
change the USB capture format. Changing resolution/FOV requires corresponding
geometry/intrinsic handling. Removing a colour-conversion warning can merely move
the conversion elsewhere because YOLO still needs colour input.

Official driver references: [v4l2_camera](https://index.ros.org/p/v4l2_camera/),
[usb_cam](https://github.com/ros-drivers/usb_cam),
[RealSense ROS](https://github.com/realsenseai/realsense-ros).

## Team handover checklist

### Perception owner provides

- Feature branch/version and lockfile; final model download and matching SHA256.
- Canonical classes, physical reference dimensions and known class confusion.
- Source topic/frame/resolution and actual camera settings.
- Original-pixel/angle semantics, fresh service and sample validated packet.
- Accepted/uncertain policy; duplicate, fixture, overlap and layer-order limits.
- Intrinsics/registration candidates with honest independent physical-validation status.
- Software/recovery/speed results, example photos and reproducible commands.

### Motion owner provides or confirms

- Controlled planning link (`tcp`, `flange` or another) and physically verified tool geometry.
- Validated camera/plane-to-robot mapping, surface heights and orientation convention.
- Unique-board selection and stale/changed-observation rejection.
- Registered class-to-bin transforms and placement directions.
- Existing gripper command interface and available seal/contact/visual feedback.
- Pregrasp/approach/retreat, stack updates, recovery and collision policy.
- Ownership of the real controller during calibration, manual positioning and execution.

### Shared first integration sequence

1. Another laptop passes checksum/CUDA, included offline example and isolated replay.
2. Actual camera delivers sharp, timestamped frames with the expected frame/FOV.
3. A separated known board is detected and both teammates agree on its pixel centre/axis.
4. Independently validate mapping and known TCP at safe clearance.
5. Run a controlled pick/place through the commissioned motion interface and supervision.
6. Verify placement, orientation and stack updates before repeated operation.
7. Add difficult overlaps/environmental changes only with an explicit handling policy.

### Branch backup

Work on this feature branch; never push/merge these changes directly into main.
Commit source/config/docs and reviewed small examples, not `.pixi`, full datasets,
ROS bags, credentials or caches. Final weights are a versioned release asset.
Use the project's human Git identity and do not add automated co-author trailers.

```bash
git status --short --branch
git diff --stat
git push origin feat/perception-geometric-detector
```

## Photo and results galleries

The gallery includes successes **and failures**. Documentation previews may be
downscaled; output JSON refers to the original inference resolution. Original
teammate JPEGs and the crowded benchmark PNG retain their bytes under
`docs/samples/`. Preview source hashes/captions are recorded in
[photo_manifest.json](docs/handover_data/photo_manifest.json).

### Hardware and historical scene

These photographs show earlier arrangements. Use current configuration and measured final geometry rather than inferring dimensions from photographs.

![Historical opposite view of FR5, black table and camera post.](docs/images/perception/hardware-workcell-side.jpg)

![Historical Logitech mounting on the wooden post.](docs/images/perception/hardware-logitech-mount.jpg)

![Earlier Logitech view with destination fixture and substrates.](docs/images/perception/hardware-scene-reference.jpg)

### Calibration examples

Historical rigid-grid views show target visibility and pose. Intrinsic reprojection quality alone does not validate hand-eye, TCP or absolute placement accuracy.

![Historical calibration view; not proof of validated robot registration.](docs/images/perception/calibration-view-0001.jpg)

![Historical calibration view; not proof of validated robot registration.](docs/images/perception/calibration-view-0048.jpg)

### Cloud training diagnostics

Training curves, class confusion, labelled batches and predictions from the final cloud run. Good training loss does not guarantee reliable overlapping-board picks.

![Cloud training diagnostic: results.png](docs/images/perception/training-results.jpg)

![Cloud training diagnostic: MaskPR_curve.png](docs/images/perception/training-MaskPR_curve.jpg)

![Cloud training diagnostic: MaskF1_curve.png](docs/images/perception/training-MaskF1_curve.jpg)

![Cloud training diagnostic: confusion_matrix_normalized.png](docs/images/perception/training-confusion_matrix_normalized.jpg)

![Cloud training diagnostic: train_batch0.jpg](docs/images/perception/training-train_batch0.jpg)

![Cloud training diagnostic: train_batch1.jpg](docs/images/perception/training-train_batch1.jpg)

![Cloud training diagnostic: val_batch0_labels.jpg](docs/images/perception/training-val_batch0_labels.jpg)

![Cloud training diagnostic: val_batch0_pred.jpg](docs/images/perception/training-val_batch0_pred.jpg)

### Held-out test diagnostics

These plots/batches come from the local held-out evaluation of the cloud checkpoint, not from its training batches.

![Independent test diagnostic: MaskPR_curve.png](docs/images/perception/test-MaskPR_curve.jpg)

![Independent test diagnostic: confusion_matrix_normalized.png](docs/images/perception/test-confusion_matrix_normalized.jpg)

![Independent test diagnostic: val_batch0_labels.jpg](docs/images/perception/test-val_batch0_labels.jpg)

![Independent test diagnostic: val_batch0_pred.jpg](docs/images/perception/test-val_batch0_pred.jpg)

![Independent test diagnostic: val_batch1_labels.jpg](docs/images/perception/test-val_batch1_labels.jpg)

![Independent test diagnostic: val_batch1_pred.jpg](docs/images/perception/test-val_batch1_pred.jpg)

### Earlier labelled frame comparisons

Exported labels are on the left and predictions on the right. Inspect actual instance masks/classes: an unmatched exported row can also reflect a fragmented or duplicate annotation.

![s0_scene_0065: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0065.jpg)

![s0_scene_0071: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0071.jpg)

![s0_scene_0111: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0111.jpg)

![s0_scene_0137: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0137.jpg)

![s0_scene_0143: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0143.jpg)

![s0_scene_0213: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0213.jpg)

![s0_scene_0214: exported labels on left, checkpoint predictions on right.](docs/images/perception/evaluation-s0_scene_0214.jpg)

### Outline recovery diagnostics

The outline check can recover small defects without lowering its .93 mask/quad overlap requirement. It still does not certify a unique topmost instance or correct class.

![oct25_scene_0173: supported outline recovery diagnostic, not a grasp success.](docs/images/perception/geometry-oct25_scene_0173.jpg)

![s0_scene_0065: supported outline recovery diagnostic, not a grasp success.](docs/images/perception/geometry-s0_scene_0065.jpg)

![s0_scene_0213: supported outline recovery diagnostic, not a grasp success.](docs/images/perception/geometry-s0_scene_0213.jpg)

![s0_scene_0214: supported outline recovery diagnostic, not a grasp success.](docs/images/perception/geometry-s0_scene_0214.jpg)

### Training-only appearance variants

Original, dim, bright, shadow and surface-mark examples. Geometry/labels stay unchanged and these variants are confined to training.

![Training-only appearance variants: original, dim, bright, shadow, surface marks.](docs/images/perception/augmentation-preview.jpg)

### Capture guide examples

Synthetic arrangement instructions, not real photographs or ground-truth training labels. Physical captures must be inspected independently.

![Synthetic capture guide; instructions, not labelled ground truth.](docs/images/perception/capture-guide-013.jpg)

![Synthetic capture guide; instructions, not labelled ground truth.](docs/images/perception/capture-guide-026.jpg)

![Synthetic capture guide; instructions, not labelled ground truth.](docs/images/perception/capture-guide-045.jpg)

![Synthetic capture guide; instructions, not labelled ground truth.](docs/images/perception/capture-guide-059.jpg)

### All 13 new teammate scenes

Expand each scene to compare original pixels, raw predicted masks, and full-resolution versus fast coordinate output. Green coordinate outlines are supported; orange coordinate outlines are uncertain; magenta denotes predicted hands. In raw-mask views, contour colours denote classes instead. None of these colours certifies a valid robot pick.

Per-frame details: [visual review CSV](docs/handover_data/sim_scenes_visual_review.csv). These 13 images were not labelled or used for training in this evaluation.

<details>
<summary>Scene 01: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 01 original](docs/samples/scene_01.jpg) | ![Scene 01 fast coordinates](docs/images/perception/sim-01-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 01 raw masks](docs/images/perception/sim-01-raw-masks.jpg) | ![Scene 01 full coordinates](docs/images/perception/sim-01-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_01_fast.json) · [Full-resolution JSON](docs/handover_data/scene_01_full.json)

</details>

<details>
<summary>Scene 02: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 02 original](docs/samples/scene_02.jpg) | ![Scene 02 fast coordinates](docs/images/perception/sim-02-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 02 raw masks](docs/images/perception/sim-02-raw-masks.jpg) | ![Scene 02 full coordinates](docs/images/perception/sim-02-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_02_fast.json) · [Full-resolution JSON](docs/handover_data/scene_02_full.json)

</details>

<details>
<summary>Scene 03: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 03 original](docs/samples/scene_03.jpg) | ![Scene 03 fast coordinates](docs/images/perception/sim-03-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 03 raw masks](docs/images/perception/sim-03-raw-masks.jpg) | ![Scene 03 full coordinates](docs/images/perception/sim-03-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_03_fast.json) · [Full-resolution JSON](docs/handover_data/scene_03_full.json)

</details>

<details>
<summary>Scene 04: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 04 original](docs/samples/scene_04.jpg) | ![Scene 04 fast coordinates](docs/images/perception/sim-04-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 04 raw masks](docs/images/perception/sim-04-raw-masks.jpg) | ![Scene 04 full coordinates](docs/images/perception/sim-04-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_04_fast.json) · [Full-resolution JSON](docs/handover_data/scene_04_full.json)

</details>

<details>
<summary>Scene 05: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 05 original](docs/samples/scene_05.jpg) | ![Scene 05 fast coordinates](docs/images/perception/sim-05-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 05 raw masks](docs/images/perception/sim-05-raw-masks.jpg) | ![Scene 05 full coordinates](docs/images/perception/sim-05-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_05_fast.json) · [Full-resolution JSON](docs/handover_data/scene_05_full.json)

</details>

<details>
<summary>Scene 06: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 06 original](docs/samples/scene_06.jpg) | ![Scene 06 fast coordinates](docs/images/perception/sim-06-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 06 raw masks](docs/images/perception/sim-06-raw-masks.jpg) | ![Scene 06 full coordinates](docs/images/perception/sim-06-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_06_fast.json) · [Full-resolution JSON](docs/handover_data/scene_06_full.json)

</details>

<details>
<summary>Scene 07: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 07 original](docs/samples/scene_07.jpg) | ![Scene 07 fast coordinates](docs/images/perception/sim-07-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 07 raw masks](docs/images/perception/sim-07-raw-masks.jpg) | ![Scene 07 full coordinates](docs/images/perception/sim-07-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_07_fast.json) · [Full-resolution JSON](docs/handover_data/scene_07_full.json)

</details>

<details>
<summary>Scene 08: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 08 original](docs/samples/scene_08.jpg) | ![Scene 08 fast coordinates](docs/images/perception/sim-08-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 08 raw masks](docs/images/perception/sim-08-raw-masks.jpg) | ![Scene 08 full coordinates](docs/images/perception/sim-08-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_08_fast.json) · [Full-resolution JSON](docs/handover_data/scene_08_full.json)

</details>

<details>
<summary>Scene 09: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 09 original](docs/samples/scene_09.jpg) | ![Scene 09 fast coordinates](docs/images/perception/sim-09-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 09 raw masks](docs/images/perception/sim-09-raw-masks.jpg) | ![Scene 09 full coordinates](docs/images/perception/sim-09-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_09_fast.json) · [Full-resolution JSON](docs/handover_data/scene_09_full.json)

</details>

<details>
<summary>Scene 10: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 10 original](docs/samples/scene_10.jpg) | ![Scene 10 fast coordinates](docs/images/perception/sim-10-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 10 raw masks](docs/images/perception/sim-10-raw-masks.jpg) | ![Scene 10 full coordinates](docs/images/perception/sim-10-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_10_fast.json) · [Full-resolution JSON](docs/handover_data/scene_10_full.json)

</details>

<details>
<summary>Scene 11: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 11 original](docs/samples/scene_11.jpg) | ![Scene 11 fast coordinates](docs/images/perception/sim-11-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 11 raw masks](docs/images/perception/sim-11-raw-masks.jpg) | ![Scene 11 full coordinates](docs/images/perception/sim-11-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_11_fast.json) · [Full-resolution JSON](docs/handover_data/scene_11_full.json)

</details>

<details>
<summary>Scene 12: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 12 original](docs/samples/scene_12.jpg) | ![Scene 12 fast coordinates](docs/images/perception/sim-12-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 12 raw masks](docs/images/perception/sim-12-raw-masks.jpg) | ![Scene 12 full coordinates](docs/images/perception/sim-12-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_12_fast.json) · [Full-resolution JSON](docs/handover_data/scene_12_full.json)

</details>

<details>
<summary>Scene 13: original, fast output, raw masks and full-resolution output</summary>

| Original image | Fast coordinate output |
|---|---|
| ![Scene 13 original](docs/samples/scene_13.jpg) | ![Scene 13 fast coordinates](docs/images/perception/sim-13-fast-coordinates.jpg) |

| Raw predicted masks | Full-resolution coordinate output |
|---|---|
| ![Scene 13 raw masks](docs/images/perception/sim-13-raw-masks.jpg) | ![Scene 13 full coordinates](docs/images/perception/sim-13-full-coordinates.jpg) |

[Fast JSON](docs/handover_data/scene_13_fast.json) · [Full-resolution JSON](docs/handover_data/scene_13_full.json)

</details>

### Original crowded replay benchmark

This unmodified held-out input was used for the 20-second ROS throughput run. It is an evaluation example; do not silently move it into training and continue claiming the old evaluation is independent.

![Original crowded benchmark input](docs/samples/benchmark_crowded.png)


## Repository map and further reading

```text
config/
  yolo_checkpoint.json             # final release/checksum/model provenance
  yolo_live_fast.json              # final checkpoint and fast measurement
  yolo_live.json                   # historical pilot default
  logitech_ros.yaml                # standard scene-camera configuration
  d405_ros.yaml                    # standard wrist configuration
  perception_full_table.toml       # current full-image search
  calibration/                    # saved intrinsics/candidate registrations
scripts/
  yolo_pixels.py                   # trained instance masks to pixel observations
  live_b_pixels.py                 # shared ROS bridge/persistent worker
  read_live_b_pixels.py            # validated fresh-result consumer
  b_live_geometry.py               # supported outline, centre and axis
  check_yolo_ros_replay.py          # isolated streaming/recovery checks
  capture_board_dataset.py         # press-S capture with metadata
  prepare_yolo_pilot.py            # merge/inventory/reviewed build
  extend_yolo_review.py            # add exports, keep old assignments frozen
  review_yolo_plan.py              # localhost group/split browser UI
  augment_board_dataset.py         # training-only appearance variants
  train_yolo_pilot.py              # pinned training entry point
  evaluate_yolo_boards.py          # standard full-resolution evaluation
  evaluate_yolo_fast.py            # compact-mask operating-point checks
docs/
  samples/                         # original examples and benchmark image
  images/perception/               # gallery previews and plots
  handover_data/                   # results, example response, CSVs and provenance
```

Further reading:

- [Repository and real-robot rules](AGENTS.md)
- [Robot/MoveIt handoff](docs/FR5%20Board%20Stacking%20Robot%20Handoff.md)
- [GPU/camera setup history](docs/perception_setup.md)
- [ROS pixel interface](docs/live_b_pixels.md)
- [YOLO adapter notes](docs/yolo_ros_adapter.md)
- [Measured speed results](docs/perception_speed_validation.md)
- [Camera calibration](docs/calibration_setup.md)
- [Flange survey and homography](docs/pendant_flange_homography.md)
- [Capture UI](docs/dataset_capture_ui.md)
- [Review tool](docs/yolo_review_tool.md)
- [Appearance augmentation](docs/appearance_augmentation.md)
- [Training workflow](docs/yolo_pilot.md)
- [Brev workflow](docs/brev_training.md)
- [Outstanding questions/history](docs/perception_open_questions.txt)

Some guides preserve earlier commissioning history and pilot defaults. For the
final model, canonical labels, current launch/ROI and measured limitations, use
this README and `config/yolo_checkpoint.json` first.
