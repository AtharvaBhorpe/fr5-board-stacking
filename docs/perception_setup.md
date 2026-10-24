# Perception setup

Stage order: verify GPU and camera streams, implement and test geometric
perception (A), then add pretrained segmentation (B) and voice selection (E).
These checks never connect to the robot or send motion/tool commands.

## Standard ROS camera workflow

The live path uses upstream `v4l2_camera`, `realsense2_camera`,
`camera_calibration`, `image_proc` and `image_view`. ROS launch files only
configure these packages; there is no custom live camera driver. The Logitech
driver uses its stable device link directly. Its focus is set through ROS
parameters, without `v4l2-ctl` or a symlink-resolution helper.

Run each long-running command in a separate terminal from the repository root.
Do not start duplicate camera drivers or two robot-control stacks. Stop a
process with Ctrl+C in its terminal.

```bash
./.tools/pixi run --frozen -e camera-ros cameras
./.tools/pixi run --frozen -e camera-ros view-cameras
./.tools/pixi run --frozen -e camera-ros calibrate-scene
```

The first command starts both camera drivers. The second opens three standard
viewer windows. The third opens the standard ChArUco calibration GUI for the
large grid, using dimensions from `config/perception.toml`. Move the grid
through varied image positions, sizes and tilts; pause for sharp frames and
watch the GUI's X/Y/size/skew coverage indicators. Use only the large print in
that session: the Humble ChArUco fitting path uses one board model.
Do not commit or apply a matrix until its errors and independent checks pass.
Save the ROS calibration output, then set its file URL in
`config/logitech_ros.yaml` and restart the scene driver. Until then, the scene
`CameraInfo` matrix is zero, so metric projection must remain disabled.

| Data | ROS topic |
| --- | --- |
| Logitech colour | `/scene/image_raw` |
| Logitech calibration | `/scene/camera_info` |
| Wrist colour | `/wrist/wrist_camera/color/image_raw` |
| Wrist calibration | `/wrist/wrist_camera/color/camera_info` |
| Wrist depth aligned to colour | `/wrist/wrist_camera/aligned_depth_to_color/image_raw` |
| Aligned-depth calibration | `/wrist/wrist_camera/aligned_depth_to_color/camera_info` |

The D405 driver aligns depth and synchronizes its own streams. The Logitech
is a separate camera; this launch does not hardware-synchronize both cameras.
The scene profile is 1920x1080 YUYV for calibration; actual frame rate must be
measured before obstacle monitoring. D405 profiles are 848x480 at 30 fps.
Treat ROS `16UC1` depth values as millimetres (standard RealSense ROS output),
not as raw SDK counts using the earlier 0.0001-m device scale.

The optional `camera-ros` pixi environment keeps camera packages separate from
the team's working motion dependencies; all existing environment lock entries
were verified unchanged. Both runtimes use ROS 2 Humble and exchange normal
ROS messages. They must share ROS_DOMAIN_ID/RMW settings and a consistent
clock. Do not source system Jazzy inside a pixi session. Future perception
outputs must retain source timestamps/frame IDs; wrist transforms must use
measured poses at those timestamps, with stale results rejected.

Verified locally: 20 synthetic 1920x1080 RGB messages crossed from camera-ros
to motion with full payload checks; maximum observed age was 13.82 ms. This
includes test-side validation and excludes exposure and perception inference;
it is not an end-to-end latency guarantee. Repeat with:

```bash
python3 scripts/check_ros_transport.py
./.tools/pixi run --frozen -e camera-ros test-ros-calibration
```

The transport test uses its own localhost-only ROS domain and no robot.
The calibration check recovers known intrinsics from synthetic projections
through the upstream ROS solver. OpenCV is constrained below 5 because the
Humble solver uses an API removed in OpenCV 5.

For the D405 USB permission setup, install `config/99-realsense-d405.rules`
into `/etc/udev/rules.d`, reload udev rules and reconnect/trigger the D405.
The rule grants the plugdev group access only to the D405 USB product.

Sources: [V4L2 ROS driver](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/humble),
[ROS calibrator](https://github.com/ros-perception/image_pipeline/tree/humble/camera_calibration),
[RealSense ROS driver](https://github.com/realsenseai/realsense-ros).

## Robot feedback

The team's pinned Fairino driver and workcell model build successfully.
Its state/command server can be started without a trajectory controller:

```bash
./.tools/pixi run --frozen ros2 run fairino_hardware_v3_9_0_1 ros2_cmd_server
./.tools/pixi run --frozen ros2 topic echo /nonrt_state_data --once
```

This creates a command service too; starting it alone sends no motion command.
The hardware reports v3.9.2. Five non-real-time feedback samples with nonzero
joint/TCP positions and zero controller errors were received. The SDK also
warns about its real-time feedback packet size; that path must be checked
before enabling ros2_control trajectories. Non-real-time feedback lacks a
source Header; do not present callback receipt time as exact exposure-time
robot pose. No robot motion has been tested by this workflow.

## Environments

The team's default pixi environment retains ROS 2 Humble. The `perception`
environment uses Python 3.12, OpenCV 4, NumPy and the RealSense SDK; it does not
source or modify the separate Jazzy teleop workspace. Camera processing can
be tested before agreeing on the ROS/API integration with the motion owner.

From the repository root, with pixi installed:

```bash
pixi install -e perception
pixi run -e perception test-perception
pixi run -e perception hardware
```

The local bootstrap copy is `.tools/pixi` if pixi is not on your PATH. Replace
`pixi` with `./.tools/pixi` in these commands. Bootstrap tools and captures are
ignored by Git; commit dependency changes together with `pixi.lock`.

## GPU commissioning

The setup inspection found Ubuntu 24.04, an RTX 4070 Mobile and Secure Boot.
The running kernel was `7.0.0-38-generic`, but the installed NVIDIA module was
for `7.0.0-28-generic`. The user installed the matching Canonical-signed
595.99.02 driver/module packages. After the restart, `nvidia-smi` and the
inventory checker successfully detected the RTX 4070 Laptop GPU, 8188 MiB
memory and driver 595.99.02.

The optional `perception-gpu` environment adds PyTorch 2.7.1 from its official
CUDA 12.8 wheel index. The driver reporting CUDA 13.2 is its supported API
level; the environment's bundled CUDA runtime can have a different version.
Neither the default ROS environment nor the camera-only environment needs
these GPU packages.

```bash
pixi install -e perception-gpu
pixi run -e perception-gpu check-cuda
```

This test creates GPU tensors, performs matrix multiplication, synchronizes,
copies the result to the CPU and checks the numerical result. It fails if
CUDA is unavailable instead of silently falling back to CPU execution.
The real RTX 4070 test passed after restart with PyTorch 2.7.1+cu128,
CUDA runtime 12.8 and approximately 7.4 GiB free CUDA memory at the check.
This establishes runtime operation, not model performance or accuracy.
No B/E model weights are installed by this step, and a system CUDA compiler
toolkit is not needed for the prebuilt runtime.

Sources: [PyTorch CUDA wheels](https://pytorch.org/get-started/previous-versions/#v271),
[pixi PyTorch configuration](https://pixi.prefix.dev/latest/python/pytorch/).

## Camera commissioning

External cameras were not connected at the initial inspection. Inventory
detected only the integrated laptop camera; it was not opened.

1. Connect the Logitech and D405 with suitable data cables.
2. Run the inventory task and identify the Logitech's stable
   `/dev/v4l/by-id/...` device path.
3. Set `logitech.device` in `config/perception.toml` to that path. An empty
   value refuses capture rather than selecting the laptop camera.
4. If multiple D405 cameras are present, set `d405.serial` in the same file.
5. Close any other application using either camera, then run:

```bash
pixi run -e perception check-logitech
pixi run -e perception check-d405
```

Each stream check requires the configured number of frames and runs behind
a process timeout. Logitech capture reports actual frame dimensions and the
driver's reported frame rate. D405 capture reports color/depth dimensions,
depth scale and the fraction of nonzero depth in the last frame.

Passing these checks establishes frame delivery only. Next visually check
focus, lighting, coverage, depth validity on actual substrates and timestamp
alignment. Calibration and measured robot poses are required before any
metric grasp output can be accepted. A synthetic test is not a hardware test.

## Backups

Work is on `feat/perception-geometric-detector`. Commits use the configured
human Git identity; no AI author or co-author attribution is added.
Push to that branch, never directly to `main`:

```bash
git push -u origin feat/perception-geometric-detector
```

If GitHub login is needed, the local pixi tool can launch GitHub CLI:

```bash
./.tools/pixi exec --spec gh gh auth login --hostname github.com --git-protocol https --web
```

Keep passwords, access tokens and authentication output out of commits.
The current unanswered questions are in `docs/perception_open_questions.txt`.
