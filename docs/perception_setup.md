# Perception setup: stage 0

Stage order: verify GPU and camera streams, implement and test geometric
perception (A), then add pretrained segmentation (B) and voice selection (E).
These checks never connect to the robot or send motion/tool commands.

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
595.99.02 driver/module packages. Ubuntu then reported a reboot required;
the GPU was not yet active at that check.

After saving work and restarting, run `nvidia-smi` and the hardware task.
Working `nvidia-smi` verifies driver access, not PyTorch CUDA execution.
Install and verify the GPU model runtime through pixi when adding B.
Do not install a separate system CUDA toolkit just to obtain a driver.

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

Work is on `feat/perception-hardware-checks`. Commits use the configured
human Git identity; no AI author or co-author attribution is added.
Push to that branch, never directly to `main`:

```bash
git push -u origin feat/perception-hardware-checks
```

If GitHub login is needed, the local pixi tool can launch GitHub CLI:

```bash
./.tools/pixi exec --spec gh gh auth login --hostname github.com --git-protocol https --web
```

Keep passwords, access tokens and authentication output out of commits.
The current unanswered questions are in `docs/perception_open_questions.txt`.
