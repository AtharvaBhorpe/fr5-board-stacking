# Live B pixel-coordinate handover

Runs the existing Grounding DINO + SAM 2 detector on the latest Logitech image.
No dimension-measurement extension, robot registration or robot driver is started.
Models stay loaded on CUDA; the frame queue retains only the latest image.

## Launch alongside the existing camera and motion processes

From the repository root, using the feature branch:

```bash
git switch feat/perception-geometric-detector
git pull --ff-only origin feat/perception-geometric-detector
pixi install --frozen -e perception-gpu
pixi run --frozen live-b-pixels
```

Use `./.tools/pixi` instead of `pixi` on the commissioning laptop. The default
ROS environment must also be installed (the team's motion setup already uses it).
CUDA is required. Model weights stay under ignored `.local/models/huggingface`;
they are not in Git. The existing model loader contacts Hugging Face at startup,
even when weights are cached, so startup currently needs network access.

The standard ROS launch-file equivalent is:

```bash
pixi run --frozen ros2 launch config/live_b_pixels.launch.py
```

This launch can also be included in another ROS launch description. It starts
only the perception process and expects the existing `/scene/image_raw` publisher.
Do not start a second instance of the same live B node.

If no scene camera is running, start it separately:

```bash
pixi run --frozen -e camera-ros scene-camera
```

The camera input must be raw Logitech pixels, with frame ID
`scene_camera_optical_frame`. The committed pickup ROI is in
`config/perception_review_roi_v3.toml`; check it against the current mounted view.
The live config never depends on another person's ignored replay folder.

## Receive fresh coordinates

Wait for `READY`, then in another terminal:

```bash
pixi run --frozen read-b-pixels
```

This requests an observation from an image captured after the request. It prints
the snapshot ID, exact source stamp, image size and per-instance coordinates.
Call it before each pick; it never commands motion.

For an existing controller, use standard `std_srvs/srv/Trigger` at
`/perception/get_boards_pixels`. Parse the JSON in `response.message` only after
checking `response.success`. `scripts/read_live_b_pixels.py` contains the minimal
client and the reusable `handoff(packet, now, max_age)` validator.

Continuous output is standard `std_msgs/msg/String` containing JSON:

```bash
pixi run --frozen ros2 topic echo /perception/boards_pixels
```

For the annotated image, with the camera-ros environment installed:

```bash
pixi run --frozen -e camera-ros ros2 run image_view image_view --ros-args \
  -r image:=/perception/boards_pixels/debug_image
```

## Interface

| Field | Meaning |
|---|---|
| `coordinate_system` | `raw_image_pixels`; not rectified pixels or robot metres |
| `frame_id` | `scene_camera_optical_frame` |
| `image_size` | `[width, height]`, currently `[1920, 1080]` |
| `source_stamp` | Exact ROS image stamp, seconds plus nanoseconds |
| `source_timestamp_sec` | Same source time as floating-point seconds |
| `snapshot_id` | Unique inference snapshot |
| `objects[].instance_id` | Unique within this snapshot; not a persistent track |
| `objects[].centre_px` | `[u, v]`: right/down from top-left, in the full original image |
| `objects[].orientation_image_deg` | Clockwise from image-right, modulo 180 degrees |
| `objects[].supported_pixel_corners` | Four mask-supported corners, or null |
| `objects[].outline_kind` | Supported quadrilateral or enclosing-rectangle fallback |
| `objects[].text_prompt_label` | Raw detector phrase; not a validated size/class label |

For a supported quadrilateral the centre is the diagonal intersection; otherwise
it is the enclosing rotated-rectangle centre. Green overlay means a supported
outline; orange means the enclosing-rectangle fallback. Neither certifies that
one mask contains exactly one complete board or that the suction point is clear.

## Motion-side rules and limits

- Recompute observation age at use, not only when received. Default maximum age
  is two seconds. Separate computers need synchronized clocks and matching ROS
  domain/network configuration.
- `success=true` means fresh analysis completed, not permission to execute a pick.
  The publisher always reports `execution_allowed=false` because robot geometry
  and motion decisions belong to the consumer.
- Convert pixels through the consumer's independently validated mapping before
  commanding MoveIt. Image angle is not robot yaw. Board height and commanded
  tool frame remain the motion interface's responsibility.
- IDs change each frame. Do not launch a motion every time the topic publishes;
  use one fresh snapshot for one controlled pick cycle, then reacquire.
- Merged touching/overlapping boards can have incorrect centres. An enclosing
  rectangle is not a verified full-board boundary.
- Default period is 0.5 seconds; actual rate depends on GPU/inference/I/O time.
  This is a perception update stream, not a high-frequency servo controller.
- Camera interruption invalidates cached coordinates. A fresh request fails if
  no new processed image arrives.

## Tests

```bash
pixi run --frozen -e perception python scripts/live_b_pixels.py self-test
pixi run --frozen -e perception \
  python -m unittest discover -s tests -p test_live_b_pixels.py -v
```

Tests cover perspective centres, occluded-outline rejection, stale/invalid
observations, worker timeouts, clean GPU-environment startup command and portable
ROI configuration. Physical pick accuracy requires separate acceptance tests.

On 2026-10-26 an isolated localhost ROS-domain replay using the actual RTX 4070
and a saved 1920x1080 scene verified continuous publication, six detected objects,
the debug image, and a service response from an image newer than its request.
One observed detector time was 509 ms with 1.09 s source-to-receiver age.
Stopping replay invalidated cached observations and made a new service request
fail. This was a software/GPU replay, not a live-camera or physical-pick test.
