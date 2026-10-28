# Runnable evaluation photographs

`scene_01.jpg` through `scene_13.jpg` are the original 1920×1080 JPEG bytes from
the teammate-provided `sim_scenes.zip`. Archive SHA256:

```text
b7a01b01e40c3669ab13f3ae5b281153932f87d58fde03501e923d9c7c18a79a
```

No ground-truth class/mask annotations were supplied for those 13 images. They
contain brown tagged fixtures, boards on both sides of the robot, stacks and
overlaps that reveal current model failures. Use them for inference and visual
review; do not invent precision/recall from proposal counts.

`benchmark_crowded.png` is an unmodified copy of held-out `s0_scene_0143.png` from
the labelled dataset, included so a new laptop can repeat the exact crowded ROS
speed input without downloading the full dataset. Its labels are not included here.

These are evaluation examples. Moving them into future training invalidates
claims that subsequent results on them remain independent.

All setup, inference commands and the photo gallery are in the root
[handover README](../../README.md).
