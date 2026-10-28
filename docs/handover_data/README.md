# Portable handover evidence

These small reports let another laptop inspect the existing results without the
full datasets or ROS bags. They do not replace actual reproduction or physical
robot commissioning.

| File | Contents |
|---|---|
| `full_resolution_evaluation.json` | Standard metrics and fixed-confidence class counts, 48 val / 46 test images |
| `fast_evaluation.json` | Compact-mask operating-point evaluation; not an mAP sweep |
| `speed_replay.json` | Full-window 20-second crowded ROS replay, source age, cadence and failure checks |
| `example_observation.json` | Complete real saved reader result; old timestamp is intentionally stale for live use |
| `cloud_training_results.csv` | Preserved cloud epoch metrics; best epoch 58, stopped at 78 |
| `sim_scenes_evaluation.json` | Both processing modes on the 13 unlabelled teammate images |
| `sim_scenes_report.md` | Visual findings and concrete remaining failures |
| `sim_scenes_visual_review.csv` | Per-frame review notes and proposal counts |
| `scene_XX_fast.json` / `scene_XX_full.json` | Per-frame raw class scores, accepted/uncertain geometry and masks' contours |
| `photo_manifest.json` | Source hashes, dimensions and captions for gallery previews |

Pixel centres/angles are not robot poses. Green supported quadrilaterals do not
prove class correctness, unique instance separation, layer order or safe grasp.
False hands on brown fixtures and duplicate conflicting classes are documented
failures. Execution and physical/safety validation flags remain false.
