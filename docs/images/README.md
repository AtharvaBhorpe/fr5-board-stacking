# Image sources

[Back to the README](../../README.md)

The README uses project photos and existing test screenshots. The files are copied without image edits.
The diagram is new and describes the integrated source tree. No image is an AI-generated hardware scene.

## Lab photographs and recorded outputs

Source: `feat/vision1-overhead`, `docs/img/`, available in commit `fa33223`.
These images came from the project's earlier lab and vision documentation.

| File here | Original file | Context |
|---|---|---|
| `workcell.jpg` | `photo_workcell.jpg` | Real FR5 workcell |
| `tool-and-wrist.jpg` | `photo_tool_d405.jpg` | Real vacuum tool and wrist camera |
| `vision-overhead.jpg` | `overhead_annotated.jpg` | Historical detection overlay and template labels |
| `vision-stacks.jpg` | `model_real_stacks.jpg` | Historical segmentation output on difficult overlap cases |
| `lab-rviz-archive.jpg` | `rviz_real_robot.jpg` | Earlier RViz pick interface, retired from this integrated branch |

## Web screenshots

The existing browser tests produced these captures on 28 October 2026, before this README rewrite.
They show the integrated web app connected to simulation. They do not prove operation on a physical phone.

| File here | Original file |
|---|---|
| `web-desktop.png` | `web/test-results/desktop.png` |
| `web-phone.png` | `web/test-results/responsive/390-chat.png` |

## System diagram

`system.svg` is an editable SVG with no remote assets or scripts.
Its sources are `conversation/bridge.py`, `conversation/planner.py`, `vision1/node.py`, and `src/hk_fr5_task/`.
The diagram shows the normal free-text job path. Direct commands bypass model planning.
The wrist camera provides web views. It does not supply pick heights to the integrated controller.
