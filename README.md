# FR5 Board Stacking

Voice-commanded pick-and-place on a Fairino FR5 cobot: detect white boards
in the pick zone, pick each from its center with a Schmalz vacuum gripper,
and stack them inside the taped boundary.

## Setup

1. Install pixi: https://pixi.sh
2. Clone and install:

       git clone --recurse-submodules https://github.com/AtharvaBhorpe/fr5-board-stacking.git
       cd fr5-board-stacking
       pixi install
       pixi run build
       pixi run demo   # FR5 in RViz with fake hardware

Do not use conda `base` alongside pixi (`conda config --set auto_activate false`).

## Team workflow

- `main` is protected: never commit to it directly.
- One branch per task: `git switch -c feat/<area>-<short-description>`
  - Areas: `robot`, `perception`, `calib`, `task`, `voice`, `infra`
  - Example: `feat/perception-webcam-threshold`
- Open a PR into `main`; reviews are optional, merge when ready.
- Pull often: `git switch main && git pull`
- Commit `pixi.lock` whenever dependencies change.
- AI agents never merge PRs; a human always merges.
- Full rules for humans and AI agents: see [AGENTS.md](AGENTS.md).

## Perception commissioning

Start with [GPU and camera checks](docs/perception_setup.md) before geometric
perception, pretrained segmentation or voice integration. Camera checks do not
connect to or move the robot. The [running questions list](docs/perception_open_questions.txt)
tracks missing calibration, material and interface information.
