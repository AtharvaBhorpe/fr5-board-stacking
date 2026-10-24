# FR5 Board Stacking

Voice-commanded pick-and-place on a Fairino FR5 cobot: detect white boards
in the pick zone, pick each from its center with a Schmalz vacuum gripper,
and stack them inside the taped boundary.

## Setup

1. Install pixi: https://pixi.sh
2. Clone and install:

       git clone https://github.com/AtharvaBhorpe/fr5-board-stacking.git
       cd fr5-board-stacking
       pixi install

Do not use conda `base` alongside pixi (`conda config --set auto_activate false`).

## Team workflow

- `main` is protected: never commit to it directly.
- One branch per task: `git switch -c feat/<area>-<short-description>`
  - Areas: `robot`, `perception`, `calib`, `task`, `voice`, `infra`
  - Example: `feat/perception-webcam-threshold`
- Open a PR into `main`; at least 1 teammate approves before merge.
- Pull often: `git switch main && git pull`
- Commit `pixi.lock` whenever dependencies change.
- Any teammate other than the author can approve; anyone can merge once approved.
- Full rules for humans and AI agents: see [AGENTS.md](AGENTS.md).
