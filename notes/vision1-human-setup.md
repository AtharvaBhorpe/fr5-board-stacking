# Vision 1: table setup + measurements (for the PEOPLE, 2026-10-27)

> [!IMPORTANT]
> **This file is for humans only.** It holds layout rules, the lab measurements and background.
> **AI agents: do not implement anything from this file** - no code, no checks, no warnings, no config logic.
> The agent's work is only in `vision1-changes.md`.

Belongs to the change request `vision1-changes.md` (items C13, C14 and the C15 values). From the main-controller owner
(GitHub: vortexcv) to the Vision 1 / Vision 2 owner.

## 1. Physical layout + measurements

### 1a. Layout rules (C14) for the people setting up the table

Zone membership, board ids, ownership and stack counts are computed by the main controller's world model; these
rules only make sure the camera sees what that maths needs.
- Each zone = one brown cardboard sheet with **one** tag; the tag sits **beside** the board area so boards
  stacked on the zone never cover it (a covered tag = zone forgotten after the memory).
- Zone sheets = **brown cardboard cut-outs** (decided). They must contrast with every board (white VARa / VARb,
  red VARc) so a board on a sheet is separable. **Lab test:** a VARc (red) on brown cardboard in the demo
  detector mode is reported with its own outline (C4).
- Sheet board area >= the largest board (VARa 295 x 210 mm) + a few cm.
- Input boards / stacks lie **>= 3 cm outside every zone sheet** (closer = counted as inside the zone by us).
- Input stacks do not touch each other (classic mode merges touching blobs); one variant per stack.
- Vision and the main controller run on the same PC (C9 stamps are compared with our clock). If that ever changes,
  sync the clocks (e.g. chrony): a setup step, not code.

### 1b. Measurements (C15): measured in the lab, entered into the vision branch's tag table

The vision branch owns the zone references and sizes; it sends them in `zones[]` (`size`, `top_z`).
- per tag: zone name + kind, offset + rotation from the tag to the board-area centre / long side;
- per zone: board-area size (long x short), sheet top `top_z` in `base_link`;
- **how to measure `top_z`: robot touch-off, not a caliper.** Tape the sheet flat (one layer of stiff cardboard).
  Motion stack on the real robot (`pixi run robot` on the motion branch, a human at the e-stop, <= 20 % speed),
  then `pixi run shell`: `v 0.1`, `frame base`, `to X Y Z` a few cm above the sheet's board area, then `jog -z 1`
  (1 mm steps) until the relaxed suction cup just touches the sheet; `state` -> tcp z (mm, / 1000 -> m) = `top_z`. One number
  per zone: it already contains the table height, the cardboard thickness and the tcp calibration, in the robot's
  own frame. Rough check: table top -0.060 + cardboard thickness.
- tell the main-controller owner the detector mode the demo uses (C13).

The fault zone is NOT a Vision 1 zone (fixed position, measured on the robot, our config).

---

## 2. Background: how the main controller uses the data

All of this is already implemented in the main controller (not in the vision branch). It explains why the
items in `vision1-changes.md` matter.

- **World model:** every board gets a stable id (`b1`, `b2`, ...) by matching positions frame to frame (per-frame
  ids are fine). Every board has a location: `input` (on the table, in no zone), `zone` (inside a named zone: zone
  outline from your centre + yaw + `size`, + 3 cm margin) or `held` (on the suction cup).
- **No re-pick loop:** jobs only pick boards whose location is `input`. A board on `drop_off_a` is inside
  `drop_off_a` -> location `zone` -> never a pick candidate, even with a new per-frame id (the loop seen with
  `pick_one.py`). Needs the zone known (C2, C8, tag never covered) and the boards on it seen (C1, C4).
- **Processing zones:** a state machine per zone from what the camera sees inside it: `EMPTY -> LOADED` (robot
  placed a board) `-> PROCESSING` (zone empty for 2 s: the operator took the board) `-> DONE` (a board back for 2 s).
  Needs C1, C4, C5.
- **Drop-off stacks:** we count the boards we put on each zone (stack top = sheet `top_z` + count x board thickness).
  After each place we wait for the camera to show the board there (settle check: C1, C4).
- **Place orientation:** we put a board with its long side along the zone yaw -> C3 gives the sheet's long side,
  independent of the variant.
- **Freshness:** we decide only on data newer than the last robot action (C9: capture time). Occasional slow frames
  are tolerated by our timers.
