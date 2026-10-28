# Vision 1 change request (from `feat/task-main-controller`, 2026-10-27)

For: the Vision 1 / Vision 2 owner and their agent, branch `feat/vision1-overhead`.
From: the main controller (`feat/task-main-controller`): world model + skills + job scheduler that consumes
`/vision/boards`. This is the written version of what was agreed verbally in the lab. **It is meant to be the last
interface change on the vision branch** (model training excepted). Nothing here touches the robot branch
`feat/robot-motion-gripper` (frozen). All example numbers are examples, not real positions.

---

## 0. Read this first (agent)

Every item has an **owner tag**. Only CODE and CONFIG items are work in the vision branch.
This file is the agent's whole task list.

| Tag | Who | What to do |
|---|---|---|
| **CODE** | agent | change code in the vision branch |
| **CONFIG** | agent | extend the existing tag table (`table.DEST` or its replacement) with entries / columns: no new logic |
| **NO-OP** | nobody | already true today: keep it, do not rewrite it |
| **HUMAN** | people at the table | details are in a separate file for the people. **Agent: write NO code for it** |
| **LATER** | nobody, now | agreed for later. **Agent: do NOT implement it now** |

**REQUIRED** = the main controller cannot run the demo without it. **NICE** = optional.

Rules for the agent:
1. Implement only the items tagged CODE / CONFIG. Section 4 (LATER) is not work. Items C13 / C14 are human
   tasks (detector mode, table layout), kept in a separate file for the people: write no code for them.
   C15 = the zone geometry VALUES: the people measure them in the lab and enter them into your tag table;
   until then use placeholders.
2. **Do not implement main-controller logic in the vision branch**, not even as a helper, a check or a warning:
   board tracking / stable ids, "which zone is this board in", ownership, stack counting / heights, processing-zone
   state, re-pick prevention, layout checks (gaps, overlaps, covered tags), fault zone. Vision 1 reports what it
   sees in each frame plus the zone memory (C8); the main controller decides everything else.
3. No new fields beyond the ones listed in section 2. In particular: no `boards[].zone_name` / zone assignment for
   boards, no sheet outline detection (zone geometry comes from the tag + config, C3).
4. Do not touch `feat/robot-motion-gripper`.

---

## 1. Summary

| # | Change | Tag | Status | Work today |
|---|---|---|---|---|
| C1 | Board detection ROI = **the whole table** (not only the pick area) | CODE | REQUIRED | ROI change |
| C2 | New `zones[]` list: drop-off zones `drop_off_a`, `drop_off_b`, ... and processing zones `processing_1`, `processing_2`, ..., one QR / ArUco tag each. **Generic names**: no variant <-> zone mapping (any variant may go to any drop-off zone) | CODE + CONFIG | REQUIRED | reuse the `destinations[]` code; tag id -> name in the tag table |
| C3 | Zone `x`, `y` = **centre of the board area** (not the tag centre), `yaw` = direction of the zone sheet's **long side** (modulo pi), `base_link`. Computed exactly like `DEST` today: tag pose + fixed per-tag offset and rotation from the tag table. **No sheet detection**. The tag table (offsets, sizes, C15) is owned by the vision branch | CONFIG | REQUIRED | extend the tag table: per tag name, kind, offset, rotation, `size`, `top_z` |
| C4 | A board lying **on** a zone sheet is reported in `boards[]` with its own outline: not rejected as `has_tag`, not merged with the sheet | CODE | REQUIRED | today the whole tagged blob is rejected |
| C5 | The empty zone sheet itself is **never** reported as a board | NO-OP | REQUIRED | today: `rejected` / `has_tag`; must stay true after C4 |
| C6 | Every board-sized object inside a zone reported, `UNKWN` if it matches no variant (processed boards may look different) | LATER | LATER | first live runs: the same variant comes back (main controller `expanded_detection: false`) |
| C7 | `zones[].kind`: `drop_off` or `processing` | CONFIG | REQUIRED | one more column in the tag table |
| C8 | `zones[]` memory exactly like `destinations[]` today (~60 s after the tag was last seen, `age`, median pose) | CODE (reuse) | REQUIRED | same code as `DEST_MEMORY` / `median_dest`, one memory per zone **name** (today keyed by variant) |
| C9 | `stamp` = **capture time** of the frame (Unix s) | CODE | REQUIRED | one line: today `now = time.time()` is taken after `detect()` (up to ~0.9 s late); take it right after `cap.read()` |
| C10 | `corners_xy` stay 4 corners in order around the board | NO-OP | REQUIRED | - |
| C11 | Rate >= 3 Hz average with the whole-table ROI (GPU env) | CODE (measure) | REQUIRED | today ~5 Hz; check after C1 |
| C12 | Everything else unchanged: `calibrated`, `boards[]` fields, `occluded[]`, `rejected[]`, the `/vision/detect` service, one publisher only | NO-OP | REQUIRED | - |
| C13 - C14 | Detector mode (a message), table layout rules: for the people | HUMAN | REQUIRED | **no code** |
| C15 | Zone geometry per tag in your tag table: board-area `size` [long, short] (m) and `top_z` = sheet top in `base_link` (m, like `DEST_SUPPORT_Z` today). Values measured by the people in the lab (brown cardboard cut-outs); placeholders until then | CONFIG | REQUIRED | config values only |
| C16 | `destinations[]` may stay for `pick_one.py`; the main controller reads `zones[]` only | NO-OP | NICE | - |
| C17 | `boards[].zone` (`pick` / `drop` by the y sign): keep or drop, the main controller ignores it | NO-OP | NICE | no change needed |
| C18 | `zones[].size` and `zones[].top_z` in the JSON, copied from the tag table (C15). Not measured from the image | CODE | REQUIRED | two fields |
| C19 | `zones[].tag_id`, `zones[].frames` (debug) | CODE | NICE | trivial, like `destinations[]` |

Not needed (the main controller does it): tracked board ids, pixel -> base_link conversion (we use your
`corners_xy`), zone membership of boards, a fault zone, obstacles / hands, "clear the boards on top first",
board / stack heights (`z`: only the configured sheet `top_z`, C15), layout checks.

---

## 2. JSON proposal (`/vision/boards`, `std_msgs/String`)

Unchanged top level + one new key `zones`:

```json
{
  "stamp": 1793104812.53, "source": "C920", "calibrated": true, "image_size": [1920, 1080],
  "boards":   [ "...unchanged, see 3..." ],
  "occluded": [ "...unchanged..." ],
  "rejected": [ "...unchanged..." ],
  "destinations": [ "...optional, for pick_one.py (C16)..." ],
  "zones": [
    {"name": "drop_off_a",   "kind": "drop_off",   "x": -0.4500, "y":  0.3000, "yaw": 1.5708, "age": 0.0,
     "size": [0.32, 0.24], "top_z": -0.055, "frames": 15, "tag_id": 1},
    {"name": "drop_off_b",   "kind": "drop_off",   "x": -0.4500, "y":  0.6000, "yaw": 1.5708, "age": 3.2,
     "size": [0.32, 0.24], "top_z": -0.055, "frames": 15, "tag_id": 2},
    {"name": "processing_1", "kind": "processing", "x": -0.3000, "y": -0.4500, "yaw": 0.0,    "age": 0.0,
     "size": [0.32, 0.24], "top_z": -0.055, "frames": 12, "tag_id": 20}
  ]
}
```

| Field | Type | Meaning | Status |
|---|---|---|---|
| `name` | str | from the tag table; **unique, the same every run** (one tag per zone) | REQUIRED |
| `kind` | str | `drop_off` or `processing`, from the tag table | REQUIRED |
| `x`, `y` | m, `base_link` | centre of the board area = tag pose + per-tag offset (like `DEST`) | REQUIRED |
| `yaw` | rad, `base_link` | sheet's long side = tag yaw + per-tag rotation (like `DEST`), modulo pi | REQUIRED |
| `age` | s | since the tag was last seen; the zone leaves the list after the memory (~60 s) | REQUIRED |
| `size` | [m, m] | board area [long, short] along / across `yaw`, from the tag table (C15) | REQUIRED |
| `top_z` | m, `base_link` | top of the zone sheet (a board placed on it lies here), from the tag table (C15) | REQUIRED |
| `frames` | int | detections in the median pose | NICE |
| `tag_id` | int | the defining tag (debug) | NICE |

---

## 3. `boards[]`: fields kept, what changes

Used by the main controller (keep exactly as now, NO-OP):

| Field | Used for (on our side) |
|---|---|
| `variant` (`VARa` / `VARb` / `VARc` / `UNKWN`) | job selection; `UNKWN` is never picked by a normal job |
| `corners_xy` (4 x [x, y], `base_link`, in order: C10) | outline -> centre, yaw, size check, zone membership |
| `conf` | detection score (model mode); missing in classic mode is fine |
| `inside_calibration` | information only |

Not used (keep or not, no change needed): `id`, `u`, `v`, `corners` (pixels), `z`, `yaw` / `yaw_deg`, `zone`.

What changes (CODE): C1 (boards anywhere on the table, also on zone sheets and on stacks in zones) and C4 (a board
on a sheet gets its own outline). C5 stays true. Unchanged (NO-OP): only the top board of a stack is reported; the
board held on the suction cup may be reported or not (we mask the area under the tool). C6 is LATER: do not
implement now.

---

## 4. LATER (agent: do NOT implement now; logged on our side)

- C6: report processed / unknown-looking boards inside zones.
- "Clear the boards on top first" from `occluded[]` / `covered_by` (when the model is robust on stacks).
- Hands / obstacles: a separate `obstacles[]` with `corners_xy`, only if your detector can tell a hand from the arm
  and from overlapping boards.
- Vision 2 (wrist D405) as a ROS service instead of the Python class (only if needed outside the pick skill).

## 5. What does NOT change for you

- `pick_one.py`, the RViz panel and `destinations[]` can keep working.
- The main controller only subscribes to `/vision/boards` (and may call `/vision/detect` for a fresh frame).

Questions / changes to this request: the main-controller owner (GitHub: vortexcv).
