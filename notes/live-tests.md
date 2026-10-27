# Real-robot test plan (first slot, 2026-10-27)

Run by Claude through `tools/task_cli.py`, the user at the e-stop. Bring-up + checks: `notes/live-run.md` (debug
session, monitor quiet). **Every motion only after the user's OK; speeds as in skills.yaml (<= 20 %).**
Hands are NOT detected (Vision 1 sends no `obstacles[]` yet): nobody reaches into the work area while the arm moves.
Order = risk order: each test assumes the previous one passed. Stop a test at the first unexpected thing, read the
logs, decide. Per test: `task_cli.py status` / `world` before and after; note results + tuned values in decision-log.

Material: 3 red (VARc), 4-5 white (VARa / VARb). Zones: processing_1, drop_off_a, drop_off_b (brown cardboard,
ArUco DICT_5X5_100 tag beside the board area), fault zone (our config).

## 0. Setup (no motion)

- [ ] Tags = ArUco DICT_5X5_100 (not QR) with known ids; Vision 1 tag table (`vision1/table.py` ZONES, the teammate's
      worktree, local edit) = per tag: name (processing_1 / drop_off_a / drop_off_b), kind, offset, rotation, size
      (board area long x short), top_z (touch-off, TM-O9).
- [ ] Fault zone: world.yaml `fixed_zones` fault x / y (+ size), skills.yaml `fault_drop_z` (TM-O5).
- [ ] `control home` (first motion, OK): tcp at home_pose above the profile; `status` tcp = home_xy (TM-O2).

## T0 Perception only (no motion, ~5 min)

Layout: one board of each variant alone on the pick table; a 2-stack (one variant); one red VARc ON each zone sheet;
one white board 3-4 cm OUTSIDE a zone sheet edge. Robot at home.
Check (`world`, monitor): every board seen once, right variant, size_ok; boards on sheets = zone, the edge board =
input; zones have the measured base_z; VIS 5 Hz, gap < 0.5 s. Then by hand: slide a board 2 cm (same id), remove one
(gone after lost_after 2 s), cover a zone tag 5 s (zone stays), put it back.
Exposes: red on brown (C4, TM-O7), wrong zone sizes / offsets / yaw, zone margin vs real gaps, real jitter vs
match_dist, home pose hiding boards from the overhead camera, low conf.
Pass: all of the above as described. Then clear the sheets for T1.

## T1 One board, then a stack of 2 on a drop-off (first pick / place)

1. `transfer VARa drop_off_a 1` - one white board, alone on the pick table.
2. `transfer VARb drop_off_a 1` - a second white board onto the same stack (mixed sizes).
Watch (skill log): cup above the board centre at hover (calibration + tcp yaw), contact-search seal z vs table + 5 mm,
unstick lift + attach, OMPL carry, place: tcp z = base_z + n x 5 mm - 1 mm, blow-off 2 s, settle check (camera sees
it within 2 cm), stack_count / stack_top_z, stack box.
Exposes: x / y offset between camera and robot, real board thickness vs 5 mm, seal overtravel (vacuum latency x
search speed), place accuracy / board sliding at blow-off, NOT_VERIFIED from jitter, stack height maths on a sheet.
Pass: both DONE, count 2, both boards on the sheet, aligned with the zone yaw.

## T2 Red input stack of 3 -> other drop-off (stack heights + red detection)

Layout: 3 VARc stacked on the pick table. `transfer VARc drop_off_b 0` (all).
Watch: seal z 15 / 10 / 5 mm above the table (contact search per layer), after each pick the next board appears at the
same spot (new id, input); place red on brown (settle check = C4 in motion); drop_off_b count 3, top = base + 15 mm.
Exposes: stacked boards not flat / offset, the world losing / duplicating the stack after a pick (stale-frame drop),
red-on-brown detection while placing, the arm hiding the stack during hover.
Pass: DONE 3 delivered, drop_off_b count 3, nothing left on the pick table.

## T3 Processing with the operator + a parallel transfer (the real scheduling)

Layout: 1 VARa single + 2 white boards of another variant on the pick table.
Jobs: `process VARa processing_1 drop_off_a 1 10` (prio 10) + `transfer VARb drop_off_b 0 5` (prio 5).
Operator: when the robot placed the VARa in processing_1 and left, take it away (>= 2 s, gone_after), bring the SAME
board back after ~10 s, put it down roughly where it was.
Expected order: VARa -> processing_1, VARb -> drop_off_b while it is processed, VARa (DONE) -> drop_off_a first, then
the rest of the VARb.
Watch: zone states LOADED -> PROCESSING -> DONE (monitor WRLD), known-top pick from the sheet (base_z + 5 mm, press 4 mm),
the board put back shifted / rotated keeps its id, decisions made only on fresh frames.
Exposes: gone_after / back_after vs real Vision 1 flicker (TM-O6), the arm mask over processing_1, the operator's
hand counted as a board, known-top pick errors (GRIP_FAILED -> retry 3 mm deeper), scheduling order.
Pass: both jobs DONE, order as expected, processing_1 EMPTY at the end.

## T4 Recovery commands on real hardware (if time)

1. `transfer any drop_off_a 1`; while it HOLDS the board: `pause` -> the board is placed first, nothing moves; `resume`.
2. Next transfer; while holding: `stop` -> holds; `resume` -> release_safe into the FAULT zone (drop from fault_drop_z).
3. `clear drop_off_a drop_off_b 1` -> pick the top board of a drop-off stack (known top from our count) and move it.
Exposes: fault zone position / drop height, a pick from a real stack top with accumulated height error, pause / stop
semantics on the real robot.
Pass: each step as described; the board in the fault zone leaves the world.

## T5 Perception edge cases (optional)

A board rotated ~45 deg; a board at the far reach corner; a non-variant object (UNKWN) on the pick table (must never be
picked by `transfer any`); a board moved by hand while the robot hovers over it (BOARD_MOVED / re-decision).

## After each test

`~/hackathon/live_logs/current/`: skill_server trace (seal z, place z, durations), monitor `vision_stats`, alerts.
Note: seal z offsets, place offsets (camera vs target), NOT_VERIFIED / GRIP_FAILED counts, any tuned value.
