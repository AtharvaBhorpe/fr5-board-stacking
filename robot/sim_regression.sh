#!/bin/bash
# Unattended simulation test: replays recorded overhead images through the vision node and runs pick_one.py
# (pick + put back, and pick + place on the template) for each case. Prints PASS / FAIL per case.
#
#   terminal 1 (fr5-board-stacking):  pixi run sim          # NEVER together with `pixi run robot`
#   terminal 2 (vision1-overhead):    robot/sim_regression.sh
#
# 01-03 VARa: the board lies 1.01 m from the base (beyond the FR5 reach) -> must be refused before any motion.
# samples/composite_vara_dest.png = left half of logitech_dataset_1 scene_0000 (templates) + right half of
# samples/06.jpg (VARa board in reach), same camera pose: the only image with a reachable VARa board + its template.
# Needs: fr5-board-stacking next to vision1-overhead, built (pixi run build). No other vision node running.
cd "$(dirname "$0")/.." || exit 1
VISION=$PWD; ROBOT=$VISION/../fr5-board-stacking
CASES=(  # image | pick_one.py args | expected last line
  "samples/06.jpg|--variant VARa|done"
  "samples/06.jpg|--variant VARb|done"
  "samples/06.jpg|--variant VARc|done"
  "samples/composite_vara_dest.png|--place --variant VARa|done"
  "samples/01.jpg|--place --variant VARa|no reachable board"
  "samples/02.jpg|--place --variant VARa|no reachable board"
  "samples/03.jpg|--place --variant VARa|no reachable board"
  "samples/05.jpg|--place --variant VARb|done"
  "samples/05.jpg|--place --variant VARc|done"
  "samples/01.jpg|--place --variant VARc --stack 1|done"
  "samples/06.jpg|--place --variant VARa|--place: no board whose destination template (tag) is visible"
)
pass=0; fail=0
for c in "${CASES[@]}"; do
  IFS='|' read -r img args want <<< "$c"
  setsid bash -c "cd '$VISION' && exec pixi run -q vision --ros-args -p device:=$img" > /tmp/sim_regression_vision.log 2>&1 &
  vp=$!; sleep 7
  out=$(cd "$ROBOT" && printf '\n\n' | timeout 400 pixi run -q python3 "$VISION/robot/pick_one.py" $args --speed 0.3 2>&1)
  kill -- -$vp 2>/dev/null; sleep 2
  last=$(echo "$out" | grep -v '^\s*$' | tail -1 | sed 's/^>>> .*   //')
  if [[ "$last" == *"$want"* ]]; then pass=$((pass+1)); echo "PASS  $img $args"
  else fail=$((fail+1)); echo "FAIL  $img $args"; echo "$out" | tail -5 | sed 's/^/      /'; fi
done
echo "$pass passed, $fail failed"
[ $fail -eq 0 ]
