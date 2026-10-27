#!/bin/bash
# Sim test of the "requested board is covered" path (needs models/boards-seg.onnx and `pixi run sim` running).
# Vision replays samples/stack_varb_on_vara.png (VARb pasted over the reachable VARa); once pick_one starts moving
# the VARb to the temp spot, the replayed file is swapped for the uncovered scene, as the real camera would see it.
cd "$(dirname "$0")/.." || exit 1
VISION=$PWD; ROBOT=$VISION/../fr5-board-stacking; IMG=/tmp/sim_clear_scene.png
cp samples/stack_varb_on_vara.png $IMG
setsid bash -c "cd '$VISION' && exec pixi run -q vision --ros-args -p device:=$IMG -p model:=true" > /tmp/sim_clear_vision.log 2>&1 &
sleep 8
(cd "$ROBOT" && yes '' | timeout 600 pixi run -q python3 "$VISION/robot/pick_one.py" --place --variant VARa --temp -0.60 0.12 --speed 0.3) \
  > /tmp/sim_clear.log 2>&1 &
pp=$!
until grep -q "moving it to the temp spot" /tmp/sim_clear.log || ! kill -0 $pp 2>/dev/null; do sleep 1; done
cp samples/composite_vara_dest.png $IMG
wait $pp
pkill -f "device:=$IMG"; sleep 1
cat /tmp/sim_clear.log | grep -vE "^\s*$"
grep -q "moving it to the temp spot" /tmp/sim_clear.log && tail -1 /tmp/sim_clear.log | grep -q done && echo "PASS clear + place" || { echo FAIL; exit 1; }
