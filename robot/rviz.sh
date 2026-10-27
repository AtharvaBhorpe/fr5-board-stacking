#!/bin/bash
# Demo view: RViz with the camera (detections drawn), robot + planning scene, board markers and the pick panel.
# Run next to `pixi run robot` (or sim) in fr5-board-stacking and `pixi run vision` here. Builds the panel first
# (in the robot env: its RViz, compilers and pick_one's interfaces). Tip: `pixi run robot use_rviz:=false` = 1 RViz.
V=$(cd "$(dirname "$0")/.." && pwd)
FR5=${FR5_DIR:-$V/../fr5-board-stacking}
cd "$FR5" || { echo "robot workspace not found: $FR5 (set FR5_DIR)"; exit 1; }
exec pixi run bash -c "cd '$V' && colcon build --base-paths rviz_panel --build-base build_panel \
  --install-base install_panel > /tmp/vision1_panel_build.log 2>&1 \
  || { tail -40 /tmp/vision1_panel_build.log; echo 'panel build failed, full log: /tmp/vision1_panel_build.log'; exit 1; }
  source install_panel/setup.bash && exec rviz2 -d rviz/demo.rviz"
