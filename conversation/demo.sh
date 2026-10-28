#!/usr/bin/env bash
# Simulation demo: main controller (fake arm, gripper, camera: tools/trial_up.sh --sim) + chat bridge + RViz panel.
# Model questions need conversation/model.sh running; robot commands do not. Ctrl+C stops everything.
set -euo pipefail
cd "$(dirname "$0")/.."
CTRL=${HK_TASK_WT:-$PWD/../wt/feat-task-main-controller}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-62} ROS_LOCALHOST_ONLY=1
"$CTRL/tools/trial_up.sh" --sim --no-gui
trap 'trap - EXIT; "$CTRL/tools/trial_down.sh"; kill 0' EXIT INT TERM
pixi run conversation > /tmp/conversation-bridge.log 2>&1 &
bash robot/rviz.sh > /tmp/conversation-rviz.log 2>&1 &
echo "SIMULATION, ROS domain $ROS_DOMAIN_ID. Type: move two variant B boards to drop off b / status / pause / stop"
wait -n
