#!/usr/bin/env bash
# Web UI on the simulated main controller (tools/trial_up.sh --sim). http://localhost:8088. Ctrl+C stops everything.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."
CTRL=${HK_TASK_WT:-$PWD/../wt/feat-task-main-controller}
robot="${FR5_DIR:-$PWD/../fr5-board-stacking}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-62} ROS_LOCALHOST_ONLY=1
python3 web/setup_tls.py
npm --prefix web run build
"$CTRL/tools/trial_up.sh" --sim --no-gui
trap 'trap - EXIT; "$CTRL/tools/trial_down.sh"; kill 0' EXIT INT TERM
pixi run conversation --ros-args -p browser_speech:=true > /tmp/workcell-bridge.log 2>&1 &
pixi run --manifest-path "$robot/pixi.toml" python "$PWD/web/gateway.py" > /tmp/workcell-gateway.log 2>&1 &
echo 'Web simulation: http://localhost:8088. Model services: conversation/model.sh (separate).'
wait -n
