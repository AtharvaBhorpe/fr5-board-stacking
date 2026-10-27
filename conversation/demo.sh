#!/usr/bin/env bash
# One isolated localhost-only mock demo. Ctrl+C stops only processes launched here.
set -euo pipefail
cd "$(dirname "$0")/.."
VISION=$PWD
ROBOT=${FR5_DIR:-$VISION/../fr5-board-stacking}
export ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 CONVERSATION_SIM=1
children=()
cleanup() {
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
start() {
  local name=$1; shift
  setsid "$@" > "/tmp/conversation-$name.log" 2>&1 &
  children+=("$!")
}
if ! python -c 'import urllib.request; urllib.request.urlopen("http://127.0.0.1:8766", timeout=1)' 2>/dev/null; then
  echo 'Start conversation/model.sh in a separate terminal first (add --download for first use).'
  exit 1
fi
start sim pixi run --manifest-path "$ROBOT/pixi.toml" sim use_rviz:=false
start vision pixi run -e gpu vision --ros-args -p "device:=${1:-samples/05.jpg}" -p model:=true
start bridge pixi run conversation --ros-args -p allow_replay:=true
start rviz bash robot/rviz.sh
echo 'SIMULATION ONLY: ROS domain 61. Wait for SIMULATION and vision in RViz.'
echo 'Type: status / what do you see? / pick variant B. Confirm descents in the panel.'
echo 'Hold to talk records only while held (20-second maximum). Ctrl+C here stops this demo.'
wait
