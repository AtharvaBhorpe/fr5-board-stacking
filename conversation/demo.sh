#!/usr/bin/env bash
# One isolated localhost-only mock demo. Ctrl+C stops only processes launched here.
set -euo pipefail
cd "$(dirname "$0")/.."
VISION=$PWD
ROBOT=${FR5_DIR:-$VISION/../fr5-board-stacking}
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 CONVERSATION_SIM=1
children=()
cleanup() {
  for pid in "${children[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
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
pixi run python - <<'PY'
import time, rclpy
rclpy.init()
n = rclpy.create_node('demo_domain_preflight')
end = time.monotonic() + 2
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
assert n.count_publishers('/robot_description') == 0, 'Domain 62 already has a robot; close the existing demo first'
assert n.count_publishers('/vision/boards') == 0, 'Domain 62 already has vision; close the existing demo first'
n.destroy_node(); rclpy.shutdown()
PY
start sim pixi run --manifest-path "$ROBOT/pixi.toml" sim use_rviz:=false use_fake_hardware:=true
if [[ $# -eq 0 ]]; then
  mkdir -p /tmp/robot-batch-demo
  rm -f /tmp/robot-batch-demo/scene.png
  start world pixi run -e gpu python conversation/demo_world.py
  for _ in {1..30}; do
    [[ -f /tmp/robot-batch-demo/scene.png ]] && break
    sleep 1
  done
  [[ -f /tmp/robot-batch-demo/scene.png ]] || { echo 'Demo scene startup failed; see /tmp/conversation-world.log'; exit 1; }
fi
start vision pixi run -e gpu vision --ros-args -p "device:=${1:-/tmp/robot-batch-demo/scene.png}" -p model:=true
start bridge pixi run conversation --ros-args -p allow_replay:=true
start rviz bash robot/rviz.sh
echo 'SIMULATION ONLY: ROS domain 62. Wait for SIMULATION and vision in RViz.'
echo 'Type: pick two variant B boards / sort all visible boards / status / stop.'
echo 'Hold to talk records only while held (20-second maximum). Ctrl+C here stops this demo.'
wait -n
