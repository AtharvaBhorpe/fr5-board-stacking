#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1 CONVERSATION_SIM=1
robot="${FR5_DIR:-$PWD/../fr5-board-stacking}"
python3 web/setup_tls.py
npm --prefix web run build
children=()
cleanup() { for pid in "${children[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
start() { local name=$1; shift; setsid "$@" > "/tmp/workcell-$name.log" 2>&1 & children+=("$!"); }
pixi run python - <<'PY'
import rclpy,time
rclpy.init(); n=rclpy.create_node('web_preflight')
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
assert not n.count_publishers('/robot_description'), 'Close existing simulation before web launch'
n.destroy_node();rclpy.shutdown()
PY
start sim pixi run --manifest-path "$robot/pixi.toml" sim use_rviz:=false use_fake_hardware:=true
mkdir -p /tmp/robot-batch-demo
rm -f /tmp/robot-batch-demo/scene.png
start world pixi run -e gpu python conversation/demo_world.py
for _ in {1..30}; do [[ -f /tmp/robot-batch-demo/scene.png ]] && break; sleep 1; done
start vision pixi run -e gpu vision --ros-args -p device:=/tmp/robot-batch-demo/scene.png -p model:=true
start bridge pixi run conversation --ros-args -p allow_replay:=true -p browser_speech:=true
start gateway pixi run --manifest-path "$robot/pixi.toml" python "$PWD/web/gateway.py"
echo 'Web simulation: http://localhost:8088 (no RViz required). Model services must be started separately.'
wait -n
