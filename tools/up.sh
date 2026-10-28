#!/bin/bash
# Start the whole workcell in ONE tmux session "fr5" (one window per process), in order, and wait for each part.
# Stop everything: tools/down.sh (pixi run down).
#
#   pixi run up [debug|demo] [--sim] [--rviz] [--no-gui]
#     debug (default): every step logged, node windows quiet. demo: normal logging.
#     --sim:    fake arm + fake gripper + fake camera (fake_world). No robot, no cameras.
#     --rviz:   RViz with the motion stack.
#     --no-gui: do not open a terminal or the browser (attach later: pixi run attach).
#   WORKCELL_REAL=1 pixi run up demo   # the web app may send jobs to the REAL robot (default: refused)
#
# Windows: 0 monitor | 1 motion | 2 vision (or fake) | 3 world | 4 skills | 5 ctrl | 6 cli (tc ...) | 7 bridge
#          | 8 demo-ui (:8765) | 9 llm (Gemma :8770) | 10 speech (:8766) | 11 chat | 12 web (:8088) | 13 wrist (real only)
# Nothing moves at start-up: the robot moves only for a job.
set -eu
cd "$(dirname "$(readlink -f "$0")")/.."
R=$PWD
S=fr5
T="env -u TERMINFO TERM=xterm-256color tmux -L fr5"   # own tmux server: pixi tmux, kitty-safe TERM
MODE=debug; SIM=0; RVIZ=false; GUI=1
for a in "$@"; do
  case "$a" in
    debug|demo) MODE=$a ;;
    --sim) SIM=1 ;;
    --rviz) RVIZ=true ;;
    --no-gui) GUI=0 ;;
    *) echo "unknown argument '$a'"; sed -n 4,10p "$0"; exit 1 ;;
  esac
done
if $T has-session -t $S 2>/dev/null; then
  echo "session '$S' already runs: 'pixi run attach' to look, 'pixi run down' to stop"; exit 1
fi
[ -f install/setup.sh ] && [ -f web/dist/index.html ] || { echo "not built: run 'pixi run build' first"; exit 1; }
[ -f web/.local/server.crt ] || python3 web/setup_tls.py   # HTTPS certificate for phones / tablets (port 8443)
# simulation: stay off the network (no talk with a real robot stack on the same ROS domain)
[ $SIM = 1 ] && export ROS_LOCALHOST_ONLY=1   # the ROS activation sets 0: force it

# log session: live_logs/session_<date>_<time>/ (symlink live_logs/current); ros/ = node logs, monitor/ = streams
LOG=$R/live_logs/session_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG/ros" "$LOG/monitor"
ln -sfn "$LOG" "$R/live_logs/current"
if [ $MODE = demo ]; then VERBOSE=0; NODE_ARGS=""; else VERBOSE=1; NODE_ARGS="--disable-stdout-logs"; fi
ENV="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} WORKCELL_REAL=${WORKCELL_REAL:-0} HK_LIVE_DIR=$LOG ROS_LOG_DIR=$LOG/ros"
ENV="$ENV HK_VERBOSE=$VERBOSE HK_NODE_ARGS='$NODE_ARGS'; ${ROS_LOCALHOST_ONLY:+export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY;}"

win() {  # win <index> <name> <command>
  if [ "$1" = 0 ]; then $T new-session -d -s $S -n "$2" -x 220 -y 50
  else $T new-window -t $S:$1 -n "$2"; fi
  $T send-keys -t $S:$1 "$ENV cd $R; clear; $3" Enter
}
ros() { timeout 15 pixi run ros2 "$@" --no-daemon 2>/dev/null; }
wait_for() {  # wait_for <what> <seconds> <test command...>
  local what=$1 end=$(( $(date +%s) + $2 )); shift 2
  printf "waiting for %s " "$what"
  while [ "$(date +%s)" -lt $end ]; do
    if "$@" > /dev/null 2>&1; then echo "- ready"; return 0; fi
    printf "."; sleep 2
  done
  echo "- NOT ready (look at its window: pixi run attach)"; return 1
}
svc() { ros service list | grep -qx "$1"; }
pub() { ros topic info "$1" | grep -q "Publisher count: [1-9]"; }
url() { curl -sf "$1" > /dev/null; }

echo "starting session '$S' ($MODE$( [ $SIM = 1 ] && echo ', SIM'), logs $LOG)"
win 0 monitor "pixi run python tools/live_monitor.py $( [ $MODE = demo ] && echo --lite )"
$T has-session -t $S 2>/dev/null || { echo "ERROR: tmux did not start"; exit 1; }
win 9 llm "pixi run -e models llm"               # loads while the robot stack starts
if [ $SIM = 1 ]; then win 1 motion "pixi run sim use_rviz:=$RVIZ"; else win 1 motion "pixi run robot use_rviz:=$RVIZ"; fi
wait_for "motion (/motion/get_state)" 120 svc /motion/get_state
if [ $SIM = 1 ]; then
  win 2 fake "pixi run ros2 run hk_fr5_task fake_world --ros-args -p contact_sim:=true \$HK_NODE_ARGS"
else
  win 2 vision "pixi run vision --ros-args -p model:=true"
  win 13 wrist "pixi run wrist"
fi
wait_for "vision (/vision/boards)" 90 pub /vision/boards
win 3 world "pixi run ros2 run hk_fr5_task world_model --ros-args \$HK_NODE_ARGS"
wait_for "world model (/task/world)" 60 pub /task/world
win 4 skills "pixi run ros2 run hk_fr5_task skill_server --ros-args \$HK_NODE_ARGS"
win 5 ctrl "pixi run ros2 run hk_fr5_task task_main_controller --ros-args \$HK_NODE_ARGS"
wait_for "controller (/task/list_jobs)" 60 svc /task/list_jobs
win 6 cli "tc() { pixi run tc \"\$@\"; }; echo 'tc help = all commands'; sleep 3; tc status"
win 7 bridge "pixi run python tools/agent_bridge.py"
win 8 demo-ui "pixi run python tools/vacuum_demo_ui.py"
wait_for "Gemma (:8770)" 180 url localhost:8770/health
win 10 speech "pixi run -e models speech"
wait_for "speech server (:8766)" 180 url localhost:8766/health
win 11 chat "pixi run conversation"
win 12 web "pixi run web"
wait_for "web app (:8088)" 30 url localhost:8088
wait_for "vacuum demo page (:8765)" 20 url localhost:8765/status
$T select-window -t $S:6
echo "all started. Logs: $LOG"
echo "web app: http://localhost:8088 | vacuum-failure page: http://localhost:8765"
echo "look: pixi run attach (Ctrl+b <n> = window n, Ctrl+b d = detach) | stop: pixi run down"
if [ $GUI = 1 ] && [ -n "${DISPLAY:-}" ]; then
  gnome-terminal --maximize --title="fr5 ($MODE)" -- $T attach -t $S > /dev/null 2>&1 &
  xdg-open http://localhost:8088 > /dev/null 2>&1 &
  xdg-open http://localhost:8765 > /dev/null 2>&1 &
fi
