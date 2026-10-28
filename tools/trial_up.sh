#!/bin/bash
# Start the whole pipeline in ONE tmux session "trial" (one window per process), in the right order, waiting for each
# part to be ready; then open a terminal attached to it. Stop everything cleanly: tools/trial_down.sh
#
#   tools/trial_up.sh [debug|demo] [--sim] [--rviz] [--no-gui]
#     debug (default): verbose logs, node windows quiet (TM-27); demo: normal logging
#     --sim:    fake arm + fake gripper (pixi run sim) and fake_world (scenario_demo) instead of the robot + Vision 1
#     --rviz:   RViz with the motion stack
#     --no-gui: do not open a terminal window nor the demo page in the browser (attach later: tmux attach -t trial)
#
# Windows: 0 monitor | 1 motion | 2 vision (or fake) | 3 world | 4 skills | 5 ctrl | 6 cli (tc = task_cli function)
#          | 7 bridge (agent_bridge: $HK_LIVE_DIR/bridge/state.txt, events.txt, cmd pipe) | 8 demo-ui (vacuum failure page)
# In tmux: Ctrl+b then a window number = switch window, Ctrl+b d = detach (everything keeps running).
# The real robot does NOT move at start-up: motion only on a job / command (user's OK, CLAUDE.md section 3).
set -u
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"      # this worktree + its siblings, pixi, tmux (TM-34)
WT=$HK_WT; MW=$HK_MOTION_WT; VW=$HK_VISION1_WT; PX=$HK_PIXI; TMUX_BIN=$HK_TMUX_CMD
for d in "$MW" "$VW"; do
  [ -d "$d" ] || { echo "missing worktree $d (README: layout; or set HK_MOTION_WT / HK_VISION1_WT)"; exit 1; }
done
S=${HK_TMUX_SESSION:-trial}   # HK_TMUX_SESSION: offline tests next to a running trial
MODE=debug; SIM=0; RVIZ=false; GUI=1
for a in "$@"; do
  case "$a" in
    debug|demo) MODE=$a ;;
    --sim) SIM=1 ;;
    --rviz) RVIZ=true ;;
    --no-gui) GUI=0 ;;
    *) echo "unknown argument '$a'"; sed -n 4,9p "$0"; exit 1 ;;
  esac
done
if $TMUX_BIN has-session -t $S 2>/dev/null; then
  echo "tmux session '$S' already runs: attach with 'tmux attach -t $S' or stop it with tools/trial_down.sh"; exit 1
fi

# new log session (all windows join it: tools/live_env.sh without 'new')
source $WT/tools/live_env.sh new $MODE
ENV="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; ${ROS_LOCALHOST_ONLY:+export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY;} export HK_LIVE_ROOT=$HK_LIVE_ROOT;"
JOIN="source $WT/tools/live_env.sh > /dev/null"

win() {  # win <index> <name> <dir> <command>
  if [ "$1" = 0 ]; then $TMUX_BIN new-session -d -s $S -n "$2" -x 220 -y 50
  else $TMUX_BIN new-window -t $S:$1 -n "$2"; fi
  $TMUX_BIN send-keys -t $S:$1 "$ENV $JOIN; cd $3; clear; $4" Enter
}
ros() { (cd $WT && timeout 15 $PX run ros2 "$@" --no-daemon 2>/dev/null); }
wait_for() {  # wait_for <what> <seconds> <test command...>
  local what=$1 secs=$2; shift 2
  local end=$(( $(date +%s) + secs ))
  printf "waiting for %s " "$what"
  while [ "$(date +%s)" -lt $end ]; do
    if "$@" > /dev/null 2>&1; then echo "- ready"; return 0; fi
    printf "."; sleep 2
  done
  echo "- NOT ready after ${secs} s (check its window)"; return 1
}
svc() { ros service list | grep -qx "$1"; }
pub() { ros topic info "$1" | grep -q "Publisher count: [1-9]"; }

echo "starting tmux session '$S' ($MODE$( [ $SIM = 1 ] && echo ', SIM'))"
win 0 monitor $WT "$PX run python3 tools/live_monitor.py $( [ $MODE = demo ] && echo --lite )"
if ! $TMUX_BIN has-session -t $S 2>/dev/null; then echo "ERROR: tmux did not start (see the message above)"; exit 1; fi
if [ $SIM = 1 ]; then
  win 1 motion $MW "$PX run sim use_rviz:=$RVIZ"
else
  win 1 motion $MW "$PX run robot use_rviz:=$RVIZ"
fi
wait_for "motion (/motion/get_state)" 120 svc /motion/get_state
if [ $SIM = 1 ]; then
  win 2 fake $WT "$PX run ros2 run hk_fr5_task fake_world --ros-args -p contact_sim:=true \$HK_NODE_ARGS"
else
  win 2 vision $VW "$PX run -e gpu vision --ros-args -p model:=true"
fi
wait_for "vision (/vision/boards)" 90 pub /vision/boards
win 3 world $WT "$PX run ros2 run hk_fr5_task world_model --ros-args \$HK_NODE_ARGS"
wait_for "world model (/task/world)" 60 pub /task/world
win 4 skills $WT "$PX run ros2 run hk_fr5_task skill_server --ros-args \$HK_NODE_ARGS"
win 5 ctrl $WT "$PX run ros2 run hk_fr5_task task_main_controller --ros-args \$HK_NODE_ARGS"
wait_for "controller (/task/list_jobs)" 60 svc /task/list_jobs
win 6 cli $WT "tc() { $PX run python3 $WT/tools/task_cli.py \"\$@\"; }; echo 'tc help = all commands'; sleep 3; tc status"
win 7 bridge $WT "$PX run python3 tools/agent_bridge.py"
win 8 demo-ui $WT "$PX run python3 tools/vacuum_demo_ui.py"     # TM-47: 'Simulate vacuum failure' page
$TMUX_BIN select-window -t $S:6
UI_URL="http://localhost:${HK_DEMO_UI_PORT:-8765}"                  # TM-49: demo page address, logged + opened
wait_for "demo page ($UI_URL)" 20 curl -sf "$UI_URL/status"
echo "$UI_URL" > "$HK_LIVE_DIR/demo_ui_url.txt"
echo "all started. Logs: $HK_LIVE_DIR"
echo "vacuum-failure demo page: $UI_URL   (also in $HK_LIVE_DIR/demo_ui_url.txt)"
echo "attach: $WT/tools/trial_attach.sh   (Ctrl+b <n> = window n, Ctrl+b d = detach) | stop: $WT/tools/trial_down.sh"
if [ $GUI = 1 ] && [ -n "${DISPLAY:-}" ]; then
  gnome-terminal --maximize --title="trial ($MODE)" -- $TMUX_BIN attach -t $S > /dev/null 2>&1 &
  command -v xdg-open > /dev/null && xdg-open "$UI_URL" > /dev/null 2>&1 &              # TM-49: the demo page
fi
