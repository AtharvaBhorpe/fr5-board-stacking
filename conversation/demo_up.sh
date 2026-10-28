#!/usr/bin/env bash
# Chat + web stack in tmux session "chat": Gemma (llama.cpp), model server (Parakeet, Phonon), bridge, web gateway.
# Start the main controller separately with the SAME ROS settings: tools/trial_up.sh [--sim] (its README).
#   conversation/demo_up.sh                  # web chat: simulation only
#   WORKCELL_REAL=1 conversation/demo_up.sh  # web chat may command the REAL robot
# Web: http://localhost:8088 | look: tmux attach -t chat | stop: tmux kill-session -t chat
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."
V=$PWD
CTRL=${HK_TASK_WT:-$V/../wt/feat-task-main-controller}
ROBOT=${FR5_DIR:-$V/../fr5-board-stacking}
T="env -u TERMINFO TERM=xterm-256color ${HK_TMUX:-$(command -v tmux || echo "$HOME/.pixi/bin/tmux")}"
# same graph as trial_up.sh: ROS_DOMAIN_ID default 0, ROS_LOCALHOST_ONLY only when set
ENV="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} WORKCELL_REAL=${WORKCELL_REAL:-0}; ${ROS_LOCALHOST_ONLY:+export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY;} cd $V;"
wait_for() {  # url, name
  for _ in $(seq 1 180); do curl -sf "$1" >/dev/null && return; sleep 1; done
  echo "ERROR: $2 did not start (tmux attach -t chat)"; exit 1
}
if $T has-session -t chat 2>/dev/null; then echo "session 'chat' already runs (tmux attach -t chat)"; exit 1; fi
$T new-session -d -s chat -n gemma -x 200 -y 50 "$ENV conversation/gemma_qat.sh; read"
wait_for localhost:8770/health "Gemma (llama.cpp)"   # the adapter fails (503) if Gemma is still loading
$T new-window -t chat -n model "$ENV conversation/model.sh --backend llama --asr parakeet --background-asr; read"
wait_for localhost:8766/health "model server"
$T new-window -t chat -n bridge "$ENV pixi run conversation --ros-args -p browser_speech:=true; read"
$T new-window -t chat -n web "$ENV pixi run --manifest-path $ROBOT/pixi.toml bash -c \
  'source $CTRL/install/local_setup.bash && exec python $V/web/gateway.py'; read"
wait_for localhost:8088 "web gateway"
echo "chat stack up (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, real robot web control: ${WORKCELL_REAL:-0})"
echo "web: http://localhost:8088 | tmux attach -t chat | stop: tmux kill-session -t chat"
