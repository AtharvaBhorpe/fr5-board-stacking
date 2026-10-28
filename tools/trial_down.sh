#!/bin/bash
# Stop the tmux session "trial" cleanly: Ctrl+C into each window in the right order (our nodes first, then vision,
# motion, monitor last - it writes the log index at exit), wait, then close the session.
#   tools/trial_down.sh
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"      # tmux (TM-34)
TMUX_BIN=$HK_TMUX_CMD
S=${HK_TMUX_SESSION:-trial}
if ! $TMUX_BIN has-session -t $S 2>/dev/null; then echo "no tmux session '$S'"; exit 0; fi
for w in bridge ctrl skills world vision fake motion monitor; do
  if $TMUX_BIN list-windows -t $S -F '#W' | grep -qx $w; then
    echo "stopping $w"
    $TMUX_BIN send-keys -t $S:$w C-c
    sleep $([ $w = motion ] && echo 6 || echo 3)
  fi
done
$TMUX_BIN kill-session -t $S
sleep 1
left=$(pgrep -f "task_main_controller|skill_server|world_model|fake_world|vision1/node.py|move_group|ros2_control_node|live_monitor" | wc -l)
echo "session closed; processes still running: $left"
[ "$left" -gt 0 ] && pgrep -af "task_main_controller|skill_server|world_model|fake_world|vision1/node.py|move_group|ros2_control_node|live_monitor" | cut -c1-120
exit 0
