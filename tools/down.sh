#!/bin/bash
# Stop the tmux session "fr5" cleanly: Ctrl+C into each window in order (web + chat first, then our nodes, vision,
# motion; the monitor last, it writes the log index at exit), wait, then close the session.   pixi run down
T="env -u TERMINFO TERM=xterm-256color tmux -L fr5"
S=fr5
if ! $T has-session -t $S 2>/dev/null; then echo "no session '$S'"; exit 0; fi
for w in web chat speech llm demo-ui bridge ctrl skills world wrist vision fake motion monitor; do
  if $T list-windows -t $S -F '#W' | grep -qx $w; then
    echo "stopping $w"
    $T send-keys -t $S:$w C-c
    sleep $([ $w = motion ] && echo 6 || echo 2)
  fi
done
$T kill-server
sleep 1
P="task_main_controller|skill_server|world_model|fake_world|vision1/node.py|move_group|ros2_control_node|live_monitor|llama-server|model_server.py|gateway.py|bridge.py|realsense2_camera"
left=$(pgrep -f "$P" | wc -l)
echo "session closed; processes still running: $left"
[ "$left" -gt 0 ] && pgrep -af "$P" | cut -c1-120
exit 0
