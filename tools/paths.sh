# Shared paths for the tools (TM-34). SOURCED by trial_up / trial_down / trial_attach / live_env, never run.
# Everything is derived from THIS worktree's location; every value can be overridden by setting it before.
# Expected layout (README): <wt>/feat-task-main-controller (this), <wt>/feat-robot-motion-gripper,
# <wt>/feat-vision1-overhead, logs in <wt>/../live_logs.
HK_WT="${HK_WT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
HK_WT_PARENT="$(dirname "$HK_WT")"
HK_MOTION_WT="${HK_MOTION_WT:-$HK_WT_PARENT/feat-robot-motion-gripper}"
HK_VISION1_WT="${HK_VISION1_WT:-$HK_WT_PARENT/feat-vision1-overhead}"
HK_LIVE_ROOT="${HK_LIVE_ROOT:-$(dirname "$HK_WT_PARENT")/live_logs}"
HK_PIXI="${HK_PIXI:-$(command -v pixi || echo "$HOME/.pixi/bin/pixi")}"
_hk_tmux="${HK_TMUX:-$(command -v tmux || echo "$HOME/.pixi/bin/tmux")}"
# kitty sets TERM / TERMINFO that a pixi-installed tmux cannot read: run it with a standard TERM + the system terminfo
HK_TMUX_CMD="env -u TERMINFO TERM=xterm-256color $_hk_tmux"
export TERMINFO_DIRS="${TERMINFO_DIRS:+$TERMINFO_DIRS:}/usr/share/terminfo:/lib/terminfo"
unset _hk_tmux
