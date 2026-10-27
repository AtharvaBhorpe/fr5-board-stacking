#!/bin/bash
# Attach this terminal to the running trial session (works from kitty too).   tools/trial_attach.sh
export TERMINFO_DIRS="/usr/share/terminfo:/lib/terminfo"
exec env -u TERMINFO TERM=xterm-256color "$HOME/.pixi/bin/tmux" attach -t trial
