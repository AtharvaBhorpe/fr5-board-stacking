#!/bin/bash
# Attach this terminal to the running trial session (works from kitty too).   tools/trial_attach.sh
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"      # tmux (TM-34)
exec $HK_TMUX_CMD attach -t "${HK_TMUX_SESSION:-trial}"
