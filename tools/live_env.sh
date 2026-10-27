# Live-run session for every terminal of a real run (TM-27). SOURCE it, do not run it:
#
#   source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh new [debug|demo]   # 1st terminal: new session
#   source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh                    # every other terminal: join it
#
# Session folder ~/hackathon/live_logs/session_<date>_<time of the 'new'>/ (symlink ~/hackathon/live_logs/current):
#   ros/      every node's own log file (ROS_LOG_DIR): motion, move_group, gripper, Vision 1, world_model, skill_server,
#             task_main_controller, launch logs
#   monitor/  tools/live_monitor.py files (main / vision_raw / world_raw / joints .jsonl, rosout.log = all nodes)
#   mode      debug | demo
# debug (default): HK_VERBOSE=1 -> our nodes log every step (trial runs). demo: HK_VERBOSE=0 -> normal logging.
# HK_NODE_ARGS: '--disable-stdout-logs' in debug (node terminals stay quiet, everything still goes to the files and
# /rosout; the monitor console shows every WARN / ERROR); empty in demo.

_hk_root="${HK_LIVE_ROOT:-$HOME/hackathon/live_logs}"   # HK_LIVE_ROOT: tests only
mkdir -p "$_hk_root"
_hk_new=0; _hk_mode=""
for _a in "$@"; do
  case "$_a" in
    new) _hk_new=1 ;;
    debug|demo) _hk_mode="$_a" ;;
    *) echo "live_env: unknown argument '$_a' (new | debug | demo)"; return 1 2>/dev/null || exit 1 ;;
  esac
done
if [ "$_hk_new" = 1 ] || [ ! -e "$_hk_root/current" ]; then
  _hk_dir="$_hk_root/session_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$_hk_dir/ros" "$_hk_dir/monitor"
  echo "${_hk_mode:-debug}" > "$_hk_dir/mode"
  ln -sfn "$_hk_dir" "$_hk_root/current"
  echo "live_env: NEW session $_hk_dir (${_hk_mode:-debug})"
else
  _hk_dir="$(readlink -f "$_hk_root/current")"
  [ -n "$_hk_mode" ] && echo "$_hk_mode" > "$_hk_dir/mode"
  _hk_age=$(( ( $(date +%s) - $(stat -c %Y "$_hk_dir/mode") ) / 60 ))
  echo "live_env: joined session $_hk_dir ($(cat "$_hk_dir/mode"), started ${_hk_age} min ago)"
  [ "$_hk_age" -gt 240 ] && echo "live_env: WARNING - that session is old: 'source ... live_env.sh new' for a new run"
fi
export HK_LIVE_DIR="$_hk_dir"
export ROS_LOG_DIR="$_hk_dir/ros"
if [ "$(cat "$_hk_dir/mode")" = demo ]; then
  export HK_VERBOSE=0 HK_NODE_ARGS=""
else
  export HK_VERBOSE=1 HK_NODE_ARGS="--disable-stdout-logs"
fi
echo "live_env: ROS_LOG_DIR=$ROS_LOG_DIR HK_VERBOSE=$HK_VERBOSE HK_NODE_ARGS='$HK_NODE_ARGS'"
unset _hk_root _hk_new _hk_mode _hk_dir _hk_age _a
