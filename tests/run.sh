#!/bin/bash
# Run the test suites one after another and print one summary line per suite.   pixi run test [all]
#   (no argument)  offline suites: no ROS graph, no robot (under 1 minute)
#   all            also the ROS suites; they start their own headless sim stacks (about 20 minutes)
# Every ROS suite gets its own ROS_DOMAIN_ID on localhost, so a running 'pixi run up' is not disturbed.
# Do not run 'all' next to a running sim stack: the CPU load makes timing checks flaky.
# Node logs: $HK_TEST_LOG_DIR (default /tmp/hk_fr5_tests).
cd "$(dirname "$(readlink -f "$0")")/.."
OFFLINE="launcher world_core fake_world skill_math vision1 main_node demo_ui main_core main_core_regress main_core_fuzz gripper_io"
ROS="inject demo_ui_bridge world_ros main_node_ros gripper_node gripper_actions scene_smoke scene_launch scene_motion
     joint_config skills_ros main_ros"
SUITES=$OFFLINE; [ "${1:-}" = all ] && SUITES="$OFFLINE $ROS"
export ROS_LOCALHOST_ONLY=1
fail=0; domain=82; summary=""
for s in $SUITES; do
  domain=$((domain + 1))
  printf "%-20s " "$s"
  start=$(date +%s)
  out=$(ROS_DOMAIN_ID=$domain python3 "tests/${s}_check.py" 2>&1); code=$?
  last=$(echo "$out" | grep -iE "passed|[0-9]+/[0-9]+" | tail -1 | cut -c1-70)
  if [ $code = 0 ]; then echo "ok    $(( $(date +%s) - start )) s  $last"
  else echo "FAIL  (exit $code) $last"; echo "$out" | grep -E "FAIL" | head -5 | sed 's/^/    /'; fail=1; summary="$summary $s"; fi
done
# Vision 1 on the recorded photos, conversation unit tests, planner / grammar self-checks (no ROS, no Gemma)
for c in "python3 vision1/check_samples.py" "python3 -m unittest discover -s conversation/tests" "python3 conversation/planner.py" "python3 conversation/policy.py"; do
  printf "%-20s " "${c##* }"
  if out=$($c 2>&1); then echo "ok    $(echo "$out" | tail -1 | cut -c1-60)"
  else echo "FAIL"; echo "$out" | tail -5 | sed 's/^/    /'; fail=1; summary="$summary ${c##* }"; fi
done
[ $fail = 0 ] && echo "ALL PASSED" || echo "FAILED:$summary"
exit $fail
