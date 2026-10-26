#!/bin/bash
# Gripper in the sim bringup (step 20d): nodes / actions, grip + release via CLI, use_gripper:=false / bad value,
# SIGTERM to the node switches both pumps off. Output is printed; compare with implementation-ref "Step 20d results".
# Run from the worktree:  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 pixi run bash tests/gripper_launch_check.sh
LOG=${HK_TEST_LOG_DIR:-/tmp/hk_fr5_tests}; mkdir -p "$LOG"; cd "$LOG" || exit 1
setsid ros2 launch hk_fr5_moveit_config bringup.launch.py use_rviz:=false > gripper_launch.log 2>&1 &
L=$!
sleep 16
echo "== nodes"; ros2 node list | grep -E "gripper|motion|scene"
echo "== actions"; ros2 action list | grep -E "gripper|motion"
echo "== grip"; ros2 action send_goal /gripper/grip hk_fr5_interfaces/action/Grip "{}" | grep -E "status|success|message|time_to"
echo "== state"; ros2 topic echo --once /gripper/state hk_fr5_interfaces/msg/GripperState | grep -E "state|vacuum|blow"
echo "== release"; ros2 action send_goal /gripper/release hk_fr5_interfaces/action/Release "{}" | grep -E "status|success|message"
echo "== param use_fake_hardware"; ros2 param get /gripper/gripper_controller use_fake_hardware
kill -INT -$L; sleep 8; kill -9 -$L 2>/dev/null
echo "== after Ctrl+C: shutdown lines"; grep -E "shutdown: (vacuum|blow)" gripper_launch.log | sed -E 's/\[[0-9.]+\] //'
echo "== use_gripper:=bad"
timeout 15 ros2 launch hk_fr5_moveit_config bringup.launch.py use_rviz:=false use_gripper:=bad 2>&1 | grep -E "must be"
echo "== SIGTERM to the node"
ros2 run hk_fr5_gripper gripper_controller --ros-args --params-file "$(ros2 pkg prefix hk_fr5_gripper)/share/hk_fr5_gripper/config/gripper.yaml" > gripper_term.log 2>&1 &
sleep 3; P=$(ps -eo pid,args | grep "lib/hk_fr5_gripper/gripper_controller" | grep -v grep | awk '{print $1}')
kill -TERM $P; sleep 2
grep -E "shutdown:|Traceback" gripper_term.log | sed -E 's/\[[0-9.]+\] //'; ps -p $P > /dev/null && echo "STILL RUNNING" || echo "node exited"
