"""Ctrl+C probe (T26, informative): SIGINT to the bringup's process group (like a terminal) and count nodes that
'finished cleanly' vs 'died'. Known pre-existing result: several nodes die (-2) from the 2nd SIGINT that ros2 launch sends."""
import time, sys
from scene_lib import launch, stop
L = launch("use_obstacles:=false", log="stop_launch.log"); time.sleep(16); stop(L, wait=12)
log = open("stop_launch.log").read().splitlines()
print("cleanly:", sum("finished cleanly" in l for l in log), "| died:", [l.split("]")[1].strip(" [") for l in log if "process has died" in l])
