"""Bringup with scene_manager (T20d): static boxes identical to the obstacle_loader reference, /scene/* up,
no obstacle_loader; use_obstacles:=false -> no static boxes. Also prints the Ctrl+C result (see ctrl_c_probe.py, T26)."""
import os, signal, subprocess, time, rclpy
from moveit_msgs.srv import GetPlanningScene
from hk_fr5_interfaces.srv import AddBox
from scene_lib import *  # noqa: F401,F403 (also sets up testenv)
REF = {  # T20b reference (obstacle_loader result)
 'camera_post': ((0.1, 0.07, 1.7), 'base_link', (-0.87, 0.116, 0.806), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
 'floor_guard': ((3.0, 3.0, 0.02), 'base_link', (0.0, 0.0, -0.056), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
 'profile': ((1.0, 0.16, 0.043), 'base_link', (-0.42, 0.0, -0.0225), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
 'table': ((1.0, 2.0, 0.03), 'base_link', (-0.42, 0.03, -0.06), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))}
rclpy.init(); n = rclpy.create_node("t20d"); gps = n.create_client(GetPlanningScene, "/get_planning_scene")
add = n.create_client(AddBox, "/scene/add_box")
def nodes(): return subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=20).stdout.split()
for args, expect_static in (((), True), (("use_obstacles:=false",), False)):
    L = launch(*args, log="t20d_launch.log"); time.sleep(16)
    nl = nodes(); w = world(scene(n, gps))
    print(f"--- bringup {args or 'default'}: nodes with scene/obstacle: {[x for x in nl if 'scene' in x or 'obstacle' in x]}")
    print("  world:", sorted(w))
    ok_static = (w == REF) if expect_static else (w == {})
    r = call(n, add, AddBox.Request(name="probe2", size=[0.1, 0.1, 0.1], position=[-0.4, 0.4, 0.2]))
    print(f"  {'PASS' if ok_static else 'FAIL'} static boxes {'identical to obstacle_loader reference' if expect_static else 'absent'}")
    print(f"  {'PASS' if r and r.success else 'FAIL'} /scene/add_box works: {r.message if r else None}")
    print(f"  {'PASS' if '/scene/scene_manager' in nl and not any('obstacle_loader' in x for x in nl) else 'FAIL'} scene_manager running, no obstacle_loader")
    stop(L, wait=12); log = open("t20d_launch.log").read()
    clean = [l for l in log.splitlines() if "finished cleanly" in l or "process has died" in l or "exited with" in l]
    print(f"  INFO (T26, known: 2nd SIGINT from ros2 launch) Ctrl+C: "
          f"{sum('cleanly' in l for l in clean)} processes finished cleanly, others: {[l for l in clean if 'cleanly' not in l]}")
    time.sleep(3)
n.destroy_node(); rclpy.shutdown()
