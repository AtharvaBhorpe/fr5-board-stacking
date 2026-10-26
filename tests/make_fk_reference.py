"""One-off: FK reference from OUR URDF via robot_state_publisher TF (what MoveIt / get_state use) -> fixture JSON.
Runs in the MOTION branch environment (needs hk_fr5_description):

    cd ~/hackathon/wt/feat-robot-motion-gripper
    ROS_DOMAIN_ID=79 pixi run python3 ~/hackathon/wt/feat-vision-transform/tests/make_fk_reference.py

Writes tests/fixtures/fk_reference.json next to this file (50 random joint sets in the motion joint limits)."""
import json
import math
import os
import random
import subprocess
import time

import rclpy
import tf2_ros
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "fixtures", "fk_reference.json")
urdf = xacro.process_file(os.path.join(get_package_share_directory("hk_fr5_description"), "urdf", "hk_fr5.urdf.xacro")).toxml()
params = os.path.join(HERE, "fixtures", ".rsp_params.yaml")   # temp, removed below (URDF too long / not CLI-safe)
os.makedirs(os.path.dirname(params), exist_ok=True)
with open(params, "w") as f:
    yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": urdf}}}, f)
rsp = subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                        "--params-file", params], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       start_new_session=True)
rclpy.init(); n = rclpy.create_node("fk_reference")
pub = n.create_publisher(JointState, "/joint_states", 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
LIM = [(-175, 175), (-165, 10), (10, 160), (-265, 85), (-170, -10), (-175, 175)]   # deg, motion joint_limits.yaml (H-11)
random.seed(4)
sets = [[0.0, -90.0, 90.0, -90.0, -90.0, 0.0]] + [[random.uniform(a, b) for a, b in LIM] for _ in range(49)]
out = []
try:
    prev = None
    for q in sets:
        js = JointState(name=[f"j{i}" for i in range(1, 7)], position=[math.radians(a) for a in q])
        end = time.time() + 0.4   # publish this joint set for 0.4 s (robot_state_publisher reacts in ms), then read TF
        while time.time() < end or not buf.can_transform("base_link", "tcp", rclpy.time.Time()):
            js.header.stamp = n.get_clock().now().to_msg(); pub.publish(js); rclpy.spin_once(n, timeout_sec=0.02)
            if time.time() > end + 10:
                raise RuntimeError("no TF from robot_state_publisher")
        t = buf.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform
        f = buf.lookup_transform("base_link", "flange", rclpy.time.Time()).transform.translation
        tcp = [t.translation.x, t.translation.y, t.translation.z]
        if prev is not None and max(abs(a - b) for a, b in zip(tcp, prev)) < 1e-6:
            raise RuntimeError(f"TF did not change for joint set {q}")
        prev = tcp
        out.append({"joints_deg": q, "tcp": tcp,
                    "tcp_quat_xyzw": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], "flange": [f.x, f.y, f.z]})
finally:
    os.killpg(rsp.pid, 2); rsp.wait(timeout=5); os.remove(params)
    n.destroy_node(); rclpy.shutdown()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump({"source": "robot_state_publisher TF, hk_fr5_description (branch feat/robot-motion-gripper)", "poses": out},
          open(OUT, "w"), indent=1)
print(f"wrote {OUT}: {len(out)} poses")
