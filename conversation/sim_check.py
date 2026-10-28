"""Exercise the real RViz panel and MoveIt mock stack; refuses non-mock hardware.

Run on a separate ROS_DOMAIN_ID with replay vision, MoveIt demo and RViz open.
--chat exercises the model bridge too; default isolates the panel/controller contract.
"""
import argparse
import json
import time
import uuid

import rclpy
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument("--variant", choices=["VARa", "VARb", "VARc"], required=True)
ap.add_argument("--chat", action="store_true")
args = ap.parse_args()
rclpy.init()
node = rclpy.create_node("conversation_sim_check")
state, output = {}, []


def receive(message):
    state.clear()
    state.update(json.loads(message.data))


node.create_subscription(String, "/pick_panel/state", receive, 10)
node.create_subscription(String, "/conversation/output", lambda m: output.append(json.loads(m.data)), 20)
tasks = node.create_publisher(String, "/conversation/task", 10)
chat = node.create_publisher(String, "/conversation/input", 10)
requests = node.create_publisher(String, "/pick_panel/request", 10)
deadline = time.time() + 10
while not state and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
if state.get("hardware") != "SIMULATION" or state.get("busy"):
    raise SystemExit("REFUSED: expected idle SIMULATION panel")
discovery_deadline = time.time() + 5
while (chat if args.chat else tasks).get_subscription_count() == 0 and time.time() < discovery_deadline:
    rclpy.spin_once(node, timeout_sec=.1)
settle = time.time() + .5
while time.time() < settle:
    rclpy.spin_once(node, timeout_sec=.05)
task_id = str(uuid.uuid4())
if args.chat:
    chat.publish(String(data="pick " + args.variant))
else:
    tasks.publish(String(data=json.dumps({"id": task_id, "variant": args.variant, "stamp": time.time()})))
began = time.time()
deadline = began + 180
last_confirm, phases = 0, []
started = False
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
    if state.get("hardware") != "SIMULATION":
        raise SystemExit("REFUSED: hardware identity changed")
    if state.get("busy"):
        started = True
    if state.get("phase") not in phases:
        phases.append(state.get("phase"))
    if started and state.get("waiting") and time.time() - last_confirm > 2:
        requests.publish(String(data="confirm"))
        last_confirm = time.time()
    if started and state.get("phase") in {"completed", "failed", "stopped"}:
        break
    if args.chat and not started and any(
        m.get("phase") == "error" or
        (m.get("role") == "assistant" and not m.get("text", "").startswith(("Requested one ", "Picking ")))
        for m in output
    ):
        break
    if not args.chat and state.get("task_id") == task_id and state.get("phase") == "rejected":
        break
passed = started and state.get("phase") == "completed"
print(json.dumps({"variant": args.variant, "chat": args.chat, "passed": passed,
                  "seconds": round(time.time()-began, 2), "phases": phases,
                  "final": state, "conversation": output}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
