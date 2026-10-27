"""Verify action cancellation and duplicate suppression on isolated MOCK hardware only."""
import json
import time
import uuid
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("conversation_cancel_check")
state, logs = {}, []
def update(m):
    state.clear(); state.update(json.loads(m.data))
node.create_subscription(String, "/pick_panel/state", update, 10)
node.create_subscription(String, "/pick_panel/log", lambda m: logs.append(m.data), 50)
tasks = node.create_publisher(String, "/conversation/task", 10)
commands = node.create_publisher(String, "/conversation/input", 10)
deadline = time.time()+10
while not state and time.time()<deadline: rclpy.spin_once(node, timeout_sec=.1)
assert state.get("hardware") == "SIMULATION" and not state.get("busy"), state
discovery_deadline = time.time() + 5
while (tasks).get_subscription_count() == 0 and time.time() < discovery_deadline:
    rclpy.spin_once(node, timeout_sec=.1)
settle = time.time() + .5
while time.time() < settle:
    rclpy.spin_once(node, timeout_sec=.05)
task = {"id": str(uuid.uuid4()), "variant": "VARa", "stamp": time.time()}
tasks.publish(String(data=json.dumps(task)))
started = time.time()
sent_stop = False
while time.time()-started < 30:
    rclpy.spin_once(node, timeout_sec=.05)
    assert state.get("hardware") == "SIMULATION", state
    # Once the process is in its action path, send conversational Stop without waiting for the model.
    if not sent_stop and sum("action: accepted" in line for line in logs) >= 2:
        commands.publish(String(data="stop")); sent_stop=True
    if sent_stop and state.get("phase") in {"stopped", "failed"} and not state.get("busy"):
        break
assert sent_stop, logs
assert state.get("phase") == "stopped", state
assert any("cancel:" in line for line in logs), logs
# Replaying a previously accepted task ID must not start another process.
task["stamp"] = time.time()
tasks.publish(String(data=json.dumps(task)))
deadline = time.time()+2
while time.time()<deadline:
    rclpy.spin_once(node, timeout_sec=.1)
    assert not state.get("busy"), "duplicate task started"
print(json.dumps({"passed": True, "state": state, "cancel_log": [x for x in logs if "cancel" in x],
                  "duplicate_suppressed": True}, indent=2))
node.destroy_node(); rclpy.shutdown()
