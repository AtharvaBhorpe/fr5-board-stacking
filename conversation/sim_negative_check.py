"""Live ROS checks that conversation does not dispatch unsupported or negated requests."""
import json
import argparse
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=Path(__file__).parent/'results/negative_ros.json')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node("conversation_negative_check")
state, output, task_messages = {}, [], []
def update(m):
    state.clear(); state.update(json.loads(m.data))
node.create_subscription(String, "/pick_panel/state", update, 10)
node.create_subscription(String, "/conversation/output", lambda m: output.append(json.loads(m.data)), 20)
node.create_subscription(String, "/conversation/task", lambda m: task_messages.append(m.data), 10)
pub = node.create_publisher(String, "/conversation/input", 10)
deadline = time.time()+10
while not state and time.time()<deadline: rclpy.spin_once(node, timeout_sec=.1)
assert state.get("hardware") == "SIMULATION" and not state.get("busy"), state
discovery_deadline = time.time() + 5
while (pub).get_subscription_count() == 0 and time.time() < discovery_deadline:
    rclpy.spin_once(node, timeout_sec=.1)
settle = time.time() + .5
while time.time() < settle:
    rclpy.spin_once(node, timeout_sec=.05)
cases = ["status", "do not pick VARb", "pick eleven VARa boards", "pick that white board",
         "move down 10 mm", "confirm descent", "release vacuum", "pick variant D",
         "why did you pick VARa?", "stop"]
results = []
for text in cases:
    output.clear()
    pub.publish(String(data=text))
    began = time.time()
    deadline = began + 25
    answer = None
    while time.time()<deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        assert not task_messages, (text, task_messages)
        assert not state.get("busy"), state
        answers = [m for m in output if m.get("role") == "assistant" or m.get("phase") == "error"]
        if answers:
            answer=answers[-1]; break
    assert answer is not None, (text, output)
    results.append({"input": text, "response": answer, "seconds": time.time()-began, "dispatches": 0})
path=args.output
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps({"passed": len(results), "dispatches": len(task_messages)}, indent=2))
node.destroy_node(); rclpy.shutdown()
