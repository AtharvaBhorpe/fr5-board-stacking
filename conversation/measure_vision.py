"""Measure actual message delivery during concurrent inference; no robot commands."""
import json
import argparse
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=Path(__file__).parent / "results/vision_rate.json")
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node("conversation_vision_rate")
stamps = []
node.create_subscription(String, "/vision/boards", lambda _: stamps.append(time.monotonic()), 10)
deadline = time.monotonic() + 40
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
intervals = [b-a for a,b in zip(stamps, stamps[1:])]
result = {"messages": len(stamps), "hz": (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0,
          "max_interval_s": max(intervals) if intervals else None, "duration_s": 40}
path = args.output
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
