"""Measure the lag between commanded and actual joint motion (real robot debugging, decision-log 5l).

    pixi run python3 notes/lag_probe.py 15      # record 15 s; start a small move during that time

Reads /arm_controller/controller_state: `reference` = what ros2_control commands (sent to the FR5 via
ServoJ), `feedback` = what the driver reads back from the FR5. Changes nothing.
"""
import sys
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
rclpy.init()
node = rclpy.create_node("lag_probe")
rec = []  # (t, reference positions, feedback positions)
node.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state",
                         lambda m: rec.append((time.monotonic(), list(m.reference.positions),
                                               list(m.feedback.positions))), 200)
print(f"recording {seconds:.0f} s ... start a SMALL slow move now (e.g. shell: `v 0.1` then `d 0 0 20 base`)")
end = time.monotonic() + seconds
try:
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.try_shutdown()

if len(rec) < 10:
    sys.exit(f"only {len(rec)} controller_state messages: is the bringup running?")
t0 = rec[0][0]
rate = (len(rec) - 1) / (rec[-1][0] - t0)


def moving_window(index, eps=1e-3):
    """(first time it left the start value, last time it changed) for series index 1 (ref) / 2 (fb)."""
    start = rec[0][index]
    first = next((t for t, *s in rec if max(abs(a - b) for a, b in zip(s[index - 1], start)) > eps), None)
    last_val = rec[-1][index]
    last = next((t for t, *s in reversed(rec) if max(abs(a - b) for a, b in zip(s[index - 1], last_val)) > eps), None)
    return first, last


rs, re_ = moving_window(1)
fs, fe = moving_window(2)
err = max(max(abs(a - b) for a, b in zip(r, f)) for _, r, f in rec if r and f)
print(f"messages: {len(rec)} at {rate:.0f} Hz")
if rs is None or fs is None:
    sys.exit("no motion detected in commanded and/or actual positions")
print(f"commanded motion: starts {rs - t0:6.2f} s, ends {re_ - t0:6.2f} s  (duration {re_ - rs:.2f} s)")
print(f"actual    motion: starts {fs - t0:6.2f} s, ends {fe - t0:6.2f} s  (duration {fe - fs:.2f} s)")
print(f"LAG actual behind commanded: at start {fs - rs:+.3f} s, at end {fe - re_:+.3f} s")
print(f"max |commanded - actual| during the recording: {err:.4f} rad ({err * 57.2958:.2f} deg)")
