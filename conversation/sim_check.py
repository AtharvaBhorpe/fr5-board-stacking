"""End-to-end: chat text -> bridge -> main controller (simulation) -> jobs. Needs conversation/demo.sh running
(or trial_up.sh --sim + pixi run conversation) on the same ROS_DOMAIN_ID.  pixi run python conversation/sim_check.py
"""
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node("conversation_sim_check")
said, status = [], {}
n.create_subscription(String, "/conversation/output", lambda m: said.append(json.loads(m.data)), 50)
n.create_subscription(String, "/task/status", lambda m: status.update(json.loads(m.data)),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
chat = n.create_publisher(String, "/conversation/input", 10)
ok = bad = 0


def spin(sec, until=lambda: False):
    end = time.monotonic() + sec
    while time.monotonic() < end and not until():
        rclpy.spin_once(n, timeout_sec=0.05)


def ask(text, want, sec=15):
    """Send text; PASS when an assistant reply contains `want`."""
    global ok, bad
    start = len(said)
    chat.publish(String(data=text))
    hit = lambda: any(want.lower() in s["text"].lower() for s in said[start:] if s["role"] == "assistant")  # noqa
    spin(sec, hit)
    replies = [s["text"] for s in said[start:] if s["role"] == "assistant"]
    print(("PASS " if hit() else "FAIL ") + f"{text!r} -> {replies[-1:] or 'no reply'}", flush=True)
    ok += hit(); bad += not hit()


def job_state(i):
    return next((j for j in status.get("jobs", []) if j["id"] == i), {})


spin(20, lambda: chat.get_subscription_count() and status.get("motion_ready"))
ask("status", "controller")
ask("pick variant c", "which zone")                               # fake table: drop_off_a, drop_off_b only
ask("transfer VARz drop_off_a 1", "refused")                      # controller validation, verbatim command
ask("move one variant a to drop off a", "queued")
job = next((j["id"] for j in reversed(status.get("jobs", [])) if j.get("select_variant") == "VARa"), "")
ask("pause", "pause")
spin(3)
paused = status.get("paused")
ask("resume", "resume")
spin(180, lambda: job_state(job).get("state") == "DONE")
spin(3)  # the bridge announces after the same status
s = job_state(job)
for name, cond in [("paused on request", paused), ("job DONE, 1 delivered", s.get("state") == "DONE"
                   and s.get("delivered") == 1), ("chat reported the result", any(
                   f"job {job} done" in x["text"].lower() for x in said))]:
    print(("PASS " if cond else "FAIL ") + name, flush=True)
    ok += bool(cond); bad += not cond
ask("move all variant b to drop off b", "queued")
ask("cancel batch", "clear_queue")
spin(120, lambda: all(j["state"] in ("DONE", "CANCELLED") for j in status.get("jobs", [])))
cond = all(j["state"] in ("DONE", "CANCELLED") for j in status.get("jobs", []))
print(("PASS " if cond else "FAIL ") + "cancel batch ended every job", flush=True)
ok += cond; bad += not cond
ask("stop", "stop")
print(f"{ok} passed, {bad} failed")
raise SystemExit(bool(bad))
