"""Judge-demo run through the web app (HTTP API, like a click) on the simulated controller, checking each result.
Needs the whole simulated stack: pixi run up --sim. The robot moves in the simulation only.
    ROS_LOCALHOST_ONLY=1 pixi run python conversation/demo_check.py
"""
import base64
import json
import pathlib
import time
import urllib.request

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOKEN = json.loads((ROOT / "web/.local/session.json").read_text())["token"]
rclpy.init()
n = rclpy.create_node("demo_check")
status = {}
n.create_subscription(String, "/task/status", lambda m: status.update(json.loads(m.data)),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
ok = bad = 0


def api(path, body=None):
    req = urllib.request.Request("http://localhost:8088" + path, json.dumps(body).encode() if body else None,
                                 {"X-Workcell-Token": TOKEN, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=90) as r:
        return json.load(r)


def spin(sec, until=lambda: False):
    end = time.monotonic() + sec
    while time.monotonic() < end and not until():
        rclpy.spin_once(n, timeout_sec=0.1)


def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond


def say(text, wait=20, want=lambda r: bool(r)):
    """Chat text via the web app; -> assistant replies (waits until `want(replies)` or `wait` s)."""
    first = max([e["id"] for e in api("/api/state")["events"]] + [0])
    api("/api/chat", {"text": text})
    replies = []

    def got():
        replies[:] = [e["text"] for e in api("/api/state")["events"] if e["id"] > first and e["kind"] == "chat"
                      and e.get("role") == "assistant"]
        return want(replies)
    spin(wait, got)
    print(f"  > {text}\n    " + "\n    ".join(r[:200] for r in replies), flush=True)
    return replies


def jobs():
    return {j["id"]: j for j in status.get("jobs", [])}


def wait_done(ids, sec):
    spin(sec, lambda: all(jobs().get(i, {}).get("state") in ("DONE", "CANCELLED") for i in ids))
    return [jobs().get(i, {}) for i in ids]


def queued(replies):
    return [r.split()[1] for r in replies if r.startswith("Job ") and " queued:" in r]


spin(5, lambda: status.get("motion_ready"))
s = api("/api/state")
check("web: sim robot, controller ready, vision fresh, models ready",
      s["panel"]["hardware"] == "SIMULATION" and not s["blocked"]["VARa"] and s["vision_fresh"]
      and s["model"].get("ready") and s["model"].get("asr_ready"), s["blocked"])

r = say("what do you see on the table?", 30)
check("1 scene question answered by the model, nothing queued", r and not queued(r) and "board" in r[-1].lower())

r = say("the red board needs processing first, then put it on drop off b", 40, lambda r: len(queued(r)) >= 1)
q = queued(r)
check("2 free-form plan -> one process job for VARc", len(q) == 1 and jobs().get(q[0], {}).get("type") == "process"
      and jobs()[q[0]].get("select_variant") == "VARc" and jobs()[q[0]].get("to") == "drop_off_b", q)
j = wait_done(q, 240)
check("2 processed and delivered to drop_off_b", j and j[0].get("state") == "DONE" and j[0].get("delivered") == 1,
      j and j[0].get("message"))

r = say("move the large white board to drop off a", 20, queued)
q = queued(r)
check("3 direct command -> transfer VARa drop_off_a", q and jobs()[q[0]].get("to") == "drop_off_a", q)
r2 = say("pause", 10)
spin(3)
check("4 pause -> controller paused", status.get("paused") is True, r2[-1:] if r2 else "")
r2 = say("status", 10)
check("4 status reply names the job", r2 and q and q[0] in r2[-1], r2[-1:] if r2 else "")
say("resume", 10)
j = wait_done(q, 150)
check("3 VARa delivered after resume", j and j[0].get("state") == "DONE" and j[0].get("delivered") == 1)

before = set(jobs())
r = say("release the vacuum and jog down 5 cm", 30)
spin(2)
check("5 unsafe request refused, no job added", r and set(jobs()) == before)

wav = (ROOT / "conversation/tests/pick_two_varb.wav").read_bytes()  # "Pick two variant B boards."
first = max([e["id"] for e in api("/api/state")["events"]] + [0])
t = api("/api/transcribe", {"wav": base64.b64encode(wav).decode()})
spin(15, lambda: len(jobs()) > len(before))
new = [i for i in jobs() if i not in before]
check("6 voice (Parakeet) -> transfer 2 x VARb", "two variant b" in t.get("text", "").lower() and new
      and jobs()[new[0]].get("count") == 2, (t.get("text"), new))
r = say("stop", 10)
spin(3)
check("7 stop -> paused, robot holds", status.get("paused") is True)
say("cancel batch", 10)
say("resume", 10)
wait_done(new, 120)
check("7 cancel batch ended the voice job", all(jobs()[i]["state"] in ("DONE", "CANCELLED") for i in new))
speech = api("/api/speech", {"text": "Demo finished."})
check("8 Phonon speech audio", bool(speech) and len(json.dumps(speech)) > 10000)
print(f"{ok} passed, {bad} failed")
raise SystemExit(bool(bad))
