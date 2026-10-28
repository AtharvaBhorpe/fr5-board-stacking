"""Web chat bridge. The model answers questions; fixed command patterns become main-controller jobs.

Robot actions go only through the main controller (feat/task-main-controller) via its agent bridge (task_link.py):
jobs (transfer / process / clear_zone / board) and control (pause / resume / stop / home / cancel / clear_queue).
Status and job progress: /task/status. Scene: /vision/boards (boards + zones, Vision 1).
"""
import base64
import copy
import json
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from policy import boundary_reply, controller_block, describe_status, intent
import planner
import task_link

FINAL = ("DONE", "CANCELLED")
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Conversation(Node):
    def __init__(self):
        super().__init__("robot_conversation")
        self.scene, self.images = {}, {}
        self.status, self.status_rx = {}, float("nan")
        self.mine = {}      # job id -> last (state, delivered, to_fault, lost, skipped) of jobs sent from this chat
        self.alerts = set()
        self.history = []
        self.lock = threading.RLock()
        self.epoch = 0
        self.busy = False
        self.out = self.create_publisher(String, "/conversation/output", 20)
        self.batch_out = self.create_publisher(String, "/conversation/batch_state", 10)
        self.create_subscription(String, "/vision/boards", self.boards, 1)
        self.create_subscription(String, "/task/status", self.task_status, LATCHED)
        self.create_subscription(String, "/conversation/input", lambda m: self.submit(m.data), 10)
        self.create_subscription(String, "/conversation/control", lambda m: self.control(m.data), 10)
        for topic, key in [("/vision/image", "overhead"), ("/wrist/wrist_camera/color/image_raw", "wrist")]:
            self.create_subscription(Image, topic, lambda m, k=key: self.picture(m, k), qos_profile_sensor_data)
        self.create_timer(0.5, self.watchdog)
        self.emit("system", "Conversation ready. Robot jobs go to the main controller.", "ready")

    def emit(self, role, text, phase="ready"):
        self.out.publish(String(data=json.dumps({"role": role, "text": text, "phase": phase})))

    @staticmethod
    def http(path, body, timeout=20):
        req = urllib.request.Request("http://127.0.0.1:8766" + path,
                                     json.dumps(body).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)

    def say(self, text):  # the web app reads assistant lines aloud (model server /synthesize)
        self.emit("assistant", text)

    def boards(self, message):
        try:
            scene = json.loads(message.data)
            if not isinstance(scene, dict):
                return
            with self.lock:
                self.scene = scene
        except ValueError:
            with self.lock:
                self.scene = {}

    def picture(self, message, key):
        if message.encoding not in {"bgr8", "rgb8"}:
            return
        try:
            frame = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            frame = frame[:, :message.width * 3].reshape(message.height, message.width, 3)
            if message.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.resize(frame, (640, max(1, round(640 * message.height / message.width))))
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                with self.lock:
                    self.images[key] = (stamp, base64.b64encode(encoded).decode())
        except (ValueError, cv2.error):
            pass

    def task_status(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        said = []
        with self.lock:
            first = not self.status  # alerts already in the first status are old: remember them silently
            self.status, self.status_rx = status, time.time()
            for j in status.get("jobs", []):
                if j.get("id") not in self.mine:
                    continue
                k = (j.get("state"), j.get("delivered"), j.get("to_fault"), j.get("lost"), j.get("skipped"))
                if k != self.mine[j["id"]]:
                    self.mine[j["id"]] = k
                    if k[0] in FINAL or k[0] == "PAUSED" or (k[0] == "WAITING" and j.get("message")):
                        said.append(f"Job {j['id']} {k[0].lower()}: {k[1]} delivered, {k[2]} to the fault zone, "
                                    f"{k[3]} lost, {k[4]} skipped. {j.get('message', '')}".strip())
            for e in status.get("events", []):
                key = json.dumps(e, sort_keys=True)
                if e.get("level") in ("warn", "error") and key not in self.alerts:
                    self.alerts.add(key)
                    if not first:
                        said.append("Controller alert: " + str(e.get("text", ""))[:200])
        for text in said:
            self.say(text)

    def watchdog(self):
        with self.lock:
            jobs = [j for j in self.status.get("jobs", []) if j.get("id") in self.mine]
        if not jobs:
            return
        active = [j for j in jobs if j.get("state") not in FINAL]
        phase = "executing" if active else ("completed" if all(j["state"] == "DONE" for j in jobs) else "stopped")
        snap = {"id": ",".join(j["id"] for j in jobs), "phase": phase,
                "completed": sum(j.get("delivered", 0) for j in jobs),
                "total": sum(max(j.get("count", 0), j.get("delivered", 0)) for j in jobs),
                "remaining": [j["id"] for j in active], "reason": (jobs[-1].get("message") or "")[:120]}
        self.batch_out.publish(String(data=json.dumps(snap)))

    def control(self, command):
        if command == "stop":
            self.submit("stop")
            return
        if command == "clear":
            with self.lock:
                self.epoch += 1
                self.history.clear()
            self.emit("system", "Chat cleared. Robot jobs are unchanged.", "clear")

    def submit(self, text):
        text = text.strip()[:1000]
        if not text:
            return
        with self.lock:
            parsed = intent(text, self.scene.get("zones", []) if isinstance(self.scene, dict) else [])
        self.emit("user", text)
        if parsed.kind == "clear":
            self.control("clear")
            return
        if parsed.command in ("stop", "pause", "clear_queue"):  # always allowed, never gated
            with self.lock:
                self.epoch += 1          # cut ongoing speech / model answers
            threading.Thread(target=self.dispatch, args=(parsed.command,), daemon=True).start()
            return
        if parsed.kind == "status":
            with self.lock:
                self.say(describe_status(self.status, self.status_rx, time.time()))
            return
        if parsed.kind == "ask":
            self.say(parsed.command)
            return
        if parsed.kind == "command":
            with self.lock:
                reason = controller_block(self.status, self.status_rx, time.time())
            if reason:
                self.say(reason + " Nothing sent.")
                return
            threading.Thread(target=self.dispatch, args=(parsed.command,), daemon=True).start()
            return
        if boundary_reply(text):  # motion-like free text: the model plans, the grammar + controller check
            threading.Thread(target=self.plan, args=(text,), daemon=True).start()
            return
        with self.lock:
            if self.busy:
                self.say("I am still answering the previous question. Stop remains available.")
                return
            self.busy = True
            epoch = self.epoch
        threading.Thread(target=self.respond, args=(text, epoch), daemon=True).start()

    def dispatch(self, command):
        """One command to the main controller; say its answer (accepted job id or its reason)."""
        try:
            ok, job, msg = task_link.send(command)
        except (RuntimeError, OSError) as exc:
            self.say(f"Not sent: {exc}")
            return False
        if ok and job:
            with self.lock:
                self.mine[job] = None
        self.say(f"Job {job} queued: {command}." if ok and job else
                 f"Done: {command}. {msg}".strip() if ok else f"The main controller refused '{command}': {msg}")
        return ok

    def plan(self, text):
        """Free-form request -> Gemma plans controller commands from the live state (planner.py); each one must pass
        the fixed grammar before it is sent, and the controller validates the job."""
        with self.lock:
            scene, status = copy.deepcopy(self.scene), copy.deepcopy(self.status)
        zones = [{"name": z.get("name"), "kind": z.get("kind")} for z in scene.get("zones", [])] + \
            [{"name": "fault", "kind": "fault"}]  # controller's fixed fault spot
        boards = [{"variant": b.get("variant"), "on": b.get("zone")} for b in scene.get("boards", [])]
        jobs = [{k: j.get(k) for k in ("id", "state", "type", "select_variant", "to", "delivered")}
                for j in status.get("jobs", []) if j.get("state") not in FINAL]
        self.emit("system", "Planning with the local model...", "thinking")
        try:
            reason, commands, rejected = planner.plan(text, zones, boards, jobs)
        except Exception as exc:  # noqa: BLE001  no model / bad JSON: nothing moves
            self.say(boundary_reply(text) + f" (planner unavailable: {str(exc)[:80]})")
            return
        self.say((reason or "No plan.") + (f" ({len(rejected)} proposed command(s) rejected by the grammar.)"
                                           if rejected else "") + (f" Queueing {len(commands)} jobs." if len(commands) > 1 else ""))
        if not commands:
            return
        reason_block = controller_block(status, self.status_rx, time.time())
        if reason_block:
            self.say(reason_block + " Nothing sent.")
            return
        for c in commands:  # a plan is a sequence: stop at the first refusal
            if not self.dispatch(c):
                self.say("Plan stopped: the rest was not sent.")
                return

    def respond(self, text, epoch):
        """Questions only: the local model sees the scene, the controller status and fresh images."""
        try:
            with self.lock:
                scene, status = copy.deepcopy(self.scene), copy.deepcopy(self.status)
                selected = [self.images[k] for k in ("overhead", "wrist")
                            if k in self.images and 0 <= time.time() - self.images[k][0] <= 2
                            and (k == "overhead" or any(w in text.lower() for w in ("wrist", "cup", "depth")))]
                history = list(self.history)
            self.emit("system", "Thinking locally...", "thinking")
            result = self.http("/ask", {"text": text, "variant": None, "scene": scene, "state": status,
                                        "images": [x[1] for x in selected], "history": history})
            with self.lock:
                if epoch != self.epoch:
                    return
                answer = str(result.get("answer", "I could not answer that."))[:1200]
                self.history.extend([{"role": "user", "content": text}, {"role": "assistant", "content": answer}])
                self.history = self.history[-8:]
            self.say(answer)
        except Exception as exc:
            self.emit("system", "Local model unavailable: " + str(exc)[:180], "error")
        finally:
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = Conversation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with node.lock:
            node.epoch += 1
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
