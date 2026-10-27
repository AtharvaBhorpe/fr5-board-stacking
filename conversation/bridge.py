"""RViz/ROS bridge. The model explains images; only bounded intents can request tasks."""
import base64
import copy
import json
import queue
import threading
import time
import urllib.request
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from policy import block_reason, intent, visual_agreement, boundary_reply


class Conversation(Node):
    def __init__(self):
        super().__init__("robot_conversation")
        self.allow_replay = self.declare_parameter("allow_replay", False).value
        self.scene, self.state, self.images = {}, {}, {}
        self.history = []
        self.lock = threading.RLock()
        self.epoch = 0
        self.audio_queue = queue.Queue()
        threading.Thread(target=self.audio_loop, daemon=True).start()
        self.busy = False
        self.pending = None
        self.last_phase = None
        self.out = self.create_publisher(String, "/conversation/output", 20)
        self.tasks = self.create_publisher(String, "/conversation/task", 10)
        self.requests = self.create_publisher(String, "/pick_panel/request", 10)
        self.create_subscription(String, "/vision/boards", self.boards, 1)
        self.create_subscription(String, "/pick_panel/state", self.panel_state, 10)
        self.create_subscription(String, "/conversation/input", lambda m: self.submit(m.data), 10)
        self.create_subscription(String, "/conversation/control", lambda m: self.control(m.data), 10)
        for topic, key in [("/vision/image", "overhead"), ("/wrist/wrist_camera/color/image_raw", "wrist")]:
            self.create_subscription(Image, topic, lambda m, k=key: self.picture(m, k), qos_profile_sensor_data)
        self.create_timer(0.5, self.watchdog)
        self.emit("system", "Local conversation bridge ready. Start the model server before chatting.", "ready")

    def emit(self, role, text, phase="ready"):
        self.out.publish(String(data=json.dumps({"role": role, "text": text, "phase": phase})))

    @staticmethod
    def http(path, body, timeout=20):
        req = urllib.request.Request("http://127.0.0.1:8766" + path,
                                     json.dumps(body).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)

    def say(self, text):
        self.emit("assistant", text)
        def speak():
            try:
                self.http("/speak", {"text": text}, timeout=2)
            except Exception:
                pass  # text remains available if audio output fails
        threading.Thread(target=speak, daemon=True).start()

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

    def panel_state(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict):
                return
            with self.lock:
                self.state = state
                if not self.pending or state.get("task_id") != self.pending[0]:
                    return
                phase = state.get("phase")
                if phase == self.last_phase:
                    return
                self.last_phase = phase
                if phase in {"completed", "failed", "rejected", "stopped"}:
                    self.pending = None
            phrases = {"running": "The controller is executing the pick sequence.",
                       "waiting": "The controller is waiting for your Confirm button.",
                       "completed": "The controller reports the pick and place completed.",
                       "failed": "The controller reports failure. Review the log; I will not retry.",
                       "rejected": "The controller rejected the request: " + state.get("detail", ""),
                       "stopped": "The pick script stopped. Check the controller log for action cancellation confirmation; this is not an emergency stop."}
            if phase in phrases:
                self.say(phrases[phase])
        except (ValueError, TypeError):
            pass

    def watchdog(self):
        with self.lock:
            if self.pending and time.time() - self.pending[1] > 5 and self.last_phase is None:
                self.pending = None
                self.say("No controller acknowledgement. I will not retry; check the panel before another request.")

    def control(self, command):
        if command == "clear":
            with self.lock:
                self.epoch += 1
                self.history.clear()
            self.emit("system", "Chat cleared. An active robot task is unchanged.", "clear")
            return
        if command not in {"ptt_start", "ptt_stop"}:
            return
        with self.lock:
            if command == "ptt_start":
                self.epoch += 1
            epoch = self.epoch
        self.audio_queue.put((command, epoch))

    def audio_loop(self):
        while True:
            command, epoch = self.audio_queue.get()
            try:
                if command == "ptt_start":
                    self.http("/listen/start", {}, timeout=3)
                    self.emit("system", "Listening (maximum 20 seconds).", "listening")
                else:
                    self.emit("system", "Transcribing locally...", "transcribing")
                    result = self.http("/listen/stop", {}, timeout=15)
                    with self.lock:
                        if epoch != self.epoch:
                            continue
                    if result.get("text"):
                        self.submit(result["text"])
                    else:
                        self.say("I did not hear a command. Please try again or type it.")
            except Exception as exc:
                self.emit("system", "Microphone/transcription unavailable: " + str(exc)[:180], "error")

    def submit(self, text):
        text = text.strip()[:1000]
        if not text:
            return
        parsed = intent(text)
        self.emit("user", text)
        if parsed.kind == "clear":
            self.control("clear")
            return
        if parsed.kind == "stop":
            with self.lock:
                self.epoch += 1
            self.requests.publish(String(data="stop"))
            self.say("Cancellation requested. This is not an emergency stop; check the controller confirms the stop.")
            return
        boundary = boundary_reply(text) if parsed.kind == "chat" else None
        if boundary:
            self.say(boundary)
            return
        with self.lock:
            if parsed.kind == "status":
                if time.time() - self.state.get("stamp", 0) > 2:
                    self.say("Controller status is unavailable or stale.")
                else:
                    self.say(f"{self.state.get('hardware', 'Unknown hardware')}. Controller: "
                             f"{self.state.get('phase', 'unknown')}. "
                             f"{'Manual confirmation needed.' if self.state.get('waiting') else ''}")
                return
            if self.busy:
                self.say("I am still processing the previous question. Stop remains available.")
                return
            if parsed.kind == "pick":
                reason = block_reason(self.scene, self.state, time.time(), parsed.variant, self.allow_replay)
                if self.pending:
                    reason = "A task request is already pending. I will not queue another."
                if reason:
                    self.say(reason)
                    return
            self.busy = True
            epoch = self.epoch
        threading.Thread(target=self.respond, args=(text, parsed, epoch), daemon=True).start()

    def respond(self, text, parsed, epoch):
        try:
            with self.lock:
                scene, state = copy.deepcopy(self.scene), copy.deepcopy(self.state)
                selected = [self.images[k] for k in ("overhead", "wrist")
                            if k in self.images and 0 <= time.time() - self.images[k][0] <= 2
                            and (k == "overhead" or any(w in text.lower() for w in ("wrist", "cup", "depth")))]
                history = list(self.history)
            if parsed.kind == "pick" and not selected:
                self.say("No fresh overhead image is available to verify the pick.")
                return
            self.emit("system", "Checking the scene locally..." if parsed.kind == "pick" else "Thinking locally...", "thinking")
            result = self.http("/ask", {"text": text, "variant": parsed.variant or None,
                               "scene": scene, "state": state, "images": [x[1] for x in selected],
                               "history": history})
            with self.lock:
                if epoch != self.epoch:
                    return
                answer = str(result.get("answer", "I could not answer that."))[:1200]
                if parsed.kind == "pick":
                    reason = block_reason(self.scene, self.state, time.time(), parsed.variant, self.allow_replay)
                    if not visual_agreement(result):
                        reason = "The image check could not confirm the target. No pick requested; review the camera view."
                    # Do not reuse visual approval if the requested board's measured pose changed during inference.
                    before = [(b.get("x"), b.get("y")) for b in scene.get("boards", []) if b.get("variant") == parsed.variant]
                    after = [(b.get("x"), b.get("y")) for b in self.scene.get("boards", []) if b.get("variant") == parsed.variant]
                    if len(before) != len(after) or any(
                        x is None or y is None or abs(x-y) > .005
                        for old, new in zip(before, after) for x, y in zip(old, new)):
                        reason = "The target moved during the image check. Please request the pick again."
                    if reason:
                        answer = reason
                    else:
                        task_id = str(uuid.uuid4())
                        self.pending = (task_id, time.time())
                        self.last_phase = None
                        self.tasks.publish(String(data=json.dumps({"id": task_id, "variant": parsed.variant,
                                                                   "stamp": time.time()})))
                        answer = f"Requested one {parsed.variant}. Confirm each descent in the panel."
                self.history.extend([{"role": "user", "content": text}, {"role": "assistant", "content": answer}])
                self.history = self.history[-8:]
            self.say(answer)
        except Exception as exc:
            self.emit("system", "Local model unavailable; no task dispatched. " + str(exc)[:180], "error")
        finally:
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = Conversation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
