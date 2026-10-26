import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HOST = "127.0.0.1"
PORT = 8082
STALE_AFTER_S = 10.0
ALLOWED_ORIGINS = {
    "http://127.0.0.1:8080",
    "http://localhost:8080",
}

lock = threading.Lock()
latest = None
received_at = None
last_error = None


def reject_constant(value):
    raise ValueError(f"Non-finite JSON value: {value}")


class SceneBridge(Node):
    def __init__(self):
        super().__init__("fr5_replay_scene_bridge")
        self.subscription = self.create_subscription(
            String, "/fr5/replay/scene_json", self.receive, 1
        )

    def receive(self, message):
        global latest, received_at, last_error
        try:
            scene = json.loads(
                message.data, parse_constant=reject_constant
            )
            if not isinstance(scene, dict):
                raise ValueError("Expected a scene object")
            if scene.get("mode") != "ros_image_replay":
                raise ValueError("Expected replay data")
            if scene.get("historical_image") is not True:
                raise ValueError("Missing historical-image flag")
            if scene.get("execution_allowed") is not False:
                raise ValueError("Replay must disable execution")
            if scene.get("robot_targets") != []:
                raise ValueError("Replay must have no robot targets")
            if scene.get("status") not in ("processed", "error"):
                raise ValueError("Unknown perception status")
            if not all(
                isinstance(scene.get(key), list)
                for key in ("destination_tags", "source_candidates")
            ):
                raise ValueError("Missing detection lists")

            with lock:
                latest = scene
                received_at = time.monotonic()
                last_error = None
        except Exception as error:
            # Invalid input clears the previous observation.
            with lock:
                latest = None
                received_at = time.monotonic()
                last_error = str(error)
            self.get_logger().error(str(error))


def snapshot():
    with lock:
        scene = latest
        error = last_error
        age = (
            None if received_at is None
            else time.monotonic() - received_at
        )

    if age is None:
        state = "waiting"
    elif age > STALE_AFTER_S:
        state = "stale"
    elif error is not None or (
        scene is not None and scene["status"] == "error"
    ):
        state = "error"
    else:
        state = "receiving"

    return {
        "schema_version": 1,
        "stream_state": state,
        "age_since_bridge_receive_s": (
            None if age is None else round(age, 3)
        ),
        "stale_after_s": STALE_AFTER_S,
        "data_kind": "historical_replay",
        "execution_allowed": False,
        "error": error or (scene.get("error") if scene else None),
        "scene": scene,
    }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.headers.get("Host") not in {
            f"127.0.0.1:{PORT}", f"localhost:{PORT}"
        }:
            self.send_error(403)
            return

        origin = self.headers.get("Origin")
        if origin is not None and origin not in ALLOWED_ORIGINS:
            self.send_error(403)
            return

        if self.path != "/api/scene":
            self.send_error(404)
            return

        payload = json.dumps(snapshot(), allow_nan=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Vary", "Origin")
        if origin:
            self.send_header("Access-Control-Allow-Origin", origin)
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *_):
        pass


def main():
    rclpy.init()
    node = None
    server = None
    thread = None
    try:
        node = SceneBridge()
        server = ThreadingHTTPServer((HOST, PORT), Handler)
        server.daemon_threads = True
        thread = threading.Thread(
            target=server.serve_forever, daemon=True
        )
        thread.start()
        print(f"Scene API: http://{HOST}:{PORT}/api/scene", flush=True)
        print("Read-only historical replay; no robot commands.", flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(timeout=2)
            server.server_close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
