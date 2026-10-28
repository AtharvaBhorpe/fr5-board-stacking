"""Start and supervise the workcell, or stop the supervisor in this checkout."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
STOP_ORDER = "web chat speech llm demo-ui bridge ctrl skills world wrist vision fake motion monitor".split()


def group_alive(pgid):
    # Ignore unreaped zombies: they cannot run or receive signals.
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != "Z":
                return True
        except (FileNotFoundError, ProcessLookupError):
            pass
    return False


def down(lock):
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as control:
                control.sendto(b"stop", str(Path(lock.name).with_name("stack.sock")))
        except (FileNotFoundError, ConnectionRefusedError):
            raise RuntimeError("The launcher is starting or exiting. Run pixi run down again.") from None
        print("Waiting for the stack to stop...", flush=True)
        end = time.monotonic() + 180
        while time.monotonic() < end:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                print("Stack stopped.", flush=True)
                return 0
            except BlockingIOError:
                time.sleep(0.2)
        raise RuntimeError("Shutdown did not finish. Read the launcher terminal and process logs.")
    print("No stack is running.", flush=True)
    return 0


class Stack:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.log = root / "live_logs" / time.strftime("session_%Y%m%d_%H%M%S")
        self.log = self.log.with_name(f"{self.log.name}_{os.getpid()}")
        for name in ("ros", "monitor", "processes"):
            (self.log / name).mkdir(parents=True, exist_ok=True)
        current = root / "live_logs/current"
        current.unlink(missing_ok=True)
        current.symlink_to(self.log.name, target_is_directory=True)
        self.env = dict(os.environ, HK_LIVE_DIR=str(self.log), ROS_LOG_DIR=str(self.log / "ros"),
                        HK_VERBOSE=str(int(args.mode == "debug")), PYTHONUNBUFFERED="1")
        if args.sim:
            self.env["ROS_LOCALHOST_ONLY"] = "1"
            self.env["WORKCELL_REAL"] = "0"
        self.children = {}
        self.reported = set()
        self.stopping = False
        self.control_path = root / "live_logs/stack.sock"
        self.control_path.unlink(missing_ok=True)
        self.control = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.control.bind(str(self.control_path))
        self.control.setblocking(False)

    def request_stop(self, *_):
        self.stopping = True

    def check(self, *, startup=True):
        try:
            if self.control.recv(32) == b"stop":
                self.stopping = True
        except BlockingIOError:
            pass
        if self.stopping:
            raise KeyboardInterrupt
        for name, child in self.children.items():
            code = child.poll()
            if code is not None:
                message = f"{name} exited ({code}). Read {self.log / 'processes' / (name + '.log')}"
                if startup:
                    raise RuntimeError(message)
                if name not in self.reported:
                    print(f"ERROR: {message}", flush=True)
                    self.reported.add(name)

    def start(self, name, *command):
        self.check()
        with (self.log / "processes" / f"{name}.log").open("w") as out:
            self.children[name] = subprocess.Popen(command, cwd=self.root, env=self.env,
                                                   stdin=subprocess.DEVNULL, stdout=out,
                                                   stderr=subprocess.STDOUT, start_new_session=True)

    def wait_for(self, name, seconds, probe):
        print(f"Waiting for {name}...", flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check()
            if probe():
                print(f"{name}: ready", flush=True)
                return
            time.sleep(0.5)
        raise RuntimeError(f"{name}: NOT ready. Read {self.log / 'processes'}")

    def ros(self, *args):
        try:
            result = subprocess.run(["ros2", *args, "--no-daemon"], env=self.env,
                                    capture_output=True, text=True, timeout=5)
            return result.stdout if result.returncode == 0 else ""
        except subprocess.TimeoutExpired:
            return ""

    @staticmethod
    def url(url):
        try:
            with urllib.request.urlopen(url, timeout=2) as response:
                return response.status == 200
        except (OSError, urllib.error.URLError):
            return False

    def run(self):
        # Refuse occupied endpoints before a readiness probe can accept a different stack.
        for port in (8770, 8766, 8088, 8443, 8765):
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("127.0.0.1", port))
        if not (self.root / "web/.local/server.crt").exists():
            subprocess.run([sys.executable, "web/setup_tls.py"], cwd=self.root, env=self.env, check=True)
        print(f"Starting stack. Logs: {self.log}", flush=True)
        self.start("monitor", sys.executable, "tools/live_monitor.py", *(["--lite"] if self.args.mode == "demo" else []))
        self.start("llm", "pixi", "run", "--locked", "-e", "models", "llm")
        self.start("motion", "ros2", "launch", "hk_fr5_moveit_config", "bringup.launch.py",
                   f"use_fake_hardware:={str(self.args.sim).lower()}", f"use_rviz:={str(self.args.rviz).lower()}")
        self.wait_for("motion", 120, lambda: "/motion/get_state" in self.ros("service", "list").splitlines())
        node_args = ["--ros-args"] + (["--disable-stdout-logs"] if self.args.mode == "debug" else [])
        if self.args.sim:
            self.start("fake", "ros2", "run", "hk_fr5_task", "fake_world", *node_args, "-p", "contact_sim:=true")
        else:
            self.start("vision", sys.executable, "vision1/node.py", "--ros-args", "-p", "model:=true")
            self.start("wrist", "ros2", "launch", "vision1/wrist.launch.py")
        self.wait_for("vision", 90, lambda: self.has_publisher("/vision/boards"))
        self.start("world", "ros2", "run", "hk_fr5_task", "world_model", *node_args)
        self.wait_for("world model", 60, lambda: self.has_publisher("/task/world"))
        self.start("skills", "ros2", "run", "hk_fr5_task", "skill_server", *node_args)
        self.start("ctrl", "ros2", "run", "hk_fr5_task", "task_main_controller", *node_args)
        self.wait_for("controller", 60, lambda: "/task/list_jobs" in self.ros("service", "list").splitlines())
        self.start("bridge", sys.executable, "tools/agent_bridge.py")
        self.start("demo-ui", sys.executable, "tools/vacuum_demo_ui.py")
        self.wait_for("Gemma", 180, lambda: self.url("http://localhost:8770/health"))
        self.start("speech", "pixi", "run", "--locked", "-e", "models", "speech")
        self.wait_for("speech server", 180, lambda: self.url("http://localhost:8766/health"))
        self.start("chat", sys.executable, "conversation/bridge.py")
        self.start("web", sys.executable, "web/gateway.py")
        self.wait_for("web app", 30, lambda: self.url("http://localhost:8088"))
        self.wait_for("vacuum demo page", 20, lambda: self.url("http://localhost:8765/status"))
        print("All started. Web: http://localhost:8088 | Vacuum demo: http://localhost:8765\n"
              "Logs: pixi run logs | Stop: Ctrl+C or pixi run down", flush=True)
        if not self.args.no_gui and os.environ.get("DISPLAY"):
            for url in ("http://localhost:8088", "http://localhost:8765"):
                try:
                    subprocess.run(["xdg-open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   check=True, timeout=5)
                except (OSError, subprocess.SubprocessError):
                    print(f"Open {url} in a browser.", flush=True)
        while True:
            # A model/UI failure must not switch off the gripper during a running job.
            self.check(startup=False)
            time.sleep(0.5)

    def has_publisher(self, topic):
        return any(line.startswith("Publisher count:") and line.split(":", 1)[1].strip().isdigit()
                   and int(line.split(":", 1)[1]) > 0 for line in self.ros("topic", "info", topic).splitlines())

    def stop(self):
        # Signal only our process groups. The monitor closes last to finish its log index.
        remaining = []
        for name in STOP_ORDER:
            if name not in self.children:
                continue
            child = self.children[name]
            print(f"Stopping {name}...", flush=True)
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    break
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    child.poll()
                    if not group_alive(child.pid):
                        break
                    time.sleep(0.1)
                if not group_alive(child.pid):
                    break
            if group_alive(child.pid):
                remaining.append(name)
            else:
                child.wait(timeout=1)
        self.control.close()
        self.control_path.unlink(missing_ok=True)
        if remaining:
            raise RuntimeError(f"Processes did not stop: {', '.join(remaining)}. Read the process logs.")
        print("Stack stopped. Logs retained.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("up", "down"))
    parser.add_argument("mode", choices=("debug", "demo"), default="debug", nargs="?")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()
    (ROOT / "live_logs").mkdir(exist_ok=True)
    with (ROOT / "live_logs/stack.lock").open("a+") as lock:
        if args.action == "down":
            return down(lock)
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("A stack is already running in this checkout. Use pixi run down first.")
        if not (ROOT / "install/setup.sh").is_file() or not (ROOT / "web/dist/index.html").is_file():
            raise RuntimeError("Not built: run pixi run build first.")
        stack = Stack(ROOT, args)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, stack.request_stop)
        try:
            stack.run()
        except KeyboardInterrupt:
            return 0
        finally:
            stack.stop()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        sys.exit(1)
