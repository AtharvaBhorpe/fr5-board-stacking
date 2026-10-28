"""Launcher lifecycle checks with local stand-ins. No ROS, models or hardware."""
import argparse
import contextlib
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import stack


class LauncherCheck(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="stack test ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ("install/setup.sh", "web/dist/index.html", "web/.local/server.crt"):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        # Exercise the real supervisor and lock with harmless stand-ins for ROS/models.
        self.harness = self.root / "harness.py"
        self.harness.write_text(f'''
import sys, time
from pathlib import Path
sys.path.insert(0, {str(Path(stack.__file__).parent)!r})
import stack
stack.ROOT = Path(__file__).parent
def run(self):
    self.start("monitor", sys.executable, "-u", "-c", "import time; print('monitor ready', flush=True); time.sleep(60)")
    self.start("motion", sys.executable, "-u", "-c", "import time; print('motion ready', flush=True); time.sleep(60)")
    (self.root / "ready").write_text(str(self.children["motion"].pid))
    if self.args.mode == "demo":
        self.wait_for("fixture", 0.2, lambda: False)
    while True:
        self.check(startup=False)
        time.sleep(0.02)
stack.Stack.run = run
if __name__ == "__main__":
    try:
        sys.exit(stack.main())
    except RuntimeError as error:
        print(error, flush=True)
        sys.exit(1)
''')

    def launch(self, *args):
        path = self.root / f"launcher-{time.monotonic_ns()}.log"
        with path.open("w") as log:
            p = subprocess.Popen([sys.executable, str(self.harness), "up", "--sim", *args],
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(self.finish, p)
        return p, path

    @staticmethod
    def finish(p):
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=15)

    def ready(self):
        end = time.monotonic() + 5
        while time.monotonic() < end:
            if (self.root / "ready").exists():
                return int((self.root / "ready").read_text())
            time.sleep(0.02)
        self.fail("Fixture did not start")

    def down(self):
        return subprocess.run([sys.executable, str(self.harness), "down"],
                              text=True, capture_output=True, timeout=15)

    def test_down_stops_owned_processes_and_is_idempotent(self):
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True)
        self.addCleanup(self.finish, unrelated)
        p, _ = self.launch()
        motion = self.ready()
        result = self.down()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(p.wait(timeout=5), 0)
        self.assertFalse(stack.group_alive(motion))
        self.assertIsNone(unrelated.poll())
        self.assertIn("No stack is running", self.down().stdout)
        (self.root / "ready").unlink()
        second, _ = self.launch()
        self.ready()
        self.assertIsNone(second.poll())
        self.assertEqual(self.down().returncode, 0)

    def test_duplicate_start_preserves_running_stack(self):
        p, _ = self.launch()
        self.ready()
        duplicate, log = self.launch()
        self.assertEqual(duplicate.wait(timeout=5), 1)
        self.assertIn("already running", log.read_text())
        self.assertIsNone(p.poll())
        self.assertEqual(self.down().returncode, 0)

    def test_readiness_failure_cleans_up_and_keeps_logs(self):
        p, log = self.launch("demo")
        motion = self.ready()
        self.assertEqual(p.wait(timeout=10), 1)
        self.assertIn("NOT ready", log.read_text())
        self.assertFalse(stack.group_alive(motion))
        self.assertIn("motion ready", (self.root / "live_logs/current/processes/motion.log").read_text())

    def test_signals_stop_the_stack(self):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=sig):
                (self.root / "ready").unlink(missing_ok=True)
                p, _ = self.launch()
                motion = self.ready()
                p.send_signal(sig)
                self.assertEqual(p.wait(timeout=10), 0)
                self.assertFalse(stack.group_alive(motion))

    def test_runtime_crash_is_reported_without_stopping_healthy_processes(self):
        p, log = self.launch()
        motion = self.ready()
        os.killpg(motion, signal.SIGKILL)
        end = time.monotonic() + 5
        while "motion exited" not in log.read_text() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertIn("motion exited", log.read_text())
        self.assertIsNone(p.poll())
        self.assertNotIn("Stopping monitor", log.read_text())
        self.assertEqual(self.down().returncode, 0)
        self.assertIn("Stopping monitor", log.read_text())

    def test_real_start_sequence_forces_simulation_isolation(self):
        args = argparse.Namespace(sim=True, mode="debug", rviz=False, no_gui=False)
        with patch.dict(os.environ, ROS_LOCALHOST_ONLY="0", WORKCELL_REAL="1"):
            s = stack.Stack(self.root, args)
        self.addCleanup(s.control.close)
        with patch.object(s, "start") as start, patch.object(s, "ros", return_value=(
                "/motion/get_state\n/task/list_jobs\nPublisher count: 1\n")), \
                patch.object(s, "url", return_value=True), patch("stack.socket.socket"), \
                patch("stack.time.sleep", side_effect=lambda _: s.request_stop()), \
                patch.dict(os.environ, DISPLAY=":0"), \
                patch("stack.subprocess.run", side_effect=FileNotFoundError("xdg-open")), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            with self.assertRaises(KeyboardInterrupt):
                s.run()
        self.assertIn("All started", output.getvalue())
        self.assertIn("Open http://localhost:8088 in a browser", output.getvalue())
        names = [call.args[0] for call in start.call_args_list]
        self.assertEqual(names, ["monitor", "llm", "motion", "fake", "world", "skills", "ctrl",
                                 "bridge", "demo-ui", "speech", "chat", "web"])
        self.assertEqual(s.env["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(s.env["WORKCELL_REAL"], "0")
        self.assertIn("use_fake_hardware:=true", start.call_args_list[2].args)


if __name__ == "__main__":
    unittest.main()
