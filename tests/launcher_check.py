"""Exercise startup with stand-in processes. No ROS graph, models or hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LauncherCheck(unittest.TestCase):
    def launch(self, failure=""):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("tools", "install", "web/dist", "web/.local", "bin"):
                (root / name).mkdir(parents=True)
            shutil.copy2(ROOT / "tools/up.sh", root / "tools/up.sh")
            for name in ("install/setup.sh", "web/dist/index.html", "web/.local/server.crt"):
                (root / name).touch()
            scripts = {
                "tmux": '''
echo "$*" >> "$FIXTURE/tmux.log"
case "$3" in
  has-session) test -f "$FIXTURE/session" ;;
  new-session) touch "$FIXTURE/session" ;;
esac
''',
                "pixi": '''
case "$*" in
  *"service list"*)
    [ "$FAILURE" = motion ] && exit 1
    printf '/motion/get_state\\n/task/list_jobs\\n' ;;
  *"topic info"*) echo 'Publisher count: 1' ;;
  *) exit 1 ;;
esac
''',
                "curl": '''
[ "$FAILURE" = web ] && [ "$2" = localhost:8088 ] && exit 1
exit 0
''',
                "date": '''
if [ "$1" = +%s ]; then
  n=0
  [ -f "$FIXTURE/clock" ] && read -r n < "$FIXTURE/clock"
  echo "$n"
else
  echo fixture
fi
''',
                "sleep": '''
n=0
[ -f "$FIXTURE/clock" ] && read -r n < "$FIXTURE/clock"
echo $((n + 10000)) > "$FIXTURE/clock"
''',
            }
            for name, body in scripts.items():
                path = root / "bin" / name
                path.write_text("#!/bin/bash\n" + body)
                path.chmod(0o755)
            env = dict(os.environ, PATH=f"{root / 'bin'}:{os.environ['PATH']}",
                       FIXTURE=str(root), FAILURE=failure, ROS_LOCALHOST_ONLY="0")
            result = subprocess.run(["bash", str(root / "tools/up.sh"), "--sim", "--no-gui"],
                                    env=env, text=True, capture_output=True, timeout=10)
            return result, (root / "tmux.log").read_text()

    def test_motion_timeout_stops_before_dependent_nodes(self):
        result, commands = self.launch("motion")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("NOT ready", result.stdout)
        self.assertNotIn("-n fake", commands)
        self.assertNotIn("all started", result.stdout)
        self.assertNotIn("kill-server", commands)

    def test_web_timeout_does_not_report_success(self):
        result, _ = self.launch("web")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertRegex(result.stdout, r"web app \(:8088\).*NOT ready")
        self.assertNotIn("all started", result.stdout)

    def test_ready_stack_succeeds_and_simulation_forces_localhost(self):
        result, commands = self.launch()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("all started", result.stdout)
        windows = [line for line in commands.splitlines() if "send-keys" in line]
        self.assertTrue(windows)
        self.assertTrue(all("export ROS_LOCALHOST_ONLY=1;" in line for line in windows))
        self.assertNotIn("pixi run robot", commands)
        self.assertNotIn("-n wrist", commands)


if __name__ == "__main__":
    unittest.main()
