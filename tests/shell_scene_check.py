"""motion_shell scene commands (T20e): without a bringup, then with one (use_gripper:=false). Output is printed;
compare with notes/implementation-ref.md "T20e results"."""
import os, subprocess, sys, time
from scene_lib import launch, stop, SHELL
def shell(lines):
    p = subprocess.Popen([sys.executable, SHELL], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return p.communicate("\n".join(lines) + "\nquit\n", timeout=120)[0]
print("=== no bringup running"); print(shell(["status", "attach 270 200 6", "detach", "box rm"]))
L = launch("use_gripper:=false", log="shell_scene_launch.log"); time.sleep(18)
try:
    print("=== bringup running")
    print(shell(["status", "attach 270 200 6", "attach 270 200 6", "detach keep",
                 "box add wall 300 20 300 -450 0 150", "box add wall 300 20 300 -450 0 160 15", "box add table 10 10 10 0 0 0",
                 "box rm table", "box rm wall", "box rm nothere", "box rm",
                 "detach", "attach 270 200 6 0 0 0 10", "attach 270 200 6 30 10 -5 1.5", "detach",
                 "attach 270 200", "box foo", "detach now", "history"]))
finally:
    stop(L, wait=12)
