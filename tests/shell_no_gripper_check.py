"""motion_shell without / with a disappearing gripper node."""
from testenv import MOCK, SHELL, GRIPPER_CFG, LOG_DIR  # noqa: F401 (chdir to the log dir)
import os, signal, subprocess, sys, time
launch = subprocess.Popen(["ros2", "launch", "hk_fr5_moveit_config", "bringup.launch.py", "use_rviz:=false", "use_gripper:=false"],
                          stdout=open("ng_launch.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
def gripper():
    return subprocess.Popen(["ros2", "run", "hk_fr5_gripper", "gripper_controller", "--ros-args", "-r", "__ns:=/gripper",
                             "--params-file", GRIPPER_CFG, "-p", "fake_vacuum:=false"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
sh = None
try:
    time.sleep(15)
    sh = subprocess.Popen([sys.executable, SHELL], stdin=subprocess.PIPE, stdout=open("ng_shell.log", "w"),
                          stderr=subprocess.STDOUT, text=True)
    def say(line, wait):
        sh.stdin.write(line + "\n"); sh.stdin.flush(); time.sleep(wait)
    time.sleep(6)                                   # startup status
    say("status", 4); say("gstate", 2); say("grip", 3); say("release 0.5", 3)
    say("d 0 0 10 base", 2.5); say("d 0 0 -10 base", 2.5)
    g = gripper(); time.sleep(4)                    # B: gripper starts later
    say("gstate", 2); say("grip 1", 3)
    os.killpg(g.pid, signal.SIGINT); g.wait(timeout=10); time.sleep(1)
    say("gstate", 2)                                # must not show the old state
    g = gripper(); time.sleep(4)                    # C1: clean stop during a grip
    say("grip 8", 1.5); t = time.time(); os.killpg(g.pid, signal.SIGINT); g.wait(timeout=10); time.sleep(3)
    g = gripper(); time.sleep(4)                    # C2: hard kill during a grip
    say("grip 8", 1.5); os.killpg(g.pid, signal.SIGKILL); g.wait(timeout=10); time.sleep(12)
    say("history", 1); say("quit", 2)
    sh.wait(timeout=20)
finally:
    if sh and sh.poll() is None: sh.kill()
    for p in (launch,):
        try: os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError: pass
    time.sleep(8)
    try: os.killpg(launch.pid, signal.SIGKILL)
    except ProcessLookupError: pass
print(open("ng_shell.log").read())
