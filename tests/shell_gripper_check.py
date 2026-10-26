"""Step 20e: drive motion_shell via stdin next to a headless sim bringup."""
from testenv import MOCK, SHELL, GRIPPER_CFG, LOG_DIR  # noqa: F401 (chdir to the log dir)
import os, signal, subprocess, sys, time
launch = subprocess.Popen(["ros2", "launch", "hk_fr5_moveit_config", "bringup.launch.py", "use_rviz:=false"],
                          stdout=open("shell_launch.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
try:
    time.sleep(15)
    def shell(lines, interrupt_after=None):
        p = subprocess.Popen([sys.executable, SHELL], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if interrupt_after is None:
            return p.communicate("\n".join(lines) + "\n", timeout=90)[0]
        p.stdin.write("\n".join(lines) + "\n"); p.stdin.flush()
        time.sleep(interrupt_after); p.send_signal(signal.SIGINT); time.sleep(1.5)
        return p.communicate("quit\n", timeout=30)[0]
    out = shell(["gstate", "grip", "gstate", "grip", "release", "gstate", "release 0.3",
                 "grip -1", "grip 1 2", "grip abc", "d 0 0 10 base", "d 0 0 -10 base", "history", "quit"])
    print(out)
    subprocess.run(["ros2", "param", "set", "/gripper/gripper_controller", "fake_vacuum", "false"], capture_output=True, timeout=20)
    print("=========== no vacuum, grip 1, then grip 5 + Ctrl+C after 1.5 s")
    print(shell(["grip 1"]))
    print(shell(["grip 5"], interrupt_after=3.0))
finally:
    os.killpg(launch.pid, signal.SIGINT); time.sleep(8)
    try: os.killpg(launch.pid, signal.SIGKILL)
    except ProcessLookupError: pass
