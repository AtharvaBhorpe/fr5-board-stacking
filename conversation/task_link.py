"""Send one command to the main controller (feat/task-main-controller) through its agent bridge
(tools/agent_bridge.py, started by tools/trial_up.sh): a line into <bridge>/cmd, the answer from <bridge>/events.txt
("CMD <command> -> OK j3 ..." / "-> REJECTED <reason>"). No ROS types: the controller runs Python 3.14, this env 3.12.

    python conversation/task_link.py transfer VARa drop_off_a 1
"""
import glob
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def bridge_dir():
    """$HK_BRIDGE_DIR, else the newest <ws>/live_logs/session_*/bridge (as the controller's tools/paths.sh)."""
    if os.environ.get("HK_BRIDGE_DIR"):
        return os.environ["HK_BRIDGE_DIR"]
    pipes = [p for up in ("..", "../..", "../../..") for p in
             glob.glob(os.path.join(HERE, up, "live_logs", "session_*", "bridge", "cmd"))]
    if not pipes:
        raise RuntimeError("main controller not found: start it with tools/trial_up.sh")
    return os.path.dirname(max(pipes, key=os.path.getmtime))


def send(command, timeout=12.0):
    """-> (ok, job_id, message). RuntimeError when the bridge is not running or does not answer."""
    d = bridge_dir()
    events = os.path.join(d, "events.txt")
    start = os.path.getsize(events) if os.path.exists(events) else 0
    try:  # non-blocking: fails at once (ENXIO) when no bridge reads the pipe
        fd = os.open(os.path.join(d, "cmd"), os.O_WRONLY | os.O_NONBLOCK)
    except OSError:
        time.sleep(0.1)  # the bridge re-opens the pipe after each writer
        try:
            fd = os.open(os.path.join(d, "cmd"), os.O_WRONLY | os.O_NONBLOCK)
        except OSError as e:
            raise RuntimeError("main controller agent bridge is not running (tools/trial_up.sh)") from e
    os.write(fd, (command + "\n").encode())
    os.close(fd)
    want, end = f" CMD {command} -> ", time.monotonic() + timeout
    while time.monotonic() < end:
        with open(events) as f:
            f.seek(start)
            for line in f:
                if want in line:
                    ok, _, rest = line.split(want, 1)[1].strip().partition(" ")
                    if ok != "OK":
                        return False, "", line.split(want, 1)[1].strip().removeprefix("REJECTED").strip()
                    job, _, msg = rest.partition(" ") if rest.startswith("j") else ("", "", rest)
                    return True, job, msg
        time.sleep(0.05)
    raise RuntimeError(f"no answer from the main controller within {timeout:.0f} s")


if __name__ == "__main__":
    import sys
    print(send(" ".join(sys.argv[1:])))
