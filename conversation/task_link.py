"""Send one command to the main controller through its agent bridge
(tools/agent_bridge.py, started by tools/stack.py): a line into <bridge>/cmd, the answer from <bridge>/events.txt
("CMD <command> -> OK j3 ..." / "-> REJECTED <reason>"). The same text commands as the tc command line.

    python conversation/task_link.py transfer VARa drop_off_a 1
"""
import glob
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def bridge_dir():
    """$HK_LIVE_DIR/bridge (set by tools/stack.py), else the newest <repo>/live_logs/session_*/bridge."""
    if os.environ.get("HK_LIVE_DIR"):
        return os.path.join(os.environ["HK_LIVE_DIR"], "bridge")
    pipes = glob.glob(os.path.join(HERE, "..", "live_logs", "session_*", "bridge", "cmd"))
    if not pipes:
        raise RuntimeError("main controller not found: start it with: pixi run up")
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
            raise RuntimeError("main controller agent bridge is not running (pixi run up)") from e
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
