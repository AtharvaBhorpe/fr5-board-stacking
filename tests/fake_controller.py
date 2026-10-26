"""Fake Fairino controller state port (ROBOT_STATE_PKG packets, v3.9.0.1 layout) for tests and app development.

    pixi run python3 tests/fake_controller.py [port] [--ready]   # stream on 127.0.0.1:<port> (default 20004), 100 Hz
Standalone: joints move slowly around the 'ready' pose (j1 sweeps +-20 deg), manual mode, flange from our FK.
"""
import math
import os
import socket
import struct
import sys
import threading
import time

from hk_fr5_vision import robot_pose as rp


def packet(joints=(0, -90, 90, -90, -90, 0), tool=(0,) * 6, flange=(0,) * 6, mode=1, rstate=4, cnt=0, data_len=1229,
           estop=0, checksum="sum"):
    p = bytearray(5 + data_len + 2)
    rp.HDR.pack_into(p, 0, 0x5A5A, cnt & 0xFF, data_len, 1, rstate, 0, 0, mode)
    rp.POSES.pack_into(p, 16, *joints, *tool, *flange)
    if len(p) > rp.ESTOP_OFFSET + 2:
        p[rp.ESTOP_OFFSET] = estop
    cs = sum(p[:-2]) & 0xFFFF
    struct.pack_into("<H", p, len(p) - 2, cs if checksum == "sum" else (cs + 1) & 0xFFFF)
    return bytes(p)


class FakeController(threading.Thread):
    def __init__(self, port=0, rate=100.0):
        super().__init__(daemon=True)
        self.srv = socket.create_server(("127.0.0.1", port)); self.port = self.srv.getsockname()[1]
        self.rate, self.running, self.received, self.clients = rate, True, b"", 0
        from ament_index_python.packages import get_package_share_directory
        self.cfg = rp.load_config(os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "robot.yaml"))
    def run(self):
        self.srv.settimeout(0.2)
        while self.running:
            try:
                c, _ = self.srv.accept()
            except OSError:
                continue
            self.clients += 1; c.settimeout(0.0); cnt = 0
            try:
                while self.running:
                    j1 = (cnt % 360) * 0.1
                    q = (j1, -90, 90, -90, -90, 0)
                    T_fl, _ = rp.fk([math.radians(a) for a in q], self.cfg)   # flange like a real controller (pendant frame)
                    c.sendall(packet(joints=q, flange=(-T_fl[0, 3] * 1000, -T_fl[1, 3] * 1000, T_fl[2, 3] * 1000, 0, 0, 0),
                                     cnt=cnt)); cnt += 1
                    try:
                        self.received += c.recv(100)   # the reader must never send anything
                    except (BlockingIOError, OSError):
                        pass
                    time.sleep(1.0 / self.rate)
            except OSError:
                pass
            finally:
                c.close()
    def close(self):
        self.running = False; self.srv.close()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 20004
    from ament_index_python.packages import get_package_share_directory
    cfg = rp.load_config(os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "robot.yaml"))
    srv = socket.create_server(("127.0.0.1", port))
    print(f"fake controller on 127.0.0.1:{port} (Ctrl+C to stop)", flush=True)
    try:
        while True:
            c, _ = srv.accept(); t0 = time.time()
            try:
                while True:
                    a = time.time() - t0
                    q = (20 * math.sin(a / 4), -90, 90, -90, -90, 0) if "--ready" not in sys.argv else (0, -90, 90, -90, -90, 0)
                    T_fl, _ = rp.fk([math.radians(x) for x in q], cfg)
                    fl = (-T_fl[0, 3] * 1000, -T_fl[1, 3] * 1000, T_fl[2, 3] * 1000, 0, 0, 0)
                    c.sendall(packet(joints=q, flange=fl, cnt=int(a * 100))); time.sleep(0.01)
            except OSError:
                c.close()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
