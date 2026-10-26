"""Robot tcp pose for "record current tcp" (decision-log V-9, V-12). No ROS, READ-ONLY: never sends to the robot.

Reads the controller real-time state packet (TCP port 20004, ROBOT_STATE_PKG in libfairino robot_types.h,
packed, little-endian) while the robot is hand-guided in manual mode (our motion bringup must NOT run), and computes
the tcp in base_link by own forward kinematics from the joints (same chain as our URDF, config/robot.yaml).
The controller's own flange pose (pendant frame) is only a cross-check (x / y negated vs base_link).

    cfg = load_config(path); r = RobotReader(cfg); r.start()
    st, age = r.latest(); p = tcp_from_state(st, cfg)   # p["x"], p["y"], p["z"], p["tilt_deg"], p["flange_check_mm"]
"""
import math
import socket
import struct
import threading
import time

import numpy as np
import yaml

HEAD = b"\x5a\x5a"
HDR = struct.Struct("<HBHBBiiB")        # frame_head, frame_cnt, data_len, program_state, robot_state, main, sub, mode
POSES = struct.Struct("<18d")           # jt_cur_pos[6] (deg), tl_cur_pos[6], flange_cur_pos[6] (mm / deg) at offset 16
MIN_LEN = 16 + POSES.size + 2           # we need the fields up to flange_cur_pos + the checksum
MAX_LEN = 8192
ESTOP_OFFSET = 549                      # uint8 EmergencyStop (v3.9.0.1 header layout; None if the packet is shorter)


class State:
    """One decoded packet (only the fields we use)."""
    __slots__ = ("t", "frame_cnt", "data_len", "program_state", "robot_state", "main_code", "sub_code",
                 "robot_mode", "joints_deg", "tool_pose", "flange_pose", "estop", "checksum_ok")

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class PacketParser:
    """Stream -> packets. Packet = 5-byte header + data_len bytes + uint16 checksum. Accepted when the checksum
    (sum of all bytes before it, & 0xFFFF) matches, OR the next packet header follows exactly (checksum convention
    unverified on v3.9.2: counted in stats, lab item VT1). Garbage / misaligned data -> resync on 0x5A5A."""

    def __init__(self):
        self.buf = bytearray()
        self.stats = {"packets": 0, "checksum_ok": 0, "checksum_bad_framing_ok": 0, "rejected": 0, "resync_bytes": 0}

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(HEAD)
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\x5a" else 0
                self.stats["resync_bytes"] += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                break
            if i:
                self.stats["resync_bytes"] += i
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            data_len = struct.unpack_from("<H", self.buf, 3)[0]
            n = 5 + data_len + 2
            if n < MIN_LEN or n > MAX_LEN:
                self._reject(); continue
            if len(self.buf) < n:
                break
            cs_ok = (sum(self.buf[:n - 2]) & 0xFFFF) == struct.unpack_from("<H", self.buf, n - 2)[0]
            if not cs_ok:
                if len(self.buf) < n + 2:
                    break                      # wait: does the next packet start right after this one?
                if self.buf[n:n + 2] != HEAD:
                    self._reject(); continue
            st = self._decode(bytes(self.buf[:n]), cs_ok)
            del self.buf[:n]
            if st is None:
                self.stats["rejected"] += 1; continue
            self.stats["packets"] += 1
            self.stats["checksum_ok" if cs_ok else "checksum_bad_framing_ok"] += 1
            out.append(st)
        return out

    def _reject(self):
        self.stats["rejected"] += 1
        del self.buf[:2]                       # skip this header, search the next one

    @staticmethod
    def _decode(p, cs_ok):
        _, cnt, dlen, prog, rstate, main, sub, mode = HDR.unpack_from(p, 0)
        v = POSES.unpack_from(p, 16)
        if not all(math.isfinite(a) for a in v) or max(abs(a) for a in v[:6]) > 400:
            return None                        # not a plausible joint state -> misaligned / corrupt
        return State(t=time.monotonic(), frame_cnt=cnt, data_len=dlen, program_state=prog, robot_state=rstate,
                     main_code=main, sub_code=sub, robot_mode=mode, joints_deg=v[0:6], tool_pose=v[6:12],
                     flange_pose=v[12:18], estop=p[ESTOP_OFFSET] if len(p) > ESTOP_OFFSET + 2 else None,
                     checksum_ok=cs_ok)


class RobotReader(threading.Thread):
    """Background reader: connect, read, decode, keep the latest state; reconnects; read-only (never sends)."""

    def __init__(self, cfg):
        super().__init__(daemon=True)
        self.cfg = cfg
        self._quit = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._sock = None
        self.parser = PacketParser()
        self.connected = False
        self.last_error = "not started"
        self._times = []

    def run(self):
        c = self.cfg
        while not self._quit.is_set():
            try:
                self._sock = socket.create_connection((c["robot_ip"], c["state_port"]), timeout=c["connect_timeout"])
                self._sock.settimeout(0.2)
                self.connected, self.last_error = True, ""
                self.parser = PacketParser()
                while not self._quit.is_set():
                    try:
                        data = self._sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not data:
                        raise ConnectionError("controller closed the connection")
                    states = self.parser.feed(data)
                    if states:
                        with self._lock:
                            self._latest = states[-1]
                            now = time.monotonic()
                            self._times = [t for t in self._times if now - t < 1.0] + [now] * len(states)
            except OSError as e:
                self.last_error = f"{type(e).__name__}: {e}"
            finally:
                self.connected = False
                if self._sock:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
            self._quit.wait(c["reconnect_period"])

    def latest(self):
        """(State or None, age in s or None)."""
        with self._lock:
            st = self._latest
        return st, (time.monotonic() - st.t) if st else None

    def status(self):
        st, age = self.latest()
        with self._lock:
            now = time.monotonic()
            rate = len([t for t in self._times if now - t < 1.0])
        return {"connected": self.connected, "last_error": self.last_error, "rate_hz": rate,
                "age_s": None if age is None else round(age, 3),
                "fresh": age is not None and age < self.cfg["stale_after"], **self.parser.stats}

    def stop(self):
        self._quit.set()
        s = self._sock
        if s:
            try:
                s.close()
            except OSError:
                pass


def _rot(r, p, y):
    """URDF rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(xyz, rpy=(0.0, 0.0, 0.0)):
    T = np.eye(4); T[:3, :3] = _rot(*rpy); T[:3, 3] = xyz
    return T


def fk(joints_rad, cfg):
    """(T_flange, T_tcp) 4x4 in base_link from 6 joint angles (rad)."""
    T = np.eye(4)
    for link, q in zip(cfg["chain"], joints_rad):
        T = T @ _tf(link["xyz"], link["rpy"]) @ _tf((0, 0, 0), (0, 0, q))
    T_fl = T @ _tf((0, 0, cfg["flange_z"]))
    return T_fl, T_fl @ _tf((0, 0, cfg["tcp_z"]))


def tcp_from_state(st, cfg):
    """tcp x / y / z (m, base_link), tool tilt from straight down (deg), FK flange vs controller flange (mm)."""
    T_fl, T = fk([math.radians(a) for a in st.joints_deg], cfg)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -T[2, 2]))))   # tool z axis vs -z of base_link
    f = st.flange_pose   # pendant frame, mm: base_link = x, y negated (CLAUDE.md / decision-log: base_link rotated 180 deg)
    ctrl = np.array([-f[0], -f[1], f[2]]) / 1000.0
    return {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3]), "tilt_deg": tilt,
            "tilt_ok": tilt <= cfg["max_tilt_deg"], "flange_check_mm": float(np.linalg.norm(T_fl[:3, 3] - ctrl) * 1000),
            "manual_mode": st.robot_mode == 1, "robot_state": st.robot_state, "estop": st.estop,
            "joints_deg": list(st.joints_deg)}


def load_config(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    for k in ("robot_ip", "state_port", "connect_timeout", "stale_after", "reconnect_period", "max_tilt_deg",
              "chain", "flange_z", "tcp_z"):
        if k not in c:
            raise ValueError(f"{path}: missing '{k}'")
    if len(c["chain"]) != 6 or not all(len(l["xyz"]) == 3 and len(l["rpy"]) == 3 for l in c["chain"]):
        raise ValueError(f"{path}: 'chain' needs 6 entries with xyz and rpy")
    return c


def main():
    """Lab probe (VT1), READ-ONLY: prints the decoded state + our tcp at ~2 Hz.
    ros2 run hk_fr5_vision robot_pose_probe [seconds] [config]   (bringup off, robot in manual mode)"""
    import os
    import sys
    from ament_index_python.packages import get_package_share_directory
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "robot.yaml")
    cfg = load_config(path)
    r = RobotReader(cfg); r.start()
    print(f"read-only probe {cfg['robot_ip']}:{cfg['state_port']} for {secs:.0f} s (nothing is sent to the robot)")
    end = time.time() + secs
    try:
        while time.time() < end:
            time.sleep(0.5)
            s, (st, _) = r.status(), r.latest()
            line = (f"conn {s['connected']} fresh {s['fresh']} {s['rate_hz']} Hz pk {s['packets']} cs_ok {s['checksum_ok']} "
                    f"cs_bad {s['checksum_bad_framing_ok']} rej {s['rejected']}")
            if st:
                t = tcp_from_state(st, cfg)
                line += (f" | mode {'manual' if t['manual_mode'] else 'auto'} state {st.robot_state} len {st.data_len} "
                         f"estop {st.estop} | joints {' '.join(f'{a:+.2f}' for a in st.joints_deg)} | tcp "
                         f"{t['x'] * 1000:+.1f} {t['y'] * 1000:+.1f} {t['z'] * 1000:+.1f} mm tilt {t['tilt_deg']:.1f} deg "
                         f"| flange check {t['flange_check_mm']:.1f} mm")
            elif s["last_error"]:
                line += f" | {s['last_error']}"
            print(line, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        r.stop()
