"""Offline checks of hk_fr5_vision.robot_pose (V5): own FK vs our URDF (robot_state_publisher TF fixture),
packet parser (framing, resync, checksum, plausibility), RobotReader against a fake port-20004 server (rate,
latest, read-only, disconnect -> stale, reconnect), tcp / tilt / flange cross-check, config.

    pixi run python3 tests/robot_pose_check.py
"""
import json
import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from hk_fr5_vision import robot_pose as rp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_controller import FakeController, packet  # noqa: E402

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "robot.yaml")
cfg = rp.load_config(CFG_PATH)
REF = json.load(open(os.path.join(HERE, "fixtures", "fk_reference.json")))["poses"]

# 1 FK vs our URDF (TF from robot_state_publisher, motion branch)
def quat_z(q):   # z axis of a rotation given as quaternion xyzw
    x, y, z, w = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
dp, df, dz = [], [], []
for r in REF:
    T_fl, T = rp.fk([math.radians(a) for a in r["joints_deg"]], cfg)
    dp.append(np.linalg.norm(T[:3, 3] - r["tcp"]) * 1000); df.append(np.linalg.norm(T_fl[:3, 3] - r["flange"]) * 1000)
    dz.append(math.degrees(math.acos(min(1.0, float(np.dot(T[:3, 2], quat_z(r["tcp_quat_xyzw"])))))))
check(f"FK vs URDF TF, {len(REF)} random joint sets: tcp < 0.01 mm, flange < 0.01 mm, tool axis < 0.01 deg",
      max(dp) < 0.01 and max(df) < 0.01 and max(dz) < 0.01, f"max tcp {max(dp):.5f} mm, flange {max(df):.5f} mm, axis {max(dz):.5f} deg")

# 2 packets (layout of ROBOT_STATE_PKG, v3.9.0.1 header: 1236 bytes, data_len 1229)
P = packet(joints=(1, -91, 92, -93, -94, 5), flange=(10, 20, 30, 180, 0, 90), cnt=7, estop=1)
pp = rp.PacketParser(); s = pp.feed(P)
check("one packet: decoded fields (joints, flange, mode, robot_state, estop, cnt, checksum ok)",
      len(s) == 1 and s[0].joints_deg == (1, -91, 92, -93, -94, 5) and s[0].flange_pose[:3] == (10, 20, 30)
      and s[0].robot_mode == 1 and s[0].robot_state == 4 and s[0].estop == 1 and s[0].frame_cnt == 7 and s[0].checksum_ok
      and s[0].data_len == 1229)
pp = rp.PacketParser(); s = []
for b in packet(cnt=1) + packet(cnt=2) + packet(cnt=3):
    s += pp.feed(bytes([b]))
check("3 packets fed byte by byte: all 3, in order", [x.frame_cnt for x in s] == [1, 2, 3], str([x.frame_cnt for x in s]))
pp = rp.PacketParser(); s = pp.feed(b"\x00\x5a\x13garbage" + packet(cnt=4) + b"\x5a\x00junk" + packet(cnt=5))
check("garbage before / between packets: resync, both decoded", [x.frame_cnt for x in s] == [4, 5] and pp.stats["resync_bytes"] > 0,
      str(pp.stats))
pp = rp.PacketParser(); s = pp.feed(packet(cnt=6, checksum="bad") + packet(cnt=7))
check("checksum mismatch but next header follows: accepted, counted (convention unverified, VT1)",
      [x.frame_cnt for x in s] == [6, 7] and pp.stats["checksum_bad_framing_ok"] == 1 and not s[0].checksum_ok, str(pp.stats))
pp = rp.PacketParser(); s = pp.feed(packet(cnt=8, checksum="bad") + b"xx" + packet(cnt=9))
check("checksum mismatch and no header after it: rejected, next packet found", [x.frame_cnt for x in s] == [9]
      and pp.stats["rejected"] >= 1, str(pp.stats))
pp = rp.PacketParser(); s = pp.feed(packet(cnt=10, data_len=1400) + packet(cnt=11, data_len=1400))
check("longer packet (newer firmware, data_len 1400): decoded", [x.frame_cnt for x in s] == [10, 11])
pp = rp.PacketParser(); s = pp.feed(b"\x5a\x5a\x00\x05\x00" + b"\x00" * 20 + packet(cnt=12))
check("implausible data_len (5): skipped, next packet decoded", [x.frame_cnt for x in s] == [12], str(pp.stats))
pp = rp.PacketParser(); s = pp.feed(packet(joints=(float("nan"), 0, 0, 0, 0, 0), cnt=13) + packet(cnt=14)
                                   + packet(joints=(999, 0, 0, 0, 0, 0), cnt=15) + packet(cnt=16))
check("NaN / 999 deg joints: rejected, good packets kept", [x.frame_cnt for x in s] == [14, 16], str([x.frame_cnt for x in s]))
pp = rp.PacketParser(); s = pp.feed(packet(cnt=17)[:400])
check("incomplete packet: waits (nothing yet), completes later", s == [] and [x.frame_cnt for x in pp.feed(packet(cnt=17)[400:])] == [17])

# 3 tcp / tilt / flange cross-check from a packet built from the URDF reference
ready = REF[0]
fl = ready["flange"]; ctrl_fl = (fl[0] * 1000, fl[1] * 1000, fl[2] * 1000, 0, 0, 0)   # controller = base_link frame, mm (V-26)
st = rp.PacketParser().feed(packet(joints=ready["joints_deg"], flange=ctrl_fl))[0]
t = rp.tcp_from_state(st, cfg)
check("ready pose: tcp = URDF reference, tilt 0, flange cross-check ~0 mm, manual mode",
      np.linalg.norm(np.array([t["x"], t["y"], t["z"]]) - ready["tcp"]) < 1e-5 and t["tilt_deg"] < 0.01 and t["tilt_ok"]
      and t["flange_check_mm"] < 0.01 and t["manual_mode"], f"tcp {t['x']:.4f} {t['y']:.4f} {t['z']:.4f}, tilt {t['tilt_deg']:.3f}")
st = rp.PacketParser().feed(packet(joints=ready["joints_deg"], flange=(-fl[0] * 1000, -fl[1] * 1000, fl[2] * 1000, 0, 0, 0)))[0]
check("flange cross-check catches a frame error (x / y negated): > 100 mm", rp.tcp_from_state(st, cfg)["flange_check_mm"] > 100)
# real FR5, 2026-10-26 (lab, read-only): joints + the controller's own flange from one packet -> our FK within 1 mm
st = rp.PacketParser().feed(packet(joints=(-3.25, -65.37, 82.62, -115.88, -108.94, 6.05),
                                   flange=(-672.5, -30.99, 342.85, -168.93, -17.66, 77.53)))[0]
t = rp.tcp_from_state(st, cfg)
check("real FR5 packet (lab 2026-10-26): flange check < 1 mm, tilt ~21 deg", t["flange_check_mm"] < 1.0
      and abs(t["tilt_deg"] - 20.8) < 0.2, f"{t['flange_check_mm']:.2f} mm, tilt {t['tilt_deg']:.1f}")
j = list(ready["joints_deg"]); j[4] += 5.0   # wrist 5 deg off straight down
st = rp.PacketParser().feed(packet(joints=j, mode=0))[0]; t = rp.tcp_from_state(st, cfg)
check("j5 + 5 deg: tilt ~5 deg -> tilt_ok false; auto mode -> manual_mode false",
      abs(t["tilt_deg"] - 5.0) < 0.05 and not t["tilt_ok"] and not t["manual_mode"], f"tilt {t['tilt_deg']:.2f}")

# 4 RobotReader vs a fake controller (127.0.0.1)
fc = FakeController(); fc.start()
rcfg = dict(cfg, robot_ip="127.0.0.1", state_port=fc.port, reconnect_period=0.3)
r = rp.RobotReader(rcfg); r.start(); time.sleep(1.5)
s = r.status(); st, age = r.latest()
check("reader: connected, fresh, ~100 Hz, latest joints follow the stream, checksum ok",
      s["connected"] and s["fresh"] and 70 <= s["rate_hz"] <= 130 and st is not None and st.joints_deg[1] == -90
      and s["checksum_ok"] > 50 and s["rejected"] == 0, str(s))
check("reader is read-only: the controller received 0 bytes", fc.received == b"", repr(fc.received[:20]))
port = fc.port; fc.close(); time.sleep(1.2)
s = r.status()
check("controller gone: disconnected, stale (not fresh), error reported", not s["connected"] and not s["fresh"]
      and s["last_error"] != "", str(s))
fc2 = FakeController(port=port); fc2.start(); time.sleep(1.5)
s = r.status()
check("controller back: reconnected, fresh again", s["connected"] and s["fresh"], str(s))
r.stop(); r.join(timeout=2); fc2.close()
check("reader stops (thread ends)", not r.is_alive())

r = rp.RobotReader(dict(cfg, robot_ip="127.0.0.1", state_port=1, connect_timeout=0.2, reconnect_period=0.2)); r.start()
time.sleep(0.6); s = r.status(); r.stop(); r.join(timeout=2)
check("nothing listening: not connected, error shown, no crash", not s["connected"] and "Connection" in s["last_error"],
      s["last_error"])

# 5 config
check("installed config/robot.yaml loads (6 joints, port 20004)", len(cfg["chain"]) == 6 and cfg["state_port"] == 20004)
try:
    import tempfile, yaml
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump({k: v for k, v in cfg.items() if k != "tcp_z"}, f)
    rp.load_config(f.name); check("config without tcp_z rejected", False)
except ValueError as e:
    check("config without tcp_z rejected", "tcp_z" in str(e), str(e))
finally:
    os.remove(f.name)

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
