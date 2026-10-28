"""Demo page for the vacuum-failure recovery (TM-47): one big button 'Simulate vacuum failure' + 'Now', and
'Pause' / 'Resume' for the main controller (TM-48: 'pause' / 'resume' through the bridge, as `tc pause` / `tc resume`).

    pixi run python3 tools/vacuum_demo_ui.py [--port 8765] [--lan] [--delay 1.5] [--bridge DIR]
    (started by tools/trial_up.sh as window 8 'demo-ui'; open http://localhost:8765)

Simulate vacuum failure = ARM: at the next carry (skill place, phase HOVER, gripper HOLDING) wait DELAY s (the board
falls clear of its stack), then 'inject vacuum_loss' into the agent bridge. Now = inject at once (the tool refuses
unless the gripper is HOLDING). No ROS node: reads <bridge>/state.txt + events.txt, writes <bridge>/cmd.
--lan: listen on all interfaces (a phone on the same network); default this PC only.
REAL robot: the injection switches the gripper -> only with a person at the e-stop.
"""
import argparse
import errno
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LIVE_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
                         "live_logs")              # <worktrees>/../live_logs (as tools/paths.sh)
A = {"armed": False, "carry_t": None, "msg": "ready", "fired": 0}
LOCK = threading.Lock()


def parse_state(path):
    """(dict, age s) from the bridge's state.txt; empty dict if missing."""
    try:
        txt = open(path).read()
        age = time.time() - os.path.getmtime(path)
    except OSError:
        return {}, None
    m = re.search(r"\| skill (\S+)(.*?) phase (\S+) \|", txt)
    g = re.search(r"gripper (\S+)", txt)
    h = re.search(r"held '([^']*)'", txt)
    c = re.search(r"CTRL (\S+)", txt)
    paused = bool(re.search(r"CTRL [^|]*PAUSED", txt))
    return {"ctrl": c.group(1) if c else "?", "paused": paused, "skill": m.group(1) if m else "-", "detail": (m.group(2).strip() if m else ""),
            "phase": m.group(3) if m else "-", "gripper": g.group(1) if g else "?", "held": h.group(1) if h else ""}, age


def send(fifo, cmd):
    """One command line into the bridge pipe; never blocks (no reader = bridge not running)."""
    try:
        fd = os.open(fifo, os.O_WRONLY | os.O_NONBLOCK)
    except OSError as e:
        return f"bridge not running ({'no reader on the pipe' if e.errno == errno.ENXIO else e})"
    try:
        os.write(fd, (cmd + "\n").encode())
    finally:
        os.close(fd)
    return ""


def watcher(bridge, delay):
    """ARM: fire `delay` s into the next carry (still carrying + holding then)."""
    state = os.path.join(bridge, "state.txt")
    while True:
        time.sleep(0.05)
        with LOCK:
            if not A["armed"]:
                A["carry_t"] = None
                continue
        s, _ = parse_state(state)
        carrying = s.get("skill") == "place" and s.get("phase") == "HOVER" and s.get("gripper") == "HOLDING"
        with LOCK:
            if not carrying:
                A["carry_t"] = None
                continue
            if A["carry_t"] is None:
                A["carry_t"] = time.time()
                A["msg"] = f"carry started - vacuum failure in {delay:.1f} s"
                continue
            if time.time() - A["carry_t"] < delay:
                continue
            err = send(os.path.join(bridge, "cmd"), "inject vacuum_loss")
            A["armed"], A["carry_t"] = False, None
            A["msg"] = err or f"vacuum failure injected at {time.strftime('%H:%M:%S')} ({delay:.1f} s into the carry)"
            A["fired"] += 0 if err else 1


PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Vacuum Failure Demo</title><style>
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;display:flex;flex-direction:column;
 align-items:center;justify-content:center;min-height:100vh;gap:22px;padding:16px;box-sizing:border-box}
h1{font-weight:600;font-size:clamp(20px,4vw,34px);margin:0;text-align:center}
#arm{font-size:clamp(26px,6vw,56px);padding:.6em 1.2em;border:0;border-radius:18px;background:#d33;color:#fff;
 cursor:pointer;box-shadow:0 6px 0 #811;max-width:100%}
#arm.armed{background:#e90;box-shadow:0 6px 0 #a60}
#arm:active{transform:translateY(4px);box-shadow:0 2px 0 #811}
.row{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
.small{font-size:18px;padding:.5em 1em;border:1px solid #666;border-radius:10px;background:#222;color:#eee;cursor:pointer}
#st{font-size:clamp(15px,2.4vw,22px);text-align:center;line-height:1.5}
#msg{font-size:clamp(16px,2.6vw,24px);color:#fc6;text-align:center}
#ev{font-family:ui-monospace,monospace;font-size:13px;color:#aaa;white-space:pre-wrap;max-width:900px;width:100%}
.warn{color:#f88;font-size:14px}
#pause{border-color:#fc6}#resume{border-color:#6c6}.on{background:#553 !important}
</style></head><body>
<h1>FR5 board stacking - vacuum failure recovery</h1>
<button id="arm" onclick="post(armed?'/disarm':'/arm')">Simulate vacuum failure</button>
<div class="row"><button class="small" onclick="post('/now')">Now (while holding)</button>
<button class="small" id="pause" onclick="post('/pause')">Pause</button>
<button class="small" id="resume" onclick="post('/resume')">Resume</button></div>
<div id="msg"></div><div id="st"></div><div id="ev"></div>
<div class="warn">Real robot: the gripper is switched - a person at the e-stop.</div>
<script>
let armed=false;
async function post(p){await fetch(p,{method:'POST'});poll();}
async function poll(){try{const r=await (await fetch('/status')).json();armed=r.armed;
 const b=document.getElementById('arm');b.classList.toggle('armed',armed);
 b.textContent=armed?'Armed - next carry (tap to cancel)':'Simulate vacuum failure';
 document.getElementById('msg').textContent=r.msg;
 const s=r.state;document.getElementById('pause').classList.toggle('on',!!s.paused);
 document.getElementById('st').innerHTML=s.ctrl?('controller <b>'+s.ctrl+(s.paused?' (PAUSED)':'')+'</b> | skill <b>'+s.skill+'</b> '+
  s.detail+' | phase <b>'+s.phase+'</b><br>gripper <b>'+s.gripper+'</b>'+(s.held?' holding <b>'+s.held+'</b>':'')+
  (r.age!==null&&r.age>10?' | state '+Math.round(r.age)+' s old (bridge running?)':'')):'no bridge state (is trial_up running?)';
 document.getElementById('ev').textContent=r.events.join('\\n');}catch(e){document.getElementById('msg').textContent='page server not reachable';}}
setInterval(poll,500);poll();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=int(os.environ.get("HK_DEMO_UI_PORT", "8765")))
    ap.add_argument("--lan", action="store_true", help="listen on all interfaces (default: this PC only)")
    ap.add_argument("--delay", type=float, default=1.5, help="s into the carry before the injection (armed mode)")
    ap.add_argument("--bridge", default=None, help="agent bridge dir (default $HK_LIVE_DIR/bridge)")
    a = ap.parse_args()
    bridge = a.bridge or os.path.join(os.environ.get("HK_LIVE_DIR") or os.environ.get("HK_LIVE_ROOT") or LIVE_ROOT, "bridge")
    fifo, state, events = (os.path.join(bridge, f) for f in ("cmd", "state.txt", "events.txt"))

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype):
            data = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                return self._send(200, PAGE, "text/html; charset=utf-8")
            if self.path == "/status":
                s, age = parse_state(state)
                try:
                    ev = [l.rstrip() for l in open(events).readlines()[-200:]
                          if re.search(r"inject|LOST|lost|picked|placed|DONE", l)][-6:]
                except OSError:
                    ev = []
                with LOCK:
                    out = {"armed": A["armed"], "msg": A["msg"], "fired": A["fired"], "state": s,
                           "age": None if age is None else round(age, 1), "events": ev}
                return self._send(200, json.dumps(out), "application/json")
            self._send(404, "not found", "text/plain")

        def do_POST(self):
            with LOCK:
                if self.path == "/arm":
                    A["armed"], A["carry_t"] = True, None
                    A["msg"] = f"armed: vacuum failure {a.delay:.1f} s into the next carry"
                elif self.path == "/disarm":
                    A["armed"], A["carry_t"], A["msg"] = False, None, "cancelled"
                elif self.path in ("/pause", "/resume"):          # TM-48: whole controller (finish the skill /
                    cmd = self.path[1:]                               # place a held board first, then hold)
                    err = send(fifo, cmd)
                    A["msg"] = err or f"{cmd} sent at {time.strftime('%H:%M:%S')}"
                elif self.path == "/now":
                    err = send(fifo, "inject vacuum_loss")
                    A["msg"] = err or f"vacuum failure sent at {time.strftime('%H:%M:%S')} (refused unless holding)"
                    A["fired"] += 0 if err else 1
                else:
                    return self._send(404, "not found", "text/plain")
            self._send(200, "ok", "text/plain")

    threading.Thread(target=watcher, args=(bridge, a.delay), daemon=True).start()
    host = "0.0.0.0" if a.lan else "127.0.0.1"
    srv = ThreadingHTTPServer((host, a.port), H)
    print(f"vacuum demo page: http://localhost:{a.port}  (bridge {bridge}, delay {a.delay:.1f} s"
          f"{', LAN' if a.lan else ''})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
