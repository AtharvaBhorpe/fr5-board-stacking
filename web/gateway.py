"""Local web gateway and simulation-only headless pick controller."""
import argparse
import collections
import json
import mimetypes
import os
from pathlib import Path
import re
import secrets
import signal
import ssl
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, unquote
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from hk_fr5_interfaces.msg import GripperState
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'conversation'))
from speech_input import normalize_voice_command
from policy import block_reason


def model_request(path, body):
    req = urllib.request.Request('http://127.0.0.1:8766' + path,
        json.dumps(body).encode(), {'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(req, timeout=45))


class Workcell(Node):
    def __init__(self):
        super().__init__('web_workcell')
        self.lock = threading.RLock()
        self.vision = {}; self.vision_at = 0.; self.joints = {}; self.joints_at = 0.
        self.description = ''; self.hardware = 'UNKNOWN'; self.gripper = 'UNKNOWN'
        self.images = {}; self.events = collections.deque(maxlen=350); self.sequence = 0
        self.model_health = {}
        threading.Thread(target=self.health_loop, daemon=True).start()
        self.batch = {}; self.proc = None; self.phase = 'idle'; self.task_id = ''; self.waiting = False
        self.input_epoch = 0
        self.detail = ''; self.seen = collections.deque(maxlen=1000); self.stopping = False
        self.state_pub = self.create_publisher(String, '/pick_panel/state', 10)
        self.log_pub = self.create_publisher(String, '/pick_panel/log', 50)
        self.chat_pub = self.create_publisher(String, '/conversation/input', 10)
        self.control_pub = self.create_publisher(String, '/conversation/control', 10)
        self.demo_pub = self.create_publisher(String, '/conversation/demo/control', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.robot, latched)
        self.create_subscription(JointState, '/joint_states', self.joint, qos_profile_sensor_data)
        self.create_subscription(String, '/vision/boards', self.boards, 1)
        self.create_subscription(GripperState, '/gripper/state', self.grip, latched)
        self.create_subscription(String, '/conversation/task', self.task, 10)
        self.create_subscription(String, '/pick_panel/request', self.request, 10)
        self.create_subscription(String, '/conversation/output', self.conversation, 20)
        self.create_subscription(String, '/conversation/batch_state', self.batch_state, 10)
        for topic, key in [('/vision/image', 'overhead'), ('/wrist/wrist_camera/color/image_raw', 'wrist')]:
            self.create_subscription(Image, topic, lambda m, k=key: self.picture(m, k), qos_profile_sensor_data)
        self.create_timer(.2, self.publish_state)

    def health_loop(self):
        while rclpy.ok():
            try:
                with urllib.request.urlopen('http://127.0.0.1:8766/health', timeout=2) as response:
                    health = json.load(response)
            except Exception:
                health = {'ready': False, 'asr_ready': False}
            with self.lock: self.model_health = health
            time.sleep(2)

    def event(self, kind, **data):
        with self.lock:
            self.sequence += 1
            self.events.append(dict(id=self.sequence, kind=kind, time=time.time(), **data))

    def log(self, text):
        self.event('log', text=text)
        self.log_pub.publish(String(data=text))

    def robot(self, msg):
        with self.lock:
            self.description = msg.data
            self.hardware = 'SIMULATION' if 'mock_components' in msg.data else 'REAL FR5'

    def joint(self, msg):
        with self.lock:
            self.joints = dict(zip(msg.name, msg.position)); self.joints_at = time.time()

    def grip(self, msg):
        with self.lock: self.gripper = msg.state

    def boards(self, msg):
        try:
            value = json.loads(msg.data)
            with self.lock: self.vision = value; self.vision_at = time.time()
        except ValueError: pass

    def conversation(self, msg):
        try: self.event('chat', **json.loads(msg.data))
        except ValueError: pass

    def batch_state(self, msg):
        try:
            with self.lock: self.batch = json.loads(msg.data)
        except ValueError: pass

    def picture(self, msg, key):
        if msg.encoding not in ('rgb8', 'bgr8'): return
        now = time.time()
        if now - self.images.get(key, (0, b''))[0] < .18: return
        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        pixels = pixels[:, :msg.width*3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8': pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        if msg.width > 960: pixels = cv2.resize(pixels, (960, round(msg.height*960/msg.width)))
        ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock: self.images[key] = (now, encoded.tobytes())

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def blocked(self, variant):
        if self.hardware != 'SIMULATION': return 'Web execution is currently simulation-only'
        if time.time() - self.joints_at > 2: return 'Robot state is stale'
        if time.time() - self.vision_at > 2: return 'Camera data is stale'
        if self.busy(): return 'A pick is already running'
        return block_reason(self.vision, self.panel(), now=time.time(), variant=variant, allow_replay=True)

    def panel(self):
        return dict(stamp=time.time(), hardware=self.hardware, busy=self.busy(), waiting=self.waiting,
                    task_id=self.task_id, phase='waiting' if self.waiting else self.phase, detail=self.detail)

    def snapshot(self):
        with self.lock:
            now = time.time(); fresh = now-self.vision_at <= 2 and 0 <= now-self.vision.get('stamp', 0) <= 2
            return dict(stamp=now, panel=self.panel(), joints=self.joints,
                robot_fresh=now-self.joints_at < 2, vision=self.vision if fresh else {}, vision_fresh=fresh,
                gripper=self.gripper, batch=self.batch, model=self.model_health, events=list(self.events),
                cameras={k:now-v[0]<2 for k,v in self.images.items()},
                blocked={v:self.blocked(v) for v in ('VARa','VARb','VARc')})

    def publish_state(self):
        with self.lock: payload = self.panel()
        self.state_pub.publish(String(data=json.dumps(payload)))

    def task(self, msg):
        try:
            data = json.loads(msg.data); key = data.get('id'); variant = data.get('variant')
            with self.lock:
                if not key or key in self.seen: return
                self.seen.append(key)
                if self.busy(): self.log('Task rejected: controller busy'); return
                self.task_id = key
                age = time.time()-data.get('stamp', 0)
                reason = 'Expired task' if not 0 <= age <= 2 else self.blocked(variant)
                if reason:
                    self.phase = 'rejected'; self.detail = reason; self.publish_state(); return
                self.phase = 'running'; self.detail = ''; self.waiting = False; self.stopping = False
                self.proc = subprocess.Popen([sys.executable, '-u', str(ROOT/'robot/pick_one.py'),
                    '--place', '--variant', variant, '--speed', '0.3', '--no-wrist-height'],
                    cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    start_new_session=True)
                threading.Thread(target=self.read_process, args=(self.proc,), daemon=True).start()
                self.publish_state()
        except Exception as exc:
            with self.lock: self.phase = 'failed'; self.detail = str(exc)
            self.log(str(exc)); self.publish_state()

    def read_process(self, proc):
        buf = ''
        while True:
            char = proc.stdout.read(1)
            if not char: break
            buf += char.decode('utf-8', errors='replace')
            if char == b'\n': self.log(buf.rstrip()); buf = ''
            elif buf.startswith('>>>') and buf.endswith(': '):
                with self.lock: self.waiting = True
                self.log(buf); buf = ''; self.publish_state()
        if buf: self.log(buf)
        code = proc.wait()
        with self.lock:
            self.waiting = False
            self.phase = 'stopped' if self.stopping else 'completed' if code == 0 else 'failed'
            self.detail = f'Pick process exited {code}'
        self.log(f'[web] pick_one finished, exit code {code}'); self.publish_state()

    def request(self, msg):
        if msg.data == 'stop': self.stop()
        elif msg.data == 'confirm': self.confirm()

    def confirm(self):
        with self.lock:
            if self.hardware != 'SIMULATION' or not self.busy() or not self.waiting:
                raise ValueError('No simulation confirmation is pending')
            self.proc.stdin.write(b'\n'); self.proc.stdin.flush(); self.waiting = False

    def stop(self):
        with self.lock:
            if not self.busy(): return
            self.stopping = True; self.phase = 'stopping'
            os.killpg(self.proc.pid, signal.SIGINT)
        self.publish_state()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--tls-port', type=int, default=8443)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '62' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Web MVP requires isolated simulation domain 62')
    rclpy.init(); node = Workcell()
    # A second task executor would cause duplicate picks.
    deadline = time.monotonic()+2
    while time.monotonic()<deadline: rclpy.spin_once(node, timeout_sec=.1)
    if node.count_publishers('/pick_panel/state') > 1:
        raise RuntimeError('Another controller is running; close RViz/demo before web launch')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    token = secrets.token_urlsafe(32)
    local = ROOT/'web/.local'; local.mkdir(exist_ok=True)
    (local/'session.json').write_text(json.dumps({'token':token}))
    (local/'session.json').chmod(0o600)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def send(self, status, payload, mime='application/json'):
            data = json.dumps(payload).encode() if mime == 'application/json' else payload
            self.send_response(status); self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data))); self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff'); self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass
        def authorized(self):
            return secrets.compare_digest(self.headers.get('X-Workcell-Token',''), token)
        def do_GET(self):
            path = unquote(urlsplit(self.path).path)
            if path == '/api/session':
                host = self.headers.get('Host','').split(':')[0]
                if self.client_address[0] not in ('127.0.0.1','::1') or host not in ('localhost','127.0.0.1'):
                    return self.send(403, {'error':'Pair this device using the code from the laptop'})
                return self.send(200, {'token':token})
            if path == '/workcell-ca.crt':
                return self.send(200,(local/'ca.crt').read_bytes(),'application/x-x509-ca-cert')
            if path.startswith('/api/'):
                if not self.authorized(): return self.send(401, {'error':'Device not paired'})
                if path == '/api/state': return self.send(200,node.snapshot())
                if path == '/api/robot': return self.send(200,node.description.encode(),'application/xml')
                if path == '/api/connection':
                    return self.send(200, json.loads((local/'connection.json').read_text()))
                if path.startswith('/api/image/'):
                    stamp, image = node.images.get(path.rsplit('/',1)[-1], (0,b''))
                    return self.send(200 if time.time()-stamp<2 else 404,image,'image/jpeg')
                if path.startswith('/api/mesh/'):
                    pieces = path[len('/api/mesh/'):].split('/',1)
                    try:
                        if len(pieces)!=2 or pieces[0] not in ('hk_fr5_description','fairino_description'): raise ValueError()
                        base=Path(get_package_share_directory(pieces[0])).resolve()
                        file=(base/pieces[1]).resolve()
                        if '..' in Path(pieces[1]).parts or not pieces[1].startswith('meshes/') or file.suffix.lower() not in ('.stl','.dae','.obj','.png','.jpg'): raise ValueError()
                        return self.send(200,file.read_bytes(),mimetypes.guess_type(file)[0] or 'application/octet-stream')
                    except Exception: return self.send(404,{'error':'Mesh unavailable'})
                return self.send(404,{'error':'Unknown API'})
            file = (ROOT/'web/dist'/('index.html' if path=='/' else path.lstrip('/'))).resolve()
            if not file.is_relative_to((ROOT/'web/dist').resolve()) or not file.is_file():
                return self.send(404,{'error':'Not found'})
            self.send(200,file.read_bytes(),mimetypes.guess_type(file)[0] or 'application/octet-stream')
        def do_POST(self):
            if not self.authorized(): return self.send(401,{'error':'Device not paired'})
            origin=self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'): return self.send(403,{'error':'Cross-origin request denied'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<1_000_000: raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(size)); path=urlsplit(self.path).path
                if path == '/api/chat':
                    text=str(data.get('text','')).strip()
                    if not text or len(text)>1000: raise ValueError('Enter a command or question')
                    node.chat_pub.publish(String(data=text)); result={}
                elif path == '/api/control':
                    action=data.get('action')
                    if action in ('stop','clear'):
                        with node.lock: node.input_epoch += 1
                        node.control_pub.publish(String(data=action))
                    elif action=='confirm': node.confirm()
                    elif action in ('two_b','mixed'):
                        if node.hardware!='SIMULATION' or node.busy() or node.batch.get('phase') in ('checking','executing','verifying'):
                            raise ValueError('Reset is available only in idle simulation')
                        node.demo_pub.publish(String(data=action))
                    else: raise ValueError('Unknown control')
                    result={}
                elif path == '/api/transcribe':
                    with node.lock: epoch = node.input_epoch
                    result=model_request('/transcribe',data)
                    result['normalized']=normalize_voice_command(result['text'])
                    with node.lock:
                        if epoch != node.input_epoch:
                            raise ValueError('Transcription cancelled; no command sent')
                        if result['normalized']: node.chat_pub.publish(String(data=result['normalized']))
                elif path == '/api/speech': result=model_request('/synthesize',{'text':str(data.get('text',''))[:500]})
                else: return self.send(404,{'error':'Unknown API'})
                self.send(200,result)
            except Exception as exc: self.send(400,{'error':str(exc)[:300]})

    http=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    https=ThreadingHTTPServer(('0.0.0.0',args.tls_port),Handler)
    ctx=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER); ctx.load_cert_chain(local/'server.crt',local/'server.key')
    https.socket=ctx.wrap_socket(https.socket,server_side=True)
    threading.Thread(target=https.serve_forever,daemon=True).start()
    print(f'Web ready: http://localhost:{args.port}; HTTPS port {args.tls_port}',flush=True)
    def shutdown(*_):
        node.stop()
        threading.Thread(target=http.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,shutdown); signal.signal(signal.SIGINT,shutdown)
    try: http.serve_forever()
    finally:
        node.stop(); https.shutdown(); rclpy.shutdown()

if __name__ == '__main__': main()
