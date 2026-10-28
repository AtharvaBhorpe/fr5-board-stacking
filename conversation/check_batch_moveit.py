"""Drive actual Gemma, vision and MoveIt mock hardware; only domain 62 is permitted."""
import argparse
import base64
import json
import os
from pathlib import Path
import time
import urllib.request

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from speech_input import normalize_voice_command
from scheduler import batch_request

parser = argparse.ArgumentParser()
parser.add_argument('scenario', choices=['counted', 'mixed', 'cancel', 'unchanged', 'missing'])
parser.add_argument('--voice-wav', type=Path, help='Use actual local ASR for a synthetic two-board command')
parser.add_argument('--output', type=Path, help='Save a separate result without overwriting earlier runs')
args = parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '62' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init(); node = rclpy.create_node('batch_moveit_check')
data, logs, tasks, answers = {}, [], [], []
def recv(key, msg):
    data[key] = json.loads(msg.data)
node.create_subscription(String, '/pick_panel/state', lambda m: recv('panel', m), 10)
node.create_subscription(String, '/vision/boards', lambda m: recv('vision', m), 10)
node.create_subscription(String, '/conversation/batch_state', lambda m: recv('batch', m), 10)
node.create_subscription(String, '/pick_panel/log', lambda m: logs.append(m.data), 50)
node.create_subscription(String, '/conversation/task', lambda m: tasks.append(json.loads(m.data)), 10)
node.create_subscription(String, '/conversation/output', lambda m: answers.append(json.loads(m.data)), 20)
node.create_subscription(Image, '/vision/image', lambda m: data.update(
    image_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9), qos_profile_sensor_data)
world = node.create_publisher(String, '/conversation/demo/control', 10)
chat = node.create_publisher(String, '/conversation/input', 10)

def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)

spin(3)
assert data.get('panel', {}).get('hardware') == 'SIMULATION', data
assert not data['panel']['busy'], data
assert node.count_publishers('/robot_description') == 1
layout = 'mixed' if args.scenario == 'mixed' else 'two_b'
world.publish(String(data=layout)); spin(3)
assert len(data['vision']['boards']) == (3 if layout == 'mixed' else 2), data['vision']
deadline = time.monotonic()+10
while not 0 <= time.time()-data.get('image_stamp', 0) < .5 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
assert 0 <= time.time()-data.get('image_stamp', 0) < .5, 'No fresh image; test not started'
if args.scenario == 'unchanged':
    world.publish(String(data='freeze')); spin(.5)
logs.clear(); tasks.clear(); answers.clear()
old_id = data.get('batch', {}).get('id')
command = {'mixed': 'sort all visible boards', 'missing': 'pick three variant B boards'}.get(
    args.scenario, 'pick two variant B boards')
voice = None
if args.voice_wav:
    assert args.scenario == 'counted', 'Voice fixture currently checks a two-board request'
    req = urllib.request.Request('http://127.0.0.1:8766/transcribe',
        json.dumps({'wav': base64.b64encode(args.voice_wav.read_bytes()).decode()}).encode(),
        {'Content-Type': 'application/json'})
    voice = json.load(urllib.request.urlopen(req, timeout=30))
    command = normalize_voice_command(voice['text'])
    assert batch_request(command) == ('count', ('VARb', 2)), voice
chat.publish(String(data=command)); began = time.monotonic()
cancelled = False; last = None
while time.monotonic()-began < 360:
    rclpy.spin_once(node, timeout_sec=.1)
    assert data['panel']['hardware'] == 'SIMULATION'
    if args.scenario == 'cancel' and not cancelled and sum('action: accepted' in x for x in logs) >= 2:
        chat.publish(String(data='stop')); cancelled = True
    batch = data.get('batch', {})
    if batch != last:
        print(json.dumps(batch), flush=True); last = dict(batch)
    if args.scenario == 'missing' and any(a.get('role') == 'assistant' for a in answers):
        break
    if batch.get('id') != old_id and batch.get('phase') in ('completed', 'failed', 'stopped'):
        if not data['panel'].get('busy'):
            spin(1); break
result = dict(scenario=args.scenario, seconds=round(time.monotonic()-began, 2),
              batch=data.get('batch'), panel=data.get('panel'), tasks=tasks, logs=logs, answers=answers,
              voice=voice)
if args.scenario in ('counted', 'mixed'):
    passed = result['batch']['phase'] == 'completed' and len(tasks) == (3 if args.scenario == 'mixed' else 2)
elif args.scenario == 'missing':
    passed = not tasks and any('Only 2' in a.get('text', '') for a in answers)
elif args.scenario == 'cancel':
    passed = cancelled and result['batch']['phase'] == 'stopped' and len(tasks) == 1 and any('cancel:' in x for x in logs)
else:
    passed = result['batch']['phase'] == 'failed' and len(tasks) == 1 and 'removal' in result['batch']['reason']
result['passed'] = passed
path = Path(__file__).parent / 'results/batch-moveit' / (('voice-' if args.voice_wav else '')+args.scenario+'.json')
path = args.output or path
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({'passed': passed, 'evidence': str(path), 'seconds': result['seconds']}))
node.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if passed else 1)
