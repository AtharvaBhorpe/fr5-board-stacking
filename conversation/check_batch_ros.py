"""Isolated ROS scheduler integration: simulated panel/perception and explicit VLM stub.

ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 pixi run python conversation/check_batch_ros.py
This does not launch MoveIt or command hardware, and does not evaluate VLM accuracy.
"""
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from bridge import Conversation

assert os.environ.get('ROS_DOMAIN_ID') == '186', 'Use dedicated test domain 186'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Local test only'
rclpy.init()
driver = Node('batch_fixture')
assert driver.count_publishers('/robot_description') == 0, 'Refusing a robot domain'
# Explicit test doubles; the bridge, threads, topic transport and correlation remain real.
Conversation.http = staticmethod(lambda path, body, timeout=20:
                                {'target_visible': True, 'conflict': False, 'answer': 'fixture'})
Conversation.say = lambda self, text: self.emit('assistant', text)
bridge = Conversation()
executor = SingleThreadedExecutor()
executor.add_node(driver); executor.add_node(bridge)
state_pub = driver.create_publisher(String, '/pick_panel/state', 10)
scene_pub = driver.create_publisher(String, '/vision/boards', 10)
image_pub = driver.create_publisher(Image, '/vision/image', 10)
chat_pub = driver.create_publisher(String, '/conversation/input', 10)
control_pub = driver.create_publisher(String, '/conversation/control', 10)
boards = []
state = dict(hardware='SIMULATION', busy=False, phase='idle', task_id='')
tasks = []
active = None
mode = 'success'


def task(message):
    global active
    request = json.loads(message.data)
    tasks.append(request)
    assert not state['busy'], 'overlapping dispatch'
    state.update(task_id=request['id'], busy=True, phase='running')
    active = time.monotonic()+.3


def tick():
    global active
    if active and mode == 'cancel':
        control_pub.publish(String(data='stop'))
    if active and time.monotonic() >= active:
        state.update(busy=False, phase='failed' if mode == 'failure' else 'completed')
        if mode == 'success':
            boards.pop(0)
        active = None
    now = time.time()
    state['stamp'] = now
    state_pub.publish(String(data=json.dumps(state)))
    scene_pub.publish(String(data=json.dumps(dict(stamp=now, live=True, calibrated=True,
        boards=boards, destinations=[{'variant': 'VARb'}]))))
    image = Image(height=8, width=8, encoding='rgb8', step=24, data=bytes(192))
    image.header.stamp = driver.get_clock().now().to_msg()
    image_pub.publish(image)


driver.create_subscription(String, '/conversation/task', task, 10)
def stop(message):
    global active
    if message.data == 'stop':
        state.update(busy=False, phase='stopped')
        active = None
driver.create_subscription(String, '/pick_panel/request', stop, 10)
driver.create_timer(.1, tick)


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.05)


results = []
try:
    for mode in ('success', 'failure', 'unchanged', 'cancel'):
        boards[:] = [dict(variant='VARb', zone='pick', x=x, y=0., inside_calibration=True,
                          size_ok=True) for x in (.1, .3)]
        state.update(busy=False, phase='idle', task_id='')
        tasks.clear(); bridge.batch = None
        spin(1)
        chat_pub.publish(String(data='pick two variant B boards'))
        deadline = time.monotonic()+18
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if bridge.batch and bridge.batch.phase in ('completed', 'failed', 'stopped') and not bridge.batch_worker_active:
                break
        assert bridge.batch, 'request not parsed'
        result = dict(scenario=mode, dispatches=len(tasks), **bridge.batch.snapshot())
        if mode == 'success':
            assert result['phase'] == 'completed' and result['completed'] == 2 and len(tasks) == 2, result
        elif mode == 'cancel':
            assert result['phase'] == 'stopped' and len(tasks) == 1, result
        else:
            assert result['phase'] == 'failed' and len(tasks) == 1, result
        results.append(result)
    path = Path(__file__).parent / 'results/batch-ros.json'
    path.write_text(json.dumps({'passed': True, 'scope': 'ROS transport with simulated panel and VLM stub',
                                'results': results}, indent=2)+'\n')
    print(path.read_text())
finally:
    bridge.epoch += 1
    executor.shutdown()
    driver.destroy_node(); bridge.destroy_node()
    rclpy.shutdown()
