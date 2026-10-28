"""Image-based simulated workcell; changes replay pixels after MOCK controller completion.

This is a test scene, not live perception or a physics simulation. The regular vision node
still detects and measures each rendered image; no detections or VLM approvals are injected.
"""
import json
import os
from pathlib import Path
import re
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'vision1'))
import table
from detect import detect

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = Path('/tmp/robot-batch-demo/scene.png')
LAYOUTS = {
    'two_b': [('VARb', -.21, -.40, 0), ('VARb', -.43, -.40, 0)],
    'mixed': [('VARa', -.42, -.365, 90), ('VARb', -.18, -.40, 0), ('VARc', -.35, -.59, 90)],
}


def pixels(calib, xy):
    q = np.linalg.inv(calib['homography']) @ np.vstack([np.asarray(xy).T, np.ones(len(xy))])
    corrected = (q[:2] / q[2]).T
    centre = np.asarray(calib['image_size']) / 2
    out = corrected.copy()
    for _ in range(12):
        factor = 1 + calib['lens_k'] * np.sum(((out-centre)/max(calib['image_size']))**2, axis=1, keepdims=True)
        out = centre + (corrected-centre)/factor
    return out.astype(np.float32)


class Renderer:
    def __init__(self):
        self.calib = table.load()
        self.original = cv2.imread(str(ROOT / 'samples/composite_vara_dest.png'))
        assert self.original is not None
        found = detect(self.original, use_model=True)
        self.sources = {b['variant']: table.add_metric(self.calib, b) for b in found}
        assert set(self.sources) == {'VARa', 'VARb', 'VARc'}
        mask = np.zeros(self.original.shape[:2], np.uint8)
        for b in self.sources.values():
            cv2.fillPoly(mask, [np.int32(b['corners'])], 255)
        mask = cv2.dilate(mask, np.ones((17, 17), np.uint8))
        self.background = cv2.inpaint(self.original, mask, 7, cv2.INPAINT_TELEA)

    def render(self, items):
        image = self.background.copy()
        for variant, x, y, angle in items:
            b = self.sources[variant]
            xy = np.asarray(b['corners_xy'])
            theta = np.deg2rad(angle)
            rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            target = pixels(self.calib, (xy-xy.mean(0)) @ rot.T + [x, y])
            transform = cv2.getPerspectiveTransform(np.float32(b['corners']), target)
            mask = np.zeros(image.shape[:2], np.uint8)
            cv2.fillPoly(mask, [np.int32(b['corners'])], 255)
            warped = cv2.warpPerspective(self.original, transform, (image.shape[1], image.shape[0]))
            mask = cv2.warpPerspective(mask, transform, (image.shape[1], image.shape[0]))
            image[mask > 128] = warped[mask > 128]
        cv2.putText(image, 'SIMULATED BOARD SCENE - NOT LIVE', (20, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 180, 255), 2)
        return image


class World(Node):
    def __init__(self):
        super().__init__('conversation_demo_world')
        self.renderer = Renderer()
        self.state, self.batch = {}, {}
        self.selected = None
        self.completed = set()
        self.frozen = False
        self.items = list(LAYOUTS['two_b'])
        self.out = self.create_publisher(String, '/conversation/demo/state', 10)
        self.create_subscription(String, '/pick_panel/state', self.panel, 10)
        self.create_subscription(String, '/pick_panel/log', self.log, 50)
        self.create_subscription(String, '/conversation/batch_state', self.batch_state, 10)
        self.create_subscription(String, '/conversation/demo/control', self.control, 10)
        self.write()

    def batch_state(self, msg):
        self.batch = json.loads(msg.data)

    def write(self):
        OUTPUT.parent.mkdir(exist_ok=True)
        temp = OUTPUT.with_name('next.png')
        assert cv2.imwrite(str(temp), self.renderer.render(self.items))
        temp.replace(OUTPUT)
        self.out.publish(String(data=json.dumps({'items': self.items, 'frozen': self.frozen})))
        self.get_logger().info(f'Simulated scene: {len(self.items)} source boards; frozen={self.frozen}')

    def panel(self, msg):
        self.state = json.loads(msg.data)
        if self.state.get('hardware') not in ('', 'UNKNOWN', 'SIMULATION'):
            raise RuntimeError('Demo world refuses non-simulation hardware')
        task = self.state.get('task_id')
        if self.state.get('phase') == 'completed' and task and task not in self.completed:
            self.completed.add(task)
            if self.selected is not None and not self.frozen:
                self.items.pop(self.selected)
                self.write()
            self.selected = None

    def log(self, msg):
        m = re.match(r'board (VAR[abc]) at x=([\d.-]+) y=([\d.-]+)', msg.data)
        if m and self.state.get('hardware') == 'SIMULATION':
            candidates = [(np.hypot(b[1]-float(m[2]), b[2]-float(m[3])), i)
                          for i, b in enumerate(self.items) if b[0] == m[1]]
            if candidates:
                distance, index = min(candidates)
                self.selected = index if distance < .03 else None

    def control(self, msg):
        if self.state.get('hardware') != 'SIMULATION' or self.state.get('busy'):
            self.get_logger().warn('Scene changes require an idle SIMULATION controller')
            return
        if self.batch.get('phase') in ('checking', 'executing', 'verifying'):
            self.get_logger().warn('Cancel the batch before changing the scene')
            return
        if msg.data in LAYOUTS:
            self.items = list(LAYOUTS[msg.data]); self.selected = None; self.frozen = False
            self.write()
        elif msg.data in ('freeze', 'resume'):
            self.frozen = msg.data == 'freeze'
            self.write()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '62', 'Demo world requires isolated domain 62'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = World()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
