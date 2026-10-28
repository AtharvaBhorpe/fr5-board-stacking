#!/usr/bin/env python3
"""Joint sliders: one slider per movable URDF joint, publishes /joint_states.

Small PyQt6 replacement for joint_state_publisher_gui, whose 2.4.0 release
breaks on RoboStack's PyQt6 (see decision-log D-i). For URDF checks only.
"""
import math
import signal
import sys
import xml.etree.ElementTree as ET

import rclpy
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication, QGridLayout, QLabel, QPushButton, QSlider, QWidget
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SCALE = 1000        # slider counts in milliradians (mm for prismatic): 0 is exact
PUBLISH_MS = 100    # /joint_states at 10 Hz
SPIN_MS = 10        # ROS callbacks handled from the Qt loop (single thread)


def movable_joints(urdf_xml):
    """[(name, lower, upper)] for revolute/prismatic/continuous joints, in URDF order."""
    joints = []
    for j in ET.fromstring(urdf_xml).iter("joint"):
        jtype = j.get("type")
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        lim = j.find("limit")
        if jtype == "continuous" or lim is None:
            lo, hi = -math.pi, math.pi
        else:
            lo, hi = float(lim.get("lower", 0.0)), float(lim.get("upper", 0.0))
        joints.append((j.get("name"), lo, hi))
    return joints


class JointSliders(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.joints = []    # [(name, lo, hi, slider, value_label)]
        self.setWindowTitle("Joint sliders")
        self.grid = QGridLayout(self)
        self.status = QLabel("Waiting for /robot_description ...")
        self.grid.addWidget(self.status, 0, 0, 1, 3)

        self.pub = node.create_publisher(JointState, "joint_states", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "robot_description", self.on_description, latched)

        self.spin_timer = QTimer(self, timeout=lambda: rclpy.spin_once(node, timeout_sec=0))
        self.spin_timer.start(SPIN_MS)
        self.pub_timer = QTimer(self, timeout=self.publish)
        self.pub_timer.start(PUBLISH_MS)

    def on_description(self, msg):
        if self.joints:
            return  # build once
        for row, (name, lo, hi) in enumerate(movable_joints(msg.data), start=1):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(math.ceil(lo * SCALE), math.floor(hi * SCALE))  # round inward: never past a limit
            slider.setMinimumWidth(300)
            value = QLabel()
            self.grid.addWidget(QLabel(name), row, 0)
            self.grid.addWidget(slider, row, 1)
            self.grid.addWidget(value, row, 2)
            self.joints.append((name, lo, hi, slider, value))
            slider.valueChanged.connect(self.update_labels)
        zero = QPushButton("Zero all")
        zero.clicked.connect(self.zero_all)
        self.grid.addWidget(zero, len(self.joints) + 1, 0, 1, 3)
        self.status.setText(f"{len(self.joints)} movable joints")
        self.zero_all()

    def position(self, slider):
        return slider.value() / SCALE

    def zero_all(self):
        for *_, slider, _ in self.joints:
            slider.setValue(0)  # Qt clamps into the joint range if 0 is outside it
        self.update_labels()

    def update_labels(self):
        for *_, slider, value in self.joints:
            q = self.position(slider)
            value.setText(f"{q:+.3f} rad  {math.degrees(q):+7.1f} deg")

    def publish(self):
        if not self.joints:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [j[0] for j in self.joints]
        msg.position = [self.position(j[3]) for j in self.joints]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = rclpy.create_node("joint_sliders")
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl+C in the terminal closes the window
    window = JointSliders(node)
    window.show()
    code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
