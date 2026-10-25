import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from fake_moveit_check import RobotInterface


JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True)
    parser.add_argument(
        "--output", type=Path,
        default=Path("config/waypoints_fake.json"),
    )
    args = parser.parse_args()

    if args.output.exists():
        data = json.loads(args.output.read_text())
        if (
            data.get("schema_version") != 1
            or data.get("hardware_mode") != "fake"
            or data.get("units") != "radians"
            or not isinstance(data.get("waypoints"), dict)
        ):
            raise RuntimeError("Existing file is not a supported fake waypoint file")
    else:
        data = {
            "schema_version": 1,
            "hardware_mode": "fake",
            "units": "radians",
            "waypoints": {},
        }

    if args.name in data["waypoints"]:
        raise RuntimeError(
            f"Waypoint {args.name!r} already exists; choose a new name."
        )

    rclpy.init()
    robot = RobotInterface()
    captured = {}
    status = 0

    def on_state(message):
        if len(message.name) != len(message.position):
            return
        if len(set(message.name)) != len(message.name):
            return

        positions = dict(zip(message.name, message.position))
        if not all(
            name in positions and math.isfinite(positions[name])
            for name in JOINTS
        ):
            return

        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        age_s = (robot.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not 0 <= age_s <= 1.0:
            return

        captured.update({
            "joint_positions": {
                name: float(positions[name]) for name in JOINTS
            },
            "state_stamp_ns": stamp_ns,
        })

    try:
        robot.require_fake_hardware()
        subscription = robot.create_subscription(
            JointState, "/joint_states", on_state, qos_profile_sensor_data
        )

        deadline = time.monotonic() + 10.0
        while not captured and time.monotonic() < deadline:
            rclpy.spin_once(robot, timeout_sec=0.2)

        if not captured:
            raise RuntimeError(
                "No complete, finite joint state less than one second old."
            )

        data["waypoints"][args.name] = captured
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(data, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(f"Recorded fake waypoint: {args.name}")
        print(json.dumps(captured, indent=2))
        print(f"Saved: {args.output.resolve()}")
    except (Exception, KeyboardInterrupt) as error:
        status = 1
        print(f"FAILED: {str(error) or type(error).__name__}")
    finally:
        robot.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(status)


if __name__ == "__main__":
    main()
