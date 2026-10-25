"""Add static collision boxes from a yaml file to MoveIt's planning scene (one-shot).

    ros2 run hk_fr5_motion obstacle_loader
    ros2 run hk_fr5_motion obstacle_loader --ros-args -p obstacles_file:=<path> -p wait_timeout:=30.0

Waits for move_group's /apply_planning_scene service, adds all boxes in one call, logs them, exits.
A box with an existing name replaces it. Removing a box from the yaml does NOT remove it from a
running scene: restart the bringup (decision O-4).
"""
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

FRAME = "base_link"


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _vector3(box, key, where):
    value = box.get(key)
    if not (isinstance(value, list) and len(value) == 3 and all(_number(v) for v in value)):
        raise ValueError(f"{where}: '{key}' must be a list of 3 numbers, got {value!r}")
    return [float(v) for v in value]


def load_boxes(path):
    """Return [(name, size, position, yaw)] or raise ValueError with a clear message."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    boxes = data.get("boxes") if isinstance(data, dict) else None
    if not isinstance(boxes, list) or not boxes:
        raise ValueError(f"{path}: 'boxes' must be a non-empty list")
    result, names = [], set()
    for i, box in enumerate(boxes):
        where = f"{path}: boxes[{i}]"
        if not isinstance(box, dict):
            raise ValueError(f"{where} must be a mapping")
        name = box.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{where}: 'name' must be a non-empty string")
        if name in names:
            raise ValueError(f"{where}: duplicate name '{name}'")
        names.add(name)
        where = f"{where} ({name})"
        size = _vector3(box, "size", where)
        if any(s <= 0.0 for s in size):
            raise ValueError(f"{where}: all sizes must be > 0, got {size}")
        position = _vector3(box, "position", where)
        yaw = box.get("yaw", 0.0)
        if not _number(yaw):
            raise ValueError(f"{where}: 'yaw' must be a number, got {yaw!r}")
        result.append((name, size, position, float(yaw)))
    return result


def collision_object(name, size, position, yaw):
    obj = CollisionObject(id=name, operation=CollisionObject.ADD)
    obj.header.frame_id = FRAME
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = position
    obj.pose.orientation.z = math.sin(yaw / 2.0)
    obj.pose.orientation.w = math.cos(yaw / 2.0)
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
    obj.primitive_poses = [Pose()]  # box centred on the object pose (identity)
    return obj


def main():
    rclpy.init()
    node = rclpy.create_node("obstacle_loader")
    log = node.get_logger()
    default_file = os.path.join(get_package_share_directory("hk_fr5_motion"), "config", "obstacles.yaml")
    path = node.declare_parameter("obstacles_file", default_file).value
    timeout = float(node.declare_parameter("wait_timeout", 30.0).value)

    ok = False
    try:
        boxes = load_boxes(path)
        client = node.create_client(ApplyPlanningScene, "apply_planning_scene")
        log.info(f"waiting up to {timeout:.0f} s for /apply_planning_scene (move_group) ...")
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/apply_planning_scene not available (is move_group running?)")
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [collision_object(*b) for b in boxes]
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError("no answer from /apply_planning_scene")
        if not future.result().success:
            raise RuntimeError("move_group rejected the planning scene update")
        for name, size, position, yaw in boxes:
            log.info(f"added '{name}': size {size} m, centre {position} m, yaw {yaw:.3f} rad")
        log.info(f"{len(boxes)} obstacles added to the planning scene from {path}")
        ok = True
    except (OSError, ValueError, yaml.YAMLError, RuntimeError) as e:
        log.error(f"obstacles NOT added: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
