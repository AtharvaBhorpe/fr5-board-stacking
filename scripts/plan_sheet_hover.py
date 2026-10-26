"""Plan a collision-checked horizontal hover only; never execute a trajectory."""
import argparse
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--variant', required=True, choices=['VARa', 'VARb', 'VARc'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ompl', action='store_true')
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    candidates = [c for c in report['candidates']
                  if c['metric_classification']['variant'] == args.variant
                  and c['metric_classification']['status'] == 'matched']
    if report['frame_id'] != 'base_link' or len(candidates) != 1:
        raise RuntimeError('Need exactly one matching candidate in base_link')
    centre = candidates[0]['diagnostic_centre_m']
    if len(centre) != 3 or not all(math.isfinite(v) for v in centre):
        raise RuntimeError('Invalid candidate centre')
    rclpy.init()
    node = Node('sheet_hover_plan_only')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    states = []
    subscription = node.create_subscription(
        JointState, '/joint_states', lambda msg: states.append(msg),
        qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 10
        transform = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if states and buffer.can_transform('base_link', 'tcp', rclpy.time.Time()):
                transform = buffer.lookup_transform('base_link', 'tcp', rclpy.time.Time())
                break
        if transform is None:
            raise RuntimeError('No current TCP transform / joint state')
        now = node.get_clock().now().nanoseconds
        for stamp in (transform.header.stamp, states[-1].header.stamp):
            age = (now - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            if not -0.1 <= age <= 1:
                raise RuntimeError(f'Stale robot feedback: age={age:.3f}s')
        target = Pose()
        target.position.x, target.position.y = centre[:2]
        target.position.z = transform.transform.translation.z
        target.orientation = copy.deepcopy(transform.transform.rotation)
        if target.position.z - centre[2] < 0.2:
            raise RuntimeError('Current height leaves less than 200 mm above sheet')
        client = node.create_client(GetMotionPlan if args.ompl else GetCartesianPath,
                                   '/plan_kinematic_path' if args.ompl else '/compute_cartesian_path')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Cartesian planning service unavailable')
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'fairino5_v6_group'
        req.link_name = 'tcp'
        req.start_state.joint_state = states[-1]
        req.start_state.is_diff = True
        req.waypoints = [target]
        req.max_step = 0.005
        # Relative-to-average thresholds can reject smooth, nonuniform motion.
        # Use an absolute step bound and validate every returned joint step below.
        req.jump_threshold = 0.0
        req.revolute_jump_threshold = 0.15
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        if args.ompl:
            cartesian = req
            req = GetMotionPlan.Request()
            motion = req.motion_plan_request
            motion.group_name = cartesian.group_name
            motion.start_state = cartesian.start_state
            motion.pipeline_id = 'ompl'
            motion.allowed_planning_time = 10.0
            motion.num_planning_attempts = 3
            motion.max_velocity_scaling_factor = 0.1
            motion.max_acceleration_scaling_factor = 0.1
            position = PositionConstraint()
            position.header.frame_id = 'base_link'
            position.link_name = 'tcp'
            position.weight = 1.0
            position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
            position.constraint_region.primitive_poses = [target]
            orientation = OrientationConstraint()
            orientation.header.frame_id = 'base_link'
            orientation.link_name = 'tcp'
            orientation.orientation = target.orientation
            orientation.absolute_x_axis_tolerance = 0.03
            orientation.absolute_y_axis_tolerance = 0.03
            orientation.absolute_z_axis_tolerance = 0.03
            orientation.weight = 1.0
            motion.goal_constraints = [Constraints(position_constraints=[position], orientation_constraints=[orientation])]
            height = copy.deepcopy(position)
            height.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0,3.0,1.0])]
            region_pose = Pose()
            region_pose.position.z = target.position.z + 0.48
            region_pose.orientation.w = 1.0
            height.constraint_region.primitive_poses = [region_pose]
            motion.path_constraints = Constraints(position_constraints=[height], orientation_constraints=[orientation])
        print('PLAN ONLY target:', message_to_ordereddict(target), flush=True)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done():
            raise RuntimeError('Planning timed out; no motion was requested')
        result = future.result()
        if args.ompl:
            response = result.motion_plan_response
            normalized = GetCartesianPath.Response()
            normalized.start_state = cartesian.start_state
            normalized.solution = response.trajectory
            normalized.error_code = response.error_code
            normalized.fraction = 1.0 if response.error_code.val == MoveItErrorCodes.SUCCESS else 0.0
            result = normalized
        else:
            result.start_state = req.start_state
        print(f'fraction={result.fraction:.6f}, error={result.error_code.val}', flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            'purpose': 'plan_only_horizontal_hover',
            'source_report': str(args.report.resolve()),
            'variant': args.variant,
            'created_unix': time.time(),
            'target': message_to_ordereddict(target),
            'response': message_to_ordereddict(result),
        }, indent=2) + '\n')
        if result.error_code.val != MoveItErrorCodes.SUCCESS or result.fraction < 0.999999:
            raise RuntimeError('Incomplete path: do not execute')
        points = result.solution.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError('No usable trajectory')
        for before, after in zip(points, points[1:]):
            if len(before.positions) != 6 or len(after.positions) != 6:
                raise RuntimeError('Expected six joints')
            if any(not math.isfinite(v) for v in after.positions):
                raise RuntimeError('Nonfinite trajectory')
            if max(abs(a-b) for a,b in zip(before.positions, after.positions)) > 0.05:
                raise RuntimeError('Joint step exceeds 0.05 rad; do not execute')
        display = DisplayTrajectory()
        display.trajectory_start = result.start_state
        display.trajectory = [result.solution]
        publisher = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        for _ in range(10):
            publisher.publish(display)
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'Complete hover plan: {len(points)} points. Preview published. NO EXECUTION.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
